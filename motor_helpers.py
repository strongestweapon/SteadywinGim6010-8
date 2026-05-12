"""
motor_helpers.py
===============================================================================
모든 스크립트가 공통으로 쓰는 헬퍼 모듈.

설계 원칙
---------
- 위치/속도/토크 명령은 ODrive 의 내장 input_mode 로 부드럽게 램프.
- 사인파처럼 시간에 연속인 명령은 외부 envelope 으로 시작/끝 진폭을 cosine
  으로 fade-in/out → 갑작스러운 전류 소모 방지.
- 안전 흐름: 캘리브레이션 검증 → 전류/속도 한계 적용 → CLOSED_LOOP 진입
  → 종료 시 부드러운 감쇠 → IDLE.

핵심 우회: snap-to-zero
-----------------------
SteadyWin GIM6010-8 (펌웨어 v0.6.5) 는 mono-turn 절대 인코더(MA600) 라서
부팅 시 multi-turn 인덱스 결정이 두 분기 사이를 토글한다. 측정 결과
정확히 ±TOGGLE_TURN 만큼 어긋남이 결정론적으로 반복됨.
→ `enter_position_mode(snap_to_zero=True)` 가 부팅 직후 분기값을 감지하고
   center 를 보정해서 swing 의 진폭 범위가 매 부팅마다 일관되게 잡힌다.
"""
from __future__ import annotations

import math
import sys
import time

import odrive
from odrive.enums import (
    AXIS_STATE_IDLE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_POSITION_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    CONTROL_MODE_TORQUE_CONTROL,
    INPUT_MODE_POS_FILTER,
    INPUT_MODE_VEL_RAMP,
    INPUT_MODE_TORQUE_RAMP,
)

# Windows PowerShell 콘솔의 cp949 기본 인코딩 때문에 한글이 깨지는 것을 방지.
# import 만 해도 자동 적용되도록 모듈 최상단에서 실행.
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    # Python 3.6 이하 또는 비콘솔 환경에서는 실패할 수 있음 — 무시.
    pass


# -----------------------------------------------------------------------------
# 부팅 시 multi-turn 인덱스 토글 보정 (snap-to-zero)
# -----------------------------------------------------------------------------
# 측정값: 같은 책상 위 정지 상태로 odrv.reboot() 를 반복하면 pos_estimate 가
# 두 값 사이 결정론적으로 토글:
#   회차 1: -0.142515
#   회차 2: -0.000089
#   회차 3: -0.142610
# 평균 토글 폭 = 0.142469 turn ≈ 모터축 51.3° ≈ 출력축 6.4°
#
# 이 값은 mono-turn 절대 인코더(MA600) + 8:1 기어 + 14 pole-pair 모터 조합에서
# 발생하는 펌웨어 차원의 알려진 ambiguity. ODrive 공식 문서도 "absolute encoder
# + multi-turn axis 사용 시 reference frame 이 정수 turn 단위로 shift 가능"
# 이라고 인정. 모터/캘리브 값에 따라 다를 수 있으니 정확값을 측정해서 세팅.
TOGGLE_TURN = 0.142469

# 부팅된 pos_estimate 가 TOGGLE_TURN 정수배 격자에서 이만큼 이내 떨어져 있으면
# "그 격자에 snap" 으로 판단. 모터축 약 7.2° 이내. 이걸 넘으면 사용자가 0점을
# 아직 안 굽혔거나, 진자가 외력에 밀린 상태로 본다.
SNAP_TOLERANCE = 0.020


def detect_boot_branch(pos: float) -> float | None:
    """부팅 직후 pos_estimate 가 어떤 토글 분기에 lock 됐는지 감지.

    pos 가 TOGGLE_TURN 의 정수배 ±SNAP_TOLERANCE 안에 있으면 그 정수배를 반환,
    아니면 None.

    예시 (TOGGLE_TURN ≈ 0.142):
      pos = +0.0001 → 0 회차 분기, return  0.000
      pos = -0.1424 → −1 회차 분기, return -0.142
      pos = +0.0700 → 격자 어디에도 안 가까움 → None
    """
    n = round(pos / TOGGLE_TURN)
    branch = n * TOGGLE_TURN
    if abs(pos - branch) > SNAP_TOLERANCE:
        return None
    return branch


# -----------------------------------------------------------------------------
# 연결 / 상태 확인
# -----------------------------------------------------------------------------

def connect(timeout: float = 10.0):
    """ODrive 보드 검색 + 기본 정보 1줄 출력. 실패 시 예외."""
    print(f"ODrive 검색 중... (최대 {timeout:.0f}초)")
    odrv = odrive.find_any(timeout=timeout)
    sn = f"{odrv.serial_number:012X}"
    fw = f"{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}"
    vbus = float(odrv.vbus_voltage)
    print(f"  serial={sn}  fw=v{fw}  vbus={vbus:.2f}V")
    return odrv


def ensure_calibrated(axis) -> None:
    """공장 캘리브가 끝나 있는지 검증. 미완료면 RuntimeError 로 즉시 중단.

    공장 출하 시 pre_calibrated=True 로 굽혀 나오므로 사용자가 별도 캘리브를
    돌리면 안 된다 (잘못하면 phase_offset 등의 출하값이 덮어씌워짐).
    """
    if not bool(axis.motor.is_calibrated):
        raise RuntimeError("axis.motor.is_calibrated == False — 모터 캘리브 미완료")
    if not bool(axis.encoder.is_ready):
        raise RuntimeError("axis.encoder.is_ready == False — 인코더 ready 아님")


def apply_safety(axis, current_lim: float, vel_limit: float) -> None:
    """매번 모드 진입할 때 보수적 전류/속도 한계를 다시 set.

    save_configuration 은 호출하지 않으므로 플래시에는 안 굽힘 — 다음 부팅에는
    보드의 원래 값(공장 60A 등)이 적용됨. 무대 운영 중에만 안전값을 강제.
    """
    axis.motor.config.current_lim = float(current_lim)
    axis.controller.config.vel_limit = float(vel_limit)


# -----------------------------------------------------------------------------
# 모드 진입 (위치 / 속도 / 토크) — 모두 부드러운 ramp 가 기본
# -----------------------------------------------------------------------------

def enter_position_mode(axis,
                        current_lim: float = 10.0,
                        vel_limit: float = 20.0,
                        input_filter_hz: float = 100.0,
                        vel_integrator_gain: float = 5.0,
                        snap_to_zero: bool = True) -> float:
    """위치 제어 진입.

    INPUT_MODE_POS_FILTER 가 input_pos 의 급변을 input_filter_bandwidth Hz 의
    2차 lowpass 로 부드럽게 만든다. 사용자가 갑자기 값을 점프시켜도 모터는
    필터된 값을 추종 → 갑작스러운 전류 폭주 방지.

    SteadyWin 매뉴얼 P32 권장: bandwidth = **명령 전송 주파수의 1/2**.
    우리 swing 루프는 200Hz 갱신 → input_filter_hz = 100Hz 가 권장.
    낮게(예: 8Hz) 잡으면 명령이 너무 강하게 lowpass 되어 진폭 감쇠 + 위상 반전.
    실측: 8Hz → act 가 cmd 반대 방향 / 50% 진폭. 100Hz → 진폭 80%+ 정상 추종.

    vel_integrator_gain: 매뉴얼 P30 공식 = 0.5 * control_bandwidth_Hz * vel_gain.
    우리 vel_gain=0.1, bandwidth=100Hz → 5.0. 정상상태 추종오차 줄여 사인파의
    peak 부근 lag (= 덜그럭 거림) 감소.

    Returns
    -------
    center : float
        snap_to_zero=True 면 부팅 토글 분기값에 snap 한 origin (모터 회전 turns).
        False 면 단순히 현재 pos_estimate.
        호출자는 swing 명령을 `center + amp*sin(...)` 으로 만들면 매 부팅마다
        같은 진폭 범위에서 움직임.
    """
    ensure_calibrated(axis)
    apply_safety(axis, current_lim, vel_limit)

    # 컨트롤러 설정: 위치 제어 + lowpass 입력 필터 + 매뉴얼 권장 적분기 게인.
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_POS_FILTER
    axis.controller.config.input_filter_bandwidth = float(input_filter_hz)
    axis.controller.config.vel_integrator_gain = float(vel_integrator_gain)

    # 현재 위치를 측정해서 center 결정 — 점프 방지의 핵심.
    raw_pos = float(axis.encoder.pos_estimate)
    if snap_to_zero:
        branch = detect_boot_branch(raw_pos)
        if branch is None:
            # 격자 어디에도 안 가까움 → set_zero 가 안 됐거나 외력으로 진자가
            # 격자에서 멀리 밀려난 상태. 그냥 현재 위치를 origin 으로 사용.
            print(f"  [경고] 부팅 분기 감지 실패 (raw={raw_pos:+.4f}). "
                  f"raw 위치를 그대로 origin 으로 사용.")
            center = raw_pos
        else:
            center = branch
            print(f"  [snap] raw={raw_pos:+.6f} → branch={branch:+.6f} "
                  f"({branch / TOGGLE_TURN:.0f}회차 토글)")
    else:
        center = raw_pos

    # 첫 명령은 반드시 현재 raw 위치 — center 와 raw 가 다르면 명령 점프 발생할
    # 수 있어서 closed-loop 진입 후 부드럽게 raw → center 로 흘러가도록 함.
    axis.controller.input_pos = raw_pos

    # 클로즈드 루프 진입.
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.2)
    _verify_closed_loop(axis)
    return center


def enter_velocity_mode(axis,
                        current_lim: float = 10.0,
                        vel_limit: float = 20.0,
                        vel_ramp_rate: float = 5.0) -> None:
    """속도 제어 진입.

    INPUT_MODE_VEL_RAMP 가 input_vel 변화량을 vel_ramp_rate [turn/s²] 로 제한
    → 가속이 부드러움. 사용자가 갑자기 큰 input_vel 을 줘도 모터는 ramp 따라
    천천히 도달.
    """
    ensure_calibrated(axis)
    apply_safety(axis, current_lim, vel_limit)
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    axis.controller.config.vel_ramp_rate = float(vel_ramp_rate)
    axis.controller.input_vel = 0.0
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.2)
    _verify_closed_loop(axis)


def enter_torque_mode(axis,
                      current_lim: float = 5.0,
                      torque_ramp_rate: float = 0.5) -> None:
    """토크 제어 진입.

    INPUT_MODE_TORQUE_RAMP 가 input_torque 변화량을 torque_ramp_rate [N·m/s]
    로 제한. 무대용은 current_lim 을 더 보수적으로 (기본 5A) 잡는다.
    """
    ensure_calibrated(axis)
    axis.motor.config.current_lim = float(current_lim)
    axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_TORQUE_RAMP
    axis.controller.config.torque_ramp_rate = float(torque_ramp_rate)
    axis.controller.input_torque = 0.0
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.2)
    _verify_closed_loop(axis)


def _verify_closed_loop(axis) -> None:
    """CLOSED_LOOP 진입에 실제로 성공했는지 axis.current_state 로 검증."""
    state = int(axis.current_state)
    if state != int(AXIS_STATE_CLOSED_LOOP_CONTROL):
        err = int(axis.error)
        raise RuntimeError(
            f"CLOSED_LOOP_CONTROL 진입 실패. current_state={state} axis.error={err:#x}"
        )


# -----------------------------------------------------------------------------
# 안전 정지
# -----------------------------------------------------------------------------

def safe_stop(axis, ramp_s: float = 0.8) -> None:
    """현재 명령을 ramp_s 동안 부드럽게 감쇠시킨 뒤 IDLE.

    - 토크/속도 모드: input 을 0 까지 선형 감쇠
    - 위치 모드: 현재 실측 위치를 명령으로 잡아 잠시 hold (점프 없는 정지)
    Ctrl+C, 한계 초과, 인코더 비정상 등 어떤 경로로 호출돼도 finally 보장.
    """
    try:
        cm = int(getattr(axis.controller.config, "control_mode", -1))
    except Exception:
        cm = -1

    try:
        if cm == int(CONTROL_MODE_TORQUE_CONTROL):
            _ramp_to_zero(axis, "input_torque", ramp_s)
        elif cm == int(CONTROL_MODE_VELOCITY_CONTROL):
            _ramp_to_zero(axis, "input_vel", ramp_s)
        elif cm == int(CONTROL_MODE_POSITION_CONTROL):
            # 위치 모드는 "지금 거기 그대로" 가 가장 부드러운 정지.
            try:
                axis.controller.input_pos = float(axis.encoder.pos_estimate)
                time.sleep(min(0.3, ramp_s))
            except Exception:
                pass
    finally:
        # 어떤 단계에서 실패해도 IDLE 은 무조건 시도.
        try:
            axis.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass


def _ramp_to_zero(axis, attr: str, ramp_s: float) -> None:
    """controller.input_xxx 를 ramp_s 동안 선형으로 0 까지 감쇠."""
    try:
        start = float(getattr(axis.controller, attr))
    except Exception:
        return
    t0 = time.time()
    while True:
        t = time.time() - t0
        if t >= ramp_s:
            break
        alpha = t / ramp_s  # 0 → 1
        try:
            setattr(axis.controller, attr, start * (1.0 - alpha))
        except Exception:
            break
        time.sleep(0.01)
    try:
        setattr(axis.controller, attr, 0.0)
    except Exception:
        pass


# -----------------------------------------------------------------------------
# 사인파 envelope — 시작/끝 진폭을 cosine 으로 fade
# -----------------------------------------------------------------------------

def envelope(t: float, duration: float, ramp_s: float = 1.0) -> float:
    """Half-Hann window — 사인파 진폭에 곱해서 사일런트 시작/종료를 만든다.

      [0, ramp_s]                    : 0 → 1 (cosine ease-in)
      [ramp_s, duration-ramp_s]      : 1 (full amplitude)
      [duration-ramp_s, duration]    : 1 → 0 (cosine ease-out)
      바깥                           : 0

    cosine 곡선이라 진폭 변화율(가속도)도 부드러워서 진자가 갑자기 튀지 않음.
    ramp_s * 2 가 duration 보다 크면 자동으로 duration/2 로 클리핑.
    """
    if duration <= 0:
        return 0.0
    rs = min(ramp_s, duration / 2.0)
    if t <= 0.0 or t >= duration:
        return 0.0
    if rs <= 0.0:
        return 1.0
    if t < rs:
        # ease-in: cos(π*t/rs) 가 1→-1 이므로 (1-cos)/2 = 0→1
        return 0.5 * (1.0 - math.cos(math.pi * t / rs))
    if t > duration - rs:
        # ease-out: 거꾸로 0 까지
        return 0.5 * (1.0 - math.cos(math.pi * (duration - t) / rs))
    return 1.0


# -----------------------------------------------------------------------------
# 유틸리티
# -----------------------------------------------------------------------------

def is_finite_safe(*values: float, max_abs: float = 1e9) -> bool:
    """NaN / inf / 비현실적 크기 검사. 인코더 깨짐 감지용."""
    for v in values:
        if v != v:  # NaN 은 자기자신과 안 같음
            return False
        if abs(v) > max_abs:
            return False
    return True


def deg_to_turn(deg: float) -> float:
    """모터축 각도(°) → ODrive 의 위치 단위 turn 변환."""
    return deg / 360.0


def turn_to_deg(turn: float) -> float:
    """ODrive turn → 모터축 ° 변환."""
    return turn * 360.0
