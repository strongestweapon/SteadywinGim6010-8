"""
swing_trap.py
===============================================================================
TRAP_TRAJ (사다리꼴 trajectory) 모드 ±amp 왕복.

기존 swing_sine.py 와의 차이
-----------------------------
- swing_sine.py: 호스트가 200Hz 로 사인파 cmd 를 직접 계산해 streaming.
  매 5ms cmd 점프 → 모터가 그걸 다 추종하려고 미세 진동 / 덜그럭 발생.
- swing_trap.py: **끝 위치만** 한 번 명령하고 **펌웨어 내부의 사다리꼴
  trajectory generator** 가 가속/등속/감속을 자동 생성. cmd streaming 없음.
  → 명령 점프 0 → 모터 응답 매끈.

매뉴얼 P32-33 권장 모드. SteadyWin Motor Wizard 의 부드러움도 이 모드일 가능성.

trap_traj 설정 (axis.trap_traj.config)
--------------------------------------
- vel_limit  [turn/s]   : 등속 구간(글라이드) 속도
- accel_limit [turn/s²] : 가속 한계
- decel_limit [turn/s²] : 감속 한계

진폭이 작거나 accel 이 크면 등속 구간이 없는 삼각 profile 이 되기도 함 (정상).
한 방향 이동 시간 ≈ 2 × sqrt(amp / accel) (삼각 case).

사용법
------
    python swing_trap.py --amp 240 --cycles 5
    python swing_trap.py --amp 240 --cycles 10 --vel-limit 4 --accel 12
"""
import argparse
import signal
import sys
import time

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_POSITION_CONTROL,
    INPUT_MODE_TRAP_TRAJ,
)

from motor_helpers import (
    connect,
    ensure_calibrated,
    apply_safety,
    safe_stop,
    deg_to_turn,
    is_finite_safe,
    detect_boot_branch,
)


class SafeStop(Exception):
    """Ctrl+C / 한계 초과 신호."""
    pass


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--amp", type=float, required=True,
                    help="진폭 [모터축 °]. 출력축 진폭 = amp/8.")
    ap.add_argument("--cycles", type=int, default=5,
                    help="왕복 사이클 수 (한 사이클 = +amp 갔다 -amp 도착)")
    # ─── trap_traj 파라미터 ────────────────────────────────────────────
    # default = SteadyWin 공장값 (probe_tree dump 에서 확인).
    # 매뉴얼 P32-33 이 trap_traj 용도로 노출한 정상 운용 파라미터라 변경 안전.
    ap.add_argument("--vel-limit", type=float, default=100.0,
                    help="trap_traj 글라이드 속도 [turn/s]. 공장 default 100.")
    ap.add_argument("--accel", type=float, default=20.0,
                    help="trap_traj 가속 [turn/s²]. 공장 default 20.")
    ap.add_argument("--decel", type=float, default=None,
                    help="trap_traj 감속 [turn/s²]. 기본=accel.")
    ap.add_argument("--rest-s", type=float, default=0.0,
                    help="각 사이클 사이 정지 안정화 [s]. 0.1~0.2 권장.")
    # ─── 게인/한계 ─ default None: 공장값 그대로 (안 건드림) ─────────────
    # 매뉴얼 P29-30 의 PID 튜닝 절차를 따라야 안전. 사용자가 명시해야만 set.
    ap.add_argument("--current-lim", type=float, default=None,
                    help="전류 한계 [A]. None=공장값(60A) 그대로.")
    ap.add_argument("--motor-vel-limit", type=float, default=None,
                    help="controller.vel_limit [turn/s]. None=공장값(30) 그대로.")
    ap.add_argument("--limit", type=float, default=None,
                    help="위치 한계 [모터축 °]. 기본 amp*1.5.")
    ap.add_argument("--no-snap", action="store_true",
                    help="snap-to-zero 비활성")
    ap.add_argument("--end-center", action="store_true",
                    help="시퀀스 끝에 center 로 복귀")
    ap.add_argument("--vel-integrator", type=float, default=None,
                    help="vel_integrator_gain. None=공장값(1.0) 그대로. "
                         "매뉴얼 공식: 0.5*bandwidth*vel_gain.")
    ap.add_argument("--pos-gain", type=float, default=None,
                    help="pos_gain. None=공장값(20.0) 그대로.")
    ap.add_argument("--vel-gain", type=float, default=None,
                    help="vel_gain. None=공장값(0.10) 그대로.")
    ap.add_argument("--encoder-bandwidth", type=float, default=None,
                    help="encoder.config.bandwidth [Hz]. None=공장값(500) 그대로. "
                         "**500→100 변경은 PID 발산 위험 — 매뉴얼 절차에 없는 파라미터.**")
    args = ap.parse_args()

    if args.amp <= 0 or args.cycles <= 0:
        print("[오류] amp / cycles 양수.")
        return 1

    limit_deg = args.limit if args.limit is not None else args.amp * 1.5
    limit_turn = deg_to_turn(limit_deg)
    amp_turn = deg_to_turn(args.amp)

    print(f"파라미터: amp={args.amp}° ({amp_turn:.4f} turn)  "
          f"cycles={args.cycles}  "
          f"vel_limit={args.vel_limit} turn/s  accel={args.accel} turn/s²  "
          f"limit={limit_deg}°  current_lim={args.current_lim}A")

    try:
        odrv = connect()
    except Exception as e:
        print(f"[오류] 연결: {e}")
        return 1
    axis = odrv.axis0

    try:
        ensure_calibrated(axis)
        # 게인/한계는 사용자가 명시한 것만 set. None 이면 공장값 그대로 둠.
        if args.current_lim is not None:
            axis.motor.config.current_lim = float(args.current_lim)
            print(f"  motor.config.current_lim = {args.current_lim} A")
        if args.motor_vel_limit is not None:
            axis.controller.config.vel_limit = float(args.motor_vel_limit)
            print(f"  controller.config.vel_limit = {args.motor_vel_limit} turn/s")
        if args.encoder_bandwidth is not None:
            axis.encoder.config.bandwidth = float(args.encoder_bandwidth)
            print(f"  encoder.config.bandwidth = {args.encoder_bandwidth} Hz")
    except Exception as e:
        print(f"[오류] 안전 설정: {e}")
        return 1

    # ---- trap_traj 설정 ----
    # 매뉴얼 P32 의 핵심 설정. 펌웨어가 vel_limit/accel_limit 안에서
    # 사다리꼴 속도 프로파일을 자동 생성.
    # decel 을 accel 보다 작게 잡으면 정지 시점이 부드러워서 방향 전환 충격 감소.
    decel = args.decel if args.decel is not None else args.accel
    try:
        axis.trap_traj.config.vel_limit = float(args.vel_limit)
        axis.trap_traj.config.accel_limit = float(args.accel)
        axis.trap_traj.config.decel_limit = float(decel)
    except Exception as e:
        print(f"[오류] trap_traj 설정: {e}")
        return 1
    print(f"  trap_traj: vel_lim={args.vel_limit}  accel={args.accel}  decel={decel}  "
          f"rest_s={args.rest_s}")

    # ---- 컨트롤러 모드 ----
    # control_mode = POSITION, input_mode = TRAP_TRAJ 는 trap_traj 동작에 필수.
    # 게인 (pos_gain, vel_gain, vel_integrator_gain) 은 사용자가 명시한 것만 set.
    try:
        axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        if args.pos_gain is not None:
            axis.controller.config.pos_gain = float(args.pos_gain)
        if args.vel_gain is not None:
            axis.controller.config.vel_gain = float(args.vel_gain)
        if args.vel_integrator is not None:
            axis.controller.config.vel_integrator_gain = float(args.vel_integrator)
    except Exception as e:
        print(f"[오류] 컨트롤러 설정: {e}")
        return 1
    # 진입 후 실제 보드 값 출력 (None 이었으면 공장값이 그대로 보임)
    print(f"  게인 (현재 보드값): pos_gain={float(axis.controller.config.pos_gain):.2f}  "
          f"vel_gain={float(axis.controller.config.vel_gain):.4f}  "
          f"vel_integrator={float(axis.controller.config.vel_integrator_gain):.2f}")

    # ---- center 결정 (snap-to-zero) ----
    raw = float(axis.encoder.pos_estimate)
    if not args.no_snap:
        branch = detect_boot_branch(raw)
        if branch is None:
            print(f"  [경고] snap 격자 감지 실패 (raw={raw:+.4f}). raw 사용.")
            center = raw
        else:
            center = branch
            print(f"  [snap] raw={raw:+.6f} → branch={branch:+.6f}")
    else:
        center = raw

    # 첫 명령은 현재 raw 위치 — 점프 방지
    axis.controller.input_pos = raw

    # CLOSED_LOOP 진입
    try:
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.2)
        if int(axis.current_state) != int(AXIS_STATE_CLOSED_LOOP_CONTROL):
            raise RuntimeError(
                f"CLOSED_LOOP 진입 실패. axis.error={int(axis.error):#x}"
            )
    except Exception as e:
        print(f"[오류] 진입: {e}")
        safe_stop(axis)
        return 1

    # raw 에서 center 로 천천히 정착 (trap_traj 가 자동 처리)
    if abs(raw - center) > 0.001:
        print(f"  raw → center 정착 중...")
        axis.controller.input_pos = center
        _wait_until_done(axis, timeout=3.0)

    # Ctrl+C 처리
    def _sigint(_s, _f):
        raise SafeStop()
    signal.signal(signal.SIGINT, _sigint)

    print(f"시작. center={center:+.4f}. amp=±{amp_turn:.4f} turn.")
    try:
        for i in range(args.cycles):
            for sign in (+1, -1):
                target = center + sign * amp_turn
                axis.controller.input_pos = target
                # 펌웨어가 끝까지 trajectory 돌리는 동안 폴링
                arrived = _wait_until_done(
                    axis, timeout=10.0, center=center, limit_turn=limit_turn
                )
                if not arrived:
                    print("\n[비상] 타임아웃 또는 한계 초과")
                    raise SafeStop()
                # 정지 안정화 — 백래시 정렬 + 토크 spike 진정
                if args.rest_s > 0:
                    time.sleep(args.rest_s)
            print(f"  ✓ cycle {i+1}/{args.cycles} 완료")

        if args.end_center:
            print("  중심 복귀...")
            axis.controller.input_pos = center
            _wait_until_done(axis, timeout=3.0)

        print("\n시퀀스 완료.")
        return 0
    except SafeStop:
        print("\n안전 정지...")
        return 0
    except Exception as e:
        print(f"\n[오류] 예외: {e}")
        return 1
    finally:
        safe_stop(axis)
        print("axis0 → IDLE.")


def _wait_until_done(axis, timeout: float,
                     center: float | None = None,
                     limit_turn: float | None = None) -> bool:
    """trajectory_done 이 True 가 될 때까지 폴링.

    중간에 위치 한계 / 인코더 비정상 / 타임아웃 발생 시 False.
    """
    t0 = time.time()
    last_log = t0
    while True:
        if time.time() - t0 > timeout:
            return False
        try:
            done = bool(axis.controller.trajectory_done)
            actual = float(axis.encoder.pos_estimate)
            setp = float(axis.controller.pos_setpoint)
        except Exception:
            time.sleep(0.02)
            continue

        if not is_finite_safe(actual, setp):
            print(f"\n[비상] 인코더/setpoint 비정상")
            return False
        if center is not None and limit_turn is not None:
            if abs(actual - center) > limit_turn:
                print(f"\n[비상] 위치 한계 초과: {actual - center:+.4f}")
                return False

        # 0.2초마다 로그
        now = time.time()
        if now - last_log >= 0.2:
            tag = "✓" if done else " "
            sys.stdout.write(
                f"\r  {tag} setp={setp:+.4f}  act={actual:+.4f}  done={done}    "
            )
            sys.stdout.flush()
            last_log = now

        if done:
            print()  # 로그 줄바꿈
            return True
        time.sleep(0.01)


if __name__ == "__main__":
    sys.exit(main())
