"""
swing_sine_pos.py
===============================================================================
사인파 swing — **POS 제어 + velocity feedforward (PASSTHROUGH)**.

목적: POS_FILTER 의 거친 추적 (vel_setpoint 가 필터 미분으로 추정됨) 대신
호스트가 사인의 위치 AND 속도 둘 다 직접 명령. 컨트롤러는 pos error 거의 0
상태에서 동작 → cogging 보정 적음 → VEL 모드처럼 부드러우면서 위치 정확.

  input_pos(t) = center + env(t) × amp × sin(ωt)
  input_vel(t) = env(t) × ω × amp × cos(ωt)        ← vel feedforward (핵심)

비교:
- swing_sine.py     (POS_FILTER, input_pos만)         → 거침
- swing_sine_vel.py (VEL_RAMP, input_vel만)           → 부드러움 + drift
- swing_sine_pos.py (PASSTHROUGH, input_pos+input_vel) → 부드러움 + 정확

안전:
- PASSTHROUGH 는 ramp 없음. 우리 사인 cmd 의 가속도(ω²×amp) 가 모터 한계 안에
  있어야 함. current_lim 10A → α_max ≈ 308 turn/s². 명령 가속도 미리 검사.

사용
----
    python swing_sine_pos.py --amp 240 --freq 1.0 --duration 20
    python swing_sine_pos.py --amp 60 --freq 4.0 --duration 10

옵션은 swing_sine_vel.py 와 동일 (--vel-ramp 만 없음).
"""
from __future__ import annotations

import argparse
import math
import signal
import sys
import time

from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_POSITION_CONTROL,
    INPUT_MODE_PASSTHROUGH,
)

from motor_helpers import (
    connect,
    ensure_calibrated,
    apply_safety,
    safe_stop,
    envelope,
    deg_to_turn,
    is_finite_safe,
)


UPDATE_HZ = 200
TRACKING_ERR_LIMIT = 0.5  # turn — pos error 한계


class SafeStop(Exception):
    pass


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--amp", type=float, required=True, help="진폭 [모터축 °]")
    ap.add_argument("--freq", type=float, required=True, help="주파수 [Hz]")
    ap.add_argument("--duration", type=float, required=True, help="지속시간 [s]")
    ap.add_argument("--ramp", type=float, default=1.5, help="envelope fade [s]")
    ap.add_argument("--limit", type=float, default=None, help="위치 한계 [모터°]")
    ap.add_argument("--current-lim", type=float, default=10.0, help="전류 한계 [A]")
    ap.add_argument("--vel-limit", type=float, default=12.0, help="속도 한계 [turn/s]")
    args = ap.parse_args()

    if args.amp <= 0 or args.freq <= 0 or args.duration <= 0:
        print("[오류] amp / freq / duration 양수")
        return 1

    limit_deg = args.limit if args.limit is not None else args.amp * 1.5
    if limit_deg < args.amp:
        print("[오류] limit < amp")
        return 1

    amp_turn = deg_to_turn(args.amp)
    limit_turn = deg_to_turn(limit_deg)
    omega = 2.0 * math.pi * args.freq

    peak_vel = omega * amp_turn         # turn/s
    peak_accel = omega * omega * amp_turn  # turn/s²

    # 우리 모터의 current_lim 기반 α_max 추정 (current_lim=10A, J~0.0005,
    # Kt=0.097 N·m/A → α_max ~ 308 turn/s²)
    j_effective = 0.0005  # kg·m² 추정
    Kt = 0.097            # N·m/A
    alpha_max_est = args.current_lim * Kt / j_effective / (2 * math.pi)  # turn/s²

    print(f"파라미터: amp={args.amp}° ({amp_turn:.4f} turn) freq={args.freq}Hz "
          f"duration={args.duration}s limit={limit_deg}° ramp={args.ramp}s "
          f"current_lim={args.current_lim}A")
    print(f"  peak_vel    = {peak_vel:.2f} turn/s ({peak_vel*45:.0f}°/s 출력)")
    print(f"  peak_accel  = {peak_accel:.1f} turn/s²")
    print(f"  α_max(추정) = {alpha_max_est:.1f} turn/s² (current_lim {args.current_lim}A 기준)")
    if peak_accel > alpha_max_est:
        print(f"  ⚠ peak_accel > α_max — 오버커런트 위험. amp 줄이거나 current_lim 상향 권장")
    if peak_vel > args.vel_limit * 0.95:
        print(f"  ⚠ peak_vel 이 vel_limit 의 95% 초과")

    try:
        odrv = connect()
    except Exception as e:
        print(f"[오류] 연결 실패: {e}")
        return 1
    axis = odrv.axis0

    try:
        ensure_calibrated(axis)
        apply_safety(axis, args.current_lim, args.vel_limit)

        # PASSTHROUGH 위치제어 — input_pos 와 input_vel 직접 setpoint
        axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        print("모드: POS_CONTROL + INPUT_PASSTHROUGH (pos + vel feedforward)")

        # 현재 위치로 시작 — 점프 방지
        start_pos = float(axis.encoder.pos_estimate)
        center = start_pos
        axis.controller.input_pos = center
        axis.controller.input_vel = 0.0
        print(f"center = {center:+.4f} turn")

        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.2)
        if int(axis.current_state) != int(AXIS_STATE_CLOSED_LOOP_CONTROL):
            err = int(axis.error)
            print(f"[오류] CLOSED_LOOP 진입 실패. state={int(axis.current_state)} err={err:#x}")
            return 1
    except Exception as e:
        print(f"[오류] 초기화 실패: {e}")
        safe_stop(axis)
        return 1

    def _sigint(_sig, _frm):
        raise SafeStop()
    signal.signal(signal.SIGINT, _sigint)

    print(f"시작.")
    period = 1.0 / UPDATE_HZ
    t0 = time.time()
    last_log = t0

    try:
        while True:
            now = time.time()
            t = now - t0
            if t >= args.duration:
                break

            env = envelope(t, args.duration, args.ramp)

            # 사인 위치 + 미분 = 속도 feedforward
            sinwt = math.sin(omega * t)
            coswt = math.cos(omega * t)
            cmd_pos = center + env * amp_turn * sinwt
            cmd_vel = env * omega * amp_turn * coswt
            # 주: env' 항 (ramp 동안 미세 보정) 은 무시 — 영향 작음

            # 한계 체크 (명령 자체)
            if abs(cmd_pos - center) > limit_turn:
                print(f"\n[비상] 명령 한계 초과: {cmd_pos - center:+.4f}")
                raise SafeStop()

            try:
                actual_pos = float(axis.encoder.pos_estimate)
                actual_vel = float(axis.encoder.vel_estimate)
            except Exception as e:
                print(f"\n[비상] 인코더 읽기 실패: {e}")
                raise SafeStop()

            if not is_finite_safe(actual_pos, actual_vel):
                print(f"\n[비상] 인코더 비정상")
                raise SafeStop()

            # 실측 한계
            if abs(actual_pos - center) > limit_turn:
                print(f"\n[비상] 실측 한계 초과: {actual_pos - center:+.4f}")
                raise SafeStop()
            # 추종 오차 (피드포워드 있어서 작아야 함)
            pos_err = cmd_pos - actual_pos
            if abs(pos_err) > TRACKING_ERR_LIMIT:
                print(f"\n[비상] 추종 오차 초과: {pos_err:+.4f}")
                raise SafeStop()

            # 명령 인가 — 둘 다
            axis.controller.input_pos = cmd_pos
            axis.controller.input_vel = cmd_vel

            if now - last_log >= 0.5:
                sys.stdout.write(
                    f"\rt={t:6.2f}s env={env:4.2f}  "
                    f"p_cmd={cmd_pos - center:+.4f}  p_act={actual_pos - center:+.4f}  "
                    f"err={pos_err:+.4f}  v_cmd={cmd_vel:+.2f}  v_act={actual_vel:+.2f}   "
                )
                sys.stdout.flush()
                last_log = now

            sleep_left = period - (time.time() - now)
            if sleep_left > 0:
                time.sleep(sleep_left)

        print("\n지속시간 완료.")
        return 0

    except SafeStop:
        print("\n안전 정지...")
        return 0
    except Exception as e:
        print(f"\n[오류] 루프 예외: {e}")
        return 1
    finally:
        safe_stop(axis)
        print("axis0 → IDLE.")


if __name__ == "__main__":
    sys.exit(main())
