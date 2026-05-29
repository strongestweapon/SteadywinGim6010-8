"""
swing_sine_vel.py
===============================================================================
사인파 swing — **속도제어 (VEL_RAMP) 모드**.

POS_FILTER 의 매 시점 위치 추적이 cogging 을 강하게 드러내는 문제 우회.
속도 명령은 cos(ωt) — 적분하면 sin(ωt) 위치가 자연스럽게 형성.
컨트롤러는 평균 속도만 맞추므로 cogging 보정이 부드러워짐 (= 시각적 매끈).

  v(t) = env(t) × ω × amp_motor × cos(ω × t)
  pos  = 적분 = env(t) × amp_motor × sin(ω × t)  (이론)

위치 drift 검사 (실측 vs 이론 가드밴드 ±limit) — 한계 넘으면 비상 정지.

사용
----
    python swing_sine_vel.py --amp 240 --freq 1.0 --duration 20

옵션은 swing_sine.py 와 동일.
"""
from __future__ import annotations

import argparse
import math
import signal
import sys
import time

from odrive.enums import (
    AXIS_STATE_IDLE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_VEL_RAMP,
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
TRACKING_DRIFT_LIMIT_RATIO = 2.0  # 위치 drift 가 amp 의 N 배 넘으면 비상


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
    ap.add_argument("--vel-ramp", type=float, default=100.0,
                    help="vel_ramp_rate [turn/s²]. 사인 peak 가속도 = ω²×amp 보다 커야 함. "
                         "1Hz × 0.667turn → 26 필요. 기본 100 (안전 마진).")
    ap.add_argument("--passthrough", action="store_true",
                    help="POS+VEL feedforward (PASSTHROUGH) 모드로 — 위치도 명령")
    ap.add_argument("--vel-gain", type=float, default=None,
                    help="vel_gain 을 RAM 에만 임시 설정 (영구 저장 X). "
                         "미지정 시 보드 현재값 사용. 1번 모터 P29 튜닝값=0.145.")
    args = ap.parse_args()

    if args.amp <= 0 or args.freq <= 0 or args.duration <= 0:
        print("[오류] amp / freq / duration 양수")
        return 1

    limit_deg = args.limit if args.limit is not None else args.amp * 1.5
    amp_turn = deg_to_turn(args.amp)
    limit_turn = deg_to_turn(limit_deg)
    omega = 2.0 * math.pi * args.freq

    peak_vel = omega * amp_turn  # turn/s
    print(f"파라미터: amp={args.amp}° ({amp_turn:.4f} turn) freq={args.freq}Hz "
          f"duration={args.duration}s limit={limit_deg}° ramp={args.ramp}s "
          f"current_lim={args.current_lim}A vel_limit={args.vel_limit}")
    print(f"  peak_vel = {peak_vel:.3f} turn/s "
          f"({peak_vel * 360:.0f}°/sec 모터, {peak_vel * 45:.0f}°/sec 출력)")
    if peak_vel > args.vel_limit * 0.95:
        print(f"  ⚠ peak_vel 이 vel_limit 의 95% 초과 — 한계 걸릴 위험")

    try:
        odrv = connect()
    except Exception as e:
        print(f"[오류] 연결 실패: {e}")
        return 1
    axis = odrv.axis0

    try:
        ensure_calibrated(axis)
        apply_safety(axis, args.current_lim, args.vel_limit)

        # vel_gain RAM 임시 설정 (영구 저장 안 함 — 비교/시연용)
        if args.vel_gain is not None:
            old_vg = float(axis.controller.config.vel_gain)
            axis.controller.config.vel_gain = float(args.vel_gain)
            print(f"vel_gain: {old_vg:.4f} → {args.vel_gain:.4f} (RAM only, 영구 저장 안 함)")

        if args.passthrough:
            # POS_PASSTHROUGH 위치제어 + velocity feedforward
            axis.controller.config.control_mode = 3  # CONTROL_MODE_POSITION_CONTROL
            from odrive.enums import CONTROL_MODE_POSITION_CONTROL
            axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            print("모드: POS_CONTROL + INPUT_PASSTHROUGH (pos + vel feedforward)")
        else:
            axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            axis.controller.config.vel_ramp_rate = float(args.vel_ramp)
            print(f"모드: VEL_CONTROL + INPUT_VEL_RAMP (vel_ramp_rate={args.vel_ramp})")

        # 현재 위치 기록 (drift 검사용)
        start_pos = float(axis.encoder.pos_estimate)
        center = start_pos
        print(f"center (기준) = {center:+.4f} turn")

        # 첫 명령 0 으로
        if args.passthrough:
            axis.controller.input_pos = center
            axis.controller.input_vel = 0.0
        else:
            axis.controller.input_vel = 0.0

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

    print(f"시작. Ctrl+C 로 정지.")
    period = 1.0 / UPDATE_HZ
    t0 = time.time()
    last_log = t0

    try:
        while True:
            now = time.time()
            t = now - t0
            if t >= args.duration:
                break

            # envelope (fade in/out)
            env = envelope(t, args.duration, args.ramp)

            # 사인 위치 (이론) + 그 시간미분 = 속도 명령
            theory_pos = center + env * amp_turn * math.sin(omega * t)
            cmd_vel = env * omega * amp_turn * math.cos(omega * t)

            # 인코더 읽기
            try:
                actual_pos = float(axis.encoder.pos_estimate)
                actual_vel = float(axis.encoder.vel_estimate)
            except Exception as e:
                print(f"\n[비상] 인코더 읽기 실패: {e}")
                raise SafeStop()

            if not is_finite_safe(actual_pos, actual_vel):
                print(f"\n[비상] 인코더 비정상: pos={actual_pos} vel={actual_vel}")
                raise SafeStop()

            # 위치 drift 검사
            drift = actual_pos - theory_pos
            if abs(drift) > amp_turn * TRACKING_DRIFT_LIMIT_RATIO and t > args.ramp:
                print(f"\n[비상] drift {drift:+.4f} turn (>amp×{TRACKING_DRIFT_LIMIT_RATIO}). 정지.")
                raise SafeStop()

            # 한계 (실측 기준)
            if abs(actual_pos - center) > limit_turn:
                print(f"\n[비상] 위치 한계 초과: {actual_pos - center:+.4f}")
                raise SafeStop()

            # 명령 인가
            if args.passthrough:
                axis.controller.input_pos = theory_pos
                axis.controller.input_vel = cmd_vel
            else:
                axis.controller.input_vel = cmd_vel

            # 로그
            if now - last_log >= 0.5:
                sys.stdout.write(
                    f"\rt={t:6.2f}s env={env:4.2f}  "
                    f"v_cmd={cmd_vel:+.3f}  v_act={actual_vel:+.3f}  "
                    f"pos={actual_pos - center:+.4f}  drift={drift:+.4f}    "
                )
                sys.stdout.flush()
                last_log = now

            # 정주기 sleep
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
