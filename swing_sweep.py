"""
swing_sweep.py
===============================================================================
사인파 위치 제어, 주파수 선형 sweep (천천히 → 빠르게).
영상 촬영용으로 시간에 따라 속도가 시각적으로 명확히 변화.

phase 식 (선형 chirp)
---------------------
순간 주파수 f(t) = f_start + k * t,  k = (f_end - f_start) / duration
위상      φ(t) = 2π * (f_start * t + 0.5 * k * t²)
명령      pos(t) = center + env(t) * amp * sin(φ(t))

env(t) 는 양 끝 ramp 구간에서 cosine fade — 갑작스러운 시작/정지 방지.

사용
----
    python swing_sweep.py --amp 240 --f-start 0.2 --f-end 1.0 --duration 20
        모터축 ±240° (출력축 ±30°), 0.2→1.0 Hz, 20초.

옵션
----
    --amp         진폭 [모터축 °]. 출력축 진폭 = amp/8.
    --f-start     시작 주파수 [Hz]
    --f-end       끝 주파수 [Hz]
    --duration    지속시간 [s]
    --ramp        진폭 fade-in/out 시간 [s], 기본 1.5
    --limit       위치 한계 [모터축 °], 기본 amp*1.5
    --current-lim 전류 한계 [A], 기본 10
    --vel-limit   속도 한계 [turn/s], 기본 12 (영구값 5 override)
    --filter-hz   POS_FILTER 대역폭 [Hz], 기본 100 (200Hz 명령 / 2)
"""
import argparse
import math
import signal
import sys
import time

from motor_helpers import (
    connect,
    enter_position_mode,
    safe_stop,
    envelope,
    deg_to_turn,
    is_finite_safe,
)


UPDATE_HZ = 200
TRACKING_ERR_LIMIT = 0.5  # turn


class SafeStop(Exception):
    pass


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--amp", type=float, required=True,
                    help="진폭 [모터축 °]")
    ap.add_argument("--f-start", type=float, required=True, help="시작 주파수 [Hz]")
    ap.add_argument("--f-end", type=float, required=True, help="끝 주파수 [Hz]")
    ap.add_argument("--duration", type=float, required=True, help="지속시간 [s]")
    ap.add_argument("--ramp", type=float, default=1.5,
                    help="진폭 fade-in/out [s]")
    ap.add_argument("--limit", type=float, default=None,
                    help="위치 한계 [모터축 °], 기본 amp*1.5")
    ap.add_argument("--current-lim", type=float, default=10.0,
                    help="전류 한계 [A]")
    ap.add_argument("--vel-limit", type=float, default=12.0,
                    help="속도 한계 [turn/s]. 영구 저장값(5) override 기본 12.")
    ap.add_argument("--filter-hz", type=float, default=100.0,
                    help="POS_FILTER 대역폭 [Hz]")
    ap.add_argument("--no-snap", action="store_true",
                    help="snap-to-zero 비활성")
    args = ap.parse_args()

    if args.amp <= 0 or args.duration <= 0:
        print("[오류] amp / duration 양수")
        return 1
    if args.f_start <= 0 or args.f_end <= 0:
        print("[오류] f-start / f-end 양수")
        return 1

    limit_deg = args.limit if args.limit is not None else args.amp * 1.5
    if limit_deg < args.amp:
        print("[오류] limit < amp")
        return 1
    if args.ramp * 2 > args.duration:
        print(f"[경고] ramp*2 > duration — full-amp 구간 없음")

    amp_turn = deg_to_turn(args.amp)
    limit_turn = deg_to_turn(limit_deg)
    f0 = args.f_start
    fT = args.f_end
    T = args.duration
    k = (fT - f0) / T   # 주파수 변화율 [Hz/s]

    # 최대 순간 peak velocity 사전 추정 — vel_limit 안에 들어오는지 검증
    f_max = max(f0, fT)
    peak_vel_pred = 2 * math.pi * f_max * amp_turn   # turn/s
    print(f"파라미터: amp={args.amp}° ({amp_turn:.4f} turn) "
          f"sweep {f0}→{fT} Hz over {T}s "
          f"ramp={args.ramp}s filter={args.filter_hz}Hz")
    print(f"  예상 최대 peak_vel = {peak_vel_pred:.2f} turn/s "
          f"(vel_limit={args.vel_limit})")
    if peak_vel_pred > args.vel_limit * 0.9:
        print(f"  ⚠ peak_vel 이 vel_limit 의 90% 초과 — 한계 걸릴 위험")

    try:
        odrv = connect()
    except Exception as e:
        print(f"[오류] 연결 실패: {e}")
        return 1
    axis = odrv.axis0

    try:
        center = enter_position_mode(
            axis,
            current_lim=args.current_lim,
            vel_limit=args.vel_limit,
            input_filter_hz=args.filter_hz,
            snap_to_zero=not args.no_snap,
        )
    except Exception as e:
        print(f"[오류] 위치모드 진입 실패: {e}")
        safe_stop(axis)
        return 1

    def _sigint(_sig, _frm):
        raise SafeStop()
    signal.signal(signal.SIGINT, _sigint)

    print(f"시작. center={center:+.4f}. {T:.1f}초 후 자동 정지.")
    period = 1.0 / UPDATE_HZ
    t0 = time.time()
    last_log = t0

    try:
        while True:
            now = time.time()
            t = now - t0
            if t >= T:
                break

            # 순간 주파수 (로깅용)
            f_inst = f0 + k * t

            # 위상 = 2π × (f0×t + 0.5×k×t²) — chirp
            phase = 2.0 * math.pi * (f0 * t + 0.5 * k * t * t)

            env = envelope(t, T, args.ramp)
            target = center + env * amp_turn * math.sin(phase)

            if abs(target - center) > limit_turn:
                print(f"\n[비상] 명령 한계 초과: {target - center:+.4f}")
                raise SafeStop()

            try:
                actual = float(axis.encoder.pos_estimate)
            except Exception as e:
                print(f"\n[비상] 인코더 읽기 실패: {e}")
                raise SafeStop()

            if not is_finite_safe(actual):
                print(f"\n[비상] 인코더 값 비정상: {actual}")
                raise SafeStop()

            if abs(actual - center) > limit_turn:
                print(f"\n[비상] 실측 한계 초과: {actual - center:+.4f}")
                raise SafeStop()
            if abs(target - actual) > TRACKING_ERR_LIMIT:
                print(f"\n[비상] 추종 오차 초과: {target - actual:+.4f}")
                raise SafeStop()

            axis.controller.input_pos = target

            if now - last_log >= 0.5:
                sys.stdout.write(
                    f"\rt={t:5.2f}s f={f_inst:.3f}Hz env={env:.2f}  "
                    f"cmd={target - center:+.4f}  act={actual - center:+.4f}    "
                )
                sys.stdout.flush()
                last_log = now

            sleep_left = period - (time.time() - now)
            if sleep_left > 0:
                time.sleep(sleep_left)

        print("\n지속시간 완료.")
        return 0

    except SafeStop:
        print("\n안전 정지 진행 중...")
        return 0
    except Exception as e:
        print(f"\n[오류] 루프 예외: {e}")
        return 1
    finally:
        safe_stop(axis)
        print("axis0 → IDLE.")


if __name__ == "__main__":
    sys.exit(main())
