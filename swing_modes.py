"""
swing_modes.py
===============================================================================
모드 전환 데모 — 단일 모터(axis0).
시퀀스: 그네(X 동위상) → 비틀기(Z 역위상) → 정지 → 물결(X 고주파).

부드러운 램핑
-------------
- 각 모드는 자체 envelope (cosine fade-in / fade-out) 으로 진폭 0→amp→0.
- 모드 사이 짧은 휴식 (INTER_MODE_REST_S) 동안 center 위치 유지 → 자연 transition.
- 정지 모드는 freq=0 으로 표현, center 위치를 그대로 hold.
- snap-to-zero 가 적용되어 매 부팅마다 center 가 일관되게 잡힘.

NOTE
----
- 듀얼 모터 동기화(ESPNow)는 미구현. 단일 모터로 각 모드의 주파수/진폭 검증용.
  실제 X-swing / Z-twist 의 차이는 두 모터의 위상(동위상 vs 역위상) 으로
  결정되므로, 본 스크립트는 단일 모터에서 freq/amp 프로파일만 다르게 적용.
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
TRACKING_ERR_LIMIT = 0.5

# 시퀀스 정의: (이름, amp[°], freq[Hz], duration[s], ramp[s])
# freq=0 → 정지 (envelope 도 0 반환됨, center 유지)
SEQUENCE = [
    ("그네  (swing)",   25.0, 1.0,  8.0, 1.5),
    ("비틀기(twist)",   15.0, 1.2,  6.0, 1.2),
    ("정지  (hold)",     0.0, 0.0,  3.0, 0.0),
    ("물결  (ripple)",   5.0, 5.0,  6.0, 1.0),
]
INTER_MODE_REST_S = 0.6  # 모드 사이 휴식 시간 [s]


class SafeStop(Exception):
    pass


def run_mode(axis, center: float, name: str,
             amp_deg: float, freq_hz: float, duration_s: float,
             ramp_s: float, limit_turn: float) -> None:
    """한 모드의 사인파를 duration_s 동안 실행.

    Parameters
    ----------
    center : center 기준 위치 (snap-to-zero 보정값)
    amp_deg : 진폭 [모터축 °]
    freq_hz : 주파수. 0 이면 정지 모드.
    ramp_s : envelope fade-in/out 시간
    limit_turn : 안전 한계 [turn]
    """
    amp_turn = deg_to_turn(amp_deg)
    omega = 2.0 * math.pi * freq_hz
    period = 1.0 / UPDATE_HZ

    print(f"\n=== [{name}] amp={amp_deg}° freq={freq_hz}Hz "
          f"dur={duration_s}s ramp={ramp_s}s ===")
    t0 = time.time()
    last_log = t0

    while True:
        now = time.time()
        t = now - t0
        if t >= duration_s:
            return

        # 정지 모드는 envelope/sin 무시하고 center 유지
        if freq_hz == 0.0:
            target = center
            env = 0.0
        else:
            env = envelope(t, duration_s, ramp_s)
            target = center + env * amp_turn * math.sin(omega * t)

        # 명령 한계 검사
        if abs(target - center) > limit_turn:
            print(f"\n[비상] 명령 위치 한계 초과: {target - center:+.4f}")
            raise SafeStop()

        # 실측 인코더 검사
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

        # 명령 인가
        axis.controller.input_pos = target

        # 0.5초마다 진행 로그
        if now - last_log >= 0.5:
            sys.stdout.write(
                f"\rt={t:5.2f}s env={env:4.2f} "
                f"cmd={target - center:+.4f}  act={actual - center:+.4f}    "
            )
            sys.stdout.flush()
            last_log = now

        sleep_left = period - (time.time() - now)
        if sleep_left > 0:
            time.sleep(sleep_left)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--limit", type=float, default=60.0,
                    help="전 모드 공통 위치 한계 [모터축 °]")
    ap.add_argument("--current-lim", type=float, default=10.0, help="전류 한계 [A]")
    ap.add_argument("--vel-limit", type=float, default=20.0,
                    help="속도 한계 [turn/s]")
    ap.add_argument("--filter-hz", type=float, default=100.0,
                    help="POS_FILTER 대역폭 [Hz]. 매뉴얼 권장 = 명령주파수/2.")
    ap.add_argument("--no-snap", action="store_true",
                    help="snap-to-zero 비활성")
    args = ap.parse_args()
    limit_turn = deg_to_turn(args.limit)

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

    # Ctrl+C → SafeStop
    def _sigint(_sig, _frm):
        raise SafeStop()
    signal.signal(signal.SIGINT, _sigint)

    print(f"시작. center={center:+.4f} turn. 위치한계 ±{args.limit}°.")

    try:
        for name, amp, freq, dur, ramp in SEQUENCE:
            run_mode(axis, center, name, amp, freq, dur, ramp, limit_turn)
            # 모드 사이 자연 휴식 — 다음 모드의 fade-in 직전까지 center 유지
            t_rest = time.time()
            while time.time() - t_rest < INTER_MODE_REST_S:
                axis.controller.input_pos = center
                time.sleep(0.01)
        print("\n전 모드 완료.")
        return 0
    except SafeStop:
        print("\n안전 정지 진행 중...")
        return 0
    except Exception as e:
        print(f"\n[오류] 예외: {e}")
        return 1
    finally:
        safe_stop(axis)
        print("axis0 → IDLE.")


if __name__ == "__main__":
    sys.exit(main())
