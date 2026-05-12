"""
swing_sine.py
===============================================================================
사인파 위치 제어 — 단일 축(axis0).

부드러운 램핑 정책
-----------------
1. ODrive 내장 입력 필터: INPUT_MODE_POS_FILTER (input_filter_bandwidth Hz lowpass)
   → input_pos 의 급변을 펌웨어가 자체적으로 부드럽게 추종.
2. 외부 envelope: 시작 ramp_s 초 동안 진폭 0→amp 로 cosine fade-in,
                   마지막 ramp_s 초 동안 amp→0 으로 cosine fade-out.
   → 사인파가 사일런트하게 시작/종료. 진자가 갑자기 튀지 않음.
3. 부팅 분기 자동 보정 (snap-to-zero): GIM6010-8 의 mono-turn 절대 인코더가
   부팅 시 두 분기 사이를 토글하는 문제를 motor_helpers 가 자동으로 격자에
   snap → 매 부팅마다 swing range 가 일관됨.

안전 종료
---------
- Ctrl+C, 한계 초과(amp / 추종 오차), 인코더 NaN 등 어떤 경로로든
  safe_stop() → 현재 위치 잡고 IDLE.

단위
----
- ODrive 의 position 단위는 turns (모터축 기준). 1 turn = 모터 360°.
- --amp / --limit 은 "모터축 ° " 입력 → 내부에서 turn 으로 변환.
- 출력축 각도는 8:1 기어비로 별도 환산 필요 (출력축 ° = 모터축 ° / 8).

사용법
------
    python swing_sine.py --amp 30 --freq 1.0 --duration 60
    python swing_sine.py --amp 20 --freq 0.8 --duration 30 --limit 45 --current-lim 8
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


UPDATE_HZ = 200            # 위치 명령 갱신 주기 (200Hz = 5ms 마다)
TRACKING_ERR_LIMIT = 0.5   # 명령-실측 오차 한계 [turn] — 초과 시 비상 정지


class SafeStop(Exception):
    """Ctrl+C 또는 한계 초과 시 던지는 신호. finally 블록에서 safe_stop 호출."""
    pass


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--amp", type=float, required=True,
                    help="진폭 [모터축 °]. 출력축 진폭 = amp/8.")
    ap.add_argument("--freq", type=float, required=True, help="주파수 [Hz]")
    ap.add_argument("--duration", type=float, required=True,
                    help="지속시간 [s]. envelope ramp 포함.")
    ap.add_argument("--limit", type=float, default=None,
                    help="위치 한계 [모터축 °]. 미지정 시 amp*1.5.")
    ap.add_argument("--ramp", type=float, default=1.5,
                    help="진폭 fade-in / fade-out 시간 [s]. ramp*2 < duration 권장.")
    ap.add_argument("--current-lim", type=float, default=10.0,
                    help="전류 한계 [A]. 무대용 보수값 (공장 기본 60A).")
    ap.add_argument("--vel-limit", type=float, default=20.0,
                    help="속도 한계 [turn/s] — peak 사인파 속도가 이 안에 들어와야 함.")
    ap.add_argument("--filter-hz", type=float, default=100.0,
                    help="POS_FILTER 대역폭 [Hz]. 매뉴얼 권장 = 명령전송주파수/2 "
                         "(우리는 200Hz/2=100Hz).")
    ap.add_argument("--no-snap", action="store_true",
                    help="snap-to-zero 보정 비활성 (raw pos_estimate 를 origin 으로).")
    args = ap.parse_args()

    # 입력값 검증 — 양수, 한계 일관성
    if args.amp <= 0 or args.freq <= 0 or args.duration <= 0:
        print("[오류] amp / freq / duration 은 모두 양수.")
        return 1

    limit_deg = args.limit if args.limit is not None else args.amp * 1.5
    if limit_deg < args.amp:
        print("[오류] limit 이 amp 보다 작습니다.")
        return 1
    if args.ramp * 2 > args.duration:
        print(f"[경고] ramp({args.ramp}s)*2 가 duration({args.duration}s) 보다 큼 — "
              f"full-amplitude 구간이 없음. envelope 가 0→max→0 으로 즉시 갈 것.")

    # 모터축 ° → turn 변환 (ODrive 내부 단위)
    amp_turn = deg_to_turn(args.amp)
    limit_turn = deg_to_turn(limit_deg)
    omega = 2.0 * math.pi * args.freq  # 각주파수 [rad/s]

    print(f"파라미터: amp={args.amp}° ({amp_turn:.4f} turn) "
          f"freq={args.freq}Hz duration={args.duration}s "
          f"limit={limit_deg}° ramp={args.ramp}s "
          f"current_lim={args.current_lim}A filter={args.filter_hz}Hz")

    # ODrive 연결
    try:
        odrv = connect()
    except Exception as e:
        print(f"[오류] 연결 실패: {e}")
        return 1

    axis = odrv.axis0

    # 위치 제어 모드 진입 + center 결정 (snap-to-zero 자동 적용)
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

    # Ctrl+C 처리: SafeStop 예외로 변환 → 메인 루프의 except 가 받음
    def _sigint(_sig, _frm):
        raise SafeStop()
    signal.signal(signal.SIGINT, _sigint)

    print(f"시작. center = {center:+.4f} turn ({center*360:.1f}° 모터축). "
          f"Ctrl+C 로 정지.")
    period = 1.0 / UPDATE_HZ
    t0 = time.time()
    last_log = t0

    try:
        # 메인 루프 — 200Hz 갱신
        while True:
            now = time.time()
            t = now - t0
            if t >= args.duration:
                break

            # 사인파 명령 계산: center + envelope × amp × sin(ωt)
            env = envelope(t, args.duration, args.ramp)
            target = center + env * amp_turn * math.sin(omega * t)

            # 명령값 자체가 한계 넘으면 (이론상 amp <= limit 이라 안 일어남)
            if abs(target - center) > limit_turn:
                print(f"\n[비상] 명령 위치 한계 초과: {target - center:+.4f}")
                raise SafeStop()

            # 실측 인코더 값 검사
            try:
                actual = float(axis.encoder.pos_estimate)
            except Exception as e:
                print(f"\n[비상] 인코더 읽기 실패: {e}")
                raise SafeStop()

            if not is_finite_safe(actual):
                print(f"\n[비상] 인코더 값 비정상: {actual}")
                raise SafeStop()

            # 한계 1: 실측 위치가 center ± limit 안에 있는가?
            if abs(actual - center) > limit_turn:
                print(f"\n[비상] 실측 한계 초과: {actual - center:+.4f} turn")
                raise SafeStop()
            # 한계 2: 명령-실측 추종 오차가 TRACKING_ERR_LIMIT 안인가?
            #         (POS_FILTER 가 부드럽게 만들어서 보통 작은 값)
            if abs(target - actual) > TRACKING_ERR_LIMIT:
                print(f"\n[비상] 추종 오차 초과: {target - actual:+.4f} turn")
                raise SafeStop()

            # 명령 인가
            axis.controller.input_pos = target

            # 0.5초마다 진행 로그
            if now - last_log >= 0.5:
                sys.stdout.write(
                    f"\rt={t:6.2f}s env={env:4.2f}  "
                    f"cmd={target - center:+.4f}  act={actual - center:+.4f}    "
                )
                sys.stdout.flush()
                last_log = now

            # 정주기 sleep — 처리 시간 빼고 period 채움
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
        # 어떤 경로든 finally 에서 safe_stop. 현재 위치 잡고 IDLE.
        safe_stop(axis)
        print("axis0 → IDLE.")


if __name__ == "__main__":
    sys.exit(main())
