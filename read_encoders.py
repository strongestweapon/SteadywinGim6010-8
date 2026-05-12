"""
read_encoders.py
axis0(단일 축) 인코더의 위치/속도와 컨트롤러 setpoint 를 실시간 출력.
단위: turns (회전수), turns/s. 펌웨어 v0.6.5 / 단일축 보드 기준.

사용법:
    python read_encoders.py
    python read_encoders.py --hz 50
Ctrl+C 로 종료.
"""
import argparse
import sys
import time

import odrive

from motor_helpers import connect, is_finite_safe


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=20.0, help="표시 주기 [Hz]")
    args = ap.parse_args()
    period = 1.0 / max(args.hz, 1.0)

    try:
        odrv = connect()
    except Exception as e:
        print(f"[오류] 연결 실패: {e}")
        return 1

    enc = odrv.axis0.encoder
    ctrl = odrv.axis0.controller

    print()
    print("pos_estimate[turn]  vel_estimate[turn/s]  pos_abs   "
          "input_pos    pos_setpoint")
    print("-" * 78)

    fail_streak = 0
    try:
        while True:
            try:
                pos = float(enc.pos_estimate)
                vel = float(enc.vel_estimate)
                pos_abs = int(enc.pos_abs)
                inp = float(ctrl.input_pos)
                setp = float(ctrl.pos_setpoint)
            except Exception as e:
                fail_streak += 1
                if fail_streak == 1:
                    print(f"\n[경고] 읽기 실패: {e}")
                if fail_streak > 20:
                    print("\n[오류] 연속 읽기 실패 — 종료")
                    return 2
                time.sleep(period)
                continue
            fail_streak = 0

            if not is_finite_safe(pos, vel, inp, setp):
                print("\n[오류] 인코더 값 비정상 — 종료")
                return 2

            sys.stdout.write(
                f"\r{pos:+10.4f}        {vel:+10.4f}         "
                f"{pos_abs:6d}   {inp:+10.4f}   {setp:+10.4f}   "
            )
            sys.stdout.flush()
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n사용자 중단.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
