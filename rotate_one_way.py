"""
rotate_one_way.py
===============================================================================
출력축 한 방향 N turn 회전 — 기어박스 cogging/덜그럭 진단용.

설계
----
방향 전환 없음 → stick-slip (정지마찰 ↔ 동마찰 전환) 영향 제거.
일정 속도 cruising → cogging 토크 ripple 가 있으면 그게 느낌으로 옴.
사인파 sweep 대비 훨씬 단순한 dynamics — 기어/모터 자체 문제 진단 용도.

사용
----
    python rotate_one_way.py --output-turns 10
    python rotate_one_way.py --output-turns 10 --motor-vel 3.0 --direction -1

옵션
----
    --output-turns  출력축 기준 회전수 (양수). 모터축 = output_turns × 8.
    --motor-vel     모터 회전 속도 [turn/s]. 기본 3.0 (= 출력축 0.375 turn/s)
    --direction     회전 방향 +1 / -1. 기본 +1
    --vel-ramp      가속률 [turn/s²]. 기본 2.0
    --vel-limit     속도 한계 [turn/s]. 기본 8.0
    --current-lim   전류 한계 [A]. 기본 10
"""
from __future__ import annotations

import argparse
import signal
import sys
import time

import motor_helpers as mh
from odrive.enums import AXIS_STATE_IDLE


class SafeStop(Exception):
    pass


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--output-turns", type=float, required=True,
                    help="출력축 회전수 (양수)")
    ap.add_argument("--motor-vel", type=float, default=3.0,
                    help="모터축 속도 [turn/s], 기본 3.0")
    ap.add_argument("--direction", type=int, default=1, choices=[1, -1],
                    help="회전 방향 +1 / -1")
    ap.add_argument("--vel-ramp", type=float, default=2.0,
                    help="가속률 [turn/s²]")
    ap.add_argument("--vel-limit", type=float, default=8.0,
                    help="안전 vel_limit [turn/s]")
    ap.add_argument("--current-lim", type=float, default=10.0,
                    help="전류 한계 [A]")
    args = ap.parse_args()

    if args.output_turns <= 0:
        print("[오류] output-turns > 0")
        return 1
    if args.motor_vel <= 0 or args.motor_vel > args.vel_limit:
        print(f"[오류] motor-vel 은 0 ~ vel-limit({args.vel_limit}) 사이")
        return 1

    motor_turns = args.output_turns * 8.0   # 8:1 기어
    target_vel = args.motor_vel * args.direction
    cruise_s = motor_turns / args.motor_vel
    ramp_s = args.motor_vel / args.vel_ramp  # 가/감속 시간
    total_s = cruise_s + 2 * ramp_s + 0.5    # 마진

    print(f"파라미터: 출력 {args.output_turns} turn (모터 {motor_turns:.1f} turn) "
          f"방향 {'+' if args.direction > 0 else '-'} "
          f"motor_vel={args.motor_vel:.2f} turn/s")
    print(f"  예상 가/감속 {ramp_s:.1f}s × 2 + cruise {cruise_s:.1f}s = "
          f"총 {total_s:.1f}s")

    try:
        odrv = mh.connect()
    except Exception as e:
        print(f"[오류] 연결 실패: {e}")
        return 1
    axis = odrv.axis0

    try:
        mh.enter_velocity_mode(
            axis,
            current_lim=args.current_lim,
            vel_limit=args.vel_limit,
            vel_ramp_rate=args.vel_ramp,
        )
    except Exception as e:
        print(f"[오류] vel 모드 진입 실패: {e}")
        mh.safe_stop(axis)
        return 1

    def _sigint(_sig, _frm):
        raise SafeStop()
    signal.signal(signal.SIGINT, _sigint)

    start_pos = float(axis.encoder.pos_estimate)
    print(f"시작 pos = {start_pos:+.4f} turn")

    try:
        # 명령 인가 — vel_ramp 가 자동으로 부드럽게 가속
        axis.controller.input_vel = target_vel
        print(f"  input_vel = {target_vel} → 가속 시작")

        # cruise 시작 시점까지 대기 (가속 중)
        t0 = time.time()
        last_log = t0

        # 목표 위치 도달까지 모니터링
        target_pos = start_pos + motor_turns * args.direction
        print(f"  목표 pos = {target_pos:+.4f}")

        while True:
            now = time.time()
            t = now - t0
            try:
                pos = float(axis.encoder.pos_estimate)
                vel = float(axis.encoder.vel_estimate)
                iq = float(axis.motor.current_control.Iq_measured)
            except Exception as e:
                print(f"\n[비상] 인코더 읽기 실패: {e}")
                raise SafeStop()

            # 0.5s 주기 로그
            if now - last_log >= 0.5:
                progress = (pos - start_pos) / (motor_turns * args.direction) * 100
                sys.stdout.write(
                    f"\rt={t:5.1f}s pos={pos:+.3f} ({progress:5.1f}%)  "
                    f"vel={vel:+.3f}  Iq={iq:+.3f}A   "
                )
                sys.stdout.flush()
                last_log = now

            # 목표 위치 근처 도달 → 감속 시작
            distance_remaining = (target_pos - pos) * args.direction
            decel_distance = (vel ** 2) / (2 * args.vel_ramp)
            if distance_remaining <= decel_distance:
                print(f"\n  목표 {distance_remaining:+.3f} turn 남음 → 감속 시작")
                axis.controller.input_vel = 0.0
                # 감속 완료 대기
                deadline = now + ramp_s + 0.5
                while time.time() < deadline:
                    try:
                        v = float(axis.encoder.vel_estimate)
                    except Exception:
                        break
                    if abs(v) < 0.05:
                        break
                    time.sleep(0.05)
                break

            # 타임아웃 — 너무 오래 걸리면 비상 정지
            if t > total_s + 5:
                print(f"\n[비상] 타임아웃 ({t:.1f}s)")
                raise SafeStop()

            time.sleep(0.02)

        # 마무리 위치 출력
        final_pos = float(axis.encoder.pos_estimate)
        actual_turns = (final_pos - start_pos) / 8.0  # 출력축 기준
        print(f"  최종 pos = {final_pos:+.4f}  (실측 출력축 회전 = {actual_turns:+.3f} turn)")

    except SafeStop:
        print("\n안전 정지...")
    except Exception as e:
        print(f"\n[오류] 루프 예외: {e}")
        return 1
    finally:
        mh.safe_stop(axis, ramp_s=1.0)
        print("axis0 → IDLE.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
