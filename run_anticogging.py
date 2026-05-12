"""
run_anticogging.py — ODrive anti-cogging calibration.

절차
----
1. 위치제어 모드 (CONTROL_MODE_POSITION_CONTROL + INPUT_MODE_PASSTHROUGH).
2. 현재 위치를 setpoint 로 잡고 CLOSED_LOOP_CONTROL 진입.
3. controller.start_anticogging_calibration() 호출.
4. 펌웨어가 16384 위치를 순차적으로 통과하며 각 위치 cogging Iq 측정.
5. anticogging_valid = True 또는 calib_anticogging = False 로 완료 감지.
6. IDLE 로 전환.

영구 저장은 사용자가 별도로 save_configuration() 해야 함 (본 스크립트는 안 함).

진행 상황은 stdout 으로 매 1초마다 출력. Ctrl+C 로 안전 중단.
"""
from __future__ import annotations

import sys
import time

from odrive.enums import (
    AXIS_STATE_IDLE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_POSITION_CONTROL,
    INPUT_MODE_PASSTHROUGH,
    INPUT_MODE_TRAP_TRAJ,
)

import motor_helpers as mh


def main() -> int:
    odrv = mh.connect()
    ax = odrv.axis0
    mh.ensure_calibrated(ax)

    # 안전 한계 보수적으로
    ax.motor.config.current_lim = 10.0
    ax.controller.config.vel_limit = 5.0

    # ODrive anti-cog cal 은 pos_setpoint_ 을 0~1 turn (절대값) 으로 설정.
    # 우리 모터는 +157 turn 에 있으니 cal 시작 시 컨트롤러가 0 으로 슬루.
    # vel_limit 일시 상향 + TRAP_TRAJ 로 빠르게 0 으로 이동.
    start_pos = float(ax.encoder.pos_estimate)
    print(f"start_pos = {start_pos:+.4f} → 0.0 으로 이동 후 cal 시작")

    if abs(start_pos) > 0.5:
        # TRAP_TRAJ 이동 — vel_limit 임시 상향
        ax.controller.config.vel_limit = 15.0
        ax.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        ax.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        ax.controller.input_pos = start_pos
        ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.3)
        if int(ax.current_state) != int(AXIS_STATE_CLOSED_LOOP_CONTROL):
            print(f"[오류] CLOSED_LOOP 진입 실패")
            ax.requested_state = AXIS_STATE_IDLE
            return 1
        ax.controller.input_pos = 0.0
        # 도달 대기 — 최대 60초
        t_start = time.time()
        last_log = t_start
        while time.time() - t_start < 60:
            done = bool(ax.controller.trajectory_done)
            pos = float(ax.encoder.pos_estimate)
            now = time.time()
            if now - last_log >= 1.0:
                print(f"  이동 중 pos={pos:+.3f}  done={done}")
                last_log = now
            if done and abs(pos) < 0.1:
                break
            time.sleep(0.1)
        # vel_limit 복원
        ax.controller.config.vel_limit = 5.0
        print(f"  도착 pos = {float(ax.encoder.pos_estimate):+.4f}")
    else:
        print(f"  이미 0 근처 — 이동 생략")

    # PASSTHROUGH 모드 — cal 중에는 펌웨어가 직접 pos_setpoint_ 조작
    ax.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    ax.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

    current_pos = float(ax.encoder.pos_estimate)
    ax.controller.input_pos = current_pos
    print(f"cal 시작 직전 pos = {current_pos:+.4f}")

    # CLOSED_LOOP 진입 (이미 들어 있을 수 있음)
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.3)
    state = int(ax.current_state)
    if state != int(AXIS_STATE_CLOSED_LOOP_CONTROL):
        err = int(ax.error)
        print(f"[오류] CLOSED_LOOP 진입 실패. state={state} error={err:#x}")
        ax.requested_state = AXIS_STATE_IDLE
        return 1

    # cal 시작 — 가속 위해 threshold 임시 상향 (cal 정확도 vs 속도 trade-off)
    ax.controller.config.anticogging.calib_pos_threshold = 10.0
    ax.controller.config.anticogging.calib_vel_threshold = 10.0
    print(f"calib_pos_threshold = {ax.controller.config.anticogging.calib_pos_threshold}")
    print(f"calib_vel_threshold = {ax.controller.config.anticogging.calib_vel_threshold}")

    print("start_anticogging_calibration() 호출")
    sys.stdout.flush()
    ax.controller.start_anticogging_calibration()

    cpr = int(ax.encoder.config.cpr)
    print(f"cpr = {cpr} 위치 스캔 시작")
    sys.stdout.flush()

    t0 = time.time()
    last_index = -1
    stall_count = 0  # index 가 안 증가하는 횟수

    try:
        while True:
            t = time.time() - t0

            try:
                idx = int(ax.controller.config.anticogging.index)
                calib = bool(ax.controller.config.anticogging.calib_anticogging)
                valid = bool(ax.controller.anticogging_valid)
            except Exception as e:
                print(f"\n[오류] 읽기 실패: {e}")
                break

            progress = idx / cpr * 100.0 if cpr > 0 else 0
            # 진행 추정: 남은 시간 = (현재 시간 / 진행률) - 현재 시간
            if idx > 0 and t > 0:
                eta_s = (cpr - idx) / idx * t
                eta_str = f"  ETA {eta_s:5.0f}s"
            else:
                eta_str = ""

            print(f"\rt={t:6.1f}s  index={idx:5d}/{cpr} ({progress:5.1f}%)  "
                  f"calib={calib} valid={valid}{eta_str}      ", end="")
            sys.stdout.flush()

            # 완료 감지
            if valid:
                print()
                print(f"✓ anticogging_valid = True  (소요 {t:.1f}s)")
                break
            if not calib and idx > 100:
                # calib 가 끝났는데 valid 안 됐으면 실패
                print()
                print(f"⚠ calib_anticogging = False, valid 도 False. cal 실패 의심.")
                break

            # 정지 감지
            if idx == last_index:
                stall_count += 1
            else:
                stall_count = 0
                last_index = idx
            if stall_count > 30 and idx < cpr - 5:
                print()
                print(f"⚠ 30초 동안 index 정지 (idx={idx}/{cpr}). 모터 외력? 한계?")
                # 그래도 계속 — 사용자 판단 후 Ctrl+C 가능
                stall_count = 0

            # 안전 타임아웃: 60분
            if t > 60 * 60:
                print()
                print(f"⏱ 60분 초과 — 중단")
                break

            time.sleep(1.0)

    except KeyboardInterrupt:
        print()
        print("Ctrl+C — 중단")
    finally:
        try:
            ax.requested_state = AXIS_STATE_IDLE
            print("axis0 → IDLE")
        except Exception:
            pass

        # 결과 요약
        try:
            print(f"  최종 anticogging_valid = {bool(ax.controller.anticogging_valid)}")
            print(f"  최종 index             = {int(ax.controller.config.anticogging.index)}")
            print(f"  현재 anticogging_enabled = {bool(ax.controller.config.anticogging.anticogging_enabled)}")
        except Exception:
            pass

    print()
    print("영구 저장하려면: python -c \"import motor_helpers; o=motor_helpers.connect(); o.save_configuration()\"")
    return 0


if __name__ == "__main__":
    sys.exit(main())
