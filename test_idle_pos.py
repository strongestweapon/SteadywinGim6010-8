"""
test_idle_pos.py
===============================================================================
"전류를 걸어야 위치를 안다" 가 사실인지 직접 눈으로 확인하는 진단.

요지
----
- MA600 은 절대(absolute) 인코더라 모터 전류(토크)와 무관하게 각도를 상시 측정.
- 1단계: IDLE(무여자) 상태에서 손으로 돌려도 pos_estimate / count_in_cpr 가
  실시간으로 변한다 → 전류 0 인데도 위치를 안다. (Iq_measured ≈ 0 확인)
- 2단계(--arm): VEL 모드 vel=0 으로 arm. 이제 Iq 가 흐르지만(버팀), 위치를
  홱 끌어당기지 않고 "지금 그 자리"를 잡는다 → arm 은 위치를 '아는' 게 아니라
  '잡는' 동작임을 확인. (USB 에선 idle 에도 위치가 읽히고, CAN(0.6.5) 에서만
  컨트롤러 소스라 idle pos 가 0 으로 와서 arm 이 필요했던 것.)

사용법
------
    python test_idle_pos.py              # 1단계만 (전류 없이 위치 읽기, 안전)
    python test_idle_pos.py --secs 12    # 관찰 시간 조절
    python test_idle_pos.py --arm        # 1단계 후 2단계(arm) 까지 (확인 프롬프트)
    python test_idle_pos.py --arm --yes
"""
import argparse
import sys
import time

from odrive.enums import (
    AXIS_STATE_IDLE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_VEL_RAMP,
)

from motor_helpers import connect, safe_stop


def read_row(axis):
    """현재 (state, pos, count_in_cpr, shadow, Iq) 를 안전하게 읽어 반환."""
    enc = axis.encoder
    try:    state = int(axis.current_state)
    except Exception: state = -1
    try:    pos = float(enc.pos_estimate)
    except Exception: pos = float("nan")
    try:    cic = int(enc.count_in_cpr)
    except Exception: cic = -1
    try:    sc = int(enc.shadow_count)
    except Exception: sc = -1
    try:    iq = float(axis.motor.current_control.Iq_measured)
    except Exception: iq = float("nan")
    return state, pos, cic, sc, iq


def monitor(axis, secs, hz, tag):
    """secs 동안 hz 로 위치/전류 출력."""
    print(f"\n=== {tag} ({secs:.0f}s, 손으로 천천히 돌려보세요) ===")
    print("  state   pos_estimate   count_in_cpr  shadow   Iq(A)")
    t0 = time.time()
    dt = 1.0 / hz
    while time.time() - t0 < secs:
        state, pos, cic, sc, iq = read_row(axis)
        print(f"  {state:<5}  {pos:+12.6f}   {cic:>10}   {sc:>6}   {iq:+6.3f}")
        time.sleep(dt)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=8.0, help="각 단계 관찰 시간")
    ap.add_argument("--hz", type=float, default=4.0, help="출력 주기")
    ap.add_argument("--arm", action="store_true", help="2단계(전류 인가) 까지 진행")
    ap.add_argument("--yes", action="store_true", help="arm 확인 생략")
    args = ap.parse_args()

    try:
        odrv = connect()
    except Exception as e:
        print(f"[오류] 연결 실패: {e}")
        return 1
    axis = odrv.axis0

    # IDLE 보장 (전류 0 상태에서 시작)
    if int(axis.current_state) != int(AXIS_STATE_IDLE):
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.3)

    # ---- 1단계: 전류 없이 위치 읽기 ----
    monitor(axis, args.secs, args.hz, "1단계: IDLE(무여자) — 전류 0, 위치는?")
    print("\n  ↑ state=1(IDLE), Iq≈0 인데도 손으로 돌리면 pos/count 가 변했을 것.")
    print("    → 위치 감지엔 전류가 필요 없음 (절대 인코더). 'arm=위치 잡기'지 '위치 알기'가 아님.")

    if not args.arm:
        print("\n  (--arm 안 줬으니 전류는 한 번도 안 걸고 종료. 안전.)")
        return 0

    # ---- 2단계: arm (전류 인가) ----
    if not args.yes:
        ans = input("\n  2단계: VEL vel=0 으로 arm(전류 인가)합니다. 진행? [y/N]: ").strip().lower()
        if ans not in ("y", "yes"):
            print("취소 (전류 안 걸고 종료).")
            return 0

    _, pos_before, _, _, _ = read_row(axis)
    print(f"\n  arm 직전 위치 = {pos_before:+.6f} turn (이 자리를 잡을 것)")
    try:
        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        axis.controller.input_vel = 0.0
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.3)
        if int(axis.current_state) != int(AXIS_STATE_CLOSED_LOOP_CONTROL):
            print(f"[오류] arm 실패 state={int(axis.current_state)} err={int(axis.error):#x}")
            safe_stop(axis)
            return 1
        monitor(axis, args.secs, args.hz, "2단계: CLOSED_LOOP vel=0 — 전류 흐름, 위치는?")
        _, pos_after, _, _, _ = read_row(axis)
        print(f"\n  ↑ state=8(CLOSED_LOOP), Iq 가 0 이 아님(버티는 중).")
        print(f"    arm 전 {pos_before:+.6f} → arm 후 {pos_after:+.6f} turn "
              f"(차이 {abs(pos_after-pos_before)*360:.2f}°)")
        print("    → 홱 끌어당기지 않고 '그 자리'를 잡음. 진자라면 늘어진 위치 그대로.")
    finally:
        print("\n  safe_stop → IDLE (전류 해제)")
        safe_stop(axis)
    return 0


if __name__ == "__main__":
    sys.exit(main())
