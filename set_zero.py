"""
set_zero.py
사용자 기구부 0점을 펌웨어 flash 에 영구 저장.

방식 (SteadyWin 매뉴얼 P43, P33 권장):
    odrv0.axis0.encoder.config.index_offset = odrv0.axis0.encoder.pos_estimate
    odrv0.save_configuration()  # 보드 재부팅

원리:
- pos_estimate 는 인코더 0점 기준 위치
- 현재 위치를 index_offset 으로 저장 → 이후부터 pos_estimate 가 user zero 기준
- use_index_offset=True 면 부팅 시 자동 적용
- save_configuration() 이 flash 에 굽기 + 보드 reboot

사용법:
    python set_zero.py              # 확인 프롬프트 있음
    python set_zero.py --yes        # 확인 생략
    python set_zero.py --undo       # index_offset 을 0 으로 되돌림
"""
import argparse
import json
import sys
import time
from pathlib import Path

import odrive
from odrive.enums import AXIS_STATE_IDLE

from motor_helpers import connect


BACKUP_PATH = Path(__file__).with_name("zero_offset.json")


def save_backup(payload: dict) -> None:
    base = {}
    if BACKUP_PATH.exists():
        try:
            base = json.loads(BACKUP_PATH.read_text(encoding="utf-8"))
        except Exception:
            base = {}
    base.update(payload)
    base["timestamp"] = time.strftime("%Y-%m-%d %H:%M:%S")
    BACKUP_PATH.write_text(
        json.dumps(base, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(f"  백업 저장: {BACKUP_PATH}")


def ensure_idle(axis) -> None:
    if int(axis.current_state) != int(AXIS_STATE_IDLE):
        print("  IDLE 로 전환합니다...")
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.3)


def reconnect_after_save() -> object:
    """save_configuration 호출 후 보드 reboot → 다시 찾기."""
    print("  보드 재부팅 대기 (최대 15초)...")
    # save_configuration 호출 자체로 USB 가 한 번 끊김
    for _ in range(30):
        try:
            odrv = odrive.find_any(timeout=2)
            return odrv
        except Exception:
            time.sleep(0.5)
    raise RuntimeError("재부팅 후 보드 재인식 실패")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--yes", action="store_true", help="확인 없이 즉시 실행")
    ap.add_argument("--undo", action="store_true",
                    help="index_offset 을 0 으로 되돌리고 저장")
    args = ap.parse_args()

    try:
        odrv = connect()
    except Exception as e:
        print(f"[오류] 연결 실패: {e}")
        return 1

    axis = odrv.axis0
    enc = axis.encoder

    try:
        ensure_idle(axis)
    except Exception as e:
        print(f"[오류] IDLE 전환 실패: {e}")
        return 1

    try:
        current_offset = float(enc.config.index_offset)
        current_pos = float(enc.pos_estimate)
        use_offset = bool(enc.config.use_index_offset)
    except Exception as e:
        print(f"[오류] 인코더 값 읽기 실패: {e}")
        return 1

    print(f"  현재 index_offset    = {current_offset:+.6f} turn")
    print(f"  현재 pos_estimate    = {current_pos:+.6f} turn  (user zero 기준)")
    print(f"  use_index_offset     = {use_offset}")
    if not use_offset:
        print("  [경고] use_index_offset 이 False — 영구 0점이 적용되지 않습니다.")
        print("         odrv0.axis0.encoder.config.use_index_offset = True 로 켜고 save 필요.")

    # --- undo 분기
    if args.undo:
        if not args.yes:
            ans = input("  index_offset = 0 으로 되돌리고 저장할까요? [y/N]: ").strip().lower()
            if ans not in ("y", "yes"):
                print("취소.")
                return 0
        save_backup({"prev_index_offset_turn": current_offset, "action": "undo"})
        try:
            enc.config.index_offset = 0.0
            print("  index_offset = 0.0 적용. save_configuration 실행...")
            odrv.save_configuration()
        except Exception as e:
            print(f"[오류] 저장 실패: {e}")
            return 1
        try:
            odrv = reconnect_after_save()
            print(f"  재부팅 후 index_offset = "
                  f"{float(odrv.axis0.encoder.config.index_offset):+.6f}, "
                  f"pos_estimate = {float(odrv.axis0.encoder.pos_estimate):+.6f}")
        except Exception as e:
            print(f"[경고] 재연결 실패(저장은 됐을 수 있음): {e}")
        return 0

    # --- 정상 set
    # 새 index_offset = 기존 index_offset + 현재 pos_estimate
    # pos_estimate 자체가 이미 (raw - 기존 offset) 이므로, raw 기준의 새 offset 은
    # current_pos_raw = current_pos + current_offset 가 됨.
    new_offset = current_offset + current_pos

    print(f"  새 index_offset      = {new_offset:+.6f} turn  "
          f"(저장 후 pos_estimate 는 0 근처가 됨)")
    if not args.yes:
        ans = input("  이 위치를 user zero 로 굽고 저장할까요? [y/N]: ").strip().lower()
        if ans not in ("y", "yes"):
            print("취소.")
            return 0

    save_backup({
        "prev_index_offset_turn": current_offset,
        "prev_pos_estimate_turn": current_pos,
        "new_index_offset_turn": new_offset,
        "action": "set",
    })

    try:
        enc.config.index_offset = new_offset
        if not use_offset:
            print("  use_index_offset = True 로 같이 설정.")
            enc.config.use_index_offset = True
        print("  save_configuration 실행 (보드 재부팅됨)...")
        odrv.save_configuration()
    except Exception as e:
        # save_configuration 은 보통 reboot 시켜서 통신 끊김을 예외로 던질 수 있음
        print(f"  (save_configuration 중 통신 끊김 — 정상일 수 있음: {e})")

    try:
        odrv = reconnect_after_save()
    except Exception as e:
        print(f"[경고] 재연결 실패: {e}")
        print("  USB 케이블 / 전원 확인 후 다시 시도하세요.")
        return 1

    try:
        new_pos = float(odrv.axis0.encoder.pos_estimate)
        new_off = float(odrv.axis0.encoder.config.index_offset)
        print(f"  ✓ 저장 완료. index_offset = {new_off:+.6f}, "
              f"pos_estimate = {new_pos:+.6f}")
        if abs(new_pos) > 0.02:
            print(f"  [참고] pos_estimate 가 0 근처는 아님 — "
                  f"보드가 reboot 직전 위치와 약간 달라졌을 수 있음.")
    except Exception as e:
        print(f"[경고] 검증 실패: {e}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
