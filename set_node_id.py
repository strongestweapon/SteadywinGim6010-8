"""
set_node_id.py
===============================================================================
ODrive 의 CAN node_id 를 USB 로 변경 + flash 에 영구 저장.

왜 USB 인가
-----------
- node_id 는 "이 보드가 CAN 버스에서 몇 번으로 불릴지" 를 정하는 값.
  CAN 으로 바꾸려면 이미 통신이 돼야 하는 닭-달걀 문제 + ODrive 0.6.5 는
  CAN SDO(임의 파라미터 쓰기) 미지원 → **USB + save_configuration 이 유일**.
- 그래서 듀얼 모터 셋업 시: 각 보드를 한 대씩 USB 로 붙여 ID 를 1, 2 로 확정한
  뒤 같은 CAN 버스에 올린다. (둘 다 1 이면 버스에서 ID 충돌.)

⚠️ 안전 (이 스크립트가 건드리는 것 / 안 건드리는 것)
----------------------------------------------------
- 건드림   : axis0.config.can.node_id  (+ 필요 시 can.config.baud_rate 확인만)
- 안 건드림: phase_offset, pole_pairs, gear_ratio, 게인, index_offset,
             그 외 공장 캘리브 일체. save_configuration 은 node_id 변경분만 굽는다.

사용법
------
    python set_node_id.py                 # 현재 node_id 만 읽고 종료 (안전)
    python set_node_id.py --id 2          # node_id=2 로 변경 (확인 프롬프트 있음)
    python set_node_id.py --id 2 --yes    # 확인 생략
"""
import argparse
import sys
import time

import odrive

from motor_helpers import connect


def reconnect_after_save(timeout_total: float = 15.0):
    """save_configuration 호출 후 보드 reboot → 다시 찾기 (set_zero.py 와 동일 패턴)."""
    print("  보드 재부팅 대기 (최대 15초)...")
    deadline = time.time() + timeout_total
    while time.time() < deadline:
        try:
            return odrive.find_any(timeout=2)
        except Exception:
            time.sleep(0.5)
    raise RuntimeError("재부팅 후 보드 재인식 실패")


def read_node_id(odrv):
    """현재 node_id 와 baud_rate 를 읽어서 (node_id, baud) 반환."""
    node_id = int(odrv.axis0.config.can.node_id)
    try:
        baud = int(odrv.can.config.baud_rate)
    except Exception:
        baud = None
    return node_id, baud


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=None,
                    help="설정할 CAN node_id (0~62). 생략 시 현재값만 읽고 종료.")
    ap.add_argument("--yes", action="store_true", help="확인 없이 즉시 실행")
    args = ap.parse_args()

    try:
        odrv = connect()
    except Exception as e:
        print(f"[오류] 연결 실패: {e}")
        return 1

    sn = f"{odrv.serial_number:012X}"

    try:
        cur_id, baud = read_node_id(odrv)
    except Exception as e:
        print(f"[오류] node_id 읽기 실패: {e}")
        return 1

    baud_str = f"{baud}" if baud is not None else "읽기 실패"
    print(f"  serial={sn}")
    print(f"  현재 node_id   = {cur_id}")
    print(f"  현재 baud_rate = {baud_str}  (ODrive↔T-2CAN 는 500000 이어야 함)")

    # --- 읽기 전용 모드
    if args.id is None:
        print("  (--id 미지정 — 현재값만 확인하고 종료. 변경하려면 --id N)")
        return 0

    new_id = args.id
    if not (0 <= new_id <= 62):
        print(f"[오류] node_id 는 0~62 범위. 입력값={new_id}")
        return 1
    if new_id == cur_id:
        print(f"  이미 node_id={new_id} 입니다. 변경 없음.")
        return 0

    print(f"\n  변경: node_id  {cur_id}  →  {new_id}")
    print( "  (이 보드를 듀얼 버스에서 이 번호로 호출하게 됨. save_configuration 으로 영구 저장.)")
    if not args.yes:
        ans = input(f"  serial={sn} 보드의 node_id 를 {new_id} 로 굽고 저장할까요? [y/N]: ").strip().lower()
        if ans not in ("y", "yes"):
            print("취소.")
            return 0

    try:
        odrv.axis0.config.can.node_id = int(new_id)
        print("  save_configuration 실행 (보드 재부팅됨)...")
        odrv.save_configuration()
    except Exception as e:
        # save_configuration 은 보통 reboot 시켜 통신이 끊겨 예외를 던질 수 있음 — 정상일 수 있다.
        print(f"  (save_configuration 중 통신 끊김 — 정상일 수 있음: {e})")

    try:
        odrv = reconnect_after_save()
    except Exception as e:
        print(f"[경고] 재연결 실패(저장은 됐을 수 있음): {e}")
        print("  USB 재연결 후 `python set_node_id.py` 로 값 확인하세요.")
        return 1

    try:
        v_id, v_baud = read_node_id(odrv)
        if v_id == new_id:
            print(f"  ✓ 저장 완료. node_id = {v_id}, baud_rate = {v_baud}")
        else:
            print(f"  [경고] 재부팅 후 node_id = {v_id} (기대 {new_id}). 다시 시도 필요.")
            return 1
    except Exception as e:
        print(f"[경고] 검증 실패: {e}")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
