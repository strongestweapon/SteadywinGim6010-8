"""
set_can_term.py
===============================================================================
ODrive 온보드 CAN 120Ω 종단저항을 USB 로 켜고/끄기 + flash 영구 저장.

근거 (SteadyWin GIM6010-8 매뉴얼 P47):
    odrv0.can.config.r120_gpio_num = 5      # 종단 스위치가 물린 GPIO
    odrv0.can.config.enable_r120   = True   # True=종단 ON, False=OFF

즉 이 보드는 온보드 120Ω 을 GPIO5 로 소프트 제어함 (점퍼 아님). 듀얼 모터
한 버스 구성에서 "가운데/중복" 노드의 종단을 USB 로 꺼서 총 부하를 60Ω 로
맞출 때 사용.

⚠️ 안전
- 건드림   : can.config.r120_gpio_num, can.config.enable_r120 (+ save)
- 안 건드림: 모터/인코더 캘리브, 게인, index_offset, node_id 등 일체.
- save_configuration 은 위 두 값 변경분만 굽는다.

사용법
------
    python set_can_term.py            # 현재 종단 상태만 읽고 종료 (안전)
    python set_can_term.py --off      # 종단 OFF (확인 프롬프트)
    python set_can_term.py --on       # 종단 ON
    python set_can_term.py --off --yes
"""
import argparse
import sys
import time

import odrive

from motor_helpers import connect

R120_GPIO = 5  # 매뉴얼 P47 — GIM6010-8 의 종단 스위치 GPIO


def reconnect_after_save(timeout_total: float = 15.0):
    print("  보드 재부팅 대기 (최대 15초)...")
    deadline = time.time() + timeout_total
    while time.time() < deadline:
        try:
            return odrive.find_any(timeout=2)
        except Exception:
            time.sleep(0.5)
    raise RuntimeError("재부팅 후 보드 재인식 실패")


def read_term(odrv):
    """(enable_r120, gpio_num) 반환. 속성 없으면 (None, None)."""
    try:
        en = bool(odrv.can.config.enable_r120)
    except Exception:
        return None, None
    try:
        gp = int(odrv.can.config.r120_gpio_num)
    except Exception:
        gp = None
    return en, gp


def main() -> int:
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--on", action="store_true", help="종단 120Ω ON")
    g.add_argument("--off", action="store_true", help="종단 120Ω OFF")
    ap.add_argument("--yes", action="store_true", help="확인 없이 즉시 실행")
    args = ap.parse_args()

    try:
        odrv = connect()
    except Exception as e:
        print(f"[오류] 연결 실패: {e}")
        return 1

    sn = f"{odrv.serial_number:012X}"
    en, gp = read_term(odrv)

    if en is None:
        print(f"  serial={sn}")
        print("  [미지원] 이 펌웨어/보드에 can.config.enable_r120 속성이 없습니다.")
        print("           → 종단은 물리 점퍼/솔더패드로 처리해야 합니다.")
        return 1

    print(f"  serial={sn}")
    print(f"  현재 enable_r120  = {en}   ({'종단 ON' if en else '종단 OFF'})")
    print(f"  현재 r120_gpio_num = {gp}")

    # 읽기 전용
    if not args.on and not args.off:
        print("  (--on / --off 미지정 — 현재 상태만 확인하고 종료.)")
        return 0

    target = bool(args.on)  # --on → True, --off → False
    if target == en:
        print(f"  이미 {'ON' if target else 'OFF'} 입니다. 변경 없음.")
        return 0

    print(f"\n  변경: enable_r120  {en}  →  {target}   ({'ON' if target else 'OFF'})")
    if not args.yes:
        ans = input(f"  serial={sn} 보드 종단을 {'ON' if target else 'OFF'} 으로 굽고 저장할까요? [y/N]: ").strip().lower()
        if ans not in ("y", "yes"):
            print("취소.")
            return 0

    try:
        # gpio_num 이 비어있거나 다르면 매뉴얼 권장값(5)으로 보정
        if gp != R120_GPIO:
            odrv.can.config.r120_gpio_num = R120_GPIO
            print(f"  r120_gpio_num = {R120_GPIO} 설정")
        odrv.can.config.enable_r120 = target
        print("  save_configuration 실행 (보드 재부팅됨)...")
        odrv.save_configuration()
    except Exception as e:
        print(f"  (save_configuration 중 통신 끊김 — 정상일 수 있음: {e})")

    try:
        odrv = reconnect_after_save()
    except Exception as e:
        print(f"[경고] 재연결 실패(저장은 됐을 수 있음): {e}")
        return 1

    v_en, v_gp = read_term(odrv)
    if v_en == target:
        print(f"  ✓ 저장 완료. enable_r120 = {v_en} ({'ON' if v_en else 'OFF'}), gpio={v_gp}")
        return 0
    print(f"  [경고] 재부팅 후 enable_r120 = {v_en} (기대 {target}). 다시 시도 필요.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
