"""
probe_reboot_stability.py
save 없이 odrv.reboot() 만 3회 반복하면서 pos_estimate 가
매번 같은 값으로 돌아오는지 검증.

같으면 → 인코더는 안정 (set_zero 의 ±0.14 turn 어긋남은 save 절차 / 진자 흔들림 원인)
다르면 → 2차 인코더 멀티턴 복원이 불안정 또는 진자가 자유 진동 중
"""
import sys
import time

import odrive


try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


def reconnect(timeout: float = 20.0):
    t0 = time.time()
    last_err = None
    while time.time() - t0 < timeout:
        try:
            return odrive.find_any(timeout=2)
        except Exception as e:
            last_err = e
            time.sleep(0.5)
    raise RuntimeError(f"재연결 실패: {last_err}")


def main() -> int:
    print("초기 연결...")
    odrv = odrive.find_any(timeout=10)
    samples = []

    # 0회차 — reboot 전
    p = float(odrv.axis0.encoder.pos_estimate)
    print(f"[0회차 (reboot 전)] pos_estimate = {p:+.6f} turn")
    samples.append(p)

    for i in range(1, 4):
        print(f"\n--- {i}회차 reboot ---")
        try:
            odrv.reboot()
        except Exception as e:
            # reboot 자체는 통신 끊김이 정상
            print(f"  (reboot 통신 끊김 — 정상: {e})")
        time.sleep(1.5)
        try:
            odrv = reconnect()
        except Exception as e:
            print(f"[오류] 재연결 실패: {e}")
            return 1

        p = float(odrv.axis0.encoder.pos_estimate)
        print(f"[{i}회차] pos_estimate = {p:+.6f} turn  "
              f"(전 회차 대비 Δ = {p - samples[-1]:+.6f} turn = "
              f"{(p - samples[-1])*360:+.3f}° 모터축 = "
              f"{(p - samples[-1])*360/8:+.3f}° 출력축)")
        samples.append(p)

    print()
    print("== 요약 ==")
    for i, s in enumerate(samples):
        print(f"  회차 {i}: {s:+.6f}")
    spread = max(samples) - min(samples)
    print(f"  최대 - 최소 = {spread:+.6f} turn "
          f"= {spread*360:.2f}° 모터축 = {spread*360/8:.2f}° 출력축")
    if spread < 0.01:
        print("  → 인코더 안정. set_zero 어긋남은 save 절차 / 진자 흔들림이 원인.")
    elif spread < 1.0:
        print("  → 출력축이 약간 흔들렸거나 2차 인코더 노이즈.")
    else:
        print("  → 2차 인코더의 멀티턴 복원 인덱스가 매번 달라짐 가능.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
