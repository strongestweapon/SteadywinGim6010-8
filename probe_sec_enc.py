"""
probe_sec_enc.py
2차 인코더 실제 존재 여부 검증. 1회성 도구.
출력축을 손으로 살짝 돌리면서 실행하면 값 변화로 확인 가능.

사용법:
    python probe_sec_enc.py        # 1회 측정 + 30초간 변화 관측
"""
import sys
import time
import odrive
from odrive.utils import dump_errors

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


def safe(obj, name, default="<없음>"):
    try:
        v = getattr(obj, name)
        return v
    except Exception as e:
        return f"<err: {e}>"


def main() -> int:
    print("ODrive 연결 중...")
    odrv = odrive.find_any(timeout=10)
    enc = odrv.axis0.encoder

    print()
    print("== 인코더 config 핵심 값 ==")
    cfg = enc.config
    for k in ("mode", "cpr", "sec_enc_cpr", "use_index", "pre_calibrated",
              "abs_spi_cs_gpio_pin", "direction", "index_offset",
              "use_index_offset", "phase_offset"):
        print(f"  {k:25s} = {safe(cfg, k)!r}")

    print()
    print("== 인코더 런타임 값 (1차) ==")
    for k in ("pos_estimate", "pos_abs", "pos_circular", "pos_cpr_counts",
              "count_in_cpr", "shadow_count", "vel_estimate",
              "is_ready", "index_found", "error", "delta_pos_cpr_counts",
              "spi_error_rate"):
        print(f"  {k:25s} = {safe(enc, k)!r}")

    print()
    print("== poll_sec_enc() 호출 시도 ==")
    poll = getattr(enc, "poll_sec_enc", None)
    if poll is None:
        print("  poll_sec_enc 메소드 없음")
    else:
        try:
            r = poll()
            print(f"  poll_sec_enc() return = {r!r}")
        except Exception as e:
            print(f"  poll_sec_enc() 예외: {e}")

    print()
    print("== 5초 동안 변화 관측 — 자동 ==")
    print(f"{'t':>5} {'pos_est':>10} {'pos_abs':>10} {'count_cpr':>10} {'delta':>10} {'shadow':>8}")
    t0 = time.time()
    try:
        while time.time() - t0 < 5.0:
            # poll_sec_enc 가 매 호출마다 새 값 fetch 한다고 가정 (안 그래도 무해)
            if poll is not None:
                try:
                    poll()
                except Exception:
                    pass
            line = (
                f"{time.time()-t0:5.1f} "
                f"{safe(enc, 'pos_estimate', 0):10.4f} "
                f"{safe(enc, 'pos_abs', 0)!s:>10} "
                f"{safe(enc, 'count_in_cpr', 0)!s:>10} "
                f"{safe(enc, 'delta_pos_cpr_counts', 0):10.4f} "
                f"{safe(enc, 'shadow_count', 0)!s:>8}"
            )
            print(line)
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    print()

    print()
    print("== dump_errors ==")
    try:
        dump_errors(odrv)
    except Exception as e:
        print(f"dump_errors 실패: {e}")

    print()
    print("== odrv0.misconfigured ==", safe(odrv, "misconfigured"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
