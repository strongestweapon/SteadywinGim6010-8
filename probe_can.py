"""
probe_can.py
ODrive CAN 설정 확인 — **읽기 전용** (아무 값도 변경/저장하지 않음).

T-2CAN(ESP32) 이행 전, ESP32 송신 속도와 맞춰야 하는 baud_rate 와
node_id / heartbeat / 엔코더 송신 주기 / watchdog 상태를 한 번에 출력.

사용법:
    python probe_can.py
"""
import sys
import odrive

# Windows 콘솔 한글 깨짐 방지
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass


CONNECT_TIMEOUT_S = 10


def safe(obj, name, default="<없음>"):
    """속성 안전 읽기 (펌웨어 버전별 경로 차이 흡수)."""
    try:
        return getattr(obj, name)
    except Exception as e:
        return f"<err: {e}>"


def fmt_serial(sn: int) -> str:
    return f"{sn:012X}"


def main() -> int:
    print(f"ODrive 검색 중... (최대 {CONNECT_TIMEOUT_S}초)")
    try:
        odrv = odrive.find_any(timeout=CONNECT_TIMEOUT_S)
    except TimeoutError:
        print("[오류] ODrive를 찾을 수 없습니다. USB 연결 / 드라이버 확인.")
        return 1
    except Exception as e:
        print(f"[오류] 연결 실패: {e}")
        return 1

    print(f"연결 OK — 시리얼 {fmt_serial(safe(odrv, 'serial_number', 0))}")
    print()

    # ---- 보드 전역 CAN 설정 (odrv0.can.config) ----
    print("== odrv0.can.config (전역 CAN 설정) ==")
    can = safe(odrv, "can")
    if isinstance(can, str):
        print(f"  can 객체 없음: {can}")
    else:
        cfg = safe(can, "config")
        # baud_rate 가 핵심 — ESP32(MCP2518FD) 와 반드시 일치
        for k in ("baud_rate", "protocol", "data_baud_rate"):
            v = safe(cfg, k)
            print(f"  {k:18s} = {v!r}")
        # 런타임 에러 카운터 (있으면)
        err = safe(can, "error")
        print(f"  error (런타임)     = {err!r}")

    print()

    # ---- 축별 CAN 설정 (axis0.config.can) + watchdog ----
    for name in ("axis0", "axis1"):
        axis = getattr(odrv, name, None)
        if axis is None:
            continue
        print(f"== {name}.config.can ==")
        acfg = safe(axis, "config")
        acan = safe(acfg, "can")
        for k in ("node_id", "is_extended", "heartbeat_rate_ms",
                  "encoder_rate_ms", "encoder_count_rate_ms",
                  "controller_error_rate_ms", "motor_error_rate_ms",
                  "iq_rate_ms", "bus_vi_rate_ms"):
            v = safe(acan, k)
            print(f"  {k:24s} = {v!r}")
        # watchdog (현재 비활성 0.0 예상 — CAN 운용 단계에서만 켤 것)
        print(f"  -- watchdog --")
        print(f"  enable_watchdog          = {safe(acfg, 'enable_watchdog')!r}")
        print(f"  watchdog_timeout         = {safe(acfg, 'watchdog_timeout')!r}")
        print()

    print("※ 읽기 전용 — 변경/저장 없음. baud_rate 를 ESP32 송신 속도와 일치시킬 것.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
