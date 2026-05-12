"""
test_connect.py
ODrive USB 연결 확인 및 펌웨어 / 시리얼 / 버스 전압 출력.

사용법:
    python test_connect.py
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


def fmt_serial(sn: int) -> str:
    # ODrive 시리얼은 일반적으로 hex 12자리로 표기
    return f"{sn:012X}"


def main() -> int:
    print(f"ODrive 검색 중... (최대 {CONNECT_TIMEOUT_S}초)")
    try:
        odrv = odrive.find_any(timeout=CONNECT_TIMEOUT_S)
    except TimeoutError:
        print("[오류] ODrive를 찾을 수 없습니다. USB 연결 / Zadig 드라이버를 확인하세요.")
        return 1
    except Exception as e:
        print(f"[오류] 연결 실패: {e}")
        return 1

    try:
        sn = fmt_serial(odrv.serial_number)
        fw = f"{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}"
        hw = f"{odrv.hw_version_major}.{odrv.hw_version_minor}-{odrv.hw_version_variant}V"
        vbus = float(odrv.vbus_voltage)
    except Exception as e:
        print(f"[오류] 장치 정보 읽기 실패: {e}")
        return 1

    print("---- ODrive 연결 OK ----")
    print(f"시리얼     : {sn}")
    print(f"펌웨어     : v{fw}")
    print(f"하드웨어   : v{hw}")
    print(f"VBus 전압  : {vbus:6.2f} V")

    # 24V 시스템이므로 너무 낮으면 배터리 / 전원 점검
    if vbus < 18.0:
        print("[경고] 버스 전압이 낮습니다 (<18V). 전원/배터리를 확인하세요.")

    # 축 상태 (이 보드는 단일 축이므로 axis0 만)
    for name in ("axis0", "axis1"):
        axis = getattr(odrv, name, None)
        if axis is None:
            print(f"{name}: 존재하지 않음 (단일 축 보드)")
            continue
        try:
            state = int(axis.current_state)
            # 0.6.x: motor.is_calibrated / encoder.is_ready 경로 변경 가능 →
            #        존재 여부를 먼저 확인
            motor_cal = bool(getattr(axis.motor, "is_calibrated", "?"))
            enc_ready_attr = getattr(axis, "encoder", None)
            enc_ready = bool(enc_ready_attr.is_ready) if enc_ready_attr is not None else "n/a"
            print(f"{name}: state={state}  motor_cal={motor_cal}  encoder_ready={enc_ready}")
        except Exception as e:
            print(f"{name}: 상태 읽기 실패 ({e})")

    return 0


if __name__ == "__main__":
    sys.exit(main())
