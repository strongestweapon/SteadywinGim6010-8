"""
read_serial.py — T-2CAN 시리얼 출력을 정해진 시간 동안만 읽고 종료 (개발 보조).
사용:  python read_serial.py [PORT] [SECONDS]
예:    python read_serial.py COM5 7
"""
import sys
import time

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

import serial  # pyserial

port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 7.0
baud = 115200

try:
    s = serial.Serial(port, baud, timeout=0.5)
except Exception as e:
    print(f"[열기 실패] {port}: {e}")
    sys.exit(1)

print(f"[{port} @ {baud}, {dur:.0f}s 읽기]")
t0 = time.time()
try:
    while time.time() - t0 < dur:
        line = s.readline()
        if line:
            sys.stdout.write(line.decode("utf-8", "replace"))
            sys.stdout.flush()
finally:
    s.close()
print("\n[종료]")
