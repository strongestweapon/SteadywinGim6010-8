"""
measure_freeswing.py — 봉 자유진동(중력 진자) 고유주파수 측정
===============================================================================
모터 STOP(무여자) 상태에서 봉을 손으로 밀어 놓으면, IMU(봉 가운데)가 보내는
telemetry(tilt/gyro)를 받아 자유 감쇠 진동의 주기를 측정한다.

원리:
  - ESP32 는 "마지막으로 명령패킷을 보낸 주소"로 telemetry 를 회신함.
    → 이 스크립트가 run=0(idle) 명령을 보내 그 주소를 차지하고 telemetry 수신.
    (실행 중 controller_app/main.py 는 닫아둘 것 — 안 그러면 telemetry 가 갈림)
  - 모터는 run=0 만 보내므로 계속 무여자(안전). 봉은 중력으로 자유 진동.

분석:
  - tilt(°) 와 가장 활발한 gyro 축을 FFT → 0.1~5Hz 의 dominant 주파수.
  - gyro zero-cross 로도 교차 검증.

사용:
  python measure_freeswing.py            # 15초 측정 (기본 IP)
  python measure_freeswing.py --secs 20 --ip 192.168.0.102
"""
from __future__ import annotations
import argparse
import socket
import sys
import time

import numpy as np

import protocol as proto

# Windows cp949 콘솔에서 한글/특수문자 깨짐·크래시 방지
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.0.102")
    ap.add_argument("--secs", type=float, default=15.0)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    sock.bind(("", 0))  # 임시 포트 — ESP32 가 이 주소로 telemetry 회신
    dst = (args.ip, proto.UDP_PORT)

    print(f">>> {args.secs:.0f}초 측정. 지금 봉을 한쪽으로 밀었다 놓으세요 (모터는 무여자 유지).")
    print("    (controller_app 메인 앱은 닫혀 있어야 telemetry 안 갈림)\n")

    ts, tilt, pitch, roll, gx, gy, gz = [], [], [], [], [], [], []
    seq = 0
    t0 = time.time()
    last_send = 0.0
    last_print = 0.0
    n_telem = 0
    while True:
        now = time.time()
        if now - t0 >= args.secs:
            break
        # idle 명령 송신(50Hz) — ESP32 가 우리 주소로 telemetry 보내게 + 모터 무여자
        if now - last_send >= 0.02:
            last_send = now
            seq = (seq + 1) & 0xFFFF
            try:
                sock.sendto(proto.pack(seq, 0, 0.0, 0.0, 0.0), dst)
            except OSError:
                pass
        # telemetry 수신
        try:
            while True:
                data, _ = sock.recvfrom(256)
                t = proto.unpack_telem(data)
                if t:
                    ts.append(now - t0)
                    tilt.append(t["tilt"]); pitch.append(t["pitch"]); roll.append(t["roll"])
                    gx.append(t["gx"]); gy.append(t["gy"]); gz.append(t["gz"])
                    n_telem += 1
        except (BlockingIOError, OSError):
            pass
        # 진행 표시
        if now - last_print >= 1.0:
            last_print = now
            cur = (f"tilt={tilt[-1]:.1f}° gyro=({gx[-1]:.2f},{gy[-1]:.2f},{gz[-1]:.2f})"
                   if tilt else "수신 대기...")
            print(f"  t={now-t0:4.1f}s  telem={n_telem:4d}  {cur}")
        time.sleep(0.002)

    if n_telem < 20:
        print(f"\n[오류] telemetry 샘플 부족({n_telem}). "
              f"앱이 떠있거나 IMU 미동작/IP 확인. 메인 앱 닫고 재시도.")
        return 1

    ts = np.array(ts)
    fs = n_telem / (ts[-1] - ts[0]) if ts[-1] > ts[0] else 0
    print(f"\n=== 분석 (샘플 {n_telem}, ~{fs:.0f}Hz) ===")

    # 균일 시간축으로 리샘플 (FFT 용)
    tu = np.linspace(ts[0], ts[-1], len(ts))
    def dominant_freq(sig):
        s = np.interp(tu, ts, np.array(sig, dtype=float))
        s = s - np.mean(s)
        if np.max(np.abs(s)) < 1e-6:
            return None, 0.0
        win = np.hanning(len(s))
        sp = np.abs(np.fft.rfft(s * win))
        fr = np.fft.rfftfreq(len(s), d=(tu[-1]-tu[0])/(len(tu)-1))
        band = (fr >= 0.1) & (fr <= 5.0)
        if not band.any():
            return None, 0.0
        k = np.argmax(sp[band])
        return fr[band][k], float(sp[band][k])

    # 부호있는 신호(pitch/roll/gyro)가 실제 주파수. tilt(크기)는 중심통과마다 0 → 2배로 보임.
    signed = {"pitch": pitch, "roll": roll, "gx": gx, "gy": gy, "gz": gz}
    best = None
    print("  부호있는 신호(= 실제 주파수):")
    for name, sig in signed.items():
        f, power = dominant_freq(sig)
        amp = float(np.std(np.array(sig, dtype=float)))
        if f:
            print(f"    {name:5s}: {f:.3f} Hz  (주기 {1/f:.2f}s)  std={amp:.3f}")
            if best is None or power > best[2]:
                best = (name, f, power, amp)
    ft, _ = dominant_freq(tilt)
    if ft:
        print(f"    tilt(크기,2x): {ft:.3f} Hz → 실제 {ft/2:.3f} Hz  std={np.std(tilt):.2f}")

    if best:
        name, f, _, amp = best
        weak = (name in ("pitch", "roll") and amp < 2.0) or (name.startswith("g") and amp < 0.15)
        print(f"\n>>> 추정 고유 진동수 = {f:.3f} Hz  (주기 {1/f:.2f}s)  [최강 신호: {name}]")
        print(f"    그네 드라이브를 이 근처({f:.2f}Hz)로 두면 공진(가장 자연스러움).")
        if weak:
            print("    [주의] 진동 진폭이 작아 신뢰도 낮음 — 봉을 더 세게 밀고 재측정 권장.")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
