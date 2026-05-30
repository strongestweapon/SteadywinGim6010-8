"""
protocol.py — 컴퓨터 앱 ↔ ESP32 와이어 프로토콜 (전송 무관: UDP/serial/ESP-NOW 공통)
===============================================================================
설계 철학 (CLAUDE.md / esp32t2can 참고):
  - 앱은 "파형 모델"을 스트리밍한다: {run, waveform, freq, amp, phase}.
  - ESP32 는 로컬 위상 오실레이터를 이 스트림에 PLL 처럼 soft-lock 한다.
  - 패킷이 끊겨도 ESP32 는 마지막 freq/amp 로 자유진행(사인 계속) → 입력 갭 0.
  - 다시 오면 위상 부드럽게 재동기 → 튐 없음.
  - 긴 무수신(타임아웃) → ESP32 가 fade-out 후 IDLE (안전).

패킷 레이아웃 (little-endian, 고정 20 bytes):
  off  type   field      설명
   0   u16    magic      0x0DCA (동기/sanity)
   2   u16    seq        시퀀스 번호 (손실/순서 감지)
   4   u8     run        1=run, 0=stop  (앱 start/stop = BOOT 역할)
   5   u8     waveform   0=SINE (1=TRI, 2=SAW ... 향후)
   6   u16    reserved   0
   8   f32    freq       Hz (0 ~ FREQ_MAX)
  12   f32    amp_deg    출력축 진폭 [deg] (ESP32 가 모터 turn 으로 변환·안전 클램프)
  16   f32    phase      앱 오실레이터 현재 위상 [rad] (ESP32 lock 기준)
"""
from __future__ import annotations
import struct
import math

# ---- 상수 ----
MAGIC = 0x0DCA
UDP_PORT = 4210            # ESP32 가 listen 하는 UDP 포트
PACKET_FMT = "<HHBBHfff"   # 20 bytes
PACKET_SIZE = struct.calcsize(PACKET_FMT)  # = 20

# 안전 한계 (앱·ESP32 공통 인식; 실제 강제는 ESP32 가 함)
FREQ_MAX = 4.0             # Hz — 이 모터 출력축 상한
AMP_DEG_MAX = 30.0         # 출력축 ±deg (저주파에서. 고주파는 ESP32 가 vel_limit 으로 더 깎음)

# waveform enum (ESP32 main.cpp 와 동일)
WAVE_SINE = 0
WAVE_SWING = 1   # 큰 진폭 진자(그네) — waveforms.py 참고
# (2~ : 향후 웨이브테이블 추가 슬롯)

# 모터/제어 상수 (ESP32 와 동일) — 진폭·주파수 coupling 계산용
VEL_LIM_MOTOR = 5.0        # rev/s (ESP32 vel_limit)
GEAR = 8.0                 # 출력→모터 기어비


def amp_deg_max(freq: float, peak_vel: float = 1.0) -> float:
    """현재 주파수에서 vel_limit 이 허용하는 출력축 최대 진폭 [deg].
    peak 모터속도 = 2π·f·amp_turn·peak_vel ≤ VEL_LIM_MOTOR → amp_turn_max = VEL_LIM/(2πf·peak_vel).
    peak_vel 은 파형별 중심통과 속도배수(사인=1.0, 그네>1) — ESP32 clamp_amp 와 동일 식.
    출력° = amp_turn/GEAR×360. 기구 상한 AMP_DEG_MAX 와 min."""
    if freq <= 0.01:
        return AMP_DEG_MAX
    amp_turn_max = VEL_LIM_MOTOR / (2.0 * math.pi * freq * peak_vel)
    amp_deg_v = amp_turn_max / GEAR * 360.0
    return min(AMP_DEG_MAX, amp_deg_v)


def pack(seq: int, run: int, freq: float, amp_deg: float, phase: float,
         waveform: int = WAVE_SINE) -> bytes:
    """명령 패킷 직렬화."""
    return struct.pack(
        PACKET_FMT,
        MAGIC,
        seq & 0xFFFF,
        1 if run else 0,
        waveform & 0xFF,
        0,                       # reserved
        float(freq),
        float(amp_deg),
        float(phase),
    )


def unpack(data: bytes):
    """수신 패킷 역직렬화. magic 불일치/길이 오류면 None."""
    if len(data) != PACKET_SIZE:
        return None
    magic, seq, run, waveform, _res, freq, amp_deg, phase = struct.unpack(PACKET_FMT, data)
    if magic != MAGIC:
        return None
    return {
        "seq": seq, "run": run, "waveform": waveform,
        "freq": freq, "amp_deg": amp_deg, "phase": phase,
    }
