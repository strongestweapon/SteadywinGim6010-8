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
   6   u16    flags      bit0=모터1 극성반전, bit1=모터2 극성반전 (나머지 reserved)
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
PACKET_FMT = "<HHBBHfff"   # 20 bytes (앱 → ESP32 명령)
PACKET_SIZE = struct.calcsize(PACKET_FMT)  # = 20

# ---- 텔레메트리 (ESP32 → 앱) : IMU 센싱 표시용 ----
TELEM_MAGIC = 0x0DCB
# magic(H) seq(H) status(B) pad(B) + tilt pitch roll gx gy gz m1pos m2pos (f32×8)
#   + m1_iq m2_iq vbus ibus (f32×4) — 전류/전원 모니터
TELEM_FMT = "<HHBBffffffffffff"   # 54 bytes
TELEM_SIZE = struct.calcsize(TELEM_FMT)
# status 비트
TST_IMU_OK   = 0x01
TST_REST_SET = 0x02
TST_STILL    = 0x04
TST_APEX     = 0x08

# 안전 한계 (앱·ESP32 공통 인식; 실제 강제는 ESP32 가 함)
FREQ_MAX = 5.0             # Hz — 이 모터 출력축 상한
AMP_DEG_MAX = 60.0         # 출력축 ±deg 절대상한 (= 1Hz 앵커값. 저주파에서 bind)

# 진폭-주파수 곡선 앵커: A(f)=a/f+b 가 두 점 통과 (peak 모터속도 단조감소 → 안전).
#   (1Hz,60°)–(5Hz,10°). 그네(peak_vel>1)는 같은 peak속도 위해 진폭을 peak_vel 로 나눠 더 깎음.
AMP_F_LO, AMP_A_LO = 1.0, 60.0
AMP_F_HI, AMP_A_HI = 5.0, 10.0
_AMP_A = (AMP_A_LO - AMP_A_HI) / (1.0 / AMP_F_LO - 1.0 / AMP_F_HI)   # = 62.5
_AMP_B = AMP_A_LO - _AMP_A / AMP_F_LO                                # = -2.5

# waveform enum (ESP32 main.cpp 와 동일)
WAVE_SINE = 0
WAVE_SWING = 1   # 큰 진폭 진자(그네) — waveforms.py 참고
WAVE_PUMP = 2    # 토크 펌핑(anti-damping 증폭) — 토크모드
WAVE_LIFTDROP = 3  # 들어올림+프리폴 래칫 — 토크모드
# 토크 모드 패킷 재해석: amp_deg=진폭(±출력°), phase=세기(0..1), freq 무시
PUMP_AMP_MAX = 60.0   # 진폭 슬라이더 100% [출력°]

# 토크 기반 모드(펌프/리프트) — 앱에서 같은 컨트롤(강도=진폭, 펌프세기=세기) 사용
TORQUE_WAVES = (WAVE_PUMP, WAVE_LIFTDROP)

# 극성/요청 플래그 (flags 필드 비트)
POL_M1    = 0x01   # bit0: 모터1 극성 반전 (ESP32 가 START 시 latch)
POL_M2    = 0x02   # bit1: 모터2 극성 반전
REQ_TARE  = 0x04   # bit2: IMU 0점 재설정 요청 (rising-edge)
BTN_LEFT  = 0x08   # bit3: 들어올림+낙하 좌(뒤로 -90°) (rising-edge, one-shot)
BTN_RIGHT = 0x10   # bit4: 우(앞으로 +90°)

# 모터/제어 상수 (ESP32 와 동일)
VEL_LIM_MOTOR = 9.0        # rev/s (ESP32 안전 vel 클램프) — 1Hz·60°의 peak(8.4)을 안 깎게
GEAR = 8.0                 # 출력→모터 기어비


def amp_deg_max(freq: float, peak_vel: float = 1.0) -> float:
    """현재 주파수·파형에서 허용 출력축 최대 진폭 [deg].
    곡선 A(f)=a/f+b 가 (1Hz,60°)~(5Hz,10°) 통과 + peak 모터속도 단조감소(안전).
    그네(peak_vel>1)는 같은 peak속도 유지 위해 진폭을 peak_vel 로 나눠 더 깎음.
    ESP32 는 출력 시점에 vel/accel 로 한 번 더 안전 클램프(이 곡선보다 느슨한 천장)."""
    if freq <= 0.01:
        return AMP_DEG_MAX
    a_sine = _AMP_A / freq + _AMP_B
    return max(0.0, min(AMP_DEG_MAX, a_sine / peak_vel))


def pack(seq: int, run: int, freq: float, amp_deg: float, phase: float,
         waveform: int = WAVE_SINE, flags: int = 0) -> bytes:
    """명령 패킷 직렬화. flags = POL_M1|POL_M2 비트 OR."""
    return struct.pack(
        PACKET_FMT,
        MAGIC,
        seq & 0xFFFF,
        1 if run else 0,
        waveform & 0xFF,
        flags & 0xFFFF,          # flags (극성 등)
        float(freq),
        float(amp_deg),
        float(phase),
    )


def unpack(data: bytes):
    """수신 패킷 역직렬화. magic 불일치/길이 오류면 None."""
    if len(data) != PACKET_SIZE:
        return None
    magic, seq, run, waveform, flags, freq, amp_deg, phase = struct.unpack(PACKET_FMT, data)
    if magic != MAGIC:
        return None
    return {
        "seq": seq, "run": run, "waveform": waveform, "flags": flags,
        "freq": freq, "amp_deg": amp_deg, "phase": phase,
    }


def unpack_telem(data: bytes):
    """ESP32 → 앱 텔레메트리 역직렬화 (IMU). magic/길이 불일치면 None."""
    if len(data) != TELEM_SIZE:
        return None
    (magic, seq, status, _pad,
     tilt, pitch, roll, gx, gy, gz, m1, m2,
     iq1, iq2, vbus, ibus) = struct.unpack(TELEM_FMT, data)
    if magic != TELEM_MAGIC:
        return None
    return {
        "seq": seq, "status": status,
        "tilt": tilt, "pitch": pitch, "roll": roll,
        "gx": gx, "gy": gy, "gz": gz,
        "m1_pos": m1, "m2_pos": m2,
        "m1_iq": iq1, "m2_iq": iq2, "vbus": vbus, "ibus": ibus,
        "imu_ok": bool(status & TST_IMU_OK),
        "rest_set": bool(status & TST_REST_SET),
        "still": bool(status & TST_STILL),
        "apex": bool(status & TST_APEX),
    }
