"""
main.py — 듀얼 ESP32 모터 컨트롤러 데스크탑 앱 (PySide6)
===============================================================================
ESP32 **2대**(각각 모터쌍 1조)를 한 앱에서 제어. 좌우 2패널(자유 연주) + 아래 공연(씬).

레이아웃:
  - 상단(공유): drop 시뮬 / PS5 상태·대상
  - 좌/우 패널(자유 연주): IP·포트 / 속도(Hz) / 강도(진폭%) / 극성 / 동작·정지 / IMU 0점 / 상태
  - 아래 공연(씬): 씬 칸들(A/B on·freq·amp·극성·위상차) — 누르면 그 씬으로 부드럽게 전환
  - PS5/키보드는 "제어 대상"으로 고른 패널만 조작.

전송: 소켓 1개로 두 IP 에 각각 UDP 송신. 텔레메트리는 발신 IP 로 구분 수신.
조작:
  - 사인 : 속도=주파수(Hz), 강도=진폭%
  - PS5  : ○=동작, ✕=정지 (제어 대상 패널)

실행: python main.py   (ESP32 없이도 송신만 동작)
"""
from __future__ import annotations
import sys
import os
import json
import math
import socket
import time
from collections import deque

from PySide6 import QtWidgets, QtCore, QtGui
import numpy as np
import pyqtgraph as pg

# 다크(나이트) 모드 — 그래프 배경/전경
pg.setConfigOption("background", "#1e1e1e")
pg.setConfigOption("foreground", "#a0a0a0")

try:
    import pygame   # PS5(DualSense) 등 조이스틱 (선택 — 없으면 키보드만)
    _HAS_PYGAME = True
except ImportError:
    _HAS_PYGAME = False

import threading
try:
    from pythonosc.dispatcher import Dispatcher
    from pythonosc.osc_server import ThreadingOSCUDPServer
    _HAS_OSC = True
except ImportError:
    _HAS_OSC = False

import protocol as proto

SEND_HZ = 60                  # 송신 주기
PLOT_SECONDS = 4.0            # 파형 그래프 표시 구간
PLOT_N = int(SEND_HZ * PLOT_SECONDS)
PLOT_LIMIT_DEG = proto.AMP_DEG_MAX  # 그래프 Y 범위 기준 [출력°]
CONN_TIMEOUT = 1.0            # 텔레메트리 이 시간 이상 무수신 = 통신두절 [s] (ESP32 COMM_TIMEOUT 0.5s 보다 여유)
TWO_PI = 2.0 * math.pi
SCENE_COUNT = 5              # 공연 씬(버튼) 개수
CROSSFADE_S = 1.5            # 씬 전환 크로스페이드 기본 시간 [s] (freq/amp 부드럽게 이어받기)
SHOWS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "shows", "show.json")
OSC_SCENE = "/mirror/scene"   # 씬 트리거 OSC 주소 베이스 (/mirror/scene/<N> 또는 인자형)


class SystemPanel(QtWidgets.QGroupBox):
    """ESP32 1대 분의 컨트롤 + 상태. 자기 IP 로 패킷을 만들어 보낸다(소켓은 공유)."""

    def __init__(self, name: str, default_ip: str):
        super().__init__(f"System {name}")
        self.name = name
        self._default_ip = default_ip
        # 현재 반영 중인 씬 (제목에 표시). 수동 조작하면 dirty=수정 표시.
        self.scene_label = ""
        self.scene_dirty = False
        # 송신 상태
        self.running = False
        self.seq = 0
        self.phase = 0.0          # 사인 위상 [rad]
        self.tare_ticks = 0
        # 수신/표시 상태
        self.telem = None
        self.pk_i = 0.0
        self.pk_vmin = 999.0
        # 실측 패킷손실 (ESP32→앱 텔레메트리 seq 간격으로 측정)
        self._telem_last_seq = None
        self._gap_window = deque(maxlen=90)   # 최근 ~3s (telem ≈30Hz)
        self._telem_last_t = 0.0              # 마지막 수신 시각 [monotonic] (0=한번도 못받음)
        self.drop_meas = 0.0                  # 측정 손실율 [%]
        self.active = False                   # 제어 대상 강조(초록) 여부 — Controller 가 설정
        self._style_key = None                # 테두리 스타일 변경 감지용
        self._status_red = None               # 상태라벨 빨강 여부 변경 감지용
        # 씬 전환(크로스페이드) 상태
        self._tr = None                       # 진행 중 ramp: dict(sp0,sp1,an0,an1,on,i,n) | None
        self._deferred = False                # 스태거드 스타트 대기(위상차) — trigger_start 까지 보류
        # 씬 엔벨로프(시간 자동화) 상태: 크로스페이드로 start 도달 후 start→end 로 진행
        self._env = None                      # 진행 중 엔벨로프 dict | None
        self._pending_env = None              # _tr 끝나면 시작할 엔벨로프 | None
        self.value = 0.0                  # 현재 명령값 [출력°] (그래프용)
        self.buf = np.zeros(PLOT_N)       # 파형 롤링 버퍼
        self._build()

    # ---------------------------------------------------------------- UI
    def _build(self):
        g = QtWidgets.QGridLayout(self)
        r = 0
        g.addWidget(QtWidgets.QLabel("IP"), r, 0)
        self.ip_edit = QtWidgets.QLineEdit(self._default_ip)
        g.addWidget(self.ip_edit, r, 1)
        g.addWidget(QtWidgets.QLabel("Port"), r, 2)
        self.port_edit = QtWidgets.QLineEdit(str(proto.UDP_PORT))
        self.port_edit.setMaximumWidth(60)
        g.addWidget(self.port_edit, r, 3)

        r += 1
        g.addWidget(QtWidgets.QLabel("Speed"), r, 0)
        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(1, int(proto.FREQ_MAX * 10))   # 0.1 단위
        self.speed_slider.setValue(10)
        g.addWidget(self.speed_slider, r, 1, 1, 2)
        self.speed_lbl = QtWidgets.QLabel("")
        g.addWidget(self.speed_lbl, r, 3)
        self.speed_slider.valueChanged.connect(self._labels)
        self.speed_slider.sliderMoved.connect(self._user_touch)   # 사용자 드래그만 (씬 ramp 의 setValue 는 제외)

        r += 1
        g.addWidget(QtWidgets.QLabel("Amplitude"), r, 0)
        self.angle_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.angle_slider.setRange(0, int(proto.AMP_DEG_MAX))   # 출력 진폭 [°] (0~60)
        self.angle_slider.setValue(30)
        g.addWidget(self.angle_slider, r, 1, 1, 2)
        self.angle_lbl = QtWidgets.QLabel("")
        g.addWidget(self.angle_lbl, r, 3)
        self.angle_slider.valueChanged.connect(self._labels)
        self.angle_slider.sliderMoved.connect(self._user_touch)

        r += 1
        g.addWidget(QtWidgets.QLabel("Polarity"), r, 0)
        self.twist = QtWidgets.QCheckBox("Two motors opposite (twist)")
        self.twist.setToolTip("Unchecked = two motors same direction (parallel, default).  Checked = opposite (twist).")
        self.twist.clicked.connect(self._user_touch)   # 사용자 클릭만 (씬 적용 setChecked 는 제외)
        g.addWidget(self.twist, r, 1, 1, 3)

        r += 1
        self.btn = QtWidgets.QPushButton("▶ Run")
        self.btn.setCheckable(True)
        self.btn.setStyleSheet("font-weight:bold; padding:10px;")
        self.btn.toggled.connect(self._toggle)
        self.btn.clicked.connect(self._user_touch)   # 사용자 클릭만 (씬 start/stop 의 setChecked 는 제외)
        g.addWidget(self.btn, r, 0, 1, 2)
        self.tare_btn = QtWidgets.QPushButton("IMU zero")
        self.tare_btn.clicked.connect(lambda: setattr(self, "tare_ticks", 6))
        g.addWidget(self.tare_btn, r, 2, 1, 2)

        r += 1
        self.status_lbl = QtWidgets.QLabel("Idle")
        self.status_lbl.setStyleSheet("font-family:monospace;")
        self.status_lbl.setWordWrap(True)
        g.addWidget(self.status_lbl, r, 0, 1, 4)

        r += 1
        self.plot = pg.PlotWidget()
        self.plot.setMinimumHeight(140)
        self.plot.setYRange(-PLOT_LIMIT_DEG * 1.1, PLOT_LIMIT_DEG * 1.1)
        self.plot.setLabel("left", "cmd", units="deg")
        self.plot.setLabel("bottom", "time", units="s")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.curve = self.plot.plot(pen=pg.mkPen("#33cc88", width=2))
        self._t_axis = np.linspace(-PLOT_SECONDS, 0.0, PLOT_N)
        g.addWidget(self.plot, r, 0, 1, 4)

        self._labels()

    def set_ip(self, ip: str):
        self.ip_edit.setText(ip)

    def ip(self) -> str:
        return self.ip_edit.text().strip()

    def _toggle(self, on: bool):
        self.running = on
        self.btn.setText("■ Stop" if on else "▶ Run")
        if on:
            self.pk_i = 0.0
            self.pk_vmin = 999.0
            self.phase = 0.0              # 앱 위상모델 0 으로 리셋 (펌웨어 g_phase=0 at start 와 정합 → 위상차 계산 기준)
            self._telem_last_seq = None   # 손실 측정 리셋
            self._gap_window.clear()
            self.drop_meas = 0.0
        else:
            self.phase = 0.0

    def start(self):
        if not self.btn.isChecked():
            self.btn.setChecked(True)

    def stop(self):
        if self.btn.isChecked():
            self.btn.setChecked(False)

    # ---- 현재 씬 표시 (제목) ----
    def set_scene(self, name: str):
        """이 패널이 반영 중인 씬 이름 설정 → 제목에 '적용중' 표시, 수정플래그 해제."""
        self.scene_label = name
        self.scene_dirty = False
        self._update_title()

    def _user_touch(self, *args):
        """사용자가 슬라이더/버튼/극성을 직접 만짐 → 씬에서 벗어남(수정)."""
        if self.scene_label and not self.scene_dirty:
            self.scene_dirty = True
            self._update_title()

    def _update_title(self):
        if self.scene_label:
            self.setTitle(f"System {self.name}  ({self.scene_label}{' *edited' if self.scene_dirty else ' active'})")
        else:
            self.setTitle(f"System {self.name}")

    # ---------------------------------------------------------------- 씬 전환
    def begin_transition(self, speed_units: int, angle_pct: int, on: bool,
                         n_steps: int, defer_start: bool = False, env: dict | None = None):
        """씬 적용: 슬라이더(속도/강도)를 start 값으로 n_steps 동안 램프(=부드러운 이어받기).
        on=False 면 강도 0 으로 페이드 후 정지. defer_start=True 면 START 를 보류
        (위상차 스태거드 스타트용 — Controller 가 적절한 시점에 trigger_start).
        env 가 있으면 크로스페이드로 start 도달 후 그 엔벨로프(start→end, 시간/곡선)로 진행."""
        if not on:
            angle_pct = 0                       # 끄는 씬: 진폭 0 으로 페이드
            env = None                          # 끄는 씬엔 엔벨로프 없음
        fade_in = on and (not self.running)     # 정지→동작: 0 에서 페이드인
        if fade_in:
            # amp 0 에서 출발 → freq 는 즉시 목표로(무동작이라 점프 무방), 진폭만 페이드.
            # (freq 까지 램프하면 A·B 페이드 구간 freq 가 달라 위상차가 어긋남)
            self.speed_slider.setValue(int(speed_units))
            self.angle_slider.setValue(0)
            sp0, an0 = int(speed_units), 0
        else:
            sp0, an0 = self.speed_slider.value(), self.angle_slider.value()
        self._tr = {
            "sp0": sp0, "sp1": int(speed_units),
            "an0": an0, "an1": int(angle_pct),
            "on": on, "i": 0, "n": max(1, n_steps),
        }
        # start→end 가 실제로 변하는 엔벨로프만 보관 (정적 씬이면 None → 기존 동작 그대로)
        self._pending_env = env if (env and (env["f0"] != env["f1"] or env["a0"] != env["a1"])) else None
        self._env = None
        self._deferred = bool(defer_start and fade_in)
        if fade_in and not self._deferred:
            self.start()

    def trigger_start(self):
        """스태거드 스타트 보류 해제 → 지금부터 START + 페이드인."""
        if self._deferred:
            self._deferred = False
            self.start()

    def step_transition(self):
        """매 틱 호출. 1) 크로스페이드로 start 도달 → 2) 엔벨로프로 start→end 진행."""
        if self._deferred:
            return
        # 1) 크로스페이드 (현재값 → start)
        if self._tr is not None:
            tr = self._tr
            tr["i"] += 1
            f = tr["i"] / tr["n"]
            if f >= 1.0:
                self.speed_slider.setValue(tr["sp1"])
                self.angle_slider.setValue(tr["an1"])
                if not tr["on"] and self.running:
                    self.stop()
                self._tr = None
                # 크로스페이드 끝 → 대기 중 엔벨로프 시작
                if self._pending_env is not None:
                    self._env = dict(self._pending_env, i=0)
                    self._pending_env = None
                return
            self.speed_slider.setValue(int(round(tr["sp0"] + (tr["sp1"] - tr["sp0"]) * f)))
            self.angle_slider.setValue(int(round(tr["an0"] + (tr["an1"] - tr["an0"]) * f)))
            return
        # 2) 엔벨로프 (start → end, A/B·freq/amp 각자 곡선) — 끝값 유지
        if self._env is not None:
            ev = self._env
            ev["i"] += 1
            p = ev["i"] / ev["n"]
            if p >= 1.0:
                self.speed_slider.setValue(ev["f1"])
                self.angle_slider.setValue(ev["a1"])
                self._env = None
                return
            self.speed_slider.setValue(int(round(_ease(ev["f0"], ev["f1"], ev["fc"], p))))
            self.angle_slider.setValue(int(round(_ease(ev["a0"], ev["a1"], ev["ac"], p))))

    def _labels(self):
        freq = self.speed_slider.value() / 10.0
        drive = self.angle_slider.value()           # 진폭 [출력°]
        mx = proto.amp_deg_max(freq, 1.0)
        self.speed_lbl.setText(f"{freq:.1f} Hz")
        if drive > mx:
            self.angle_lbl.setText(f"±{mx:.0f}° (max)")   # 이 주파수 한계로 클램프
        else:
            self.angle_lbl.setText(f"±{drive}°")

    # ---------------------------------------------------------------- 송신
    def tick(self, dt: float, drop_pct: int, sock: socket.socket):
        freq = self.speed_slider.value() / 10.0
        drive = self.angle_slider.value()                       # 진폭 [출력°]
        amp_deg = min(float(drive), proto.amp_deg_max(freq, 1.0))  # 주파수 한계로 클램프(안전)
        if self.running:
            self.phase += 2.0 * math.pi * freq * dt
            if self.phase > 2.0 * math.pi:
                self.phase -= 2.0 * math.pi
            self.value = amp_deg * math.sin(self.phase)
        else:
            self.value = 0.0

        # 파형 그래프 갱신 (명령값)
        self.buf = np.roll(self.buf, -1)
        self.buf[-1] = self.value
        self.curve.setData(self._t_axis, self.buf)

        self.seq = (self.seq + 1) & 0xFFFF
        if drop_pct > 0 and _rand100() < drop_pct:
            return   # 패킷손실 시뮬
        flags = 0
        # 극성: 같이(평행)=모터2 반전(기본), 반대(트위스트)=반전 없음
        if not self.twist.isChecked():
            flags |= proto.POL_M2
        if self.tare_ticks > 0:
            flags |= proto.REQ_TARE
            self.tare_ticks -= 1
        pkt = proto.pack(self.seq, 1 if self.running else 0,
                         freq, amp_deg, self.phase, waveform=proto.WAVE_SINE, flags=flags)
        try:
            sock.sendto(pkt, (self.ip(), int(self.port_edit.text())))
        except (OSError, ValueError):
            pass

    # ---------------------------------------------------------------- 수신/표시
    def apply_telem(self, t: dict):
        self.telem = t
        self.pk_i = max(self.pk_i, abs(t["m1_iq"]), abs(t["m2_iq"]))
        if 1.0 < t["vbus"] < self.pk_vmin:
            self.pk_vmin = t["vbus"]
        # 실측 손실율: 연속 텔레메트리 seq 간격(gap). gap=1=무손실, gap=N → N-1개 유실.
        self._telem_last_t = time.monotonic()
        seq = t["seq"]
        if self._telem_last_seq is not None:
            gap = (seq - self._telem_last_seq) & 0xFFFF
            if 1 <= gap <= 500:
                self._gap_window.append(gap)
            else:               # 재시작/큰 점프 → 윈도우 리셋
                self._gap_window.clear()
        self._telem_last_seq = seq
        if self._gap_window:
            total = sum(self._gap_window)        # 기대 패킷수
            recv = len(self._gap_window)          # 실수신 패킷수
            self.drop_meas = (total - recv) / total * 100.0

    def conn_status(self) -> str:
        """'never'(한번도 못받음) | 'stale'(끊김) | 'ok'."""
        if self._telem_last_t == 0.0:
            return "never"
        if (time.monotonic() - self._telem_last_t) > CONN_TIMEOUT:
            return "stale"
        return "ok"

    def update_style(self):
        """테두리 색 = 통신두절(빨강) > 제어대상(초록) > 비활성(회색). 변경 시에만 적용."""
        disc = self.conn_status() != "ok"
        key = "disc" if disc else ("act" if self.active else "idle")
        if key == self._style_key:
            return
        self._style_key = key
        if key == "disc":
            w, c, title = 3, "#ff4444", "#ff5555"      # 빨강 = 통신두절
        elif key == "act":
            w, c, title = 2, "#33cc88", "#33cc88"      # 초록 = 제어 대상
        else:
            w, c, title = 1, "#555", "#888"            # 회색 = 비활성
        self.setStyleSheet(
            f"QGroupBox{{border:{w}px solid {c}; border-radius:6px; margin-top:8px;}}"
            f"QGroupBox::title{{subcontrol-origin:margin; left:10px; color:{title}; font-weight:bold;}}")

    def refresh_status(self):
        run = "RUN" if self.running else "IDLE"
        state = self.conn_status()
        if state != "ok":
            # 통신두절 = drop 의 완전손실 상태. 처음 시작 때 응답없어도 여기로.
            if self._status_red is not True:
                self.status_lbl.setStyleSheet("font-family:monospace; color:#ff5555; font-weight:bold;")
                self._status_red = True
            msg = "no response — check ESP32 / IP / power" if state == "never" else "signal lost"
            self.status_lbl.setText(f"{run}   ⛔ DISCONNECTED ({msg})")
            return
        if self._status_red is not False:
            self.status_lbl.setStyleSheet("font-family:monospace;")
            self._status_red = False
        t = self.telem
        warn = ""
        if t["ibus"] > 3.0:
            warn = " ⚠OVERCURRENT"
        elif 1.0 < t["vbus"] < 19.0:
            warn = " ⚠LOW VOLT"
        imu = f"tilt {t['tilt']:.0f}°" if t["imu_ok"] else "no IMU"
        vmin = f"{self.pk_vmin:.1f}" if self.pk_vmin < 900 else "—"
        self.status_lbl.setText(
            f"{run}   I {t['m1_iq']:+.1f}/{t['m2_iq']:+.1f}A  V {t['vbus']:.1f}  "
            f"Ibus {t['ibus']:.2f}A{warn}\n{imu}   drop {self.drop_meas:.0f}%   "
            f"pk: Imax {self.pk_i:.1f}A  Vmin {vmin}")


def _rand100() -> int:
    # numpy 의존 제거용 간단 난수 (drop 시뮬 only)
    import random
    return random.randint(0, 99)


# 씬 한 개의 기본값.
# 씬 = "시간 가진 엔벨로프": 누르면 duration[s] 동안 freq/amp 가 start(0)→end(1) 로 변함.
#   - 곡선(curve)은 A/B · freq/amp 각각 따로: "lin"(선형) | "exp"(지수=처음 느리고 끝에서 빨라짐)
#   - 끝에 닿으면 end 값으로 유지(hold). Δφ·twist 는 씬 동안 고정, 변하는 건 freq/amp 만.
# 진폭=출력° / twist=False=두 모터 같이·평행 기본.
_SCENE_DEFAULTS = {
    "name": "씬", "duration": 0.0,
    "a_on": True, "a_freq0": 0.4, "a_freq1": 0.4, "a_fcurve": "lin",
    "a_amp0": 30, "a_amp1": 30, "a_acurve": "lin", "a_twist": False,
    "b_on": True, "b_freq0": 0.4, "b_freq1": 0.4, "b_fcurve": "lin",
    "b_amp0": 30, "b_amp1": 30, "b_acurve": "lin", "b_twist": False,
    "phase": 0,
}


def _norm_scene(sc) -> dict:
    """누락 키를 기본값으로 채움 + 진폭을 주파수 한계로 클램프.
    옛 쇼파일(정적 a_freq/a_amp 단일값)도 start=end 로 자동 변환."""
    sc = dict(sc or {})
    # 구버전 단일값 → start/end 양쪽으로 펼침
    for side in ("a", "b"):
        if f"{side}_freq" in sc:
            sc.setdefault(f"{side}_freq0", sc[f"{side}_freq"])
            sc.setdefault(f"{side}_freq1", sc[f"{side}_freq"])
        if f"{side}_amp" in sc:
            sc.setdefault(f"{side}_amp0", sc[f"{side}_amp"])
            sc.setdefault(f"{side}_amp1", sc[f"{side}_amp"])
    out = dict(_SCENE_DEFAULTS)
    out.update(sc)
    # 진폭 start/end 각각 자기 주파수 한계로 클램프
    out["a_amp0"] = min(int(out["a_amp0"]), int(amp_deg_max_safe(out["a_freq0"])))
    out["a_amp1"] = min(int(out["a_amp1"]), int(amp_deg_max_safe(out["a_freq1"])))
    out["b_amp0"] = min(int(out["b_amp0"]), int(amp_deg_max_safe(out["b_freq0"])))
    out["b_amp1"] = min(int(out["b_amp1"]), int(amp_deg_max_safe(out["b_freq1"])))
    for k in ("a_fcurve", "a_acurve", "b_fcurve", "b_acurve"):
        if out[k] not in ("lin", "exp"):
            out[k] = "lin"
    return out


def amp_deg_max_safe(freq) -> float:
    """그 주파수에서 허용 최대 출력각 [°]."""
    return proto.amp_deg_max(float(freq), 1.0)


def _ease(v0: float, v1: float, curve: str, p: float) -> float:
    """v0→v1 보간. p∈[0,1]. 'lin'=선형, 'exp'=지수(처음 느리고 끝에서 가속).
    exp 는 어떤 끝값(0 포함)에도 안전한 단조 ease 곡선."""
    if curve == "exp":
        k = 3.0
        p = (math.exp(k * p) - 1.0) / (math.exp(k) - 1.0)
    return v0 + (v1 - v0) * p


def _default_scenes():
    """기본 5씬: 동상 / 역상(180°) / A만 / B만 / 정지. (start=end=정적, duration 0)."""
    def s(name, a_on, a_f, a_a, b_on, b_f, b_a, ph):
        return _norm_scene({"name": name, "a_on": a_on, "a_freq": a_f, "a_amp": a_a,
                            "b_on": b_on, "b_freq": b_f, "b_amp": b_a, "phase": ph})
    return [
        s("1 In-phase", True, 0.4, 40, True, 0.4, 40, 0),
        s("2 Anti-phase", True, 0.4, 40, True, 0.4, 40, 180),
        s("3 A only", True, 1.0, 30, False, 1.0, 30, 0),
        s("4 B only", False, 1.0, 30, True, 1.0, 30, 0),
        s("5 Stop", False, 0.4, 0, False, 0.4, 0, 0),
    ]


def _phase_crossed(prev: float, cur: float, target: float) -> bool:
    """전진 방향으로 prev→cur 사이에 target 위상을 통과했나 (모두 mod 2π, 하한 포함)."""
    p = prev % TWO_PI
    c = cur % TWO_PI
    t = target % TWO_PI
    if c >= p:
        return p <= t <= c
    return t >= p or t <= c   # wrap 구간


class OscBridge(QtCore.QObject):
    """OSC 수신(별도 스레드) → 메인스레드로 씬 트리거. 시그널 큐잉으로 스레드 안전.
    주소 규칙: OSC_SCENE/<N> (1-based) 또는 OSC_SCENE <int> → 그 씬 적용."""
    trigger = QtCore.Signal(int)   # 0-based 씬 인덱스
    info = QtCore.Signal(str)      # 상태 텍스트

    def __init__(self):
        super().__init__()
        self._server = None
        self._thread = None

    def start(self, port: int) -> bool:
        self.stop()
        if not _HAS_OSC:
            self.info.emit("python-osc 미설치")
            return False
        disp = Dispatcher()
        disp.map(f"{OSC_SCENE}/*", self._on_addr)
        disp.map(OSC_SCENE, self._on_arg)
        try:
            self._server = ThreadingOSCUDPServer(("0.0.0.0", port), disp)
        except OSError:
            self.info.emit(f"port {port} busy")
            return False
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        self.info.emit(f"listening :{port}")
        return True

    def stop(self):
        if self._server is not None:
            try:
                self._server.shutdown()
                self._server.server_close()
            except Exception:
                pass
            self._server = None
            self._thread = None

    def _on_addr(self, address, *args):
        try:
            self.trigger.emit(int(address.rsplit("/", 1)[1]) - 1)
        except (ValueError, IndexError):
            pass

    def _on_arg(self, address, *args):
        if args:
            try:
                self.trigger.emit(int(float(args[0])) - 1)   # int/float/숫자문자열 다 허용
            except (ValueError, TypeError):
                pass


class Controller(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Steadywin Dual ESP32 Controller")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        # PS5
        self._js = None
        self._js_name = ""
        self._xprev = False
        self._oprev = False
        if _HAS_PYGAME:
            pygame.init()
            pygame.joystick.init()
            self._init_js()

        # 공연 씬 (저장된 쇼파일 있으면 그걸로 시작)
        self.scenes = self._read_shows_file() or _default_scenes()
        self._pending_start = None     # 스태거드 스타트 대기: dict(panel, ref, target, prev)

        # OSC 수신 (씬 트리거)
        self.osc = OscBridge()
        self.osc.trigger.connect(self._osc_trigger)
        self.osc.info.connect(self._osc_info)

        self._build()

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(int(1000 / SEND_HZ))
        self.timer.timeout.connect(self._tick)
        self.timer.start()

    # ---------------------------------------------------------------- UI
    def _build(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        # 상단 공유 바
        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("drop sim%"))
        self.drop_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.drop_slider.setRange(0, 90)
        self.drop_slider.setMaximumWidth(120)
        top.addWidget(self.drop_slider)
        top.addStretch(1)
        self.pad_lbl = QtWidgets.QLabel("PS5: —")
        top.addWidget(self.pad_lbl)
        root.addLayout(top)

        # 좌우 패널 — 각 패널 위 중앙에 "제어 대상" 라디오, 선택된 패널은 초록 강조
        panels = QtWidgets.QHBoxLayout()
        self.A = SystemPanel("A", "192.168.0.56")   # Mirror A (MAC 14-C1-9F-38-EE-24, COM7)
        self.B = SystemPanel("B", "192.168.0.57")   # Mirror B (MAC 14-C1-9F-38-EE-2C, COM5)
        self.tgt_a = QtWidgets.QRadioButton("● Control A")
        self.tgt_b = QtWidgets.QRadioButton("● Control B")
        self.tgt_a.setChecked(True)
        self._tgt_grp = QtWidgets.QButtonGroup(self)
        self._tgt_grp.addButton(self.tgt_a)
        self._tgt_grp.addButton(self.tgt_b)
        self.tgt_a.toggled.connect(self._refresh_highlight)
        # 무대 배치에 맞춰 왼쪽=B, 오른쪽=A 로 표시
        for radio, panel in ((self.tgt_b, self.B), (self.tgt_a, self.A)):
            radio.setStyleSheet("font-weight:bold;")
            col = QtWidgets.QVBoxLayout()
            rrow = QtWidgets.QHBoxLayout()
            rrow.addStretch(1)
            rrow.addWidget(radio)
            rrow.addStretch(1)
            col.addLayout(rrow)
            col.addWidget(panel)
            panels.addLayout(col)
        root.addLayout(panels)

        hint = QtWidgets.QLabel(
            "PS5: ○=Run  ✕=Stop (controls the targeted panel).")
        hint.setStyleSheet("color:#888;")
        root.addWidget(hint)

        root.addWidget(self._build_show())   # 공연(씬) 섹션 — 화면 아래

        self._update_active_enabled()

    # ---------------------------------------------------------------- 공연(씬) UI
    def _build_show(self):
        """씬을 각각 [적용버튼 + 칸 안에 A/B 켜기·freq·amp·위상차]로 나란히. + 씬 추가/삭제·가로스크롤."""
        box = QtWidgets.QGroupBox("Show (Scenes) — click a button or press number keys 1-9,0 (= scenes 1-10): ramps freq/amp start→end over Duration, then holds")
        outer = QtWidgets.QVBoxLayout(box)
        self._sw_block = False
        self.scene_btns = []
        self.sw = []   # 씬별 위젯 모음 [{name,a_on,a_f0,a_f1,a_fc,a_a0,a_a1,a_ac,a_twist,...,phase,dur}]

        # 씬 칸들 — 가로 스크롤(씬 늘어나면 옆으로)
        self._scene_host = QtWidgets.QWidget()
        self._scene_cols = QtWidgets.QHBoxLayout(self._scene_host)
        self._scene_cols.setContentsMargins(2, 2, 2, 2)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self._scene_host)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        scroll.setMinimumHeight(340)
        outer.addWidget(scroll)

        # 공통 하단: + 씬 추가 / 전환시간 / 저장·불러오기
        bot = QtWidgets.QHBoxLayout()
        addb = QtWidgets.QPushButton("+ Add scene"); addb.clicked.connect(self._add_scene)
        bot.addWidget(addb)
        bot.addWidget(QtWidgets.QLabel("Lead-in"))
        self.crossfade_spin = QtWidgets.QDoubleSpinBox()
        self.crossfade_spin.setRange(0.0, 5.0); self.crossfade_spin.setSingleStep(0.1)
        self.crossfade_spin.setValue(CROSSFADE_S); self.crossfade_spin.setSuffix(" s")
        self.crossfade_spin.setToolTip("현재 상태 → 씬 start 값까지 부드럽게 도달하는 시간. 그 뒤 Duration 동안 start→end 진행.")
        bot.addWidget(self.crossfade_spin)
        bot.addSpacing(20)
        self.osc_cb = QtWidgets.QCheckBox("OSC in")
        self.osc_cb.setToolTip(
            "Trigger a scene by sending OSC (no argument needed):\n"
            f"  {OSC_SCENE}/1   {OSC_SCENE}/2   {OSC_SCENE}/3 ...\n"
            f"Or address {OSC_SCENE} with one number arg (int or float):\n"
            f"  {OSC_SCENE}  2     {OSC_SCENE}  2.0")
        self.osc_cb.toggled.connect(self._toggle_osc)
        bot.addWidget(self.osc_cb)
        bot.addWidget(QtWidgets.QLabel("port"))
        self.osc_port = QtWidgets.QLineEdit("9000"); self.osc_port.setMaximumWidth(70)
        bot.addWidget(self.osc_port)
        self.osc_lbl = QtWidgets.QLabel(f"{OSC_SCENE}/N" if _HAS_OSC else "(no python-osc)")
        self.osc_lbl.setStyleSheet("color:#888;")
        bot.addWidget(self.osc_lbl)
        bot.addStretch(1)
        sv = QtWidgets.QPushButton("Save"); sv.clicked.connect(self._save_shows)
        ld = QtWidgets.QPushButton("Load"); ld.clicked.connect(self._load_shows)
        bot.addWidget(sv); bot.addWidget(ld)
        outer.addLayout(bot)

        note = QtWidgets.QLabel(
            "Each scene is a timed envelope: f/A ramp from start→end over Duration (Lin/Exp per field), then hold. "
            "Dur 0 = jump to end. Lead-in smooths the approach to the start values. "
            "Phase = staggered start (B starts when A reaches the offset); "
            "to change phase while both already run, restart B in that scene (Stop, then apply).")
        note.setStyleSheet("color:#888;"); note.setWordWrap(True)
        outer.addWidget(note)

        self._rebuild_scene_cols()
        return box

    @staticmethod
    def _curve_combo() -> QtWidgets.QComboBox:
        """보간 곡선 선택 (Lin/Exp). currentData()='lin'|'exp'."""
        c = QtWidgets.QComboBox()
        c.addItem("Lin", "lin")
        c.addItem("Exp", "exp")
        c.setMaximumWidth(58)
        c.setToolTip("Lin = 일정 속도로 변화\nExp = 처음 느리고 끝에서 가속")
        return c

    def _make_scene_col(self, i: int) -> QtWidgets.QGroupBox:
        """씬 한 칸 위젯 생성. self.sw / self.scene_btns 에 순서대로 append.
        씬 = 시간 엔벨로프: [start]→[end] freq/amp + 각자 곡선 + duration."""
        col = QtWidgets.QGroupBox()
        col.setMinimumWidth(360)
        cv = QtWidgets.QVBoxLayout(col)

        btn = QtWidgets.QPushButton()
        btn.setMinimumHeight(76)
        btn.setStyleSheet("font-weight:bold; padding:6px; text-align:center;")
        _k = self._scene_key_label(i)
        btn.setToolTip((f"Key: {_k}\n" if _k else "") + f"OSC: {OSC_SCENE}/{i + 1}")
        btn.clicked.connect(lambda _=False, idx=i: self._apply_scene(idx))
        cv.addWidget(btn)
        self.scene_btns.append(btn)

        name = QtWidgets.QLineEdit()
        name.setPlaceholderText("Name")
        cv.addWidget(name)

        nob = QtWidgets.QAbstractSpinBox.NoButtons   # 위아래 화살표 없이 타이핑만 (공간 절약)

        def freq_spin():
            s = QtWidgets.QDoubleSpinBox(); s.setRange(0.1, proto.FREQ_MAX)
            s.setSingleStep(0.1); s.setButtonSymbols(nob); s.setMaximumWidth(64)
            s.setKeyboardTracking(False)
            return s

        def amp_spin():
            s = QtWidgets.QSpinBox(); s.setRange(0, 360); s.setSuffix("°")
            s.setButtonSymbols(nob); s.setKeyboardTracking(False); s.setMaximumWidth(64)
            return s

        def env_row(tag, tip):
            """[☑tag] freq: [f0]→[f1] [curve] amp: [a0]→[a1] [curve] [Twist] 한 줄 묶음.
            2줄로 나눔(freq 줄 / amp 줄) — 칸 너비 절약."""
            on = QtWidgets.QCheckBox(tag)
            tw = QtWidgets.QCheckBox("Twist"); tw.setToolTip(tip)
            f0, f1, fc = freq_spin(), freq_spin(), self._curve_combo()
            a0, a1, ac = amp_spin(), amp_spin(), self._curve_combo()
            # freq 줄
            fr = QtWidgets.QHBoxLayout()
            fr.addWidget(on); fr.addWidget(QtWidgets.QLabel("f")); fr.addWidget(f0)
            fr.addWidget(QtWidgets.QLabel("→")); fr.addWidget(f1)
            fr.addWidget(QtWidgets.QLabel("Hz")); fr.addWidget(fc); fr.addWidget(tw)
            cv.addLayout(fr)
            # amp 줄
            am = QtWidgets.QHBoxLayout()
            am.addSpacing(4); am.addWidget(QtWidgets.QLabel("A")); am.addWidget(a0)
            am.addWidget(QtWidgets.QLabel("→")); am.addWidget(a1)
            am.addWidget(ac); am.addStretch(1)
            cv.addLayout(am)
            return on, f0, f1, fc, a0, a1, ac, tw

        (a_on, a_f0, a_f1, a_fc, a_a0, a_a1, a_ac, a_tw) = env_row(
            "A", "A: two motors opposite (twist). Unchecked = parallel")
        (b_on, b_f0, b_f1, b_fc, b_a0, b_a1, b_ac, b_tw) = env_row(
            "B", "B: two motors opposite (twist). Unchecked = parallel")

        ph = QtWidgets.QSpinBox(); ph.setRange(0, 359); ph.setSuffix("°"); ph.setButtonSymbols(nob)
        dur = QtWidgets.QDoubleSpinBox(); dur.setRange(0.0, 600.0); dur.setSingleStep(0.5)
        dur.setSuffix(" s"); dur.setButtonSymbols(nob); dur.setKeyboardTracking(False)
        dur.setToolTip("Envelope time: start→end 에 걸리는 시간. 0=즉시.")
        pr = QtWidgets.QHBoxLayout()
        pr.addWidget(QtWidgets.QLabel("Δφ")); pr.addWidget(ph)
        pr.addSpacing(10)
        pr.addWidget(QtWidgets.QLabel("Dur")); pr.addWidget(dur); pr.addStretch(1)
        cv.addLayout(pr)

        cap = QtWidgets.QPushButton("Capture current")
        cap.clicked.connect(lambda _=False, idx=i: self._capture_scene(idx))
        delb = QtWidgets.QPushButton("✕ Delete")
        delb.clicked.connect(lambda _=False, idx=i: self._del_scene(idx))
        cr = QtWidgets.QHBoxLayout(); cr.addWidget(cap); cr.addWidget(delb)
        cv.addLayout(cr)

        self.sw.append({
            "name": name,
            "a_on": a_on, "a_f0": a_f0, "a_f1": a_f1, "a_fc": a_fc,
            "a_a0": a_a0, "a_a1": a_a1, "a_ac": a_ac, "a_twist": a_tw,
            "b_on": b_on, "b_f0": b_f0, "b_f1": b_f1, "b_fc": b_fc,
            "b_a0": b_a0, "b_a1": b_a1, "b_ac": b_ac, "b_twist": b_tw,
            "phase": ph, "dur": dur,
        })
        name.textChanged.connect(lambda _=None, idx=i: self._scene_widgets_changed(idx))
        for wdg in (a_f0, a_f1, a_a0, a_a1, b_f0, b_f1, b_a0, b_a1, ph, dur):
            wdg.valueChanged.connect(lambda _=None, idx=i: self._scene_widgets_changed(idx))
        for wdg in (a_on, b_on, a_tw, b_tw):
            wdg.toggled.connect(lambda _=None, idx=i: self._scene_widgets_changed(idx))
        for wdg in (a_fc, a_ac, b_fc, b_ac):
            wdg.currentIndexChanged.connect(lambda _=None, idx=i: self._scene_widgets_changed(idx))
        return col

    def _rebuild_scene_cols(self):
        """self.scenes 길이에 맞춰 씬 칸 전부 다시 만든다 (추가/삭제/불러오기 후)."""
        while self._scene_cols.count():
            item = self._scene_cols.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()
        self.scene_btns = []
        self.sw = []
        for i in range(len(self.scenes)):
            self._scene_cols.addWidget(self._make_scene_col(i))
        self._scene_cols.addStretch(1)
        self._refresh_scene_ui()

    def _add_scene(self):
        n = len(self.scenes)
        self.scenes.append(_norm_scene({"name": f"Scene {n + 1}"}))
        self._rebuild_scene_cols()

    def _del_scene(self, i: int):
        if len(self.scenes) <= 1:
            return                      # 최소 1개 유지
        del self.scenes[i]
        self._rebuild_scene_cols()

    # ---- 씬 데이터 ↔ 씬칸 위젯 ----
    @staticmethod
    def _scene_btn_text(sc) -> str:
        """버튼 라벨 = 이름 + A/B 요약. start→end 가 다르면 화살표로 표시."""
        def rng(v0, v1, unit):
            return f"{v0:g}{unit}" if v0 == v1 else f"{v0:g}→{v1:g}{unit}"
        def part(on, f0, f1, a0, a1, tw):
            if not on:
                return "off"
            return rng(f0, f1, "Hz") + "·" + rng(a0, a1, "°") + ("↔" if tw else "")
        a = part(sc["a_on"], sc["a_freq0"], sc["a_freq1"], sc["a_amp0"], sc["a_amp1"], sc["a_twist"])
        b = part(sc["b_on"], sc["b_freq0"], sc["b_freq1"], sc["b_amp0"], sc["b_amp1"], sc["b_twist"])
        d = f"  {sc['duration']:g}s" if sc["duration"] > 0 else ""
        return f"{sc['name']}\nA {a}   B {b}   Δφ{sc['phase']}°{d}"

    @staticmethod
    def _set_curve(combo, val):
        idx = combo.findData(val)
        combo.setCurrentIndex(idx if idx >= 0 else 0)

    def _refresh_scene_ui(self):
        """self.scenes → 각 씬칸 위젯 + 버튼 라벨."""
        self._sw_block = True
        for i, sc in enumerate(self.scenes):
            sw = self.sw[i]
            sw["name"].setText(sc["name"])
            for side in ("a", "b"):
                sw[f"{side}_on"].setChecked(sc[f"{side}_on"])
                sw[f"{side}_f0"].setValue(sc[f"{side}_freq0"]); sw[f"{side}_f1"].setValue(sc[f"{side}_freq1"])
                sw[f"{side}_a0"].setValue(sc[f"{side}_amp0"]); sw[f"{side}_a1"].setValue(sc[f"{side}_amp1"])
                sw[f"{side}_twist"].setChecked(sc[f"{side}_twist"])
                self._set_curve(sw[f"{side}_fc"], sc[f"{side}_fcurve"])
                self._set_curve(sw[f"{side}_ac"], sc[f"{side}_acurve"])
            sw["phase"].setValue(sc["phase"]); sw["dur"].setValue(sc["duration"])
            key = self._scene_key_label(i)
            self.scene_btns[i].setText((f"[{key}]  " if key else "") + self._scene_btn_text(sc))
        self._sw_block = False

    def _clamp_scene_amp(self, i: int):
        """입력 끝난 진폭 start/end 를 각자 주파수의 최대각으로 줄임 (입력 중엔 안 막음)."""
        self._sw_block = True
        sw = self.sw[i]
        for side in ("a", "b"):
            for fk, ak in ((f"{side}_f0", f"{side}_a0"), (f"{side}_f1", f"{side}_a1")):
                mx = int(amp_deg_max_safe(sw[fk].value()))
                if sw[ak].value() > mx:
                    sw[ak].setValue(mx)
        self._sw_block = False

    def _scene_widgets_changed(self, i: int):
        """씬칸 위젯 변경 → self.scenes[i] 갱신 + 버튼 라벨."""
        if self._sw_block:
            return
        self._clamp_scene_amp(i)   # 주파수 한계로 줄임(초과 입력/주파수 상승 시)
        sw = self.sw[i]
        sc = {"name": sw["name"].text() or f"Scene {i + 1}",
              "duration": round(sw["dur"].value(), 1), "phase": sw["phase"].value()}
        for side in ("a", "b"):
            sc[f"{side}_on"] = sw[f"{side}_on"].isChecked()
            sc[f"{side}_freq0"] = round(sw[f"{side}_f0"].value(), 1)
            sc[f"{side}_freq1"] = round(sw[f"{side}_f1"].value(), 1)
            sc[f"{side}_amp0"] = sw[f"{side}_a0"].value()
            sc[f"{side}_amp1"] = sw[f"{side}_a1"].value()
            sc[f"{side}_fcurve"] = sw[f"{side}_fc"].currentData()
            sc[f"{side}_acurve"] = sw[f"{side}_ac"].currentData()
            sc[f"{side}_twist"] = sw[f"{side}_twist"].isChecked()
        self.scenes[i] = sc
        self.scene_btns[i].setText(self._scene_btn_text(sc))

    def _capture_scene(self, i: int):
        """현재 두 패널 상태를 씬 i 의 start·end 양쪽에 담기(정적 스냅샷). 곡선/duration/위상차는 유지."""
        old = self.scenes[i]
        def snap(panel):
            return (panel.running, round(panel.speed_slider.value() / 10.0, 1),
                    panel.angle_slider.value(), panel.twist.isChecked())
        a_on, a_f, a_a, a_tw = snap(self.A)
        b_on, b_f, b_a, b_tw = snap(self.B)
        self.scenes[i] = _norm_scene({
            "name": old["name"], "duration": old["duration"], "phase": old["phase"],
            "a_on": a_on, "a_freq0": a_f, "a_freq1": a_f, "a_amp0": a_a, "a_amp1": a_a,
            "a_fcurve": old["a_fcurve"], "a_acurve": old["a_acurve"], "a_twist": a_tw,
            "b_on": b_on, "b_freq0": b_f, "b_freq1": b_f, "b_amp0": b_a, "b_amp1": b_a,
            "b_fcurve": old["b_fcurve"], "b_acurve": old["b_acurve"], "b_twist": b_tw,
        })
        self._refresh_scene_ui()

    # ---- 쇼파일 저장/불러오기 ----
    def _read_shows_file(self):
        try:
            with open(SHOWS_FILE, encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data, list) and data:
                return [_norm_scene(s) for s in data]
        except (OSError, ValueError):
            pass
        return None

    def _save_shows(self):
        try:
            os.makedirs(os.path.dirname(SHOWS_FILE), exist_ok=True)
            with open(SHOWS_FILE, "w", encoding="utf-8") as f:
                json.dump(self.scenes, f, ensure_ascii=False, indent=2)
        except OSError:
            pass

    def _load_shows(self):
        data = self._read_shows_file()
        if data:
            self.scenes = data
            self._rebuild_scene_cols()

    # ---- OSC 수신 (씬 트리거) ----
    def _toggle_osc(self, on: bool):
        if on:
            try:
                port = int(self.osc_port.text())
            except ValueError:
                port = 9000
            if self.osc.start(port):
                self.osc_port.setEnabled(False)
            else:
                self.osc_cb.setChecked(False)   # 실패 → 토글 해제
        else:
            self.osc.stop()
            self.osc_port.setEnabled(True)
            if _HAS_OSC:
                self.osc_lbl.setText(f"{OSC_SCENE}/N")

    def _osc_trigger(self, idx: int):
        """OSC 로 받은 씬 인덱스 적용 (메인스레드 슬롯)."""
        if 0 <= idx < len(self.scenes):
            self._apply_scene(idx)
            self.osc_lbl.setText(f"▶ scene {idx + 1}")

    def _osc_info(self, text: str):
        self.osc_lbl.setText(text)

    # ---- 키보드 숫자키로 씬 트리거 (공연용) ----
    @staticmethod
    def _scene_key_label(i: int) -> str:
        """씬 i 를 트리거하는 숫자키 라벨. 0~8→'1'~'9', 9→'0', 그 이상은 단축키 없음."""
        if 0 <= i <= 8:
            return str(i + 1)
        if i == 9:
            return "0"
        return ""

    def keyPressEvent(self, ev):
        """숫자키 1~9,0 → 씬 1~10 트리거. 이름/주파수/진폭 칸을 편집 중이면 키가 그
        위젯으로 가서 여기 안 오므로 타이핑과 충돌하지 않는다 (필드 밖 포커스일 때만 동작)."""
        key = ev.key()
        idx = None
        if QtCore.Qt.Key_1 <= key <= QtCore.Qt.Key_9:
            idx = key - QtCore.Qt.Key_1          # '1'→0 ... '9'→8
        elif key == QtCore.Qt.Key_0:
            idx = 9                               # '0' → 10번째 씬
        if idx is not None and 0 <= idx < len(self.scenes):
            self._apply_scene(idx)
            self.osc_lbl.setText(f"⌨ scene {idx + 1}")   # ⌨
            ev.accept()
            return
        super().keyPressEvent(ev)

    def closeEvent(self, ev):
        self.osc.stop()
        super().closeEvent(ev)

    # ---- 씬 적용 + 스태거드 스타트 엔진 ----
    def _apply_scene(self, idx: int):
        sc = self.scenes[idx]
        self.A.set_scene(sc["name"]); self.B.set_scene(sc["name"])   # 제목에 '적용중' 표시
        # 극성(반대방향) 패널에 적용 (운전 중 바뀌면 ESP32 가 fade-restart 로 매끈 반영)
        self.A.twist.setChecked(sc["a_twist"]); self.B.twist.setChecked(sc["b_twist"])
        n = max(1, int(self.crossfade_spin.value() * SEND_HZ))
        dur = max(1, int(sc["duration"] * SEND_HZ))   # 엔벨로프 진행 스텝 수
        a_speed = max(1, round(sc["a_freq0"] * 10))   # 크로스페이드 도달 = start
        b_speed = max(1, round(sc["b_freq0"] * 10))
        # 엔벨로프(슬라이더 단위/°): start→end, freq/amp 각자 곡선
        a_env = {"f0": a_speed, "f1": max(1, round(sc["a_freq1"] * 10)), "fc": sc["a_fcurve"],
                 "a0": int(sc["a_amp0"]), "a1": int(sc["a_amp1"]), "ac": sc["a_acurve"], "n": dur}
        b_env = {"f0": b_speed, "f1": max(1, round(sc["b_freq1"] * 10)), "fc": sc["b_fcurve"],
                 "a0": int(sc["b_amp0"]), "a1": int(sc["b_amp1"]), "ac": sc["b_acurve"], "n": dur}
        self._pending_start = None
        both_on = sc["a_on"] and sc["b_on"]
        # A = 위상 기준. 바로 적용.
        self.A.begin_transition(a_speed, sc["a_amp0"], sc["a_on"], n, defer_start=False, env=a_env)
        # B = 둘 다 켜는 씬에서 B가 새로 켜질 때만 위상차만큼 스태거.
        defer_b = both_on and (not self.B.running)
        self.B.begin_transition(b_speed, sc["b_amp0"], sc["b_on"], n, defer_start=defer_b, env=b_env)
        if defer_b:
            target = math.radians(sc["phase"]) % TWO_PI
            self._pending_start = {"panel": self.B, "ref": self.A, "target": target, "prev": self.A.phase}

    def _show_step(self):
        """매 틱: 스태거드 스타트 대기 확인 + 두 패널 ramp 진행."""
        p = self._pending_start
        if p is not None and p["ref"].running:
            cur = p["ref"].phase
            if _phase_crossed(p["prev"], cur, p["target"]):
                p["panel"].trigger_start()
                self._pending_start = None
            else:
                p["prev"] = cur
        self.A.step_transition()
        self.B.step_transition()

    def _update_active_enabled(self):
        self._refresh_highlight()

    def _refresh_highlight(self):
        """제어 대상(PS5/키보드) 패널 초록 강조. 통신두절(빨강)은 update_style 이 우선."""
        self.A.active = self.tgt_a.isChecked()
        self.B.active = self.tgt_b.isChecked()
        self.A.update_style()
        self.B.update_style()

    def _active(self) -> SystemPanel:
        return self.A if self.tgt_a.isChecked() else self.B

    # ---------------------------------------------------------------- 입력
    def _input_targets(self):
        return [self._active()]   # PS5/키보드는 제어 대상 패널만

    def _init_js(self):
        try:
            pygame.event.pump()
            if pygame.joystick.get_count() > 0:
                j = pygame.joystick.Joystick(0)
                j.init()
                self._js = j
                self._js_name = j.get_name()
            else:
                self._js = None
                self._js_name = ""
        except Exception:
            self._js = None

    def _poll_pad(self):
        if not _HAS_PYGAME:
            self.pad_lbl.setText("PS5: no pygame")
            return
        try:
            pygame.event.pump()
            if self._js is None and pygame.joystick.get_count() > 0:
                self._init_js()
            if self._js is None:
                self.pad_lbl.setText("PS5: not connected (use keyboard)")
                return
            xb = bool(self._js.get_button(0))   # ✕ Cross = STOP
            ob = bool(self._js.get_button(1))   # ○ Circle = START
        except Exception:
            self._js = None
            self.pad_lbl.setText("PS5: not connected")
            return
        tgts = self._input_targets()
        if xb and not self._xprev:
            for p in tgts:
                p.stop()
        if ob and not self._oprev:
            for p in tgts:
                p.start()
        self._xprev, self._oprev = xb, ob
        self.pad_lbl.setText(f"PS5: {self._js_name[:16]}  target {self._active().name}")

    # ---------------------------------------------------------------- 60Hz
    def _tick(self):
        dt = 1.0 / SEND_HZ
        drop = self.drop_slider.value()

        self._poll_pad()
        self._show_step()      # 씬 전환 ramp + 스태거드 스타트 (슬라이더를 목표로 이동)

        self.A.tick(dt, drop, self.sock)
        self.B.tick(dt, drop, self.sock)

        self._recv_telem()
        self.A.refresh_status()
        self.B.refresh_status()
        self.A.update_style()   # 통신두절↔복구 시 테두리 색 라이브 갱신 (변경 시에만 적용)
        self.B.update_style()

    def _recv_telem(self):
        try:
            while True:
                data, addr = self.sock.recvfrom(256)
                t = proto.unpack_telem(data)
                if not t:
                    continue
                ip = addr[0]
                if ip == self.A.ip():
                    self.A.apply_telem(t)
                elif ip == self.B.ip():
                    self.B.apply_telem(t)
        except (BlockingIOError, OSError):
            pass


def _apply_dark(app):
    """다크(나이트) 모드 — Fusion + 어두운 팔레트."""
    app.setStyle("Fusion")
    g = QtGui.QColor
    p = QtGui.QPalette()
    text = g(220, 220, 220)
    p.setColor(QtGui.QPalette.Window, g(37, 37, 38))
    p.setColor(QtGui.QPalette.WindowText, text)
    p.setColor(QtGui.QPalette.Base, g(30, 30, 30))
    p.setColor(QtGui.QPalette.AlternateBase, g(45, 45, 46))
    p.setColor(QtGui.QPalette.Text, text)
    p.setColor(QtGui.QPalette.Button, g(53, 53, 54))
    p.setColor(QtGui.QPalette.ButtonText, text)
    p.setColor(QtGui.QPalette.ToolTipBase, g(45, 45, 46))
    p.setColor(QtGui.QPalette.ToolTipText, text)
    p.setColor(QtGui.QPalette.Highlight, g(0x33, 0xcc, 0x88))
    p.setColor(QtGui.QPalette.HighlightedText, g(0, 0, 0))
    dim = g(120, 120, 120)
    p.setColor(QtGui.QPalette.Disabled, QtGui.QPalette.Text, dim)
    p.setColor(QtGui.QPalette.Disabled, QtGui.QPalette.ButtonText, dim)
    p.setColor(QtGui.QPalette.Disabled, QtGui.QPalette.WindowText, dim)
    app.setPalette(p)


def main():
    app = QtWidgets.QApplication(sys.argv)
    _apply_dark(app)
    # 글씨 1.5배 — 무대 현장 가독성 (2배는 너무 커서 공간 부족)
    f = app.font()
    base = f.pointSizeF() if f.pointSizeF() > 0 else 9.0
    f.setPointSizeF(base * 1.5)
    app.setFont(f)
    win = Controller()
    win.resize(1500, 940)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
