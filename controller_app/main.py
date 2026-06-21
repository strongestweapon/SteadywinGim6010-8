"""
main.py — 듀얼 ESP32 모터 컨트롤러 데스크탑 앱 (PySide6)
===============================================================================
ESP32 **2대**(각각 모터쌍 1조)를 한 앱에서 제어. 좌우 2패널 + LINK(동기) 토글.

레이아웃:
  - 상단(공유): LINK 토글 / 파형(사인·수동) / drop(패킷손실 시뮬) / PS5 상태·대상
  - 좌/우 패널: IP·포트 / 속도 / 강도(=수동 최대각) / 극성 / START·STOP / IMU 0점 / 상태
  - LINK ON  : A 가 마스터 → B 가 A 를 미러(컨트롤 비활성). PS5·키보드는 A 에 적용(둘 다 움직임).
  - LINK OFF : 각 패널 독립. PS5·키보드는 "대상"으로 고른 패널만.

전송: 소켓 1개로 두 IP 에 각각 UDP 송신. 텔레메트리는 발신 IP 로 구분 수신.
조작:
  - 사인 : 속도=주파수(Hz), 강도=진폭%
  - 수동 : 속도=슬루(turn/s), 강도=토글 최대각(0~90°). Space/←/→ = -측↔+측 토글(스탠바이 -측).
  - PS5  : ○=START, ✕=STOP, 왼쪽 스틱 좌우=측 직접지정. (LINK 따라 둘다/선택 패널)

실행: python main.py   (ESP32 없이도 송신만 동작)
"""
from __future__ import annotations
import sys
import math
import socket
import time
from collections import deque

from PySide6 import QtWidgets, QtCore, QtGui
import numpy as np
import pyqtgraph as pg

try:
    import pygame   # PS5(DualSense) 등 조이스틱 (선택 — 없으면 키보드만)
    _HAS_PYGAME = True
except ImportError:
    _HAS_PYGAME = False

import protocol as proto

SEND_HZ = 60                  # 송신 주기
PLOT_SECONDS = 4.0            # 파형 그래프 표시 구간
PLOT_N = int(SEND_HZ * PLOT_SECONDS)
MANUAL_LIMIT_DEG = 90.0       # 수동 ±한계 [출력°] (ESP32 HARD_LIMIT_TURN 과 동일)
MANUAL_SPEED_TS = 10.0        # 수동 최대 슬루 속도 [turn/s] (속도 슬라이더 최대). ESP32 MANUAL_SPEED_MAX 와 동일
JS_DEADZONE = 0.12            # PS5 스틱 데드존


class SystemPanel(QtWidgets.QGroupBox):
    """ESP32 1대 분의 컨트롤 + 상태. 자기 IP 로 패킷을 만들어 보낸다(소켓은 공유)."""

    def __init__(self, name: str, default_ip: str):
        super().__init__(f"시스템 {name}")
        self.name = name
        self._default_ip = default_ip
        # 송신 상태
        self.running = False
        self.seq = 0
        self.phase = 0.0          # 사인 위상 [rad]
        self.manual_side = -1     # 수동 토글: -1=-측(스탠바이), +1=+측
        self.manual_right = False
        self.tare_ticks = 0
        # 수신/표시 상태
        self.telem = None
        self.pk_i = 0.0
        self.pk_vmin = 999.0
        # 실측 패킷손실 (ESP32→앱 텔레메트리 seq 간격으로 측정)
        self._telem_last_seq = None
        self._gap_window = deque(maxlen=90)   # 최근 ~3s (telem ≈30Hz)
        self._telem_last_t = 0.0              # 마지막 수신 시각 [monotonic]
        self.drop_meas = 0.0                  # 측정 손실율 [%]
        self.value = 0.0                  # 현재 명령값 [출력°] (그래프용)
        self.buf = np.zeros(PLOT_N)       # 파형 롤링 버퍼
        self._build()

    # ---------------------------------------------------------------- UI
    def _build(self):
        g = QtWidgets.QGridLayout(self)
        r = 0
        g.addWidget(QtWidgets.QLabel("파형"), r, 0)
        self.wave_combo = QtWidgets.QComboBox()
        self.wave_combo.addItem("사인 (ripple/진동)", proto.WAVE_SINE)
        self.wave_combo.addItem("수동 스윙 (Space/←/→ 토글)", proto.WAVE_MANUAL)
        self.wave_combo.currentIndexChanged.connect(self._labels)
        g.addWidget(self.wave_combo, r, 1, 1, 3)

        r += 1
        g.addWidget(QtWidgets.QLabel("IP"), r, 0)
        self.ip_edit = QtWidgets.QLineEdit(self._default_ip)
        g.addWidget(self.ip_edit, r, 1)
        g.addWidget(QtWidgets.QLabel("Port"), r, 2)
        self.port_edit = QtWidgets.QLineEdit(str(proto.UDP_PORT))
        self.port_edit.setMaximumWidth(60)
        g.addWidget(self.port_edit, r, 3)

        r += 1
        g.addWidget(QtWidgets.QLabel("속도"), r, 0)
        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(1, int(proto.FREQ_MAX * 10))   # 0.1 단위
        self.speed_slider.setValue(10)
        g.addWidget(self.speed_slider, r, 1, 1, 2)
        self.speed_lbl = QtWidgets.QLabel("")
        g.addWidget(self.speed_lbl, r, 3)
        self.speed_slider.valueChanged.connect(self._labels)

        r += 1
        g.addWidget(QtWidgets.QLabel("강도/각도"), r, 0)
        self.angle_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.angle_slider.setRange(0, 100)
        self.angle_slider.setValue(60)
        g.addWidget(self.angle_slider, r, 1, 1, 2)
        self.angle_lbl = QtWidgets.QLabel("")
        g.addWidget(self.angle_lbl, r, 3)
        self.angle_slider.valueChanged.connect(self._labels)

        r += 1
        g.addWidget(QtWidgets.QLabel("극성반전"), r, 0)
        pol = QtWidgets.QHBoxLayout()
        self.pol_m1 = QtWidgets.QCheckBox("모터1")
        self.pol_m2 = QtWidgets.QCheckBox("모터2")
        self.pol_m2.setChecked(True)   # node2 미러 장착 기본 반전
        pol.addWidget(self.pol_m1)
        pol.addWidget(self.pol_m2)
        pol.addStretch(1)
        w = QtWidgets.QWidget()
        w.setLayout(pol)
        g.addWidget(w, r, 1, 1, 3)

        r += 1
        self.btn = QtWidgets.QPushButton("▶ START")
        self.btn.setCheckable(True)
        self.btn.setStyleSheet("font-weight:bold; padding:10px;")
        self.btn.toggled.connect(self._toggle)
        g.addWidget(self.btn, r, 0, 1, 2)
        self.tare_btn = QtWidgets.QPushButton("IMU 0점")
        self.tare_btn.clicked.connect(lambda: setattr(self, "tare_ticks", 6))
        g.addWidget(self.tare_btn, r, 2, 1, 2)

        r += 1
        self.status_lbl = QtWidgets.QLabel("정지")
        self.status_lbl.setStyleSheet("font-family:monospace;")
        self.status_lbl.setWordWrap(True)
        g.addWidget(self.status_lbl, r, 0, 1, 4)

        r += 1
        self.plot = pg.PlotWidget()
        self.plot.setMinimumHeight(140)
        self.plot.setYRange(-MANUAL_LIMIT_DEG * 1.1, MANUAL_LIMIT_DEG * 1.1)
        self.plot.setLabel("left", "명령", units="deg")
        self.plot.setLabel("bottom", "시간", units="s")
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
        self.btn.setText("■ STOP" if on else "▶ START")
        if on:
            self.pk_i = 0.0
            self.pk_vmin = 999.0
            self.manual_side = -1   # START 시 -측(스탠바이)
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

    def toggle_manual(self):
        self.manual_side = -self.manual_side

    def set_controls_enabled(self, en: bool):
        for w in (self.wave_combo, self.speed_slider, self.angle_slider, self.pol_m1,
                  self.pol_m2, self.btn, self.tare_btn):
            w.setEnabled(en)

    def wave(self) -> int:
        return self.wave_combo.currentData()

    def _labels(self):
        wave = self.wave_combo.currentData()
        freq = self.speed_slider.value() / 10.0
        drive = self.angle_slider.value()
        if wave == proto.WAVE_MANUAL:
            self.speed_lbl.setText(f"{freq / proto.FREQ_MAX * MANUAL_SPEED_TS:.1f} t/s")
            self.angle_lbl.setText(f"±{drive / 100.0 * MANUAL_LIMIT_DEG:.0f}°")
        else:
            self.speed_lbl.setText(f"{freq:.1f} Hz")
            self.angle_lbl.setText(f"±{drive / 100.0 * proto.amp_deg_max(freq, 1.0):.0f}°")

    # ---------------------------------------------------------------- 송신
    def tick(self, dt: float, drop_pct: int, sock: socket.socket):
        wave = self.wave_combo.currentData()
        freq = self.speed_slider.value() / 10.0
        drive = self.angle_slider.value()
        self.manual_right = False

        if wave == proto.WAVE_MANUAL:
            amp_max = drive / 100.0 * MANUAL_LIMIT_DEG   # 강도 = 최대각 0~90°
            amp_deg = amp_max                            # 송신=크기(부호는 BTN_RIGHT)
            send_freq = freq / proto.FREQ_MAX * MANUAL_SPEED_TS
            send_phase = 0.0
            self.manual_right = (self.manual_side > 0)
            self.value = (self.manual_side * amp_max) if self.running else 0.0
        else:
            amp_deg = drive / 100.0 * proto.amp_deg_max(freq, 1.0)
            if self.running:
                self.phase += 2.0 * math.pi * freq * dt
                if self.phase > 2.0 * math.pi:
                    self.phase -= 2.0 * math.pi
                self.value = amp_deg * math.sin(self.phase)
            else:
                self.value = 0.0
            send_freq = freq
            send_phase = self.phase

        # 파형 그래프 갱신 (명령값)
        self.buf = np.roll(self.buf, -1)
        self.buf[-1] = self.value
        self.curve.setData(self._t_axis, self.buf)

        self.seq = (self.seq + 1) & 0xFFFF
        if drop_pct > 0 and _rand100() < drop_pct:
            return   # 패킷손실 시뮬
        flags = 0
        if self.pol_m1.isChecked():
            flags |= proto.POL_M1
        if self.pol_m2.isChecked():
            flags |= proto.POL_M2
        if self.tare_ticks > 0:
            flags |= proto.REQ_TARE
            self.tare_ticks -= 1
        if self.manual_right:
            flags |= proto.BTN_RIGHT
        pkt = proto.pack(self.seq, 1 if self.running else 0,
                         send_freq, amp_deg, send_phase, waveform=wave, flags=flags)
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

    def refresh_status(self):
        run = "RUN" if self.running else "정지"
        if self.manual_right:
            run += " +측"
        elif self.running:
            run += " -측"
        t = self.telem
        if t is None:
            self.status_lbl.setText(f"{run}  (텔레메트리 대기)")
            return
        # 신호 끊김 = 텔레메트리가 1초 이상 안 들어옴 → 실측 손실 100%
        stale = (time.monotonic() - self._telem_last_t) > 1.0
        drop = 100.0 if stale else self.drop_meas
        dwarn = "  ⚠신호끊김" if stale else ""
        warn = ""
        if t["ibus"] > 3.0:
            warn = " ⚠과전류"
        elif 1.0 < t["vbus"] < 19.0:
            warn = " ⚠저전압"
        imu = f"tilt {t['tilt']:.0f}°" if t["imu_ok"] else "IMU없음"
        vmin = f"{self.pk_vmin:.1f}" if self.pk_vmin < 900 else "—"
        self.status_lbl.setText(
            f"{run}   I {t['m1_iq']:+.1f}/{t['m2_iq']:+.1f}A  V {t['vbus']:.1f}  "
            f"Ibus {t['ibus']:.2f}A{warn}\n{imu}   실drop {drop:.0f}%{dwarn}   "
            f"pk: Imax {self.pk_i:.1f}A  Vmin {vmin}")


def _rand100() -> int:
    # numpy 의존 제거용 간단 난수 (drop 시뮬 only)
    import random
    return random.randint(0, 99)


class Controller(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Steadywin 듀얼 ESP32 컨트롤러")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        # PS5
        self._js = None
        self._js_name = ""
        self._xprev = False
        self._oprev = False
        self._sqprev = False   # □ Square 직전 상태 (rising-edge=방향 토글)
        if _HAS_PYGAME:
            pygame.init()
            pygame.joystick.init()
            self._init_js()

        self._build()
        QtWidgets.QApplication.instance().installEventFilter(self)

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
        self.link_cb = QtWidgets.QCheckBox("LINK 동기 (A=마스터, B 미러)")
        self.link_cb.toggled.connect(self._update_active_enabled)
        top.addWidget(self.link_cb)
        top.addWidget(QtWidgets.QLabel("drop 시뮬%"))
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
        self.A = SystemPanel("A", "192.168.0.44")   # 무대 오른쪽 (기존 ESP32)
        self.B = SystemPanel("B", "192.168.0.46")   # 무대 왼쪽 (새 ESP32)
        self.tgt_a = QtWidgets.QRadioButton("● A 를 제어")
        self.tgt_b = QtWidgets.QRadioButton("● B 를 제어")
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
            "수동: Space/←/→ = -측↔+측 토글 (스탠바이 -측).  PS5: ○=START ✕=STOP 왼쪽스틱=측.  "
            "LINK 켜면 A가 둘 다 제어.")
        hint.setStyleSheet("color:#888;")
        root.addWidget(hint)

        self._update_active_enabled()

    def _update_active_enabled(self):
        linked = self.link_cb.isChecked()
        self.tgt_a.setEnabled(not linked)
        self.tgt_b.setEnabled(not linked)
        self._refresh_highlight()

    def _refresh_highlight(self):
        """제어 중인 패널을 초록 테두리로. LINK 면 A 가 둘 다 제어 → 둘 다 강조."""
        linked = self.link_cb.isChecked()
        self._set_active_style(self.A, linked or self.tgt_a.isChecked())
        self._set_active_style(self.B, linked or self.tgt_b.isChecked())

    @staticmethod
    def _set_active_style(panel, active: bool):
        if active:
            panel.setStyleSheet(
                "QGroupBox{border:2px solid #33cc88; border-radius:6px; margin-top:8px;}"
                "QGroupBox::title{subcontrol-origin:margin; left:10px; color:#33cc88; font-weight:bold;}")
        else:
            panel.setStyleSheet(
                "QGroupBox{border:1px solid #555; border-radius:6px; margin-top:8px;}"
                "QGroupBox::title{subcontrol-origin:margin; left:10px; color:#888;}")

    def _active(self) -> SystemPanel:
        return self.A if self.tgt_a.isChecked() else self.B

    # ---------------------------------------------------------------- 입력
    def eventFilter(self, obj, ev):
        if ev.type() == QtCore.QEvent.KeyPress and not ev.isAutoRepeat():
            if ev.key() in (QtCore.Qt.Key_Left, QtCore.Qt.Key_Right, QtCore.Qt.Key_Space):
                for p in self._input_targets():
                    p.toggle_manual()
                return True
        return super().eventFilter(obj, ev)

    def _input_targets(self):
        # LINK 면 A 만 조작(=B 미러로 따라옴), 아니면 선택 패널
        return [self.A] if self.link_cb.isChecked() else [self._active()]

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
            self.pad_lbl.setText("PS5: pygame 없음")
            return
        try:
            pygame.event.pump()
            if self._js is None and pygame.joystick.get_count() > 0:
                self._init_js()
            if self._js is None:
                self.pad_lbl.setText("PS5: 미연결 (키보드 사용)")
                return
            xb = bool(self._js.get_button(0))   # ✕ Cross = STOP
            ob = bool(self._js.get_button(1))   # ○ Circle = START
            sq = bool(self._js.get_button(2))   # □ Square = 방향 토글
            ax = self._js.get_axis(0)
        except Exception:
            self._js = None
            self.pad_lbl.setText("PS5: 미연결")
            return
        tgts = self._input_targets()
        if xb and not self._xprev:
            for p in tgts:
                p.stop()
        if ob and not self._oprev:
            for p in tgts:
                p.start()
        if sq and not self._sqprev:
            for p in tgts:
                p.toggle_manual()       # □ 누를 때마다 -측↔+측 토글
        self._xprev, self._oprev, self._sqprev = xb, ob, sq
        if abs(ax) > 0.5:               # 스틱은 방향 직접지정(보조)
            side = +1 if ax > 0 else -1
            for p in tgts:
                p.manual_side = side
        scope = "A+B (LINK)" if self.link_cb.isChecked() else self._active().name
        self.pad_lbl.setText(f"PS5: {self._js_name[:16]}  대상 {scope}")

    # ---------------------------------------------------------------- 60Hz
    def _tick(self):
        dt = 1.0 / SEND_HZ
        drop = self.drop_slider.value()
        link = self.link_cb.isChecked()

        self._poll_pad()

        if link:
            self._mirror(self.A, self.B)
            self.B.set_controls_enabled(False)
        else:
            self.B.set_controls_enabled(True)

        self.A.tick(dt, drop, self.sock)
        self.B.tick(dt, drop, self.sock)

        self._recv_telem()
        self.A.refresh_status()
        self.B.refresh_status()

    def _mirror(self, a: SystemPanel, b: SystemPanel):
        """LINK: A → B 미러 (IP/Port 제외한 제어값 — 파형/속도/각도/극성/측/run)."""
        if b.wave_combo.currentIndex() != a.wave_combo.currentIndex():
            b.wave_combo.setCurrentIndex(a.wave_combo.currentIndex())
        if b.speed_slider.value() != a.speed_slider.value():
            b.speed_slider.setValue(a.speed_slider.value())
        if b.angle_slider.value() != a.angle_slider.value():
            b.angle_slider.setValue(a.angle_slider.value())
        b.pol_m1.setChecked(a.pol_m1.isChecked())
        b.pol_m2.setChecked(a.pol_m2.isChecked())
        b.manual_side = a.manual_side
        if b.running != a.running:
            b.btn.setChecked(a.running)

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


def main():
    app = QtWidgets.QApplication(sys.argv)
    # 글씨 2배 — 무대 현장에서 멀리서도 보이게 (시스템 기본 폰트의 2배)
    f = app.font()
    base = f.pointSizeF() if f.pointSizeF() > 0 else 9.0
    f.setPointSizeF(base * 2.0)
    app.setFont(f)
    win = Controller()
    win.resize(1500, 820)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
