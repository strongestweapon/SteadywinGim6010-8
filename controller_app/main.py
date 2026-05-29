"""
main.py — 모터 LFO 컨트롤러 데스크탑 앱 (PySide6 + pyqtgraph)
===============================================================================
컴퓨터에서 사인 LFO 를 생성해 60Hz 로 ESP32 에 {run, freq, amp, phase} 송신 + 실시간 파형 시각화.
ESP32 는 이 스트림에 로컬 오실레이터를 lock; 패킷 끊기면 자유진행 → 안전.

전송: 일단 UDP (target IP:port). 이후 ESP-NOW 로 이행해도 이 앱/프로토콜 그대로.

실행:
    pip install -r requirements.txt
    python main.py

테스트 팁:
  - ESP32 없이도 실행됨 (파형 비주얼 + 송신만). 'drop %' 올려 패킷손실 시뮬 가능.
  - ESP32 IP 는 ESP32 가 시리얼에 찍는 값으로 설정.
"""
from __future__ import annotations
import sys
import math
import socket

import numpy as np
from PySide6 import QtWidgets, QtCore
import pyqtgraph as pg

import protocol as proto

SEND_HZ = 60                  # 송신/생성 주기
PLOT_SECONDS = 4.0            # 파형 표시 구간
PLOT_N = int(SEND_HZ * PLOT_SECONDS)


class Controller(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Steadywin LFO Controller")

        # ---- LFO 상태 ----
        self.phase = 0.0          # rad
        self.running = False
        self.seq = 0
        self.sent = 0
        self.dropped = 0

        # ---- UDP 소켓 ----
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        # ---- 플롯 버퍼 ----
        self.buf = np.zeros(PLOT_N, dtype=float)

        self._build_ui()

        # ---- 60Hz 타이머 ----
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(int(1000 / SEND_HZ))
        self.timer.timeout.connect(self._tick)
        self.timer.start()

    # ---------------------------------------------------------------- UI
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        # 파형 플롯
        self.plot = pg.PlotWidget()
        self.plot.setYRange(-proto.AMP_DEG_MAX * 1.1, proto.AMP_DEG_MAX * 1.1)
        self.plot.setLabel("left", "출력축 명령", units="deg")
        self.plot.setLabel("bottom", "시간", units="s")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.curve = self.plot.plot(pen=pg.mkPen("#33cc88", width=2))
        self._t_axis = np.linspace(-PLOT_SECONDS, 0.0, PLOT_N)
        root.addWidget(self.plot, stretch=1)

        # 컨트롤 영역
        form = QtWidgets.QGridLayout()
        root.addLayout(form)

        # 대상 IP / 포트
        form.addWidget(QtWidgets.QLabel("ESP32 IP"), 0, 0)
        self.ip_edit = QtWidgets.QLineEdit("192.168.0.102")
        form.addWidget(self.ip_edit, 0, 1)
        form.addWidget(QtWidgets.QLabel("Port"), 0, 2)
        self.port_edit = QtWidgets.QLineEdit(str(proto.UDP_PORT))
        self.port_edit.setMaximumWidth(70)
        form.addWidget(self.port_edit, 0, 3)

        # 주파수 슬라이더 (0.1 ~ FREQ_MAX Hz)
        form.addWidget(QtWidgets.QLabel("주파수"), 1, 0)
        self.freq_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.freq_slider.setRange(1, int(proto.FREQ_MAX * 10))  # 0.1Hz 단위
        self.freq_slider.setValue(10)                            # 1.0Hz
        form.addWidget(self.freq_slider, 1, 1, 1, 2)
        self.freq_lbl = QtWidgets.QLabel("1.0 Hz")
        form.addWidget(self.freq_lbl, 1, 3)
        self.freq_slider.valueChanged.connect(self._update_labels)

        # 강도(drive) 슬라이더 (0~100%) — 실제 진폭 = 강도 × 그 주파수의 허용 최대치
        # → 주파수 올리면 진폭이 자동으로 줄어듦 (vel_limit coupling)
        form.addWidget(QtWidgets.QLabel("강도(%)"), 2, 0)
        self.amp_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.amp_slider.setRange(0, 100)
        self.amp_slider.setValue(60)
        form.addWidget(self.amp_slider, 2, 1, 1, 2)
        self.amp_lbl = QtWidgets.QLabel("60 % → ±0°")
        form.addWidget(self.amp_lbl, 2, 3)
        self.amp_slider.valueChanged.connect(self._update_labels)

        # 패킷 손실 시뮬 (ESP32 자유진행 테스트용)
        form.addWidget(QtWidgets.QLabel("drop %"), 3, 0)
        self.drop_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.drop_slider.setRange(0, 90)
        self.drop_slider.setValue(0)
        form.addWidget(self.drop_slider, 3, 1, 1, 2)
        self.drop_lbl = QtWidgets.QLabel("0 %")
        form.addWidget(self.drop_lbl, 3, 3)
        self.drop_slider.valueChanged.connect(self._update_labels)

        # Start/Stop (= BOOT 역할) + 상태
        self.btn = QtWidgets.QPushButton("▶ START")
        self.btn.setCheckable(True)
        self.btn.setStyleSheet("font-size:16px; padding:8px;")
        self.btn.toggled.connect(self._toggle_run)
        form.addWidget(self.btn, 4, 0, 1, 2)
        self.status_lbl = QtWidgets.QLabel("정지")
        form.addWidget(self.status_lbl, 4, 2, 1, 2)

        self._update_labels()

    def _update_labels(self):
        freq = self.freq_slider.value() / 10.0
        drive = self.amp_slider.value()
        amp_deg = drive / 100.0 * proto.amp_deg_max(freq)   # 주파수 coupling
        self.freq_lbl.setText(f"{freq:.1f} Hz")
        self.amp_lbl.setText(f"{drive} % → ±{amp_deg:.1f}°")
        self.drop_lbl.setText(f"{self.drop_slider.value()} %")

    def _toggle_run(self, on: bool):
        self.running = on
        self.btn.setText("■ STOP" if on else "▶ START")
        if not on:
            self.phase = 0.0  # 정지 시 위상 리셋 (재시작 시 중심부터)

    # ---------------------------------------------------------------- 60Hz 루프
    def _tick(self):
        dt = 1.0 / SEND_HZ
        freq = self.freq_slider.value() / 10.0
        # 강도% × 주파수 허용 최대 → 진폭 자동 coupling (주파수↑ → 진폭↓)
        amp_deg = self.amp_slider.value() / 100.0 * proto.amp_deg_max(freq)

        if self.running:
            self.phase += 2.0 * math.pi * freq * dt
            if self.phase > 2.0 * math.pi:
                self.phase -= 2.0 * math.pi
            value = amp_deg * math.sin(self.phase)
        else:
            value = 0.0

        # ---- 패킷 송신 (drop 시뮬 제외) ----
        self.seq = (self.seq + 1) & 0xFFFF
        drop = self.drop_slider.value()
        do_send = not (drop > 0 and (np.random.randint(0, 100) < drop))
        if do_send:
            pkt = proto.pack(self.seq, 1 if self.running else 0,
                             freq, amp_deg, self.phase)
            try:
                self.sock.sendto(pkt, (self.ip_edit.text().strip(),
                                       int(self.port_edit.text())))
                self.sent += 1
            except OSError:
                pass  # ESP32 미연결이어도 앱은 계속 (비주얼/테스트)
        else:
            self.dropped += 1

        # ---- 플롯 갱신 ----
        self.buf = np.roll(self.buf, -1)
        self.buf[-1] = value
        self.curve.setData(self._t_axis, self.buf)

        # ---- 상태 ----
        self.status_lbl.setText(
            f"{'RUN' if self.running else '정지'}  seq={self.seq}  "
            f"sent={self.sent} drop={self.dropped}"
        )


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = Controller()
    win.resize(820, 560)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
