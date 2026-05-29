# controller_app — 모터 LFO 컨트롤러 (데스크탑)

컴퓨터에서 사인 LFO 를 생성해 ESP32(T-2CAN)로 `{run, freq, amp, phase}` 를 60Hz 스트리밍 + 실시간 파형 시각화.
ESP32 는 이 스트림에 로컬 오실레이터를 PLL 처럼 lock; 패킷 끊기면 자유진행(사인 계속) → 안전.

## 스택
- **PySide6** (GUI) + **pyqtgraph** (실시간 파형) + **numpy**
- 전송: **UDP** (이후 ESP-NOW 로 이행 가능 — 프로토콜 동일)

## 설치 / 실행
```
pip install -r requirements.txt
python main.py
```
> ESP32 없이도 실행됨 (파형 비주얼 + 송신만). 'drop %' 슬라이더로 패킷손실 시뮬.

## 사용
- **ESP32 IP/Port**: ESP32 가 시리얼에 찍는 IP 입력 (UDP_PORT 기본 4210)
- **주파수/진폭 슬라이더**: 라이브 변조 (스피커 LFO 처럼)
- **drop %**: 패킷 일부러 버려 ESP32 자유진행/재동기 테스트
- **START/STOP**: 앱의 시작/정지 (= ESP32 의 BOOT 역할, run 플래그)

## 프로토콜
`protocol.py` 참고. 20바이트 LE 패킷: `{magic, seq, run, waveform, freq, amp_deg, phase}`.

## 설계 메모
- 앱은 "파형 모델"을 던지고, ESP32 가 로컬에서 합성 → 패킷손실에 강함.
- 긴 무수신 → ESP32 가 fade-out 후 IDLE (안전). 짧은 손실 → 자유진행으로 메움.
- 4Hz 상한 + 고주파에서 진폭은 ESP32 가 vel_limit 으로 자동 클램프.
