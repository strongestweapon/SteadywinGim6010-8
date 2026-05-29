# esp32t2can — LilyGo T-2CAN → ODrive CAN + Wi-Fi 무선 LFO 제어

LilyGo **T-2CAN** (ESP32-S3, MCP2515) 이 ODrive 를 CAN 으로 구동하고, 데스크탑 앱(`../controller_app`)이
Wi-Fi UDP 로 던지는 `{run, freq, amp}` 를 받아 **로컬 오실레이터**로 부드러운 swing 을 생성.

**현재 구현(2026-05-30)**: POS+ff(`Set_Input_Pos`) 200Hz 구동, 파라미터 무선 스트리밍 + 로컬 오실레이터
(패킷손실에 강함), 통신두절 시 IDLE, 주파수-진폭 coupling(4Hz 상한). 상세는 루트 `CLAUDE.md`
"CAN / ESP32 무선 LFO 제어 — 실제 구현" 참고.

> 멘탈 모델: **"모터 = 느리고(≤4Hz) 한계 있는 DC 무선 스피커."**

## 하드웨어: LilyGo T-2CAN

| 항목 | 값 |
|---|---|
| MCU | ESP32-S3-WROOM-1U (16MB flash, 8MB PSRAM, Wi-Fi/BLE5) |
| CAN | **듀얼 CAN, 컨트롤러 = MCP2515 (SPI), Classic CAN 2.0** |
| 전원 | DC 5~12V / USB |
| 프레임워크 | Arduino / PlatformIO (VS Code) |
| repo | https://github.com/Xinyuan-LilyGO/T-2Can |
| 예제 | `examples/can/can.ino`, `examples/original_test` (repo 에 존재 확인) |

> **보드 변형 주의**: T-2CAN 은 2종 — **T-2Can V1.0 = MCP2515 (Classic 2.0)**, T-2Can-Fd V1.0 = MCP2518FD (CAN FD).
> 우리 보드 = **MCP2515 변형** (칩 각인 확인). repo `pin_config.h` 기본 매크로는 FD 라 헷갈리지 말 것.

### 핀맵 (정본: repo `libraries/private_library/pin_config.h`, MCP2515 변형)
| 신호 | GPIO |
|---|---|
| SPI SCLK | 12 |
| SPI MOSI | 11 |
| SPI MISO | 13 |
| MCP2515 CS (CAN0) | 10 |
| MCP2515 INT | 8 |
| MCP2515 RST | 9 |
| MCP2515 크리스탈 | **16 MHz** (아래 근거) |
| native TWAI (미사용) | TX=7, RX=6 |
| BOOT | 0 |

⚠️ 듀얼 채널 중 2번째(CAN1) 의 CS/INT 핀은 풀 `pin_config.h` 에서 **듀얼 모터 단계 때 재확인**.
단일 모터/단일 버스 단계에선 CAN0(CS=10) 만 사용.

## 핵심: MCP2515 = Classic CAN 2.0 → ODrive 와 네이티브로 맞음

- T-2CAN(우리 변형) 의 **MCP2515 는 Classic CAN 2.0** 컨트롤러.
- **ODrive can_simple 도 Classic CAN 2.0** → FD↔classic 모드 변환 불필요, 딱 맞음.
- ODrive 공식 `ODriveArduino` 예제 `SineWaveCAN.ino` 의 **`IS_MCP2515` 경로** 그대로 사용.

### MCP2515 크리스탈 = 16 MHz (근거)
- LilyGo `examples/can/can.ino` 가 `Can_A.setBitrate(CAN_500KBPS)` **단일 인자** 호출.
- autowp `arduino-mcp2515` 의 단일 인자 `setBitrate` 구현은 `MCP_16MHZ` 기본 → LilyGo 보드 동작.
- 즉 보드 크리스탈 = **16MHz**. (sandeepmistry 라이브러리에선 `CAN.setClockFrequency(16000000)` 로 명시)
- 최종 확인: 보드 MCP2515 옆 크리스탈 각인 "16.000".

## 펌웨어 스택

| 역할 | 라이브러리 |
|---|---|
| ODrive can_simple 메시지 인코드/디코드 + MCP 래퍼 | `ODriveArduino` (`ODriveMCPCAN.hpp`) |
| MCP2515 드라이버 | `sandeepmistry/arduino-CAN` (`MCP2515.h`) |

`platformio.ini` 의 `lib_deps` 로 자동 설치.
⚠️ sandeepmistry/arduino-CAN 가 ESP32-S3 에서 빌드 실패 시 → maintained fork 또는 autowp(+자체 래퍼)로 교체.
(LilyGo 예제는 autowp `mcp2515.h` 사용 — 단 ODrive 래퍼는 sandeepmistry API 전제.)

## ODrive 쪽 CAN 설정 (Phase 1 — USB 로 접속, 보드 명령 + save 승인 필요)

> **주의**: 아래 속성 경로/값은 **Phase 1 직전 ODrive 0.6.5 공식 문서로 재확인** 후 적용
> (펌웨어 버전별 경로 상이. 추측 금지 — 프로젝트 규칙 2). `save_configuration()` 은 영구 변경.

### 보드 실측값 (2026-05-29 `probe_can.py`, 모터1 89340A6C3037 기준)
이미 CAN 에 맞게 설정돼 있어 **대부분 변경 불필요**:

| 항목 | 경로 | 실측값 | 조치 |
|---|---|---|---|
| baud_rate | `odrv0.can.config.baud_rate` | **500000** | ESP32 를 500k 로 맞춤 (보드 변경 X) |
| protocol | `odrv0.can.config.protocol` | 1 = CANSimple | OK |
| node_id | `axis0.config.can.node_id` | 1 | OK (듀얼 모터 시 2번째를 2로) |
| is_extended | `axis0.config.can.is_extended` | False (11-bit) | OK |
| heartbeat | `axis0.config.can.heartbeat_rate_ms` | 100 | OK |
| 엔코더 송신 | `axis0.config.can.encoder_rate_ms` | 10 (100Hz) | OK — ESP32 가 CAN 으로 pos/vel 수신 가능 |
| watchdog | `axis0.config.enable_watchdog` / `.watchdog_timeout` | False / 0.0 | 지금은 그대로. 무인/무대 운용 직전에만 True/0.1 |

> ⚠️ baud_rate=500000 은 모터1(save 된 개체) 실측값. **모터 2~5 는 공장 default 라 다를 수 있음**
> → 듀얼/다중 모터 단계에서 각 모터 `probe_can.py` 로 baud 일치 재확인.

### watchdog 는 왜/언제? (테스트엔 불필요)
통신(CAN)이 끊겼을 때 모터가 마지막 명령을 계속 실행하는 걸 막는 **안전장치** —
`enable_watchdog=True` + `watchdog_timeout=0.1` 이면 100ms 명령 없을 시 자동 IDLE.
**모터 동작 필수 아님.** 매 프레임 feed 가 전제라 60Hz 송신이 안정적으로 돈 뒤에 켜야 함.
→ 초기 벤치 테스트는 **꺼둠**. 무인/무대 운용 직전 또는 failsafe 테스트 시에만 켜고 `save_configuration()`.

CAN 활성화해도 USB 접속은 계속 가능 → USB 로 모니터하며 CAN 송신 테스트 가능.

## 배선

```
T-2CAN CAN0 [CANH] ──────── [CANH] ODrive
T-2CAN CAN0 [CANL] ──────── [CANL] ODrive
                  └ 120Ω ┘        └ 120Ω ┘   (버스 양 끝 종단저항)
GND 공통 연결
```

## 단계별 계획

- [x] **Phase 1** · ODrive CAN 설정 — 변경 불필요 확인 (node_id=1, baud=500k, CANSimple 이미 설정됨. `probe_can.py`)
- [x] **Phase 2** · 배선 + 종단저항 120Ω + baud/크리스탈(16MHz) 일치 확인 (heartbeat 수신으로 검증)
- [x] **Phase 3a** · CAN 상태 모니터 (수신 전용, 모션 없음) — `main.cpp`.
      MCP2515 SPI OK + ODrive heartbeat(state=IDLE, err=0) + 엔코더(pos/vel) 수신 확인 (2026-05-29).
- [ ] **Phase 3b** · 부드러운 동작: `Set_Axis_State`(CLOSED_LOOP) → `Set_Controller_Mode`(VEL_RAMP)
      → 60Hz `Set_Input_Vel` 송신 (`swing_sine_vel.py` 의 `v = ω·amp·cos(ωt)` 이식). 매 프레임 = watchdog feed.
      ※ 송신 추가 → can_simple encode 필요 (heartbeat/encoder 처럼 little-endian 직접 패킹).
- [ ] **Phase 4** · 단일 모터 벤치 테스트 (CAN 60Hz 사인 + USB `read_encoders.py` 동시 관측)
- [ ] **Phase 5** · 듀얼 모터 (node_id 분리 또는 듀얼 CAN 채널, 동기 송신)

## 트러블슈팅 (실측 교훈)

- **MCP2515 SPI 무응답 (`reset` 실패)** → **하드웨어 RESET 핀(GPIO 9) 펄스 누락**이 원인.
  `SPI.begin` 전에 `pinMode(9,OUTPUT); HIGH→LOW→HIGH (각 100ms)` 필수. LilyGo `can.ino` 와 동일.
  (FD 변형에선 GPIO 9 = INT_0, MCP2515 변형에선 = RST)
- **sandeepmistry/arduino-CAN 은 ESP32-S3 빌드 실패** (`ESP32SJA1000.cpp` 가 구형 레지스터 참조).
  → autowp/arduino-mcp2515 사용 (LilyGo 예제와 동일, S3 호환).
- baud=500k / 크리스탈 16MHz 어긋나면 통신 전부 실패 — `setBitrate(CAN_500KBPS, MCP_16MHZ)`.
- **CAN `Get_Encoder_Estimates` 가 IDLE 에서 pos/vel=0** → ODrive 정상 동작. CAN 의 엔코더 추정값은
  컨트롤러의 추정 소스(`controller.pos_estimate_linear_src`)를 읽고, **CLOSED_LOOP 일 때만 갱신**됨.
  IDLE 에선 0. (USB 의 `encoder.pos_estimate` 와 소스 다름.) → 손으로 돌려도 0 인 게 정상.
  실시간 pos/vel 은 Phase 3b(CLOSED_LOOP 진입) 후 확인 가능.
  진단 확인: enc 프레임 카운트가 100Hz 로 증가 = 수신 정상, encRaw=00.. = 페이로드가 0.

## 참고 링크
- ODrive CAN Protocol (0.6.5): https://docs.odriverobotics.com/v/0.6.5/can-protocol.html
- ODrive Arduino CAN 가이드: https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html
- ODriveArduino (+SineWaveCAN 예제, 참고용): https://github.com/odriverobotics/ODriveArduino
- autowp/arduino-mcp2515 (실제 사용 CAN 드라이버): https://github.com/autowp/arduino-mcp2515
- T-2CAN repo (+examples/can/can.ino): https://github.com/Xinyuan-LilyGO/T-2Can
