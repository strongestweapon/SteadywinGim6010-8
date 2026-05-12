# Steadywin GIM6010-8 모터 제어 프로젝트

무대 공연용 마일라 필름 스윙 제어. 천장-바닥 사이에 매달린 5m × 1.2m 마일라 필름
하단의 모터+배터리 원통(진자)을 사인파로 왕복시키는 시스템.

- 모터: SteadyWin **GIM6010-8** (24V, 출력축 인코더 포함, 8:1 기어비)
- 컨트롤러: **하드웨어 v3.12-1V (단일축 보드)**, **펌웨어 v0.6.5**
- 호스트 도구: `odrive==0.6.5.post2` (Python 3.10)
- 인코더 구성: 1차(모터쪽 SPI MA732, mode=260) + 2차(출력축, I2C/UART) — 둘 다
  `odrv0.axis0.encoder` 에 통합. **별도 `axis1` 없음.**
- 연결: 현재 Windows + USB Type-C → 추후 macOS + CANable
- 모드: X 그네(동위상) / Z 비틀기(역위상) / X 고주파 진동(물결)
- 양 끝에 모터 2대 → 추후 ESPNow 동기화 (미구현)

## 1. 환경 세팅

### 1.1 Python 3.10
`odrive==0.6.5.post2` 는 **Python 3.10** 권장. winget 으로 설치 완료:
```powershell
winget install Python.Python.3.10
```

### 1.2 venv + 패키지
```powershell
cd C:\Users\songh\OneDrive\Desktop\Steadywin
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

### 1.3 PowerShell UTF-8 (한글 깨짐 방지)
스크립트 자체에 `sys.stdout.reconfigure(encoding="utf-8")` 가 들어가 있지만,
PowerShell 콘솔 자체의 코드페이지도 맞춰주면 더 깨끗:
```powershell
chcp 65001 > $null
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$env:PYTHONIOENCODING = "utf-8"
```
영구로 하려면 PowerShell `$PROFILE` 에 위 3줄 추가.

### 1.4 Zadig 으로 libusb-win32 드라이버 (이미 적용된 경우 생략)
이미 동작 중이면 건너뛰어도 됨. `python test_connect.py` 가 인식하면 OK.
새 보드 추가 시:
1. [Zadig](https://zadig.akeo.ie/) 실행 → Options → List All Devices
2. **ODrive Native Interface** (USB ID `1209 0D32`) 선택
3. 드라이버를 **libusb-win32** 로 바꾸고 Replace Driver
4. `python test_connect.py` 확인

## 2. 캘리브레이션

이 GIM6010-8 은 **SteadyWin 이 공장에서 미리 캘리브해서 출하** 합니다
(`motor.config.pre_calibrated = True`, `encoder.config.pre_calibrated = True`).
모터 캘리브 / 인코더 offset 캘리브를 **다시 돌리면 안 됨** — 잘못하면 출하 값이 덮어
씌워짐. 우리가 할 일은 **사용자 기구부 0점만 잡는 것**.

### 2.1 user zero 설정 — `set_zero.py`

원리 (SteadyWin 매뉴얼 P43 권장):
```
odrv0.axis0.encoder.config.index_offset = odrv0.axis0.encoder.pos_estimate
odrv0.save_configuration()
```
1. 진자를 손으로(또는 위치제어로) **기구부 0점** 위치에 정렬
2. `python set_zero.py` 실행 → 확인 프롬프트 → `y`
3. 보드가 자동 재부팅
4. 이후 부팅마다 `pos_estimate` 가 0 근처에서 시작

옵션:
```powershell
python set_zero.py --yes     # 확인 생략
python set_zero.py --undo    # index_offset = 0 으로 되돌림
```

**다시 캘리브 필요한 경우**: 타이밍벨트 풀고 재조립, 펌웨어 업데이트,
`erase_configuration()` 후, 모터 교체.

### 2.2 부팅 분기 토글 (snap-to-zero 자동 보정)

이 보드의 **알려진 한계**: 1차 인코더(MA600) 가 mono-turn 절대 인코더라서
모터-출력축 8:1 기어 환경에서 부팅 시 multi-turn 인덱스 결정이 두 분기를
토글한다. ODrive 공식 문서도 *"absolute encoder + multi-turn axis 사용 시
reference frame 이 정수 turn 단위로 shift 가능"* 이라고 인정한 동작.

실측 결과:
| 부팅 회차 | pos_estimate | 차이 |
|---|---|---|
| 1 | -0.142515 | — |
| 2 | -0.000089 | +0.142426 |
| 3 | -0.142610 | -0.142521 |

→ 정확히 두 값 사이 토글. 폭 = **모터축 51.3° / 출력축 6.4°**.

**우리 대응**: `motor_helpers.enter_position_mode(snap_to_zero=True)` (기본)
가 부팅 직후 `pos_estimate` 가 어떤 분기에 lock 됐는지 자동 감지하고 가장
가까운 격자(`TOGGLE_TURN` 정수배) 를 origin 으로 잡는다. **결과: 매 부팅
마다 swing 의 진폭 범위가 일관됨.**

상수는 `motor_helpers.py` 상단:
```python
TOGGLE_TURN = 0.142469      # 측정값. 다른 모터에선 다를 수 있음.
SNAP_TOLERANCE = 0.020      # 격자에서 이만큼 이내면 snap.
```
다른 GIM6010-8 모터로 옮기면 `probe_reboot_stability.py` 로 토글 폭을 다시
측정하고 `TOGGLE_TURN` 을 갱신.

비활성화 옵션: `--no-snap` (디버깅용. 운영에선 거의 안 씀).

## 3. 사용 스크립트

| 스크립트 | 용도 |
|---|---|
| `test_connect.py` | USB 연결 + 펌웨어/시리얼/캘리브 상태 확인 |
| `read_encoders.py` | axis0 위치/속도/setpoint 실시간 출력 |
| `set_zero.py` | user zero (index_offset) 영구 저장 |
| `swing_sine.py` | `--amp ° --freq Hz --duration s` 사인파 (부드러운 ramp) |
| `swing_modes.py` | 그네→비틀기→정지→물결 시퀀스 |
| `motor_helpers.py` | 공통 함수 (연결, 모드 진입, safe_stop, envelope) |

부가 (1회성 / 디버깅):
| | |
|---|---|
| `probe_tree.py` | ODrive 객체 트리 전체 dump |
| `probe_sec_enc.py` | 2차 인코더 통신 검증 |
| `probe_reboot_stability.py` | 부팅 토글 폭 측정 (TOGGLE_TURN 갱신용) |

### 3.1 부드러운 램핑 정책 (모든 동작 스크립트 공통)

전류 갑작스러운 소모를 방지하기 위해 2단계로 입력을 부드럽게:

1. **ODrive 내장 입력 필터**
   - 위치 제어: `INPUT_MODE_POS_FILTER` + `input_filter_bandwidth = 8 Hz` (기본)
   - 속도 제어: `INPUT_MODE_VEL_RAMP` + `vel_ramp_rate = 5 turn/s²` (기본)
   - 토크 제어: `INPUT_MODE_TORQUE_RAMP` + `torque_ramp_rate = 0.5 N·m/s` (기본)
   → input 값의 급변을 펌웨어가 자체적으로 lowpass / ramp 처리

2. **외부 envelope** (사인파 스크립트)
   - 시작 `ramp` 초 동안 진폭 0 → amp 로 cosine fade-in
   - 마지막 `ramp` 초 동안 amp → 0 으로 cosine fade-out
   - 즉 사인파가 사일런트하게 시작/종료 → 진자가 갑자기 튀지 않음

3. **안전 한계** (`motor_helpers.apply_safety`)
   - 전류 한계: `--current-lim` (기본 10A, 무대용 보수값)
   - 속도 한계: `--vel-limit` (기본 20 turn/s)
   - 위치 한계: `--limit ° ` 초과 시 즉시 safe_stop

4. **safe_stop**
   - 모드별로 input 을 0 까지 짧게 ramp → IDLE 진입
   - Ctrl+C / 한계 초과 / 인코더 비정상 시 모두 호출
   - 종료 직전 갑작스러운 토크 변화 방지

### 3.2 예시

```powershell
python test_connect.py
python read_encoders.py --hz 50

# 작은 진폭부터 점진 테스트
python swing_sine.py --amp 5 --freq 1.0 --duration 10 --ramp 1.5
python swing_sine.py --amp 15 --freq 1.0 --duration 30 --ramp 1.5
python swing_sine.py --amp 30 --freq 1.0 --duration 60 --ramp 2.0 --current-lim 15

# 모드 데모
python swing_modes.py --limit 45 --current-lim 10
```

## 4. 공진 주파수 측정

마일라 필름 + 진자 시스템의 자연 진동 주파수를 찾아 사인 가진 주파수의 기준으로 삼음.

### 4.1 자유 진동 측정 (가장 빠름)
1. 모터 IDLE 상태 (`python test_connect.py` 후 그대로 두면 IDLE).
2. `python read_encoders.py --hz 100` 띄움 (터미널 캡쳐 또는 화면 영상 녹화).
3. 진자를 10–15° 한쪽으로 당겼다 놓음.
4. `pos_estimate` 의 연속 zero-crossing 사이 시간(주기 T) 측정 → 공진주파수 = 1/T.
5. 진폭이 큰 첫 몇 사이클은 비선형성/마찰로 약간 짧을 수 있으므로 **감쇠 후** 측정.

### 4.2 사인 스윕 (확인용)
```powershell
python swing_sine.py --amp 5 --freq 0.6 --duration 15 --ramp 1.0
python swing_sine.py --amp 5 --freq 0.8 --duration 15 --ramp 1.0
python swing_sine.py --amp 5 --freq 1.0 --duration 15 --ramp 1.0
python swing_sine.py --amp 5 --freq 1.2 --duration 15 --ramp 1.0
```
가장 큰 실측 진폭이 나오는 주파수가 공진점.

### 4.3 운용 가이드
- 실제 공연 가진은 **공진점 ±5% 이내**.
- 정확히 공진점에 두면 작은 외란으로 진폭이 폭주 가능 → `--limit` 반드시 설정.

## 5. macOS + CANable 전환 시

미리 Windows 단계에서:
```python
# odrivetool shell
odrv0.axis0.config.can.node_id = 0x10           # 두 번째 보드는 0x20
odrv0.axis0.config.can.heartbeat_rate_ms = 100
odrv0.can.config.baud_rate = 250000             # 또는 500000
odrv0.save_configuration()
```

코드 변경 포인트:
- `motor_helpers.connect()` 의 `odrive.find_any()` → `can.Bus(...)` 어댑터로 교체
- 위치/상태 호출은 ODrive CAN Simple Protocol 프레임으로 송수신
- ESP32 가 양쪽 ODrive 에 같은 t=0 기준 사인파 명령을 송신
  - 그네: φ₂ = φ₁ (동위상)
  - 비틀기: φ₂ = φ₁ + π (역위상)
  - 물결: 같은 freq, 위상 offset(예 π/4) 으로 진행파 생성

## 6. 안전 체크리스트

- [ ] 첫 가동: `--amp 3 --freq 0.5 --duration 5` 같은 **최소 진폭**부터.
- [ ] 진폭 늘리기 전 `--limit` 명시.
- [ ] `--current-lim` 은 무대 진자 무게/길이에 맞게 조정 (기본 10A 는 가벼운 진자 기준).
- [ ] 캘리브 미완료 (`is_calibrated`/`is_ready` False) 면 스크립트가 자체 거부 — 해제 후 수동 캘리브 금지.
- [ ] 위급 시 USB 빼지 말고 **24V 전원 차단** (USB 만 빼면 토크 유지 가능).
- [ ] 양쪽 모터 동시 운영 전 단일 모터로 모든 모드 검증.

## 7. 보드 상태 메모

`probe_tree.py` 로 dump 한 현 상태 요약 (참고용, 변할 수 있음):
- 시리얼: `89340A6C3037`
- 펌웨어: v0.6.5, 하드웨어 v3.12-1V
- 1차 인코더 mode=260 (SPI Abs MA732), cpr=16384, `pre_calibrated=True`
- 2차 인코더 sec_enc_cpr=16384, `poll_sec_enc() → True` (통신 OK)
- motor `pole_pairs=14`, `torque_constant=0.097`, `current_lim=60A`(공장 기본)
- gear_ratio=8.0 (8:1)
- `misconfigured = True` ← 원인 미상, 동작에는 지장 없음. 추후 dump_errors 추적.
