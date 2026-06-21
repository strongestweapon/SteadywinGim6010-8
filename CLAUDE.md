# Steadywin GIM6010-8 무대 공연용 마일라 필름 스윙 제어

**GitHub**: https://github.com/strongestweapon/SteadywinGim6010-8

## 2026-06-21 세션 요약 (컨트롤러 앱 공연모드 + OSC — 전부 앱만, 펌웨어/모터 변경 0)

브랜치 **`work-2026-06-21`** (체크포인트 1~19, origin 푸시). `main`=어제(bfb24d0) 안전점 유지(머지 안 함). **모터 실연결 못 해 헤드리스 검증만 — 실모터 검증 미완.**

- **공연(씬) 모드** (`controller_app/main.py`, 화면 아래): 씬 = 칸별 인라인(편집콤보 없음). 각 칸 = [적용버튼(이름+A/B 요약) + 이름 + A줄(☑A·freq·amp·Twist) + B줄 + Δφ + Capture/Delete]. **+Add scene/✕Delete 동적 + 가로스크롤**, 전환시간(크로스페이드), Save/Load(`shows/show.json`, gitignore).
  - **부드러운 전환**: 슬라이더를 목표로 ramp(위상 연속). fade-in 시 freq 즉시·진폭만 페이드(A·B 위상 안 어긋나게).
  - **A↔B 위상차 = 앱-side 스태거드 스타트** (펌웨어 재플래시 불필요! `start_running(0.0f)` 이용, B를 A위상모델 목표각 도달 시 START). 검증 180/90/0→180/91/0°.
  - **극성 = "Twist"(두 모터 반대방향) 토글 1개** (모터1/2 2체크 아님). 안 누름=평행(POL_M2), 누름=반대.
  - **진폭 = 각도(°)** (강도% 아님). 0~60°, **주파수별 자동 클램프**(출력=min(°, amp_deg_max(freq))+ESP32 이중). 씬 진폭칸 타이핑 자유(80 쳐도 됨)→확정 시 freq한계로 줄임.
  - Run/Stop 라벨, 패널 제목에 현재 씬 표시(수동조작 시 *edited).
- **통신두절 빨간 피드백**(시스템별 never/stale), **실측 drop%**(텔레메트리 seq간격), **글씨 1.5배**, **다크모드**(Fusion 팔레트+pyqtgraph), **UI 전부 영어**(코드 주석만 한글), **수동 스윙·LINK 제거**.
- **OSC 수신(receive-only)**: python-osc(.venv 설치). 주소 **`/mirror/scene/<N>`**(1-based, 인자없음) 또는 `/mirror/scene <num>`(int/float/string). "OSC in" 토글+port 9000. 별도 스레드→Qt Signal(스레드 안전). 송신/learn/MIDI 추후.
- 상세: 메모리 `project_show_mode`, `project_app_conn_feedback`.
- **다음(모터 연결 시)**: 실배선 2대 검증(부호/위상차/통신두절 실동작) + 진폭 각도 감각 + 둘째 ESP32 플래시·IP(B=192.168.0.47).

## 하드웨어 / 펌웨어 상태 (2026-05-12 기준)

- **모터**: SteadyWin GIM6010-8 (24V, 출력축 인코더 포함, 8:1 기어비, 14 pole-pair, 5N·m peak)
- **드라이버**: ODrive 3.6 클론, **하드웨어 v3.12-1V (단일축 보드)**
- **펌웨어**: **v0.6.5** (SteadyWin 공장 캘리브 + 굽힘. 절대 재캘리브 금지)
- **호스트 도구**: `odrive==0.6.5.post2` (Python 3.10)
- **인코더**: 1차 = MA600 SPI 절대 **mono-turn** (모터축, mode=260), 2차 = 출력축 I2C/UART, 둘 다 `axis0.encoder` 에 통합. **별도 `axis1` 없음.**
- **연결**: 현재 Windows + USB Type-C → 추후 macOS + CANable (이행 시 `motor_helpers.connect` 만 어댑터 교체)
- **VBus**: 약 19.97~20V (24V 시스템 기준 살짝 낮음 — 큰 전류 인입 시 언더볼티지 위험)

## 작업 규칙 (어기지 말 것)

1. **보드에 명령 가하기 전 매번 사용자 승인 받기**. `python -c "import odrive..."` 같은 inline 도 포함. (`feedback_user_authorization.md` 참고)
2. **게인/bandwidth/한계 변경 시 매뉴얼 + 커뮤니티 검증값 먼저 검색**. 추측 step 금지. (`feedback_search_before_guess.md`)
3. **`save_configuration()` 은 영구 변경** — `set_zero.py` 외엔 호출 금지. SteadyWin 공장 캘리브 값을 절대 덮어쓰지 않음.
4. **`erase_configuration()` / `clear_errors()` 자동 호출 금지** — 사용자 명시 승인 필요.
5. **모터 캘리브 (`AXIS_STATE_MOTOR_CALIBRATION` 등) 절대 실행 금지** — 공장값 손상 위험.
6. **한글 주석** 사용.

## 현재 보드 영구값 (probe_tree dump 기준, 변경 시 위 규칙 준수)

2026-05-12 P29-30 절차로 튜닝 + `save_configuration()` 영구화 완료. 공장값에서 차이 나는 부분은 비고에 표시.

| 파라미터 | 값 | 공장값 | 비고 |
|---|---|---|---|
| `motor.config.current_lim` | **10 A** | 60.0 | 무대 보수값 |
| `motor.config.pole_pairs` | 14 | 14 | 변경 절대 금지 |
| `motor.config.gear_ratio` | 8.0 | 8.0 | 8:1 기어 |
| `motor.config.torque_constant` | 0.097 N·m/A | 0.097 | |
| `motor.config.phase_offset` | 22076 | 22076 | **공장 cal — 절대 보호** |
| `controller.config.vel_limit` | **5.0 turn/s** | 30.0 | 무대 보수값 |
| `controller.config.pos_gain` | **50** | 20.0 | P29 step 4 튜닝 결과 |
| `controller.config.vel_gain` | **0.145** | 0.10 | P29 step 3 튜닝 결과 |
| `controller.config.vel_integrator_gain` | 1.0 | 1.0 | P29 step 5 — 공장값이 최적 |
| `controller.config.input_filter_bandwidth` | 10.0 | 10.0 | POS_FILTER 모드 한정 (실행 시 100Hz 로 override) |
| `encoder.config.bandwidth` | 500 Hz | 500 | **변경 시 PID 발산 위험** |
| `encoder.config.cpr` | 16384 | 16384 | mono-turn |
| `encoder.config.mode` | 260 | 260 | SPI Abs MA600 |
| `trap_traj.config.vel_limit` | 100 turn/s | 100 | 운용 파라미터, 변경 안전 |
| `trap_traj.config.accel_limit` | 20 turn/s² | 20 | 운용 파라미터, 변경 안전 |
| `trap_traj.config.decel_limit` | 20 turn/s² | 20 | 운용 파라미터, 변경 안전 |
| `axis0.encoder.config.index_offset` | -2.142441 | 0 | `set_zero.py --yes` 로 굳힘 (모터/기구부 개체차) |

## 모터 개체별 특성 (시리얼 기준, 2026-05-29 기록)

여러 GIM6010-8 모터를 같은 응용에 쓰면서 측정한 개체별 값. **`phase_offset` 은 모터마다 다른 게 정상** (공장이 각 자석/권선 특성 측정해 저장 — 절대 복제/덮어쓰기 금지, `pre_calibrated=True` 면 재캘리브 불필요). **최적 `vel_gain` 도 개체마다 다름이 실측 확인됨** → 1번 값을 다른 모터에 그대로 복제하면 안 됨.

| 모터 | 시리얼 | `phase_offset`(공장) | 공장 출하 게인 | 부드러움 (무부하 실측) | `index_offset` | `TOGGLE_TURN` | 영구저장 상태 |
|---|---|---|---|---|---|---|---|
| **1번** | `89340A6C3037` | 22076 | vel_gain 0.10 / vel_int 1.0 / curr_lim 60 | 0.145 가 0.10보다 부드러움 (튜닝 채택 0.145) | -2.142441 | 0.142469 | ✅ 게인/한계/0점 전부 `save_configuration` 완료 (pos=50, vel_gain=0.145, vel_limit=5, curr=10) |
| **2번** | `C4610A6C3037` | 19916 | vel_gain 0.10 / vel_int 1.0 / curr_lim 60 | 0.10 이 부드럽고 **0.145 는 더 떨림 (1번과 반대)** | 0 (미설정) | 미측정 | ⚠️ 게인/0점은 공장 default. **단 `can.node_id=2` + `can.config.enable_r120=False`(종단 OFF) 영구 저장됨 (2026-06-04, `set_node_id.py`/`set_can_term.py`)** — 듀얼 버스 가운데 노드용 |
| **3번** | `7A360A6C3037` | 13017 | **vel_gain 0.05 / vel_int 0.20 / curr_lim 45** | 4번(`7B600A6C3037`)보다 **조금 더 떨림** (같은 게인인데도) | 0 (미설정) | 미측정 | ⚠️ **System B node 1** (node_id=1 공장값 유지, 종단 r120=ON 공장값 유지 — 버스 끝 종단). 게인/0점은 공장 default |
| **4번** | `7B600A6C3037` | 21012 | **vel_gain 0.05 / vel_int 0.20 / curr_lim 45** | **무부하에서 가장 부드러움 (공장 0.05 그대로)** | 0 (미설정) | 미측정 | ⚠️ **`can.node_id=2` + `enable_r120=False`(종단 OFF) 영구 저장 (2026-06-20, `set_node_id.py`/`set_can_term.py`)** — System B 듀얼버스 node 2 용. 게인/0점은 공장 default |
| **5번** | `8F350A6C3037` | 10439 | **vel_gain 0.05 / vel_int 0.20 / curr_lim 45** (공장) | **4번(`7B600A6C3037`)과 비슷하게 부드러움 (4·5번이 베스트)** | 0 (미설정) | 미측정 | ❌ 전부 공장 default. 튜닝/0점/저장 안 함 |

**핵심 교훈 (2026-05-29 발견):**
1. **공장 출하 게인·전류한계가 개체마다 다르다.** 1·2번은 `vel_gain 0.10 / vel_int 1.0 / curr_lim 60`, 3·4번은 `0.05 / 0.20 / 45` (넷 다 fw v0.6.5). 공장 게인이 두 그룹으로 갈림. SteadyWin 이 배치/개체별로 다른 값을 굽는 듯.
2. **최적 `vel_gain` 이 모터마다 다르다.** 1번은 0.145 가 부드러웠는데 2번은 0.145 가 오히려 더 떨림, 3번은 공장 0.05 가 가장 부드러움 (사용자 실측). → **"옵션 B (1번 값 복제)" 위험. "옵션 A (개별 P29)" 또는 최소한 vel_gain 개체별 비교 후 채택** 권장. `apply_tuning.py` 는 무조건 복제 말고 "시작점 제시 + 개체 확인" 설계.
3. **패턴: 낮은 `vel_gain` 일수록 무부하에서 부드럽다** (0.05 > 0.10 > 0.145 순으로 부드러움). cogging 토크 ripple 보정이 약해지기 때문 — POS vs VEL 부드러움 논리와 동일. **단 무부하 책상 기준**이고, 게인이 너무 낮으면 진자 부하 시 추종이 처질 수 있음 → 진자 부착 후 재확인 필요.
4. **같은 게인이어도 기계적 개체차로 부드러움이 다르다.** 3·4번 둘 다 공장 0.05 인데 4번이 3번보다 조금 더 떨림 (사용자 실측). 게인으로 다 설명 안 되는 순수 기계차(베어링/기어 맞물림/코깅 위상). → 부드러움 최우선이면 **여러 개체 중 실제로 가장 부드러운 것을 골라 쓰는** 선별도 유효.

## 새 모터 세팅 가이드 (다른 GIM6010-8 보드에 같은 튜닝 적용)

같은 응용(마일라 swing) + 같은 모터 모델이라 게인 값을 그대로 시작점으로 적용 가능. 단 **`index_offset` 은 모터/기구부 개체차** 이므로 반드시 재측정.

**순서:**

1. **연결 확인**: `python test_connect.py`
2. **공장 default 확인**: `python probe_tree.py > tree_new.txt` → `pole_pairs=14`, `gear_ratio=8.0`, `phase_offset` 존재, `pre_calibrated=True` 검증
3. **0점 세팅**: 기구부를 원하는 0° 위치로 두고 `python set_zero.py --yes` → 이 모터의 `index_offset` 영구 저장
4. **TOGGLE_TURN 재측정** (다른 모터는 다를 수 있음): `python probe_reboot_stability.py` 여러 회 → `motor_helpers.TOGGLE_TURN` 상수 업데이트
5. **튜닝값 적용 + 저장**: `python apply_tuning.py` (별도 작성 예정 — 위 표의 값을 RAM 에 set + save_configuration)
6. **검증**: `python swing_trap.py --amp 240 --cycles 3 --end-center` → 부드러우면 OK. 거치면 P29-30 절차 재실행.

**개체차로 재튜닝이 필요할 수 있는 경우:**
- 모터 발열/저항 차이로 vel_gain 발산점이 다를 수 있음 → P29 step 3 재실행 권장
- 기구부 마찰/관성 차이로 pos_gain overshoot 임계점이 다를 수 있음 → P29 step 4 재실행 권장
- 의심스러우면 `tune_vel_gain.py` / `tune_pos_gain.py` 로 전체 절차 다시 진행

## 튜닝 이력 (2026-05-12 완료, 영구 저장됨)

매뉴얼 P29-30 PID 절차 적용. 결과는 위 "현재 보드 영구값" 표에 반영. 측정 데이터는 다른 모터 튜닝 시 비교 기준 + 같은 절차 재현 가이드 용도.

**P29 step 3 (vel_gain 스윕, target=1.0 turn/s, vel_integrator=0):**
- 0.07 → CoV 30.9%, pk-pk 0.96
- 0.10 → CoV 20.5%, pk-pk 0.74 (공장값)
- 0.13 → CoV 16.1%, pk-pk 0.55
- 0.17 → CoV 12.5%, pk-pk 0.49 (최소 CoV)
- 0.22 → CoV 12.2%, pk-pk 0.64 (pk-pk 상승)
- **0.29 → CoV 173%, pk-pk 8.79 (🚨 발산)**
- → **0.145 채택** (= 0.29 × 0.5, 매뉴얼 rule)

**P29 step 4 (pos_gain 스윕, step=0.05turn, vel_gain=0.145, vel_integrator=0):**
- 20 (공장) → overshoot 9.0%, 교차 1
- 26 → 4.9%, 1
- 34 → 0%, 0 (stiction undershoot)
- 44 → 5.4%, 1
- 50 → 9.7%, 1
- 57 → 9.8%, 1 (경계)
- **65 → 3.7%, 교차 6 (🚨 oscillation)**
- **74 → 7.2%, 교차 4 (🚨 oscillation)**
- → **50 채택** (oscillation 시작점 65 대비 23% 마진)

**P29 step 5 (vel_integrator, pos_gain=50):**
- 1.45 (공식 `0.5×20×0.145`): ss 0.26°, peak-ss 0.79°
- **1.0 (공장값): ss 0.03°, peak-ss 0.55° ← 채택** (실제 ringing 더 작음)

**검증**: `swing_trap.py --amp 240 --cycles 3` 에서 사용자 평가 "어제(공장값) 대비 훨씬 좋다" → save_configuration() 영구화.

## 알려진 한계 / 우회

### Mono-turn 인코더 + multi-turn 축 토글 (모터축 51.3° / 출력축 6.4°)
- **원인**: MA600 이 모터 1 turn 안 절대값만 알고, 2차 인코더(출력축)로 multi-turn 인덱스 복원. 부팅 시 두 분기 사이 결정론적 토글.
- **수치**: `TOGGLE_TURN = 0.142469` (probe_reboot_stability.py 측정).
- **우회**: `motor_helpers.enter_position_mode(snap_to_zero=True)` (기본) 가 부팅 직후 격자에 snap → swing center 일관.
- **다른 모터로 옮기면** TOGGLE_TURN 재측정 필요.

### 진동 / 덜그럭 분석 (2026-05-12)
- **기계적 (cogging, 백래시)** ≠ **전자적 (PID ripple, PWM whine)**: 사용자가 두 다른 감각으로 인지.
- TRAP_TRAJ 모드가 사인 streaming 보다 매끄럽지만, 끝점에서 vel=0 정지 후 가속 → 방향 전환 충격.
- 무부하에서 두드러짐. **진자 부하로 자연 해소 가능성 큼** — 운용 환경 검증 필요.
- P29-30 튜닝 (vel_gain 0.10→0.145, pos_gain 20→50) 후 사용자 평가 "훨씬 부드러움".
- **모드별 부드러움 차이 (2026-05-12 발견)**: VEL_RAMP 가 POS_FILTER 보다 압도적으로 부드러움. POS+ff 도 `pos_gain=5` 이하로 낮추면 VEL 근사. 자세한 분석은 아래 "사인 swing 모드별 부드러움 순위" 섹션 참고.

### Anti-cogging cal 시도 결과 (2026-05-12)
- ODrive 0.6.5 의 `start_anticogging_calibration()` 시도. 초기에는 모터가 +157 turn 위치에 있어서 cal 의 절대 0~1 turn setpoint 추적으로 31s 슬루 필요 → 진행 안 됨.
- 모터를 0 으로 먼저 이동 후 cal 재시도 → 시간당 ~6 indices 진행 (50분 ETA, 일반적인 5-6분 대비 매우 느림). 부분 cal 후 사용자 중단.
- ODrive 개발자 본인이 "current version of anticogging calibration kinda sucks" 라고 공언한 기능. **유성기어 모터에서는 BLDC cogging 만 보상 가능 (기어 cogging 못 잡음) → 효과 제한적.**
- 결론: **anti-cog 우회. VEL_RAMP 모드 사용 + 진자 부하의 자연 댐핑으로 cogging 마스킹** 이 더 실용적.

### 저속 cogging 인식의 본질 (2026-05-12 검증)
- 1방향 회전 (`rotate_one_way.py`) 으로 검증: 저속 (0.5 turn/s) 거침, 고속 (7 turn/s) 부드러움. 즉 cogging 자체 문제 아닌 **속도-가시성** 문제.
- 사인 swing 은 endpoint 에서 vel→0 거치므로 cogging 항상 노출됨.
- 진자 inertia + 댐핑이 노출 시간을 평활할 것 — 부하 시 자연 개선 기대.

### 고주파 사인 (4Hz) 이슈 (2026-05-13 시연 리허설, 미해결)
- **4Hz VEL_RAMP drift 빠름**: `swing_sine_vel.py --amp 60 --freq 4.0` 가 ~3초만에 drift 안전정지 (drift > amp×2). v_act 가 v_cmd 보다 계속 큼 → 한 방향 누적. 1Hz 는 cycle 길어 drift 작지만 4Hz 는 cycle 8배라 누적 빠름.
- **4Hz 가 1Hz 보다 본질적으로 거침**:
  1. 방향 전환 빈도 — 1Hz 2회/초 vs 4Hz 8회/초 zero-velocity 통과 → cogging 4배 자주 노출
  2. `vel_integrator_gain=1.0` 이 4Hz 반주기 125ms 안에 못 안정 → 적분항 출렁
  3. feedforward 만으로 부족 → 컨트롤러 보정 토크 = 덜그럭
- **해결 후보 (미검증)**:
  - 4Hz 는 `swing_sine_pos.py` (POS+ff, soft pos_gain) 사용 → drift 없음. 단 cogging 노출 빈도는 모드 무관이라 거친 느낌은 비슷할 것
  - amp 더 축소 (±60°→±30° 모터) → peak vel 낮아져 cogging 영향 ↓
  - 진자 부하의 inertia 가 zero-crossing cogging 평활 기대 — 실부하 검증 필요
- **시연 리허설 결과**: 1Hz (`swing_sine_vel.py --amp 240 --freq 1.0 --vel-ramp 100`) 깨끗. 4Hz 는 위 이슈로 추가 작업 필요.

## 모드 선택 가이드

| 응용 | 모드 | 이유 |
|---|---|---|
| 사인파 (연속 곡선) | FPC (`INPUT_MODE_POS_FILTER`) | 호스트가 위치 곡선 생성, filter 가 명령 점프 부드럽게. 매뉴얼 P32 권장. **`input_filter_bandwidth = 명령주파수/2`** (200Hz 명령 → 100Hz). |
| 끝점 사이 부드러운 이동 | TRAP_TRAJ (`INPUT_MODE_TRAP_TRAJ`) | 펌웨어 내부 사다리꼴 trajectory. 호스트는 끝 위치만 명령. `vel_limit / accel_limit / decel_limit` 으로 부드러움 조절. |
| 속도제어 | VEL_RAMP | `vel_ramp_rate` 로 가속 제한 |
| 토크제어 | TORQUE_RAMP | `torque_ramp_rate` 로 토크 변화율 제한 |

**중요**: input_mode 만 바꾸는 것은 안전. 게인 변경은 매뉴얼 P29-30 절차 따라.

## 사인 swing 모드별 부드러움 순위 (2026-05-12 무부하 실측)

진자 매달기 전 무부하 책상 테스트 기준. **저속 cogging 토크 ripple 이 모든 모드의 공통 문제**. 모드는 그 ripple 을 컨트롤러가 얼마나 강하게 보정하느냐가 부드러움 결정.

| 순위 | 모드 | 부드러움 | 위치 정확도 | 비고 |
|---|---|---|---|---|
| 🥇 (이론) | `TORQUE_RAMP` | 최고 — pos/vel loop 없음 | ❌ 자유 drift | **무부하에선 무용** (위치 발산). 진자 매달면 중력 복원력 + 토크 명령으로 가능. |
| 🥈 (실측) | `VEL_RAMP` | 매우 부드러움 | ⚠️ 적분 drift | `swing_sine_vel.py`. cmd 는 v(t)=ω·amp·cos(ωt). pos 는 적분 결과로 자연 형성. **가장 부드러운 실용 옵션**. |
| 🥉 | POS + vel ff (PASSTHROUGH, **soft gain**) | 부드러움 | ✓ 정확 | `swing_sine_pos.py`. 호스트가 pos AND vel 둘 다 명령. `pos_gain` 5 이하로 낮춰야 효과. **튜닝값 pos_gain=50 그대로 쓰면 거침** — vel ff 추가 효과가 stiff pos loop 에 묻힘. |
| 4 | POS_FILTER (input_pos만) | 거침 | ✓ 정확 | `swing_sine.py`. 호스트는 pos 만 명령, 필터가 vel 자체 추정. cogging 마다 pos error → 강한 보정 토크 → 덜그럭. |

**왜 POS 보다 VEL 이 부드러운가:**
- POS 컨트롤러는 pos error → 토크. cogging 으로 0.001 turn pos error 생기면 `pos_gain × err` 만큼 vel cmd 추가 → 토크 펄스 → 덜그럭.
- VEL 컨트롤러는 vel error 만 봄. cogging 으로 vel ripple 발생해도 평균만 맞춤 → 토크 매끈.

**오버커런트 방지 (vel_ramp_rate vs current_lim):**
- 사인 peak 가속도 = ω² × amp
- 모터 α_max = current_lim × torque_constant / J_effective ≈ 308 turn/s² (current_lim=10A, J≈0.0005 kg·m²)
- `vel_ramp_rate ≤ α_max` 으로 두면 컨트롤러가 한계 초과 명령 안 만듦 → 오버커런트 발생 불가
- 주파수가 높아질수록 amp 가 자연 제한 (amp_max = α_max / ω²)
  - 1 Hz → amp_max 7.8 turn 모터 (충분히 큼)
  - 2 Hz → 1.95 turn
  - 3 Hz → 0.87 turn
  - 4 Hz → 0.49 turn ≈ 모터 175° = 출력축 22°
  - 5 Hz → 0.31 turn ≈ 모터 112° = 출력축 14°

**스크립트 사용 가이드 (실측 기준):**
- 부드러움 우선 + 정확한 위치 필요 없음 → `swing_sine_vel.py`
- 부드러움 + 정확한 위치 필요 → `swing_sine_pos.py` (단 `pos_gain` 을 RAM 에서 5 이하로 낮추고 실행)
- 끝점 사이 이동만 (사인 아님) → `swing_trap.py`
- 진자 매달린 후 토크만으로 흔들기 → torque ramp 모드 (향후 작업)

## CAN / 무선 (ESP-NOW) 환경 권장 구성

USB 200Hz 환경에서 검증된 부드러움을 CAN/무선 환경 (60Hz) 으로 이행할 때.

**기본 설정:**
- **업데이트 주기**: 60 Hz (16.7ms 간격)
  - 1Hz 사인 → 60 sample/cycle ✓✓
  - 2Hz 사인 → 30 sample/cycle ✓
  - 4Hz 사인 → 15 sample/cycle ✓ (단계 자취 미세하게 보일 수도)
- **모드**: `VEL_RAMP` 가 가장 부드러움 + 60Hz 단계 자연 평활
- **명령**: `input_vel = ω × amp × cos(ωt)` 매 frame
- **보드 설정**:
  - `controller.config.vel_ramp_rate = 100 turn/s²` (가속 한계, 토크 폭주 방지)
  - `controller.config.vel_limit = 12 turn/s` (peak vel + 마진)
  - `motor.config.current_lim = 10 A` (현재 영구값 그대로)

**통신 두절 대비 (필수):**
- `axis.config.watchdog_timeout = 0.1` (100ms — 6 frame 누락 시 트리거)
- ODrive 가 watchdog timeout 발생 시 자동으로 IDLE 상태 진입 → 모터 disarm → 안전하게 코스트
- 매 CAN frame 수신이 자동 watchdog feed (별도 처리 불필요)
- ⚠️ **현재 영구값은 `watchdog_timeout = 0` (비활성)** — CAN 이행 시 반드시 설정 후 save_configuration

**ESP-NOW 듀얼 모터 동기화 (미래 작업):**
- 두 모터 모두 같은 watchdog 설정 필요 — 한쪽만 멈추면 비대칭 동작
- 라우터/송신기 장애 시 두 모터 동시 IDLE → 진자 자연 감쇠로 안전 정지
- 패킷 손실 < 1% 가정 시 60Hz 면 100ms watchdog 거의 트리거 안 됨
- 송신측에 시계 동기 (e.g. ESP32 RTC 동기) 필요 — 두 모터 위상 정확히 맞추려면

**모드 전환 시 주의:**
- 현재 영구값 (`pos_gain=50, vel_gain=0.145`) 은 POS 제어 가정 튜닝
- VEL 모드 사용 시 `pos_gain` 은 무시되니 신경 안 써도 됨
- POS+ff 사용 시 `pos_gain` 을 5 정도로 낮춰야 부드러움 (RAM, 운용 시점 적용)

## 스크립트 사용 흐름

⚠️ **odrive 모듈은 시스템 `python`(Python310)이 아니라 `.venv` 에 있음** → ODrive 스크립트는 반드시 `.venv\Scripts\python.exe <script>.py` 로 실행. (그냥 `python` 쓰면 `ModuleNotFoundError: odrive`.)

1. `python test_connect.py` — 연결 + 보드 상태 확인
2. `python read_encoders.py --hz 50` — 인코더 실시간 모니터
3. `python set_zero.py` — user zero 영구 저장 (사용자 명시 시에만)
4. `python tune_vel_gain.py --vel-gain <V>` — P29 step 3 한 step 테스트 (튜닝용)
5. `python tune_pos_gain.py --pos-gain <P>` — P29 step 4 한 step 테스트 (튜닝용)
6. `python rotate_one_way.py --output-turns 10 --motor-vel 3` — 1방향 회전 (기어/cogging 진단)
7. `python swing_trap.py --amp 240 --cycles 3 --end-center` — TRAP_TRAJ 왕복
8. `python swing_sine.py --amp 30 --freq 1.0 --duration 60 --filter-hz 100` — 사인파 (POS_FILTER, 거침)
9. `python swing_sine_vel.py --amp 240 --freq 1.0 --duration 20 --vel-ramp 100` — **사인파 (VEL_RAMP, 부드러움)**
10. `python swing_sine_pos.py --amp 240 --freq 1.0 --duration 20` — 사인파 (POS+vel ff, pos_gain=5 권장)
11. `python swing_sweep.py --amp 240 --f-start 0.2 --f-end 1.0 --duration 20` — 주파수 sweep
12. `python swing_modes.py` — 모드 전환 데모

진단 (1회성):
- `python probe_tree.py > tree.txt` — 객체 트리 dump
- `python probe_sec_enc.py` — 2차 인코더 확인
- `python probe_reboot_stability.py` — TOGGLE_TURN 측정

## CAN / ESP32 무선 LFO 제어 — 실제 구현 (2026-05-30)

USB 검증을 마치고 **LilyGo T-2CAN (ESP32-S3) 으로 CAN 제어 + Wi-Fi 무선 스트리밍** 구현 완료.
코드: `esp32t2can/` (ESP32 펌웨어, PlatformIO) + `controller_app/` (데스크탑 앱, PySide6).

### 하드웨어 (T-2CAN)
- **T-2Can V1.0** = ESP32-S3-WROOM-1U + **MCP2515** (Classic CAN 2.0, SPI). ※ FD 변형(MCP2518FD)도 있으니 칩 각인 확인.
- 크리스탈 **16MHz**, CAN **500kbps**, ODrive **node_id=1** (probe_can.py 실측).
- 핀: SPI SCLK12/MOSI11/MISO13, MCP CS10 / INT8 / **RST9**.
- CAN 드라이버 = **autowp/arduino-mcp2515** (LilyGo 예제와 동일, ESP32-S3 호환).
- **CAN 채널 2개·둘 다 절연 (스키매틱 `T-2Can_V1.0.pdf` 확정, 레포 `project/`):**
  - **CAN-A** = MCP2515(SPI) → 절연 트랜시버 U2 `TD501MCAN`. **펌웨어가 쓰는 채널.**
  - **CAN-B** = ESP32 네이티브 TWAI → 절연 트랜시버 U1 `TD501MCAN`. 현재 미사용.
  - 두 채널 **독립 버스 + 갈바닉 절연** (모터 전원 GND ↔ ESP32 분리). "포트 2개 = 같은 버스" 아님.
  - **종단 120Ω 양 채널 다 내장·고정** (RZ2=CAN-A, RZ1=CAN-B, 솔더 고정 — 점퍼 아님). → **T2CAN 은 항상 버스 끝에 두기.** 데이지체인 시 반대쪽 ODrive 만 종단 ON, 중간 ODrive OFF.
- **듀얼 모터 배선 (검증된 권장)**: 두 ODrive 를 CAN-A 한 버스에 묶고 (데이지체인 또는 T2CAN 중심 스타), node_id 1/2 구분. 로컬 오실레이터 1개가 두 setpoint 생성 → 위상 100% 동기 (ESP-NOW 시계동기 불필요). 절연 필요 시에만 CAN-B 분리(네이티브 TWAI 드라이버 추가 작업 필요).
- **ODrive 종단 120Ω = USB 소프트 제어** (매뉴얼 P47): `can.config.r120_gpio_num=5` + `can.config.enable_r120=True/False`. 물리 점퍼 아님. `set_can_term.py` 로 읽기/켜기/끄기 + save. 공장 default = `enable_r120=True`.
- **종단 셋업 (실제 배선: T2CAN 중심 대칭 스타, 각 stub ~1m)**: T2CAN 120Ω 고정(못 끔, 중심) + ODrive 한쪽만 ON → **총 60Ω** 목표. 둘 다 ON 이면 40Ω(과부하), 둘 다 OFF 면 120Ω(부족). → **모터2 = OFF(2026-06-04 저장), 모터1 = ON 유지.** 1m stub 의 미종단 반사는 500kbps(비트 2µs) 대비 ~10ns 라 무시 가능. 검증: 전원 OFF 후 CAN_H↔L = 60Ω.

### 핵심 교훈 / 함정 (반드시 기억)
1. **MCP2515 RST 핀(9) 펄스 필수** — `SPI.begin` 전에 HIGH→LOW→HIGH 안 하면 칩이 리셋에 묶여 SPI 무응답.
2. **sandeepmistry/arduino-CAN 은 ESP32-S3 빌드 실패** (구형 DPORT 레지스터). autowp 사용.
3. **ODrive 0.6.5 는 CAN SDO(RxSdo) 미지원** (0.6.6+ 추가) → CAN 으로 `vel_ramp_rate` 설정 불가, idle 에서 raw 엔코더 읽기 불가. (vel_limit/current_limit/pos_gain 은 전용 메시지로 가능: Set_Limits 0x0F, Set_Pos_Gain 0x1A)
4. **`Get_Encoder_Estimates`(0x09) 는 IDLE 에서 0** (컨트롤러 소스라 CLOSED_LOOP 에서만 라이브). 손으로 돌려도 idle pos=0 이 정상.
   - **단 `Get_Encoder_Count`(0x0A: shadow_count/count_in_cpr)는 IDLE(전류 0)에서도 라이브** (인코더 드라이버 소스). 2026-06-04 `enc_test.cpp` 로 실측 확인: arm 없이 RTR 요청 → 손 움직임 추적됨 (0x09 는 0 고정인데 0x0A 는 변함). `encoder_count_rate_ms` 기본 0 이라 **RTR(remote request)로 on-demand 요청**하면 config 변경 없이 응답. → **전류 없이 CAN 으로 위치 파악 가능.** `count_in_cpr/16384` = mono-turn 위치 (pos_estimate 의 정수turn+이걸로 구성, USB idle 실측에서 pos_estimate ≈ count_in_cpr/cpr 확인). shadow_count 는 raw 누적카운터라 turn 직환산 안 됨.
5. **🚨 0.6.5 MISSING_INPUT(err=0x40) 버그**: PASSTHROUGH 에서 입력이 잠깐 끊기면 **축이 최대 음(-)속도로 폭주** → disarm (0.6.7 에서 수정). 2Hz 직접 setpoint 스트리밍 때 발생. **→ 로컬 오실레이터 구조로 근본 해결** (아래).

### 제어 방식 (POS + velocity feedforward)
- `swing_sine_pos.py` 방식을 CAN 으로: **`Set_Input_Pos`(0x0C) 에 위치 + Vel_FF** 한 프레임. drift 없음.
- arm 시퀀스: VEL_RAMP(vel0)로 arm → 라이브 pos 를 **center 캡처**(idle CAN pos=0 우회) → `Set_Pos_Gain(5)` soft → POS/PASSTHROUGH 전환 → center hold (점프0). **이 순서가 시작 덜커덕/오버커런트 방지의 핵심.**
- **center = arm 시점 실제 위치** → 손/중력으로 둔 위치가 자동으로 swing 중심 (실배포: 진자 중력 정지위치).
- **ESP32→ODrive 송신율 = 200Hz** (무선율과 무관). 60Hz 면 큰 진폭에서 위치 계단 거침 → 200Hz 로 매끈.

### 정지/안전 철학 (실측으로 정립)
- **추종오차(손으로 잡음/무게 뒤처짐)는 에러 아님** → 모터는 계속 추종(버팀), 놓으면 따라잡음. ESP32 가 자체 위치/추종 한계로 disarm 안 함.
- **진짜 정지 = ODrive 가 과전류 등으로 스스로 disarm**(state≠CLOSED_LOOP 또는 axis_error≠0) 일 때만 → fault_stop(IDLE + `Clear_Errors` 0x18).
- 재시작: 앱 run=1 (또는 로컬 BOOT). Clear_Errors 가 latch 풀어줘서 깔끔히 재arm.
- 18V Milwaukee 배터리(전류 여유 충분) 사용 예정 → 75W SMPS 벤치 과전류 걱정은 벤치 한정.
- **⚠️ 실측(2026-06-04): 듀얼 모터 + 토크 펌핑 실테스트 중 75W 벤치 SMPS 의 폴리퓨즈(PTC) 트립** → 모터 전원 끊겨 CAN 하트비트 0 → 전부 먹통(사인·펌프 모두). 펌웨어 정상, 전원 한계 문제였음. PTC 라 식으면 자동복구. **교훈: 75W 로는 듀얼+토크 인러시/피크 부족.** 벤치 계속 쓰려면 current_lim↓(예 5~6A)/PUMP_T_MAX↓/순차 arm/단일 모터, 실테스트는 배터리. 전류예산 ≈ 75W/20V ≈ 3.7A 버스.

### 무선 아키텍처 (Wi-Fi UDP, 파라미터 스트리밍)
**핵심 원칙: 파형(setpoint)을 던지지 말고 파라미터를 던진다.**
- 앱이 60Hz 로 `{run, freq, amp_deg, phase, seq}` (20바이트 LE, `controller_app/protocol.py`) 를 UDP 송신.
- ESP32 가 **로컬 위상 오실레이터**를 freq/amp 로 돌림 (LPF 추종) → ODrive 200Hz 구동. 
  **패킷 끊겨도 로컬에서 사인 계속 생성 → 입력 갭 0 → MISSING_INPUT 폭주 없음.**
- **무수신 COMM_TIMEOUT(0.5s) → fade-out + IDLE** (통신 두절 안전).
- **위상 lock 은 제거함** — Wi-Fi 지터(age 7~34ms)로 위상 끌어당기면 튐/엇박 발생. 단일 모터는 위상 임의값이라 불필요. (듀얼 모터 위상동기는 저속 공유기준으로 별도 구현 예정, 매 패킷 yank 금지.)
- **주파수-진폭 coupling (2026-06-04 갱신)**: 앱 슬라이더 = "강도(%)", 실제 진폭 = 강도 × `amp_deg_max(f)`. **단순 1/f(속도일정) 아님** — 앵커 두 점 `(1Hz,60°)–(5Hz,10°)` 통과하는 `A(f)=a/f+b`(a=62.5,b=−2.5, peak속도 단조감소=안전) 사용. **상한 `FREQ_MAX`=5Hz**. 사인 기준: 1Hz=±60°, 2Hz=±28.8°, 3Hz=±18.3°, 4Hz=±13.1°, 5Hz=±10°. 그네(peak_vel 1.3)는 진폭/1.3 추가 derate. 펌웨어값: `AMP_DEG_MAX`=60, `AMP_MAX_TURN`=1.34turn(=출력60°), `VEL_LIM`=9(안전 속도클램프, 1Hz·60°의 peak 8.4 안 깎게 — **앱 곡선이 실제 shaping, 펌웨어 clamp_amp 는 더 느슨한 vel/accel 안전천장**). ESP32 출력 시점 재클램프 유지.

### 멘탈 모델 (방향성)
**"모터 = 아주 느리고(≤4Hz) 한계 있는 DC 무선 스피커."**
- 현재(파라미터 기반): 지연0, 손실에 무한 강함, 단 파라메트릭 파형(sin/tri/saw)만.
- 다음(데이터 기반, 미구현): 임의 파형(DAW/Ableton) 라이브 스트리밍을 위해 **샘플 스트림 + 지터버퍼(~30~50ms) + 물리 리미터(amp/vel/accel/대역) + 언더런 시 파라메트릭 자유진행 fallback**. = "오디오 스피커" 원리를 UDP 에 구현. (Bluetooth A2DP 는 S3 미지원, AES67 은 과중이라 기성 무선오디오 ❌.)
- DAW 연결: BlackHole(mac)/VB-Cable(win) 가상 오디오 → Python 브리지 → UDP, 또는 ESP32-S3 를 USB Audio Class 장치로(유선).

### 빌드/플래시/모니터 (PlatformIO)
- 빌드+업로드: `platformio run -d esp32t2can -t upload --upload-port COM5` (penv: `C:\Users\songh\.platformio\penv\Scripts\platformio.exe`)
- 시리얼 모니터: `esp32t2can/tools/read_serial.py COM5 <초>` (**penv 파이썬으로** — pyserial 있음. .venv 엔 없음)
- 앱: `.venv` 에 PySide6/pyqtgraph 설치됨. `python controller_app/main.py`
- Wi-Fi: `esp32t2can/src/wifi_config.h` (SSID/PW, **gitignore**). 부팅 시 IP 시리얼 출력 + 상태줄에 상시 표시. 앱 IP 필드에 입력.

## 웨이브테이블 오실레이터 — 사인 + 그네 2파형 구현 완료 (2026-05-30)

⚠️ **코드·컴파일 전부 완료. 보드 플래시는 아직 안 함** (내일 할 일 참고).

### 무엇을 했나
- ESP32 로컬 오실레이터의 `sinf(g_phase)` → **웨이브테이블 lookup + 모프(크로스페이드)** 로 교체.
- **2파형**: `사인`(ripple/진동용) + `그네`(swing/진자). 패킷 변경 없이 기존 `waveform` 바이트 재활용.
- **파형 전환 = 자동 크로스페이드(morph 0.3s)** — 사인↔그네 고르면 ESP32 가 두 테이블 블렌드, C1 연속(튐 없음). 사용자 morph 슬라이더는 다음 단계.

### "그네 파형" 정의 (오늘 확정한 핵심)
- 큰 진폭 **진자 운동방정식** `θ̈=-sin(θ), θ(0)=θ₀, θ̇(0)=0` 의 해를 θ₀ 로 정규화한 것.
  θ₀→0=사인, θ₀ 클수록 **정점에서 머물고 중심을 빠르게 휙**(그네 느낌). shape_param=θ₀.
- **그네 느낌은 위치보다 속도 프로파일에 있다**: 위치 차이는 미묘하고, 중심통과 peak 속도가 사인보다 큼.
- 현재 테이블 **θ₀=150°** → peak 속도 **1.30×**, peak 가속 1.19×. (취향 바꾸려면 `gen_wavetables.py --theta0 168` 등으로 재생성. 단 진폭 derate↑)
- ⚠️ **안전(필수)**: 그네는 peak 속도 큼 → 진폭 클램프를 peak_vel 배수만큼 더 깎아야 vel_limit 안 넘음(MISSING_INPUT 폭주 방지). `clamp_amp(amp, freq, peak_vel, peak_acc)` 로 구현, morph 블렌드값으로 출력 시점 재클램프.

### 새/변경 파일
- `waveforms.py` (신규, 레포 루트): 진자 테이블 정의(numpy, scipy 불필요). 앱 미리보기 + 헤더 생성 공유.
- `gen_wavetables.py` (신규): `esp32t2can/src/wavetables.h` (C 배열 4개 + peak 계수) 생성. **보드 명령 아님**.
- `esp32t2can/src/main.cpp`: `#include wavetables.h`, `wt_lookup()`, g_waveform/g_morph, clamp_amp derate, 출력부 lookup+morph.
- `controller_app/protocol.py`: `WAVE_SWING=1`(기존 TRI/SAW 대체), `amp_deg_max(freq, peak_vel)` derate.
- `controller_app/main.py`: **파형 콤보(사인/그네)** + 미리보기 morph + 주파수·파형 coupling.
- 시각화(1회성): `wavetable_preview.py`, `wavetable_shipped.png` (사인 vs 그네 모양·속도 비교).

## IMU(BNO085) — 진단 모드 작성 + 아키텍처 결정 (2026-05-30)

⚠️ **코드·컴파일 완료. 플래시 안 함. BNO085 실물 아직 미연결.** 상세는 메모리 `project_imu_plan` 참고.

### 결정 (사용자 요구)
- IMU = **Adafruit BNO085**, T-2CAN **QWIIC(STEMMA)=I2C** 연결. **SDA=GPIO1, SCL=GPIO2** (LilyGo 공식 핀맵 확인, 기존 SPI/MCP 핀과 무충돌), 주소 0x4A.
- **IMU 없어도 그네 동작은 그대로** — IMU 는 오픈루프 오실레이터에 얹는 **선택 레이어**. 못 찾으면 `g_imu_ok=false` 로 기존 동작 유지.
- **설치마다 위치 달라짐 → 부팅 자동 tare**: 모터 `g_center` 캡처와 같은 패턴. 정지 확정 시 현재 방향을 0점으로. (지자기 안 쓰고 Game Rotation Vector = 중력기준 tilt.)

### `esp32t2can/src/imu_test.cpp` (신규, 별도 PlatformIO env `imu-test`)
- 제어/모터 완전 분리(메인 `main.cpp` 안 건드림). `platformio.ini` 에 `build_src_filter` 로 env 별 소스 분리, `imu-test` 는 `adafruit/Adafruit BNO08x` 만.
- 출력: 쿼터니언 / **rest 대비 tilt°** / gyro[xyz] / STILL / **▲APEX(정점)** 마커. 자동 tare + 수동 재-tare(시리얼 `t`/BOOT).
- 빌드/플래시: `platformio run -d esp32t2can -e imu-test -t upload --upload-port COM5`

## 듀얼 모터 펌웨어 — Phase 5 (2026-06-04)

⚠️ **코드·컴파일 완료(빌드 SUCCESS). 보드 플래시·실배선 검증 안 함.**

### 무엇을 했나
- `main.cpp` 를 단일노드 → **2 ODrive(node_id 1,2) 동시 제어**로 리팩터. 한 CAN-A 버스, 오실레이터 1개로 두 모터 구동 → 위상 원천 동기 (ESP-NOW/시계동기 불필요).
- `set_node_id.py` 로 모터2 = node_id 2 저장 완료. `set_can_term.py` 로 모터2 종단 OFF(60Ω) 저장 완료.

### main.cpp 상단 조정 상수 (조립 후 한 줄로 바꿈)
- `NODE_IDS[] = {1, 2}` — 두 모터 노드.
- **극성 = 앱 런타임 제어** (2026-06-04): 더 이상 컴파일 상수 아님. 앱 체크박스 "모터1/모터2 반전" → 패킷 `flags`(offset6, bit0=m1, bit1=m2) → ESP32 `g_motor_sign[]`. 기본 = node2 체크(미러). **운전 중 토글하면 즉시 반전(슬램) 안 하고 `g_restart_pending` 으로 fade-out→새 극성 fade-in 자동 재시작** (amp≈0 시점에 부호 교체, 재arm 없이 매끈). 앱 STOP/통신두절/fault 는 재시작 취소하고 IDLE.
- `MOTOR_REL_PHASE[] = {0, 0}` — **평행 swing(batten 수평 유지).** Z-twist(중심축 비틀기)는 `{0.0f, M_PI}` 로.

### 구조 변경 핵심
- `St st[2]`, `g_center[2]` 모터별. `sendCmd(node,…)`/`tx_*(node,…)` 노드 인자. `drainRx` 가 `motor_index(node)` 로 분배.
- `arm_and_center()` 두 모터 동시 arm → 각자 center 캡처 → 각자 soft gain POS hold.
- **fault = 어느 한쪽이라도 disarm(과전류/err) → `fault_stop` 가 양쪽 동시 IDLE** (batten 비대칭 응력 방지, CLAUDE 체크리스트 반영). 통신두절·BOOT 도 양쪽 정지.
- CAN 부하: 200Hz × 2모터 = 400 frame/s ≈ 500kbps 의 ~15%, 여유.

### 빌드/플래시
- 빌드: `platformio run -d esp32t2can -e lilygo-t-2can` (env 명 = **`lilygo-t-2can`**, imu-test 아님)
- 플래시: `… -e lilygo-t-2can -t upload --upload-port COM5`

### 검증 순서 (다음, 전부 실배선 후)
1. 두 모터 CAN-A 버스 연결(종단 60Ω 확인) + 플래시 → 앱 START.
2. **부호 확인**: 두 모터가 같은 물리 방향인가? 반대면 `MOTOR_SIGN` 한쪽 -1, 재플래시.
3. 한쪽 강제 disarm(손으로 막아 과전류) → 양쪽 같이 멈추는지.
4. (선택) `MOTOR_REL_PHASE={0,M_PI}` 로 twist 모드 시험.

## IMU 메인 통합 + 앱 2D 시각화 (2026-06-04)

⚠️ **컴파일·플래시·앱 실행 완료. 실제 IMU 움직임 표시 검증은 사용자 확인 단계.**

- **방침**: IMU 는 **센싱만**(제어엔 아직 안 씀). 방향(공진/펌핑) 미정이라 일단 "보기"만.
- **main.cpp 에 BNO085 통합** (모터 제어와 공존): `imu_poll()` 매 loop, `Wire.begin(SDA1,SCL2)`, GameRotVec+Gyro, 정지 자동 tare, apex 감지. **IMU 없어도 모터 정상**(g_imu_ok=false 면 스킵). BOOT=비상정지 유지(IMU tare 와 분리), 시리얼 't'/앱 버튼으로 재-tare.
- **텔레메트리 ESP32→앱** (신규): `TELEM_MAGIC=0x0DCB`, 38바이트 `{seq,status,tilt,pitch,roll,gx,gy,gz,m1pos,m2pos}`. 앱이 보낸 패킷 송신자 주소로 회신(`g_app_ip`), 30Hz. status 비트=imu_ok/rest_set/still/apex.
- **앱(`controller_app`)**: `unpack_telem` + **2D 틸트(roll/pitch 진자) + tilt° 시간그래프 + 상태라벨 + "IMU 0점(tare)" 버튼**. 3D 불필요(그네=1축). tare 는 `flags` bit2 `REQ_TARE` one-shot(6틱) → ESP32 rising-edge.
- **프로토콜 확장**: 명령패킷 offset6 `flags` = bit0 m1극성 / bit1 m2극성 / **bit2 REQ_TARE**. 극성변경 감지는 `&0x03` 마스크(tare 비트 무관).
- 빌드 RAM 14.9%/Flash 22.6%. 검증: telem 왕복 OK, main.py 컴파일 OK.

## 2026-06-04~05 세션 요약 (듀얼모터 + IMU통합 + 펌프/리프트드롭)

**완료·검증:**
- **듀얼모터**: 모터2 `node_id=2` + `enable_r120=False`(종단OFF) 영구저장 (`set_node_id.py`/`set_can_term.py`). T2CAN=2채널 절연버스(스키매틱 확인). `main.cpp` 듀얼노드 `{1,2}` 리팩터, **MOTOR_SIGN 미러보정**(node2=−1), fault 시 양쪽 동시정지.
- **극성 토글**(앱 체크박스→flags, 운전중 변경 시 fade-restart), **per-motor 게인 CAN 적용**(Set_Limits 0x0F/Set_Vel_Gains 0x1B).
- **CAN 0x0A(Get_Encoder_Count) idle 위치읽기 확인**(`enc_test.cpp`, RTR): **전류0으로 위치파악 가능** → teach&playback 토대.
- **IMU(BNO085) 메인통합**: telemetry(38→54B) → 앱 2D틸트/tilt/자이로 플롯 + tare. **전류(Iq)/VBus/버스전류(Ibus) 모니터 + 피크홀드**(RTR 0x14/0x17).
- **진폭곡선**: `FREQ_MAX=5`, 앵커 `(1Hz,60°)–(5Hz,10°)` a/f+b. **고유진동수 ≈0.4Hz 측정**(`measure_freeswing.py`).
- **토크펌핑 모드**(`WAVE_PUMP`, 공통모드 anti-damping). **들어올림+낙하**(`WAVE_LIFTDROP`, 수동 좌/우 버튼 press-hold).

**⚠️ 안전사고 + 교훈 (중요):**
- **토크모드 프리폴(TORQUE/PASSTHROUGH torque 0)이 ODrive 0.6.5 MISSING_INPUT 버그로 폭주** → 360°+ 연속회전, 전선 끊길 뻔. **이 기구는 파이프가 자유롭게 돌아 토크모드 자체가 위험.** → **lift-drop은 위치제어 only + 프리폴은 IDLE(비여자, STOP과 동일)** 로만. ±90° 하드리미트(`fault_stop`).
- **버그 수정**: 프리폴로 IDLE 끄면 "ODrive fault(state≠CLOSED_LOOP)" 안전체크가 오인 → arm/disarm 무한반복(터턱). `intentional_idle`(lift-drop+!g_ld_armed) 예외처리로 해결.

**미완 (다음 세션):** lift-drop "안된다"(사용자) — 위치제어 buttons는 동작하나 프리폴 후 catch/스윙 느낌 미흡. 기구 특성(무거운 암 고관성 + 파이프 자유회전 + 토크모드 금지)이 근본 제약. **위치제어 기반으로만** 자연스러운 스윙 재설계 필요. (관련 메모리: [[project_teach_playback]])

## 내일 할 일 (리마인드 — 전부 보드 명령, 승인 필요)
1. **그네 파형 무부하 검증**: `main.cpp` 플래시 → 앱(`python controller_app/main.py`) 으로 사인↔그네 전환·느낌 실측. (필요시 θ₀ 조정 후 `gen_wavetables.py` 재생성→재플래시.)
2. **BNO085 진단**: 실물 QWIIC 연결 → `imu-test` 플래시 → tilt/gyro/APEX/자동tare 동작 확인. (안 되면 SDA1/SCL2·주소·배선부터.)
3. 그 다음(선택): morph 슬라이더 + shape_param 프로토콜 확장 / IMU 를 제어 루프에 얹기(공진 펌핑·공진주파수 측정).

### (이후) 프로토콜 확장 — 미구현
현재 20바이트 `{magic,seq,run,waveform,freq,amp_deg,phase}`. 향후 `morph(f32)` + `shape_param(f32)` 추가 →
앱에 morph 슬라이더 + θ₀ 슬라이더. (지금은 waveform 바이트만 실사용, morph 는 ESP32 자동 전환.)

### 그 이후
- **OSC 수신 전환** (TouchDesigner `OSC Out CHOP`/Max → ESP32 OSC 라이브러리). 무선·DAW연동 기성품화.
- 데이터 기반(진짜 비주기 임의신호) 필요 시 **지터버퍼 + 리미터** (슬로우 스피커 보호단).
- 듀얼 모터(ESP-NOW broadcast) + **Ableton Link** 박자동기(ESP32 포팅 존재).
- DAW 오디오 경로: BlackHole(mac)/VB-Cable(win) 또는 VBAN(audio-over-UDP).

## 다음 작업 (TODO)

- [x] ~~무부하 무대 동작 시 잔존 진동 잡기~~ — 2026-05-12 P29-30 절차로 vel_gain=0.145, pos_gain=50 영구 저장 완료.
- [ ] 진자 부착 후 동작 검증 (잔존 진동 자연 해소 + 새 게인의 실제 효과)
- [x] ~~공진 주파수 측정 (자유 진동)~~ — 2026-06-04 `controller_app/measure_freeswing.py` 로 IMU telemetry 받아 측정. **고유 진동수 ≈ 0.4Hz (주기 2.5s)**. roll(부호각, std 4.3°)+tilt(크기)÷2 두 독립신호 일치. 봉+필름=중력진자 확인. ⚠️ tilt 는 크기라 중심통과마다 0→**실제의 2배 주파수**(부호있는 pitch/roll/gyro 로 봐야 진짜값). gx 1.33Hz 는 스윙 아닌 진동/구조모드 추정.
- [ ] `apply_tuning.py` 작성 — 다른 모터에 같은 게인 일괄 적용용
- [ ] CAN 이행 시 `watchdog_timeout = 0.1` 설정 + `save_configuration` (현재 0 = 비활성)
- [ ] `odrivetool backup-config` 로 공장 캘리브 + 튜닝 설정 백업
- [x] ~~CAN(T-2CAN) 제어~~ — 2026-05-30 완료 (POS+ff, 200Hz, autowp/mcp2515)
- [x] ~~Wi-Fi UDP 무선 LFO 스트리밍~~ — 2026-05-30 완료 (파라미터 기반 + 로컬 오실레이터 + 통신두절 IDLE)
- [x] ~~데스크탑 컨트롤러 앱~~ — 2026-05-30 controller_app (PySide6+pyqtgraph, freq/amp 슬라이더, 파형 비주얼)
- [x] ~~웨이브테이블: 사인+그네 2파형~~ — 2026-05-30 구현·컴파일 완료 (morph 크로스페이드, peak_vel derate). **플래시 미실행**
- [x] ~~IMU 진단 모드 작성~~ — 2026-05-30 `imu_test.cpp` (별도 env, BNO085 QWIIC, tilt/gyro/APEX/자동tare). **플래시·실물연결 미실행**
- [ ] **(내일) 그네 파형 무부하 플래시·검증** — main.cpp 플래시 + 앱 사인↔그네 전환 실측
- [x] ~~BNO085 실물 연결 + imu-test 플래시·검증~~ — 2026-06-04 완료. I2C(SDA1/SCL2,0x4A) OK, tilt°(33→86°) 추종, gyro, 자동tare(rest=set) 정상. (APEX 는 주기 swing 시 표시)
- [ ] **(A) idle 위치 모니터링 + "진자 정지 후 시작"** — main.cpp 에 0x0A RTR idle 읽기 추가. 앱에 전류0 현재각도 표시 + 0x0A 안정 시에만 arm. (center 캡처는 안전한 VEL-arm 유지)
- [ ] **teach & playback (손 녹화→부드럽게 재생)** — 사용자 핵심 목표. 모터 무여자(전류0)로 손 backdrive → 0x0A 로 궤적 녹화 → POS 로 재생. 2026-06-04 0x0A idle-read 실측 확인으로 토대 마련. (A 위에 구현)
- [ ] morph 슬라이더 + shape_param(θ₀) 프로토콜 확장 (앱 + ESP32 파서)
- **(2026-06-04) 기구 발견: 모터-암(무거운 추) 사이 고관성/탄성** → 빠른 펌프 지터 토크론 암이 안 움직이고 파이프/탄성만 돎. **느린 위치/토크 구동은 암을 움직임.** → 펌프(anti-damping)는 이 기구에 비효율 → **`WAVE_LIFTDROP=3` "들어올림+낙하(래칫)" 모드 구현**: 토크 PD로 ±amp 까지 올림(올리는 절반만 모터 일함) → 토크0 프리폴(중력, 내리는 절반) → 홈 통과 시 반대편 올림 반복. 공통(강체)swing 각/속도 사용(미러보정). 파라미터 LD_KP=4,LD_KD=1,LD_T_MAX_BASE=0.8N·m(펌프세기 슬라이더로 스케일), 못 올리면 4s timeout→release(스톨 방지). 안전(펌프와 공유): IMU>80°/vel>6/angle>amp×1.5/err→정지. **무부하/실부하 미검증, 폴리퓨즈 주의(올릴 때 전류↑).**
- [x] ~~공진 주파수 측정~~ 완료(≈0.4Hz). [ ] **토크 펌핑 모드 구현 완료(2026-06-04, 무부하/실부하 미검증)** — `WAVE_PUMP=2` 파형. 토크제어(PASSTHROUGH)로 arm. **공통(강체 swing) 속도 기반 펌핑**: `swing_vel = avg(g_motor_sign[i]·vel_i)`, `T_i = g_motor_sign[i]·clamp(pump_gain×swing_vel)`, 진폭한계 밖이면 `−K_brake`. **미러보정(g_motor_sign) 적용 필수** — 안 그러면 ① 사인모드와 좌표 불일치 ② twist(비틀림)모드까지 증폭됨. 공통모드 법칙이 강체 스윙만 펌핑하고 twist 무시(2026-06-04 수정). arm 시 apply_polarity latch. 속도/각도=엔코더 0x09(CL에서 라이브), IMU tilt=독립 안전한계. 안전: IMU>70°/|vel|>3/|angle|>한계×1.5/ODrive err→fault_stop. 펌프게인=앱 phase필드(0..1×0.3), 진폭한계=amp_deg. T_MAX=0.3N·m. 펌프↔파형 전환은 재arm 필요(정지 후 재START). **실검증 시 펌프세기 0부터 천천히, STOP 손.** 다음: 정점 위상 펌핑·공진 자동추종
- [ ] **데이터 기반 "슬로우 스피커"**: 샘플 스트림 + 지터버퍼 + 물리 리미터 + 파라메트릭 fallback (임의 파형/DAW용)
- [ ] DAW(Ableton) 연결: 가상 오디오(BlackHole/VB-Cable) → 브리지 → UDP
- [x] ~~듀얼 모터 node_id=2 + 종단 OFF + 펌웨어 듀얼노드 리팩터~~ — 2026-06-04 완료(컴파일). **플래시·실배선 미검증**. ESP-NOW 불필요(한 CAN-A 버스, 오실레이터 1개로 동기). 다음: 실배선 + MOTOR_SIGN 부호 확인
- [ ] CAN 배포 전 ODrive 에 `watchdog_timeout=0.1` save (ESP32 죽으면 모터 disarm 2중 안전)

### 듀얼 모터 셋업 체크리스트 (5m × 1.2m 마일라 batten)

**개별 모터 준비 (각 모터마다):**
- [ ] 모터 2 에 firmware/하드웨어 동일성 확인 (`probe_tree.py` dump 비교)
- [ ] 모터 2 에 동일 게인 적용 — **옵션 A**: P29-30 절차 독립 실행 (보수적, 개체차 반영) / **옵션 B**: `apply_tuning.py` 로 모터 1 값 복제 (빠름, 개체차 무시)
- [ ] 모터 2 의 `TOGGLE_TURN` 측정 (`probe_reboot_stability.py`) — 모터마다 다름
- [ ] 모터 2 의 `index_offset` 설정 (`set_zero.py`) — 기구부 0° (batten 수평 위치) 기준
- [ ] 모터 2 의 `watchdog_timeout = 0.1` 영구 저장

**동기화:**
- [ ] 양쪽 모터 0° 기준 = batten 정확히 수평 (기구 조립 시 정렬)
- [ ] ESP-NOW (또는 CAN bridge) 두 모터에 동시 명령 송신
- [ ] 송신 시계 동기 (ESP32 RTC 동기 또는 한쪽이 master, 다른쪽이 echo 추종)

**드리프트 방지 — 모드 선택:**
- VEL_RAMP 단독 ❌: 각 모터 drift 누적 다름 → 시간 갈수록 batten 기울어짐
- **POS+ff (soft gain) 또는 TRAP_TRAJ ✓**: 위치 cmd 추종 → drift 없음

**동시 동작 테스트:**
- [ ] 정지 hold 상태에서 batten 수평 유지 (양쪽 모터 IDLE 아닌 CLOSED_LOOP)
- [ ] X-swing (같은 방향) — batten 평행 유지, 비대칭 없음
- [ ] Z-twist (반대 방향) — batten 회전축이 중심에 있음, 한쪽으로 끌리지 않음
- [ ] 1분 이상 운용 후 두 모터 pos 차이 측정 (10° 이하 목표)

**에러 / 통신 두절 시 안전 동작:**
- [ ] 한쪽 모터 disarm (watchdog timeout 또는 overcurrent) 시 다른 모터도 동시 disarm 되어야 batten 안전. 단독 hold 면 batten 한쪽으로 기울어 응력.
- [ ] 송신측 (ESP32/PC) 에서 "한쪽 응답 없음" 감지 시 다른 모터에도 stop 명령 전송 로직
- [ ] watchdog 작동 후 batten 자연 정지 (진자 댐핑) 까지의 시간 측정
