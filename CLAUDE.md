# Steadywin GIM6010-8 무대 공연용 마일라 필름 스윙 제어

**GitHub**: https://github.com/strongestweapon/SteadywinGim6010-8

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
| **2번** | `C4610A6C3037` | 19916 | vel_gain 0.10 / vel_int 1.0 / curr_lim 60 | 0.10 이 부드럽고 **0.145 는 더 떨림 (1번과 반대)** | 0 (미설정) | 미측정 | ❌ 전부 공장 default. 튜닝/0점/저장 안 함 |
| **3번** | `7B600A6C3037` | 21012 | **vel_gain 0.05 / vel_int 0.20 / curr_lim 45** | **현재까지 가장 부드러움 (공장 0.05 그대로)** | 0 (미설정) | 미측정 | ❌ 전부 공장 default. 튜닝/0점/저장 안 함 |
| **4번** | `7A360A6C3037` | 13017 | **vel_gain 0.05 / vel_int 0.20 / curr_lim 45** (3번과 동일) | 3번보다 **조금 더 떨림** (같은 게인인데도) | 0 (미설정) | 미측정 | ❌ 전부 공장 default. 튜닝/0점/저장 안 함 |
| **5번** | `8F350A6C3037` | 10439 | **vel_gain 0.05 / vel_int 0.20 / curr_lim 45** (3번과 동일) | **3번과 비슷하게 부드러움** (3·5번이 베스트) | 0 (미설정) | 미측정 | ❌ 전부 공장 default. 튜닝/0점/저장 안 함 |

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

### 핵심 교훈 / 함정 (반드시 기억)
1. **MCP2515 RST 핀(9) 펄스 필수** — `SPI.begin` 전에 HIGH→LOW→HIGH 안 하면 칩이 리셋에 묶여 SPI 무응답.
2. **sandeepmistry/arduino-CAN 은 ESP32-S3 빌드 실패** (구형 DPORT 레지스터). autowp 사용.
3. **ODrive 0.6.5 는 CAN SDO(RxSdo) 미지원** (0.6.6+ 추가) → CAN 으로 `vel_ramp_rate` 설정 불가, idle 에서 raw 엔코더 읽기 불가. (vel_limit/current_limit/pos_gain 은 전용 메시지로 가능: Set_Limits 0x0F, Set_Pos_Gain 0x1A)
4. **`Get_Encoder_Estimates`(0x09) 는 IDLE 에서 0** (컨트롤러 소스라 CLOSED_LOOP 에서만 라이브). 손으로 돌려도 idle pos=0 이 정상.
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

### 무선 아키텍처 (Wi-Fi UDP, 파라미터 스트리밍)
**핵심 원칙: 파형(setpoint)을 던지지 말고 파라미터를 던진다.**
- 앱이 60Hz 로 `{run, freq, amp_deg, phase, seq}` (20바이트 LE, `controller_app/protocol.py`) 를 UDP 송신.
- ESP32 가 **로컬 위상 오실레이터**를 freq/amp 로 돌림 (LPF 추종) → ODrive 200Hz 구동. 
  **패킷 끊겨도 로컬에서 사인 계속 생성 → 입력 갭 0 → MISSING_INPUT 폭주 없음.**
- **무수신 COMM_TIMEOUT(0.5s) → fade-out + IDLE** (통신 두절 안전).
- **위상 lock 은 제거함** — Wi-Fi 지터(age 7~34ms)로 위상 끌어당기면 튐/엇박 발생. 단일 모터는 위상 임의값이라 불필요. (듀얼 모터 위상동기는 저속 공유기준으로 별도 구현 예정, 매 패킷 yank 금지.)
- **주파수-진폭 coupling**: 앱 슬라이더 = "강도(%)", 실제 진폭 = 강도 × vel_limit 허용 최대 = `35.8/f`(출력°). 1Hz→±30°, 2Hz→±17.9°, 4Hz→±9°. 주파수 올리면 진폭 자동 축소. **4Hz 상한**. ESP32 도 출력 시점 재클램프(안전).

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

## 다음 세션 계획: 웨이브테이블 오실레이터 (2026-05-31 바로 시작)

**목표**: ESP32 로컬 오실레이터를 단일 사인 → **웨이브테이블 신스**로 확장. 재미난 파형 몇 개 + 파라미터,
파형 간 튐 없이 전환. DAW/앱은 "어느 파형 + 모프 + 파라미터 + freq/amp" 만 전송.

### 왜 이 방향인가 (어제 정립한 결론)
- **웨이브테이블을 ESP32 에 저장 → 샘플 스트리밍 불필요 → 무지연 + 손실 견고** + 임의 파형 표현력.
  ("데이터 스트리밍 vs 파라미터" 갈림길의 정답. 샘플을 버퍼링하면 지연 발생하지만, 파형을 ESP32 가
  자기 클럭으로 생성하면 모션 지연 0. LFO 는 주기적이라 이걸로 충분.)
- 멘탈 모델: **"모터 = 느리고(≤4Hz) 한계 있는 DC 무선 스피커."**

### 구현 단계
1. **웨이브테이블 뱅크**: N개 파형 (256 or 1024 샘플/주기 `float[]`). 시작 세트 = SINE, TRIANGLE, +α
   (사용자가 "재미난" 모양 — 비대칭 swing, 더블범프, exp 감쇠 등 — 정할 것). 각 파형 shape_param 몇 개.
2. **위상 누산기(이미 있음)** 로 테이블 보간 읽기: `value = lerp(tbl[i], tbl[i+1])`.
   `pos = center + amp·value(phase)`, `vel_ff = amp·2πf·deriv(phase)` (미분테이블 미리계산 or 수치미분).
3. **파형 전환 2모드**:
   - **모프(크로스페이드)**: `out = (1-m)·A + m·B`, m 0→1 ~0.2~0.5s 램프. 어떤 모양이든 **위치·속도 연속(C1)**.
   - **즉시 전환**: ⚠️ **모터는 끝점(turnaround, vel=0, pos=±amp)에서 교체** — 중심(0점)은 vel 최대라
     거기서 바꾸면 저크. (오디오의 "zero-cross 스위치" 의 모터판 = 끝점). 또는 그냥 모프로 안전하게.
4. **기존 재사용**: freq/amp LPF, 진폭 클램프(vel_limit, 4Hz상한), 자유진행, 통신두절 IDLE,
   fault_stop(Clear_Errors), arm 시퀀스(VEL→center→POS soft gain), **200Hz** 송신.

### 프로토콜 확장 (controller_app/protocol.py + ESP32 파서)
현재 20바이트 `{magic,seq,run,waveform,freq,amp_deg,phase}` → `waveform_id` 실사용 + `morph(f32)` +
`shape_param(f32)` 추가. 앱에 파형 선택 + 모프 슬라이더 추가.

### 시작점 (현재 코드)
- `esp32t2can/src/main.cpp`: 60Hz 루프의 `sinf(g_phase)` 한 줄을 **테이블 lookup + 모프**로 교체.
  나머지(Wi-Fi UDP, arm, 클램프, 안전, 200Hz)는 그대로.
- `controller_app/main.py`: freq/강도 슬라이더에 **waveform 콤보 + morph 슬라이더** 추가.

### 그 이후
- **OSC 수신 전환** (TouchDesigner `OSC Out CHOP`/Max → ESP32 OSC 라이브러리). 무선·DAW연동 기성품화.
- 데이터 기반(진짜 비주기 임의신호) 필요 시 **지터버퍼 + 리미터** (슬로우 스피커 보호단).
- 듀얼 모터(ESP-NOW broadcast) + **Ableton Link** 박자동기(ESP32 포팅 존재).
- DAW 오디오 경로: BlackHole(mac)/VB-Cable(win) 또는 VBAN(audio-over-UDP).

## 다음 작업 (TODO)

- [x] ~~무부하 무대 동작 시 잔존 진동 잡기~~ — 2026-05-12 P29-30 절차로 vel_gain=0.145, pos_gain=50 영구 저장 완료.
- [ ] 진자 부착 후 동작 검증 (잔존 진동 자연 해소 + 새 게인의 실제 효과)
- [ ] 공진 주파수 측정 (자유 진동)
- [ ] `apply_tuning.py` 작성 — 다른 모터에 같은 게인 일괄 적용용
- [ ] CAN 이행 시 `watchdog_timeout = 0.1` 설정 + `save_configuration` (현재 0 = 비활성)
- [ ] `odrivetool backup-config` 로 공장 캘리브 + 튜닝 설정 백업
- [x] ~~CAN(T-2CAN) 제어~~ — 2026-05-30 완료 (POS+ff, 200Hz, autowp/mcp2515)
- [x] ~~Wi-Fi UDP 무선 LFO 스트리밍~~ — 2026-05-30 완료 (파라미터 기반 + 로컬 오실레이터 + 통신두절 IDLE)
- [x] ~~데스크탑 컨트롤러 앱~~ — 2026-05-30 controller_app (PySide6+pyqtgraph, freq/amp 슬라이더, 파형 비주얼)
- [ ] **데이터 기반 "슬로우 스피커"**: 샘플 스트림 + 지터버퍼 + 물리 리미터 + 파라메트릭 fallback (임의 파형/DAW용)
- [ ] DAW(Ableton) 연결: 가상 오디오(BlackHole/VB-Cable) → 브리지 → UDP
- [ ] 듀얼 모터: 2번째 ODrive node_id=2, ESP-NOW broadcast + 저속 위상동기
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
