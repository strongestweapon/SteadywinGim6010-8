# GIM6010-8 모터 인벤토리 (1~5번)

2026-05-29 측정. 각 모터를 USB Type-C 로 단독 연결해 `test_connect.py` + `probe_tree.py` 로 dump 한 값.
부드러움 평가는 `swing_sine_vel.py --amp 240 --freq 1.0 --duration 20 --vel-ramp 100` (출력축 ±30°, current_lim RAM 10A) **무부하 책상 테스트** 기준.

> **주의**: 1번은 이미 P29-30 튜닝 + `save_configuration` 영구화된 상태라 "현재 보드값"이 공장값과 다름. 2~5번은 전부 공장 default 그대로 (RAM 변경도 영구 저장 안 함).

## 전 모터 공통 (5대 모두 동일)

| 항목 | 값 |
|---|---|
| 펌웨어 | **v0.6.5** |
| 하드웨어 | **v3.12-1V** (단일축 보드, `axis1` 없음) |
| `motor.config.pole_pairs` | 14 |
| `motor.config.gear_ratio` | 8.0 (8:1 유성기어) |
| `motor.config.torque_constant` | 0.097 N·m/A |
| `motor.config.pre_calibrated` | True |
| `encoder.config.pre_calibrated` | True |
| `encoder.config.mode` | 260 (SPI Abs MA600 mono-turn) |
| `encoder.config.cpr` | 16384 |
| `encoder.config.sec_enc_cpr` | 16384 (출력축 2차 인코더) |
| `encoder.config.bandwidth` | 500 Hz (변경 금지 — PID 발산 위험) |

→ **모델 상수·인코더·캘리브 구조는 5대 전부 동일.** `pre_calibrated=True` 이므로 **재캘리브 절대 불필요/금지** (공장 `phase_offset` 보호).

## 개체별 값 (모터마다 다름)

| 모터 | 시리얼 | `phase_offset` (공장) | 공장 출하 게인 그룹 | 부드러움 (무부하 실측) | `index_offset` | `TOGGLE_TURN` |
|---|---|---|---|---|---|---|
| **1번** | `89340A6C3037` | 22076 | A: vel_gain 0.10 / vel_int 1.0 / curr_lim 60 | (튜닝 후 0.145 채택 — 0.10보다 부드러움) | **-2.142441** (설정됨) | **0.142469** (측정됨) |
| **2번** | `C4610A6C3037` | 19916 | A: vel_gain 0.10 / vel_int 1.0 / curr_lim 60 | 0.10 부드러움, **0.145 는 더 떨림** | 0 (미설정) | 미측정 |
| **3번** | `7B600A6C3037` | 21012 | B: vel_gain 0.05 / vel_int 0.20 / curr_lim 45 | ⭐ **가장 부드러움** | 0 (미설정) | 미측정 |
| **4번** | `7A360A6C3037` | 13017 | B: vel_gain 0.05 / vel_int 0.20 / curr_lim 45 | 3번보다 조금 더 떨림 | 0 (미설정) | 미측정 |
| **5번** | `8F350A6C3037` | 10439 | B: vel_gain 0.05 / vel_int 0.20 / curr_lim 45 | ⭐ 3번과 비슷하게 부드러움 | 0 (미설정) | 미측정 |

### 공장 default (그룹 A·B 공통, 게인 외)
- `controller.config.pos_gain` = 20
- `controller.config.vel_limit` = 30 turn/s
- `encoder.config.index_offset` = 0 (1번만 -2.142441 로 영구 설정됨)

## 핵심 발견 (2026-05-29)

1. **`phase_offset` 은 모터마다 전부 다르다** (22076 / 19916 / 21012 / 13017 / 10439). 공장이 각 자석·권선 특성을 측정해 구운 값 → **복제·덮어쓰기 금지, 재캘리브 금지.** 다른 게 정상.

2. **공장 출하 게인이 두 그룹으로 갈린다.** 1·2번 = 그룹 A (`0.10 / 1.0 / 60`), 3·4·5번 = 그룹 B (`0.05 / 0.20 / 45`). 5대 모두 fw v0.6.5 인데도 다름 — SteadyWin 이 배치/개체별로 다른 값을 굽는 듯.

3. **최적 `vel_gain` 이 개체마다 다르다.** 1번은 0.145 가 부드러웠는데 2번은 0.145 가 오히려 더 떨림. → **"모터 1 값 일괄 복제" 전략은 위험.** 개체별로 비교 후 채택해야 함.

4. **낮은 `vel_gain` 일수록 무부하에서 부드럽다** (0.05 > 0.10 > 0.145 순). cogging 토크 ripple 보정이 약해지기 때문. **단 무부하 책상 기준** — 게인이 너무 낮으면 진자 부하 시 추종이 처질 수 있어 부착 후 재확인 필요.

5. **같은 게인이어도 기계적 개체차로 부드러움이 다르다.** 3·4·5번 모두 공장 0.05 인데 3·5번이 부드럽고 4번이 조금 더 떨림. 게인으로 다 설명 안 되는 순수 기계차(베어링/기어 맞물림/코깅 위상). → **부드러움 최우선이면 여러 개체 중 가장 부드러운 것을 선별** 하는 것도 유효. 현재 베스트: **3번, 5번**.

## 부드러움 순위 (무부하, 2026-05-29)

1. **3번 ≈ 5번** (공장 0.05) — 가장 부드러움
2. **4번** (공장 0.05) — 3·5번보다 약간 떨림
3. **1·2번** (공장 0.10 그룹) — 게인이 달라 직접 비교는 애매. 1번은 0.145 로 튜닝됨.

## 듀얼 모터 셋업 시 모터 선택 참고

- 5m × 1.2m 마일라 batten 양끝에 2대 사용 예정 (CLAUDE.md 듀얼 모터 체크리스트 참고).
- **양끝 모터는 부드러움·특성이 비슷한 쌍으로 고르는 게 유리** (비대칭 동작 최소화). 같은 게인 그룹(B) + 둘 다 부드러운 **3번·5번 조합** 이 현재로선 후보.
- 각 모터마다 `index_offset` (0점) + `TOGGLE_TURN` 은 개별 측정 필수 (개체차).

---

# GIM6010-8 Motor Inventory (#1–#5) — English

Measured 2026-05-29. Each motor connected individually via USB Type-C and dumped with `test_connect.py` + `probe_tree.py`.
Smoothness ratings are from `swing_sine_vel.py --amp 240 --freq 1.0 --duration 20 --vel-ramp 100` (output shaft ±30°, current_lim 10A in RAM), **no-load bench test**.

> **Note**: #1 is already P29-30 tuned and persisted via `save_configuration`, so its *current* board values differ from factory. #2–#5 are all factory default (RAM changes are not persisted).

## Common to all 5 motors

| Item | Value |
|---|---|
| Firmware | **v0.6.5** |
| Hardware | **v3.12-1V** (single-axis board, no `axis1`) |
| `motor.config.pole_pairs` | 14 |
| `motor.config.gear_ratio` | 8.0 (8:1 planetary) |
| `motor.config.torque_constant` | 0.097 N·m/A |
| `motor.config.pre_calibrated` | True |
| `encoder.config.pre_calibrated` | True |
| `encoder.config.mode` | 260 (SPI Abs MA600 mono-turn) |
| `encoder.config.cpr` | 16384 |
| `encoder.config.sec_enc_cpr` | 16384 (output-shaft 2nd encoder) |
| `encoder.config.bandwidth` | 500 Hz (do NOT change — PID divergence risk) |

→ **Model constants, encoder, and calibration structure are identical across all 5.** Since `pre_calibrated=True`, **re-calibration is unnecessary and forbidden** (protects factory `phase_offset`).

## Per-unit values (differ per motor)

| Motor | Serial | `phase_offset` (factory) | Factory gain group | Smoothness (no-load) | `index_offset` | `TOGGLE_TURN` |
|---|---|---|---|---|---|---|
| **#1** | `89340A6C3037` | 22076 | A: vel_gain 0.10 / vel_int 1.0 / curr_lim 60 | (tuned to 0.145 — smoother than 0.10) | **-2.142441** (set) | **0.142469** (measured) |
| **#2** | `C4610A6C3037` | 19916 | A: vel_gain 0.10 / vel_int 1.0 / curr_lim 60 | 0.10 smooth, **0.145 rougher** | 0 (unset) | not measured |
| **#3** | `7B600A6C3037` | 21012 | B: vel_gain 0.05 / vel_int 0.20 / curr_lim 45 | ⭐ **smoothest** | 0 (unset) | not measured |
| **#4** | `7A360A6C3037` | 13017 | B: vel_gain 0.05 / vel_int 0.20 / curr_lim 45 | slightly rougher than #3 | 0 (unset) | not measured |
| **#5** | `8F350A6C3037` | 10439 | B: vel_gain 0.05 / vel_int 0.20 / curr_lim 45 | ⭐ as smooth as #3 | 0 (unset) | not measured |

### Factory defaults (common to groups A·B, besides gains)
- `controller.config.pos_gain` = 20
- `controller.config.vel_limit` = 30 turn/s
- `encoder.config.index_offset` = 0 (only #1 set to -2.142441)

## Key findings (2026-05-29)

1. **`phase_offset` differs on every motor** (22076 / 19916 / 21012 / 13017 / 10439). These are factory-measured per-unit magnet/winding values → **never copy/overwrite, never re-calibrate.** Different is normal.

2. **Factory gains split into two groups.** #1·#2 = group A (`0.10 / 1.0 / 60`), #3·#4·#5 = group B (`0.05 / 0.20 / 45`). All 5 are fw v0.6.5, yet they differ — SteadyWin appears to flash per-batch/per-unit values.

3. **Optimal `vel_gain` is per-motor.** #1 was smoother at 0.145, but #2 was actually rougher at 0.145. → **Blindly copying motor #1's gains is risky.** Compare and adopt per unit.

4. **Lower `vel_gain` = smoother at no-load** (0.05 > 0.10 > 0.145). Because cogging torque ripple correction is weaker. **But this is a no-load bench result** — if gain is too low, tracking may sag under pendulum load; re-verify after attaching.

5. **Same gains still feel different due to mechanical unit variation.** #3·#4·#5 are all factory 0.05, yet #3·#5 are smooth and #4 is slightly rougher. Pure mechanical difference (bearing/gear mesh/cogging phase) not explained by gains. → **If smoothness is top priority, selecting the smoothest unit among several** is valid. Current best: **#3, #5**.

## Smoothness ranking (no-load, 2026-05-29)

1. **#3 ≈ #5** (factory 0.05) — smoothest
2. **#4** (factory 0.05) — slightly rougher than #3/#5
3. **#1·#2** (factory 0.10 group) — not directly comparable (different gains). #1 is tuned to 0.145.

## Dual-motor setup — unit selection note

- Two motors planned for the ends of a 5m × 1.2m Mylar batten (see CLAUDE.md dual-motor checklist).
- **Pick a pair with similar smoothness/characteristics** to minimize asymmetric motion. Same gain group (B) + both smooth → **#3 + #5 combo** is the current candidate.
- `index_offset` (zero) and `TOGGLE_TURN` must be measured individually per motor (unit variation).
