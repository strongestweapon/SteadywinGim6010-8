"""
tune_pos_gain.py — 매뉴얼 P29 step 4 한 step 테스트

지정한 pos_gain 값으로 위치제어 모드에서 step 명령 응답을 측정.
overshoot / 정착시간 / 정상상태 오차를 자동 계산.

매뉴얼 P29 권장 진행
--------------------
1) 거칠면 pos_gain 감소 → 매끄러워질 때까지
2) 매끄러우면 ×1.3 씩 증가 → 눈에 띄는 overshoot 가 생길 때까지
3) overshoot 발생 시점부터 천천히 감소시켜 overshoot 가 사라질 때까지

사용
----
    python tune_pos_gain.py --pos-gain 26
    python tune_pos_gain.py --pos-gain 30 --step 0.05 --duration 2.0

옵션
----
    --pos-gain     설정할 pos_gain (필수)
    --step         step 크기 (motor turn), 기본 0.05 (= 18°모터 = 2.25°출력)
    --duration     관찰 시간 (초), 기본 1.5
    --vel-limit    안전 한계 (turn/s), 기본 5.0
    --current-lim  전류 한계 (A), 기본 10

주의
----
- 변경 휘발성. save_configuration() 호출 없음.
- INPUT_MODE_PASSTHROUGH 로 step 을 즉시 명령 — 컨트롤러 raw 응답 관찰용.
- 실운용은 POS_FILTER / TRAP_TRAJ 모드 — 본 테스트는 튜닝 전용.
- vel_gain 은 Step 3 에서 정한 값 유지 (이 스크립트는 변경 안 함).
"""
from __future__ import annotations

import argparse
import time

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_POSITION_CONTROL,
    INPUT_MODE_PASSTHROUGH,
)

import motor_helpers as mh


POS_GAIN_MIN = 1.0
POS_GAIN_MAX = 200.0


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--pos-gain", type=float, required=True,
                   help=f"설정할 pos_gain 값 ({POS_GAIN_MIN} ~ {POS_GAIN_MAX})")
    p.add_argument("--step", type=float, default=0.05,
                   help="step 크기 (motor turn) — 기본 0.05 (출력축 2.25°)")
    p.add_argument("--duration", type=float, default=1.5,
                   help="관찰 시간 (초) — 기본 1.5")
    p.add_argument("--vel-limit", type=float, default=5.0,
                   help="vel_limit (turn/s) — 안전 한계, 기본 5.0")
    p.add_argument("--current-lim", type=float, default=10.0,
                   help="current_lim (A) — 기본 10A")
    args = p.parse_args()

    if args.pos_gain < POS_GAIN_MIN or args.pos_gain > POS_GAIN_MAX:
        raise SystemExit(
            f"[거부] pos_gain={args.pos_gain} 가 안전 범위({POS_GAIN_MIN} ~ {POS_GAIN_MAX}) 밖."
        )

    odrv = mh.connect()
    axis = odrv.axis0

    # 현재 값 dump
    initial_pos_gain = float(axis.controller.config.pos_gain)
    vel_gain = float(axis.controller.config.vel_gain)
    vel_integrator = float(axis.controller.config.vel_integrator_gain)
    print(f"\n[현재값] pos_gain={initial_pos_gain:.4f}, vel_gain={vel_gain:.4f}, "
          f"vel_integrator_gain={vel_integrator:.4f}")
    if vel_integrator != 0:
        print(f"  ⚠ vel_integrator_gain != 0 — 매뉴얼 P29 절차는 적분항 0 상태에서 진행.")

    # pos_gain 변경
    axis.controller.config.pos_gain = args.pos_gain
    actual = float(axis.controller.config.pos_gain)
    print(f"[변경] pos_gain → {actual:.4f}")

    # 위치제어 + PASSTHROUGH 모드 직접 설정 (motor_helpers.enter_position_mode 는
    # POS_FILTER 강제라서 step 응답 관찰에 부적합)
    mh.ensure_calibrated(axis)
    mh.apply_safety(axis, args.current_lim, args.vel_limit)
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

    # 초기 위치 = 현재 실측. 점프 방지.
    start_pos = float(axis.encoder.pos_estimate)
    axis.controller.input_pos = start_pos
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.3)
    if int(axis.current_state) != int(AXIS_STATE_CLOSED_LOOP_CONTROL):
        raise RuntimeError(f"CLOSED_LOOP 진입 실패. state={int(axis.current_state)}")

    target_pos = start_pos + args.step
    print(f"[운용] start_pos={start_pos:+.4f}, target={target_pos:+.4f}, "
          f"step={args.step:+.4f} turn ({args.step*360:.1f}° 모터, {args.step*45:.1f}° 출력)")

    try:
        # 50Hz 샘플링
        sample_dt = 0.02
        samples = []  # (t, pos, vel, iq)

        # 0.3s 베이스라인 (step 전)
        baseline_s = 0.3
        t0 = time.time()
        next_t = 0.0
        while True:
            t = time.time() - t0
            if t >= baseline_s:
                break
            if t >= next_t:
                try:
                    samples.append((
                        t,
                        float(axis.encoder.pos_estimate),
                        float(axis.encoder.vel_estimate),
                        float(axis.motor.current_control.Iq_measured),
                    ))
                except Exception:
                    pass
                next_t += sample_dt
            time.sleep(0.002)

        # step 명령
        step_time = time.time() - t0
        axis.controller.input_pos = target_pos
        print(f"  step at t={step_time:.3f}s → input_pos={target_pos:+.4f}")

        # 응답 관찰
        end_time = baseline_s + args.duration
        while True:
            t = time.time() - t0
            if t >= end_time:
                break
            if t >= next_t:
                try:
                    samples.append((
                        t,
                        float(axis.encoder.pos_estimate),
                        float(axis.encoder.vel_estimate),
                        float(axis.motor.current_control.Iq_measured),
                    ))
                except Exception:
                    pass
                next_t += sample_dt
            time.sleep(0.002)

        # 분석
        print(f"\n  [분석 n={len(samples)}]")
        if not samples:
            return

        # step 전후 분리
        pre_samples = [s for s in samples if s[0] < step_time]
        post_samples = [s for s in samples if s[0] >= step_time]

        # 정상상태 = 마지막 0.2s 평균
        if args.duration > 0.3:
            tail_t = end_time - 0.2
            tail = [s for s in post_samples if s[0] >= tail_t]
        else:
            tail = post_samples[-10:]
        if not tail:
            tail = post_samples[-10:]
        ss_pos = sum(s[1] for s in tail) / len(tail)
        ss_err = abs(ss_pos - target_pos)

        # overshoot = max(pos) - target, step 방향 기준
        if args.step > 0:
            peak_pos = max(s[1] for s in post_samples)
            overshoot = max(0.0, peak_pos - target_pos)
        else:
            peak_pos = min(s[1] for s in post_samples)
            overshoot = max(0.0, target_pos - peak_pos)
        overshoot_pct = overshoot / abs(args.step) * 100 if args.step != 0 else 0.0

        # 정착시간 = step 후 ±5% step 안에 들어와서 유지되는 시점
        settle_band = abs(args.step) * 0.05
        settle_t = None
        for i, s in enumerate(post_samples):
            if abs(s[1] - target_pos) <= settle_band:
                # 뒤로 0.2s 동안 밴드 안에 머무는지 확인
                stable = True
                for sj in post_samples[i:]:
                    if sj[0] - s[0] > 0.2:
                        break
                    if abs(sj[1] - target_pos) > settle_band:
                        stable = False
                        break
                if stable:
                    settle_t = s[0] - step_time
                    break

        # vel 최대값 (slew rate 추적)
        max_vel = max(abs(s[2]) for s in post_samples)
        max_iq = max(abs(s[3]) for s in post_samples)

        # 진동 횟수 = target 통과 횟수 (sign change)
        crossings = 0
        prev_err = post_samples[0][1] - target_pos
        for s in post_samples[1:]:
            err = s[1] - target_pos
            if prev_err * err < 0:
                crossings += 1
            prev_err = err

        print(f"    target            = {target_pos:+.4f}")
        print(f"    정상상태 pos      = {ss_pos:+.4f}  (오차 {ss_err:.5f} turn = "
              f"{ss_err*360:.3f}° 모터)")
        print(f"    peak pos          = {peak_pos:+.4f}")
        print(f"    overshoot         = {overshoot:.5f} turn ({overshoot_pct:.1f}% of step)")
        print(f"    정착시간 (±5%)    = {settle_t:.3f}s" if settle_t is not None else
              f"    정착시간 (±5%)    = (구간 내 미정착)")
        print(f"    최대 vel          = {max_vel:.3f} turn/s")
        print(f"    최대 Iq           = {max_iq:.3f} A")
        print(f"    target 교차 횟수  = {crossings} (0=overshoot 없음, ≥1=진동)")

        # 자동 판정
        print(f"\n  [자동 판정]")
        if overshoot_pct > 30:
            print(f"    → overshoot {overshoot_pct:.1f}% > 30%: 심함 — pos_gain 감소 필요")
        elif overshoot_pct > 10:
            print(f"    → overshoot {overshoot_pct:.1f}% (10~30%): 눈에 띄는 overshoot — 매뉴얼 step 4.3 영역")
        elif overshoot_pct > 2:
            print(f"    → overshoot {overshoot_pct:.1f}% (2~10%): 약함 — sweet spot 근처")
        else:
            print(f"    → overshoot {overshoot_pct:.1f}% < 2%: 거의 없음 — pos_gain 더 올려 볼 수 있음")
        if crossings > 2:
            print(f"    → {crossings}회 진동 — 명백한 oscillation, 감소 필요")
    finally:
        print("\n[정지] 현재 위치 hold → IDLE")
        try:
            axis.controller.input_pos = float(axis.encoder.pos_estimate)
            time.sleep(0.3)
            axis.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass
        print(f"[기록] pos_gain = {float(axis.controller.config.pos_gain):.4f} "
              f"(RAM, 재부팅 시 공장값 {initial_pos_gain:.4f} 복귀)")


if __name__ == "__main__":
    main()
