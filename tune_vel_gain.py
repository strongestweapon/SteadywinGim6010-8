"""
tune_vel_gain.py — 매뉴얼 P29 step 3 한 step 테스트

지정한 vel_gain 값으로 모터를 속도제어 모드에서 회전시키고 정지.
관찰 후 다음 값으로 재호출. 매 호출 = 한 step.

매뉴얼 P29 권장 진행
--------------------
1) 거칠면 vel_gain 감소 → 매끄러워질 때까지
2) 매끄러우면 ×1.3 씩 증가 → 눈에 띄는 jitter 가 나타날 때까지
3) jitter 시점에서 ×0.5 감소해서 안정화

사용
----
    python tune_vel_gain.py --vel-gain 0.13
    python tune_vel_gain.py --vel-gain 0.17 --target-vel 1.5 --duration 8

옵션
----
    --vel-gain     설정할 vel_gain (필수)
    --target-vel   회전 속도 (모터축 turn/s), 기본 1.0
    --duration     관찰 시간 (초), 기본 5
    --vel-ramp     vel_ramp_rate (turn/s²), 기본 2.0
    --vel-limit    안전 한계 (turn/s), 기본 5.0
    --current-lim  전류 한계 (A), 기본 10

주의
----
- vel_gain 변경은 휘발성 (RAM only). save_configuration() 호출 안 함.
- 종료 후에도 RAM 의 vel_gain 은 마지막 값 유지. 재부팅하면 공장값(0.10) 복귀.
- 매뉴얼 P29 step 3 은 vel_integrator_gain = 0 상태에서 진행. 시작 전 별도 세팅.
"""
from __future__ import annotations

import argparse
import time

import motor_helpers as mh


VEL_GAIN_MIN = 0.001
VEL_GAIN_MAX = 2.0


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--vel-gain", type=float, required=True,
                   help=f"설정할 vel_gain 값 ({VEL_GAIN_MIN} ~ {VEL_GAIN_MAX})")
    p.add_argument("--target-vel", type=float, default=1.0,
                   help="회전 속도 (모터축 turn/s) — 기본 1.0")
    p.add_argument("--duration", type=float, default=5.0,
                   help="관찰 시간 (초) — 기본 5")
    p.add_argument("--vel-ramp", type=float, default=2.0,
                   help="vel_ramp_rate (turn/s²) — 가속률, 기본 2.0")
    p.add_argument("--vel-limit", type=float, default=5.0,
                   help="vel_limit (turn/s) — 안전 한계, 기본 5.0")
    p.add_argument("--current-lim", type=float, default=10.0,
                   help="current_lim (A) — 기본 10A")
    args = p.parse_args()

    # 안전 cap
    if args.vel_gain < VEL_GAIN_MIN or args.vel_gain > VEL_GAIN_MAX:
        raise SystemExit(
            f"[거부] vel_gain={args.vel_gain} 가 안전 범위({VEL_GAIN_MIN} ~ {VEL_GAIN_MAX}) 밖."
        )

    odrv = mh.connect()
    axis = odrv.axis0

    # 시작 값 백업 (참고용)
    initial_gain = float(axis.controller.config.vel_gain)
    initial_integrator = float(axis.controller.config.vel_integrator_gain)
    print(f"\n[현재값] vel_gain={initial_gain:.4f}, vel_integrator_gain={initial_integrator:.4f}")
    if initial_integrator != 0:
        print(f"  ⚠ vel_integrator_gain != 0 — 매뉴얼 P29 step 3 은 적분항 0 상태에서 튜닝.")

    # vel_gain 변경 (RAM only)
    axis.controller.config.vel_gain = args.vel_gain
    actual = float(axis.controller.config.vel_gain)
    print(f"[변경] vel_gain → {actual:.4f}")

    print(f"[운용] target_vel={args.target_vel} turn/s (출력축 {args.target_vel*360/8:.1f}°/s), "
          f"duration={args.duration}s\n")

    mh.enter_velocity_mode(
        axis,
        current_lim=args.current_lim,
        vel_limit=args.vel_limit,
        vel_ramp_rate=args.vel_ramp,
    )

    try:
        # 회전 시작
        axis.controller.input_vel = args.target_vel
        print(f"  input_vel = {args.target_vel} → 가속 중...")

        # 가속 안정화 대기 (vel_ramp_rate 기준)
        accel_s = args.target_vel / args.vel_ramp + 0.5
        time.sleep(accel_s)

        # 관찰 구간 — 50Hz 샘플링 (20ms 간격) → 진동 / jitter 포착
        sample_dt = 0.02
        print(f"  관찰 ({args.duration}s, 샘플레이트 {1/sample_dt:.0f}Hz)...")
        t0 = time.time()
        samples = []  # (t, vel, iq)
        next_t = 0.0
        while True:
            t = time.time() - t0
            if t >= args.duration:
                break
            if t >= next_t:
                try:
                    vel = float(axis.encoder.vel_estimate)
                    iq = float(axis.motor.current_control.Iq_measured)
                    samples.append((t, vel, iq))
                except Exception as e:
                    print(f"    [샘플 실패] {e}")
                next_t += sample_dt
            # busy-wait 보다는 짧게 sleep — 너무 짧으면 USB 부하
            time.sleep(0.002)

        # 통계 + 자동 판정
        if samples:
            ts = [s[0] for s in samples]
            vels = [s[1] for s in samples]
            iqs = [s[2] for s in samples]
            n = len(samples)
            vel_avg = sum(vels) / n
            vel_var = sum((v - vel_avg) ** 2 for v in vels) / n
            vel_std = vel_var ** 0.5
            vel_pp = max(vels) - min(vels)
            iq_avg = sum(iqs) / n
            iq_var = sum((i - iq_avg) ** 2 for i in iqs) / n
            iq_std = iq_var ** 0.5
            iq_pp = max(iqs) - min(iqs)

            # 1차 차분 RMS — 시간 단위 jitter (sample-to-sample 변화량)
            d_vel = [vels[i+1] - vels[i] for i in range(n-1)]
            d_vel_rms = (sum(d**2 for d in d_vel) / len(d_vel)) ** 0.5 if d_vel else 0.0
            d_iq = [iqs[i+1] - iqs[i] for i in range(n-1)]
            d_iq_rms = (sum(d**2 for d in d_iq) / len(d_iq)) ** 0.5 if d_iq else 0.0

            err = abs(vel_avg - args.target_vel)
            cov = vel_std / vel_avg * 100 if vel_avg != 0 else 0.0  # 변동계수 %

            print(f"\n  [통계 n={n}]")
            print(f"    vel = {vel_avg:+.4f} ± {vel_std:.4f} turn/s "
                  f"(peak-peak {vel_pp:.4f}, CoV {cov:.1f}%)")
            print(f"    Iq  = {iq_avg:+.3f} ± {iq_std:.3f} A (peak-peak {iq_pp:.3f})")
            print(f"    Δvel RMS (jitter, {sample_dt*1000:.0f}ms step) = {d_vel_rms:.4f} turn/s")
            print(f"    ΔIq  RMS = {d_iq_rms:.4f} A")
            print(f"    정상상태 오차 = {err:.4f} turn/s "
                  f"({err / args.target_vel * 100:.1f}%)")

            # 자동 판정 — 임계값은 관찰 결과 누적해서 조정 예정
            print(f"\n  [자동 판정]")
            if cov > 25.0:
                print(f"    → vel CoV {cov:.1f}% 가 25% 초과: 매우 거침 (rough)")
            elif cov > 10.0:
                print(f"    → vel CoV {cov:.1f}% 가 10~25%: 거침 (rough)")
            elif cov > 4.0:
                print(f"    → vel CoV {cov:.1f}% 가 4~10%: 보통 (moderate)")
            else:
                print(f"    → vel CoV {cov:.1f}% < 4%: 매끄러움 (smooth)")
            if d_iq_rms > 0.5:
                print(f"    → Iq jitter {d_iq_rms:.3f}A > 0.5A: PID ripple 의심")
            elif d_iq_rms > 0.2:
                print(f"    → Iq jitter {d_iq_rms:.3f}A (0.2~0.5): 약한 ripple")
            else:
                print(f"    → Iq jitter {d_iq_rms:.3f}A < 0.2: 안정")
    finally:
        print("\n[정지] ramp → IDLE")
        mh.safe_stop(axis, ramp_s=0.8)
        print(f"[기록] vel_gain = {float(axis.controller.config.vel_gain):.4f} (RAM, 재부팅 시 복귀)")


if __name__ == "__main__":
    main()
