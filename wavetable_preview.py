"""
wavetable_preview.py — 사인 vs 그네(진자) 파형 시각 비교
===============================================================================
목적:
  ESP32 로컬 오실레이터에 넣을 웨이브테이블 후보를 펌웨어/보드 손대기 전에
  눈으로 먼저 확인한다. 보드 명령 전혀 없음 (순수 계산 + 그래프).

"그네 파형" 의 정의 (CLAUDE.md 다음 세션 계획 참고):
  큰 진폭 진자는 사인이 아니다. 정규화 진자 운동방정식
        θ̈ = -sin(θ),   θ(0)=θ₀,  θ̇(0)=0   (θ₀ 에서 정지 상태로 놓음)
  의 해 θ(t) 를 θ₀ 로 정규화한 것이 그네 파형.
    - θ₀ → 0°   : 순수 사인 (소진폭 극한)
    - θ₀ 커질수록: 끝점(정점)에서 오래 머물고 중심(바닥)을 빠르게 휙
  shape_param = θ₀ 가 "그네스러움" 노브. 출력 진폭 amp_deg 는 별개(스케일만).

속도(vel_ff) 의미:
  진자는 같은 진폭·주파수라도 중심 통과 peak 속도가 사인보다 높다(θ₀ 클수록 ↑).
  → ESP32 재클램프(35.8/f, 사인 가정)를 그대로 쓰면 안 되고, 테이블의 실제
    peak|dvalue/dφ| 로 클램프를 스케일해야 함. 아래 그래프 하단 + 콘솔 출력 참고.
"""
from __future__ import annotations
import warnings
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

# 한글 폰트 (Windows 맑은 고딕) — 없으면 기본 폰트로 폴백
for _f in ("Malgun Gothic", "AppleGothic", "NanumGothic"):
    try:
        matplotlib.rcParams["font.family"] = _f
        break
    except Exception:
        continue
matplotlib.rcParams["axes.unicode_minus"] = False  # 마이너스 깨짐 방지
warnings.filterwarnings("ignore", message="Glyph")


def pendulum_wavetable(theta0_deg: float, n: int = 1024):
    """θ₀(deg)에서 놓은 진자의 한 주기 파형을 위상 격자 N개로 반환.

    반환:
      phase : [0, 2π) 균일 위상 (rad)
      value : 정규화 위치 θ/θ₀  (위상 0 에서 +1 = 정점, cos 위상)
      dval  : dvalue/dφ (속도 모양, vel_ff 비례)
    수치적분 RK4, scipy 불필요.
    """
    theta0 = np.deg2rad(theta0_deg)
    dt = 1e-4

    # θ̈ = -sin θ. 정점(θ=θ₀, ω=0)에서 출발 → cos 위상으로 자연 정렬.
    def deriv(s):
        th, om = s
        return np.array([om, -np.sin(th)])

    ts, ths = [0.0], [theta0]
    s = np.array([theta0, 0.0])
    t = 0.0
    sign_changes = 0
    prev_om = 0.0
    # ω 부호변화 2회 = 한 주기 (정점→반대정점→정점)
    while sign_changes < 2 and t < 1000.0:
        k1 = deriv(s)
        k2 = deriv(s + 0.5 * dt * k1)
        k3 = deriv(s + 0.5 * dt * k2)
        k4 = deriv(s + dt * k3)
        s = s + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        t += dt
        om = s[1]
        if prev_om != 0.0 and np.sign(om) != np.sign(prev_om):
            sign_changes += 1
        prev_om = om
        ts.append(t)
        ths.append(s[0])

    ts = np.array(ts)
    ths = np.array(ths)
    period = ts[-1]  # 마지막 부호변화 = 한 주기

    # 균일 위상 격자로 리샘플
    phase = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    t_uniform = phase / (2 * np.pi) * period
    theta_u = np.interp(t_uniform, ts, ths)
    value = theta_u / theta0

    # dvalue/dφ (균일 격자, 원형이라 wrap 고려)
    dval = np.gradient(value, phase, edge_order=2)
    return phase, value, dval


def main():
    n = 1024
    phase = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    sine = np.cos(phase)          # 진자가 cos 위상이라 사인 기준도 cos 로 정렬
    sine_d = -np.sin(phase)       # dcos/dφ

    theta0_list = [30, 90, 130, 160]
    waves = {t0: pendulum_wavetable(t0, n) for t0 in theta0_list}

    print("=== peak 속도 (|dvalue/dφ|) — 클램프 스케일 기준 ===")
    print(f"  {'파형':<18} peak|dval|   사인대비")
    print(f"  {'사인(cos)':<18} {np.max(np.abs(sine_d)):.3f}        1.00x")
    for t0 in theta0_list:
        _, _, dv = waves[t0]
        pk = np.max(np.abs(dv))
        print(f"  {'진자 θ₀=' + str(t0) + '°':<18} {pk:.3f}        {pk:.2f}x")
    print("\n  → 같은 amp·freq 라도 진자는 중심 통과가 빨라 peak 속도가 큼.")
    print("    ESP32 진폭 클램프를 이 배수만큼 더 깎아야 vel_limit 안 넘음.\n")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    x = phase / (2 * np.pi)  # 주기 분율 0..1

    ax1.plot(x, sine, "k--", lw=2.2, label="사인 (cos) — 둥둥 떠다님")
    for t0 in theta0_list:
        _, v, _ = waves[t0]
        ax1.plot(x, v, lw=1.8, label=f"진자 θ₀={t0}°")
    ax1.set_ylabel("위치  value = θ/θ₀")
    ax1.set_title("위치 파형 — 진자는 정점(±1)에서 머물고 중심(0)을 빠르게 지나감")
    ax1.grid(alpha=0.3)
    ax1.legend(loc="upper right", fontsize=9)
    ax1.axhline(0, color="gray", lw=0.6)

    ax2.plot(x, sine_d, "k--", lw=2.2, label="사인 속도")
    for t0 in theta0_list:
        _, _, dv = waves[t0]
        ax2.plot(x, dv, lw=1.8, label=f"진자 θ₀={t0}°")
    ax2.set_ylabel("속도 모양  dvalue/dφ")
    ax2.set_xlabel("주기 분율 (phase / 2π)")
    ax2.set_title("속도 프로파일 — 진자는 중심 통과 peak 속도가 높음 (θ₀ 클수록 ↑)")
    ax2.grid(alpha=0.3)
    ax2.legend(loc="upper right", fontsize=9)
    ax2.axhline(0, color="gray", lw=0.6)

    fig.tight_layout()
    out = "wavetable_preview.png"
    fig.savefig(out, dpi=110)
    print(f"그래프 저장: {out}")
    try:
        plt.show()
    except Exception:
        pass


if __name__ == "__main__":
    main()
