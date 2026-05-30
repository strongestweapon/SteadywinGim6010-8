"""
waveforms.py — LFO 웨이브테이블 정의 (앱 미리보기 + ESP32 헤더 생성 공유)
===============================================================================
한 곳에서 파형을 정의해 두 소비처가 같은 모양을 쓰게 한다:
  - controller_app/main.py  : 미리보기 플롯
  - gen_wavetables.py       : ESP32 src/wavetables.h (C 배열) 생성

위상 규약 (★ 펌웨어 sinf(g_phase) 와 동일):
  phase=0 → value=0 (중심, 최대 속도), 상승. quarter 에서 +1(정점).
  즉 SIN 위상. vel_ff = amp·2πf·deriv 라서 deriv 테이블도 같이 만든다.

"그네(진자)" 정의:
  정규화 진자 운동방정식  θ̈ = -sin(θ),  θ(0)=θ₀, θ̇(0)=0  의 해를 θ₀ 로 정규화.
  θ₀ → 0°: 사인,  θ₀ 클수록 정점에서 머물고 중심을 빠르게 통과(그네 느낌).
  같은 amp·freq 라도 중심 통과 peak 속도가 사인보다 커서(θ₀↑) 진폭 클램프를
  그만큼 더 깎아야 함 → peak_vel/peak_acc 계수를 함께 반환.
"""
from __future__ import annotations
import numpy as np

WAVE_SINE = 0
WAVE_SWING = 1
DEFAULT_THETA0_DEG = 150.0   # 그네 진자 진폭(모양 결정). 취향껏 변경 가능.


def _pendulum_cos_phase(theta0_deg: float, n: int):
    """θ₀ 에서 놓은 진자 한 주기를 cos 위상(phase0=정점)으로 적분해 반환.
    value_cos[0]=+1(정점). RK4, scipy 불필요."""
    theta0 = np.deg2rad(theta0_deg)
    dt = 1e-4

    def deriv(s):
        th, om = s
        return np.array([om, -np.sin(th)])

    ts, ths = [0.0], [theta0]
    s = np.array([theta0, 0.0])
    t, prev_om, changes = 0.0, 0.0, 0
    while changes < 2 and t < 1000.0:
        k1 = deriv(s)
        k2 = deriv(s + 0.5 * dt * k1)
        k3 = deriv(s + 0.5 * dt * k2)
        k4 = deriv(s + dt * k3)
        s = s + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        t += dt
        om = s[1]
        if prev_om != 0.0 and np.sign(om) != np.sign(prev_om):
            changes += 1
        prev_om = om
        ts.append(t)
        ths.append(s[0])
    ts = np.array(ts)
    ths = np.array(ths)
    period = ts[-1]
    phase = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    theta_u = np.interp(phase / (2 * np.pi) * period, ts, ths)
    return theta_u / theta0  # value_cos (peak 1)


def wavetable(kind: int, n: int = 1024, theta0_deg: float = DEFAULT_THETA0_DEG):
    """SIN 위상 위치/속도 테이블 + peak 계수 반환.

    반환 dict:
      value : 위치 [-1,1], value[0]=0 상승 (SIN 위상)
      deriv : dvalue/dφ   (vel_ff = amp·2πf·deriv)
      peak_vel : max|deriv|        (사인=1.0 기준 속도 배수)
      peak_acc : max|d²value/dφ²|  (사인=1.0 기준 가속 배수)
    """
    phase = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    if kind == WAVE_SINE:
        value = np.sin(phase)
    else:
        # cos 위상 진자 → π/2 시프트로 SIN 위상화: value_sin(φ)=value_cos(φ-π/2)
        vc = _pendulum_cos_phase(theta0_deg, n)
        shift = n // 4  # π/2
        value = np.roll(vc, shift)
    deriv = np.gradient(value, phase, edge_order=2)
    d2 = np.gradient(deriv, phase, edge_order=2)
    return {
        "value": value,
        "deriv": deriv,
        "peak_vel": float(np.max(np.abs(deriv))),
        "peak_acc": float(np.max(np.abs(d2))),
    }
