"""
gen_wavetables.py — esp32t2can/src/wavetables.h (C 배열) 생성
===============================================================================
waveforms.py 의 정의로 SIN 위상 위치/속도 테이블을 만들어 ESP32 헤더로 굽는다.
펌웨어는 이 테이블을 lookup + morph(크로스페이드) 한다.

사용:
    python gen_wavetables.py                 # θ₀=150°, N=1024 기본
    python gen_wavetables.py --theta0 130 --n 512

⚠️ 보드 명령 아님 (순수 파일 생성). 헤더 갱신 후 펌웨어 재빌드/플래시는 별도(사용자 승인).
"""
from __future__ import annotations
import argparse
import numpy as np

import waveforms as wf

HEADER_PATH = "esp32t2can/src/wavetables.h"


def fmt_array(name: str, arr: np.ndarray, per_line: int = 8) -> str:
    lines = [f"static const float {name}[WT_N] = {{"]
    vals = [f"{v:.6f}f" for v in arr]
    for i in range(0, len(vals), per_line):
        lines.append("  " + ", ".join(vals[i:i + per_line]) + ",")
    lines.append("};")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--theta0", type=float, default=wf.DEFAULT_THETA0_DEG,
                    help="그네 진자 진폭(deg) — 모양 결정")
    ap.add_argument("--n", type=int, default=1024, help="테이블 길이 (2의 거듭제곱 권장)")
    ap.add_argument("--out", default=HEADER_PATH)
    args = ap.parse_args()

    sine = wf.wavetable(wf.WAVE_SINE, args.n)
    swing = wf.wavetable(wf.WAVE_SWING, args.n, args.theta0)

    out = []
    out.append("// === 자동 생성: gen_wavetables.py — 직접 수정 금지 ===")
    out.append(f"// 그네 진자 θ₀={args.theta0:.1f}°, N={args.n}")
    out.append("// SIN 위상: tbl[0]=0 상승, vel_ff = amp·2πf·deriv[i].")
    out.append("// 파형 0=SINE, 1=SWING. peak_vel/acc 는 진폭 클램프 derate 계수(사인=1.0).")
    out.append("#pragma once")
    out.append(f"#define WT_N {args.n}")
    out.append("")
    out.append("// ---- 위치 테이블 (정규화 [-1,1]) ----")
    out.append(fmt_array("WT_POS_SINE", sine["value"]))
    out.append(fmt_array("WT_POS_SWING", swing["value"]))
    out.append("")
    out.append("// ---- 속도 테이블 (dvalue/dφ) ----")
    out.append(fmt_array("WT_VEL_SINE", sine["deriv"]))
    out.append(fmt_array("WT_VEL_SWING", swing["deriv"]))
    out.append("")
    out.append("// ---- peak 계수 (vel_limit / accel_limit derate 용) ----")
    out.append(f"static const float WT_PEAKVEL_SINE  = {sine['peak_vel']:.6f}f;")
    out.append(f"static const float WT_PEAKVEL_SWING = {swing['peak_vel']:.6f}f;")
    out.append(f"static const float WT_PEAKACC_SINE  = {sine['peak_acc']:.6f}f;")
    out.append(f"static const float WT_PEAKACC_SWING = {swing['peak_acc']:.6f}f;")
    out.append("")

    text = "\n".join(out) + "\n"
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(text)

    print(f"생성: {args.out}  (N={args.n}, θ₀={args.theta0:.1f}°)")
    print(f"  SINE  peak_vel={sine['peak_vel']:.3f}  peak_acc={sine['peak_acc']:.3f}")
    print(f"  SWING peak_vel={swing['peak_vel']:.3f}  peak_acc={swing['peak_acc']:.3f}")
    print(f"  → 그네는 사인 대비 진폭을 vel {swing['peak_vel']:.2f}x / "
          f"acc {swing['peak_acc']/sine['peak_acc']:.2f}x 만큼 더 깎아야 안전.")


if __name__ == "__main__":
    main()
