"""experiment_pll_sweep.py — twin study #3: how aggressive can the PLL
blind schedule be? Sweep PLL_START_ERPM0 x ACCEL over 12 rotor angles each.
Variant .so files are built by sweep_build.sh as libgaruda_sil_pll_<tag>.so.

Goal: T-motor feel — shortest time-to-sync/-idle (also = shortest 22A
slip-current window on the bench) that still starts 12/12.
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from garuda_sil import Sim, Plant2810, STATES, PWM_HZ
from experiment_pll import one_start
import experiment_pll

HERE = os.path.dirname(os.path.abspath(__file__))

COMBOS = [  # (erpm0, accel)
    (300, 4000),      # bench-proven baseline
    (300, 8000),
    (300, 16000),
    (1000, 8000),
    (1000, 16000),
    (1500, 16000),
    (1500, 24000),
    (1500, 20000),
    (2000, 16000),
    (2500, 12000),
    (300, 20000),
    (300, 24000),
    (300, 32000),
]


def main():
    for erpm0, accel in COMBOS:
        tag = f"e{erpm0}_a{accel}"
        so = os.path.join(HERE, "..", f"libgaruda_sil_pll_{tag}.so")
        if not os.path.exists(so):
            print(f"{tag}: MISSING .so, skip"); continue
        experiment_pll.SO = so
        oks, syncs, idles = 0, [], []
        fails = []
        for a in range(0, 360, 30):
            r = one_start(a, deadline_s=5.0)
            if r["ok"]:
                oks += 1; syncs.append(r["t_sync"]); idles.append(r["idle"])
            else:
                fails.append((a, r["why"]))
        line = f"e0={erpm0:5d} accel={accel:6d}: {oks:2d}/12"
        if syncs:
            line += (f"  t_sync {min(syncs):.2f}-{max(syncs):.2f}s"
                     f"  idle {min(idles):.0f}-{max(idles):.0f}")
        print(line)
        for a, why in fails[:3]:
            print(f"      θ0={a}° {why}")


if __name__ == "__main__":
    main()
