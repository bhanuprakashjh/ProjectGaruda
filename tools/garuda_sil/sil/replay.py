"""replay.py — drive the twin from a recorded bench session and score the match.

Usage:
  .../garuda_debug/.venv/bin/python sil/replay.py <session_dir_or_telemetry.csv> [--sim-seconds N]

Replays the recorded pot (throttle ADC) and Vbus traces through the SIL twin,
time-aligned at the bench's ALIGN entry (twin start command is issued ARM_TIME
=0.5 s earlier so the state machines line up). Scores:
  - state-edge times (ALIGN/OL_RAMP/MORPH/CL entries), twin vs bench
  - CL-entry eRPM (the hand-off seed truthfulness)
  - idle eRPM (mean over the first second of CL at idle pot)
  - eRPM trajectory RMSE/correlation over the whole replay window
"""
from __future__ import annotations
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from garuda_sil import Sim, Plant2810, STATES, PWM_HZ, _adc

ARM_TIME_S = 0.5


def load_session(path):
    if os.path.isdir(path):
        path = os.path.join(path, "telemetry.csv")
    rows = list(csv.DictReader(open(path)))
    t0 = float(rows[0]["t"])
    out = []
    for r in rows:
        name = r["state_name"]
        if name == "CL":
            name = "CLOSED_LOOP"            # GUI short name -> enum name
        out.append(dict(
            t=float(r["t"]) - t0,
            state=name,
            pot=int(float(r["throttle"])),
            vbus=float(r["vbus_V"]),
            erpm=float(r["eRPM"]),
        ))
    return out


def state_edges(samples, key_t="t", key_s="state"):
    edges, last = [], None
    for s in samples:
        if s[key_s] != last:
            edges.append((s[key_t], s[key_s]))
            last = s[key_s]
    return edges


def main():
    path = sys.argv[1]
    sim_seconds = None
    if "--sim-seconds" in sys.argv:
        sim_seconds = float(sys.argv[sys.argv.index("--sim-seconds") + 1])

    bench = load_session(path)
    b_edges = state_edges(bench)
    t_align = next((t for t, s in b_edges if s == "ALIGN"), None)
    if t_align is None:
        print("no ALIGN entry in session — cannot align traces"); return 1
    t_end = bench[-1]["t"] if sim_seconds is None else min(bench[-1]["t"], sim_seconds)

    sim = Sim(plant=Plant2810(vbus=bench[0]["vbus"]), pot=bench[0]["pot"])
    started = False
    t_start_cmd = max(t_align - ARM_TIME_S, 0.0)

    twin_samples = []
    n_ticks = int(t_end * PWM_HZ)
    ridx = 0
    next_sample_tick = 0
    for k in range(n_ticks):
        t = k / PWM_HZ
        # advance recorded inputs
        while ridx + 1 < len(bench) and bench[ridx + 1]["t"] <= t:
            ridx += 1
            sim.pot = bench[ridx]["pot"]
            sim.plant.vbus = bench[ridx]["vbus"]
            sim.vbus_counts = _adc(bench[ridx]["vbus"])
        if not started and t >= t_start_cmd:
            sim.cmd_start(); started = True
        sim.step_tick()
        if k >= next_sample_tick:                       # ~30 Hz like the bench
            s = sim.snapshot()
            twin_samples.append(dict(t=t, state=s["state"],
                                     erpm=s["plant_erpm"], fw_erpm=s["fw_erpm"]))
            next_sample_tick += int(PWM_HZ / 30)
        if STATES[sim.lib.sil_state()] == "FAULT":
            print(f"twin FAULTED at t={t:.2f}s: {sim.fault_name()}")
            break

    # ── scoring ─────────────────────────────────────────────────────────
    t_edges = state_edges(twin_samples)
    print("=== state edges (s) ===")
    print(f"{'state':12s} {'bench':>8s} {'twin':>8s} {'delta':>8s}")
    for name in ("ALIGN", "OL_RAMP", "MORPH", "CLOSED_LOOP"):
        tb = next((t for t, s in b_edges if s == name), None)
        tw = next((t for t, s in t_edges if s == name), None)
        if tb is not None and tw is not None:
            print(f"{name:12s} {tb:8.2f} {tw:8.2f} {tw - tb:+8.2f}")
        else:
            print(f"{name:12s} {str(tb):>8s} {str(tw):>8s}      n/a")

    # CL-entry eRPM
    def entry_erpm(samples, edges, key):
        t_cl = next((t for t, s in edges if s == "CLOSED_LOOP"), None)
        if t_cl is None:
            return None
        for s in samples:
            if s["t"] >= t_cl:
                return s[key]
        return None
    e_b = entry_erpm(bench, b_edges, "erpm")
    e_t = entry_erpm(twin_samples, t_edges, "erpm")
    if e_b and e_t:
        print(f"\nCL entry eRPM: bench {e_b:.0f}  twin {e_t:.0f}  "
              f"({100 * (e_t / e_b - 1):+.1f}%)")

    # trajectory RMSE over matched times (only while both in CL)
    import bisect
    tb_list = [s["t"] for s in twin_samples]
    err2, n, pairs = 0.0, 0, []
    for s in bench:
        if s["state"] != "CLOSED_LOOP" or s["t"] > t_end:
            continue
        i = bisect.bisect_left(tb_list, s["t"])
        if i >= len(twin_samples):
            break
        tw = twin_samples[i]
        if tw["state"] != "CLOSED_LOOP":
            continue
        err2 += (tw["erpm"] - s["erpm"]) ** 2
        n += 1
        pairs.append((s["erpm"], tw["erpm"]))
    if n:
        rmse = math.sqrt(err2 / n)
        mean_b = sum(p[0] for p in pairs) / n
        print(f"CL eRPM trajectory: {n} matched samples, RMSE {rmse:.0f} eRPM "
              f"({100 * rmse / mean_b:.1f}% of mean {mean_b:.0f})")
        print(f"  bench max {max(p[0] for p in pairs):.0f}   "
              f"twin max {max(p[1] for p in pairs):.0f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
