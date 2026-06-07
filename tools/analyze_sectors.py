#!/usr/bin/env python3
"""Per-sector ZC / CPU analysis from a session CSV (step6 or GUI bundle).

Turns the raw miss_s0..5 (cumulative per-sector guess counters), hwzc_zc/miss
(aggregate measured vs guess), eRPM, duty and cpu_load_pct columns into a
speed-binned breakdown so we can SEE, from data not visuals:

  * which of the 6 commutation sectors carry the guesses,
  * the EVEN(0,2,4) vs ODD(1,3,5) split == the two ZC polarities,
  * how the split scales with eRPM (idle -> top),
  * aggregate measured-vs-guess and CPU load per speed band.

Usage:  python3 tools/analyze_sectors.py sessions/session_YYYYMMDD_HHMMSS.csv
        python3 tools/analyze_sectors.py tools/garuda_debug/sessions/gui_*/telemetry.csv
"""
import csv
import sys


def num(r, k, d=0.0):
    try:
        return float(r[k])
    except (KeyError, ValueError, TypeError):
        return d


def main(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        print("(empty CSV)")
        return
    have_sectors = all(f"miss_s{i}" in rows[0] for i in range(6))
    if not have_sectors:
        print("CSV has no miss_s0..5 columns — needs a run with the 242B "
              "telemetry build. Columns present:",
              ", ".join(rows[0].keys()))
        return

    # eRPM bands (electrical). Idle ~14k, top ~232k on 2810@24V.
    bands = [(0, 20_000), (20_000, 50_000), (50_000, 90_000),
             (90_000, 130_000), (130_000, 180_000), (180_000, 999_999)]
    agg = {b: {"sect": [0]*6, "dacc": 0, "dmiss": 0, "drej": 0,
               "cpu": [], "duty": [], "n": 0, "dt": 0.0,
               "fmin": [], "fmax": [], "zcth": []} for b in bands}
    have_offc = "fall_off_min" in rows[0]

    def band_of(erpm):
        for b in bands:
            if b[0] <= erpm < b[1]:
                return b
        return bands[-1]

    prev = None
    for r in rows:
        erpm = num(r, "eRPM")
        b = band_of(erpm)
        if prev is not None:
            dt = num(r, "t") - num(prev, "t")
            if 0 < dt < 1.0:                       # ignore gaps / RUN restarts
                for i in range(6):
                    d = (int(num(r, f"miss_s{i}")) -
                         int(num(prev, f"miss_s{i}"))) & 0xFFFF
                    agg[b]["sect"][i] += d
                agg[b]["dacc"]  += max(0, int(num(r, "hwzc_zc"))   - int(num(prev, "hwzc_zc")))
                agg[b]["dmiss"] += max(0, int(num(r, "hwzc_miss")) - int(num(prev, "hwzc_miss")))
                agg[b]["drej"]  += max(0, int(num(r, "hwzc_reject"))- int(num(prev, "hwzc_reject")))
                agg[b]["dt"]    += dt
        agg[b]["cpu"].append(num(r, "cpu_load_pct"))
        agg[b]["duty"].append(num(r, "duty"))
        # falling OFF-center envelope (blank cells = no falling-WATCHING samples)
        if r.get("fall_off_min") not in (None, "", "None"):
            agg[b]["fmin"].append(num(r, "fall_off_min"))
            agg[b]["fmax"].append(num(r, "fall_off_max"))
        if r.get("zc_thresh") not in (None, "", "None"):
            agg[b]["zcth"].append(num(r, "zc_thresh"))
        agg[b]["n"] += 1
        prev = r

    print(f"\nPER-SECTOR ZC ANALYSIS  —  {path}")
    print(f"{len(rows)} samples\n")
    hdr = (f"{'eRPM band':>16} {'duty%':>5} {'samp':>5} "
           f"{'S0':>6} {'S1':>6} {'S2':>6} {'S3':>6} {'S4':>6} {'S5':>6}  "
           f"{'EVEN':>7} {'ODD':>7}  {'meas%':>6} {'rej%':>5} {'cpu%':>5}")
    print(hdr)
    print("-" * len(hdr))
    for b in bands:
        a = agg[b]
        if a["n"] == 0:
            continue
        sect = a["sect"]
        dt = a["dt"] if a["dt"] > 0 else 1.0
        rate = [s / dt for s in sect]                  # guesses/s per sector
        even = rate[0] + rate[2] + rate[4]
        odd  = rate[1] + rate[3] + rate[5]
        tot = a["dacc"] + a["dmiss"]
        meas = 100.0 * a["dacc"] / tot if tot else 0.0
        rej = 100.0 * a["drej"] / (a["drej"] + a["dacc"]) if (a["drej"] + a["dacc"]) else 0.0
        duty = sum(a["duty"]) / len(a["duty"]) if a["duty"] else 0
        cpu = sum(a["cpu"]) / len(a["cpu"]) if a["cpu"] else 0
        label = f"{b[0]//1000}-{b[1]//1000}k" if b[1] < 999_999 else f">{b[0]//1000}k"
        print(f"{label:>16} {duty:>5.0f} {a['n']:>5} "
              + " ".join(f"{x:>6.0f}" for x in rate)
              + f"  {even:>7.0f} {odd:>7.0f}  {meas:>6.1f} {rej:>5.0f} {cpu:>5.0f}")

    # Falling OFF-center BEMF vs neutral — is the masked crossing visible there?
    if have_offc:
        print()
        print("FALLING-SECTOR OFF-CENTER BEMF  (does it cross neutral?)")
        h2 = (f"{'eRPM band':>16}  {'fOFF min':>8} {'fOFF max':>8} "
              f"{'neutral':>8} {'swing':>6}  brackets-neutral?")
        print(h2)
        print("-" * len(h2))
        for b in bands:
            a = agg[b]
            if not a["fmin"]:
                continue
            fmin = min(a["fmin"]); fmax = max(a["fmax"])
            neu = (sum(a["zcth"]) / len(a["zcth"])) if a["zcth"] else 0
            brackets = (fmin < neu < fmax)
            label = f"{b[0]//1000}-{b[1]//1000}k" if b[1] < 999_999 else f">{b[0]//1000}k"
            print(f"{label:>16}  {fmin:>8.0f} {fmax:>8.0f} {neu:>8.0f} "
                  f"{fmax-fmin:>6.0f}  "
                  + ("YES — crossing IS at OFF-center → OFF-window detector viable"
                     if brackets else
                     "NO — stays one side of neutral (clamped); OFF-center won't help"))
        print()

    # Verdict: which polarity carries the guesses, across the whole run.
    tot_even = sum(agg[b]["sect"][0] + agg[b]["sect"][2] + agg[b]["sect"][4] for b in bands)
    tot_odd  = sum(agg[b]["sect"][1] + agg[b]["sect"][3] + agg[b]["sect"][5] for b in bands)
    tot = tot_even + tot_odd
    print()
    if tot == 0:
        print("VERDICT: zero guesses all run — every sector measured. (No polarity issue.)")
    else:
        hi, lo = ("EVEN (S0/S2/S4)", "ODD (S1/S3/S5)") if tot_even >= tot_odd else ("ODD (S1/S3/S5)", "EVEN (S0/S2/S4)")
        frac = 100.0 * max(tot_even, tot_odd) / tot
        print(f"VERDICT: {frac:.0f}% of all guesses are on {hi}  "
              f"(even={tot_even:,} odd={tot_odd:,}).")
        if frac >= 70:
            print(f"  -> POLARITY-LOCKED: {hi} is the masked ZC polarity. "
                  f"Apply the per-polarity offset / second sample point to {hi}.")
        else:
            print("  -> NOT cleanly polarity-split; misses are spread "
                  "(state-driven, not a single-polarity structural gap).")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1])
