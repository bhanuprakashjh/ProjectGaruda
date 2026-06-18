#!/usr/bin/env python3
"""
HWZC per-sector bias probe — Phase 1a measurement.

Reads GSP command 0x18 (GET_HWZC_DIAG) and prints, per read:
  eRPM, rising-sector ZC position (permille), falling-sector ZC position,
  R/F-bias = fall - rise, and the per-sector accept/miss counts (as deltas
  since the previous read, so you see which sectors are actively capturing).

The R/F-bias is the rising-HW vs falling-SW path-latency split that Tier-1
per-sector bias would correct. Run this ALONGSIDE the GUI (it shares the live
board through the broker), sweep the throttle/pot through the speed range, and
read the bias off at each plateau:

    python -m scripts.hwzc_bias_probe                # 2 Hz continuous
    python -m scripts.hwzc_bias_probe --hz 5         # faster
    python -m scripts.hwzc_bias_probe --csv bias.csv # also log to CSV

Go/no-go (2810, profile 2): if |R/F-bias| is ~40-60 permille and grows with
speed, Tier-1 per-sector bias is justified. If it's ~5 permille and flat, the
228-235k bounce is the random floor -> skip Tier 1.
"""
import argparse
import time

from garuda_gsp.broker import connect_auto


def main():
    ap = argparse.ArgumentParser(description="HWZC per-sector bias probe (cmd 0x18)")
    ap.add_argument("--hz", type=float, default=2.0, help="read rate (default 2)")
    ap.add_argument("--csv", default=None, help="optional CSV log path")
    args = ap.parse_args()

    c = connect_auto()
    csv = open(args.csv, "w") if args.csv else None
    if csv:
        csv.write("t,erpm,rise_pm,fall_pm,rf_bias_pm,good_zc,"
                  + ",".join(f"cap{i}" for i in range(6)) + ","
                  + ",".join(f"miss{i}" for i in range(6)) + "\n")

    print(f"{'eRPM':>8} {'rise‰':>6} {'fall‰':>6} {'bias‰':>7} {'good':>5}   "
          f"cap[0..5] Δ   miss[0..5] Δ      (Ctrl-C to stop)")
    prev_cap = prev_miss = None
    t0 = time.monotonic()
    try:
        while True:
            d = c.get_hwzc_diag()
            cap, miss = d["cap_by_sector"], d["miss_by_sector"]
            dcap  = [cap[i]  - prev_cap[i]  for i in range(6)] if prev_cap  else cap
            dmiss = [miss[i] - prev_miss[i] for i in range(6)] if prev_miss else miss
            prev_cap, prev_miss = cap, miss
            print(f"{d['erpm']:>8} {d['rise_pm']:>6} {d['fall_pm']:>6} "
                  f"{d['rf_bias_pm']:>+7} {d['good_zc']:>5}   "
                  f"{str(dcap):<22} {dmiss}")
            if csv:
                csv.write(f"{time.monotonic()-t0:.3f},{d['erpm']},{d['rise_pm']},"
                          f"{d['fall_pm']},{d['rf_bias_pm']},{d['good_zc']},"
                          + ",".join(map(str, cap)) + ","
                          + ",".join(map(str, miss)) + "\n")
                csv.flush()
            time.sleep(1.0 / args.hz)
    except KeyboardInterrupt:
        pass
    finally:
        if csv:
            csv.close()


if __name__ == "__main__":
    main()
