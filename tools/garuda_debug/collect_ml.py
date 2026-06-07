#!/usr/bin/env python3
"""Collect an ML training dataset from the bench, no GUI needed.

Repeatedly grabs the 24 kHz burst scope while YOU sweep the throttle pot, and
logs each 128-sample window (bemf/zcthr/sector + eRPM/duty) to
sessions/ml_<timestamp>.jsonl. Train it later with garuda_gui/zcml.py.

The motor is pot-driven (there's no host throttle command), so the workflow is:
  1. start this script
  2. slowly sweep the pot idle -> top -> idle (and try a few accel/decel ramps)
  3. Ctrl-C to stop and save

Usage:
  .venv/bin/python collect_ml.py --motor "2810 1350KV"
  .venv/bin/python collect_ml.py --port /dev/ttyUSB0 --motor cobra --seconds 120
"""
import argparse
import json
import os
import time

from garuda_gsp.client import GspClient, find_port, list_ports_human
from garuda_gsp import protocol as P


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (auto-detected if omitted)")
    ap.add_argument("--motor", default="bench", help="motor label stored in each record")
    ap.add_argument("--seconds", type=float, default=0,
                    help="auto-stop after N seconds (0 = until Ctrl-C)")
    ap.add_argument("--min-erpm", type=float, default=0,
                    help="skip windows below this eRPM (default 0 = keep all)")
    args = ap.parse_args()

    try:
        port = find_port(args.port)
    except Exception:
        print("No serial port found. Available ports:")
        print(list_ports_human())
        return

    os.makedirs("sessions", exist_ok=True)
    path = os.path.join("sessions", f"ml_{time.strftime('%Y%m%d_%H%M%S')}.jsonl")
    print(f"port {port}  ->  {path}")
    print("Sweep the throttle pot now (idle -> top -> idle). Ctrl-C to stop.\n")

    n_written = n_skip = 0
    t0 = time.monotonic()
    with GspClient(port) as c, open(path, "w") as f:
        try:
            while True:
                if args.seconds and (time.monotonic() - t0) >= args.seconds:
                    break
                # arm Manual (immediate) trigger, wait for the buffer to fill
                c.scope_arm(trig_mode=P.SCOPE_TRIG["Manual"] if hasattr(P, "SCOPE_TRIG")
                            else 0, pre_pct=25)
                ready = False
                for _ in range(50):                 # ~1 s max to fill 128 samples
                    st = c.scope_status()
                    if st.get("state") == 3:        # READY
                        ready = True
                        break
                    time.sleep(0.02)
                if not ready:
                    continue
                samples = c.scope_read_all()
                if not samples:
                    continue
                erpm = _median([s.get("eRPM", 0) for s in samples])
                duty = _median([s.get("duty_pct", 0) for s in samples])
                if erpm < args.min_erpm:
                    n_skip += 1
                    continue
                rec = {
                    "erpm": erpm, "duty": duty, "motor": args.motor,
                    "bemf": [s.get("bemf_raw", 0) for s in samples],
                    "zcthr": [s.get("zc_thresh", 0) for s in samples],
                    "sector": [s.get("sector", 0) for s in samples],
                }
                f.write(json.dumps(rec) + "\n")
                f.flush()
                n_written += 1
                print(f"\r  {n_written} windows  (last {erpm:6.0f} eRPM, "
                      f"{duty:4.0f}% duty)   skipped {n_skip}   ", end="", flush=True)
        except KeyboardInterrupt:
            pass

    print(f"\n\nsaved {n_written} windows -> {path}")
    if n_written:
        print("train it with:")
        print("  .venv/bin/python -c \"import glob,numpy as np; from garuda_gui import zcml; "
              "caps=zcml.load_jsonl(sorted(glob.glob('sessions/ml_*.jsonl'))); "
              "X,y,m=zcml.extract_examples(caps); i=np.random.default_rng(0).permutation(len(X)); "
              "n=int(.8*len(X)); md=zcml.MLP(X.shape[1]).fit(X[i[:n]],y[i[:n]]); "
              "zcml.evaluate(md,X[i[n:]],y[i[n:]],[m[k] for k in i[n:]])\"")


def _median(xs):
    xs = sorted(xs)
    n = len(xs)
    if n == 0:
        return 0.0
    return float(xs[n // 2] if n % 2 else (xs[n // 2 - 1] + xs[n // 2]) / 2.0)


if __name__ == "__main__":
    main()
