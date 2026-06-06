#!/usr/bin/env python3
"""garuda-gsp-record — capture a shareable session bundle.

Streams telemetry (GET_SNAPSHOT polling, no motor commands), records identity +
full param dump + the timeseries, and writes a bundle directory you can hand to
anyone for offline diagnosis. Stop with Ctrl+C.

    garuda-gsp-record --name cobra_firsttry
    # -> sessions/cobra_firsttry/{meta.json,params.json,telemetry.csv,faults.json}
"""
import argparse
import os
import platform
import sys
import time

from garuda_gsp import GspClient, GspError, Session, __version__


def main():
    ap = argparse.ArgumentParser(description="Record a GSP session bundle")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate", type=float, default=50.0, help="poll Hz (default 50)")
    ap.add_argument("--name", default=None, help="bundle name (default: timestamped)")
    ap.add_argument("--outdir", default="sessions")
    args = ap.parse_args()

    name = args.name or f"session_{time.strftime('%Y%m%d_%H%M%S')}"
    path = os.path.join(args.outdir, name)

    try:
        c = GspClient(args.port, args.baud)
    except OSError as e:
        print(f"ERROR: cannot open port: {e}")
        return 1

    try:
        if not c.ping():
            print("FAIL: no PING (port/baud/power?)")
            return 2
        info = c.get_info()
        print(f"  {c.port}  fw v{info['fwVersion']}  "
              f"build={('0x%08X' % info['buildHash']) if info['buildHash'] else 'n/a'}  "
              f"profile={info['motorProfile']}")
        print("  reading params…")
        params = c.dump_params()
        sess = Session(info=info, params=params,
                       host={"os": platform.system(), "release": platform.release()},
                       tool_version=__version__,
                       started_at=time.strftime("%Y-%m-%dT%H:%M:%S"))

        period = 1.0 / args.rate
        t0 = time.monotonic()
        c.telem_stop()  # ensure clean polling
        print(f"  recording @ {args.rate:g} Hz → {path}   (Ctrl+C to stop)")
        n = 0
        while True:
            tick = time.monotonic()
            try:
                snap = c.get_snapshot()
                snap["t"] = tick - t0
                sess.add(snap)
                n += 1
                if n % int(max(args.rate, 1)) == 0:
                    last = sess.samples[-1]
                    print(f"\r  t={last['t']:6.1f}s  {last['state_name']:<8} "
                          f"eRPM={last['eRPM']:>7,} duty={last['duty']:>3}% "
                          f"fault={last['fault_name']:<8} ({n} samples)", end="")
            except GspError:
                pass
            dt = period - (time.monotonic() - tick)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        print()
    finally:
        c.close()

    sess.save(path)
    print(f"  saved bundle: {path}  ({len(sess.samples)} samples, "
          f"{len(sess.faults)} fault events)")
    if sess.faults:
        for fe in sess.faults:
            print(f"    fault @ t={fe['t']:.2f}s: {fe['fault_name']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
