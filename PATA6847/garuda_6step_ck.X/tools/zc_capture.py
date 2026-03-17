#!/usr/bin/env python3
"""ZC/IC instrumentation capture tool for Garuda 6-step CK.

Connects to the ESC's debug UART, enables verbose mode, and captures
all telemetry into a timestamped CSV file for analysis.

Usage:
    python3 zc_capture.py                      # auto-detect port
    python3 zc_capture.py -p /dev/ttyACM0      # specify port
    python3 zc_capture.py -d 60                 # capture for 60 seconds
    python3 zc_capture.py -o run1.csv           # custom output file

Output CSV columns:
    timestamp_ms, elapsed_s, state, pot, vbus, duty, step, tp, zc_count,
    synced, missed, bemf_a, bemf_b, bemf_c, dc1, io1, io2, io3,
    ic_accepted, ic_lcout_accepted, ic_false, ic_captures, ic_chatter,
    tp_hr, zc_interval, zc_interval_hr, prev_zc_interval,
    ic_phase, forced_steps, erpm, duty_pct_x10, fault_code

Requires: pyserial (pip install pyserial)
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


# ── Field parsers ──────────────────────────────────────────────────────

# Map of tag → (csv_column_name, parser)
# parser: 'u16' = unsigned int, 'u32' = unsigned int, 'str' = string,
#         'hex' = hex int, 'bool' = 0/1 from SYN/---

FIELD_DEFS = [
    # Always present
    ("S:",    "state",           "str"),
    ("P:",    "pot",             "u16"),
    ("V:",    "vbus",            "u16"),
    ("D:",    "duty",            "u32"),
    ("St:",   "step",            "u16"),
    # Present when state >= OL_RAMP
    ("Tp:",   "tp",              "u16"),
    ("ZC:",   "zc_count",        "u16"),
    ("Miss:", "missed",          "u16"),
    ("DC1:",  "dc1",             "u16"),
    # IC diagnostics (verbose)
    ("IC:",   "ic_diag",         "ic"),   # special: A/F/C/Ch
    ("TpHR:", "tp_hr",           "u16"),
    ("ZcI:",  "zc_interval",     "u16"),
    ("ZcIHR:","zc_interval_hr",  "u16"),
    ("PZcI:", "prev_zc_interval","u16"),
    ("ICph:", "ic_phase",        "u16"),
    ("Frc:",  "forced_steps",    "u16"),
    ("eRPM:", "erpm",            "u32"),
    ("D%:",   "duty_pct_x10",    "u16"),
    # Fault
    ("F:",    "fault_code",      "u16"),
]

# CSV column order
CSV_COLUMNS = [
    "timestamp_ms", "elapsed_s", "raw_line",
    "state", "pot", "vbus", "duty", "step",
    "tp", "zc_count", "synced", "missed",
    "bemf_a", "bemf_b", "bemf_c",
    "dc1", "io1", "io2", "io3",
    "ic_accepted", "ic_lcout_accepted", "ic_false", "ic_captures", "ic_chatter",
    "tp_hr", "zc_interval", "zc_interval_hr", "prev_zc_interval",
    "ic_phase", "ic_filter_level", "forced_steps", "erpm", "duty_pct_x10",
    "fault_code",
]


def parse_line(line: str) -> dict | None:
    """Parse a firmware status line into a dict of field values."""
    line = line.strip()
    if not line.startswith("S:"):
        return None  # Not a status line

    row = {}

    # State
    m = re.search(r"S:(\S+)", line)
    if m:
        row["state"] = m.group(1)

    # Simple numeric fields
    for tag, col, fmt in FIELD_DEFS:
        if fmt in ("u16", "u32"):
            pattern = re.escape(tag) + r"(\d+)"
            m = re.search(pattern, line)
            if m:
                row[col] = int(m.group(1))
        elif fmt == "str" and col != "state":
            pattern = re.escape(tag) + r"(\S+)"
            m = re.search(pattern, line)
            if m:
                row[col] = m.group(1)

    # Synced flag
    if " SYN" in line:
        row["synced"] = 1
    elif " ---" in line:
        row["synced"] = 0

    # BEMF comparator states (B:xyz)
    m = re.search(r"B:([01])([01])([01])", line)
    if m:
        row["bemf_a"] = int(m.group(1))
        row["bemf_b"] = int(m.group(2))
        row["bemf_c"] = int(m.group(3))

    # IO register triple (IO:xxxx/yyyy/zzzz)
    m = re.search(r"IO:([0-9A-Fa-f]+)/([0-9A-Fa-f]+)/([0-9A-Fa-f]+)", line)
    if m:
        row["io1"] = int(m.group(1), 16)
        row["io2"] = int(m.group(2), 16)
        row["io3"] = int(m.group(3), 16)

    # IC diagnostics — firmware format: FP:accepted/BK:lcout/false Cyc:polls
    m = re.search(r"FP:(\d+)/BK:(\d+)/(\d+)", line)
    if m:
        row["ic_accepted"] = int(m.group(1))
        row["ic_lcout_accepted"] = int(m.group(2))
        row["ic_false"] = int(m.group(3))
    # Poll cycle count
    m = re.search(r"Cyc:(\d+)", line)
    if m:
        row["ic_captures"] = int(m.group(1))
    # Filter level
    m = re.search(r"FL:(\d+)", line)
    if m:
        row["ic_filter_level"] = int(m.group(1))

    return row


# ── Port detection ─────────────────────────────────────────────────────

def find_serial_port() -> str | None:
    """Auto-detect the EV43F54A USB-UART port."""
    ports = serial.tools.list_ports.comports()
    candidates = []
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        # Microchip PKoB4 or common USB-UART chips
        if any(kw in desc for kw in ("usb", "uart", "serial", "pkob", "cdc")):
            candidates.append(p.device)
        elif "04d8" in hwid or "0403" in hwid or "10c4" in hwid:
            candidates.append(p.device)
    # Prefer /dev/ttyACM* over /dev/ttyUSB*
    candidates.sort(key=lambda d: (0 if "ACM" in d else 1, d))
    return candidates[0] if candidates else None


# ── Main capture loop ──────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Capture ZC/IC telemetry from Garuda 6-step CK ESC")
    parser.add_argument("-p", "--port", help="Serial port (auto-detect if omitted)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("-d", "--duration", type=float, default=0,
                        help="Capture duration in seconds (0 = until Ctrl+C)")
    parser.add_argument("-o", "--output", help="Output CSV file (auto-named if omitted)")
    parser.add_argument("--no-verbose", action="store_true",
                        help="Don't send 'v' to enable verbose mode")
    parser.add_argument("--raw", action="store_true",
                        help="Also print raw lines to stdout")
    args = parser.parse_args()

    # Find port
    port = args.port or find_serial_port()
    if not port:
        print("Error: No serial port found. Specify with -p /dev/ttyACM0")
        sys.exit(1)

    # Output file
    if args.output:
        csv_path = Path(args.output)
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        logs_dir = Path(__file__).parent / "logs"
        logs_dir.mkdir(exist_ok=True)
        csv_path = logs_dir / f"zc_capture_{ts}.csv"

    print(f"Port:     {port} @ {args.baud}")
    print(f"Output:   {csv_path}")
    if args.duration > 0:
        print(f"Duration: {args.duration}s")
    else:
        print("Duration: until Ctrl+C")
    print()

    # Stats
    total_lines = 0
    parsed_lines = 0
    start_time = time.monotonic()
    running = True

    def handle_sigint(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_sigint)

    # Connect
    try:
        ser = serial.Serial(port, args.baud, timeout=0.5)
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}")
        sys.exit(1)

    time.sleep(0.3)  # Wait for connection to stabilize
    ser.reset_input_buffer()

    # Enable verbose mode
    if not args.no_verbose:
        ser.write(b"v")
        time.sleep(0.1)
        print("Sent 'v' to enable verbose mode (100ms telemetry)")

    print("Capturing... (Ctrl+C to stop)\n")

    # Track IC diag deltas
    prev_ic = {"ic_accepted": 0, "ic_lcout_accepted": 0, "ic_false": 0, "ic_captures": 0, "ic_chatter": 0}

    with open(csv_path, "w", newline="") as f:
        # Add delta columns for IC diagnostics
        extended_cols = CSV_COLUMNS + [
            "d_ic_accepted", "d_ic_lcout_accepted", "d_ic_false", "d_ic_captures", "d_ic_chatter",
            "ic_accept_rate", "ic_false_rate",
        ]
        writer = csv.DictWriter(f, fieldnames=extended_cols, extrasaction="ignore")
        writer.writeheader()

        while running:
            if args.duration > 0:
                elapsed = time.monotonic() - start_time
                if elapsed >= args.duration:
                    break

            try:
                raw = ser.readline()
            except serial.SerialException:
                print("\nSerial connection lost!")
                break

            if not raw:
                continue

            try:
                line = raw.decode("ascii", errors="replace").strip()
            except Exception:
                continue

            if not line:
                continue

            total_lines += 1

            if args.raw:
                print(line)

            # Parse
            row = parse_line(line)
            if row is None:
                # Print non-status lines for context (state transitions, etc.)
                if line.startswith("->") or line.startswith(">>"):
                    elapsed = time.monotonic() - start_time
                    print(f"  [{elapsed:7.1f}s] {line}")
                continue

            parsed_lines += 1
            elapsed = time.monotonic() - start_time
            now_ms = int(time.time() * 1000)

            row["timestamp_ms"] = now_ms
            row["elapsed_s"] = round(elapsed, 3)
            row["raw_line"] = line

            # Compute IC diagnostic deltas
            for key in ("ic_accepted", "ic_lcout_accepted", "ic_false", "ic_captures", "ic_chatter"):
                cur = row.get(key, 0)
                row[f"d_{key}"] = cur - prev_ic.get(key, 0)
                prev_ic[key] = cur

            # Compute accept/false rates (per 100ms interval)
            d_cap = row.get("d_ic_captures", 0)
            if d_cap > 0:
                row["ic_accept_rate"] = round(
                    row.get("d_ic_accepted", 0) / d_cap * 100, 1)
                row["ic_false_rate"] = round(
                    row.get("d_ic_false", 0) / d_cap * 100, 1)
            else:
                row["ic_accept_rate"] = ""
                row["ic_false_rate"] = ""

            writer.writerow(row)

            # Live summary on stdout (compact, one line)
            state = row.get("state", "?")
            tp = row.get("tp", "")
            duty = row.get("duty", "")
            erpm = row.get("erpm", "")
            zc = row.get("zc_count", "")
            syn = "SYN" if row.get("synced") else "---"
            miss = row.get("missed", "")
            frc = row.get("forced_steps", "")
            tp_hr = row.get("tp_hr", "")
            d_acc = row.get("d_ic_accepted", 0)
            d_lco = row.get("d_ic_lcout_accepted", 0)
            d_fls = row.get("d_ic_false", 0)
            d_cap = row.get("d_ic_captures", 0)
            d_cht = row.get("d_ic_chatter", 0)
            acc_r = row.get("ic_accept_rate", "")

            if state in ("CL", "OL_RAMP"):
                print(
                    f"\r{elapsed:6.1f}s  {state:7s}  "
                    f"D:{duty:<5}  Tp:{tp:<4}  TpHR:{tp_hr:<5}  "
                    f"eRPM:{erpm:<6}  ZC:{zc:<5} {syn}  "
                    f"Miss:{miss}  Frc:{frc}  "
                    f"dIC:{d_acc} dLCO:{d_lco} dF:{d_fls} dCap:{d_cap}  "
                    f"AccR:{acc_r}%",
                    end="", flush=True,
                )
            else:
                print(
                    f"\r{elapsed:6.1f}s  {state:7s}  D:{duty}  P:{row.get('pot','')}",
                    end="", flush=True,
                )

    # Disable verbose before exit
    if not args.no_verbose:
        try:
            ser.write(b"v")
            time.sleep(0.1)
        except Exception:
            pass

    ser.close()

    elapsed_total = time.monotonic() - start_time
    print(f"\n\nCapture complete.")
    print(f"  Duration:  {elapsed_total:.1f}s")
    print(f"  Total lines: {total_lines}")
    print(f"  Parsed:    {parsed_lines}")
    print(f"  CSV:       {csv_path}")
    print(f"  Size:      {csv_path.stat().st_size / 1024:.1f} KB")

    # Print summary statistics from the CSV
    if parsed_lines > 0:
        print_summary(csv_path)


def print_summary(csv_path: Path):
    """Print summary statistics from captured CSV."""
    import statistics

    rows = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)

    if not rows:
        return

    print("\n── Summary Statistics ──────────────────────────────────")

    # Filter to CL rows
    cl_rows = [r for r in rows if r.get("state") == "CL"]
    if not cl_rows:
        print("  No closed-loop data captured.")
        return

    print(f"  CL samples: {len(cl_rows)}")

    # Step period stats
    tps = [int(r["tp"]) for r in cl_rows if r.get("tp")]
    if tps:
        print(f"  Tp range:   {min(tps)} - {max(tps)}  "
              f"(mean: {statistics.mean(tps):.1f})")

    # HR step period
    tp_hrs = [int(r["tp_hr"]) for r in cl_rows if r.get("tp_hr")]
    if tp_hrs:
        print(f"  TpHR range: {min(tp_hrs)} - {max(tp_hrs)}  "
              f"(mean: {statistics.mean(tp_hrs):.1f})")

    # Duty
    duties = [int(r["duty"]) for r in cl_rows if r.get("duty")]
    if duties:
        print(f"  Duty range: {min(duties)} - {max(duties)}  "
              f"(mean: {statistics.mean(duties):.0f})")

    # eRPM
    erpms = [int(r["erpm"]) for r in cl_rows if r.get("erpm")]
    if erpms:
        print(f"  eRPM range: {min(erpms)} - {max(erpms)}  "
              f"(mean: {statistics.mean(erpms):.0f})")

    # Missed steps
    misses = [int(r["missed"]) for r in cl_rows if r.get("missed")]
    if misses:
        max_miss = max(misses)
        print(f"  Max missed: {max_miss}")

    # Forced steps
    frcs = [int(r["forced_steps"]) for r in cl_rows if r.get("forced_steps")]
    if frcs:
        print(f"  Forced steps: max={max(frcs)}, "
              f"mean={statistics.mean(frcs):.1f}")

    # IC accept rate
    acc_rates = [float(r["ic_accept_rate"]) for r in cl_rows
                 if r.get("ic_accept_rate") and r["ic_accept_rate"] != ""]
    if acc_rates:
        print(f"  IC accept rate: {min(acc_rates):.0f}% - {max(acc_rates):.0f}%  "
              f"(mean: {statistics.mean(acc_rates):.1f}%)")

    # IC totals (use last row)
    last = cl_rows[-1]
    for key in ("ic_accepted", "ic_lcout_accepted", "ic_false", "ic_captures", "ic_chatter"):
        if last.get(key):
            print(f"  {key}: {last[key]}")
    # IC vs LCOUT split
    ic_tot = int(last.get("ic_accepted", 0) or 0)
    lco_tot = int(last.get("ic_lcout_accepted", 0) or 0)
    if ic_tot + lco_tot > 0:
        print(f"  IC/LCOUT split: {ic_tot}/{lco_tot} "
              f"({ic_tot*100//(ic_tot+lco_tot)}%/{lco_tot*100//(ic_tot+lco_tot)}%)")

    # ZC interval alternation analysis
    zc_is = [int(r["zc_interval"]) for r in cl_rows if r.get("zc_interval")]
    pzc_is = [int(r["prev_zc_interval"]) for r in cl_rows
              if r.get("prev_zc_interval")]
    if zc_is and pzc_is and len(zc_is) == len(pzc_is):
        diffs = [abs(z - p) for z, p in zip(zc_is, pzc_is) if z > 0 and p > 0]
        if diffs:
            print(f"  ZcI alternation: mean_diff={statistics.mean(diffs):.1f}, "
                  f"max_diff={max(diffs)}")

    # Desync detection: find where Tp jumps
    print("\n── Desync Events ──────────────────────────────────────")
    desync_count = 0
    for i in range(1, len(cl_rows)):
        tp_prev = int(cl_rows[i-1].get("tp", 0) or 0)
        tp_cur = int(cl_rows[i].get("tp", 0) or 0)
        if tp_prev > 0 and tp_cur > 0 and tp_cur > tp_prev * 2:
            desync_count += 1
            t = cl_rows[i].get("elapsed_s", "?")
            d = cl_rows[i].get("duty", "?")
            print(f"  [{t}s] Tp jump: {tp_prev} -> {tp_cur}  "
                  f"D:{d}  eRPM:{cl_rows[i].get('erpm','?')}")
            if desync_count >= 10:
                print("  ... (truncated)")
                break

    if desync_count == 0:
        print("  None detected.")

    # Speed/duty profile
    print("\n── Speed vs Duty Profile ──────────────────────────────")
    # Bucket by Tp
    tp_buckets: dict[int, list[int]] = {}
    for r in cl_rows:
        tp = int(r.get("tp", 0) or 0)
        d = int(r.get("duty", 0) or 0)
        if tp > 0 and d > 0:
            tp_buckets.setdefault(tp, []).append(d)

    for tp in sorted(tp_buckets.keys()):
        duties = tp_buckets[tp]
        erpm = 200000 // tp if tp > 0 else 0
        print(f"  Tp:{tp:4d}  eRPM:{erpm:6d}  "
              f"D: {min(duties):5d}-{max(duties):5d}  "
              f"({len(duties)} samples)")


if __name__ == "__main__":
    main()
