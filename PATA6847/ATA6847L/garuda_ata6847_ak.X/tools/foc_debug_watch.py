#!/usr/bin/env python3
"""
FOC debug UART watcher for the AK ATA6847L board.

Streams the dsPIC debug UART (115200 8N1) to stdout AND a logfile.
Auto-tries /dev/ttyACM1 first (PKoB CDC bridge to dsPIC UART), falls
back to /dev/ttyACM0 if that's busy or silent.

Usage:
  python3 tools/foc_debug_watch.py                    # auto-detect, log to /tmp
  python3 tools/foc_debug_watch.py /dev/ttyACM1       # explicit port
  python3 tools/foc_debug_watch.py /dev/ttyACM1 my.log

Ctrl-C to quit. Press the board's reset / SW1 / SW2 with this script
running — output appears live.
"""

import sys
import time
import serial
import serial.tools.list_ports
from datetime import datetime


def list_acm_ports():
    """Return all /dev/ttyACM* paths the kernel knows about."""
    return sorted(p.device for p in serial.tools.list_ports.comports()
                  if "ttyACM" in p.device or "ttyUSB" in p.device)


def try_open(path, baud=115200):
    """Try to open `path` at `baud`. Returns Serial on success, None on fail."""
    try:
        s = serial.Serial(path, baud, timeout=0.1)
        return s
    except (serial.SerialException, OSError) as e:
        print(f"  {path}: {e}", file=sys.stderr)
        return None


def main():
    argv = sys.argv[1:]

    port_arg = argv[0] if len(argv) >= 1 else None
    log_arg = argv[1] if len(argv) >= 2 else None

    if port_arg:
        candidates = [port_arg]
    else:
        all_ports = list_acm_ports()
        if not all_ports:
            print("No /dev/ttyACM* or /dev/ttyUSB* devices found.",
                  file=sys.stderr)
            sys.exit(1)
        # Prefer ACM1 (debug bridge), then ACM0, then anything else.
        candidates = sorted(all_ports,
                            key=lambda p: (0 if p.endswith("ACM1") else
                                           1 if p.endswith("ACM0") else 2))

    ser = None
    for path in candidates:
        print(f"trying {path}...", file=sys.stderr)
        ser = try_open(path)
        if ser is not None:
            print(f"opened {path} @ 115200", file=sys.stderr)
            break
    if ser is None:
        print("could not open any candidate port", file=sys.stderr)
        sys.exit(1)

    log_path = log_arg or f"/tmp/foc_run_{datetime.now():%H%M%S}.log"
    try:
        logf = open(log_path, "w", buffering=1)
        print(f"logging to {log_path}", file=sys.stderr)
    except OSError as e:
        print(f"could not open log {log_path}: {e}", file=sys.stderr)
        logf = None

    print("─" * 60, file=sys.stderr)
    print("Watching. Press the board's RESET / SW1 / SW2 now.", file=sys.stderr)
    print("Ctrl-C to quit.", file=sys.stderr)
    print("─" * 60, file=sys.stderr)

    buf = b""
    try:
        while True:
            chunk = ser.read(256)
            if not chunk:
                continue
            buf += chunk
            # Split on either \r\n or \n so we print whole lines.
            while True:
                # find earliest line-ender
                idx_n = buf.find(b"\n")
                if idx_n < 0:
                    break
                line, buf = buf[:idx_n + 1], buf[idx_n + 1:]
                text = line.decode("utf-8", errors="replace").rstrip("\r\n")
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                stamped = f"[{ts}] {text}"
                print(stamped, flush=True)
                if logf is not None:
                    logf.write(stamped + "\n")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if logf is not None:
            logf.close()
            print(f"\nlog saved to {log_path}", file=sys.stderr)


if __name__ == "__main__":
    main()
