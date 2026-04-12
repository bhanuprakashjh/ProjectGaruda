#!/usr/bin/env python3
"""
DMA edge burst LOGGER — repeatedly arm the burst capture and append
every captured step to a CSV.

Workflow per iteration:
  1. GET_SNAPSHOT (for motor state context: eRPM, duty, vbus, state)
  2. BURST_ARM
  3. BURST_STATUS (poll until FULL)
  4. BURST_GET_STEP × 12 → append 12 rows to CSV
  5. loop

Use this when you want enough steps to get real aggregate statistics
on the cluster-based ZC candidates. Each iteration captures 12
consecutive commutation steps; 100 iterations = 1200 steps.

Usage:
  python3 tools/edge_burst_log.py /dev/ttyACM1 [--iterations N] [--csv FILE]

Press Ctrl+C to stop early.
"""

import serial
import struct
import sys
import time
import argparse
import csv
from datetime import datetime

# GSP protocol ---------------------------------------------------------
GSP_START            = 0x02
CMD_PING             = 0x00
CMD_GET_SNAPSHOT     = 0x02
CMD_BURST_ARM        = 0x20
CMD_BURST_STATUS     = 0x21
CMD_BURST_GET_STEP   = 0x22

HR_TICK_NS  = 640
NS_PER_US   = 1000.0

BURST_EDGES = 32
BURST_STEP_FMT  = f'<HHH{BURST_EDGES}HBBBB'
BURST_STEP_SIZE = struct.calcsize(BURST_STEP_FMT)  # 74
BURST_STEPS     = 12

CK_VBUS_SCALE = 1211.0

# --------------------------------------------------------------------- GSP

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def build_packet(cmd_id, payload=b''):
    pkt_len = 1 + len(payload)
    body = bytes([pkt_len, cmd_id]) + payload
    crc = crc16(body)
    return bytes([GSP_START]) + body + bytes([crc >> 8, crc & 0xFF])


def parse_packet(buf):
    while len(buf) >= 5:
        idx = buf.find(bytes([GSP_START]))
        if idx < 0:
            return None, b''
        if idx > 0:
            buf = buf[idx:]
        if len(buf) < 2:
            return None, buf
        pkt_len = buf[1]
        if pkt_len < 1 or pkt_len > 249:
            buf = buf[1:]
            continue
        total = 2 + pkt_len + 2
        if len(buf) < total:
            return None, buf
        body = buf[1:2 + pkt_len]
        crc_recv = (buf[2 + pkt_len] << 8) | buf[2 + pkt_len + 1]
        if crc16(body) != crc_recv:
            buf = buf[1:]
            continue
        cmd = buf[2]
        payload = buf[3:2 + pkt_len]
        return (cmd, payload), buf[total:]
    return None, buf


def send_and_wait(ser, cmd_id, payload=b'', expected_cmd=None, timeout=0.5):
    ser.reset_input_buffer()
    ser.write(build_packet(cmd_id, payload))
    deadline = time.time() + timeout
    buf = b''
    target = expected_cmd if expected_cmd is not None else cmd_id
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
        while True:
            pkt, buf = parse_packet(buf)
            if not pkt:
                break
            cmd, resp = pkt
            if cmd == target:
                return resp
    return None


def hr_delta_us(tick_a, tick_b):
    d = (tick_a - tick_b) & 0xFFFF
    if d >= 0x8000:
        d -= 0x10000
    return d * HR_TICK_NS / NS_PER_US


def decode_step(raw):
    fields = struct.unpack_from(BURST_STEP_FMT, raw, 0)
    commHR       = fields[0]
    pollHR       = fields[1]
    predictedHR  = fields[2]
    edges        = list(fields[3:3 + BURST_EDGES])
    edgeCount    = fields[3 + BURST_EDGES]
    stepIndex    = fields[4 + BURST_EDGES]
    risingZc     = fields[5 + BURST_EDGES]
    wasTimeout   = fields[6 + BURST_EDGES]
    return {
        'commHR':      commHR,
        'pollHR':      pollHR,
        'predictedHR': predictedHR,
        'edges':       edges[:edgeCount],
        'edgeCount':   edgeCount,
        'stepIndex':   stepIndex,
        'risingZc':    bool(risingZc),
        'wasTimeout':  bool(wasTimeout),
    }


def decode_snapshot_minimal(data):
    """Minimal snapshot decode — just the fields we need for context."""
    if len(data) < 28:
        return None
    s = {}
    s['state']    = data[0]
    s['fault']    = data[1]
    s['step']     = data[2]
    s['dutyPct']  = data[6]
    s['vbusRaw']  = struct.unpack_from('<H', data, 8)[0]
    s['eRpm']     = struct.unpack_from('<I', data, 22)[0]
    s['vbusV']    = round(s['vbusRaw'] / CK_VBUS_SCALE, 2)
    return s


# --------------------------------------------------------------------- main

CSV_FIELDS = [
    'ts',             # host time (seconds since start)
    'iter',           # iteration index (0-based)
    'step_in_iter',   # 0..11 within this burst
    # Motor state context (snapshot captured right before ARM):
    'eRpm', 'state', 'duty_pct', 'vbus_v',
    # Step raw timestamps (HR domain, 16-bit):
    'commHR', 'pollHR', 'predictedHR',
    # Step metadata:
    'step_index', 'rising_zc', 'was_timeout', 'edge_count',
    # Derived (µs from commutation, wrap-safe):
    'poll_us', 'pred_us',
    # Raw edge list as µs from commutation, semicolon-separated
    # (makes reimport trivial; keeps CSV reasonably compact):
    'edges_us',
]


def main():
    ap = argparse.ArgumentParser(description="DMA edge burst CSV logger")
    ap.add_argument('port', help="Serial port (e.g. /dev/ttyACM1)")
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--iterations', type=int, default=200,
                    help="Number of burst arm cycles (default 200 → 2400 steps)")
    ap.add_argument('--csv', default=None,
                    help="Output CSV path (default auto timestamped)")
    ap.add_argument('--wait', type=float, default=2.0,
                    help="Max seconds to wait for each capture to fill")
    args = ap.parse_args()

    csv_path = args.csv
    if csv_path is None:
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = f"edge_burst_{ts}.csv"

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(0.1)
    ser.reset_input_buffer()

    # Ping
    resp = send_and_wait(ser, CMD_PING)
    if resp is None:
        print("ERROR: no response to PING")
        sys.exit(1)
    print(f"Connected to {args.port}")
    print(f"  Iterations: {args.iterations}  (up to {args.iterations * BURST_STEPS} steps)")
    print(f"  CSV: {csv_path}")
    print(f"  Press Ctrl+C to stop early.")
    print()

    t_start = time.time()
    rows_written = 0
    iter_idx = 0

    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()

        try:
            while iter_idx < args.iterations:
                # 1) Snapshot for motor state context
                snap_resp = send_and_wait(ser, CMD_GET_SNAPSHOT,
                                          expected_cmd=CMD_GET_SNAPSHOT,
                                          timeout=0.3)
                snap = decode_snapshot_minimal(snap_resp) if snap_resp else None

                # 2) Arm burst
                arm_resp = send_and_wait(ser, CMD_BURST_ARM, timeout=0.3)
                if arm_resp is None:
                    print(f"[iter {iter_idx}] ARM timeout, retrying")
                    continue

                # 3) Poll for FULL
                poll_deadline = time.time() + args.wait
                state = 0
                count = 0
                while time.time() < poll_deadline:
                    st_resp = send_and_wait(ser, CMD_BURST_STATUS, timeout=0.2)
                    if st_resp and len(st_resp) >= 2:
                        state, count = st_resp[0], st_resp[1]
                        if state == 2:  # FULL
                            break
                    time.sleep(0.005)
                if state != 2:
                    print(f"[iter {iter_idx}] capture not full (state={state} count={count}) — skipping")
                    iter_idx += 1
                    continue

                # 4) Download all steps
                steps = []
                fetch_ok = True
                for i in range(BURST_STEPS):
                    r = send_and_wait(ser, CMD_BURST_GET_STEP, bytes([i]),
                                      expected_cmd=CMD_BURST_GET_STEP,
                                      timeout=0.3)
                    if r is None or len(r) < BURST_STEP_SIZE:
                        print(f"[iter {iter_idx}] download step {i} failed")
                        fetch_ok = False
                        break
                    steps.append(decode_step(r))

                if not fetch_ok:
                    iter_idx += 1
                    continue

                # 5) Write rows
                ts_now = time.time() - t_start
                for si, step in enumerate(steps):
                    # Compute derived values (wrap-safe)
                    poll_us = (hr_delta_us(step['pollHR'], step['commHR'])
                               if step['pollHR'] != 0 else '')
                    pred_us = (hr_delta_us(step['predictedHR'], step['commHR'])
                               if step['predictedHR'] != 0 else '')
                    edges_us = ';'.join(
                        f"{hr_delta_us(e, step['commHR']):.2f}"
                        for e in step['edges']
                    )

                    row = {
                        'ts':            f"{ts_now:.3f}",
                        'iter':          iter_idx,
                        'step_in_iter':  si,
                        'eRpm':          snap['eRpm'] if snap else '',
                        'state':         snap['state'] if snap else '',
                        'duty_pct':      snap['dutyPct'] if snap else '',
                        'vbus_v':        snap['vbusV'] if snap else '',
                        'commHR':        step['commHR'],
                        'pollHR':        step['pollHR'],
                        'predictedHR':   step['predictedHR'],
                        'step_index':    step['stepIndex'],
                        'rising_zc':     int(step['risingZc']),
                        'was_timeout':   int(step['wasTimeout']),
                        'edge_count':    step['edgeCount'],
                        'poll_us':       poll_us if isinstance(poll_us, str) else f"{poll_us:.2f}",
                        'pred_us':       pred_us if isinstance(pred_us, str) else f"{pred_us:.2f}",
                        'edges_us':      edges_us,
                    }
                    writer.writerow(row)
                    rows_written += 1

                # Progress line every ~10 iterations
                if iter_idx % 10 == 0 or iter_idx == args.iterations - 1:
                    erpm_str = f"{snap['eRpm']:>6d}" if snap else "   ---"
                    print(f"  iter {iter_idx:>4d}/{args.iterations}  "
                          f"rows={rows_written:>5d}  eRPM={erpm_str}  "
                          f"elapsed={ts_now:6.1f}s")

                iter_idx += 1

        except KeyboardInterrupt:
            print("\nStopped by user.")

    print(f"\nWrote {rows_written} rows to {csv_path}")
    print(f"Total time: {time.time() - t_start:.1f}s")


if __name__ == '__main__':
    main()
