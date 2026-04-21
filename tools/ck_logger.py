#!/usr/bin/env python3
"""
V4 telemetry logger for the CK board — phase-current peak focus.

Renders the V4 snapshot with phase-current rolling peaks from the
firmware at offset 114+ (added 2026-04-21 for kickback theory validation
against the AKESC A2212/12V dataset in docs/commutation_kickback_findings.md).

Protocol mirrors PATA6847/tools/pot_capture.py (CRC-16 CCITT,
CMD_TELEM_START=0x14, TELEM_FRAME arrives on cmd=0x80).

Usage:
    python3 tools/ck_logger.py [--port /dev/ttyACM0] [--rate-ms 50]

Output:
    logs/ck_YYYYMMDD_HHMMSS.csv
"""

import argparse
import csv
import os
import signal
import struct
import sys
import time
from datetime import datetime

import serial

GSP_START = 0x02
CMD_PING = 0x00
CMD_TELEM_START = 0x14
CMD_TELEM_STOP = 0x15
CMD_TELEM_FRAME = 0x80   # incoming telemetry frame

CK_CURRENT_SCALE = 1.049   # from pot_capture.py — amps per raw unit (approx)

STATE_NAMES = ['IDLE', 'ARMED', 'ALIGN', 'OL_RAMP', 'CL', 'RECOVERY', 'FAULT']


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
    c = crc16(body)
    return bytes([GSP_START]) + body + bytes([c >> 8, c & 0xFF])


def parse_packet(buf):
    while len(buf) >= 5:
        idx = buf.find(bytes([GSP_START]))
        if idx < 0:
            return None, b''
        if idx > 0:
            buf = buf[idx:]
        if len(buf) < 2:
            return None, buf
        plen = buf[1]
        if plen < 1 or plen > 249:
            buf = buf[1:]
            continue
        total = 2 + plen + 2
        if len(buf) < total:
            return None, buf
        body = buf[1:2 + plen]
        crc_rx = (buf[2 + plen] << 8) | buf[2 + plen + 1]
        if crc16(body) != crc_rx:
            buf = buf[1:]
            continue
        cmd = buf[2]
        payload = buf[3:2 + plen]
        return (cmd, payload), buf[total:]
    return None, buf


def decode_frame(payload):
    """V4 snapshot frame: 2B seq + 146B snapshot. Peaks at offset 114+
    INSIDE the snapshot (i.e. offset 116+ inside the payload with the
    2-byte seq prefix)."""
    if len(payload) < 148:
        return None
    s = {}
    s['seq'] = struct.unpack_from('<H', payload, 0)[0]
    d = payload[2:]  # snapshot bytes

    s['state'] = d[0]
    s['step'] = d[2]
    s['potRaw'] = struct.unpack_from('<H', d, 4)[0]
    s['dutyPct'] = d[6]

    s['vbusRaw'] = struct.unpack_from('<H', d, 8)[0]
    s['iaRaw'] = struct.unpack_from('<h', d, 10)[0]
    s['ibRaw'] = struct.unpack_from('<h', d, 12)[0]
    s['ibusRaw'] = struct.unpack_from('<H', d, 14)[0]
    s['duty'] = struct.unpack_from('<H', d, 16)[0]

    s['timerPeriod'] = struct.unpack_from('<H', d, 18)[0]
    s['eRpm'] = struct.unpack_from('<I', d, 22)[0]
    s['erpmNow'] = struct.unpack_from('<H', d, 38)[0]
    s['captures'] = struct.unpack_from('<H', d, 32)[0]
    s['piRuns'] = struct.unpack_from('<H', d, 34)[0]

    # Peak block at offset 114 (snapshot-relative)
    s['iaPkMax'] = struct.unpack_from('<h', d, 114)[0]
    s['iaPkMin'] = struct.unpack_from('<h', d, 116)[0]
    s['ibPkMax'] = struct.unpack_from('<h', d, 118)[0]
    s['ibPkMin'] = struct.unpack_from('<h', d, 120)[0]
    # ibusPk fields are zero (firmware doesn't compute); host derives.
    s['iaAtFaultMax'] = struct.unpack_from('<h', d, 126)[0]
    s['iaAtFaultMin'] = struct.unpack_from('<h', d, 128)[0]
    s['ibAtFaultMax'] = struct.unpack_from('<h', d, 130)[0]
    s['ibAtFaultMin'] = struct.unpack_from('<h', d, 132)[0]
    s['iaAtFaultInst'] = struct.unpack_from('<h', d, 138)[0]
    s['ibAtFaultInst'] = struct.unpack_from('<h', d, 140)[0]
    s['faultValid'] = d[144]

    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate-ms", type=int, default=50,
                    help="Firmware emit period in ms")
    ap.add_argument("--duration", type=float, default=0.0)
    args = ap.parse_args()

    print(f"Connecting to {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.1)

    # Ping
    ser.write(build_packet(CMD_PING))
    time.sleep(0.1)
    buf = ser.read(32)
    if not buf or bytes([GSP_START]) not in buf:
        print("  PING timeout")
        return 1
    print("  PING OK")

    # Start telemetry
    ser.write(build_packet(CMD_TELEM_START, struct.pack('<H', args.rate_ms)))
    time.sleep(0.05)

    os.makedirs("logs", exist_ok=True)
    csv_path = f"logs/ck_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    print(f"  Logging to: {os.path.abspath(csv_path)}")

    t0 = time.monotonic()
    csv_file = open(csv_path, "w", newline="")
    writer = None

    stop = [False]
    def on_int(*_): stop[0] = True
    signal.signal(signal.SIGINT, on_int)

    print("─" * 170)
    print(f"{'time':>7} {'state':>8} st duty {'eRPM':>7} {'erpmNow':>7} "
          f"{'Tp':>5} caps/pi   "
          f"Ia(pk+/-,|pk|)        Ib(pk+/-,|pk|)        fault")
    print("─" * 170)

    rxbuf = b""
    sample_count = 0
    last_print = 0.0
    try:
        while not stop[0]:
            if args.duration > 0 and time.monotonic() - t0 > args.duration:
                break
            rxbuf += ser.read(256)
            while True:
                parsed, rxbuf = parse_packet(rxbuf)
                if parsed is None:
                    break
                cmd, payload = parsed
                if cmd == CMD_TELEM_FRAME:
                    row = decode_frame(payload)
                    if row is None:
                        continue
                    row['t'] = time.monotonic() - t0
                    if writer is None:
                        writer = csv.DictWriter(csv_file, fieldnames=list(row.keys()))
                        writer.writeheader()
                    writer.writerow(row)
                    sample_count += 1
                    if time.monotonic() - last_print > 0.2:
                        state = STATE_NAMES[row['state']] if row['state'] < len(STATE_NAMES) else f"?{row['state']}"
                        iapk = max(abs(row['iaPkMax']), abs(row['iaPkMin']))
                        ibpk = max(abs(row['ibPkMax']), abs(row['ibPkMin']))
                        # raw counts → approximate amps (placeholder scale)
                        ia_a = row['iaRaw'] * CK_CURRENT_SCALE / 2048.0 * 10  # heuristic
                        iapk_a = iapk * CK_CURRENT_SCALE / 2048.0 * 10
                        ib_a = row['ibRaw'] * CK_CURRENT_SCALE / 2048.0 * 10
                        ibpk_a = ibpk * CK_CURRENT_SCALE / 2048.0 * 10
                        flt = " FLT!" if row['faultValid'] else ""
                        print(f"{row['t']:7.2f} {state:>8s} {row['step']:2d} "
                              f"{row['dutyPct']:3d}% {row['eRpm']:7d} "
                              f"{row['erpmNow']:7d} {row['timerPeriod']:5d} "
                              f"{row['captures']:5d}/{row['piRuns']:5d} "
                              f"Ia{row['iaRaw']:+5d}/{row['iaPkMax']:+5d}/{row['iaPkMin']:+5d}|{iapk:4d}| "
                              f"Ib{row['ibRaw']:+5d}/{row['ibPkMax']:+5d}/{row['ibPkMin']:+5d}|{ibpk:4d}|{flt}")
                        last_print = time.monotonic()
    finally:
        ser.write(build_packet(CMD_TELEM_STOP))
        csv_file.close()

    print("\n── Summary ──")
    print(f"  Duration: {time.monotonic() - t0:.1f}s")
    print(f"  Samples:  {sample_count}")
    print(f"  CSV:      {os.path.abspath(csv_path)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
