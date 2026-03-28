#!/usr/bin/env python3
"""
Pot capture — streams CK snapshot telemetry while you manually control
the pot. Records everything to CSV for later analysis.

Usage:
  python3 tools/pot_capture.py /dev/ttyACM1 [--duration 60] [--csv pot_run.csv]

Press Ctrl+C to stop. Motor start/stop via board buttons (SW1/SW2).
"""

import serial
import struct
import time
import sys
import argparse
from datetime import datetime

# GSP protocol
GSP_START = 0x02
CMD_PING = 0x00
CMD_GET_SNAPSHOT = 0x02
CMD_TELEM_START = 0x14
CMD_TELEM_STOP = 0x15

CK_CURRENT_SCALE = 1.049
CK_VBUS_SCALE = 1211.0

STATE_NAMES = ['IDLE', 'ARMED', 'ALIGN', 'OL_RAMP', 'CL', 'RECOVERY', 'FAULT']
FAULT_NAMES = ['NONE', 'OV', 'UV', 'STALL', 'DESYNC', 'START_TO', 'ATA6847']

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
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
        crc_calc = crc16(body)
        if crc_recv != crc_calc:
            buf = buf[1:]
            continue
        cmd = buf[2]
        payload = buf[3:2 + pkt_len]
        return (cmd, payload), buf[total:]
    return None, buf

def decode_ck_snapshot(data):
    if len(data) < 48:
        return None
    s = {}
    s['state'] = data[0]
    s['fault'] = data[1]
    s['step'] = data[2]
    s['ataStatus'] = data[3]
    s['potRaw'] = struct.unpack_from('<H', data, 4)[0]
    s['dutyPct'] = data[6]
    s['zcSynced'] = data[7] != 0
    s['vbusRaw'] = struct.unpack_from('<H', data, 8)[0]
    s['iaRaw'] = struct.unpack_from('<h', data, 10)[0]
    s['ibRaw'] = struct.unpack_from('<h', data, 12)[0]
    s['ibusRaw'] = struct.unpack_from('<h', data, 14)[0]
    s['duty'] = struct.unpack_from('<H', data, 16)[0]
    s['stepPeriod'] = struct.unpack_from('<H', data, 18)[0]
    s['stepPeriodHR'] = struct.unpack_from('<H', data, 20)[0]
    s['eRpm'] = struct.unpack_from('<I', data, 22)[0]
    s['goodZc'] = struct.unpack_from('<H', data, 26)[0]
    s['zcInterval'] = struct.unpack_from('<H', data, 28)[0]
    s['prevZcInterval'] = struct.unpack_from('<H', data, 30)[0]
    s['icAccepted'] = struct.unpack_from('<H', data, 32)[0]
    s['icFalse'] = struct.unpack_from('<H', data, 34)[0]
    s['filterLevel'] = data[36]
    s['missed'] = data[37]
    s['forced'] = data[38]
    s['ilimActive'] = data[39] != 0
    s['systemTick'] = struct.unpack_from('<I', data, 40)[0]
    s['uptime'] = struct.unpack_from('<I', data, 44)[0]
    if len(data) >= 52:
        s['zcLatencyPct'] = data[48]
        s['zcBlankPct'] = data[49]
        s['zcBypassCount'] = struct.unpack_from('<H', data, 50)[0]
    else:
        s['zcLatencyPct'] = 0
        s['zcBlankPct'] = 0
        s['zcBypassCount'] = 0
    # V3 ZC diagnostics
    if len(data) >= 64:
        s['zcMode'] = data[52]
        s['actualForcedComm'] = data[53]
        s['zcTimeoutCount'] = struct.unpack_from('<H', data, 54)[0]
        s['risingZcCount'] = struct.unpack_from('<H', data, 56)[0]
        s['fallingZcCount'] = struct.unpack_from('<H', data, 58)[0]
        s['risingTimeouts'] = struct.unpack_from('<H', data, 60)[0]
        s['fallingTimeouts'] = struct.unpack_from('<H', data, 62)[0]
    else:
        s['zcMode'] = 0
        s['actualForcedComm'] = 0
        s['zcTimeoutCount'] = 0
        s['risingZcCount'] = 0
        s['fallingZcCount'] = 0
        s['risingTimeouts'] = 0
        s['fallingTimeouts'] = 0
    # V4 per-step 0..5 counters (88 bytes total)
    if len(data) >= 88:
        for i in range(6):
            s[f'stepAcc{i}'] = struct.unpack_from('<H', data, 64 + i*2)[0]
        for i in range(6):
            s[f'stepTO{i}'] = struct.unpack_from('<H', data, 76 + i*2)[0]
    else:
        for i in range(6):
            s[f'stepAcc{i}'] = 0
            s[f'stepTO{i}'] = 0
    # Derived
    s['iaMa'] = round(s['iaRaw'] * CK_CURRENT_SCALE)
    s['ibMa'] = round(s['ibRaw'] * CK_CURRENT_SCALE)
    s['ibusMa'] = round(s['ibusRaw'] * CK_CURRENT_SCALE)
    s['vbusV'] = round(s['vbusRaw'] / CK_VBUS_SCALE, 2)
    return s

def main():
    parser = argparse.ArgumentParser(description='Pot capture — stream telemetry while using pot')
    parser.add_argument('port', help='Serial port (e.g. /dev/ttyACM1)')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--duration', type=float, default=120, help='Max duration seconds (default 120)')
    parser.add_argument('--csv', type=str, default=None, help='Output CSV file')
    parser.add_argument('--rate', type=int, default=10, help='Telemetry rate Hz (default 10)')
    args = parser.parse_args()

    csv_file = args.csv or f'pot_capture_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.5)
    ser.reset_input_buffer()

    # Ping
    ser.write(build_packet(CMD_PING))
    time.sleep(0.3)
    buf = ser.read(4096)
    pkt, buf = parse_packet(buf)
    if not pkt or pkt[0] != CMD_PING:
        print("ERROR: No PING response")
        return

    # Start telemetry
    rate_ms = max(10, 1000 // args.rate)
    ser.write(build_packet(CMD_TELEM_START, struct.pack('<H', rate_ms)))
    time.sleep(0.3)
    # Drain any response
    resp = ser.read(4096)
    if resp:
        print(f"  Telem start response: {len(resp)} bytes")

    print(f"Pot Capture — {args.port} @ {args.baud}")
    print(f"  Telemetry: {1000//rate_ms} Hz, max {args.duration}s")
    print(f"  CSV: {csv_file}")
    print(f"  Control motor with SW1 (start) / SW2 (stop) and pot")
    print(f"  Press Ctrl+C to stop")
    print()
    ZC_MODES = ['ACQ', 'TRK', 'RCV']
    print(f"{'Time':>7s} {'State':>8s} {'ZcM':>3s} {'eRPM':>7s} {'Duty':>4s} {'Pot':>5s} {'Ibus':>7s} {'Vbus':>6s} {'FrcTO':>5s} {'R/F ZC':>10s} {'R/F TO':>10s} {'ZcLat':>6s}")
    print("-" * 105)

    rows = []
    buf = b''
    t0 = time.time()
    sample_count = 0

    try:
        while (time.time() - t0) < args.duration:
            chunk = ser.read(512)
            if chunk:
                buf += chunk

            # Poll snapshots manually every 100ms as fallback
            if (time.time() - t0) > 0.5 and (sample_count == 0 or not chunk):
                ser.write(build_packet(CMD_GET_SNAPSHOT))

            while True:
                pkt, buf = parse_packet(buf)
                if not pkt:
                    break
                cmd, payload = pkt

                # Accept both telem frames (0x80) and snapshot responses (0x02)
                snap_data = None
                if cmd == 0x80 and len(payload) >= 50:  # TELEM_FRAME (seq + snapshot)
                    snap_data = payload[2:]  # skip 2-byte seq counter
                elif cmd == CMD_GET_SNAPSHOT and len(payload) >= 48:
                    snap_data = payload

                if snap_data is None:
                    continue
                snap = decode_ck_snapshot(snap_data)
                if not snap:
                    continue

                sample_count += 1
                t = time.time() - t0

                state_str = STATE_NAMES[snap['state']] if snap['state'] < len(STATE_NAMES) else f"?{snap['state']}"
                zc_mode_str = ZC_MODES[snap['zcMode']] if snap['zcMode'] < len(ZC_MODES) else f"?{snap['zcMode']}"
                zc_lat = snap['zcLatencyPct']
                lat_str = " TO" if zc_lat == 255 else f"{zc_lat * 100 // 255:3d}%"
                fault_str = ""
                if snap['fault'] > 0:
                    fname = FAULT_NAMES[snap['fault']] if snap['fault'] < len(FAULT_NAMES) else f"?{snap['fault']}"
                    fault_str = f"  *** FAULT: {fname} ***"

                rfc = f"{snap['risingZcCount']:5d}/{snap['fallingZcCount']:<5d}"
                rft = f"{snap['risingTimeouts']:5d}/{snap['fallingTimeouts']:<5d}"
                print(f"{t:7.1f} {state_str:>8s} {zc_mode_str:>3s} {snap['eRpm']:7d} {snap['dutyPct']:3d}% {snap['potRaw']:5d} {snap['ibusMa']:6d}mA {snap['vbusV']:5.1f}V {snap['actualForcedComm']:5d} {rfc:>10s} {rft:>10s} {lat_str:>6s}{fault_str}")

                rows.append({
                    'time': round(t, 3),
                    'state': snap['state'],
                    'fault': snap['fault'],
                    'eRpm': snap['eRpm'],
                    'duty_pct': snap['dutyPct'],
                    'pot_raw': snap['potRaw'],
                    'ibus_mA': snap['ibusMa'],
                    'ia_mA': snap['iaMa'],
                    'ib_mA': snap['ibMa'],
                    'vbus_V': snap['vbusV'],
                    'zc_synced': 1 if snap['zcSynced'] else 0,
                    'missed': snap['missed'],
                    'forced': snap['forced'],
                    'step_period': snap['stepPeriod'],
                    'step_period_hr': snap['stepPeriodHR'],
                    'filter_level': snap['filterLevel'],
                    'zc_latency_pct': snap['zcLatencyPct'],
                    'zc_blank_pct': snap['zcBlankPct'],
                    'zc_bypass_count': snap['zcBypassCount'],
                    'ic_accepted': snap['icAccepted'],
                    'ic_false': snap['icFalse'],
                    'zc_interval': snap['zcInterval'],
                    'good_zc': snap['goodZc'],
                    'zc_mode': snap['zcMode'],
                    'actual_forced_comm': snap['actualForcedComm'],
                    'zc_timeout_count': snap['zcTimeoutCount'],
                    'rising_zc_count': snap['risingZcCount'],
                    'falling_zc_count': snap['fallingZcCount'],
                    'rising_timeouts': snap['risingTimeouts'],
                    'falling_timeouts': snap['fallingTimeouts'],
                    'step_acc_0': snap['stepAcc0'], 'step_acc_1': snap['stepAcc1'],
                    'step_acc_2': snap['stepAcc2'], 'step_acc_3': snap['stepAcc3'],
                    'step_acc_4': snap['stepAcc4'], 'step_acc_5': snap['stepAcc5'],
                    'step_to_0': snap['stepTO0'], 'step_to_1': snap['stepTO1'],
                    'step_to_2': snap['stepTO2'], 'step_to_3': snap['stepTO3'],
                    'step_to_4': snap['stepTO4'], 'step_to_5': snap['stepTO5'],
                })

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\nStopped by user")

    # Stop telemetry
    ser.write(build_packet(CMD_TELEM_STOP))
    time.sleep(0.1)
    ser.close()

    # Write CSV
    if rows:
        import csv
        keys = rows[0].keys()
        with open(csv_file, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
        print(f"\n{sample_count} samples saved to {csv_file}")

        # Print per-step breakdown from last sample
        last = rows[-1]
        print("\n=== Per-Step ZC Analysis (phase-pair vs polarity-class test) ===")
        print("Step | Phase | Polarity | PrevState | Accepted | Timeouts")
        print("-----|-------|----------|-----------|----------|--------")
        step_info = [
            (0, 'C', 'Rising',  'B=LOW'),
            (1, 'A', 'Falling', 'A=PWM'),
            (2, 'B', 'Rising',  'A=LOW'),
            (3, 'C', 'Falling', 'B=PWM'),
            (4, 'A', 'Rising',  'A=LOW'),
            (5, 'B', 'Falling', 'A=PWM'),
        ]
        for i, (step, phase, pol, prev) in enumerate(step_info):
            acc = int(last.get(f'step_acc_{i}', last.get(f'stepAcc{i}', 0)))
            to = int(last.get(f'step_to_{i}', last.get(f'stepTO{i}', 0)))
            print(f"  {step}  |   {phase}   | {pol:<8s} | {prev:<9s} | {acc:>8d} | {to:>7d}")
        # Summary
        rising_acc = sum(int(last.get(f'step_acc_{i}', last.get(f'stepAcc{i}', 0))) for i in [0,2,4])
        falling_acc = sum(int(last.get(f'step_acc_{i}', last.get(f'stepAcc{i}', 0))) for i in [1,3,5])
        rising_to = sum(int(last.get(f'step_to_{i}', last.get(f'stepTO{i}', 0))) for i in [0,2,4])
        falling_to = sum(int(last.get(f'step_to_{i}', last.get(f'stepTO{i}', 0))) for i in [1,3,5])
        print(f"\nRising  (0,2,4 = float-after-LOW): acc={rising_acc} to={rising_to}")
        print(f"Falling (1,3,5 = float-after-PWM): acc={falling_acc} to={falling_to}")
        if rising_acc + falling_acc > 0:
            # Check phase pairs
            print("\nPhase pairs (0/3=C, 1/4=A, 2/5=B):")
            for name, a, b in [('C', 0, 3), ('A', 1, 4), ('B', 2, 5)]:
                pa = int(last.get(f'step_acc_{a}', last.get(f'stepAcc{a}', 0))) + int(last.get(f'step_acc_{b}', last.get(f'stepAcc{b}', 0)))
                pt = int(last.get(f'step_to_{a}', last.get(f'stepTO{a}', 0))) + int(last.get(f'step_to_{b}', last.get(f'stepTO{b}', 0)))
                print(f"  Phase {name} (steps {a},{b}): acc={pa} to={pt}")
    else:
        print("\nNo data captured")

if __name__ == '__main__':
    main()
