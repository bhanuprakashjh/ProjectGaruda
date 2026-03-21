#!/usr/bin/env python3
"""
Motor sweep test — ramps throttle from 0→max→0 via GSP SET_THROTTLE,
captures telemetry, and prints a report with current vs speed analysis.

Usage:
  python3 motor_sweep_test.py /dev/ttyACM0 [--max-pct 50] [--ramp-time 5] [--hold 2]

The motor must be started manually (press SW1 or GUI Start) before running.
"""

import serial
import struct
import time
import sys
import argparse
import csv
from datetime import datetime

# GSP protocol
GSP_START = 0x02
CMD_PING = 0x00
CMD_GET_INFO = 0x01
CMD_GET_SNAPSHOT = 0x02
CMD_START_MOTOR = 0x03
CMD_STOP_MOTOR = 0x04
CMD_SET_THROTTLE = 0x06
CMD_TELEM_START = 0x14
CMD_TELEM_STOP = 0x15
CMD_ERROR = 0xFF

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
    return bytes([GSP_START]) + body + struct.pack('>H', crc)

def parse_packet(buf):
    """Try to parse a GSP packet from buffer. Returns (cmd, payload, remaining) or None."""
    while len(buf) >= 5:
        idx = buf.find(bytes([GSP_START]))
        if idx < 0:
            return None, buf
        if idx > 0:
            buf = buf[idx:]
        if len(buf) < 5:
            return None, buf
        pkt_len = buf[1]
        total = 1 + 1 + pkt_len + 2
        if len(buf) < total:
            return None, buf
        body = buf[2:2+pkt_len]
        crc_recv = struct.unpack('>H', buf[2+pkt_len:2+pkt_len+2])[0]
        crc_calc = crc16(buf[1:2+pkt_len])
        if crc_calc != crc_recv:
            buf = buf[1:]
            continue
        cmd = body[0]
        payload = body[1:]
        return (cmd, payload), buf[total:]
    return None, buf

CK_CURRENT_SCALE = 1.049
CK_VBUS_SCALE = 1211.0

def decode_ck_snapshot(data):
    """Decode 48-byte CK snapshot."""
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
    # Derived
    s['iaMa'] = round(s['iaRaw'] * CK_CURRENT_SCALE)
    s['ibMa'] = round(s['ibRaw'] * CK_CURRENT_SCALE)
    s['ibusMa'] = round(s['ibusRaw'] * CK_CURRENT_SCALE)
    s['vbusV'] = round(s['vbusRaw'] / CK_VBUS_SCALE, 2)
    return s

def pct_to_throttle(pct):
    if pct <= 0:
        return 0
    POT_DEADZONE = 2000
    POT_MAX = 64000
    return int(POT_DEADZONE + (pct / 100.0) * (POT_MAX - POT_DEADZONE))

def set_throttle(ser, pct, activate=True):
    val = pct_to_throttle(pct)
    flags = 0x01 if activate and pct > 0 else 0x00
    payload = struct.pack('<HB', val, flags)
    ser.write(build_packet(CMD_SET_THROTTLE, payload))

def release_throttle(ser):
    payload = struct.pack('<HB', 0, 0x00)
    ser.write(build_packet(CMD_SET_THROTTLE, payload))

def read_responses(ser, buf, timeout=0.1):
    """Read all available packets within timeout."""
    packets = []
    deadline = time.time() + timeout
    while time.time() < deadline:
        data = ser.read(ser.in_waiting or 1)
        if data:
            buf += data
        while True:
            result, buf = parse_packet(buf)
            if result is None:
                break
            packets.append(result)
    return packets, buf

def get_snapshot(ser, buf):
    """Request and receive a single snapshot."""
    ser.write(build_packet(CMD_GET_SNAPSHOT))
    packets, buf = read_responses(ser, buf, timeout=0.15)
    for cmd, payload in packets:
        if cmd == CMD_GET_SNAPSHOT and len(payload) >= 48:
            return decode_ck_snapshot(payload), buf
    return None, buf

STATE_NAMES = ['IDLE', 'ARMED', 'ALIGN', 'OL_RAMP', 'CL', 'RECOVERY', 'FAULT']
FAULT_NAMES = ['NONE', 'OV', 'UV', 'STALL', 'DESYNC', 'START_TIMEOUT', 'ATA6847']

def main():
    parser = argparse.ArgumentParser(description='Motor sweep test')
    parser.add_argument('port', help='Serial port (e.g. /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--max-pct', type=int, default=100, help='Max throttle %% (default 100)')
    parser.add_argument('--ramp-time', type=float, default=10.0, help='Ramp up time in seconds (default 10)')
    parser.add_argument('--hold', type=float, default=2.0, help='Hold at max time in seconds (default 2)')
    parser.add_argument('--step-ms', type=int, default=100, help='Update interval ms (default 100)')
    parser.add_argument('--auto-start', action='store_true', help='Auto-start motor via GSP')
    parser.add_argument('--csv', type=str, default=None, help='Output CSV file')
    parser.add_argument('--no-ramp-down', action='store_true', help='Skip ramp down (just release)')
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.5)
    ser.reset_input_buffer()
    buf = b''

    print(f"Motor Sweep Test — {args.port} @ {args.baud}")
    print(f"  Max throttle: {args.max_pct}%")
    print(f"  Ramp time:    {args.ramp_time}s up + {'skip' if args.no_ramp_down else f'{args.ramp_time}s'} down")
    print(f"  Hold time:    {args.hold}s")
    print()

    # Ping
    ser.write(build_packet(CMD_PING))
    packets, buf = read_responses(ser, buf, timeout=0.3)
    if not any(cmd == CMD_PING for cmd, _ in packets):
        print("ERROR: No response to PING. Is GSP enabled?")
        return

    # Get initial snapshot
    snap, buf = get_snapshot(ser, buf)
    if snap:
        print(f"  State: {STATE_NAMES[snap['state']] if snap['state'] < len(STATE_NAMES) else snap['state']}")
        print(f"  Vbus:  {snap['vbusV']}V")
        print()

    if args.auto_start:
        print("Starting motor...")
        ser.write(build_packet(CMD_START_MOTOR))
        time.sleep(2.0)  # Wait for alignment + OL ramp
        snap, buf = get_snapshot(ser, buf)
        if snap and snap['state'] < 3:
            print(f"WARNING: Motor state is {STATE_NAMES[snap['state']]}, expected OL_RAMP or CL")

    samples = []
    start_time = time.time()

    def sample(throttle_pct):
        snap, _ = get_snapshot(ser, buf)
        if not snap:
            return
        t = time.time() - start_time
        samples.append({
            'time': round(t, 3),
            'throttle_pct': throttle_pct,
            'eRpm': snap['eRpm'],
            'duty_pct': snap['dutyPct'],
            'ibus_mA': snap['ibusMa'],
            'ia_mA': snap['iaMa'],
            'ib_mA': snap['ibMa'],
            'vbus_V': snap['vbusV'],
            'zc_synced': 1 if snap['zcSynced'] else 0,
            'missed': snap['missed'],
            'forced': snap['forced'],
            'state': snap['state'],
            'fault': snap['fault'],
            'step_period': snap['stepPeriod'],
            'filter_level': snap['filterLevel'],
        })
        state_str = STATE_NAMES[snap['state']] if snap['state'] < len(STATE_NAMES) else f"?{snap['state']}"
        sync_str = "SYN" if snap['zcSynced'] else "---"
        print(f"  t={t:6.1f}s  thr={throttle_pct:5.1f}%  eRPM={snap['eRpm']:7d}  duty={snap['dutyPct']:3d}%  "
              f"Ibus={snap['ibusMa']:6d}mA  Vbus={snap['vbusV']:5.1f}V  {state_str} {sync_str}  "
              f"Miss={snap['missed']} Frc={snap['forced']} FL={snap['filterLevel']}")
        if snap['fault'] > 0:
            fname = FAULT_NAMES[snap['fault']] if snap['fault'] < len(FAULT_NAMES) else f"?{snap['fault']}"
            print(f"  *** FAULT: {fname} ***")

    step_s = args.step_ms / 1000.0
    ramp_steps = max(1, int(args.ramp_time / step_s))
    hold_steps = max(1, int(args.hold / step_s))

    try:
        # Ramp up
        print("=== RAMP UP ===")
        for i in range(ramp_steps + 1):
            pct = args.max_pct * i / ramp_steps
            set_throttle(ser, pct)
            time.sleep(step_s)
            sample(pct)
            # Check for fault
            if samples and samples[-1]['fault'] > 0:
                print("FAULT detected, aborting ramp up")
                break

        # Hold
        if not samples or samples[-1]['fault'] == 0:
            print(f"\n=== HOLD at {args.max_pct}% ===")
            for i in range(hold_steps):
                set_throttle(ser, args.max_pct)
                time.sleep(step_s)
                sample(args.max_pct)
                if samples[-1]['fault'] > 0:
                    print("FAULT detected, aborting hold")
                    break

        # Ramp down
        if not args.no_ramp_down and (not samples or samples[-1]['fault'] == 0):
            print("\n=== RAMP DOWN ===")
            for i in range(ramp_steps + 1):
                pct = args.max_pct * (1.0 - i / ramp_steps)
                set_throttle(ser, pct)
                time.sleep(step_s)
                sample(pct)
                if samples[-1]['fault'] > 0:
                    print("FAULT detected, aborting ramp down")
                    break

    except KeyboardInterrupt:
        print("\n\nAborted by user")
    finally:
        print("\nReleasing throttle and stopping motor...")
        release_throttle(ser)
        time.sleep(0.2)
        if args.auto_start:
            ser.write(build_packet(CMD_STOP_MOTOR))
        time.sleep(0.3)

    # Report
    if samples:
        print("\n" + "=" * 70)
        print("TEST REPORT")
        print("=" * 70)
        max_erpm = max(s['eRpm'] for s in samples)
        max_ibus = max(s['ibus_mA'] for s in samples)
        max_ia = max(abs(s['ia_mA']) for s in samples)
        max_ib = max(abs(s['ib_mA']) for s in samples)
        avg_vbus = sum(s['vbus_V'] for s in samples) / len(samples)
        min_vbus = min(s['vbus_V'] for s in samples)
        max_power = max(s['vbus_V'] * s['ibus_mA'] / 1000 for s in samples)
        faults = [s for s in samples if s['fault'] > 0]
        desyncs = [s for s in samples if s['zc_synced'] == 0 and s['state'] >= 3]

        print(f"  Duration:      {samples[-1]['time']:.1f}s ({len(samples)} samples)")
        print(f"  Max eRPM:      {max_erpm}")
        print(f"  Max IBus:      {max_ibus} mA")
        print(f"  Max |Ia|:      {max_ia} mA")
        print(f"  Max |Ib|:      {max_ib} mA")
        print(f"  Max Power:     {max_power:.1f} W")
        print(f"  Avg Vbus:      {avg_vbus:.2f} V")
        print(f"  Min Vbus:      {min_vbus:.2f} V")
        print(f"  Desyncs:       {len(desyncs)}")
        print(f"  Faults:        {len(faults)}")
        print(f"  Result:        {'FAIL' if faults else 'PASS'}")

        # Current vs speed table
        print(f"\n  Speed vs Current (at ramp-up points):")
        print(f"  {'eRPM':>8s}  {'Duty%':>5s}  {'IBus mA':>8s}  {'|Ia| mA':>8s}  {'|Ib| mA':>8s}  {'Vbus V':>7s}  {'Power W':>8s}")
        print(f"  {'-'*8}  {'-'*5}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*7}  {'-'*8}")
        # Pick ~10 evenly spaced samples from ramp up
        ramp_up = [s for s in samples if s['time'] <= args.ramp_time + 1]
        step = max(1, len(ramp_up) // 10)
        for s in ramp_up[::step]:
            power = s['vbus_V'] * s['ibus_mA'] / 1000
            print(f"  {s['eRpm']:8d}  {s['duty_pct']:5d}  {s['ibus_mA']:8d}  {abs(s['ia_mA']):8d}  "
                  f"{abs(s['ib_mA']):8d}  {s['vbus_V']:7.2f}  {power:8.1f}")

    # CSV export
    csv_file = args.csv or f"motor_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    if samples:
        with open(csv_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=samples[0].keys())
            writer.writeheader()
            writer.writerows(samples)
        print(f"\n  CSV saved: {csv_file}")

    ser.close()
    print("\nDone.")

if __name__ == '__main__':
    main()
