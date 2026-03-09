#!/usr/bin/env python3
"""
SMO Startup Capture — watches IDLE→ARMED→ALIGN→OL_RAMP→CL transition.

Arms the motor via GSP, captures snapshots at max rate through the entire
startup sequence, detects state transitions, and saves a CSV with all
SMO/FOC fields for post-analysis.

Auto-stops when:
  - Closed-loop is stable for --cl-dwell seconds (default 3s)
  - A fault occurs
  - Timeout expires (--timeout, default 30s)
  - Ctrl+C

Usage:
    python3 tools/smo_startup_capture.py [--port /dev/ttyACM0] [--throttle 200]
    python3 tools/smo_startup_capture.py --no-arm   # just observe, don't start motor

Output:
    logs/smo_startup_YYYYMMDD_HHMMSS.csv

Requirements:
    pip install pyserial
"""

import argparse
import csv
import os
import struct
import signal
import sys
import time
from datetime import datetime
from typing import Optional

import serial


# ── GSP Protocol ────────────────────────────────────────────────────────

GSP_START_BYTE = 0x02

GSP_CMD_PING         = 0x00
GSP_CMD_GET_INFO     = 0x01
GSP_CMD_GET_SNAPSHOT = 0x02
GSP_CMD_START_MOTOR  = 0x03
GSP_CMD_STOP_MOTOR   = 0x04
GSP_CMD_CLEAR_FAULT  = 0x05
GSP_CMD_SET_THROTTLE = 0x06

ESC_STATE_NAMES = {
    0: "IDLE", 1: "ARMED", 2: "DETECT", 3: "ALIGN", 4: "OL_RAMP",
    5: "MORPH", 6: "CL", 7: "BRAKING", 8: "RECOVERY", 9: "FAULT"
}

FOC_SUB_NAMES = {
    0: "idle", 1: "armed", 2: "align", 3: "ol_ramp", 4: "cl"
}

FAULT_NAMES = {
    0: "NONE", 1: "OVERVOLT", 2: "UNDERVOLT", 3: "OC_SW", 4: "BOARD_PCI",
    5: "STALL", 6: "DESYNC", 7: "STARTUP_TO", 8: "MORPH_TO",
    9: "RX_LOSS", 10: "FOC_INTERNAL", 11: "FOC_BUSLOSS"
}

# CSV columns — all snapshot fields + transition markers
CSV_COLUMNS = [
    "time_s", "dt_ms",
    # State
    "state", "state_name", "faultCode", "fault_name", "focSubState", "focSubName",
    "throttle", "dutyPct",
    # Bus
    "vbusRaw", "ibusRaw",
    # FOC core
    "focIdMeas", "focIqMeas", "focTheta", "focOmega", "focVbus",
    "focIa", "focIb", "focThetaObs", "focVd", "focVq",
    # SMO observer internals (mapped to flux fields)
    "smoEalpha", "smoEbeta", "smoGain",
    # PI internals
    "focPidDInteg", "focPidQInteg", "focPidSpdInteg",
    # Diagnostics
    "focModIndex", "focObsConfidence",
    "focOffsetIa", "focOffsetIb",
    # Derived
    "focRPM", "focPower",
    # Transition markers
    "transition",
    # System
    "systemTick",
]


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def build_packet(cmd_id: int, payload: bytes = b"") -> bytes:
    pkt_len = 1 + len(payload)
    crc_data = bytes([pkt_len, cmd_id]) + payload
    crc = crc16_ccitt(crc_data)
    return bytes([GSP_START_BYTE, pkt_len, cmd_id]) + payload + struct.pack(">H", crc)


def read_response(ser: serial.Serial, timeout: float = 1.0) -> Optional[tuple]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ser.timeout = max(deadline - time.monotonic(), 0.001)
        b = ser.read(1)
        if len(b) == 0:
            continue
        if b[0] == GSP_START_BYTE:
            break
    else:
        return None

    ser.timeout = max(deadline - time.monotonic(), 0.01)
    b = ser.read(1)
    if len(b) == 0:
        return None
    pkt_len = b[0]

    ser.timeout = max(deadline - time.monotonic(), 0.01)
    data = ser.read(pkt_len + 2)
    if len(data) < pkt_len + 2:
        return None

    cmd_id = data[0]
    payload = data[1:pkt_len]
    rx_crc = struct.unpack(">H", data[pkt_len:pkt_len + 2])[0]

    crc_data = bytes([pkt_len]) + data[:pkt_len]
    if crc16_ccitt(crc_data) != rx_crc:
        return None

    return (cmd_id, payload)


def send_cmd(ser: serial.Serial, cmd_id: int, payload: bytes = b"",
             timeout: float = 1.0) -> Optional[tuple]:
    ser.write(build_packet(cmd_id, payload))
    return read_response(ser, timeout=timeout)


def drain(ser: serial.Serial):
    ser.timeout = 0.05
    while ser.read(256):
        pass


# ── Snapshot Parser (focused on SMO fields) ────────────────────────────

def parse_snapshot(p: bytes, t0: float, prev_time: float) -> Optional[dict]:
    """Parse snapshot into dict with SMO-relevant fields."""
    if len(p) < 68:
        return None

    now = time.monotonic()
    dt_ms = (now - prev_time) * 1000.0 if prev_time > 0 else 0.0

    v = struct.unpack_from("<BBBBHBx", p, 0)
    state, fault, step, direction, throttle, duty = v

    vbus_raw, ibus_raw = struct.unpack_from("<HH", p, 8)
    sys_tick, = struct.unpack_from("<I", p, 60)

    # FOC fields — zero defaults
    foc_id = foc_iq = foc_theta = foc_omega = foc_vbus = 0.0
    foc_ia = foc_ib = foc_theta_obs = foc_vd = foc_vq = 0.0
    smo_ea = smo_eb = smo_gain = 0.0
    foc_pid_d = foc_pid_q = foc_pid_spd = 0.0
    foc_mod_idx = foc_obs_conf = 0.0
    foc_sub = 0
    foc_off_ia = foc_off_ib = 0

    if len(p) >= 150:
        # V3 format: full observer/PI/diag
        foc_id, foc_iq, foc_theta, foc_omega, foc_vbus = \
            struct.unpack_from("<fffff", p, 68)
        foc_ia, foc_ib, foc_theta_obs, foc_vd, foc_vq = \
            struct.unpack_from("<fffff", p, 88)
        smo_ea, smo_eb, _, smo_gain = \
            struct.unpack_from("<ffff", p, 108)
        foc_pid_d, foc_pid_q, foc_pid_spd = \
            struct.unpack_from("<fff", p, 124)
        foc_mod_idx, foc_obs_conf = \
            struct.unpack_from("<ff", p, 136)
        foc_sub, = struct.unpack_from("<B", p, 144)
        foc_off_ia, foc_off_ib = struct.unpack_from("<HH", p, 146)
    elif len(p) >= 114:
        foc_id, foc_iq, foc_theta, foc_omega, foc_vbus = \
            struct.unpack_from("<fffff", p, 68)
        foc_ia, foc_ib, foc_theta_obs, foc_vd, foc_vq = \
            struct.unpack_from("<fffff", p, 88)
        foc_sub, = struct.unpack_from("<B", p, 108)
        foc_off_ia, foc_off_ib = struct.unpack_from("<HH", p, 110)

    foc_rpm = abs(foc_omega) * 60.0 / (2.0 * 3.14159265)
    foc_power = foc_vq * foc_iq + foc_vd * foc_id

    return {
        "time_s": f"{now - t0:.4f}",
        "dt_ms": f"{dt_ms:.1f}",
        "state": state,
        "state_name": ESC_STATE_NAMES.get(state, f"?{state}"),
        "faultCode": fault,
        "fault_name": FAULT_NAMES.get(fault, f"?{fault}"),
        "focSubState": foc_sub,
        "focSubName": FOC_SUB_NAMES.get(foc_sub, f"?{foc_sub}"),
        "throttle": throttle,
        "dutyPct": duty,
        "vbusRaw": vbus_raw,
        "ibusRaw": ibus_raw,
        "focIdMeas": f"{foc_id:.4f}",
        "focIqMeas": f"{foc_iq:.4f}",
        "focTheta": f"{foc_theta:.4f}",
        "focOmega": f"{foc_omega:.2f}",
        "focVbus": f"{foc_vbus:.2f}",
        "focIa": f"{foc_ia:.4f}",
        "focIb": f"{foc_ib:.4f}",
        "focThetaObs": f"{foc_theta_obs:.4f}",
        "focVd": f"{foc_vd:.4f}",
        "focVq": f"{foc_vq:.4f}",
        "smoEalpha": f"{smo_ea:.6f}",
        "smoEbeta": f"{smo_eb:.6f}",
        "smoGain": f"{smo_gain:.4f}",
        "focPidDInteg": f"{foc_pid_d:.4f}",
        "focPidQInteg": f"{foc_pid_q:.4f}",
        "focPidSpdInteg": f"{foc_pid_spd:.4f}",
        "focModIndex": f"{foc_mod_idx:.4f}",
        "focObsConfidence": f"{foc_obs_conf:.4f}",
        "focOffsetIa": foc_off_ia,
        "focOffsetIb": foc_off_ib,
        "focRPM": f"{foc_rpm:.1f}",
        "focPower": f"{foc_power:.3f}",
        "transition": "",
        "systemTick": sys_tick,
        "_raw_time": now,
    }


# ── Main ────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="SMO Startup Capture — watches FOC v3 startup transitions")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--throttle", type=int, default=200,
                        help="GSP throttle value 0-4095 (default 200, ~5%%)")
    parser.add_argument("--no-arm", action="store_true",
                        help="Don't send start command — just observe")
    parser.add_argument("--cl-dwell", type=float, default=3.0,
                        help="Seconds of stable CL before auto-stop (default 3)")
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="Max capture time in seconds (default 30)")
    parser.add_argument("--no-stop", action="store_true",
                        help="Don't send stop command at end")
    args = parser.parse_args()

    logs_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "logs")
    os.makedirs(logs_dir, exist_ok=True)
    filename = os.path.join(
        logs_dir, f"smo_startup_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

    print(f"SMO Startup Capture")
    print(f"  Port:     {args.port}")
    print(f"  Throttle: {args.throttle} ({'observe only' if args.no_arm else 'will arm'})")
    print(f"  CL dwell: {args.cl_dwell}s")
    print(f"  Timeout:  {args.timeout}s")
    print()

    # Connect
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    time.sleep(0.2)
    drain(ser)

    # Ping
    resp = send_cmd(ser, GSP_CMD_PING)
    if resp is None or resp[0] != GSP_CMD_PING:
        print("ERROR: No PING response")
        ser.close()
        sys.exit(1)
    print("  PING OK")

    # Get info
    resp = send_cmd(ser, GSP_CMD_GET_INFO)
    if resp and resp[0] == GSP_CMD_GET_INFO and len(resp[1]) >= 14:
        flags = struct.unpack_from("<I", resp[1], 8)[0]
        is_foc = bool(flags & (1 << 23))
        print(f"  Mode: {'FOC' if is_foc else '6-step'} (flags=0x{flags:08X})")
        if not is_foc:
            print("  WARNING: FOC flag not set — SMO data will be zeros")

    # Check initial state
    resp = send_cmd(ser, GSP_CMD_GET_SNAPSHOT)
    if resp and resp[0] == GSP_CMD_GET_SNAPSHOT:
        init_state = resp[1][0]
        snap_size = len(resp[1])
        print(f"  State: {ESC_STATE_NAMES.get(init_state, '?')} "
              f"(snapshot={snap_size}B)")
        if init_state == 9:  # FAULT
            print("  Clearing fault first...")
            send_cmd(ser, GSP_CMD_CLEAR_FAULT)
            time.sleep(0.3)
    else:
        print("  WARNING: Could not read snapshot")

    # Open CSV
    csv_cols_out = [c for c in CSV_COLUMNS]  # copy
    csvfile = open(filename, "w", newline="")
    writer = csv.DictWriter(csvfile, fieldnames=csv_cols_out, extrasaction='ignore')
    writer.writeheader()
    print(f"  CSV: {filename}")
    print()

    # State tracking
    transitions = []
    prev_state = -1
    prev_sub = -1
    cl_enter_time = None
    sample_count = 0
    stop = False

    def signal_handler(sig, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, signal_handler)

    # Set throttle first (before arming)
    if not args.no_arm:
        thr_payload = struct.pack("<H", args.throttle)
        resp = send_cmd(ser, GSP_CMD_SET_THROTTLE, thr_payload)
        if resp:
            print(f"  Throttle set to {args.throttle}")
        else:
            print("  WARNING: SET_THROTTLE failed")

    t0 = time.monotonic()
    prev_time = 0.0

    # Arm motor
    if not args.no_arm:
        print(f"  Sending START command...")
        resp = send_cmd(ser, GSP_CMD_START_MOTOR)
        if resp is None:
            print("  WARNING: START command got no response")
        else:
            print(f"  Motor armed at t=0")
    else:
        print(f"  Observing (no arm)...")

    print()
    print(f"  {'Time':>7s}  {'State':>8s}  {'Sub':>7s}  {'Iq':>7s}  "
          f"{'Id':>7s}  {'RPM':>7s}  {'Vbus':>6s}  {'SMO_Ea':>9s}  {'SMO_Eb':>9s}")
    print(f"  {'─'*7}  {'─'*8}  {'─'*7}  {'─'*7}  {'─'*7}  "
          f"{'─'*7}  {'─'*6}  {'─'*9}  {'─'*9}")

    last_print = 0.0

    while not stop:
        elapsed = time.monotonic() - t0
        if elapsed >= args.timeout:
            print(f"\n\n  TIMEOUT ({args.timeout}s)")
            break

        # Poll snapshot at max rate
        resp = send_cmd(ser, GSP_CMD_GET_SNAPSHOT, timeout=0.2)
        if resp is None or resp[0] != GSP_CMD_GET_SNAPSHOT:
            continue

        row = parse_snapshot(resp[1], t0, prev_time)
        if row is None:
            continue

        prev_time = row.pop("_raw_time", prev_time)

        # Detect state transitions
        cur_state = row["state"]
        cur_sub = row["focSubState"]
        transition_label = ""

        if cur_state != prev_state:
            cur_name = ESC_STATE_NAMES.get(cur_state, f"?{cur_state}")
            prev_name = ESC_STATE_NAMES.get(prev_state, f"?{prev_state}")
            transition_label = f"{prev_name}->{cur_name}"
            transitions.append((elapsed, transition_label))
            print(f"\n  *** TRANSITION at t={elapsed:.3f}s: {transition_label} ***")
            prev_state = cur_state

        if cur_sub != prev_sub and cur_state == prev_state:
            cur_sname = FOC_SUB_NAMES.get(cur_sub, f"?{cur_sub}")
            prev_sname = FOC_SUB_NAMES.get(prev_sub, f"?{prev_sub}")
            sub_label = f"sub:{prev_sname}->{cur_sname}"
            if not transition_label:
                transition_label = sub_label
                transitions.append((elapsed, sub_label))
                print(f"\n  *** SUB-STATE at t={elapsed:.3f}s: {sub_label} ***")
            prev_sub = cur_sub
        else:
            prev_sub = cur_sub

        row["transition"] = transition_label

        writer.writerow(row)
        sample_count += 1

        # Check for CL dwell completion
        if cur_state == 6:  # ESC_CLOSED_LOOP
            if cl_enter_time is None:
                cl_enter_time = elapsed
            elif (elapsed - cl_enter_time) >= args.cl_dwell:
                print(f"\n\n  CL stable for {args.cl_dwell}s — stopping capture")
                break
        else:
            cl_enter_time = None

        # Check for fault
        if cur_state == 9:  # ESC_FAULT
            fault_name = FAULT_NAMES.get(row["faultCode"], "?")
            print(f"\n\n  FAULT: {fault_name} (code={row['faultCode']})")
            # Capture a few more samples after fault
            for _ in range(5):
                resp = send_cmd(ser, GSP_CMD_GET_SNAPSHOT, timeout=0.2)
                if resp and resp[0] == GSP_CMD_GET_SNAPSHOT:
                    r = parse_snapshot(resp[1], t0, prev_time)
                    if r:
                        prev_time = r.pop("_raw_time", prev_time)
                        r["transition"] = "POST_FAULT"
                        writer.writerow(r)
                        sample_count += 1
            break

        # Live display at ~10 Hz
        now = time.monotonic()
        if now - last_print >= 0.100:
            line = (f"\r  {row['time_s']:>7s}  {row['state_name']:>8s}  "
                    f"{row['focSubName']:>7s}  Iq={row['focIqMeas']:>6s}  "
                    f"Id={row['focIdMeas']:>6s}  {row['focRPM']:>6s}rpm  "
                    f"{row['focVbus']:>5s}V  Ea={row['smoEalpha']:>8s}  "
                    f"Eb={row['smoEbeta']:>8s}")
            print(line, end="", flush=True)
            last_print = now

    # Stop motor (unless --no-stop)
    if not args.no_arm and not args.no_stop:
        print(f"\n  Sending STOP command...")
        send_cmd(ser, GSP_CMD_STOP_MOTOR)

    csvfile.close()
    ser.close()

    # ── Summary ──
    duration = time.monotonic() - t0
    print(f"\n  ═══ Startup Capture Summary ═══")
    print(f"  Duration:   {duration:.2f}s")
    print(f"  Samples:    {sample_count}")
    if duration > 0:
        print(f"  Rate:       {sample_count / duration:.1f} Hz")
    print(f"  CSV file:   {filename}")

    if transitions:
        print(f"\n  ── Transition Timeline ──")
        for t, label in transitions:
            print(f"    t={t:7.3f}s  {label}")

        # Key timing metrics
        t_armed = next((t for t, l in transitions if "ARMED" in l), None)
        t_align = next((t for t, l in transitions if "ALIGN" in l), None)
        t_ol = next((t for t, l in transitions if "OL_RAMP" in l), None)
        t_cl = next((t for t, l in transitions if "->CL" in l), None)
        t_fault = next((t for t, l in transitions if "FAULT" in l), None)

        print(f"\n  ── Key Timings ──")
        if t_armed is not None:
            print(f"    ARM time:        {t_armed:.3f}s")
        if t_align is not None and t_armed is not None:
            print(f"    ARM→ALIGN:       {t_align - t_armed:.3f}s")
        if t_ol is not None and t_align is not None:
            print(f"    ALIGN→OL_RAMP:   {t_ol - t_align:.3f}s")
        if t_cl is not None and t_ol is not None:
            print(f"    OL_RAMP→CL:      {t_cl - t_ol:.3f}s")
        if t_cl is not None and t_armed is not None:
            print(f"    Total startup:   {t_cl - t_armed:.3f}s")
        if t_fault is not None:
            print(f"    FAULT at:        {t_fault:.3f}s")

    print()


if __name__ == "__main__":
    main()
