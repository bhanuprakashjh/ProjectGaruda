#!/usr/bin/env python3
"""
FOC Telemetry Logger — captures all FOC snapshot fields to CSV.

Connects via GSP, enables telemetry streaming, parses 114-byte snapshots,
and writes every field to a timestamped CSV file in logs/.

Usage:
    python3 tools/foc_logger.py [--port /dev/ttyACM0] [--rate 50] [--duration 30]

The CSV file is written to logs/foc_YYYYMMDD_HHMMSS.csv and can be
read by Claude for closed-loop AI-assisted debugging.

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
GSP_CMD_STOP_MOTOR   = 0x04
GSP_CMD_CLEAR_FAULT  = 0x05
GSP_CMD_TELEM_START  = 0x14
GSP_CMD_TELEM_STOP   = 0x15
GSP_CMD_TELEM_FRAME  = 0x80
GSP_CMD_ERROR        = 0xFF

GSP_SNAPSHOT_SIZE = 150

ESC_STATE_NAMES = {
    0: "IDLE", 1: "ARMED", 2: "DETECT", 3: "ALIGN", 4: "OL_RAMP",
    5: "MORPH", 6: "CL", 7: "BRAKING", 8: "RECOVERY", 9: "FAULT"
}

FOC_SUB_NAMES = {
    0: "idle", 1: "armed", 2: "align", 3: "if_ramp", 4: "closed_loop"
}

FAULT_NAMES = {
    0: "NONE", 1: "OVERVOLT", 2: "UNDERVOLT", 3: "OC_SW", 4: "BOARD_PCI",
    5: "STALL", 6: "DESYNC", 7: "STARTUP_TO", 8: "MORPH_TO",
    9: "RX_LOSS", 10: "FOC_INTERNAL", 11: "FOC_BUSLOSS"
}

# CSV column names — matches the snapshot struct exactly
CSV_COLUMNS = [
    "time_s",
    # Core state (8B)
    "state", "state_name", "faultCode", "fault_name",
    "currentStep", "direction", "throttle", "dutyPct",
    # Bus (6B)
    "vbusRaw", "ibusRaw", "ibusMax",
    # BEMF/ZC (8B)
    "bemfRaw", "zcThreshold", "stepPeriod", "goodZcCount",
    # ZC flags (3B)
    "risingZcWorks", "fallingZcWorks", "zcSynced",
    # ZC diag (4B)
    "zcConfirmedCount", "zcTimeoutForceCount",
    # HWZC (18B)
    "hwzcEnabled", "hwzcPhase", "hwzcTotalZcCount",
    "hwzcTotalMissCount", "hwzcStepPeriodHR",
    "hwzcDbgLatchDisable",
    # Morph (6B)
    "morphSubPhase", "morphStep", "morphZcCount", "morphAlpha",
    # OC (8B)
    "clpciTripCount", "fpciTripCount",
    # System (8B)
    "systemTick", "uptimeSec",
    # FOC (46B)
    "focIdMeas", "focIqMeas", "focTheta", "focOmega", "focVbus",
    "focIa", "focIb", "focThetaObs", "focVd", "focVq",
    # Observer internals
    "focFluxAlpha", "focFluxBeta", "focLambdaEst", "focObsGain",
    # PI controller internals
    "focPidDInteg", "focPidQInteg", "focPidSpdInteg",
    # Derived diagnostics
    "focModIndex", "focObsConfidence",
    "focSubState", "focSubName", "focOffsetIa", "focOffsetIb",
    # Derived
    "focRPM", "focPower",
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


# ── Snapshot Parser ─────────────────────────────────────────────────────

def parse_snapshot(p: bytes, t0: float) -> Optional[dict]:
    """Parse a 150-byte snapshot into a dict matching CSV_COLUMNS."""
    if len(p) < 68:
        return None

    v = struct.unpack_from("<BBBBHBx", p, 0)
    state, fault, step, direction, throttle, duty = v

    vbus_raw, ibus_raw, ibus_max = struct.unpack_from("<HHH", p, 8)
    bemf_raw, zc_thresh, step_period, good_zc = struct.unpack_from("<HHHH", p, 14)
    rising, falling, synced = struct.unpack_from("<BBB", p, 22)
    zc_confirmed, zc_timeout = struct.unpack_from("<HH", p, 26)
    hwzc_en, hwzc_phase = struct.unpack_from("<BB", p, 30)
    hwzc_zc, hwzc_miss, hwzc_hr = struct.unpack_from("<III", p, 32)
    hwzc_latch, = struct.unpack_from("<B", p, 44)
    morph_sub, morph_step, morph_zc, morph_alpha = struct.unpack_from("<BBHH", p, 46)
    clpci, fpci = struct.unpack_from("<II", p, 52)
    sys_tick, uptime = struct.unpack_from("<II", p, 60)

    # FOC fields
    foc_id = foc_iq = foc_theta = foc_omega = foc_vbus = 0.0
    foc_ia = foc_ib = foc_theta_obs = foc_vd = foc_vq = 0.0
    foc_flux_a = foc_flux_b = foc_lambda_est = foc_obs_gain = 0.0
    foc_pid_d = foc_pid_q = foc_pid_spd = 0.0
    foc_mod_idx = foc_obs_conf = 0.0
    foc_sub = 0
    foc_off_ia = foc_off_ib = 0

    if len(p) >= 150:
        # V3 format with observer/PI/diag fields
        foc_id, foc_iq, foc_theta, foc_omega, foc_vbus = \
            struct.unpack_from("<fffff", p, 68)
        foc_ia, foc_ib, foc_theta_obs, foc_vd, foc_vq = \
            struct.unpack_from("<fffff", p, 88)
        foc_flux_a, foc_flux_b, foc_lambda_est, foc_obs_gain = \
            struct.unpack_from("<ffff", p, 108)
        foc_pid_d, foc_pid_q, foc_pid_spd = \
            struct.unpack_from("<fff", p, 124)
        foc_mod_idx, foc_obs_conf = \
            struct.unpack_from("<ff", p, 136)
        foc_sub, = struct.unpack_from("<B", p, 144)
        foc_off_ia, foc_off_ib = struct.unpack_from("<HH", p, 146)
    elif len(p) >= 114:
        # V2 format with Vd/Vq
        foc_id, foc_iq, foc_theta, foc_omega, foc_vbus = \
            struct.unpack_from("<fffff", p, 68)
        foc_ia, foc_ib, foc_theta_obs, foc_vd, foc_vq = \
            struct.unpack_from("<fffff", p, 88)
        foc_sub, = struct.unpack_from("<B", p, 108)
        foc_off_ia, foc_off_ib = struct.unpack_from("<HH", p, 110)
    elif len(p) >= 106:
        # V1 format without Vd/Vq
        foc_id, foc_iq, foc_theta, foc_omega, foc_vbus = \
            struct.unpack_from("<fffff", p, 68)
        foc_ia, foc_ib, foc_theta_obs = \
            struct.unpack_from("<fff", p, 88)
        foc_sub, = struct.unpack_from("<B", p, 100)
        foc_off_ia, foc_off_ib = struct.unpack_from("<HH", p, 102)

    # Derived
    foc_rpm = abs(foc_omega) * 60.0 / (2.0 * 3.14159265)
    foc_power = foc_vq * foc_iq + foc_vd * foc_id  # P = Vd*Id + Vq*Iq

    return {
        "time_s": f"{time.monotonic() - t0:.3f}",
        "state": state,
        "state_name": ESC_STATE_NAMES.get(state, f"?{state}"),
        "faultCode": fault,
        "fault_name": FAULT_NAMES.get(fault, f"?{fault}"),
        "currentStep": step,
        "direction": direction,
        "throttle": throttle,
        "dutyPct": duty,
        "vbusRaw": vbus_raw,
        "ibusRaw": ibus_raw,
        "ibusMax": ibus_max,
        "bemfRaw": bemf_raw,
        "zcThreshold": zc_thresh,
        "stepPeriod": step_period,
        "goodZcCount": good_zc,
        "risingZcWorks": rising,
        "fallingZcWorks": falling,
        "zcSynced": synced,
        "zcConfirmedCount": zc_confirmed,
        "zcTimeoutForceCount": zc_timeout,
        "hwzcEnabled": hwzc_en,
        "hwzcPhase": hwzc_phase,
        "hwzcTotalZcCount": hwzc_zc,
        "hwzcTotalMissCount": hwzc_miss,
        "hwzcStepPeriodHR": hwzc_hr,
        "hwzcDbgLatchDisable": hwzc_latch,
        "morphSubPhase": morph_sub,
        "morphStep": morph_step,
        "morphZcCount": morph_zc,
        "morphAlpha": morph_alpha,
        "clpciTripCount": clpci,
        "fpciTripCount": fpci,
        "systemTick": sys_tick,
        "uptimeSec": uptime,
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
        "focFluxAlpha": f"{foc_flux_a:.6f}",
        "focFluxBeta": f"{foc_flux_b:.6f}",
        "focLambdaEst": f"{foc_lambda_est:.6f}",
        "focObsGain": f"{foc_obs_gain:.4f}",
        "focPidDInteg": f"{foc_pid_d:.4f}",
        "focPidQInteg": f"{foc_pid_q:.4f}",
        "focPidSpdInteg": f"{foc_pid_spd:.4f}",
        "focModIndex": f"{foc_mod_idx:.4f}",
        "focObsConfidence": f"{foc_obs_conf:.4f}",
        "focSubState": foc_sub,
        "focSubName": FOC_SUB_NAMES.get(foc_sub, f"?{foc_sub}"),
        "focOffsetIa": foc_off_ia,
        "focOffsetIb": foc_off_ib,
        "focRPM": f"{foc_rpm:.1f}",
        "focPower": f"{foc_power:.3f}",
    }


# ── Telemetry Stream Reader ────────────────────────────────────────────

def read_telem_frame(ser: serial.Serial, timeout: float = 0.5) -> Optional[bytes]:
    """Read one telemetry frame (unsolicited GSP_CMD_TELEM_FRAME)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ser.timeout = max(deadline - time.monotonic(), 0.001)
        b = ser.read(1)
        if len(b) == 0:
            continue
        if b[0] != GSP_START_BYTE:
            continue

        ser.timeout = max(deadline - time.monotonic(), 0.01)
        b = ser.read(1)
        if len(b) == 0:
            continue
        pkt_len = b[0]

        ser.timeout = max(deadline - time.monotonic(), 0.01)
        data = ser.read(pkt_len + 2)
        if len(data) < pkt_len + 2:
            continue

        cmd_id = data[0]
        payload = data[1:pkt_len]
        rx_crc = struct.unpack(">H", data[pkt_len:pkt_len + 2])[0]

        crc_data = bytes([pkt_len]) + data[:pkt_len]
        if crc16_ccitt(crc_data) != rx_crc:
            continue

        if cmd_id == GSP_CMD_TELEM_FRAME:
            # Strip 2-byte sequence prefix (seq_lo, seq_hi)
            return payload[2:] if len(payload) > 2 else payload

    return None


# ── Main ────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="FOC Telemetry Logger")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--rate", type=int, default=50,
                        help="Telemetry rate in Hz (default 50)")
    parser.add_argument("--duration", type=int, default=0,
                        help="Duration in seconds (0=until Ctrl+C)")
    parser.add_argument("--poll", action="store_true",
                        help="Use polling (GET_SNAPSHOT) instead of streaming")
    args = parser.parse_args()

    # Output file
    logs_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "logs")
    os.makedirs(logs_dir, exist_ok=True)
    filename = os.path.join(logs_dir,
                            f"foc_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

    print(f"Connecting to {args.port} @ {args.baud}...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    time.sleep(0.2)
    drain(ser)

    # Verify connectivity
    resp = send_cmd(ser, GSP_CMD_PING)
    if resp is None or resp[0] != GSP_CMD_PING:
        print("ERROR: No PING response — check connection")
        ser.close()
        sys.exit(1)
    print("  PING OK")

    # Get info
    resp = send_cmd(ser, GSP_CMD_GET_INFO)
    if resp and resp[0] == GSP_CMD_GET_INFO and len(resp[1]) >= 14:
        proto_ver = resp[1][0]
        fw_maj    = resp[1][1]
        fw_min    = resp[1][2]
        fw_pat    = resp[1][3]
        flags     = struct.unpack_from("<I", resp[1], 8)[0]
        is_foc = bool(flags & (1 << 23))

        # FOC variant from feature flags bits 23 + others.
        # Logger can't directly tell V2/V3/AN1078 apart from flags alone
        # — they all set bit 23.  Use buildHash + fw version as primary
        # build identifier instead.
        print(f"  GSP proto v{proto_ver}  FW v{fw_maj}.{fw_min}.{fw_pat}")

        # buildHash (V3+ protocol, bytes 20-23 of GSP_INFO_T)
        if proto_ver >= 3 and len(resp[1]) >= 24:
            build_hash = struct.unpack_from("<I", resp[1], 20)[0]
            print(f"  Build hash: 0x{build_hash:08X}")
        else:
            print(f"  Build hash: (unsupported, proto v{proto_ver} < 3)")

        print(f"  Mode: {'FOC' if is_foc else '6-step'}")
        print(f"  Feature flags: 0x{flags:08X}")
    else:
        is_foc = False

    # Open CSV
    csvfile = open(filename, "w", newline="")
    writer = csv.DictWriter(csvfile, fieldnames=CSV_COLUMNS)
    writer.writeheader()
    print(f"  Logging to: {filename}")

    # Start telemetry (if streaming mode)
    if not args.poll:
        rate_hz = max(min(args.rate, 100), 10)
        resp = send_cmd(ser, GSP_CMD_TELEM_START, bytes([rate_hz]))
        if resp is None or resp[0] != GSP_CMD_TELEM_START:
            print(f"  WARNING: TELEM_START failed — falling back to poll mode")
            args.poll = True
        else:
            actual_hz = resp[1][0] if resp[1] else rate_hz
            print(f"  Telemetry streaming at {actual_hz} Hz")

    # Verify snapshot size with one test read
    test_resp = send_cmd(ser, GSP_CMD_GET_SNAPSHOT)
    if test_resp and test_resp[0] == GSP_CMD_GET_SNAPSHOT:
        snap_size = len(test_resp[1])
        fmt_name = ('v3 obs/PI/diag' if snap_size >= 150
                    else 'v2 Vd/Vq' if snap_size >= 114
                    else 'v1' if snap_size >= 106 else 'short')
        print(f"  Snapshot size: {snap_size} bytes ({fmt_name})")
        # Hex dump first 20 bytes for debug
        hexdump = test_resp[1][:20].hex(' ')
        print(f"  First 20 bytes: {hexdump}")
        # Parse state from first byte to sanity check
        test_state = test_resp[1][0]
        print(f"  State byte: {test_state} ({ESC_STATE_NAMES.get(test_state, '?')})")
    else:
        print("  WARNING: Could not read test snapshot")

    # Capture loop
    t0 = time.monotonic()
    sample_count = 0
    fault_count = 0
    stop = False

    def signal_handler(sig, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, signal_handler)

    print(f"\n  Recording... (Ctrl+C to stop)\n")
    print(f"  {'Time':>7s}  {'State':>8s}  {'Sub':>5s}  {'Iq':>7s}  "
          f"{'Id':>7s}  {'Vq':>7s}  {'Vd':>7s}  {'RPM':>7s}  {'Vbus':>6s}  "
          f"{'Thr':>5s}  {'Fault':>6s}")
    print(f"  {'─'*7}  {'─'*8}  {'─'*5}  {'─'*7}  {'─'*7}  {'─'*7}  "
          f"{'─'*7}  {'─'*7}  {'─'*6}  {'─'*5}  {'─'*6}")

    last_print = 0.0

    while not stop:
        if args.duration > 0 and (time.monotonic() - t0) >= args.duration:
            break

        # Get snapshot data
        if args.poll:
            resp = send_cmd(ser, GSP_CMD_GET_SNAPSHOT, timeout=0.5)
            if resp is None or resp[0] != GSP_CMD_GET_SNAPSHOT:
                continue
            payload = resp[1]
            time.sleep(1.0 / args.rate)
        else:
            payload = read_telem_frame(ser, timeout=1.0)
            if payload is None:
                # Try a ping to keep connection alive
                send_cmd(ser, GSP_CMD_PING)
                continue

        row = parse_snapshot(payload, t0)
        if row is None:
            continue

        writer.writerow(row)
        sample_count += 1

        if row["faultCode"] != 0:
            fault_count += 1

        # Live display at ~5 Hz
        now = time.monotonic()
        if now - last_print >= 0.200:
            state_name = row["state_name"]
            sub_name = row["focSubName"]
            foc_iq = row["focIqMeas"]
            foc_id = row["focIdMeas"]
            foc_vq = row["focVq"]
            foc_vd = row["focVd"]
            rpm = row["focRPM"]
            vbus = row["focVbus"]
            thr = row["throttle"]
            fault = row["fault_name"]
            mod_idx = row["focModIndex"]
            obs_conf = row["focObsConfidence"]

            line = (f"\r  {row['time_s']:>7s}  {state_name:>8s}  {sub_name:>5s}  "
                    f"Iq={foc_iq:>6s}  Id={foc_id:>6s}  Vq={foc_vq:>6s}  "
                    f"{rpm:>6s}rpm  {vbus:>5s}V  mod={mod_idx:>5s}  "
                    f"conf={obs_conf:>5s}  {fault:>6s}")
            print(line, end="", flush=True)
            last_print = now

    # Stop telemetry
    if not args.poll:
        send_cmd(ser, GSP_CMD_TELEM_STOP)

    csvfile.close()
    ser.close()

    # Summary
    duration = time.monotonic() - t0
    print(f"\n\n  ── Summary ──")
    print(f"  Duration:   {duration:.1f}s")
    print(f"  Samples:    {sample_count}")
    print(f"  Rate:       {sample_count / duration:.1f} Hz" if duration > 0 else "")
    print(f"  Faults:     {fault_count}")
    print(f"  CSV file:   {filename}")
    print()


if __name__ == "__main__":
    main()
