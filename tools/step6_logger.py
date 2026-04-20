#!/usr/bin/env python3
"""
6-Step Telemetry Logger for dsPIC33AKESC.

Streams the GSP snapshot, decodes only the 6-step-relevant fields
(state, step, duty, Vbus, Ibus, BEMF, ZC health) and prints a
scrolling table while writing a full CSV.

Use when FEATURE_BEMF_CLOSED_LOOP=1 (FOC fields will be zero).

Usage:
    python3 tools/step6_logger.py [--port /dev/ttyACM0] [--rate 50]
                                   [--duration 0] [--poll]

Output:
    logs/step6_YYYYMMDD_HHMMSS.csv
"""

import argparse
import csv
import os
import signal
import struct
import sys
import time
from datetime import datetime
from typing import Optional

import serial


# ── GSP Protocol ────────────────────────────────────────────────────────
GSP_START_BYTE = 0x02

GSP_CMD_PING         = 0x00
GSP_CMD_GET_SNAPSHOT = 0x02
GSP_CMD_TELEM_START  = 0x14
GSP_CMD_TELEM_STOP   = 0x15
GSP_CMD_TELEM_FRAME  = 0x80
GSP_CMD_ERROR        = 0xFF

ESC_STATE_NAMES = {
    0: "IDLE", 1: "ARMED", 2: "DETECT", 3: "ALIGN", 4: "OL_RAMP",
    5: "MORPH", 6: "CL", 7: "BRAKING", 8: "RECOVERY", 9: "FAULT"
}

FAULT_NAMES = {
    0: "NONE", 1: "OV", 2: "UV", 3: "OC_SW", 4: "BOARD_PCI",
    5: "STALL", 6: "DESYNC", 7: "START_TO", 8: "MORPH_TO",
    9: "RX_LOSS", 10: "FOC_INT", 11: "FOC_BUS"
}

# Board scale factors (from garuda_foc_params.h)
#   VBUS_SCALE_V_PER_COUNT = 3.3 * 23.2 / 4095 = 0.01869
#   CURRENT_SCALE_A_PER_COUNT = 3.3 / (4095 * 24.95 * 0.003) = 0.01075
VBUS_SCALE_V = 3.3 * 23.2 / 4095.0
IBUS_SCALE_A = 3.3 / (4095.0 * 24.95 * 0.003)
IBUS_BIAS    = 2048   # ibusRaw is biased at midpoint

# PWM freq for eRPM conversion (from garuda_config.h: PWMFREQUENCY_HZ=24000)
#   SW path: eRPM = PWMFREQUENCY_HZ * 10 / stepPeriod  (adcIsrTick units)
#   HWZC path: eRPM = 1e9 / hwzcStepPeriodHR  (SCCP2 ticks @ 100 MHz = 10 ns each)
ERPM_FROM_STEP_NUM    = 24000 * 10
HWZC_ERPM_FROM_TICKS  = 1000000000


# ── CRC16-CCITT ─────────────────────────────────────────────────────────
def crc16(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def build_packet(cmd_id: int, payload: bytes = b"") -> bytes:
    pkt_len = 1 + len(payload)
    body = bytes([pkt_len, cmd_id]) + payload
    c = crc16(body)
    return bytes([GSP_START_BYTE]) + body + struct.pack(">H", c)


def read_response(ser: serial.Serial, timeout: float = 1.0) -> Optional[tuple]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] == GSP_START_BYTE:
            break
    else:
        return None

    ser.timeout = max(deadline - time.monotonic(), 0.01)
    b = ser.read(1)
    if not b:
        return None
    pkt_len = b[0]

    ser.timeout = max(deadline - time.monotonic(), 0.01)
    data = ser.read(pkt_len + 2)
    if len(data) < pkt_len + 2:
        return None

    cmd_id = data[0]
    payload = data[1:pkt_len]
    rx_crc = struct.unpack(">H", data[pkt_len:pkt_len + 2])[0]
    if crc16(bytes([pkt_len]) + data[:pkt_len]) != rx_crc:
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


# ── Snapshot parse (6-step fields only) ─────────────────────────────────
CSV_COLUMNS = [
    "time_s", "state", "state_name", "fault", "fault_name",
    "currentStep", "direction", "throttle", "dutyPct",
    "vbusRaw", "vbus_V", "ibusRaw", "ibus_A", "ibusMax", "ibusMax_A",
    "bemfRaw", "zcThreshold", "stepPeriod", "eRPM", "goodZcCount",
    "risingZcWorks", "fallingZcWorks", "zcSynced",
    "zcConfirmedCount", "zcTimeoutForceCount",
    "hwzcEnabled", "hwzcPhase", "hwzcTotalZcCount",
    "hwzcTotalMissCount", "hwzcStepPeriodHR", "hwzcNoiseReject",
    "clpciTripCount", "fpciTripCount",
    "systemTick", "uptimeSec",
    "iaRaw", "ibRaw", "iaMax", "iaMin", "ibMax", "ibMin",
    "ia_A", "ib_A",
    "iaPkPos_A", "iaPkNeg_A", "ibPkPos_A", "ibPkNeg_A",
    "iaPkMag_A", "ibPkMag_A",
    "iaAtFault_A", "ibAtFault_A",
    "iaPkAtFault_A", "ibPkAtFault_A",
    "iaPkPosAtFault_A", "iaPkNegAtFault_A",
    "ibusPkPos_A", "ibusPkNeg_A", "ibusPkMag_A",
    "ibusAtFault_A", "ibusPkPosAtFault_A", "ibusPkNegAtFault_A", "ibusPkAtFault_A",
]

# Phase-current scale: ~93 ADC counts/A on MCLV-48V-300W (3mΩ shunt, 24.95 gain, 3.3V Vref, 12-bit)
IADC_BIAS = 2048         # 0 A at ADC_midpoint
IADC_COUNTS_PER_AMP = 93.0


def parse_snapshot(p: bytes, t0: float) -> Optional[dict]:
    """Parse the 6-step-relevant fields. Requires snapshot >= 174 bytes
    (with hwzcNoiseReject at offset 170). Falls back gracefully on shorter
    snapshots from older firmware."""
    if len(p) < 68:
        return None

    state, fault, step, direction, throttle, duty = \
        struct.unpack_from("<BBBBHBx", p, 0)
    vbus_raw, ibus_raw, ibus_max = struct.unpack_from("<HHH", p, 8)
    bemf_raw, zc_thresh, step_period, good_zc = \
        struct.unpack_from("<HHHH", p, 14)
    rising, falling, synced = struct.unpack_from("<BBB", p, 22)
    zc_confirmed, zc_timeout = struct.unpack_from("<HH", p, 26)
    hwzc_en, hwzc_phase = struct.unpack_from("<BB", p, 30)
    hwzc_zc, hwzc_miss, hwzc_hr = struct.unpack_from("<III", p, 32)
    clpci, fpci = struct.unpack_from("<II", p, 52)
    sys_tick, uptime = struct.unpack_from("<II", p, 60)
    hwzc_reject = struct.unpack_from("<I", p, 170)[0] if len(p) >= 174 else 0
    # Default zeros for IBUS fields (older firmware compat)
    ibus_win_max = 0
    ibus_win_min = 0xFFFF
    ibus_at_flt = ibus_max_flt = 0
    ibus_min_flt = 0
    if len(p) >= 208:
        (ia_raw, ib_raw, ia_max, ia_min, ib_max, ib_min,
         ia_at_flt, ib_at_flt, ia_max_flt, ia_min_flt, ib_max_flt, ib_min_flt,
         ibus_win_max, ibus_win_min,
         ibus_at_flt, ibus_max_flt, ibus_min_flt) = \
            struct.unpack_from("<HHHHHHHHHHHHHHHHH", p, 174)
    elif len(p) >= 198:
        (ia_raw, ib_raw, ia_max, ia_min, ib_max, ib_min,
         ia_at_flt, ib_at_flt, ia_max_flt, ia_min_flt, ib_max_flt, ib_min_flt) = \
            struct.unpack_from("<HHHHHHHHHHHH", p, 174)
    elif len(p) >= 186:
        ia_raw, ib_raw, ia_max, ia_min, ib_max, ib_min = struct.unpack_from("<HHHHHH", p, 174)
        ia_at_flt = ib_at_flt = ia_max_flt = ib_max_flt = 0
        ia_min_flt = ib_min_flt = 0
    else:
        ia_raw = ib_raw = ia_max = ib_max = 0
        ia_min = ib_min = 0xFFFF
        ia_at_flt = ib_at_flt = ia_max_flt = ib_max_flt = 0
        ia_min_flt = ib_min_flt = 0

    # Scale raw ADC to amps (deviation from bias, +ve = positive current)
    def _to_a(raw):
        if raw == 0 or raw == 0xFFFF:
            return 0.0
        return (raw - IADC_BIAS) / IADC_COUNTS_PER_AMP
    ia_inst_a = _to_a(ia_raw)
    ib_inst_a = _to_a(ib_raw)
    ia_pk_pos_a = _to_a(ia_max)
    ia_pk_neg_a = _to_a(ia_min)
    ib_pk_pos_a = _to_a(ib_max)
    ib_pk_neg_a = _to_a(ib_min)
    # Max-magnitude peak per phase in the last ~20ms window
    ia_pk_mag_a = max(abs(ia_pk_pos_a), abs(ia_pk_neg_a))
    ib_pk_mag_a = max(abs(ib_pk_pos_a), abs(ib_pk_neg_a))
    # Frozen-at-fault snapshot (what did currents do in the 20ms window ending at the trip?)
    ia_flt_a = _to_a(ia_at_flt)
    ib_flt_a = _to_a(ib_at_flt)
    ia_flt_pk_pos = _to_a(ia_max_flt)
    ia_flt_pk_neg = _to_a(ia_min_flt)
    ib_flt_pk_pos = _to_a(ib_max_flt)
    ib_flt_pk_neg = _to_a(ib_min_flt)
    ia_flt_pk_mag = max(abs(ia_flt_pk_pos), abs(ia_flt_pk_neg))
    ib_flt_pk_mag = max(abs(ib_flt_pk_pos), abs(ib_flt_pk_neg))
    # Bus-current window + at-fault (same 93 counts/A scale — OA3 gain matches OA1/OA2)
    ibus_pk_pos_a = _to_a(ibus_win_max)
    ibus_pk_neg_a = _to_a(ibus_win_min)
    ibus_pk_mag_a = max(abs(ibus_pk_pos_a), abs(ibus_pk_neg_a))
    ibus_flt_a = _to_a(ibus_at_flt)
    ibus_flt_pk_pos = _to_a(ibus_max_flt)
    ibus_flt_pk_neg = _to_a(ibus_min_flt)
    ibus_flt_pk_mag = max(abs(ibus_flt_pk_pos), abs(ibus_flt_pk_neg))

    vbus_v = vbus_raw * VBUS_SCALE_V
    ibus_a = (ibus_raw - IBUS_BIAS) * IBUS_SCALE_A
    ibus_max_a = (ibus_max - IBUS_BIAS) * IBUS_SCALE_A if ibus_max else 0.0
    # Prefer HWZC-derived eRPM when HWZC is enabled and has a valid period;
    # fall back to SW stepPeriod otherwise.
    if hwzc_en and hwzc_hr > 0:
        erpm = HWZC_ERPM_FROM_TICKS // hwzc_hr
    elif step_period > 0:
        erpm = ERPM_FROM_STEP_NUM // step_period
    else:
        erpm = 0

    return {
        "time_s":               f"{time.monotonic() - t0:.3f}",
        "state":                state,
        "state_name":           ESC_STATE_NAMES.get(state, f"?{state}"),
        "fault":                fault,
        "fault_name":           FAULT_NAMES.get(fault, f"?{fault}"),
        "currentStep":          step,
        "direction":            direction,
        "throttle":             throttle,
        "dutyPct":              duty,
        "vbusRaw":              vbus_raw,
        "vbus_V":               f"{vbus_v:.2f}",
        "ibusRaw":              ibus_raw,
        "ibus_A":               f"{ibus_a:.2f}",
        "ibusMax":              ibus_max,
        "ibusMax_A":            f"{ibus_max_a:.2f}",
        "bemfRaw":              bemf_raw,
        "zcThreshold":          zc_thresh,
        "stepPeriod":           step_period,
        "eRPM":                 erpm,
        "goodZcCount":          good_zc,
        "risingZcWorks":        rising,
        "fallingZcWorks":       falling,
        "zcSynced":             synced,
        "zcConfirmedCount":     zc_confirmed,
        "zcTimeoutForceCount":  zc_timeout,
        "hwzcEnabled":          hwzc_en,
        "hwzcPhase":            hwzc_phase,
        "hwzcTotalZcCount":     hwzc_zc,
        "hwzcTotalMissCount":   hwzc_miss,
        "hwzcStepPeriodHR":     hwzc_hr,
        "hwzcNoiseReject":      hwzc_reject,
        "iaRaw":                ia_raw,
        "ibRaw":                ib_raw,
        "iaMax":                ia_max,
        "iaMin":                ia_min,
        "ibMax":                ib_max,
        "ibMin":                ib_min,
        "ia_A":                 f"{ia_inst_a:+.1f}",
        "ib_A":                 f"{ib_inst_a:+.1f}",
        "iaPkPos_A":            f"{ia_pk_pos_a:+.1f}",
        "iaPkNeg_A":            f"{ia_pk_neg_a:+.1f}",
        "ibPkPos_A":            f"{ib_pk_pos_a:+.1f}",
        "ibPkNeg_A":            f"{ib_pk_neg_a:+.1f}",
        "iaPkMag_A":            f"{ia_pk_mag_a:.1f}",
        "ibPkMag_A":            f"{ib_pk_mag_a:.1f}",
        "iaAtFault_A":          f"{ia_flt_a:+.1f}",
        "ibAtFault_A":          f"{ib_flt_a:+.1f}",
        "iaPkAtFault_A":        f"{ia_flt_pk_mag:.1f}",
        "ibPkAtFault_A":        f"{ib_flt_pk_mag:.1f}",
        "iaPkPosAtFault_A":     f"{ia_flt_pk_pos:+.1f}",
        "iaPkNegAtFault_A":     f"{ia_flt_pk_neg:+.1f}",
        "ibusPkPos_A":          f"{ibus_pk_pos_a:+.1f}",
        "ibusPkNeg_A":          f"{ibus_pk_neg_a:+.1f}",
        "ibusPkMag_A":          f"{ibus_pk_mag_a:.1f}",
        "ibusAtFault_A":        f"{ibus_flt_a:+.1f}",
        "ibusPkPosAtFault_A":   f"{ibus_flt_pk_pos:+.1f}",
        "ibusPkNegAtFault_A":   f"{ibus_flt_pk_neg:+.1f}",
        "ibusPkAtFault_A":      f"{ibus_flt_pk_mag:.1f}",
        "clpciTripCount":       clpci,
        "fpciTripCount":        fpci,
        "systemTick":           sys_tick,
        "uptimeSec":            uptime,
    }


# ── Main ────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="6-step Telemetry Logger")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate", type=int, default=50,
                    help="Telemetry rate in Hz (streaming mode)")
    ap.add_argument("--duration", type=int, default=0,
                    help="Duration in seconds (0 = until Ctrl+C)")
    ap.add_argument("--poll", action="store_true",
                    help="Poll GET_SNAPSHOT instead of stream")
    ap.add_argument("--csv", default=None, help="Override CSV path")
    args = ap.parse_args()

    print(f"Connecting to {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    time.sleep(0.3)
    drain(ser)

    resp = send_cmd(ser, GSP_CMD_PING)
    if not resp or resp[0] == GSP_CMD_ERROR:
        print("  PING failed — is the board powered and firmware flashed?")
        ser.close()
        return 1
    print("  PING OK")

    # CSV setup
    os.makedirs("logs", exist_ok=True)
    csv_path = args.csv or (
        f"logs/step6_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    csvfile = open(csv_path, "w", newline="")
    writer = csv.DictWriter(csvfile, fieldnames=CSV_COLUMNS)
    writer.writeheader()
    print(f"  Logging to: {os.path.abspath(csv_path)}")

    # Start streaming
    if not args.poll:
        resp = send_cmd(ser, GSP_CMD_TELEM_START,
                        payload=struct.pack("<B", args.rate))
        if not resp:
            print("  TELEM_START failed — falling back to poll")
            args.poll = True
        else:
            print(f"  Telemetry streaming at {args.rate} Hz")

    # Install clean Ctrl+C handler
    stop = {"flag": False}
    def sigint(signum, frame):
        stop["flag"] = True
    signal.signal(signal.SIGINT, sigint)

    t0 = time.monotonic()
    sample_count = 0
    state_changes = 0
    last_state = None
    last_print = 0.0
    buf = b""

    # Header — print once
    hdr = (f"{'time':>7s}  {'state':>8s}  {'st':>2s} "
           f"{'thr':>4s} {'duty':>4s}  "
           f"{'eRPM':>6s} {'hw':>1s}  {'Tstep':>5s} {'hwT':>7s}  "
           f"{'Vbus':>5s} {'Ibus':>6s}  "
           f"{'bemf':>4s} {'zcTh':>4s}  "
           f"{'gZc':>4s} {'RFS':>3s}  "
           f"{'hwZC':>6s} {'hwMs':>5s} {'rej':>5s}  "
           f"{'Ia_now(pk)':>14s} {'Ibus(pk)':>10s}   "
           f"{'fault':>8s}")
    print("\n" + "─" * len(hdr))
    print(hdr)
    print("─" * len(hdr))

    deadline = t0 + args.duration if args.duration > 0 else float("inf")

    while not stop["flag"] and time.monotonic() < deadline:
        row = None
        if args.poll:
            resp = send_cmd(ser, GSP_CMD_GET_SNAPSHOT, timeout=0.3)
            if resp and resp[0] == GSP_CMD_GET_SNAPSHOT:
                row = parse_snapshot(resp[1], t0)
            else:
                time.sleep(max(0.0, 1.0 / args.rate - 0.005))
        else:
            chunk = ser.read(256)
            if not chunk:
                continue
            buf += chunk
            # Pull all complete packets out of buf
            while True:
                idx = buf.find(bytes([GSP_START_BYTE]))
                if idx < 0:
                    buf = b""
                    break
                if idx > 0:
                    buf = buf[idx:]
                if len(buf) < 2:
                    break
                pkt_len = buf[1]
                total = 2 + pkt_len + 2
                if len(buf) < total:
                    break
                body = buf[1:2 + pkt_len]
                rx_crc = struct.unpack(">H", buf[2+pkt_len:2+pkt_len+2])[0]
                if crc16(body) != rx_crc:
                    buf = buf[1:]
                    continue
                cmd = buf[2]
                payload = buf[3:2+pkt_len]
                buf = buf[total:]
                if cmd == GSP_CMD_TELEM_FRAME:
                    # TELEM_FRAME has a 2-byte sequence number prefix
                    # before the snapshot — strip it.
                    snap = payload[2:] if len(payload) > 2 else payload
                    row = parse_snapshot(snap, t0)

        if not row:
            continue

        writer.writerow(row)
        csvfile.flush()
        sample_count += 1

        # Always print on state change; else rate-limit to ~5 Hz
        now = time.monotonic()
        state_changed = (row["state_name"] != last_state)
        if state_changed:
            state_changes += 1
            last_state = row["state_name"]

        if state_changed or (now - last_print >= 0.2):
            line = (f"{row['time_s']:>7s}  "
                    f"{row['state_name']:>8s}  "
                    f"{row['currentStep']:>2d} "
                    f"{row['throttle']:>4d} "
                    f"{row['dutyPct']:>3d}%  "
                    f"{row['eRPM']:>6d} {row['hwzcEnabled']:>1d}  "
                    f"{row['stepPeriod']:>5d} {row['hwzcStepPeriodHR']:>7d}  "
                    f"{row['vbus_V']:>5s} "
                    f"{row['ibus_A']:>6s}  "
                    f"{row['bemfRaw']:>4d} "
                    f"{row['zcThreshold']:>4d}  "
                    f"{row['goodZcCount']:>4d} "
                    f"{row['risingZcWorks']:>1d}"
                    f"{row['fallingZcWorks']:>1d}"
                    f"{row['zcSynced']:>1d}  "
                    f"{row['hwzcTotalZcCount']:>6d} "
                    f"{row['hwzcTotalMissCount']:>5d} "
                    f"{row['hwzcNoiseReject']:>5d}  "
                    f"Ia{row['ia_A']:>6s}(pk{row['iaPkMag_A']:>4s}) "
                    f"Ibus(pk{row['ibusPkMag_A']:>4s})  "
                    f"{row['fault_name']:>8s}")
            marker = " ◀" if state_changed else ""
            print(line + marker)
            # On transition into BOARD_PCI, print the frozen-at-fault snapshot.
            if state_changed and row['fault_name'] == 'BOARD_PCI':
                print(f"   >>> AT-FAULT Ia:  inst={row['iaAtFault_A']} A  "
                      f"pk±={row['iaPkPosAtFault_A']}/{row['iaPkNegAtFault_A']} A  "
                      f"|pk|={row['iaPkAtFault_A']} A")
                print(f"   >>> AT-FAULT Ibus: inst={row['ibusAtFault_A']} A  "
                      f"pk±={row['ibusPkPosAtFault_A']}/{row['ibusPkNegAtFault_A']} A  "
                      f"|pk|={row['ibusPkAtFault_A']} A")
            last_print = now

    # Shutdown
    if not args.poll:
        send_cmd(ser, GSP_CMD_TELEM_STOP)
    csvfile.close()
    ser.close()

    duration = time.monotonic() - t0
    print(f"\n── Summary ──")
    print(f"  Duration: {duration:.1f}s")
    print(f"  Samples:  {sample_count}")
    print(f"  State transitions: {state_changes}")
    print(f"  CSV:      {os.path.abspath(csv_path)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
