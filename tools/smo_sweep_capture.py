#!/usr/bin/env python3
"""
SMO Sweep Capture — automated throttle sweep with full telemetry capture.

Arms the motor, waits for CL, then runs a programmable throttle profile:
  1. Ramp up to target throttle over ramp_time
  2. Hold at target for hold_time
  3. Ramp down to idle over ramp_time
  4. Hold idle for hold_time
  5. Stop motor

Captures snapshots at max rate (~70 Hz) throughout. Auto-stops on fault
or Ctrl+C. All data saved to CSV for analysis.

Usage:
    python3 tools/smo_sweep_capture.py
    python3 tools/smo_sweep_capture.py --max-throttle 3000 --ramp-time 3 --hold-time 2
    python3 tools/smo_sweep_capture.py --manual   # manual pot control, capture until Ctrl+C

Output:
    logs/smo_sweep_YYYYMMDD_HHMMSS.csv
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
GSP_CMD_SET_THROTTLE     = 0x06
GSP_CMD_SET_THROTTLE_SRC = 0x07

THROTTLE_SRC_ADC = 0
THROTTLE_SRC_GSP = 1

ESC_STATE_NAMES = {
    0: "IDLE", 1: "ARMED", 2: "DETECT", 3: "ALIGN", 4: "OL_RAMP",
    5: "MORPH", 6: "CL", 7: "BRAKING", 8: "RECOVERY", 9: "FAULT"
}

FOC_SUB_NAMES = {
    0: "idle", 1: "armed", 2: "align", 3: "ol_ramp", 4: "cl", 5: "vf_assist"
}

FAULT_NAMES = {
    0: "NONE", 1: "OVERVOLT", 2: "UNDERVOLT", 3: "OC_SW", 4: "BOARD_PCI",
    5: "STALL", 6: "DESYNC", 7: "STARTUP_TO", 8: "MORPH_TO",
    9: "RX_LOSS", 10: "FOC_INTERNAL", 11: "FOC_BUSLOSS"
}

CSV_COLUMNS = [
    "time_s", "dt_ms", "phase", "cmd_throttle",
    "state", "state_name", "faultCode", "fault_name", "focSubState", "focSubName",
    "throttle", "dutyPct", "vbusRaw", "ibusRaw",
    "focIdMeas", "focIqMeas", "focTheta", "focOmega", "focVbus",
    "focIa", "focIb", "focThetaObs", "focVd", "focVq",
    "smoEalpha", "smoEbeta", "omegaFilt", "smoGain",
    "focPidDInteg", "focPidQInteg", "focPidSpdInteg",
    "focModIndex", "focObsConfidence",
    "focOffsetIa", "focOffsetIb",
    "smoResidual", "pllInnovation", "pllOmega", "omegaOl",
    "handoffCtr", "smoObservable",
    "focRPM", "focPower",
    "transition", "systemTick",
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


def set_throttle_src(ser: serial.Serial, src: int) -> bool:
    """Switch throttle source (must be in IDLE state)."""
    resp = send_cmd(ser, GSP_CMD_SET_THROTTLE_SRC, bytes([src]), timeout=0.5)
    return resp is not None and resp[0] == GSP_CMD_SET_THROTTLE_SRC


def set_throttle(ser: serial.Serial, thr: int):
    """Set throttle value (0-2000 GSP scale)."""
    thr = max(0, min(2000, thr))
    send_cmd(ser, GSP_CMD_SET_THROTTLE, struct.pack("<H", thr), timeout=0.1)


def parse_snapshot(p: bytes, t0: float, prev_time: float, pole_pairs: int = 1) -> Optional[dict]:
    if len(p) < 68:
        return None

    now = time.monotonic()
    dt_ms = (now - prev_time) * 1000.0 if prev_time > 0 else 0.0

    v = struct.unpack_from("<BBBBHBx", p, 0)
    state, fault, step, direction, throttle, duty = v

    vbus_raw, ibus_raw = struct.unpack_from("<HH", p, 8)
    sys_tick, = struct.unpack_from("<I", p, 60)

    foc_id = foc_iq = foc_theta = foc_omega = foc_vbus = 0.0
    foc_ia = foc_ib = foc_theta_obs = foc_vd = foc_vq = 0.0
    smo_ea = smo_eb = smo_gain = omega_filt = 0.0
    foc_pid_d = foc_pid_q = foc_pid_spd = 0.0
    foc_mod_idx = foc_obs_conf = 0.0
    foc_sub = 0
    foc_off_ia = foc_off_ib = 0
    smo_residual = pll_innovation = pll_omega = omega_ol = 0.0
    handoff_ctr = 0
    smo_observable = 0

    if len(p) >= 170:
        # Full V4 snapshot (170 bytes)
        foc_id, foc_iq, foc_theta, foc_omega, foc_vbus = \
            struct.unpack_from("<fffff", p, 68)
        foc_ia, foc_ib, foc_theta_obs, foc_vd, foc_vq = \
            struct.unpack_from("<fffff", p, 88)
        smo_ea, smo_eb, omega_filt, smo_gain = \
            struct.unpack_from("<ffff", p, 108)
        foc_pid_d, foc_pid_q, foc_pid_spd = \
            struct.unpack_from("<fff", p, 124)
        foc_mod_idx, foc_obs_conf = \
            struct.unpack_from("<ff", p, 136)
        foc_sub, = struct.unpack_from("<B", p, 144)
        foc_off_ia, foc_off_ib = struct.unpack_from("<HH", p, 146)
        smo_residual, pll_innovation, pll_omega, omega_ol = \
            struct.unpack_from("<ffff", p, 150)
        handoff_ctr, smo_observable = struct.unpack_from("<HB", p, 166)
    elif len(p) >= 150:
        # V3 snapshot (150 bytes, no V4 diagnostics)
        foc_id, foc_iq, foc_theta, foc_omega, foc_vbus = \
            struct.unpack_from("<fffff", p, 68)
        foc_ia, foc_ib, foc_theta_obs, foc_vd, foc_vq = \
            struct.unpack_from("<fffff", p, 88)
        smo_ea, smo_eb, omega_filt, smo_gain = \
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

    foc_rpm = abs(foc_omega) * 60.0 / (2.0 * 3.14159265 * pole_pairs)
    foc_power = foc_vq * foc_iq + foc_vd * foc_id

    return {
        "time_s": f"{now - t0:.4f}",
        "dt_ms": f"{dt_ms:.1f}",
        "state": state,
        "state_name": "VF_ASSIST" if foc_sub == 5 else ESC_STATE_NAMES.get(state, f"?{state}"),
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
        "omegaFilt": f"{omega_filt:.2f}",
        "smoGain": f"{smo_gain:.4f}",
        "focPidDInteg": f"{foc_pid_d:.4f}",
        "focPidQInteg": f"{foc_pid_q:.4f}",
        "focPidSpdInteg": f"{foc_pid_spd:.4f}",
        "focModIndex": f"{foc_mod_idx:.4f}",
        "focObsConfidence": f"{foc_obs_conf:.4f}",
        "focOffsetIa": foc_off_ia,
        "focOffsetIb": foc_off_ib,
        "smoResidual": f"{smo_residual:.4f}",
        "pllInnovation": f"{pll_innovation:.4f}",
        "pllOmega": f"{pll_omega:.2f}",
        "omegaOl": f"{omega_ol:.2f}",
        "handoffCtr": handoff_ctr,
        "smoObservable": smo_observable,
        "focRPM": f"{foc_rpm:.1f}",
        "focPower": f"{foc_power:.3f}",
        "transition": "",
        "systemTick": sys_tick,
        "_raw_time": now,
    }


# ── Throttle Profile ────────────────────────────────────────────────────

class ThrottleProfile:
    """Generates a time-based throttle sweep profile."""

    def __init__(self, max_thr: int, ramp_time: float, hold_time: float,
                 arm_thr: int = 200, steps: int = 1):
        self.arm_thr = arm_thr
        self.max_thr = max_thr
        self.ramp_time = ramp_time
        self.hold_time = hold_time
        self.steps = steps

        if steps <= 1:
            # Simple profile: idle → ramp up → hold max → ramp down → idle
            self.segments = self._build_simple()
        else:
            # Staircase: idle → step1 → step2 → ... → max → ... → step1 → idle
            self.segments = self._build_staircase()

        self.total_time = self.segments[-1][0] if self.segments else 0.0

    def _build_simple(self):
        """Build segment list for simple ramp-up/hold/ramp-down."""
        segs = []
        t = 0.0
        segs.append((t, self.arm_thr, self.arm_thr, "idle_start"))
        t += self.hold_time
        segs.append((t, self.arm_thr, self.max_thr, "ramp_up"))
        t += self.ramp_time
        segs.append((t, self.max_thr, self.max_thr, "hold_max"))
        t += self.hold_time
        segs.append((t, self.max_thr, self.arm_thr, "ramp_down"))
        t += self.ramp_time
        segs.append((t, self.arm_thr, self.arm_thr, "idle_end"))
        t += self.hold_time
        segs.append((t, -1, -1, "done"))
        return segs

    def _build_staircase(self):
        """Build staircase: ramp→hold at each step level up, then back down."""
        segs = []
        t = 0.0
        n = self.steps
        thr_range = self.max_thr - self.arm_thr
        levels = [self.arm_thr + int(thr_range * (i + 1) / n) for i in range(n)]

        # Initial idle
        segs.append((t, self.arm_thr, self.arm_thr, "idle_start"))
        t += self.hold_time

        # Ramp up through each step
        prev_thr = self.arm_thr
        for i, level in enumerate(levels):
            label = f"ramp_{int(100*(i+1)/n)}%"
            segs.append((t, prev_thr, level, label))
            t += self.ramp_time
            label = f"hold_{int(100*(i+1)/n)}%"
            segs.append((t, level, level, label))
            t += self.hold_time
            prev_thr = level

        # Ramp down through each step (reverse, skip max since we just held it)
        for i, level in enumerate(reversed(levels[:-1])):
            pct = int(100 * (n - 1 - i) / n)
            label = f"down_{pct}%"
            segs.append((t, prev_thr, level, label))
            t += self.ramp_time
            label = f"dwell_{pct}%"
            segs.append((t, level, level, label))
            t += self.hold_time
            prev_thr = level

        # Final ramp to idle
        segs.append((t, prev_thr, self.arm_thr, "ramp_idle"))
        t += self.ramp_time
        segs.append((t, self.arm_thr, self.arm_thr, "idle_end"))
        t += self.hold_time
        segs.append((t, -1, -1, "done"))
        return segs

    def get(self, t_cl: float) -> tuple:
        """Returns (throttle_value, phase_name) for time since CL entry."""
        if t_cl < 0:
            return self.arm_thr, "startup"

        # Find current segment
        for i in range(len(self.segments) - 1):
            t_start, thr_from, thr_to, label = self.segments[i]
            t_end = self.segments[i + 1][0]
            if t_cl < t_end:
                if thr_from == thr_to:
                    return thr_from, label
                # Interpolate ramp
                frac = (t_cl - t_start) / (t_end - t_start)
                thr = int(thr_from + frac * (thr_to - thr_from))
                return thr, label

        return -1, "done"


# ── Main ────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="SMO Sweep Capture — automated throttle sweep with telemetry")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--max-throttle", type=int, default=1500,
                        help="Peak throttle (0-2000 GSP scale, default 1500 = 75%%)")
    parser.add_argument("--ramp-time", type=float, default=3.0,
                        help="Ramp duration in seconds (default 3)")
    parser.add_argument("--hold-time", type=float, default=2.0,
                        help="Hold duration at each end (default 2)")
    parser.add_argument("--arm-throttle", type=int, default=200,
                        help="Throttle for arming/idle (default 200, GSP 0-2000). "
                             "Must be >147 to stay above firmware deadband (300/4095).")
    parser.add_argument("--steps", type=int, default=1,
                        help="Number of staircase steps (default 1 = simple ramp). "
                             "E.g. --steps 4 does 25%%→50%%→75%%→100%% and back down.")
    parser.add_argument("--manual", action="store_true",
                        help="Manual pot control — capture until Ctrl+C or STOP")
    parser.add_argument("--timeout", type=float, default=60.0,
                        help="Max capture time in seconds (default 60)")
    args = parser.parse_args()

    logs_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "logs")
    os.makedirs(logs_dir, exist_ok=True)
    filename = os.path.join(
        logs_dir, f"smo_sweep_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

    profile = ThrottleProfile(args.max_throttle, args.ramp_time,
                              args.hold_time, args.arm_throttle, args.steps)

    print("SMO Sweep Capture")
    if args.manual:
        print("  Mode:     MANUAL (use pot, Ctrl+C to stop)")
    else:
        if args.steps > 1:
            step_pcts = [f"{int(100*i/args.steps)}%" for i in range(1, args.steps+1)]
            print(f"  Profile:  staircase {' → '.join(step_pcts)} → back down")
        else:
            print(f"  Profile:  idle({args.hold_time}s) → ramp({args.ramp_time}s) → "
                  f"{args.max_throttle}({args.hold_time}s) → ramp({args.ramp_time}s) → idle")
        print(f"  Total:    {profile.total_time:.1f}s sweep + ~2s startup")
    print(f"  Port:     {args.port}")
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
    pole_pairs = 1  # default fallback
    resp = send_cmd(ser, GSP_CMD_GET_INFO)
    if resp and resp[0] == GSP_CMD_GET_INFO and len(resp[1]) >= 14:
        flags = struct.unpack_from("<I", resp[1], 8)[0]
        is_foc = bool(flags & (1 << 23))
        pole_pairs = resp[1][7] if resp[1][7] > 0 else 1
        print(f"  Mode: {'FOC' if is_foc else '6-step'} (flags=0x{flags:08X}, pp={pole_pairs})")

    # Ensure motor is in IDLE before switching throttle source
    resp = send_cmd(ser, GSP_CMD_GET_SNAPSHOT)
    if resp and resp[0] == GSP_CMD_GET_SNAPSHOT:
        init_state = resp[1][0]
        print(f"  State: {ESC_STATE_NAMES.get(init_state, '?')} ({len(resp[1])}B)")
        if init_state == 9:
            print("  Clearing fault...")
            send_cmd(ser, GSP_CMD_CLEAR_FAULT)
            time.sleep(0.5)
        elif init_state != 0:  # Not IDLE — stop motor first
            print("  Motor running — sending STOP...")
            send_cmd(ser, GSP_CMD_STOP_MOTOR)
            time.sleep(1.0)
            # Wait for IDLE
            for _ in range(20):
                resp2 = send_cmd(ser, GSP_CMD_GET_SNAPSHOT, timeout=0.3)
                if resp2 and resp2[0] == GSP_CMD_GET_SNAPSHOT and resp2[1][0] == 0:
                    break
                time.sleep(0.1)
            print(f"  Motor stopped.")

    # Open CSV
    csvfile = open(filename, "w", newline="")
    writer = csv.DictWriter(csvfile, fieldnames=CSV_COLUMNS, extrasaction='ignore')
    writer.writeheader()
    print(f"  CSV: {filename}")
    print()

    # State tracking
    transitions = []
    prev_state = -1
    cl_enter_time = None
    sample_count = 0
    stop = False
    current_cmd_thr = args.arm_throttle
    current_phase = "startup"

    def signal_handler(sig, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, signal_handler)

    # Switch throttle source to GSP (must be done in IDLE, before START)
    if not args.manual:
        if set_throttle_src(ser, THROTTLE_SRC_GSP):
            print("  Throttle source: GSP")
        else:
            print("  WARNING: Failed to set throttle source to GSP")
            print("           Speed control will use physical pot instead")

    # Set throttle to 0 for arming (firmware requires throttle <= deadband to arm)
    if not args.manual:
        set_throttle(ser, 0)
        print(f"  Throttle set to 0 for arming (idle will use {args.arm_throttle})")

    print(f"  Sending START command...")
    resp = send_cmd(ser, GSP_CMD_START_MOTOR)
    if resp is None:
        print("  WARNING: START got no response")

    t0 = time.monotonic()
    prev_time = 0.0
    last_print = 0.0
    last_thr_update = 0.0

    print(f"\n  {'Time':>7s}  {'Phase':>10s}  {'Thr':>5s}  {'State':>8s}  "
          f"{'Iq':>7s}  {'Id':>7s}  {'RPM':>7s}  "
          f"{'Resid':>6s}  {'PLL_w':>7s}  {'w_OL':>7s}  {'HO':>4s}  {'Obs':>3s}")
    print(f"  {'─'*7}  {'─'*10}  {'─'*5}  {'─'*8}  "
          f"{'─'*7}  {'─'*7}  {'─'*7}  "
          f"{'─'*6}  {'─'*7}  {'─'*7}  {'─'*4}  {'─'*3}")

    while not stop:
        elapsed = time.monotonic() - t0
        if elapsed >= args.timeout:
            print(f"\n\n  TIMEOUT ({args.timeout}s)")
            break

        # Update throttle from profile (every ~50ms to avoid flooding)
        now = time.monotonic()
        if not args.manual and now - last_thr_update >= 0.050:
            last_thr_update = now
            if cl_enter_time is not None:
                t_cl = elapsed - cl_enter_time
                thr, phase = profile.get(t_cl)
                if phase == "done":
                    print(f"\n\n  Sweep complete!")
                    break
                if thr != current_cmd_thr:
                    set_throttle(ser, thr)
                    current_cmd_thr = thr
                current_phase = phase

        # Poll snapshot
        resp = send_cmd(ser, GSP_CMD_GET_SNAPSHOT, timeout=0.15)
        if resp is None or resp[0] != GSP_CMD_GET_SNAPSHOT:
            continue

        row = parse_snapshot(resp[1], t0, prev_time, pole_pairs)
        if row is None:
            continue

        prev_time = row.pop("_raw_time", prev_time)

        # Detect transitions
        cur_state = row["state"]
        transition_label = ""

        if cur_state != prev_state:
            cur_name = ESC_STATE_NAMES.get(cur_state, f"?{cur_state}")
            prev_name = ESC_STATE_NAMES.get(prev_state, f"?{prev_state}")
            transition_label = f"{prev_name}->{cur_name}"
            transitions.append((elapsed, transition_label))
            print(f"\n  *** {transition_label} at t={elapsed:.3f}s ***")
            prev_state = cur_state

        row["transition"] = transition_label
        row["phase"] = current_phase
        row["cmd_throttle"] = current_cmd_thr

        writer.writerow(row)
        sample_count += 1

        # Track CL entry
        if cur_state == 6:
            if cl_enter_time is None:
                cl_enter_time = elapsed
        else:
            cl_enter_time = None

        # Check for fault
        if cur_state == 9:
            fault_name = FAULT_NAMES.get(row["faultCode"], "?")
            print(f"\n\n  FAULT: {fault_name} (code={row['faultCode']})")
            for _ in range(5):
                resp = send_cmd(ser, GSP_CMD_GET_SNAPSHOT, timeout=0.2)
                if resp and resp[0] == GSP_CMD_GET_SNAPSHOT:
                    r = parse_snapshot(resp[1], t0, prev_time, pole_pairs)
                    if r:
                        prev_time = r.pop("_raw_time", prev_time)
                        r["transition"] = "POST_FAULT"
                        r["phase"] = "fault"
                        r["cmd_throttle"] = current_cmd_thr
                        writer.writerow(r)
                        sample_count += 1
            break

        # Live display at ~10 Hz
        if now - last_print >= 0.100:
            line = (f"\r  {row['time_s']:>7s}  {current_phase:>10s}  "
                    f"{current_cmd_thr:>5d}  {row['state_name']:>8s}  "
                    f"Iq={row['focIqMeas']:>6s}  Id={row['focIdMeas']:>6s}  "
                    f"{row['focRPM']:>6s}rpm  "
                    f"res={row['smoResidual']:>6s}  "
                    f"inn={row['pllInnovation']:>5s}  "
                    f"pllW={row['pllOmega']:>7s}  "
                    f"wOl={row['omegaOl']:>7s}  "
                    f"ho={row['handoffCtr']:>4d}  "
                    f"obs={row['smoObservable']}")
            print(line, end="", flush=True)
            last_print = now

    # Stop motor
    print(f"\n  Sending STOP command...")
    send_cmd(ser, GSP_CMD_STOP_MOTOR)
    set_throttle(ser, 0)
    # Restore throttle source to ADC (needs IDLE — wait briefly for stop)
    time.sleep(0.3)
    set_throttle_src(ser, THROTTLE_SRC_ADC)

    csvfile.close()
    ser.close()

    # ── Summary ──
    duration = time.monotonic() - t0
    print(f"\n  ═══ Sweep Capture Summary ═══")
    print(f"  Duration:   {duration:.2f}s")
    print(f"  Samples:    {sample_count}")
    if duration > 0:
        print(f"  Rate:       {sample_count / duration:.1f} Hz")
    print(f"  CSV file:   {filename}")

    if transitions:
        print(f"\n  ── Transition Timeline ──")
        for t, label in transitions:
            print(f"    t={t:7.3f}s  {label}")

        t_cl = next((t for t, l in transitions if "->CL" in l), None)
        t_fault = next((t for t, l in transitions if "FAULT" in l), None)

        print(f"\n  ── Key Timings ──")
        if t_cl is not None:
            print(f"    First CL at:    {t_cl:.3f}s")
        if t_fault is not None:
            print(f"    FAULT at:       {t_fault:.3f}s")
        if t_cl and not t_fault:
            print(f"    CL duration:    {duration - t_cl:.1f}s (no fault)")

    # ── Per-phase statistics (only for hold phases in CL) ──
    if not args.manual:
        try:
            with open(filename, "r") as rf:
                rows = list(csv.DictReader(rf))
            # Group by phase, only CL hold phases
            from collections import defaultdict
            phase_data = defaultdict(lambda: {"iq": [], "id": [], "rpm": [], "power": [], "inn": []})
            for row in rows:
                phase = row.get("phase", "")
                if row.get("state_name") != "CL" or "hold" not in phase and "dwell" not in phase:
                    continue
                try:
                    phase_data[phase]["iq"].append(float(row["focIqMeas"]))
                    phase_data[phase]["id"].append(float(row["focIdMeas"]))
                    phase_data[phase]["rpm"].append(float(row["focRPM"]))
                    phase_data[phase]["power"].append(float(row["focPower"]))
                    phase_data[phase]["inn"].append(float(row["pllInnovation"]))
                except (ValueError, KeyError):
                    pass

            if phase_data:
                print(f"\n  ── Step Statistics (CL hold phases) ──")
                print(f"  {'Phase':<14s}  {'Samples':>7s}  {'Iq(A)':>8s}  {'Id(A)':>8s}  "
                      f"{'RPM':>8s}  {'Power(W)':>9s}  {'Inn(rad)':>9s}")
                print(f"  {'─'*14}  {'─'*7}  {'─'*8}  {'─'*8}  {'─'*8}  {'─'*9}  {'─'*9}")
                for phase in sorted(phase_data.keys()):
                    d = phase_data[phase]
                    n = len(d["iq"])
                    if n == 0:
                        continue
                    avg = lambda lst: sum(lst) / len(lst)
                    print(f"  {phase:<14s}  {n:>7d}  {avg(d['iq']):>8.2f}  {avg(d['id']):>8.2f}  "
                          f"{avg(d['rpm']):>8.0f}  {avg(d['power']):>9.2f}  {avg(d['inn']):>9.3f}")
        except Exception:
            pass

    print()


if __name__ == "__main__":
    main()
