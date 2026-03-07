#!/usr/bin/env python3
"""
GSP RX Input Simulation Test — PWM & DShot state machine exercise.

Injects frames into the RX seqlock mailbox via GSP_CMD_RX_INJECT (0x27),
bypassing SCCP4 hardware. Tests the full downstream path:

  GSP inject → rxMailbox → RX_Service() lock FSM → rxCachedThrottleAdc
  → ADC ISR throttle mux → garudaData.throttle

Verifies:
  1. PWM pulse-width → throttle ADC conversion
  2. DShot value → throttle ADC conversion
  3. Lock FSM: DETECTING → LOCKING → LOCKED (N valid frames)
  4. Lock FSM: invalid frames reset lock count
  5. Timeout: no frames → LOST
  6. Auto-arm: LOCKED + zero throttle → ESC_ARMED
  7. Fault on loss: motor running + timeout → FAULT_RX_LOSS

Usage:
    python3 gsp_rx_test.py [--port /dev/ttyACM0] [--baud 115200]

Requirements:
    pip install pyserial
"""

import argparse
import struct
import sys
import time
from typing import Optional

import serial


# ── GSP Protocol Constants ──────────────────────────────────────────────

GSP_START_BYTE = 0x02

GSP_CMD_PING            = 0x00
GSP_CMD_GET_INFO        = 0x01
GSP_CMD_GET_SNAPSHOT    = 0x02
GSP_CMD_START_MOTOR     = 0x03
GSP_CMD_STOP_MOTOR      = 0x04
GSP_CMD_CLEAR_FAULT     = 0x05
GSP_CMD_SET_THROTTLE_SRC= 0x07
GSP_CMD_GET_RX_STATUS   = 0x26
GSP_CMD_RX_INJECT       = 0x27
GSP_CMD_TELEM_FRAME     = 0x80
GSP_CMD_ERROR           = 0xFF

GSP_ERR_WRONG_STATE     = 0x04

# RX enums (match garuda_types.h)
RX_LINK_UNLOCKED  = 0
RX_LINK_DETECTING = 1
RX_LINK_LOCKING   = 2
RX_LINK_LOCKED    = 3
RX_LINK_LOST      = 4

RX_PROTO_NONE     = 0
RX_PROTO_PWM      = 1
RX_PROTO_DSHOT150 = 2
RX_PROTO_DSHOT300 = 3
RX_PROTO_DSHOT600 = 4

LINK_STATE_NAMES = {0: "UNLOCKED", 1: "DETECTING", 2: "LOCKING", 3: "LOCKED", 4: "LOST"}
PROTO_NAMES = {0: "NONE", 1: "PWM", 2: "DSHOT150", 3: "DSHOT300", 4: "DSHOT600"}

# ESC states (match garuda_types.h ESC_STATE_T enum)
ESC_IDLE        = 0
ESC_ARMED       = 1
ESC_DETECT      = 2
ESC_ALIGN       = 3
ESC_OL_RAMP     = 4
ESC_MORPH       = 5
ESC_CLOSED_LOOP = 6
ESC_BRAKING     = 7
ESC_RECOVERY    = 8
ESC_FAULT       = 9

ESC_STATE_NAMES = {
    0: "IDLE", 1: "ARMED", 2: "DETECT", 3: "ALIGN", 4: "OL_RAMP",
    5: "MORPH", 6: "CL", 7: "BRAKING", 8: "RECOVERY", 9: "FAULT"
}

# Fault codes
FAULT_NONE    = 0
FAULT_RX_LOSS = 9

# Throttle sources
THROTTLE_SRC_PWM   = 2
THROTTLE_SRC_DSHOT = 3
THROTTLE_SRC_AUTO  = 4

# Firmware constants (from garuda_config.h)
RX_LOCK_COUNT      = 10
RX_TIMEOUT_MS      = 200
RX_PWM_DEADBAND_US = 25
RX_DSHOT_CMD_MAX   = 47

GSP_SNAPSHOT_SIZE = 106  # FOC-aware snapshot


# ── CRC-16-CCITT ────────────────────────────────────────────────────────

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


# ── Packet Builder / Parser ────────────────────────────────────────────

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


def drain_unsolicited(ser: serial.Serial):
    """Drain any pending telemetry frames."""
    ser.timeout = 0.05
    while True:
        data = ser.read(256)
        if not data:
            break


# ── Command helpers ─────────────────────────────────────────────────────

def send_cmd(ser: serial.Serial, cmd_id: int, payload: bytes = b"") -> Optional[tuple]:
    ser.write(build_packet(cmd_id, payload))
    return read_response(ser)


def rx_inject(ser: serial.Serial, protocol: int, value: int, valid: int,
              verbose: bool = False) -> bool:
    """Inject a frame into the RX mailbox. Returns True on ACK."""
    payload = struct.pack("<BHB", protocol, value, valid)
    resp = send_cmd(ser, GSP_CMD_RX_INJECT, payload)
    if resp is None:
        if verbose:
            print(f"    DBG: rx_inject → no response (timeout)")
        return False
    if resp[0] != GSP_CMD_RX_INJECT:
        if verbose:
            err_hex = resp[1].hex() if resp[1] else "empty"
            print(f"    DBG: rx_inject → cmd=0x{resp[0]:02X} payload={err_hex}")
    return resp[0] == GSP_CMD_RX_INJECT


def get_rx_status(ser: serial.Serial) -> Optional[dict]:
    """Read RX status (12 bytes)."""
    resp = send_cmd(ser, GSP_CMD_GET_RX_STATUS)
    if resp is None or resp[0] != GSP_CMD_GET_RX_STATUS:
        return None
    p = resp[1]
    if len(p) < 12:
        return None
    link, proto, drate, throttle, pulse_us, crc_err, dropped = \
        struct.unpack_from("<BBBxHHHH", p)
    return {
        "linkState": link,
        "protocol": proto,
        "dshotRate": drate,
        "throttle": throttle,
        "pulseUs": pulse_us,
        "crcErrors": crc_err,
        "droppedFrames": dropped,
    }


def get_snapshot(ser: serial.Serial) -> Optional[dict]:
    """Read snapshot, extract state/fault/throttle."""
    resp = send_cmd(ser, GSP_CMD_GET_SNAPSHOT)
    if resp is None or resp[0] != GSP_CMD_GET_SNAPSHOT:
        return None
    p = resp[1]
    if len(p) < 8:
        return None
    state, fault, step, direction, throttle, duty_pct = \
        struct.unpack_from("<BBBBHBx", p)
    return {
        "state": state,
        "faultCode": fault,
        "throttle": throttle,
        "dutyPct": duty_pct,
    }


def ensure_idle(ser: serial.Serial) -> bool:
    """Stop motor and clear faults to get to ESC_IDLE."""
    snap = get_snapshot(ser)
    if snap is None:
        return False
    if snap["state"] == ESC_FAULT:
        send_cmd(ser, GSP_CMD_CLEAR_FAULT)
        time.sleep(0.05)
    if snap["state"] != ESC_IDLE:
        send_cmd(ser, GSP_CMD_STOP_MOTOR)
        time.sleep(0.05)
    snap = get_snapshot(ser)
    return snap is not None and snap["state"] == ESC_IDLE


def set_throttle_source(ser: serial.Serial, src: int) -> bool:
    """Set throttle source (must be IDLE)."""
    resp = send_cmd(ser, GSP_CMD_SET_THROTTLE_SRC, bytes([src]))
    if resp is None:
        return False
    return resp[0] == GSP_CMD_SET_THROTTLE_SRC


# ── PWM throttle model (mirrors PwmUsToThrottleAdc in rx_decode.c) ─────

def pwm_us_to_throttle_adc(pulse_us: int) -> int:
    if pulse_us <= (1000 + RX_PWM_DEADBAND_US):
        return 0
    # Mirrors C: if (pulseUs >= 2000) throttle2000 = 2000; else throttle2000 = pulseUs - 1000;
    if pulse_us >= 2000:
        throttle2000 = 2000
    else:
        throttle2000 = pulse_us - 1000
    return (throttle2000 * 4095) // 2000


# ── DShot throttle model (mirrors DshotToThrottleAdc) ──────────────────

def dshot_to_throttle_adc(dshot_val: int) -> int:
    if dshot_val <= RX_DSHOT_CMD_MAX:
        return 0
    throttle1999 = min(dshot_val - 48, 1999)
    return (throttle1999 * 4095) // 1999


# ── Test Infrastructure ────────────────────────────────────────────────

pass_count = 0
fail_count = 0


def check(condition: bool, label: str, detail: str = ""):
    global pass_count, fail_count
    if condition:
        pass_count += 1
        print(f"  PASS  {label}")
    else:
        fail_count += 1
        msg = f"  FAIL  {label}"
        if detail:
            msg += f" — {detail}"
        print(msg)
    return condition


def wait_for_rx_service(ser: serial.Serial, cycles: int = 5, delay: float = 0.01):
    """Wait for main loop RX_Service() to process the mailbox.
    Also ping to keep GSP alive and consume any output."""
    for _ in range(cycles):
        time.sleep(delay)
        send_cmd(ser, GSP_CMD_PING)


# ── Test Cases ──────────────────────────────────────────────────────────

def test_ping(ser: serial.Serial):
    """T0: Verify basic GSP connectivity."""
    print("\n── T0: PING ──")
    resp = send_cmd(ser, GSP_CMD_PING)
    check(resp is not None and resp[0] == GSP_CMD_PING, "PING ACK")


def test_rx_inject_ack(ser: serial.Serial, rx_src: int):
    """T1: RX_INJECT command accepted."""
    print("\n── T1: RX_INJECT ACK ──")
    ensure_idle(ser)

    # Debug: check SET_THROTTLE_SRC response
    resp = send_cmd(ser, GSP_CMD_SET_THROTTLE_SRC, bytes([rx_src]))
    if resp is None:
        print(f"    DBG: SET_THROTTLE_SRC → timeout")
    else:
        print(f"    DBG: SET_THROTTLE_SRC → cmd=0x{resp[0]:02X} payload={resp[1].hex() if resp[1] else 'empty'}")

    ok = rx_inject(ser, RX_PROTO_PWM, 1500, 1, verbose=True)
    check(ok, "RX_INJECT returns ACK")


def test_pwm_lock_fsm(ser: serial.Serial, rx_src: int):
    """T2: PWM lock FSM: DETECTING → LOCKING → LOCKED after N valid frames."""
    print("\n── T2: PWM Lock FSM ──")
    ensure_idle(ser)
    set_throttle_source(ser, rx_src)

    # First injection bootstraps to LOCKING
    rx_inject(ser, RX_PROTO_PWM, 1500, 1)
    wait_for_rx_service(ser)
    st = get_rx_status(ser)
    check(st is not None and st["linkState"] == RX_LINK_LOCKING,
          "First inject → LOCKING",
          f"got {LINK_STATE_NAMES.get(st['linkState'], '?') if st else 'None'}")
    check(st is not None and st["protocol"] == RX_PROTO_PWM,
          "Protocol = PWM",
          f"got {PROTO_NAMES.get(st['protocol'], '?') if st else 'None'}")

    # Inject RX_LOCK_COUNT-1 more valid frames to reach LOCKED
    for i in range(RX_LOCK_COUNT - 1):
        rx_inject(ser, RX_PROTO_PWM, 1500, 1)
        wait_for_rx_service(ser, cycles=2)

    st = get_rx_status(ser)
    check(st is not None and st["linkState"] == RX_LINK_LOCKED,
          f"After {RX_LOCK_COUNT} valid frames → LOCKED",
          f"got {LINK_STATE_NAMES.get(st['linkState'], '?') if st else 'None'}")


def test_pwm_throttle_conversion(ser: serial.Serial, rx_src: int):
    """T3: PWM pulse-width → throttle ADC conversion accuracy."""
    print("\n── T3: PWM Throttle Conversion ──")
    ensure_idle(ser)
    set_throttle_source(ser, rx_src)

    # Lock first
    for _ in range(RX_LOCK_COUNT + 2):
        rx_inject(ser, RX_PROTO_PWM, 1500, 1)
        wait_for_rx_service(ser, cycles=2)

    test_values = [
        (1000, "1000us (below deadband → 0)"),
        (1025, "1025us (at deadband → 0)"),
        (1100, "1100us (low throttle)"),
        (1500, "1500us (mid throttle)"),
        (2000, "2000us (full throttle)"),
    ]

    for pulse_us, label in test_values:
        rx_inject(ser, RX_PROTO_PWM, pulse_us, 1)
        wait_for_rx_service(ser, cycles=3)

        st = get_rx_status(ser)
        snap = get_snapshot(ser)

        expected = pwm_us_to_throttle_adc(pulse_us)
        actual = snap["throttle"] if snap else -1

        # Allow ±1 for integer rounding
        ok = abs(actual - expected) <= 1
        check(ok, f"PWM {label}: expected={expected}, got={actual}")


def test_dshot_lock_and_conversion(ser: serial.Serial, rx_src: int):
    """T4: DShot lock FSM + throttle conversion."""
    print("\n── T4: DShot Lock + Throttle Conversion ──")
    ensure_idle(ser)
    set_throttle_source(ser, rx_src)

    # Lock with DShot600 frames
    for _ in range(RX_LOCK_COUNT + 2):
        rx_inject(ser, RX_PROTO_DSHOT600, 1000, 1)
        wait_for_rx_service(ser, cycles=2)

    st = get_rx_status(ser)
    check(st is not None and st["linkState"] == RX_LINK_LOCKED,
          "DShot LOCKED")
    check(st is not None and st["protocol"] == RX_PROTO_DSHOT600,
          "Protocol = DSHOT600")

    test_values = [
        (0,    "DShot 0 (disarm → 0)"),
        (47,   "DShot 47 (last cmd → 0)"),
        (48,   "DShot 48 (min throttle)"),
        (1000, "DShot 1000 (mid throttle)"),
        (2047, "DShot 2047 (max throttle)"),
    ]

    for dshot_val, label in test_values:
        rx_inject(ser, RX_PROTO_DSHOT600, dshot_val, 1)
        wait_for_rx_service(ser, cycles=3)

        snap = get_snapshot(ser)
        expected = dshot_to_throttle_adc(dshot_val)
        actual = snap["throttle"] if snap else -1

        ok = abs(actual - expected) <= 1
        check(ok, f"{label}: expected={expected}, got={actual}")


def test_invalid_frames_reset_lock(ser: serial.Serial, rx_src: int):
    """T5: Invalid frame during LOCKING resets lock count."""
    print("\n── T5: Invalid Frames Reset Lock Count ──")
    ensure_idle(ser)
    set_throttle_source(ser, rx_src)

    # Send 5 valid frames (less than RX_LOCK_COUNT=10)
    for _ in range(5):
        rx_inject(ser, RX_PROTO_PWM, 1500, 1)
        wait_for_rx_service(ser, cycles=2)

    st = get_rx_status(ser)
    check(st is not None and st["linkState"] == RX_LINK_LOCKING,
          "5 valid frames → still LOCKING")

    # Send 1 invalid frame — should reset lockCount to 0
    rx_inject(ser, RX_PROTO_PWM, 1500, 0)
    wait_for_rx_service(ser, cycles=2)

    # Now send RX_LOCK_COUNT-1 valid frames — should NOT be enough
    for _ in range(RX_LOCK_COUNT - 1):
        rx_inject(ser, RX_PROTO_PWM, 1500, 1)
        wait_for_rx_service(ser, cycles=2)

    st = get_rx_status(ser)
    check(st is not None and st["linkState"] == RX_LINK_LOCKING,
          "After invalid + 9 valid → still LOCKING (reset worked)")

    # One more valid → now should lock (total 10 consecutive valid)
    rx_inject(ser, RX_PROTO_PWM, 1500, 1)
    wait_for_rx_service(ser, cycles=2)

    st = get_rx_status(ser)
    check(st is not None and st["linkState"] == RX_LINK_LOCKED,
          "After 10 consecutive valid → LOCKED")


def test_timeout_to_lost(ser: serial.Serial, rx_src: int):
    """T6: No frames for RX_TIMEOUT_MS → LOST state."""
    print("\n── T6: Timeout → LOST ──")
    ensure_idle(ser)
    set_throttle_source(ser, rx_src)

    # Lock first
    for _ in range(RX_LOCK_COUNT + 2):
        rx_inject(ser, RX_PROTO_PWM, 1500, 1)
        wait_for_rx_service(ser, cycles=2)

    st = get_rx_status(ser)
    check(st is not None and st["linkState"] == RX_LINK_LOCKED,
          "Pre-condition: LOCKED")

    # Wait for timeout (RX_TIMEOUT_MS=200ms + margin)
    print(f"  Waiting {RX_TIMEOUT_MS + 100}ms for timeout...")
    time.sleep((RX_TIMEOUT_MS + 100) / 1000.0)

    # Ping to keep GSP alive, let RX_Service() run
    wait_for_rx_service(ser, cycles=5)

    st = get_rx_status(ser)
    check(st is not None and st["linkState"] == RX_LINK_LOST,
          "After timeout → LOST",
          f"got {LINK_STATE_NAMES.get(st['linkState'], '?') if st else 'None'}")

    snap = get_snapshot(ser)
    check(snap is not None and snap["throttle"] == 0,
          "Throttle zeroed on LOST")


def test_auto_arm(ser: serial.Serial, rx_src: int):
    """T7: Auto-arm: LOCKED + zero throttle → ESC_ARMED."""
    print("\n── T7: Auto-Arm from RX ──")
    ensure_idle(ser)
    set_throttle_source(ser, rx_src)

    # Lock with zero throttle (1000µs = zero after deadband)
    for _ in range(RX_LOCK_COUNT + 2):
        rx_inject(ser, RX_PROTO_PWM, 1000, 1)
        wait_for_rx_service(ser, cycles=2)

    st = get_rx_status(ser)
    check(st is not None and st["linkState"] == RX_LINK_LOCKED,
          "Pre-condition: LOCKED with zero throttle")

    # Give main loop time to process auto-arm logic
    # Keep injecting zero throttle to avoid timeout
    for _ in range(5):
        rx_inject(ser, RX_PROTO_PWM, 1000, 1)
        wait_for_rx_service(ser, cycles=3)

    snap = get_snapshot(ser)
    check(snap is not None and snap["state"] == ESC_ARMED,
          "ESC state → ARMED",
          f"got state={snap['state'] if snap else 'None'}")

    # Clean up: stop motor
    send_cmd(ser, GSP_CMD_STOP_MOTOR)
    time.sleep(0.05)
    ensure_idle(ser)


def test_crc_error_counter(ser: serial.Serial, rx_src: int):
    """T8: CRC error counter increments on invalid DShot frames."""
    print("\n── T8: CRC Error Counter ──")
    ensure_idle(ser)
    set_throttle_source(ser, rx_src)

    # Lock first
    for _ in range(RX_LOCK_COUNT + 2):
        rx_inject(ser, RX_PROTO_DSHOT600, 1000, 1)
        wait_for_rx_service(ser, cycles=2)

    st = get_rx_status(ser)
    crc_before = st["crcErrors"] if st else 0

    # Send 5 invalid frames
    for _ in range(5):
        rx_inject(ser, RX_PROTO_DSHOT600, 500, 0)
        wait_for_rx_service(ser, cycles=2)

    st = get_rx_status(ser)
    crc_after = st["crcErrors"] if st else 0
    check(crc_after >= crc_before + 5,
          f"CRC error count increased by ≥5 (before={crc_before}, after={crc_after})")


def test_pwm_full_range_sweep(ser: serial.Serial, rx_src: int):
    """T9: Sweep PWM 1000-2000µs in steps, verify monotonic throttle."""
    print("\n── T9: PWM Full Range Sweep ──")
    ensure_idle(ser)
    set_throttle_source(ser, rx_src)

    # Lock
    for _ in range(RX_LOCK_COUNT + 2):
        rx_inject(ser, RX_PROTO_PWM, 1500, 1)
        wait_for_rx_service(ser, cycles=2)

    last_throttle = -1
    monotonic = True
    values_checked = 0

    for pulse_us in range(1000, 2001, 50):
        rx_inject(ser, RX_PROTO_PWM, pulse_us, 1)
        wait_for_rx_service(ser, cycles=3)
        snap = get_snapshot(ser)
        if snap is None:
            continue
        t = snap["throttle"]
        expected = pwm_us_to_throttle_adc(pulse_us)
        if abs(t - expected) > 1:
            check(False, f"PWM {pulse_us}µs: expected={expected}, got={t}")
            monotonic = False
        if t < last_throttle:
            monotonic = False
        last_throttle = t
        values_checked += 1

    check(monotonic and values_checked > 15,
          f"PWM sweep monotonic ({values_checked} points)")


def test_dshot_full_range_sweep(ser: serial.Serial, rx_src: int):
    """T10: Sweep DShot 0-2047 in steps, verify monotonic throttle."""
    print("\n── T10: DShot Full Range Sweep ──")
    ensure_idle(ser)
    set_throttle_source(ser, rx_src)

    # Lock
    for _ in range(RX_LOCK_COUNT + 2):
        rx_inject(ser, RX_PROTO_DSHOT600, 500, 1)
        wait_for_rx_service(ser, cycles=2)

    last_throttle = -1
    monotonic = True
    values_checked = 0

    for dval in range(0, 2048, 100):
        rx_inject(ser, RX_PROTO_DSHOT600, dval, 1)
        wait_for_rx_service(ser, cycles=3)
        snap = get_snapshot(ser)
        if snap is None:
            continue
        t = snap["throttle"]
        expected = dshot_to_throttle_adc(dval)
        if abs(t - expected) > 1:
            check(False, f"DShot {dval}: expected={expected}, got={t}")
            monotonic = False
        if t < last_throttle:
            monotonic = False
        last_throttle = t
        values_checked += 1

    check(monotonic and values_checked > 15,
          f"DShot sweep monotonic ({values_checked} points)")


def test_redetect_after_lost(ser: serial.Serial, rx_src: int):
    """T11: After LOST + fault clear, re-enters DETECTING."""
    print("\n── T11: Re-detect After LOST ──")
    ensure_idle(ser)
    set_throttle_source(ser, rx_src)

    # Lock
    for _ in range(RX_LOCK_COUNT + 2):
        rx_inject(ser, RX_PROTO_PWM, 1500, 1)
        wait_for_rx_service(ser, cycles=2)

    # Let it time out
    time.sleep((RX_TIMEOUT_MS + 100) / 1000.0)
    wait_for_rx_service(ser, cycles=5)

    st = get_rx_status(ser)
    check(st is not None and st["linkState"] in (RX_LINK_LOST, RX_LINK_DETECTING),
          "Pre-condition: LOST or already re-DETECTING")

    # If motor wasn't running, no fault — goes straight to re-detect on IDLE
    snap = get_snapshot(ser)
    if snap:
        print(f"    DBG: state={snap['state']} faultCode={snap['faultCode']} throttle={snap['throttle']}")
    if snap and snap["faultCode"] != FAULT_NONE:
        send_cmd(ser, GSP_CMD_CLEAR_FAULT)
        time.sleep(0.05)

    # Wait for main loop to re-enter DETECTING
    wait_for_rx_service(ser, cycles=10, delay=0.02)

    st = get_rx_status(ser)
    if st:
        print(f"    DBG: linkState={LINK_STATE_NAMES.get(st['linkState'],'?')} proto={PROTO_NAMES.get(st['protocol'],'?')} throttle={st['throttle']}")
    check(st is not None and st["linkState"] in (RX_LINK_DETECTING, RX_LINK_UNLOCKED),
          "After IDLE + no fault → re-DETECTING",
          f"got {LINK_STATE_NAMES.get(st['linkState'], '?') if st else 'None'}")


# ── Motor Test Mode ─────────────────────────────────────────────────────

def get_full_snapshot(ser: serial.Serial) -> Optional[dict]:
    """Parse extended snapshot with FOC fields."""
    resp = send_cmd(ser, GSP_CMD_GET_SNAPSHOT)
    if resp is None or resp[0] != GSP_CMD_GET_SNAPSHOT:
        return None
    p = resp[1]
    if len(p) < 8:
        return None
    state, fault, step, direction, throttle, duty_pct = \
        struct.unpack_from("<BBBBHBx", p)
    result = {
        "state": state,
        "faultCode": fault,
        "throttle": throttle,
        "dutyPct": duty_pct,
    }
    # Bus voltage (offset 8)
    if len(p) >= 14:
        vbus_raw, ibus_raw = struct.unpack_from("<HH", p, 8)
        result["vbusRaw"] = vbus_raw
        result["ibusRaw"] = ibus_raw
    # Step period for eRPM calc (offset 20)
    if len(p) >= 22:
        step_period, = struct.unpack_from("<H", p, 20)
        result["stepPeriod"] = step_period
    # FOC fields (offset 68)
    if len(p) >= 106:
        foc_id, foc_iq, foc_theta, foc_omega, foc_vbus = \
            struct.unpack_from("<fffff", p, 68)
        result["focId"] = foc_id
        result["focIq"] = foc_iq
        result["focTheta"] = foc_theta
        result["focOmega"] = foc_omega
        result["focVbus"] = foc_vbus
        foc_sub, = struct.unpack_from("<B", p, 100)
        result["focSubState"] = foc_sub
    return result


def motor_test(ser: serial.Serial, rx_src: int, is_foc: bool,
               use_dshot: bool = False):
    """Interactive motor test via RX inject — spins the motor."""
    if use_dshot:
        proto = RX_PROTO_DSHOT600
        proto_name = "DShot600"
        # DShot: 0=disarm, 48-2047=throttle
        val_zero = 0
        val_min = 48
        val_max = 2047
        val_step = 100
    else:
        proto = RX_PROTO_PWM
        proto_name = "PWM"
        val_zero = 1000
        val_min = 1000
        val_max = 2000
        val_step = 50

    print("\n" + "=" * 60)
    print(f"  MOTOR TEST MODE — {proto_name} RX Inject Throttle Control")
    print("=" * 60)
    print()
    print("  Controls:")
    if use_dshot:
        print(f"    UP/+/]    Increase throttle by {val_step} (more speed)")
        print(f"    DOWN/-/[  Decrease throttle by {val_step} (less speed)")
        print(f"    0         Zero throttle (DShot 0 = disarm)")
        print(f"    1-9       Set throttle preset (270-1847)")
    else:
        print(f"    UP/+/]    Increase throttle by {val_step}us (more speed)")
        print(f"    DOWN/-/[  Decrease throttle by {val_step}us (less speed)")
        print(f"    0         Zero throttle (1000us)")
        print(f"    1-9       Set throttle preset (1100-1900us)")
    print("    q/Ctrl+C  Stop motor and exit")
    print()
    if use_dshot:
        print(f"  Throttle range: {val_min} (min) to {val_max} (full)")
    else:
        print(f"  Throttle range: {val_min}us (zero) to {val_max}us (full)")
    print("  Injection rate: ~50 Hz (20ms)")
    print()

    # Phase 1: ensure IDLE, set throttle source
    if not ensure_idle(ser):
        print("ERROR: Cannot reach ESC_IDLE state")
        return
    if not set_throttle_source(ser, rx_src):
        print("ERROR: Cannot set throttle source")
        return

    # Phase 2: Lock the RX link with zero throttle
    print("  Locking RX link (zero throttle)...", end="", flush=True)
    for i in range(RX_LOCK_COUNT + 5):
        rx_inject(ser, proto, val_zero, 1)
        wait_for_rx_service(ser, cycles=2)

    st = get_rx_status(ser)
    if st is None or st["linkState"] != RX_LINK_LOCKED:
        print(f" FAILED (link={LINK_STATE_NAMES.get(st['linkState'] if st else -1, '?')})")
        return
    print(" LOCKED")

    # Phase 3: Wait for auto-arm
    print("  Waiting for auto-arm...", end="", flush=True)
    for _ in range(30):
        rx_inject(ser, proto, val_zero, 1)
        time.sleep(0.02)
        snap = get_full_snapshot(ser)
        if snap and snap["state"] >= ESC_ARMED:
            break
    else:
        print(" TIMEOUT — ESC did not arm")
        print("  (Is SW1 pressed? Motor may need START_MOTOR or auto-arm via zero throttle)")
        return

    if snap["state"] == ESC_ARMED:
        print(" ARMED")
    else:
        print(f" state={ESC_STATE_NAMES.get(snap['state'], '?')}")

    throttle_val = val_zero  # start at zero throttle

    # Set up terminal for single-keypress input
    import select
    import termios
    import tty
    old_settings = termios.tcgetattr(sys.stdin)

    def cleanup():
        """Stop motor and restore terminal."""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\n  Stopping motor...", end="", flush=True)
        for _ in range(10):
            rx_inject(ser, proto, val_zero, 1)
            time.sleep(0.02)
        send_cmd(ser, GSP_CMD_STOP_MOTOR)
        time.sleep(0.1)
        snap = get_full_snapshot(ser)
        state_name = ESC_STATE_NAMES.get(snap["state"], "?") if snap else "?"
        print(f" {state_name}")

    try:
        tty.setcbreak(sys.stdin.fileno())
        print()
        print("  Ready — press keys to control throttle")
        print()

        last_display = 0
        inject_interval = 0.020  # 50 Hz
        last_inject = 0

        while True:
            now = time.monotonic()

            # Check for keypress (non-blocking)
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                # Handle arrow key escape sequences: \x1b [ A/B/C/D
                if ch == '\x1b':
                    if select.select([sys.stdin], [], [], 0.05)[0]:
                        ch2 = sys.stdin.read(1)
                        if ch2 == '[' and select.select([sys.stdin], [], [], 0.05)[0]:
                            ch3 = sys.stdin.read(1)
                            if ch3 == 'A':    # Up arrow
                                throttle_val = min(throttle_val + val_step, val_max)
                            elif ch3 == 'B':  # Down arrow
                                throttle_val = max(throttle_val - val_step, val_min if throttle_val > val_min else val_zero)
                elif ch in ('q', 'Q', '\x03'):  # q or Ctrl+C
                    break
                elif ch in ('+', '=', ']'):  # increase
                    throttle_val = min(throttle_val + val_step, val_max)
                elif ch in ('-', '_', '['):  # decrease
                    throttle_val = max(throttle_val - val_step, val_min if throttle_val > val_min else val_zero)
                elif ch == '0':
                    throttle_val = val_zero
                elif ch.isdigit():
                    if use_dshot:
                        # 1→270, 2→492, ... 9→1847 (spread across 48-2047)
                        throttle_val = val_min + int(ch) * (val_max - val_min) // 10
                    else:
                        throttle_val = 1000 + int(ch) * 100  # 1→1100, 9→1900

            # Inject at 50 Hz
            if now - last_inject >= inject_interval:
                rx_inject(ser, proto, throttle_val, 1)
                last_inject = now

            # Display telemetry at ~5 Hz
            if now - last_display >= 0.200:
                snap = get_full_snapshot(ser)
                if snap:
                    state_name = ESC_STATE_NAMES.get(snap["state"], f"?{snap['state']}")
                    if use_dshot:
                        if throttle_val <= RX_DSHOT_CMD_MAX:
                            throttle_pct = 0.0
                        else:
                            throttle_pct = (throttle_val - 48) / 19.99
                        line = (f"\r  DS={throttle_val:4d} ({throttle_pct:4.0f}%) "
                                f"| State={state_name:8s} "
                                f"| Duty={snap['dutyPct']:3d}% "
                                f"| Thr={snap['throttle']:4d}")
                    else:
                        throttle_pct = (throttle_val - 1000) / 10
                        line = (f"\r  PWM={throttle_val}us ({throttle_pct:4.0f}%) "
                                f"| State={state_name:8s} "
                                f"| Duty={snap['dutyPct']:3d}% "
                                f"| Thr={snap['throttle']:4d}")

                    if is_foc and "focIq" in snap:
                        omega = snap.get("focOmega", 0)
                        rpm = abs(omega) * 60 / (2 * 3.14159)
                        line += (f" | Iq={snap['focIq']:5.2f}A"
                                 f" Id={snap['focId']:5.2f}A"
                                 f" {rpm:5.0f}RPM")
                    elif not is_foc and snap.get("stepPeriod", 0) > 0:
                        sp = snap["stepPeriod"]
                        if sp > 0:
                            line += f" | stepPer={sp}"

                    line += "    "  # clear trailing chars
                    print(line, end="", flush=True)
                last_display = now

            time.sleep(0.005)  # 5ms poll

    except KeyboardInterrupt:
        pass
    finally:
        cleanup()


# ── Main ────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="GSP RX Input Simulation Test")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--skip-timeout", action="store_true",
                        help="Skip timeout tests (T6, T11) — they take ~300ms each")
    parser.add_argument("--motor-test", action="store_true",
                        help="Interactive motor control via RX inject (spins the motor!)")
    parser.add_argument("--dshot", action="store_true",
                        help="Use DShot600 protocol for motor test (default: PWM)")
    args = parser.parse_args()

    print(f"Connecting to {args.port} @ {args.baud}...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    time.sleep(0.2)
    drain_unsolicited(ser)

    # Verify connectivity
    test_ping(ser)

    # Check RX features and pick the right throttle source
    rx_src = THROTTLE_SRC_AUTO  # default
    resp = send_cmd(ser, GSP_CMD_GET_INFO)
    if resp and resp[0] == GSP_CMD_GET_INFO and len(resp[1]) >= 14:
        flags = struct.unpack_from("<I", resp[1], 8)[0]
        has_rx_pwm   = bool(flags & (1 << 20))
        has_rx_dshot = bool(flags & (1 << 21))
        has_rx_auto  = bool(flags & (1 << 22))
        print(f"\n  Feature flags: RX_PWM={has_rx_pwm} RX_DSHOT={has_rx_dshot} RX_AUTO={has_rx_auto}")
        if has_rx_auto:
            rx_src = THROTTLE_SRC_AUTO
        elif has_rx_pwm:
            rx_src = THROTTLE_SRC_PWM
        elif has_rx_dshot:
            rx_src = THROTTLE_SRC_DSHOT
        else:
            print("  WARNING: No RX features enabled — inject tests will fail")
            print("  Build with FEATURE_RX_PWM=1 or FEATURE_RX_AUTO=1")
        print(f"  Using throttle source: {rx_src} ({'AUTO' if rx_src == 4 else 'PWM' if rx_src == 2 else 'DSHOT'})")

    # Detect FOC mode from feature flags
    is_foc = False
    if resp and resp[0] == GSP_CMD_GET_INFO and len(resp[1]) >= 14:
        is_foc = bool(flags & (1 << 23))  # FEATURE_FOC_V2
        if is_foc:
            print("  Mode: FOC V2")
        else:
            print("  Mode: 6-step")

    # Motor test mode
    if args.motor_test:
        motor_test(ser, rx_src, is_foc, use_dshot=args.dshot)
        ser.close()
        sys.exit(0)

    # Run tests
    test_rx_inject_ack(ser, rx_src)
    test_pwm_lock_fsm(ser, rx_src)
    test_pwm_throttle_conversion(ser, rx_src)
    test_dshot_lock_and_conversion(ser, rx_src)
    test_invalid_frames_reset_lock(ser, rx_src)
    test_crc_error_counter(ser, rx_src)
    test_pwm_full_range_sweep(ser, rx_src)
    test_dshot_full_range_sweep(ser, rx_src)
    test_auto_arm(ser, rx_src)

    if not args.skip_timeout:
        test_timeout_to_lost(ser, rx_src)
        test_redetect_after_lost(ser, rx_src)

    # Clean up
    ensure_idle(ser)
    ser.close()

    # Summary
    total = pass_count + fail_count
    print(f"\n{'='*50}")
    print(f"  Results: {pass_count}/{total} passed, {fail_count} failed")
    print(f"{'='*50}")
    sys.exit(0 if fail_count == 0 else 1)


if __name__ == "__main__":
    main()
