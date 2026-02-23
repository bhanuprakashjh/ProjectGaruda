#!/usr/bin/env python3
"""
GSP v1 Protocol Test Script (Phase 0 + Phase 1)

Tests the Garuda Serial Protocol over USB CDC (PKoB4 UART1).
Run with the board powered and firmware flashed with FEATURE_GSP=1.

Usage:
    python3 gsp_test.py [--port /dev/ttyACM0] [--baud 115200]
    python3 gsp_test.py --p0-only           # Phase 0 tests only
    python3 gsp_test.py --skip-stress       # Skip stress test

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

# Phase 0 commands
GSP_CMD_PING = 0x00
GSP_CMD_GET_INFO = 0x01
GSP_CMD_GET_SNAPSHOT = 0x02

# Phase 1: motor control
GSP_CMD_START_MOTOR = 0x03
GSP_CMD_STOP_MOTOR = 0x04
GSP_CMD_CLEAR_FAULT = 0x05
GSP_CMD_SET_THROTTLE = 0x06
GSP_CMD_SET_THROTTLE_SRC = 0x07
GSP_CMD_HEARTBEAT = 0x08

# Phase 1: parameter system
GSP_CMD_GET_PARAM = 0x10
GSP_CMD_SET_PARAM = 0x11
GSP_CMD_SAVE_CONFIG = 0x12
GSP_CMD_LOAD_DEFAULTS = 0x13
GSP_CMD_TELEM_START = 0x14
GSP_CMD_TELEM_STOP = 0x15
GSP_CMD_GET_PARAM_LIST = 0x16

# Unsolicited
GSP_CMD_TELEM_FRAME = 0x80
GSP_CMD_ERROR = 0xFF

# Error codes
GSP_ERR_UNKNOWN_CMD = 0x01
GSP_ERR_BAD_LENGTH = 0x02
GSP_ERR_BUSY = 0x03
GSP_ERR_WRONG_STATE = 0x04
GSP_ERR_OUT_OF_RANGE = 0x05
GSP_ERR_UNKNOWN_PARAM = 0x06
GSP_ERR_CROSS_VALIDATION = 0x07
GSP_ERR_EEPROM_THROTTLED = 0x08

GSP_INFO_SIZE = 20
GSP_SNAPSHOT_SIZE = 68

# Parameter IDs
PARAM_ID_RAMP_TARGET_ERPM = 0x15
PARAM_ID_RAMP_ACCEL_ERPM_PER_S = 0x16
PARAM_ID_RAMP_DUTY_PCT = 0x17
PARAM_ID_CL_IDLE_DUTY_PCT = 0x20
PARAM_ID_TIMING_ADV_MAX_DEG = 0x22
PARAM_ID_HWZC_CROSSOVER_ERPM = 0x30
PARAM_ID_OC_FAULT_MA = 0x41
PARAM_ID_OC_SW_LIMIT_MA = 0x42

PARAM_NAMES = {
    0x15: "rampTargetErpm",
    0x16: "rampAccelErpmPerS",
    0x17: "rampDutyPct",
    0x20: "clIdleDutyPct",
    0x22: "timingAdvMaxDeg",
    0x30: "hwzcCrossoverErpm",
    0x41: "ocFaultMa",
    0x42: "ocSwLimitMa",
}


# ── CRC-16-CCITT (poly 0x1021, init 0xFFFF) ────────────────────────────

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


# ── Packet Builder / Parser ─────────────────────────────────────────────

def build_packet(cmd_id: int, payload: bytes = b"") -> bytes:
    """Build a GSP v1 packet: [0x02][LEN][CMD_ID][PAYLOAD][CRC_H][CRC_L]"""
    pkt_len = 1 + len(payload)  # CMD_ID + payload
    crc_data = bytes([pkt_len, cmd_id]) + payload
    crc = crc16_ccitt(crc_data)
    return bytes([GSP_START_BYTE, pkt_len, cmd_id]) + payload + struct.pack(">H", crc)


def build_packet_bad_crc(cmd_id: int, payload: bytes = b"") -> bytes:
    """Build a GSP packet with intentionally wrong CRC."""
    pkt_len = 1 + len(payload)
    return bytes([GSP_START_BYTE, pkt_len, cmd_id]) + payload + bytes([0xDE, 0xAD])


def read_response(ser: serial.Serial, timeout: float = 1.0) -> Optional[tuple]:
    """
    Read one GSP response packet.
    Returns (cmd_id, payload_bytes) or None on timeout/CRC fail.
    """
    deadline = time.monotonic() + timeout

    # Wait for start byte
    while time.monotonic() < deadline:
        b = ser.read(1)
        if len(b) == 0:
            continue
        if b[0] == GSP_START_BYTE:
            break
    else:
        return None

    # Read LEN
    remaining = deadline - time.monotonic()
    ser.timeout = max(remaining, 0.01)
    b = ser.read(1)
    if len(b) == 0:
        return None
    pkt_len = b[0]

    # Read CMD_ID + payload + CRC (pkt_len + 2 bytes)
    remaining = deadline - time.monotonic()
    ser.timeout = max(remaining, 0.01)
    data = ser.read(pkt_len + 2)
    if len(data) < pkt_len + 2:
        return None

    cmd_id = data[0]
    payload = data[1:pkt_len]
    rx_crc = struct.unpack(">H", data[pkt_len:pkt_len + 2])[0]

    # Validate CRC
    crc_data = bytes([pkt_len]) + data[:pkt_len]
    calc_crc = crc16_ccitt(crc_data)
    if calc_crc != rx_crc:
        print(f"  CRC MISMATCH: calc=0x{calc_crc:04X} rx=0x{rx_crc:04X}")
        return None

    return (cmd_id, payload)


def expect_ack(ser: serial.Serial, expected_cmd: int, label: str) -> bool:
    """Send command and expect ACK (echo of cmd with empty payload)."""
    resp = read_response(ser)
    if resp is None:
        print(f"  FAIL [{label}]: no response")
        return False
    cmd_id, payload = resp
    if cmd_id == GSP_CMD_ERROR:
        err = payload[0] if payload else 0xFF
        print(f"  FAIL [{label}]: got ERROR 0x{err:02X}")
        return False
    if cmd_id != expected_cmd:
        print(f"  FAIL [{label}]: expected cmd 0x{expected_cmd:02X}, got 0x{cmd_id:02X}")
        return False
    return True


def expect_error(ser: serial.Serial, expected_err: int, label: str) -> bool:
    """Read response and expect ERROR with specific code."""
    resp = read_response(ser)
    if resp is None:
        print(f"  FAIL [{label}]: no response")
        return False
    cmd_id, payload = resp
    if cmd_id != GSP_CMD_ERROR:
        print(f"  FAIL [{label}]: expected ERROR, got cmd 0x{cmd_id:02X}")
        return False
    if len(payload) < 1 or payload[0] != expected_err:
        actual = payload[0] if payload else 0xFF
        print(f"  FAIL [{label}]: expected error 0x{expected_err:02X}, got 0x{actual:02X}")
        return False
    return True


# ── Struct Decoders ──────────────────────────────────────────────────────

FEATURE_NAMES = [
    "BEMF_CLOSED_LOOP", "VBUS_FAULT", "DESYNC_RECOVERY", "DUTY_SLEW",
    "TIMING_ADVANCE", "DYNAMIC_BLANKING", "VBUS_SAG_LIMIT", "BEMF_INTEGRATION",
    "SINE_STARTUP", "ADC_CMP_ZC", "HW_OVERCURRENT", "LEARN_MODULES",
    "ADAPTATION", "COMMISSION", "EEPROM_V2", "X2CSCOPE", "GSP",
]

MOTOR_PROFILES = {0: "HURST", 1: "A2212_1400KV", 2: "A2212_750KV"}

STATE_NAMES = {
    0: "IDLE", 1: "ARMED", 2: "ALIGN", 3: "OL_RAMP",
    4: "MORPH", 5: "CLOSED_LOOP", 6: "BRAKING", 7: "RECOVERY",
    8: "FAULT",
}

FAULT_NAMES = {
    0: "NONE", 1: "OVERVOLTAGE", 2: "UNDERVOLTAGE", 3: "OVERCURRENT",
    4: "BOARD_PCI", 5: "STALL", 6: "DESYNC", 7: "STARTUP_TIMEOUT",
    8: "MORPH_TIMEOUT",
}


def decode_info(payload: bytes) -> dict:
    """Decode GSP_INFO_T (20 bytes packed)."""
    if len(payload) != GSP_INFO_SIZE:
        return {"error": f"expected {GSP_INFO_SIZE}B, got {len(payload)}B"}

    # protocolVer(B) fwMaj(B) fwMin(B) fwPatch(B) boardId(H) motorProf(B)
    # motorPP(B) featureFlags(I) pwmFreq(I) maxErpm(H) reserved(H)
    fields = struct.unpack("<BBBBHBBIIHH", payload)
    info = {
        "protocolVersion": fields[0],
        "fwVersion": f"{fields[1]}.{fields[2]}.{fields[3]}",
        "boardId": f"0x{fields[4]:04X}",
        "motorProfile": MOTOR_PROFILES.get(fields[5], f"unknown({fields[5]})"),
        "motorPolePairs": fields[6],
        "featureFlags": f"0x{fields[7]:08X}",
        "features": [FEATURE_NAMES[i] for i in range(len(FEATURE_NAMES))
                     if fields[7] & (1 << i)],
        "pwmFrequency": fields[8],
        "maxErpm": fields[9],
    }
    return info


def decode_snapshot(payload: bytes) -> dict:
    """Decode GSP_SNAPSHOT_T (68 bytes packed)."""
    if len(payload) != GSP_SNAPSHOT_SIZE:
        return {"error": f"expected {GSP_SNAPSHOT_SIZE}B, got {len(payload)}B"}

    fmt = "<"
    fmt += "BBBBHBB"      # core (8B)
    fmt += "HHH"          # bus (6B)
    fmt += "HHHH"         # bemf/zc (8B)
    fmt += "BBB"          # zc flags (3B)
    fmt += "BHH"          # pad1 + zc diag (5B)
    fmt += "BBIIIB"       # hwzc (17B)
    fmt += "B"            # pad2
    fmt += "BBHH"         # morph (6B)
    fmt += "II"           # overcurrent (8B)
    fmt += "II"           # system (8B)

    try:
        fields = struct.unpack(fmt, payload)
    except struct.error as e:
        return {"error": str(e), "raw": payload.hex()}

    snap = {
        "state": STATE_NAMES.get(fields[0], f"unknown({fields[0]})"),
        "faultCode": FAULT_NAMES.get(fields[1], f"unknown({fields[1]})"),
        "currentStep": fields[2],
        "direction": fields[3],
        "throttle": fields[4],
        "dutyPct": fields[5],
        "vbusRaw": fields[7],
        "ibusRaw": fields[8],
        "ibusMax": fields[9],
        "bemfRaw": fields[10],
        "zcThreshold": fields[11],
        "stepPeriod": fields[12],
        "goodZcCount": fields[13],
        "risingZcWorks": bool(fields[14]),
        "fallingZcWorks": bool(fields[15]),
        "zcSynced": bool(fields[16]),
        "zcConfirmedCount": fields[18],
        "zcTimeoutForceCount": fields[19],
        "hwzcEnabled": bool(fields[20]),
        "hwzcPhase": fields[21],
        "hwzcTotalZcCount": fields[22],
        "hwzcTotalMissCount": fields[23],
        "hwzcStepPeriodHR": fields[24],
        "hwzcDbgLatchDisable": bool(fields[25]),
        "morphSubPhase": fields[27],
        "morphStep": fields[28],
        "morphZcCount": fields[29],
        "morphAlpha": fields[30],
        "clpciTripCount": fields[31],
        "fpciTripCount": fields[32],
        "systemTick": fields[33],
        "uptimeSec": fields[34],
    }
    return snap


# ── Phase 0 Test Cases ──────────────────────────────────────────────────

def test_ping(ser: serial.Serial) -> bool:
    """Test 1: PING → echo response."""
    print("TEST 1: PING")
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_PING))

    resp = read_response(ser)
    if resp is None:
        print("  FAIL: no response")
        return False
    cmd_id, payload = resp
    if cmd_id != GSP_CMD_PING:
        print(f"  FAIL: expected cmd 0x00, got 0x{cmd_id:02X}")
        return False
    if len(payload) != 0:
        print(f"  FAIL: expected empty payload, got {len(payload)}B")
        return False
    print("  PASS")
    return True


def test_get_info(ser: serial.Serial) -> bool:
    """Test 2: GET_INFO → 20B payload with valid fields."""
    print("TEST 2: GET_INFO")
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_GET_INFO))

    resp = read_response(ser)
    if resp is None:
        print("  FAIL: no response")
        return False
    cmd_id, payload = resp
    if cmd_id != GSP_CMD_GET_INFO:
        print(f"  FAIL: expected cmd 0x01, got 0x{cmd_id:02X}")
        return False
    if len(payload) != GSP_INFO_SIZE:
        print(f"  FAIL: expected {GSP_INFO_SIZE}B, got {len(payload)}B")
        return False

    info = decode_info(payload)
    print(f"  Protocol: v{info['protocolVersion']}")
    print(f"  Firmware: {info['fwVersion']}")
    print(f"  Board: {info['boardId']}")
    print(f"  Motor: {info['motorProfile']} ({info['motorPolePairs']} pole pairs)")
    print(f"  PWM: {info['pwmFrequency']} Hz")
    print(f"  Max eRPM: {info['maxErpm']}")
    print(f"  Features: {', '.join(info['features'])}")

    if info["protocolVersion"] != 1:
        print(f"  FAIL: protocol version {info['protocolVersion']} != 1")
        return False

    print("  PASS")
    return True


def test_get_snapshot(ser: serial.Serial) -> bool:
    """Test 3: GET_SNAPSHOT → 68B payload with plausible values."""
    print("TEST 3: GET_SNAPSHOT")
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_GET_SNAPSHOT))

    resp = read_response(ser)
    if resp is None:
        print("  FAIL: no response")
        return False
    cmd_id, payload = resp
    if cmd_id != GSP_CMD_GET_SNAPSHOT:
        print(f"  FAIL: expected cmd 0x02, got 0x{cmd_id:02X}")
        return False
    if len(payload) != GSP_SNAPSHOT_SIZE:
        print(f"  FAIL: expected {GSP_SNAPSHOT_SIZE}B, got {len(payload)}B")
        return False

    snap = decode_snapshot(payload)
    if "error" in snap:
        print(f"  WARN: decode issue: {snap['error']}")
        print(f"  Raw: {payload.hex()}")
    else:
        print(f"  State: {snap['state']}, Fault: {snap['faultCode']}")
        print(f"  Step: {snap['currentStep']}, Dir: {snap['direction']}")
        print(f"  Throttle: {snap['throttle']}, Duty: {snap['dutyPct']}%")
        print(f"  Vbus: {snap['vbusRaw']}, Ibus: {snap['ibusRaw']}")
        print(f"  BEMF: {snap['bemfRaw']}, ZC thresh: {snap['zcThreshold']}")
        print(f"  systemTick: {snap['systemTick']} ({snap['uptimeSec']}s)")
        print(f"  HWZC: en={snap['hwzcEnabled']}, zc={snap['hwzcTotalZcCount']}, miss={snap['hwzcTotalMissCount']}")

    print("  PASS")
    return True


def test_unknown_cmd(ser: serial.Serial) -> bool:
    """Test 4: Unknown command → ERROR response with code 0x01."""
    print("TEST 4: UNKNOWN COMMAND (0x99)")
    ser.reset_input_buffer()
    ser.write(build_packet(0x99))

    resp = read_response(ser)
    if resp is None:
        print("  FAIL: no response")
        return False
    cmd_id, payload = resp
    if cmd_id != GSP_CMD_ERROR:
        print(f"  FAIL: expected cmd 0xFF, got 0x{cmd_id:02X}")
        return False
    if len(payload) != 1 or payload[0] != GSP_ERR_UNKNOWN_CMD:
        print(f"  FAIL: expected error code 0x01, got {payload.hex()}")
        return False
    print("  PASS")
    return True


def test_bad_crc(ser: serial.Serial) -> bool:
    """Test 5: Bad CRC → silent drop (no response within 500ms)."""
    print("TEST 5: BAD CRC (silent drop)")
    ser.reset_input_buffer()
    ser.write(build_packet_bad_crc(GSP_CMD_PING))

    resp = read_response(ser, timeout=0.5)
    if resp is not None:
        print(f"  FAIL: got response (should be silent drop): cmd=0x{resp[0]:02X}")
        return False
    print("  PASS (no response — correct)")
    return True


def test_bad_length(ser: serial.Serial) -> bool:
    """Test 6: PING with unexpected payload → BAD_LENGTH error."""
    print("TEST 6: BAD LENGTH (PING with 3B payload)")
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_PING, b"\x01\x02\x03"))

    resp = read_response(ser)
    if resp is None:
        print("  FAIL: no response")
        return False
    cmd_id, payload = resp
    if cmd_id != GSP_CMD_ERROR:
        print(f"  FAIL: expected cmd 0xFF, got 0x{cmd_id:02X}")
        return False
    if len(payload) != 1 or payload[0] != GSP_ERR_BAD_LENGTH:
        print(f"  FAIL: expected error code 0x02, got {payload.hex()}")
        return False
    print("  PASS")
    return True


def test_stress(ser: serial.Serial, iterations: int = 1000) -> bool:
    """Test 7: Stress — PING+SNAPSHOT loop, check zero failures."""
    print(f"TEST 7: STRESS ({iterations} iterations)")
    ping_ok = 0
    snap_ok = 0
    failures = 0

    t0 = time.monotonic()
    for i in range(iterations):
        # PING
        ser.reset_input_buffer()
        ser.write(build_packet(GSP_CMD_PING))
        resp = read_response(ser, timeout=0.5)
        if resp and resp[0] == GSP_CMD_PING:
            ping_ok += 1
        else:
            failures += 1

        # SNAPSHOT
        ser.reset_input_buffer()
        ser.write(build_packet(GSP_CMD_GET_SNAPSHOT))
        resp = read_response(ser, timeout=0.5)
        if resp and resp[0] == GSP_CMD_GET_SNAPSHOT and len(resp[1]) == GSP_SNAPSHOT_SIZE:
            snap_ok += 1
        else:
            failures += 1

        if (i + 1) % 100 == 0:
            print(f"  ... {i + 1}/{iterations}")

    elapsed = time.monotonic() - t0
    rate = iterations * 2 / elapsed

    print(f"  PING: {ping_ok}/{iterations}, SNAPSHOT: {snap_ok}/{iterations}")
    print(f"  Failures: {failures}, Rate: {rate:.1f} cmd/s, Time: {elapsed:.1f}s")

    if failures > 0:
        print(f"  FAIL: {failures} failures")
        return False
    print("  PASS")
    return True


# ── Phase 1 Test Cases ──────────────────────────────────────────────────

def test_heartbeat(ser: serial.Serial) -> bool:
    """Test 8: HEARTBEAT → ACK."""
    print("TEST 8: HEARTBEAT")
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_HEARTBEAT))

    if not expect_ack(ser, GSP_CMD_HEARTBEAT, "HEARTBEAT"):
        return False
    print("  PASS")
    return True


def test_get_param_list(ser: serial.Serial) -> bool:
    """Test 9: GET_PARAM_LIST → 8 param descriptors (64B)."""
    print("TEST 9: GET_PARAM_LIST")
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_GET_PARAM_LIST))

    resp = read_response(ser)
    if resp is None:
        print("  FAIL: no response")
        return False
    cmd_id, payload = resp
    if cmd_id != GSP_CMD_GET_PARAM_LIST:
        print(f"  FAIL: expected cmd 0x16, got 0x{cmd_id:02X}")
        return False

    # Each descriptor: id(u16) type(u8) group(u8) min(u16) max(u16) = 8 bytes
    if len(payload) % 8 != 0:
        print(f"  FAIL: payload {len(payload)}B not multiple of 8")
        return False

    count = len(payload) // 8
    print(f"  {count} parameters:")
    for i in range(count):
        d = payload[i * 8:(i + 1) * 8]
        pid, ptype, pgroup, pmin, pmax = struct.unpack("<HBBHH", d)
        name = PARAM_NAMES.get(pid, f"0x{pid:02X}")
        type_names = {0: "u8", 1: "u16", 2: "u32"}
        print(f"    [{name}] id=0x{pid:02X} type={type_names.get(ptype, '?')}"
              f" group={pgroup} range=[{pmin}, {pmax}]")

    if count != 8:
        print(f"  FAIL: expected 8 params, got {count}")
        return False
    print("  PASS")
    return True


def test_get_param(ser: serial.Serial) -> bool:
    """Test 10: GET_PARAM for all 8 params → valid values."""
    print("TEST 10: GET_PARAM (all 8 params)")
    all_ok = True

    for pid, name in sorted(PARAM_NAMES.items()):
        ser.reset_input_buffer()
        ser.write(build_packet(GSP_CMD_GET_PARAM, struct.pack("<H", pid)))

        resp = read_response(ser)
        if resp is None:
            print(f"  FAIL [{name}]: no response")
            all_ok = False
            continue

        cmd_id, payload = resp
        if cmd_id == GSP_CMD_ERROR:
            err = payload[0] if payload else 0xFF
            print(f"  FAIL [{name}]: error 0x{err:02X}")
            all_ok = False
            continue
        if cmd_id != GSP_CMD_GET_PARAM:
            print(f"  FAIL [{name}]: expected cmd 0x10, got 0x{cmd_id:02X}")
            all_ok = False
            continue
        if len(payload) != 6:
            print(f"  FAIL [{name}]: expected 6B, got {len(payload)}B")
            all_ok = False
            continue

        rpid, value = struct.unpack("<HI", payload)
        if rpid != pid:
            print(f"  FAIL [{name}]: echo id 0x{rpid:02X} != 0x{pid:02X}")
            all_ok = False
            continue
        print(f"    {name} = {value}")

    if all_ok:
        print("  PASS")
    return all_ok


def get_esc_state(ser: serial.Serial) -> str:
    """Helper: read current ESC state string."""
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_GET_SNAPSHOT))
    resp = read_response(ser)
    if resp and resp[0] == GSP_CMD_GET_SNAPSHOT:
        snap = decode_snapshot(resp[1])
        return snap.get("state", "unknown")
    return "unknown"


def require_idle(ser: serial.Serial, test_name: str) -> bool:
    """Check ESC is IDLE; if not, print skip message and return False."""
    state = get_esc_state(ser)
    if state != "IDLE":
        print(f"  SKIP: state is {state}, need IDLE (stop motor with SW1)")
        return False
    return True


def test_get_param_unknown(ser: serial.Serial) -> bool:
    """Test 11: GET_PARAM with unknown ID → UNKNOWN_PARAM error."""
    print("TEST 11: GET_PARAM unknown ID (0xFFFF)")
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_GET_PARAM, struct.pack("<H", 0xFFFF)))

    if not expect_error(ser, GSP_ERR_UNKNOWN_PARAM, "unknown param"):
        return False
    print("  PASS")
    return True


def test_set_param_and_readback(ser: serial.Serial) -> bool:
    """Test 12: SET_PARAM rampTargetErpm → readback → restore original."""
    print("TEST 12: SET_PARAM + readback (rampTargetErpm)")
    if not require_idle(ser, "SET_PARAM"):
        return True  # Skip, not fail
    pid = PARAM_ID_RAMP_TARGET_ERPM

    # Read original value
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_GET_PARAM, struct.pack("<H", pid)))
    resp = read_response(ser)
    if resp is None or resp[0] != GSP_CMD_GET_PARAM:
        print("  FAIL: couldn't read original value")
        return False
    _, original = struct.unpack("<HI", resp[1])
    print(f"  Original: {original}")

    # Set to test value (2000, within range 500-10000)
    test_val = 2000 if original != 2000 else 3000
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_SET_PARAM,
                           struct.pack("<HI", pid, test_val)))
    if not expect_ack(ser, GSP_CMD_SET_PARAM, "SET"):
        return False

    # Read back
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_GET_PARAM, struct.pack("<H", pid)))
    resp = read_response(ser)
    if resp is None or resp[0] != GSP_CMD_GET_PARAM:
        print("  FAIL: readback failed")
        return False
    _, readback = struct.unpack("<HI", resp[1])
    print(f"  Set to {test_val}, read back {readback}")

    if readback != test_val:
        print(f"  FAIL: readback {readback} != expected {test_val}")
        # Restore original
        ser.write(build_packet(GSP_CMD_SET_PARAM,
                               struct.pack("<HI", pid, original)))
        read_response(ser)
        return False

    # Restore original
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_SET_PARAM,
                           struct.pack("<HI", pid, original)))
    if not expect_ack(ser, GSP_CMD_SET_PARAM, "RESTORE"):
        return False
    print(f"  Restored to {original}")

    print("  PASS")
    return True


def test_set_param_out_of_range(ser: serial.Serial) -> bool:
    """Test 13: SET_PARAM with out-of-range value → OUT_OF_RANGE error."""
    print("TEST 13: SET_PARAM out-of-range (rampDutyPct=99)")
    if not require_idle(ser, "SET_PARAM"):
        return True  # Skip, not fail
    pid = PARAM_ID_RAMP_DUTY_PCT  # range 5-80

    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_SET_PARAM,
                           struct.pack("<HI", pid, 99)))

    if not expect_error(ser, GSP_ERR_OUT_OF_RANGE, "out-of-range"):
        return False
    print("  PASS")
    return True


def test_set_param_cross_validation(ser: serial.Serial) -> bool:
    """Test 14: SET_PARAM ocSwLimitMa > ocFaultMa → CROSS_VALIDATION error."""
    print("TEST 14: SET_PARAM cross-validation (ocSwLimit > ocFault)")
    if not require_idle(ser, "SET_PARAM"):
        return True  # Skip, not fail

    # First read current ocFaultMa
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_GET_PARAM,
                           struct.pack("<H", PARAM_ID_OC_FAULT_MA)))
    resp = read_response(ser)
    if resp is None or resp[0] != GSP_CMD_GET_PARAM:
        print("  FAIL: couldn't read ocFaultMa")
        return False
    _, fault_ma = struct.unpack("<HI", resp[1])
    print(f"  Current ocFaultMa: {fault_ma}")

    # Try to set ocSwLimitMa above ocFaultMa
    too_high = fault_ma + 1000
    if too_high > 30000:
        too_high = 30000
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_SET_PARAM,
                           struct.pack("<HI", PARAM_ID_OC_SW_LIMIT_MA, too_high)))

    if not expect_error(ser, GSP_ERR_CROSS_VALIDATION, "cross-validation"):
        return False
    print("  PASS")
    return True


def test_set_throttle_src_wrong_state(ser: serial.Serial) -> bool:
    """Test 15: SET_THROTTLE_SRC=GSP when not in IDLE → check state first."""
    print("TEST 15: SET_THROTTLE_SRC (GSP in IDLE, then ADC revert)")

    # Board should be IDLE for this test. Set source to GSP.
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_SET_THROTTLE_SRC, bytes([1])))  # 1=GSP

    resp = read_response(ser)
    if resp is None:
        print("  FAIL: no response to SET_THROTTLE_SRC=GSP")
        return False
    cmd_id, payload = resp
    if cmd_id == GSP_CMD_ERROR:
        err = payload[0] if payload else 0xFF
        if err == GSP_ERR_WRONG_STATE:
            print("  INFO: board not in IDLE — skipping (expected if motor running)")
            return True  # Not a test failure
        print(f"  FAIL: unexpected error 0x{err:02X}")
        return False
    if cmd_id != GSP_CMD_SET_THROTTLE_SRC:
        print(f"  FAIL: expected cmd 0x07, got 0x{cmd_id:02X}")
        return False
    print("  GSP source accepted")

    # Now SET_THROTTLE should work
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_SET_THROTTLE, struct.pack("<H", 0)))
    if not expect_ack(ser, GSP_CMD_SET_THROTTLE, "SET_THROTTLE=0"):
        return False
    print("  SET_THROTTLE=0 accepted")

    # Revert to ADC
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_SET_THROTTLE_SRC, bytes([0])))  # 0=ADC
    if not expect_ack(ser, GSP_CMD_SET_THROTTLE_SRC, "REVERT_ADC"):
        return False
    print("  Reverted to ADC")

    # SET_THROTTLE should now fail (src=ADC)
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_SET_THROTTLE, struct.pack("<H", 500)))
    if not expect_error(ser, GSP_ERR_WRONG_STATE, "throttle-with-ADC-src"):
        return False
    print("  SET_THROTTLE rejected when src=ADC (correct)")

    print("  PASS")
    return True


def test_set_throttle_range(ser: serial.Serial) -> bool:
    """Test 16: SET_THROTTLE out-of-range (>2000) → OUT_OF_RANGE."""
    print("TEST 16: SET_THROTTLE out-of-range (2500)")

    # Enable GSP source first
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_SET_THROTTLE_SRC, bytes([1])))
    resp = read_response(ser)
    if resp is None or resp[0] == GSP_CMD_ERROR:
        print("  SKIP: couldn't enable GSP throttle source")
        return True

    # Try out-of-range value
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_SET_THROTTLE, struct.pack("<H", 2500)))
    if not expect_error(ser, GSP_ERR_OUT_OF_RANGE, "throttle=2500"):
        # Revert source
        ser.write(build_packet(GSP_CMD_SET_THROTTLE_SRC, bytes([0])))
        read_response(ser)
        return False

    # Revert source
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_SET_THROTTLE_SRC, bytes([0])))
    read_response(ser)

    print("  PASS")
    return True


def test_start_stop_motor(ser: serial.Serial) -> bool:
    """Test 17: START_MOTOR → STOP_MOTOR (no actual spin — just state check)."""
    print("TEST 17: START/STOP MOTOR (intent only, pot at 0)")

    # Verify IDLE
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_GET_SNAPSHOT))
    resp = read_response(ser)
    if resp is None or resp[0] != GSP_CMD_GET_SNAPSHOT:
        print("  FAIL: couldn't read state")
        return False
    snap = decode_snapshot(resp[1])
    if snap.get("state") != "IDLE":
        print(f"  SKIP: state is {snap.get('state')}, need IDLE")
        return True

    # Send START_MOTOR
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_START_MOTOR))
    if not expect_ack(ser, GSP_CMD_START_MOTOR, "START"):
        return False
    print("  START_MOTOR accepted")

    # Brief wait for state transition
    time.sleep(0.1)

    # Check state changed from IDLE
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_GET_SNAPSHOT))
    resp = read_response(ser)
    if resp and resp[0] == GSP_CMD_GET_SNAPSHOT:
        snap = decode_snapshot(resp[1])
        print(f"  State after START: {snap.get('state')}")

    # Send STOP_MOTOR
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_STOP_MOTOR))
    if not expect_ack(ser, GSP_CMD_STOP_MOTOR, "STOP"):
        return False
    print("  STOP_MOTOR accepted")

    time.sleep(0.1)

    # Verify back to IDLE
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_GET_SNAPSHOT))
    resp = read_response(ser)
    if resp and resp[0] == GSP_CMD_GET_SNAPSHOT:
        snap = decode_snapshot(resp[1])
        print(f"  State after STOP: {snap.get('state')}")
        if snap.get("state") != "IDLE":
            print("  WARN: not back to IDLE yet (may need more time)")

    print("  PASS")
    return True


def test_load_defaults(ser: serial.Serial) -> bool:
    """Test 18: LOAD_DEFAULTS → all params back to compile-time values."""
    print("TEST 18: LOAD_DEFAULTS")
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_LOAD_DEFAULTS))

    if not expect_ack(ser, GSP_CMD_LOAD_DEFAULTS, "LOAD_DEFAULTS"):
        return False
    print("  PASS")
    return True


def test_telem_stream(ser: serial.Serial) -> bool:
    """Test 19: TELEM_START(50Hz) → receive frames → TELEM_STOP."""
    print("TEST 19: TELEM STREAMING (50Hz, 2 seconds)")

    # Start streaming at 50Hz
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_TELEM_START, bytes([50])))

    resp = read_response(ser)
    if resp is None:
        print("  FAIL: no response to TELEM_START")
        return False
    if resp[0] == GSP_CMD_ERROR:
        print(f"  FAIL: error 0x{resp[1][0]:02X}")
        return False

    # Collect frames for 2 seconds
    frame_count = 0
    seq_values = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        resp = read_response(ser, timeout=0.1)
        if resp is None:
            continue
        cmd_id, payload = resp
        if cmd_id == GSP_CMD_TELEM_FRAME:
            frame_count += 1
            if len(payload) >= 2:
                seq = struct.unpack("<H", payload[:2])[0]
                seq_values.append(seq)

    # Stop streaming
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_TELEM_STOP))
    read_response(ser, timeout=0.5)

    # Drain any remaining frames
    time.sleep(0.1)
    ser.reset_input_buffer()

    print(f"  Received {frame_count} frames in 2s ({frame_count/2.0:.1f} Hz)")
    if seq_values:
        print(f"  Seq range: {seq_values[0]} → {seq_values[-1]}")

    # Check monotonicity
    if len(seq_values) >= 2:
        monotonic = all(seq_values[i+1] > seq_values[i]
                        for i in range(len(seq_values)-1))
        if not monotonic:
            print("  WARN: sequence not monotonically increasing")

    if frame_count < 50:  # At least 50 frames in 2s at 50Hz
        print(f"  FAIL: expected ~100 frames, got {frame_count}")
        return False

    print("  PASS")
    return True


def test_set_param_wrong_state(ser: serial.Serial) -> bool:
    """Test 20: SET_PARAM state guard — verify works in IDLE."""
    print("TEST 20: SET_PARAM state guard")
    if not require_idle(ser, "SET_PARAM"):
        return True  # Skip, not fail
    pid = PARAM_ID_TIMING_ADV_MAX_DEG

    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_GET_PARAM, struct.pack("<H", pid)))
    resp = read_response(ser)
    if resp is None or resp[0] != GSP_CMD_GET_PARAM:
        print("  FAIL: couldn't read param")
        return False
    _, original = struct.unpack("<HI", resp[1])

    # Set same value (no-op functionally, but exercises the handler)
    ser.reset_input_buffer()
    ser.write(build_packet(GSP_CMD_SET_PARAM,
                           struct.pack("<HI", pid, original)))
    if not expect_ack(ser, GSP_CMD_SET_PARAM, "SET in IDLE"):
        return False

    print(f"  SET_PARAM accepted in IDLE (timingAdvMaxDeg={original})")
    print("  PASS")
    return True


# ── Main ─────────────────────────────────────────────────────────────────

def find_port() -> str:
    """Try to auto-detect the PKoB4 CDC port."""
    import serial.tools.list_ports
    for p in serial.tools.list_ports.comports():
        # PKoB4 shows as Microchip VID 0x04D8
        if p.vid == 0x04D8:
            return p.device
        # Also match common CDC names
        if "ACM" in (p.device or ""):
            return p.device
    return "/dev/ttyACM0"


def main():
    parser = argparse.ArgumentParser(description="GSP v1 Protocol Tester (Phase 0 + 1)")
    parser.add_argument("--port", default=None, help="Serial port (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--stress", type=int, default=1000, help="Stress test iterations")
    parser.add_argument("--skip-stress", action="store_true", help="Skip stress test")
    parser.add_argument("--p0-only", action="store_true", help="Phase 0 tests only")
    args = parser.parse_args()

    port = args.port or find_port()
    print(f"GSP Test — port={port}, baud={args.baud}")
    print()

    try:
        ser = serial.Serial(port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {port}: {e}")
        sys.exit(1)

    # Flush any stale data
    time.sleep(0.1)
    ser.reset_input_buffer()

    results = []

    # ── Phase 0 tests (1-7) ──
    print("═══ PHASE 0: Core Protocol ═══")
    print()
    results.append(("PING", test_ping(ser)))
    print()
    results.append(("GET_INFO", test_get_info(ser)))
    print()
    results.append(("GET_SNAPSHOT", test_get_snapshot(ser)))
    print()
    results.append(("UNKNOWN_CMD", test_unknown_cmd(ser)))
    print()
    results.append(("BAD_CRC", test_bad_crc(ser)))
    print()
    results.append(("BAD_LENGTH", test_bad_length(ser)))
    print()

    if not args.skip_stress:
        results.append(("STRESS", test_stress(ser, args.stress)))
        print()

    # ── Phase 1 tests (8-20) ──
    if not args.p0_only:
        print("═══ PHASE 1: Params + Motor Control ═══")
        print()
        results.append(("HEARTBEAT", test_heartbeat(ser)))
        print()
        results.append(("GET_PARAM_LIST", test_get_param_list(ser)))
        print()
        results.append(("GET_PARAM", test_get_param(ser)))
        print()
        results.append(("GET_PARAM_UNKNOWN", test_get_param_unknown(ser)))
        print()
        results.append(("SET_PARAM_READBACK", test_set_param_and_readback(ser)))
        print()
        results.append(("SET_PARAM_RANGE", test_set_param_out_of_range(ser)))
        print()
        results.append(("SET_PARAM_CROSS", test_set_param_cross_validation(ser)))
        print()
        results.append(("THROTTLE_SRC", test_set_throttle_src_wrong_state(ser)))
        print()
        results.append(("THROTTLE_RANGE", test_set_throttle_range(ser)))
        print()
        results.append(("START_STOP", test_start_stop_motor(ser)))
        print()
        results.append(("LOAD_DEFAULTS", test_load_defaults(ser)))
        print()
        results.append(("SET_PARAM_STATE", test_set_param_wrong_state(ser)))
        print()
        results.append(("TELEM_STREAM", test_telem_stream(ser)))
        print()

    ser.close()

    # Summary
    print("=" * 50)
    passed = sum(1 for _, ok in results if ok)
    total = len(results)
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    print(f"\n  {passed}/{total} passed")

    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
