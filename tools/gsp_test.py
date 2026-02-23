#!/usr/bin/env python3
"""
GSP v1 Protocol Test Script

Tests the Garuda Serial Protocol over USB CDC (PKoB4 UART1).
Run with the board powered and firmware flashed with FEATURE_GSP=1.

Usage:
    python3 gsp_test.py [--port /dev/ttyACM0] [--baud 115200]

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
GSP_CMD_PING = 0x00
GSP_CMD_GET_INFO = 0x01
GSP_CMD_GET_SNAPSHOT = 0x02
GSP_CMD_ERROR = 0xFF

GSP_ERR_UNKNOWN_CMD = 0x01
GSP_ERR_BAD_LENGTH = 0x02

GSP_INFO_SIZE = 20
GSP_SNAPSHOT_SIZE = 68


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

    # Matches __attribute__((packed)) GSP_SNAPSHOT_T exactly:
    # Core (8B): state(B) fault(B) step(B) dir(B) throttle(H) dutyPct(B) pad0(B)
    # Bus  (6B): vbus(H) ibus(H) ibusMax(H)
    # BEMF (8B): bemf(H) zcThresh(H) stepPeriod(H) goodZcCount(H)
    # Flags(3B): rising(B) falling(B) synced(B)
    # Diag (5B): pad1(B) zcConfirmed(H) zcTimeoutForce(H)
    # HWZC(18B): enabled(B) phase(B) totalZc(I) totalMiss(I) stepPeriodHR(I) dbgLatch(B) pad2(B)
    # Morph(6B): subPhase(B) morphStep(B) morphZcCount(H) morphAlpha(H)
    # OC   (8B): clpci(I) fpci(I)
    # Sys  (8B): systemTick(I) uptimeSec(I)
    fmt = "<"
    fmt += "BBBBHBB"      # core (8B)
    fmt += "HHH"          # bus (6B)
    fmt += "HHHH"         # bemf/zc (8B)
    fmt += "BBB"          # zc flags (3B)
    fmt += "BHH"          # pad1 + zc diag (5B)
    fmt += "BBIIIB"       # hwzc (17B + pad2 = 18B... wait)
    fmt += "B"            # pad2
    fmt += "BBHH"         # morph (6B)
    fmt += "II"           # overcurrent (8B)
    fmt += "II"           # system (8B)

    try:
        fields = struct.unpack(fmt, payload)
    except struct.error as e:
        # If the packed struct doesn't match our guess, dump raw hex
        return {"error": str(e), "raw": payload.hex()}

    snap = {
        "state": STATE_NAMES.get(fields[0], f"unknown({fields[0]})"),
        "faultCode": FAULT_NAMES.get(fields[1], f"unknown({fields[1]})"),
        "currentStep": fields[2],
        "direction": fields[3],
        "throttle": fields[4],
        "dutyPct": fields[5],
        # fields[6] = pad0
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
        # fields[17] = pad1
        "zcConfirmedCount": fields[18],
        "zcTimeoutForceCount": fields[19],
        "hwzcEnabled": bool(fields[20]),
        "hwzcPhase": fields[21],
        "hwzcTotalZcCount": fields[22],
        "hwzcTotalMissCount": fields[23],
        "hwzcStepPeriodHR": fields[24],
        "hwzcDbgLatchDisable": bool(fields[25]),
        # fields[26] = pad2
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


# ── Test Cases ───────────────────────────────────────────────────────────

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
    parser = argparse.ArgumentParser(description="GSP v1 Protocol Tester")
    parser.add_argument("--port", default=None, help="Serial port (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--stress", type=int, default=1000, help="Stress test iterations")
    parser.add_argument("--skip-stress", action="store_true", help="Skip stress test")
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
