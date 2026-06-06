#!/usr/bin/env python3
"""
check_max_erpm.py — read back maxClosedLoopErpm over GSP to verify a code edit
took effect (i.e. compiled profileDefaults won over stale EEPROM).

Sends GET_INFO (no SET/SAVE — read-only, safe) and prints the active profile +
maxErpm, then PASS/FAIL against an expected sentinel.

Usage:
    python3 tools/check_max_erpm.py [--port /dev/ttyACM0] [--expect 250000]
"""
import argparse
import struct
import sys
import time

import serial

GSP_START_BYTE   = 0x02
GSP_CMD_GET_INFO = 0x01
GSP_CMD_GET_PARAM = 0x10
GSP_CMD_TELEM_STOP = 0x15
GSP_INFO_SIZE_V2 = 20   # protocolVer..maxErpm
GSP_INFO_SIZE_V3 = 24   # V3 appended buildHash (djb2 of __DATE__ __TIME__)

# A few param name -> id mappings (GET_PARAM, read-only)
PARAM_IDS = {
    "rampTargetErpm":    0x15,
    "rampAccelErpmPerS": 0x16,
    "rampDutyPct":       0x17,
    "clIdleDutyPct":     0x20,
    "timingAdvMaxDeg":   0x22,
    "alignDutyPct":      0x51,
    "initialErpm":       0x52,
    "maxClosedLoopErpm": 0x53,
    "sineAlignModPct":   0x54,
    "sineRampModPct":    0x55,
}

MOTOR_PROFILES = {0: "Hurst", 1: "A2212", 2: "2810(5010)", 3: "5055",
                  4: "Cobra-2814", 5: "XRotor-3110"}


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def build_packet(cmd_id: int, payload: bytes = b"") -> bytes:
    pkt_len = 1 + len(payload)
    crc = crc16_ccitt(bytes([pkt_len, cmd_id]) + payload)
    return bytes([GSP_START_BYTE, pkt_len, cmd_id]) + payload + struct.pack(">H", crc)


def read_response(ser: serial.Serial, timeout: float = 1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        b = ser.read(1)
        if b and b[0] == GSP_START_BYTE:
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
    if crc16_ccitt(bytes([pkt_len]) + data[:pkt_len]) != rx_crc:
        print("  CRC mismatch on response")
        return None
    return cmd_id, payload


def decode_info(payload: bytes) -> dict:
    n = len(payload)
    if n == GSP_INFO_SIZE_V3:
        f = struct.unpack("<BBBBHBBIIII", payload)   # ..maxErpm, buildHash
        build_hash = f[10]
    elif n == GSP_INFO_SIZE_V2:
        f = struct.unpack("<BBBBHBBIII", payload)
        build_hash = None
    else:
        return {"error": f"unexpected INFO size {n}B (want 20 or 24)"}
    return {
        "fw": f"{f[1]}.{f[2]}.{f[3]}",
        "profileId": f[5],
        "profile": MOTOR_PROFILES.get(f[5], f"unknown({f[5]})"),
        "polePairs": f[6],
        "maxErpm": f[9],
        "buildHash": build_hash,
    }


def main():
    ap = argparse.ArgumentParser(description="Read maxClosedLoopErpm over GSP")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--expect", type=int, default=250000,
                    help="sentinel value to check against (default 250000)")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"ERROR: cannot open {args.port}: {e}")
        sys.exit(1)

    # kill any stale telemetry stream, flush, then ask for info
    ser.write(build_packet(GSP_CMD_TELEM_STOP))
    time.sleep(0.2)
    ser.reset_input_buffer()

    ser.write(build_packet(GSP_CMD_GET_INFO))
    resp = read_response(ser)
    if resp is None or resp[0] != GSP_CMD_GET_INFO:
        print("FAIL: no valid GET_INFO response (check port/baud/board powered)")
        sys.exit(2)

    info = decode_info(resp[1])
    if "error" in info:
        print(f"FAIL: {info['error']}")
        sys.exit(2)

    bh = info.get("buildHash")
    bh_str = f"  buildHash=0x{bh:08X}" if bh is not None else ""
    print(f"  fw={info['fw']}  profile={info['profile']} (id {info['profileId']})  "
          f"polePairs={info['polePairs']}{bh_str}")
    print(f"  maxErpm = {info['maxErpm']}")
    if info["maxErpm"] == args.expect:
        print(f"  PASS — code edit took effect (== sentinel {args.expect}); "
              f"compiled defaults beat stale EEPROM.")
        sys.exit(0)
    else:
        print(f"  MISMATCH — expected {args.expect}, got {info['maxErpm']}. "
              f"EEPROM may still be overriding, or board not reflashed.")
        sys.exit(3)


if __name__ == "__main__":
    main()
