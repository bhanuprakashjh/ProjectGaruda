#!/usr/bin/env python3
"""
Load the A2212 profile defaults and save to EEPROM, then read back
the key 6-step ramp params so we can verify they match the compile-time
values in garuda_config.h / gsp_params.c.

Fixes the common "EEPROM has stale FOC-era params, 6-step ramp misbehaves"
problem without reflashing.

Usage:
    python3 tools/step6_reset_profile.py [--port /dev/ttyACM0] [--profile 1]

Profile IDs: 0=Hurst, 1=A2212 (default), 2=5010, 3=5055
"""

import argparse
import struct
import sys
import time

import serial

GSP_START = 0x02
CMD_PING          = 0x00
CMD_GET_PARAM     = 0x10
CMD_SET_PARAM     = 0x11
CMD_LOAD_DEFAULTS = 0x13
CMD_SAVE_CONFIG   = 0x12
CMD_LOAD_PROFILE  = 0x17
CMD_ERROR         = 0xFF

# Key 6-step params to verify (id, name, expected_for_profile_2_2810, units)
# Defaults below match AKESC's GSP_PROFILE_5010 slot which now holds
# 2810 1350KV (24V bench motor). For A2212 (12V), use --profile 1.
# Note: rampDutyPct and alignDutyPct differ slightly between A2212 (15/8) and
# 2810 (8/3) — current expected values target the 2810 profile.
PARAMS_TO_CHECK = [
    (0x15, "rampTargetErpm",     3000, "eRPM"),
    (0x16, "rampAccelErpmPerS",  3000, "eRPM/s"),
    (0x17, "rampDutyPct",           8, "%"),
    (0x22, "timingAdvMaxDeg",      25, "deg"),
    (0x51, "alignDutyPct",          3, "%"),
    (0x52, "initialErpm",         150, "eRPM"),
    (0x54, "sineAlignModPct",       3, "%"),
    (0x55, "sineRampModPct",        5, "%"),
]


def crc16(data, init=0xFFFF):
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def build(cmd, payload=b""):
    pkt_len = 1 + len(payload)
    body = bytes([pkt_len, cmd]) + payload
    c = crc16(body)
    return bytes([GSP_START]) + body + struct.pack(">H", c)


def read_resp(ser, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] == GSP_START:
            break
    else:
        return None
    ser.timeout = 0.3
    b = ser.read(1)
    if not b:
        return None
    pkt_len = b[0]
    data = ser.read(pkt_len + 2)
    if len(data) < pkt_len + 2:
        return None
    cmd = data[0]
    payload = data[1:pkt_len]
    rx_crc = struct.unpack(">H", data[pkt_len:pkt_len + 2])[0]
    if crc16(bytes([pkt_len]) + data[:pkt_len]) != rx_crc:
        return None
    return (cmd, payload)


def send(ser, cmd, payload=b"", timeout=1.0):
    ser.write(build(cmd, payload))
    return read_resp(ser, timeout)


def get_param(ser, param_id):
    resp = send(ser, CMD_GET_PARAM, struct.pack("<H", param_id))
    if not resp or resp[0] != CMD_GET_PARAM or len(resp[1]) < 6:
        return None
    _, value = struct.unpack("<HI", resp[1][:6])
    return value


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--profile", type=int, default=1,
                    help="Profile ID (0=Hurst, 1=A2212, 2=5010, 3=5055)")
    ap.add_argument("--no-reset", action="store_true",
                    help="Only read params, don't reload profile")
    args = ap.parse_args()

    print(f"Connecting to {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    time.sleep(0.3)
    ser.reset_input_buffer()

    if not send(ser, CMD_PING):
        print("PING failed."); return 1
    print("PING OK")

    # ── Read current values ───────────────────────────────────
    print("\nCurrent values (from EEPROM + derived):")
    print(f"  {'id':>4}  {'name':>24}  {'value':>10}  {'expected':>10}  {'units':>6}")
    print("  " + "─" * 64)
    stale = False
    for pid, name, expected, units in PARAMS_TO_CHECK:
        v = get_param(ser, pid)
        if v is None:
            print(f"  {pid:#04x}  {name:>24}  {'FAIL':>10}")
            continue
        mark = "" if v == expected else "  ← MISMATCH"
        if v != expected:
            stale = True
        print(f"  {pid:#04x}  {name:>24}  {v:>10}  {expected:>10}  {units:>6}{mark}")

    if args.no_reset:
        ser.close()
        return 0

    if not stale:
        print("\nAll params match A2212 defaults — no reset needed.")
        ser.close()
        return 0

    # ── Reload profile ────────────────────────────────────────
    print(f"\nLoading profile {args.profile}...")
    resp = send(ser, CMD_LOAD_PROFILE, bytes([args.profile]))
    if not resp or resp[0] == CMD_ERROR:
        print(f"  LOAD_PROFILE failed: {resp}")
        ser.close()
        return 1
    print("  LOAD_PROFILE OK (auto-saved to EEPROM)")

    # ── Read back ─────────────────────────────────────────────
    print("\nValues after profile reload:")
    print(f"  {'id':>4}  {'name':>24}  {'value':>10}  {'expected':>10}  {'units':>6}")
    print("  " + "─" * 64)
    for pid, name, expected, units in PARAMS_TO_CHECK:
        v = get_param(ser, pid)
        if v is None:
            print(f"  {pid:#04x}  {name:>24}  {'FAIL':>10}")
            continue
        mark = "" if v == expected else "  ← STILL WRONG"
        print(f"  {pid:#04x}  {name:>24}  {v:>10}  {expected:>10}  {units:>6}{mark}")

    print("\nDone. Power-cycle the board (or reset) to ensure clean init, then rerun step6_logger.")
    ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
