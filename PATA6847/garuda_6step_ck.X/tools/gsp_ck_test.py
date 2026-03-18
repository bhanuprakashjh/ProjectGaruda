#!/usr/bin/env python3
"""
GSP CK Board Test — verifies PING, GET_INFO, GET_SNAPSHOT on dsPIC33CK.

Usage:
    python3 gsp_ck_test.py                    # auto-detect port
    python3 gsp_ck_test.py -p /dev/ttyACM0    # specific port

Requires: pip install pyserial
"""

import argparse
import struct
import sys
import time
from typing import Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

# ── GSP Protocol ────────────────────────────────────────────────────────

GSP_START = 0x02
CRC_INIT = 0xFFFF
CRC_POLY = 0x1021

CMD_PING         = 0x00
CMD_GET_INFO     = 0x01
CMD_GET_SNAPSHOT = 0x02
CMD_START_MOTOR  = 0x03
CMD_STOP_MOTOR   = 0x04
CMD_HEARTBEAT    = 0x08
CMD_TELEM_START  = 0x14
CMD_TELEM_STOP   = 0x15
CMD_TELEM_FRAME  = 0x80
CMD_ERROR        = 0xFF

BOARD_IDS = {0x0001: "MCLV-48V-300W (AK)", 0x0002: "EV43F54A (CK)"}


def crc16(data: bytes) -> int:
    crc = CRC_INIT
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ CRC_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_packet(cmd_id: int, payload: bytes = b"") -> bytes:
    pkt_len = 1 + len(payload)
    data = bytes([pkt_len, cmd_id]) + payload
    crc = crc16(data)
    return bytes([GSP_START]) + data + struct.pack(">H", crc)


def parse_response(ser, timeout=1.0) -> Optional[tuple]:
    """Read one GSP response. Returns (cmd_id, payload) or None."""
    deadline = time.monotonic() + timeout
    state = "WAIT_START"
    pkt_buf = bytearray()
    pkt_len = 0
    crc_buf = bytearray()

    while time.monotonic() < deadline:
        if ser.in_waiting == 0:
            time.sleep(0.001)
            continue

        b = ser.read(1)[0]

        if state == "WAIT_START":
            if b == GSP_START:
                state = "GOT_START"
        elif state == "GOT_START":
            if 1 <= b <= 249:
                pkt_len = b
                pkt_buf = bytearray()
                crc_buf = bytearray()
                state = "COLLECTING"
            else:
                state = "WAIT_START"
        elif state == "COLLECTING":
            if len(pkt_buf) < pkt_len:
                pkt_buf.append(b)
            else:
                crc_buf.append(b)
                if len(crc_buf) == 2:
                    # Verify CRC
                    expected = crc16(bytes([pkt_len]) + bytes(pkt_buf))
                    actual = (crc_buf[0] << 8) | crc_buf[1]
                    if expected == actual:
                        cmd_id = pkt_buf[0]
                        payload = bytes(pkt_buf[1:])
                        return (cmd_id, payload)
                    else:
                        print(f"  CRC FAIL: expected {expected:04X}, got {actual:04X}")
                        return None

    return None


def send_and_recv(ser, cmd_id: int, payload: bytes = b"",
                  label: str = "") -> Optional[tuple]:
    pkt = build_packet(cmd_id, payload)
    ser.write(pkt)
    resp = parse_response(ser)
    if resp is None:
        print(f"  [{label}] TIMEOUT — no response")
    return resp


# ── Test Functions ──────────────────────────────────────────────────────

def test_ping(ser) -> bool:
    print("\n=== TEST: PING ===")
    resp = send_and_recv(ser, CMD_PING, label="PING")
    if resp and resp[0] == CMD_PING:
        print("  PING OK — board is alive")
        return True
    print("  PING FAILED")
    return False


def test_get_info(ser) -> bool:
    print("\n=== TEST: GET_INFO ===")
    resp = send_and_recv(ser, CMD_GET_INFO, label="GET_INFO")
    if resp is None or resp[0] == CMD_ERROR:
        print("  GET_INFO FAILED")
        return False

    cmd_id, data = resp
    if len(data) < 20:
        print(f"  GET_INFO: unexpected payload length {len(data)} (expected 20)")
        return False

    proto, fw_maj, fw_min, fw_patch = data[0], data[1], data[2], data[3]
    board_id = struct.unpack_from("<H", data, 4)[0]
    motor_profile = data[6]
    pole_pairs = data[7]
    features = struct.unpack_from("<I", data, 8)[0]
    pwm_freq = struct.unpack_from("<I", data, 12)[0]
    max_erpm = struct.unpack_from("<I", data, 16)[0]

    board_name = BOARD_IDS.get(board_id, f"Unknown (0x{board_id:04X})")

    print(f"  Protocol:     v{proto}")
    print(f"  Firmware:     v{fw_maj}.{fw_min}.{fw_patch}")
    print(f"  Board:        {board_name}")
    print(f"  Motor Profile: {motor_profile}")
    print(f"  Pole Pairs:   {pole_pairs}")
    print(f"  Features:     0x{features:08X}")
    print(f"  PWM Freq:     {pwm_freq} Hz")
    print(f"  Max eRPM:     {max_erpm}")

    # Decode feature flags
    feat_names = {
        0: "IC_ZC", 1: "VBUS_FAULT", 2: "DESYNC_REC", 3: "DUTY_SLEW",
        4: "TIM_ADVANCE", 16: "GSP", 25: "ATA6847", 26: "ILIM_HW",
        27: "CURRENT_SNS",
    }
    active = [feat_names.get(i, f"bit{i}") for i in range(32)
              if features & (1 << i)]
    print(f"  Features:     {', '.join(active)}")

    if board_id == 0x0002:
        print("  *** CK BOARD DETECTED ***")
        return True
    else:
        print(f"  WARNING: Expected CK board (0x0002), got 0x{board_id:04X}")
        return True


def test_get_snapshot(ser) -> bool:
    print("\n=== TEST: GET_SNAPSHOT ===")
    resp = send_and_recv(ser, CMD_GET_SNAPSHOT, label="GET_SNAPSHOT")
    if resp is None or resp[0] == CMD_ERROR:
        print("  GET_SNAPSHOT FAILED")
        return False

    cmd_id, data = resp
    print(f"  Snapshot size: {len(data)} bytes")

    if len(data) >= 48:
        # Decode CK snapshot (48 bytes)
        state = data[0]
        fault = data[1]
        step = data[2]
        ata_status = data[3]
        pot_raw = struct.unpack_from("<H", data, 4)[0]
        duty_pct = data[6]
        zc_synced = data[7]
        vbus_raw = struct.unpack_from("<H", data, 8)[0]
        ia_raw = struct.unpack_from("<h", data, 10)[0]
        ib_raw = struct.unpack_from("<h", data, 12)[0]
        ibus_raw = struct.unpack_from("<h", data, 14)[0]
        duty = struct.unpack_from("<H", data, 16)[0]
        step_period = struct.unpack_from("<H", data, 20)[0]
        step_period_hr = struct.unpack_from("<H", data, 22)[0]
        erpm = struct.unpack_from("<H", data, 24)[0]
        good_zc = struct.unpack_from("<H", data, 26)[0]
        zc_interval = struct.unpack_from("<H", data, 28)[0]
        ic_accepted = struct.unpack_from("<H", data, 32)[0]
        ic_false = struct.unpack_from("<H", data, 34)[0]
        filter_level = data[36]
        missed = data[37]
        forced = data[38]
        ilim = data[39]
        sys_tick = struct.unpack_from("<I", data, 40)[0]
        uptime = struct.unpack_from("<I", data, 44)[0]

        state_names = ["IDLE", "ARMED", "ALIGN", "OL_RAMP", "CL", "RECOV", "FAULT"]
        state_str = state_names[state] if state < len(state_names) else f"?{state}"

        print(f"  State:        {state_str}")
        print(f"  Fault:        {fault}")
        print(f"  Step:         {step}")
        print(f"  Pot:          {pot_raw}")
        print(f"  Duty:         {duty} ({duty_pct}%)")
        print(f"  Vbus:         {vbus_raw}")
        print(f"  Ia:           {ia_raw} ({ia_raw * 1.049:.0f} mA)")
        print(f"  Ib:           {ib_raw} ({ib_raw * 1.049:.0f} mA)")
        print(f"  IBus:         {ibus_raw} ({ibus_raw * 1.049:.0f} mA)")
        print(f"  Step Period:  {step_period} (HR: {step_period_hr})")
        print(f"  eRPM:         {erpm}")
        print(f"  ZC Synced:    {'SYN' if zc_synced else '---'}")
        print(f"  Good ZC:      {good_zc}")
        print(f"  ZC Interval:  {zc_interval}")
        print(f"  IC Accepted:  {ic_accepted}")
        print(f"  IC False:     {ic_false}")
        print(f"  Filter Level: {filter_level}")
        print(f"  Missed/Forced:{missed}/{forced}")
        print(f"  ILIM Active:  {'YES' if ilim else 'no'}")
        print(f"  ATA Status:   0x{ata_status:02X}")
        print(f"  Uptime:       {uptime}s (tick: {sys_tick})")
        return True
    else:
        print(f"  Unexpected snapshot size: {len(data)} (expected 48)")
        return False


def test_telemetry_stream(ser, duration=3) -> bool:
    print(f"\n=== TEST: TELEMETRY STREAM ({duration}s) ===")

    # Start at 20Hz
    resp = send_and_recv(ser, CMD_TELEM_START, bytes([20]), label="TELEM_START")
    if resp is None:
        print("  TELEM_START FAILED")
        return False
    print(f"  Streaming at {resp[1][0]}Hz...")

    frames = 0
    start = time.monotonic()
    while time.monotonic() - start < duration:
        r = parse_response(ser, timeout=0.2)
        if r and r[0] == CMD_TELEM_FRAME:
            frames += 1
            data = r[1]
            seq = struct.unpack_from("<H", data, 0)[0]
            # CK snapshot starts at offset 2
            if len(data) >= 50:
                state = data[2]
                erpm = struct.unpack_from("<H", data, 26)[0]
                duty_pct = data[8]
                ibus = struct.unpack_from("<h", data, 16)[0]
                state_names = ["IDLE", "ARMED", "ALIGN", "OL_RAMP", "CL", "RECOV", "FAULT"]
                s = state_names[state] if state < len(state_names) else "?"
                print(f"  #{seq:4d}  {s:7s}  eRPM:{erpm:<6}  D:{duty_pct}%  "
                      f"IBus:{ibus * 1.049:.0f}mA", end="\r")

    # Stop telemetry
    send_and_recv(ser, CMD_TELEM_STOP, label="TELEM_STOP")

    elapsed = time.monotonic() - start
    rate = frames / elapsed if elapsed > 0 else 0
    print(f"\n  Received {frames} frames in {elapsed:.1f}s ({rate:.1f} Hz)")
    return frames > 0


# ── Main ────────────────────────────────────────────────────────────────

def find_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(kw in desc for kw in ("usb", "uart", "serial", "pkob", "cdc")):
            return p.device
        if "04d8" in hwid or "0403" in hwid:
            return p.device
    return None


def main():
    parser = argparse.ArgumentParser(description="GSP CK Board Test")
    parser.add_argument("-p", "--port", help="Serial port")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    args = parser.parse_args()

    port = args.port or find_port()
    if not port:
        print("No serial port found. Specify with -p /dev/ttyACM0")
        sys.exit(1)

    print(f"Port: {port} @ {args.baud}")

    ser = serial.Serial(port, args.baud, timeout=0.5)
    time.sleep(0.3)
    ser.reset_input_buffer()

    # Stop any active telemetry stream from a previous session
    ser.write(build_packet(CMD_TELEM_STOP))
    time.sleep(0.3)
    ser.reset_input_buffer()

    passed = 0
    failed = 0

    for test_fn in [test_ping, test_get_info, test_get_snapshot,
                    test_telemetry_stream]:
        try:
            if test_fn(ser):
                passed += 1
            else:
                failed += 1
        except Exception as e:
            print(f"  EXCEPTION: {e}")
            failed += 1

    ser.close()
    print(f"\n{'=' * 40}")
    print(f"Results: {passed} passed, {failed} failed")
    if failed == 0:
        print("ALL TESTS PASSED")
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
