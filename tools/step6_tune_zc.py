#!/usr/bin/env python3
"""
Tune the 6-step ZC detection filters for low-speed noise immunity.

Current defaults are tuned for high-speed tight detection (short blanking,
narrow deadband, low filter count). At low eRPM (~1500), this lets PWM
switching noise produce phantom ZCs → CL desync.

Bumps to conservative values:
    zcBlankingPercent   3 → 15   (longer blanking after commutation)
    zcAdcDeadband       4 → 10   (wider ignore band around threshold)
    zcFilterThreshold   2 →  3   (require 3 consecutive matching samples)

No rebuild required. Saves to EEPROM.

Usage:
    python3 tools/step6_tune_zc.py [--port /dev/ttyACM0]
    python3 tools/step6_tune_zc.py --revert   # back to defaults (3/4/2)
"""
import argparse, struct, sys, time
import serial

GSP_START = 0x02
CMD_PING          = 0x00
CMD_GET_SNAPSHOT  = 0x02
CMD_STOP_MOTOR    = 0x04
CMD_CLEAR_FAULT   = 0x05
CMD_GET_PARAM     = 0x10
CMD_SET_PARAM     = 0x11
CMD_SAVE_CONFIG   = 0x12
CMD_ERROR         = 0xFF

ESC_STATE_NAMES = {
    0: "IDLE", 1: "ARMED", 2: "DETECT", 3: "ALIGN", 4: "OL_RAMP",
    5: "MORPH", 6: "CL", 7: "BRAKING", 8: "RECOVERY", 9: "FAULT"
}

# (id, name, conservative_value, aggressive_value)
TUNES = [
    (0x64, "zcBlankingPercent",  15,  3),
    (0x65, "zcAdcDeadband",      10,  4),
    (0x67, "zcFilterThreshold",   3,  2),
]


def crc16(data, init=0xFFFF):
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def build(cmd, payload=b""):
    pkt_len = 1 + len(payload)
    body = bytes([pkt_len, cmd]) + payload
    return bytes([GSP_START]) + body + struct.pack(">H", crc16(body))


def read_resp(ser, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b: continue
        if b[0] == GSP_START: break
    else:
        return None
    ser.timeout = 0.3
    b = ser.read(1)
    if not b: return None
    pkt_len = b[0]
    data = ser.read(pkt_len + 2)
    if len(data) < pkt_len + 2: return None
    cmd = data[0]; payload = data[1:pkt_len]
    rx_crc = struct.unpack(">H", data[pkt_len:pkt_len+2])[0]
    if crc16(bytes([pkt_len]) + data[:pkt_len]) != rx_crc: return None
    return (cmd, payload)


def send(ser, cmd, payload=b"", timeout=1.0):
    ser.write(build(cmd, payload))
    return read_resp(ser, timeout)


def get_param(ser, pid):
    r = send(ser, CMD_GET_PARAM, struct.pack("<H", pid))
    if not r or r[0] != CMD_GET_PARAM or len(r[1]) < 6: return None
    _, v = struct.unpack("<HI", r[1][:6])
    return v


def set_param(ser, pid, value):
    payload = struct.pack("<HI", pid, value)
    r = send(ser, CMD_SET_PARAM, payload)
    if not r:
        return False, "no response"
    if r[0] == CMD_ERROR:
        err = r[1][0] if r[1] else 0
        err_names = {1: "BAD_LENGTH", 2: "BAD_CRC", 3: "UNKNOWN_CMD",
                     4: "UNKNOWN_PARAM", 5: "OUT_OF_RANGE",
                     6: "WRONG_STATE", 7: "CROSS_VALIDATION"}
        return False, err_names.get(err, f"err={err}")
    return True, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--revert", action="store_true",
                    help="Set back to aggressive defaults instead of conservative")
    args = ap.parse_args()

    print(f"Connecting to {args.port}...")
    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    time.sleep(0.3); ser.reset_input_buffer()
    if not send(ser, CMD_PING):
        print("PING failed"); return 1
    print("PING OK")

    # Force to IDLE: stop motor, clear fault
    print("Ensuring IDLE state...")
    send(ser, CMD_STOP_MOTOR)
    time.sleep(0.1)
    send(ser, CMD_CLEAR_FAULT)
    time.sleep(0.1)

    # Read back state from snapshot
    r = send(ser, CMD_GET_SNAPSHOT)
    if r and r[0] == CMD_GET_SNAPSHOT and len(r[1]) >= 1:
        st = r[1][0]
        name = ESC_STATE_NAMES.get(st, f"?{st}")
        print(f"  state = {name} (byte={st})")
        if st != 0:
            print(f"  ERROR: not IDLE. Power-cycle or press stop-button manually.")
            ser.close()
            return 1

    mode = "aggressive (revert)" if args.revert else "conservative (low-speed)"
    print(f"\nApplying {mode} settings:")
    print(f"  {'id':>4}  {'name':>22}  {'old':>4} → {'new':>4}")
    ok = True
    for pid, name, conserv, aggr in TUNES:
        old = get_param(ser, pid)
        new = aggr if args.revert else conserv
        if old == new:
            print(f"  {pid:#04x}  {name:>22}  {old:>4}   (already set)")
            continue
        success, err = set_param(ser, pid, new)
        if success:
            after = get_param(ser, pid)
            mark = "✓" if after == new else "?"
            print(f"  {pid:#04x}  {name:>22}  {old:>4} → {after:>4}  {mark}")
        else:
            print(f"  {pid:#04x}  {name:>22}  {old:>4} → FAIL ({err})")
            ok = False

    if ok:
        print("\nSaving to EEPROM...")
        r = send(ser, CMD_SAVE_CONFIG)
        if r and r[0] != CMD_ERROR:
            print("  Saved ✓")
        else:
            print("  SAVE failed")

    ser.close()
    print("\nDone. Power-cycle the board (or reset) for new values to take effect cleanly, then try again.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
