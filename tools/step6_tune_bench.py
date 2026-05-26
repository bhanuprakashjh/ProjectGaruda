#!/usr/bin/env python3
"""
Quick bench-test tuning knobs for AKESC 6-step/HWZC.

Applies settings that help bare A2212 on the bench:
  rampAccelErpmPerS   300 → 1500   (faster sine/trap ramp)
  ocLimitMa          12000 → 18000 (CMP3 operational threshold)
  ocFaultMa          18000 → 22000 (SW fault threshold keeps margin above CMP3)

These raise OC tolerance against commutation-transient current spikes at
high eRPM (~80k+). The real bench current stays low — CMP3 trips on the
peak of a mis-commutation, not average. Raising the peak trip bumps
headroom without losing board protection.

Motor MUST be at IDLE. Run from repo root.

Usage:
    python3 tools/step6_tune_bench.py [--port /dev/ttyACM0]
    python3 tools/step6_tune_bench.py --revert   # back to defaults
"""
import argparse, struct, sys, time
import serial

GSP_START        = 0x02
CMD_PING         = 0x00
CMD_GET_SNAPSHOT = 0x02
CMD_STOP_MOTOR   = 0x04
CMD_CLEAR_FAULT  = 0x05
CMD_GET_PARAM    = 0x10
CMD_SET_PARAM    = 0x11
CMD_SAVE_CONFIG  = 0x12
CMD_ERROR        = 0xFF

ESC_STATE_NAMES = {0:"IDLE",1:"ARMED",2:"DETECT",3:"ALIGN",4:"OL_RAMP",5:"MORPH",6:"CL",7:"BRAKING",8:"RECOVERY",9:"FAULT"}

# (id, name, bench_value, default_value)
TUNES = [
    (0x16, "rampAccelErpmPerS",  5000,   300),
    (0x58, "ocLimitMa",          18000, 12000),
    (0x41, "ocFaultMa",          22000, 18000),
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
    r = send(ser, CMD_SET_PARAM, struct.pack("<HI", pid, value))
    if not r: return False, "no response"
    if r[0] == CMD_ERROR:
        err = r[1][0] if r[1] else 0
        names = {4:"UNKNOWN_PARAM",5:"OUT_OF_RANGE",6:"WRONG_STATE",7:"CROSS_VALIDATION"}
        return False, names.get(err, f"err={err}")
    return True, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()

    print(f"Connecting to {args.port}...")
    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    time.sleep(0.3); ser.reset_input_buffer()
    if not send(ser, CMD_PING):
        print("PING failed"); return 1
    print("PING OK")

    print("Forcing IDLE...")
    send(ser, CMD_STOP_MOTOR); time.sleep(0.1)
    send(ser, CMD_CLEAR_FAULT); time.sleep(0.1)
    r = send(ser, CMD_GET_SNAPSHOT)
    if r and r[0] == CMD_GET_SNAPSHOT and len(r[1]) >= 1:
        st = r[1][0]
        print(f"  state = {ESC_STATE_NAMES.get(st, f'?{st}')}")
        if st != 0:
            print("  ERROR: not IDLE. Power-cycle and retry.")
            ser.close(); return 1

    mode = "default (revert)" if args.revert else "bench (fast ramp + OC headroom)"
    print(f"\nApplying {mode} settings:")
    print(f"  {'id':>4}  {'name':>22}  {'old':>6}  {'new':>6}")
    ok = True
    for pid, name, bench, defv in TUNES:
        old = get_param(ser, pid)
        new = defv if args.revert else bench
        if old == new:
            print(f"  {pid:#04x}  {name:>22}  {old:>6}  (no change)")
            continue
        success, err = set_param(ser, pid, new)
        if success:
            after = get_param(ser, pid)
            mark = "✓" if after == new else "?"
            print(f"  {pid:#04x}  {name:>22}  {old:>6} → {after:>6}  {mark}")
        else:
            print(f"  {pid:#04x}  {name:>22}  {old:>6} FAIL ({err})")
            ok = False

    if ok:
        print("\nSaving to EEPROM...")
        r = send(ser, CMD_SAVE_CONFIG)
        if r and r[0] != CMD_ERROR:
            print("  Saved ✓")
        else:
            print("  SAVE failed")

    ser.close()
    print("\nPower-cycle the board for CMP3/derived values to fully reload, then test.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
