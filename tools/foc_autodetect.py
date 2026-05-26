#!/usr/bin/env python3
"""
Trigger FOC V2 motor auto-detection (Rs / Ls / λ_pm / PI tune).

Motor must be bench-stationary with no prop (rotor will twitch during
measurement). Board in ESC_IDLE. No throttle.

Usage:
    python3 tools/foc_autodetect.py --port /dev/ttyACM0

The firmware runs DETECT_R → DETECT_L → DETECT_ALIGN → DETECT_LAMBDA → DETECT_TUNE,
then writes measured values into foc->Rs / foc->Ls / foc->lambda_pm and
recomputes PI gains (pole cancellation at 1 kHz). Values are not persisted —
they live only for this power cycle.

Script streams state and final values through GSP telemetry.
"""
import argparse
import struct
import sys
import time

import serial

GSP_START           = 0x02
CMD_PING            = 0x00
CMD_GET_INFO        = 0x01
CMD_TELEM_START     = 0x14
CMD_TELEM_STOP      = 0x15
CMD_AUTO_DETECT     = 0x20
CMD_TELEM_FRAME     = 0x80

STATE_NAMES = {
    0: "IDLE", 1: "ARMED", 2: "DETECT", 3: "ALIGN", 4: "OL_RAMP",
    5: "CL", 6: "FAULT"
}

# FOC snapshot offsets (v2 subset — see garuda_service.c FOC snapshot build)
OFF_STATE     =   0
OFF_FAULT     =   1
OFF_VBUS      =  40   # focVbus float
OFF_LAMBDA    =  48   # focLambdaEst float
OFF_IA        =  41   # focIa — adjusts later with sizes; use pattern below
# We'll parse by field names known to work with foc_logger.py


def crc16(data, init=0xFFFF):
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def build(cmd, payload=b""):
    body = bytes([1 + len(payload), cmd]) + payload
    return bytes([GSP_START]) + body + struct.pack(">H", crc16(body))


def send(ser, cmd, payload=b""):
    ser.write(build(cmd, payload))


def parse_next(buf):
    while True:
        idx = buf.find(bytes([GSP_START]))
        if idx < 0:
            return None, None, b""
        buf = buf[idx:]
        if len(buf) < 5:
            return None, None, buf
        pkt_len = buf[1]
        total = 2 + pkt_len + 2
        if len(buf) < total:
            return None, None, buf
        cmd = buf[2]
        payload = bytes(buf[3:2 + pkt_len])
        rx_crc = struct.unpack(">H", buf[2 + pkt_len:total])[0]
        if crc16(buf[1:2 + pkt_len]) != rx_crc:
            buf = buf[1:]
            continue
        return cmd, payload, buf[total:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.2)
    ser.reset_input_buffer()

    # Force telem off & drain
    send(ser, CMD_TELEM_STOP)
    time.sleep(0.3)
    ser.read(4096)

    # Sanity ping
    send(ser, CMD_PING)
    time.sleep(0.1)
    ser.read(4096)

    print("Requesting AUTO_DETECT...")
    send(ser, CMD_AUTO_DETECT)
    time.sleep(0.2)
    ser.read(4096)

    # Start telemetry to watch state
    send(ser, CMD_TELEM_START, struct.pack("<H", 50))
    time.sleep(0.2)

    print(f"{'time':>7}  {'state':>8}  {'sub':>3}  {'Rs(Ω)':>8}  "
          f"{'Ls(µH)':>8}  {'λpm':>9}  Ia(A)  Ib(A)  Vbus")
    print("-" * 90)

    rx = b""
    t0 = time.monotonic()
    last_state = None
    done = False
    final = None
    deadline = t0 + 30.0

    while time.monotonic() < deadline and not done:
        rx += ser.read(512)
        while True:
            cmd, payload, rx = parse_next(rx)
            if cmd is None:
                break
            if cmd != CMD_TELEM_FRAME or len(payload) < 60:
                continue
            d = payload[2:]  # strip 2-byte seq
            state = d[0]
            fault = d[1]
            # Key FOC fields (from foc_logger.py positions):
            focVbus   = struct.unpack_from("<f", d, 40)[0]
            focIa     = struct.unpack_from("<f", d, 44)[0]
            focIb     = struct.unpack_from("<f", d, 48)[0]
            focLambda = struct.unpack_from("<f", d, 60)[0]
            focRs     = struct.unpack_from("<f", d, 96)[0] if len(d) >= 100 else 0.0
            focLs     = struct.unpack_from("<f", d, 100)[0] if len(d) >= 104 else 0.0
            # Note: exact offsets for detect_Rs / detect_Ls depend on snapshot
            # layout. Fall back to displaying state + basic signals; the
            # firmware's foc_detect_apply writes the values into foc->Rs,
            # foc->Ls, foc->lambda_pm at end of sequence. Those may or may
            # not be in the telemetry frame.
            focSubState = d[68] if len(d) > 68 else 0

            state_name = STATE_NAMES.get(state, f"?{state}")
            t = time.monotonic() - t0
            print(f"{t:7.2f}  {state_name:>8}  {focSubState:>3}  "
                  f"{focRs:8.4f}  {focLs*1e6:8.2f}  {focLambda:9.6f}  "
                  f"{focIa:5.2f}  {focIb:5.2f}  {focVbus:.2f}")

            last_state = state
            if fault != 0:
                print(f"*** FAULT {fault}")
                done = True
                break
            if state == 0 and last_state == 2:   # DETECT -> IDLE = completed
                done = True
                final = (focRs, focLs, focLambda)
                break

    send(ser, CMD_TELEM_STOP)
    time.sleep(0.1)
    ser.close()

    if final:
        Rs, Ls, lam = final
        print()
        print("=== MEASURED ===")
        print(f"  Rs       = {Rs:.4f} Ω   ({Rs*1000:.1f} mΩ phase-to-neutral)")
        print(f"  Ls       = {Ls*1e6:.2f} µH")
        print(f"  λ_pm     = {lam:.6f} V·s/rad")
        if lam > 0:
            kv = 60.0 / (1.732 * 6.2832 * lam * 7)
            print(f"  ⇒ ~KV   = {kv:.0f} RPM/V (7PP assumed)")
        print()
        print("Drop these into dspic33AKESC/garuda_foc_params.h Profile 2:")
        print(f"  #define MOTOR_RS_OHM       {Rs:.4f}f")
        print(f"  #define MOTOR_LS_H         {Ls:.3e}f")
        print(f"  #define MOTOR_KE_VPEAK     {lam:.6f}f")
    else:
        print("Detection did not complete within 30 s timeout.")


if __name__ == "__main__":
    sys.exit(main())
