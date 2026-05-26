#!/usr/bin/env python3
"""Reset EEPROM runtime params to compile-time defaults, then save.
Sends GSP LOAD_DEFAULTS (0x13) and SAVE_CONFIG (0x12)."""
import serial
import struct
import sys
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM2"

def crc16(data, init=0xFFFF):
    c = init
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) if (c & 0x8000) else (c << 1)
            c &= 0xFFFF
    return c

def build(cmd, payload=b""):
    body = bytes([1 + len(payload), cmd]) + payload
    return bytes([0x02]) + body + struct.pack(">H", crc16(body))

def send(ser, cmd, label):
    ser.write(build(cmd))
    time.sleep(0.2)
    resp = ser.read(256)
    print(f"{label:<18}  → {resp.hex() if resp else '(no response)'}")

ser = serial.Serial(PORT, 115200, timeout=0.5)
time.sleep(0.2)
ser.reset_input_buffer()

# PING sanity
send(ser, 0x00, "PING")

# LOAD_DEFAULTS — pulls compile-time defaults into RAM gspParams
send(ser, 0x13, "LOAD_DEFAULTS")

# SAVE_CONFIG — writes RAM gspParams into EEPROM so it persists
send(ser, 0x12, "SAVE_CONFIG")

ser.close()
print()
print("Done. Power-cycle the board to be safe, then re-run the test.")
