#!/usr/bin/env python3
"""Quick GSP PING test on both ACM ports — also checks for text output."""
import serial, time, sys

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def build_ping():
    body = bytes([0x01, 0x00])  # LEN=1, CMD=PING(0x00)
    c = crc16(body)
    return bytes([0x02]) + body + bytes([c >> 8, c & 0xFF])

ping_pkt = build_ping()
print(f'PING packet: {ping_pkt.hex()}')

for port in ['/dev/ttyACM0', '/dev/ttyACM1']:
    try:
        s = serial.Serial(port, 115200, timeout=2)
        time.sleep(1.0)
        # Read any unsolicited data first (debug text or telem)
        pre = s.read(s.in_waiting or 1)
        if pre:
            print(f'{port}: Pre-read {len(pre)} bytes:')
            try:
                print(f'  Text: {pre.decode("ascii", errors="replace")[:200]}')
            except:
                pass
            print(f'  Hex:  {pre[:64].hex()}')
        else:
            print(f'{port}: No unsolicited data')

        # Send PING with correct CRC
        s.reset_input_buffer()
        s.write(ping_pkt)
        time.sleep(0.5)
        data = s.read(100)
        print(f'{port}: PING response: {len(data)} bytes: {data.hex() if data else "(empty)"}')
        s.close()
    except Exception as e:
        print(f'{port}: {e}')
    print()
