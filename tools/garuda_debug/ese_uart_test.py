#!/usr/bin/env python3
"""GarudaESE UART/GSP bench tester — standalone, pyserial only.

Answers three questions in one run:
  A. Does the board TRANSMIT?   (boot banner watch — power-cycle the board)
  B. Does the board RECEIVE?    (GSP PING/GET_INFO probe, raw reply dump)
  C. Which firmware is flashed? (banner present = post-fix build of
                                 garuda-ese-pristine; silence on power-cycle
                                 = pre-fix build with dead UART TX, reflash!)

Usage:
    python3 ese_uart_test.py                 # autodetect port, 115200
    python3 ese_uart_test.py --port /dev/ttyUSB0
    python3 ese_uart_test.py --skip-banner   # go straight to the probe
"""
import argparse
import glob
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: pip install pyserial")

GSP_START = 0x02
CMD_PING = 0x00
CMD_GET_INFO = 0x01
BANNER_MARK = b"GarudaESE"


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def build_packet(cmd_id: int, payload: bytes = b"") -> bytes:
    body = bytes([1 + len(payload), cmd_id]) + payload
    return bytes([GSP_START]) + body + struct.pack(">H", crc16(body))


def hexdump(data: bytes, prefix: str = "    "):
    if not data:
        print(prefix + "(no bytes)")
        return
    for off in range(0, len(data), 16):
        chunk = data[off:off + 16]
        hx = " ".join(f"{b:02X}" for b in chunk)
        asc = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"{prefix}{off:04X}  {hx:<47}  |{asc}|")


def pick_port(explicit):
    if explicit:
        return explicit
    # ttyUSB* first: USB-serial converters (FTDI/CP210x/CH340) enumerate there.
    # ttyACM* is usually a debug probe (PICkit/SNAP present as CDC) — last resort.
    cands = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
    if not cands:
        sys.exit("no /dev/ttyUSB* or /dev/ttyACM* found — is the adapter plugged in?")
    if len(cands) > 1:
        print(f"multiple ports found, using {cands[0]} (override with --port): {cands}")
    return cands[0]


def stage_banner(ser, seconds):
    print("\n== STAGE A: boot-banner watch (board TX path) " + "=" * 20)
    print(f">>> POWER-CYCLE THE BOARD NOW — listening {seconds}s at {ser.baudrate} baud <<<")
    ser.reset_input_buffer()
    deadline = time.monotonic() + seconds
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)
            if BANNER_MARK in buf:
                # keep reading briefly to catch the whole line
                time.sleep(0.3)
                buf.extend(ser.read(512))
                break
    print(f"received {len(buf)} bytes:")
    hexdump(bytes(buf))
    if BANNER_MARK in buf:
        line = bytes(buf).split(b"\r\n")
        text = next((l for l in line if BANNER_MARK in l), b"").decode(errors="replace")
        print(f"  -> BANNER FOUND: {text!r}")
        print("  -> board TX path GOOD, post-fix firmware is flashed.")
        return "banner"
    if buf:
        print("  -> bytes received but no banner: WRONG BAUD or noise.")
        print("     (garbage at power-cycle usually means baud mismatch)")
        return "garbage"
    print("  -> SILENCE. Either pre-fix firmware (dead UART TX) is flashed,")
    print("     or board TX is not reaching this adapter (wiring/pin).")
    return "silent"


def stage_probe(ser, tries=3):
    print("\n== STAGE B: GSP probe (board RX path + protocol) " + "=" * 16)
    results = []
    for name, cmd in (("PING", CMD_PING), ("GET_INFO", CMD_GET_INFO)):
        for attempt in range(tries):
            pkt = build_packet(cmd)
            ser.reset_input_buffer()
            ser.write(pkt)
            ser.flush()
            time.sleep(0.25)
            reply = ser.read(512)
            print(f"  {name} attempt {attempt+1}: sent {pkt.hex(' ').upper()}")
            print(f"    reply {len(reply)} bytes:")
            hexdump(reply, prefix="      ")
            if reply:
                ok = parse_reply(reply)
                results.append((name, ok))
                break
        else:
            results.append((name, None))
    return results


def parse_reply(data: bytes):
    i = data.find(bytes([GSP_START]))
    if i < 0 or len(data) < i + 4:
        print("      -> no GSP start byte / too short")
        return False
    ln = data[i + 1]
    body = data[i + 1: i + 2 + ln]
    if len(data) < i + 2 + ln + 2:
        print(f"      -> truncated frame (len byte {ln})")
        return False
    rx_crc = struct.unpack(">H", data[i + 2 + ln: i + 4 + ln])[0]
    calc = crc16(body)
    if rx_crc == calc:
        print(f"      -> VALID GSP frame, cmd=0x{body[1]:02X}, {ln-1} payload bytes. CRC OK.")
        return True
    print(f"      -> frame found but CRC mismatch (got {rx_crc:04X}, calc {calc:04X})")
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--banner-wait", type=float, default=8.0)
    ap.add_argument("--skip-banner", action="store_true")
    args = ap.parse_args()

    port = pick_port(args.port)
    print(f"opening {port} @ {args.baud} (8N1)")
    with serial.Serial(port, args.baud, timeout=0.1, exclusive=True) as ser:
        banner = None if args.skip_banner else stage_banner(ser, args.banner_wait)
        probes = stage_probe(ser)

    print("\n== VERDICT " + "=" * 52)
    got_reply = any(ok for _, ok in probes)
    if got_reply:
        print("  GSP LINK WORKS. If the GUI still fails, it is picking the wrong")
        print("  port or the port is held open elsewhere.")
    elif banner == "banner":
        print("  Board TX proven (banner) but no reply to probes ->")
        print("  host->board RX direction broken: adapter TX wire to J1.2, or")
        print("  TX/RX crossover. Firmware RX path is next suspect only after")
        print("  rechecking that wire with a scope on J1.2 during this test.")
    elif banner == "garbage":
        print("  Baud/framing mismatch. Board transmits but bytes are garbled.")
    elif banner == "silent":
        print("  Nothing from the board at all -> most likely the PRE-FIX build")
        print("  is flashed (dead UART TX by design). REFLASH garuda-ese-pristine")
        print("  (hex of 2026-07-10 or later), power-cycle, run this again.")
    else:
        print("  Probe-only run, no reply. Run again without --skip-banner and")
        print("  power-cycle for the full picture.")


if __name__ == "__main__":
    main()
