#!/usr/bin/env python3
"""Minimal GSP smoke test for the AK port.

Usage:
    python3 gsp_ak_test.py [port] [baud]              # smoke + 5 telem frames
    python3 gsp_ak_test.py mon  [port] [baud]         # live monitor (Ctrl-C to stop)

Frame format (V2):  STX=0x02 | LEN | CMD | PAYLOAD | CRC16-BE (CCITT, init 0xFFFF, poly 0x1021)
CRC covers LEN+CMD+PAYLOAD.
"""
import sys, time, struct, serial

STX = 0x02
CMDS = [
    ("PING",         0x00),
    ("GET_INFO",     0x01),
    ("GET_SNAPSHOT", 0x02),
    ("HEARTBEAT",    0x08),
]

def crc16_ccitt(data: bytes) -> int:
    c = 0xFFFF
    for b in data:
        c ^= (b << 8) & 0xFFFF
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if (c & 0x8000) else (c << 1) & 0xFFFF
    return c

def frame(cmd: int, payload: bytes = b'') -> bytes:
    body = bytes([len(payload) + 1, cmd]) + payload
    return bytes([STX]) + body + struct.pack('>H', crc16_ccitt(body))

def parse_reply(buf: bytes):
    """Return (cmd, payload) or None on bad frame."""
    if len(buf) < 5 or buf[0] != STX:
        return None
    length = buf[1]
    if len(buf) < 2 + length + 2:
        return None
    cmd = buf[2]
    payload = buf[3:2 + length]
    crc_rx = struct.unpack('>H', buf[2 + length:4 + length])[0]
    crc_calc = crc16_ccitt(buf[1:2 + length])
    if crc_rx != crc_calc:
        return ("BAD_CRC", payload)
    return (cmd, payload)

ESC_STATE = {0: "IDLE", 1: "ARM ", 2: "ALN ", 3: "RAMP",
             4: "CL  ", 5: "RCV ", 6: "FLT "}

def _kfmt(n: int) -> str:
    """Compact count formatter — '1.2M', '187k', or raw <10k."""
    if n >= 1_000_000:
        return f"{n/1e6:4.1f}M"
    if n >= 10_000:
        return f"{n//1000:4d}k"
    return f"{n:5d}"

def decode_telem(buf: bytes) -> str:
    """Decode key fields out of a 148-byte V4 telemetry frame payload."""
    if len(buf) < 48:
        return f"short frame ({len(buf)}B)"
    seq      = buf[0] | (buf[1] << 8)
    state    = buf[2]; step = buf[4]
    potRaw   = buf[6]  | (buf[7]  << 8)
    dutyPct  = buf[8]
    cmdEn    = buf[9]                                    # d[7] — commandEnabled
    vbusRaw  = buf[10] | (buf[11] << 8)
    timerPer = buf[20] | (buf[21] << 8)                  # d[18:20] — timerPeriod (HR ticks)
    eRpm     = struct.unpack('<I', buf[24:28])[0]
    sectors  = buf[28] | (buf[29] << 8)
    diagCap  = buf[34] | (buf[35] << 8)                  # d[32:34] — diagCaptures (PI-fed)
    diagPI   = buf[36] | (buf[37] << 8)                  # d[34:36] — diagPiRuns
    stallCnt = buf[38]                                   # d[36] — stallCounter (max 200)
    tick     = struct.unpack('<I', buf[42:46])[0]
    # ADC diag counters live in the V3 tail of the 64-byte snapshot.
    #   snap[48..51]  adcBlankReject  (u32) → buf[50..53]
    #   snap[52..55]  adcStateMismatch(u32) → buf[54..57]
    #   snap[56..59]  adcCaptureSet   (u32) → buf[58..61]
    #   snap[60..63]  adcSetRising    (u32) → buf[62..65]
    blankRej = struct.unpack('<I', buf[50:54])[0] if len(buf) >= 54 else 0
    stateMis = struct.unpack('<I', buf[54:58])[0] if len(buf) >= 58 else 0
    capSet   = struct.unpack('<I', buf[58:62])[0] if len(buf) >= 62 else 0
    capRise  = struct.unpack('<I', buf[62:66])[0] if len(buf) >= 66 else 0
    capFall  = capSet - capRise
    # V5_POST_ZC_ACCEPT shadow — raw "comp == expectedPost" per polarity
    # (no edge gate). High Acc/(Acc+Rej) means BEMF is crossing neutral.
    #   snap[92..95]  postZcRisingAcc   → buf[94..97]
    #   snap[96..99]  postZcRisingRej   → buf[98..101]
    #   snap[100..103]postZcFallingAcc  → buf[102..105]
    #   snap[104..107]postZcFallingRej  → buf[106..109]
    pR_acc = struct.unpack('<I', buf[94:98])[0]  if len(buf) >= 98  else 0
    pR_rej = struct.unpack('<I', buf[98:102])[0] if len(buf) >= 102 else 0
    pF_acc = struct.unpack('<I', buf[102:106])[0] if len(buf) >= 106 else 0
    pF_rej = struct.unpack('<I', buf[106:110])[0] if len(buf) >= 110 else 0
    pR_pct = (100 * pR_acc // (pR_acc + pR_rej)) if (pR_acc + pR_rej) else 0
    pF_pct = (100 * pF_acc // (pF_acc + pF_rej)) if (pF_acc + pF_rej) else 0
    name     = ESC_STATE.get(state, f"S{state}")
    vbusV    = vbusRaw / 100.0   # raw counts → volts (0.01 V/lsb)
    return (f"#{seq:5d} {name} s{step} pot={potRaw:4d} d={dutyPct:3d}% "
            f"V={vbusV:5.2f} RPM={eRpm:7d} Tp={timerPer:5d} "
            f"stl={stallCnt:3d} cE={cmdEn} "
            f"cR={_kfmt(capRise)} cF={_kfmt(capFall)} "
            f"pR={pR_pct:3d}% pF={pF_pct:3d}%")

def read_frames(ser, want_cmd: int, count: int, timeout_s: float = 3.0):
    """Read `count` frames matching want_cmd or until timeout. Yields (cmd, payload)."""
    ser.timeout = 0.05
    end = time.time() + timeout_s
    buf = bytearray()
    got = 0
    while time.time() < end and got < count:
        chunk = ser.read(256)
        if chunk: buf.extend(chunk)
        # Frame scanner: find STX, validate length/CRC, peel one frame
        while len(buf) >= 5:
            if buf[0] != STX:
                buf.pop(0); continue
            length = buf[1]
            need = 2 + length + 2
            if len(buf) < need: break
            cmd = buf[2]
            payload = bytes(buf[3:2 + length])
            crc_rx = struct.unpack('>H', bytes(buf[2 + length:4 + length]))[0]
            crc_calc = crc16_ccitt(bytes(buf[1:2 + length]))
            del buf[:need]
            if crc_rx != crc_calc:
                continue
            if cmd == want_cmd:
                yield (cmd, payload)
                got += 1
                if got >= count: return

def live_monitor(port: str, baud: int):
    """Stream telemetry frames forever; one line per frame; Ctrl-C to stop."""
    print(f"Opening {port} @ {baud}  (live monitor — Ctrl-C to stop)")
    s = serial.Serial(port, baud, timeout=0.05)
    time.sleep(0.2); s.reset_input_buffer()
    s.write(frame(0x14)); s.flush()   # TELEM_START
    # Track pot min/max to help spot whether the wheel reaches the ADC at all
    pot_min, pot_max = 0xFFFF, 0
    try:
        for cmd, payload in read_frames(s, want_cmd=0x80, count=10**9, timeout_s=10**9):
            potRaw = payload[6] | (payload[7] << 8)
            pot_min = min(pot_min, potRaw); pot_max = max(pot_max, potRaw)
            line = decode_telem(payload)
            print(f"{line} pot[{pot_min}..{pot_max}]")
    except KeyboardInterrupt:
        pass
    finally:
        try: s.write(frame(0x15)); s.flush()
        except Exception: pass
        s.close()
        print(f"\nstopped. final pot range: {pot_min}..{pot_max}")

def ata_diag(port: str, baud: int):
    """Read ATA6847L gate-driver diagnostic registers and decode."""
    print(f"Opening {port} @ {baud}")
    s = serial.Serial(port, baud, timeout=0.6)
    time.sleep(0.2); s.reset_input_buffer()
    s.write(frame(0x40)); s.flush()    # GSP_CMD_ATA_DIAG
    time.sleep(0.4)
    rx = s.read(s.in_waiting or 1)
    time.sleep(0.05); rx += s.read(s.in_waiting)
    s.close()

    parsed = parse_reply(rx)
    if not parsed or parsed[0] != 0x40:
        print(f"  no/bad ATA_DIAG response: {rx.hex()}")
        return
    d = parsed[1]
    if len(d) < 8:
        print(f"  short payload: {d.hex()}")
        return
    dsr1, dsr2, sir1, sir2, sir3, sir4, sir5, gopm = d[:8]
    last_dsr1 = d[8] if len(d) >= 9 else None
    last_attempts = (d[9] | (d[10] << 8)) if len(d) >= 11 else None
    last_result   = d[11] if len(d) >= 12 else None

    print(f"  raw: DSR1={dsr1:02x} DSR2={dsr2:02x}  "
          f"SIR1={sir1:02x} SIR2={sir2:02x} SIR3={sir3:02x} "
          f"SIR4={sir4:02x} SIR5={sir5:02x}  GOPMCR={gopm:02x}")

    # DSR1 bit decode (per AN6285 + ATA6847L datasheet)
    print(f"  DSR1: GDUS={'READY' if dsr1 & 0x04 else 'not-ready'}  "
          f"VDS_PRECHARGE={'done' if dsr1 & 0x02 else 'in-progress'}  "
          f"raw=0x{dsr1:02x}")
    print(f"  GOPMCR: mode=0x{gopm & 0x07:x}  "
          f"({'NORMAL' if (gopm & 7) == 7 else ('STANDBY' if (gopm & 7) == 4 else 'OFF/other')})")

    # SIR1 latched-fault decode
    faults = []
    if sir1 & 0x80: faults.append("OT(over-temperature)")
    if sir1 & 0x40: faults.append("UVLO(under-voltage)")
    if sir1 & 0x20: faults.append("ILIM")
    if sir1 & 0x10: faults.append("VDSC_BOOT(bootstrap-short)")
    if sir1 & 0x08: faults.append("VDS_LS_SC")
    if sir1 & 0x02: faults.append("VDS_SC")
    if sir1 & 0x01: faults.append("OL_OPEN_LOAD")
    print(f"  SIR1 faults: {', '.join(faults) if faults else 'NONE'}")

    if last_dsr1 is not None:
        if last_dsr1 == 0xAB:
            print(f"  EnterGduNormal: NEVER CALLED YET (run motor first)")
        else:
            res = "SUCCESS" if last_result else "TIMEOUT"
            print(f"  EnterGduNormal last call: {res} after {last_attempts} polls, "
                  f"final DSR1=0x{last_dsr1:02x}")
            if last_dsr1 == 0xFF:
                print(f"    -> 0xFF = SPI read failed every time. Gate driver was NOT actually in Normal mode.")

def bemf_probe(port: str, baud: int, count: int = 1):
    """Read BEMF GPIO state count times (1 second between reads).
    Use during hand-rotation to see all 3 lines toggle through 6 patterns."""
    s = serial.Serial(port, baud, timeout=0.5)
    time.sleep(0.2); s.reset_input_buffer()
    print(f"Opening {port} @ {baud}  — {count} BEMF read(s)")
    print("        floatPhase  expectedPost  step  risingZc")
    for i in range(count):
        s.write(frame(0x41)); s.flush()
        time.sleep(0.2)
        rx = s.read(s.in_waiting or 1)
        time.sleep(0.05); rx += s.read(s.in_waiting)
        parsed = parse_reply(rx)
        if not parsed or parsed[0] != 0x41:
            print(f"  bad reply: {rx.hex()}")
        else:
            d = parsed[1]
            a, b, c, fp, expC, step, rising = d[:7]
            phase_name = {0: 'A', 1: 'B', 2: 'C'}.get(fp, '?')
            print(f"  [{i:3d}] BEMF: A={a} B={b} C={c}  "
                  f"floating={fp}({phase_name})  expectedPost={expC}  "
                  f"step={step}  rising={rising}")
        if i < count - 1:
            time.sleep(0.8)
    s.close()

def main():
    if len(sys.argv) > 1 and sys.argv[1] == "bemf":
        port = sys.argv[2] if len(sys.argv) > 2 else '/dev/ttyACM1'
        cnt  = int(sys.argv[3]) if len(sys.argv) > 3 else 30
        bemf_probe(port, 115200, cnt)
        return
    if len(sys.argv) > 1 and sys.argv[1] == "mon":
        port = sys.argv[2] if len(sys.argv) > 2 else '/dev/ttyACM1'
        baud = int(sys.argv[3]) if len(sys.argv) > 3 else 115200
        live_monitor(port, baud)
        return
    if len(sys.argv) > 1 and sys.argv[1] == "ata":
        port = sys.argv[2] if len(sys.argv) > 2 else '/dev/ttyACM1'
        baud = int(sys.argv[3]) if len(sys.argv) > 3 else 115200
        ata_diag(port, baud)
        return

    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM1'
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    print(f"Opening {port} @ {baud}")
    s = serial.Serial(port, baud, timeout=0.5)
    time.sleep(0.2); s.reset_input_buffer()

    print("\n== Basic GSP commands ==")
    for name, op in CMDS:
        pkt = frame(op)
        s.write(pkt); s.flush()
        time.sleep(0.4)
        rx = s.read(s.in_waiting or 1)
        time.sleep(0.05); rx += s.read(s.in_waiting)
        parsed = parse_reply(rx)
        tag = "OK" if parsed and parsed[0] == op else ("EMPTY" if not rx else "?")
        if parsed and parsed[0] == "BAD_CRC": tag = "BAD_CRC"
        print(f"  {name:13s} TX={pkt.hex():12s}  RX({len(rx):3d}B)={rx.hex()[:120]}  [{tag}]")

    print("\n== V4 telemetry (5 frames @ ~10 Hz) ==")
    s.write(frame(0x14)); s.flush()   # TELEM_START
    time.sleep(0.1); s.reset_input_buffer()
    n = 0
    for cmd, payload in read_frames(s, want_cmd=0x80, count=5, timeout_s=3.0):
        print(f"  [{n}] {decode_telem(payload)}")
        n += 1
    if n == 0:
        print("  (no TELEM_FRAME received — V4 telem path may not be active)")
    s.write(frame(0x15)); s.flush()   # TELEM_STOP
    s.close()

if __name__ == '__main__':
    main()
