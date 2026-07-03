#!/media/bhanu1234/Development/ProjectGaruda-ak512/venv/bin/python
"""
esc_ak.py - bench console for the FULL dspic33AKESC firmware (GSP v1).

Decodes the full 68-byte AK snapshot (same layout as the React GUI):
eRPM comes from hwzcStepPeriodHR (the sector-PI HR period) with the
450000/stepPeriod SW fallback - esc.py (Simplified tool) reads the wrong
offsets for this firmware in closed loop.

USAGE
    python3 esc_ak.py /dev/ttyACM0            # interactive
    python3 esc_ak.py /dev/ttyACM0 watch 5    # watch 5 s, exit

COMMANDS (type + Enter while streaming)
    start | stop | clr        motor control
    t <0..2000>               throttle (auto-switches source to GSP)
    pot                       hand throttle back to the board pot
    diag                      one-shot HWZC 0x18 diag (per-sector cap/miss)
    q                         quit (sends STOP first)

Only one program may own the port - close the GUI/MPLAB terminal first.
"""
import sys, time, struct, select

try:
    import serial
except ImportError:
    sys.exit("pyserial missing - use the venv python:\n"
             "  /media/bhanu1234/Development/ProjectGaruda-ak512/venv/bin/python esc_ak.py /dev/ttyACM0")

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD = 115200

# Scaling (matches the React GUI's AK decode)
VBUS_V   = 3.3 * 19.8 / 4096          # volts per count
IBUS_CPA = 93.0                        # counts per amp, 2048-centered

CMD = dict(PING=0x00, INFO=0x01, SNAP=0x02, START=0x03, STOP=0x04,
           CLR=0x05, THR=0x06, SRC=0x07, HB=0x08, DIAG=0x18)

STATES = ["IDLE","ARMED","DETECT","ALIGN","OL_RAMP","MORPH",
          "CLOSED_LOOP","BRAKING","RECOVERY","FAULT"]

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc

def pkt(cmd, payload=b"") -> bytes:
    body = bytes([1 + len(payload), cmd]) + payload
    c = crc16(body)
    return bytes([0x02]) + body + bytes([c >> 8, c & 0xFF])

class Link:
    def __init__(self, port):
        self.s = serial.Serial(port, BAUD, timeout=0.05)
        self.buf = b""
    def send(self, cmd, payload=b""):
        self.s.write(pkt(cmd, payload))
    def poll(self):
        """Yield (cmdId, payload) for every complete frame in the stream."""
        self.buf += self.s.read(512)
        out = []
        while True:
            i = self.buf.find(b"\x02")
            if i < 0:
                self.buf = b""; break
            if len(self.buf) < i + 2: break
            ln = self.buf[i+1]
            if not (1 <= ln <= 252):
                self.buf = self.buf[i+1:]; continue
            end = i + 2 + ln + 2
            if len(self.buf) < end: break
            body = self.buf[i+1:i+2+ln]
            rx   = (self.buf[end-2] << 8) | self.buf[end-1]
            self.buf = self.buf[end:]
            if crc16(body) == rx:
                out.append((body[1], body[2:]))
        return out

def fmt_snapshot(p: bytes, n: int) -> str:
    st, fault, step = p[0], p[1], p[2]
    thr   = struct.unpack_from("<H", p, 4)[0]
    duty  = p[6]
    vbus  = struct.unpack_from("<H", p, 8)[0]  * VBUS_V
    ibus  = (struct.unpack_from("<H", p, 10)[0] - 2048) / IBUS_CPA
    ibmax = (struct.unpack_from("<H", p, 12)[0] - 2048) / IBUS_CPA
    zcth  = struct.unpack_from("<H", p, 16)[0]
    stepP = struct.unpack_from("<H", p, 18)[0]
    good  = struct.unpack_from("<H", p, 20)[0]
    sync  = p[24]
    zconf = struct.unpack_from("<H", p, 26)[0]
    ztmo  = struct.unpack_from("<H", p, 28)[0]
    hwen  = p[30]
    hwHR  = struct.unpack_from("<I", p, 40)[0]
    up    = struct.unpack_from("<I", p, 64)[0]

    if hwen and hwHR:
        erpm = 1_000_000_000 // hwHR
    elif stepP:
        erpm = 450_000 // stepP
    else:
        erpm = 0

    return (f"*{n:4d} {STATES[st] if st < len(STATES) else st:<11.11s} "
            f"flt={fault} thr={thr:4d} duty={duty:3d}% "
            f"Vbus={vbus:5.1f}V eRPM={erpm:6d} hwHR={hwHR:6d} "
            f"Ibus={ibus:+5.1f}/{ibmax:+5.1f}A zcTh={zcth:4d} "
            f"good={good:5d} conf={zconf:5d} tmo={ztmo:4d} sync={sync} up={up}s")

def fmt_diag(p: bytes) -> str:
    cap  = struct.unpack_from("<H", p, 0)[0]
    xsec = struct.unpack_from("<H", p, 2)[0]
    hr   = struct.unpack_from("<I", p, 4)[0]
    good = struct.unpack_from("<H", p, 8)[0]
    caps = struct.unpack_from("<6I", p, 12)
    miss = struct.unpack_from("<6I", p, 36)
    erpm = 1_000_000_000 // hr if hr else 0
    return (f"  DIAG capPm={cap} crossPm={xsec} bias={xsec-cap:+d}pm "
            f"HR={hr} ({erpm} eRPM) good={good}\n"
            f"       caps/sector {list(caps)}\n"
            f"       miss/sector {list(miss)}")

def main():
    link = Link(PORT)
    print(f"Connected {PORT} @ {BAUD}. commands: start | stop | t <0..2000> | pot | clr | diag | q")

    watch = None
    if len(sys.argv) > 3 and sys.argv[2] == "watch":
        watch = time.time() + float(sys.argv[3])

    gsp_src = False
    n = 0
    last = 0.0
    try:
        while True:
            if watch and time.time() > watch:
                break
            now = time.time()
            if now - last >= 0.1:
                last = now
                link.send(CMD["SNAP"])
                link.send(CMD["HB"])

            for cmd, pl in link.poll():
                if cmd == CMD["SNAP"] and len(pl) >= 68:
                    n += 1
                    print(fmt_snapshot(pl, n))
                elif cmd == CMD["DIAG"] and len(pl) >= 60:
                    print(fmt_diag(pl))
                elif cmd == 0xFF and pl:
                    print(f"  !! firmware error 0x{pl[0]:02x}")

            if not watch and select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip().lower()
                if line == "q":
                    link.send(CMD["STOP"]); break
                elif line == "start":
                    link.send(CMD["START"])
                elif line == "stop":
                    link.send(CMD["STOP"])
                elif line == "clr":
                    link.send(CMD["CLR"])
                elif line == "pot":
                    link.send(CMD["SRC"], bytes([0])); gsp_src = False
                elif line == "diag":
                    link.send(CMD["DIAG"])
                elif line.startswith("t "):
                    try:
                        v = max(0, min(2000, int(line.split()[1])))
                    except ValueError:
                        continue
                    if not gsp_src:
                        link.send(CMD["SRC"], bytes([1]))  # IDLE-only in fw
                        gsp_src = True
                        time.sleep(0.05)
                    link.send(CMD["THR"], struct.pack("<H", v))
            time.sleep(0.01)
    finally:
        try: link.send(CMD["STOP"])
        except Exception: pass
        link.s.close()

if __name__ == "__main__":
    main()
