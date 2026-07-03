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

# Scaling. Vbus divider on THIS board is 23.2:1 (bench-verified 2026-07-03:
# PSU at 16.0V read 13.6 with the React GUI's 19.8 assumption -> 19.8*16/13.6
# = 23.3). The GUI under-reads Vbus by ~15% until its scale is fixed too.
VBUS_V   = 3.3 * 23.2 / 4096          # volts per count
IBUS_CPA = 93.0                        # counts per amp, 2048-centered

CMD = dict(PING=0x00, INFO=0x01, SNAP=0x02, START=0x03, STOP=0x04,
           CLR=0x05, THR=0x06, SRC=0x07, HB=0x08,
           GETP=0x10, SETP=0x11, DIAG=0x18,
           SARM=0x30, SSTAT=0x31, SREAD=0x32)

# Burst scope (24 kHz, 128 x 26B ring). Auto-armed on CLOSED_LOOP entry:
# threshold trigger, bus current (CH_ID=2) rising through SCOPE_TRIG_A,
# 75% pre-trigger -> ~16 sectors of history before a current slam.
SCOPE_TRIG_A   = 15.0    # amps, bus; normal 4A-load ripple never crosses
SCOPE_PRE_PCT  = 75
SCOPE_CHUNK    = 9
SCOPE_STATES   = ["IDLE", "ARMED", "FILLING", "READY"]

# Handy param-name shortcuts for set/get (full IDs in gsp/gsp_params.h).
# Keys must be lowercase — the console lowercases all typed input.
# SET_PARAM is IDLE-only in fw (except AN1078/OC live IDs): stop, set, start.
PARAMS = dict(
    pera   =0x5B,   # zcDemagBlankPerA    WS1: % sector per 256 cts phase excess
    peradb =0x5C,   # zcDemagBlankIbusDb  WS1: deadband, raw counts
    blankmax=0x9E,  # zcDemagBlankMaxPct  WS1: total blank cap % (25 legacy, 33 = +headroom)
    blankextra=0x57,# zcDemagBlankExtraPct base demag extra %
)

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

def scope_arm_payload(amps: float) -> bytes:
    """trigMode=3 THRESHOLD, prePct, trigCh=2 CH_ID(bus mA), edge=0 RISING, thresh mA."""
    return struct.pack("<4BhH", 3, SCOPE_PRE_PCT, 2, 0, int(amps * 1000), 0)

def fmt_scope_sample(i: int, trig: int, raw: bytes) -> str:
    (ia, ib, ibus, _iq, vd, vq, step, bemf, _x2,
     omega, mod) = struct.unpack_from("<11h", raw, 0)
    flags, st = raw[22], raw[23]
    tick = struct.unpack_from("<H", raw, 24)[0]
    vbus = vd * 3.3 * 23.2 / 4096
    mark = ">" if i == trig else " "
    return (f"  {mark}{i:3d} t={tick:5d} st={st} S{step} duty={mod/100:5.1f}% "
            f"eRPM={omega*10:6d} ia={ia/1000:+6.2f} ibus={ibus/1000:+6.2f} "
            f"vbus={vbus:5.1f} zcTh={vq:4d} bemf={bemf:4d}")

def fmt_diag(p: bytes) -> str:
    cap  = struct.unpack_from("<H", p, 0)[0]
    xsec = struct.unpack_from("<H", p, 2)[0]
    hr   = struct.unpack_from("<I", p, 4)[0]
    good = struct.unpack_from("<H", p, 8)[0]
    caps = struct.unpack_from("<6I", p, 12)
    miss = struct.unpack_from("<6I", p, 36)
    erpm = 1_000_000_000 // hr if hr else 0
    return (f"  DIAG bias={xsec-cap:+4d}pm (cap={cap} cross={xsec}) {erpm:6d} eRPM good={good:5d} "
            f"caps={list(caps)} miss={list(miss)}")

def main():
    link = Link(PORT)
    print(f"Connected {PORT} @ {BAUD}. commands: start | stop | t <0..2000> | pot | clr | diag | "
          f"status | tel | get/set <{'|'.join(PARAMS)}> | q")

    watch = None
    if len(sys.argv) > 3 and sys.argv[2] == "watch":
        watch = time.time() + float(sys.argv[3])   # watch mode streams everything

    # Startup settings dump: read the tuning params once (responses print
    # as "  GET <name> = <val>" when they arrive).
    print("settings:")
    for pid in PARAMS.values():
        link.send(CMD["GETP"], struct.pack("<H", pid))

    # Telemetry is quiet while IDLE (state 0): snapshots are still polled so
    # state changes are seen, but lines only print once the motor leaves IDLE
    # (board button or 'start').  'status' prints one line on demand; 'tel'
    # toggles the full stream regardless of state.
    gsp_src = False
    n = 0
    last = 0.0
    last_diag = 0.0
    last_sstat = 0.0
    last_st = None
    manual_tel = watch is not None
    oneshot = True          # print the first snapshot as the connect status
    scope_armed = False     # armed this spin (re-arms next CL entry after dump)
    scope_trig = 0          # trigger index from last status
    scope_total = 0         # sample count to read
    scope_samples = {}      # offset -> raw bytes, collected via SREAD
    try:
        while True:
            if watch and time.time() > watch:
                break
            now = time.time()
            if now - last >= 0.1:
                last = now
                link.send(CMD["SNAP"])
                link.send(CMD["HB"])
            # Auto-diag: one HWZC diag per second while in CLOSED_LOOP so
            # every loaded run captures per-sector cap/miss + bias evidence
            # (nine bench runs went by with zero manual diag under load).
            if last_st == 6 and now - last_diag >= 1.0:
                last_diag = now
                link.send(CMD["DIAG"])
            # Poll scope status while armed - INCLUDING after a fault, which
            # is exactly when the frozen capture is waiting to be read.
            if scope_armed and now - last_sstat >= 1.0:
                last_sstat = now
                link.send(CMD["SSTAT"])
            # Auto-arm the burst scope on CLOSED_LOOP entry
            if last_st == 6 and not scope_armed and not scope_samples:
                scope_armed = True
                link.send(CMD["SARM"], scope_arm_payload(SCOPE_TRIG_A))
                print(f"  (scope auto-armed: bus current rising {SCOPE_TRIG_A:.0f}A, "
                      f"{SCOPE_PRE_PCT}% pre-trigger)")

            for cmd, pl in link.poll():
                if cmd == CMD["SNAP"] and len(pl) >= 68:
                    n += 1
                    st = pl[0]
                    if st != last_st and last_st is not None:
                        a = STATES[last_st] if last_st < len(STATES) else last_st
                        b = STATES[st] if st < len(STATES) else st
                        print(f"  -- {a} -> {b}")
                        if st == 0:
                            print("  (idle - telemetry paused; 'status' for one line, 'tel' to stream)")
                    last_st = st
                    if st != 0 or manual_tel or oneshot:
                        print(fmt_snapshot(pl, n))
                        oneshot = False
                elif cmd == CMD["DIAG"] and len(pl) >= 60:
                    print(fmt_diag(pl))
                elif cmd == CMD["SSTAT"] and len(pl) >= 6:
                    sst, _tm, _pp, scope_trig, cnt, _sz = pl[0], pl[1], pl[2], pl[3], pl[4], pl[5]
                    if sst == 3 and not scope_samples and not scope_total:
                        scope_total = cnt
                        print(f"  == SCOPE TRIGGERED == reading {cnt} samples "
                              f"(trigger at index {scope_trig})")
                        for off in range(0, cnt, SCOPE_CHUNK):
                            link.send(CMD["SREAD"],
                                      bytes([off, min(SCOPE_CHUNK, cnt - off)]))
                elif cmd == CMD["SREAD"] and len(pl) >= 2:
                    off, cnt2 = pl[0], pl[1]
                    for k in range(cnt2):
                        scope_samples[off + k] = pl[2 + k*26 : 2 + (k+1)*26]
                    if scope_total and len(scope_samples) >= scope_total:
                        print(f"  -- scope capture ({scope_total} samples @24kHz, "
                              f"'>' = trigger, S<n> = sector) --")
                        for i in sorted(scope_samples):
                            print(fmt_scope_sample(i, scope_trig, scope_samples[i]))
                        scope_samples = {}
                        scope_total = 0
                        scope_armed = False   # re-arms on next CL entry
                elif cmd == CMD["SARM"]:
                    pass  # arm ack, already announced
                elif cmd in (CMD["GETP"], CMD["SETP"]) and len(pl) >= 6:
                    pid = struct.unpack_from("<H", pl, 0)[0]
                    val = struct.unpack_from("<I", pl, 2)[0]
                    name = next((k for k, v in PARAMS.items() if v == pid), f"0x{pid:02x}")
                    verb = "SET" if cmd == CMD["SETP"] else "GET"
                    print(f"  {verb} {name} = {val}")
                elif cmd == 0xFF and pl:
                    errs = {0x04: "WRONG_STATE (motor must be stopped to set)",
                            0x05: "OUT_OF_RANGE", 0x06: "UNKNOWN_PARAM"}
                    print(f"  !! firmware error 0x{pl[0]:02x} {errs.get(pl[0], '')}")

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
                elif line in ("status", "s"):
                    oneshot = True
                elif line == "tel":
                    manual_tel = not manual_tel
                    print(f"  (stream {'ON' if manual_tel else 'auto: only when running'})")
                elif line.startswith(("set ", "get ")):
                    parts = line.split()
                    pid = PARAMS.get(parts[1]) if len(parts) > 1 else None
                    if pid is None:
                        try: pid = int(parts[1], 0)
                        except (ValueError, IndexError):
                            print(f"  params: {' '.join(PARAMS)} (or numeric id)"); continue
                    if parts[0] == "get":
                        link.send(CMD["GETP"], struct.pack("<H", pid))
                    elif len(parts) > 2:
                        try: val = int(parts[2], 0)
                        except ValueError:
                            print("  usage: set <name|id> <value>"); continue
                        link.send(CMD["SETP"], struct.pack("<HI", pid, val))
                    else:
                        print("  usage: set <name|id> <value>")
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
    try:
        main()
    except KeyboardInterrupt:
        print("\n(stopped)")
