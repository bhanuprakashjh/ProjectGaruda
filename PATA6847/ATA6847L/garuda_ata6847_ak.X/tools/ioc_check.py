#!/usr/bin/env python3
"""
IOC verification — confirms the edge-triggered BEMF path is actually
doing the work (vs. accidentally falling back to PTG or OL).

Per-sample deltas + plateau auto-detector for diagnosing the 14k eRPM
startup stall (2026-05-21). Cumulative columns made it impossible to
see which gate was active in any given window.

Columns:
  dA   dDem  dBlk  dPol  dInt  dUnAn  fires/s  acc/s  DOM
  ^^^^ deltas since previous frame (per ~100ms snapshot) ^^^^^^

  dom: tag for the rejection-gate that ate >70% of rejects in this
       sample. "UNAN" = unanimous-wrong-polarity → sector-arm mismatch.
       "POL" = split-vote polarity → comparator noise. "DEM"/"BLK" usual.

Plateau detection: if eRPM is stable ±5% AND iAccept doesn't grow for
≥0.5s, accumulate gate-share stats. End-of-run prints the dominant
gate during each detected plateau.

Usage:
  python3 tools/ioc_check.py [/dev/ttyACM1]  [--duration N]  [--csv]
"""

import argparse
import struct
import sys
import time
from datetime import datetime

import serial

GSP_START = 0x02
CMD_GET_SNAPSHOT = 0x02
CMD_TELEM_START = 0x14
CMD_TELEM_STOP = 0x15
CMD_TELEM_FRAME = 0x80

STATE_NAMES = ["IDLE", "ARMED", "ALIGN", "OL_RAMP", "CL", "RECOVERY", "FAULT"]

PLATEAU_MIN_SECONDS = 0.5
PLATEAU_ERPM_TOLERANCE = 0.05    # ±5%
DOMINANT_FRACTION = 0.70


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def build_packet(cmd_id, payload=b""):
    body = bytes([1 + len(payload), cmd_id]) + payload
    c = crc16(body)
    return bytes([GSP_START]) + body + bytes([c >> 8, c & 0xFF])


def parse_packets(buf):
    out = []
    while len(buf) >= 5:
        i = buf.find(bytes([GSP_START]))
        if i < 0:
            return out, b""
        if i > 0:
            buf = buf[i:]
        if len(buf) < 2:
            break
        plen = buf[1]
        if plen < 1 or plen > 249:
            buf = buf[1:]
            continue
        total = 2 + plen + 2
        if len(buf) < total:
            break
        body = buf[1:2 + plen]
        crc_recv = (buf[2 + plen] << 8) | buf[2 + plen + 1]
        if crc_recv != crc16(body):
            buf = buf[1:]
            continue
        out.append((buf[2], buf[3:2 + plen]))
        buf = buf[total:]
    return out, buf


def decode(data):
    """Decode the IOC-overlaid 6-step snapshot. See gsp_commands.c."""
    if len(data) < 176:
        return None
    s = {}
    s["state"] = data[0]
    s["fault"] = data[1]
    s["eRpm"] = struct.unpack_from("<I", data, 22)[0]
    s["goodZc"] = struct.unpack_from("<H", data, 26)[0]
    s["stepPeriod"] = struct.unpack_from("<H", data, 18)[0]
    s["iocAccept"]     = struct.unpack_from("<I", data, 72)[0]
    s["iocDemagRej"]   = struct.unpack_from("<I", data, 76)[0]
    s["iocBlankRej"]   = struct.unpack_from("<I", data, 80)[0]
    s["iocPolRej"]     = struct.unpack_from("<I", data, 84)[0]
    s["iocFiresA"]     = struct.unpack_from("<I", data, 88)[0]
    s["iocIntRej"]     = struct.unpack_from("<I", data, 92)[0]
    s["iocPolUnAn"]    = struct.unpack_from("<I", data, 96)[0]
    s["iocFiresB"]     = struct.unpack_from("<I", data, 172)[0]
    # PI/timing diagnostics
    s["Tact"]      = struct.unpack_from("<H", data, 20)[0]   # actualStepPeriodHR (1× real)
    s["capVal"]    = struct.unpack_from("<H", data, 28)[0]   # diagLastCapValue (HR ticks since prev comm)
    s["delta"]     = struct.unpack_from("<h", data, 30)[0]   # diagDelta (signed)
    s["tMeasHR"]   = struct.unpack_from("<H", data, 108)[0]  # smoothed (halved per legacy)
    return s


def dominant_gate(deltas):
    """Return tag of gate that took >70% of rejects this sample."""
    total = sum(deltas.values())
    if total < 10:
        return "    "
    for tag, n in deltas.items():
        if n >= total * DOMINANT_FRACTION:
            return f"{tag:<4}"
    return "mix "


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="/dev/ttyACM1")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--csv", action="store_true")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.05)
    print(f"opened {args.port}", file=sys.stderr)
    ser.write(build_packet(CMD_TELEM_START))

    csv_file = None
    csv_path = None
    if args.csv:
        csv_path = f"ioc_run_{datetime.now():%H%M%S}.csv"
        csv_file = open(csv_path, "w")
        csv_file.write("t,state,eRpm,Tp,goodZc,"
                       "iAccept,iDem,iBlk,iPol,iInt,iUnAn,iFiresA,iFiresB,"
                       "dA,dDem,dBlk,dPol,dInt,dUnAn,fires_s,dom\n")

    header = ("  t      state  eRpm   Tp   Tact   cap  delta  Tp/Tact   dCom    dA   dDem  dBlk  dPol  dUnAn"
              "  fires/s   DOM")
    print("─" * len(header), file=sys.stderr)
    print(header, file=sys.stderr)
    print("─" * len(header), file=sys.stderr)

    t0 = time.time()
    buf = b""
    prev = None         # previous snapshot
    prev_t = t0

    # Plateau tracker: collect samples where eRPM is stable AND iAccept frozen.
    plateaus = []
    cur_plat = None     # dict if currently tracking

    try:
        while True:
            if args.duration is not None and (time.time() - t0) > args.duration:
                break
            chunk = ser.read(512)
            if chunk:
                buf += chunk
            packets, buf = parse_packets(buf)
            for cmd, payload in packets:
                if cmd != CMD_TELEM_FRAME:
                    continue
                snap_data = payload[2:]
                s = decode(snap_data)
                if s is None:
                    continue
                t = time.time() - t0

                if prev is None:
                    dA = dDem = dBlk = dPol = dInt = dUnAn = dFiresA = dFiresB = 0
                    dCom = 0
                    rate = 0
                else:
                    dt = max(t - prev_t, 1e-6)
                    dA      = s["iocAccept"]     - prev["iocAccept"]
                    dDem    = s["iocDemagRej"]   - prev["iocDemagRej"]
                    dBlk    = s["iocBlankRej"]   - prev["iocBlankRej"]
                    dPol    = s["iocPolRej"]     - prev["iocPolRej"]
                    dInt    = s["iocIntRej"]     - prev["iocIntRej"]
                    dUnAn   = s["iocPolUnAn"]    - prev["iocPolUnAn"]
                    dFiresA = s["iocFiresA"]     - prev["iocFiresA"]
                    dFiresB = s["iocFiresB"]     - prev["iocFiresB"]
                    # sectorCount is uint16 — handle wraparound
                    dCom    = (s["goodZc"] - prev["goodZc"]) & 0xFFFF
                    rate = int((dFiresA + dFiresB) / dt)

                # Split L4: unanimous + (split-vote) — disjoint
                dPolSplit = max(dPol - dUnAn, 0)
                gate_deltas = {
                    "DEM": dDem, "BLK": dBlk, "INT": dInt,
                    "UNAN": dUnAn, "POL": dPolSplit,
                }
                dom = dominant_gate(gate_deltas)

                # Plateau detection: in CL, eRPM > 1000, no accepts growth
                state_name = STATE_NAMES[s["state"]] if s["state"] < len(STATE_NAMES) else f"S{s['state']}"
                is_plateau_candidate = (
                    state_name == "CL"
                    and s["eRpm"] >= 1000
                    and prev is not None
                    and dA == 0
                    and prev["eRpm"] > 0
                    and abs(s["eRpm"] - prev["eRpm"]) <= prev["eRpm"] * PLATEAU_ERPM_TOLERANCE
                )

                if is_plateau_candidate:
                    if cur_plat is None:
                        cur_plat = {
                            "t_start": prev_t, "eRpm0": prev["eRpm"],
                            "sumDem": 0, "sumBlk": 0, "sumPol": 0,
                            "sumInt": 0, "sumUnAn": 0, "sumFires": 0,
                            "sumCom": 0,
                        }
                    cur_plat["t_end"]    = t
                    cur_plat["eRpm1"]    = s["eRpm"]
                    cur_plat["sumDem"]  += dDem
                    cur_plat["sumBlk"]  += dBlk
                    cur_plat["sumPol"]  += dPolSplit
                    cur_plat["sumInt"]  += dInt
                    cur_plat["sumUnAn"] += dUnAn
                    cur_plat["sumFires"] += dFiresA + dFiresB
                    cur_plat["sumCom"]  += dCom
                else:
                    if cur_plat is not None:
                        dur = cur_plat["t_end"] - cur_plat["t_start"]
                        if dur >= PLATEAU_MIN_SECONDS:
                            plateaus.append(cur_plat)
                        cur_plat = None

                # PI timing ratio: Tp (PI commanded) / Tact (real measured)
                # — comment in sector_pi.c says raw is 2× real. If this
                # ratio is ~2.0 at steady state and drifts at desync,
                # something timestamp-related is breaking.
                ratio = (s['stepPeriod'] / s['Tact']) if s['Tact'] > 0 else 0.0
                print(f"  {t:6.2f}  {state_name:<5}  {s['eRpm']:>6}  "
                      f"{s['stepPeriod']:>4}  {s['Tact']:>5}  "
                      f"{s['capVal']:>4}  {s['delta']:>5}  {ratio:6.2f}  "
                      f"{dCom:>5}  {dA:>4}  {dDem:>5}  {dBlk:>5}  {dPol:>4}  {dUnAn:>5}"
                      f"  {rate:>6}   {dom}",
                      flush=True)

                if csv_file:
                    csv_file.write(f"{t:.3f},{s['state']},{s['eRpm']},{s['stepPeriod']},"
                                   f"{s['goodZc']},{s['iocAccept']},{s['iocDemagRej']},"
                                   f"{s['iocBlankRej']},{s['iocPolRej']},{s['iocIntRej']},"
                                   f"{s['iocPolUnAn']},{s['iocFiresA']},{s['iocFiresB']},"
                                   f"{dA},{dDem},{dBlk},{dPol},{dInt},{dUnAn},"
                                   f"{rate},{dom.strip()}\n")

                prev = s
                prev_t = t

    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.write(build_packet(CMD_TELEM_STOP))
        except Exception:
            pass
        ser.close()
        if csv_file:
            csv_file.close()
            print(f"\nlog saved to {csv_path}", file=sys.stderr)

        # Close any open plateau
        if cur_plat is not None:
            dur = cur_plat["t_end"] - cur_plat["t_start"]
            if dur >= PLATEAU_MIN_SECONDS:
                plateaus.append(cur_plat)

        if plateaus:
            print("\n" + "=" * 72, file=sys.stderr)
            print("PLATEAU REPORT — windows with stable eRPM and no new accepts",
                  file=sys.stderr)
            print("=" * 72, file=sys.stderr)
            for i, p in enumerate(plateaus, 1):
                dur = p["t_end"] - p["t_start"]
                total_rej = p["sumDem"] + p["sumBlk"] + p["sumPol"] + p["sumInt"] + p["sumUnAn"]
                com_rate = p["sumCom"] / dur if dur > 0 else 0
                print(f"\n#{i}  t={p['t_start']:.2f}→{p['t_end']:.2f}s  "
                      f"({dur:.2f}s @ ~{p['eRpm0']} eRPM)", file=sys.stderr)
                print(f"   Commutate fires : {p['sumCom']} ({com_rate:.0f}/s)", file=sys.stderr)
                print(f"   IOC edge fires  : {p['sumFires']}", file=sys.stderr)
                print(f"   IOC rejects     : {total_rej}", file=sys.stderr)

                # Top-level disambiguation: CCT3 chain alive or hung?
                if p["sumCom"] == 0:
                    print(f"   → CCT3 chain HUNG — Commutate didn't fire at all.",
                          file=sys.stderr)
                    print(f"     Investigate hal_com_timer.c schedule path / phase guards.",
                          file=sys.stderr)
                    continue
                elif p["sumFires"] == 0:
                    print(f"   → Commutate runs but BEMF signal flat — rotor coasting/stalled.",
                          file=sys.stderr)
                    print(f"     PWM duty went to 0 OR motor stopped spinning entirely.",
                          file=sys.stderr)
                    print(f"     Fix is on the startup-tuning side (gentler OL ramp, lock detect).",
                          file=sys.stderr)
                    continue

                if total_rej == 0:
                    print(f"   → fires but no rejects — odd state.", file=sys.stderr)
                else:
                    def pct(n): return f"{100.0*n/total_rej:5.1f}%"
                    print(f"   DEM   {p['sumDem']:>7}   {pct(p['sumDem'])}", file=sys.stderr)
                    print(f"   BLK   {p['sumBlk']:>7}   {pct(p['sumBlk'])}", file=sys.stderr)
                    print(f"   INT   {p['sumInt']:>7}   {pct(p['sumInt'])}", file=sys.stderr)
                    print(f"   UNAN  {p['sumUnAn']:>7}   {pct(p['sumUnAn'])}  "
                          "← unanimous-wrong = sector-arm mismatch", file=sys.stderr)
                    print(f"   POL   {p['sumPol']:>7}   {pct(p['sumPol'])}  "
                          "← split-vote = comparator noise", file=sys.stderr)
                    # Diagnosis
                    if p['sumUnAn'] > 0.4 * total_rej:
                        print(f"   DIAGNOSIS: sector arm out of sync with rotor "
                              "(L4 unanimous dominates)", file=sys.stderr)
                    elif p['sumDem'] > 0.4 * total_rej:
                        print(f"   DIAGNOSIS: demag window swallowing real ZC "
                              "(L1 dominates — try shorter demag)", file=sys.stderr)
                    elif p['sumBlk'] > 0.4 * total_rej:
                        print(f"   DIAGNOSIS: static blanking still active "
                              "(L2 dominates — Commutate seeded too long)", file=sys.stderr)
                    elif p['sumPol'] > 0.4 * total_rej:
                        print(f"   DIAGNOSIS: comparator noise dominates "
                              "(L4 split — need HW blanking)", file=sys.stderr)


if __name__ == "__main__":
    main()
