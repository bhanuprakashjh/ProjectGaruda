#!/usr/bin/env python3
"""
Session tracker for bench testing — captures everything, dumps a
paste-friendly summary on Ctrl+C.

Streams telemetry at 50 Hz (same protocol as step6_logger.py), saves
a CSV, and at the end prints a compact text report with:

  - Header: build hash, motor profile, feature flags, run duration
  - State timeline: every state transition with timestamps
  - Per-state stats: time spent, peak eRPM, peak duty, peak Ibus,
    peak phase current
  - Top eRPM moments
  - Fault events with the at-fault current snapshot
  - PI health: ZC totals, miss/reject ratios

Use this for A/B testing controller changes — capture baseline,
then capture new build, paste both summaries into chat.

Usage:
    python3 tools/step6_session.py [--port /dev/ttyACM0]

Output:
    sessions/session_YYYYMMDD_HHMMSS.csv
    plus a clean text summary printed to stdout when you Ctrl+C.

Requires: pyserial
"""

import argparse
import csv
import os
import signal
import struct
import sys
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import serial


# ── GSP protocol (subset, mirroring step6_logger.py) ────────────────────
GSP_START_BYTE       = 0x02
GSP_CMD_PING         = 0x00
GSP_CMD_GET_INFO     = 0x01
GSP_CMD_GET_SNAPSHOT = 0x02
GSP_CMD_TELEM_START  = 0x14
GSP_CMD_TELEM_STOP   = 0x15
GSP_CMD_TELEM_FRAME  = 0x80
GSP_CMD_ERROR        = 0xFF

ESC_STATE_NAMES = {
    0: "IDLE", 1: "ARMED", 2: "DETECT", 3: "ALIGN", 4: "OL_RAMP",
    5: "MORPH", 6: "CL", 7: "BRAKING", 8: "RECOVERY", 9: "FAULT",
}
FAULT_NAMES = {
    0: "NONE", 1: "OV", 2: "UV", 3: "OC_SW", 4: "BOARD_PCI",
    5: "STALL", 6: "DESYNC", 7: "START_TO", 8: "MORPH_TO",
    9: "RX_LOSS", 10: "FOC_INT", 11: "FOC_BUS",
}

VBUS_SCALE_V        = 3.3 * 23.2 / 4095.0
IBUS_SCALE_A        = 3.3 / (4095.0 * 24.95 * 0.003)
IBUS_BIAS           = 2048
IADC_BIAS           = 2048
IADC_COUNTS_PER_AMP = 93.0
HWZC_ERPM_FROM_TICKS = 1_000_000_000


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def build_packet(cmd_id: int, payload: bytes = b"") -> bytes:
    """Frame: [START][LEN=1+payload_len][CMD][PAYLOAD][CRC_H][CRC_L]"""
    pkt_len = 1 + len(payload)
    body = bytes([pkt_len, cmd_id]) + payload
    c = crc16(body)
    return bytes([GSP_START_BYTE]) + body + struct.pack(">H", c)


def read_packet(ser, timeout=1.0):
    """Returns (cmd_id, payload) or None on timeout/CRC fail."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b: continue
        if b[0] == GSP_START_BYTE: break
    else:
        return None

    ser.timeout = max(deadline - time.monotonic(), 0.01)
    b = ser.read(1)
    if not b: return None
    pkt_len = b[0]

    ser.timeout = max(deadline - time.monotonic(), 0.01)
    data = ser.read(pkt_len + 2)
    if len(data) < pkt_len + 2: return None

    cmd_id = data[0]
    payload = data[1:pkt_len]
    rx_crc = struct.unpack(">H", data[pkt_len:pkt_len + 2])[0]
    if crc16(bytes([pkt_len]) + data[:pkt_len]) != rx_crc:
        return None
    return (cmd_id, payload)


def send_cmd(ser, cmd_id, payload=b"", timeout=1.0):
    ser.write(build_packet(cmd_id, payload))
    return read_packet(ser, timeout=timeout)


# ── Snapshot parser (focused on what we need for analysis) ──────────────
def parse_snapshot(p: bytes, t0: float) -> dict:
    """Return a dict with the fields the session report needs."""
    if len(p) < 68:
        return {}

    state, fault, _step, _direction, throttle, duty = \
        struct.unpack_from("<BBBBHBx", p, 0)
    vbus_raw, ibus_raw, ibus_max = struct.unpack_from("<HHH", p, 8)
    bemf_raw, zc_thresh, step_period, good_zc = struct.unpack_from("<HHHH", p, 14)
    _rising, _falling, synced = struct.unpack_from("<BBB", p, 22)
    zc_confirmed, zc_timeout = struct.unpack_from("<HH", p, 26)
    hwzc_en, _hwzc_phase = struct.unpack_from("<BB", p, 30)
    hwzc_zc, hwzc_miss, hwzc_hr = struct.unpack_from("<III", p, 32)
    sys_tick, uptime = struct.unpack_from("<II", p, 60)
    hwzc_reject = struct.unpack_from("<I", p, 170)[0] if len(p) >= 174 else 0

    # Phase current peaks (if frame is large enough)
    ia_pk_pos = ia_pk_neg = ib_pk_pos = ib_pk_neg = 0
    ibus_pk_pos = ibus_pk_neg = 0
    if len(p) >= 198:
        (_ia_raw, _ib_raw, ia_max, ia_min, ib_max, ib_min) = struct.unpack_from("<HHHHHH", p, 174)
        def _to_a(raw):
            if raw == 0 or raw == 0xFFFF: return 0.0
            return (raw - IADC_BIAS) / IADC_COUNTS_PER_AMP
        ia_pk_pos = _to_a(ia_max); ia_pk_neg = _to_a(ia_min)
        ib_pk_pos = _to_a(ib_max); ib_pk_neg = _to_a(ib_min)
    if len(p) >= 208:
        ibus_win_max, ibus_win_min = struct.unpack_from("<HH", p, 198)
        def _to_a(raw):
            if raw == 0 or raw == 0xFFFF: return 0.0
            return (raw - IADC_BIAS) / IADC_COUNTS_PER_AMP
        ibus_pk_pos = _to_a(ibus_win_max); ibus_pk_neg = _to_a(ibus_win_min)

    # Speed PI telemetry (offsets 208-227, present in builds with the new struct)
    sp_enabled = sp_zcs = sp_target = sp_error = sp_output = 0
    sp_integ = 0.0
    if len(p) >= 228:
        (sp_enabled, _sp_pad, sp_zcs, sp_target, sp_error,
         sp_output, sp_integ) = struct.unpack_from("<BBHIiIf", p, 208)

    vbus_v = vbus_raw * VBUS_SCALE_V
    ibus_a = (ibus_raw - IBUS_BIAS) * IBUS_SCALE_A
    if hwzc_en and hwzc_hr > 0:
        erpm = HWZC_ERPM_FROM_TICKS // hwzc_hr
    elif step_period > 0:
        # Best-effort SW eRPM (sim assumes PWM 45kHz)
        erpm = 450_000 // step_period
    else:
        erpm = 0

    return {
        "t":            time.monotonic() - t0,
        "state":        state,
        "state_name":   ESC_STATE_NAMES.get(state, f"?{state}"),
        "fault":        fault,
        "fault_name":   FAULT_NAMES.get(fault, f"?{fault}"),
        "throttle":     throttle,
        "duty":         duty,
        "vbus_V":       vbus_v,
        "ibus_A":       ibus_a,
        "eRPM":         erpm,
        "hwzc_en":      hwzc_en,
        "hwzc_hr":      hwzc_hr,
        "hwzc_zc":      hwzc_zc,
        "hwzc_miss":    hwzc_miss,
        "hwzc_reject":  hwzc_reject,
        "good_zc":      good_zc,
        "ia_pk_mag":    max(abs(ia_pk_pos), abs(ia_pk_neg)),
        "ib_pk_mag":    max(abs(ib_pk_pos), abs(ib_pk_neg)),
        "ibus_pk_mag":  max(abs(ibus_pk_pos), abs(ibus_pk_neg)),
        "uptime":       uptime,
        # Speed PI telemetry — non-zero when FEATURE_SPEED_PI=1 and CL active
        "spi_en":       sp_enabled,
        "spi_zcs":      sp_zcs,
        "spi_target":   sp_target,
        "spi_error":    sp_error,
        "spi_output":   sp_output,
        "spi_integ":    sp_integ,
    }


# ── Session analysis ────────────────────────────────────────────────────
def summarize(samples, info=None):
    """Build the paste-friendly report from collected samples."""
    if not samples:
        return "(no samples captured)"

    out = []
    out.append("=" * 78)
    out.append(f"BENCH SESSION SUMMARY  ({len(samples):,} samples, "
               f"{samples[-1]['t']:.1f}s)")
    out.append("=" * 78)

    if info:
        out.append(f"Firmware build : v{info.get('fwMajor','?')}."
                   f"{info.get('fwMinor','?')}.{info.get('fwPatch','?')}  "
                   f"hash=0x{info.get('buildHash',0):08x}")
        out.append(f"Motor profile  : {info.get('motorProfile','?')}  "
                   f"PP={info.get('motorPolePairs','?')}")
        out.append(f"PWM frequency  : {info.get('pwmFrequency','?'):,} Hz")
        out.append(f"Max eRPM cap   : {info.get('maxErpm','?'):,}")
        out.append("")

    # Steady values from first sample
    out.append(f"First sample   : state={samples[0]['state_name']}  "
               f"Vbus={samples[0]['vbus_V']:.1f}V  thr={samples[0]['throttle']}")

    # State transitions
    transitions = []
    cur = samples[0]['state_name']
    cur_t = 0.0
    state_time = defaultdict(float)
    state_peaks = defaultdict(lambda: {'eRPM': 0, 'duty': 0, 'ibus': 0,
                                       'ia_pk': 0, 'ibus_pk': 0})
    last_t = 0.0
    for s in samples:
        dt = s['t'] - last_t
        state_time[s['state_name']] += dt
        sp = state_peaks[s['state_name']]
        sp['eRPM']    = max(sp['eRPM'], s['eRPM'])
        sp['duty']    = max(sp['duty'], s['duty'])
        sp['ibus']    = max(sp['ibus'], abs(s['ibus_A']))
        sp['ia_pk']   = max(sp['ia_pk'], s['ia_pk_mag'])
        sp['ibus_pk'] = max(sp['ibus_pk'], s['ibus_pk_mag'])
        if s['state_name'] != cur:
            transitions.append((cur_t, s['t'], cur, s['state_name'],
                                s.get('fault_name','NONE')))
            cur = s['state_name']
            cur_t = s['t']
        last_t = s['t']
    transitions.append((cur_t, samples[-1]['t'], cur, '(end)',
                        samples[-1].get('fault_name','NONE')))

    out.append("")
    out.append("STATE TIMELINE")
    out.append("-" * 78)
    out.append(f"{'t_start':>8} {'t_end':>8} {'duration':>9} {'state':<10} "
               f"{'→ next':<10}  notes")
    for ts, te, st, nxt, ft in transitions:
        dur = te - ts
        note = f"fault={ft}" if ft != "NONE" else ""
        out.append(f"{ts:8.3f} {te:8.3f} {dur:8.3f}s {st:<10} {nxt:<10}  {note}")

    out.append("")
    out.append("PER-STATE PEAKS")
    out.append("-" * 78)
    out.append(f"{'state':<10} {'time(s)':>8} {'peak eRPM':>10} "
               f"{'peak duty%':>11} {'peak |Ibus|':>11} "
               f"{'peak Ia':>9} {'peak Ibus_pk':>13}")
    for st in sorted(state_time, key=lambda x: -state_time[x]):
        if state_time[st] < 0.05:
            continue
        sp = state_peaks[st]
        out.append(f"{st:<10} {state_time[st]:8.2f} {sp['eRPM']:>10,} "
                   f"{sp['duty']:>10}% {sp['ibus']:>10.2f}A "
                   f"{sp['ia_pk']:>8.2f}A {sp['ibus_pk']:>12.2f}A")

    # Speed PI summary — only when at least one sample has it enabled
    spi_samples = [s for s in samples if s.get('spi_en', 0) == 1]
    if spi_samples:
        out.append("")
        out.append("SPEED PI (v2: eRPM control + feedforward + deadband)")
        out.append("-" * 78)
        n_locked = sum(1 for s in spi_samples
                       if s.get('spi_zcs', 0) >= 100)  # past integral-disable window
        err_abs = [abs(s.get('spi_error', 0)) for s in spi_samples]
        err_max = max(err_abs) if err_abs else 0
        err_avg = sum(err_abs) / len(err_abs) if err_abs else 0
        out.append(f"Enabled samples : {len(spi_samples)}  "
                   f"(past integral-window: {n_locked})")
        out.append(f"|error| eRPM    : max {err_max:,}  avg {err_avg:,.1f}")
        if spi_samples:
            last = spi_samples[-1]
            out.append(f"Last sample     : target {last['spi_target']:,} eRPM  "
                       f"error {last['spi_error']:+,} eRPM  "
                       f"output {last['spi_output']:,} ticks  "
                       f"correction {last['spi_integ']:+.1f}")

    # Top eRPM moments
    out.append("")
    out.append("TOP eRPM SAMPLES")
    out.append("-" * 78)
    top = sorted(samples, key=lambda s: -s['eRPM'])[:5]
    out.append(f"{'t(s)':>7} {'state':<10} {'thr':>5} {'duty%':>6} "
               f"{'eRPM':>9} {'Vbus':>6} {'Ibus':>7}")
    for s in top:
        out.append(f"{s['t']:7.3f} {s['state_name']:<10} {s['throttle']:>5} "
                   f"{s['duty']:>5}% {s['eRPM']:>9,} "
                   f"{s['vbus_V']:>5.1f}V {s['ibus_A']:>+6.2f}A")

    # Faults
    faults = [s for s in samples if s['fault'] != 0]
    if faults:
        # Find unique fault entries (first time each fault appears)
        seen_fault = set()
        out.append("")
        out.append("FAULTS")
        out.append("-" * 78)
        out.append(f"{'t(s)':>7} {'fault':<10} {'thr':>5} {'duty%':>6} "
                   f"{'eRPM':>9} {'Vbus':>6} {'Ibus':>7} {'Ia_pk':>7}")
        for s in faults:
            key = (s['fault_name'], int(s['t']))
            if key in seen_fault: continue
            seen_fault.add(key)
            out.append(f"{s['t']:7.3f} {s['fault_name']:<10} {s['throttle']:>5} "
                       f"{s['duty']:>5}% {s['eRPM']:>9,} "
                       f"{s['vbus_V']:>5.1f}V {s['ibus_A']:>+6.2f}A "
                       f"{s['ia_pk_mag']:>6.2f}A")

    # ZC / PI health (last sample's totals)
    last = samples[-1]
    if last['hwzc_zc'] or last['hwzc_miss']:
        out.append("")
        out.append("HWZC TOTALS (cumulative at session end)")
        out.append("-" * 78)
        total = last['hwzc_zc']
        miss  = last['hwzc_miss']
        rej   = last['hwzc_reject']
        out.append(f"hwzc accepted   : {total:>10,}")
        out.append(f"hwzc misses     : {miss:>10,}   "
                   f"({100.0 * miss / max(1, total + miss):.2f}%)")
        out.append(f"hwzc rejections : {rej:>10,}   "
                   f"({100.0 * rej / max(1, total + rej):.2f}%)")

    # Lock acquisition (rough)
    cl_entries = [(i, s) for i, s in enumerate(samples)
                  if s['state_name'] == 'CL']
    if cl_entries:
        first_cl = cl_entries[0][1]
        # Find first sample after CL entry where eRPM stays > 5000 for 100ms
        out.append("")
        out.append(f"FIRST CL ENTRY  : t={first_cl['t']:.3f}s  "
                   f"duty={first_cl['duty']}%  thr={first_cl['throttle']}")
        peak_cl = max((s for s in samples if s['state_name'] == 'CL'),
                      key=lambda x: x['eRPM'])
        out.append(f"PEAK eRPM IN CL : {peak_cl['eRPM']:,} at t={peak_cl['t']:.3f}s  "
                   f"duty={peak_cl['duty']}%")

    out.append("=" * 78)
    return "\n".join(out)


# ── Main ────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate", type=int, default=50, help="Telemetry rate Hz")
    ap.add_argument("--no-csv", action="store_true")
    ap.add_argument("--label", default="", help="Tag for the session filename")
    args = ap.parse_args()

    # Connect
    print(f"Connecting to {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(0.1)
    ser.reset_input_buffer()

    resp = send_cmd(ser, GSP_CMD_PING)
    if not resp or resp[0] == GSP_CMD_ERROR:
        print("PING failed — is the firmware running?")
        sys.exit(1)
    print("  PING OK")

    # GET_INFO for header
    info = {}
    resp = send_cmd(ser, GSP_CMD_GET_INFO)
    if resp and resp[0] != GSP_CMD_ERROR and len(resp[1]) >= 20:
        d = resp[1]
        info = dict(
            protocolVersion = d[0],
            fwMajor = d[1], fwMinor = d[2], fwPatch = d[3],
            boardId = struct.unpack_from("<H", d, 4)[0],
            motorProfile = d[6], motorPolePairs = d[7],
            featureFlags = struct.unpack_from("<I", d, 8)[0],
            pwmFrequency = struct.unpack_from("<I", d, 12)[0],
            maxErpm = struct.unpack_from("<I", d, 16)[0],
        )
        if len(d) >= 24:
            info['buildHash'] = struct.unpack_from("<I", d, 20)[0]
        print(f"  Firmware v{info['fwMajor']}.{info['fwMinor']}.{info['fwPatch']}  "
              f"profile={info['motorProfile']}  PWM={info['pwmFrequency']:,}Hz")

    # Try TELEM_START first; fall back to polling if it errors
    use_stream = False
    period_ms = max(10, 1000 // args.rate)
    resp = send_cmd(ser, GSP_CMD_TELEM_START,
                    struct.pack("<HB", period_ms, 0))
    if resp and resp[0] != GSP_CMD_ERROR:
        use_stream = True
        print(f"  Streaming telemetry @ {args.rate} Hz")
    else:
        print(f"  Polling telemetry @ {args.rate} Hz")

    # Set up CSV
    csv_path = None
    csv_writer = None
    csv_file = None
    if not args.no_csv:
        out_dir = Path("sessions")
        out_dir.mkdir(exist_ok=True)
        suffix = ("_" + args.label) if args.label else ""
        csv_path = out_dir / f"session_{datetime.now():%Y%m%d_%H%M%S}{suffix}.csv"
        csv_file = open(csv_path, 'w', newline='')
        csv_writer = None  # created on first sample
        print(f"  CSV: {csv_path}")

    print()
    print("=" * 78)
    print(f"{'t(s)':>6} {'state':<10} {'thr':>4} {'duty':>5} {'eRPM':>8} "
          f"{'Vbus':>5} {'Ibus':>6} {'Ia_pk':>6} fault")
    print("=" * 78)

    samples = []
    t0 = time.monotonic()
    last_print_t = 0.0
    stop_flag = {"stop": False}

    def handle_signal(*_):
        stop_flag["stop"] = True
    signal.signal(signal.SIGINT, handle_signal)

    poll_interval = 1.0 / args.rate
    next_poll = time.monotonic()
    was_running = False   # run-only telemetry gate

    try:
        while not stop_flag["stop"]:
            now = time.monotonic()

            if use_stream:
                resp = read_packet(ser, timeout=0.2)
                if resp is None:
                    continue
                cmd_id, payload = resp
                if cmd_id != GSP_CMD_TELEM_FRAME:
                    continue
                snap = parse_snapshot(payload, t0)
            else:
                if now < next_poll:
                    time.sleep(min(0.005, next_poll - now))
                    continue
                next_poll += poll_interval
                resp = send_cmd(ser, GSP_CMD_GET_SNAPSHOT, timeout=0.2)
                if not resp or resp[0] != GSP_CMD_GET_SNAPSHOT:
                    continue
                snap = parse_snapshot(resp[1], t0)

            if not snap:
                continue

            # Run-only telemetry: the continuous idle stream is a waste. Stream
            # (print + CSV + summary samples) only from start (leaving IDLE) to
            # stop (back to IDLE), with banners. FAULT still streams.
            running = snap['state'] != 0   # 0 = IDLE
            if running and not was_running:
                print(f"\n▶──────── RUN START @ {snap['t']:6.2f}s ────────")
            elif was_running and not running:
                print(f"■──────── STOPPED @ {snap['t']:6.2f}s ────────\n")
            was_running = running
            if not running:
                continue

            samples.append(snap)

            # CSV
            if csv_writer is None and csv_file:
                csv_writer = csv.DictWriter(csv_file, fieldnames=list(snap.keys()))
                csv_writer.writeheader()
            if csv_writer:
                csv_writer.writerow(snap)

            # Print every ~250ms or on state/fault change
            do_print = False
            if snap['t'] - last_print_t > 0.25: do_print = True
            if len(samples) >= 2:
                prev = samples[-2]
                if prev['state'] != snap['state']: do_print = True
                if prev['fault'] != snap['fault']: do_print = True
            if len(samples) == 1: do_print = True
            if do_print:
                marker = "◀" if (len(samples) >= 2 and
                                  samples[-2]['state'] != snap['state']) else ""
                spi_tag = ""
                if snap.get('spi_en', 0) == 1:
                    # v2: target/error are in eRPM units, integrator is correction
                    spi_tag = (f" spi tgt={snap['spi_target']:>6,} "
                               f"err={snap['spi_error']:>+6} "
                               f"corr={int(snap['spi_integ']):>+5}")
                print(f"{snap['t']:6.2f} {snap['state_name']:<10} "
                      f"{snap['throttle']:>4} {snap['duty']:>4}% "
                      f"{snap['eRPM']:>8,} "
                      f"{snap['vbus_V']:>4.1f}V {snap['ibus_A']:>+5.2f}A "
                      f"{snap['ia_pk_mag']:>5.1f}A "
                      f"{snap['fault_name']}{spi_tag} {marker}")
                last_print_t = snap['t']

    finally:
        if use_stream:
            try: send_cmd(ser, GSP_CMD_TELEM_STOP)
            except Exception: pass
        ser.close()
        if csv_file:
            csv_file.close()

        print()
        print(summarize(samples, info))
        if csv_path:
            print(f"\nCSV saved: {csv_path}")


if __name__ == "__main__":
    main()
