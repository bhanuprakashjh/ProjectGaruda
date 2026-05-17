#!/usr/bin/env python3
"""Minimal GSP smoke test for the AK port.

Usage:
    python3 gsp_ak_test.py [port] [baud]                     # smoke + 5 telem frames
    python3 gsp_ak_test.py mon  [port] [baud] [csv_path]     # live monitor + CSV log
    python3 gsp_ak_test.py mon  [port] [csv_path]            # baud defaults to 115200
    python3 gsp_ak_test.py bemf [port] [count]               # BEMF GPIO probe (hand-rotation)

`mon` mode now decodes every documented telemetry field, prints a one-line
summary per snapshot, and writes the full set as CSV on Ctrl-C. Default
CSV path is `ak_telem_<timestamp>.csv` in the current directory.

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

# Snapshot field layout — buf[] indices.
# Built from gsp_commands.c BuildV4Snapshot (snap[0..1] = seq prefix, then d[]
# from offset 0 → buf[2]). Keep this table in sync with that function.
SNAPSHOT_FIELDS = [
    # (key,            offset, struct_fmt, scale, unit, comment)
    ('seq',                0, '<H', 1,     '',     'sequence number'),
    ('state',              2, 'B',  1,     '',     'gV4State (0=IDLE..6=FLT)'),
    ('step',               4, 'B',  1,     '',     'commutation step 0..5'),
    ('potRaw',             6, '<H', 1,     '',     'pot ADC counts'),
    ('dutyPct',            8, 'B',  1,     '%',    'g_pwmActualDuty/g_pwmPer, 100=block-comm override'),
    ('cmdEn',              9, 'B',  1,     '',     't.commandEnabled (throttle gate)'),
    ('vbus_mV',           10, '<H', 1,     'mV',   'Vbus in millivolts (firmware-scaled)'),
    ('ia_mA',             12, '<h', 1,     'mA',   'phase A current, signed milliamps'),
    ('ib_mA',             14, '<h', 1,     'mA',   'phase B current, signed milliamps'),
    ('ibus_mA',           16, '<h', 1,     'mA',   'DC bus current, signed milliamps (direct OA3)'),
    ('actualAmp_Q15',     18, '<H', 1,     '',     'actualAmplitude Q15 (32768 = 100%)'),
    ('timerPeriod',       20, '<H', 1,     'HR',   'PI estimate of sector period'),
    ('eRpm',              24, '<I', 1,     '',     'electrical RPM'),
    ('sectors16',         28, '<H', 1,     '',     'low 16 bits of sectorCount'),
    ('diagLastCap',       30, '<H', 1,     'HR',   'last capValue from PI'),
    ('diagDelta',         32, '<h', 1,     'HR',   'PI delta (signed: target - measured)'),
    ('diagCaptures',      34, '<H', 1,     '',     'PI-fed captures (16-bit rolling)'),
    ('diagPiRuns',        36, '<H', 1,     '',     'PI run counter'),
    ('stallCnt',          38, 'B',  1,     '',     'stall counter (trip at V4_STALL_THRESHOLD)'),
    # d[37] = bit0:spMode  bit1:spRequest  bit2:g_blockCommActive
    ('spBcByte',          39, 'B',  1,     '',     'SP/BC packed flags (firmware d[37])'),
    ('erpmNow16',         40, '<H', 1,     '',     'erpmNow derived from timerPeriod'),
    ('systemTick',        42, '<I', 1,     '',     '50µs Timer1 tick'),
    ('uptime_s',          46, '<I', 1,     's',    'uptime seconds'),
    ('adcBlankRej',       50, '<I', 1,     '',     'ADC samples rejected by blanking'),
    ('adcStateMis',       54, '<I', 1,     '',     'ADC samples: comp != expected'),
    ('adcCapSet',         58, '<I', 1,     '',     'ADC captures (cR + cF total)'),
    ('adcSetRising',      62, '<I', 1,     '',     'ADC captures on rising sectors (cR)'),
    ('offMidCap',         66, '<I', 1,     '',     'OFF-mid sampler captures (unused on AK)'),
    ('offMidMis',         70, '<I', 1,     '',     'OFF-mid sampler mismatches'),
    ('ptgFires',          74, '<I', 1,     '',     'V5.0 PTG ISR fire count'),
    ('ptgR_acc',          78, '<I', 1,     '',     'PTG rising-sector accept'),
    ('ptgR_rej',          82, '<I', 1,     '',     'PTG rising-sector reject'),
    ('ptgF_acc',          86, '<I', 1,     '',     'PTG falling-sector accept'),
    ('ptgF_rej',          90, '<I', 1,     '',     'PTG falling-sector reject'),
    ('pR_acc',            94, '<I', 1,     '',     'V5 shadow rising: comp==0 (POST for rising)'),
    ('pR_rej',            98, '<I', 1,     '',     'V5 shadow rising: comp==1 (PRE  for rising)'),
    ('pF_acc',           102, '<I', 1,     '',     'V5 shadow falling: comp==1 (POST for falling)'),
    ('pF_rej',           106, '<I', 1,     '',     'V5 shadow falling: comp==0 (PRE  for falling)'),
    ('tMeasHR',          110, '<H', 1,     'HR',   'measurement-PI tracker (raw measured Tsector)'),
    ('sectorCount32',    112, '<I', 1,     '',     'sectorCount full 32-bit'),
    ('iaPkMax_mA',       116, '<h', 1,     'mA',   'rolling max Ia in window, mA'),
    ('iaPkMin_mA',       118, '<h', 1,     'mA',   'rolling min Ia, mA'),
    ('ibPkMax_mA',       120, '<h', 1,     'mA',   'rolling max Ib, mA'),
    ('ibPkMin_mA',       122, '<h', 1,     'mA',   'rolling min Ib, mA'),
    # 2026-05-14 mechanism probe: elapsed snapshots from the plausibility
    # filter in sector_pi.c. Tell us *why* one polarity is rejected.
    # Compare elapRejR vs filterHR: rejR ≈ filterHR+small → drift; rejR
    # near 65535 → stale capture (wrap from prior sector). Compare aF vs
    # filterHR: aF < filterHR/2 means accepted polarity grabs early edge.
    ('elapAccR',         148, '<H', 1,     'HR',   'last accepted elapsed, rising'),
    ('elapAccF',         150, '<H', 1,     'HR',   'last accepted elapsed, falling'),
    ('elapRejR',         152, '<H', 1,     'HR',   'last rejected elapsed, rising'),
    ('elapRejF',         154, '<H', 1,     'HR',   'last rejected elapsed, falling'),
    ('filterHRLast',     156, '<H', 1,     'HR',   'filterHR at last filter decision'),
    # Capture-layer probe: comp tally past blanking, by sector polarity.
    # PRE-ZC state on inverted ATA6847: rising→1, falling→0.
    # POST-ZC state:                     rising→0, falling→1.
    ('cR_hi',            158, '<I', 1,     '',     'rising sector, comp=1 (pre-ZC)'),
    ('cR_lo',            162, '<I', 1,     '',     'rising sector, comp=0 (post-ZC)'),
    ('cF_hi',            166, '<I', 1,     '',     'falling sector, comp=1 (post-ZC)'),
    ('cF_lo',            170, '<I', 1,     '',     'falling sector, comp=0 (pre-ZC)'),
    # PTG postscale experiment: count of fires that bypassed
    # V4_ProcessBemfSample(). Effective sample rate = (ptgFires-ptgSkipped)/sec.
    ('ptgSkipped',       174, '<I', 1,     '',     'PTG postscaled-out fires'),
    # 2026-05-15 — per-sector hit counters from sector_pi.c. Tell us
    # whether all 6 commutation positions actually fire, or only half
    # (s0/2/4 stuck at 0 → position++ is +2 somewhere → pR=0% explained).
    ('sectHit0',         178, '<I', 1,     '',     'commutate hits at position 0'),
    ('sectHit1',         182, '<I', 1,     '',     'commutate hits at position 1'),
    ('sectHit2',         186, '<I', 1,     '',     'commutate hits at position 2'),
    ('sectHit3',         190, '<I', 1,     '',     'commutate hits at position 3'),
    ('sectHit4',         194, '<I', 1,     '',     'commutate hits at position 4'),
    ('sectHit5',         198, '<I', 1,     '',     'commutate hits at position 5'),
    # 2026-05-15 — multi-phase BEMF tally.
    #   total[6]:           per-sector post-blanking fires
    #   tally[6][3]:        per-sector × per-phase count of comp=1
    # Ratio comp=1/total per (sector, phase) tells us if the phase the
    # firmware *thinks* is floating in sector S matches what's actually
    # transitioning.  0% or 100% = driven phase (likely bug); 10..90% =
    # actually floating.
    # bTot/bS offsets follow the firmware's d[200..247] = snap[202..249].
    # The previous table had bTot0 at 200, which overlapped sectHit5's
    # high half and shifted every bemf-tally field by 2 — see fix in
    # gsp_commands.c that also extended snap[] to 252 to give fpStale a
    # legal home at snap[250..251].
    ('bTot0',            202, '<H', 1,     '',     'sector 0 post-blanking fires'),
    ('bTot1',            204, '<H', 1,     '',     'sector 1 post-blanking fires'),
    ('bTot2',            206, '<H', 1,     '',     'sector 2 post-blanking fires'),
    ('bTot3',            208, '<H', 1,     '',     'sector 3 post-blanking fires'),
    ('bTot4',            210, '<H', 1,     '',     'sector 4 post-blanking fires'),
    ('bTot5',            212, '<H', 1,     '',     'sector 5 post-blanking fires'),
    ('bS0A',             214, '<H', 1,     '',     'sector 0, phase A, comp=1 count'),
    ('bS0B',             216, '<H', 1,     '',     'sector 0, phase B, comp=1 count'),
    ('bS0C',             218, '<H', 1,     '',     'sector 0, phase C, comp=1 count'),
    ('bS1A',             220, '<H', 1,     '',     'sector 1, phase A, comp=1 count'),
    ('bS1B',             222, '<H', 1,     '',     'sector 1, phase B, comp=1 count'),
    ('bS1C',             224, '<H', 1,     '',     'sector 1, phase C, comp=1 count'),
    ('bS2A',             226, '<H', 1,     '',     'sector 2, phase A, comp=1 count'),
    ('bS2B',             228, '<H', 1,     '',     'sector 2, phase B, comp=1 count'),
    ('bS2C',             230, '<H', 1,     '',     'sector 2, phase C, comp=1 count'),
    ('bS3A',             232, '<H', 1,     '',     'sector 3, phase A, comp=1 count'),
    ('bS3B',             234, '<H', 1,     '',     'sector 3, phase B, comp=1 count'),
    ('bS3C',             236, '<H', 1,     '',     'sector 3, phase C, comp=1 count'),
    ('bS4A',             238, '<H', 1,     '',     'sector 4, phase A, comp=1 count'),
    ('bS4B',             240, '<H', 1,     '',     'sector 4, phase B, comp=1 count'),
    ('bS4C',             242, '<H', 1,     '',     'sector 4, phase C, comp=1 count'),
    ('bS5A',             244, '<H', 1,     '',     'sector 5, phase A, comp=1 count'),
    ('bS5B',             246, '<H', 1,     '',     'sector 5, phase B, comp=1 count'),
    ('bS5C',             248, '<H', 1,     '',     'sector 5, phase C, comp=1 count'),
    ('fpStale',          250, '<H', 1,     '',     'PTG fires where snapshot fp != table'),
]


def parse_snapshot(buf: bytes) -> dict:
    """Pull every field defined in SNAPSHOT_FIELDS into a dict. Missing tail
    bytes return 0 for that field."""
    out = {}
    for key, off, fmt, _scale, _unit, _comment in SNAPSHOT_FIELDS:
        size = struct.calcsize(fmt)
        if len(buf) >= off + size:
            out[key] = struct.unpack_from(fmt, buf, off)[0]
        else:
            out[key] = 0
    # Derived counters
    out['capRise']    = out['adcSetRising']
    out['capFall']    = out['adcCapSet'] - out['adcSetRising']
    out['amp_pct']    = (out['actualAmp_Q15'] * 100) / 32768 if out['actualAmp_Q15'] else 0
    # The firmware ships physical units directly (see garuda_config.h).  The
    # host only divides by 1000 to display volts / amps; it never knows the
    # shunt resistance, op-amp gain, or Vbus divider.  If the AK DIM is
    # reworked for a different sense gain, ONLY the firmware Q8 constants
    # change — this tool tracks it automatically.
    out['vbusV']      = out['vbus_mV']    / 1000.0
    out['ia_A']       = out['ia_mA']      / 1000.0
    out['ib_A']       = out['ib_mA']      / 1000.0
    out['ibus_A']     = out['ibus_mA']    / 1000.0
    out['iaPkMax_A']  = out['iaPkMax_mA'] / 1000.0
    out['iaPkMin_A']  = out['iaPkMin_mA'] / 1000.0
    out['ibPkMax_A']  = out['ibPkMax_mA'] / 1000.0
    out['ibPkMin_A']  = out['ibPkMin_mA'] / 1000.0
    pR_tot = out['pR_acc'] + out['pR_rej']
    pF_tot = out['pF_acc'] + out['pF_rej']
    out['pR_pct'] = (100 * out['pR_acc'] // pR_tot) if pR_tot else 0
    out['pF_pct'] = (100 * out['pF_acc'] // pF_tot) if pF_tot else 0
    # Capture-layer ratios. preR% = fraction of rising-sector samples that
    # see comp=1 (pre-ZC). High preR% (≥90) means rising BEMF never crossed
    # the (shifted) virtual neutral by the sample point — physics
    # asymmetry. Low preR% means comp is transitioning fine, but the
    # accept logic is failing.
    cR_tot = out['cR_hi'] + out['cR_lo']
    cF_tot = out['cF_hi'] + out['cF_lo']
    out['preR_pct']  = (100 * out['cR_hi'] // cR_tot) if cR_tot else 0
    out['postF_pct'] = (100 * out['cF_hi'] // cF_tot) if cF_tot else 0
    # Firmware flags from packed d[37]
    sp_bc = out.get('spBcByte', 0)
    out['spMode']     = bool(sp_bc & 0x01)
    out['spRequest']  = bool(sp_bc & 0x02)
    out['blockComm']  = bool(sp_bc & 0x04)
    return out


def decode_telem(buf: bytes) -> str:
    """One-line live decode of a telemetry frame payload."""
    if len(buf) < 48:
        return f"short frame ({len(buf)}B)"
    s = parse_snapshot(buf)
    name = ESC_STATE.get(s['state'], f"S{s['state']}")
    # Block-comm tag: read the actual firmware flag (d[37] bit 2), not the
    # old "dutyPct==100 and amp<32768" heuristic which missed BC at full-amp.
    duty_tag = " BC" if s.get('blockComm', False) else "PWM"
    # Phase peak in this snapshot window (|max| or |min|, whichever bigger).
    iaPk_A = max(abs(s['iaPkMax_A']), abs(s['iaPkMin_A']))
    ibPk_A = max(abs(s['ibPkMax_A']), abs(s['ibPkMin_A']))
    # Per-sector hit counters — diagnostic for "step always odd" anomaly.
    # If even (0/2/4) stay at 0 while odd (1/3/5) climb, position is
    # incrementing by 2 somewhere and rising-sector BEMF never runs.
    sh = [s.get(f'sectHit{i}', 0) for i in range(6)]
    sect_str = "/".join(_kfmt(x).strip() for x in sh)
    return (f"#{s['seq']:5d} {name} s{s['step']} "
            f"pot={s['potRaw']:4d} d={s['dutyPct']:3d}%{duty_tag} "
            f"amp={s['amp_pct']:5.1f}% "
            f"V={s['vbusV']:5.2f} "
            f"iaPk={iaPk_A:5.1f}A ibPk={ibPk_A:5.1f}A ibus={s['ibus_A']:+5.1f}A "
            f"RPM={s['eRpm']:7d} Tp={s['timerPeriod']:5d} "
            f"PIdlt={s['diagDelta']:+5d} "
            f"stl={s['stallCnt']:3d} cE={s['cmdEn']} "
            f"cR={_kfmt(s['capRise'])} cF={_kfmt(s['capFall'])} "
            f"pR={s['pR_pct']:3d}% pF={s['pF_pct']:3d}% | "
            f"aR{s['elapAccR']:5d} aF{s['elapAccF']:5d} "
            f"rR{s['elapRejR']:5d} rF{s['elapRejF']:5d} "
            f"fH{s['filterHRLast']:5d} | "
            f"preR{s['preR_pct']:3d}% postF{s['postF_pct']:3d}% "
            f"skip={_kfmt(s['ptgSkipped'])} "
            f"sect[{sect_str}]")

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

def live_monitor(port: str, baud: int, csv_path: str = None):
    """Stream telemetry frames forever; one line per frame + log every field
    to CSV at the end. Ctrl-C to stop."""
    import csv
    from datetime import datetime
    if csv_path is None:
        csv_path = f"ak_telem_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    print(f"Opening {port} @ {baud}  (live monitor — Ctrl-C to stop)")
    print(f"CSV log → {csv_path}")
    s = serial.Serial(port, baud, timeout=0.05)
    time.sleep(0.2); s.reset_input_buffer()
    s.write(frame(0x14)); s.flush()   # TELEM_START
    # Track pot min/max to help spot whether the wheel reaches the ADC at all
    pot_min, pot_max = 0xFFFF, 0
    rows = []  # collected snapshot dicts, written to CSV on exit
    field_keys = [k for (k, *_rest) in SNAPSHOT_FIELDS] + [
        'capRise', 'capFall', 'vbusV', 'amp_pct', 'pR_pct', 'pF_pct',
        'ia_A', 'ib_A', 'ibus_A',
        'iaPkMax_A', 'iaPkMin_A', 'ibPkMax_A', 'ibPkMin_A',
        'host_t_ms',
    ]
    t0 = time.time()
    first_frame = True
    try:
        for cmd, payload in read_frames(s, want_cmd=0x80, count=10**9, timeout_s=10**9):
            if first_frame:
                # 178 = pre per-sector counters
                # 202 = with per-sector hit probe at d[176..199]
                # 250 = multi-phase BEMF tally (firmware before the OOB fix
                #       wrote fpStale OOB — that build was technically UB)
                # 252 = current firmware: snap[] sized correctly, fpStale
                #       lands at snap[250..251], bTot/bS offsets repaired
                if len(payload) >= 252:
                    tag = "(NEW — fpStale fixed, bTot/bS offsets repaired)"
                elif len(payload) >= 250:
                    tag = "(OLD-250 — fpStale may be torn, bTot misaligned)"
                elif len(payload) >= 202:
                    tag = "(MID — sectHit probe but no multi-phase tally)"
                else:
                    tag = "(OLD — sectHit + tally fields will be 0)"
                print(f"first frame: {len(payload)} bytes {tag}")
                first_frame = False
            potRaw = payload[6] | (payload[7] << 8)
            pot_min = min(pot_min, potRaw); pot_max = max(pot_max, potRaw)
            # Parse all fields + display the compact line.
            snap = parse_snapshot(payload)
            snap['host_t_ms'] = int((time.time() - t0) * 1000)
            rows.append(snap)
            print(f"{decode_telem(payload)} pot[{pot_min}..{pot_max}]")
    except KeyboardInterrupt:
        pass
    finally:
        try: s.write(frame(0x15)); s.flush()
        except Exception: pass
        s.close()
        # Write CSV with EVERY field per snapshot. One row per telemetry frame.
        try:
            with open(csv_path, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=field_keys, extrasaction='ignore')
                w.writeheader()
                for r in rows:
                    w.writerow(r)
            print(f"\nstopped. wrote {len(rows)} rows to {csv_path}")
        except Exception as e:
            print(f"\nCSV write failed: {e}")
        print(f"final pot range: {pot_min}..{pot_max}")

        # ── Multi-phase BEMF tally (2026-05-15) ───────────────────────
        # For each (sector, phase), the firmware counts how often comp=1
        # past the blanking gate.  The firmware resets the tally after
        # every snapshot send, so each frame's bTot/bS values are a fresh
        # ~50 ms delta (avoids uint16 wrap at 60 kHz PWM × ~30 % blanking
        # — was hitting wrap inside a single sector before reset landed).
        # We SUM across all frames here to recover the run-total ratios.
        # The phase the code BELIEVES is floating (per commutationTable)
        # is annotated with [F].
        if rows:
            totals = [0] * 6
            cnts   = {f'bS{s}{p}': 0 for s in range(6) for p in 'ABC'}
            for r in rows:
                for s in range(6):
                    totals[s] += r.get(f'bTot{s}', 0)
                    for p in 'ABC':
                        cnts[f'bS{s}{p}'] += r.get(f'bS{s}{p}', 0)
            # Code's belief about which phase floats per sector — must
            # match commutation.c.  0=A, 1=B, 2=C.
            float_by_sector = {0: 2, 1: 0, 2: 1, 3: 2, 4: 0, 5: 1}
            polarity        = {0: 'R', 1: 'F', 2: 'R', 3: 'F', 4: 'R', 5: 'F'}
            print("\n=== Multi-phase BEMF tally — comp=1 ratios per (sector, phase) ===")
            print("       │   phase A   │   phase B   │   phase C   │  total")
            print("───────┼─────────────┼─────────────┼─────────────┼────────")
            for s in range(6):
                tot = totals[s] or 1
                ratios = []
                for p_idx, phase in enumerate('ABC'):
                    cnt = cnts[f'bS{s}{phase}']
                    pct = 100.0 * cnt / tot
                    # Clamp to avoid format-width overflow if the firmware
                    # ever produces cnt > tot (transient between reset and
                    # snapshot read — increment can land after memcpy).
                    if pct > 999.9: pct = 999.9
                    mark = '[F]' if float_by_sector[s] == p_idx else '   '
                    ratios.append(f"{pct:5.1f}% {mark}")
                print(f"  s{s} {polarity[s]} │ {ratios[0]}  │ {ratios[1]}  │ {ratios[2]}  │ {totals[s]:8d}")
            print("Reading: [F] = phase code BELIEVES floats.  Look for a [F]")
            print("phase that's stuck at 0% or 100% — that's a phase-mapping bug.")

            # Stale-floatingPhase summary.  Sum across all frames.
            fp_stale_total = sum(r.get('fpStale', 0) for r in rows)
            total_samples  = sum(totals)
            stale_pct      = (100.0 * fp_stale_total / total_samples) if total_samples else 0.0
            print(f"v4_floatingPhase stale count: {fp_stale_total} / {total_samples} "
                  f"samples ({stale_pct:.2f}%)")
            if fp_stale_total > 0:
                print("  → v4_floatingPhase disagreed with commutationTable[v4_currentSector]")
                print("    on at least one PTG fire.  Deglitched comp (via global) is reading")
                print("    a different pin than the multi-phase tally (via direct A/B/C reads).")

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
        # Accept: mon [port] [baud] [csv_path]
        args = sys.argv[2:]
        port = args[0] if len(args) >= 1 else '/dev/ttyACM1'
        baud = int(args[1]) if len(args) >= 2 and args[1].isdigit() else 115200
        # Anything that doesn't look like baud goes to csv_path.
        csv_path = None
        if len(args) >= 2 and not args[1].isdigit():
            csv_path = args[1]
        elif len(args) >= 3:
            csv_path = args[2]
        live_monitor(port, baud, csv_path)
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
