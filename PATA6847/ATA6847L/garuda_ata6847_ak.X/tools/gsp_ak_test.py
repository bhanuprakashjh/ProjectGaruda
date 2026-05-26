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
    ('state',              2, 'B',  1,     '',     'gEscState (0=IDLE..6=FAULT)'),
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
    ('stallCnt',          38, 'B',  1,     '',     'stall counter (trip at STALL_THRESHOLD)'),
    # d[37] = bit0:spMode  bit1:spRequest  bit2:g_blockCommActive
    ('spBcByte',          39, 'B',  1,     '',     'SP/BC packed flags (firmware d[37])'),
    ('erpmNow16',         40, '<H', 1,     '',     'erpmNow derived from timerPeriod'),
    ('systemTick',        42, '<I', 1,     '',     '50µs Timer1 tick'),
    ('uptime_s',          46, '<I', 1,     's',    'uptime seconds'),
    ('adcBlankRej',       50, '<I', 1,     '',     'ADC samples rejected by blanking'),
    ('adcStateMis',       54, '<I', 1,     '',     'ADC samples: comp != expected'),
    ('adcCapSet',         58, '<I', 1,     '',     'BEMF ISR captures (total accepted, all sectors)'),
    # adcSetRising removed from firmware (always 0 on this hardware — rising
    # sectors land POST-ZC, never reach the accept gate). Slot kept as zero
    # pad for snap-layout stability; counters that used to live at d[60..71]
    # are gone.
    ('pad_60',            62, '<I', 1,     '',     'reserved (was adcSetRising — removed)'),
    ('pad_64',            66, '<I', 1,     '',     'reserved zero pad'),
    ('pad_68',            70, '<I', 1,     '',     'reserved zero pad'),
    ('ptgFires',          74, '<I', 1,     '',     'PTG ISR fire count (heartbeat)'),
    # PTG per-polarity counters removed (always 0 in TelemGet — diagnostic
    # path that was never useful). Slots kept for layout stability.
    ('pad_76',            78, '<I', 1,     '',     'reserved (was ptgRisingAcc)'),
    ('pad_80',            82, '<I', 1,     '',     'reserved (was ptgRisingRej)'),
    ('pad_84',            86, '<I', 1,     '',     'reserved (was ptgFallingAcc)'),
    ('pad_88',            90, '<I', 1,     '',     'reserved (was ptgFallingRej)'),
    # Per-polarity PI-feed counters. The firmware populates these from
    # diagPiFedRising / diagPiMissRising / diagPiFedFalling /
    # diagPiMissFalling (sector_pi.c). Host computes pR_pct / pF_pct as
    # accepted/(accepted+missed) to show which polarities reach the PI.
    ('piFedRising',       94, '<I', 1,     '',     'PI feed: rising sectors accepted'),
    ('piMissRising',      98, '<I', 1,     '',     'PI feed: rising sectors dropped'),
    ('piFedFalling',     102, '<I', 1,     '',     'PI feed: falling sectors accepted'),
    ('piMissFalling',    106, '<I', 1,     '',     'PI feed: falling sectors dropped'),
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
    # Capture-layer probe: comp tally past blanking on falling sectors only.
    # On inverted ATA6847, falling sector POST-ZC = comp 1, PRE-ZC = comp 0.
    # The rising-sector counterparts were always ~0% High / 100% Low on this
    # hardware (sample lands POST-ZC of rising → comp 0) so they were
    # removed from the firmware. Slots kept as zero pad for stability.
    ('pad_156',          158, '<I', 1,     '',     'reserved (was compRising_High)'),
    ('pad_160',          162, '<I', 1,     '',     'reserved (was compRising_Low)'),
    ('cF_hi',            166, '<I', 1,     '',     'falling sector, comp=1 (POST-ZC)'),
    ('cF_lo',            170, '<I', 1,     '',     'falling sector, comp=0 (PRE-ZC)'),
    # PTG postscale: count of fires that bypassed ProcessBemfSample().
    # Effective BEMF-sample rate = (ptgFires - ptgSkipped) / second.
    ('ptgSkipped',       174, '<I', 1,     '',     'PTG fires bypassed by postscaler'),
    # Frame trimmed to 180 bytes (was 254) 2026-05-19 to reduce UART
    # congestion. Dropped fields lived past offset 200 and were only used
    # in the Ctrl-C summary, not the per-frame line.
    ('mainHb',           178, '<H', 1,     '',     'main-loop heartbeat (low16 of free-running counter)'),
    # PTG dual-sample mode byte (added 2026-05-20, frame grew 180 → 182).
    #   0 = SINGLE_OFF — only mid-OFF sample per PWM cycle (low duty)
    #   1 = DUAL       — mid-OFF + mid-ON alternating (mid duty, ~2× ISR rate)
    #   2 = SINGLE_ON  — only mid-ON sample per PWM cycle (high duty / BC)
    ('ptgMode',          180, '<B', 1,     '',     'PTG sample mode state machine'),
    # PG1DC + MPER read DIRECTLY from PWM peripheral registers (added
    # 2026-05-20, frame grew 182 → 186). Lets the host verify the gate
    # is actually seeing the duty% the firmware reports. True gate
    # duty% = pg1dc / mper × 100. PG counter clock is 800 MHz
    # (= 8 × FPGX_CLK_HZ = 8 × 100 MHz), so 1 tick = 1.25 ns.
    ('pg1dc',            182, '<H', 1,     '',     'PG1DC compare register (PG-clock ticks)'),
    ('mper',             184, '<H', 1,     '',     'MPER period register (PG-clock ticks)'),
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
    # Derived counters. adcSetRising was removed from firmware (always 0 on
    # this hardware), so capRise is hardcoded 0 and capFall == adcCapSet.
    # The display labels stay the same for backwards compatibility.
    out['capRise']    = 0
    out['capFall']    = out['adcCapSet']
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
    # Per-polarity PI-feed ratios. pR_pct = fraction of rising sectors that
    # got an accepted BEMF capture into the PI. pF_pct = same for falling.
    # On AK with the polarity-symmetric gate, rising-sector samples land
    # POST-ZC and don't satisfy `comp == expected`, so pR_pct stays at 0%
    # by physics — that's normal, not a bug. pF_pct ~100% confirms the PI
    # is being fed reliably.
    pR_tot = out['piFedRising']  + out['piMissRising']
    pF_tot = out['piFedFalling'] + out['piMissFalling']
    out['pR_pct'] = (100 * out['piFedRising']  // pR_tot) if pR_tot else 0
    out['pF_pct'] = (100 * out['piFedFalling'] // pF_tot) if pF_tot else 0
    # Capture-layer ratios. preR is hardcoded 0 on AK (compRising counters
    # were removed from firmware — rising-sector samples always land POST-ZC
    # of rising, never PRE-ZC, so the rising tally carried no information).
    # postF = fraction of falling-sector samples that land POST-ZC (comp=1
    # on inverted ATA6847). Typically 80-95% across the speed range.
    cF_tot = out['cF_hi'] + out['cF_lo']
    out['preR_pct']  = 0
    out['postF_pct'] = (100 * out['cF_hi'] // cF_tot) if cF_tot else 0
    # True gate duty% read straight from PWM registers, independent of the
    # firmware-side g_pwmActualDuty mirror. Useful for confirming that the
    # reported amp% actually reaches the gate. PG clock is 800 MHz, so the
    # tick value also gives an absolute on-time in ns (= pg1dc × 1.25).
    if out.get('mper', 0) > 0:
        out['gate_duty_pct'] = 100.0 * out['pg1dc'] / out['mper']
        out['pwm_period_us'] = out['mper'] * 1.25e-3   # ticks × 1.25 ns / 1000
        out['gate_on_ns']    = out['pg1dc'] * 1.25
    else:
        out['gate_duty_pct'] = 0.0
        out['pwm_period_us'] = 0.0
        out['gate_on_ns']    = 0.0
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
    def _ptg_mode_tag(v):
        if v == 0:    return 'oFF'
        elif v == 1:  return 'DUAL'
        elif v == 2:  return 'on '
        return f"?{v:02x}"

    duty_tag = " BC" if s.get('blockComm', False) else "PWM"
    # Phase peak in this snapshot window (|max| or |min|, whichever bigger).
    iaPk_A = max(abs(s['iaPkMax_A']), abs(s['iaPkMin_A']))
    ibPk_A = max(abs(s['ibPkMax_A']), abs(s['ibPkMin_A']))
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
            f"hb={s.get('mainHb', 0):5d} "
            f"M={_ptg_mode_tag(s.get('ptgMode', 0xFF))} "
            f"gate={s.get('gate_duty_pct', 0):5.1f}% "
            f"PG1DC={s.get('pg1dc', 0):5d}/{s.get('mper', 0):5d}")

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
                # 180 = current firmware (trimmed): heartbeat at snap[178..179]
                # 254 = older firmware: full snapshot with bemf tally
                # 252 = older firmware: fpStale at snap[250..251]
                if len(payload) >= 254:
                    tag = "(OLD-254 — full snap with bemfTally; trimmed in newer fw)"
                elif len(payload) >= 186:
                    tag = "(NEW-186 — adds PG1DC/MPER at snap[182..185] for gate-duty verification)"
                elif len(payload) >= 182:
                    tag = "(NEW-182 — PTG sample-mode byte at snap[180]; PG1DC/MPER missing — flash newer fw)"
                elif len(payload) >= 180:
                    tag = "(NEW-180 — trimmed snap, ptgMode missing — flash newer fw)"
                elif len(payload) >= 252:
                    tag = "(OLD-252 — no heartbeat, hb=0)"
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
            # HWZC diag piggyback — fire 0x90 every ~10 frames so we see
            # SCCP1 IC capture state while the motor is running, without
            # needing a second terminal.
            hwzc_line = ""
            if (len(rows) % 10) == 0:
                try:
                    s.write(frame(0x90)); s.flush()
                    time.sleep(0.02)
                    rx = s.read(s.in_waiting or 1)
                    time.sleep(0.02); rx += s.read(s.in_waiting)
                    parsed = parse_reply(rx)
                    if parsed and parsed[0] == 0x90 and len(parsed[1]) == 250:
                        d = parsed[1]
                        cnt = struct.unpack_from('<I', d, 0)[0]
                        pre, ov, xe = struct.unpack_from('<III', d, 4)
                        le, lb, be = struct.unpack_from('<HHH', d, 16)
                        a, _b, _c = struct.unpack_from('<III', d, 24)
                        sh = struct.unpack_from('<IIIIII', d, 36)
                        # s1-elapsed histogram: 16 u32 at offset 60.
                        # Bin n = lE in [n*128, (n+1)*128). lE > 2047 → wrap.
                        hist = struct.unpack_from('<16I', d, 60)
                        wrap = struct.unpack_from('<I', d, 124)[0]
                        drained = struct.unpack_from('<I', d, 128)[0]
                        garm, gos = struct.unpack_from('<II', d, 132)
                        # Phase 3b wrap-diagnosis fields
                        eih = struct.unpack_from('<16I', d, 140)
                        eihuge = struct.unpack_from('<I', d, 204)[0]
                        leiR, lciR, weHR, wlcHR, wiHR = struct.unpack_from(
                            '<HHHHH', d, 208)
                        # Phase 4-diag pair fields (offset 220)
                        llR, llF, lhR, lhF = struct.unpack_from('<HHHH', d, 220)
                        lvR, lvF, hvR, hvF = struct.unpack_from('<IIII', d, 228)
                        # Stage A OL fields (offset 244)
                        olIn, olOut, olLast = struct.unpack_from('<HHH', d, 244)
                        eih_s = "/".join(_kfmt(x) for x in eih)
                        hist_s = "/".join(_kfmt(x) for x in hist)
                        # Compact "pair" string: legacy vs hwzc latest elapsed per polarity.
                        # 0xFFFF = "no capture yet" → render as "-".
                        def _ll(v): return "-" if v == 0xFFFF else str(v)
                        hwzc_line = (f" | HW cnt={_kfmt(cnt)} prB={_kfmt(pre)}"
                                     f" xE={_kfmt(xe)} A={a} lE={le}"
                                     f" sec[{_kfmt(sh[0])}/{_kfmt(sh[1])}/{_kfmt(sh[2])}"
                                     f"/{_kfmt(sh[3])}/{_kfmt(sh[4])}/{_kfmt(sh[5])}]"
                                     f" lEh[{hist_s}] wrap={_kfmt(wrap)} drn={_kfmt(drained)}"
                                     f" gArm={_kfmt(garm)} gOS={_kfmt(gos)}"
                                     f" e2i={leiR} c2i={lciR}"
                                     f" eih[{eih_s}]+H{_kfmt(eihuge)}"
                                     f" W(e={weHR},c={wlcHR},n={wiHR})"
                                     f" pair[R lg={_ll(llR)}/hw={_ll(lhR)}"
                                     f" F lg={_ll(llF)}/hw={_ll(lhF)}]"
                                     f" pairN[R l={_kfmt(lvR)}/h={_kfmt(hvR)}"
                                     f" F l={_kfmt(lvF)}/h={_kfmt(hvF)}]"
                                     f" OL[in={olIn} out={olOut} lE={olLast}]")
                except Exception:
                    pass
            print(f"{decode_telem(payload)} pot[{pot_min}..{pot_max}]{hwzc_line}")
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

        # PTG sample-mode breakdown — was the dual-sample state machine
        # actually doing anything? Show how many frames spent in each mode.
        if rows:
            mode_names = {0: 'SINGLE_OFF', 1: 'DUAL', 2: 'SINGLE_ON'}
            mode_counts = {0: 0, 1: 0, 2: 0, 'other': 0}
            for r in rows:
                m = r.get('ptgMode', 0xff)
                if m in mode_counts:
                    mode_counts[m] += 1
                else:
                    mode_counts['other'] += 1
            tot = sum(mode_counts.values()) or 1
            print("\n=== PTG sample-mode breakdown ===")
            for k in (0, 1, 2):
                pct = 100.0 * mode_counts[k] / tot
                print(f"  {k}={mode_names[k]:11s}: {mode_counts[k]:4d} frames ({pct:5.1f}%)")
            if mode_counts['other'] > 0:
                print(f"  other (unknown): {mode_counts['other']} frames "
                      f"— firmware likely missing ptgMode byte; flash the 182-byte build")

        # (The bemfTally / fpStaleCount / sectorHits summary was removed
        # when the firmware dropped those counters and shrunk the snap to
        # 180 bytes on 2026-05-19. Their backing globals no longer exist;
        # the per-fire ISR cost they incurred is gone too.)

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

def hwzc_diag(port: str, baud: int, count: int = 1, interval_s: float = 0.2):
    """Poll GSP_CMD_HWZC_DIAG (0x90) and decode the 250-byte HWZC_DIAG_T.

    Wire layout (little-endian, 250 bytes):
      uint32 count, dropPreBlank, dropFifoOv, dropExtraEdge       [0-15]
      uint16 lastElapsed, lastRawBuf, blankingEndHR, pad           [16-23]
      uint32 phaseA, phaseB, phaseC                                [24-35]
      uint32 secHit[6]                                             [36-59]
      uint32 elapsedHist[16]   (lE: 128 HR/bin, 0-2047)            [60-123]
      uint32 lEhugeWrap                                            [124-127]
      uint32 fifoDrained                                           [128-131]
      uint32 gatedArms, gatedOneShot                               [132-139]
      uint32 edgeIsrHist[16]   (8 HR/bin, 0-127)                   [140-203]
      uint32 edgeIsrHuge       (count of edge_to_isr > 127 HR)     [204-207]
      uint16 lastEdgeToIsr, lastCommToIsr                          [208-211]
      uint16 wrapEdgeHR, wrapLastCommHR, wrapIsrNowHR, pad2        [212-219]
      uint16 latestLegacyElapsedRising/Falling                     [220-223]
      uint16 latestHwzcElapsedRising/Falling                       [224-227]
      uint32 legacyValidRising/Falling                             [228-235]
      uint32 hwzcValidRising/Falling                               [236-243]
      uint16 olCapInCorridor, olCapOutOfCorridor, olLastElapsedHR  [244-249]
    """
    s = serial.Serial(port, baud, timeout=0.5)
    time.sleep(0.2); s.reset_input_buffer()
    print(f"Opening {port} @ {baud}  — {count} HWZC diag read(s)")
    print(f"{'#':>4} {'count':>8} {'preBlnk':>8} {'xEdge':>6} "
          f"{'lastE':>5} {'lastBuf':>7} {'A':>5} "
          f"{'s0':>7} {'s1':>7} {'s2':>7} {'s3':>7} {'s4':>7} {'s5':>7}")
    for i in range(count):
        s.write(frame(0x90)); s.flush()
        time.sleep(0.1)
        rx = s.read(s.in_waiting or 1)
        time.sleep(0.05); rx += s.read(s.in_waiting)
        parsed = parse_reply(rx)
        if not parsed or parsed[0] != 0x90 or len(parsed[1]) != 250:
            print(f"  bad reply (len={len(parsed[1]) if parsed else 0}): {rx.hex()}")
        else:
            d = parsed[1]
            cnt, pre, ov, xe = struct.unpack_from('<IIII', d, 0)
            le, lb, be, _pad = struct.unpack_from('<HHHH', d, 16)
            pa, pb, pc = struct.unpack_from('<III', d, 24)
            sh = struct.unpack_from('<IIIIII', d, 36)
            hist = struct.unpack_from('<16I', d, 60)
            wrap = struct.unpack_from('<I', d, 124)[0]
            drained = struct.unpack_from('<I', d, 128)[0]
            garm, gos = struct.unpack_from('<II', d, 132)
            eih = struct.unpack_from('<16I', d, 140)
            eihuge = struct.unpack_from('<I', d, 204)[0]
            leiR, lciR = struct.unpack_from('<HH', d, 208)
            weHR, wlcHR, wiHR, _pad2 = struct.unpack_from('<HHHH', d, 212)
            # Phase 4-diag pair fields
            llR, llF, lhR, lhF = struct.unpack_from('<HHHH', d, 220)
            lvR, lvF, hvR, hvF = struct.unpack_from('<IIII', d, 228)
            # Stage A OL fields
            olIn, olOut, olLast = struct.unpack_from('<HHH', d, 244)
            print(f"{i:>4} {cnt:>8} {pre:>8} {xe:>6} "
                  f"{le:>5} {lb:>7} {pa:>5} "
                  f"{sh[0]:>7} {sh[1]:>7} {sh[2]:>7} {sh[3]:>7} {sh[4]:>7} {sh[5]:>7}")
            print(f"      fifoDrained = {drained}  gatedArms = {garm}  gatedOneShot = {gos}")
            # Phase 3b: edge_to_isr histogram (8 HR/bin → 0-127 HR)
            eih_total = sum(eih) + eihuge
            print(f"      edge_to_isr histogram (lastEdgeToIsr={leiR} HR, "
                  f"lastCommToIsr={lciR} HR, total={eih_total}):")
            for b, c in enumerate(eih):
                lo = b * 8
                hi = lo + 7
                label = f"{lo:>3}-{hi:<3}"
                pct = (100.0 * c / eih_total) if eih_total else 0.0
                bar = "#" * int(pct / 2)
                print(f"        [{b:>2}] {label} HR  {c:>7}  {pct:5.1f}%  {bar}")
            pct_h = (100.0 * eihuge / eih_total) if eih_total else 0.0
            bar_h = "#" * int(pct_h / 2)
            print(f"        [HU] >127   HR  {eihuge:>7}  {pct_h:5.1f}%  {bar_h}")
            # Wrap snapshot — compute the deltas by hand for the postmortem.
            wrap_lE      = (weHR - wlcHR) & 0xFFFF
            wrap_e2i     = (wiHR - weHR)  & 0xFFFF
            wrap_c2i     = (wiHR - wlcHR) & 0xFFFF
            print(f"      latest wrap snapshot:")
            print(f"        edgeHR={weHR}  lastCommHR={wlcHR}  isrNowHR={wiHR}")
            print(f"        wrap lE       = (edgeHR - lastCommHR) = {wrap_lE}  (expect huge if real wrap)")
            print(f"        edge_to_isr   = (isrNowHR - edgeHR)   = {wrap_e2i}  (small=ISR fast; huge=FIFO stale or timer mismatch)")
            print(f"        comm_to_isr   = (isrNowHR - lastComm) = {wrap_c2i}  (small=Commutate just fired; race)")
            # Phase 4-diag pair comparison: legacy vs hwzc elapsed per polarity.
            # When both paths see the same physical ZC, the two values should
            # be within a few HR of each other. Large divergence (one near 0
            # or near sector-end while the other is at T/2 + advance) means
            # the hwzc path is capturing chatter, not real BEMF.
            def _ll2(v): return "    -" if v == 0xFFFF else f"{v:>5}"
            print(f"      pair comparison (legacy vs hwzc elapsed, HR):")
            print(f"        Rising  : legacy={_ll2(llR)}  hwzc={_ll2(lhR)}  "
                  f"count[legacy={lvR}  hwzc={hvR}]")
            print(f"        Falling : legacy={_ll2(llF)}  hwzc={_ll2(lhF)}  "
                  f"count[legacy={lvF}  hwzc={hvF}]")
            # Stage A: OL-ramp ZC observation. olIn (corridor hits) is
            # the evidence the rotor is locked to the forced commutation.
            ol_total = olIn + olOut
            ol_ratio = (100.0 * olIn / ol_total) if ol_total else 0.0
            print(f"      OL-ramp ZC observation:")
            print(f"        in-corridor  = {olIn}  out-of-corridor = {olOut}  "
                  f"last elapsed = {olLast} HR  in-ratio = {ol_ratio:.1f}%")
            # Histogram visualization: bin label = HR range, count, bar
            total = sum(hist) + wrap
            in_range = sum(hist)
            print(f"      lE histogram (s1 only, in-range={in_range}, wrap={wrap}, total={total}):")
            for b, c in enumerate(hist):
                lo = b * 128
                hi = f"{lo+127}"
                label = f"{lo:>4}-{hi:<4}"
                pct = (100.0 * c / total) if total else 0.0
                bar = "#" * int(pct / 2)  # 50% = 25 chars
                print(f"        [{b:>2}] {label} HR  {c:>7}  {pct:5.1f}%  {bar}")
            pct_w = (100.0 * wrap / total) if total else 0.0
            bar_w = "#" * int(pct_w / 2)
            print(f"        [WR] >2047    HR  {wrap:>7}  {pct_w:5.1f}%  {bar_w}")
        if i < count - 1:
            time.sleep(interval_s)
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
    if len(sys.argv) > 1 and sys.argv[1] == "hwzc":
        port = sys.argv[2] if len(sys.argv) > 2 else '/dev/ttyACM1'
        baud = 115200
        cnt  = int(sys.argv[3]) if len(sys.argv) > 3 else 30
        hwzc_diag(port, baud, cnt)
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
