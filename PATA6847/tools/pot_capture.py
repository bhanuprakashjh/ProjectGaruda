#!/usr/bin/env python3
"""
Pot capture — streams CK snapshot telemetry while you manually control
the pot. Records everything to CSV for later analysis.

Usage:
  python3 tools/pot_capture.py /dev/ttyACM1 [--duration 60] [--csv pot_run.csv]

Press Ctrl+C to stop. Motor start/stop via board buttons (SW1/SW2).
"""

import serial
import struct
import time
import sys
import argparse
from datetime import datetime

# GSP protocol
GSP_START = 0x02
CMD_PING = 0x00
CMD_GET_SNAPSHOT = 0x02
CMD_TELEM_START = 0x14
CMD_TELEM_STOP = 0x15

CK_CURRENT_SCALE = 1.049
CK_VBUS_SCALE = 1211.0

STATE_NAMES = ['IDLE', 'ARMED', 'ALIGN', 'OL_RAMP', 'CL', 'RECOVERY', 'FAULT']
FAULT_NAMES = ['NONE', 'OV', 'UV', 'STALL', 'DESYNC', 'START_TO', 'ATA6847']

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

def build_packet(cmd_id, payload=b''):
    pkt_len = 1 + len(payload)
    body = bytes([pkt_len, cmd_id]) + payload
    crc = crc16(body)
    return bytes([GSP_START]) + body + bytes([crc >> 8, crc & 0xFF])

def parse_packet(buf):
    while len(buf) >= 5:
        idx = buf.find(bytes([GSP_START]))
        if idx < 0:
            return None, b''
        if idx > 0:
            buf = buf[idx:]
        if len(buf) < 2:
            return None, buf
        pkt_len = buf[1]
        if pkt_len < 1 or pkt_len > 249:
            buf = buf[1:]
            continue
        total = 2 + pkt_len + 2
        if len(buf) < total:
            return None, buf
        body = buf[1:2 + pkt_len]
        crc_recv = (buf[2 + pkt_len] << 8) | buf[2 + pkt_len + 1]
        crc_calc = crc16(body)
        if crc_recv != crc_calc:
            import sys
            print(f"  [CRC FAIL] len={pkt_len} cmd=0x{buf[2]:02x} recv=0x{crc_recv:04x} calc=0x{crc_calc:04x}", file=sys.stderr)
            buf = buf[1:]
            continue
        cmd = buf[2]
        payload = buf[3:2 + pkt_len]
        return (cmd, payload), buf[total:]
    return None, buf

def decode_ck_snapshot(data):
    if len(data) < 48:
        return None
    s = {}
    s['state'] = data[0]
    s['fault'] = data[1]
    s['step'] = data[2]
    s['ataStatus'] = data[3]
    s['potRaw'] = struct.unpack_from('<H', data, 4)[0]
    s['dutyPct'] = data[6]
    s['zcSynced'] = data[7] != 0
    s['vbusRaw'] = struct.unpack_from('<H', data, 8)[0]
    s['iaRaw'] = struct.unpack_from('<h', data, 10)[0]
    s['ibRaw'] = struct.unpack_from('<h', data, 12)[0]
    s['ibusRaw'] = struct.unpack_from('<h', data, 14)[0]
    s['duty'] = struct.unpack_from('<H', data, 16)[0]
    s['stepPeriod'] = struct.unpack_from('<H', data, 18)[0]
    s['stepPeriodHR'] = struct.unpack_from('<H', data, 20)[0]
    s['eRpm'] = struct.unpack_from('<I', data, 22)[0]
    s['goodZc'] = struct.unpack_from('<H', data, 26)[0]
    s['zcInterval'] = struct.unpack_from('<H', data, 28)[0]
    s['prevZcInterval'] = struct.unpack_from('<H', data, 30)[0]
    s['icAccepted'] = struct.unpack_from('<H', data, 32)[0]
    s['icFalse'] = struct.unpack_from('<H', data, 34)[0]
    s['filterLevel'] = data[36]
    s['spMode'] = data[37]          # V4: SP mode active flag
    s['erpmTP'] = struct.unpack_from('<H', data, 38)[0]  # V4: eRPM from timerPeriod
    # s['ilimActive'] = data[39] != 0  # overwritten by erpmTP high byte
    s['systemTick'] = struct.unpack_from('<I', data, 40)[0]
    s['uptime'] = struct.unpack_from('<I', data, 44)[0]
    if len(data) >= 52:
        s['zcLatencyPct'] = data[48]
        s['zcBlankPct'] = data[49]
        s['zcBypassCount'] = struct.unpack_from('<H', data, 50)[0]
    else:
        s['zcLatencyPct'] = 0
        s['zcBlankPct'] = 0
        s['zcBypassCount'] = 0
    # V3 ZC diagnostics
    if len(data) >= 64:
        s['zcMode'] = data[52]
        s['actualForcedComm'] = data[53]
        s['zcTimeoutCount'] = struct.unpack_from('<H', data, 54)[0]
        s['risingZcCount'] = struct.unpack_from('<H', data, 56)[0]
        s['fallingZcCount'] = struct.unpack_from('<H', data, 58)[0]
        s['risingTimeouts'] = struct.unpack_from('<H', data, 60)[0]
        s['fallingTimeouts'] = struct.unpack_from('<H', data, 62)[0]
    else:
        s['zcMode'] = 0
        s['actualForcedComm'] = 0
        s['zcTimeoutCount'] = 0
        s['risingZcCount'] = 0
        s['fallingZcCount'] = 0
        s['risingTimeouts'] = 0
        s['fallingTimeouts'] = 0
    # V4 per-step 0..5 counters (88 bytes total)
    if len(data) >= 88:
        for i in range(6):
            s[f'stepAcc{i}'] = struct.unpack_from('<H', data, 64 + i*2)[0]
        for i in range(6):
            s[f'stepTO{i}'] = struct.unpack_from('<H', data, 76 + i*2)[0]
    else:
        for i in range(6):
            s[f'stepAcc{i}'] = 0
            s[f'stepTO{i}'] = 0
    # V5 raw comparator edge trace (112 bytes total)
    if len(data) >= 112:
        for i in range(6):
            s[f'stepFlips{i}'] = struct.unpack_from('<H', data, 88 + i*2)[0]
        for i in range(6):
            s[f'stepPolls{i}'] = struct.unpack_from('<H', data, 100 + i*2)[0]
    else:
        for i in range(6):
            s[f'stepFlips{i}'] = 0
            s[f'stepPolls{i}'] = 0
    # V6 raw corroboration & IC age diagnostics (118 bytes total)
    if len(data) >= 118:
        s['rawVetoCount'] = struct.unpack_from('<H', data, 112)[0]
        s['icAgeRejectCount'] = struct.unpack_from('<H', data, 114)[0]
        s['trackFallbackCount'] = struct.unpack_from('<H', data, 116)[0]
    else:
        s['rawVetoCount'] = 0
        s['icAgeRejectCount'] = 0
        s['trackFallbackCount'] = 0
    # V7 Phase 2: raw stability & timestamp source diagnostics (130 bytes)
    if len(data) >= 130:
        s['rawStableBlock'] = struct.unpack_from('<H', data, 118)[0]
        s['tsFromIc'] = struct.unpack_from('<H', data, 120)[0]
        s['tsFromRaw'] = struct.unpack_from('<H', data, 122)[0]
        s['tsFromClc'] = struct.unpack_from('<H', data, 124)[0]
        s['tsFromPoll'] = struct.unpack_from('<H', data, 126)[0]
        s['icLeadReject'] = struct.unpack_from('<H', data, 128)[0]
    else:
        s['rawStableBlock'] = 0
        s['tsFromIc'] = 0
        s['tsFromRaw'] = 0
        s['tsFromClc'] = 0
        s['tsFromPoll'] = 0
        s['icLeadReject'] = 0
    # V8 Scheduler margin diagnostics (134 bytes total)
    if len(data) >= 134:
        s['targetPastCount'] = struct.unpack_from('<H', data, 130)[0]
        s['schedMarginHR'] = struct.unpack_from('<h', data, 132)[0]  # signed
    else:
        s['targetPastCount'] = 0
        s['schedMarginHR'] = 0
    # V9 PLL predictor telemetry (152 bytes)
    if len(data) >= 152:
        s['predPhaseErrHR'] = struct.unpack_from('<h', data, 134)[0]
        s['predPhaseErrRxHR'] = struct.unpack_from('<h', data, 136)[0]
        s['predStepHR'] = struct.unpack_from('<H', data, 138)[0]
        s['predZcInWindow'] = struct.unpack_from('<H', data, 140)[0]
        s['predZcOutWindow'] = struct.unpack_from('<H', data, 142)[0]
        s['predLocked'] = data[144]
        s['predMissCount'] = data[145]
        s['predMinMarginHR'] = struct.unpack_from('<h', data, 146)[0]
        s['predRealZcDelayHR'] = struct.unpack_from('<H', data, 148)[0]
        s['predZcOffsetHR'] = struct.unpack_from('<H', data, 150)[0]
    else:
        s['predPhaseErrHR'] = 0
        s['predPhaseErrRxHR'] = 0
        s['predStepHR'] = 0
        s['predZcInWindow'] = 0
        s['predZcOutWindow'] = 0
        s['predLocked'] = 0
        s['predMissCount'] = 0
        s['predMinMarginHR'] = 0
        s['predRealZcDelayHR'] = 0
        s['predZcOffsetHR'] = 0
    # Gate readiness fields removed from snapshot — zero-fill for CSV compat
    s['winCandInGated'] = 0
    s['winCandOutGated'] = 0
    s['winOutEarly'] = 0
    s['winOutLate'] = 0
    s['gateActive'] = 0
    s['windowReject'] = 0
    s['windowRecovered'] = 0
    # V11 Step 3: predictive scheduling (shifted -14 by gate removal)
    if len(data) >= 172:
        s['predCommOwned'] = struct.unpack_from('<H', data, 152)[0]
        s['predictiveMode'] = data[154]
        s['handoffPending'] = data[155]
        s['predExitMiss'] = struct.unpack_from('<H', data, 156)[0]
        s['predExitTimeout'] = struct.unpack_from('<H', data, 158)[0]
        s['predEnter'] = struct.unpack_from('<H', data, 160)[0]
        s['predEntryLate'] = struct.unpack_from('<H', data, 162)[0]
        s['predVsReactiveDelta'] = struct.unpack_from('<h', data, 164)[0]
        s['deltaOkCount'] = data[166]
        s['entryScore'] = data[167]
        s['predIsrFired'] = struct.unpack_from('<H', data, 168)[0]
        s['predIsrEntries'] = struct.unpack_from('<H', data, 170)[0]
    else:
        s['predCommOwned'] = 0
        s['predictiveMode'] = 0
        s['handoffPending'] = 0
        s['predExitMiss'] = 0
        s['predExitTimeout'] = 0
        s['predVsReactiveDelta'] = 0
        s['deltaOkCount'] = 0
        s['entryScore'] = 0
        s['predEnter'] = 0
        s['predEntryLate'] = 0
        s['predIsrFired'] = 0
        s['predIsrEntries'] = 0
    # V12b: DPLL state (shifted -14 by gate removal)
    if len(data) >= 186:
        s['dpllPhaseBiasHR'] = struct.unpack_from('<h', data, 172)[0]
        s['dpllErrHR'] = struct.unpack_from('<h', data, 174)[0]
        s['dmaMeasUsed'] = struct.unpack_from('<H', data, 176)[0]
        s['dmaMeasReject'] = struct.unpack_from('<H', data, 178)[0]
        s['predCloseAgree'] = struct.unpack_from('<H', data, 180)[0]
        s['predCloseDisagree'] = struct.unpack_from('<H', data, 182)[0]
        s['dpllFallbackReason'] = data[184]
        s['measSource'] = data[185]
    else:
        s['dpllPhaseBiasHR'] = 0
        s['dpllErrHR'] = 0
        s['dmaMeasUsed'] = 0
        s['dmaMeasReject'] = 0
        s['predCloseAgree'] = 0
        s['predCloseDisagree'] = 0
        s['dpllFallbackReason'] = 0
        s['measSource'] = 0
    # Sector PI synchronizer telemetry (14 bytes, shifted -14)
    if len(data) >= 200:
        s['syncErrHR'] = struct.unpack_from('<h', data, 186)[0]
        s['syncT_hatHR'] = struct.unpack_from('<H', data, 188)[0]
        s['syncVsReactive'] = struct.unpack_from('<h', data, 190)[0]
        s['syncMode'] = data[192]
        s['syncGoodStreak'] = data[193]
        s['syncMissStreak'] = data[194]
        s['syncClusterCount'] = data[195]
        s['syncAccepts'] = struct.unpack_from('<H', data, 196)[0]
        s['syncMisses'] = struct.unpack_from('<H', data, 198)[0]
    else:
        s['syncErrHR'] = 0
        s['syncT_hatHR'] = 0
        s['syncVsReactive'] = 0
        s['syncMode'] = 0
        s['syncGoodStreak'] = 0
        s['syncMissStreak'] = 0
        s['syncClusterCount'] = 0
        s['syncAccepts'] = 0
        s['syncMisses'] = 0
    # V12: icBounce (shifted -14 by gate removal, net 0 from sync add)
    if len(data) >= 202:
        s['icBounce'] = struct.unpack_from('<H', data, 200)[0]
    else:
        s['icBounce'] = 0
    # V13: DMA shadow telemetry (24 bytes, net 0 offset shift)
    if len(data) >= 226:
        s['dmaStepCount']       = struct.unpack_from('<I', data, 202)[0]
        s['dmaMatchCount']      = struct.unpack_from('<I', data, 206)[0]
        s['dmaRingOverflow']    = struct.unpack_from('<I', data, 210)[0]
        s['dmaEdgesAvgX16']     = struct.unpack_from('<H', data, 214)[0]
        s['dmaEarlyVsPoll']     = struct.unpack_from('<h', data, 216)[0]
        s['dmaEarlyVsExp']      = struct.unpack_from('<h', data, 218)[0]
        s['dmaClosestVsExp']    = struct.unpack_from('<h', data, 220)[0]
        s['dmaPollVsExp']       = struct.unpack_from('<h', data, 222)[0]
        s['dmaLastEdgeCount']   = data[224]
        s['dmaLastFound']       = data[225]
    else:
        s['dmaStepCount'] = 0
        s['dmaMatchCount'] = 0
        s['dmaEdgesAvgX16'] = 0
        s['dmaRingOverflow'] = 0
        s['dmaEarlyVsPoll'] = 0
        s['dmaEarlyVsExp'] = 0
        s['dmaClosestVsExp'] = 0
        s['dmaPollVsExp'] = 0
        s['dmaLastEdgeCount'] = 0
        s['dmaLastFound'] = 0
    # V14: DMA-direct substitution counters (18 bytes starting at offset 212)
    # Fields: subCount (u32), subSkipGated (u32), subSkipRange (u32),
    #         lastCorrectionHR (i16), minCorrectionHR (i16),
    #         maxCorrectionHR (i16)
    if len(data) >= 244:
        s['dmaSubCount']        = struct.unpack_from('<I', data, 226)[0]
        s['dmaSubSkipGated']    = struct.unpack_from('<I', data, 230)[0]
        s['dmaSubSkipRange']    = struct.unpack_from('<I', data, 234)[0]
        s['dmaLastCorrectionHR'] = struct.unpack_from('<h', data, 238)[0]
        s['dmaMinCorrectionHR']  = struct.unpack_from('<h', data, 240)[0]
        s['dmaMaxCorrectionHR']  = struct.unpack_from('<h', data, 242)[0]
    else:
        s['dmaSubCount'] = 0
        s['dmaSubSkipGated'] = 0
        s['dmaSubSkipRange'] = 0
        s['dmaLastCorrectionHR'] = 0
        s['dmaMinCorrectionHR'] = 0
        s['dmaMaxCorrectionHR'] = 0
    # Derived
    s['iaMa'] = round(s['iaRaw'] * CK_CURRENT_SCALE)
    s['ibMa'] = round(s['ibRaw'] * CK_CURRENT_SCALE)
    s['ibusMa'] = round(s['ibusRaw'] * CK_CURRENT_SCALE)
    s['vbusV'] = round(s['vbusRaw'] / CK_VBUS_SCALE, 2)
    return s

def main():
    parser = argparse.ArgumentParser(description='Pot capture — stream telemetry while using pot')
    parser.add_argument('port', help='Serial port (e.g. /dev/ttyACM1)')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--duration', type=float, default=120, help='Max duration seconds (default 120)')
    parser.add_argument('--csv', type=str, default=None, help='Output CSV file')
    parser.add_argument('--rate', type=int, default=10, help='Telemetry rate Hz (default 10)')
    args = parser.parse_args()

    csv_file = args.csv or f'pot_capture_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.5)
    ser.reset_input_buffer()

    # Ping
    ser.write(build_packet(CMD_PING))
    time.sleep(0.3)
    buf = ser.read(4096)
    pkt, buf = parse_packet(buf)
    if not pkt or pkt[0] != CMD_PING:
        print("ERROR: No PING response")
        return

    # Start telemetry
    rate_ms = max(10, 1000 // args.rate)
    ser.write(build_packet(CMD_TELEM_START, struct.pack('<H', rate_ms)))
    time.sleep(0.3)
    # Drain any response
    resp = ser.read(4096)
    if resp:
        print(f"  Telem start response: {len(resp)} bytes")

    print(f"Pot Capture — {args.port} @ {args.baud}")
    print(f"  Telemetry: {1000//rate_ms} Hz, max {args.duration}s")
    print(f"  CSV: {csv_file}")
    print(f"  Control motor with SW1 (start) / SW2 (stop) and pot")
    print(f"  Press Ctrl+C to stop")
    print()
    ZC_MODES = ['ACQ', 'TRK', 'RCV']
    SYNC_MODES = ['.', 'S', 'O']  # OFF, SHADOW, OWNED
    print(f"{'Time':>7s} {'State':>8s} {'ZcM':>3s} {'eRPM':>7s} {'Duty':>4s} {'Vbus':>6s} {'SP':>2s} {'eTP':>6s} {'Mrgn':>5s} {'SErr':>5s} {'ST_h':>5s} {'Cap':>5s} {'SM':>2s} {'SGd':>3s} {'SAcc':>5s} {'Corr':>5s} {'Lkd':>3s}")
    print("-" * 120)

    rows = []
    buf = b''
    t0 = time.time()
    sample_count = 0

    try:
        while (time.time() - t0) < args.duration:
            chunk = ser.read(512)
            if chunk:
                buf += chunk

            # Poll snapshots manually every 100ms as fallback
            if (time.time() - t0) > 0.5 and (sample_count == 0 or not chunk):
                ser.write(build_packet(CMD_GET_SNAPSHOT))

            while True:
                pkt, buf = parse_packet(buf)
                if not pkt:
                    break
                cmd, payload = pkt

                # Accept both telem frames (0x80) and snapshot responses (0x02)
                snap_data = None
                if cmd == 0x80 and len(payload) >= 50:  # TELEM_FRAME (seq + snapshot)
                    snap_data = payload[2:]  # skip 2-byte seq counter
                elif cmd == CMD_GET_SNAPSHOT and len(payload) >= 48:
                    snap_data = payload

                if snap_data is None:
                    continue
                snap = decode_ck_snapshot(snap_data)
                if not snap:
                    continue

                sample_count += 1
                t = time.time() - t0

                state_str = STATE_NAMES[snap['state']] if snap['state'] < len(STATE_NAMES) else f"?{snap['state']}"
                zc_mode_str = ZC_MODES[snap['zcMode']] if snap['zcMode'] < len(ZC_MODES) else f"?{snap['zcMode']}"
                zc_lat = snap['zcLatencyPct']
                lat_str = " TO" if zc_lat == 255 else f"{zc_lat * 100 // 255:3d}%"
                fault_str = ""
                if snap['fault'] > 0:
                    fname = FAULT_NAMES[snap['fault']] if snap['fault'] < len(FAULT_NAMES) else f"?{snap['fault']}"
                    fault_str = f"  *** FAULT: {fname} ***"

                rfc = f"{snap['risingZcCount']:5d}/{snap['fallingZcCount']:<5d}"
                rft = f"{snap['risingTimeouts']:5d}/{snap['fallingTimeouts']:<5d}"
                lck = 'Y' if snap.get('predLocked', 0) else 'N'
                gat = 'Y' if snap.get('gateActive', 0) else 'N'
                locked = snap.get('predLocked', 0)
                corr = snap.get('dmaLastCorrectionHR', 0)
                serr = snap.get('syncErrHR', 0)
                sthat = snap.get('syncT_hatHR', 0)
                svr = snap.get('syncVsReactive', 0)
                smode = snap.get('syncMode', 0)
                sgood = snap.get('syncGoodStreak', 0)
                sacc = snap.get('syncAccepts', 0)
                sm_str = SYNC_MODES[smode] if smode < len(SYNC_MODES) else '?'
                capv = snap.get('syncVsReactive', 0)  # repurposed: carries capValueHR
                sp_str = 'SP' if snap.get('spMode', 0) else '  '
                etp = snap.get('erpmTP', 0)
                print(f"{t:7.1f} {state_str:>8s} {zc_mode_str:>3s} {snap['eRpm']:7d} {snap['dutyPct']:3d}% {snap['vbusV']:5.1f}V {sp_str:>2s} {etp:6d} {snap.get('sched_margin_hr', snap.get('schedMarginHR',0)):5d} {serr:+5d} {sthat:5d} {capv:5d} {sm_str:>2s} {sgood:3d} {sacc:5d} {corr:+5d} {'Y' if locked else '.':>3s}{fault_str}")

                rows.append({
                    'time': round(t, 3),
                    'state': snap['state'],
                    'fault': snap['fault'],
                    'eRpm': snap['eRpm'],
                    'duty_pct': snap['dutyPct'],
                    'pot_raw': snap['potRaw'],
                    'ibus_mA': snap['ibusMa'],
                    'ia_mA': snap['iaMa'],
                    'ib_mA': snap['ibMa'],
                    'vbus_V': snap['vbusV'],
                    'zc_synced': 1 if snap['zcSynced'] else 0,
                    'missed': snap.get('spMode', 0),
                    'forced': snap.get('erpmTP', 0),
                    'step_period': snap['stepPeriod'],
                    'step_period_hr': snap['stepPeriodHR'],
                    'filter_level': snap['filterLevel'],
                    'zc_latency_pct': snap['zcLatencyPct'],
                    'zc_blank_pct': snap['zcBlankPct'],
                    'zc_bypass_count': snap['zcBypassCount'],
                    'ic_accepted': snap['icAccepted'],
                    'ic_false': snap['icFalse'],
                    'zc_interval': snap['zcInterval'],
                    'good_zc': snap['goodZc'],
                    'zc_mode': snap['zcMode'],
                    'actual_forced_comm': snap['actualForcedComm'],
                    'zc_timeout_count': snap['zcTimeoutCount'],
                    'rising_zc_count': snap['risingZcCount'],
                    'falling_zc_count': snap['fallingZcCount'],
                    'rising_timeouts': snap['risingTimeouts'],
                    'falling_timeouts': snap['fallingTimeouts'],
                    'step_acc_0': snap['stepAcc0'], 'step_acc_1': snap['stepAcc1'],
                    'step_acc_2': snap['stepAcc2'], 'step_acc_3': snap['stepAcc3'],
                    'step_acc_4': snap['stepAcc4'], 'step_acc_5': snap['stepAcc5'],
                    'step_to_0': snap['stepTO0'], 'step_to_1': snap['stepTO1'],
                    'step_to_2': snap['stepTO2'], 'step_to_3': snap['stepTO3'],
                    'step_to_4': snap['stepTO4'], 'step_to_5': snap['stepTO5'],
                    'flips_0': snap['stepFlips0'], 'flips_1': snap['stepFlips1'],
                    'flips_2': snap['stepFlips2'], 'flips_3': snap['stepFlips3'],
                    'flips_4': snap['stepFlips4'], 'flips_5': snap['stepFlips5'],
                    'polls_0': snap['stepPolls0'], 'polls_1': snap['stepPolls1'],
                    'polls_2': snap['stepPolls2'], 'polls_3': snap['stepPolls3'],
                    'polls_4': snap['stepPolls4'], 'polls_5': snap['stepPolls5'],
                    'raw_veto': snap['rawVetoCount'],
                    'ic_age_reject': snap['icAgeRejectCount'],
                    'track_fallback': snap['trackFallbackCount'],
                    'raw_stable_block': snap['rawStableBlock'],
                    'ts_from_ic': snap['tsFromIc'],
                    'ts_from_raw': snap['tsFromRaw'],
                    'ts_from_clc': snap['tsFromClc'],
                    'ts_from_poll': snap['tsFromPoll'],
                    'ic_lead_reject': snap['icLeadReject'],
                    'target_past': snap['targetPastCount'],
                    'sched_margin_hr': snap['schedMarginHR'],
                    'pred_phase_err_hr': snap.get('predPhaseErrHR', 0),
                    'pred_phase_err_rx_hr': snap.get('predPhaseErrRxHR', 0),
                    'pred_step_hr': snap.get('predStepHR', 0),
                    'pred_real_zc_delay_hr': snap.get('predRealZcDelayHR', 0),
                    'pred_zc_in_window': snap.get('predZcInWindow', 0),
                    'pred_zc_out_window': snap.get('predZcOutWindow', 0),
                    'pred_locked': snap.get('predLocked', 0),
                    'pred_miss_count': snap.get('predMissCount', 0),
                    'pred_min_margin_hr': snap.get('predMinMarginHR', 0),
                    'pred_zc_offset_hr': snap.get('predZcOffsetHR', 0),
                    'win_cand_in_gated': snap.get('winCandInGated', 0),
                    'win_cand_out_gated': snap.get('winCandOutGated', 0),
                    'win_out_early': snap.get('winOutEarly', 0),
                    'win_out_late': snap.get('winOutLate', 0),
                    'gate_active': snap.get('gateActive', 0),
                    'window_reject': snap.get('windowReject', 0),
                    'window_recovered': snap.get('windowRecovered', 0),
                    'pred_comm_owned': snap.get('predCommOwned', 0),
                    'predictive_mode': snap.get('predictiveMode', 0),
                    'pred_exit_miss': snap.get('predExitMiss', 0),
                    'pred_exit_timeout': snap.get('predExitTimeout', 0),
                    'pred_vs_reactive_delta': snap.get('predVsReactiveDelta', 0),
                    'delta_ok_count': snap.get('deltaOkCount', 0),
                    'handoff_pending': snap.get('handoffPending', 0),
                    'entry_score': snap.get('entryScore', 0),
                    'pred_enter': snap.get('predEnter', 0),
                    'pred_entry_late': snap.get('predEntryLate', 0),
                    'pred_isr_fired': snap.get('predIsrFired', 0),
                    'pred_isr_entries': snap.get('predIsrEntries', 0),
                    'ic_bounce': snap.get('icBounce', 0),
                    'dma_step_count': snap.get('dmaStepCount', 0),
                    'dma_match_count': snap.get('dmaMatchCount', 0),
                    'dma_edges_avg_x16': snap.get('dmaEdgesAvgX16', 0),
                    'dma_ring_overflow': snap.get('dmaRingOverflow', 0),
                    'dma_early_vs_poll': snap.get('dmaEarlyVsPoll', 0),
                    'dma_early_vs_exp': snap.get('dmaEarlyVsExp', 0),
                    'dma_closest_vs_exp': snap.get('dmaClosestVsExp', 0),
                    'dma_poll_vs_exp': snap.get('dmaPollVsExp', 0),
                    'dma_last_edge_count': snap.get('dmaLastEdgeCount', 0),
                    'dma_last_found': snap.get('dmaLastFound', 0),
                    'dma_sub_count': snap.get('dmaSubCount', 0),
                    'dma_sub_correction_hr': snap.get('dmaLastCorrectionHR', 0),
                    'dma_min_correction_hr': snap.get('dmaMinCorrectionHR', 0),
                    'dma_max_correction_hr': snap.get('dmaMaxCorrectionHR', 0),
                    'dpll_phase_bias_hr': snap.get('dpllPhaseBiasHR', 0),
                    'dpll_err_hr': snap.get('dpllErrHR', 0),
                    'dma_meas_used': snap.get('dmaMeasUsed', 0),
                    'dma_meas_reject': snap.get('dmaMeasReject', 0),
                    'pred_close_agree': snap.get('predCloseAgree', 0),
                    'pred_close_disagree': snap.get('predCloseDisagree', 0),
                    'dpll_fallback_reason': snap.get('dpllFallbackReason', 0),
                    'meas_source': snap.get('measSource', 0),
                    'pred_phase_err_hr': snap.get('predPhaseErrHR', 0),
                    'pred_locked': snap.get('predLocked', 0),
                    'pred_step_hr': snap.get('predStepHR', 0),
                    'sync_err_hr': snap.get('syncErrHR', 0),
                    'sync_t_hat_hr': snap.get('syncT_hatHR', 0),
                    'sync_vs_reactive': snap.get('syncVsReactive', 0),
                    'sync_mode': snap.get('syncMode', 0),
                    'sync_good_streak': snap.get('syncGoodStreak', 0),
                    'sync_miss_streak': snap.get('syncMissStreak', 0),
                    'sync_cluster_count': snap.get('syncClusterCount', 0),
                    'sync_accepts': snap.get('syncAccepts', 0),
                    'sync_misses': snap.get('syncMisses', 0),
                })

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\nStopped by user")

    # Stop telemetry
    ser.write(build_packet(CMD_TELEM_STOP))
    time.sleep(0.1)
    ser.close()

    # Write CSV
    if rows:
        import csv
        keys = rows[0].keys()
        with open(csv_file, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
        print(f"\n{sample_count} samples saved to {csv_file}")

        # Print per-step breakdown from last sample
        last = rows[-1]
        print("\n=== Per-Step ZC Analysis (phase-pair vs polarity-class test) ===")
        print("Step | Phase | Polarity | PrevState | Accepted | Timeouts")
        print("-----|-------|----------|-----------|----------|--------")
        step_info = [
            (0, 'C', 'Rising',  'B=LOW'),
            (1, 'A', 'Falling', 'A=PWM'),
            (2, 'B', 'Rising',  'A=LOW'),
            (3, 'C', 'Falling', 'B=PWM'),
            (4, 'A', 'Rising',  'A=LOW'),
            (5, 'B', 'Falling', 'A=PWM'),
        ]
        for i, (step, phase, pol, prev) in enumerate(step_info):
            acc = int(last.get(f'step_acc_{i}', last.get(f'stepAcc{i}', 0)))
            to = int(last.get(f'step_to_{i}', last.get(f'stepTO{i}', 0)))
            print(f"  {step}  |   {phase}   | {pol:<8s} | {prev:<9s} | {acc:>8d} | {to:>7d}")
        # Summary
        rising_acc = sum(int(last.get(f'step_acc_{i}', last.get(f'stepAcc{i}', 0))) for i in [0,2,4])
        falling_acc = sum(int(last.get(f'step_acc_{i}', last.get(f'stepAcc{i}', 0))) for i in [1,3,5])
        rising_to = sum(int(last.get(f'step_to_{i}', last.get(f'stepTO{i}', 0))) for i in [0,2,4])
        falling_to = sum(int(last.get(f'step_to_{i}', last.get(f'stepTO{i}', 0))) for i in [1,3,5])
        print(f"\nRising  (0,2,4 = float-after-LOW): acc={rising_acc} to={rising_to}")
        print(f"Falling (1,3,5 = float-after-PWM): acc={falling_acc} to={falling_to}")
        if rising_acc + falling_acc > 0:
            # Check phase pairs
            print("\nPhase pairs (0/3=C, 1/4=A, 2/5=B):")
            for name, a, b in [('C', 0, 3), ('A', 1, 4), ('B', 2, 5)]:
                pa = int(last.get(f'step_acc_{a}', last.get(f'stepAcc{a}', 0))) + int(last.get(f'step_acc_{b}', last.get(f'stepAcc{b}', 0)))
                pt = int(last.get(f'step_to_{a}', last.get(f'stepTO{a}', 0))) + int(last.get(f'step_to_{b}', last.get(f'stepTO{b}', 0)))
                print(f"  Phase {name} (steps {a},{b}): acc={pa} to={pt}")

        # Raw comparator edge trace summary
        total_flips = sum(int(last.get(f'flips_{i}', 0)) for i in range(6))
        total_polls = sum(int(last.get(f'polls_{i}', 0)) for i in range(6))
        if total_polls > 0:
            print("\n=== Raw Comparator Edge Trace ===")
            print("Step | Phase | Polarity | Flips | Polls  | Flip Rate")
            print("-----|-------|----------|-------|--------|----------")
            for i, (step, phase, pol, prev) in enumerate(step_info):
                flips = int(last.get(f'flips_{i}', 0))
                polls = int(last.get(f'polls_{i}', 0))
                rate = f"{flips*100/polls:.1f}%" if polls > 0 else "N/A"
                print(f"  {step}  |   {phase}   | {pol:<8s} | {flips:>5d} | {polls:>6d} | {rate:>8s}")
            rising_flips = sum(int(last.get(f'flips_{i}', 0)) for i in [0,2,4])
            falling_flips = sum(int(last.get(f'flips_{i}', 0)) for i in [1,3,5])
            rising_polls = sum(int(last.get(f'polls_{i}', 0)) for i in [0,2,4])
            falling_polls = sum(int(last.get(f'polls_{i}', 0)) for i in [1,3,5])
            print(f"\nRising  flips={rising_flips} polls={rising_polls} rate={rising_flips*100/rising_polls:.1f}%" if rising_polls > 0 else "")
            print(f"Falling flips={falling_flips} polls={falling_polls} rate={falling_flips*100/falling_polls:.1f}%" if falling_polls > 0 else "")
    else:
        print("\nNo data captured")

if __name__ == '__main__':
    main()
