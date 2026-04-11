#!/usr/bin/env python3
"""
DMA edge burst dump — research tool.

Arms the firmware's one-shot burst capture, waits until it's full, downloads
the 12 captured steps, and renders each as a text timeline showing every raw
DMA edge relative to commutation, plus markers for the poll-accepted ZC and
the predicted ZC.

Usage:
  python3 tools/edge_burst_dump.py /dev/ttyACM1

Expects the motor to already be running in CL at a stable speed when armed.
"""

import serial
import struct
import sys
import time
import argparse

# GSP protocol -------------------------------------------------------------
GSP_START            = 0x02
CMD_PING             = 0x00
CMD_BURST_ARM        = 0x20
CMD_BURST_STATUS     = 0x21
CMD_BURST_GET_STEP   = 0x22

# 640 ns per HR tick (SCCP4, Fp/64)
HR_TICK_NS  = 640
NS_PER_US   = 1000.0

# Per-step struct layout (must match DMA_BURST_STEP_T in hal_dma_burst.h)
#   uint16 commHR
#   uint16 pollHR
#   uint16 predictedHR
#   uint16 edges[20]
#   uint8  edgeCount
#   uint8  stepIndex
#   uint8  risingZc
#   uint8  wasTimeout
BURST_EDGES = 32
BURST_STEP_FMT  = f'<HHH{BURST_EDGES}HBBBB'
BURST_STEP_SIZE = struct.calcsize(BURST_STEP_FMT)  # 74
BURST_STEPS     = 12

PHASE_NAMES    = ['C', 'A', 'B', 'C', 'A', 'B']
POLARITY_NAMES = ['Rising', 'Falling', 'Rising', 'Falling', 'Rising', 'Falling']


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
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
        if crc16(body) != crc_recv:
            buf = buf[1:]
            continue
        cmd = buf[2]
        payload = buf[3:2 + pkt_len]
        return (cmd, payload), buf[total:]
    return None, buf


def send_and_wait(ser, cmd_id, payload=b'', expected_cmd=None, timeout=1.0):
    """Send a command and wait for its response."""
    ser.reset_input_buffer()
    ser.write(build_packet(cmd_id, payload))
    deadline = time.time() + timeout
    buf = b''
    target = expected_cmd if expected_cmd is not None else cmd_id
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
        while True:
            pkt, buf = parse_packet(buf)
            if not pkt:
                break
            cmd, resp = pkt
            if cmd == target:
                return resp
    return None


def decode_step(raw):
    fields = struct.unpack_from(BURST_STEP_FMT, raw, 0)
    commHR       = fields[0]
    pollHR       = fields[1]
    predictedHR  = fields[2]
    edges        = list(fields[3:3 + BURST_EDGES])
    edgeCount    = fields[3 + BURST_EDGES]
    stepIndex    = fields[4 + BURST_EDGES]
    risingZc     = fields[5 + BURST_EDGES]
    wasTimeout   = fields[6 + BURST_EDGES]
    return {
        'commHR':     commHR,
        'pollHR':     pollHR,
        'predictedHR': predictedHR,
        'edges':      edges[:edgeCount],
        'edgeCount':  edgeCount,
        'stepIndex':  stepIndex,
        'risingZc':   bool(risingZc),
        'wasTimeout': bool(wasTimeout),
    }


def hr_delta_us(tick_a, tick_b):
    """Signed (tick_a - tick_b) in µs, wrap-safe for 16-bit."""
    d = (tick_a - tick_b) & 0xFFFF
    if d >= 0x8000:
        d -= 0x10000
    return d * HR_TICK_NS / NS_PER_US


def extract_clusters(edges_us, gap_threshold_us=6.0):
    """Split a time-ordered list of edge times into clusters.

    Adjacent edges closer than `gap_threshold_us` belong to the same
    cluster. Returns a list of dicts, one per cluster:
        {
            'start':       first edge time (µs from commutation),
            'end':         last edge time,
            'span':        end - start,
            'count':       number of edges,
            'min_gap':     smallest intra-cluster gap (0 for single),
            'gap_to_next': gap to the next cluster's start, or None,
            'edges':       list of edge times in the cluster,
        }
    """
    if not edges_us:
        return []
    clusters = []
    cur = [edges_us[0]]
    for t in edges_us[1:]:
        if (t - cur[-1]) <= gap_threshold_us:
            cur.append(t)
        else:
            clusters.append(cur)
            cur = [t]
    clusters.append(cur)

    out = []
    for i, c in enumerate(clusters):
        start = c[0]
        end = c[-1]
        span = end - start
        count = len(c)
        if count > 1:
            min_gap = min(c[j + 1] - c[j] for j in range(count - 1))
        else:
            min_gap = 0.0
        gap_to_next = clusters[i + 1][0] - end if i + 1 < len(clusters) else None
        out.append({
            'start': start,
            'end': end,
            'span': span,
            'count': count,
            'min_gap': min_gap,
            'gap_to_next': gap_to_next,
            'edges': c,
        })
    return out


def zc_candidates(clusters, blanking_us=12.0, silence_threshold_us=25.0, dense_min_count=3):
    """Derive 2-3 candidate ZC estimates from cluster analysis.

    Returns a dict with keys:
        first_dense_us:      first cluster with count >= dense_min_count
                             that starts after blanking. Approximates
                             "threshold-dithering onset".
        before_silence_us:   start of the last cluster preceding a gap
                             >= silence_threshold_us. Approximates
                             "crossing confirmed → comparator settles".
        densest_mid_us:      midpoint of the cluster with the highest
                             edge count (tiebreak: earliest start).
                             Approximates "peak of dithering".
        All values are µs from commutation, or None if no candidate.
    """
    first_dense = None
    before_silence = None
    densest = None

    for i, c in enumerate(clusters):
        if first_dense is None and c['start'] >= blanking_us and c['count'] >= dense_min_count:
            first_dense = c['start']
        if (c['gap_to_next'] is not None
                and c['gap_to_next'] >= silence_threshold_us
                and before_silence is None):
            before_silence = c['start']
        if densest is None or c['count'] > densest['count']:
            densest = c

    densest_mid = None
    if densest is not None:
        densest_mid = 0.5 * (densest['start'] + densest['end'])

    return {
        'first_dense_us':      first_dense,
        'before_silence_us':   before_silence,
        'densest_mid_us':      densest_mid,
    }


def analyze_step(step, gap_threshold_us=6.0):
    """Run cluster extraction + candidate estimators for one step.
    Returns a dict with clusters and candidates added to the step data.
    """
    edges_us = [hr_delta_us(e, step['commHR']) for e in step['edges']]
    clusters = extract_clusters(edges_us, gap_threshold_us)
    cands = zc_candidates(clusters)
    poll_us = hr_delta_us(step['pollHR'], step['commHR']) if step['pollHR'] != 0 else None
    pred_us = hr_delta_us(step['predictedHR'], step['commHR']) if step['predictedHR'] != 0 else None
    return {
        'edges_us': edges_us,
        'clusters': clusters,
        'candidates': cands,
        'poll_us': poll_us,
        'pred_us': pred_us,
        'polarity': 'R' if step['risingZc'] else 'F',
        'stepIndex': step['stepIndex'],
        'wasTimeout': step['wasTimeout'],
    }


def print_cluster_analysis(steps):
    print("\n" + "="*78)
    print("CLUSTER ANALYSIS — offline extractor (gap threshold 6 µs)")
    print("="*78)
    print("Per-step cluster breakdown:")
    print(f"  {'#':>3s} {'pol':>3s} {'clusters (start:count:span)':<44s} "
          f"{'first_dense':>12s} {'bef_silence':>12s} {'dens_mid':>10s}")

    rows = []
    for i, step in enumerate(steps):
        a = analyze_step(step)
        rows.append(a)
        cluster_str = " ".join(
            f"{c['start']:.0f}:{c['count']}:{c['span']:.1f}"
            for c in a['clusters']
        )
        if len(cluster_str) > 44:
            cluster_str = cluster_str[:41] + "..."
        fd = f"{a['candidates']['first_dense_us']:.1f}" \
            if a['candidates']['first_dense_us'] is not None else "   --"
        bs = f"{a['candidates']['before_silence_us']:.1f}" \
            if a['candidates']['before_silence_us'] is not None else "   --"
        dm = f"{a['candidates']['densest_mid_us']:.1f}" \
            if a['candidates']['densest_mid_us'] is not None else "   --"
        print(f"  {i:>3d} {a['polarity']:>3s} {cluster_str:<44s} "
              f"{fd:>12s} {bs:>12s} {dm:>10s}")

    # Latency summary vs poll
    print()
    print("Candidate-vs-poll deltas (poll − candidate; positive = poll is LATE):")
    print(f"  {'#':>3s} {'pol':>3s} {'poll':>7s} {'first_dense':>12s} "
          f"{'bef_silence':>12s} {'dens_mid':>10s}")
    for i, a in enumerate(rows):
        if a['poll_us'] is None:
            continue
        fd = a['candidates']['first_dense_us']
        bs = a['candidates']['before_silence_us']
        dm = a['candidates']['densest_mid_us']
        fd_delta = f"{a['poll_us'] - fd:+7.1f}" if fd is not None else "    --"
        bs_delta = f"{a['poll_us'] - bs:+7.1f}" if bs is not None else "    --"
        dm_delta = f"{a['poll_us'] - dm:+7.1f}" if dm is not None else "    --"
        print(f"  {i:>3d} {a['polarity']:>3s} {a['poll_us']:7.1f} "
              f"{fd_delta:>12s} {bs_delta:>12s} {dm_delta:>10s}")

    # Aggregate stats (all, rising, falling)
    def stats(values):
        vs = [v for v in values if v is not None]
        if not vs:
            return None
        vs_sorted = sorted(vs)
        return {
            'n': len(vs),
            'mean': sum(vs) / len(vs),
            'min': vs_sorted[0],
            'max': vs_sorted[-1],
            'median': vs_sorted[len(vs) // 2],
        }

    def group(name, rows_):
        print(f"\n{name} (n={len(rows_)}):")
        for key in ('first_dense_us', 'before_silence_us', 'densest_mid_us'):
            deltas = [r['poll_us'] - r['candidates'][key]
                      for r in rows_
                      if r['poll_us'] is not None
                      and r['candidates'][key] is not None]
            st = stats(deltas)
            if st is None:
                print(f"  {key:18s}: no data")
            else:
                print(f"  {key:18s}: n={st['n']} mean={st['mean']:+6.1f} "
                      f"med={st['median']:+6.1f} min={st['min']:+6.1f} "
                      f"max={st['max']:+6.1f} µs")

    group("ALL", rows)
    group("RISING", [r for r in rows if r['polarity'] == 'R'])
    group("FALLING", [r for r in rows if r['polarity'] == 'F'])

    # Cluster counts per step
    counts = [len(r['clusters']) for r in rows]
    if counts:
        print(f"\nClusters per step: mean={sum(counts)/len(counts):.1f} "
              f"min={min(counts)} max={max(counts)}")


def render_step(idx, step):
    print(f"\n── Step #{idx}  (commStep={step['stepIndex']} "
          f"phase={PHASE_NAMES[step['stepIndex']]} "
          f"polarity={POLARITY_NAMES[step['stepIndex']]}) ──")
    commHR      = step['commHR']
    pollHR      = step['pollHR']
    predictedHR = step['predictedHR']
    edges       = step['edges']

    if step['wasTimeout']:
        print("  *** TIMEOUT — no ZC accepted ***")
    print(f"  commHR      = {commHR:>6d}")
    if pollHR != 0:
        pdelta = hr_delta_us(pollHR, commHR)
        print(f"  pollHR      = {pollHR:>6d}  (+{pdelta:7.2f} µs from commutation)")
    else:
        print(f"  pollHR      =    (none)")
    if predictedHR != 0:
        xdelta = hr_delta_us(predictedHR, commHR)
        print(f"  predictedHR = {predictedHR:>6d}  (+{xdelta:7.2f} µs from commutation)")
    else:
        print(f"  predictedHR =    (none)")
    print(f"  edgeCount   = {step['edgeCount']}")
    if not edges:
        print("  (no edges)")
        return

    # Compute POLL and PRED offsets from commutation once
    poll_us_from_comm = hr_delta_us(pollHR, commHR) if pollHR != 0 else None
    pred_us_from_comm = hr_delta_us(predictedHR, commHR) if predictedHR != 0 else None

    # Build an ordered list of (time_us, label, marker_text) entries for
    # POLL and PRED so that when both fall in the same inter-edge gap we
    # emit them in time order.
    markers = []
    if pred_us_from_comm is not None:
        markers.append((pred_us_from_comm, 'PRED', '-- PRED --'))
    if poll_us_from_comm is not None:
        markers.append((poll_us_from_comm, 'POLL', '** POLL **'))
    markers.sort(key=lambda m: m[0])
    marker_idx = 0  # next marker to emit

    print()
    print(f"  {'#':>3s} {'raw HR':>7s} {'+µs from comm':>14s} {'Δ vs prev':>11s} marker")
    prev_us = None
    for i, e in enumerate(edges):
        us_from_comm = hr_delta_us(e, commHR)

        # Emit any markers that fall strictly before this edge (in
        # time-order). They share the gap between prev_us and this edge.
        while marker_idx < len(markers) and markers[marker_idx][0] <= us_from_comm:
            m_time, _m_label, m_text = markers[marker_idx]
            if prev_us is None:
                # Marker is before the very first edge — show absolute time
                print(f"  {'·':>3s} {'·':>7s} {m_time:>11.2f}  {'(before #0)':>11s}  {m_text}")
            else:
                dp = m_time - prev_us
                print(f"  {'·':>3s} {'·':>7s} {m_time:>11.2f}  {dp:+8.2f} µs  {m_text}")
            marker_idx += 1

        dprev_str = f"{us_from_comm - prev_us:+8.2f} µs" if prev_us is not None else ""
        print(f"  {i:>3d} {e:>7d} {us_from_comm:>11.2f}  {dprev_str:>11s}")
        prev_us = us_from_comm

    # Any remaining markers fall after the last edge
    while marker_idx < len(markers):
        m_time, _m_label, m_text = markers[marker_idx]
        print(f"  {'·':>3s} {'·':>7s} {m_time:>11.2f}  (after edges)  {m_text}")
        marker_idx += 1


def main():
    ap = argparse.ArgumentParser(description="DMA edge burst dump tool")
    ap.add_argument('port', help="Serial port (e.g. /dev/ttyACM1)")
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--wait', type=float, default=3.0,
                    help="Max seconds to wait for capture to fill")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(0.1)
    ser.reset_input_buffer()

    # Ping to confirm link
    print("Pinging firmware...")
    resp = send_and_wait(ser, CMD_PING)
    if resp is None:
        print("ERROR: no response to PING — is the firmware alive?")
        sys.exit(1)
    print(f"  PING OK ({len(resp)} bytes)")

    # Arm the burst
    print("\nArming burst capture...")
    resp = send_and_wait(ser, CMD_BURST_ARM)
    if resp is None:
        print("ERROR: no response to BURST_ARM")
        sys.exit(1)
    print("  Armed.")

    # Poll until full
    print(f"\nWaiting for capture to fill (timeout {args.wait:.1f}s)...")
    t0 = time.time()
    last_count = -1
    while time.time() - t0 < args.wait:
        resp = send_and_wait(ser, CMD_BURST_STATUS)
        if resp is None or len(resp) < 2:
            time.sleep(0.05)
            continue
        state, count = resp[0], resp[1]
        if count != last_count:
            print(f"  state={state} count={count}/{BURST_STEPS}")
            last_count = count
        if state == 2:  # FULL
            break
        time.sleep(0.05)
    else:
        print(f"  WARNING: timed out at count={last_count}. Motor may not be running.")
        if last_count <= 0:
            sys.exit(1)

    # Download all steps
    print(f"\nDownloading {BURST_STEPS} steps...")
    steps = []
    for i in range(BURST_STEPS):
        resp = send_and_wait(ser, CMD_BURST_GET_STEP, bytes([i]),
                             expected_cmd=CMD_BURST_GET_STEP)
        if resp is None:
            print(f"  ERROR: no response for step {i}")
            break
        if len(resp) < BURST_STEP_SIZE:
            print(f"  ERROR: short response for step {i}: {len(resp)} bytes "
                  f"(expected {BURST_STEP_SIZE})")
            break
        steps.append(decode_step(resp))

    # Render
    for i, step in enumerate(steps):
        render_step(i, step)

    # Offline cluster analysis
    print_cluster_analysis(steps)

    # Summary
    print("\n" + "="*70)
    print(f"SUMMARY — {len(steps)} steps captured")
    print("="*70)
    for i, s in enumerate(steps):
        ec = s['edgeCount']
        to = 'TIMEOUT' if s['wasTimeout'] else 'ok'
        commStep = s['stepIndex']
        pol = 'R' if s['risingZc'] else 'F'
        poll_us = hr_delta_us(s['pollHR'], s['commHR']) if s['pollHR'] else 0
        pred_us = hr_delta_us(s['predictedHR'], s['commHR']) if s['predictedHR'] else 0
        print(f"  #{i:2d} step={commStep} {pol} edges={ec:2d} "
              f"poll={poll_us:7.2f}µs pred={pred_us:7.2f}µs {to}")


if __name__ == '__main__':
    main()
