#!/usr/bin/env python3
"""
Offline analyzer for edge_burst_log.py CSVs.

Reads a burst CSV, filters out bad-quality steps, runs the cluster
extractor on each good step, and prints aggregate statistics for the
three candidate ZC estimators (first_dense_us, before_silence_us,
densest_mid_us) vs poll, broken down by polarity and speed bucket.

Usage:
  python3 tools/edge_burst_analyze.py edge_burst_YYYYMMDD_HHMMSS.csv
  python3 tools/edge_burst_analyze.py *.csv --min-edges 6 --gap 6 --blanking 20
"""

import argparse
import csv
import glob
import statistics
import sys


def extract_clusters(edges_us, gap_threshold_us):
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
        min_gap = min((c[j+1] - c[j] for j in range(count-1)), default=0.0)
        gap_to_next = clusters[i+1][0] - end if i + 1 < len(clusters) else None
        out.append({
            'start': start,
            'end': end,
            'span': span,
            'count': count,
            'min_gap': min_gap,
            'gap_to_next': gap_to_next,
        })
    return out


def zc_candidates(clusters, blanking_us, silence_threshold_us, dense_min_count):
    first_dense = None
    before_silence = None
    densest = None
    for c in clusters:
        if first_dense is None and c['start'] >= blanking_us and c['count'] >= dense_min_count:
            first_dense = c['start']
        if (c['gap_to_next'] is not None
                and c['gap_to_next'] >= silence_threshold_us
                and before_silence is None):
            before_silence = c['start']
        if densest is None or c['count'] > densest['count']:
            densest = c
    densest_mid = 0.5 * (densest['start'] + densest['end']) if densest else None
    return {
        'first_dense_us':    first_dense,
        'before_silence_us': before_silence,
        'densest_mid_us':    densest_mid,
    }


# ── PWM-phase-aware ZC detection ──────────────────────────────────────────

PWM_PERIOD_US = 25.0  # 40 kHz center-aligned → 25 µs full period


def pwm_cycle_analysis(edges_us, blanking_us=20.0, pwm_period=PWM_PERIOD_US):
    """Assign each edge to a PWM cycle and derive ZC candidates
    from per-cycle edge density.

    Returns a dict with:
        cycles        : dict[int → list[float]]  — edges per PWM cycle
        densest_cycle : int  — cycle index with most edges
        densest_zc_us : float — midpoint of that cycle's first and last edge
        last_active_us: float — midpoint of last cycle with edges before ≥2 empty
        density_shift_us: float — first cycle after blanking where edge count
                          drops vs the previous cycle (transition from "at
                          threshold" to "past threshold"). Uses edges-per-cycle
                          running comparison.
    All *_us values are µs from commutation. None if not determinable.
    """
    if not edges_us or len(edges_us) < 2:
        return None

    # Build PWM cycle bins. Cycle 0 starts at t=0 (commutation).
    # This uses absolute position in the step, not relative to first edge,
    # so that results are commutation-aligned and comparable across steps.
    cycles = {}
    for t in edges_us:
        if t < 0:
            continue
        idx = int(t / pwm_period)
        cycles.setdefault(idx, []).append(t)

    if not cycles:
        return None

    max_cycle = max(cycles.keys())

    # --- densest cycle (after blanking) ---
    densest_idx = None
    densest_count = 0
    for idx in sorted(cycles.keys()):
        cycle_start = idx * pwm_period
        if cycle_start < blanking_us:
            continue
        c = len(cycles[idx])
        if c > densest_count:
            densest_count = c
            densest_idx = idx

    densest_zc_us = None
    if densest_idx is not None:
        e = cycles[densest_idx]
        densest_zc_us = 0.5 * (e[0] + e[-1])

    # --- last active cycle before ≥2 empty cycles (silence onset) ---
    last_active_us = None
    for idx in sorted(cycles.keys()):
        cycle_start = idx * pwm_period
        if cycle_start < blanking_us:
            continue
        # Check if the next 2 cycle slots are both empty
        if (idx + 1) not in cycles and (idx + 2) not in cycles:
            e = cycles[idx]
            last_active_us = 0.5 * (e[0] + e[-1])
            break  # first occurrence

    # --- density drop: first cycle (after blanking) where count < prev ---
    # This detects the transition from "high sensitivity at threshold" to
    # "comparator moving away from threshold." The cycle BEFORE the drop
    # is the best ZC approximation.
    density_shift_us = None
    sorted_idxs = sorted(idx for idx in cycles if idx * pwm_period >= blanking_us)
    for i in range(1, len(sorted_idxs)):
        prev_idx = sorted_idxs[i - 1]
        cur_idx = sorted_idxs[i]
        # Only consider consecutive cycles (skip gaps — those are already silence)
        if cur_idx != prev_idx + 1:
            # Gap between cycles — the prev_idx IS the last active before silence
            e = cycles[prev_idx]
            density_shift_us = 0.5 * (e[0] + e[-1])
            break
        prev_count = len(cycles[prev_idx])
        cur_count = len(cycles[cur_idx])
        if cur_count < prev_count and prev_count >= 2:
            # Density dropped — prev cycle is the peak
            e = cycles[prev_idx]
            density_shift_us = 0.5 * (e[0] + e[-1])
            break

    # --- per-cycle edge count table (for debug/display) ---
    cycle_counts = []
    for idx in range(max_cycle + 1):
        cycle_counts.append(len(cycles.get(idx, [])))

    return {
        'cycles':           cycles,
        'cycle_counts':     cycle_counts,
        'densest_zc_us':    densest_zc_us,
        'last_active_us':   last_active_us,
        'density_shift_us': density_shift_us,
        'densest_count':    densest_count,
        'total_cycles':     max_cycle + 1,
        'active_cycles':    sum(1 for c in cycle_counts if c > 0),
    }


def parse_edges_us(cell):
    cell = (cell or '').strip()
    if not cell:
        return []
    return [float(x) for x in cell.split(';') if x]


def parse_float_or_none(v):
    if v is None:
        return None
    s = str(v).strip()
    if not s:
        return None
    try:
        return float(s)
    except ValueError:
        return None


def parse_int_or_zero(v):
    try:
        return int(v)
    except (ValueError, TypeError):
        return 0


def load_rows(paths):
    rows = []
    for path in paths:
        with open(path) as f:
            r = csv.DictReader(f)
            for row in r:
                rows.append(row)
    return rows


def stats_block(deltas):
    vs = [v for v in deltas if v is not None]
    if not vs:
        return None
    vs_sorted = sorted(vs)
    n = len(vs)
    return {
        'n':      n,
        'mean':   statistics.fmean(vs),
        'median': statistics.median(vs),
        'stdev':  statistics.pstdev(vs) if n > 1 else 0.0,
        'min':    vs_sorted[0],
        'max':    vs_sorted[-1],
        'p10':    vs_sorted[int(n * 0.10)],
        'p90':    vs_sorted[int(n * 0.90) if n > 1 else 0],
    }


def fmt(stats):
    if stats is None:
        return "no data"
    return (f"n={stats['n']:>4d}  "
            f"mean={stats['mean']:+7.1f}  "
            f"med={stats['median']:+7.1f}  "
            f"std={stats['stdev']:5.1f}  "
            f"p10={stats['p10']:+7.1f}  "
            f"p90={stats['p90']:+7.1f}  "
            f"min={stats['min']:+7.1f}  "
            f"max={stats['max']:+7.1f}")


def speed_bucket(eRpm):
    if eRpm is None or eRpm <= 0:
        return 'unknown'
    if eRpm < 20000:  return '  <20k'
    if eRpm < 40000:  return '20-40k'
    if eRpm < 60000:  return '40-60k'
    if eRpm < 80000:  return '60-80k'
    if eRpm < 100000: return '80-100k'
    return '>=100k'


def main():
    ap = argparse.ArgumentParser(description="Offline edge-burst cluster analyzer")
    ap.add_argument('csv_paths', nargs='+',
                    help="One or more CSVs from edge_burst_log.py (globs OK)")
    ap.add_argument('--gap', type=float, default=6.0,
                    help="Cluster gap threshold in µs (default 6)")
    ap.add_argument('--blanking', type=float, default=20.0,
                    help="Ignore dense clusters starting before this µs (default 20)")
    ap.add_argument('--silence', type=float, default=25.0,
                    help="Silence gap threshold in µs (default 25)")
    ap.add_argument('--dense', type=int, default=3,
                    help="Min edges for a 'dense' cluster (default 3)")
    ap.add_argument('--min-edges', type=int, default=5,
                    help="Skip steps with fewer raw edges (default 5)")
    ap.add_argument('--require-cl', action='store_true',
                    help="Only include steps with state=4 (CLOSED_LOOP)")
    args = ap.parse_args()

    # Expand globs
    paths = []
    for p in args.csv_paths:
        expanded = glob.glob(p)
        if expanded:
            paths.extend(expanded)
        else:
            paths.append(p)

    rows = load_rows(paths)
    if not rows:
        print("No rows loaded — check CSV paths.")
        sys.exit(1)

    print(f"Loaded {len(rows)} rows from {len(paths)} CSV(s)")
    print(f"Filters: gap={args.gap}  blanking={args.blanking}  silence={args.silence}  "
          f"dense={args.dense}  min_edges={args.min_edges}  "
          f"require_cl={args.require_cl}")
    print()

    good = []
    skip_reasons = {
        'no_edges': 0,
        'low_edges': 0,
        'timeout': 0,
        'no_poll': 0,
        'not_cl': 0,
    }

    for row in rows:
        was_timeout = parse_int_or_zero(row.get('was_timeout'))
        edge_count = parse_int_or_zero(row.get('edge_count'))
        state = parse_int_or_zero(row.get('state'))
        poll_us = parse_float_or_none(row.get('poll_us'))
        eRpm = parse_int_or_zero(row.get('eRpm'))

        if was_timeout:
            skip_reasons['timeout'] += 1
            continue
        if edge_count == 0:
            skip_reasons['no_edges'] += 1
            continue
        if edge_count < args.min_edges:
            skip_reasons['low_edges'] += 1
            continue
        if poll_us is None:
            skip_reasons['no_poll'] += 1
            continue
        if args.require_cl and state != 4:
            skip_reasons['not_cl'] += 1
            continue

        edges_us = parse_edges_us(row.get('edges_us'))
        clusters = extract_clusters(edges_us, args.gap)
        cands = zc_candidates(clusters, args.blanking, args.silence, args.dense)

        # PWM-phase-aware analysis
        pwm = pwm_cycle_analysis(edges_us, blanking_us=args.blanking)
        if pwm:
            cands['pwm_densest_us']    = pwm['densest_zc_us']
            cands['pwm_last_active_us'] = pwm['last_active_us']
            cands['pwm_density_shift_us'] = pwm['density_shift_us']
        else:
            cands['pwm_densest_us']    = None
            cands['pwm_last_active_us'] = None
            cands['pwm_density_shift_us'] = None

        good.append({
            'row_ref':   row,
            'poll_us':   poll_us,
            'candidates': cands,
            'polarity':  'R' if parse_int_or_zero(row.get('rising_zc')) else 'F',
            'eRpm':      eRpm,
            'bucket':    speed_bucket(eRpm),
            'edge_count': edge_count,
            'n_clusters': len(clusters),
        })

    total_skipped = sum(skip_reasons.values())
    print(f"Good: {len(good)} rows")
    print(f"Skipped: {total_skipped} rows  "
          f"(timeout={skip_reasons['timeout']}, "
          f"no_edges={skip_reasons['no_edges']}, "
          f"low_edges={skip_reasons['low_edges']}, "
          f"no_poll={skip_reasons['no_poll']}, "
          f"not_cl={skip_reasons['not_cl']})")
    print()

    if not good:
        print("No usable rows.")
        sys.exit(1)

    # Compute candidate-vs-poll deltas per step
    def deltas_for(rows_, key):
        out = []
        for r in rows_:
            v = r['candidates'][key]
            if v is None:
                out.append(None)
            else:
                out.append(r['poll_us'] - v)
        return out

    # --------------------------------------------------------- Overall
    print("=" * 96)
    print("OVERALL — poll − candidate (positive = poll is LATE vs the candidate)")
    print("=" * 96)
    for key in ('first_dense_us', 'before_silence_us', 'densest_mid_us', 'pwm_densest_us', 'pwm_last_active_us', 'pwm_density_shift_us'):
        s = stats_block(deltas_for(good, key))
        print(f"  {key:<20s} {fmt(s)}")
    print()

    # --------------------------------------------------------- By polarity
    for pol in ('R', 'F'):
        subset = [g for g in good if g['polarity'] == pol]
        if not subset:
            continue
        label = "RISING" if pol == 'R' else "FALLING"
        print(f"{label} (n={len(subset)})")
        for key in ('first_dense_us', 'before_silence_us', 'densest_mid_us', 'pwm_densest_us', 'pwm_last_active_us', 'pwm_density_shift_us'):
            s = stats_block(deltas_for(subset, key))
            print(f"  {key:<20s} {fmt(s)}")
        print()

    # --------------------------------------------------------- By speed bucket
    buckets_order = ['  <20k', '20-40k', '40-60k', '60-80k', '80-100k', '>=100k', 'unknown']
    any_bucket = False
    for bucket in buckets_order:
        subset = [g for g in good if g['bucket'] == bucket]
        if not subset:
            continue
        if not any_bucket:
            print("=" * 96)
            print("BY SPEED BUCKET")
            print("=" * 96)
            any_bucket = True
        print(f"{bucket} (n={len(subset)})")
        for key in ('first_dense_us', 'before_silence_us', 'densest_mid_us', 'pwm_densest_us', 'pwm_last_active_us', 'pwm_density_shift_us'):
            s = stats_block(deltas_for(subset, key))
            print(f"  {key:<20s} {fmt(s)}")
        print()

    # --------------------------------------------------------- By polarity × bucket
    print("=" * 96)
    print("BY POLARITY × SPEED BUCKET  (PWM-aware candidates)")
    print("=" * 96)
    for key in ('pwm_densest_us', 'pwm_last_active_us', 'pwm_density_shift_us'):
        print(f"\n  {key}:")
        print(f"  {'bucket':>10s} {'pol':>4s}  stats")
        for bucket in buckets_order:
            for pol in ('R', 'F'):
                subset = [g for g in good if g['bucket'] == bucket and g['polarity'] == pol]
                if not subset:
                    continue
                s = stats_block(deltas_for(subset, key))
                if s is None:
                    continue
                print(f"  {bucket:>10s} {pol:>4s}  {fmt(s)}")
    print()

    # --------------------------------------------------------- Candidate availability
    print("=" * 96)
    print("CANDIDATE AVAILABILITY (fraction of good steps where each candidate fired)")
    print("=" * 96)
    total = len(good)
    for key in ('first_dense_us', 'before_silence_us', 'densest_mid_us', 'pwm_densest_us', 'pwm_last_active_us', 'pwm_density_shift_us'):
        n = sum(1 for g in good if g['candidates'][key] is not None)
        pct = 100.0 * n / total if total else 0.0
        print(f"  {key:<20s} {n:>5d} / {total:<5d}  ({pct:5.1f}%)")
    print()


if __name__ == '__main__':
    main()
