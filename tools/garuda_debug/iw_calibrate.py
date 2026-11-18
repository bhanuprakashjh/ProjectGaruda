#!/usr/bin/env python3
"""A2 — Iw channel calibration from a GarudaESE AN1078 telemetry CSV.

The A2 firmware streams the raw Iw ADC sample (AD3CH3, ATA op-amp) in the
dormant `fall_off_max` column, latched at the SAME PG1TRIGA instant as the
FOC's ia/ib. Because iu + iv + iw = 0 at every instant, the true Iw in amps is
    iw_ref = -(focIa + focIb)
so a linear fit of iw_ref against the raw count recovers the channel's
gain (A/count, sign included) and zero-offset — no bench meter needed.

Usage:
    python3 iw_calibrate.py <telemetry.csv>

Compares the recovered gain against the dsPIC OA1/OA2 gain (Iu/Iv) so we know
whether best-2-of-3 can share one scale or needs a separate Iw constant, and
checks whether the fit degrades at high duty (the window-collapse we're chasing).
"""
import sys, csv, math

DSPIC_A_PER_COUNT = 0.011078   # AN_CURRENT_A_PER_COUNT (Iu/Iv, dsPIC OA, INVERT=1)

def fnum(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError, TypeError):
        return float('nan')

def fit(xs, ys):
    """Ordinary least squares y = m*x + b; returns (m, b, r2, n)."""
    n = len(xs)
    if n < 3:
        return (float('nan'),)*3 + (n,)
    mx = sum(xs)/n; my = sum(ys)/n
    sxx = sum((x-mx)**2 for x in xs)
    sxy = sum((x-mx)*(y-my) for x, y in zip(xs, ys))
    if sxx == 0:
        return (float('nan'),)*3 + (n,)
    m = sxy/sxx; b = my - m*mx
    syy = sum((y-my)**2 for y in ys)
    ss_res = sum((y-(m*x+b))**2 for x, y in zip(xs, ys))
    r2 = 1 - ss_res/syy if syy > 0 else float('nan')
    return m, b, r2, n

def robust_fit(xs, ys, iters=3):
    """LS with iterative 2.5-sigma residual trimming (kills desync spikes)."""
    keep = list(zip(xs, ys))
    m = b = r2 = float('nan'); n = len(keep)
    for _ in range(iters):
        if len(keep) < 3:
            break
        X = [p[0] for p in keep]; Y = [p[1] for p in keep]
        m, b, r2, n = fit(X, Y)
        res = [abs(y-(m*x+b)) for x, y in keep]
        mr = sum(res)/len(res)
        sd = math.sqrt(sum((rr-mr)**2 for rr in res)/len(res)) or 1e-9
        nk = [p for p, rr in zip(keep, res) if rr <= mr + 2.5*sd]
        if len(nk) == len(keep):
            break
        keep = nk
    return m, b, r2, n

def main():
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    path = sys.argv[1]
    rows = list(csv.DictReader(open(path)))
    cols = rows[0].keys() if rows else []
    for need in ('focIa', 'focIb', 'fall_off_max'):
        if need not in cols:
            print(f"!! column '{need}' missing — is this an A2 AN1078 CSV? "
                  f"(have: {', '.join(list(cols)[:12])} ...)")
            sys.exit(2)

    # Build aligned samples while the drive is actually running.
    X, Y, DUTY, ERPM = [], [], [], []
    for r in rows:
        if r.get('state_name') != 'CL':
            continue
        ia = fnum(r, 'focIa'); ib = fnum(r, 'focIb'); raw = fnum(r, 'fall_off_max')
        if any(math.isnan(v) for v in (ia, ib, raw)):
            continue
        if raw <= 0 or raw >= 4095:      # rail / no-sample
            continue
        X.append(raw)
        Y.append(-(ia + ib))             # iw_ref in amps
        DUTY.append(fnum(r, 'duty'))
        ERPM.append(fnum(r, 'eRPM'))

    print(f"file: {path}")
    print(f"CL aligned samples: {len(X)}")
    if len(X) < 20:
        print("!! too few samples — capture a longer CL run across the speed range.")
        sys.exit(3)

    rawmin, rawmax = min(X), max(X)
    print(f"raw Iw span: {rawmin:.0f} .. {rawmax:.0f} counts "
          f"(dynamic range {rawmax-rawmin:.0f})")
    if rawmax - rawmin < 40:
        print("!! raw Iw barely moves — channel may be dead/misrouted. Check AD3CH3.")

    m, b, r2, n = robust_fit(X, Y)
    offset_count = -b/m if m else float('nan')
    invert = m < 0
    print("\n── Iw calibration (robust LS, iu+iv+iw=0) ─────────────")
    print(f"  gain        : {m:+.6f} A/count   (|{abs(m):.6f}|)")
    print(f"  offset      : {offset_count:8.1f} counts   (zero-current rest)")
    print(f"  sign/invert : {'INVERTED (like dsPIC, INVERT=1)' if invert else 'NON-inverted (opposite dsPIC)'}")
    print(f"  fit R^2     : {r2:.4f}   over {n} pts")
    ratio = abs(m)/DSPIC_A_PER_COUNT
    print(f"  vs dsPIC    : |gain|/{DSPIC_A_PER_COUNT} = {ratio:.3f}x "
          f"({'≈ same scale — can share' if 0.9 < ratio < 1.1 else 'DIFFERENT scale — needs own constant'})")
    print(f"\n  → AN_CURRENT_W_A_PER_COUNT  {(-abs(m) if invert else abs(m)):+.6f}f")
    print(f"    AN_CURRENT_W_MIDPOINT     {offset_count:.0f}")

    # Duty-banded fit quality — does Iw stay linear as the window collapses?
    print("\n── fit quality vs duty (does Iw survive high duty?) ───")
    bands = [(0, 40), (40, 60), (60, 75), (75, 85), (85, 101)]
    print(f"  {'duty %':>10} {'n':>6} {'R^2':>7} {'gain':>10} {'resid RMS(A)':>13}")
    for lo, hi in bands:
        bx = [x for x, d in zip(X, DUTY) if lo <= d < hi]
        by = [y for y, d in zip(Y, DUTY) if lo <= d < hi]
        if len(bx) < 5:
            continue
        bm, bb, br2, bn = robust_fit(bx, by)
        res = [ (y-(bm*x+bb)) for x, y in zip(bx, by) ]
        rms = math.sqrt(sum(e*e for e in res)/len(res))
        print(f"  {lo:3d}-{hi-1:<3d}    {bn:6d} {br2:7.3f} {bm:+10.6f} {rms:13.4f}")
    print("\n  (R^2 falling / RMS rising with duty = Iw window collapsing too —\n"
          "   tells us up to what duty best-2-of-3 can trust the Iw leg.)")

if __name__ == '__main__':
    main()
