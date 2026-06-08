"""analyze.py — shared Garuda diagnostic analyzers.

Pure functions over telemetry/scope samples (lists of dicts). Each check returns
findings of one shape:

    {"check", "severity", "title", "detail", "evidence": {...}, "fix": str|None}

severity ∈ {"info","warn","critical"}. Encode-once: the GUI rule engine, the MCP
server, and headless scripts all call these, so the signatures we keep
re-deriving by hand (phantom, half-period, railed Ia, polarity-locked misses)
become instant flags with zero AI tokens. Accepts decoded sample dicts or raw
CSV rows (column names from both the decoder and the saved telemetry.csv).
"""
from __future__ import annotations
import csv as _csv

# No-load eRPM per unit duty at 24 V (bench fit, 2810 @ 24 V) — the same
# physics the firmware abs-floor guard uses, for the operating-point check.
KSPEED = 235_000.0
PHANTOM_OVERSPEED = 1.30          # eRPM above this × no-load(duty) = phantom


# ── helpers ──────────────────────────────────────────────────────────────
def _f(r, k, d=0.0):
    try:
        return float(r.get(k, d))
    except (TypeError, ValueError):
        return d


def load_csv(path):
    with open(path, newline="") as fh:
        return list(_csv.DictReader(fh))


def _rows(samples):
    """Accept a CSV path or an already-loaded list of dicts."""
    return load_csv(samples) if isinstance(samples, str) else list(samples)


def _finding(check, sev, title, detail, evidence=None, fix=None):
    return {"check": check, "severity": sev, "title": title,
            "detail": detail, "evidence": evidence or {}, "fix": fix}


def _state(r):
    return (r.get("state_name") or str(r.get("state", "")) or "").strip()


def _ia(r):
    for k in ("ia_pk_mag", "ia_A", "ia"):
        if k in r:
            return _f(r, k)
    return 0.0


def _duty(r):
    return _f(r, "duty", _f(r, "duty_pct"))


def _miss_sectors(r):
    if isinstance(r.get("miss_by_sector"), (list, tuple)):
        return [int(x) for x in r["miss_by_sector"]]
    if all(f"miss_s{i}" in r for i in range(6)):
        return [int(_f(r, f"miss_s{i}")) for i in range(6)]
    return None


# ── checks (telemetry) ─────────────────────────────────────────────────────
def current_offset_check(samples):
    """Ia ≉ 0 when there is provably no current (ARMED/standstill) → a phase-A
    current-sense DC offset (not real current)."""
    rows = _rows(samples)
    still = [r for r in rows if _state(r) in ("ARMED", "IDLE", "ALIGN")
             and _f(r, "eRPM") < 1]
    if not still:
        return []
    iavals = [_ia(r) for r in still]
    mean = sum(iavals) / len(iavals)
    if mean < 2.0:                                  # ~real zero
        return []
    sev = "critical" if mean > 15 else "warn"
    return [_finding(
        "current_offset", sev,
        "Phase-A current reads non-zero at standstill",
        f"Ia ≈ {mean:.1f} A while disarmed/standstill (true current = 0). This is "
        f"a DC offset on the phase-A sense (AD1CH3/OA1), not real current, and it "
        f"inflates every Ia reading and would defeat a phase-current limiter.",
        {"ia_standstill_A": round(mean, 1), "n": len(still)},
        "Add a startup current-offset auto-calibration (sample iaRaw/ibRaw while "
        "disarmed, use as the per-channel bias instead of the fixed 2048).")]


def phantom_check(samples):
    """CL rows where the commanded eRPM exceeds what the duty can physically
    produce (PLL harmonic false-lock / pot-zero phantom)."""
    rows = _rows(samples)
    worst = None
    n = 0
    for r in rows:
        if _state(r) != "CL":
            continue
        duty = _duty(r) / 100.0
        e = _f(r, "eRPM")
        if duty <= 0.0:
            continue
        ceiling = KSPEED * duty * PHANTOM_OVERSPEED
        if e > ceiling and duty <= 0.12:            # only the low-duty regime
            n += 1
            if worst is None or e > worst[0]:
                worst = (e, duty, ceiling)
    if not worst:
        return []
    e, duty, ceiling = worst
    return [_finding(
        "phantom", "critical",
        "PLL phantom / harmonic false-lock",
        f"Commanded {e:,.0f} eRPM at {duty*100:.0f}% duty — the rotor can't exceed "
        f"~{ceiling:,.0f} eRPM there. The period estimate has false-locked onto a "
        f"noise harmonic (controller spins while the rotor idles/coasts).",
        {"peak_eRPM": round(e), "duty_pct": round(duty * 100, 1),
         "floor_eRPM": round(ceiling), "rows": n},
        "Enable FEATURE_HWZC_ABS_FLOOR (operating-point period floor, shrink-only). "
        "Validate in pisim.py.")]


def polarity_miss_check(samples):
    """Even(0/2/4)=rising vs odd(1/3/5)=falling ZC miss split — clustering on one
    polarity = a structural detection gap, not healthy state-driven misses."""
    rows = _rows(samples)
    seen = [_miss_sectors(r) for r in rows]
    seen = [s for s in seen if s]
    if len(seen) < 2:
        return []
    even = odd = 0
    prev = seen[0]
    for cur in seen[1:]:
        for i in range(6):
            d = (cur[i] - prev[i]) & 0xFFFF
            if i % 2 == 0:
                even += d
            else:
                odd += d
        prev = cur
    tot = even + odd
    if tot < 50:
        return []
    hi = max(even, odd)
    frac = 100.0 * hi / tot
    if frac < 70:
        return []
    pol = "ODD/falling (S1,3,5)" if odd >= even else "EVEN/rising (S0,2,4)"
    return [_finding(
        "polarity_miss", "warn",
        "ZC misses are polarity-locked",
        f"{frac:.0f}% of guesses are on {pol} sectors (even={even:,} odd={odd:,}) — "
        f"a structural detection gap on one polarity, not state-driven misses.",
        {"even": even, "odd": odd, "frac_pct": round(frac)},
        "Extend the OFF-center (PWM-OFF) sample to the masked polarity at low "
        "speed (FEATURE_HWZC_LOWSPD_OFFCTR / FALLING_SW).")]


# ── checks (scope / burst capture) ─────────────────────────────────────────
def half_period_check(scope_rows):
    """Compare the commutation cadence in a scope capture (sector dwell) against
    the reported eRPM. ~2× = the half-period the PLL period estimate is prone to."""
    rows = _rows(scope_rows)
    if not rows or "sector" not in rows[0] or "t_us" not in rows[0]:
        return []
    # sector dwell in samples → seconds
    secs = [int(_f(r, "sector")) for r in rows]
    trans = [i for i in range(1, len(secs)) if secs[i] != secs[i - 1]]
    if len(trans) < 2:
        return []
    dwell_samples = [trans[i] - trans[i - 1] for i in range(1, len(trans))]
    dwell = sum(dwell_samples) / len(dwell_samples)
    dt_us = _f(rows[1], "t_us") - _f(rows[0], "t_us")
    if dt_us <= 0:
        return []
    sector_s = dwell * dt_us * 1e-6
    measured_erpm = 10.0 / sector_s if sector_s > 0 else 0     # 6 sectors/erev → ×60/6
    reported = _f(rows[0], "eRPM")
    if reported <= 0 or measured_erpm <= 0:
        return []
    ratio = reported / measured_erpm
    if 1.6 <= ratio <= 2.4:
        return [_finding(
            "half_period", "warn",
            "Reported eRPM is ~2× the measured commutation rate",
            f"Scope sector dwell = {dwell:.0f} samples ({sector_s*1e3:.2f} ms) → "
            f"~{measured_erpm:,.0f} eRPM, but telemetry reports {reported:,.0f} "
            f"(×{ratio:.2f}). The period estimate is half-counting — the benign, "
            f"stable form of the phantom mechanism.",
            {"measured_eRPM": round(measured_erpm), "reported_eRPM": round(reported),
             "ratio": round(ratio, 2), "sector_samples": round(dwell)},
            "Confirm hwzc_hr vs sector dwell; if consistent, the eRPM scale is 2× "
            "at this regime.")]
    return []


# ── top level ──────────────────────────────────────────────────────────────
def analyze(samples):
    """Run all telemetry checks; return findings sorted most-severe first."""
    rows = _rows(samples)
    out = []
    for fn in (phantom_check, current_offset_check, polarity_miss_check):
        try:
            out += fn(rows)
        except Exception as e:           # a bad column shouldn't kill the rest
            out.append(_finding(fn.__name__, "info", "check skipped", str(e)))
    if not out:
        out = [_finding("ok", "info", "No known issues flagged",
                        "None of the encoded signatures matched this run.")]
    order = {"critical": 0, "warn": 1, "info": 2}
    return sorted(out, key=lambda f: order.get(f["severity"], 3))


def analyze_scope(scope_rows):
    """Run scope-capture checks (half-period, current offset)."""
    rows = _rows(scope_rows)
    out = half_period_check(rows) + current_offset_check(rows)
    return out or [_finding("ok", "info", "Scope looks nominal",
                            "No half-period or offset flags in this capture.")]
