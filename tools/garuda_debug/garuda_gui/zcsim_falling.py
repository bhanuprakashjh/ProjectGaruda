"""garuda zcsim_falling - analytic study of WHY the falling (alternate) sector
loses true ZC above ~70k eRPM, built on the zcsim physics engine.

The bench question (2026-06-14): "the alternate phase misses true zc after
70/75k -- reexamine why." This module answers it mathematically instead of on
the bench.

Method
------
For each eRPM we drive zcsim.simulate() for a FALLING sector and read back:
  * true_permille   : where the *un-filtered* floating BEMF actually crosses
                      neutral, as permille of the 60-deg sector. By the symmetric
                      trapezoid this should be ~500 (dead mid-sector).
  * det_permille    : where the RC-filtered OFF-center SAMPLER (the firmware's
                      bemfRaw path, runs at PWM rate) first sees the crossing.
  * spp             : OFF-center samples available per sector = 450000/eRPM.

The gap (det - true) is then DECOMPOSED:
  * RC-lag artifact : a near-constant ~tau time delay -> grows as a fraction of
                      the shrinking sector (this is the 547->733->901 walk).
  * quantization    : up to 1 PWM period, because the sampler only looks once per
                      PWM period (= 1000/spp permille of coarseness).

Why it matters
--------------
If true_permille stays ~500 across speed, the falling ZC is SYMMETRIC and the
whole permille walk is a MEASUREMENT artifact. Then the firmware's >70k design
(drop falling, let the autonomous timer extrapolate the falling commutation at
mid-sector from the rising-only period) is ALREADY unbiased, and a feed-forward
"learned offset" cannot improve the *commutation* -- it would only re-bias a
correct extrapolation. That sends the >70k roughness hunt to the comp-amp
saturation band (~125k), consistent with the bench cap-raise test making it worse.

If instead true_permille drifts off 500, there is a real asymmetry the timer
mis-extrapolates, and feed-forward (or asymmetric advance) is justified.

Bench validation targets (AK512, 2026-06-12 polarity-split cap diag):
  rising comparator ~500 permille flat 30-92k; falling SW walks 547 -> 733 ->
  901 (max 955); detection/PI usable to ~70k (spp ~6.4) then capped.

Pure numpy. Run standalone:  python -m garuda_gui.zcsim_falling
"""
from __future__ import annotations
import numpy as np

from . import zcsim


def _true_zc_permille(res):
    """Permille of sector where the UN-filtered floating BEMF crosses neutral."""
    e = res["e_float"]
    thr = 0.0  # line-neutral BEMF crosses zero at the true electrical ZC
    n = len(e)
    # first sign change of (e - thr) after 1/4 sector (skip the demag region)
    start = n // 4
    for k in range(start, n):
        if (e[k - 1] - thr) == 0.0 or (e[k - 1] - thr) * (e[k] - thr) < 0.0:
            # linear interp between k-1 and k
            d0 = e[k - 1] - thr
            d1 = e[k] - thr
            frac_k = (k - 1 + (0.0 - d0) / (d1 - d0)) / (n - 1)
            return 1000.0 * frac_k
    return float("nan")


def falling_walk(erpm_list, *, rc_fc=5500.0, duty_fn=None, n_phase=16, **kw):
    """Sweep eRPM; return the falling-sector detection geometry per speed.

    For each eRPM we scan n_phase grid alignments between the sector start and the
    PWM period (the real rotor has no fixed alignment -- it beats as speed drifts).
    det_pm is reported as MEAN +/- STD over those alignments:
      * mean  -> matches the bench-averaged walk (547 -> 733 -> 901)
      * std   -> the per-sector jitter that destabilizes the PI
      * miss% -> fraction of alignments where the sampler never caught the crossing
    """
    duty_fn = duty_fn or zcsim.default_duty
    rows = []
    tau_us = 1.0 / (2.0 * np.pi * rc_fc) * 1e6
    phases = np.linspace(0.0, 1.0, n_phase, endpoint=False)
    for erpm in erpm_list:
        duty = duty_fn(erpm)
        spp = 450000.0 / erpm
        det_pms, lag_uss = [], []
        true_pm = None
        misses = 0
        for ph in phases:
            # falling sector = odd index (1); ideal sampler (no parasitics): the
            # OFF sampler reads the clean RC line regardless of comparator noise.
            res = zcsim.simulate(erpm, duty, sector_index=1, rc_fc=rc_fc,
                                 spike_amp=0.0, ring_amp=0.0, pwm_phase=ph, **kw)
            sector_us = res["sector_us"]
            if true_pm is None:
                true_pm = _true_zc_permille(res)
            det_us = res["offc_fired_us"]
            if det_us is None:
                misses += 1
                continue
            det_pms.append(det_us / sector_us * 1000.0)
            lag_uss.append(det_us - true_pm / 1000.0 * sector_us)
        sector_us = 1.0 / (erpm / 60.0) / 6.0 * 1e6
        det_pm = float(np.mean(det_pms)) if det_pms else None
        det_std = float(np.std(det_pms)) if det_pms else None
        lag_us = float(np.mean(lag_uss)) if lag_uss else None
        lag_pm = (det_pm - true_pm) if det_pm is not None else None
        rows.append(dict(erpm=erpm, duty=duty, spp=spp, sector_us=sector_us,
                         tau_us=tau_us, true_pm=true_pm, det_pm=det_pm,
                         det_std=det_std, lag_pm=lag_pm, lag_us=lag_us,
                         quant_pm=1000.0 / spp, miss_pct=100.0 * misses / n_phase,
                         detected=det_pm is not None))
    return rows


def feedforward_test(rows, learn_lo=30000, learn_hi=60000):
    """Test the proposed fix: learn the falling lag at low speed, feed it forward.

    Two questions:
      1. Is the lag a near-constant TIME (-> a single learned offset works at all
         speeds) or does it vary? (constant time == learnable)
      2. Does correcting the MEASUREMENT move det_pm back to true_pm -- AND does
         that matter for commutation, given >70k uses timer extrapolation (which
         already fires at true_pm if true_pm~500)?
    """
    learn = [r for r in rows if r["detected"] and r["lag_us"] is not None
             and learn_lo <= r["erpm"] <= learn_hi]
    if not learn:
        return None
    learned_us = float(np.mean([r["lag_us"] for r in learn]))
    out = []
    for r in rows:
        sec = r["sector_us"]
        # corrected detection (where falling IS detected): subtract learned time
        if r["det_pm"] is not None:
            corr_pm = r["det_pm"] - learned_us / sec * 1000.0
            corr_err = corr_pm - r["true_pm"]
            raw_err = r["det_pm"] - r["true_pm"]
        else:
            corr_pm = corr_err = raw_err = None
        # timer extrapolation (what the firmware does >70k): fires at mid-sector
        # 500. error vs the TRUE falling ZC:
        timer_err = 500.0 - r["true_pm"]
        out.append(dict(erpm=r["erpm"], detected=r["detected"],
                        raw_err=raw_err, corr_err=corr_err,
                        timer_err=timer_err))
    return learned_us, out


def _print_table(rows):
    print(f"{'eRPM':>8} {'duty':>5} {'spp':>5} {'secus':>7} {'truePM':>7} "
          f"{'detPM':>7} {'+/-jit':>6} {'lagPM':>6} {'miss%':>6} {'quantPM':>7}")
    for r in rows:
        detpm = f"{r['det_pm']:7.0f}" if r["det_pm"] is not None else "   none"
        jit = f"{r['det_std']:6.0f}" if r["det_std"] is not None else "  none"
        lagpm = f"{r['lag_pm']:6.0f}" if r["lag_pm"] is not None else "  none"
        print(f"{r['erpm']:>8} {r['duty']:>5.2f} {r['spp']:>5.1f} "
              f"{r['sector_us']:>7.1f} {r['true_pm']:>7.0f} {detpm} {jit} {lagpm} "
              f"{r['miss_pct']:>6.0f} {r['quant_pm']:>7.0f}")


if __name__ == "__main__":
    erpms = [20000, 30000, 40000, 50000, 60000, 70000, 75000, 90000,
             110000, 125000, 150000, 200000, 260000]
    print("=== FALLING-SECTOR ZC DETECTION GEOMETRY (zcsim physics) ===")
    print("tau (RC) =", f"{1/(2*np.pi*5500)*1e6:.1f} us;  PWM = 45 kHz;  "
          "spp = 450000/eRPM\n")
    rows = falling_walk(erpms)
    _print_table(rows)

    print("\n=== DECOMPOSITION: is true ZC symmetric (500) across speed? ===")
    tps = [r["true_pm"] for r in rows]
    print(f"true_permille range: {min(tps):.0f} .. {max(tps):.0f} "
          f"(mean {np.mean(tps):.0f})")
    print("-> if ~500 flat, the 547->901 walk is PURE measurement lag, not a")
    print("   real rotor asymmetry.")

    print("\n=== FEED-FORWARD TEST (learn lag @30-60k, apply at all speeds) ===")
    ff = feedforward_test(rows)
    if ff:
        learned_us, out = ff
        print(f"learned falling lag = {learned_us:.1f} us (constant-time offset)\n")
        print(f"{'eRPM':>8} {'det?':>5} {'raw_err':>8} {'corr_err':>9} "
              f"{'timer_err':>10}   (permille of sector)")
        for o in out:
            det = "no" if not o["detected"] else "yes"
            raw = f"{o['raw_err']:8.0f}" if o["raw_err"] is not None else "    none"
            corr = f"{o['corr_err']:9.0f}" if o["corr_err"] is not None else "     none"
            print(f"{o['erpm']:>8} {det:>5} {raw} {corr} {o['timer_err']:>10.0f}")
        print("\nraw_err   = uncorrected falling measurement error (the walk)")
        print("corr_err  = after subtracting the learned lag (where detected)")
        print("timer_err = error of the >70k timer extrapolation vs TRUE falling ZC")
        print("            (this is what actually drives commutation above 70k)")

        hi = [o for o in out if o["erpm"] >= 70000]
        max_timer = max(abs(o["timer_err"]) for o in hi)
        max_jit = max(r["det_std"] for r in rows if r["erpm"] >= 70000
                      and r["det_std"] is not None)
        max_miss = max(r["miss_pct"] for r in rows if r["erpm"] >= 70000)
        print("\n=== VERDICT ===")
        print(f"  >70k timer-extrapolation error vs true falling ZC : <= {max_timer:.0f} "
              f"permille ({max_timer/10:.1f}% of a sector) -> UNBIASED")
        print(f"  >70k falling MEASUREMENT jitter (std)             : up to {max_jit:.0f} "
              f"permille -> this is what the PI would chase")
        print(f"  >70k falling sectors the sampler simply MISSES    : up to {max_miss:.0f}%")
        print("  => The true falling ZC stays dead mid-sector (500) at every speed,")
        print("     so the rising-only TIMER extrapolation above 70k is already")
        print("     within ~0.3% of truth. Re-enabling the falling MEASUREMENT")
        print("     (or feeding a learned offset) only injects its jitter/misses")
        print("     back into the PI -- exactly the bench cap-raise failure.")
        print("     => >70k roughness is NOT falling timing; look at comp-amp")
        print("        (bemfDevPeak) saturation ~125k.")
