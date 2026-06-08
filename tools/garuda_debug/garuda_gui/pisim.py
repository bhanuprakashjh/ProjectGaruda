"""garuda pisim — standalone simulator for the HWZC sector-PI / PLL loop.

Mirrors the firmware math in motor/hwzc.c::HWZC_OnPiPeriodExpired EXACTLY
(float PLL form: integratorF += Ki*delta; period = integratorF + Kp*delta),
so we can study the loop in isolation — no MCU, no hardware — and reproduce /
fix the pot-zero "phantom lock" (PLL harmonic false-lock) offline.

Two layers, like zcsim:
  * Layer A (PI-alone unit test): captures placed at the TRUE rotor ZC every
    sector (perfect detection). Drives a speed profile and checks the loop
    tracks. This is "the PI alone" — validates gains/clamps with no plant noise.
  * Layer B (phantom repro): a closed-loop rotor + a corrupted-capture model.
    During a decel/regen window a burst of NOISE captures (fixed real-time
    spacing, ~PWM-coupled) kicks the loop out of lock; the relative-only gates
    let it ratchet and FALSE-LOCK onto a noise-determined harmonic period.
    Toggle GUARD_ABS_FLOOR to test the proposed operating-point-aware floor.

Pure-Python/numpy, no Qt. Run:  python3 pisim.py
Constants below are copied verbatim from garuda_config.h / garuda_calc_params.h
(profile 2: 2810 @ 24V, MAX_CL=260k, advMax=25, MIN_INTERVAL=50%).
"""
from __future__ import annotations
import numpy as np

# ── firmware constants (SCCP2 = 100 MHz; period in HR ticks; 1e9/erpm) ───────
SCCP_HZ        = 100_000_000
def ticks(erpm):  return 1_000_000_000.0 / max(erpm, 1e-6)
def erpm(T):      return 1_000_000_000.0 / max(T, 1e-9)

KP             = 0.25         # 2*zeta*wn, zeta=0.5 wn=0.25
KI             = 0.0625       # wn^2
CLAMP_SHIFT       = 3         # +-T/8 normal
CLAMP_SHIFT_HI_P  = 4         # +T/16 at high rpm
CLAMP_SHIFT_HI_N  = 5         # -T/32 at high rpm (tighter shrink)
CLAMP_HIGH_TICKS  = ticks(150_000)
MAX_CL_ERPM    = 260_000
MIN_STEP_TICKS = ticks(MAX_CL_ERPM)       # the (too-permissive) hard floor
MIN_INTERVAL_PCT = 50
ADV_MIN_DEG, ADV_MAX_DEG = 0, 25
RAMP_TARGET_ERPM = 3_000
DEF_TRIGGER, DEF_EXIT, DEF_GROW = 6, 2, 0.01

PWM_HZ         = 45_000
KSPEED         = 235_000      # no-load equilibrium eRPM per unit duty (bench fit)


def adv_fp8(T):
    e = erpm(T)
    if   e <= RAMP_TARGET_ERPM: adv = ADV_MIN_DEG
    elif e >= MAX_CL_ERPM:      adv = ADV_MAX_DEG
    else:
        adv = ADV_MIN_DEG + (ADV_MAX_DEG - ADV_MIN_DEG) * \
              (e - RAMP_TARGET_ERPM) / (MAX_CL_ERPM - RAMP_TARGET_ERPM)
    return ((30.0 + adv) * 256.0) / 60.0       # the /256 cancels in set_frac


class PLL:
    """Exact port of HWZC_OnPiPeriodExpired (float path, defensive on)."""
    def __init__(self, T0, guard_abs_floor=False):
        self.timerPeriod = float(T0)
        self.integratorF = float(T0)
        self.defActive = False
        self.missStreak = 0
        self.goodStreak = 0
        self.guard = guard_abs_floor
        self.absFloor = 0.0                # set per-step from duty (operating point)

    def set_duty(self, duty_frac):
        # proposed guard: rotor physically can't spin faster than the no-load
        # equilibrium for this duty (+margin) -> period can't be shorter.
        max_erpm = KSPEED * max(duty_frac, 1e-3) * 1.25
        self.absFloor = ticks(max_erpm)

    def step(self, capValue):
        """One PI sector event. capValue=None => true silence (miss)."""
        T = self.timerPeriod                         # period at entry (for shrink test)
        had_capture = capValue is not None
        if had_capture and capValue <= T:
            setValue = (adv_fp8(T) * T) / 256.0
            delta = capValue - setValue
            if T < CLAMP_HIGH_TICKS:
                posCap = T / (1 << CLAMP_SHIFT_HI_P)
                negCap = T / (1 << CLAMP_SHIFT_HI_N)
            else:
                posCap = negCap = T / (1 << CLAMP_SHIFT)
            delta = min(posCap, max(-negCap, delta))
            if not self.defActive:
                self.integratorF = max(self.integratorF + KI * delta, MIN_STEP_TICKS)
                newPer = max(self.integratorF + KP * delta, MIN_STEP_TICKS)
                if self.guard and newPer < self.absFloor and newPer < T:
                    # shrink-only: block PI-driven accel below floor (phantom),
                    # but let a real high-speed decel coast down through it.
                    newPer = self.absFloor
                    self.integratorF = max(self.integratorF, self.absFloor)
                self.timerPeriod = newPer
        elif had_capture:
            pass                                     # cap>T: rejected, period held
        # defensive streaks
        if had_capture:
            self.missStreak = 0; self.goodStreak += 1
        else:
            self.missStreak += 1; self.goodStreak = 0
        if not self.defActive and self.missStreak >= DEF_TRIGGER:
            self.defActive = True
        elif self.defActive and self.goodStreak >= DEF_EXIT:
            self.defActive = False
        if self.defActive:                           # walk T larger (relax)
            self.integratorF *= (1.0 + DEF_GROW)
            self.timerPeriod = self.integratorF
        return self.timerPeriod


# ── Layer A: PI alone, ideal captures at the true rotor ZC ───────────────────
def run_ideal(profile, jitter_frac=0.01, seed=0):
    """profile: list of (erpm_rotor) per sector. Capture = true ZC + jitter.
    Returns commanded eRPM per sector."""
    rng = np.random.default_rng(seed)
    pll = PLL(ticks(profile[0]))
    out = []
    for e_rot in profile:
        Trot = ticks(e_rot)
        setf = adv_fp8(pll.timerPeriod) / 256.0
        cap = setf * Trot * (1.0 + jitter_frac * rng.standard_normal())
        pll.step(cap)
        out.append(erpm(pll.timerPeriod))
    return np.array(out)


# ── Layer B: closed-loop rotor + corrupted-capture model ─────────────────────
def run_closed(duty_seq, *, guard=False, noise_window=None,
               noise_pwm_cycles=1.5, sync_tol=0.45, seed=1):
    """duty_seq: per-sector commanded duty (frac). noise_window: (i0,i1) sector
    range where the comparator fires NOISE at a fixed real-time spacing of
    `noise_pwm_cycles` PWM periods (the regen/decel kick). Returns dict of
    arrays (erpm_cmd, erpm_rotor, defensive)."""
    rng = np.random.default_rng(seed)
    e_rot = KSPEED * duty_seq[0]
    pll = PLL(ticks(e_rot), guard_abs_floor=guard)
    t_noise = noise_pwm_cycles / PWM_HZ * SCCP_HZ        # ticks between noise fires
    cmd, rot, dfn = [], [], []
    for i, duty in enumerate(duty_seq):
        pll.set_duty(duty)
        Tctrl = pll.timerPeriod
        Trot  = ticks(max(e_rot, 1.0))
        synced = abs(Tctrl - Trot) / Trot < sync_tol
        in_noise = noise_window and noise_window[0] <= i < noise_window[1]
        # capture model -------------------------------------------------------
        if synced and not in_noise:
            setf = adv_fp8(Tctrl) / 256.0
            cap = setf * Trot * (1.0 + 0.01 * rng.standard_normal())
            cap = cap if cap <= Tctrl else None        # cross-sector -> miss
        else:
            # NOISE: the comparator free-runs at a fixed real-time spacing
            # t_noise (PWM-coupled). The min-interval gate accepts the first
            # fire at >= 50% of the sector. If t_noise already fits the sector
            # it lands at t_noise (period-locks to the noise); if the sector is
            # still wider, the gate boundary forces a shrink toward t_noise.
            gate = MIN_INTERVAL_PCT / 100.0 * Tctrl
            cap = t_noise if t_noise >= gate else gate    # drive shrink to t_noise
            cap = cap if cap <= Tctrl else None
        pll.step(cap)
        # rotor plant (gentle, so the rate-clamped controller can follow) -----
        e_eq = KSPEED * duty
        if abs(pll.timerPeriod - Trot) / Trot < sync_tol:
            e_rot += (e_eq - e_rot) * 0.05             # accel toward equilibrium
        else:
            e_rot += (0.0 - e_rot) * 0.03              # desynced -> coast down
        e_rot = max(e_rot, 200.0)
        cmd.append(erpm(pll.timerPeriod)); rot.append(e_rot); dfn.append(pll.defActive)
    return dict(cmd=np.array(cmd), rot=np.array(rot), dfn=np.array(dfn))


# ── replay a REAL recorded run and overlay the absolute-floor guard ──────────
# Uses the SAME no-load-period formula as the firmware FF seed / the guard:
#   P_ff = 181.380 * lambda / (Vbus * dutyFrac)   [HR ticks]
# (lambda = gspParams.focKeUvSRad, profile 2 = 583). This pins the analysis to
# the actual bench phantom: it shows, row by row, where the guard floor sits
# against the recorded eRPM and what the phantom would have been clamped to.
FF_K            = 181.380
LAMBDA_PROFILE2 = 583.0
ABS_FLOOR_OVERSPEED_PCT = 130
ABS_FLOOR_MIN_DUTYFRAC  = 0.03


def floor_erpm(duty_frac, vbus_v, lam=LAMBDA_PROFILE2,
               overspeed_pct=ABS_FLOOR_OVERSPEED_PCT):
    """The guard's speed ceiling for this operating point (eRPM)."""
    if duty_frac <= ABS_FLOOR_MIN_DUTYFRAC or vbus_v < 6.0 or lam <= 0:
        return None
    P_ff   = FF_K * lam / (vbus_v * duty_frac)
    floorP = P_ff * (100.0 / overspeed_pct)
    return erpm(floorP)


def replay_csv(path, lam=LAMBDA_PROFILE2, overspeed_pct=ABS_FLOOR_OVERSPEED_PCT):
    import csv
    rows = list(csv.DictReader(open(path)))
    n_clamp = 0; obs_peak = 0.0; capped_peak = 0.0; worst = None
    guarded = None                       # stateful guarded eRPM (what the loop would do)
    for r in rows:
        try:
            duty = float(r.get("duty", 0)) / 100.0
            vbus = float(r.get("vbus_V", 0))
            e_obs = float(r.get("eRPM", 0))
        except (TypeError, ValueError):
            continue
        obs_peak = max(obs_peak, e_obs)
        fe = floor_erpm(duty, vbus, lam, overspeed_pct)
        if guarded is None:
            guarded = e_obs
        # SHRINK-ONLY, STATEFUL: the controller tries to follow e_obs, but it
        # cannot be DRIVEN UP through the floor (phantom). It CAN fall through
        # the floor (real high-speed decel coasting down).
        if fe is not None and e_obs > fe and e_obs > guarded:
            guarded = fe                 # block the rise above floor
            n_clamp += 1
            if worst is None or e_obs > worst[0]:
                worst = (e_obs, fe, duty, vbus)
        else:
            guarded = e_obs              # follow (accel under floor, or decel)
        capped_peak = max(capped_peak, guarded)
    print(f"  replayed {len(rows)} rows from {path.split('/')[-1]}")
    print(f"  observed peak eRPM      : {obs_peak:8.0f}")
    print(f"  guard would cap peak to : {capped_peak:8.0f}  "
          f"(rows clamped: {n_clamp})")
    if worst:
        print(f"  worst phantom row       : observed {worst[0]:.0f} eRPM at "
              f"duty {100*worst[2]:.0f}% / {worst[3]:.1f}V -> floor "
              f"{worst[1]:.0f} eRPM (clamped {worst[0]/worst[1]:.1f}x)")
    return dict(obs_peak=obs_peak, capped_peak=capped_peak, n_clamp=n_clamp)


def _summary(name, d):
    cmd = d["cmd"]
    print(f"  {name:22s} final cmd={cmd[-1]:8.0f} eRPM   "
          f"peak={cmd.max():8.0f}   phantom={'YES' if cmd[-1] > 120_000 else 'no'}")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:                         # replay a real recorded run
        print(f"\n=== Real-run replay + guard overlay ===")
        for p in sys.argv[1:]:
            replay_csv(p); print()
        sys.exit(0)

    print("\n=== Layer A: PI alone (ideal captures, gentle accel→cruise→decel) ===")
    # realistic rates: the asymmetric neg-clamp (T/32 high-rpm) deliberately
    # rate-limits period SHRINK (accel), so use bench-like gentle ramps.
    prof = (list(np.linspace(10_500, 80_000, 400)) +
            [80_000] * 200 +
            list(np.linspace(80_000, 10_500, 400)))
    cmd = run_ideal(prof)
    prof = np.array(prof)
    err = np.abs(cmd - prof) / prof
    cruise = err[400:600]                       # steady-state region
    print(f"  tracking: cruise mean |err| = {100*cruise.mean():.2f}%   "
          f"full-run max |err| = {100*err.max():.2f}%   (locks at cruise: "
          f"{'YES' if cruise.mean() < 0.02 else 'NO'})")

    print("\n=== Layer B: pot-zero decel with a noise kick (the phantom) ===")
    # idle → gentle ramp up → hold → pot-zero decel; noise burst during decel.
    duty = (np.r_[np.full(60, 0.05),
                  np.linspace(0.05, 0.45, 200),
                  np.full(100, 0.45),
                  np.linspace(0.45, 0.05, 200),    # pot-zero decel
                  np.full(600, 0.05)])             # idle tail (phantom develops)
    nwin = (360, 385)                                # regen/decel noise window
    base = run_closed(duty, guard=False, noise_window=nwin)
    fixed = run_closed(duty, guard=True,  noise_window=nwin)
    _summary("relative gates only", base)
    _summary("+ absolute floor guard", fixed)

    print("\n  (Layer B uses a MODEL capture source; absolute numbers depend on "
          "noise_pwm_cycles. The POINT is the qualitative result: relative-only "
          "gates false-lock, the operating-point floor prevents it.)\n")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        for a, d, t in ((ax[0], base, "relative gates only — FALSE LOCK"),
                        (ax[1], fixed, "+ absolute floor guard — holds")):
            a.plot(d["cmd"], label="commanded eRPM", lw=1.5)
            a.plot(d["rot"], label="rotor eRPM", lw=1.2, ls="--")
            a.axvspan(*nwin, color="r", alpha=0.12, label="noise kick")
            a.set_title(t); a.set_ylabel("eRPM"); a.legend(fontsize=8); a.grid(alpha=.3)
        ax[1].set_xlabel("sector")
        fig.tight_layout(); fig.savefig("pisim_phantom.png", dpi=110)
        print("  wrote pisim_phantom.png\n")
    except Exception as e:
        print(f"  (matplotlib unavailable, text-only: {e})\n")
