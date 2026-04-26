# Tuning a New Motor on AN1078+PLL Using the GUI

End-to-end procedure for bringing up a new sensorless FOC motor on the AN1078
observer, using only the GUI scope and parameter editor.  Aimed at: a motor
where you know `Rs`, `Ls`, `λ_pm` (or KV), and `polePairs`, but the observer
phase shift, FW thresholds, and PI gains are unknown.

**Prerequisites**: motor has working OL→CL handoff on default tunings (GUI
shows it transitions to `CLOSED_LOOP` state without faulting).  If startup
itself doesn't work, fix that first using `AN_LOCK_TIME` /
`AN_Q_CURRENT_REF_OPENLOOP` / `AN_OL_RAMP_RATE_RPS2` in `an1078_params.h`.

---

## Step 0 — Plug in motor params

In `dspic33AKESC/foc/an1078_params.h`:

```c
#define AN_NOPOLESPAIRS    /* count rotor magnet pairs */
#define AN_MOTOR_RS        /* phase-to-neutral, Ω.  Datasheet or measure */
#define AN_MOTOR_LS        /* phase-to-neutral, H. */
#define AN_MOTOR_LAMBDA    /* V·s/rad_elec.  Compute: 60 / (√3 · 2π · KV · PP) */
#define AN_NOMINAL_SPEED_RPM_MECH  /* set 1.5× expected max speed */
```

Flash, then verify the **GUI Profile Selector** shows the right name —
profile slot 2 in `gui/src/protocol/types.ts:PROFILE_NAMES` should match
your motor.  Mismatch causes confusion (e.g. GUI labeled "5010" while
firmware tuned for "2810" — tuning silently writes to the wrong target).

---

## Step 1 — Verify observer is alive

**Open Scope tab → preset "AN1078 Angle Health"** (channels: Drive Theta,
Observer Theta, θ Drive − θ Obs, eRPM).

Spin motor low CL (10–30k eRPM).  Watch:

- **Drive Theta vs Observer Theta** — both should rotate at the same rate
  with a small fixed lag.  If they diverge or one is rotating in the
  opposite direction, the observer is broken (PLL locked to wrong
  direction, or `Kslide` / `MaxLinearErr` are wrong for this motor).
- **θ Drive − θ Obs** — should settle near a small offset (≈ the value of
  `AN_SMC_THETA_OFFSET_BASE`).  If it wanders >10° random walk, the LPF
  cutoff is too low or the PLL is on the edge.

If the observer doesn't track at all (Drive Theta moves but Observer Theta
stays at 0 or drifts randomly):
- BEMF probably too weak vs `Kslide`.  Lower `AN_SMC_KSLIDE` until Z is
  smaller than steady-state V.  For low-Rs motors: 2–3 V; for higher-Rs
  motors: AN1078 default 0.85·Vbus is fine.

---

## Step 2 — Tune the angle offset

**Open Scope tab → preset "AN1078 SMO Tune"** (channels: focVd, focVq,
focIq, focId, focERPM, focModIndex).

This is the main tuning view.  At any steady speed in CL with the d-PI
holding `Id_ref = 0`, the relationship between Vd and observer alignment is:

| `focVd` | Observer state | Action |
|---------|---------------|--------|
| ≈ 0 V | Aligned correctly | done at this speed |
| Negative (e.g. -3 V) | **Lags** rotor | INCREASE `THETA_OFFSET_BASE` |
| Positive (e.g. +3 V) | **Leads** rotor | DECREASE `THETA_OFFSET_BASE` |

**Procedure**:

1. Bring motor to a safe low CL speed (around 10× the OL handoff speed).
2. Hold steady, read `focVd` average from the scope.
3. Adjust `AN_SMC_THETA_OFFSET_BASE` in `an1078_params.h` (1–2° steps,
   convert to radians: `1° = 0.01745 rad`), recompile, reflash.
4. Repeat until `focVd` is within ±0.5 V at this speed.

**For wide speed range — tune `AN_SMC_THETA_OFFSET_K`**:

The LPF group delay grows with motor frequency, so a single offset only
fits one speed.  After BASE is dialed in at low speed:

1. Bring motor to high speed (e.g. half of `AN_NOMINAL_SPEED_RPM_MECH`).
2. Read `focVd` average.
3. If `focVd < 0` (lags more at high speed): increase `THETA_OFFSET_K`.
4. If `focVd > 0` (leads at high speed): decrease `THETA_OFFSET_K`.
5. Step in `1e-5` increments — at typical speeds this shifts the offset
   by a few degrees.

Goal: `focVd` stays within ±1.5 V across the entire CL speed range.
With the PLL upgrade in this codebase, `K = 0` often works — only tune K
if you see speed-dependent drift.

---

## Step 3 — Verify PI gains

**Scope preset "FOC Currents" or default Iq/Id**:

- Set throttle to give a moderate speed (50% of nominal).
- Iq should track its setpoint with small error and minimal overshoot
  during throttle changes.
- Id should stay near 0 (the d-PI is doing its job).

If Id swings wildly (>3 A) when throttle changes:
- d-PI bandwidth too low.  Check `AN_KP_DQ`, `AN_KI_DQ` are set from
  `Kp = bw·Ls`, `Ki = bw·Rs` for ~1 kHz bandwidth.

If speed oscillates around setpoint (visible in `focERPM`):
- Speed PI Ki too high.  Drop `AN_KI_SPD` to 0.05 then walk up.

---

## Step 4 — Push to top speed and tune field weakening

**Scope preset "AN1078 SMO Tune"** with `focModIndex` watched.

- Sweep throttle to maximum.
- `focModIndex` should rise toward 0.95 then plateau as FW engages.
- Watch `focId` — as FW engages, it should drift smoothly negative.
  Choppy/stepwise Id means FW gain is too high or trigger is too close
  to the clamp.

If motor trips with `BOARD_PCI` near top speed:
- FW pulling too much current.  In `an1078_motor.c`, the field-weakening
  block has `ID_FW_MAX_NEG = -12.0f` — reduce to `-8.0f` if your
  inverter rating is below MCLV-48V-300W's.

If motor speed plateaus before FW engages:
- `FW_TRIGGER` too high (currently 0.91).  Lower to 0.88 to engage earlier.

If FW engages but speed doesn't grow:
- Motor is at electrical max (V_avail/λ_pm).  Need higher Vbus, or accept
  this as the speed ceiling.

---

## Step 5 — Sanity sweep

Final verification.  Capture a CSV (logger or scope export) sweeping from
min to max throttle and back:

- **focVd**: should stay within ±2 V at all speeds (well-aligned observer)
- **focIq**: small at no-load, smoothly tracks throttle changes
- **focId**: ~0 below FW threshold, smoothly negative above
- **focObsConfidence**: should be > 0.5 across the range (low conf in
  burst transients is okay)
- **No state transitions to FAULT** during the sweep

Save the CSV alongside the param values in `an1078_params.h` — that is
your motor profile baseline.

---

## What's missing (auto-tune roadmap)

The procedure above is manual.  Building these would shrink it from
hours to minutes:

1. **Auto θ-offset cal command (GSP)**: spin at known speed, internally
   sweep theta_offset, find Vd minimum, write back.  ~50 lines firmware,
   one new GUI button.
2. **Auto Kslide adapter**: monitor BEMF amplitude during a CL run,
   adjust Kslide so Z signal stays in linear region.  AN1078's
   "MaxSMCError" already has this idea — make it speed-adaptive.
3. **PI gain auto-derive**: GUI already has this in MotorTuningPanel
   (`Kp = bw·Ls` etc).  Surface it as part of the workflow above.
4. **One-button "Tune Motor"**: chain the above into a guided wizard
   that captures the user's `Rs/Ls/λ` once and produces a working
   profile.

Until those exist, the manual procedure above gets a new motor running
in 30–60 minutes if the parameters are reasonably close to defaults.
