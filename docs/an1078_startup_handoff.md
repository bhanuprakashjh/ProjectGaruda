# AN1078 Startup & OL→CL Handoff — Mechanics and Tuning

This doc covers the open-loop startup sequence, the OL→CL handoff
mechanism, and the various corrections that make it reliable across
load conditions (no-prop, with-prop, different motors).  Updated
2026-04-27 after a long bench session that uncovered several latent
bugs.

## Five-phase startup sequence

```
IDLE → ALIGN → OL_RAMP → (BEMF gate) → CLOSED_LOOP
        │       │             │              │
        │       │             │              └─ thetaError bleed
        │       │             └─ wait for sustained BEMF magnitude
        │       └─ ramp synth angle 0 → AN_END_SPEED
        └─ rotor magnetic alignment
```

### Phase 1 — `ALIGN` (LOCK_TIME ticks)

Synth angle = 0.  Iq ramps 0 → `AN_Q_CURRENT_REF_OPENLOOP` over
`AN_IQ_SOFT_START_TICKS`.  Then holds until `AN_LOCK_TIME` ticks total
have elapsed.

**Iq application at θ_synth = 0** puts current on stator β-axis.
Rotor magnetic d-axis pulls toward stator field direction → rotor
settles at **θ_rotor = π/2 in stator frame** (NOT zero!).

This is the key insight that took us a while to figure out:

> The rotor doesn't end up at θ_rotor = 0 after alignment.  Iq applied
> at θ_synth = 0 means current on stator β.  Rotor d-axis aligns to β,
> so rotor d is at stator angle 90°, NOT 0°.

### Phase 2 — `OL_RAMP` start (LOCK→RAMP transition)

This is where a **critical fix** lives.  At end of alignment, we set:
```c
m->thetaOpenLoop = π/2;
```

Why?  The rotor's d-axis is at stator angle π/2.  For OL ramp to
generate proper torque from tick 1, synth d-axis must coincide with
rotor d-axis.  Setting thetaOpenLoop = π/2 puts synth d at stator π/2
= rotor d → aligned.  Iq applied on synth-q direction (perpendicular
to rotor d) → maximum forward torque.

Without this offset, OL would start with synth d at stator 0° while
rotor d is at stator 90° (90° misalignment).  Current parallel to
rotor d → **zero initial torque**.

**Effect of the bug**: with prop, prop friction nudged the rotor out
of the dead zone, motor caught up after a few ms of zero-torque
struggle.  Without prop, low-inertia free rotor stayed stuck at the
90° offset and never followed synth angle → motor stuck in OL.

**With the π/2 fix**: motor follows synth angle from tick 1, BEMF
develops as expected, handoff fires reliably regardless of load.

### Phase 3 — `OL_RAMP` ramping

Synth angle ramps at `AN_OL_RAMP_RATE_RPS2` (default 1000 rad/s²)
until `AN_END_SPEED_ELEC_RS`.  Iq held at `AN_Q_CURRENT_REF_OPENLOOP`
(default 12 A) provides torque to drag rotor along.

Rotor follows synth angle with a load angle (typically 30-60°).
BEMF in (Eα, Eβ) grows as ω·λ.  SMC observer estimates this BEMF and
PLL extracts θ_observer.

### Phase 4 — Handoff gate (BEMF threshold)

Once startupRamp ≥ AN_END_SPEED_ELEC_RS, motor is at idle target.
Handoff requires:
```
|EalphaFinal² + EbetaFinal²|  ≥  THRESHOLD × (λ × startupRamp)
```
sustained for `AN_HANDOFF_DWELL_TICKS` (default 2400 ticks ≈ 53 ms).

**Threshold history**:
- Original 80%: too tight, conf naturally sits 0.30-0.40 due to LPF
  attenuation
- Then 50%: still failed when motor slipped synth angle (BEMF below
  expected because rotor isn't really at synth speed)
- Now **20%**: well below the natural noise floor (0.30+) so handoff
  fires reliably regardless of slip dynamics

The conf float `bemf_observed/bemf_expected` is exposed in telemetry
as `focObsConfidence`.

### Phase 5 — `CLOSED_LOOP` with `thetaError` bleed

At handoff:
```c
m->thetaError = an_wrap_delta(m->thetaOpenLoop - m->smc.Theta);
```

This captures the difference between the synth angle (where we WERE
driving) and the SMC observer's estimate.  Now in CL:
```c
theta_drive = smc.Theta + thetaError
```

So initially `theta_drive == thetaOpenLoop` (continuity).  Then
`thetaError` bleeds to zero over time:
```c
if (transCounter == 0) {  // every TRANSITION_STEPS (~11 ticks)
    if (|thetaError| > AN_THETA_ERROR_BLEED_RAD) {
        thetaError -= sign(thetaError) × AN_THETA_ERROR_BLEED_RAD;
    } else {
        thetaError = 0;
    }
}
```

`AN_THETA_ERROR_BLEED_RAD = 0.005°` per pace, paced by transCounter.
At 45 kHz with TRANSITION_STEPS = 11, effective bleed rate ≈ 20°/sec.

**What this achieves**: smooth migration from synth-driven angle to
observer-driven angle.  Without the bleed, theta_drive would step from
thetaOpenLoop to smc.Theta on the first CL tick, which is risky if
they differ by 90°+.

**Trade-off**: if observer is drifting (PLL marginal lock), the bleed
exposes that drift over the bleed duration.  At zero throttle (motor
at AN_END_SPEED), lower BEMF → marginal PLL → drift becomes visible
after ~2 seconds of bleed → desync.

We mitigate by:
- Higher idle (`AN_END_SPEED_RPM_MECH = 2000` → BEMF 0.83V at handoff
  for A2212@12V) gives PLL more lock margin
- Bumpless `velRef = smc.OmegaFltred` at handoff prevents speed-PI
  from braking the motor while observer is converging

## CL handoff: bumpless transfer details

The naive `changeMode` block had:
```c
if (m->changeMode) {
    an_pi_preload(&m->pi_spd, AN_Q_CURRENT_REF_OPENLOOP);  // 12 A
    m->pi_d.integrator = 0;
    m->pi_q.integrator = 0;
    m->id_ref_fw = 0;
}
```

Three problems:
1. **`velRef_rad_s` stays at 0** (initialized in reset_parameters,
   never updated in OL).  At first CL tick, error = 0 - measured
   ≈ -1100 rad/s.  Speed PI computes Iq_ref = -16A, clamped to -12.
   **Motor brakes immediately.**
2. **q-PI integrator reset** loses the steady-state value that was
   delivering 12 A in OL.  q-PI re-builds integrator from zero.
3. **Speed PI integrator preloaded to constant 12 A** — fine in OL
   (we needed 12 A there) but OL torque ≠ CL idle torque.

The fix (current code):
```c
if (m->changeMode) {
    m->changeMode = false;
    m->velRef_rad_s = m->smc.OmegaFltred;  // ← bumpless setpoint
    an_pi_preload(&m->pi_spd, AN_Q_CURRENT_REF_OPENLOOP);
    m->pi_d.integrator = 0.0f;
    m->pi_q.integrator = 0.0f;
    m->id_ref_fw = 0.0f;
}
```

Key change: **velRef = current measured speed**.  Now error ≈ 0 at
first CL tick → Iq_ref ≈ 12 A (preload).  Iq holds smooth across the
boundary.  Then velRef slews toward speed_target via
`AN_CL_VELREF_SLEW_RPS2` while speed PI integrator gradually adjusts
to deliver the right torque.

## Per-profile motor model

As of 2026-04-27, AN1078 reads motor params from `gspParams` (per-
profile, GUI-editable) instead of compile-time `#defines`:

| Param | Source | Notes |
|-------|--------|-------|
| `Rs` | `gspParams.focRsMilliOhm` × 1e-3 | F_PLANT depends on it |
| `Ls` | `gspParams.focLsMicroH` × 1e-6 | G_PLANT, F_PLANT |
| `λ` (Ke) | `gspParams.focKeUvSRad` × 1e-6 | Used in handoff BEMF threshold |
| Max speed | `gspParams.focMaxElecRadS` | Throttle map ceiling |
| Pole pairs | still `AN_NOPOLESPAIRS` `#define` | Macros depend on it; refactor TODO |
| End speed | still `AN_END_SPEED_RPM_MECH` `#define` | Refactor TODO |

**Switch motor profiles in GUI** → motor model + max speed automatically
re-derive on next motor start.  No firmware recompile.

When `Rs/Ls/Ke` change via SET_PARAM (GUI Parameter Editor or profile
load), `AN_SMCInit(&s_foc_an.smc)` is called immediately (in idle state)
to recompute `F_PLANT`, `G_PLANT` from the new values.

## Tuning summary by motor

| Motor / Vbus | Rs | Ls | Ke | focMaxElecRadS |
|-------------|-----|------|------|----------------|
| Hurst 300 / 24V | 0.534 Ω | 471 µH | 0.00742 V·s/rad | 2000 (~19k eRPM) |
| A2212 / 12V | 65 mΩ | 30 µH | 0.000563 | 12000 (~115k eRPM) |
| **2810 / 24V** | **22 mΩ** | **10 µH** | **0.000583** | **25000 (~239k eRPM)** |
| 5055 / 14.8V | 50 mΩ | 18 µH | 0.001355 | 7000 (~67k eRPM) |

## Validation runs

- **2810 @ 24V**: 25.8s clean, idle stable at 14k eRPM, peak 213k eRPM
  with FW.  0 faults.  Build hash `0xFF530391`.
- **A2212 @ 12V**: 20s clean.  Idle stable at 14k eRPM, throttle map
  reaches voltage limit at ~115k eRPM.  After per-profile maxSpeed
  fix, full pot range responsive.

## What's still hardcoded (TODOs)

1. **`AN_NOPOLESPAIRS`**: used in macros that compute end-speed
   constants, hard to make runtime.  Both A2212 and 2810 are 7PP so
   not a practical issue today.  Future: derive from
   `gspParams.motorPolePairs`.
2. **`AN_END_SPEED_RPM_MECH`**: same — needs macro refactor.  Single
   value (2000 RPM mech) works across motors at the rates we tested.
3. **Speed PI gains** (`AN_KP_SPD`, `AN_KI_SPD`): currently per-build,
   but the gains are dimensionally tied to speed range so should
   probably scale with `focMaxElecRadS` per profile.  Today they're
   0.015/0.30 which works for both motors at our test ranges.
