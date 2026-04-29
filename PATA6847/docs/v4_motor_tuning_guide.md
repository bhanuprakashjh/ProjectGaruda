# V4 Motor Tuning Guide — Symptom → Knob

When bringing up a new motor profile or debugging a startup issue, find the symptom in the table below and adjust the listed knob. All parameters live in `garuda_6step_ck.X/garuda_config.h` unless noted.

## Quick Decision Tree

```
Motor doesn't move at all (no audible commutation)
  → Check ALIGN/PWM enable path; not a tuning issue

Motor cogs/jitters during align but never spins
  → ALIGN_DUTY too low, or ALIGN_TIME_MS too short

Motor spins during OL_RAMP but eRPM doesn't reach handoff target
  → Rotor isn't tracking forced commutation. RAMP_DUTY_CAP too low,
    or RAMP_ACCEL_ERPM_S too fast

OL_RAMP completes but motor decays at CL entry, returns to IDLE
  → Classic stage-2 stall. PI overcorrects on noisy low-BEMF captures.
    See "Startup decay / death spiral" below

CL succeeds but motor doesn't accelerate beyond a few k eRPM at idle pot
  → V4_MIN_AMPLITUDE_PROFILE too low for this motor's load

CL works but eRPM jumps to phantom 700k+ and trips
  → PI runaway. RAMP_ACCEL_ERPM_S too slow (rotor not at MIN_STEP eRPM
    when CL takes over → seed mismatch). Or V4_PHASE_ADVANCE_DEG wrong
    direction for this motor/Vbus

Block-comm doesn't engage at full pot
  → V4_BLOCK_ENTER_ERPM higher than this motor/Vbus combination can reach.
    Check your eRPM ceiling = KV × Vbus × PoleP (with losses ~ 0.85×)

Motor desyncs when releasing pot from full speed
  → Block-comm exit transient. Increase blockExitCooldown
    (sector_pi.c:176 default 25 = 500 ms) or widen Q15 hysteresis

After first desync, motor won't restart cleanly
  → Known state-machine reset gap. Power cycle to clear (will be fixed
    in a future session)
```

## Parameters by Symptom

### A. OL_RAMP can't reach handoff speed

Symptom: telemetry shows OL_RAMP for 2-3 sec, eRPM stalls below the implied target (e.g., reaches only 2500 when target is 4000), CL never starts cleanly.

| Knob | Effect | When to change |
|---|---|---|
| `RAMP_DUTY_CAP` | Max % duty during ramp | Increase if motor lacks torque to track ramp. Limit: ILIM_DAC trips. |
| `RAMP_ACCEL_ERPM_S` | eRPM/s acceleration rate | Decrease if rotor can't keep up with forced step rate |
| `ALIGN_DUTY` | Hold-current % during align | Increase if rotor doesn't lock to align position before ramp |
| `ALIGN_TIME_MS` | Align duration | Increase for heavier rotors (typically 100-200 ms is enough) |

**Profile 1 / Profile 2 / Profile 3:** `RAMP_ACCEL_ERPM_S = 1500` is the proven bench cadence. Don't go below 1000 without bench testing.

### B. Startup decay / "death spiral" at CL entry

Symptom: OL_RAMP completes, CL takes over briefly (eRPM 5-8k), `Cap%` ~30%, motor then decays back toward 4-5k and goes IDLE. Common on first-start, sometimes succeeds on retry.

Root cause: PI's `setValue` formula uses `V4_PHASE_ADVANCE_DEG` (10° default) as a poll-lag bias. At low BEMF, captures are noisy → PI overshoots → wrong commutation timing → braking torque → motor slows → BEMF gets weaker → runaway slowdown.

| Knob | Effect | When to change |
|---|---|---|
| `V4_PHASE_ADVANCE_DEG` | PI's expected ZC offset | Lower (e.g., 7°) makes PI less aggressive at low speed. **Caution:** breaks high-speed on some motors. Tested: 7° fixed A2212@12V startup, **broke** 2810@24V (PI runaway). Per-motor knob. |
| `V4_MIN_AMPLITUDE_PROFILE` | Idle Q15 amplitude floor | Raise to give motor more torque to escape low-BEMF regime. Typical: 5000 (15.3%) for low-Z motors, 16384 (50%) for high-Z motors |
| `V4_ALIGN_DURATION_MS` | V4-specific align time | Longer = better starting position lock |
| `V4_STARTUP_CURRENT_MA` | (Currently advisory only — not actuated) | Change with caution |

### C. PI runaway — phantom 700k+ eRPM at CL

Symptom: CL takes over, eRPM immediately jumps to nonsense (500k-1M), SP mode triggers, motor faults out.

Root cause: `timerPeriod` seeded from `rampStepPeriod` (forced commutation rate at OL exit). If rotor's actual speed is slower than that, PI sees captures arriving "early," responds by commutating faster, motor falls further behind, runaway.

| Knob | Effect |
|---|---|
| `RAMP_ACCEL_ERPM_S` | **Increase** so rotor reaches `MIN_STEP_PERIOD` eRPM by handoff. Counter-intuitive: too-slow ramp causes runaway because rotor doesn't catch up. |
| `MIN_STEP_PERIOD` | **Increase** (longer step period = lower handoff eRPM) for motors that can't reach 4000 eRPM under load |
| `RAMP_DUTY_CAP` | Increase so motor has torque to follow faster ramp |

### D. Motor sustains but doesn't accelerate at idle pot

Symptom: pot at zero, CL holds at ~5k eRPM, doesn't climb to natural no-load speed (which would be `KV × Vbus × PP / 6` = e.g., 110k for A2212@12V).

| Knob | Effect |
|---|---|
| `V4_MIN_AMPLITUDE_PROFILE` | Idle Q15 amplitude floor. Raise to push motor harder at zero-pot. |
| `CL_IDLE_DUTY_PERCENT` | Effective idle duty %. Raise for high-Rs motors that need V to make I. |

**Caution:** At zero pot, `V4_MIN_AMPLITUDE_PROFILE` becomes the steady-state amplitude. Higher = more idle current draw. For low-Rs motors (65 mΩ), 50% Q15 = 200 A peak — way too high. For high-Rs motors (16 Ω), 50% is fine because Rs limits current.

### E. Block-commutation doesn't engage at full pot

Symptom: motor reaches near-peak eRPM at full pot, BLK indicator doesn't appear in telemetry.

| Knob | Effect |
|---|---|
| `V4_BLOCK_ENTER_ERPM` | Threshold to enter block-comm. Set just below your motor's natural saturation eRPM. |
| `V4_BLOCK_EXIT_ERPM` | Hysteresis (typically 0.85 × enter) |

**Rule of thumb:** ENTER threshold = motor peak eRPM × 0.85. For example, A2212@12V peaks ~123k → set ENTER to 100k. 2810@25V peaks ~237k → ENTER 150k. HiZ1460@30V theoretical 306k → ENTER 250k.

### F. Block-comm exit causes desync at high speed

Symptom: motor at 200k+ eRPM in BLK mode, pot released, BLK exits, motor decays normally for 1-2 sec then desyncs/IDLE. Vbus often shows brief spike (regen).

| Knob | Effect |
|---|---|
| `blockExitCooldown` (sector_pi.c:176, hardcoded 25 → 500 ms) | Longer cooldown lets motor settle on PWM before re-evaluating BLK |
| The 90% Q15 amplitude exit threshold (sector_pi.c:1027) | Raise to 95% to reduce window of unstable decay |

### G. Top-end speed is lower than expected

Theoretical max eRPM = KV × Vbus × PoleP. Practical typically 80-95% of that depending on losses. If you're seeing only 60-70% of theoretical:

| Knob | Effect |
|---|---|
| `MAX_DUTY` (line 50, `LOOPTIME_TCY - 200U`) | Already at maximum effective; not the issue |
| `TIMING_ADVANCE_LEVEL` (per-profile, scheduler advance) | Higher level (3=22.5°, 4=30°) helps top-end on faster motors |
| `V4_PHASE_ADVANCE_DEG` (PI advance) | Higher (10° → 12°) compensates for measurement latency at high speed; too high causes desyncs |
| Block-comm | If `V4_BLOCK_ENTER_ERPM` is reachable, BLK gives ~5% boost |

## Profile-Specific Notes (validated 2026-04-29)

### Profile 1 — A2212 1400KV @ 12V
- Validated peak with 2810 motor on bench: **235-238k eRPM**
- Cadence (`RAMP_ACCEL=1500`, `INITIAL_STEP=800`, `MIN_STEP=50`) is the proven baseline
- Default `MOTOR_PROFILE` value — flash this as the bench config
- Block-comm: `ENTER=100k / EXIT=85k`

### Profile 2 — 2810 1350KV @ 25V
- Profile 2's settings have NOT been independently validated. The bench tests use profile 1 settings driving the 2810 motor.
- `RAMP_ACCEL_ERPM_S` was 500 (broken — fixed to 1500 on 2026-04-29)
- Block-comm: `ENTER=150k / EXIT=130k`

### Profile 3 — HiZ1460 16Ω @ 30V (UNTESTED)
- Untested as of this writing. Inherits 2810-style ZC detection (`FEATURE_IC_ZC_ADAPTIVE`, `ZC_BLANK_FLOOR=15µs`, `TIMING_ADVANCE_LEVEL=3`).
- Profile-1-style ramp cadence (proven on bench).
- High duty caps because Rs is the current limit, not duty.
- If startup fails → likely needs `ALIGN_DUTY` higher (try 60%) or `V4_MIN_AMPLITUDE_PROFILE` lower (try 12000) depending on symptom direction.

## What Each Profile Macro Means

| Macro | Default location | Actuation |
|---|---|---|
| `MOTOR_RS_MILLIOHM` | per-profile block | Telemetry only — not used in firmware logic |
| `MOTOR_LS_MICROH` | per-profile block | Telemetry only |
| `MOTOR_KV` | per-profile block | Telemetry only |
| `MOTOR_POLE_PAIRS` | per-profile block | Used in eRPM↔mech-RPM conversions |
| `ALIGN_DUTY` | per-profile | Forced duty during align phase |
| `ALIGN_TIME_MS` | per-profile | Align duration (V3 path) |
| `V4_ALIGN_DURATION_MS` | V4 startup block | Align duration (V4 path — actually used) |
| `INITIAL_STEP_PERIOD` | per-profile | Starting commutation period in Timer1 ticks |
| `MIN_STEP_PERIOD` | per-profile | OL→CL handoff threshold (Timer1 ticks; smaller = higher target eRPM) |
| `RAMP_ACCEL_ERPM_S` | per-profile | OL_RAMP acceleration rate |
| `RAMP_DUTY_CAP` | per-profile | Max duty during OL ramp |
| `V4_PHASE_ADVANCE_DEG` | global (config:791) | PI's poll-lag bias (NOT scheduler advance) |
| `V4_MIN_AMPLITUDE_PROFILE` | per-profile (V4 block) | Q15 floor for actualAmplitude |
| `V4_BLOCK_ENTER_ERPM` | per-profile (V4 block) | Block-comm engagement |
| `V4_BLOCK_EXIT_ERPM` | per-profile (V4 block) | Block-comm exit |
| `CL_IDLE_DUTY_PERCENT` | per-profile or shared default | Idle duty in CL |

## What's Documented Where

- **Block commutation:** `v4_block_commutation.md` (this directory)
- **228k milestone (pre-block):** `v4_228k_milestone_session.md`
- **Architecture:** `v4_architecture.md`
- **This file:** symptom → knob lookup
