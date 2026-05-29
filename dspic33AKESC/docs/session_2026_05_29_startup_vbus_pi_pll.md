# 2026-05-29 Session — Sine Startup, Vbus Protection, PI PLL-Form

**Commit**: `acad22b` (bench-validated)
**Bench session**: 47s CL operation, 8+ top-speed runs to 233k eRPM, **zero faults**, Vbus capped at 30.1V (previously 47.8V → PSU OV → UV cascade)

---

## Wins

### 1. Sine startup enabled and tuned
`FEATURE_SINE_STARTUP=1`. The smooth sinusoidal V/f startup replaces the trapezoidal forced commutation path.

Per-profile tuning (2810 at 24V):
- `sineRampModPct = 5` (EEPROM, profile 2) — OL_RAMP peak current 12.1A → 5.6A
- `MORPH_CONVERGE_SECTORS = 12` — doubles sine→trap blend duration (was 6), halves dV/dt on float-phase
- `SINE_TRAP_DUTY_NUM = 16` — trap_duty at α=256 matches `clIdleDutyPct=6%`, eliminating MORPH→CL displayed duty step

**The big MORPH win** — at α=256 (sine→trap transition), set `garudaData.duty = MIN_DUTY` instead of the full trap_duty. This holds the bridge at near-coast voltage during sub-B (windowed Hi-Z) while HWZC captures clean ZCs:
- MORPH peak Ibus: **21.83A → 0.78A** (97% reduction)
- MORPH peak Ia: 21.95A → 15A (residual is the structural CL-entry spike, not MORPH itself)

Code: `garuda_service.c` line ~1985 area, in the α=256 transition block.

### 2. Vbus 3-tier protection stack
The bench PSU had been tripping OV during regen on hard throttle releases. Added an emergency hold tier on top of the existing regen brake:

```
Vbus regime       │ Slew-down rate          │ Mechanism
──────────────────┼─────────────────────────┼──────────────────────
< 28V (normal)    │ Full (5%/ms)            │ —
28V → 30V         │ /16 = 0.31%/ms          │ Regen brake (existing)
> 30V (emergency) │ 0 (frozen)              │ Emergency hold (NEW)
```

`FEATURE_VBUS_EMERGENCY_HOLD=1`. When Vbus crosses 30V, `effectiveDownRate` is forced to 0 — the duty literally cannot decrease, breaking the positive-feedback regen loop. Sticky hysteresis 30V→27V + 20ms min hold prevents chatter. Slew-UP is unaffected (rising duty consumes regen energy, helps the bus).

Bench result: bus capped at 30.1V across multiple full-speed regen events that previously sent it to 47.8V.

Code: `garuda_service.c` line ~3055 area, in the duty-slew-down block.

### 3. HWZC PI gain spec — PLL form
Express the timing PI's Kp and Ki via natural frequency ω_n and damping ratio ζ instead of raw gains:

```c
Kp = 2ζω_n          // proportional (transient response)
Ki = ω_n²           // integral (steady-state tracking)
```

Bench-validated combo: `ω_n=0.25, ζ=0.5` → exactly Kp=0.25, Ki=0.0625 (numerical gains unchanged, behavior identical). Reverse-derived from the original bit-shift gains.

Tuning interface now physics-intuitive:
- Raise ω_n for faster tracking, lower for noise immunity
- Raise ζ for more damping (less overshoot), default 0.5 is slightly underdamped

Code: `garuda_config.h` `HWZC_PI_OMEGA_N` + `HWZC_PI_DAMPING_RATIO` defines, with `HWZC_PI_KP_FLOAT` / `HWZC_PI_KI_FLOAT` derived from them at compile time.

---

## What was tried and reverted

### Sine startup attempts that failed
- **`rampTargetErpm = 8000` (with `sineRampModPct=9`)**: rotor slipped during ramp, HWZC latched on noise → phantom 162-252k eRPM telemetry
- **`rampTargetErpm = 10000` (with `sineRampModPct=5`)**: rotor stalled at acceleration boundary, same failure mode
- Conclusion: open-loop ramp endpoint on this 2810 motor at 24V is structurally capped at ~6-7k eRPM. Pushing higher requires either much more amplitude (which itself causes spikes) or a different startup architecture.

### MORPH float-phase fade (no effect)
Linearly faded the float-phase target from `trapFloat` down to `MIN_DUTY` over the last 25% of the sine→trap blend, expecting to drain float-winding inductor current before HiZ release. Bench showed zero measurable effect. Removed.

### CL-side coast + 100ms soft slew (limited effect)
- `MORPH_TO_CL_COAST_TICKS=100` (4ms coast at `MIN_DUTY` after MORPH→CL)
- `SOFT_SLEW_TOTAL_TICKS=2400` (100ms linear slew from `MIN_DUTY` to `mappedDuty`)
- Reason: `MIN_DUTY` is structurally 5.4% of LOOPTIME (= 2 × DEADTIME at 45kHz PWM). At 24V that's 1.3V applied. Against a 3k eRPM rotor (BEMF 0.31V), the gap × 1/R = 20A regardless of any slew. The slew was operating in a 0.6% range (5.4% → 6%) and couldn't get below 5.4%. Removed.

### Smoothed I-term (broke the loop)
Added IIR-filtered delta feeding the integrator (α=0.125). HWZC immediately collapsed → phantom 250k eRPM at idle throttle.

**Root cause**: filter cutoff (α=0.125 rad/sample) was set at HALF of the PI's natural frequency (ω_n=0.25 rad/sample). The IIR was filtering out the very signals the PI was trying to track → added phase lag → loop unstable.

**Lesson**: in-loop low-pass filters must have cutoff **above** the loop bandwidth, ideally 2-3×. With ω_n=0.25, smoothing α would need to be ≥0.5 to be safe — at which point only ~2 sectors of averaging happens and the benefit is minimal.

Reverted to clean state (no broken code path retained).

---

## Structural limits identified

### CL-entry current spike on low-Rs motors
Independent of any startup tuning, the 2810 hits ~22A briefly at CL takeover because:
- Rotor exits sine ramp at 3k eRPM (BEMF = 0.31V)
- Bridge applies CL_IDLE_DUTY ≈ 6% × 24V = 1.44V
- Gap / Rs = 1.13V / 0.05Ω = **22.6A**
- This pulse is also what physically accelerates the rotor up to 14k equilibrium speed in ~250ms

The pulse is bounded by `OC_SW_LIMIT` (18A) + bench shunt saturation (22A) + brief duration (<2ms). Not damaging, but visible/audible as a "jerk". Three real fixes:
1. Higher-amplitude sine ramp reaching closer to 14k eRPM rotor speed — capped by open-loop slip on this motor
2. Higher motor Rs (different motor) — not a firmware path
3. Switch to FOC startup (AN1078 build already does this end-to-end) — already proven to 213k eRPM

### `MIN_DUTY` floor as voltage floor
With 300ns deadtime at 45kHz PWM, `MIN_DUTY = 2 × DEADTIME = 3840 ticks = 5.4% of LOOPTIME`. Below this, the high-side never conducts (deadtime clipped). So any "coast" using complementary PWM still applies ~1.3V at 24V Vbus — enough to spike current on low-Rs motors.

True coast requires full bridge tri-state (`HAL_PWM_AllPhasesHiZ` equivalent — would need to add).

---

## Remaining improvement list

Ordered by impact-to-risk ratio after this session's lessons.

### Recommended next: Speed PI (CL state)
Per-ZC interval-based speed PID, AM32 architecture:
- Throttle ADC → target stepPeriodHR (not target eRPM, avoids div math at runtime)
- error = measured stepPeriodHR − target stepPeriodHR
- PI output → mappedDuty (replaces direct throttle scaling)
- Anti-windup at MIN_DUTY and MAX_DUTY clamps
- Integral disabled until 100 ZCs after CL entry
- PLL-form gain spec (Kp_speed, Ki_speed derived from ω_n_speed, ζ_speed)
- Essential for unipolar mode (where no braking exists)
- Improves load behavior in normal mode too — speed holds under prop load

Estimated effort: 100-150 lines. Builds on the PLL form work from this session.

### Other safe additions
- **Demag detection**: monitor current decay duration after commutation, flag if exceeds expected. Observer only, no actuation. ~50 lines.
- **BEMF filter compensation Path 2/3**: push past current 203k milestone, tune AMP_PCT=80-85 and MAX_OFFSET=700-800. Empirical tuning, see `docs/bemf_filter_compensation.md`.
- **Setpoint smoothing (corrected)**: smooth `capValueHR` itself (outside the feedback loop), then compute delta. Doesn't add phase lag inside the PI. ~30 lines.

### Bigger features
- **Variable PWM frequency**: AM32-style speed-adaptive PWM (30/45/60kHz tiers with IIR transition). Touches a lot of code that depends on PWM frequency.
- **EEPROM parameter storage** for sine startup tunables (`MORPH_CONVERGE_SECTORS`, `SINE_TRAP_DUTY_NUM`) — currently compile-time.

### Architectural reconsiderations
- **FOC for startup, 6-step for high-RPM**: the AN1078 build already does sinusoidal start-to-finish to 213k eRPM with no MORPH discontinuity. Could be the cleaner answer to the CL-entry spike than continued tuning of the 6-step path.

---

## Files touched this session

```
dspic33AKESC/garuda_config.h     +PLL-form PI, +sine tunables, +emergency Vbus
dspic33AKESC/garuda_service.c    +MORPH sub-B coast, +Vbus emergency hold
dspic33AKESC/gsp/gsp_params.c    sineRampModPct=5 profile 2
tools/step6_reset_profile.py     +sineAlign/sineRamp param checks
tools/pi_pll_simulator.py        +FloatPIDefensive class
```

Commit message: see `acad22b` body.
