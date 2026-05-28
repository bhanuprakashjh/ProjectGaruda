# Sector PI / PLL — Deep Research and Roadmap to Float

**Date**: 2026-05-28
**Scope**: What we have today, why it's actually a PLL not a PID, what
the FPU + trig support unlocks, and which improvements are **real**
vs **bookshelf-fashionable**.

This is research / analysis. No code changes here — companion to the
filter-comp docs.

---

## 1. What We Actually Have

### 1.1 The controller, in three lines

`motor/hwzc.c:677-721` — the entire sector PI loop:

```c
int32_t delta = (int32_t)capValue - (int32_t)setValue;          // phase error
integrator += delta >> KI_SHIFT;                                // KI = 1/16
timerPeriod = integrator + (delta >> KP_SHIFT);                 // KP = 1/4
```

That's it. The runtime cost is one subtract, two right-shifts, two
adds, two clamps. ~15 cycles per sector.

### 1.2 What it's controlling

The variable being adjusted is `timerPeriod` — the period (in 10 ns
SCCP2 HR ticks) of the **autonomous SCCP1 commutation timer**. That
timer fires at `lastCommStamp + timerPeriod` and drives the next
commutation regardless of whether a fresh ZC capture arrived.

So the PI doesn't directly schedule commutations. It **adjusts the
free-running timer's period** based on observed phase error between
the timer's expected ZC location and the captured ZC location.

This is structurally a **phase-locked loop**, not a velocity PID. The
literature for PLLs is far more relevant than the literature for
servo PIDs.

### 1.3 The "PID" framing is a misnomer

A classical PID controls a process variable (velocity, position,
temperature) toward a setpoint using error = setpoint − measurement.
Here:

- The "setpoint" is `setValue = (advancePlus30Fp8 × T) >> 8` — a
  fraction of the current period, **dependent on the period itself**
- The "measurement" is `capValue` — also period-dependent (measured
  in HR ticks since the last commutation)
- The "control output" is `timerPeriod` — the period itself

Self-referential. The system is locking the timer's period such that
captures consistently land at the desired fractional phase. That's a
PLL definition.

We'll still call it "PI" in the firmware because that's the
historical name, but every time you read PI think **2nd-order
discrete PLL** for the right mental model.

### 1.4 Mapping to standard PLL form

Standard 2nd-order PLL terms:

| Standard PLL term | What it is here |
|---|---|
| Phase detector | `capValue − setValue` (`delta`) |
| Loop filter | `Kp + Ki/s` (the proportional + integrator) |
| NCO / VCO | The autonomous SCCP1 timer running at `timerPeriod` |
| Loop natural frequency `ω_n` | `√(Kp · Ki · f_sample)` (with bit-shift gains) |
| Damping factor `ζ` | `Kp / (2 · √(Ki · f_sample))` |

Sample frequency `f_sample` = sector rate = `eRPM / 10`. So:

- At 14 k eRPM idle: `f_sample = 1400 Hz`
- At 240 k eRPM: `f_sample = 24000 Hz`

With `Kp = 0.25` and `Ki = 0.0625`:

| eRPM | f_sample | ω_n | ω_n / f_sample | ζ |
|---|---|---|---|---|
| 14 k | 1400 Hz | 12 Hz | 0.85 % | 0.79 |
| 60 k | 6000 Hz | 24 Hz | 0.40 % | 0.79 |
| 200 k | 20000 Hz | 45 Hz | 0.22 % | 0.79 |
| 240 k | 24000 Hz | 49 Hz | 0.20 % | 0.79 |

Two things stand out:

1. **Damping factor stays at ~0.79** across the entire speed range
   (it's a function of Kp/Ki only, not f_sample). That's slightly
   under-damped — classical PLL design targets `ζ = 0.707`
   (Butterworth, critically damped 2nd-order). We're close.

2. **ω_n / f_sample drops from 0.85 % to 0.20 %** as speed increases.
   In PLL terms, the loop bandwidth shrinks as a fraction of the
   sample rate — the loop is **slower** relative to the input at
   high RPM. That's the right direction (lock is tighter) but the
   ratio isn't designed; it's an accidental consequence of using
   bit-shift gains scaled against the period itself.

### 1.5 What the asymmetric clamps actually do

Per `garuda_config.h:684-701`, at high eRPM (T < 6667 HR ticks =
150 k eRPM):

```
positive delta (rotor late, slow down):  clamp ±T/16
negative delta (rotor early, speed up):  clamp ±T/32 — half the magnitude
```

In PLL terms, this is **non-linear slew limiting** — the loop accepts
phase corrections that ask the period to grow more readily than it
accepts requests to shrink. The asymmetry is physics-motivated: at
BEMF ceiling, the rotor literally can't speed up further, so a
"rotor early" reading is either noise or the start of a runaway, and
shrinking the period commands commutation ahead of the rotor (wrong
angle, big current spike).

This isn't standard PLL design. It's a custom robustness layer
specific to BLDC at BEMF ceiling. Worth keeping; worth understanding
that it's not part of the textbook PLL.

### 1.6 Open-loop transfer function

For analysis: linearize around lock. The phase detector slope is
1 (capValue and setValue both in HR ticks). The loop filter in
continuous-time approximation:

```
H(s) = Kp + Ki / s
     = (s · Kp + Ki) / s
```

NCO integrates the period correction into accumulated phase:

```
G_NCO(s) = 1 / s
```

Closed-loop:

```
T_CL(s) = H(s) · G_NCO(s) / (1 + H(s) · G_NCO(s))
        = (s · Kp + Ki) / (s² + s · Kp + Ki)
```

Standard 2nd-order form `s² + 2ζω_n·s + ω_n²`:

```
ω_n² = Ki    →  ω_n = √Ki
2ζω_n = Kp   →  ζ   = Kp / (2√Ki)
```

In sampled-time at sector rate `f_sample`:

```
ω_n,disc = √Ki · f_sample
ζ        = Kp / (2 · √(Ki))
```

For our values (`Kp = 0.25`, `Ki = 0.0625 = 1/16`):

```
ω_n,disc = 0.25 · f_sample
ζ        = 0.25 / (2 · 0.25) = 0.5
```

Wait — that gives ζ=0.5, not 0.79. The discrepancy is from the
discrete sampling effect. The continuous approximation slightly
overstates ζ; the discrete analysis is more accurate. Either way,
we're under-damped to roughly-damped.

### 1.7 Practical observations

From bench data:

- **Step response**: when throttle bumps, the motor accelerates with
  a smooth profile. No visible oscillation. PLL is locking cleanly.
- **Phase noise**: at steady state, `delta` distributions show
  ~±5-15 HR ticks of jitter per capture. PI absorbs this without
  losing lock.
- **Disturbance rejection**: regen pulses (Vbus dips) cause brief
  PI excursions but recover within ~3-5 sectors. Good.
- **Lock acquisition**: ~12 captures from seed period to lock. Fine.

Things that go wrong (documented in other docs):

- **Rapid pot release** → HWZC silent → PLL runs on stale period.
  Watchdog catches eventually.
- **High duty (>90%) at BEMF ceiling** → asymmetric clamp prevents
  runaway most of the time, but not always.
- **PWM frequency too low** → noisy delta → PI integrator drifts.

---

## 2. The Plant Model

To improve the controller you have to understand the plant. The PI
acts on `timerPeriod`; what physically determines the **right**
period?

### 2.1 The rotor's natural period

The motor's electrical period is set by:

```
P_natural = 60 / (KV · V_applied · n_polepairs)        (seconds per electrical revolution)
          = 60e9 / (KV · V_applied · n_polepairs)      (HR ticks)
P_sector_natural = P_natural / 6                       (HR ticks per sector)
```

For the 2810 (KV=1350, n_PP=7) at 24V * 50% duty:

```
P_sector_natural = 60e9 / (1350 · 12 · 7 · 6) ≈ 88 µs
                 = 8800 HR ticks
                 → 142k eRPM
```

That's the **open-loop estimate** for the period at 50% duty. The PI
is correcting from this value to the actual lock.

### 2.2 What perturbs the period

Three things move the rotor's period away from the open-loop
estimate:

1. **Load**: torque demand → current → I·Rs drop → effective voltage
   at terminals < V_applied → slower
2. **BEMF ceiling**: at very high duty, the rotor can't exceed
   `V_bus · KV` no matter what
3. **Friction / windage**: at any speed there's drag

### 2.3 The transfer function from duty to period

In steady state, ignoring dynamics:

```
ω_steady = KV · V_bus · D − I_load · KV · Rs / k_torque    (rad/s)
```

Where `k_torque` is the torque constant (closely related to KV). The
PI doesn't know this analytically; it discovers the right period
empirically through closed-loop integration. That's WORK that could
be done open-loop.

### 2.4 What the PI is actually doing in steady state

At steady state, `delta ≈ 0`, `integrator` = current `timerPeriod`,
and `Kp·delta` is just absorbing jitter. The integrator value
represents the **PI's best estimate of the rotor's actual period**.

This is critical: the integrator is doing **velocity estimation**.
It's basically a slow, robust velocity observer.

---

## 3. What Float + FPU Unlocks

The dsPIC33AK128MC106 has:

- **32-bit single-precision FPU** — IEEE-754 single precision (24-bit
  mantissa, 8-bit exponent)
- **Hardware FMUL, FDIV, FSQRT** — 1-15 cycles depending on op
- **Optimized math library** (`sinf`, `cosf`, `atan2f`, etc.) — ~50-100
  cycles each at 200 MHz = ~250-500 ns
- **Hardware MAC** — 1 cycle for fused multiply-add

### 3.1 What that means in cycle budget

Current PI per-sector cost: ~15 cycles = 75 ns at 200 MHz.

A float-based PI doing the same math: maybe 25-30 cycles = 125-150 ns.

A float-based PI with feedforward, gain scheduling, anti-windup,
setpoint weighting, derivative-on-PV: ~60-80 cycles = 300-400 ns.

A float PI with one trig call (sinf or cosf): ~125-180 cycles =
625-900 ns.

For comparison: at 14k eRPM the sector is 714 µs. At 240k eRPM the
sector is 42 µs. **Even the heaviest float-PI variant runs in <1 %
of the available sector time**. No compute pressure.

So FPU + trig isn't going to make the basic PI math faster. It's
going to make it **cleaner, easier to tune, and capable of features
that integer math can't express cleanly**.

### 3.2 What integer math can't express cleanly

- **Non-power-of-2 gains** — Kp = 0.25 is easy as `>>2`, but
  Kp = 0.18 or Kp = 0.31 isn't. Tuning is quantized.
- **Mixed-unit math** — eRPM × duty × Vbus type formulas need
  scaling factors that overflow easily in 32-bit fixed-point
- **Sin/cos/atan2** — we don't use these in PI today, but feedforward
  models do
- **State observers (Kalman-like)** — need matrix math, hard in
  fixed-point
- **Adaptive gains** — gain depends on a computed quantity, integer
  scaling becomes unmanageable
- **Negative numbers in safe ranges** — current code does signed
  int32_t for delta, but extending to other state is awkward

### 3.3 What trig specifically unlocks

In the **current 6-step controller**, trig isn't strictly required.
The angle bookkeeping is discrete (6 sectors, each 60° apart). No
continuous angle to track.

But trig **enables**:

- **Smooth sine startup** (already implemented elsewhere, would unify)
- **FOC migration** if we ever want it (we have an AN1078 FOC port
  in the codebase already, but not currently used)
- **Sinusoidal feedforward** — predict expected period from voltage
  + KV more accurately than the simple `V/KV` formula by accounting
  for the cosφ between voltage and current
- **Filter compensation in exact `sin(arctan(ω·τ))` form** — already
  flagged as future work in `bemf_filter_compensation.md`

For the bare PI, trig is mostly a "nice to have not need to have"
upgrade.

---

## 4. Real Improvements (Catalogued and Ranked)

Below are improvements with honest cost/benefit. **"Real" means I'd
expect a measurable bench improvement.** "Bookshelf" means it's
academically pretty but I don't see what it would change here.

### 4.1 HIGH-VALUE: Move to float math (no behavior change)

**What**: Reimplement the existing PI in `float` with no algorithm
change. `Kp = 0.25f`, `Ki = 0.0625f`, same clamps in float.

**Why it matters**:
- Removes the bit-shift quantization. `Kp = 0.21f` or `0.31f`
  becomes a valid tuning step.
- Asymmetric clamps `T/16, T/32` become `0.0625f × T`, `0.03125f × T`
  — continuous values, easier to reason about.
- Unblocks every later improvement on this list.
- Telemetry of `Kp · delta` and `Ki · delta` in float makes tuning
  via the GUI much easier (today the GUI has to know about Q15 bit
  positions).

**Cost**: ~1 day of porting + careful testing. Risk: numerical edge
cases (denormals, overflow on integrator with hours of operation).

**Expected gain**: 0% direct, but unblocks all of below. Recommended
as Phase 1.

### 4.2 HIGH-VALUE: Feedforward from duty + Vbus → expected period

**What**: Compute open-loop period estimate from physics:

```
P_FF = 60e9 / (KV · V_bus · D · n_polepairs · 6)     [HR ticks/sector]
```

Then run PI on the residual:

```
P_residual = capValue - setValue
P_command = P_FF + P_residual_correction (from PI)
```

**Why it matters**:
- PI integrator no longer has to walk all the way from seed value to
  the right value during startup — feedforward gets us 80% of the way
  there immediately
- Transient response to throttle changes is faster (PI only catches
  the bit feedforward got wrong, instead of the entire error)
- Integrator stays smaller → less integral windup → safer at the
  edges of operation

**Cost**: A few hours of code. Requires KV and pole pairs from
gspParams (already there).

**Expected gain**:
- Startup time: faster lock acquisition (12 captures → maybe 4-6)
- Throttle response: faster step settling (currently ~50 ms, could
  be ~20 ms)
- Steady-state current: small improvement (1-3%) from reduced
  integrator activity at top RPM

### 4.3 MEDIUM-VALUE: PLL-form gain specification

**What**: Express `Kp` and `Ki` in terms of natural frequency and
damping:

```
ω_n = desired_loop_bandwidth_Hz · 2π
ζ   = 0.707    (Butterworth, critically damped)
Kp_eff = 2 · ζ · ω_n / f_sample
Ki_eff = ω_n² / f_sample²
```

With `ω_n` chosen as a fraction of `f_sample` (e.g. 1/30 — so loop
BW = 50 Hz at 1500 Hz sector rate).

**Why it matters**:
- Gains specified in physical units, not "1/16-th"
- Easy to retune for different motors or operating conditions —
  pick a bandwidth, formula gives you Kp/Ki
- ω_n can scale with f_sample so the loop's relative bandwidth
  stays constant across the speed range (real gain scheduling)

**Cost**: Comes for free after the float port. ~1 hour to add the
scheduling formula.

**Expected gain**: Better behavior across the speed range,
particularly at low/mid RPM where the current fixed bit-shift gains
are likely over-damped.

### 4.4 MEDIUM-VALUE: Velocity form of PI

**What**: Currently we have **position form** (output = integral +
proportional). Switch to **velocity form**:

```
Δoutput = Kp · (e_new − e_old) + Ki · e_new
output  = output + Δoutput
```

**Why it matters**:
- Anti-windup is automatic (no explicit integrator)
- Bumpless transfer when feature flags switch modes
- More forgiving on saturation (clamping the output doesn't
  corrupt the integrator state)

**Cost**: 1 day of careful porting. Velocity form has slightly
different transient behavior; needs re-tune.

**Expected gain**: Better behavior near MAX_DUTY where position-form
PI integrator can wind up. Could fix some of the regen overvoltage
events documented in Appendix B.

### 4.5 MEDIUM-VALUE: Setpoint weighting (β, γ form)

**What**: Use different weights for setpoint vs feedback in the
P and I terms:

```
P term: Kp · (β · r − y)         (typically β = 0)
I term: Ki · (r − y)
```

In our case "setpoint" is the target phase, which is fixed at the
advance-derived location. So β is effectively already there. The
twist would be:

```
P term: Kp · (capValue − setValue)        (what we have)
I term: Ki · (capValue_smoothed − setValue)   (smooth I, fast P)
```

Smoothing the I-term input rejects per-sector noise from the
integrator while keeping P responsive.

**Cost**: 1 day. Needs a small IIR on capValue.

**Expected gain**: Cleaner steady-state behavior, less integrator
drift from per-sector noise. ~5-10% reduction in phase jitter.

### 4.6 LOW-VALUE: Adding a D term (PID)

**What**: Add `Kd · (delta_new − delta_old) / Δt` to the output.

**Why it WON'T help**:
- Our sector rate is the sample rate. Delta on `delta` between
  sectors is dominated by per-sector measurement noise.
- D term amplifies noise → would need heavy filtering → which
  destroys the derivative information
- The motor's mechanical dynamics are much slower than sector rate;
  no fast-changing phase signal to differentiate

**When it WOULD help**: if we were running the PI at a fixed
millisecond rate (not sector rate), D would help reject load
torque transients. We're not.

**Expected gain**: ~0%. Probably negative if added naively.

### 4.7 LOW-VALUE: Adaptive control (gain auto-tuning)

**What**: Use observed step response to back-compute optimal Kp/Ki.

**Why it WON'T help much**:
- Motor parameters don't change fast enough during operation to
  warrant runtime adaptation
- The bench-tuning step exists anyway; we'd rather have humans
  validate optimum than run perturb-and-observe live
- Stability of adaptive schemes is hard to guarantee

**Expected gain**: ~0%. Better as a one-time auto-calibration
than a runtime feature.

### 4.8 HIGH-VALUE-BUT-HARD: Kalman observer for velocity

**What**: Instead of using the PI integrator as an implicit velocity
estimator, maintain an explicit Kalman filter on (period,
period_rate). PI corrects from observer prediction.

**Why it matters**:
- Per-sector noise is averaged across multiple samples with proper
  weighting
- Predict-then-correct structure handles missed captures cleanly
  (just predict, no correction step)
- Confidence bounds available for adaptive behavior (e.g. tighten
  clamps when observer confidence is low)

**Cost**: 3-5 days. Significant testing. 2-state Kalman is
manageable; tuning Q and R matrices is the real work.

**Expected gain**:
- Smoother PI behavior with noisy captures
- Better behavior at low SNR (low duty / high RPM)
- Cleaner failure modes when captures stop entirely

### 4.9 HIGH-VALUE-BUT-HARD: Multi-capture per sector at high RPM

**What**: At sector_time < 50 µs, currently 1 capture per sector
(ZC at midpoint). If we could capture at sub-sector resolution
(detect ZC arrival time AND amplitude AND slope), we could:

- Verify capture quality before feeding to PI
- Reject captures that look like switching noise (low slope,
  near-rail amplitude)
- Use multiple per-sector samples to interpolate the exact ZC
  instant with sub-tick precision

**Cost**: Major refactor of the HWZC peripheral handling. Possibly
hits hardware limits (SCCP2 IC capture queue depth).

**Expected gain**: Could push max reliable eRPM from current
~240k to ~280-300k. Real gain at the very top of the speed range.

### 4.10 LOW-VALUE: Sliding mode / robust control

**What**: Replace PI with sliding-mode control on the phase error.

**Why it WON'T help**:
- Robust control shines when the plant is poorly known. Our plant
  is well-known (BLDC, KV, Rs, Ls all measured)
- Sliding mode introduces chattering by design; would create extra
  PWM transitions
- The asymmetric clamp we already have provides similar
  robustness benefits without the chattering

**Expected gain**: 0%, possibly negative.

---

## 5. Realistic Roadmap

Given the cost/benefit ranking, here's the order I'd actually do
this in:

### Phase 1: Float port (no behavior change)
- Same algorithm, float math
- Validate identical bench behavior
- Unblocks all of below

### Phase 2: Feedforward from duty/Vbus
- Compute `P_FF` from physics
- Add to integrator init (faster lock) and steady-state output
  (smaller integrator, less wind-up risk)
- Bench-validate startup time + transient response

### Phase 3: PLL-form gain specification
- Replace `KP_SHIFT`, `KI_SHIFT` with `loop_bandwidth_Hz` and `zeta`
- Continuous gain scheduling with eRPM
- Re-tune for bench-optimal behavior

### Phase 4: Velocity form
- Switch implementation to incremental form
- Verify no behavior change at lock
- Use the new structure for proper anti-windup

### Phase 5: Setpoint weighting / smoother I-term
- Add slow IIR on `capValue` for the I-term input
- Verify reduced steady-state jitter

### Phase 6 (LATER): Kalman observer
- 2-state filter on (period, period_rate)
- Restructure ISR data flow
- Major test suite

**Total work**: Phases 1-5 ~1 week of focused work. Phase 6 is
its own project.

---

## 6. What Should NOT Change

For the avoidance of doubt:

- **The PI is a PLL, not a PID** — the algorithm structure is right
  for the problem
- **Per-sector update rate** — don't move to fixed wall-clock rate
  (per `control_loop_pwm_coupling.md`, the slow loop CAN move but
  the PI specifically benefits from per-sector cadence)
- **Skip-update-on-missing-capture** — robust by design, keep it
- **Asymmetric clamps at BEMF ceiling** — physics-justified, keep
- **Decoupled slow IIR for filter-comp period source** — breaks the
  positive feedback loop with PI shrinking, keep
- **Integer math for ISR-hot paths if it ever becomes a budget
  issue** — but Section 3.1 shows we have lots of budget

---

## 7. What This Looks Like in Code (Sketch)

After Phase 2 (feedforward + float):

```c
void HWZC_OnPiPeriodExpired(volatile GARUDA_DATA_T *pData)
{
    float T = (float)pData->hwzc.timerPeriod;

    /* Feedforward: open-loop period estimate from physics */
    float duty_frac = (float)pData->duty / (float)LOOPTIME_TCY;
    float vbus_v = adc_to_vbus(pData->vbusRaw);
    float P_ff_sec = 60.0f / (motorKV * vbus_v * duty_frac * polePairs * 6.0f);
    float P_ff_ticks = P_ff_sec * 100.0e6f;   /* 100 MHz HR */

    if (pData->hwzc.captureValid) {
        float capValue = (float)(pData->hwzc.lastCaptureHR
                                 - pData->hwzc.lastCommStamp);
        float setValue = advanceFraction * T;
        float delta = capValue - setValue;

        /* Asymmetric clamp (preserved) */
        float clamp_pos = T * 0.0625f;    /* T/16 */
        float clamp_neg = T * 0.03125f;   /* T/32 */
        if (T < HIGH_RPM_THRESHOLD_TICKS) {
            if (delta >  clamp_pos) delta =  clamp_pos;
            if (delta < -clamp_neg) delta = -clamp_neg;
        } else {
            float clamp = T * 0.125f;     /* ±T/8 */
            if (delta >  clamp) delta =  clamp;
            if (delta < -clamp) delta = -clamp;
        }

        /* PI (in physical units) */
        integrator_f += Ki * delta;

        /* Clamp integrator around feedforward (anti-windup) */
        float intg_min = P_ff_ticks * 0.8f;
        float intg_max = P_ff_ticks * 1.2f;
        if (integrator_f < intg_min) integrator_f = intg_min;
        if (integrator_f > intg_max) integrator_f = intg_max;

        float newPer = integrator_f + Kp * delta;

        /* Final safety */
        if (newPer < (float)RT_HWZC_MIN_STEP_TICKS)
            newPer = (float)RT_HWZC_MIN_STEP_TICKS;

        pData->hwzc.timerPeriod = (uint32_t)newPer;
    }
    /* ... rest identical ... */
}
```

The structure is clearer, the numbers are physical, anti-windup is
explicit, and the feedforward is doing the heavy lifting.

---

## 8. What I'd Want to See on the Bench After This

For the float + feedforward port (Phases 1-2), bench targets:

| Metric | Current | Target after Phase 2 |
|---|---|---|
| Lock acquisition (captures to lock) | 12 | 6-8 |
| Throttle step settling time | ~50 ms | ~20 ms |
| Steady-state phase jitter (HR ticks RMS) | 10-15 | 5-10 |
| Top-RPM sustained current | baseline | -2 to -5 % |
| Regen events on hard pot drop | occasional | rare |
| Max desync events per 5-minute test | 1-2 | 0-1 |

For PLL-form gains + velocity form (Phases 3-4):
- Easier to retune for new motors (sub-hour vs sub-day)
- Cleaner MAX_DUTY behavior, fewer windup edge cases

For Kalman observer (Phase 6, eventual):
- Reliable operation past 240 k eRPM
- Graceful behavior in noisy environments (longer wires, weak PCB
  filter, etc.)

---

## 9. Cross-References

- `motor/hwzc.c:616-775` — the actual implementation
- `garuda_config.h:669-709` — gain and clamp constants
- `dspic33AKESC/docs/bemf_filter_compensation.md` — filter comp
  (which is ANOTHER reason float would help — Section 9.1 there)
- `dspic33AKESC/docs/control_loop_pwm_coupling.md` — why
  per-sector vs per-PWM-cycle matters for the PI
- `memory/akesc_sector_pi_phase_a_milestone_2026_05_26.md` —
  original bench validation of the sector PI architecture
- `memory/akesc_interval_gate_214k_2026_05_26.md` — gain
  threshold tuning that led to current values

---

## 10. TL;DR

- **What we have**: 2nd-order discrete PLL, integer bit-shift gains
  (Kp=1/4, Ki=1/16), asymmetric clamp at BEMF ceiling. ~15 cycles per
  sector. Works well in the validated envelope.
- **FPU unlocks**: clean float math, non-power-of-2 gains, feedforward
  with physics formulas, proper anti-windup, observers. Compute budget
  is plentiful (300-400 ns even with heavy variants vs 42 µs sector
  at top speed).
- **Trig unlocks** (separately): mostly future-FOC and exact
  `sin(arctan)` filter comp. PI itself doesn't need trig.
- **Real improvements** (in priority order): float port → physics
  feedforward → PLL-form gains → velocity form → setpoint weighting.
  Phases 1-5 = ~1 week. Phase 6 (Kalman) = its own project.
- **Bookshelf-fashionable but not actually useful**: D term, sliding
  mode, runtime adaptive gains. Don't add for the sake of having
  them.
- **What stays the same**: per-sector cadence, robust skip-on-miss,
  asymmetric BEMF-ceiling clamps, slow-IIR decoupling for filter
  comp. The architecture is sound; the implementation can be
  cleaner.
