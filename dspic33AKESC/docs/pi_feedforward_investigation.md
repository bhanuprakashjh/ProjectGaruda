# Sector-PI Feedforward Investigation — Conclusion

**Date**: 2026-05-28
**Outcome**: Feedforward (Phase 2 of `pi_controller_research.md`) **does
not work in sensorless 6-step** as originally proposed. Pure float PI
(Phase 1) is the right stopping point.

This doc captures what we learned across one bench attempt and a
follow-up simulator investigation.

---

## What we tried

`FEATURE_HWZC_PI_FLOAT=1, HWZC_PI_FF_ENABLE=1` —
per-iteration feedforward:

```c
P_ff = (2π√3/60) · λ_pm · 1e9 / (V_bus · duty)        // physics no-load period
integratorF += Ki · delta                              // PI integral on residual
integratorF = clamp(integratorF, ±0.2 × P_ff)         // anti-windup
new_per = P_ff + integratorF + Kp · delta              // FF + PI output
```

The integrator carries the **residual** between the FF prediction and
the actual commutation period. PI corrects from there.

## What happened on the bench (2026-05-28)

Motor failed to start. Controller reported 240k eRPM with throttle at
zero and duty at 6%. Ia_pk pinned at 22 A (current limiter). Rotor
physically stationary — the "20k–240k eRPM" telemetry was a phantom
read of the autonomous SCCP1 timer running at the FF-predicted period.

Three iterations:
1. Wrong ke convention in formula → P_ff way too small → controller
   pinned at 260k cap immediately. Fixed.
2. Corrected formula → motor still stalls but now with phantom 20k
   eRPM reading.
3. Reverted FF feature flag to 0.

## What the simulator missed (originally)

The first sim version had a plant that **snapped rotor electrical phase
to the commanded sector** on each commutation. This implicitly assumed
the bridge can always drive the rotor to match — like a stepper motor
in microstepping mode. Real BLDCs don't work that way; the bridge can
absolutely command the rotor into wrong-angle territory if its
estimate is faster than the rotor's actual rate.

In that idealized sim, the per-iteration FF showed a clean ~16×
improvement on step response and looked great. Bench reality showed
the opposite.

## The fix to the simulator

Replaced the plant with realistic torque-vs-angle physics:

```python
theta_rotor   # absolute electrical angle, free-running at omega_elec
theta_command # bridge's target angle (advances π/3 per commutation)
lead = theta_commanded − theta_rotor   # signed, wrapped to [-π, π]
torque_efficiency = cos(lead)
```

- Full torque when aligned (`lead = 0`)
- Zero torque at `±π/2` (90° wrong-angle stall)
- Negative torque (braking) beyond
- Soft current limit at 22 A mirrors firmware

ZC events now fire from the rotor's natural sector midpoints, using
cumulative `total_rotation` for wrap-safe counting — independent of
what the bridge is doing.

## What the realistic sim shows

Gradual duty ramp 6%→96% over 1.8 s (mimics typical bench throttle
sweep):

| Controller | eRPM @ 96% duty | Status |
|---|---|---|
| Pure Float PI | 217k (smooth) | ✓ tracks |
| Float + per-iter FF | 130k (rotor crashes during ramp) | ✗ same failure as bench |

The sim now matches the bench-observed failure. Either both work or
both don't.

## Why per-iteration FF is fundamentally broken here

In sensorless 6-step:

1. **The PI's integrator IS the rotor's velocity estimate.** It exists
   precisely because we cannot know the rotor speed any other way —
   ZC captures are the only signal.
2. **Feedforward predicts no-load steady-state.** It does not know the
   rotor's actual instantaneous speed. It only knows what the speed
   *would* be if all load and dynamics were ignored.
3. **Per-iteration FF biases the output toward the no-load
   prediction.** Each iteration, `new_per = P_ff + residual_integ`.
   The residual is clamped tight (±20% by design) so the output never
   strays too far from P_ff.
4. **At handoff or during acceleration, the rotor is well below
   no-load**. P_ff predicts the destination speed; the rotor is at
   the starting speed. Per-iteration FF commands the bridge to
   commutate at the destination rate immediately.
5. **The bridge over-commits.** It races ahead of the rotor → lead
   angle grows past 90° → torque collapses → motor stalls.
6. **Self-reinforcing failure.** Once stalled, no useful ZCs arrive
   → PI doesn't update → bridge keeps running at FF-predicted period
   → motor stays stalled.

The PI's natural lock dynamic is exactly: **the integrator should
walk toward the rotor's actual period at a rate the rotor can
follow**. Per-iteration FF circumvents this by jumping ahead. That's
the mistake.

## What FF designs *could* work (and don't, on reflection)

| Design | Idea | Why it fails |
|---|---|---|
| **Seed-only at handoff** | Initialize `integratorF = P_ff` at HWZC enable | Rotor at handoff is at 6 k eRPM (SW ramp end), not at no-load. Same stall. |
| **Duty-kick on changes** | Add `(P_ff_new − P_ff_old)` to integrator on duty steps | Same stall on big steps: rotor can't accelerate as fast as the kick demands. |
| **Soft minimum period** | Clamp `T >= P_ff` (don't run faster than no-load) | At BEMF ceiling, real motors slightly exceed the no-load formula (~5%). Clamp blocks top speed. |
| **Slow FF blend** | Gradually drift integrator toward `P_ff` (small α) | Same problem at a slower rate. Bench wouldn't see acute stall but PI would track worse than pure float. |
| **FF in `setValue` not output** | Use FF prediction as target for delta | Math breaks: `setValue = adv_frac × P_ff` doesn't match the geometric relationship at lock. |

Every design that pushes the controller toward the no-load prediction
**before the rotor is there** has the same failure mode.

## Conclusion

**Phase 1 (float port) is the right stopping point.** Phase 2
(feedforward) as originally conceived has a structural problem with
sensorless 6-step that the simulator missed at first.

The float port itself is the actual win:
- Non-power-of-2 gains (Kp = 0.31 etc. possible)
- Easier per-motor tuning
- No bit-shift quantization
- Path open to PLL-form (ω_n, ζ) gain specification
- Path open to velocity form + back-calc anti-windup
- Path open to setpoint weighting

These are real improvements that don't require FF. They're listed in
`pi_controller_research.md` Sections 4.3–4.5.

## Current firmware state

```c
#define FEATURE_HWZC_PI_FLOAT          1   // Phase 1 — bench-validated
#define HWZC_PI_FF_ENABLE              0   // Phase 2 — does not work, parked
```

The FF code path stays in `motor/hwzc.c` (under `#if HWZC_PI_FF_ENABLE`)
in its safest form — applied only as a one-shot seed at handoff if it
predicts a **slower** period (which essentially never happens given
the SW ramp's slow handoff). Effectively a no-op when enabled. Future
work can either delete it or repurpose for a different design.

## What to do next instead (priority order)

Per `pi_controller_research.md`:

1. **PLL-form gain specification** — Express Kp, Ki in terms of ω_n
   and ζ. Easier to retune for different motors.
2. **Velocity form + proper anti-windup** — Fixes the MAX_DUTY
   windup edge cases that occasionally bite us on the bench.
3. **Setpoint weighting / smoothed I-term** — Slow IIR on capValue
   for the integral path, fast path stays direct. ~5–10% jitter
   reduction.
4. **Better OL_RAMP → CL handoff** — Make the handoff smoother by
   keeping the SW ramp running for a few more sectors after HWZC
   takes over. Reduces the transient where wrong-angle stall risk
   is highest.

None of these change the PI's fundamental architecture. They tune it.

## References

- `dspic33AKESC/docs/pi_controller_research.md` — the original roadmap
- `tools/pi_pll_simulator.py` — sim now models the realistic plant
- Bench session 2026-05-28: chat history — three FF iterations on bench
- `motor/hwzc.c` — the FF code path, currently disabled

---

## Phase 2 Milestone (2026-05-28 evening)

Defensive PI bench-validated. Session step6_20260528_211557.csv:

- **7 pot-zero releases**, all stayed in CL, no restart cycles
- **Peak 236,350 eRPM** (new bench record, +2k over previous best)
- **86.8 seconds clean CL operation**
- The 2026-05-27 rapid-pot-zero stall bug is resolved
- Only fault was BOARD_PCI from regen-OV during sustained 98% — unrelated
  to defensive PI, fixed by future duty-down slew limiter (Appendix B)

Current production state:
  FEATURE_HWZC_PI_FLOAT     = 1   (Phase 1 ✓)
  FEATURE_HWZC_PI_DEFENSIVE = 1   (Phase 2 ✓)
  HWZC_PI_FF_ENABLE         = 0   (rejected, see above)

Phase 2 stopping point. Remaining real improvements: duty-down slew
limiter (regen-OV fix), demag detection, variable PWM frequency.
