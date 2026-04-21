# Commutation Kickback Mitigation — Preliminary Findings

**Date**: 2026-04-21
**Boards tested**: MCLV-48V-300W + dsPIC33AK128MC106 (AKESC)
**Motors**: A2212 1400KV (7PP, 65 mΩ, 30 µH), 2810 1350KV (7PP, 50 mΩ, 25 µH)
**Status**: A2212/12V — 55% kickback reduction validated. 2810/24V bare — no benefit (different regime).

## Summary

Reducing complementary-PWM deadtime from 500 ns to 300 ns cuts peak bus current by ~55 % at the bare-motor top-speed regime where commutation kickback dominates losses. At the accelerating-into-free-running regime (2810 at 24 V bare, far from BEMF-match ceiling), the same change has no effect — the peak current in that regime is real drive current, not kickback.

These two outcomes are consistent with a simple mental model that separates *drive current* from *kickback current* and predicts where each dominates.

## Key Numbers

### A2212 / 12 V / bare — near BEMF-match ceiling (BEMF ≈ Vbus)

| Metric                    | 500 ns   | 300 ns   | 200 ns   |
|---------------------------|----------|----------|----------|
| Top eRPM at 95% duty      | 103,734  | 107,979  | 108,849  |
| **Ibus pk at top**        | **12 A** | **5.5 A**| **5 A**  |
| Ia pk at top              | 5–6 A    | 5–6 A    | 6–7 A    |
| ALIGN Ia peak             | ~5 A     | ~6 A     | **-7.3 A** (shoot-through start) |

- **300 ns is the sweet spot.** 200 ns gave no further Ibus reduction but raised ALIGN current — early shoot-through margin loss.
- Top eRPM increase (+4 %) is real: energy previously wasted in body-diode reverse-recovery is now going into torque.
- Steady-state bus current at 118k eRPM bare: **~1.2 A** — near the physics floor (iron + friction losses).

### 2810 / 24 V / bare — accelerating toward 226k eRPM no-load ceiling

| Metric                    | 500 ns   | 300 ns   |
|---------------------------|----------|----------|
| Trip eRPM (BOARD_PCI)     | 78,412   | 78,412   |
| Trip duty                 | 34 %     | 34 %     |
| **Ibus pk at trip**       | **21.9 A** | **21.9 A** |
| Ia pk at trip             | 16 A     | 15 A     |

- **Deadtime has no effect here.** Same trip eRPM, same trip current.
- At 78k eRPM with 24 V supply, BEMF ≈ 8 V. Net driving voltage = 16 V across a 50 mΩ winding means the motor legitimately demands ~20 A of drive current to accelerate. This is not kickback.
- The 22 A MCLV U25B comparator trips because the board current-sense path hits its threshold — not because of commutation imperfection.

## Why the Two Regimes Behave Differently

**Two components to phase current** at any operating point:

1. **Drive current** — power delivered by supply, turned into torque. Proportional to (V_applied − BEMF) / R. Time scale: PWM period.
2. **Kickback / circulating current** — inductor-stored energy (½ L I²) freewheeling through body diodes at each commutation event. Time scale: nanoseconds per event, but enough to trigger fast peak comparators.

At **near-BEMF-match ceiling** (A2212/12V at 118k):
- V_applied ≈ BEMF → drive current ≈ 0
- Phase current is dominated by the kickback spike at each commutation
- Reducing body-diode dwell (300 ns deadtime) shrinks this spike directly
- Ibus pk drops dramatically; Ia pk is already small

At **far from ceiling, accelerating** (2810/24V at 78k):
- V_applied − BEMF = 16 V of real driving voltage
- Phase current is dominated by drive current building up in the winding each PWM cycle
- Commutation kickback is a small addition on top of an already-large drive current
- Deadtime reduction doesn't change the drive current, so peak doesn't change

## Mental Model for Predicting Which Lever Works

Before tuning, ask: *at the operating point of interest, is the motor near BEMF-match or far from it?*

| Regime | BEMF/Vbus | Drive current | Kickback fraction | Best lever |
|---|---|---|---|---|
| Top-end BEMF-limited | ~1.0 | ~0 | dominant | Deadtime, snubbers, synchronous rectification |
| Mid-range | 0.4–0.8 | moderate | significant | Both deadtime *and* active current control help |
| Acceleration / startup | <0.3 | dominant | small relative | Active current limiting, PWM scheme, layout |

A2212/12V lives near the top (BEMF/Vbus ≈ 0.98 at top speed) on a bench supply — kickback-dominated.
2810/24V on a stiff 24 V supply sits in the acceleration regime — drive-current-dominated.

## Session Changes — Detailed Log

### Firmware (AKESC)

- **`FEATURE_SINE_STARTUP` → 0**: disabled sine morph for simpler forced-commutation startup. Makes `duty` telemetry a direct read of `RAMP_DUTY_PERCENT` etc., and isolates startup debugging from sine-shaping variables.
- **A2212 profile startup duties bumped**: `ALIGN_DUTY_PERCENT` 8 → 12, `RAMP_DUTY_PERCENT` 15 → 22, `CL_IDLE_DUTY_PERCENT` 12 → 18. Original values were bare-motor tuned; these move enough torque for an 8x4.5 prop to break static friction.
- **A2212 profile ramp slowed**: `INITIAL_ERPM` 200 → 100, `RAMP_TARGET_ERPM` 3000 → 2000, `RAMP_ACCEL_ERPM_PER_S` 1000 → 400. Gives prop inertia time to actually spin through the OL ramp instead of lagging the commanded field.
- **A2212 profile `DEADTIME_NS` 500 → 300**: -55 % Ibus pk at top throttle, +4 % eRPM. Validated and committed.
- **2810 profile `DEADTIME_NS` 500 → 300**: applied for consistency. No measurable benefit in this regime (see above).
- **`HWZC_IIR_FREEZE_ZC_COUNT = 3`**: freezes the `stepPeriodHR` IIR for the first 3 ZCs after `HWZC_Enable`, then lets it update. Without the freeze, a single phantom ZC during the morph handoff pulled the IIR to floor in ~50 % of startup attempts → stall → phantom 119k eRPM read. Described in `motor/hwzc.c`.
- **First-ZC bypass (`firstZcAfterEnable` flag)**: the first ZC after enable bypasses the plausibility gate AND skips the IIR update. The seeded `lastZcStamp` is a guess; the first real interval is rotor-position-dependent and not representative. Bypass just anchors `lastZcStamp` so the second ZC gives a clean full-step interval.
- **Plausibility gate hard floor**: minimum interval is now `RT_HWZC_MIN_STEP_TICKS` (the period at `MAX_CLOSED_LOOP_ERPM`) instead of 2/3 of it. Closes a runaway-collapse path where phantom intervals shorter than physically possible could pass the 70 % gate as `stepPeriodHR` dropped.

### Telemetry

- **Phase-current instrumentation**: OA1OUT (Ia) and OA2OUT (Ib) sampled at PG1TRIGA (24 kHz mid-ON) via `AD1CH3` and `AD2CH2`. Rolling 20 ms window peak tracking, plus a frozen "at-fault" snapshot captured on BOARD_PCI transition.
- **Ibus peak tracking** added alongside phase peaks.
- **GSP snapshot grew from 170 → 208 bytes** to carry the new phase-current / at-fault fields. `tools/step6_logger.py` updated to render them (`Ia_now(pk)`, `Ibus(pk)`, `>>> AT-FAULT` block).
- **`dutyPct` in snapshot** = actual applied PWM duty (`duty × 100 / LOOPTIME_TCY`), which makes misconfigured builds easy to spot in the log — telemetry directly reveals whether config changes reached the firmware.

### PWM / Hardware

- **PWM frequency 24 → 40 kHz**: lowers the phantom-crossing window and lets us push higher eRPM without the off-time gate latency bottleneck.
- **Deadtime 500 → 300 ns** (both A2212 and 2810 profiles). See above.

## Not Done Yet (Next Phase)

- **V4 sector-PI synchronizer on PATA (CK) board**, at 200k eRPM target. The 3-step interpolation approach predicts commutation timing from a PI-smoothed `stepPeriodHR` instead of firing per-ZC. Running the same phase-current instrumentation there and comparing Ia/Ibus peaks against this A2212 dataset is the direct physics validation:
  - If at 200k bare on CK+SMO the Ibus pk is still small (<10 A), the BEMF-match/kickback model holds generically.
  - If CK+SMO shows a similar 22 A regime at some intermediate speed, there's another mechanism we haven't accounted for.
- **Prop test on 2810/24V** — the prop should clamp eRPM below the runaway regime and avoid the 22 A trip entirely without any firmware change. Expected equilibrium: 50-60k eRPM at ~10-15 A steady bus.
- **Software current limit PID** as a backup for situations where a prop can't clamp fast enough (transients).
- **VESC-style FIR+integrator BEMF detector** for startup robustness. dsPIC33AK already does 4-sample oversampling at 1 MHz on the HWZC channel (equivalent to a short moving-average FIR) — would need to extend to a wider FIR with PWM-synchronized sampling to materially change startup behavior.
- **Commutation overlap** — firmware-only lever to smooth the current transition at each commutation. Stacks additively with deadtime reduction per literature. Not yet tested.

## References

- Session logs: `logs/step6_20260421_085731.csv` (A2212 500 ns baseline), `logs/step6_20260421_085900.csv` (A2212 300 ns), `logs/step6_20260421_090300.csv` (A2212 200 ns), `logs/step6_20260421_091242.csv` (2810 300 ns trip).
- Related memory: `memory/v4_status.md`, `memory/sp_mode_implementation_notes.md`, `memory/high_speed_zc_approaches.md`.

---

*Generated from the session debug log; numbers pulled directly from telemetry CSVs. Reproduce by flashing the AKESC build at commit `17bf682` for A2212, then changing `MOTOR_PROFILE=2` for 2810.*
