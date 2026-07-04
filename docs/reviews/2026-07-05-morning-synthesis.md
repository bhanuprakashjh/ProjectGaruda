# Morning Synthesis — Overnight Review Verdict (2026-07-05)

Three independent overnight reviewers (A: git archaeology vs the proven 285k-era code;
B: full mechanism/interaction audit of the current capture path; C: flagship vs
Simplified-tree engine comparison) examined the user's hypothesis: **"we broke the
complexity of the code."** Full reports sit beside this file.

## Verdict — all three converge

**The user's hypothesis is confirmed, and the disease is now named.**

The stochastic 1.5–2.0x speed snap during throttle-up (40–65k band) and the 2x idle
alias are a **stable false fixed point of the sector-PI itself**:

- The PI measures **phase only** (where the capture lands inside the expected sector).
  It never measures **frequency** (the actual ZC-to-ZC interval). (C §2.3)
- Commutation runs from an **autonomous timer** that fires whether or not a crossing
  was heard; captures only nudge it. A wrong-frequency grid therefore geometrically
  self-sustains. (C §1, hwzc.c:461-474)
- Once aliased, the captures the loop *does* accept produce near-zero phase error
  (the PI is satisfied), the late true crossings are **rejected** by the cross-sector
  gate (`cap > T`, hwzc.c:904-909), and the defensive/watchdog layers see "captures
  arriving, no silence" — every containment bounds the integrator; none re-anchors
  it to a measurement. The false lock defends itself. (C §3.1-3.2)
- Acceleration is the trigger because during a ramp the rotor outruns the PI's
  clamped catch-up authority (±T/8 per capture) while the 50% interval gate and
  PWM-ON gate reject the *legitimately early* crossings that would let it catch up —
  starvation carries the integrator into the alias basin. Onset is stochastic
  because accept-probability is phase luck. (B §4)
- The proven 285k-era build (bd85e9b) had the same PI holes but survived on
  capture availability: rising-only HW comparator + separate falling-SW path =
  robust 6-spoke feed. That hybrid was purged (9ec01c8) and replaced by
  falling-HW-everywhere + the 70k mute — capture availability collapsed exactly
  where the snaps live. (A findings 1-3)

Secondary confirmed findings:
- The advance anchor at 260000 eRPM starves timing advance (~4-6°) in the 40-65k
  operating band, shrinking the PI's early-capture headroom. (B interaction C2)
- ~28 live mechanisms touch every capture; several are dead weight or redundant
  (B §5); the rej% telemetry is dominated by the PWM gate and is useless as a
  health signal (B §2).
- The Simplified tree's ported PLL **had the identical disease** (period wound 28%
  short, inst 192k vs true 150k) and **cured it on 2026-07-01** by interval-anchoring:
  `integrator = measured_interval_IIR` each tick, PI demoted to a bounded ±3% phase
  trim, commutation re-anchored to every accepted crossing. Two implementations,
  same PI law → same disease; same interval law → same cure. (C §1.5, §3.4)

## The plan (ranked, one step at a time)

1. **TODAY'S A/B (built and ready, single flip + safety raise):**
   `FEATURE_HWZC_SECTOR_PI 1 → 0` — reactive mode IS an interval tracker
   (IIR (3T+interval)/4 + reschedule-on-ZC), i.e. the Simplified architecture
   already living inside the flagship. Paired with `HWZC_MISS_LIMIT 3 → 8` so the
   tight SW-fallback latch doesn't contaminate the experiment.
   **Prediction if the diagnosis is right: no 1.5-2x snaps at any throttle rate;
   no 2x idle alias.** Watch instead for: lower top end, SW-fallback behavior.
2. **If confirmed → the keeper fix:** port the Simplified 2026-07-01 interval
   anchor into the sector-PI (~30-40 lines in hwzc.c): measure interval, IIR it,
   pin `integratorF = measuredPeriod` per tick (PI becomes bounded phase trim),
   re-anchor SCCP1 to each accepted crossing. Keeps all gates/telemetry/tuning.
   This is a transplant of a bench-proven fix, not new dynamics.
3. **Independent confirmation (any time):** bench the Simplified tree itself on
   the 2810 — one line, `MOTOR_PROFILE PROFILE_2810` (motors.h:57); 24V needs no
   other change (Vbus measured live).
4. **After the dynamics fix**, apply B's bd85e9b-grounded rerank: the PWM gate,
   interval-50 gate, delta clamp and filter comp were ALL present at 285k —
   load-bearing, do NOT strip. The post-June-17 accretions absent from the
   proven build are the strip candidates: chop gate, WS1 load-demag, WS3
   variable trigger, mistime/clip/stall watchdogs (keep caprate), CAP_DUTY_HOLD,
   spin-catch, falling-HW+mute. B's top two tuning fixes if needed: advance
   anchor 260k → ~70k (restores real advance in the operating band) and
   restoring the proven FALLING_SW hybrid in place of falling-HW+mute.

**Explicitly rejected:** more clamp/floor/gate tuning on the PI (three campaigns
prove each clamp narrows but never removes the alias basin — no clamp adds a
frequency measurement).
