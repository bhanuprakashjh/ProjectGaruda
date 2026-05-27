# Why the Control Loop Is PWM-Locked (And Why It Doesn't Have To Be)

**Date**: 2026-05-27
**Status**: Architectural analysis. No code changes — reference for
when we decide to decouple the slow loop from PWM frequency.

This is the companion doc to `pwm_frequency_effects.md`. That one
catalogs *what changes* when you move PWM frequency. This one explains
*why so much changes* — and why most of it didn't have to.

---

## The Two Clock Domains

```
HIGH-RATE PATH  (already PWM-INDEPENDENT)
─────────────────────────────────────────
  comparator transitions  →  HWZC ISR (priority 7)
                                 ↓
                          SCCP2 HR timestamp (10 ns absolute)
                                 ↓
                          PI math in OnPiPeriodExpired
                                 ↓
                          SCCP1 autonomous commutation timer
                                 ↓
                          phases commutate at lastStamp + timerPeriod

  This chain has zero PWM-frequency dependence. Times are absolute
  10 ns units. The same code runs identically at 30 / 45 / 60 kHz.


SLOW-RATE PATH  (UNNECESSARILY PWM-LOCKED)
─────────────────────────────────────────
  PG1TRIGA fires once per PWM cycle  →  ADC sample (Ia, Ib, Vbus, pot)
                                              ↓
                                       _AD1CH0Interrupt fires
                                              ↓
                                       adcIsrTick++   ← the choice
                                              ↓
                          everything else: state machine,
                          ZC timeout, no-capture watchdog,
                          throttle slew, duty mapping,
                          BEMF_ZC_OnCommutation processing
```

## What Genuinely Needs PWM-Synchronous Timing

**Exactly one thing**: reading the standard ADC channels (phase
current Ia/Ib, Vbus, throttle pot) needs to land in the middle of the
PWM valley so the samples don't catch the switching transient.
`PG1TRIGA` gives that for free.

That's it. Read samples, convert to volts / amps, store. The
*sampling* must be PWM-synchronous. Nothing else has to be.

## What Got Bolted On For Convenience, Not Necessity

Everything else in the ADC ISR slow path lives there because the ISR
was already running and `adcIsrTick` was already there. Pure
convenience:

| Slow-loop component | Real timing requirement |
|---|---|
| ZC timeout watchdog | Fixed wall-clock rate (e.g. 1 ms) |
| No-capture watchdog | Fixed wall-clock rate |
| State machine transitions | Event-driven (HWZC or Timer1) |
| Throttle slew / duty mapping | Timer1 fixed rate, or per-HWZC event |
| Soft current limiter — *decision* | Event-driven by ADC sample |
| Soft current limiter — *PWM write* | Per-ADC-ISR (this IS PWM-tied, fine) |
| BEMF_ZC_OnCommutation post-processing | Per-commutation event, not per-PWM |

## Why It Matters Across PWM Frequencies

Because the slow loop runs at PWM rate, **the same line of code
measures the same physical interval as a different number of ticks**
depending on PWM frequency:

```
14 k eRPM sector  (714 µs of wall time)
  at 30 kHz  →  21 ticks
  at 45 kHz  →  32 ticks
  at 60 kHz  →  43 ticks
```

Every threshold, every timeout, every slew rate has to be re-derived
from PWM frequency for the behavior to stay constant in wall time.
That's why `garuda_calc_params.h` is full of `× PWMFREQUENCY_HZ /
1000`-style conversions.

If the slow loop ran on a fixed wall-clock timer instead, none of
those derivations would exist. PWM frequency would be a pure
**power-stage** choice: pick it for ripple/efficiency, the control
loop wouldn't care.

The 30 / 40 / 45 kHz mid-pot behavior described in
`pwm_frequency_effects.md` reflects both:

1. **Physics** — ripple amplitude, filter attenuation. *Real*. Doesn't
   go away with a refactor.
2. **Architecture** — slow loop running at a rate that scales with
   PWM. *Incidental*. Goes away with a refactor.

Today these are tangled. Decoupling would let you see the physics
cleanly without the architectural noise.

## What's Already Available

The firmware already has fixed-rate timers:

- **Timer1 ISR (`_T1Interrupt`)** — runs at 10 kHz (100 µs ticks)
  regardless of PWM. Currently used for: heartbeat LED, board service
  (button debounce), 1 ms `systemTick` subdivision, ESC state machine
  for ALIGN / OL_RAMP / RECOVERY phases.
- **`systemTick`** — 1 ms increments, derived from Timer1.
- **HWZC SCCP2 HR** — 10 ns absolute, already used for
  `hwzcStepPeriodHR`.

So the infrastructure is in place. The slow loop just isn't using it
for its core timing math.

## What a Decoupled Slow Loop Would Look Like

Required changes (sketch only — not a plan to do today):

1. **Replace `adcIsrTick` semantics** in elapsed-time math with
   either:
   - `systemTick` (1 ms granularity — fine for watchdogs)
   - A 10 kHz Timer1 counter (100 µs granularity — fine for slew)
2. **Change `timing.stepPeriod` units** — either microseconds or use
   the existing `hwzcStepPeriodHR` field (10 ns absolute, already
   maintained).
3. **Move watchdogs** from ADC ISR → Timer1 ISR:
   - `BEMF_ZC_CheckTimeout`
   - `HWZC_NO_CAPTURE_TICKS` no-capture watchdog
   - Stall plausibility check
4. **Keep ADC ISR scoped** to what it actually needs:
   - Read ADC channels
   - Convert raw → physical
   - Apply slow loop's *already-decided* duty value to PWM registers
   - That's all
5. **Derive eRPM** directly from `hwzcStepPeriodHR` — no PWM-rate
   divisor needed anywhere on the host or firmware side.

After this:

- `PWMFREQUENCY_HZ` only appears in: PWM peripheral init,
  `LOOPTIME_TCY`, `MAX_DUTY`, and the bootstrap / deadtime calcs.
- Changing PWM frequency would re-tune ripple/efficiency only.
  Watchdog behavior, slew rates, and timeout durations would stay
  fixed in wall time.

## Why The Current Design Exists Anyway (Honest History)

- The firmware grew from a Microchip reference example where the
  *whole* control loop lived inside the ADC ISR (V/f originally, then
  FOC, then open-loop 6-step ramp).
- That pattern works fine when there's no HWZC peripheral — the ADC
  sample *is* the timing event for everything: BEMF, current, state.
- When HWZC was added, only the **detection** side got decoupled.
  The **reaction** side (state machine, watchdogs) stayed in the ADC
  ISR because moving it would have been a large refactor with no
  immediate benefit at fixed PWM.
- "No immediate benefit" was true at a fixed PWM rate. Once the user
  started changing PWM frequency, it stopped being true.

## When To Actually Do This

Not today. Costs:

- Touches every file that does timing math
- Risks breakage in startup / morph / handoff sequences that were
  tuned around the current tick rate
- Requires re-tuning the PI in HR units (currently mostly already
  there, but mixed)

Benefits:

- PWM frequency becomes a free parameter (within the physics envelope
  from `pwm_frequency_effects.md`)
- No more `× PWMFREQUENCY_HZ / 1000` constants in `garuda_calc_params.h`
- Wall-clock-true watchdogs (currently they're wall-clock-true only
  because all the constants happen to scale together)
- Cleaner mental model — "PWM rate" stops being a control-loop knob
  and becomes a power-stage knob

This goes on the **after-slew-limiter** queue. The duty-down slew
limiter (Appendix B of session doc) is higher priority because it
solves the regen overvoltage AND the rapid-pot stall in one shot.

## References

- `dspic33AKESC/docs/pwm_frequency_effects.md` — the physics side
- `dspic33AKESC/docs/akesc_session_2026_05_26_191k_to_243k.md` —
  Appendix B (slew limiter rationale)
- `dspic33AKESC/garuda_service.c:303` — `adcIsrTick` declaration
- `dspic33AKESC/garuda_service.c:756` — the one increment
- `dspic33AKESC/garuda_calc_params.h` — every PWM-dependent
  derivation in one place
