# High-Speed ZC Detection Research — 2026-04-04

## Summary

Achieved **zero timeouts at 100k eRPM** on the 2810 motor at 24V (no-load) and clean prop operation, up from a baseline of 2-14 timeouts/snapshot at 85-95k eRPM.

## Root Cause Analysis

### The Two Bugs (confirmed by Codex review)

**Bug 1: Stale CLC D-FF from blanking period**
- CLC D-FF (MODE=0b100, edge-triggered) samples comparator once per PWM cycle
- ForcePreZcState seeds Q at commutation, but the next PWM clock can overwrite it
- If the comparator is in a transient state during blanking, D-FF captures and holds that wrong state
- FL=1 in TRACK mode accepted this stale capture immediately → wrong ZC timing
- **Fix**: FL=2 at high speed (StpHR < 300, ~52k+ eRPM) requires two matching reads

**Bug 2: IC-CLC hybrid path hazard**
- IC captures the first edge after arming (may be a bounce, not the real ZC)
- Poll later reads CLC level and confirms with FL=1
- IC timestamp and CLC confirmation can refer to different physical events
- RecordZcTiming commits a precise but potentially wrong lastZcTickHR
- At high speed, the scheduling then collapses to "fire ASAP" → wrong commutation angle
- **Fix**: Validate IC timestamp age — if older than 1 PWM cycle when poll confirms, use poll timestamp instead

### Why Zero Timeouts ≠ Correct Commutation
Before the fix, the motor showed zero timeouts at 100k but drew 3-34A at no-load (should be ~2A). The ZC was "detected" before the deadline, but the IC timestamp was from a bounce, causing commutation at the wrong angle. The current spikes were from driving the motor out of phase, not from missed ZCs.

## What Was Tried

### PWM Frequency Tests

| Frequency | Aliasing Points (eRPM) | Best Zone | Result |
|-----------|----------------------|-----------|--------|
| 15 kHz | 50k(3:1), 75k(2:1) | 75-150k (no aliasing) | Fewer CLC samples, worse overall |
| 24 kHz | 60k(4:1), 80k(3:1), 120k(2:1) | **0-85k** (wide clean zone) | CLC bottleneck at 85k+ |
| 32 kHz | 64k(5:1), 80k(4:1), 107k(3:1) | 0-50k | Aliasing in 50-85k |
| **40 kHz** | 80k(5:1), 100k(4:1), 133k(3:1) | **100-133k** | Zero TOs at 100k+ with fixes |

### CLC Mode Tests

| Mode | Description | Result |
|------|-------------|--------|
| MODE=0b100 (D-FF) | Edge-triggered, samples once/PWM | **Best** — noise filtering via hold |
| MODE=0b111 (D-latch) | Transparent, real-time tracking | Worse — passes all comparator noise |

### ATA6847 Register Tuning

| Register | Change | Effect |
|----------|--------|--------|
| GDUCR3 LSSRC | Full → 12.5% | Reduced LS dV/dt coupling, 32% fewer TOs at 70-85k |
| GDUCR3 ADDTHS/LS | Off → 50-160ns | Reduced body diode noise, 74% fewer TOs at 85-95k |
| SCPCR SCFLT | 7 → 15 (7.5µs) | Prevents regen VDS faults at 24V |
| SCPCR SCTHSEL | 7 (2000mV) | Already maxed, can't go higher |

### PTG Edge-Relative Sampling (FEATURE_PTG_ZC)
Attempted three approaches:
1. PTG ISR reads raw GPIO → FastPoll uses instead of CLC → **worse** (lost D-FF filtering)
2. PTG ISR forces CLC via R/S → FastPoll reads CLC → **worse** (PWMEVTA fights PTG)
3. PTG ISR forces CLC with PWMEVTA clock disabled → **worse** (insufficient update rate)

Conclusion: PTG doesn't help because the CLC D-FF's sample-and-hold IS the noise filter. Code kept behind `FEATURE_PTG_ZC=0` for future reference.

### Other Attempts
- Raw GPIO at high speed (polarity inversion) → desync at threshold crossing
- IC direct acceptance (skip CLC) → estimator corruption from unvalidated edges
- Duty cut on forced commutation → ATA6847 VDS fault from regen spike
- Variable deglitch filter → helps at some speeds, hurts at others

## Key Discoveries

### CLC D-FF is Not a Transparent Latch
- MODE=0b100 = D Flip-Flop (edge-triggered), not D-latch
- MODE=0b111 = transparent latch (tested, worse)
- The D-FF samples once per PWM cycle and holds → noise filtering
- This is WHY the CLC helps — it filters out inter-cycle comparator bounces

### CLC GLSL Bit Mapping (Comment Bug Fixed)
```
CLC1GLSL = (1u << 7) | (1u << 9)
  Bit 7 = G1D4N (DS4 NEGATED) → CLK = !PWMEVTA (falling edge trigger)
  Bit 9 = G2D1N (DS1 NEGATED) → D = !BEMF_A
  Comment previously said "G1D4T" — WRONG, it's G1D4N
  Polarity is consistent: CLC output = !raw_GPIO, cmpExpected matches
```

### "After-PWM" Step Asymmetry
Steps 1/3/5 (floating a phase just released from PWM) consistently have more timeouts than steps 0/2/4 (floating a phase released from LOW). This is a topology/transition-history effect:
- PWM→float: switching-node dV/dt, cap charge/discharge, body-current commutation
- LOW→float: clamped, monotonic starting condition
The CLC D-FF and IC both amplify this asymmetry.

### PWM-Step Integer Aliasing
When step_period / PWM_period is an integer, switching edges land at the same position every step. This causes step-specific noise patterns that persist. The non-integer poll frequency (210.5kHz) breaks poll-PWM aliasing but not step-PWM aliasing.

### 24kHz Was Best for Low-Speed, 40kHz for High-Speed
- 24kHz: no integer aliasing at 85-110k, but CLC bottleneck (1.3 samples in window)
- 40kHz: more CLC samples, but integer aliasing at 80k and 100k
- With the FL=2 + IC validation fixes, 40kHz achieves zero timeouts at 100k+
- The fixes don't yet work at 24kHz (IC validation maxAge threshold mismatch)

## Current Configuration (Commit 4e3d587)

```
PWM: 40 kHz
CLC: MODE=0b100 (D-FF), G1D4N clock, G2D1N data
Poll: 210.5 kHz (non-integer ratio)
TRACK FL: 2 at StpHR<300, 1 otherwise
IC validation: maxAge = 1 PWM cycle, only at StpHR<300
LSSRC: 12.5% (slowest)
ADDTHS/LS: 50-160ns
EGBLT: 15 (3.75µs max)
SCFLT: 15 (7.5µs max)
SCTHSEL: 7 (2000mV max)
FEATURE_PTG_ZC: 0 (disabled)
```

## Open Issues

### P0: 24kHz Support
The FL=2 + IC validation fixes work at 40kHz but break at 24kHz. The IC validation maxAge (1 PWM cycle = 25µs at 40kHz, 41µs at 24kHz) doesn't account for the D-FF update delay correctly at 24kHz. The D-FF can take up to 42µs to update after IC captures, exceeding the 41µs maxAge → valid IC timestamps rejected → wrong poll timestamps used → cascading failures.

**Possible fixes:**
- Increase maxAge to 2+ PWM cycles at 24kHz (tried, caused other issues)
- Make maxAge speed-dependent (shorter at high speed, longer at low)
- Different validation approach (compare IC vs poll timestamps, use the closer one)
- Keep 40kHz and tune CL_IDLE for low-speed operation

### P1: CL_IDLE_DUTY at 40kHz
At 40kHz, 10% CL_IDLE_DUTY has ~3.2% dead-time overhead → only ~6.8% effective. At no-load, the motor at idle (9% duty) has constant timeouts. With prop, this is masked by load keeping the motor in a higher operating range. Fix: increase CL_IDLE_DUTY_PERCENT to 15-20% for 40kHz.

### P2: Startup Ramp Duty at 40kHz
ALIGN_DUTY and RAMP_DUTY_CAP need adjustment for 40kHz dead-time overhead. Currently using /20 (5%) and /8 (12.5%) which work but draw 15-18A during ramp. Consider motor-profile-specific startup params that account for PWM frequency.

### P3: Variable PWM Frequency
AM32 uses speed-adaptive PWM frequency. This could give the best of both worlds — 24kHz at low speed (wide aliasing gap, proven startup) and 40kHz at high speed (more CLC samples). Needs smooth transition to avoid instability at the frequency switch point.

### P4: BEMF Integration (VESC approach)
Instead of detecting instantaneous ZC crossing, accumulate BEMF area. When the integral exceeds a threshold proportional to the commutation interval, commutate. Inherently immune to comparator bounces and timing hazards. Major architecture change.

## Test Data Files

All CSV captures from this session are in the PATA6847 directory:
- `pot_capture_20260404_*.csv` — chronological test runs
- Key runs:
  - `_143926` — 24kHz baseline (1403 TOs)
  - `_151520` — LSSRC+ADT 50-160ns best at 24kHz
  - `_230438` — **40kHz + both fixes, ZERO TOs at 100k** (the breakthrough)
  - `_234101` — 40kHz confirmed zero TOs 20-107k
  - `_235436` — prop test at 40kHz
