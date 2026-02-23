# HWZC Filter Architecture Notes

## Date: 2026-02-23

## Current Implementation (committed 868b2ff)

Single-stage 70% interval filter + 3-miss limit:

- **HWZC_MIN_INTERVAL_PCT = 70**: Rejects any ZC candidate whose interval
  from the previous ZC is < 70% of stepPeriodHR. Catches PWM switching noise
  (very short intervals) reliably.
- **HWZC_MISS_LIMIT = 3**: After 3 consecutive timeouts, disable HWZC and
  fall back to software ZC. Latches `dbgLatchDisable` to prevent cycling.
- **stepsSinceLastHwZc == 1 guard**: Only updates IIR estimator from
  consecutive ZC-to-ZC measurements. Multi-step gaps (containing forced
  steps or timeouts) are excluded from IIR.
- **rejectsThisStep**: Diagnostic counter, reset each commutation.
- **Stall detector**: Existing noise-reject-limit + interval rejection as
  last safety net.

### What was removed and why

- **HWZC_IIR_GUARD_PCT (85%)**: Rejected ZC candidates whose interval
  deviated >15% from stepPeriodHR. Failed because motor overshoots ramp
  target without prop (real ZC at 75-80% of seeded period), writeSeq
  stayed 0, IIR never adapted, noise detect limit then disabled HWZC.
  Confirmed broken in cl17, fixed in cl18.
- **HWZC_NOISE_DETECT_LIMIT (100)**: Disabled HWZC after 100 rejected
  candidates. Triggered as a cascade from IIR guard rejecting real events.

---

## Proposed Multi-Stage Architecture (evaluated, deferred)

A 2-stage filter + quality supervisor was proposed:

### 1. Time-window gate (relative to commutation)

Accept candidates only in expected ZC window:
`open = max(blank_end, 0.22*sp), close = 0.82*sp`

**Assessment: RISKY for current hardware.** During acceleration (no prop,
motor overshooting ramp target), real ZC arrives early — possibly before
0.22*sp. During deceleration (adding load), it arrives late — possibly
after 0.82*sp. Same family of problem as the IIR guard failure (cl17).

### 2. Dual-threshold acceptance

- Candidate gate (loose): for scheduling commutation.
- Trusted gate (strict, 0.85-1.20*sp): for IIR and goodZcCount.

**Assessment: RISKY.** The strict bounds (0.85*sp) are nearly identical to
the HWZC_IIR_GUARD_PCT=85 we removed. Re-introduces the same failure mode
where real ZCs during speed overshoot are classified as untrusted, IIR
never adapts, and HWZC eventually disables.

### 3. Per-step reject cap (ISR storm control)

Use rejectsThisStep. If rejects exceed N (e.g. 8), stop re-arming
comparator for that step and let timeout fire.

**Assessment: WORTH IMPLEMENTING.** Simple (5 lines), prevents ISR storm
from comparator ringing eating all CPU. We already have the counter field.
Only addition needed in the current code.

### 4. Don't reward weak accepts

If candidate passes loose gate but fails strict gate: allow commutation
but do not increment goodZcCount and do not update IIR.

**Assessment: SOUND CONCEPT.** Separating commutation trigger from
confidence building is reasonable. We already do a partial version with
the stepsSinceLastHwZc==1 guard.

### 5. Quality score state machine

Maintain signed score (-16..+16):
+2 trusted ZC, +1 loose accept, -3 reject storm, -4 timeout.
Disable HWZC when score drops below threshold.

**Assessment: OVER-ENGINEERED for current hardware.** Adds ~5 tuning
constants and non-obvious interaction paths inside a priority-7 ISR.
On the MCLV-48V-300W board, observed noise modes are:
- PWM switching → 70% interval filter catches these (very short)
- Comparator ringing → reject cap would catch these (burst)
- Motor desync → stall detector catches these
No observed failure mode where noise closely mimics plausible ZC timing.

### 6. Keep stall plausibility

**Assessment: ALREADY IMPLEMENTED.** Noise reject limit + interval
rejection serve as last safety net.

---

## Key Insight: Over-filtering vs Under-filtering

On the MCLV-48V-300W board with 48V-rated voltage dividers running at 12V,
BEMF signals at handoff speeds produce only 18-59 ADC counts. The primary
risk is **missing real ZCs** (weak signals), not accepting fake ones.
Every filtering layer increases the chance of rejecting weak-but-real events.

The IIR guard failure (cl17) proved this: the motor was running fine, real
ZCs were arriving, but they were being rejected by an overly strict filter.

## Recommendation

1. Add per-step reject cap (#3) — next available slot.
2. Keep current simple architecture for MCLV-48V-300W board.
3. Revisit multi-stage architecture for custom board where:
   - CMP inputs give clean, high-amplitude signals
   - 12V-rated dividers provide 4x better ADC resolution
   - Target speeds (100K+ eRPM) may produce structured noise
