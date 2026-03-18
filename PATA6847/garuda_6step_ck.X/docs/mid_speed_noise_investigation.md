# Mid-Speed Motor Noise Investigation

**Date**: 2026-03-18
**Motor**: 5010 750KV at 24V, no load
**Symptom**: Motor is silent at high speed (Tp:2-4), noisy at mid speed (Tp:7-10)

---

## Observation

From telemetry capture (run3, 5010 at 24V):

| Speed Band | eRPM | ZcI Alternation (avg) | ZcI Alternation (max) | Sound |
|-----------|------|----------------------|----------------------|-------|
| Tp:2-4 | 50k-100k | 0.1-0.5 | 1 | Silent |
| Tp:5-6 | 33k-40k | 0.5-1.1 | 2-4 | Quiet |
| **Tp:7-10** | **20k-28k** | **1.8-6.9** | **11-21** | **Noisy** |

ZC interval alternation = consecutive step periods differ significantly.
This causes uneven commutation spacing = mechanical vibration = acoustic noise.

---

## Hypothesis 1: BEMF Comparator Offset Asymmetry

**Claim**: The three ATA6847 BEMF comparators (RC6/RC7/RD10) have
different voltage offsets, causing one phase's ZC to be detected
consistently early/late relative to the others.

**Supporting observation** (from CSV at Tp:8):
```
step:1(sense A) ZcI:11 → step:4(sense A) ZcI:8 → repeats
step:2(sense B) ZcI:10 → step:5(sense B) ZcI:10 → repeats
step:0(sense C) ZcI: 8 → step:3(sense C) ZcI: 9 → repeats
```

The step pairs (which sense the SAME phase at opposite polarity)
show consistent ZcI values across many samples. This SUGGESTS
per-phase asymmetry.

**Weakness of this evidence**: The telemetry samples at 100ms (10Hz)
capture whichever step is active at that moment. The ZcI/PZcI values
are the LAST two intervals, not necessarily from the displayed step.
The correlation between step number and ZcI could be coincidental
due to the sampling aliasing.

**Why it's worse at mid-speed**: At low speed, BEMF crosses zero slowly.
A comparator offset of ±50mV translates to a larger timing error when
dV/dt is small. At high speed, BEMF slew rate is fast (steep crossing),
so ±50mV offset = negligible timing error.

---

## Hypothesis 2: Timer1 Quantization at Mid-Speed

**Claim**: At Tp:7-10, Timer1 ticks (50µs) are a coarse fraction of
the step period. ±1 tick = ±7-14% timing jitter. The IIR step period
filter averages alternating intervals, making BOTH sides slightly wrong.

**Counter-evidence**: The HR timer (640ns) IS active at these speeds.
The scheduling condition `stepPeriod < HR_MAX_STEP_PERIOD (800)` is
met at Tp:8. So commutation scheduling already uses HR precision.

However, the ZC DETECTION still uses the 100kHz poll timer (10µs
resolution). The detected ZC moment has ±10µs uncertainty regardless
of HR scheduling. At Tp:8 (400µs step), ±10µs = ±2.5% — smaller
than the observed alternation of 30%.

**Conclusion**: Timer1 quantization is NOT the primary cause.
The 100kHz poll adds some jitter but not enough to explain
the 30% ZcI alternation.

---

## Hypothesis 3: IIR Step Period Filter Artifacts

**Claim**: The IIR filter `(3*old + 1*new) / 4` tracks the AVERAGE
of alternating intervals. If real intervals are 8 and 11 ticks,
the IIR converges to ~9.5. The scheduled commutation delay is
computed from this averaged value, which is wrong for BOTH the
short and long intervals. This amplifies the alternation instead
of correcting it.

**This is testable**: change the scheduling to use the RAW latest
interval instead of the IIR-filtered value. If alternation decreases,
the IIR was amplifying it.

---

## Test Plan

### Test A: Prove/Disprove Per-Phase Asymmetry

**Firmware change**: Add per-step ZcI accumulators.

```c
// In GARUDA_DATA_T or IC_ZC_STATE_T:
uint32_t zcIntervalAccum[6];   // Sum of ZcI per step
uint16_t zcIntervalCount[6];   // Count per step

// In RecordZcTiming(), after accepting a ZC:
gData.zcIntervalAccum[gData.currentStep] += interval;
gData.zcIntervalCount[gData.currentStep]++;
```

**Test procedure**:
1. Run motor at constant speed (hold pot at Tp:8 zone) for 10s
2. Read accumulators via diagnostic command or GSP
3. Compute average ZcI per step: `avg[i] = accum[i] / count[i]`
4. If steps 0/3, 1/4, 2/5 have systematically different averages → **proof of comparator asymmetry**
5. If all 6 steps have similar averages → disproved, look elsewhere

**Expected result if hypothesis is correct**:
```
Step 0 (sense C rising):  avg_ZcI ≈ 8.2
Step 1 (sense A falling): avg_ZcI ≈ 10.8
Step 2 (sense B rising):  avg_ZcI ≈ 10.1
Step 3 (sense C falling): avg_ZcI ≈ 8.5
Step 4 (sense A rising):  avg_ZcI ≈ 7.8
Step 5 (sense B falling): avg_ZcI ≈ 10.3
```
Steps sensing the same phase (0/3, 1/4, 2/5) should pair up.

### Test B: HR Timer for All Speeds

**Firmware change**: Remove the HR_MAX_STEP_PERIOD gate.

```c
// In BEMF_ZC_ScheduleCommutation(), change:
if (pData->timing.hasPrevZcHR &&
    pData->timing.stepPeriodHR > 0 &&
    pData->timing.stepPeriod < HR_MAX_STEP_PERIOD)

// To:
if (pData->timing.hasPrevZcHR &&
    pData->timing.stepPeriodHR > 0)
```

**What this changes**: HR scheduling (640ns precision) used at ALL
speeds, not just Tp < 800. This tests whether Timer1 quantization
contributes to the alternation.

**What could break**:
- HR timer wraps at 65535 × 640ns = 41.9ms. At Tp:800 (40ms step),
  the HR timer barely fits. Above Tp:800, the step period exceeds
  the HR timer wrap → scheduling would fail (target wraps around).
  This is WHY HR_MAX_STEP_PERIOD exists.
- At Tp:7-10 (350-500µs step), HR timer is well within range.
  Should be safe to test.

**Conservative approach**: Lower HR_MAX_STEP_PERIOD from 800 to
a value that covers Tp:10 (500µs = 781 HR ticks) but not Tp:800.
Set to 50 (2.5ms step = 3906 HR ticks, well within 65535 wrap).

```c
#define HR_MAX_STEP_PERIOD  50U  // Was 800, test at 50
```

**Test procedure**:
1. Run motor at Tp:7-10 speed range
2. Capture ZcI alternation
3. Compare with baseline (HR_MAX_STEP_PERIOD=800)
4. If alternation decreases → Timer1 quantization was contributing
5. If no change → confirms it's comparator offset, not scheduling

### Test C: Raw Interval Scheduling (IIR Bypass)

**Firmware change**: In `BEMF_ZC_ScheduleCommutation()`, use raw
2-step average instead of IIR for scheduling delay.

```c
// Current code uses sp = stepPeriod (IIR filtered)
// Change to always use raw average:
uint16_t sp;
if (pData->timing.zcInterval > 0 && pData->timing.prevZcInterval > 0) {
    sp = (pData->timing.zcInterval + pData->timing.prevZcInterval) / 2;
} else {
    sp = pData->timing.stepPeriod;
}
```

**What this tests**: Whether the IIR filter is amplifying alternation
by averaging asymmetric intervals into a value that's wrong for both.

**Risk**: Raw intervals are noisier. May increase jitter at other speeds.
Only test at the noisy speed band (Tp:7-10).

### Test D: Per-Phase Trim (if Hypothesis 1 confirmed)

**After Test A confirms asymmetry**, implement per-phase correction:

```c
// Measured offsets from Test A (example values):
static const int8_t phaseZcTrim[6] = {
    +2,  // Step 0 (sense C): detected early, delay more
    -1,  // Step 1 (sense A): detected late, delay less
     0,  // Step 2 (sense B): reference
    +2,  // Step 3 (sense C): same as step 0
    -1,  // Step 4 (sense A): same as step 1
     0,  // Step 5 (sense B): same as step 2
};

// In ScheduleCommutation, after computing delay:
delay += phaseZcTrim[pData->currentStep];
```

**This would eliminate the asymmetry at all speeds** — not just the
noisy range. The silent high-speed range would become even cleaner.

---

## Priority Assessment

| Test | Effort | Value | Risk |
|------|--------|-------|------|
| **A: Per-step accumulators** | 1 hour | High (proves root cause) | None |
| **B: HR for all speeds** | 10 min | Medium (eliminates one variable) | Low |
| **C: Raw interval scheduling** | 10 min | Medium (tests IIR theory) | Medium |
| **D: Per-phase trim** | 2 hours | High (if A confirms) | Low |

**Recommended order**: B first (quickest, eliminates Timer1), then A
(proves comparator theory), then D (fix if confirmed).

---

## References

- AM32: lives with this alternation, no per-phase correction
- BLHeli_S: same issue, compensated by demag detection
- VESC: avoids entirely by using FOC (sinusoidal currents, no step-wise commutation)
- Microchip AN1078: mentions comparator offset as source of 6-step commutation noise
