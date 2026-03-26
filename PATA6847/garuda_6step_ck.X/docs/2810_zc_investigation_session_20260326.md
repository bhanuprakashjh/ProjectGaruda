# 2810 Motor ZC Investigation — Session 2026-03-26

## Starting Point

Commit `b430457` — 2810 motor showing `Frc=1` (forced commutation) on 65-95% of samples at all speeds. A2212 works fine on the same board (EV43F54A, dsPIC33CK + ATA6847).

## Fixes Applied (Commit `7622dad`)

### 1. HR Seeding at CL Entry (`garuda_service.c`)
**Problem**: At CL entry, `lastZcTickHR=0` (stale from init). Layer 2 (50% interval rejection in FastPoll) computes `elapsed = now - 0` = huge value → always passes → no interval rejection on first steps. False ZCs from demag could poison the IIR immediately.

**Fix**: Seed `lastZcTickHR = HAL_ComTimer_ReadTimer() - stepPeriodHR` and `checkpointStepPeriodHR = stepPeriodHR` at CL entry. This simulates "a ZC happened one period ago" so Layer 2 opens at the correct time.

### 2. Adaptive Layer 1 Blanking (`bemf_zc.c`, `garuda_config.h`)
**Problem**: Fixed 12% blanking + 25µs floor = 25% of step at 100k eRPM (Tp=2, 100µs step). ZcLat dropped to 0-2% — ZC arriving at the very edge of blanking.

**Fix**: Piecewise lerp 12%→8% (slow→fast speed), 15µs floor for 2810. Gated behind `FEATURE_IC_ZC_ADAPTIVE=1` (2810 only, Hurst/A2212 unchanged).

**Result at 100k eRPM**: Blk reduced from 25% to 14%. ZcLat improved from 0-2% to 3-25%.

### 3. ATA6847 EGBLT Increase (`hal_ata6847.c`)
**Problem**: GDUCR2 EGBLT was 5 (1.25µs hardware edge blanking). Switching transients at 17-24V exceeded this.

**Fix**: EGBLT=8 (2µs). Comparator output held inactive for 2µs after each PWM edge.

**Note**: EGBLT=12 (3µs) and EGBLT=15 (3.75µs) caused desync at mid-speed — too much HW blanking relative to the 5µs poll interval at 200kHz.

### 4. ZC Diagnostics (`garuda_types.h`, `gsp_commands.h`, `gsp_snapshot.c`, GUI, Python)
Added `ZC_DIAG_T` with:
- `zcLatencyPct` (0-255): ZC position in detection window, 0xFF=timeout
- `lastBlankingHR`: Layer 1 blanking applied (HR ticks)
- `diagBypassAccepted`: ZCs accepted via polarity bypass

Exported via GSP snapshot (48→52 bytes), GUI decode (length-guarded), Python tools.

## Bench Results After Commit `7622dad`

### Best Run (motor_test_20260322_200906.csv)
- Sweep 0→45% throttle, EGBLT=8, lerp 12→8%, 15µs floor
- **Stable to 102k eRPM**, ATA6847 fault (VDS short-circuit) at ~102k
- `Byp=1` throughout entire run (no bypass cascade)
- Ibus < 3A up to 90k eRPM
- Phase current spikes 5-15A at 100k+ eRPM (intermittent)
- `Frc=1` on 97% of samples (alternating miss pattern)
- `consecutiveMissedSteps=0` throughout (no real desync)

### ZcLat Distribution (Good Run, 87 CL Samples)
| Range | Count | Pct |
|-------|-------|-----|
| 0-25% | 29 | 33% |
| 25-50% | 35 | 40% |
| 50-75% | 20 | 22% |
| 75-100% | 3 | 3% |
| Timeout | 0 | 0% |

### Blanking vs Speed (Good Run)
| eRPM | Tp (T1) | Blk% | ZcLat | Ibus |
|------|---------|------|-------|------|
| 30k | 6 | 7% | 106-180 | <2A |
| 50k | 3 | 7% | 92-146 | <3A |
| 70k | 2 | 10% | 22-123 | <4A |
| 90k | 2 | 13% | 9-56 | <3A |
| 100k | 2 | 14% | 9-48 | 3-15A |

## Experiments That Failed

### 1. Bypass Guard (Commit 2 attempt)
**Change**: Gate polarity bypass (`stepsSinceLastZc >= 2`) with elapsed time bounds (40-160% of stepPeriodHR).
**Result**: Catastrophic. Byp exploded to 5722 at ~58k eRPM. At high speed, demag tail timing falls within the 40-160% window → bypass fires continuously → 34A current spikes, Vbus collapsed from 24V to 10V.

### 2. Lower Blanking Floor (10µs, 8µs)
**Change**: Reduce ZC_BLANK_FLOOR_US from 15µs to 10µs or 8µs.
**Result**: Byp exploded at CL entry. 15µs floor protects against comparator instability during the first CL steps. Below 15µs, false ZCs are accepted immediately → IIR poisoned → cascade.

### 3. Timing Advance Change (MAX_CLOSED_LOOP_ERPM 150k→110k)
**Change**: Steeper advance curve — 25° at 100k instead of 17°.
**Result**: Desync at 54k eRPM. Earlier commutation puts comparator in wrong state → bypass fires.

### 4. `min(cap, max(floor, pct×step))` Blanking Model
**Change**: Clean single-pct blanking with 25µs floor and 20% cap.
**Result**: At 100k eRPM, floor (25µs = 25% of step) dominated again. Cap can't go below floor → same over-blanking issue. Byp=25045, Vbus collapsed to 10V.

### 5. GDUCR3 Slew Rate Experiments

| GDUCR3 Setting | HS Slew | LS Slew | Adaptive DT | Result |
|----------------|---------|---------|-------------|--------|
| 0x0C (original) | 12.5% | Full | Off | **Best** — stable to 100k |
| 0x55 (both 50%) | 50% | 50% | 50-150ns | Desync at 28k eRPM |
| 0x50 (both full) | Full | Full | 50-150ns | Desync at 45k eRPM |

**Conclusion**: The asymmetric slew (slow HS, fast LS) is correct for this board. Slow HS reduces high-side ringing. Changing either direction made things worse. Adaptive dead time caused unreliable CL entry (3/8 attempts failed).

### 6. EGBLT Experiments

| EGBLT | Time | Result |
|-------|------|--------|
| 5 | 1.25µs | Blk=25% at 100k (floor dominated), ZcLat=0-2% |
| **8** | **2µs** | **Blk=14%, ZcLat=3-25% at 100k — best** |
| 12 | 3µs | Desync at 50k — comparator blind 60% of poll cycle |

### 7. Duty Slew Rate Reduction
**Change**: DUTY_SLEW_UP from 25 to 3 counts/tick (20ms → 200ms full sweep).
**Result**: No improvement. Desync still occurs with barely any pot movement (+48 counts). The issue is speed-dependent, not acceleration-dependent.

## Root Cause Analysis

### The Frc=1 Alternating Miss Pattern
- Present in ALL runs, ALL speeds, ALL firmware versions
- 97% of CL samples show `stepsSinceLastZc=1`
- `consecutiveMissedSteps=0` — no real consecutive misses
- **Partially a sampling artifact**: ZC resets `stepsSinceLastZc=0`, but commutation immediately increments it to 1. Random 10Hz sampling sees 1 for ~78% of the step duration even with perfect detection.
- **Partially real**: At high speed, phase current spikes indicate some commutations are at wrong angles

### The Desync Cascade Mechanism
Every catastrophic failure follows the same pattern:

1. Motor running at 45-60k eRPM (Tp=3-4 Timer1 ticks, spHR=300-400)
2. eRPM suddenly jumps to 100-120k in ONE sample interval (~100ms)
3. stepPeriodHR halves from ~300 to ~140 (exactly 2×)
4. `Byp` jumps from 1 to 100-300 immediately
5. Current spikes to 25-34A
6. Motor stuck at ~110k eRPM drawing massive current
7. Cannot recover — cascade is self-sustaining
8. Eventually UV fault (Vbus collapses) or ATA6847 fault

### The Desync Trigger — Not Speed-Dependent, Not Duty-Dependent

Tested with and without props:
- **Without props**: Desync at 41-52k eRPM, current near zero (200mA) → initially suspected no-load sensitivity
- **With props (loaded)**: Same desync at 45-57k eRPM, current 1.6-8.5A → **no-load hypothesis disproven**
- Desync occurs with minimal pot movement (+48 counts, duty unchanged)
- Duty slew rate reduction (25→3) did not help

### What the stepPeriodHR Halving Means

stepPeriodHR going from 300 to 140 means the IIR has converged to half the real period. This requires false ZCs being accepted at **twice the real commutation rate** — one real + one false ZC per step. The false ZC occurs at ~50% of the step (passes Layer 2 rejection) but at the wrong electrical angle.

The false ZC enters via the **polarity bypass** (`stepsSinceLastZc >= 2`). When two consecutive real ZCs are missed, the bypass accepts any stable comparator state. If the comparator shows a stable state at the 50% mark (which it often does — the BEMF waveform crosses neutral at 50%), the false ZC is accepted. This corrupts the IIR, which causes subsequent commutations to fire at wrong times, which causes more missed ZCs, which keeps the bypass active.

### Why Two Consecutive ZCs Are Missed

The 6-step commutation alternates ZC polarity every step:
- Steps 0, 2, 4: Rising ZC (expected comparator output = 1)
- Steps 1, 3, 5: Falling ZC (expected comparator output = 0)

The ATA6847 comparator likely has **asymmetric response** for rising vs falling BEMF crossings — different propagation delay, different threshold offset, or different noise immunity. If one polarity consistently fails or is delayed, the deglitch filter can't accumulate enough matching reads in the available window → miss on that polarity every step.

At lower speeds (<45k eRPM), the detection window is large enough to tolerate the asymmetry. At higher speeds, the window shrinks and the asymmetry causes occasional double-misses → bypass → cascade.

### The ATA6847 Comparator Limitation

The ATA6847's BEMF comparator is a black box:
- Internal virtual neutral — not adjustable
- No threshold adjustment register
- No propagation delay control
- BEMF enable (GDUCR1.BEMFEN) is the only on/off control
- Edge blanking (GDUCR2.EGBLT) applies to PWM edges, not BEMF comparator
- Slew rate (GDUCR3.HSSRC/LSSRC) affects FET switching, indirectly affects comparator noise

The comparator outputs are digital GPIO pins (RC6/RC7/RD10) — no analog access for software-based threshold adjustment.

## Current State (End of Session)

### Committed Code (`7622dad`)
- HR seeding at CL entry ✓
- Adaptive blanking lerp 12→8%, 15µs floor (2810 only) ✓
- EGBLT=8 (2µs HW blanking) ✓
- ZC diagnostics (zcLatencyPct, lastBlankingHR, diagBypassAccepted) ✓
- GDUCR3=0x0C (HS=12.5%, LS=full, no adaptive dead time) ✓

### Uncommitted Changes
- DUTY_SLEW_UP reduced from 25 to 3 (didn't help but doesn't hurt)

### Performance Summary
| Metric | Before (b430457) | After (7622dad) |
|--------|------------------|-----------------|
| CL entry reliability | ~70% | ~95% |
| Max stable eRPM | ~50k (cascade) | ~100k |
| High-speed Ibus | 34A (cascade) | <3A (up to 90k) |
| Phase current at 100k | 34A (cascade) | 5-15A (intermittent) |
| Bypass count (good run) | 2000+ | 1 |
| ZcLat at 100k | N/A (cascade) | 3-25% |
| ATA6847 fault threshold | ~60k (cascade) | ~102k (VDS SC) |

### Remaining Issues
1. **Frc=1 pattern**: Alternating ZC miss — likely ATA6847 comparator polarity asymmetry. Causes intermittent phase current spikes at 90-100k eRPM.
2. **Desync at ~50-60k eRPM during acceleration**: Speed-dependent, occurs even with slow duty ramp and under prop load. Root cause is double-miss → bypass → IIR corruption.
3. **102k eRPM ceiling**: ATA6847 VDS short-circuit fault (SIR1 bit 1). Switching transients at extreme speed exceed SCTHSEL=5 (1500mV) threshold.

## Next Steps

### Immediate (Next Session)
1. **Per-polarity ZC diagnostics**: Add rising/falling ZC counters to confirm polarity asymmetry hypothesis
2. **Speed PI controller**: Map pot to target eRPM, PI regulates duty. Naturally limits acceleration, prevents motor from entering unstable speed zones
3. **Speed limiter**: Hard duty cap when stepPeriod <= 3 to prevent Tp=2 cliff

### Future
4. **ADC-based BEMF sensing**: Bypass ATA6847 comparator entirely. Read motor terminal voltages via dsPIC ADC, compute virtual neutral in software, detect ZC with adjustable threshold. Full control over detection parameters.
5. **Per-polarity advance correction**: If polarity asymmetry confirmed, apply different advance delays for rising vs falling ZC steps
6. **VDS threshold increase**: Raise SCPCR.SCTHSEL from 5 (1500mV) to 6 or 7 to allow higher speed before ATA6847 fault

## Key Files

| File | Purpose |
|------|---------|
| `motor/bemf_zc.c` | ZC detection — Layer 1 blanking, FastPoll (Layer 2), RecordZcTiming (Layer 3) |
| `garuda_service.c` | CL state machine, HR seeding, duty slew |
| `garuda_config.h` | Per-profile blanking knobs, feature flags |
| `hal/hal_ata6847.c` | ATA6847 register init (GDUCR1-4, EGBLT, slew, SCPCR) |
| `garuda_types.h` | ZC_DIAG_T struct |
| `gsp/gsp_commands.h` | GSP_CK_SNAPSHOT_T (48→52 bytes) |
| `gsp/gsp_snapshot.c` | Diagnostic capture |
| `tools/pot_capture.py` | Manual pot testing with telemetry logging |
| `tools/motor_sweep_test.py` | Automated throttle sweep |

## Test Data Files

| File | Description |
|------|-------------|
| `motor_test_20260322_200906.csv` | Best run — EGBLT=8, lerp, stable to 102k |
| `motor_test_20260322_195750.csv` | First EGBLT=8 run (good baseline) |
| `motor_test_20260326_131902.csv` | 25µs floor regression — Byp=25045 |
| `motor_test_20260326_132214.csv` | 10µs floor regression — Byp=25045 |
| `motor_test_20260326_133111.csv` | Advance curve change — desync at 54k |
| `motor_test_20260326_153211.csv` | Both 50% slew — desync at 28k |
| `motor_test_20260326_153433.csv` | Both full slew — desync at 45k |
| `pot_capture_20260326_154504.csv` | Full speed + adaptive DT — 3 runs, 1 good |
| `pot_capture_20260326_155648.csv` | Original GDUCR3 — pot sweep, 8 runs |
| `pot_capture_20260326_160742.csv` | Reduced slew (3) — still desyncs |
| `pot_capture_20260326_162103.csv` | **With props** — same desync pattern at 45-57k |
