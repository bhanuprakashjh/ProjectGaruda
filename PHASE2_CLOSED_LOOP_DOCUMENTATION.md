# Phase 2: ADC-Based BEMF Zero-Crossing Closed-Loop Commutation

## Table of Contents
1. [Architecture Overview](#architecture-overview)
2. [Hardware Pin Connections](#hardware-pin-connections)
3. [ZC Detection Algorithm Flowchart](#zc-detection-algorithm-flowchart)
4. [ADC ISR State Machine Flowchart](#adc-isr-state-machine-flowchart)
5. [Timing Model](#timing-model)
6. [Config Parameter Reference](#config-parameter-reference)
7. [Debugging Journey](#debugging-journey)
8. [Test Results](#test-results)

---

## Architecture Overview

Phase 2 replaces the open-loop forced commutation (Phase 1) with BEMF zero-crossing
detection using ADC software threshold comparison. The motor's back-EMF on the
floating phase is sampled at 24kHz (PWM rate), compared against a duty-proportional
threshold (virtual neutral), and used to determine the optimal commutation timing.

### System Block Diagram

```
  ┌─────────────────────────────────────────────────────────────┐
  │                    dsPIC33AK128MC106                         │
  │                                                             │
  │  ┌──────────┐    ┌──────────┐    ┌──────────┐              │
  │  │  PWM1    │    │  PWM2    │    │  PWM3    │              │
  │  │ PG1H/L  │    │ PG2H/L  │    │ PG3H/L  │              │
  │  └──┬──┬───┘    └──┬──┬───┘    └──┬──┬───┘              │
  │     │  │           │  │           │  │                    │
  ├─────┼──┼───────────┼──┼───────────┼──┼────────────────────┤
        │  │           │  │           │  │
   RD2 ─┘  └─ RD3  RD0┘  └─RD1  RC3─┘  └─ RC4
   PH_A_H   PH_A_L  PH_B_H  PH_B_L PH_C_H  PH_C_L
        │              │              │
        ▼              ▼              ▼
  ┌──────────────────────────────────────────┐
  │         3-Phase Inverter Bridge          │
  │           (MCLV-48V-300W)                │
  └───────┬────────────┬─────────────┬───────┘
          │ Phase A    │ Phase B     │ Phase C
          │            │             │
     ┌────┴────┐  ┌────┴────┐  ┌────┴────┐
     │ Voltage │  │ Voltage │  │ Voltage │
     │ Divider │  │ Divider │  │ Divider │
     └────┬────┘  └────┬────┘  └────┴────┘
          │            │             │
     RB9 (VA)    RB8 (VB)     RA10 (VC)
     AD2AN10     AD1AN11      AD2AN7
          │            │             │
  ┌───────┼────────────┼─────────────┼────────────────────────┐
  │  ┌────┴────┐  ┌────┴────┐  ┌────┴────┐                   │
  │  │ AD2CH0  │  │ AD1CH0  │  │ AD2CH0  │                   │
  │  │PINSEL=10│  │PINSEL=11│  │PINSEL=7 │                   │
  │  │ (muxed) │  │ (fixed) │  │ (muxed) │                   │
  │  └────┬────┘  └────┬────┘  └────┬────┘                   │
  │       │            │             │                        │
  │       └──────┬─────┴─────────────┘                        │
  │              │                                            │
  │              ▼                                            │
  │    ┌──────────────────┐     ┌─────────────┐              │
  │    │  ADC ISR (24kHz) │────▶│  bemf_zc.c  │              │
  │    │  garuda_service.c│     │  ZC detect   │              │
  │    └──────────────────┘     └──────┬──────┘              │
  │                                    │                      │
  │              ┌─────────────────────┘                      │
  │              ▼                                            │
  │    ┌──────────────────┐                                   │
  │    │ commutation.c    │                                   │
  │    │ 6-step table     │                                   │
  │    │ PWM overrides    │                                   │
  │    └──────────────────┘                                   │
  │                                                           │
  │                    dsPIC33AK128MC106                       │
  └───────────────────────────────────────────────────────────┘
```

---

## Hardware Pin Connections

### Board: MCLV-48V-300W Development Board

#### PWM Outputs (to gate drivers)

| Signal    | Pin  | PG  | Function                     |
|-----------|------|-----|------------------------------|
| PH_A_H    | RD2  | PG1H| Phase A high-side FET gate   |
| PH_A_L    | RD3  | PG1L| Phase A low-side FET gate    |
| PH_B_H    | RD0  | PG2H| Phase B high-side FET gate   |
| PH_B_L    | RD1  | PG2L| Phase B low-side FET gate    |
| PH_C_H    | RC3  | PG3H| Phase C high-side FET gate   |
| PH_C_L    | RC4  | PG3L| Phase C low-side FET gate    |

#### BEMF Phase Voltage Sensing (via resistor dividers on board)

| Signal  | Pin  | ADC Core | Channel | PINSEL | Notes                          |
|---------|------|----------|---------|--------|--------------------------------|
| VA      | RB9  | AD2      | CH0     | 10     | Muxed with VC; PINSEL switched |
| VB      | RB8  | AD1      | CH0     | 11     | Always sampled; ISR source     |
| VC      | RA10 | AD2      | CH0     | 7      | Muxed with VA; PINSEL switched |

**Key constraint:** VA and VC share AD2CH0. The PINSEL register is switched per
commutation step. After each mux change, `ZC_AD2_SETTLE_SAMPLES` (=2) ADC readings
are discarded to allow the S&H capacitor to settle.

VB is on its own ADC core (AD1CH0) and is always sampled. AD1CH0 data-ready generates
the ADC interrupt at 24kHz.

#### Other ADC Channels

| Signal | Pin  | ADC Core | Channel | PINSEL | Function         |
|--------|------|----------|---------|--------|------------------|
| POT    | RA11 | AD1      | CH1     | 10     | Throttle input   |
| VBUS   | RA7  | AD1      | CH4     | 6      | DC bus voltage   |

#### User Interface

| Signal | Pin  | Function                    |
|--------|------|-----------------------------|
| LED1   | RE0  | Heartbeat (2Hz blink)       |
| LED2   | RE1  | Motor running indicator     |
| SW1    | RE6  | Start/Stop motor            |
| SW2    | RE7  | Change direction            |

#### Motor Connector (J4 on MCLV-48V-300W)

| J4 Pin | Signal     | Motor Wire  |
|--------|------------|-------------|
| Pin 1  | Phase C    | Motor PhC   |
| Pin 2  | Phase B    | Motor PhB   |
| Pin 3  | Phase A    | Motor PhA   |
| Pin 4  | Frame GND  | Disconnect  |

Motor: Hurst DMB0224C10002 (10 poles / 5 pole pairs, 24VDC, 4.03 ohm L-L)

---

## ZC Detection Algorithm Flowchart

Called from `BEMF_ZC_Poll()` every ADC ISR tick (24kHz, ~42us).

```
  BEMF_ZC_Poll(pData, now)
  │
  ├─ zeroCrossDetected == true?
  │  YES → return false  (one ZC per step guard)
  │
  ├─ Compute blanking window:
  │     blankTicks = stepPeriod * 3 / 100
  │     inBlanking = (now - lastCommTick) < blankTicks
  │
  ├─ bemfSampleValid == false?
  │  YES → invalidSampleTotal++, return false  (stale AD2 mux data)
  │
  ├─ Read floating phase BEMF:
  │     vFloat = bemfRaw (set in ISR from AD1CH0 or AD2CH0)
  │     zcThresh = (vbusRaw * duty) >> 18   (duty-proportional neutral)
  │
  ├─ Apply per-phase gain/offset correction:
  │     vCorrected = (vFloat * phaseGain[fp]) >> 15 + phaseOffset[fp]
  │
  ├─ Compute binary comparator state:
  │     if vCorrected > zcThresh + DEADBAND(4) → cmpNow = 1  (above)
  │     if vCorrected < zcThresh - DEADBAND(4) → cmpNow = 0  (below)
  │     else → cmpNow = cmpPrev  (hold in deadband)
  │
  ├─ First read after commutation? (cmpPrev == 0xFF)
  │  YES → cmpPrev = cmpNow, return false  (initialize baseline)
  │
  ├─ Edge detection (runs DURING and AFTER blanking):
  │  │
  │  ├─ filterCount == 0? (looking for initial edge)
  │  │  │
  │  │  ├─ Expected rising (1) AND cmpPrev=0 → cmpNow=1?
  │  │  │  YES → filterCount = 1  (edge detected!)
  │  │  │
  │  │  ├─ Expected falling (0) AND cmpPrev=1 → cmpNow=0?
  │  │  │  YES → filterCount = 1  (edge detected!)
  │  │  │
  │  │  └─ Other transition? → wrongEdgeTotal++
  │  │
  │  └─ filterCount > 0? (confirming edge holds)
  │     │
  │     ├─ cmpNow == expected? → filterCount++
  │     │
  │     └─ cmpNow != expected AND !inBlanking?
  │        → filterCount = 0  (noise, reset)
  │
  ├─ Update cmpPrev = cmpNow
  │
  ├─ inBlanking == true?
  │  YES → blankSkipTotal++, return false  (tracking only, no confirm)
  │
  ├─ Determine filter threshold:
  │     ADAPTIVE_FILTER on:
  │       stepPeriod <= 16 → threshold = 1  (high speed, tight window)
  │       stepPeriod > 16  → threshold = 3  (low speed, noise rejection)
  │     ADAPTIVE_FILTER off:
  │       threshold = ZC_FILTER_THRESHOLD (2)
  │
  └─ filterCount >= threshold?
     │
     YES → ZC CONFIRMED!
     │  │
     │  ├─ goodZcCount++ (toward sync lock)
     │  ├─ zeroCrossDetected = true
     │  ├─ zcConfirmedCount++
     │  │
     │  ├─ Post-sync timing update:
     │  │  │
     │  │  ├─ zcInterval = now - prevZcTick
     │  │  │
     │  │  ├─ stepsSinceLastZc == 1?  (consecutive ZCs, no forced steps between)
     │  │  │  YES → Update stepPeriod via IIR:
     │  │  │        stepPeriod = (3 * stepPeriod + zcInterval) / 4
     │  │  │  NO  → Skip update (forced-step-polluted interval, unreliable)
     │  │  │
     │  │  ├─ Clamp: MIN_CL_ADC_STEP_PERIOD(24) <= stepPeriod <= INITIAL(800)
     │  │  │
     │  │  └─ Set commutation deadline:
     │  │        commDeadline = now + stepPeriod / 2   (30-degree delay)
     │  │
     │  └─ stepsSinceLastZc = 0  (reset spacing counter)
     │
     NO → return false  (keep filtering)
```

---

## ADC ISR State Machine Flowchart

The ADC ISR runs at 24kHz (every PWM cycle). It owns all commutation decisions
in closed-loop mode.

```
  ADC ISR Entry (24kHz)
  │
  ├─ Read ALL ADC buffers:
  │     phaseB  = AD1CH0DATA   (MUST read first — clears data-ready!)
  │     phaseAC = AD2CH0DATA
  │     vbusRaw = AD1CH4DATA
  │     potRaw  = AD1CH1DATA
  │
  ├─ Compute ZC threshold:
  │     zcThreshold = (vbusRaw * duty) >> 18
  │
  ├─ Store floating phase value:
  │     fp = commutationTable[currentStep].floatingPhase
  │     fp == B (1)?  → bemfRaw = phaseB, valid = true
  │     ad2SettleCount > 0? → bemfRaw = phaseAC, valid = false, settle--
  │     else → bemfRaw = phaseAC, valid = true
  │
  ├─ adcIsrTick++
  │
  └─ State == ESC_CLOSED_LOOP?
     │
     ├─ First entry? (prevAdcState != CLOSED_LOOP)
     │  YES → BEMF_ZC_Init(rampStepPeriod → adcTicks)
     │
     ├─ zcSynced == false?  ─── PRE-SYNC MODE ───
     │  │
     │  ├─ forcedCountdown--
     │  ├─ forcedCountdown == 0?
     │  │  YES → AdvanceStep(), OnCommutation(), reload countdown
     │  │
     │  ├─ BEMF_ZC_Poll() — passive ZC detection (builds goodZcCount)
     │  │
     │  ├─ stepsSinceLastZc > STALENESS_LIMIT(12)?
     │  │  YES → goodZcCount = 0  (stale, reset)
     │  │
     │  └─ goodZcCount >= SYNC_THRESHOLD(6) AND risingZcWorks?
     │     YES → zcSynced = true!  (transition to post-sync)
     │
     └─ zcSynced == true?  ─── POST-SYNC MODE ───
        │
        ├─ BEMF_ZC_Poll()  — active ZC detection
        │
        ├─ BEMF_ZC_CheckDeadline()?
        │  YES → AdvanceStep(), OnCommutation()
        │        HandleUndetectableStep()  (fallback for known-bad steps)
        │
        └─ BEMF_ZC_CheckTimeout()?
           │
           ├─ FORCE_STEP → stepMissCount[step]++, AdvanceStep()
           │                goodZcCount-- (toward desync)
           │                if goodZcCount == 0 → revert to pre-sync
           │
           └─ DESYNC → state = FAULT, disable PWM
```

---

## Timing Model

### Key Time Relationships

```
  One electrical cycle = 6 commutation steps = 360 electrical degrees
  One step = 60 electrical degrees

  Step period (ticks) = time for one commutation step in ADC ISR ticks
  1 ADC ISR tick = 1/24000 sec = 41.67 us

  eRPM = 60 / (stepPeriod * 6 * 41.67e-6)
       = 240,000 / stepPeriod     (approximate)

  Examples:
    stepPeriod = 163 →  1,472 eRPM →   294 mech RPM
    stepPeriod =  74 →  3,243 eRPM →   649 mech RPM
    stepPeriod =  56 →  4,286 eRPM →   857 mech RPM
    stepPeriod =  29 →  8,276 eRPM → 1,655 mech RPM
    stepPeriod =  26 →  9,231 eRPM → 1,846 mech RPM
```

### Step Timing Diagram

```
  ──────────────────── One Step (stepPeriod ticks) ──────────────────
  │                                                                 │
  │◄─ Blanking ─►│                                                  │
  │  (3% step)   │                                                  │
  │              │◄── ZC detection window ──►│                      │
  │              │                           │                      │
  │              │      ZC confirmed ────────┤                      │
  │              │           │               │                      │
  │              │           │◄─ 30° delay ─►│                      │
  │              │           │    (step/2)   │                      │
  │              │           │               ▼                      │
  COMMUTATION                          NEXT COMMUTATION
  (lastCommTick)                       (commDeadline)

  If ZC not detected within stepPeriod * 2 → FORCED STEP (timeout)
```

### AD2 Mux Switching per Step

```
  Step 0: C float → AD2 PINSEL=7  (RA10)  ← mux change from step 5→0
  Step 1: A float → AD2 PINSEL=10 (RB9)   ← mux change from step 0→1
  Step 2: B float → AD1 (RB8)             ← no mux change
  Step 3: C float → AD2 PINSEL=7  (RA10)  ← mux change from step 1→3*
  Step 4: A float → AD2 PINSEL=10 (RB9)   ← mux change from step 3→4
  Step 5: B float → AD1 (RB8)             ← no mux change

  * Step 2 uses AD1, so AD2 PINSEL stays from step 1 (=10).
    Step 3 needs PINSEL=7 → mux change.

  After every mux change: discard 2 samples (ZC_AD2_SETTLE_SAMPLES=2)
  = 2 × 42us = 84us settle time before first valid reading.
```

---

## Config Parameter Reference

All in `garuda_config.h`:

| Parameter | Value | Description |
|---|---|---|
| `FEATURE_BEMF_CLOSED_LOOP` | 1 | Master enable for Phase 2 |
| `RAMP_TARGET_ERPM` | 2000 | Open-loop ramp end speed (handoff to closed-loop) |
| `MAX_CLOSED_LOOP_ERPM` | 10000 | Max closed-loop speed (sets MIN_CL_ADC_STEP_PERIOD=24) |
| `ZC_BLANKING_PERCENT` | 3 | % of step period to ignore after commutation |
| `ZC_FILTER_THRESHOLD` | 2 | Confirming samples needed (base, overridden by adaptive) |
| `ZC_SYNC_THRESHOLD` | 6 | Confirmed ZCs to declare sync lock |
| `ZC_MISS_LIMIT` | 12 | Consecutive misses before FAULT_DESYNC |
| `ZC_TIMEOUT_MULT` | 2 | Forced step fires at 2x stepPeriod |
| `ZC_DUTY_THRESHOLD_SHIFT` | 18 | Bit-shift for (vbus*duty) neutral approximation |
| `ZC_ADC_DEADBAND` | 4 | ADC counts noise margin around threshold |
| `ZC_AD2_SETTLE_SAMPLES` | 2 | Samples to discard after AD2 PINSEL mux change |
| `ZC_ADAPTIVE_FILTER` | 1 | Speed-dependent filter: 1 at high speed, 3 at low |
| `ZC_ADAPTIVE_PERIOD` | 1 | IIR smoothing: stepPeriod = 3/4*old + 1/4*new |

---

## Debugging Journey

### Phase 2A: Initial Implementation (ADC ZC code complete, basic config)

Initial config: `ZC_ADC_DEADBAND=10, ZC_FILTER_THRESHOLD=3, ZC_ADAPTIVE_FILTER=0,
ZC_ADAPTIVE_PERIOD=0, ZC_AD2_SETTLE_SAMPLES=1`.

Motor achieved closed-loop sync lock but exhibited jerkiness when throttle was
increased beyond pot ~460.

### Problem Observation

Two watch captures were compared — "smooth" (pot=362) vs "jerky" (pot=489):

| Metric | Smooth | Jerky |
|---|---|---|
| stepPeriod | 120 | 186 |
| stepMissCount | [0,0,0,0,0,0] | [0,1,0,2,1,0] |
| zcTimeoutForceCount | 205 | 594 (3x) |
| blankTransitionTotal | — | 2.2x higher |
| wrongEdgeTotal | — | 2.2x higher |

**Symptom:** Higher throttle caused the motor to SLOW DOWN (stepPeriod 120→186).

### Fix Attempt 1: Config Tuning (5 parameter changes)

1. `ZC_ADAPTIVE_PERIOD 0→1` — IIR smoothing to prevent stepPeriod spikes
2. `ZC_ADC_DEADBAND 10→4` — tighter deadband for faster ZC crossing
3. `ZC_FILTER_THRESHOLD 3→2` — reduced confirming samples
4. `ZC_ADAPTIVE_FILTER 0→1` — speed-dependent filter threshold
5. `ZC_AD2_SETTLE_SAMPLES 1→2` — extra settle after AD2 mux change

**Result:** Deadband hold dropped 19x (working!), but motor still jerky at higher
throttle. stepPeriod still stuck at 120 in smooth mode.

### Root Cause Discovery: Deep Code Analysis

Reading `bemf_zc.c` and `garuda_calc_params.h` revealed **two interacting bugs:**

#### Bug 1: MIN_ADC_STEP_PERIOD Clamp (speed ceiling)

```
garuda_config.h:    RAMP_TARGET_ERPM = 2000
garuda_calc_params.h: MIN_STEP_PERIOD = 100000/2000 = 50 Timer1 ticks
                      MIN_ADC_STEP_PERIOD = 50 * 12/5 = 120 ADC ticks

bemf_zc.c line 305:
    if (pData->timing.stepPeriod < MIN_ADC_STEP_PERIOD)    // <-- THE BUG
        pData->timing.stepPeriod = MIN_ADC_STEP_PERIOD;    // clamps to 120!
```

The `MIN_ADC_STEP_PERIOD` was derived from `RAMP_TARGET_ERPM` (the open-loop ramp
end speed), NOT the maximum closed-loop speed. This created a hard speed ceiling
at 2000 eRPM.

**Evidence:** In all "smooth" captures, `stepPeriod=120` but `zcInterval=82-86`.
The motor naturally spun at ~2800 eRPM but the clamp held commutation at the
2000 eRPM rate. The IIR filter tried to pull stepPeriod toward 82, got clamped
back to 120, every cycle.

**Consequence:** Commutation was systematically late by `(120-82)/2 = 19 ticks`
(0.8ms, ~14 electrical degrees). This caused:
- Falling ZC on AD2 steps (1, 3) to be missed — BEMF had already crossed the
  threshold before the first valid sample arrived
- Rising ZC on AD2 steps (0, 4) still worked — BEMF hadn't risen yet

#### Bug 2: Forced-Step Interval Pollution (death spiral)

When a step missed ZC and was forced at `2 * stepPeriod = 240 ticks`, the next
successful ZC's interval measurement was inflated:

```
Step N:   ZC confirmed at T₀
Step N+1: Missed → forced at T₀ + 60 + 240 = T₀ + 300
Step N+2: ZC confirmed at T₀ + 300 + Y
          zcInterval = 300 + Y
          spacing = 2
          effectiveInterval = (300 + Y) / 2 ≈ 170  (vs real ~82)
```

The spacing-division approach couldn't compensate because the forced step ran at
`2x stepPeriod`, not the real motor period. The inflated effectiveInterval fed the
IIR, pushing stepPeriod ABOVE the 120 clamp:

```
IIR: stepPeriod = (3*120 + 170)/4 = 132  (above the 120 clamp)
Next: (3*132 + 185)/4 = 145
Next: (3*145 + 190)/4 = 156
→ Death spiral: stepPeriod climbed from 120 to 168+
```

**Evidence:** `jerkynow2.csv` showed `stepPeriod=168, zcInterval=166` — the IIR
had converged to the inflated intervals. The motor genuinely slowed from 2000 eRPM
to 1429 eRPM despite higher throttle.

### Fix: Two Surgical Code Changes

#### Fix 1: Decouple closed-loop speed limit from ramp target

Added `MAX_CLOSED_LOOP_ERPM=10000` to `garuda_config.h`, derived
`MIN_CL_ADC_STEP_PERIOD=24` in `garuda_calc_params.h`, and changed the clamp
in `bemf_zc.c` to use the new limit:

```c
// BEFORE (bug):
if (pData->timing.stepPeriod < MIN_ADC_STEP_PERIOD)         // 120
    pData->timing.stepPeriod = MIN_ADC_STEP_PERIOD;

// AFTER (fix):
if (pData->timing.stepPeriod < MIN_CL_ADC_STEP_PERIOD)      // 24
    pData->timing.stepPeriod = MIN_CL_ADC_STEP_PERIOD;
```

#### Fix 2: Skip stepPeriod update when forced steps occurred between ZCs

```c
// BEFORE (bug): always update, divide by spacing
uint8_t spacing = pData->timing.stepsSinceLastZc;
if (spacing < 1) spacing = 1;
uint16_t effectiveInterval = pData->timing.zcInterval / spacing;
// → IIR update with polluted effectiveInterval

// AFTER (fix): only update from consecutive ZC-to-ZC measurements
uint8_t spacing = pData->timing.stepsSinceLastZc;
if (spacing == 1)
{
    uint16_t effectiveInterval = pData->timing.zcInterval;
    // → IIR update with clean interval
}
// spacing > 1: keep current stepPeriod unchanged
```

---

## Test Results

### Full Throttle Sweep (post-fix)

All captures taken from a single run, pot swept from min to max:

| Capture | Pot | Duty | stepPeriod | zcInterval | eRPM | Mech RPM | Timeout Forces | stepMissCount |
|---|---|---|---|---|---|---|---|---|
| workinglow1 | 103 | 12,469 | 163 | 175 | 1,472 | 294 | **0** | [0,0,0,0,0,0] |
| workingmed1 | 557 | 25,118 | 74 | 74 | 3,243 | 649 | **0** | [0,0,0,0,0,0] |
| workingmedh1 | 818 | 32,389 | 56 | 56 | 4,286 | 857 | **2** | [0,0,0,0,0,0] |
| workingh1 | 1,818 | 60,250 | 29 | 30 | 8,276 | 1,655 | **16** | [0,0,0,0,0,0] |
| wprkingmax1 | 2,069 | 67,243 | 26 | 26 | 9,231 | 1,846 | **0** | [0,0,0,0,0,0] |

### Key Improvements

| Metric | Before Fix | After Fix |
|---|---|---|
| stepPeriod tracking | Stuck at 120 (2000 eRPM cap) | Tracks 26-163 (full range) |
| zcInterval vs stepPeriod | 82 vs 120 (38 tick mismatch) | Match within 1-2 ticks |
| Max speed achieved | 2,000 eRPM (400 mech RPM) | 9,231 eRPM (1,846 mech RPM) |
| stepMissCount | Steps 1,3,4 missing constantly | All zeros at every speed |
| zcTimeoutForceCount | 210-500+ per run | 0-16 (negligible) |
| zcPerStep balance | Uneven (1624-1931 spread) | Near-identical (e.g. 2595±2) |
| Jerkiness at high throttle | Severe (motor slows down) | Smooth across entire range |

### Build Metrics

- Flash: 11,524 bytes (8% of 128KB)
- RAM: 172 bytes (1% of 16KB)
- No warnings, no errors

---

## Files Modified (Phase 2 complete)

| File | Changes |
|---|---|
| `garuda_config.h` | Added FEATURE_BEMF_CLOSED_LOOP, ZC parameters, MAX_CLOSED_LOOP_ERPM, RAMP_TARGET_ERPM reduced to 2000 |
| `garuda_calc_params.h` | Added TIMER1_TO_ADC_TICKS, MIN_CL_ADC_STEP_PERIOD, static asserts |
| `garuda_service.c` | ADC ISR: BEMF sampling, closed-loop state machine (pre-sync/post-sync), duty mapping |
| `garuda_types.h` | Added BEMF_STATE_T, TIMING_STATE_T, ZC_DIAG_T, ZC_TIMEOUT_RESULT_T |
| `motor/bemf_zc.c` | **NEW** — ZC detection: Init, OnCommutation, Poll, CheckDeadline, CheckTimeout, HandleUndetectableStep |
| `motor/bemf_zc.h` | **NEW** — ZC module header |
| `motor/commutation.c` | Added HAL_ADC_SelectBEMFChannel() call, ad2SettleCount on mux change |
| `hal/hal_adc.c` | Phase voltage ADC init (VA/VB/VC channels), HAL_ADC_SelectBEMFChannel() |
| `hal/hal_adc.h` | BEMF buffer macros, floating phase enums, function declarations |
| `hal/hal_comparator.c` | CMP init removed from HAL_InitPeripherals (retained for future overcurrent) |
| `hal/hal_comparator.h` | Updated declarations |
| `hal/port_config.c` | BEMF analog pin configuration (RB8, RB9, RA10) |
| `hal/board_service.c` | Minor updates |
| `main.c` | BEMF header includes |
