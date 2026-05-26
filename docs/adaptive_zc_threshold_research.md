# Adaptive ZC Threshold Algorithm — Research & Design

## 1. What Every Major ESC Does (Comparative Analysis)

### AM32 (STM32 ARM, drone ESC)
- **Method**: Hardware analog comparator, floating phase vs virtual neutral (resistor network)
- **Threshold**: Fixed hardware — NO software-computed threshold
- **Filtering**: Speed-adaptive debounce (1-27 consecutive reads)
- **Blanking**: TIM1_OC5 hardware blanking (100 ticks startup, 5 ticks running)
- **Key insight**: No duty-proportional computation at all

### Bluejay (EFM8BB51, drone ESC, BLHeli_S successor)
- **Method**: Hardware comparator, phase BEMF vs V_Mux (virtual neutral tap)
- **Threshold**: Fixed hardware resistor divider — NO software threshold
- **Filtering**: Speed-adaptive (1-27 reads), startup very strict
- **Demag**: IIR metric (7/8 old + 1/8 new), triggers timing advance adjustment
- **Key insight**: Demag metric drives commutation advance, not threshold adjustment

### BLHeli (Silicon Labs, legacy)
- **Method**: Same as Bluejay — hardware comparator against virtual neutral
- **Threshold**: Fixed — no hysteresis, 100ns response time
- **Filtering**: Same adaptive scheme (1-27 reads)
- **Key insight**: Time-based timeout (4× comm_period) for demag detection

### VESC (STM32F4, general BLDC)
- **Method**: Software BEMF integration (flux linkage accumulation)
- **Threshold**: ADAPTIVE — `limit = base + Vin × coupling_k / RPM`
- **Virtual neutral**: Fixed ADC midpoint when running (duty>0.2), measured 3-phase average at startup
- **Key insight**: Integration makes threshold speed-independent; coupling_k handles Vin variation

### Sapog (STM32, PX4 drone ESC)
- **Method**: Least-squares linear regression on BEMF samples
- **Threshold**: None — fits a line to N samples, solves y=0 for exact ZC timestamp
- **Virtual neutral**: Measured — `(V_positive + V_negative) / 2` per-step
- **Key insight**: Statistical ZC estimation, no threshold at all

### ESCape32 (STM32, drone ESC)
- **Method**: 100% hardware — comparator output → timer input capture
- **Threshold**: Fixed voltage divider taps — NO software threshold
- **Validation**: Only reject intervals < 50% of expected, IIR filter on interval
- **Key insight**: Simplest design — hardware does detection, firmware only validates timing

---

## 2. The Core Problem with Our Current Approach

Our current formula:
```c
rawThresh = (vbusRaw * duty) >> 18
```

**Problems identified:**

| Issue | Magnitude | Impact |
|-------|-----------|--------|
| `>> 18` approximation (2^18=262144, actual 2×LOOPTIME=266240) | +1.56% systematic | ~9 ADC counts at 12V/50% duty |
| No per-phase calibration (gain=1.0, offset=0) | Board-dependent, typ ±15 counts | Asymmetric ZC across phases |
| Asymmetric IIR (fast rise τ=0.33ms, slow fall τ=10.7ms) | Lag during decel | Late ZC → rough running |
| Duty-proportional model assumes ideal PWM | Ignores body diode, demag, winding resistance | Wrong at low duty + high current |

**Critical observation**: No production ESC uses a software duty-proportional threshold. They either:
1. Use hardware comparator against a physical/resistor virtual neutral
2. Use BEMF integration (threshold-independent)
3. Use regression (threshold-independent)

We're computing the virtual neutral mathematically, which introduces errors that hardware measurement avoids.

---

## 3. Proposed Adaptive ZC Threshold Algorithm

### Architecture: Hybrid Measured Neutral + Per-Phase Calibration + Integration Validation

```
┌─────────────────────────────────────────────┐
│  Layer 1: MEASURED Virtual Neutral          │
│  V_neutral = (V_active_high + V_active_low) / 2  │
│  Updated every ADC ISR from active phases    │
└──────────────────────┬──────────────────────┘
                       │
┌──────────────────────▼──────────────────────┐
│  Layer 2: Per-Phase Offset Correction       │
│  threshold[phase] = V_neutral + offset[phase]│
│  Auto-calibrated during alignment phase     │
└──────────────────────┬──────────────────────┘
                       │
┌──────────────────────▼──────────────────────┐
│  Layer 3: Duty-Aware Correction             │
│  At very low duty (<15%): blend toward Vbus/2│
│  At normal duty: use measured neutral       │
└──────────────────────┬──────────────────────┘
                       │
┌──────────────────────▼──────────────────────┐
│  Layer 4: Integration Cross-Check           │
│  Existing BEMF_INTEG shadow observer scores │
│  accuracy → drives gain adaptation          │
└─────────────────────────────────────────────┘
```

### Layer 1: Measured Virtual Neutral (from VESC/Sapog)

**Instead of computing** `Vbus × duty / 2`, **measure it directly** from the two active phases:

```c
// In ADC ISR, after reading phase voltages:
// Active phases are the two being driven (high-side + low-side)
uint16_t vHigh = phaseADC[activeHighPhase];  // Phase connected to Vbus
uint16_t vLow  = phaseADC[activeLowPhase];   // Phase connected to GND
uint16_t vNeutralMeasured = (vHigh + vLow) >> 1;
```

**Why this works**:
- The voltage at the Y-point of a star-connected motor equals `(V_high + V_low) / 2` when balanced
- This automatically tracks duty cycle, Vbus variations, and winding resistance drops
- Sapog uses exactly this: `neutral_voltage = (V_positive + V_negative) / 2`
- VESC uses 3-phase average at startup: `vzero = (V_L1 + V_L2 + V_L3) / 3`

**IIR smoothing** (symmetric, not asymmetric):
```c
// Symmetric IIR — same tau for rise and fall
// τ = 4 ticks at 24kHz = 0.17ms (fast tracking)
vNeutralSmooth += (vNeutralMeasured - vNeutralSmooth + 2) >> 2;
```

**Fallback**: If active phase ADC values are saturated (rail-to-rail at very high duty), fall back to duty-proportional:
```c
if (vHigh > 4000 || vLow < 96) {
    // Saturated — can't measure neutral reliably
    // Use duty-proportional with corrected divisor
    vNeutralSmooth = (uint16_t)((uint32_t)vbusRaw * duty / (2 * LOOPTIME_TCY));
}
```

### Layer 2: Per-Phase Offset Auto-Calibration

**Problem**: Board-level resistor divider mismatches + op-amp offsets cause per-phase bias.

**Solution**: Measure offsets during the ALIGN phase (motor stationary, one phase energized).

```c
// During ALIGN state: all 3 phase voltages should read the same
// (motor not spinning, driven phase pulls all phases to same point via winding coupling)
// Sample each phase and compute deviation from mean:

void BEMF_CalibratePhaseOffsets(void) {
    // Only during ALIGN, after settle time
    uint16_t vA = readPhaseA();
    uint16_t vB = readPhaseB();
    uint16_t vC = readPhaseC();
    uint16_t mean = (vA + vB + vC) / 3;

    phaseOffset[0] = (int16_t)mean - (int16_t)vA;  // Phase A correction
    phaseOffset[1] = (int16_t)mean - (int16_t)vB;  // Phase B correction
    phaseOffset[2] = (int16_t)mean - (int16_t)vC;  // Phase C correction

    // Sanity check: if any offset > ±50 counts, something's wrong
    for (int i = 0; i < 3; i++) {
        if (abs(phaseOffset[i]) > 50) phaseOffset[i] = 0;
    }
}
```

**When to run**: Once during every motor start (ALIGN → OL_RAMP transition). Store in RAM (not EEPROM — can vary with temperature).

**Apply correction**:
```c
// In existing computeVCorrected():
int32_t vCorrected = (int32_t)vFloat + phaseOffset[floatingPhase];
```

### Layer 3: Duty-Aware Blending

At very low duty cycles, the measured neutral becomes unreliable (ADC values cluster near ground, low SNR). Blend between measured and Vbus/2:

```c
uint16_t computeAdaptiveThreshold(uint16_t vNeutralMeasured, uint16_t vbusRaw, uint16_t dutyPct) {
    if (dutyPct >= 15) {
        // Normal: use measured neutral
        return vNeutralMeasured;
    } else {
        // Low duty: blend toward Vbus/2
        // At 0% duty: 100% Vbus/2, at 15% duty: 100% measured
        uint16_t vbusHalf = vbusRaw >> 1;
        uint16_t alpha = (uint16_t)((uint32_t)dutyPct * 256 / 15);  // 0-256
        return (uint16_t)(((uint32_t)vNeutralMeasured * alpha +
                           (uint32_t)vbusHalf * (256 - alpha)) >> 8);
    }
}
```

### Layer 4: Integration-Driven Gain Adaptation (uses existing shadow observer)

The BEMF integration shadow observer already scores hit/miss/no-fire. Use this feedback to auto-tune:

```c
// Every N commutations (e.g., every 100 samples):
void BEMF_AdaptThresholdGain(void) {
    if (integ.shadowSampleCount < 100) return;

    uint32_t hitRate = integ.shadowHitCount * 100 / integ.shadowSampleCount;
    int16_t  meanErr = integ.shadowErrorSum / (int32_t)integ.shadowSampleCount;

    if (hitRate < 70) {
        // Poor accuracy — widen deadband or adjust threshold bias
        zcDeadbandTrim += 1;  // Increase tolerance
    } else if (hitRate > 95 && abs(meanErr) < 2) {
        // Excellent — can tighten
        if (zcDeadbandTrim > 0) zcDeadbandTrim -= 1;
    }

    // Signed mean error tells us direction of bias:
    // meanErr < 0 → shadow fires early → threshold too low → raise it
    // meanErr > 0 → shadow fires late  → threshold too high → lower it
    if (abs(meanErr) > 3) {
        thresholdBias += (meanErr > 0) ? -1 : +1;
        // Clamp to ±20 ADC counts
        if (thresholdBias > 20) thresholdBias = 20;
        if (thresholdBias < -20) thresholdBias = -20;
    }

    // Reset counters for next window
    BEMF_INTEG_ResetCounters();
}
```

### Combined Threshold Computation (final)

```c
uint16_t BEMF_GetAdaptiveThreshold(uint8_t floatingPhase) {
    // 1. Measured neutral (Layer 1)
    uint16_t neutral = vNeutralSmooth;

    // 2. Duty-aware blend (Layer 3)
    neutral = computeAdaptiveThreshold(neutral, garudaData.vbusRaw, currentDutyPct);

    // 3. Per-phase offset (Layer 2)
    int32_t threshold = (int32_t)neutral + phaseOffset[floatingPhase];

    // 4. Integration-driven bias correction (Layer 4)
    threshold += thresholdBias;

    // Clamp
    if (threshold < 0) threshold = 0;
    if (threshold > 4095) threshold = 4095;

    return (uint16_t)threshold;
}
```

---

## 4. Adaptive Filtering (from AM32/Bluejay)

### Speed-Adaptive Filter Count

Replace fixed `ZC_FILTER_THRESHOLD=2` with speed-dependent:

```c
uint8_t getAdaptiveFilterThreshold(uint16_t stepPeriod) {
    // stepPeriod in ADC ticks (24kHz)
    if (stepPeriod > 200)      return 4;  // <1200 eRPM: strict filtering
    else if (stepPeriod > 50)  return 2;  // 1200-4800 eRPM: moderate
    else if (stepPeriod > 20)  return 1;  // 4800-12000 eRPM: minimal
    else                       return 1;  // >12000 eRPM: minimal
}
```

### Demag-Aware Timing Advance (from Bluejay)

Track demagnetization events as a smoothed metric:

```c
// IIR demag metric: 7/8 old + 1/8 new (range 0-255)
// "new" = 255 if demag detected this step, 0 otherwise
uint8_t demagMetric;

void updateDemagMetric(bool demagDetectedThisStep) {
    uint16_t acc = (uint16_t)demagMetric * 7;
    if (demagDetectedThisStep) acc += 255;
    demagMetric = (uint8_t)(acc >> 3);
}

// Adjust timing advance based on demag frequency:
// High demag → advance commutation earlier (less time for back-EMF to decay)
uint8_t getDemagTimingBoost(void) {
    if (demagMetric > 160) return 4;  // +4° advance
    if (demagMetric > 130) return 2;  // +2° advance
    return 0;
}
```

---

## 5. HWZC (Digital Comparator) Integration

For the dsPIC33AK digital comparator path, apply the same adaptive threshold:

```c
void HWZC_OnCommutation(void) {
    // ... existing blanking setup ...

    // Use adaptive threshold instead of fixed zcThreshold ± deadband
    uint16_t adaptiveThresh = BEMF_GetAdaptiveThreshold(floatingPhase);

    if (risingZc)
        adaptiveThresh += HWZC_CMP_DEADBAND;
    else
        adaptiveThresh = (adaptiveThresh > HWZC_CMP_DEADBAND)
                       ? adaptiveThresh - HWZC_CMP_DEADBAND : 0;

    HAL_ADC_ConfigComparator(core, adaptiveThresh, risingZc);
}
```

---

## 6. Implementation Priority

| Priority | Change | Effort | Expected Impact |
|----------|--------|--------|-----------------|
| **P0** | Fix `>> 18` → exact division | 5 min | Removes 1.56% systematic bias |
| **P1** | Measured virtual neutral (Layer 1) | 1 hour | Eliminates duty-tracking errors |
| **P2** | Per-phase auto-calibration (Layer 2) | 1 hour | Fixes board-level asymmetry |
| **P3** | Symmetric IIR (replace asymmetric) | 15 min | Fixes deceleration lag |
| **P4** | Speed-adaptive filter count | 30 min | Better noise immunity across RPM |
| **P5** | Integration feedback loop (Layer 4) | 2 hours | Self-tuning in the field |
| **P6** | Demag metric + timing boost | 1 hour | Better high-duty-cycle operation |
| **P7** | Low-duty blending (Layer 3) | 30 min | Better startup transition |

### Recommended Order
1. **P0 + P1 + P3** — immediate fix, test on motor
2. **P2** — if still seeing per-phase asymmetry
3. **P4 + P6** — for speed range and demag improvement
4. **P5 + P7** — self-tuning polish

---

## 7. Interpreting Results

| Symptom | Likely Cause | Fix Layer |
|---------|-------------|-----------|
| ZC consistently early on all phases | Threshold too low | P0, P1 |
| ZC consistently late on all phases | Threshold too high | P0, P1 |
| ZC early on Phase A, late on Phase C | Per-phase offset | P2 |
| ZC OK at steady speed, wrong during accel/decel | Asymmetric IIR lag | P3 |
| Rough at low RPM, smooth at high RPM | Filter too loose at low speed | P4 |
| Rough at high duty, smooth at low duty | Demag interference | P6 |
| ZC drifts slowly over time | Temperature-dependent offset | P5 (auto-tune) |

---

## 8. Sources

### Open-Source ESC Firmware (analyzed locally)
- AM32: `AM32/Src/main.c` — comparator + adaptive filter
- VESC: `vesc-bldc/motor/mcpwm.c` — BEMF flux integration + adaptive threshold
- Sapog: `sapog/firmware/src/motor/realtime/motor_rtctl.c` — least-squares ZC regression
- Bluejay: `bluejay/src/Modules/Timing.asm` — demag metric + adaptive reads
- BLHeli: `BLHeli/SiLabs/BLHeli.asm` — comparator + timeout-based demag
- ESCape32: `ESCape32/src/main.c` — hardware-first, IIR interval filter

### Application Notes
- Microchip AN1160: Majority function BEMF filtering (dsPIC)
- Microchip AN1078: Sliding mode observer (FOC, not trapezoidal)
- ST AN1946: Off-time BEMF sampling, neutral point reconstruction
- NXP AN1914: 3-phase BLDC sensorless with back-EMF

### Academic / Web
- "Adaptive threshold correction strategy" for ZC — horizontal + vertical corrections
- "BEMF integration area balance" — speed-independent commutation detection
- "Unbalanced ZCP Compensation" — IEEE Trans Power Electronics
