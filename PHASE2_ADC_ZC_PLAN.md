# Phase 2: ADC-Based BEMF Zero-Crossing Detection — Final Plan

## 1. Context

Phase 1 achieves motor spin using forced (open-loop) 6-step commutation. The motor spins but cannot respond to load changes and will desync under load. Phase 2 adds BEMF zero-crossing detection to synchronize commutation with rotor position.

**What failed**: The original CMP-based ZC plan (comparator polling) was implemented and tested. It produced zero ZC events because the BEMF sensing pins from AN1292 (RA4/RB2/RB5) are connected to current shunt amplifiers on the EV68M17A DIM, not motor phase voltage dividers. The correct phase voltage pins (RB9/RB8/RA10) are not routable to any CMP input. Hardware comparator ZC is physically impossible on this board. See `ADC_COMPARATOR_DEVICE_NOTES.md` for full analysis.

**What this plan does**: Replace hardware CMPSTAT polling with ADC software threshold comparison. Read the floating phase voltage as a 12-bit ADC value and compare against `vbusRaw >> 1` with deadband, gain, and offset correction. All control architecture (pre-sync forced, post-sync ZC-driven, deadline, timeout, desync fault) is preserved unchanged.

**Feature flag**: `FEATURE_BEMF_CLOSED_LOOP` in `garuda_config.h`. Set to 0 for instant rollback to Phase 1 open-loop behavior.

**Feature-off policy**: Setting `FEATURE_BEMF_CLOSED_LOOP=0` produces a working Phase 1 build, but NOT bit-identical to the original Phase 1 binary. Pin remapping and ADC channel changes are unconditional improvements (correct pins for all modes). No ZC runtime code paths are present when the flag is 0 — both `bemf_zc.c` and `bemf_zc.h` are fully wrapped in `#if FEATURE_BEMF_CLOSED_LOOP` / `#endif`.

---

## 2. Phase 1 Reference

### Hardware (verified, motor spinning)
| Peripheral | Configuration |
|---|---|
| **Clock** | 200 MHz system, CLK5=400MHz(PWM), CLK6=100MHz(ADC), CLK7=400MHz(DAC/CMP) |
| **PWM** | 24 kHz center-aligned, 750ns dead time. PG1=PhA(RD2/RD3), PG2=PhB(RD0/RD1), PG3=PhC(RC3/RC4) |
| **ADC** | 12-bit, PWM-triggered. AD1CH0=interrupt source, AD2CH0=muxed, AD1CH1=POT(RA11), AD1CH4=VBUS(RA7) |
| **Timer1** | 100us period, prescaler 1:8, priority 5 |

### Commutation Table
| Step | A | B | C | Float | ZC |
|---|---|---|---|---|---|
| 0 | PWM | LOW | FLOAT | C | +1 |
| 1 | FLOAT | LOW | PWM | A | -1 |
| 2 | LOW | FLOAT | PWM | B | +1 |
| 3 | LOW | PWM | FLOAT | C | -1 |
| 4 | FLOAT | PWM | LOW | A | +1 |
| 5 | PWM | FLOAT | LOW | B | -1 |

### State Machine
```
IDLE --[SW1]--> ALIGN --[500ms]--> OL_RAMP --[300->5000 eRPM]--> CLOSED_LOOP
Any state --[SW1]--> IDLE          FAULT <--[desync/overcurrent]-- CLOSED_LOOP
```

### Working Parameters
- Hurst DMB0224C10002 (10 poles/5 pairs, 24V, 4.03ohm L-L)
- Open-loop ramp: 300→5000 eRPM, 1000 eRPM/s, 40% duty cap
- Build: ~10KB flash (7%), ~112B RAM, zero warnings (XC-DSC v3.30)

---

## 3. Signal Map (locked — DIM Info Sheet DS70005527 Table 1-1)

```
M1_VA = RB9  = AD2AN10 = DIM:009   (Phase A voltage divider)
M1_VB = RB8  = AD1AN11 = DIM:011   (Phase B voltage divider)
M1_VC = RA10 = AD2AN7  = DIM:022   (Phase C voltage divider)
VBUS  = RA7  = AD1AN6  = DIM:039   (DC bus voltage — unchanged)
POT   = RA11 = AD1AN10 = DIM:028   (Speed reference — unchanged)
```

### ADC Channel Assignment
| Channel | Register | Pin | PINSEL | Role |
|---------|----------|-----|--------|------|
| AD1CH0 | AD1CH0DATA | RB8 | **11** | Phase B voltage — interrupt source, always sampled |
| AD2CH0 | AD2CH0DATA | RB9 or RA10 | **10 or 7** | Phase A or C voltage — muxed per step |
| AD1CH1 | AD1CH1DATA | RA11 | 10 | Potentiometer — unchanged |
| AD1CH4 | AD1CH4DATA | RA7 | 6 | Vbus — unchanged |

**Why AD1CH0 = VB?** AD1CH0 is the interrupt source. Reading AD1CH0DATA clears the data-ready condition. Fixing it to VB/RB8 keeps the interrupt stable regardless of which phase is floating.

**AD2CH0 muxing**:
- Floating A → PINSEL=10 (RB9)
- Floating B → no AD2 change (read AD1CH0)
- Floating C → PINSEL=7 (RA10)

---

## 4. Architecture Rules

**Rule 1: Single Commutation Owner — ADC ISR Only**
All calls to `COMMUTATION_AdvanceStep()` happen exclusively in the ADC ISR when in CLOSED_LOOP. Timer1 ISR never commutates in this state.

**Rule 2: Single Timebase — `adcIsrTick`**
One monotonic `uint16_t`, incremented every ADC ISR (24kHz = 41.67us/tick). Wraps every ~2.73s. All timing uses unsigned subtraction for correct wrap handling.

**Rule 3: Atomicity — All ZC State in ADC ISR**
All BEMF_STATE_T and TIMING_STATE_T fields are read/written only from ADC ISR. Timer1 ISR does not access ZC state.

**Rule 4: Clean Handoff**
Pre-sync: forced countdown + passive ZC detection (builds goodZcCount only). At lock: forced disabled, ZC deadline takes over. One source active at a time.

**Rule 5: One Forced Step Per Timeout**
`BEMF_ZC_CheckTimeout()` re-arms by resetting `lastCommTick`. Fires once per period, not every tick.

**Rule 6: State Transition Boundary**
Timer1 ISR only writes `garudaData.state = ESC_CLOSED_LOOP`. ADC ISR detects transition via `prevAdcState` and performs ZC init.

**Rule 7: hasPrevZc Invariant**
Every path that sets `zcSynced=false` must also set `hasPrevZc=false`.

---

## 5. ZC Detection Design

### Threshold Logic (replaces CMPSTAT)
```c
// Per-phase Q15 gain + signed offset correction
vCorrected = ((uint32_t)vFloat * phaseGain[fp]) >> 15 + phaseOffset[fp];
clamp(vCorrected, 0, 4095);

if      (vCorrected > vHalf + ZC_ADC_DEADBAND)  stateNow = 1;
else if (vCorrected < vHalf - ZC_ADC_DEADBAND)  stateNow = 0;
else    stateNow = previous;  // hold in deadband
```
After this, edge detection / filter / deadline logic is identical to the existing `bemf_zc.c`.

### AD2 Mux Stale Sample Handling
After AD2CH0 PINSEL change (A↔C transitions only), the first AD2CH0DATA read may contain stale charge. `HAL_ADC_SelectBEMFChannel()` returns `bool` indicating if the mux changed. The caller (`commutation.c`) sets `pData->bemf.discardNextAd2 = true`. The ADC ISR marks `bemfSampleValid = false` for that cycle. `BEMF_ZC_Poll()` skips the entire poll (returns false) when the sample is invalid — no edge/filter/diagnostic updates occur.

### Invalid Sample Rule
Processing order in `BEMF_ZC_Poll()`: blanking check → validity check → threshold → edge → filter.
When `bemfSampleValid == false` (checked after blanking):
- Increments `invalidSampleTotal` and returns false immediately
- No threshold computation, no edge detection, no filter update, no `cmpPrev` update
- No diagnostic min/max updates, no `zcPollTotal` increment
- Blanking counter (`blankSkipTotal`) still increments normally if in blanking window
- **Diagnostic rule**: `invalidSampleTotal` should be ≤ 2× the number of A↔C step transitions. If much higher, AD2 settling is taking multiple samples — increase SAMC or add explicit settling delay.

### Architecture Diagram
```
ADC ISR (24kHz) — SOLE COMMUTATION OWNER IN CLOSED_LOOP
├── Read ADC buffers (AD1CH0=VB always, AD2CH0=VA/VC muxed)
├── Store floating phase value in bemfRaw (or mark invalid if discardNextAd2)
├── Compute vbusHalf = vbusRaw >> 1
├── adcIsrTick++
├── [ESC_CLOSED_LOOP]:
│   ├── On first entry: BEMF_ZC_Init() + OnCommutation()
│   ├── IF NOT zcSynced (forced mode):
│   │   ├── Decrement forcedCountdown
│   │   ├── On ==0: AdvanceStep + OnCommutation, reload
│   │   │   └── If no ZC detected this step: reset goodZcCount=0
│   │   ├── BEMF_ZC_Poll() passive (goodZcCount++ only)
│   │   └── On goodZcCount >= threshold: zcSynced=true
│   └── IF zcSynced (ZC-driven):
│       ├── BEMF_ZC_Poll(): blanking → validity → gain/offset → deadband → edge → filter → confirm
│       │   └── Confirmed: update stepPeriod, set commDeadline = now + period/2
│       ├── CheckDeadline(): on reached → AdvanceStep + OnCommutation
│       └── CheckTimeout(): NONE / FORCE_STEP / DESYNC
└── Set duty cycle

Timer1 ISR (10kHz) — NO COMMUTATION IN CLOSED_LOOP
├── Heartbeat, board service, system tick
├── ESC_OL_RAMP: forced commutation (unchanged)
│   └── On complete: set state = ESC_CLOSED_LOOP (flag only)
└── ESC_CLOSED_LOOP: NO-OP
```

---

## 6. Binding Rules

These invariants must hold in all code. Violations are bugs.

1. **`discardNextAd2` set only on actual AD2 PINSEL change.**
   Source: return value of `HAL_ADC_SelectBEMFChannel()`, consumed in `COMMUTATION_AdvanceStep()`.
   NOT in `BEMF_ZC_OnCommutation()`. NOT unconditionally per commutation.
   When floating phase is B, AD2 mux doesn't change → flag must not be set.

2. **All ZC code guarded by `#if FEATURE_BEMF_CLOSED_LOOP` only.**
   No `ZC_USE_ADC` macro exists. ADC is the sole ZC method. All ZC-related `#if` guards
   use `FEATURE_BEMF_CLOSED_LOOP` directly. No secondary dispatch macro.

3. **HAL functions never access application-level globals.**
   `hal_adc.c` does NOT include `garuda_service.h` or access `garudaData`.
   HAL returns status; callers update app state.

4. **`BEMF_ZC_Poll()` checks `bemfSampleValid` after blanking, before threshold computation.**
   Flow: blanking check → validity check → gain/offset/deadband → edge/filter.
   Invalid samples: skip threshold, edge detection, filter, diagnostics, min/max. Return false.

---

## 7. File-by-File Changes

### 7.1: `garuda_config.h`
Replace the entire `#if FEATURE_BEMF_CLOSED_LOOP` section with ADC-based ZC params.

### 7.2: `garuda_calc_params.h`
Replace the entire `#if FEATURE_BEMF_CLOSED_LOOP` section with ADC static asserts.

### 7.3: `garuda_types.h`
Replace `BEMF_STATE_T` and `ZC_DIAG_T` with ADC-based fields.

### 7.4: `hal/port_config.c`
Replace shunt pins (RA4/RB2/RB5) with phase voltage pins (RB9/RB8/RA10).

### 7.5: `hal/hal_adc.h`
Rename buffer macros, add stdbool, change SelectBEMFChannel return type.

### 7.6: `hal/hal_adc.c`
Change PINSEL values, increase SAMC, rewrite SelectBEMFChannel to return bool.

### 7.7: `hal/board_service.c`
Remove InitializeCMPs() and HAL_CMP_SetReference() calls.

### 7.8: `motor/commutation.c`
Remove CMP include/call, set discardNextAd2 from HAL return.

### 7.9: `motor/bemf_zc.c`
Replace CMP polling with ADC threshold + gain/offset + valid check + expanded diagnostics.

### 7.10: `garuda_service.c`
Rewrite ADC reads, remove DAC update, store with valid flag.

### 7.11: `hal/hal_comparator.c` and `hal/hal_comparator.h`
No changes. Retained for future overcurrent PCI use.

### 7.12: Build System
No changes needed.

---

## 8. Implementation Order

1. `garuda_config.h` — ADC ZC params
2. `garuda_calc_params.h` — static asserts
3. `garuda_types.h` — BEMF_STATE_T + ZC_DIAG_T changes
4. `hal/port_config.c` — pin swap
5. `hal/hal_adc.h` — macros + return type
6. `hal/hal_adc.c` — PINSEL + SelectBEMFChannel
7. `hal/board_service.c` — remove CMP init calls
8. `motor/commutation.c` — remove CMP call + add discard flag
9. `motor/bemf_zc.c` — ADC threshold logic
10. `garuda_service.c` — rewire ISR
11. Build `FEATURE_BEMF_CLOSED_LOOP=0` — verify Phase 1 works
12. Build `FEATURE_BEMF_CLOSED_LOOP=1` — verify clean compile, flash, test

---

## 9. Verification Plan

See full plan details in the implementation conversation for V1-V10 verification steps.
