# Phase 2: Closed-Loop BEMF Commutation — Implementation Plan

---

## PART A: PHASE 1 SUMMARY (Cross-Verification Reference)

### What Was Built
A complete open-loop 6-step trapezoidal BLDC ESC firmware for the dsPIC33AK128MC106 MCU on the MCLV-48V-300W development board, targeting the Hurst DMB0224C10002 motor (10 poles / 5 pole pairs, 24V, 4.03 ohm L-L, 4.60 mH L-L).

### Hardware Configuration
| Peripheral | Configuration | Key Registers |
|---|---|---|
| **Clock** | 200 MHz system (FRC 8MHz -> PLL1 M=100, N1=1, N2=4, N3=1) | CLK5=400MHz(PWM), CLK6=100MHz(ADC), CLK7=400MHz(DAC/CMP) |
| **PWM** | 24 kHz center-aligned, 750ns dead time, 3 generators | PG1=master(PhA,RD2/RD3), PG2=slave(PhB,RD0/RD1), PG3=slave(PhC,RC3/RC4) |
| **ADC** | 12-bit, PWM-triggered at 24kHz | AD1CH0=BEMF_A(RA4), AD2CH0=BEMF_B/C(RB2/RB5 muxed), AD1CH1=POT(RA11), AD1CH4=VBUS(RA7) |
| **Comparators** | CMP1/2/3 with internal DAC, 15mV hysteresis | CMP1B=RA4, CMP2B=RB2, CMP3B=RB5, DAC ref = Vbus/2 (initialized, not used yet) |
| **Timer1** | 100us period, prescaler 1:8, PR1=1249 | 100MHz/8 = 12.5MHz tick rate, priority 5 |
| **GPIOs** | LED1=RD5, LED2=RC9, SW1=RD9, SW2=RD10 | UART TX=RC10, RX=RC11, DShot=RD8 |

### Commutation Table (Verified by Manual Step Test — All 6 Steps Produce Shaft Movement)
| Step | Phase A (PG1) | Phase B (PG2) | Phase C (PG3) | Floating | ZC Polarity |
|---|---|---|---|---|---|
| 0 | PWM | LOW | FLOAT | C | Rising (+1) |
| 1 | FLOAT | LOW | PWM | A | Falling (-1) |
| 2 | LOW | FLOAT | PWM | B | Rising (+1) |
| 3 | LOW | PWM | FLOAT | C | Falling (-1) |
| 4 | FLOAT | PWM | LOW | A | Rising (+1) |
| 5 | PWM | FLOAT | LOW | B | Falling (-1) |

### State Machine
```
IDLE --[SW1]--> ALIGN --[500ms]--> OL_RAMP --[300->5000 eRPM]--> CLOSED_LOOP
                                                                      |
Any state --[SW1]--> IDLE          FAULT <--[overcurrent*]-- Any state

* PCI overcurrent fault path is temporarily disabled for bring-up
  (floating RB11 caused false triggers). FAULT state and handler exist
  but are not reachable until PCI is re-enabled with CMP3 as fault source.
```

### Working Parameters (Verified on Real Motor — Spins Both Directions, No Stalls)
- **Alignment**: Step 0, 20% duty, 500ms hold
- **Open-loop ramp**: 300 -> 5000 eRPM, 1000 eRPM/s acceleration, 40% duty cap
- **Step period**: Timer1 ticks (100us each). Initial=333 ticks, Min=20 ticks (at 5000 eRPM)
- **Duty ramp**: +0.5% per commutation step until RAMP_DUTY_CAP (40%)

### Bugs Fixed in Phase 1
1. **ADC ISR starvation**: Must read AD1CH0DATA to clear interrupt; fixed by reading ADCBUF_BEMF_A first
2. **PG2/PG3 not producing PWM**: UPDTRG=0 prevented duty register loading. Fixed: UPDTRG=1, write slaves before master
3. **PCI false faults**: Floating RB11 pin caused continuous triggers. Fixed: disabled PCI fault entirely
4. **Bootstrap brake**: After ChargeBootstrapCapacitors(), overrides must be re-asserted via InitDutyPWM123Generators()

### Build Metrics (at commit 6b46fd7)
- **Program flash**: ~9KB (6% of 128KB) — exact value may vary between builds
- **Data RAM**: 88 bytes (<1% of 16KB)
- Zero errors, zero warnings (XC-DSC v3.30, DFP v1.4.172)

### Known Issues
1. **LED1 not blinking**: Unresolved. Possible Timer1 ISR vector mismatch or LED1 pin assignment error. Note: if Timer1 ISR is not firing, the existing open-loop ramp (which runs from Timer1 ISR) would also not work — but it does work, so Timer1 ISR IS firing. The LED issue is likely hardware (wrong pin or LED polarity). Does NOT block Phase 2.
2. **PCI fault temporarily disabled for bring-up**: Floating RB11 caused false triggers. Needs CMP3 output as fault source (PSS=0b11101). FAULT state handler exists and works; only the trigger path is disconnected
3. **Boot ROM stuck**: `reset` command leaves chip in boot ROM; `run` via debugger works fine

### Source Files (21 compiled)
```
dspic33AKESC/
├── main.c, garuda_service.c/.h, garuda_types.h, garuda_config.h, garuda_calc_params.h
├── hal/  (clock, device_config, hal_pwm, hal_adc, hal_comparator, timer1, port_config, board_service, uart1)
├── motor/  (commutation, startup, pi)
└── learn/  (ring_buffer, quality, health, adaptation, commission + hal/eeprom) [all flags=0]
```

---

## PART B: PHASE 2 IMPLEMENTATION PLAN (REVISED)

### Context
Phase 1 achieves motor spin using forced (open-loop) commutation at fixed timing. The motor spins but cannot respond to load changes, has no speed feedback, and will desync under load. Phase 2 adds BEMF zero-crossing detection to synchronize commutation with rotor position, making the ESC usable for real applications.

### Bring-Up Strategy

**Feature flag**: `FEATURE_BEMF_CLOSED_LOOP` (0=disabled, 1=enabled) in `garuda_config.h`. When disabled, the entire Phase 1 open-loop path is preserved unchanged — Timer1 ISR CLOSED_LOOP forced commutation, no ZC code compiled. This allows instant rollback to known-working behavior during bring-up by setting the flag to 0.

**Phased implementation**:
- **Phase 2A** (implement first): Fixed blanking + fixed filter threshold (`ZC_FILTER_THRESHOLD=3` for all speeds) + deadline commutation + forced fallback. No speed-dependent filtering, no IIR step-period adaptation. Minimal debug surface for first hardware ZC verification.
- **Phase 2B** (add after 2A works): Speed-dependent filter threshold, IIR step-period adaptation, timeout-to-forced fallback with desync fault, DAC reference IIR filtering. These refinements layer on top of working ZC detection.

The plan below describes the complete Phase 2A+2B design. Phase 2B additions are clearly marked with `/* Phase 2B */` comments so they can be initially compiled out or simplified during first bring-up.

### Design Approach: Comparator Polling in ADC ISR (AM32-Style)

**Why comparator polling (not comparator interrupt)?**
- ADC ISR already runs at 24kHz (41.67us period) — sufficient timing resolution for 5000-30000 eRPM range
- Simpler ISR coordination — no need to manage 3 separate CMP ISRs
- AM32 (most popular open-source ESC) uses this exact approach successfully
- Can read BEMF ADC value simultaneously for diagnostics
- Hardware CMP interrupt can be added later as optimization if needed

**Confirmed**: Comparator interrupt vectors exist (`_CMP1Interrupt`/`_CMP2Interrupt`/`_CMP3Interrupt`, indices 76-78 in device ATDF) and are triggered by `DACxCON.IRQM`. These are available for future upgrade but not used in this plan.

---

### Critical Architecture Rules

These address the structural issues identified during plan review:

**Rule 1: Single Commutation Owner — ADC ISR Only**
- ALL calls to `COMMUTATION_AdvanceStep()` happen exclusively in the ADC ISR
- Timer1 ISR NEVER calls `COMMUTATION_AdvanceStep()`
- This eliminates the double-step race condition between ISRs
- Both forced timing (pre-sync) and ZC-driven timing (post-sync) are computed in ADC ISR domain

**Rule 2: Single Timebase — `adcIsrTick` Only**
- One monotonic `uint16_t adcIsrTick` counter, incremented every ADC ISR (24kHz = 41.67us per tick)
- 16-bit wraps every ~2.73 seconds — all deadline/interval math uses unsigned subtraction (handles wrap correctly)
- All timestamps, deadlines, blanking, and timeout values are in adcIsrTick units
- No mixing of Timer1 ticks, systemTick, or TMR1 register values
- Step period conversion: at 5000 eRPM, stepPeriod = 2000us / 41.67us = **48 ticks**. At 30000 eRPM, stepPeriod = 333us / 41.67us = **8 ticks**.

**Rule 3: Atomicity — All ZC State Lives in ADC ISR Context**
- All BEMF_STATE_T and TIMING_STATE_T fields are read/written only from the ADC ISR
- Timer1 ISR does NOT read or write any ZC timing fields
- Timer1 ISR role is unchanged from Phase 1: heartbeat, board service, system tick only
- No cross-ISR 32-bit sharing issues (dsPIC33AK 32-bit reads/writes are not atomic)

**Rule 4: Clean Handoff, Not Parallel Triggering**
- Before ZC lock: forced commutation countdown runs in ADC ISR (converted to adcIsrTick units)
- During forced mode, ZC detection runs passively — increments `goodZcCount` only, does NOT update stepPeriod or set deadlines (prevents noise from destabilizing forced timing)
- ZC detection does NOT trigger commutation until lock is achieved
- At lock (`goodZcCount >= ZC_SYNC_THRESHOLD`): forced timer is disabled, ZC deadline takes over
- One commutation source active at any time — never both

**Rule 5: One Forced Step Per Timeout Event**
- Timeout detection in post-sync mode fires at most once per timeout period
- `BEMF_ZC_CheckTimeout()` returns an enum (NONE / FORCE_STEP / DESYNC), and re-arms internally by resetting `lastCommTick` — prevents forced-step storm
- Timeout basis is always `lastCommTick` (time since last commutation), never `lastZcTick`

**Rule 6: State Transition Boundary**
- Timer1 ISR only writes `garudaData.state` (enum, atomic on dsPIC33AK) for OL_RAMP → CLOSED_LOOP
- ADC ISR detects first CLOSED_LOOP pass via `prevAdcState` tracking and performs ZC init with real `adcIsrTick`
- `prevAdcState` is self-resetting: when state leaves CLOSED_LOOP, it naturally tracks the new state; no extern or manual reset needed

**Rule 7: hasPrevZc Invariant**
- Every code path that sets `zcSynced=false` MUST also set `hasPrevZc=false`
- This prevents stale `prevZcTick` from producing a bogus `zcInterval` on re-sync
- Currently two paths: `BEMF_ZC_Init()` and timeout-triggered sync-loss in ADC ISR

---

### Architecture Overview

```
ADC ISR (24kHz) — SOLE COMMUTATION OWNER
├── adcIsrTick++
├── Read ADC buffers (BEMF, Vbus, Pot)
├── Compute vbusHalf, update DAC ref (throttled: every 8th call, IIR filtered)
├── [ESC_CLOSED_LOOP]:
│   ├── On first entry: BEMF_ZC_Init() + OnCommutation() with real adcIsrTick
│   │
│   ├── IF NOT zcSynced (forced mode):
│   │   ├── Decrement forced step countdown (adcIsrTick units)
│   │   ├── On countdown==0: COMMUTATION_AdvanceStep() + OnCommutation(), reload
│   │   ├── BEMF_ZC_Poll() runs passively (goodZcCount++ only, NO stepPeriod update)
│   │   ├── On forced step with no ZC detected: reset goodZcCount=0 (anti-noise-lock)
│   │   └── On goodZcCount >= threshold: set zcSynced=true
│   │
│   └── IF zcSynced (ZC-driven mode):
│       ├── BEMF_ZC_Poll(): blanking → edge detect (0→1 or 1→0) → filter → confirm
│       │   └── On confirmed ZC: update stepPeriod (IIR), set commDeadline = now + period/2
│       ├── BEMF_ZC_CheckDeadline(): on deadline reached → COMMUTATION_AdvanceStep()
│       └── BEMF_ZC_CheckTimeout(): returns NONE / FORCE_STEP / DESYNC
│           ├── FORCE_STEP: one forced step, re-arm timeout (no storm)
│           └── DESYNC: FAULT_DESYNC
│
└── Set duty cycle

Timer1 ISR (10kHz) — NO COMMUTATION, NO ZC STATE ACCESS
├── Heartbeat LED
├── Board service (button debounce)
├── System tick (msSubCounter → systemTick)
├── ESC_OL_RAMP: forced commutation (unchanged from Phase 1)
│   └── On ramp complete: set state = ESC_CLOSED_LOOP (flag only, no ZC init)
└── ESC_CLOSED_LOOP: NO-OP (all handled in ADC ISR)
```

---

### Step-by-Step Implementation

#### Step 1: Expand Data Structures
**File**: `garuda_types.h`

**Include guard**: Add `#include "garuda_config.h"` at the top of `garuda_types.h` (before any `#if FEATURE_*` guards). This ensures feature flags are visible when `garuda_types.h` is included from any compilation unit, regardless of include order. **Circular include check**: `garuda_config.h` contains only `#define` macros (no `#include` of `garuda_types.h`), so no circular dependency exists. If a future config change introduces a dependency, move feature-gated structs (`ZC_DIAG_T`, `ZC_TIMEOUT_RESULT_T`) to a separate `garuda_zc_types.h` header instead.

```c
typedef struct
{
    uint16_t bemfRaw;           /* Raw BEMF ADC reading (floating phase) */
    uint16_t vbusHalf;          /* Vbus / 2 for comparator reference */
    bool     zeroCrossDetected; /* ZC event detected this step */
    uint8_t  cmpPrev;           /* Previous CMPSTAT reading (0 or 1) */
    uint8_t  cmpExpected;       /* Expected post-ZC state (0 or 1) */
    uint8_t  filterCount;       /* Consecutive reads matching cmpExpected */
} BEMF_STATE_T;

/* ZC timeout result enum */
typedef enum
{
    ZC_TIMEOUT_NONE = 0,        /* No timeout — keep waiting */
    ZC_TIMEOUT_FORCE_STEP,      /* One forced step needed (re-armed internally) */
    ZC_TIMEOUT_DESYNC            /* Too many misses — fault */
} ZC_TIMEOUT_RESULT_T;

typedef struct
{
    uint16_t stepPeriod;            /* Current step period in adcIsrTick units */
    uint16_t lastCommTick;          /* adcIsrTick at last commutation (timeout basis) */
    uint16_t lastZcTick;            /* adcIsrTick at last confirmed ZC */
    uint16_t prevZcTick;            /* adcIsrTick at ZC before lastZcTick (for interval) */
    uint16_t zcInterval;            /* Ticks between last two ZCs */
    uint16_t commDeadline;          /* adcIsrTick when next commutation should fire */
    uint16_t forcedCountdown;       /* Ticks remaining for forced commutation (pre-sync) */
    uint16_t goodZcCount;           /* Consecutive valid ZC events */
    uint8_t  consecutiveMissedSteps;/* Steps where ZC was not detected */
    bool     zcSynced;              /* True = ZC-driven, False = forced */
    bool     deadlineActive;        /* True when commDeadline is valid */
    bool     hasPrevZc;             /* True after first post-sync ZC (guards zcInterval calc) */
} TIMING_STATE_T;

/* Diagnostic counters — temporary, for hardware bring-up debugging.
 * Readable via debugger halt. All uint16_t to avoid atomicity issues.
 * Remove or compile-out once ZC is validated. */
#if FEATURE_BEMF_CLOSED_LOOP
typedef struct
{
    uint16_t zcConfirmedCount;      /* Total confirmed ZC events (wraps at 65535) */
    uint16_t zcTimeoutForceCount;   /* Timeout-triggered forced steps (post-sync) */
    uint16_t zcDesyncCount;         /* FAULT_DESYNC events */
    uint16_t forcedStepPresyncCount;/* Forced steps during pre-sync phase */
} ZC_DIAG_T;
#endif
```

Add `ZC_DIAG_T zcDiag` member to `GARUDA_DATA_T` (inside `#if FEATURE_BEMF_CLOSED_LOOP`).

All fields are 16-bit or smaller — no 32-bit tearing risk on dsPIC33AK. The `adcIsrTick` is `static uint16_t` inside the ADC ISR (not shared with Timer1 ISR). Timeout is computed from `lastCommTick` (not from a separate counter).

#### Step 2: Add ZC Configuration
**File**: `garuda_config.h`

```c
/* Phase 2: BEMF Closed-Loop Control */
#define FEATURE_BEMF_CLOSED_LOOP  1     /* 0=Phase 1 open-loop only, 1=Phase 2 ZC detection */

#if FEATURE_BEMF_CLOSED_LOOP
/* Phase 2A: Core ZC detection */
#define ZC_BLANKING_PERCENT     25      /* Ignore ZC for first 25% of step period after commutation */
#define ZC_FILTER_THRESHOLD     3       /* Phase 2A: fixed filter count for all speeds */
#define ZC_SYNC_THRESHOLD       6       /* Consecutive good ZCs needed to declare lock (one full e-cycle) */
#define ZC_MISS_LIMIT           12      /* Missed steps before FAULT_DESYNC (two full e-cycles) */
#define ZC_TIMEOUT_MULT         2       /* Timeout = ZC_TIMEOUT_MULT * stepPeriod (in adcIsrTick) */
#define ZC_DAC_UPDATE_DIVIDER   8       /* Update DAC ref every 8th ADC ISR call (~3kHz) */

/* Phase 2B: Adaptive refinements (set to 0 to disable during initial bring-up) */
#define ZC_ADAPTIVE_FILTER      0       /* 1=speed-dependent filter, 0=fixed ZC_FILTER_THRESHOLD */
#define ZC_ADAPTIVE_PERIOD      0       /* 1=IIR step-period adaptation, 0=use zcInterval directly */
#define ZC_DAC_IIR_FILTER       0       /* 1=IIR low-pass on DAC ref, 0=direct vbusHalf */

#if ZC_ADAPTIVE_FILTER
#define ZC_FILTER_MIN           1       /* Min consecutive CMPSTAT reads (high RPM) */
#define ZC_FILTER_MAX           3       /* Max consecutive reads (low RPM) */
#define ZC_FILTER_SPEED_THRESH  16      /* stepPeriod threshold for filter switching */
#endif
#endif /* FEATURE_BEMF_CLOSED_LOOP */
```

**File**: `garuda_calc_params.h` — step period conversions + compile-time validation:

```c
/* Convert Phase 1 Timer1-tick step periods to adcIsrTick units.
 * Timer1 tick = 100us. ADC ISR tick = 41.67us.
 * Ratio: 100/41.67 = 2.4. So Timer1_ticks * 2.4 = adcIsrTicks.
 * More precisely: adcIsrTicks = Timer1_ticks * 24000 / 10000 = Timer1_ticks * 12/5 */
#if FEATURE_BEMF_CLOSED_LOOP
#define TIMER1_TO_ADC_TICKS(t1)     (uint16_t)(((uint32_t)(t1) * 12) / 5)

/* Step periods in adcIsrTick units */
#define INITIAL_ADC_STEP_PERIOD     TIMER1_TO_ADC_TICKS(INITIAL_STEP_PERIOD)   /* ~800 ticks at 300 eRPM */
#define MIN_ADC_STEP_PERIOD         TIMER1_TO_ADC_TICKS(MIN_STEP_PERIOD)       /* ~48 ticks at 5000 eRPM */

/* Compile-time config sanity checks — MUST be after all macros are defined */
_Static_assert(MIN_ADC_STEP_PERIOD > 0,       "MIN_ADC_STEP_PERIOD must be > 0");
_Static_assert(ZC_FILTER_THRESHOLD >= 1,       "ZC_FILTER_THRESHOLD must be >= 1");
_Static_assert(ZC_TIMEOUT_MULT >= 1,           "ZC_TIMEOUT_MULT must be >= 1");
_Static_assert(ZC_SYNC_THRESHOLD >= 1,         "ZC_SYNC_THRESHOLD must be >= 1");
_Static_assert(ZC_BLANKING_PERCENT < 100,      "ZC_BLANKING_PERCENT must be < 100");
_Static_assert(ZC_DAC_UPDATE_DIVIDER >= 1,     "ZC_DAC_UPDATE_DIVIDER must be >= 1");
#if ZC_ADAPTIVE_FILTER
_Static_assert(ZC_FILTER_MIN >= 1,             "ZC_FILTER_MIN must be >= 1");
_Static_assert(ZC_FILTER_MAX >= ZC_FILTER_MIN, "ZC_FILTER_MAX must be >= ZC_FILTER_MIN");
#endif
#endif /* FEATURE_BEMF_CLOSED_LOOP */
```

#### Step 3: Add Comparator Read Function
**File**: `hal/hal_comparator.h` — add declaration:
```c
uint8_t HAL_CMP_ReadStatus(uint8_t phase);
```

**File**: `hal/hal_comparator.c` — add implementation:
```c
uint8_t HAL_CMP_ReadStatus(uint8_t phase)
{
    switch (phase)
    {
        case 0: return DAC1CONbits.CMPSTAT;
        case 1: return DAC2CONbits.CMPSTAT;
        case 2: return DAC3CONbits.CMPSTAT;
        default: return 0;
    }
}
```

#### Step 4: Create ZC Detection Module
**New file**: `motor/bemf_zc.h`

```c
#ifndef _BEMF_ZC_H
#define _BEMF_ZC_H

#include "../garuda_types.h"

void                 BEMF_ZC_Init(volatile GARUDA_DATA_T *pData, uint16_t initialStepPeriod);
void                 BEMF_ZC_OnCommutation(volatile GARUDA_DATA_T *pData, uint16_t now);
bool                 BEMF_ZC_Poll(volatile GARUDA_DATA_T *pData, uint16_t now);
bool                 BEMF_ZC_CheckDeadline(volatile GARUDA_DATA_T *pData, uint16_t now);
ZC_TIMEOUT_RESULT_T  BEMF_ZC_CheckTimeout(volatile GARUDA_DATA_T *pData, uint16_t now);

#endif
```

**New file**: `motor/bemf_zc.c`

Five functions, all called exclusively from ADC ISR:

**`BEMF_ZC_Init(pData, initialStepPeriod)`**: Called once when ADC ISR first sees CLOSED_LOOP. The `initialStepPeriod` is `TIMER1_TO_ADC_TICKS(garudaData.rampStepPeriod)` — the actual ramp speed at handoff, not a hardcoded value. Sets `zcSynced=false`, `stepPeriod=initialStepPeriod`, `forcedCountdown=initialStepPeriod`, `goodZcCount=0`, `consecutiveMissedSteps=0`, `deadlineActive=false`, `hasPrevZc=false`. Zeroes all `zcDiag` counters.

**Note**: `FAULT_DESYNC` already exists in `garuda_types.h` (line 279) — no enum change needed.

**`BEMF_ZC_OnCommutation(pData, now)`**: Called immediately after every `COMMUTATION_AdvanceStep()`. Records `lastCommTick=now`. Resets: `zeroCrossDetected=false`, `cmpPrev=0xFF` (unknown), `filterCount=0`, `deadlineActive=false`. Computes `cmpExpected` from commutation table:
- Rising ZC (zcPolarity=+1): after ZC, BEMF > Vbus/2, so `cmpExpected=1`
- Falling ZC (zcPolarity=-1): after ZC, BEMF < Vbus/2, so `cmpExpected=0`

**`BEMF_ZC_Poll(pData, now)`**: Core ZC detection. Returns true if ZC confirmed this call.

**Behavior differs based on `zcSynced`**:
- **Pre-sync** (`zcSynced=false`): Only increments `goodZcCount` (saturate at `ZC_SYNC_THRESHOLD`). Does NOT update `stepPeriod`, set `commDeadline`, or modify any timing state. This prevents noise from destabilizing the forced countdown during handoff.
  - **Decay**: On each forced commutation step where no ZC was detected (`zeroCrossDetected==false` at the time `forcedCountdown` reaches 0), `goodZcCount` is reset to 0. This is the ONLY reset point in pre-sync mode — ensures noise cannot slowly accumulate false ZCs.
  - **Reset rules summary**: `goodZcCount` is modified at exactly 3 points: (a) incremented on confirmed ZC (saturate at threshold), (b) reset to 0 at forced-step boundary when no ZC detected (pre-sync), (c) decremented on timeout (post-sync).
- **Post-sync** (`zcSynced=true`): Full behavior — updates `stepPeriod`, sets `commDeadline`, etc.

Algorithm:
0. **One ZC per step guard**: If `zeroCrossDetected` is already true, return false immediately. Prevents multiple noisy edges in one step from incrementing `goodZcCount` more than once.
1. **Blanking**: `elapsed = (uint16_t)(now - lastCommTick)`. If `elapsed < (stepPeriod * ZC_BLANKING_PERCENT / 100)` → return false
2. **Read comparator**: `cmpNow = HAL_CMP_ReadStatus(floatingPhase)`
3. **First read after blanking**: If `cmpPrev == 0xFF`, just store `cmpPrev = cmpNow`, return false
4. **Edge detection** (require explicit transition, not level-only):
   - Rising (cmpExpected=1): need `cmpPrev==0 && cmpNow==1` to start filter
   - Falling (cmpExpected=0): need `cmpPrev==1 && cmpNow==0` to start filter
   - If transition detected: set `filterCount=1`
   - If `filterCount > 0 && cmpNow == cmpExpected`: increment filterCount
   - If `filterCount > 0 && cmpNow != cmpExpected`: reset filterCount=0 (false alarm)
5. **Filter threshold**: Phase 2A uses fixed `ZC_FILTER_THRESHOLD`. Phase 2B (`ZC_ADAPTIVE_FILTER=1`) uses speed-dependent: `threshold = (stepPeriod <= ZC_FILTER_SPEED_THRESH) ? ZC_FILTER_MIN : ZC_FILTER_MAX`. **Type safety**: use `(uint16_t)` casts when comparing `goodZcCount` (uint16_t) with macro thresholds (int) to avoid signed/unsigned warnings: `goodZcCount >= (uint16_t)ZC_SYNC_THRESHOLD`
6. **On `filterCount >= threshold`** (ZC confirmed):
   - `goodZcCount++` (saturate at `ZC_SYNC_THRESHOLD` — no need to count beyond lock threshold; keeps value small and bounded)
   - `consecutiveMissedSteps = 0`
   - `zeroCrossDetected = true`
   - `pData->zcDiag.zcConfirmedCount++` (diagnostic, wraps at 65535)
   - **If zcSynced** (post-sync only):
     - `prevZcTick = lastZcTick`; `lastZcTick = now`
     - **If `hasPrevZc`** (not the first ZC since sync):
       - `zcInterval = (uint16_t)(now - prevZcTick)`
       - **Phase 2A**: `stepPeriod = zcInterval` (direct assignment — simplest)
       - **Phase 2B** (`ZC_ADAPTIVE_PERIOD=1`): `stepPeriod = (3*stepPeriod + zcInterval) >> 2` (IIR smoothing)
       - Clamp stepPeriod to [MIN_ADC_STEP_PERIOD, INITIAL_ADC_STEP_PERIOD]
     - **Else** (first ZC after sync): set `hasPrevZc = true` (skip update — no valid interval yet, keep inherited stepPeriod)
     - Compute 30° delay: `delay = stepPeriod / 2`
     - Set `commDeadline = (uint16_t)(now + delay)`, `deadlineActive = true`
   - Return true
7. **Update cmpPrev**: `cmpPrev = cmpNow`
8. Return false

**`BEMF_ZC_CheckDeadline(pData, now)`**: Returns true if it's time to commutate.
- If `deadlineActive && (uint16_t)(now - commDeadline) < 0x8000` (i.e., now >= deadline with wrap handling):
  - `deadlineActive = false`
  - Return true
- Return false

**`BEMF_ZC_CheckTimeout(pData, now)`**: Called every ADC ISR in post-sync mode. Returns an enum:
- `ZC_TIMEOUT_NONE` — no timeout, keep waiting
- `ZC_TIMEOUT_FORCE_STEP` — one forced step needed (fired once, then re-armed)
- `ZC_TIMEOUT_DESYNC` — too many misses, fault

Logic:
- If `deadlineActive`: return `ZC_TIMEOUT_NONE` immediately (a ZC was detected and deadline is pending — don't fight it with a forced step)
- `elapsed = (uint16_t)(now - lastCommTick)`
- If `(uint32_t)elapsed > (uint32_t)stepPeriod * ZC_TIMEOUT_MULT` (uint32_t to avoid uint16_t overflow when stepPeriod*MULT > 65535):
  - ZC missed for this step
  - `consecutiveMissedSteps++`
  - If `goodZcCount > 0`: `goodZcCount--`
  - **Re-arm**: Set `lastCommTick = now` (prevents re-firing every tick)
  - If `consecutiveMissedSteps >= ZC_MISS_LIMIT`: return `ZC_TIMEOUT_DESYNC`
  - Else: return `ZC_TIMEOUT_FORCE_STEP`
- Return `ZC_TIMEOUT_NONE`

The key fix is `lastCommTick = now` inside the timeout handler. This re-arms the timeout so it fires once per timeout period, not every ADC ISR tick. The caller then does ONE forced commutation + `BEMF_ZC_OnCommutation()` (which resets `lastCommTick` again properly).

#### Step 5: Modify `COMMUTATION_AdvanceStep()`
**File**: `motor/commutation.c`

Add comparator enable and ZC state reset after step advance:

```c
/* After HAL_PWM_SetCommutationStep() and HAL_ADC_SelectBEMFChannel(): */

#if FEATURE_BEMF_CLOSED_LOOP
/* Enable comparator for new floating phase */
HAL_CMP_EnableFloatingPhase(commutationTable[pData->currentStep].floatingPhase);
#endif
```

Note: The ZC state reset (`cmpPrev=0xFF`, `filterCount=0`, etc.) is done by `BEMF_ZC_OnCommutation()` which the ADC ISR calls immediately after `COMMUTATION_AdvanceStep()`. Keeping the reset in the caller (not inside AdvanceStep) avoids adding ZC-awareness to the commutation module.

#### Step 6: Rewire `garuda_service.c` — The Core Change
**File**: `garuda_service.c`

**ADC ISR changes:**

The entire CLOSED_LOOP ZC logic is wrapped in `#if FEATURE_BEMF_CLOSED_LOOP`. When the flag is 0, the Phase 1 open-loop CLOSED_LOOP path is preserved (Timer1 ISR handles forced commutation, ADC ISR just sets duty). The `adcIsrTick`, `dacUpdateDiv`, `prevAdcState` statics and the DAC update block are also inside the `#if` guard — they are not needed when ZC is disabled.

```c
#if FEATURE_BEMF_CLOSED_LOOP
/* File-scope in garuda_service.c (NOT shared with Timer1 ISR) */
static uint16_t adcIsrTick = 0;
static uint8_t dacUpdateDiv = 0;
static ESC_STATE_T prevAdcState = ESC_IDLE;  /* Detect state transitions inside ADC ISR */
#endif

void __attribute__((__interrupt__, no_auto_psv)) GARUDA_ADC_INTERRUPT(void)
{
    /* Read ADC buffers (MUST read AD1CH0DATA to clear interrupt).
     * bemfRaw always reads AD1CH0 (BEMF_A / RA4). This is the interrupt-clearing
     * read — it does NOT track the active floating phase. For Phase 2, ZC detection
     * uses the hardware comparator (CMPSTAT), not bemfRaw. The ADC value is retained
     * for diagnostics only. A future enhancement could read the floating phase ADC
     * (AD2CH0 for phases B/C) into bemfRaw for scope-style telemetry. */
    garudaData.bemf.bemfRaw = ADCBUF_BEMF_A;
    garudaData.vbusRaw = ADCBUF_VBUS;
    garudaData.potRaw = ADCBUF_POT;

    garudaData.throttle = garudaData.potRaw;
    garudaData.bemf.vbusHalf = garudaData.vbusRaw >> 1;

#if FEATURE_BEMF_CLOSED_LOOP
    /* Throttled DAC reference update (~3kHz instead of 24kHz) */
    dacUpdateDiv++;
    if (dacUpdateDiv >= ZC_DAC_UPDATE_DIVIDER)
    {
        dacUpdateDiv = 0;
#if ZC_DAC_IIR_FILTER  /* Phase 2B: IIR low-pass on DAC reference */
        static uint16_t dacRef = 0;
        if (dacRef == 0)
            dacRef = garudaData.bemf.vbusHalf;
        else
            dacRef = (uint16_t)(((uint32_t)dacRef * 7 + garudaData.bemf.vbusHalf) >> 3);
        HAL_CMP_SetReference(dacRef);
#else  /* Phase 2A: direct vbusHalf */
        HAL_CMP_SetReference(garudaData.bemf.vbusHalf);
#endif
    }

    /* Monotonic timestamp */
    adcIsrTick++;
#endif /* FEATURE_BEMF_CLOSED_LOOP */

    switch (garudaData.state)
    {
        case ESC_IDLE:
        case ESC_ARMED:
        case ESC_ALIGN:
        case ESC_OL_RAMP:
            /* Handled in Timer1 ISR (forced commutation via rampCounter) */
            break;

#if FEATURE_BEMF_CLOSED_LOOP
        case ESC_CLOSED_LOOP:
        {
            /* Detect first entry (state just changed to CLOSED_LOOP) */
            if (prevAdcState != ESC_CLOSED_LOOP)
            {
                uint16_t initPeriod = TIMER1_TO_ADC_TICKS(garudaData.rampStepPeriod);
                if (initPeriod < MIN_ADC_STEP_PERIOD) initPeriod = MIN_ADC_STEP_PERIOD;
                BEMF_ZC_Init(&garudaData, initPeriod);
                BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
            }

            if (!garudaData.timing.zcSynced)
            {
                /* === PRE-SYNC: Forced commutation + passive ZC detection === */

                /* Forced commutation countdown (in adcIsrTick units) */
                if (garudaData.timing.forcedCountdown > 0)
                    garudaData.timing.forcedCountdown--;

                if (garudaData.timing.forcedCountdown == 0)
                {
                    /* Decay: if no ZC was detected during this forced step, reset goodZcCount.
                     * Prevents noise from slowly accumulating false ZCs into a premature lock. */
                    if (!garudaData.bemf.zeroCrossDetected)
                        garudaData.timing.goodZcCount = 0;

                    COMMUTATION_AdvanceStep(&garudaData);
                    BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
                    garudaData.timing.forcedCountdown = garudaData.timing.stepPeriod;
                    garudaData.zcDiag.forcedStepPresyncCount++;
                }

                /* Passive ZC detection (builds goodZcCount but doesn't trigger commutation) */
                BEMF_ZC_Poll(&garudaData, adcIsrTick);

                /* Check for ZC lock */
                if (garudaData.timing.goodZcCount >= ZC_SYNC_THRESHOLD)
                {
                    garudaData.timing.zcSynced = true;
                    /* Clean handoff: discard any pending forced countdown */
                }
            }
            else
            {
                /* === POST-SYNC: ZC-driven commutation === */

                /* Poll for ZC */
                BEMF_ZC_Poll(&garudaData, adcIsrTick);

                /* Check commutation deadline */
                if (BEMF_ZC_CheckDeadline(&garudaData, adcIsrTick))
                {
                    COMMUTATION_AdvanceStep(&garudaData);
                    BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
                }

                /* Timeout watchdog (fires at most once per timeout period) */
                ZC_TIMEOUT_RESULT_T toResult = BEMF_ZC_CheckTimeout(&garudaData, adcIsrTick);
                if (toResult == ZC_TIMEOUT_DESYNC)
                {
                    garudaData.zcDiag.zcDesyncCount++;
                    garudaData.state = ESC_FAULT;
                    garudaData.faultCode = FAULT_DESYNC;
                    HAL_MC1PWMDisableOutputs();
                    LED2 = 0;
                    /* prevAdcState will see ESC_FAULT next pass, auto-detecting re-entry */
                }
                else if (toResult == ZC_TIMEOUT_FORCE_STEP)
                {
                    garudaData.zcDiag.zcTimeoutForceCount++;
                    /* One forced step to keep spinning */
                    COMMUTATION_AdvanceStep(&garudaData);
                    BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);

                    /* If goodZcCount dropped to 0, lose sync → revert to forced */
                    if (garudaData.timing.goodZcCount == 0)
                    {
                        garudaData.timing.zcSynced = false;
                        garudaData.timing.hasPrevZc = false;  /* Must re-establish first ZC on re-sync */
                        garudaData.timing.forcedCountdown = garudaData.timing.stepPeriod;
                    }
                }
            }

            /* Duty cycle: map pot to duty when synced, hold ramp duty when not */
            if (garudaData.timing.zcSynced)
            {
                uint32_t mappedDuty = MIN_DUTY +
                    ((uint32_t)garudaData.potRaw * (MAX_DUTY - MIN_DUTY)) / 4096;
                if (mappedDuty < MIN_DUTY) mappedDuty = MIN_DUTY;
                if (mappedDuty > MAX_DUTY) mappedDuty = MAX_DUTY;
                garudaData.duty = mappedDuty;
            }
            HAL_PWM_SetDutyCycle(garudaData.duty);
            break;
        }
#else  /* !FEATURE_BEMF_CLOSED_LOOP — Phase 1 open-loop path */
        case ESC_CLOSED_LOOP:
            HAL_PWM_SetDutyCycle(garudaData.duty);
            break;
#endif

        case ESC_BRAKING:
            break;

        case ESC_FAULT:
            HAL_MC1PWMDisableOutputs();
            break;
    }

#if FEATURE_BEMF_CLOSED_LOOP
    /* Track state for transition detection (must be last) */
    prevAdcState = garudaData.state;
#endif

    GARUDA_ClearADCIF();
}
```

**Timer1 ISR changes:**

Remove ALL commutation logic from `ESC_CLOSED_LOOP` case. Timer1 ISR only does heartbeat, board service, and system tick. The `ESC_OL_RAMP` forced commutation stays in Timer1 ISR (unchanged from Phase 1).

```c
#if FEATURE_BEMF_CLOSED_LOOP
        case ESC_CLOSED_LOOP:
            /* Phase 2: All commutation handled in ADC ISR.
             * Timer1 ISR does nothing for this state. */
            break;
#else
        case ESC_CLOSED_LOOP:
            /* Phase 1: keep forced commutation at final ramp speed (unchanged). */
            if (garudaData.rampCounter > 0)
                garudaData.rampCounter--;
            if (garudaData.rampCounter == 0)
            {
                COMMUTATION_AdvanceStep(&garudaData);
                garudaData.rampCounter = garudaData.rampStepPeriod;
            }
            break;
#endif
```

#### Step 7: OL_RAMP → CLOSED_LOOP Transition
**File**: `garuda_service.c`

**Rule 3 compliance**: Timer1 ISR ONLY sets `garudaData.state = ESC_CLOSED_LOOP`. It does NOT call any BEMF_ZC functions or touch any ZC state. All ZC initialization happens in the ADC ISR on first detection of `ESC_CLOSED_LOOP`.

**Timer1 ISR** (ESC_OL_RAMP case — minimal change):
```c
case ESC_OL_RAMP:
    if (STARTUP_OpenLoopRamp(&garudaData))
    {
        /* ONLY set the state flag. ADC ISR handles ZC init on first pass. */
        garudaData.state = ESC_CLOSED_LOOP;
    }
    break;
```

**ADC ISR** (ESC_CLOSED_LOOP entry detection):
A `static ESC_STATE_T prevAdcState` (local to ADC ISR) detects state transitions. When `prevAdcState != ESC_CLOSED_LOOP` and current state is `ESC_CLOSED_LOOP`, it's the first pass:

```c
case ESC_CLOSED_LOOP:
{
    if (prevAdcState != ESC_CLOSED_LOOP)
    {
        /* First ADC ISR pass in CLOSED_LOOP — initialize ZC with real adcIsrTick.
         * Use current ramp speed (converted from Timer1 ticks to ADC ticks). */
        uint16_t initPeriod = TIMER1_TO_ADC_TICKS(garudaData.rampStepPeriod);
        if (initPeriod < MIN_ADC_STEP_PERIOD) initPeriod = MIN_ADC_STEP_PERIOD;
        BEMF_ZC_Init(&garudaData, initPeriod);
        BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
    }
    /* ... rest of CLOSED_LOOP logic ... */
}
```

At the END of the ADC ISR (after the switch): `prevAdcState = garudaData.state;`

This is self-resetting — no extern needed. When state leaves CLOSED_LOOP (SW1→IDLE, fault, etc.), `prevAdcState` naturally tracks the new state, and the next entry into CLOSED_LOOP triggers re-init automatically.

This eliminates all Rule 3 violations:
- Timer1 ISR touches only `garudaData.state` (an enum, atomic on dsPIC33AK)
- All ZC state init uses the real `adcIsrTick` value (no seed with `0`)
- Uses actual `rampStepPeriod` at handoff moment (not hardcoded MIN)
- No extern or cross-module flag needed

#### Step 8: Update Build System
**File**: `dspic33AKESC.X/nbproject/Makefile-default.mk`

Add `bemf_zc.c` to the motor source files (same object dir hash as other motor files: `_ext/2109951130`).

**Note on fragility**: `Makefile-default.mk` is generated by MPLAB X IDE. Opening the project in the IDE and saving can overwrite manual edits. **Action**: Update BOTH `Makefile-default.mk` (for CLI builds) AND `nbproject/configurations.xml` (for IDE compatibility) to include `bemf_zc.c`. This ensures the project works regardless of whether builds are run via CLI or MPLAB X GUI.

---

### Files Summary

**Modified:**
| File | Changes |
|---|---|
| `garuda_types.h` | Expand BEMF_STATE_T (add cmpPrev, cmpExpected, filterCount), TIMING_STATE_T (all new fields), add ZC_DIAG_T + ZC_TIMEOUT_RESULT_T, add zcDiag to GARUDA_DATA_T |
| `garuda_config.h` | Add FEATURE_BEMF_CLOSED_LOOP flag, ZC_BLANKING_PERCENT, ZC_FILTER_THRESHOLD, ZC_SYNC_THRESHOLD, ZC_MISS_LIMIT, ZC_TIMEOUT_MULT, ZC_DAC_UPDATE_DIVIDER, Phase 2B sub-flags |
| `garuda_calc_params.h` | Add TIMER1_TO_ADC_TICKS macro, INITIAL_ADC_STEP_PERIOD, MIN_ADC_STEP_PERIOD |
| `hal/hal_comparator.c` | Add HAL_CMP_ReadStatus() function |
| `hal/hal_comparator.h` | Declare HAL_CMP_ReadStatus() |
| `motor/commutation.c` | Add HAL_CMP_EnableFloatingPhase() call in AdvanceStep() |
| `garuda_service.c` | Wrap CLOSED_LOOP in `#if FEATURE_BEMF_CLOSED_LOOP` (ADC ISR: ZC logic; Timer1: NO-OP). Phase 1 path preserved behind `#else`. Add conditional include for bemf_zc.h |
| `Makefile-default.mk` | Add bemf_zc.c to source list |
| `nbproject/configurations.xml` | Add bemf_zc.c to project source list (IDE compatibility) |

**Created:**
| File | Purpose |
|---|---|
| `motor/bemf_zc.c` | ZC detection module: Init, OnCommutation, Poll, CheckDeadline, CheckTimeout |
| `motor/bemf_zc.h` | ZC detection interface (5 functions) |

**Feature-off build hygiene** (`FEATURE_BEMF_CLOSED_LOOP=0`): When the flag is 0, the following must be completely absent from the compiled binary:
- All ZC structs (`ZC_DIAG_T`, `ZC_TIMEOUT_RESULT_T`) and `zcDiag` member in `GARUDA_DATA_T`
- All ZC functions (`BEMF_ZC_*`) — `bemf_zc.c` can still be compiled (functions inside `#if`) but no code emitted
- All ZC macros (`TIMER1_TO_ADC_TICKS`, `*_ADC_STEP_PERIOD`, `ZC_*`) and `_Static_assert` checks
- All ADC ISR ZC statics (`adcIsrTick`, `dacUpdateDiv`, `prevAdcState`)
- DAC reference updates (no `HAL_CMP_SetReference` calls from ADC ISR)
- `HAL_CMP_EnableFloatingPhase()` call in `COMMUTATION_AdvanceStep()`
- `#include "motor/bemf_zc.h"` in `garuda_service.c`
- Verification: build with flag=0 must produce identical binary to current Phase 1 (same flash/RAM sizes)

---

### Implementation Order

1. `garuda_types.h` — Expand structs (build-safe, no functional change)
2. `garuda_config.h` + `garuda_calc_params.h` — Add defines with `FEATURE_BEMF_CLOSED_LOOP=1`, all Phase 2B sub-flags=0
3. `hal/hal_comparator.c/.h` — Add HAL_CMP_ReadStatus() (build-safe, unused yet)
4. `motor/bemf_zc.c/.h` — Create module (build-safe, unused yet)
5. `motor/commutation.c` — Add conditional CMP enable call
6. `garuda_service.c` — Wire everything with `#if FEATURE_BEMF_CLOSED_LOOP` guards
7. `Makefile-default.mk` + `nbproject/configurations.xml` — Add bemf_zc.c
8. Build with `FEATURE_BEMF_CLOSED_LOOP=0` first — verify Phase 1 behavior unchanged
9. Build with `FEATURE_BEMF_CLOSED_LOOP=1` — verify compile, then test on hardware

---

### Verification Plan

**Phase 2A (initial bring-up — all 2B sub-flags=0):**

1. **Rollback test**: Build with `FEATURE_BEMF_CLOSED_LOOP=0`, flash, verify motor spins exactly as Phase 1 (instant rollback path confirmed)
2. **Build**: Enable flag=1, zero errors, zero warnings
3. **Bench test (motor disconnected)**: Flash firmware, verify SW1 starts open-loop sequence, state transitions work via debugger
4. **Motor test (open-loop ramp)**: Motor spins up 300→5000 eRPM, enters CLOSED_LOOP forced mode
5. **Motor test (ZC sync)**: Watch `zcSynced` become true. Motor continues spinning smoothly. Pot controls speed
6. **Desync test**: Block motor shaft briefly → forced commutation fallback. Heavy blocking → FAULT_DESYNC. SW1 clears fault
7. **Direction test**: SW2 toggles direction — works in both forced and ZC modes
8. **Scope verification (optional)**: Probe floating phase BEMF and CMP output to confirm ZC aligns with actual zero crossings

**Phase 2B (after 2A is stable — enable sub-flags one at a time):**

9. Enable `ZC_ADAPTIVE_FILTER=1` — verify smooth operation across RPM range
10. Enable `ZC_ADAPTIVE_PERIOD=1` — verify IIR tracking is stable, no oscillation
11. Enable `ZC_DAC_IIR_FILTER=1` — verify reduced noise on comparator reference
