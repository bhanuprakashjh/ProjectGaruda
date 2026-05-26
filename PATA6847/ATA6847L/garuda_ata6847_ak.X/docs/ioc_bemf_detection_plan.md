# Plan: IOC-Driven BEMF Zero-Crossing Detection

> Goal: replace our current "PTG-triggered ADC level sample + sticky
> capture" path with an **edge-triggered Interrupt-on-Change** pattern
> on the ATA6847L's digital comparator outputs. The pattern matches
> what Microchip's own AVR-DX 6-step example and AM32 do (with their
> internal analog comparators), adapted for our case where the
> comparator already lives **inside the ATA6847L** and exports a
> digital edge to the MCU pin.

---

## 1. Reference summary

### 1.1 Microchip AVR-DX (the official template)

| Aspect | What they do |
|---|---|
| BEMF input | Phase voltages divided externally → AC1 multiplexed inputs (PD1-PD3 vs PD0 reference) |
| ZC ISR | `AC1_AC_vect` (analog-comparator edge interrupt). Edge polarity set per sector via `AC_INTMODE_NORMAL_POSEDGE_gc` / `NEGEDGE_gc`. |
| Blanking | Hardware: TCA1 CMP0 disables AC1 INT, expires at `blanking_time = zero_cross_time >> 1` (50 % of half-step), then CMP0 ISR re-enables AC1. |
| Commutation scheduling | TCA1 CMP1 compare-match → ISR triggers next step. Compare value = `current_zero_cross_time − ((... * MOTOR_ADVANCE_ANGLE) / 30) − MOTOR_PASSTHROUGH_DELAY_COMP` |
| Filter | 8-tap moving average on the measured period |
| Files | `motor_control.c:443–471`, `motor_control_hal.h:58–134`, `mcc_generated_files/src/ac1.c:43–64` |

### 1.2 AM32 (the de-facto FPV ESC firmware)

| Aspect | What they do |
|---|---|
| BEMF input | Phase voltage tap → comparator + input (PA2/PA3 on G071) |
| ZC ISR | EXTI line 17/18 → `ADC1_COMP_IRQHandler` → `interruptRoutine()` |
| Blanking | **Hardware** via `LL_COMP_BLANKINGSRC_TIM1_OC5` (TIM1 OC5 gates the comp output). CCR5 = 100 ticks at low speed, 5 ticks at high speed. |
| Commutation scheduling | Software countdown: `COM_TIMER->ARR = (interval/2) − advance`, ISR fires `commutate()` |
| Filter | Multi-read confirm: ISR re-reads comparator `filter_level` (3–12) times — early exit if any disagree. **Plus** an early-reject if `INTERVAL_TIMER->CNT < interval/2` (too soon to be a real ZC). |
| Files | `Src/main.c:930–975` (interruptRoutine), `Mcu/g071/Src/comparator.c:106–159` |

### 1.3 Bluejay / BLHeli (the 8051 reference)

| Aspect | What they do |
|---|---|
| BEMF input | SiLabs CPT0 comparator on GPIO, hysteresis off |
| ZC "ISR" | **NONE.** Polled: tight loop reads comparator output via `CPT0CN` register bit 6, with Timer3 as timeout. |
| Blanking | Software-only: wait `Wt_Zc_Scan_Start` ticks after commutation before opening the read window. |
| Filter | 1–27 consecutive matching reads (`Temp3`); reset counter on disagreement. |
| Commutation scheduling | Timer3 reload (`TMR3RLL/H = Wt_Adv_Start`), Timer3 ISR commutates. |

The 8051's polling style is unique — tight, single-cycle reads compete favorably with interrupt entry latency on that arch. Doesn't translate to a 32-bit DSC.

---

## 2. Our current architecture (where we are today)

| Aspect | Current on AK ATA6847L |
|---|---|
| BEMF input | ATA6847L's three digital comparator outputs → MCU GPIOs (BEMF_A/B/C). Output is inverted (HIGH ↔ BEMF below virtual neutral). |
| Detection path | **PTG-triggered ADC + GPIO level-sample**: PTG fires at mid-PWM-cycle (`PTG_TRIG_MID_OFF_POS` / `MID_ON_POS`), PTG ISR reads `BEMF_x_GetValue()` via 3-read deglitch, decides PRE-ZC vs not (`comp == expected`). Stores `lastCaptureHR_g`. |
| ZC ISR | `_PTG0Interrupt` in `hal/hal_ptg.c` (level sampler, not edge-triggered) |
| Blanking | Software: `blankingEndHR` written by Commutate, checked at top of `ProcessBemfSample`. |
| Filter | 3-read majority vote (`r1 + r2 + r3 >= 2`). |
| Commutation scheduling | SCCP3 one-shot timer (`HAL_ComTimer_ScheduleAbsolute`), CCT3 ISR → `SectorPI_Commutate`. |

The PTG/level-sample path was developed because the 6-step sampling timing has been finicky — it relies on the PWM phase to make sure the comparator is read at the right point in the switching cycle. This works (225 k eRPM milestone) but it has two costs:

1. **Latency proportional to sample interval** — at 30 kHz PWM the ZC can be detected anywhere from 0 to 33 µs late depending on when the next PTG fire happens. At 100 k+ eRPM that's a significant fraction of a sector.
2. **PTG ISR cost** — fires every PWM cycle (30 kHz), even though most fires don't carry useful data.

An edge-triggered IOC path would replace both costs with: one interrupt per actual edge, captured by hardware with near-zero latency, and the ISR only runs on real events.

---

## 3. Proposed: IOC-based BEMF detection

### 3.1 Why IOC, not the existing CCP2 input capture path

We already have CCP2 IC capture wired to the BEMF comparator output. That path works but only on the *active* floating phase — switching the input multiplex per sector is a hard problem (the PPS remap takes cycles, and you have ≤30 µs between sectors at 100 k+ eRPM). IOC avoids the PPS remap problem because **every BEMF pin can have IOC enabled independently**. You just mask/unmask the three IOC enables in the Commutate ISR — one register write each.

### 3.2 dsPIC33AK128MC106 IOC capabilities (confirmed via DS70005539 §10)

- Every Port A/B/C/D pin has CNIE/CNIF + edge-select bits.
- Edge-select per pin: `CNEN0` (rising) / `CNEN1` (falling) registers.
- Single shared ISR per port: `_CNAInterrupt`, `_CNBInterrupt`, `_CNCInterrupt`, `_CNDInterrupt`.
- ISR must read CNFA/B/C/D status flags to know which pin fired.
- Latency: ~5 cycles ISR entry on dsPIC33AK at FCY=200 MHz → **25 ns** to ISR body.

That's an order of magnitude faster than the PTG path's ~50 ns vector + the wait-for-next-PTG-fire delay.

### 3.3 Pin mapping (BEMF outputs from ATA6847L) — VERIFIED

From `hal/port_config.h:59-64`:

| Phase | Pin | Port | Bit | RP # (PPS) | CN-capable |
|---|---|---|---|---|---|
| BEMF_A | RB9  | **B** | 9  | RP26 | ✓ |
| BEMF_B | RB8  | **B** | 8  | RP25 | ✓ |
| BEMF_C | RA10 | **A** | 10 | RP11 | ✓ |

**Two ports involved** — A and B. That adds one wrinkle: we'll have two
ISR vectors (`_CNAInterrupt` for RA10, `_CNBInterrupt` for RB8/RB9). It's
still tractable because at any given sector **only one floating phase
is active**, so we enable only the corresponding port's CN interrupt
(`_CNAIE` or `_CNBIE`) and disable the other.

### 3.3.1 IOC SFR availability (verified from p33AK128MC106.h)

| Register | Purpose | Confirmed |
|---|---|---|
| `CNCONA` / `CNCONB` | Port-level enable (CNCONx.ON = 1) | ✓ |
| `CNEN0A` / `CNEN0B` | Rising-edge enable per pin | ✓ |
| `CNEN1A` / `CNEN1B` | Falling-edge enable per pin | ✓ |
| `CNFA` / `CNFB` | Which pin fired (read + clear) | ✓ |
| `CNPUA` / `CNPDA` | Pullup/pulldown — not needed (ATA6847L drives push-pull) | ✓ |
| `_CNAIE` / `_CNBIE` (IEC3) | Port interrupt enable | ✓ |
| `_CNAIF` / `_CNBIF` (IFS3) | Port interrupt flag | ✓ |
| `_CNAIP` / `_CNBIP` (IPC...) | Priority — set to 5 to match PTG | ✓ |

### 3.4 Architecture diagram (real pin layout)

```
ATA6847L (digital comp out)             dsPIC33AK128MC106
       ┌──────────┐                  ┌─────────────────────────┐
       │ Phase A  │──BEMF_A──────────│ RB9 (CNEN0B/CNEN1B [9])  │── _CNBInterrupt ─┐
       │ Phase B  │──BEMF_B──────────│ RB8 (CNEN0B/CNEN1B [8])  │    (Port B)      │
       │          │                  │                          │                  ├─→ to ZC handler
       │ Phase C  │──BEMF_C──────────│ RA10 (CNEN0A/CNEN1A[10]) │── _CNAInterrupt ─┘
       └──────────┘                  └─────────────────────────┘    (Port A)
                                                                          │
                                                                          ▼
                                                                  ┌──────────────────┐
                                                                  │ Snapshot CCP4HR  │
                                                                  │ Reject blanking  │
                                                                  │ 3-read filter    │
                                                                  │ Publish capture  │
                                                                  │ Schedule CCP3    │
                                                                  └──────────────────┘
```

### 3.5 ISR pseudocode

```c
void __attribute__((interrupt, no_auto_psv)) _CNBInterrupt(void) {
    /* 1. Snapshot HR timestamp ASAP — captures the true ZC moment */
    uint16_t now_hr = CCP4TMR;

    /* 2. Identify which pin fired + clear the flag */
    uint16_t flags = CNFB;
    CNFB = 0;
    _CNBIF = 0;

    /* 3. Blanking gate (matches AM32's early-reject) */
    if ((int16_t)(now_hr - blankingEndHR) < 0) {
        diagAdcBlankReject++;
        return;
    }

    /* 4. Only the FLOATING phase's edge is meaningful this sector */
    uint8_t fp_pin_mask = sectorFloatingPinMask;
    if ((flags & fp_pin_mask) == 0) {
        /* edge on a non-floating phase — ignore (e.g. switching transient) */
        diagWrongPhaseEdge++;
        return;
    }

    /* 5. Multi-read consensus, matches AM32's filter_level pattern.
     * 3 reads ~120 ns apart. */
    uint8_t r1 = ReadBEMFComp(currentFloatingPhase);
    Nop(); Nop(); Nop(); Nop();
    uint8_t r2 = ReadBEMFComp(currentFloatingPhase);
    Nop(); Nop(); Nop(); Nop();
    uint8_t r3 = ReadBEMFComp(currentFloatingPhase);
    if ((r1 + r2 + r3) < 2) {
        diagFilterReject++;
        return;
    }

    /* 6. Polarity gate (sector-direction-aware), same as
     * ProcessBemfSample's PRE-ZC check today. */
    uint8_t expected = (sectorRising) ? 1u : 0u;
    if (r1 != expected) {
        diagStateMismatch++;
        return;
    }

    /* 7. Accept the ZC — publish for the scheduler */
    lastCaptureHR_g = now_hr;
    captureValid    = true;

    /* 8. Mask further IOC interrupts on all three BEMF pins until
     * Commutate re-arms for the next sector. */
    CNEN0B &= ~(BEMF_A_MASK | BEMF_B_MASK | BEMF_C_MASK);
    CNEN1B &= ~(BEMF_A_MASK | BEMF_B_MASK | BEMF_C_MASK);
}
```

### 3.6 Per-sector IOC re-arming (inside Commutate) — cross-port version

Because BEMF_C lives on Port A while BEMF_A/B live on Port B, the
re-arm logic dispatches on which port owns the floating phase. At any
given sector exactly one port is active — the other has its port-level
CN interrupt disabled.

```c
/* Pin bit masks */
#define BEMF_A_BIT  9    /* RB9 — phase A floating phase mask */
#define BEMF_B_BIT  8    /* RB8 — phase B */
#define BEMF_C_BIT  10   /* RA10 — phase C */

/* Step 1: tear down. Mask both port enables + clear per-pin enables. */
_CNAIE = 0;  _CNBIE = 0;
CNEN0A = CNEN1A = 0;
CNEN0B = CNEN1B = 0;

/* Step 2: arm the one pin that matters this sector */
const COMMUTATION_STEP_T *s = &commutationTable[currentSector];
const bool rising = (s->zcPolarity > 0);
switch (s->floatingPhase) {
    case 0: /* Phase A → RB9, Port B */
        if (rising) CNEN0B |= (1U << BEMF_A_BIT);
        else        CNEN1B |= (1U << BEMF_A_BIT);
        CNFB = 0;  _CNBIF = 0;  _CNBIE = 1;
        break;
    case 1: /* Phase B → RB8, Port B */
        if (rising) CNEN0B |= (1U << BEMF_B_BIT);
        else        CNEN1B |= (1U << BEMF_B_BIT);
        CNFB = 0;  _CNBIF = 0;  _CNBIE = 1;
        break;
    case 2: /* Phase C → RA10, Port A */
        if (rising) CNEN0A |= (1U << BEMF_C_BIT);
        else        CNEN1A |= (1U << BEMF_C_BIT);
        CNFA = 0;  _CNAIF = 0;  _CNAIE = 1;
        break;
}
```

Note **ATA6847L inverts** — `rising ZC` ↔ comparator falling edge. So
the `rising` bool above is actually applied with inverted polarity
when arming CN. (We can either invert here or in the commutation
table; consistency with how `ReadBEMFComp()` already inverts is the
right call — encode in one place.)

### 3.6.1 Shared ZC handler — same body, two ISR wrappers

Both ISRs do the same work; just point them at one inline helper:

```c
static inline __attribute__((always_inline))
void BemfIocAccept(uint16_t pin_flags_register_value) {
    const uint16_t now_hr = CCP4TMR;

    /* Blanking + filter + polarity gate — same as plan §3.5 */
    if ((int16_t)(now_hr - blankingEndHR) < 0) {
        adcBlankReject++;
        return;
    }
    uint8_t r1 = ReadBEMFComp(currentFloatingPhase);
    Nop(); Nop(); Nop(); Nop();
    uint8_t r2 = ReadBEMFComp(currentFloatingPhase);
    Nop(); Nop(); Nop(); Nop();
    uint8_t r3 = ReadBEMFComp(currentFloatingPhase);
    uint8_t comp = ((uint8_t)(r1 + r2 + r3) >= 2u) ? 1u : 0u;

    uint8_t expected = (commutationTable[currentSector].zcPolarity > 0) ? 0u : 1u;
    if (comp != expected) {
        adcStateMismatch++;
        return;
    }

    lastCaptureHR_g = now_hr;
    captureValid    = true;

    /* Disarm both ports until Commutate re-arms */
    _CNAIE = 0;  _CNBIE = 0;
    CNEN0A = CNEN1A = 0;
    CNEN0B = CNEN1B = 0;
}

void __attribute__((interrupt, no_auto_psv)) _CNAInterrupt(void) {
    uint16_t flags = CNFA; CNFA = 0; _CNAIF = 0;
    BemfIocAccept(flags);
}

void __attribute__((interrupt, no_auto_psv)) _CNBInterrupt(void) {
    uint16_t flags = CNFB; CNFB = 0; _CNBIF = 0;
    BemfIocAccept(flags);
}
```

Because only one port is enabled per sector, the other vector cannot fire — so the duplicated wrapper is essentially free (the unused one stays linked but never executes).

### 3.6.2 Five-layer masking strategy (switching noise + demag)

A naive IOC will fire dozens of times per sector from PWM ringing and
demag artifacts. Five layers, in priority order:

#### Layer 1 — Demag interval gate (catches the worst case)

After commutation the freewheeling diode on the floating phase conducts
for ~10-30% of the sector. Comparator output during this window is
clamped by the diode, not driven by real BEMF.

```c
uint16_t since_comm = now_hr - lastCommHR;
uint16_t expected_period = predictedSectorPeriodHR;
if (since_comm < (expected_period >> 2)) {     // first 25% of sector
    diagDemagReject++;
    return;
}
```

Matches AM32's `if (INTERVAL_TIMER->CNT < commutation_interval >> 1) return`.
Fraction is tunable — start 1/4, tighten to 1/8 once observer is locked.
Scales with speed for free.

#### Layer 2 — Static blanking gate (defensive backup)

`blankingEndHR` is written by Commutate and serves as a backup to L1.

```c
if ((int16_t)(now_hr - blankingEndHR) < 0) {
    adcBlankReject++;
    return;
}
```

Two paths to implement:
- **Software** (Phase 1): keep ISR alive during blanking, reject in L1/L2.
- **Hardware** (Phase 2): SCCP3 one-shot fires at `blankingEndHR` and
  re-enables `_CNxIE`. Saves ISR cycles during blanking. Upgrade later
  if ISR storm is measurable.

#### Layer 3 — Multi-sample consensus (kills sub-µs PWM ringing)

```c
uint8_t r1 = ReadBEMFComp(currentFloatingPhase);
Nop(); Nop(); Nop(); Nop();
uint8_t r2 = ReadBEMFComp(currentFloatingPhase);
Nop(); Nop(); Nop(); Nop();
uint8_t r3 = ReadBEMFComp(currentFloatingPhase);
uint8_t comp = ((r1 + r2 + r3) >= 2u) ? 1u : 0u;
```

By the time the ISR runs, the line has settled. Ringing won't match the
steady state at 3 read points.

#### Layer 4 — Polarity gate (rejects wrong-direction edges)

```c
const COMMUTATION_STEP_T *s = &commutationTable[currentSector];
uint8_t expected = (s->zcPolarity > 0) ? 0u : 1u;   // ATA6847L inverts
if (comp != expected) {
    adcStateMismatch++;
    return;
}
```

#### Layer 5 — Speed-adaptive widths

| Param | Low speed (>5 ms) | Mid (~1 ms) | High (<200 µs) |
|---|---|---|---|
| Demag interval gate | period/4 | period/6 | period/8 |
| Sample count (L3) | 5 | 3 | 1-2 |
| Static blanking (L2) | 200 µs | 50 µs | 10 µs |

Implement as a once-per-sector update in Commutate.

#### Demag-specific extras (Phase 2+)

1. **Current-aware demag** — when `gIbus_mA` is high pre-commutation,
   demag lasts longer. Scale L1's fraction with current.
2. **Wrong-polarity-edge demag detector** — count consecutive wrong
   edges; if many, extend blanking. Helps with prop kickback.
3. **Hardware CLC gate** — `(BEMF & ~demag_active)` through CLC, where
   `demag_active` comes from a one-shot timer. Removes ISR cycles
   during demag entirely. The dsPIC33AK has CLC; not used on this
   board yet.

### 3.7 Blanking — keep what we have

`blankingEndHR` is computed in the Commutate ISR today
(`blankingEndHR = now + blankingTicks`), and the IOC ISR rejects edges
before that point. This matches AVR-DX's TCA1 CMP0 software-equivalent
and is simpler than re-implementing it with a separate timer.

If we want hardware-gated blanking later (like AM32's CCR5 idea), the
dsPIC33AK has CCP "synchronous gate" features we could use. Not in
scope for the first pass.

### 3.8 Commutation scheduling — unchanged

`SectorPI_Commutate` already schedules via `HAL_ComTimer_ScheduleAbsolute(now + delay)`. The only change is the *input* to the math — the ZC timestamp now comes from the IOC ISR's `lastCaptureHR_g`, which is **edge-accurate** instead of mid-PWM-sample.

---

## 4. Implementation plan (phased)

### Phase 1 — IOC infrastructure (~1 day)

1. **Verify BEMF pin layout** — grep `BEMF_*_GetValue` + check `port_config.h` to identify exact pins. Confirm they're all on the same CN port (Port B preferred).
2. **Add `hal/hal_ioc.{c,h}`** — small init/enable/disable helpers:
   - `HAL_Ioc_Init()`: clear CNFx, set priority, enable CN interrupt.
   - `HAL_Ioc_ArmBemfRising(uint8_t pin_mask)` / `HAL_Ioc_ArmBemfFalling(...)`.
   - `HAL_Ioc_DisarmBemf()` — used inside CN ISR after accept.
3. **`FEATURE_IOC_BEMF` compile flag** in `garuda_config.h`, default 0.
   When 0, current PTG path stays unchanged. When 1, route to IOC.

### Phase 2 — IOC ISR + Commutate integration (~1 day)

4. **Add `_CNBInterrupt`** in `garuda_service.c` (or a new file) with
   the pseudocode in §3.5. Gate with `#if FEATURE_IOC_BEMF`.
5. **Modify `SectorPI_Commutate`** — when `FEATURE_IOC_BEMF=1`, after
   each step, call `HAL_Ioc_ArmBemfRising/Falling()` based on
   `commutationTable[step].zcPolarity`. Skip the PTG re-arm.
6. **Disable PTG in IOC mode** — `HAL_PTG_Start` called from
   `SectorPI_Start` becomes `#if !FEATURE_IOC_BEMF`.

### Phase 3 — Bench-test, iterate (~2-3 days)

7. **Sanity test**: motor at 24V no-load, watch GSP snapshot's
   `goodZcCount`, `adcBlankReject`, `adcStateMismatch`. Should see
   one IOC fire per sector and consistent commutation interval.
8. **Compare against PTG path** at fixed throttle — set
   `FEATURE_IOC_BEMF=0`, repeat run, compare eRPM stability + max
   speed.
9. **Tune filter** — start with the 3-read consensus + 4 NOP spacing.
   If false ZCs appear, scale filter up or add hysteresis on
   wrong-phase edges. If true ZCs are missed, reduce filter to 1 read.
10. **High-speed test**: prop on 2810 at 24V, see if IOC can sustain
    >200 k eRPM. Hypothesis: yes, and possibly with smoother CL than
    PTG path because the timestamp is edge-true.

### Phase 4 — Cleanup if successful (~0.5 day)

11. **Remove PTG path** or leave it dual-pathed depending on whether
    IOC has any failure modes the PTG path covers.
12. **Update docs** — `ak_an1078_foc_port.md` only mentions FOC; add a
    section on 6-step IOC path.

---

## 5. Risks & mitigations

| Risk | Likelihood | Mitigation |
|---|---|---|
| IOC misses an edge during sector handoff (race between `Disarm` in ISR and `Arm` in Commutate) | medium | Order writes carefully: in Commutate, mask IOC first, do the PWM phase change, then re-arm IOC after `blankingEndHR` window — same convention as the source board. |
| Switching transients on the active phases generate spurious edges that aren't filtered | medium-high | Only listen on the floating phase pin (mask the other two). Add multi-read consensus. Use `blankingEndHR` strictly. |
| ATA6847L comparator output has rise/fall slew slower than IOC sample → edge debounce | low | The ATA6847L data sheet quotes <100 ns propagation; well inside IOC sample latency. |
| dsPIC33AK CN priority below CCP scheduler → CN ISR queued behind commutation work | low | Set `_CNBIP = 5` (same priority as PTG today). |
| `CNFB` flag rules differ from CNxx on dsPIC33CK (some pins need `CNCONx` enable bit) | low | DS70005539 §10 confirms identical model: pin-level enable in `CNENx`, port-level enable in `CNCONx.ON`. Init both. |

---

## 6. Anticipated wins

- **ZC latency**: ~25 ns (IOC ISR entry) vs ~17 µs worst-case (PTG at 30 kHz)
  → at 100 k eRPM each sector is 100 µs; 17 µs is 17 % of sector → 6° error.
  IOC eliminates this.
- **CPU load**: PTG fires 30 k × /s regardless of motor speed. IOC
  fires at most 6 × eRPM/60 = 21 k/s at 210 k eRPM. ~30 % less ISR
  overhead at peak speed.
- **Conceptual simplicity**: edge-true detection matches what every
  major sensorless BLDC reference does (Microchip's own example,
  AM32, ST motor-control library, Microchip MCAF). No level-sampling
  PTG choreography.

## 7. What we keep unchanged

- 6-step commutation table (`motor/commutation.c`).
- Speed PI / sector_pi state machine.
- ATA6847L gate driver setup.
- ADC current sampling (still PTG/PWM-triggered, just not used for BEMF).
- GSP telemetry + GUI.

## 8. Bench results (2026-05-21)

Implemented + bench-tested. Two runs with the same firmware, only PWM
rate changed. Both same motor (2810 1350 KV @ 24 V, no prop), same
load (no load):

| Configuration | iAccept (33 s) | iDemag | iBlank | iPol | Total fires | Peak eRPM | Outcome |
|---|---:|---:|---:|---:|---:|---:|---|
| 60 kHz PWM, L1 = period/4 | 504 | 1 252 011 | 363 205 | 840 | ~47 k/s | 90 k | desync |
| 30 kHz PWM, L1 = period/8 | 2 007 (9 s) | 2 253 186 | 3 134 198 | 2 661 | **~700 k/s** | 65 k | desync, "extremely rough" |

The 30 kHz run was 15× *noisier* than the 60 kHz run — the slower PWM
gives the comparator output more time to drift between switching
events, generating more spurious edges per cycle.

### What we learned

1. **Comparator without hardware blanking is fundamentally noisy for
   IOC.** PWM switching transients couple capacitively/inductively to
   the BEMF input and produce comparator output edges that look
   indistinguishable from real ZCs once they pass the polarity gate.
2. **Slower PWM ≠ less noise.** It's more — faster PWM has shorter
   transient windows.
3. **`goodZcCount` ≫ `iAccept` ratio** at peak (~25:1) means the motor
   was being driven by *forced* commutations from sector_pi's timeout
   path, not real BEMF feedback. The IOC was contributing real ZCs
   ~4% of the time. That's enough to "keep up" mechanically up to ~90 k
   eRPM, but no actual closed-loop control.
4. **AVR-DX / AM32 / ST all use hardware-blanked comparators.** Their
   IOC-style designs work because the AC peripheral has a built-in
   blanking input gated by a timer. Our ATA6847L comparator output is
   a raw GPIO without that gate.

### Why PTG wins on this hardware

PTG works because it *samples at a deterministic quiet moment* in the
PWM cycle (mid-OFF, well past switching edges). The level sample is
read once per PWM cycle when the comparator output is stable. False
edges during switching are never observed.

IOC sees every edge. Without hardware blanking, that's a much harder
problem.

### Path forward (if anyone re-attempts)

To make IOC work on this hardware would require **CLC hardware
blanking**: route the BEMF pin through a CLC LUT that AND's with a
"PWM not switching" signal driven by a one-shot timer. Then IOC sees
only edges in the quiet window. dsPIC33AK has CLCs; not wired today.
Estimated 1–2 days of work; outcome still uncertain.

For now, **the IOC code stays compiled-but-off** (`FEATURE_IOC_BEMF=0`
default) as a reference implementation and starting point for future
hardware-blanked rework. The PTG path remains the production 6-step
detector.

## 9. Open questions for next session

1. Are all three BEMF pins on the **same** CN port? (need to grep port_config.h)
2. Does the ATA6847L digital comparator output have **glitching** during the dead-time window that would force us to extend blanking?
3. Do we want to keep the level-sample diagnostic counters (`bemfTally`, `compFalling_*`) running in parallel for verification?
4. After IOC works at 60 kHz PWM, can we drop PWM back to 30 kHz without giving up speed? (currently 30 kHz works because PTG sample timing depends on PWM phase — IOC is PWM-rate-independent).
