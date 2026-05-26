# V4 Dual-Sample ZC — Implementation Plan

**Status**: design, not yet started
**Author handoff**: 2026-05-01
**Target reader**: developer who will modify the code and bench-test it
**Scope**: add a second BEMF sample point at the PWM **period boundary** (true OFF-center) so falling-ZC sectors can be detected at low duty / CL-entry, without disturbing the rising-ZC ON-mid path that already works at high speed.

This plan is a *recipe*. Read sections 1–4 before touching code; the snippets in section 5 are sized to drop in with minimal restructuring. Section 6 is the bench protocol — every build step lists what to look for in telemetry.

---

## 1. Why we are doing this

### 1.1 The empirical observation
- Without a prop, current code reaches CL fine. With a prop, CL entry is fragile, especially during the first 100–200 ms.
- Falling-ZC sectors are **easier** to detect at low duty (<30%) and during CL entry; rising-ZC sectors are **easier** at high duty (>60%).
- The current ADC ISR samples once per PWM period at the trigger point set by `PG1TRIGA = 0` — empirically a single sample window inside the ON pulse.
- At 12% duty / 60 kHz PWM the ON pulse is ~2 µs wide; the inner clean window after edge transients is ~0.5–1 µs. Three-sample debounce gives ~1.5 µs of effective observation per ~16.7 µs period — borderline unusable for falling-sector reads.

### 1.2 The geometric insight
In center-aligned PWM, the OFF region of one period and the OFF region of the next period meet at the period boundary. For duty `D` and period `N`, the **wrapped OFF window** has length `N − D` and its midpoint sits **at the period boundary** — by symmetry, equidistant from the two adjacent ON-pulse edges. At 12% duty / 60 kHz this is a 14.7 µs window with **7.3 µs of margin to the nearest switching edge**. That is the cleanest single instant in the period for an OFF-time read.

### 1.3 Why a flag-based dual mode (not a hard switch)
We will support three modes via a config flag:

| `FEATURE_ZC_MODE` | Sample point(s) | Timestamp routing | Sectors detected | Sectors extrapolated |
|---|---|---|---|---|
| `ZC_MODE_RISING_ONLY` (0) | ON-mid (existing) | ON-mid → all sectors | 3 of 6 (rising) | 3 (falling) |
| `ZC_MODE_FALLING_ONLY` (1) | Period-boundary (new) | Period-bdy → all sectors | 3 of 6 (falling) | 3 (rising) |
| `ZC_MODE_DUAL` (2) | Both | Sector polarity routes | 6 of 6 | 0 |

`RISING_ONLY` is the current shipping behavior — bit-identical when the flag is set. `FALLING_ONLY` is the new sampler in isolation. `DUAL` activates both and routes by sector polarity (rising sector → ON-mid timestamp wins, falling sector → period-boundary timestamp wins). The flag exists so we can A/B each mode without entangling them; do not skip straight to `DUAL`.

### 1.4 The unsolved blocker we are sidestepping
Per `memory/ck_offmid_falling_zc_research.md` (2026-04-17): every prior periodic ISR that tried to sample on falling-ZC sectors hit a steady-state failure where `HAL_Capture_IsRisingZc()` appeared locked at `true` once CL stabilized. The 2026-04-29 work in `_ADCInterrupt` (`garuda_service.c:436`) routed around this by reading `v5_ptgExpectedComp` (a `volatile uint8_t` written by Commutate) instead of calling the function. **Reuse this same source in the new sampler** — do not call `HAL_Capture_IsRisingZc()` from the period-boundary ISR.

Also: in this plan, falling-sector sampling is only active when duty < ~30% (low duty) and at CL entry. That regime overlaps the 2026-04-17 *transient* window where prior research already saw 45% capture success. We are intentionally not trying to use this path at 100k+ eRPM.

---

## 2. Architecture

### 2.1 Two implementation options for the second sample point

We need an ISR that fires once per PWM period at the period boundary (counter wrap). Two clean options:

#### Option A — Repurpose SCCP1 (recommended)
SCCP1 is already configured in `hal_capture.c:81-92` as a free-running 24.96 µs timer with a diagnostic ISR at `garuda_service.c:534`. Currently:
- Period `(FCY/4 / PWMFREQUENCY_HZ) − 2` — deliberately drifts vs PWM (used to be for empirically scanning sample phase).
- Seeded at `CCP1PRL/2` in `HAL_Capture_Start()` to fire at "PWM peak" (mid-OFF) 12.5 µs after ADC trigger.
- ISR is `_CCT1Interrupt` — currently increments `v4_offMidCapture` / `v4_offMidMismatch` only, does **not** feed `v4_lastCaptureHR`.

Changes required:
1. Set `CCP1PRL = (FCY / 4 / PWMFREQUENCY_HZ) − 1` (no drift — exact PWM period).
2. Re-seed `CCP1TMRL = CCP1PRL/2` in `HAL_Capture_Start()` so it phase-locks to mid-OFF / period-boundary geometric center (same seed point that's already there — keep it).
3. Re-enable `_CCT1IE = 1` in `HAL_Capture_Start()` (currently disabled with "Phase B" comment).
4. Re-write `_CCT1Interrupt` body to actually capture and feed `v4_lastCaptureHR` / `v4_captureValid` based on the active `FEATURE_ZC_MODE`.

**Pro**: infrastructure exists; minimal new code; no risk of breaking PG1 trigger chain. Geometric centering already present.
**Con**: SCCP1 timer is async to PWM clock — phase-lock relies on identical clocks, will drift slowly if temperature/jitter creates ppm-level offsets. In practice the 100 MHz Fp peripheral feeds both, so drift is negligible across a sector but should be re-anchored on every motor start.

#### Option B — Enable `_PWM1Interrupt` at end-of-cycle
Use the PWM peripheral's own ISR vector at `IFS4/IEC4` (already declared in `hal_pwm.c:211-212`, presently disabled). Set `PG1EVTL[IEVTSEL] = 0b001` (end-of-cycle) to fire the ISR at counter wrap.

```c
// hal_pwm.c — extend HAL_PWM_Init or add helper
PG1EVTL = (PG1EVTL & ~0x00E0u) | (0b001u << 5);  // IEVTSEL = 001 (EOC)
IPC16bits.PWM1IP = V4_CAPTURE_ISR_PRIORITY;       // priority 5 — same as CCP2/CCP5
IFS4bits.PWM1IF  = 0;
IEC4bits.PWM1IE  = 1;
```

**Pro**: hardware-locked to PWM (zero drift), no async timer plumbing.
**Con**: `PG1EVTL` field encoding must be verified against the device datasheet — the comment in `hal_pwm.c:138` calls bits 4:0 "PGTRGSEL=EOC, ADTR1EN1" but my read of the family ref says IEVTSEL is bits 7:5. **Verify before committing** — drop a single one-shot in `HAL_PWM_Init`, scope `_PWM1Interrupt` entry, and confirm timing relative to PG1OUT before wiring it into the capture sink.

#### Recommendation
**Start with Option A.** SCCP1 is already shaped exactly for this job; the only thing wrong with it today is its period (drift) and its ISR contents (counters only, not capture). Option B becomes attractive if the SCCP1 / PWM phase relationship turns out to drift visibly over a sector.

The rest of this plan uses Option A. A two-line note at the end of section 5.4 covers the Option B variant.

### 2.2 Shared data flow — both samplers write the same sink

```
                    +----------------+
                    |  Commutate ISR | (SCCP3, garuda_service.c:555)
                    | sets v5_ptgExpectedComp per sector
                    | sets currentRisingZc via HAL_Capture_Configure
                    +-------+--------+
                            |
              +-------------+-------------+
              |                           |
   +----------v---------+      +----------v---------+
   | _ADCInterrupt      |      | _CCT1Interrupt     |
   | fires at counter=0 |      | fires at MPER/2    |
   | (ON-mid)           |      | (period boundary)  |
   | gates: rising mode |      | gates: falling mode|
   |   or DUAL+rising   |      |   or DUAL+falling  |
   +----------+---------+      +----------+---------+
              |                           |
              +-------------+-------------+
                            |
                     v4_lastCaptureHR
                     v4_captureValid
                            |
                            v
                   sector_pi.c consumer
```

Both ISRs use the **same accept rule**: `comp == v5_ptgExpectedComp` (V5 post-ZC convention). Both honor blanking via `v4_blankingEndHR`. The difference is *when* they fire and which sectors they're allowed to write to.

### 2.3 Diagnostic counters before we change behavior
Phase 1 of the rollout adds the period-boundary sampler **but only counts** what it would have done — it does not write `v4_lastCaptureHR`. This lets us compare capture rates between ON-mid and period-boundary at every duty level *without* destabilizing the motor. New counters:

```c
volatile uint32_t v6_pbRisingAcc;   // period-boundary sample, rising sector, comp matches
volatile uint32_t v6_pbRisingRej;   // rising sector, comp doesn't match
volatile uint32_t v6_pbFallingAcc;  // falling sector, comp matches
volatile uint32_t v6_pbFallingRej;  // falling sector, comp doesn't match
volatile uint32_t v6_pbBlankReject; // fired pre-blanking-end
```

These mirror the existing `v5_postZcRisingAcc/Rej/FallingAcc/Rej` set in `garuda_service.c:345-348` so we get apples-to-apples comparison in telemetry.

---

## 3. Files we will touch

| File | Lines (current) | What changes |
|---|---|---|
| `garuda_config.h` | ~870–900 | Add `ZC_MODE_*` enum + `FEATURE_ZC_MODE` flag (default = `RISING_ONLY` for byte-identity) |
| `hal/hal_capture.c` | 81–92, 109–122 | Switch SCCP1 period from drift to exact, leave seed at `PRL/2` |
| `hal/hal_capture.c` | 119–122 | Enable `_CCT1IE = 1` when `FEATURE_ZC_MODE != RISING_ONLY` |
| `garuda_service.c` | 333–348 | Add `v6_pb*` counter declarations |
| `garuda_service.c` | 534–552 | Rewrite `_CCT1Interrupt` body (Phase 1: counters only; Phase 2: capture sink) |
| `garuda_service.c` | 395–510 (`_ADCInterrupt`) | Add a single mode-gate around the capture-set so `RISING_ONLY` and `DUAL+rising` write, others skip |
| `motor/sector_pi.h` | `V4_TELEM_T` struct ~63 | Add 5 fields for `v6_pb*` counters |
| `motor/sector_pi.c` | `SectorPI_TelemGet` body | Copy the new counters into telemetry |
| `gsp/gsp_commands.c` | snapshot serializer | Pack the 5 new u32s if telemetry budget allows; otherwise reuse existing slots when an experiment is active |
| `tools/pot_capture.py` | header + parser | Parse the 5 new fields, expose as columns |

No other files need to change for the diagnostic build. The capture-sink build (Phase 2) only adds `_CCT1Interrupt` body code — no further file scope.

---

## 4. The flag

Add this near the V4 feature-flag block in `garuda_config.h` (around the `FEATURE_V5_PTG_ZC` line at 944):

```c
/* ── V6 Dual-sample ZC ────────────────────────────────────────────
 * Adds a second BEMF sample point at the PWM period boundary (true
 * geometric OFF-center) for falling-ZC sectors. Existing ADC-ISR
 * sample at counter=0 (ON-mid) handles rising-ZC sectors.
 *
 * RISING_ONLY (0): byte-identical to V5.1 baseline. New ISR is dead
 *   code (gated out by preprocessor). Use this as the regression
 *   reference whenever investigating dual-mode anomalies.
 * FALLING_ONLY (1): only the period-boundary ISR feeds the capture
 *   sink. Rising sectors get extrapolated. Diagnostic build for
 *   measuring falling-sector capture rate in isolation.
 * DUAL (2): both ISRs feed the sink, sector polarity routes which
 *   timestamp wins. 6-of-6 hardware detection target.
 *
 * See docs/v4_dual_zc_implementation_plan.md for the full plan. */
#define ZC_MODE_RISING_ONLY   0
#define ZC_MODE_FALLING_ONLY  1
#define ZC_MODE_DUAL          2

#ifndef FEATURE_ZC_MODE
#define FEATURE_ZC_MODE       ZC_MODE_RISING_ONLY
#endif

/* Diagnostic-only build: period-boundary ISR runs and counts but
 * does NOT write v4_lastCaptureHR / v4_captureValid. Use this for
 * Phase 1 bench testing — proves the new ISR fires and sees clean
 * comparator state without putting it on the critical path. */
#ifndef FEATURE_ZC_MODE_DIAGNOSTIC_ONLY
#define FEATURE_ZC_MODE_DIAGNOSTIC_ONLY  1
#endif
```

`FEATURE_ZC_MODE_DIAGNOSTIC_ONLY` lives outside the mode flag so we can switch from "counters only" to "live capture" without touching the mode itself. Default `1` for the first build — flip to `0` when telemetry confirms the new path is healthy.

---

## 5. Code snippets

All snippets are drop-in changes against the current state of the files. Quote the surrounding context so a 3-way merge is unambiguous.

### 5.1 `hal_capture.c` — exact-period SCCP1

Find the SCCP1 init block at `hal_capture.c:77-92`:

```c
    /* ── SCCP1: 40 kHz periodic timer for OFF-mid falling ZC ──────
     * ... existing comment block ... */
    CCP1CON1L = 0; CCP1CON1H = 0; CCP1CON2L = 0; CCP1CON2H = 0;
    CCP1CON1Lbits.CCSEL  = 0;
    CCP1CON1Lbits.T32    = 0;
    CCP1CON1Lbits.CLKSEL = 0b000;
    CCP1CON1Lbits.TMRPS  = 0b10;
    CCP1CON1Lbits.MOD    = 0b0000;
    CCP1PRL = (FCY / 4 / PWMFREQUENCY_HZ) - 2;  /* 623 = 24.96 µs — deliberate drift vs PWM */
```

Replace the `CCP1PRL` line with:

```c
#if FEATURE_ZC_MODE != ZC_MODE_RISING_ONLY
    /* V6 dual-sample: phase-lock to PWM period (no drift).
     * −1 because TMR runs from 0 inclusive — so PRL = period_ticks − 1. */
    CCP1PRL = (FCY / 4 / PWMFREQUENCY_HZ) - 1;
#else
    CCP1PRL = (FCY / 4 / PWMFREQUENCY_HZ) - 2;  /* legacy 24.96 µs drift */
#endif
```

The seed at `PRL/2` in `HAL_Capture_Start()` (line 119) stays as-is — it positions the first fire at half-period from start, which is the geometric wrapped-OFF center given the existing PG1 trigger config.

In `HAL_Capture_Start()` at `hal_capture.c:121`, change:

```c
    _CCT1IE = 0;                /* Phase B: was 1 — SCCP1 ISR disabled */
```

to:

```c
#if FEATURE_ZC_MODE != ZC_MODE_RISING_ONLY
    _CCT1IE = 1;                /* V6: re-enabled for period-boundary sample */
#else
    _CCT1IE = 0;                /* legacy: SCCP1 off in rising-only mode */
#endif
```

### 5.2 `garuda_service.c` — counter declarations

After `v5_postZcFallingRej` declaration at `garuda_service.c:348` add:

```c
/* V6 period-boundary sample counters (mirror v5_postZc* layout).
 * Incremented in _CCT1Interrupt for every fire past blanking, no
 * v4_captureValid sticky gate — per-sample rate matches v5_postZc*
 * for direct apples-to-apples comparison. */
volatile uint32_t v6_pbRisingAcc   = 0;
volatile uint32_t v6_pbRisingRej   = 0;
volatile uint32_t v6_pbFallingAcc  = 0;
volatile uint32_t v6_pbFallingRej  = 0;
volatile uint32_t v6_pbBlankReject = 0;
```

### 5.3 `garuda_service.c` — `_CCT1Interrupt` rewrite

Find the existing ISR at `garuda_service.c:534-552`. Replace its body with:

```c
void __attribute__((interrupt, no_auto_psv)) _CCT1Interrupt(void)
{
    _CCT1IF = 0;

#if FEATURE_ZC_MODE != ZC_MODE_RISING_ONLY
    if (v4_spActive
        || !SectorPI_IsRunning()
        || SectorPI_GetPhase() != 3)   /* CL only */
        return;

    uint16_t nowHR = CCP4TMRL;
    if ((int16_t)(nowHR - v4_blankingEndHR) < 0) {
        v6_pbBlankReject++;
        return;
    }

    /* Same accept convention as ADC ISR — read v5_ptgExpectedComp
     * (volatile global written by Commutate, NOT the function call). */
    extern volatile uint8_t v5_ptgExpectedComp;
    uint8_t expectedPost = v5_ptgExpectedComp;
    bool    sectorRising = (expectedPost == 0u);

    /* 3-read deglitch — same width as the ADC ISR path so noise
     * sensitivity is matched. Total ~400 ns. */
    uint8_t r1 = ReadBEMFComp();
    Nop(); Nop(); Nop(); Nop();
    uint8_t r2 = ReadBEMFComp();
    Nop(); Nop(); Nop(); Nop();
    uint8_t r3 = ReadBEMFComp();
    uint8_t comp = ((uint8_t)(r1 + r2 + r3) >= 2u) ? 1u : 0u;

    if (comp == expectedPost) {
        if (sectorRising) v6_pbRisingAcc++;
        else              v6_pbFallingAcc++;
    } else {
        if (sectorRising) v6_pbRisingRej++;
        else              v6_pbFallingRej++;
    }

#if !FEATURE_ZC_MODE_DIAGNOSTIC_ONLY
    /* Phase 2: feed the capture sink, but ONLY for sectors this
     * mode is allowed to write to. */
    bool allowedToWrite =
#if FEATURE_ZC_MODE == ZC_MODE_FALLING_ONLY
        !sectorRising;
#elif FEATURE_ZC_MODE == ZC_MODE_DUAL
        !sectorRising;       /* rising sectors stay with the ADC ISR */
#else
        false;
#endif

    if (allowedToWrite && (comp == expectedPost) && !v4_captureValid) {
        v4_lastCaptureHR = nowHR;
        v4_captureValid  = true;
    }
#endif /* !FEATURE_ZC_MODE_DIAGNOSTIC_ONLY */

#endif /* FEATURE_ZC_MODE != ZC_MODE_RISING_ONLY */
}
```

The structure is: gate-on-state, blanking, read-and-deglitch, count, then optionally write. The `allowedToWrite` ladder collapses to a single boolean at compile time so there's no runtime mode dispatch.

### 5.4 `garuda_service.c` — gate the ADC ISR's write

Find the V5 post-ZC accept block in `_ADCInterrupt` at `garuda_service.c:467-482` (the `FEATURE_V5_POST_ZC_OWN` arm). The block currently writes unconditionally:

```c
                    {
                        extern volatile uint8_t v5_ptgExpectedComp;
                        uint8_t expectedPost = v5_ptgExpectedComp;
                        bool    sectorRising = (expectedPost == 0u);
                        if (comp != expectedPost)
                        {
                            v4_adcStateMismatch++;
                        }
                        else
                        {
                            v4_adcCaptureSet++;
                            if (sectorRising) v4_adcSetRising++;
                            v4_lastCaptureHR = nowHR;
                            v4_captureValid  = true;
                        }
                    }
```

Wrap the write in a mode gate:

```c
                    {
                        extern volatile uint8_t v5_ptgExpectedComp;
                        uint8_t expectedPost = v5_ptgExpectedComp;
                        bool    sectorRising = (expectedPost == 0u);

                        bool allowedToWrite =
#if FEATURE_ZC_MODE == ZC_MODE_RISING_ONLY
                            true;        /* legacy: ADC writes for all sectors */
#elif FEATURE_ZC_MODE == ZC_MODE_FALLING_ONLY
                            false;       /* falling-only: SCCP1 writes, ADC counts only */
#elif FEATURE_ZC_MODE == ZC_MODE_DUAL
                            sectorRising;  /* DUAL: ADC owns rising sectors */
#endif

                        if (comp != expectedPost) {
                            v4_adcStateMismatch++;
                        } else {
                            v4_adcCaptureSet++;
                            if (sectorRising) v4_adcSetRising++;
                            if (allowedToWrite && !v4_captureValid) {
                                v4_lastCaptureHR = nowHR;
                                v4_captureValid  = true;
                            }
                        }
                    }
```

In `RISING_ONLY` mode this collapses back to the current behavior at compile time. The `!v4_captureValid` guard is added so the two ISRs cooperate cleanly when `DUAL` is active and one fires before the other within a sector.

### 5.5 Telemetry plumbing

In `motor/sector_pi.h:24-63`, add to `V4_TELEM_T`:

```c
    /* V6 period-boundary sample counters */
    uint32_t pbRisingAcc;
    uint32_t pbRisingRej;
    uint32_t pbFallingAcc;
    uint32_t pbFallingRej;
    uint32_t pbBlankReject;
```

In `SectorPI_TelemGet` (sector_pi.c — find where `postZc*` are copied), add the parallel block:

```c
    extern volatile uint32_t v6_pbRisingAcc, v6_pbRisingRej,
                             v6_pbFallingAcc, v6_pbFallingRej,
                             v6_pbBlankReject;
    out->pbRisingAcc   = v6_pbRisingAcc;
    out->pbRisingRej   = v6_pbRisingRej;
    out->pbFallingAcc  = v6_pbFallingAcc;
    out->pbFallingRej  = v6_pbFallingRej;
    out->pbBlankReject = v6_pbBlankReject;
```

### 5.6 GSP snapshot — packing the new fields

The CK board snapshot is presently 256 bytes (per memory `444de52` removal of gate-readiness fields). 5 × u32 = 20 bytes. Verify the current snapshot size in `gsp_commands.c` before just adding bytes — if it overflows the TX ring, follow the same pattern Codex used for the `zcSync` removal: drop a stale field that nothing on the GUI consumes.

A safe minimum first step is to overlay the 5 counters on top of the existing `ptgFires`, `ptgRisingAcc`, `ptgRisingRej`, `ptgFallingAcc`, `ptgFallingRej` slots when `FEATURE_V5_PTG_ZC == 0` (which is the default per `garuda_config.h:945`). Those PTG counters are dead in the current build. Reusing their bytes avoids any size change.

### 5.7 Option B (PWM ISR) variant — keep this as fallback

If SCCP1 phase-locking proves unreliable, the same `_CCT1Interrupt` body can move into `_PWM1Interrupt` with these changes:
- In `hal_pwm.c:138`, OR `0x0020` into `PG1EVTL` to set `IEVTSEL = 0b001` (end-of-cycle).
- In `hal_pwm.c:212`, set `IEC4bits.PWM1IE = 1` and add `IPC16bits.PWM1IP = V4_CAPTURE_ISR_PRIORITY;`.
- Rename `_CCT1Interrupt` → `_PWM1Interrupt`, change `_CCT1IF = 0;` to `IFS4bits.PWM1IF = 0;`.
- Leave SCCP1 disabled.

Confirm `IEVTSEL` bit positions against DS70005320 / DS70005349 before flashing — the existing register annotation in `hal_pwm.c:138` is partially wrong about which bits are which.

---

## 6. Bench protocol

This is the order in which to take the changes to the bench. **Do not skip phases.** Each phase has a single hypothesis it tests.

### Phase 0 — regression baseline (no code change)

```bash
cd garuda_6step_ck.X && make -f Makefile-cli.mk MOTOR_PROFILE=1 clean all
```

Flash, full pot sweep on A2212 12V no-load. Capture telemetry (`tools/pot_capture.py`). Save as `baseline_phase0_<date>.csv`. Look for: existing peak eRPM, existing `adcCaptureSet` rate, existing `postZcFallingAcc/Rej` distribution.

This is the regression target for phase 1.

### Phase 1 — diagnostic build (counters only)

Build with:
```c
#define FEATURE_ZC_MODE                  ZC_MODE_FALLING_ONLY  // arm SCCP1 path
#define FEATURE_ZC_MODE_DIAGNOSTIC_ONLY  1                     // count, do NOT write sink
```

The motor should be **byte-identical to phase 0 in motor behavior** — no captures from SCCP1 reach the sink. Only the new counters change.

What to look for:
- Motor reaches the same peak eRPM as phase 0 (regression check — if it doesn't, the new ISR is stealing CPU; abort).
- `pbRisingAcc / (pbRisingAcc + pbRisingRej)` ratio across the duty range.
- `pbFallingAcc / (pbFallingAcc + pbFallingRej)` ratio across the duty range.
- Compare to `postZcFallingAcc / postZcFallingRej` — are we picking up falling sectors that ON-mid was missing? Expectation per prior research: falling acceptance rate at low duty should be 40–70%, dropping at high duty.

Decision gate:
- If `pbFallingAcc/(Acc+Rej) >= 0.40` at <30% duty → proceed to phase 2.
- If <0.20 → period-boundary sample isn't actually clean on this hardware. Stop and re-verify trigger timing on a scope.
- If between 0.20 and 0.40 → marginal. Try Option B (PWM ISR) before deciding.

### Phase 2 — `FALLING_ONLY` live (no DUAL yet)

Build with:
```c
#define FEATURE_ZC_MODE                  ZC_MODE_FALLING_ONLY
#define FEATURE_ZC_MODE_DIAGNOSTIC_ONLY  0                     // SCCP1 writes the sink
```

Now SCCP1 owns the sink for falling sectors. ADC ISR is muted (per the gate in 5.4). Rising sectors will be extrapolated by the existing predictor — same 3-of-6 mechanism we already have, just inverted polarity.

Test sequence on A2212 12V no-load:
1. Cold start → arm → start. Check that ramp-out into CL succeeds.
2. Hold low pot (15-25%) for 30s. Confirm CL is stable.
3. Slow ramp pot 0→100%. Watch for the moment of regression (where ON-mid would have been better). Note the duty.
4. Aggressive pot 0→100%→0. Look for desyncs.

What to look for:
- CL entry is **better with prop than today** (the original observation).
- Above ~50% duty, motor may regress vs phase 0 — that's expected (we removed ON-mid). This phase is not for max-eRPM testing.

### Phase 3 — `DUAL` mode

```c
#define FEATURE_ZC_MODE                  ZC_MODE_DUAL
#define FEATURE_ZC_MODE_DIAGNOSTIC_ONLY  0
```

Both ISRs write, sector polarity routes. Expectation: 6-of-6 hardware detection, no extrapolation.

Test sequence:
1. Repeat phase 2 sequence — CL entry should be at least as good as phase 2.
2. Push to peak eRPM. Should match or beat phase 0 (because rising sectors retain ON-mid quality).
3. With prop on A2212 12V: full pot sweep. CL entry stability is the headline metric.
4. With prop on 2810 18V: same sweep.

What to look for:
- Peak eRPM ≥ phase 0.
- `pbFallingAcc + adcSetRising` ≈ total commutations (because both samplers fire). The split should be ~50/50.
- Telemetry `commutateNoCapture` counter should be **strictly lower** than phase 0 — that's the win condition. Each missed-capture commutation in phase 0 is one we're now hardware-detecting.

### Phase 4 — duty-adaptive override (optional, DUAL only)

Once `DUAL` is stable, the duty-based override discussed earlier (low duty → falling-only, high duty → rising-only) can be added in `_ADCInterrupt` and `_CCT1Interrupt` as a second-tier mute. This is a layered optimization, not a separate mode — do not start here. The sector-polarity routing in `DUAL` is the load-bearing logic.

---

## 7. Build matrix

Before any merge: confirm all four builds compile clean.

```bash
cd garuda_6step_ck.X
for prof in 0 1 2 3; do
  for mode in 0 1 2; do                # ZC_MODE_RISING_ONLY / FALLING_ONLY / DUAL
    make -f Makefile-cli.mk clean
    make -f Makefile-cli.mk MOTOR_PROFILE=$prof EXTRA_CFLAGS="-DFEATURE_ZC_MODE=$mode" \
      || { echo FAIL: prof=$prof mode=$mode; exit 1; }
  done
done
```

(`Makefile-cli.mk` does not currently honor `EXTRA_CFLAGS` — verify and either add it or set the flag in `garuda_config.h` between builds.)

Add `RISING_ONLY` byte-identity check: `objdump -d` the resulting ELF for `_ADCInterrupt` and `_CCT1Interrupt` against the phase-0 baseline. They must be identical at the instruction level when `FEATURE_ZC_MODE = RISING_ONLY`.

---

## 8. Rollback

Three escape hatches, in increasing order of severity:

1. **Set `FEATURE_ZC_MODE = ZC_MODE_RISING_ONLY` in `garuda_config.h`** and rebuild. Compile-time mute — the new ISR is dead code, original behavior restored. Use for: any unexplained regression in routine bench testing.

2. **Set `FEATURE_ZC_MODE_DIAGNOSTIC_ONLY = 1`** with mode still set to FALLING_ONLY or DUAL. The new ISR runs but does not write the sink. Use for: tracking intermittent issues — counters keep collecting data while motor is back on the known-good path.

3. **Revert the commit.** All changes here are additive (new counters, new flag, new ISR body) plus a single mode-gated block in `_ADCInterrupt`. A clean revert is safe.

---

## 9. What this plan does NOT solve

- The `currentRisingZc` stuck-true mystery from 2026-04-17 remains unsolved. We are not fixing it; we are sidestepping it by reading `v5_ptgExpectedComp` instead and by only operating the new path at low-duty / CL-entry where prior research showed 45% capture works.
- High-speed (>100k eRPM) behavior. `DUAL` may help slightly via the rising-sector path being unchanged; the falling-sector additions are not expected to do useful work above ~50% duty regardless of motor.
- The real cause of the prop-startup fragility documented in `docs/v4_prop_startup_investigation.md`. This plan addresses the *sampling* side; if the underlying issue is in the PI gain schedule or block-commutation handoff, the new sample point won't fix it. If phase 2 / phase 3 testing shows the same prop-startup symptom at the same duty, switch tracks back to the post-ZC convention investigation.

---

## 10. Memory updates after this lands

When the plan is implemented and the phase-3 `DUAL` build is bench-validated, update:
- `memory/MEMORY.md` — add a one-line entry pointing to the new memory file.
- `memory/v6_dual_zc_results.md` (new) — record peak eRPM, capture-rate splits, prop CL-entry behavior, and any issues encountered.
- `memory/ck_offmid_falling_zc_research.md` — append a closing section noting that the period-boundary path is now the proven solution, and which prior failure modes turned out to be sample-point artifacts vs sector-flag artifacts.

If the plan is *attempted but does not work* (any phase fails its decision gate), record a memory file describing what didn't work and why — that's just as valuable as a success memory.

---

## Appendix A — register reference

For section 5.4 / Option B, the relevant dsPIC33CK PWM control bits (verify against the device datasheet — comments in `hal_pwm.c:138` are partially wrong):

| Field | Register | Bits | Meaning |
|---|---|---|---|
| `IEVTSEL[2:0]` | `PG1EVTL` | 7:5 | PWM ISR event source (000=disabled, 001=EOC, 010=PG1TMR=PG1TRIGA, 011=TRIGB, 100=TRIGC) |
| `PGTRGSEL[2:0]` | `PG1EVTL` | 2:0 | PWM Trigger output to other peripherals (NOT the same as IEVTSEL — this is what 0x0118 calls EOC) |
| `ADTR1EN1/2/3` | `PG1EVTL` | 4,3,? | Enable PG1 trigger to ADC trigger 1 |
| `ADTR2EN1` | `PG1EVTH` | 6 | Enable PG1 TRIGA to ADC trigger 2 (per 2026-04-17 finding, original comment was wrong) |
| `PWM1IF/IE/IP` | `IFS4`/`IEC4`/`IPC16` | bit 12 / bit 12 / 14:12 | Vector for `_PWM1Interrupt` |
| `CCT1IF/IE/IP` | `IFS0`/`IEC0`/`IPC0` | bit 13 / bit 13 / 6:4 | Vector for `_CCT1Interrupt` |

When in doubt: scope the trigger pin (PG1TRIG output via PPS) and the comparator pin together to confirm what fires when.

---

## Appendix B — quick-reference symbol map

| Symbol | Where | What it is |
|---|---|---|
| `v4_lastCaptureHR` | `hal_capture.c:18` | Latest accepted ZC timestamp (HR ticks, 640 ns) |
| `v4_captureValid` | `hal_capture.c:19` | Sticky flag: a valid capture is ready for the consumer |
| `v4_blankingEndHR` | `garuda_service.c:606` | HR time before which captures are rejected |
| `v5_ptgExpectedComp` | (extern) | Per-sector expected post-ZC comparator value, written by Commutate |
| `currentRisingZc` | `hal_capture.c:22` | Sector polarity flag — DO NOT read via `HAL_Capture_IsRisingZc()` from new ISRs |
| `ReadBEMFComp()` | `garuda_service.c:595` | Inline raw GPIO read of the floating phase comparator |
| `CCP4TMRL` | hardware | The 640 ns HR timer (read for current time) |
| `v4Params.blankingPct` | `motor/v4_params.c` | Blanking width as % of sector period |
| `SectorPI_GetPhase()` | `motor/sector_pi.c` | Returns 3 in CL (gate condition for the new ISR) |
| `v4_spActive` | (extern) | Single-pulse mode active — gate the new ISR off when set |

---

End of plan.
