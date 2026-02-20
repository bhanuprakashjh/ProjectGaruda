# Project Garuda

**Open-source drone ESC firmware for the Microchip dsPIC33AK128MC106**

6-step trapezoidal BLDC motor control with sensorless ADC-based BEMF zero-crossing detection and sinusoidal V/f startup — built from scratch for high-performance multirotor applications.

---

## Current Status

**Motor runs stable closed-loop across the full speed range with zero desyncs. Sinusoidal V/f startup with waveform morphing enables reliable prop-loaded startup on the A2212 1400KV drone motor. Hardware-accelerated ZC detection via ADC digital comparator at 1 MHz with 4x oversampling. Three-level hardware overcurrent protection via CMP3 CLPCI chopping + software soft limiter + board FPCI backup.**

| Metric | Value |
|--------|-------|
| **Program flash** | ~20 KB (15%) of 128 KB |
| **Data RAM** | ~338 bytes (2%) of 16 KB |
| **Source files** | 24 source, 27 headers |
| **Speed range** | 1,300 - 18,500 eRPM (Hurst), idle - 120,000 eRPM (A2212) |
| **HW ZC detections** | 80,000+ total, 0 misses (0.000%) |
| **Desyncs in testing** | 0 |
| **Motors tested** | Hurst DMB0224C10002 (10-pole, 24V), A2212 1400KV (14-pole, 12V) |
| **Prop-loaded startup** | Verified: A2212 + 8x4.5 prop at 12V, 5A supply |
| **Dev board** | MCLV-48V-300W (Microchip) |

### Implemented Phases

| Phase | Description | Status |
|-------|-------------|--------|
| **Phase 1** | Hardware bring-up, open-loop forced commutation | HW verified |
| **Phase 2** | ADC-based BEMF zero-crossing closed-loop commutation | HW verified |
| **Phase A** | Safety baseline (arming gate, fault protection, Vbus OV/UV) | HW verified |
| **Phase B** | Control improvements (duty slew, desync recovery, timing advance) | HW verified |
| **Phase C** | Dynamic blanking, Vbus sag power limiting | HW verified |
| **Phase D** | Sinusoidal V/f startup with waveform morph transition | HW verified (Hurst + A2212 w/ prop) |
| **Phase E** | BEMF integration shadow estimator (validation mode) | HW verified |
| **Phase F** | ADC digital comparator high-speed ZC (1 MHz, 4x oversampling) | HW verified |
| **Phase G** | Hardware overcurrent protection (CMP3 CLPCI + SW soft limiter) | HW verified |

---

## Features

### Phase 1 — Hardware Foundation

- **200 MHz PLL clock** with dedicated domains: 400 MHz PWM, 100 MHz ADC, 400 MHz comparator
- **24 kHz center-aligned complementary PWM** with 750 ns dead time on 3 half-bridge generators
- **6-step commutation** with per-phase override control (PWM active / LOW sink / high-Z float)
- **Dual-core 12-bit ADC**: BEMF phase voltages, Vbus, potentiometer (speed reference)
- **Open-loop startup**: alignment hold (500 ms, 20% duty) followed by forced ramp (300 to 2,000 eRPM)
- **Hardware fault protection** via external PCI overcurrent pin (DIM040/RP28)
- **Button control**: SW1 start/stop, SW2 direction change
- **Heartbeat LED** at 2 Hz, motor-running indicator LED

### Phase 2 — Closed-Loop BEMF Commutation

- **ADC-based zero-crossing detection** with software threshold tracking (`Vbus * duty / 2`)
- **Per-phase gain/offset correction** for ADC channel mismatch compensation
- **Majority filter** with speed-adaptive filter count (reduces at high eRPM for faster response)
- **Deadband hysteresis** around threshold to reject noise
- **AD2 mux settle delay** (2 samples) for shared ADC channel between Phase B and C
- **IIR step period smoothing** (3/4 old + 1/4 new) for stable commutation timing
- **Commutation deadline engine**: ZC-to-commutation delay = `stepPeriod * (30 - advanceDeg) / 60`
- **Forced-step timeout**: `2 * stepPeriod` safety net (never triggered in normal operation)
- **Sync detection**: 6 consecutive valid ZCs declare lock, transitions from ramp to closed-loop
- **Per-step miss tracking**: fallback to timing-based deadline after 2 consecutive misses on any step
- **Asymmetric IIR threshold smoothing**: rise tau = 0.33 ms, fall tau = 10.7 ms — prevents ZC loss during rapid throttle transients
- **Speed governor**: duty frozen when `stepPeriod` hits minimum floor

### Phase A — Safety Baseline

- **A1: Arming gate** — throttle must be below 200 ADC (~5%) for 500 ms before motor start. Prevents accidental spin-up.
- **A2: Direction change restricted** — only allowed in IDLE state. Changing direction while spinning is blocked.
- **A3: Board fault PCI** — external overcurrent via DIM040/RP28. Immediate PWM shutdown, latched until SW1 clear.
- **A4: Vbus OV/UV enforcement** — 3-sample filter prevents false triggers. Overvoltage (>52 V) and undervoltage (<7 V) fault codes.

### Phase B — Control Improvements

- **B1: Asymmetric duty slew rate** — 2%/ms ramp-up, 5%/ms ramp-down. Prevents current spikes on acceleration, allows fast braking response.
- **B2: Desync recovery** — on desync fault, enters ESC_RECOVERY state: 200 ms coast-down, then automatic restart. Maximum 3 restart attempts before permanent fault.
- **B3: Timing advance** — linear 0-15 degrees by eRPM. Compensates for commutation delay at high speed, improves efficiency and top-end performance.

### Phase C — Advanced ZC Protection

- **C1: Dynamic blanking** — blanking window scales with speed (3% of step period base) plus extra blanking at high duty (>70%) to suppress demagnetization noise. Hard cap at 25% of step period.
- **C2: Vbus sag power limiting** — monitors bus voltage and proportionally reduces duty when Vbus dips below threshold (900 ADC). Hysteresis band prevents oscillation. Protects against brown-out under load.

### Phase D — Sinusoidal V/f Startup with Waveform Morph Transition

Replaces the trapezoidal forced-commutation startup with a smooth sinusoidal 3-phase drive and a two-phase waveform morph that transitions from sine to 6-step commutation without any coast gap or voltage discontinuity. The rotating field is inherently load-tolerant (like a stepper motor), enabling reliable startup under prop load where blind forced commutation stalls.

**The problem this solves**: On the A2212 1400KV with an 8x4.5 propeller at 12V, the standard forced 6-step open-loop ramp stalls — blind commutation outpaces the rotor under load. Even the original sine startup with a coast gap (2-4 ms of zero voltage) caused a torque discontinuity and phase alignment error at the sine-to-trap boundary, leading to ZC sync failure.

**How it works — 5-stage startup sequence:**

1. **Sine Alignment** (ESC_ALIGN, 500 ms): All three phases driven sinusoidally at a fixed electrical angle (90 degrees = Phase A peak). The rotor locks smoothly to a known position without the mechanical snap of single-phase energization.

2. **Sine V/f Ramp** (ESC_OL_RAMP): The electrical angle advances at increasing frequency from `INITIAL_ERPM` to `RAMP_TARGET_ERPM`. Voltage amplitude scales linearly with frequency (constant V/f ratio) to maintain constant flux. A low-frequency boost overcomes static friction. Current-gated acceleration pauses the ramp when `ibusRaw > RAMP_CURRENT_GATE_MA`, preventing overdrive stall under prop load.

3. **Waveform Morph — Sub-phase A: Duty Convergence** (ESC_MORPH, ~30 ms): All 3 phases remain driven via `HAL_PWM_SetDutyCycle3Phase()` (no Hi-Z yet). Sine duties are smoothly blended toward the 6-step duty pattern using a Q8 blend factor alpha (0 to 256):

   ```
   dutyX = sine_dutyX * (256 - alpha) / 256 + trap_targetX * alpha / 256
   ```

   For each 60-degree sector (tracked from the sine angle), the 6-step trap targets are: active phase = trap duty, low phase = MIN_DUTY, float phase = SINE_CENTER_DUTY (50%, near zero net current). Alpha is computed from sector count with rounding (not repeated addition) to guarantee alpha = 256 exactly at `MORPH_CONVERGE_SECTORS` (6 sectors = 1 electrical cycle). Software overcurrent limiting proportionally scales all 3 duties when `ibusRaw > OC_SW_LIMIT_ADC`.

4. **Waveform Morph — Sub-phase B: Progressive Hi-Z** (ESC_MORPH, ~5-36 sectors): The critical transition — switches from 3-phase PWM to actual 6-step commutation overrides:
   - **Float phase** → Hi-Z (OVRENH=1, OVRENL=1, OVRDAT=0b00 = both FETs off)
   - **Active phase** → PWM (OVRENH=0, OVRENL=0 = complementary mode)
   - **Low phase** → Sink (OVRENH=1, OVRENL=1, OVRDAT=0b01 = L-FET on)

   BEMF zero-crossing detection starts on the floating phase. `BEMF_ZC_Poll()` runs unchanged. Commutation is **forced** (open-loop timing from `rampStepPeriod`), NOT ZC-driven — the ZC detector runs as an observer only. An IIR filter adapts `stepPeriod` from measured ZC-to-ZC intervals (single-step spans only — multi-step spans are rejected to prevent the forced-step inflation death spiral).

   **Exit condition**: `goodZcCount >= MORPH_ZC_THRESHOLD && risingZcWorks && fallingZcWorks && newZcThisTick` — the motor transitions to ESC_CLOSED_LOOP with a hot handoff that preserves all ZC state (no re-initialization). The first CL commutation deadline is seeded from the last confirmed ZC timestamp.

   **Partial-lock fallback**: If the exit gate isn't satisfied but `goodZcCount >= 3` at the sector timeout (36 sectors = 6 electrical cycles), the motor enters CL anyway — the hardware ZC detector (HWZC) takes over at higher speed and compensates for incomplete SW ZC lock.

   **Fault**: If `goodZcCount < 3` at timeout or `MORPH_TIMEOUT_MS` (2000 ms) elapses → `FAULT_MORPH_TIMEOUT`, PWM disabled.

   **Staleness decay**: If `stepsSinceLastZc > ZC_STALENESS_LIMIT`, goodZcCount and polarity flags are reset to zero — preventing stale ZC history from satisfying the exit gate.

5. **Post-sync slow slew**: After ZC sync is declared, the duty slew-up rate is reduced to 1/4th of normal (0.5%/ms instead of 2%/ms) for 1000 ms (`POST_SYNC_SETTLE_MS`). This prevents low-inertia motors from accelerating faster than the stepPeriod IIR filter can track.

**ISR architecture:**
- **ADC ISR (24 kHz)** owns the sine waveform (stages 1-2) and the morph state machine (stages 3-4): duty blending, sector tracking, ZC polling, forced commutation countdown, exit gate evaluation.
- **Timer1 ISR (10 kHz)** owns the V/f ramp profile (stage 2) and triggers the OL_RAMP → MORPH transition when `STARTUP_SineRamp()` returns true.

**Key implementation details:**
- **256-entry Q15 sine lookup table** (512 bytes flash) — power-of-2 for shift-based indexing
- **Q16 fixed-point angle** (0 = 0 degrees, 65535 = ~360 degrees) — wraps naturally with `uint16_t` overflow
- **Q16 fractional eRPM accumulator** — sub-Hz ramp resolution without floating point
- **Direction-aware**: angle advances forward for CW, backward for CCW; sector mapping mirrors for CCW
- **Phase ordering A->C->B** matches the commutation table CW sequence (not the intuitive A->B->C)
- **Safe sector mapping**: `((uint32_t)adj * 6) >> 16` always produces 0-5 (never 6)
- **Sector-coherent trap targets**: `MorphComputeDuties()` re-reads the sector from the current sine angle (after `SineComputeDuties()` advances it) rather than using the cached `morphStep` — prevents a one-tick mismatch at 60-degree boundaries
- **rampStepPeriod sync**: `MorphInit()` computes `rampStepPeriod` from the actual sine eRPM (`erpmFrac >> 16`) because `SineRamp()` never touches `rampStepPeriod` — without this, forced commutation in Hi-Z would run 10x too slowly
- **prevAdcState race fix**: `entryState` is captured at ADC ISR start, saved to `prevAdcState` at ISR end (not `garudaData.state`). Without this, the morph→CL transition would be invisible to the next tick's entry-detection checks
- **Hot handoff guard**: CL entry gates on `prevAdcState == ESC_MORPH && zcSynced` (not just `zcSynced`) to prevent stale state from a previous run triggering the hot-handoff path after a stop→restart cycle

**Tuning parameters** (in `garuda_config.h`):
- `SINE_ALIGN_MODULATION_PCT` — modulation depth during alignment (Hurst: 15%, A2212: 4%)
- `SINE_RAMP_MODULATION_PCT` — modulation depth at target speed (Hurst: 35%, A2212: 8%)
- `SINE_PHASE_OFFSET_DEG` (60 degrees) — sector-to-step calibration offset
- `SINE_TRAP_DUTY_NUM` / `SINE_TRAP_DUTY_DEN` (6/5) — sine-to-trap duty scale factor, compensates for sine-to-trap L-L voltage conversion
- `MORPH_CONVERGE_SECTORS` (6) — sectors for duty convergence (1 electrical cycle, ~30 ms at 2000 eRPM)
- `MORPH_HIZ_MAX_SECTORS` (36) — max sectors in Hi-Z before timeout (6 electrical cycles)
- `MORPH_TIMEOUT_MS` (2000) — absolute morph timeout
- `MORPH_ZC_THRESHOLD` (4) — goodZcCount required to exit morph
- `POST_SYNC_SETTLE_MS` (1000 ms) — reduced slew-up rate period after ZC sync
- `POST_SYNC_SLEW_DIVISOR` (4) — slew-up rate divisor during settle (4 = 0.5%/ms)
- `RAMP_CURRENT_GATE_MA` (5000 for A2212) — current gate pauses ramp acceleration under load

**Feature flag**: `FEATURE_SINE_STARTUP`. Setting to 0 reverts to trapezoidal alignment + forced ramp with zero code overhead.

### Phase E — BEMF Integration Shadow Estimator

A **shadow-only** integration-based commutation estimator that runs alongside the primary threshold ZC detection. It does NOT control the motor — it computes when it *would* commutate and logs diagnostics comparing shadow vs actual commutation timing.

**How it works:**
1. After each commutation, the floating phase BEMF deviates from the virtual neutral threshold
2. The integrator accumulates `|BEMF - threshold|` (polarity-adjusted) each ADC tick after the blanking window
3. When the integral reaches a threshold proportional to `bemfPeakSmooth * delayTicks`, the shadow fires
4. At each actual commutation, the shadow fire tick is compared to the actual tick
5. Statistics are accumulated: hit rate, miss rate, noFire rate, mean absolute error, signed bias

**Timing-advance-aware threshold**: The integration threshold accounts for the actual ZC-to-commutation delay window, which shrinks with timing advance at high speed. Without this correction, the shadow has 90% noFire rate at max speed.

**Test Results (x6 series, Hurst DMB0224C10002, 24 V):**

| Test Point | Pot | eRPM | Step Period | Hit Rate | NoFire Rate | Bias (ticks) |
|------------|-----|------|-------------|----------|-------------|--------------|
| x6min | 1% | ~1,300 | 186 | 97.0% | 2.9% | -1.7 |
| x6min+ | 12% | ~3,000 | 79 | 99.4% | 0.5% | -1.9 |
| x6mid | 27% | ~5,300 | 45 | 98.2% | 1.7% | -1.9 |
| x6mid+ | 50% | ~9,200 | 26 | 99.9% | 0.1% | -1.3 |
| x6max | 100% | ~18,500 | 13 | 99.95% | 0.04% | -0.4 |

All 5 test points pass criteria: **hit rate >90%** and **noFire rate <5%**.

### Phase F — ADC Digital Comparator High-Speed ZC Detection

Adds a hardware-accelerated zero-crossing detection path using the dsPIC33AK's per-channel ADC digital comparator. At high eRPM, the 24 kHz software ZC polling becomes bandwidth-limited (only 2 samples per step at 20K eRPM). Phase F solves this by running dedicated ADC channels at **1 MHz** via an SCCP3 peripheral trigger, with the digital comparator checking each conversion result in hardware — firing an interrupt only when the BEMF threshold is crossed.

**Dual-mode architecture:**
- **Below 5,000 eRPM**: Software ZC in the 24 kHz ADC ISR (unchanged from Phase 2)
- **Above 5,000 eRPM**: Hardware comparator ZC via dedicated 1 MHz ADC channels. The 24 kHz ISR continues running for threshold computation, pot, Vbus, and duty control — but does not drive commutation.

Crossover is automatic with hysteresis (500 eRPM band). Once hardware ZC activates, it stays active for the entire run. Fallback to software ZC occurs only on miss-limit (error condition).

**ADC channel allocation:**
| Channel | Pin | Rate | Purpose |
|---------|-----|------|---------|
| AD1CH5 | RB8 (Phase B) | 1 MHz | Hardware comparator ZC (fixed PINSEL) |
| AD2CH1 | RB9/RA10 (Phase A/C) | 1 MHz | Hardware comparator ZC (muxed per step) |
| AD1CH0, AD2CH0, AD1CH1, AD1CH4 | Various | 24 kHz | Software ZC, pot, Vbus (unchanged) |

**4x hardware oversampling** (MODE=11, ACCNUM=00): Each SCCP3 trigger produces a burst of 4 conversions (820 ns), hardware-averaged via right-shift by 2 bits. The comparator fires on the averaged result, providing **+6 dB noise immunity** without reducing the effective 1 MHz detection rate. `ACCBRST=1` prevents 24 kHz channels from splitting the burst.

**SCCP timer state machine:**
- **SCCP1** (32-bit one-shot): Drives the per-step ZC detection cycle — blanking delay → comparator watch with timeout → commutation delay. Three states, one timer.
- **SCCP2** (32-bit free-running): High-resolution timestamps (10 ns resolution) for ZC interval measurement and step period tracking via IIR filter.
- **SCCP3** (periodic): 1 MHz trigger source for the comparator ADC channels (`TRG1SRC=14`).

**ISR priority scheme** (when Phase F enabled):
| Priority | ISRs | Purpose |
|----------|------|---------|
| 7 (highest) | `_AD1CMP5Interrupt`, `_AD2CMP1Interrupt`, `_CCT1Interrupt` | ZC detection + commutation timing |
| 6 | `_AD1CH0Interrupt` | 24 kHz BEMF/threshold/duty (lowered from 7) |
| 5 | `_T1Interrupt` | Heartbeat, state machine, V/f ramp |

**Safety features:**
- `HWZC_Disable()` called on all exit-CL paths (faults, button stop, throttle-zero shutdown)
- ISR guards prevent stale timer/comparator events from commutating after disable
- `fallbackPending` flag enables clean software ZC re-seed from the ADC ISR
- Seqlock on `stepPeriodHR` (uint32_t) for tear-free reads across ISR priorities
- Throttle-zero shutdown: if pot returns to zero after being raised, motor stops gracefully

**Hardware verification results (Hurst DMB0224C10002, MCLV-48V-300W):**

| Test | Mode | eRPM | HW ZC Count | Misses | Timing Advance |
|------|------|------|-------------|--------|----------------|
| x13min | N/A (SW ZC) | ~4,138 | 0 | N/A | 2° |
| x13mid | Single sample | ~8,300 | 10,306 | 0 | 5° |
| x13midplus | Single sample | ~8,700 | 9,003 | 0 | 5° |
| x13max | Single sample | ~16,460 | 22,405 | 0 | 12° |
| x14mid | 4x oversample | ~10,000 | 17,781 | 0 | 6° |
| x14high | 4x oversample | ~18,650 | 20,741 | 0 | 13° |

**80,236 total HW ZC detections with zero misses (0.000%).** Theoretical capability: 300K+ eRPM (33 averaged results per step at 1 MHz).

**Feature flag**: `FEATURE_ADC_CMP_ZC`. Setting to 0 removes all hardware ZC code. Requires `FEATURE_BEMF_CLOSED_LOOP`.

### Phase G — Hardware Overcurrent Protection

Three-level overcurrent protection using the on-board CMP3 comparator, OA3 current sense amplifier, and software monitoring. Protects against overcurrent at all operating points — startup, morph transition, and closed-loop.

**Protection levels:**

1. **CMP3 CLPCI Cycle-by-Cycle Chopping** (hardware, <100 ns response): CMP3 monitors the amplified bus current (OA3 output on RA5). When current exceeds the DAC threshold, CMP3 triggers the PWM current-limit PCI (CLPCI) which truncates the active PWM pulse for that cycle. The motor continues running — this is non-destructive current clamping. Two DAC thresholds:
   - **Startup threshold** (`OC_STARTUP_MA`): High during alignment and ramp — lets the power supply's CC limit be the primary limiter (e.g., 22A for A2212, letting a 10A supply CC-limit naturally)
   - **Operational threshold** (`OC_LIMIT_MA`): Lowered at morph entry or CL entry (e.g., 12A for A2212). Restored to startup threshold on fault paths.

2. **Software Soft Limiter** (24 kHz ADC ISR): Proportionally reduces duty when `ibusRaw > OC_SW_LIMIT_ADC`. The reduction is linear: at the SW limit threshold, duty is unchanged; at the CMP3 threshold, duty is zero. Runs in ESC_MORPH (both sub-phases) and ESC_CLOSED_LOOP. In MORPH_CONVERGE, all 3 phase duties are scaled proportionally.

3. **Software Hard Fault** (`OC_PROTECT_MODE=2`): If `ibusRaw > OC_FAULT_ADC_VAL`, immediate PWM shutdown and `FAULT_OVERCURRENT`. This catches sustained overcurrent that the CLPCI chopping cannot control.

4. **Board FPCI Backup**: The MCLV-48V-300W DIM has a fixed analog overcurrent circuit (~15A) on the PCI8R pin. This triggers a PWM fault PCI (FPCI) interrupt → immediate shutdown → `FAULT_BOARD_PCI`. Independent of all software — works even if firmware hangs.

**CMP3 configuration:**
- Input: OA3OUT (RA5) — bus current amplified by on-board op-amp (0.003 ohm shunt, 24.95x gain, 1.65V bias)
- DAC: 12-bit, updated at runtime via `HAL_CMP3_SetThreshold()`
- Hysteresis: 45 mV (maximum, for noise immunity)
- Digital filter: enabled for glitch rejection
- CLPCI event source: `PSS = 0b11101` (CMP3 output, not RPn pin)

**ADC current sensing:**
- `ibusRaw`: Read from AD1CH4 (RA5/OA3OUT) every 24 kHz ADC cycle
- `ibusMax`: Running maximum, never cleared (diagnostic)
- `clpciTripCount`: Accumulated from PG1/PG2/PG3 CLPCI event flags (diagnostic)

**Tuning parameters** (per motor profile in `garuda_config.h`):

| Parameter | Hurst (24V) | A2212 (12V) |
|-----------|-------------|-------------|
| `OC_LIMIT_MA` | 1,800 | 12,000 |
| `OC_STARTUP_MA` | 18,000 | 22,000 |
| `OC_FAULT_MA` | 3,000 | 18,000 |
| `OC_SW_LIMIT_MA` | 1,500 | 8,000 |
| `RAMP_CURRENT_GATE_MA` | 0 (disabled) | 5,000 |

**Feature flag**: `FEATURE_HW_OVERCURRENT`. Setting to 0 removes all CMP3/OC code. Board FPCI backup remains active regardless.

---

## Feature Flags

All features are individually toggleable via compile-time flags in `garuda_config.h`:

```c
#define FEATURE_BEMF_CLOSED_LOOP 1  /* Phase 2: BEMF ZC detection */
#define FEATURE_VBUS_FAULT       1  /* Phase A4: Bus voltage OV/UV */
#define FEATURE_DESYNC_RECOVERY  1  /* Phase B2: Restart-on-desync */
#define FEATURE_DUTY_SLEW        1  /* Phase B1: Asymmetric duty slew */
#define FEATURE_TIMING_ADVANCE   1  /* Phase B3: Linear timing advance */
#define FEATURE_DYNAMIC_BLANKING 1  /* Phase C1: Speed+duty blanking */
#define FEATURE_VBUS_SAG_LIMIT   1  /* Phase C2: Vbus sag power limit */
#define FEATURE_SINE_STARTUP     1  /* Phase D: Sine startup + waveform morph */
#define FEATURE_BEMF_INTEGRATION 1  /* Phase E: Shadow integration */
#define FEATURE_ADC_CMP_ZC       1  /* Phase F: ADC comparator high-speed ZC */
#define FEATURE_HW_OVERCURRENT  1  /* Phase G: CMP3 CLPCI + SW OC protection */
```

Setting any flag to 0 completely removes that feature from the build (dead code elimination). Compile-time guards enforce dependencies:
- `FEATURE_SINE_STARTUP` requires `FEATURE_BEMF_CLOSED_LOOP` (needs ZC for closed-loop transition)
- `FEATURE_SINE_STARTUP` is incompatible with `DIAGNOSTIC_MANUAL_STEP` (conflicting PWM writes)
- `FEATURE_ADC_CMP_ZC` requires `FEATURE_BEMF_CLOSED_LOOP` (hardware ZC extends the closed-loop path)

---

## Target Hardware

| Parameter | Value |
|-----------|-------|
| **MCU** | dsPIC33AK128MC106 (32-bit, hardware FPU) |
| **Core** | 200 MHz DSP |
| **Flash / RAM** | 128 KB / 16 KB |
| **ADC** | Dual-core, 12-bit: 24 kHz PWM-triggered (control) + 1 MHz SCCP3-triggered (ZC comparator, 4x oversampled) |
| **PWM** | 3 generators, 400 MHz timebase, complementary with dead-time |
| **Dev Board** | MCLV-48V-300W (Microchip) |
| **Motors** | Hurst DMB0224C10002 (10-pole, 24V), A2212 1400KV (14-pole, 12V) |
| **Compiler** | XC-DSC v3.30 |
| **IDE** | MPLAB X v6.30 |
| **DFP** | dsPIC33AK-MC_DFP v1.4.172 |

---

## Project Structure

```
dspic33AKESC/
├── main.c                        Entry point, init sequence, button polling
├── garuda_types.h                All data structures and enums
├── garuda_config.h               User-tunable parameters (feature flags + knobs)
├── garuda_calc_params.h          Derived constants and compile-time checks
├── garuda_service.c/.h           State machine + ADC/Timer1/PWM fault ISRs
├── util.h                        Math utilities (from AN1292 reference)
│
├── hal/
│   ├── clock.c/.h                PLL init: 200MHz sys, 400MHz PWM, 100MHz ADC
│   ├── device_config.c           Fuse pragmas (WDT, JTAG, protection bits)
│   ├── port_config.c/.h          GPIO mapping: PWM, BEMF, LED, buttons, UART, DShot
│   ├── hal_pwm.c/.h              24kHz center-aligned PWM, 6-step override control,
│   │                             3-phase sine duty writes, override release
│   ├── hal_adc.c/.h              BEMF/Vbus/Pot ADC channels, floating phase muxing,
│   │                             high-speed comparator channels (4x oversampled)
│   ├── hal_timer.c/.h            SCCP1 (one-shot), SCCP2 (timestamp), SCCP3 (1MHz trigger)
│   ├── hal_comparator.c/.h       CMP1/2/3 with DAC (initialized, not in control loop)
│   ├── board_service.c/.h        Peripheral init, button debounce, PWM enable/disable
│   ├── timer1.c/.h               100us tick timer (prescaler 1:8, 100MHz clock)
│   ├── uart1.c/.h                UART1 8N1 (for future GSP/debug)
│   └── delay.h                   Blocking delay using libpic30
│
├── motor/
│   ├── commutation.c/.h          6-step commutation table, step advance, ApplyStep
│   ├── startup.c/.h              Trap align+ramp, sine align+V/f ramp, transition
│   ├── bemf_zc.c/.h              BEMF zero-crossing detection + integration shadow
│   ├── hwzc.c/.h                 Hardware ZC state machine (comparator + SCCP1 timer)
│   └── pi.c/.h                   PI controller (from AN1292 reference)
│
└── learn/                        Self-learning modules (disabled, future use)
    ├── telemetry_ring.c/.h       Ring buffer for runtime telemetry
    ├── quality.c/.h              Signal quality scoring
    ├── health.c/.h               Motor health tracking
    └── adaptation.c/.h           Adaptive parameter tuning
```

---

## State Machine

```
           SW1 press
  [IDLE] -----------> [ARMED]
    ^                    |
    |                    | throttle < 200 ADC for 500ms
    |                    v
    |               [ALIGN]       sine: 3-phase lock at 90deg / trap: step 0 at duty%
    |                    |        duration: 500ms
    |                    v
    |              [OL_RAMP]      sine: V/f ramp to RAMP_TARGET_ERPM
    |                    |        (trap: forced commutation when SINE_STARTUP=0)
    |                    v
    |              [MORPH]        sub-A: duty blend (sine→trap, alpha 0→256)
    |                    |        sub-B: Hi-Z + BEMF ZC observer (forced comms)
    |                    |
    |                    | ZC lock OR partial-lock timeout
    |                    v
    |            [CLOSED_LOOP]    BEMF ZC commutation (SW or HWZC)
    |                    |
    | SW1 press          |
    +--------------------+
    |                    |
    |                    | desync detected
    |                    v
    |             [RECOVERY]      200ms coast, then auto-restart (max 3x)
    |                    |
    |  overcurrent/OV/UV |  3x restart fail / morph timeout
    +----[FAULT]<--------+
         SW1 clears fault -> [IDLE]
```

---

## Commutation Table

The 6-step trapezoidal commutation table used during closed-loop operation:

| Step | Phase A | Phase B | Phase C | Floating | ZC Polarity |
|------|---------|---------|---------|----------|-------------|
| 0 | PWM | LOW | FLOAT | C | Rising |
| 1 | FLOAT | LOW | PWM | A | Falling |
| 2 | LOW | FLOAT | PWM | B | Rising |
| 3 | LOW | PWM | FLOAT | C | Falling |
| 4 | FLOAT | PWM | LOW | A | Rising |
| 5 | PWM | FLOAT | LOW | B | Falling |

CW rotation sequence: A->C->B (steps 0->1->2->3->4->5). This is critical for correct sine phase ordering during Phase D startup (B leads A by +120 degrees, C lags A by -120 degrees).

---

## Pin Allocation

```
dsPIC33AK128MC106 - Project Garuda Pin Map

 MOTOR DRIVE (6 pins)
  RD2 -- PWM1H -- Phase A High     RD3 -- PWM1L -- Phase A Low
  RD0 -- PWM2H -- Phase B High     RD1 -- PWM2L -- Phase B Low
  RC3 -- PWM3H -- Phase C High     RC4 -- PWM3L -- Phase C Low

 BEMF SENSING (3 pins — ADC-based threshold detection)
  RB9  -- AD2AN10 -- Phase A voltage    (ADC2 CH0, PINSEL=10)
  RB8  -- AD1AN11 -- Phase B voltage    (ADC1 CH0, PINSEL=11)
  RA10 -- AD2AN7  -- Phase C voltage    (ADC2 CH0, PINSEL=7, muxed with A)

 ANALOG INPUTS
  RA7  -- AD1AN6  -- DC bus voltage (Vbus)
  RA11 -- AD1AN10 -- Potentiometer (speed reference)

 PROTOCOL I/O
  RD8  -- RP57/ICM1 -- DShot input (Input Capture, future)
  RC10 -- RP43/U1TX -- UART TX
  RC11 -- RP44/U1RX -- UART RX

 FAULT / UI
  RB11 -- RP28/PCI8 -- External overcurrent fault input (DIM040)
  RD5  -- LED1      -- Heartbeat (2Hz blink)
  RC9  -- LED2      -- Motor running indicator
  RD9  -- SW1       -- Start/Stop motor
  RD10 -- SW2       -- Change direction (when idle)
```

---

## Build Instructions

### Prerequisites

- MPLAB X IDE v6.20+ (tested with v6.30)
- XC-DSC compiler v3.21+ (tested with v3.30)
- dsPIC33AK-MC_DFP v1.4.172

### Build from IDE

1. Open `dspic33AKESC/dspic33AKESC.X/` in MPLAB X IDE
2. Build -> Make Project (F11)
3. Output hex at `dist/default/production/dspic33AKESC.X.production.hex`

### Build from Command Line

```bash
cd dspic33AKESC/dspic33AKESC.X
PATH="/path/to/xc-dsc/v3.30/bin:$PATH" make build
```

### Programming

Uses PKoB 4 on-board debugger (MCLV-48V-300W). Program via MPLAB X or MDB CLI:

```bash
mdb.sh <<EOF
device dsPIC33AK128MC106
hwtool pkob4
program dist/default/production/dspic33AKESC.X.production.hex
run
wait 5
quit
EOF
```

**Note**: Use `run` (debug mode), not `reset` — the dsPIC33AK has a known issue where `reset` leaves the chip stuck in boot ROM.

---

## Testing

### Tests Completed

**Hurst DMB0224C10002 (10-pole, 24V, no load):**
- Full speed range pot sweep (min to max) — stable, zero desyncs
- Rapid pot transients (short bursts) — IIR threshold smoothing handles well
- Shadow integration estimator at 5 speed points — all pass >90% hit rate
- Arming gate — verified: motor won't start with throttle above 5%
- Direction change restriction — verified: blocked outside IDLE state
- Board fault PCI — active, no false triggers (injection not tested)
- Vbus OV/UV — active, no false triggers (injection not tested)
- Sine startup — verified: smooth alignment, V/f ramp, morph transition
- Hardware ZC (4x oversampled) — 80,000+ detections, zero misses
- SW-to-HW ZC crossover at 5,000 eRPM — seamless transition

**A2212 1400KV (14-pole, 12V, no prop):**
- Sine startup + morph → CL — reliable, zero faults
- Morph waveform morph — full ZC lock in 9-15 HIZ sectors (no-load), partial-lock fallback when needed
- Full speed range (idle to max) — stable via HWZC, zero desyncs
- CMP3 CLPCI overcurrent chopping — active and functional across all speeds
- SW overcurrent soft limiter — proportional duty reduction verified

**A2212 1400KV (14-pole, 12V, 8x4.5 propeller):**
- Prop-loaded startup — **reliable**, sine ramp completes under load
- Morph full ZC lock in 5-13 HIZ sectors (prop load gives cleaner BEMF than no-load)
- Idle with prop — stable, HWZC running, zero misses, 44K+ commutations
- Min speed with prop — stable, zero misses, 31K+ commutations
- Mid speed with prop — stable, partial-lock→HWZC rescue, 81K+ commutations, zero misses
- Zero desyncs, zero faults across all prop test runs
- CMP3 CLPCI chopping active (1.3-3.3M trips per run, expected for low-impedance motor)

**A2212 prop-loaded test results (X2CScope captures):**

| Test | Morph Sectors | Exit Path | HWZC Comms | HWZC Misses | ibusMax | Desyncs |
|------|--------------|-----------|------------|-------------|---------|---------|
| Idle + prop | 5 | Full ZC lock | 44,534 | 0 | 2,020 | 0 |
| Min speed + prop | 13 | Full ZC lock | 31,749 | 0 | 2,042 | 0 |
| Mid speed + prop | 36 | Partial → HWZC | 81,892 | 0 | 2,041 | 0 |

### Tests Recommended

| Test | What to Check | How |
|------|--------------|-----|
| **Vbus fault injection** | OV/UV trip and recovery | Vary supply voltage above 52 V / below 7 V |
| **PCI fault injection** | Overcurrent path | Inject signal on DIM040 pin |
| **Load testing** | Shadow accuracy under mechanical load | Hold motor shaft at various speeds |
| **Soak test** | Long-duration stability | Run at mid speed for 30+ minutes |
| **Sustained pot oscillation** | Known weakness: prolonged rapid back-and-forth can cause drift | Jerk pot rapidly for >10 seconds |
| **Sine phase offset sweep** | Reduce remaining small transition jerk | Sweep SINE_PHASE_OFFSET_DEG: 0/60/120/180/240/300 |
| **Cold start** | Startup at low Vbus | Test alignment + ramp at minimum viable voltage |
| **Thermal** | Temperature rise at max speed | Run at 100% for 10 minutes, monitor FET temperature |
| **Direction reversal** | Clean stop -> reverse -> start | Use SW2 in IDLE, verify correct rotation |

---

## Roadmap

### Completed

- **Phase 1**: Hardware foundation, open-loop forced commutation
- **Phase 2**: ADC-based BEMF closed-loop commutation
- **Phase A**: Safety baseline (arming, fault protection, Vbus monitoring)
- **Phase B**: Control improvements (duty slew, desync recovery, timing advance)
- **Phase C**: Dynamic blanking, Vbus sag power limiting
- **Phase D**: Sinusoidal V/f startup with waveform morph (smooth sine→trap transition, prop-loaded startup verified)
- **Phase E**: BEMF integration shadow estimator (validated, >99% accuracy at high speed)
- **Phase F**: ADC digital comparator high-speed ZC (1 MHz SCCP3 trigger, 4x oversampling, 80K+ detections with zero misses)
- **Phase G**: Hardware overcurrent protection (CMP3 CLPCI chopping + SW soft limiter + board FPCI backup)

### Future

- **Hybrid Commutation** — Combine all 3 ZC methods: comparator triggers, integration validates, software ZC as fallback. "Comparator fires, integration confirms" architecture.
- **DShot Protocol** (Phase 3) — DShot150/300/600/1200 decoding via Input Capture (ICM1 pin already mapped). Auto-rate detection, full command set (CMD 0-47), bidirectional DShot with GCR encoding.
- **Garuda Serial Protocol (GSP)** — UART-based configuration and telemetry protocol. Web-based configurator using WebSerial API.
- **64-byte EEPROM Config** — Persistent configuration storage with CRC-16 integrity check.
- **Thermal Monitoring** — Linear duty derating based on temperature.
- **Signal Loss Failsafe** — Auto-disarm on DShot signal loss.
- **Motor Auto-Detection** — Measure resistance, inductance, and Ke at startup to auto-tune parameters.
- **Production ESC Board** — Custom PCB designed for the dsPIC33AK128MC106, replacing the MCLV-48V-300W dev board.

---

## Architecture Notes

### ISR Map

| ISR | Source | Priority | Rate | Purpose |
|-----|--------|----------|------|---------|
| `_AD1CMP5Interrupt` | ADC1 CH5 comparator | 7 | On ZC event | Phase B hardware ZC detection (Phase F) |
| `_AD2CMP1Interrupt` | ADC2 CH1 comparator | 7 | On ZC event | Phase A/C hardware ZC detection (Phase F) |
| `_CCT1Interrupt` | SCCP1 timer | 7 | Per step | Blanking expiry, commutation deadline, ZC timeout (Phase F) |
| `_AD1CH0Interrupt` | ADC1 CH0 complete | 6* | 24 kHz | BEMF sampling, SW ZC, integration, sine waveform, Vbus/pot |
| `_T1Interrupt` | Timer1 | 5 | 10 kHz | Heartbeat, button debounce, state machine, duty slew, V/f ramp |
| `_PWM1Interrupt` | PWM fault PCI | 4 | On fault | Immediate PWM shutdown, set ESC_FAULT |

*Priority 6 when `FEATURE_ADC_CMP_ZC=1`; priority 7 when disabled. Phase F ISRs are only enabled above the crossover speed — below crossover, the ADC ISR is effectively highest priority.

### Clock Domains

```
8 MHz FRC -> PLL1 -> 800 MHz VCO

CLK1 = 200 MHz  |  System clock (FCY)
CLK5 = 400 MHz  |  PWM timebase
CLK6 = 100 MHz  |  ADC clock
CLK7 = 400 MHz  |  DAC / Comparator clock
CLK8 = 100 MHz  |  UART clock
```

### Key Design Decisions

- **ADC digital comparator ZC**: The MCLV-48V-300W DIM board's phase voltage divider pins are not routable to analog comparator (CMP) inputs. Instead, the ADC's per-channel digital comparator checks each conversion result against a threshold in hardware — achieving analog-comparator-like event-driven detection using the digital domain. At 1 MHz with 4x oversampling, this provides 33+ averaged results per step even at 300K eRPM.
- **Native float math**: The dsPIC33AK has a hardware FPU. No fixed-point (Q8.8) used — all runtime calculations use native `float` where needed.
- **Shadow-first integration**: The BEMF integration estimator runs as a shadow alongside threshold ZC, validating its accuracy before any future promotion to active control.
- **Feature flags everywhere**: Every phase is gated behind `#if FEATURE_*` flags. Setting a flag to 0 dead-code-eliminates the entire feature with zero overhead.
- **Sine startup for production**: Trapezoidal forced commutation produces audible clunks and stepping noise. Sine V/f drive eliminates these for drone applications where acoustic signature matters. Both startup paths are available via `FEATURE_SINE_STARTUP`.
- **ISR-driven sine, not blocking loops**: Sine waveform updates run in the ADC ISR at 24 kHz (not in a `while` loop with delays). This maintains system responsiveness and allows the Timer1 ISR to handle the V/f ramp profile independently.

---

## Reference

This project adapts the HAL layer from **Microchip AN1292** (FOC-PLL for dsPIC33AK128MC106) and incorporates design lessons from:

| Firmware | Key Lesson |
|----------|-----------|
| **BLHeli_S** | Demag compensation, DShot bit-timing |
| **VESC** | Proportional current limiting, packet protocol |
| **Sapog** | Least-squares ZC regression |
| **Bluejay** | Bidirectional DShot + EDT |
| **AM32** | Sine LUT startup, multi-MCU DShot patterns |

---

## License

TBD
