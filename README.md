# Project Garuda

**Open-source drone ESC firmware for the Microchip dsPIC33AK128MC106**

6-step trapezoidal BLDC motor control with sensorless ADC-based BEMF zero-crossing detection — built from scratch for high-performance multirotor applications.

---

## Current Status

**Motor runs stable closed-loop across the full speed range (1,300 - 18,500 eRPM) with zero desyncs and zero timeout-forced commutations.**

| Metric | Value |
|--------|-------|
| **Program flash** | ~13 KB (9%) of 128 KB |
| **Data RAM** | ~200 bytes (1%) of 16 KB |
| **Source files** | 16 source, 19 headers |
| **Speed range** | 1,300 - 18,500 eRPM (pot-controlled) |
| **Desyncs in testing** | 0 |
| **Timeout-forced comms** | 0 |
| **Motor** | Hurst DMB0224C10002 (10-pole, 24V) |
| **Dev board** | MCLV-48V-300W (Microchip) |

### Implemented Phases

| Phase | Description | Status |
|-------|-------------|--------|
| **Phase 1** | Hardware bring-up, open-loop forced commutation | HW verified |
| **Phase 2** | ADC-based BEMF zero-crossing closed-loop commutation | HW verified |
| **Phase A** | Safety baseline (arming gate, fault protection, Vbus OV/UV) | HW verified |
| **Phase B** | Control improvements (duty slew, desync recovery, timing advance) | HW verified |
| **Phase C** | Dynamic blanking, Vbus sag power limiting | HW verified |
| **Phase E** | BEMF integration shadow estimator (validation mode) | HW verified |

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
#define FEATURE_BEMF_INTEGRATION 1  /* Phase E: Shadow integration */
```

Setting any flag to 0 completely removes that feature from the build (dead code elimination).

---

## Target Hardware

| Parameter | Value |
|-----------|-------|
| **MCU** | dsPIC33AK128MC106 (32-bit, hardware FPU) |
| **Core** | 200 MHz DSP |
| **Flash / RAM** | 128 KB / 16 KB |
| **ADC** | Dual-core, 12-bit, 24 kHz sample rate (PWM-triggered) |
| **PWM** | 3 generators, 400 MHz timebase, complementary with dead-time |
| **Dev Board** | MCLV-48V-300W (Microchip) |
| **Motor** | Hurst DMB0224C10002 (10-pole, 24 V, 1.0 A rated) |
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
│   ├── hal_pwm.c/.h              24kHz center-aligned PWM, 6-step override control
│   ├── hal_adc.c/.h              BEMF/Vbus/Pot ADC channels, floating phase muxing
│   ├── hal_comparator.c/.h       CMP1/2/3 with DAC (initialized, not in control loop)
│   ├── board_service.c/.h        Peripheral init, button debounce, PWM enable/disable
│   ├── timer1.c/.h               100us tick timer (prescaler 1:8, 100MHz clock)
│   ├── uart1.c/.h                UART1 8N1 (for future GSP/debug)
│   └── delay.h                   Blocking delay using libpic30
│
├── motor/
│   ├── commutation.c/.h          6-step commutation table + step advance
│   ├── startup.c/.h              Alignment hold + open-loop forced ramp
│   ├── bemf_zc.c/.h              BEMF zero-crossing detection + integration shadow
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
    |               [ALIGN]       hold step 0 at 20% duty for 500ms
    |                    |
    |                    v
    |              [OL_RAMP]      forced commutation 300 -> 2000 eRPM
    |                    |
    |                    | ZC sync lock (6 consecutive valid ZCs)
    |                    v
    |            [CLOSED_LOOP]    ADC-based BEMF ZC commutation
    |                    |
    | SW1 press          |
    +--------------------+
    |                    |
    |                    | desync detected
    |                    v
    |             [RECOVERY]      200ms coast, then auto-restart (max 3x)
    |                    |
    |  overcurrent/OV/UV |  3x restart fail
    +----[FAULT]<--------+
         SW1 clears fault -> [IDLE]
```

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

## Further Testing

### Tests Completed

- Full speed range pot sweep (min to max) — stable, zero desyncs
- Rapid pot transients (short bursts) — IIR threshold smoothing handles well
- Shadow integration estimator at 5 speed points — all pass >90% hit rate
- Arming gate — verified: motor won't start with throttle above 5%
- Direction change restriction — verified: blocked outside IDLE state
- Board fault PCI — active, no false triggers (injection not tested)
- Vbus OV/UV — active, no false triggers (injection not tested)

### Tests Recommended

| Test | What to Check | How |
|------|--------------|-----|
| **Vbus fault injection** | OV/UV trip and recovery | Vary supply voltage above 52 V / below 7 V |
| **PCI fault injection** | Overcurrent path | Inject signal on DIM040 pin |
| **Load testing** | Shadow accuracy under mechanical load | Hold motor shaft at various speeds |
| **Soak test** | Long-duration stability | Run at mid speed for 30+ minutes |
| **Sustained pot oscillation** | Known weakness: prolonged rapid back-and-forth can cause drift | Jerk pot rapidly for >10 seconds |
| **Cold start** | Startup at low Vbus | Test alignment + ramp at minimum viable voltage |
| **Thermal** | Temperature rise at max speed | Run at 100% for 10 minutes, monitor FET temperature |
| **Direction reversal** | Clean stop → reverse → start | Use SW2 in IDLE, verify correct rotation |

---

## Roadmap

### Completed

- **Phase 1**: Hardware foundation, open-loop forced commutation
- **Phase 2**: ADC-based BEMF closed-loop commutation
- **Phase A**: Safety baseline (arming, fault protection, Vbus monitoring)
- **Phase B**: Control improvements (duty slew, desync recovery, timing advance)
- **Phase C**: Dynamic blanking, Vbus sag power limiting
- **Phase E**: BEMF integration shadow estimator (validated, >99% accuracy at high speed)

### Next Up

- **Phase D: Sine Startup** — Replace forced trapezoidal ramp with sinusoidal drive for smooth, quiet low-speed startup. Eliminates the mechanical jerk during alignment and ramp. Required for production drones (acoustic signature matters).

### Future

- **DShot Protocol** (Phase 3) — DShot150/300/600/1200 decoding via Input Capture (ICM1 pin already mapped). Auto-rate detection, full command set (CMD 0-47), bidirectional DShot with GCR encoding.
- **Hybrid Commutation** — Promote integration estimator from shadow to active commutation (requires further validation on production motor/board). Threshold ZC as fallback.
- **Garuda Serial Protocol (GSP)** — UART-based configuration and telemetry protocol. Web-based configurator using WebSerial API.
- **64-byte EEPROM Config** — Persistent configuration storage with CRC-16 integrity check.
- **Thermal Monitoring** — Linear duty derating based on temperature.
- **Signal Loss Failsafe** — Auto-disarm on DShot signal loss.
- **Motor Auto-Detection** — Measure resistance, inductance, and Ke at startup to auto-tune parameters.
- **Production ESC Board** — Custom PCB designed for the dsPIC33AK128MC106, replacing the MCLV-48V-300W dev board.

---

## Architecture Notes

### ISR Map

| ISR | Source | Rate | Purpose |
|-----|--------|------|---------|
| `_AD1CH0Interrupt` | ADC1 CH0 complete | 24 kHz | BEMF sampling, ZC detection, integration, commutation, Vbus/pot reads |
| `_T1Interrupt` | Timer1 | 10 kHz (100 us) | Heartbeat, button debounce, state machine (arm/align/ramp), duty slew, systemTick |
| `_PWM1Interrupt` | PWM fault PCI | On fault | Immediate PWM shutdown, set ESC_FAULT |

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

- **ADC-based ZC, not comparator-based**: The MCLV-48V-300W DIM board's phase voltage divider pins are not routable to comparator inputs. ADC threshold detection is used instead, with software deadband and majority filter.
- **Native float math**: The dsPIC33AK has a hardware FPU. No fixed-point (Q8.8) used — all runtime calculations use native `float` where needed.
- **Shadow-first integration**: The BEMF integration estimator runs as a shadow alongside threshold ZC, validating its accuracy before any future promotion to active control.
- **Feature flags everywhere**: Every phase is gated behind `#if FEATURE_*` flags. Setting a flag to 0 dead-code-eliminates the entire feature with zero overhead.

---

## Reference

This project adapts the HAL layer from **Microchip AN1292** (FOC-PLL for dsPIC33AK128MC106) and incorporates design lessons from:

| Firmware | Key Lesson |
|----------|-----------|
| **BLHeli_S** | Demag compensation, DShot bit-timing |
| **VESC** | Proportional current limiting, packet protocol |
| **Sapog** | Least-squares ZC regression |
| **Bluejay** | Bidirectional DShot + EDT |
| **AM32** | Multi-MCU DShot patterns |

---

## License

TBD
