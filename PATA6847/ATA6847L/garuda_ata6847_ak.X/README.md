# Garuda ESC — ATA6847L (AK port)

Sensorless 6-step trapezoidal BLDC firmware for the
**EV92R69A** (ATA6847L gate driver) + **EV68M17A**
(dsPIC33AK128MC106 DIM) evaluation kit.

Target peak: **226k eRPM** on a 2810 1350KV drone motor at 24 V
(Profile 2, validated on bench).

---

## Quick start

```bash
# CLI build
make -f nbproject/Makefile-default.mk SUBPROJECTS= .build-conf

# MPLAB X: open this directory as a project, F11 to Make
# Motor profile is set via MOTOR_PROFILE in garuda_config.h (default = 2)

# Flash with MDB
/media/bhanu1234/Development/MPLABX/v6.30/mplab_platform/bin/mdb.sh
> program dist/default/production/garuda_ata6847_ak.X.production.elf
> run

# Bench monitor + CSV capture
python3 tools/gsp_ak_test.py mon /dev/ttyACM1 115200 run.csv
```

---

## Repository layout

| Path | Role |
|---|---|
| `main.c` | Boot, main loop, button polling |
| `garuda_config.h` | All compile-time configuration (clock, PWM, per-profile, ADC calibration) |
| `garuda_service.{c,h}` | ESC state machine + Timer1/ADC/PTG ISRs |
| `garuda_types.h` | `ESC_STATE_T`, `PHASE_STATE_T`, `COMMUTATION_STEP_T`, `FAULT_CODE_T` |
| `motor/sector_pi.{c,h}` | Closed-loop sector PI synchronizer + OL ramp + block commutation |
| `motor/motor_params.{c,h}` | Runtime-tunable PI/blanking parameters (GSP-tunable) |
| `motor/commutation.{c,h}` | 6-step commutation table |
| `hal/hal_pwm.{c,h}` | PG1/2/3 setup, override drives, duty + commutation step |
| `hal/hal_ptg.{c,h}` | Peripheral Trigger Generator — fires the BEMF sampler |
| `hal/hal_capture.{c,h}` | Per-sector capture state stubs |
| `hal/hal_com_timer.{c,h}` | SCCP3 one-shot sector timer + SCCP4 HR free-running |
| `hal/hal_ata6847.{c,h}` | SPI register interface to the gate driver |
| `hal/hal_adc.{c,h}` | ADC1/ADC2 channel setup |
| `hal/hal_opa.{c,h}` | Op-amp enable for shunt current sense |
| `hal/hal_timer1.{c,h}` | 50 µs system tick |
| `hal/hal_ak_compat.h` | AK 32-bit SFR ↔ L/H half-word aliases |
| `hal/clock.{c,h}` | PLL setup (Fcy = 200 MHz, CLK5 = 400 MHz) |
| `hal/port_config.{c,h}` | PPS routing + GPIO direction |
| `hal/board_service.{c,h}` | LED + button driver |
| `gsp/gsp.{c,h}` | Binary protocol framing on UART1 |
| `gsp/gsp_commands.{c,h}` | Command dispatch + 250-byte telemetry snapshot builder |
| `gsp/gsp_params.{c,h}` | Param descriptor table for GUI introspection |
| `tools/` | Python host tools (`gsp_ak_test.py`, etc.) |

---

## Architecture

### State machine (`ESC_STATE_T`)

```
   IDLE ──START──▶ ARMED ──▶ ALIGN ──▶ OL_RAMP ──▶ CLOSED_LOOP
                                                        │
                                                        └─▶ FAULT
```

| State | Driver | What runs |
|---|---|---|
| `ESC_IDLE` | main loop | pot/Vbus reads only; PWM module off |
| `ESC_ARMED` | Timer1 | short hold after START to let GDU stabilize |
| `ESC_ALIGN` | Timer1 → `SectorPI_OlTick` | force one sector at low duty; rotor locks to known angle |
| `ESC_OL_RAMP` | Timer1 → `SectorPI_OlTick` | accelerate commutation rate from `INITIAL_STEP_PERIOD` to `MIN_STEP_PERIOD` |
| `ESC_CLOSED_LOOP` | SCCP3 + PTG | sector PI owns commutation; BEMF feeds the PI |
| `ESC_FAULT` | main loop | gates off, PWM module off, awaits `CLEAR_FAULT` |

### ISR map

| ISR | Source | IPL | Rate | Job |
|---|---|---|---|---|
| `_T1Interrupt` | Timer1 match | 4 | 20 kHz | OL ramp tick, 1 ms `SectorPI_TimeTick`, throttle sample |
| `_AD1CH4Interrupt` | ADC1 ch4 done | 3 | ~60 kHz | POT/Vbus/Ia/Ib/Ibus read, mA conversion, peak tracking |
| `_PTG0Interrupt` | PTG IRQ step | 4 | ~60 kHz | `ProcessBemfSample()`: deglitch + comparator classification |
| `_CCT3Interrupt` | SCCP3 period match | 6 | per sector | one-shot guard → `SectorPI_Commutate()` |
| `_CCT2Interrupt` | SCCP2 IC FIFO | 5 | per edge | drains FIFO (timestamps consumed in Commutate) |

Priority 6 (Commutate) cannot be preempted by anything except CPU
traps. PTG (4) and ADC (3) yield to it.

### Hardware timers

| Peripheral | Use |
|---|---|
| **SCCP3** | One-shot sector timer. Fires `_CCT3Interrupt` at the scheduled commutation target. `HAL_ComTimer_ScheduleAbsolute(targetHR)` arms it. |
| **SCCP4** | 1.5625 MHz (640 ns/tick) **HR free-running timer**. All commutation/BEMF timestamps live in this domain — `lastCommHR`, `lastCaptureHR_g`, `targetHR`. |
| **SCCP2** | Input Capture on the floating-phase comparator. Drained by `_CCT2Interrupt`. |
| **PTG** | Hardware step queue triggers `_PTG0Interrupt` at every PG1TRIGB match (PWM mid-OFF or mid-ON, selected per fire by duty). Drives the BEMF sampler. |
| **Timer1** | 50 µs system tick. Drives OL ramp + 1 ms timekeeping. |
| **PG1/PG2/PG3** | PWM generators. PG1 = master (SOC update), PG2/PG3 = slaves. `MPER = LOOPTIME_TCY` (60 kHz). |

---

## Core algorithm — Sector PI synchronizer

The closed-loop control is a **set-point PI synchronizer** modeled on
Microchip's AVR high-speed example. It runs once per commutation in
`SectorPI_Commutate`, called from `_CCT3Interrupt`.

### Loop body (per sector)

```
elapsed     = lastCaptureHR_g - prevCommHR        # HR ticks: prev commutate → ZC
filterHR    = actualStepPeriodHR                  # truth, not PI estimate
if elapsed < filterHR:
    capValue = elapsed
else:
    capValue = SENTINEL                            # reject as cross-sector

setValue    = (advance + 30°) × timerPeriod / 60° + RC_DELAY_HR
delta       = capValue - setValue                  # signed phase error
delta       = clamp(delta, ±timerPeriod / 4)       # ±25% saturation

integrator += delta >> PI_KI_SHIFT                 # Ki = 1/2^N
timerPeriod = integrator + (delta >> PI_KP_SHIFT)  # Kp = 1/2^N

# Schedule next commutation, ZC-anchored:
halfHR      = timerPeriod >> 1
tal         = TAL_band(timerPeriod)                # 2..5 by speed
advHR       = (timerPeriod >> 3) × tal             # advance in HR ticks
delayHR     = halfHR - advHR
targetHR    = lastCaptureHR_g + delayHR
HAL_ComTimer_ScheduleAbsolute(targetHR)
```

### Key state variables

| Name | Domain | Role |
|---|---|---|
| `timerPeriod` | HR ticks | PI output — predicted sector period. Drives `setValue` next loop. |
| `integrator` | HR ticks | PI integrator state — slow tracker of the rotor period. |
| `lastCaptureHR_g` | HR ticks | timestamp of the last accepted ZC (written by BEMF ISR, consumed by Commutate). |
| `actualStepPeriodHR` | HR ticks | measured sector duration (truth — used for blanking & filterHR). |
| `seedIntegrator` | HR ticks | floor for `integrator` during CL_SETTLE — prevents handoff death spiral. |
| `commandEnabled` | bool | latches after 500 ms in CL. Until then, integrator can't fall below `seedIntegrator` and the pot is ignored. |
| `actualAmplitude` | Q15 | duty as 0..32767. Pot writes here via `SectorPI_CommandSet`. |
| `stallCounter` | counts | weighted: −2 on capture, +1 on miss. Hits `STALL_THRESHOLD = 200` → shutoff. |
| `corridorGoodStreak` / `corridorMissStreak` | counts | block-commutation gating. |
| `g_pwmActualDuty` | TCY ticks | last duty actually written to PGxDC (post-clamp). |
| `g_blockCommActive` | bool | true while block-commutation override is engaged. |

### TAL bands (speed-adaptive scheduler advance)

The **scheduler** uses more advance at higher speed because BEMF
latency consumes a larger fraction of a short sector:

| `timerPeriod` (HR) | eRPM band | TAL | advance |
|---|---|---|---|
| ≥260 | ≤60 k | 2 | 15.0° |
| 130–260 | 60–120 k | 3 | 22.5° |
| 100–130 | 120–156 k | 4 | 30.0° |
| <100 | >156 k | 5 | 37.5° |

The **PI's `setValue`** uses a separate fixed advance
(`PHASE_ADVANCE_DEG` — runtime-tunable). Unifying the two destabilizes
the loop.

### Block commutation

Once eRPM stays above `BLOCK_ENTER_ERPM` and the duty governor is fully
saturated, the firmware switches from chopped PWM to solid-on / solid-
off override per sector. The active phase's H FET is driven 100% on
(no chopping); the floating phase stays in BEMF-read mode. Lower
switching loss at top speed. Hysteresis exit at `BLOCK_EXIT_ERPM`
(~0.87× enter).

---

## BEMF detection — PTG-driven midpoint sampler

### Pipeline

```
PG1TRIGB match ──▶ PTG step queue ──▶ _PTG0Interrupt
                                            │
                                            ▼
                                  ProcessBemfSample()
                                       (garuda_service.c)
                                            │
                                            ▼
                              [blanking, deglitch, classify]
                                            │
                                            ▼
                              lastCaptureHR_g, captureValid
                                            │
                                            ▼
                              consumed in SectorPI_Commutate
```

### Duty-adaptive sample position

`PG1TRIGB` is rewritten in the PTG ISR every fire so the BEMF read
stays maximally far from any switching edge:

| Commanded duty | Sample point |
|---|---|
| <50% (−5% hysteresis) | mid-OFF (period boundary, TRIGB = 1) |
| >50% (+5% hysteresis) | mid-ON (TRIGB = `LOOPTIME_TCY − 1`) |

The 3-read deglitch needs ~400 ns of stable signal. Above ~80% duty,
mid-OFF would desync because the OFF window narrows below ISR-latency
+ deglitch fit.

### Acceptance gates

The BEMF ISR runs four gates in order. A sample only feeds the PI
when **all four** pass:

1. **`SectorPI_GetPhase() == 3`** — only run in closed loop.
2. **`!captureValid`** — one accepted capture per sector. After Commutate consumes one, the flag re-arms.
3. **`(nowHR − blankingEndHR) >= 0`** — past blanking. Default blanking = 25% of `actualStepPeriodHR`. Counter: `adcBlankReject`.
4. **3-read deglitch + polarity gate**: majority-of-3 reads of the floating-phase GPIO must equal `expected = 0`. On the **inverted ATA6847**, `comp=0` is PRE-ZC of a falling sector. Rising sectors stay at `comp=1` for the whole sector, so they don't pass this gate → PI is fed from falling sectors only. Counter: `adcStateMismatch`.

When all four gates pass: `lastCaptureHR_g = nowHR; captureValid = true;`

### Why falling-only feed?

`piFeedPolarity` is runtime-tunable (`0 = both`, `1 = rising-only`,
`2 = falling-only`, default `0`). The rising-sector floating phase
has an asymmetry that traps `comp` at the pre-ZC level for the entire
sector. Falling-sector samples cleanly cross neutral. With
`piFeedPolarity = 0` (both) the gate `expected = 0` already selects
falling because rising sectors never satisfy it.

---

## Public APIs

### Motor — `motor/sector_pi.h`

| Function | Called from | Purpose |
|---|---|---|
| `SectorPI_Init` | once at boot | Init HAL, struct defaults, set phase = OFF |
| `SectorPI_Start(vbusRaw)` | `GarudaService_StartMotor` | Begin ALIGN → OL_RAMP → CL |
| `SectorPI_Stop` | `GarudaService_StopMotor` | Float all phases, stop timers |
| `SectorPI_OlTick` | Timer1 ISR (20 kHz) | OL ramp commutation pacing |
| `SectorPI_Commutate` | SCCP3 ISR | Run one PI iteration, schedule next sector |
| `SectorPI_TimeTick` | Timer1 ISR (1 kHz) | Pot → amplitude, MEAS-PI EMA, speed window |
| `SectorPI_CommandSet(amplitude)` | `_T1Interrupt` | Q15 throttle command (0..32767) |
| `SectorPI_ErpmGet` | telemetry | Current eRPM from `actualStepPeriodHR` |
| `SectorPI_TelemGet(*out)` | `GSP_TelemTick` | Fill `TELEM_T` snapshot |
| `SectorPI_GetPhase` | ISRs, service | 0=OFF, 1=ALIGN, 2=OL_RAMP, 3=CL |
| `SectorPI_IsRunning` | service | `phase != OFF` |
| `SectorPI_GetCaptureLog` | GSP `PI_LOG` cmd | First 30 PI iterations after CL entry (debug) |

### Runtime parameters — `motor/motor_params.h`

| Function | Purpose |
|---|---|
| `Params_InitDefaults` | Load compile-time defaults into `escParams` |
| `Params_Get(id, *ok)` | Read by GSP `GET_PARAM`. Returns 0 on unknown ID. |
| `Params_Set(id, val)` | Write by GSP `SET_PARAM`. Range-checks; recomputes derived. Returns `false` on invalid. |
| `Params_RecomputeDerived` | Updates `escDerived.advancePlus30Fp8` after a `Set`. |
| `Params_GetDescriptorTable(*count)` | Wire-format table for GUI introspection. |

### Service — `garuda_service.h`

| Function | Purpose |
|---|---|
| `GarudaService_Init` | Init SectorPI, start Timer1 |
| `GarudaService_StartMotor` | Clear ATA6847 faults, `EnterGduNormal`, enable PWM/ADC/OPA, call `SectorPI_Start` |
| `GarudaService_StopMotor` | `SectorPI_Stop`, disable peripherals, `EnterGduStandby` |
| `GarudaService_ClearFault` | FAULT → IDLE |
| `GarudaService_Tasks` | Main-loop poll (stall recovery) |
| `ProcessBemfSample` | Called from PTG ISR — the heart of BEMF detection (see "Acceptance gates" above) |

### PWM — `hal/hal_pwm.h`

| Function | Purpose |
|---|---|
| `HAL_PWM_Init` | One-time setup: PG1/2/3 mode, polarity, dead-time |
| `HAL_PWM_EnableOutputs` / `DisableOutputs` | PWM module on/off |
| `HAL_PWM_SetCommutationStep(0..5)` | Apply phase-A/B/C states from `commutationTable[step]` |
| `HAL_PWM_SetDutyCycle(duty)` | Update PG[123]DC with clamping |
| `HAL_PWM_SetDutyCyclePeriod(duty, per)` | Variant where `per` ≠ `LOOPTIME_TCY` |
| `HAL_PWM_ChargeBootstrap` | Pre-CL boost-cap charge sequence |
| `HAL_PWM_ForceAllFloat` / `ForceAllLow` | Override-driven safety states |

### Commutation timer — `hal/hal_com_timer.h`

| Function | Purpose |
|---|---|
| `HAL_ComTimer_Init` | Configure SCCP3 one-shot + SCCP4 HR (640 ns/tick) |
| `HAL_ComTimer_ScheduleAbsolute(targetHR)` | Arm SCCP3 to fire at the specified HR timestamp |
| `HAL_ComTimer_ReadTimer` | Read current SCCP4 free-running value |
| `HAL_ComTimer_Cancel` | Disable SCCP3 interrupt |

### PTG — `hal/hal_ptg.h`

| Function | Purpose |
|---|---|
| `HAL_PTG_Init` | Build step queue (WHI input0 → IRQ0 → JMP) |
| `HAL_PTG_Start` | Re-init + enable + start queue |
| `HAL_PTG_Stop` | Disable peripheral |

### GSP protocol — `gsp/gsp.h`, `gsp/gsp_commands.h`

Frame: `[STX=0x02][LEN][CMD][PAYLOAD...][CRC16]` on UART1 @ 115200.

| Command | Direction | Purpose |
|---|---|---|
| `PING` | host→fw | Liveness check |
| `GET_INFO` | host→fw | 20-byte board ID, FW version, feature flags, max eRPM |
| `GET_SNAPSHOT` | host→fw | Trigger one telemetry frame |
| `TELEM_START` / `STOP` | host→fw | Toggle 10 Hz streaming |
| `START_MOTOR` / `STOP_MOTOR` | host→fw | Motor control |
| `CLEAR_FAULT` | host→fw | Reset FAULT state |
| `HEARTBEAT` | host→fw | Connection keep-alive |
| `SET_THROTTLE` | host→fw | (stub — firmware reads pot directly) |
| `GET_PARAM(id)` | host→fw | Read runtime param |
| `SET_PARAM(id, val)` | host→fw | Write runtime param |
| `GET_PARAM_LIST(start)` | host→fw | Paginated descriptor table |
| `PI_LOG` | host→fw | Dump first 30 PI iterations after CL entry |
| `BEMF_PROBE` | host→fw | Multi-phase BEMF tally dump |
| `ATA_DIAG` | host→fw | Read ATA6847 status registers |
| `TELEM_FRAME` | fw→host | 250-byte snapshot (auto when `TELEM_START` is active) |

---

## Configuration

### Compile-time profile selection

`garuda_config.h`:

```c
#define MOTOR_PROFILE   2   // 0=Hurst, 1=A2212, 2=2810, 3=HiZ1460
```

CLI override: `make MOTOR_PROFILE=1 ...` (works only via `Makefile-cli.mk`).
MPLAB X builds use the `.h` value.

### Per-profile parameters

The only per-motor knobs. All other tuning is universal or runtime-
tunable.

| Macro | Hurst (0) | A2212 (1) | 2810 (2) | HiZ1460 (3) | Effect of change |
|---|---|---|---|---|---|
| `MOTOR_POLE_PAIRS` | 5 | 7 | 7 | 7 | scales the eRPM → mech RPM ratio everywhere |
| `MOTOR_RS_MILLIOHM` | 534 | 65 | 50 | 16000 | informational |
| `MOTOR_LS_MICROH` | 471 | 30 | 25 | 15 | informational |
| `MOTOR_KV` | 149 | 1400 | 1350 | 1460 | RPM/V (informational) |
| `ALIGN_TIME_MS` | 200 | 150 | 150 | 200 | rotor pre-position duration. Too short → motor doesn't lock; too long → wasted startup time. |
| `ALIGN_DUTY` | LT/20 | LT/40 | LT/40 | LT/20 | duty during align. Higher = stronger torque but more current. |
| `INITIAL_STEP_PERIOD` | 1000 | 800 | 800 | 1000 | Timer1 ticks per OL step at start. Higher = slower initial commutation. |
| `MIN_STEP_PERIOD` | 66 | 50 | 50 | 50 | Timer1 ticks at OL→CL handoff. Lower = faster handoff speed. |
| `RAMP_ACCEL_ERPM_S` | 1500 | 1500 | 2500 | 500 | OL acceleration rate. Faster = shorter startup; too fast = rotor falls behind. |
| `RAMP_DUTY_CAP` | LT/6 | LT/6 | LT/6 | LT/8 | max duty during OL ramp. Lower for high-Z motors. |
| `MAX_CLOSED_LOOP_ERPM` | 15000 | 100000 | 150000 | 250000 | hard ceiling — `Tp` floors below this. |
| `MIN_CL_STEP_PERIOD` | — | 2 | 2 | 2 | Timer1-tick floor at top speed. |
| `RAMP_TARGET_ERPM` | 3000 | 4000 | 3000 | 3000 | OL→CL handoff speed. |
| `ILIM_DAC` | 80 | 85 | 120 | 120 | ATA6847 hardware current trip. Too low chops legitimate transients; too high lets desync spikes through. |
| `VBUS_OV_THRESHOLD` | ~30V | ~24V | ~40V | ~40V | over-voltage cutoff (regen spike margin). |
| `VBUS_UV_THRESHOLD` | ~7V | ~6V | ~10V | ~10V | under-voltage cutoff. |
| `STARTUP_SPEED_ERPM` | 3000 | 500 | 500 | 500 | seed speed entering CL. |
| `STARTUP_CURRENT_MA` | 2000 | 3000 | 3000 | 1500 | seed amplitude target. |
| `ALIGN_DURATION_MS` | 200 | 100 | 100 | 200 | CL align duration. |
| `MIN_AMPLITUDE_PROFILE` | 5000 | 5000 | 5000 | 5000 | Q15 idle duty floor (~15.3%). |
| `BLOCK_ENTER_ERPM` | 30k | 100k | 150k | 250k | block-commutation engagement threshold. |
| `BLOCK_EXIT_ERPM` | 25k | 85k | 130k | 220k | hysteresis exit. |

Profile 2 (2810 @ 24V) is the validated 226k-eRPM configuration.
Profile 3 (HiZ1460) uses `CL_IDLE_DUTY_PERCENT 35U` instead of the
default 10% because the high-impedance motor needs much more duty to
make idle torque.

### Universal defaults

| Macro | Default | Effect of change |
|---|---|---|
| `PWMFREQUENCY_HZ` | 60000 | switching frequency. Lowering it regresses high-speed peak (BEMF sample rate is the bottleneck, not switching loss). |
| `LOOPTIME_TCY` | derived | PWM counter period. `8 · FPGx / FPWM − 16`. |
| `MAX_DUTY` | `LOOPTIME_TCY − 200` | ~97% effective duty. Larger leaves no time for ATA6847 CCPT (cross-conduction protection). |
| `MIN_DUTY` | 200 | ~0.6% — below this the gate driver doesn't latch. |
| `DEADTIME_TCY` | 160 | 100 ns. ATA6847L handles 700 ns CCPT internally; 100 ns is sufficient. Raise only if VDS transients leak through. |
| `TIMING_ADVANCE_LEVEL` | 2 (Hurst), 3 (others) | scheduler base-band TAL. AM32-style `(interval/8) × level` = 7.5° per level. |
| `DUTY_RAMP_ERPM` | 60000 | speed where the duty governor allows full pot. Lower clips top speed; higher allows fast-pot desync. |
| `MIN_TARGET_ERPM` / `MAX_TARGET_ERPM` | 1000 / 100000 | pot → speed mapping range. |
| `PI_KP_SHIFT` | 2 | Kp = 1/4. Larger N = gentler P gain. Tuned against 1.5625 MHz HR clock. |
| `PI_KI_SHIFT` | 4 | Ki = 1/16. Larger N = slower integrator. Same clock dependency. |
| `PHASE_ADVANCE_DEG` | 12.5 | PI `setValue` advance. Tunable live via GSP param `0xF0`. |
| `RC_DELAY_US` | 2.0 | ATA6847 comparator propagation. EV92R69A has no RC filter. |
| `MIN_PERIOD_HR` | 10 | absolute floor on `timerPeriod` (~1.5 M eRPM safety). |
| `STALL_THRESHOLD` | 200 | weighted miss-count before shutoff. |
| `STARTUP_TIME_MS` | 1000 | forced-ramp duration before pot commands accepted. |
| `PTG_DUTY_ADAPT_THRESHOLD_PCT` | 50 | crossover between mid-OFF and mid-ON sampling. |
| `PTG_POSTSCALE_N` | 1 | every PTG fire is processed. N=3 (20 kHz BEMF) regresses peak. |
| `PTG_ISR_PRIORITY` | 4 | above ADC (3), below CCP (5/6). Same level as CCP regresses peak. |
| `SECTOR_TIMER_FREQ_HZ` | 1562500 | HR clock. **Don't change without re-tuning Kp/Ki shifts** — the PI is calibrated to 640 ns/tick. |
| `OL_HANDOFF_SETTLE_MS` | 200 | hold motor at handoff rate before CL entry. Too short = rotor lags → stall. |
| `CL_SETTLE_MS` | 500 | seed-integrator floor duration. Too short = PI can shrink `Tp` to floor before motor catches up. |
| `CL_IDLE_DUTY_PERCENT` | 10 (35 for HiZ) | idle duty when pot=0. |
| `FEATURE_GSP` | 1 | UART1 is GSP binary protocol. Set 0 to get debug text instead. |
| `FEATURE_POST_ZC_ACCEPT` | 1 | per-sample post-ZC shadow counters (diag only). |
| `FEATURE_MEAS_PI` | 1 | smoothed period tracker (diag/telemetry only). |
| `MEAS_PI_ALPHA_SHIFT` | 2 | EMA α = 1/4 for the measurement-PI smoother. |

### ADC calibration

| Macro | Value | Effect |
|---|---|---|
| `ADC_MA_PER_COUNT_Q8` | 2756 | mA-per-count × 256. **±22 A range** with stock 4.99 kΩ R_F. **Will saturate under prop load.** For ±55 A swap to 2 kΩ R_F → Q8 = 6872. |
| `ADC_VBUS_MV_PER_COUNT_Q8` | 3300 | mV-per-count × 256. 16:1 divider on AN6. |
| `ADC_CURRENT_BIAS` | 2048 | mid-scale subtracted in ISR to give signed mA around 0. |

---

## Runtime-tunable parameters (GSP)

Seven HOT params changeable while the motor runs (`SET_PARAM` doesn't
require IDLE):

| ID | Name | Type | Range | Default | What it does |
|---|---|---|---|---|---|
| `0xF0` | `PHASE_ADVANCE_X10` | u16 | 0–300 | 125 (12.5°) | PI's `setValue` advance. Larger = PI assumes ZC arrives earlier in the sector. 5° regresses harder than 10° → partly torque advance, not just latency. |
| `0xF1` | `PI_KP_SHIFT` | u8 | 0–8 | 2 | Kp = 1/2^N. Larger N = gentler. |
| `0xF2` | `PI_KI_SHIFT` | u8 | 0–8 | 4 | Ki = 1/2^N. Larger N = slower integrator response. |
| `0xF3` | `BLANKING_PCT` | u8 | 10–60 | 25 | blanking window as % of `actualStepPeriodHR`. Larger rejects more post-commutate ringing but risks missing fast ZCs. |
| `0xF4` | `MIN_PERIOD_HR` | u16 | 5–500 | 10 | `timerPeriod` floor. Sets the speed ceiling. |
| `0xF5` | `PI_FEED_POLARITY` | u8 | 0–2 | 0 | 0 = both, 1 = rising-only, 2 = falling-only. Restricts which sectors feed the PI. |
| `0xF6` | `TRIGA_POS` | u16 | bit-packed | 0 | bits 0–14 = TRIGA value, bit 15 = CAHALF. Diagnostic — moves the ADC trigger position. |

Read via `GET_PARAM(id) → (id, value)`. Write via `SET_PARAM(id, value) → echo`.

Runtime values **don't persist** across reboots. Once a tuning is
good, edit the compile-time default in `garuda_config.h` and re-flash.

---

## Telemetry snapshot (250 bytes)

Emitted at ~10 Hz when `TELEM_START` is active. Decoded by
`tools/gsp_ak_test.py` and `gui/`.

| Offset | Field | Size | Notes |
|---|---|---|---|
| 0..1 | seq counter | u16 | rolling |
| 2..9 | state, fault, step, pot, duty% | mixed | core state |
| 10..27 | amplitude%, Vbus, Ia/Ib/Ibus peaks | mixed | live electricals |
| 28..39 | lastCapValue, PI delta, captures, runs | u16×4 | inner-loop diag |
| 40..47 | systemTick, uptime | u32×2 | timekeeping |
| 48..63 | adcBlankReject, adcStateMismatch, adcCaptureSet, adcSetRising | u32×4 | BEMF gate counters |
| 64..71 | zero pad (was offMid*) | — | reserved |
| 72..91 | ptgFires + 4 unused per-polarity slots | u32×5 | PTG-side diag |
| 92..107 | diagPiFedRising/Miss, diagPiFedFalling/Miss | u32×4 | per-polarity PI feed |
| 108..109 | tMeasHR | u16 | smoothed period |
| 110..113 | sectorCount | u32 | total commutations |
| 114..145 | current peak block + at-fault frozen snapshot | mixed | post-fault forensics |
| 146..155 | elapsed accept/reject per polarity + filterHR | u16×5 | mechanism probe |
| 156..171 | comp × polarity tally | u32×4 | physics vs software bug |
| 172..175 | ptgSkipped | u32 | postscale counter |
| 176..199 | sectorHits[0..5] | u32×6 | all-6-positions validation |
| 200..211 | bemfTallyTotal[0..5] | u16×6 | multi-phase tally totals |
| 212..247 | bemfTally[sector][phase] | u16×6×3 | comp=1 per (sector, phase) |
| 248..249 | fpStaleCount | u16 | atomic-write race detector |

Per-window counters (`bemfTally*`, `fpStaleCount`) are zeroed after
copy, so each frame is a fresh ~100 ms delta.

---

## Hardware notes

### ATA6847L gate driver

- **Comparator output is INVERTED.** BEMF above neutral → `comp = 0`. BEMF below neutral → `comp = 1`.
- Rising ZC produces a **falling comp edge**, falling ZC a rising edge.
- SCCP IC MOD encoding (DS70005349 Table 22-3): `0001` = rising, `0010` = falling.

### dsPIC33AK128MC106 quirks

- **`Fcy = Fosc = 200 MHz`** — 1 cycle per instruction (unlike CK where `Fcy = Fosc / 2`).
- **`MPER` / `MDC` / `MPHASE`** are 20-bit but only the upper 16 bits are programmable in standard PWM mode. The bottom 4 bits are reserved for High-Resolution PWM (unused here).
- **`PCLKCON.MCLKSEL = 0`** routes the 100 MHz Standard-Speed Peripheral Clock to PWM. Routing CLK5 (400 MHz) would overflow `uint16_t PGxPER` below 60 kHz.
- AK has **SCCP1–4 only** (no SCCP5).
- **`__builtin_disi` is NOT supported on XC-DSC.** Use double-read consistency or seqlocks for atomic snapshots.
- **`UPDREQ = 1` must be written to every PG generator** during 6-step (not just the master).

### Board (EV92R69A + EV68M17A)

- 3 mΩ shunts on each phase + DC bus.
- Stock op-amp gain = 24.95× (R_F = 4.99 kΩ, R_IN = 200 Ω).
- POT on AD1CH1 (AN10/RA11), Vbus on AD1CH4 (AN6/RA7).
- Phase currents: Ia → OA1 → AN0, Ib → OA2 → AN1, Ibus → OA3 → AN3.
- BEMF inputs: J5/J7/J9 must be in the BEMF position for the ATA6847L comparators to stay live.

---

## Tuning workflow

1. Build with the right `MOTOR_PROFILE` for your motor.
2. Connect the GUI or `tools/gsp_ak_test.py mon`.
3. Start the motor. Watch:
   - `pR` / `pF` percentages — should be ~3% / ~95% in steady state with default falling-feed.
   - `fpStale` — should be <5%. Higher means atomic-write invariant is broken somewhere.
   - `stl` (stall counter) — should stay near 0.
4. Live-tune from the GUI:
   - **Stalls at idle** → raise `BLANKING_PCT` (0xF3) toward 30–35.
   - **Hunts / oscillates** → raise `PI_KI_SHIFT` (0xF2) toward 5–6, or `PI_KP_SHIFT` (0xF1) toward 3.
   - **Walls below `MAX_CLOSED_LOOP_ERPM`** → try `PHASE_ADVANCE_X10` (0xF0) between 100 and 250.
   - **High-speed desync** → keep `PHASE_ADVANCE_X10` ≤ 200 and verify the TAL bands at top end.
5. Once happy, edit the matching `garuda_config.h` default and re-flash. Runtime values don't persist.
