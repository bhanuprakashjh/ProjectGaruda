# Project Garuda — Detailed Implementation Plan v2.2

**v2.2 changes (spec pass):**
- Fixed copy paths: `reference/project/hal/` not `reference/project/pmsm.X/hal/`
- Resolved all `?` placeholders: ADC PINSEL for BEMF_C (AD2AN2, PINSEL=2,
  muxed with BEMF_B on AD2CH0), comparator INSEL values (CMP1B/CMP2B/CMP3B
  all INSEL=1)
- Fixed comparator architecture: use 3 separate CMP1/CMP2/CMP3 (one per phase),
  not CMP3 multiplexed across domains
- Fixed ISR naming: use `GARUDA_ADC_INTERRUPT` macro (matches reference pattern
  of `MC1_ADC_INTERRUPT`), resolves to `_AD1CH0Interrupt`
- Defined missing symbols: `COMMUTATION_TIMER_FREQ`, `TIMER_TICKS_PER_PWM_CYCLE`,
  renamed `HAL_PWM_SetDuty` → `HAL_PWM_SetCommutationStep`,
  renamed `DSHOT_EncodeERPMValue` → `DSHOT_EncodeERPM`,
  added `floatingPhase` parameter to `DEMAG_CheckAndExtend()`
- Fixed type bugs: `DSHOT_STATE_T.rate` uint8→uint16, `overvoltageDeciV` uint8→uint16,
  `undervoltageDeciV` uint8→uint16, `dshotRate` now enum (0-4) not raw Hz,
  config struct byte count verified with `_Static_assert`
- Fixed Timer1 math: 100µs ISR with 10-count subcount → 1ms systemTickMs
- Removed invalid DShot CMD 255 for bootloader entry (DShot spec: 0-47 only)
- Cleaned up draft working comments in commutation table
- Added `HAL_ADC_SelectBEMFChannel()` for AD2CH0 PINSEL muxing

**v2.1:** Incorporates lessons from studying BLHeli, VESC, ASAC-ESC, Sapog,
Bluejay, and AM32 reference firmware. See `reference_esc_comparison.md` for
detailed comparison notes.

## What We Borrow from the FOC Reference (and How)

Since we're dropping clean-room policy for the AN1292 FOC reference, here's a
file-by-file breakdown of what we take, what we adapt, and what we replace.

---

### REUSE TIER 1 — Copy As-Is (100% reusable)

| File | Why |
|------|-----|
| `hal/clock.c` | PLL1 → 200 MHz, CLK5 (PWM) → 400 MHz, CLK6 (ADC) → 100 MHz, CLK7 (DAC/CMP) → 400 MHz, CLK8 (UART) → 100 MHz. Every register value is correct for our target. |
| `hal/clock.h` | Just the `InitOscillator()` declaration. |
| `hal/device_config.c` | All fuse/config pragma values: WDT=SW, JTAG=OFF, protection regions disabled. Identical for ESC. |
| `hal/timer1.c/.h` | Timer1 at 100 MHz / 8 prescaler = 12.5 MHz scaled, 100 us period. We use this for the 1ms system tick (call BoardService from ISR). |
| `hal/uart1.c/.h` | Full UART1 init: 8N1, full-duplex, CLK8 source. We use it for GSP protocol + debug. Baud rate set at runtime. |
| `util.h` | `SquareFloat()`, `SaturateFloat()`, `LowPassFilter()`, `SQRT_3`, `Q15_MAX`, `Q30_MAX`, `sx1632_t` union. All useful. |
| `foc/pi.c/.h` | PI controller with anti-windup saturation. We use it for throttle-to-duty PI control. `MC_ControllerPIUpdate()` and `MC_ControllerPIReset()` are generic. |

### REUSE TIER 2 — Adapt (60-80% reusable, modify specific sections)

| File | What Changes |
|------|-------------|
| `hal/port_config.c` | **Keep:** PWM output pin mappings (PWM1H/L→RD2/RD3, PWM2H/L→RD0/RD1, PWM3H/L→RC3/RC4), LED pins (RD5, RC9), button pins (RD9, RD10), UART pin remapping (RC11→RX, RC10→TX), fault PCI input (RB11→PCI8). **Remove:** Op-amp config for current sensing (we don't use internal op-amps for BEMF). **Add:** DShot input capture pin (choose available RP pin → Input Capture), BEMF ADC channel pin configs (ANSELx, TRISx for phase voltage divider inputs). |
| `hal/pwm.c` | **Keep:** `PCLKCON` master config (DIVSEL=0, MCLKSEL=1), `MPER = LOOPTIME_TCY`, all dead time values (`PGxDT`), complementary mode (`PMOD=0`), `PENH=1/PENL=1`, bootstrap charging sequence, PG1 as master / PG2,PG3 as slaves. **Change:** Remove SVM trigger configuration. Add override-mode initialization (set `OVRENH=1, OVRENL=1, OVRDAT=0` on all 3 generators at startup = all outputs LOW). Change `ADC_SAMPLING_POINT` in `PG1TRIGA` to sample at PWM center for BEMF (same register, possibly different value). Keep fault PCI config (CMP3 → PWM shutdown). |
| `hal/pwm.h` | **Keep:** `PWM_PDC1/2/3`, `PWM_PHASE1/2/3` macros, `DEADTIME` calculation formula, `LOOPTIME_TCY` formula, `MIN_DUTY/MAX_DUTY`, bootstrap constants. **Change:** `PWMFREQUENCY_HZ` from 20000 → 24000 (or configurable). Remove `SINGLE_SHUNT` conditionals (not applicable). |
| `hal/adc.c` | **Keep:** ADC core enable sequence (`AD1CON.ON=1`, wait `ADRDY`), ADC2 core enable, Vbus channel config (AD1CH4, pin RA7, PINSEL=6). **Remove:** All op-amp output channels (IA on AD1CH0, IB on AD2CH0, IBUS on AD1CH2/CH3). **Add:** 3x BEMF phase voltage channels (one per motor phase, on appropriate ADC channels — see pin mapping section). Keep potentiometer channel for dev board testing. **Change:** Interrupt source to BEMF completion (not pot or IBUS). |
| `hal/cmp.c/.h` | **Keep:** `InitializeCMPs()` with DAC calibration read from flash (POSINLADJ, NEGINLADJ, DNLADJ), DACCTRL1 init, filter clock divider. **Change:** Repurpose CMP3 for BEMF zero-crossing detection instead of overcurrent. Change `INSEL` to select BEMF input. Change `CMPPOL` based on expected ZC polarity. Add CMP1/CMP2 initialization for multi-comparator ZC (one per phase). DAC reference = Vbus/2 (dynamic, updated from ADC reading). |
| `hal/board_service.c/.h` | **Keep:** `HAL_InitPeripherals()` call sequence, `HAL_MC1PWMEnableOutputs()` (clear overrides), `HAL_MC1PWMDisableOutputs()` (set overrides, OVRDAT=0), `HAL_MC1ClearPWMPCIFault()`, button handling (`ButtonScan`, debounce), heartbeat LED. **Remove:** `HAL_MC1MotorInputsRead()` (FOC-specific current reading), `HAL_PWM_DutyCycleRegister_Set()` / `HAL_PWM_PhaseRegister_Set()` (single-shunt phase shifting). **Add:** `HAL_SetCommutationStep()` (new, writes override registers per 6-step table), `HAL_ReadBEMF()` (reads floating phase ADC). |
| `hal/measure.c/.h` | **Keep:** `MCAPP_MeasureAvgInit()` / `MCAPP_MeasureAvg()` (moving average filter), `MCAPP_MEASURE_VDC_T` struct. **Remove:** Current offset calibration (`MeasureCurrentOffset`, `MeasureCurrentCalibrate`). **Add:** BEMF measurement structures and Vbus/2 reference calculation. |
| `main.c` | **Keep:** Init sequence (`InitOscillator()` → `SetupGPIOPorts()` → `HAL_InitPeripherals()` → service init), Timer1 ISR structure (button handling, heartbeat LED, command buffering). **Replace:** `MCAPP_MC1ServiceInit()` with `GARUDA_ServiceInit()`. Replace FOC service call with Garuda state machine. |
| `mc1_service.c` | **Keep as pattern:** ADC ISR structure (read inputs → run state machine → update outputs → clear flag), PWM fault ISR structure (clear fault → set FAULT state). **Replace entirely:** State machine states, motor control algorithm, all FOC calls. Rewrite as `garuda_service.c`. |
| `fault.c/.h` | **Keep as pattern:** Fault state enum, overcurrent detection function signature. **Rewrite:** Detection logic for BEMF-based ESC faults (desync, stall, overvoltage, thermal). |

### REUSE TIER 3 — Pattern Only (use structure, rewrite logic)

| File | What We Learn |
|------|--------------|
| `mc1_init.c/.h` | Data structure pattern (`MC1APP_DATA_T` → `GARUDA_DATA_T`), pointer setup, parameter initialization from header constants. |
| `mc1_user_params.h` | Pattern for user-configurable parameters. We create `garuda_config.h` with ESC-specific params. |
| `mc1_calc_params.h` | Pattern for derived calculations. We create `garuda_calc_params.h` with BEMF/timing formulas. |
| `foc/foc.c` | State machine pattern (INIT → LOCK → OL → CL). Our states differ but the ISR-driven FSM concept is identical. |
| `foc/estim_pll.c` | BEMF estimation concept (V - RI - LdI/dt). We don't use PLL, but the BEMF voltage equation is the same physics. We simplify: just compare floating phase voltage against Vbus/2. |

### NOT REUSED — Replace Entirely

| File | Why |
|------|-----|
| `foc/clarke_park.c/.h` | Clarke/Park transforms are FOC-only. 6-step doesn't need reference frame transforms. |
| `foc/svm.c/.h` | Space Vector Modulation is FOC-only. 6-step uses direct PWM override. |
| `foc/estim_pll.c/.h` | PLL estimator is FOC-only. We use direct BEMF zero-crossing detection. |
| `foc/flux_control.c/.h` | Flux weakening is FOC-only. Not applicable for 6-step drone ESC. |
| `foc/estim_interface.c/.h` | FOC estimator interface. Replaced by our `bemf.c` module. |
| `singleshunt/` | Single-shunt current reconstruction. Not applicable. |
| `generic_load/` | Generic load state machine. Not needed for ESC. |

---

## Pin Mapping — Dev Board (MCLV-48V-300W) vs Custom ESC

### Pins Inherited from Reference (Identical)

| Function | Pin | Port | Register | DIM | Notes |
|----------|-----|------|----------|-----|-------|
| **PWM1H** (Phase A High) | 53 | RD2 | TDO/RP51/PWM1H | 001 | TRISD2=0 |
| **PWM1L** (Phase A Low) | 54 | RD3 | TDI/RP52/PWM1L | 003 | TRISD3=0 |
| **PWM2H** (Phase B High) | 51 | RD0 | RP49/PWM2H | 005 | TRISD0=0 |
| **PWM2L** (Phase B Low) | 52 | RD1 | TCK/RP50/PWM2L | 007 | TRISD1=0 |
| **PWM3H** (Phase C High) | 43 | RC3 | PGD3/RP36/PWM3H | 002 | TRISC3=0 |
| **PWM3L** (Phase C Low) | 44 | RC4 | RP37/PWM3L | 004 | TRISC4=0 |
| **Vbus ADC** | 02 | RA7 | AD1AN6/RP8 | 039 | ANSELA7=1, TRISA7=1, AD1CH4 PINSEL=6 |
| **LED1** (Heartbeat) | 55 | RD5 | RP54/ASCL1 | 030 | TRISD5=0 |
| **LED2** (Status) | 34 | RC9 | RP42/SDO2 | 032 | TRISC9=0 |
| **SW1** (Start/Stop) | 49 | RD9 | RP58 | 034 | TRISD9=1 (input) |
| **SW2** (Direction) | 50 | RD10 | RP59 | 036 | TRISD10=1 (input) |
| **UART1 RX** | 46 | RC11 | RP44 | 054 | _U1RXR=44 |
| **UART1 TX** | 45 | RC10 | RP43 | 052 | _RP43R=9 |
| **Fault Input** | 32 | RB11 | RP28/PCI8 | 040 | _PCI8R=28 |
| **Pot (dev only)** | 06 | RA11 | AD1AN10/RP12 | 028 | ANSELA11=1, TRISA11=1 |

### Pins We Change (BEMF Sensing)

The FOC reference uses internal op-amps (OA1, OA2, OA3) for current sensing.
For 6-step BEMF, we **disable the op-amps** and repurpose these pins (or use
other available analog pins) for BEMF voltage measurement.

**Strategy:** On the MCLV-48V-300W dev board, the motor phase outputs connect
through voltage divider networks back to the MCU. We route these to ADC channels.
On a custom ESC board, we design dedicated BEMF voltage dividers (typically
100k/10k for ~10:1 ratio, mapping 0-48V motor phase → 0-4.8V ADC range).

#### Option A: Reuse Op-Amp Input Pins (Dev Board)

| Function | Pin | Port | ADC Channel | Config |
|----------|-----|------|-------------|--------|
| **BEMF Phase A** | 14 | RA4 | AD1AN1 (OA1IN+) | ANSELA4=1, TRISA4=1, Disable OA1 |
| **BEMF Phase B** | 22 | RB2 | AD2AN4 (OA2IN+) | ANSELB2=1, TRISB2=1, Disable OA2 |
| **BEMF Phase C** | 17 | RB5 | AD2AN2 (OA3IN+) | ANSELB5=1, TRISB5=1, Disable OA3 |
| **Virtual Neutral** | — | DAC3 | Internal | DAC output = Vbus/2 (calculated from ADC Vbus reading) |

ADC Channel Configuration:
```
BEMF_A: AD1CH0, PINSEL=1 (AD1AN1 = RA4)
BEMF_B: AD2CH0, PINSEL=4 (AD2AN4 = RB2)  — or use AD1 shared core
BEMF_C: AD1CH2, PINSEL=2 (AD1AN2 = RA6)  — alt: AD2CH0 PINSEL=2 (AD2AN2 = RB5)
Vbus:   AD1CH4, PINSEL=6 (AD1AN6 = RA7)  — unchanged from reference
Pot:    AD1CH1, PINSEL=10 (AD1AN10 = RA11) — dev board only
```

#### Option B: Comparator-Based ZC Detection (Hardware-Assisted)

Use the 3 comparators for direct zero-crossing detection:

| Comparator | BEMF Input Pin | DAC Reference | Purpose |
|------------|----------------|---------------|---------|
| **CMP1** | CMP1A (RA2) or CMP1B (RA4) | DAC1 = Vbus/2 | Phase A ZC |
| **CMP2** | CMP2A (RB0) or CMP2B (RB2) | DAC2 = Vbus/2 | Phase B ZC |
| **CMP3** | CMP3A (RA5) or CMP3B (RB5) | DAC3 = Vbus/2 | Phase C ZC |

In practice, we use **hybrid mode**: ADC samples BEMF for majority filtering,
comparator provides hardware-timed ZC edge detection. Only the floating phase
comparator is active at any time (the other two are masked/disabled per step).

#### Option C: Custom ESC Board (Final Product)

On a custom board, we place dedicated BEMF voltage dividers:

| Function | Suggested Pin | Port | ADC | Divider Ratio |
|----------|--------------|------|-----|---------------|
| BEMF A | RA4 | AD1AN1 | AD1CH0 | 100k/10k (11:1) |
| BEMF B | RB2 | AD2AN4 | AD2CH0 | 100k/10k (11:1) |
| BEMF C | RB5 | AD2AN2 | AD2CH1 | 100k/10k (11:1) |
| Vbus | RA7 | AD1AN6 | AD1CH4 | 100k/4.7k (22.3:1) |
| NTC Temp | RA11 | AD1AN10 | AD1CH1 | NTC + 10k pullup |
| Current | RA5 | AD1AN3/CMP3A | AD1CH2 | Via shunt + gain |

### DShot Input Pin (NEW — not in reference)

We need one pin for DShot signal input via Input Capture + DMA.

| Function | Pin | Port | Peripheral | Config |
|----------|-----|------|------------|--------|
| **DShot In** | 48 | RD8 | RP57 → IC1 | TRISD8=1, _IC1R=57 |

Why RD8: Available on the dev board header, not used by any reference function,
supports PPS remapping to Input Capture 1. Alternative: RC8 (RP41), RD7 (RP56).

Input Capture Configuration:
```c
IC1CONbits.ICM = 1;      // Capture every edge
IC1CONbits.ICI = 0;       // Interrupt on every capture
IC1CONbits.ICTSEL = 0;    // Timer2 as timebase (configure for DShot resolution)
// DMA transfers IC1BUF to RAM buffer on each capture event
```

### Complete Pin Map Summary

```
dsPIC33AK128MC106 — Project Garuda Pin Allocation
═══════════════════════════════════════════════════

 MOTOR DRIVE (6 pins — from reference)
  RD2 (PIN53) ── PWM1H ── Phase A High-Side Gate
  RD3 (PIN54) ── PWM1L ── Phase A Low-Side Gate
  RD0 (PIN51) ── PWM2H ── Phase B High-Side Gate
  RD1 (PIN52) ── PWM2L ── Phase B Low-Side Gate
  RC3 (PIN43) ── PWM3H ── Phase C High-Side Gate
  RC4 (PIN44) ── PWM3L ── Phase C Low-Side Gate

 BEMF SENSING (3 pins — NEW, replaces op-amp pins)
  RA4 (PIN14) ── AD1AN1 ── BEMF Phase A voltage divider
  RB2 (PIN22) ── AD2AN4 ── BEMF Phase B voltage divider
  RB5 (PIN17) ── AD2AN2 ── BEMF Phase C voltage divider

 ZERO-CROSSING COMPARATORS (shared pins with BEMF ADC)
  RA4 (PIN14) ── CMP1B  ── Phase A ZC comparator input
  RB2 (PIN22) ── CMP2B  ── Phase B ZC comparator input
  RB5 (PIN17) ── CMP3B  ── Phase C ZC comparator input
  DAC3 output ── Internal ── Vbus/2 reference for all comparators

 POWER MONITORING (2 pins — from reference)
  RA7 (PIN02) ── AD1AN6 ── DC Bus voltage (Vbus)
  RA11(PIN06) ── AD1AN10 ── Temperature NTC / Pot (dev board)

 CURRENT SENSING (1 pin — optional, for overcurrent protection)
  RA5 (PIN15) ── AD1AN3/CMP3A ── Bus current shunt (if available)

 PROTOCOL I/O (3 pins)
  RD8 (PIN48) ── RP57→IC1 ── DShot input (Input Capture)
  RC10(PIN45) ── RP43→U1TX ── UART TX (GSP / Debug)
  RC11(PIN46) ── RP44→U1RX ── UART RX (GSP / Debug)

 FAULT (1 pin — from reference)
  RB11(PIN32) ── RP28→PCI8 ── External overcurrent/overvoltage fault

 USER I/O (4 pins — from reference, dev board only)
  RD5 (PIN55) ── LED1 ── Heartbeat
  RC9 (PIN34) ── LED2 ── Status / Armed indicator
  RD9 (PIN49) ── SW1  ── Start/Stop (dev board)
  RD10(PIN50) ── SW2  ── Direction (dev board)

 UNUSED / RESERVED
  RA2, RA3, RA6, RB0, RB1 ── Former op-amp pins, available for expansion
  RC5-RC8, RD4, RD6-RD7    ── Available for future features
```

---

## Clock Domain Map (From Reference — No Changes)

```
8 MHz FRC ──┐
            ├─ PLL1 ─┬─ FVCO  = 800 MHz (M=100, N1=1)
            │         └─ FPLLO = 200 MHz (N2=4, N3=1)
            │
CLK1 ───────┤ Source: PLL1 FOUT  │ INTDIV=0  │ = 200 MHz  │ System clock (FCY)
CLK5 ───────┤ Source: PLL1 VCO   │ INTDIV=1  │ = 400 MHz  │ PWM timebase
CLK6 ───────┤ Source: PLL1 FOUT  │ INTDIV=1  │ = 100 MHz  │ ADC clock
CLK7 ───────┤ Source: PLL1 VCO   │ INTDIV=1  │ = 400 MHz  │ DAC/Comparator clock
CLK8 ───────┤ Source: PLL1 FOUT  │ INTDIV=1  │ = 100 MHz  │ UART clock
CLK12 ──────┘ Source: PLL1 FOUT  │ INTDIV=4000│= 25 kHz   │ Reference clock (REFO1)
```

---

## Detailed Implementation Plan

### Phase 1: Foundation — Motor Spins Open-Loop

#### Task 1.1: MPLAB X Project Setup

**Goal:** Compilable project targeting dsPIC33AK128MC106.

**Steps:**
1. Create new MPLAB X project `garudaESC.X` in `/media/bhanu1234/Development/ProjectGaruda/dspic33AKESC/`
2. Target device: dsPIC33AK128MC106
3. Compiler: XC-DSC v3.21+
4. DFP: dsPIC33AK-MC_DFP v1.1.109

**Copy from reference (verbatim):**
```
reference/project/hal/clock.c        → garudaESC.X/hal/clock.c
reference/project/hal/clock.h        → garudaESC.X/hal/clock.h
reference/project/hal/device_config.c → garudaESC.X/hal/device_config.c
reference/project/hal/timer1.c       → garudaESC.X/hal/timer1.c
reference/project/hal/timer1.h       → garudaESC.X/hal/timer1.h
reference/project/hal/uart1.c        → garudaESC.X/hal/uart1.c
reference/project/hal/uart1.h        → garudaESC.X/hal/uart1.h
reference/project/hal/delay.h        → garudaESC.X/hal/delay.h
reference/project/util.h             → garudaESC.X/util.h
reference/project/foc/pi.c           → garudaESC.X/motor/pi.c
reference/project/foc/pi.h           → garudaESC.X/motor/pi.h
```

Note: The pmsm.X/ subdirectory contains only MPLAB project metadata (Makefile,
nbproject/). All source files live directly under reference/project/.

**Create new directory structure:**
```
garudaESC.X/
├── main.c                      # Entry point (adapt from reference)
├── garuda_service.c/.h         # Motor control service (replaces mc1_service)
├── garuda_init.c/.h            # Parameter init (replaces mc1_init)
├── garuda_config.h             # User params (replaces mc1_user_params)
├── garuda_calc_params.h        # Derived params (replaces mc1_calc_params)
├── garuda_types.h              # All data structures
├── util.h                      # Utility functions (from reference)
├── hal/
│   ├── clock.c/.h              # 200 MHz PLL (from reference, verbatim)
│   ├── device_config.c         # Fuses (from reference, verbatim)
│   ├── port_config.c/.h        # GPIO + pin mapping (adapted)
│   ├── hal_pwm.c/.h            # PWM for 6-step (adapted from reference)
│   ├── hal_adc.c/.h            # ADC for BEMF + Vbus (adapted)
│   ├── hal_comparator.c/.h     # CMP for ZC detection (adapted from cmp.c)
│   ├── hal_timer.c/.h          # Commutation timer (SCCP-based, new)
│   ├── hal_ic.c/.h             # Input Capture for DShot (new)
│   ├── timer1.c/.h             # System tick (from reference, verbatim)
│   ├── uart1.c/.h              # UART (from reference, verbatim)
│   ├── board_service.c/.h      # Board support (adapted)
│   └── delay.h                 # Delay macros (from reference, verbatim)
├── motor/
│   ├── commutation.c/.h        # 6-step commutation table + execution
│   ├── bemf.c/.h               # BEMF zero-crossing detection
│   ├── startup.c/.h            # Align + open-loop ramp
│   ├── timing.c/.h             # Commutation timing engine
│   ├── desync.c/.h             # Desync detection + recovery
│   └── pi.c/.h                 # PI controller (from reference foc/pi.c)
├── protocol/
│   ├── dshot.c/.h              # DShot receiver
│   ├── dshot_bidir.c/.h        # Bidirectional DShot telemetry
│   ├── dshot_commands.c/.h     # DShot command handler
│   ├── edt.c/.h                # Extended DShot Telemetry
│   └── gsp.c/.h                # Garuda Serial Protocol
├── config/
│   ├── eeprom.c/.h             # EEPROM config storage
│   ├── defaults.h              # Factory defaults
│   └── calibration.c/.h        # Dead-time calibration
├── protection/
│   ├── overcurrent.c/.h        # HW comparator fault
│   ├── thermal.c/.h            # Temperature monitoring
│   ├── voltage.c/.h            # Under/over voltage
│   └── signal_loss.c/.h        # Signal timeout failsafe
└── telemetry/
    ├── telemetry.c/.h          # Centralized telemetry
    └── beep.c/.h               # Motor beep generation
```

**Acceptance:** Project compiles with no errors, links, and programs to dsPIC33AK via PKOBv4.

---

#### Task 1.2: Clock, GPIO, and Peripheral Init

**Goal:** MCU boots at 200 MHz, all pins configured, peripherals clocked.

**Implementation (port_config.c):**

Start from reference `port_config.c`. Make these specific changes:

```c
void MapGPIOHWFunction(void)
{
    // ========== PWM OUTPUTS (UNCHANGED FROM REFERENCE) ==========
    // Phase A: PWM1H on RD2, PWM1L on RD3
    TRISDbits.TRISD2 = 0;  // PWM1H output
    TRISDbits.TRISD3 = 0;  // PWM1L output
    // Phase B: PWM2H on RD0, PWM2L on RD1
    TRISDbits.TRISD0 = 0;  // PWM2H output
    TRISDbits.TRISD1 = 0;  // PWM2L output
    // Phase C: PWM3H on RC3, PWM3L on RC4
    TRISCbits.TRISC3 = 0;  // PWM3H output
    TRISCbits.TRISC4 = 0;  // PWM3L output

    // ========== BEMF ADC INPUTS (NEW — replaces op-amp config) ==========
    // BEMF Phase A: RA4 (AD1AN1, was OA1IN+)
    ANSELAbits.ANSELA4 = 1;  // Analog mode
    TRISAbits.TRISA4 = 1;    // Input
    // BEMF Phase B: RB2 (AD2AN4, was OA2IN+)
    ANSELBbits.ANSELB2 = 1;
    TRISBbits.TRISB2 = 1;
    // BEMF Phase C: RB5 (AD2AN2, was OA3IN+)
    ANSELBbits.ANSELB5 = 1;
    TRISBbits.TRISB5 = 1;

    // ========== VBUS ADC (UNCHANGED) ==========
    // DC Bus voltage: RA7 (AD1AN6)
    ANSELAbits.ANSELA7 = 1;
    TRISAbits.TRISA7 = 1;

    // ========== POTENTIOMETER (dev board, UNCHANGED) ==========
    ANSELAbits.ANSELA11 = 1;
    TRISAbits.TRISA11 = 1;

    // ========== DShot INPUT (NEW) ==========
    // DShot signal: RD8 (RP57) → Input Capture 1
    TRISDbits.TRISD8 = 1;    // Input
    _IC1R = 57;              // PPS: IC1 input = RP57

    // ========== UART (UNCHANGED) ==========
    TRISCbits.TRISC11 = 1;   // RX input
    TRISCbits.TRISC10 = 0;   // TX output
    _U1RXR = 44;             // UART1 RX = RP44
    _RP43R = 9;              // RP43 = UART1 TX

    // ========== FAULT PCI (UNCHANGED) ==========
    TRISBbits.TRISB11 = 1;
    _PCI8R = 28;             // PCI8 = RP28

    // ========== LEDs (UNCHANGED) ==========
    TRISDbits.TRISD5 = 0;    // LED1
    TRISCbits.TRISC9 = 0;    // LED2

    // ========== BUTTONS (UNCHANGED) ==========
    TRISDbits.TRISD9 = 1;    // SW1
    TRISDbits.TRISD10 = 1;   // SW2
}

// DO NOT call OpampConfig() — we don't use internal op-amps
```

**`main.c` init sequence (adapted from reference):**
```c
int main(void)
{
    InitOscillator();           // 200 MHz PLL (from reference, verbatim)
    SetupGPIOPorts();           // GPIO config (our modified version)
    HAL_InitPeripherals();      // PWM, ADC, CMP, Timer1 (our versions)
    GARUDA_ServiceInit();       // ESC state machine init

    while(1)
    {
        // Background tasks: EEPROM operations, diagnostics
        // All real-time work happens in ISRs
    }
}
```

**Acceptance:** Oscilloscope shows 200 MHz clock output. LEDs toggle. UART
outputs test string at 115200 baud. GPIO directions verified with multimeter.

---

#### Task 1.3: PWM Setup for 6-Step Override Control

**Goal:** 3 complementary PWM pairs at 24 kHz with override control.

**Key difference from reference:** In FOC, all 3 generators run independently
with duty cycles set by SVM. In 6-step, we use **override mode** to directly
control which FETs are on/off per commutation step.

**`hal_pwm.c` — derived from reference `pwm.c`:**

PWM Master Config (UNCHANGED from reference):
```c
PCLKCON = 0x0000;
PCLKCONbits.DIVSEL = 0;     // 1:2 divider
PCLKCONbits.MCLKSEL = 1;    // FPLLO as master clock source
```

PWM Period (CHANGED to 24 kHz):
```c
// garuda_config.h:
#define PWMFREQUENCY_HZ        24000
#define PWM_CLOCK_MHZ           400
#define DEADTIME_MICROSEC       0.75f
#define LOOPTIME_MICROSEC       (1000000.0f / PWMFREQUENCY_HZ)  // 41.667 us
#define LOOPTIME_TCY            (uint32_t)((LOOPTIME_MICROSEC * 8 * PWM_CLOCK_MHZ) - 16)
//                              = (41.667 * 8 * 400) - 16 = 133317
#define DEADTIME                (uint32_t)(DEADTIME_MICROSEC * 16 * PWM_CLOCK_MHZ)
//                              = 0.75 * 16 * 400 = 4800
#define MIN_DUTY                (uint32_t)(DEADTIME + DEADTIME)   // 9600
#define MAX_DUTY                (LOOPTIME_TCY - MIN_DUTY)         // 123717

MPER = LOOPTIME_TCY;  // Master period register
```

Generator Init (SAME structure, adapted for 6-step):
```c
void InitPWMGenerator1(void)
{
    // Identical to reference EXCEPT:
    // - MODSEL = 4 (Center-Aligned, always — no SINGLE_SHUNT path)
    // - MSTEN = 1 (PG1 is master)
    // - Start with overrides ENABLED (outputs forced LOW at init)
    PG1CONbits.CLKSEL = 1;
    PG1CONbits.MODSEL = 4;       // Center-aligned
    PG1CONbits.MPERSEL = 1;      // Use MPER
    PG1CONbits.MSTEN = 1;        // Master

    PG1IOCONbits.PMOD = 0;       // Complementary mode
    PG1IOCONbits.PENH = 1;       // PWM controls PWM1H pin
    PG1IOCONbits.PENL = 1;       // PWM controls PWM1L pin
    PG1IOCONbits.POLH = 0;       // Active high
    PG1IOCONbits.POLL = 0;       // Active high

    // START WITH OVERRIDE: outputs forced LOW (safe state)
    PG1IOCONbits.OVRENH = 1;     // Override PWM1H
    PG1IOCONbits.OVRENL = 1;     // Override PWM1L
    PG1IOCONbits.OVRDAT = 0b00;  // Both LOW

    // Dead time (from reference)
    PG1DTbits.DTH = DEADTIME;     // 4800 counts = 750ns
    PG1DTbits.DTL = DEADTIME;

    // ADC trigger at PWM center (for BEMF sampling)
    PG1EVTbits.ADTR1EN1 = 1;     // TRIGA triggers ADC
    PG1TRIGAbits.TRIGA = 0;      // Center of PWM cycle (0 = center in center-aligned)
    PG1TRIGAbits.CAHALF = 0;

    // Fault PCI: Comparator output for overcurrent (from reference)
    PG1EVTbits.FLTIEN = 1;       // Fault interrupt enable
    // ... (fault PCI config same as reference)

    PG1DCbits.DC = 0;            // Start at 0% duty
}

// PG2, PG3: Same as reference (slaves, SOCS=1, MSTEN=0)
// Also start with overrides enabled, OVRDAT=0
```

**6-Step Override Control Function (NEW):**
```c
/*
 * 6-Step Commutation Truth Table
 *
 * Standard 6-step commutation for BLDC (positive rotation, complementary PWM):
 *
 * In complementary PWM mode with override:
 * - "PWM active" = OVRENH=0, OVRENL=0 (generator drives both pins via duty cycle)
 * - "LOW"        = OVRENH=1, OVRENL=1, OVRDAT=0b01 (H=OFF, L=ON = low-side on)
 *                  Actually: OVRDAT[1]=H pin, OVRDAT[0]=L pin
 *                  OVRDAT=0b01 means H=0 (off), L=1 (on) → sinks to GND
 * - "Float"      = OVRENH=1, OVRENL=1, OVRDAT=0b00 (both off = high-Z)
 */

typedef struct {
    // PGxIOCON override bits for each generator
    uint8_t ovrenh;   // Override enable for high-side
    uint8_t ovrenl;   // Override enable for low-side
    uint8_t ovrdat;   // Override data: bit1=H, bit0=L
} PHASE_STATE_T;

typedef struct {
    PHASE_STATE_T phaseA;
    PHASE_STATE_T phaseB;
    PHASE_STATE_T phaseC;
    uint8_t       floatingPhase;  // 0=A, 1=B, 2=C
    uint8_t       zcPolarity;     // 0=falling, 1=rising
} COMMUTATION_STEP_T;

// PWM active: override disabled, generator drives complementary pair
#define PWM_ACTIVE  { .ovrenh = 0, .ovrenl = 0, .ovrdat = 0b00 }
// Low side ON (sink current to ground): both overridden, L=1 H=0
#define PHASE_LOW   { .ovrenh = 1, .ovrenl = 1, .ovrdat = 0b01 }
// Floating (high impedance): both overridden, both OFF
#define PHASE_FLOAT { .ovrenh = 1, .ovrenl = 1, .ovrdat = 0b00 }

static const COMMUTATION_STEP_T commutation_table[6] = {
    // Step 0: A→B  (A=PWM, B=Low, C=Float) — BEMF on C, falling
    { PWM_ACTIVE, PHASE_LOW,  PHASE_FLOAT, 2, 0 },
    // Step 1: A→C  (A=PWM, B=Float, C=Low) — BEMF on B, rising
    { PWM_ACTIVE, PHASE_FLOAT, PHASE_LOW,  1, 1 },
    // Step 2: B→C  (A=Float, B=PWM, C=Low) — BEMF on A, falling
    { PHASE_FLOAT, PWM_ACTIVE, PHASE_LOW,  0, 0 },
    // Step 3: B→A  (A=Low, B=PWM, C=Float) — BEMF on C, rising
    { PHASE_LOW,  PWM_ACTIVE, PHASE_FLOAT, 2, 1 },
    // Step 4: C→A  (A=Low, B=Float, C=PWM) — BEMF on B, falling
    { PHASE_LOW,  PHASE_FLOAT, PWM_ACTIVE, 1, 0 },
    // Step 5: C→B  (A=Float, B=Low, C=PWM) — BEMF on A, rising
    { PHASE_FLOAT, PHASE_LOW,  PWM_ACTIVE, 0, 1 },
};

void HAL_PWM_SetCommutationStep(uint8_t step, uint16_t duty)
{
    const COMMUTATION_STEP_T *s = &commutation_table[step % 6];

    // Phase A (PG1)
    PG1IOCONbits.OVRDAT = s->phaseA.ovrdat;
    PG1IOCONbits.OVRENH = s->phaseA.ovrenh;
    PG1IOCONbits.OVRENL = s->phaseA.ovrenl;

    // Phase B (PG2)
    PG2IOCONbits.OVRDAT = s->phaseB.ovrdat;
    PG2IOCONbits.OVRENH = s->phaseB.ovrenh;
    PG2IOCONbits.OVRENL = s->phaseB.ovrenl;

    // Phase C (PG3)
    PG3IOCONbits.OVRDAT = s->phaseC.ovrdat;
    PG3IOCONbits.OVRENH = s->phaseC.ovrenh;
    PG3IOCONbits.OVRENL = s->phaseC.ovrenl;

    // Set duty cycle on the active phase only
    PG1DCbits.DC = (s->phaseA.ovrenh == 0) ? duty : 0;
    PG2DCbits.DC = (s->phaseB.ovrenh == 0) ? duty : 0;
    PG3DCbits.DC = (s->phaseC.ovrenh == 0) ? duty : 0;
}

void HAL_PWM_DisableAllOutputs(void)
{
    // Force all outputs LOW (safe state)
    PG1IOCONbits.OVRDAT = 0; PG1IOCONbits.OVRENH = 1; PG1IOCONbits.OVRENL = 1;
    PG2IOCONbits.OVRDAT = 0; PG2IOCONbits.OVRENH = 1; PG2IOCONbits.OVRENL = 1;
    PG3IOCONbits.OVRDAT = 0; PG3IOCONbits.OVRENH = 1; PG3IOCONbits.OVRENL = 1;
    PG1DCbits.DC = 0;
    PG2DCbits.DC = 0;
    PG3DCbits.DC = 0;
}
```

**Bootstrap charging (from reference, verbatim):**
`ChargeBootstrapCapacitors()` is used as-is. It charges the high-side
bootstrap capacitors by briefly pulsing the low-side FETs.

**Acceptance:** Oscilloscope shows complementary PWM on all 6 pins at 24 kHz.
Override control confirmed: can force any phase to PWM/LOW/FLOAT independently.
Dead time = 750ns visible between complementary transitions.

---

#### Task 1.4: ADC Setup for BEMF and Vbus

**Goal:** Sample BEMF from floating phase and Vbus at PWM center.

**`hal_adc.c` — derived from reference `adc.c`:**

```c
void HAL_ADC_Initialize(void)
{
    // ---- BEMF Phase A: RA4 = AD1AN1 ----
    AD1CH0CONbits.PINSEL = 1;     // AD1AN1 (RA4)
    AD1CH0CONbits.SAMC = 3;       // 3 TAD sample time
    AD1CH0CONbits.LEFT = 0;       // Right-aligned 12-bit
    AD1CH0CONbits.DIFF = 0;       // Single-ended
    AD1CH0CONbits.TRG1SRC = 4;    // PWM1 Trigger 1 (at center)

    // ---- BEMF Phase B: RB2 = AD2AN4 ----
    AD2CH0CONbits.PINSEL = 4;     // AD2AN4 (RB2)
    AD2CH0CONbits.SAMC = 3;
    AD2CH0CONbits.LEFT = 0;
    AD2CH0CONbits.DIFF = 0;
    AD2CH0CONbits.TRG1SRC = 4;    // PWM1 Trigger 1

    // ---- BEMF Phase C: RB5 = AD2AN2 ----
    // RB5 is AD2AN2 (on ADC2 only). AD2 has only one dedicated channel (CH0).
    // Since we only sample ONE floating phase at a time in 6-step commutation,
    // we dynamically switch AD2CH0.PINSEL between BEMF_B (4) and BEMF_C (2)
    // at each commutation step. No simultaneous sampling needed.
    // Initial config — will be overwritten by HAL_ADC_SelectBEMFChannel()
    // at each commutation step.

    // ---- Vbus: RA7 = AD1AN6 (UNCHANGED from reference) ----
    AD1CH4CONbits.PINSEL = 6;     // AD1AN6 (RA7)
    AD1CH4CONbits.SAMC = 5;       // 5 TAD (higher impedance source)
    AD1CH4CONbits.LEFT = 0;
    AD1CH4CONbits.DIFF = 0;
    AD1CH4CONbits.TRG1SRC = 4;    // PWM1 Trigger 1

    // ---- Potentiometer: RA11 = AD1AN10 (dev board, UNCHANGED) ----
    AD1CH1CONbits.PINSEL = 10;
    AD1CH1CONbits.SAMC = 5;
    AD1CH1CONbits.LEFT = 0;
    AD1CH1CONbits.DIFF = 0;
    AD1CH1CONbits.TRG1SRC = 4;

    // Enable ADC cores (from reference)
    AD1CONbits.ON = 1;
    while(AD1CONbits.ADRDY == 0);
    AD2CONbits.ON = 1;
    while(AD2CONbits.ADRDY == 0);

    // ADC interrupt: use BEMF channel completion
    // We read all channels each PWM cycle, process in ISR
    _AD1CH0IP = 6;    // Priority 6 (below PWM fault)
    _AD1CH0IF = 0;
    _AD1CH0IE = 0;    // Enabled later by service init
}
```

**BEMF Channel Multiplexing:**

Since BEMF_B (RB2=AD2AN4) and BEMF_C (RB5=AD2AN2) share AD2CH0, we must
switch `AD2CH0CONbits.PINSEL` at each commutation step to select the floating
phase. BEMF_A (RA4=AD1AN1) is always on AD1CH0.

```c
// Call at each commutation to configure ADC for the floating phase
void HAL_ADC_SelectBEMFChannel(uint8_t floatingPhase)
{
    switch(floatingPhase) {
        case 0: // Phase A floating — AD1CH0 already set to PINSEL=1
            AD1CH0CONbits.PINSEL = 1;   // AD1AN1 (RA4)
            break;
        case 1: // Phase B floating — switch AD2CH0 to BEMF_B
            AD2CH0CONbits.PINSEL = 4;   // AD2AN4 (RB2)
            break;
        case 2: // Phase C floating — switch AD2CH0 to BEMF_C
            AD2CH0CONbits.PINSEL = 2;   // AD2AN2 (RB5)
            break;
    }
}

// Returns raw 12-bit ADC value for specified phase (0=A, 1=B, 2=C)
// Must call HAL_ADC_SelectBEMFChannel() first at each commutation step
uint16_t HAL_ADC_ReadBEMF(uint8_t phase)
{
    switch(phase) {
        case 0: return AD1CH0DATA;   // Phase A (AD1AN1 via AD1CH0)
        case 1: return AD2CH0DATA;   // Phase B (AD2AN4 via AD2CH0)
        case 2: return AD2CH0DATA;   // Phase C (AD2AN2 via AD2CH0, muxed)
        default: return 0;
    }
}

uint16_t HAL_ADC_ReadVbus(void)
{
    return AD1CH4DATA;               // Vbus (AD1AN6)
}

uint16_t HAL_ADC_ReadPot(void)
{
    return AD1CH1DATA;               // Potentiometer (dev board)
}
```

**Acceptance:** X2C-Scope or UART shows ADC values changing when motor is
manually rotated. Vbus reading matches multimeter. Pot reading sweeps 0-4095.

---

#### Task 1.5: Open-Loop Ramp Startup

**Goal:** Motor spins using forced commutation at increasing speed.

**State machine (garuda_service.c):**

```c
typedef enum {
    ESC_POWER_ON_RESET = 0,
    ESC_IDLE,
    ESC_ARMED,
    ESC_ALIGN,
    ESC_OPEN_LOOP_RAMP,
    ESC_CLOSED_LOOP,
    ESC_BRAKING,
    ESC_FAULT
} ESC_STATE_T;

// ---- Called from ADC ISR (every PWM cycle = 41.67 us at 24 kHz) ----
void GARUDA_StateMachine(GARUDA_DATA_T *g)
{
    switch(g->state)
    {
        case ESC_POWER_ON_RESET:
            HAL_PWM_DisableAllOutputs();
            g->state = ESC_IDLE;
            break;

        case ESC_IDLE:
            // Wait for arm signal (throttle = 0 for 500ms)
            if(g->throttle == 0 && g->armTimer >= ARM_TIME_COUNTS)
                g->state = ESC_ARMED;
            break;

        case ESC_ARMED:
            // Wait for throttle > 0
            if(g->throttle > THROTTLE_MIN) {
                g->alignTimer = 0;
                g->currentStep = 0;
                g->state = ESC_ALIGN;
            }
            break;

        case ESC_ALIGN:
            // Hold first commutation step for alignment period
            HAL_PWM_SetCommutationStep(g->currentStep, g->alignDuty);
            g->alignTimer++;
            if(g->alignTimer >= g->alignTimeCounts) {
                g->stepTimer = 0;
                g->stepPeriod = g->initialStepPeriod;  // Slow initial speed
                g->state = ESC_OPEN_LOOP_RAMP;
            }
            break;

        case ESC_OPEN_LOOP_RAMP:
            // Forced commutation at increasing frequency
            g->stepTimer++;
            if(g->stepTimer >= g->stepPeriod) {
                g->stepTimer = 0;
                g->currentStep = (g->currentStep + 1) % 6;
                HAL_PWM_SetCommutationStep(g->currentStep, g->rampDuty);

                // Accelerate: decrease step period
                if(g->stepPeriod > g->minStepPeriod) {
                    g->stepPeriod -= g->rampAcceleration;
                }
            }
            // Transition to closed-loop handled in Phase 2
            break;

        case ESC_CLOSED_LOOP:
            // Phase 2 implementation
            break;

        case ESC_BRAKING:
            HAL_PWM_DisableAllOutputs();
            break;

        case ESC_FAULT:
            HAL_PWM_DisableAllOutputs();
            // Require throttle=0 for 500ms to clear
            break;
    }
}
```

**Open-Loop Parameters:**
```c
// garuda_config.h
#define MOTOR_POLE_PAIRS           7       // Typical drone motor (14-pole)
#define ALIGN_TIME_MS              200     // Rotor alignment hold time
#define ALIGN_DUTY_PERCENT         5       // Low duty for alignment
#define INITIAL_ERPM               2000    // Starting electrical RPM
#define RAMP_TARGET_ERPM           15000   // Transition to closed-loop
#define RAMP_ACCEL_ERPM_PER_S      5000    // Acceleration rate

// Derived:
#define ALIGN_TIME_COUNTS       (ALIGN_TIME_MS * 1000 / LOOPTIME_MICROSEC)
#define ALIGN_DUTY              (LOOPTIME_TCY * ALIGN_DUTY_PERCENT / 100)
// Step period in PWM cycles for given eRPM:
// step_period = (60 * PWM_FREQ) / (eRPM * 6)
// At 2000 eRPM: (60 * 24000) / (2000 * 6) = 120 PWM cycles per step
// At 15000 eRPM: (60 * 24000) / (15000 * 6) = 16 PWM cycles per step
#define INITIAL_STEP_PERIOD     ((60UL * PWMFREQUENCY_HZ) / (INITIAL_ERPM * 6))
#define MIN_STEP_PERIOD         ((60UL * PWMFREQUENCY_HZ) / (RAMP_TARGET_ERPM * 6))
```

**ISR Structure (from reference pattern):**

The reference uses `MC1_ADC_INTERRUPT` macro (defined in adc.h) which resolves
to `_AD1CH1Interrupt` (dual shunt) or `_AD1CH3Interrupt` (single shunt).
For Garuda, we choose our own ADC ISR source based on which BEMF channel
completes last. We use a similar macro pattern:

```c
// garuda_adc.h:
// BEMF_A uses AD1CH0 (always sampled). Use AD1CH0 as the ISR trigger
// since it completes with PWM1 trigger along with Vbus (AD1CH4) and Pot (AD1CH1).
#define GARUDA_ADC_INTERRUPT        _AD1CH0Interrupt
#define GARUDA_EnableADCInterrupt() _AD1CH0IE = 1
#define GARUDA_DisableADCInterrupt() _AD1CH0IE = 0
#define GARUDA_ClearADCIF()         _AD1CH0IF = 0

// garuda_service.c:
// ADC completion ISR — runs every PWM cycle
void __attribute__((__interrupt__, no_auto_psv)) GARUDA_ADC_INTERRUPT()
{
    // 1. Read sensor inputs
    g_data.bemfRaw = HAL_ADC_ReadBEMF(
        commutation_table[g_data.currentStep].floatingPhase);
    g_data.vbusRaw = HAL_ADC_ReadVbus();

    // 2. Run state machine
    GARUDA_StateMachine(&g_data);

    // 3. Clear flag
    GARUDA_ClearADCIF();
}

// PWM Fault ISR (from reference, adapted)
void __attribute__((__interrupt__, no_auto_psv)) _PWM1Interrupt(void)
{
    HAL_PWM_DisableAllOutputs();
    PG1FPCIbits.SWTERM = 1;  // Clear PCI fault latch
    PG2FPCIbits.SWTERM = 1;
    PG3FPCIbits.SWTERM = 1;
    g_data.state = ESC_FAULT;
    g_data.faultCode = FAULT_OVERCURRENT_HW;
    _PWM1IF = 0;
}

// Timer1 ISR — fires every 100µs (from reference timer1.h: TIMER1_PERIOD_uSec=100)
static uint8_t timer1_subcount = 0;

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    BoardService();           // Button handling, heartbeat LED (has its own rate counter)
    BoardServiceStepIsr();    // Increment board service tick counter

    // 1ms system tick: Timer1 fires at 100µs, so 10 interrupts = 1ms
    timer1_subcount++;
    if (timer1_subcount >= 10) {
        timer1_subcount = 0;
        g_data.systemTickMs++;
    }

    TIMER1_InterruptFlagClear();
}
```

**Acceptance:** Connect motor to dev board. Press SW1 to start. Motor aligns
briefly then ramps up. Oscilloscope shows 6-step commutation pattern. Motor
spins smoothly at fixed duty. Press SW1 again to stop.

---

### Phase 2: Sensorless Closed-Loop

#### Task 2.1: BEMF Zero-Crossing Detection

**Goal:** Detect when floating phase voltage crosses Vbus/2.

**`bemf.c`:**
```c
typedef struct {
    uint16_t vbus;             // Latest Vbus ADC reading
    uint16_t vbusHalf;         // Vbus / 2 (virtual neutral reference)
    uint16_t bemfRaw;          // Latest BEMF ADC reading
    int16_t  bemfCentered;     // BEMF - Vbus/2 (signed, + or -)
    uint8_t  zcDetected;       // Zero-crossing flag
    uint8_t  lastPolarity;     // Previous sample polarity (0=neg, 1=pos)
    uint8_t  majorityCount;    // Consecutive same-polarity count
    uint8_t  blankingActive;   // Blanking period active flag
    uint16_t blankingCounter;  // Blanking timer (PWM cycles)
} BEMF_STATE_T;

#define ZC_MAJORITY_THRESHOLD  3    // 3 consecutive samples for valid ZC

// Called every PWM cycle from ADC ISR
// Returns 1 if zero-crossing detected, 0 otherwise
uint8_t BEMF_ProcessSample(BEMF_STATE_T *b, uint8_t expectedPolarity)
{
    b->zcDetected = 0;

    // 1. Update Vbus/2 reference (low-pass filtered)
    b->vbusHalf = b->vbus >> 1;

    // 2. Calculate centered BEMF (signed deviation from neutral)
    b->bemfCentered = (int16_t)b->bemfRaw - (int16_t)b->vbusHalf;

    // 3. Skip during blanking period (post-commutation noise)
    if(b->blankingActive) {
        b->blankingCounter--;
        if(b->blankingCounter == 0)
            b->blankingActive = 0;
        return 0;
    }

    // 4. Determine current polarity
    uint8_t currentPolarity = (b->bemfCentered > 0) ? 1 : 0;

    // 5. Majority filter: require N consecutive samples matching expected
    if(currentPolarity == expectedPolarity) {
        b->majorityCount++;
        if(b->majorityCount >= ZC_MAJORITY_THRESHOLD) {
            b->zcDetected = 1;
            b->majorityCount = 0;
        }
    } else {
        b->majorityCount = 0;
    }

    return b->zcDetected;
}

void BEMF_StartBlanking(BEMF_STATE_T *b, uint16_t blankingCycles)
{
    b->blankingActive = 1;
    b->blankingCounter = blankingCycles;
    b->majorityCount = 0;
}
```

**Comparator-Assisted ZC (hybrid approach):**

Each comparator module (CMP1/CMP2/CMP3) has its own input pin domain —
you CANNOT route CMP1B into CMP3's INSEL. The correct architecture uses
3 separate comparators, one per phase, and enables only the one monitoring
the floating phase:

```
CMP1: Input CMP1B (RA4)  → BEMF Phase A, DAC1 ref = Vbus/2
CMP2: Input CMP2B (RB2)  → BEMF Phase B, DAC2 ref = Vbus/2
CMP3: Input CMP3B (RB5)  → BEMF Phase C, DAC3 ref = Vbus/2
```

From reference port_config.c pin comments:
- RA4 = OA1IN+/AD1AN1/CMP1B/RP5/RA4 → CMP1, INSEL=1 (B input)
- RB2 = OA2IN+/AD2AN4/CMP2B/RP19/RB2 → CMP2, INSEL=1 (B input)
- RB5 = OA3IN+/AD2AN2/CMP3B/RP22/RB5 → CMP3, INSEL=1 (B input)

```c
// hal_comparator.c — uses all 3 comparators, one per phase
// Each DACx provides Vbus/2 reference to its paired CMPx

void HAL_CMP_Initialize(void)
{
    // DAC calibration (from reference cmp.c — applies to all DACs)
    uint32_t *FPDMDACaddress = (uint32_t*)(0x7F20B0);
    uint32_t FPDMDACdata = *FPDMDACaddress;
    DACCTRL1 = 0;
    DACCTRL1bits.NEGINLADJ = (FPDMDACdata & 0x0000FF00) >> 8;
    DACCTRL1bits.DNLADJ    = (FPDMDACdata & 0x000000FF);
    DACCTRL1bits.POSINLADJ = (FPDMDACdata & 0x00FF0000) >> 16;
    DACCTRL1bits.FCLKDIV = 0b111;
    DACCTRL2 = 0;

    // CMP1 — Phase A ZC (RA4 = CMP1B)
    DAC1CON = 0;
    DAC1CONbits.INSEL = 1;      // CMP1B input (RA4)
    DAC1CONbits.CMPPOL = 0;     // Non-inverted (set per step)
    DAC1CONbits.HYSSEL = 0b11;  // 45mV hysteresis
    DAC1CONbits.FLTREN = 0;     // No digital filter
    DAC1CONbits.IRQM = 0;       // Interrupts disabled (polled or SW-triggered)
    DAC1DAT = 0;

    // CMP2 — Phase B ZC (RB2 = CMP2B)
    DAC2CON = 0;
    DAC2CONbits.INSEL = 1;      // CMP2B input (RB2)
    DAC2CONbits.CMPPOL = 0;
    DAC2CONbits.HYSSEL = 0b11;
    DAC2CONbits.FLTREN = 0;
    DAC2CONbits.IRQM = 0;
    DAC2DAT = 0;

    // CMP3 — Phase C ZC (RB5 = CMP3B)
    DAC3CON = 0;
    DAC3CONbits.INSEL = 1;      // CMP3B input (RB5)
    DAC3CONbits.CMPPOL = 0;
    DAC3CONbits.HYSSEL = 0b11;
    DAC3CONbits.FLTREN = 0;
    DAC3CONbits.IRQM = 0;
    DAC3DAT = 0;
}

// Enable only the comparator for the floating phase
void HAL_CMP_SetupForZC(uint8_t floatingPhase, uint16_t dacReference,
                          uint8_t zcPolarity)
{
    // Disable all comparators first
    DAC1CONbits.DACEN = 0;
    DAC2CONbits.DACEN = 0;
    DAC3CONbits.DACEN = 0;

    // Enable only the floating phase comparator with correct polarity
    // zcPolarity: 0=falling (CMPPOL=1, invert), 1=rising (CMPPOL=0)
    switch(floatingPhase) {
        case 0: // Phase A floating → CMP1
            DAC1CONbits.CMPPOL = (zcPolarity == 0) ? 1 : 0;
            DAC1DATbits.DACDAT = dacReference;
            DAC1CONbits.DACEN = 1;
            break;
        case 1: // Phase B floating → CMP2
            DAC2CONbits.CMPPOL = (zcPolarity == 0) ? 1 : 0;
            DAC2DATbits.DACDAT = dacReference;
            DAC2CONbits.DACEN = 1;
            break;
        case 2: // Phase C floating → CMP3
            DAC3CONbits.CMPPOL = (zcPolarity == 0) ? 1 : 0;
            DAC3DATbits.DACDAT = dacReference;
            DAC3CONbits.DACEN = 1;
            break;
    }
    DACCTRL1bits.ON = 1;
}

// Read comparator output for floating phase
uint8_t HAL_CMP_ReadZC(uint8_t floatingPhase)
{
    switch(floatingPhase) {
        case 0: return DAC1CONbits.CMPSTAT;
        case 1: return DAC2CONbits.CMPSTAT;
        case 2: return DAC3CONbits.CMPSTAT;
        default: return 0;
    }
}
```

Note: CMP3 is still available for overcurrent detection if needed, but in
ZC hybrid mode it monitors Phase C BEMF. For overcurrent, use a separate
ADC channel with software threshold or dedicate CMP3 to overcurrent and
use only CMP1/CMP2 + ADC for the 3rd phase ZC.

**Acceptance:** With motor spinning (open-loop), UART outputs BEMF ADC values.
ZC transitions visible in data. LED toggles on each zero-crossing event.

---

#### Task 2.2: Commutation Timing Engine

**Goal:** After ZC detection, schedule next commutation at correct time.

**`timing.c`:**
```c
typedef struct {
    uint32_t lastZcTimestamp;      // Timer capture at last ZC
    uint32_t lastCommTimestamp;    // Timer capture at last commutation
    uint32_t stepPeriod;          // Measured time between commutations
    uint32_t halfStepPeriod;      // stepPeriod / 2 (30 electrical degrees)
    uint32_t commDelay;           // Delay from ZC to next commutation
    uint8_t  advanceDegrees;      // Timing advance (0-30 degrees)
    uint32_t stepHistory[6];      // Last 6 step periods for averaging
    uint8_t  historyIndex;
    uint8_t  speedValid;          // 1 if timing is reliable
} TIMING_STATE_T;

// Called when ZC is detected
void TIMING_OnZeroCrossing(TIMING_STATE_T *t, uint32_t timestamp)
{
    // Calculate time from last commutation to this ZC
    uint32_t zcDelay = timestamp - t->lastCommTimestamp;

    // ZC occurs at 30 degrees after commutation
    // So full step = 2 * zcDelay (ideally)
    // But we use measured step period for better tracking
    t->halfStepPeriod = zcDelay;

    // Schedule next commutation:
    // delay = 30_degrees - advance
    // In time units: delay = halfStepPeriod * (30 - advance) / 30
    t->commDelay = t->halfStepPeriod * (30 - t->advanceDegrees) / 30;

    // Update step period history
    t->stepHistory[t->historyIndex] = t->halfStepPeriod * 2;
    t->historyIndex = (t->historyIndex + 1) % 6;

    // Average step period for speed calculation
    uint32_t sum = 0;
    for(int i = 0; i < 6; i++) sum += t->stepHistory[i];
    t->stepPeriod = sum / 6;

    t->speedValid = 1;

    // Start commutation delay timer
    HAL_Timer_StartOneShot(t->commDelay);
}

// Called from commutation timer ISR
void TIMING_OnCommutationTimer(TIMING_STATE_T *t, uint8_t *currentStep,
                                uint16_t duty, BEMF_STATE_T *bemf)
{
    // Advance to next step
    *currentStep = (*currentStep + 1) % 6;
    HAL_PWM_SetCommutationStep(*currentStep, duty);

    t->lastCommTimestamp = HAL_Timer_GetTimestamp();

    // Start blanking period (suppress noise after commutation)
    // COMMUTATION_TIMER_FREQ = 12500000 (Timer1 at 100MHz/8)
    // PWM cycles per timer tick: COMMUTATION_TIMER_FREQ / PWMFREQUENCY_HZ
    #define TIMER_TICKS_PER_PWM_CYCLE  (COMMUTATION_TIMER_FREQ / PWMFREQUENCY_HZ)
    uint16_t blankingCycles = t->stepPeriod * BLANKING_PERCENT / (100 * TIMER_TICKS_PER_PWM_CYCLE);
    if(blankingCycles < MIN_BLANKING_CYCLES) blankingCycles = MIN_BLANKING_CYCLES;
    BEMF_StartBlanking(bemf, blankingCycles);
}

// eRPM calculation from step period
uint32_t TIMING_GetERPM(TIMING_STATE_T *t)
{
    if(t->stepPeriod == 0 || !t->speedValid) return 0;
    // eRPM = 60 / (stepPeriod_seconds * 6)
    //       = 60 * timer_freq / (stepPeriod * 6)
    return (60UL * COMMUTATION_TIMER_FREQ) / (t->stepPeriod * 6UL);
}

// Mechanical RPM
uint32_t TIMING_GetMechRPM(TIMING_STATE_T *t)
{
    return TIMING_GetERPM(t) / MOTOR_POLE_PAIRS;
}
```

**Timing parameters:**
```c
// garuda_config.h
#define TIMING_ADVANCE_DEG      15       // Default 15 degrees
#define BLANKING_PERCENT        10       // 10% of step period
#define MIN_BLANKING_CYCLES     2        // Minimum blanking (PWM cycles)
#define DEMAG_PERCENT           15       // 15% demagnetization window

// Commutation timer: We use SCCP1 timer at FCY (200 MHz) with 1:16 prescaler
// giving 12.5 MHz (80ns resolution). This is separate from Timer1 (system tick).
#define COMMUTATION_TIMER_FREQ  12500000UL  // 200 MHz / 16 = 12.5 MHz
// Conversions:
#define TIMER_TICKS_PER_PWM_CYCLE  (COMMUTATION_TIMER_FREQ / PWMFREQUENCY_HZ)
// At 24kHz: 12500000 / 24000 = 520.8 → ~521 timer ticks per PWM cycle
```

**Acceptance:** Motor transitions from open-loop to closed-loop sensorless
at ~15000 eRPM. Stable across throttle range. Timing advance adjustable.

---

#### Task 2.3: Desynchronization Detection and Recovery

**`desync.c`:**
```c
typedef struct {
    uint32_t expectedStepPeriod;   // Expected from averaging
    uint32_t variance;             // Step period variance
    uint8_t  missedZcCount;        // Consecutive missed ZC events
    uint8_t  forcedCommCount;      // Forced commutations (no ZC)
    uint8_t  desyncState;          // 0=OK, 1=WARNING, 2=DESYNC, 3=RECOVERY
    uint16_t recoveryTimer;
    uint32_t stressCount;          // Total desync events (for EDT telemetry)
} DESYNC_STATE_T;

#define DESYNC_VARIANCE_THRESHOLD   25    // % variance that triggers warning
#define DESYNC_MISS_THRESHOLD       3     // Consecutive misses = desync
#define DESYNC_RECOVERY_ATTEMPTS    3     // Max recovery attempts before fault
#define ZC_TIMEOUT_PERCENT          150   // % of expected period before forced comm

// Called every PWM cycle when in closed-loop
void DESYNC_Check(DESYNC_STATE_T *d, TIMING_STATE_T *t, uint32_t timeSinceLastComm)
{
    // Check for missed zero-crossing (timeout)
    uint32_t timeout = t->stepPeriod * ZC_TIMEOUT_PERCENT / 100;
    if(timeSinceLastComm > timeout && d->desyncState == 0) {
        // Force commutation
        d->missedZcCount++;
        d->forcedCommCount++;
        if(d->missedZcCount >= DESYNC_MISS_THRESHOLD) {
            d->desyncState = 2;  // DESYNC confirmed
            d->stressCount++;
        }
    }

    // Check step period variance
    // (Compare latest step to running average)
    if(t->speedValid) {
        int32_t deviation = (int32_t)t->stepHistory[t->historyIndex] -
                           (int32_t)t->stepPeriod;
        uint32_t pctVariance = (uint32_t)(100 * abs(deviation)) / t->stepPeriod;
        if(pctVariance > DESYNC_VARIANCE_THRESHOLD) {
            d->desyncState = 1;  // WARNING
        }
    }
}

// Recovery: reduce duty, attempt re-synchronization
ESC_STATE_T DESYNC_Recover(DESYNC_STATE_T *d, uint16_t *duty)
{
    if(d->desyncState == 2) {
        // Reduce duty by 50%
        *duty = *duty >> 1;
        d->recoveryTimer++;

        // Wait for 6 steps to resync
        if(d->recoveryTimer > 6 * 3) {  // 3 electrical revolutions
            d->recoveryTimer = 0;
            d->desyncState = 0;          // Attempt cleared
            d->missedZcCount = 0;
            d->forcedCommCount = 0;
        }

        // After N failed attempts → FAULT
        if(d->forcedCommCount > DESYNC_RECOVERY_ATTEMPTS * DESYNC_MISS_THRESHOLD) {
            return ESC_FAULT;
        }
    }
    return ESC_CLOSED_LOOP;
}
```

---

#### Task 2.4: Closed-Loop Integration

**Updated state machine `ESC_CLOSED_LOOP` case:**
```c
case ESC_CLOSED_LOOP:
{
    // 1. Read BEMF from floating phase
    uint8_t floatPhase = commutation_table[g->currentStep].floatingPhase;
    uint8_t zcPolarity = commutation_table[g->currentStep].zcPolarity;
    g->bemf.bemfRaw = HAL_ADC_ReadBEMF(floatPhase);
    g->bemf.vbus = HAL_ADC_ReadVbus();

    // 2. Check for zero-crossing
    if(BEMF_ProcessSample(&g->bemf, zcPolarity)) {
        uint32_t now = HAL_Timer_GetTimestamp();
        TIMING_OnZeroCrossing(&g->timing, now);
        g->desync.missedZcCount = 0;  // Reset miss counter
    }

    // 3. Desync monitoring
    uint32_t elapsed = HAL_Timer_GetTimestamp() - g->timing.lastCommTimestamp;
    DESYNC_Check(&g->desync, &g->timing, elapsed);
    ESC_STATE_T recovery = DESYNC_Recover(&g->desync, &g->duty);
    if(recovery == ESC_FAULT) {
        g->faultCode = FAULT_DESYNC;
        g->state = ESC_FAULT;
        break;
    }

    // 4. Throttle → duty mapping (PI controller or linear)
    g->duty = GARUDA_ThrottleToDuty(g->throttle, g->timing.stepPeriod);

    // 5. Update duty on active phase
    HAL_PWM_SetCommutationStep(g->currentStep, g->duty);

    // 6. Stop condition
    if(g->throttle == 0) {
        g->brakeTimer = 0;
        g->state = ESC_BRAKING;
    }
    break;
}
```

**Commutation timer ISR:**
```c
// SCCP timer compare match — fires when commutation delay expires
void __attribute__((__interrupt__, no_auto_psv)) _CCT1Interrupt(void)
{
    TIMING_OnCommutationTimer(&g_data.timing, &g_data.currentStep,
                               g_data.duty, &g_data.bemf);
    _CCT1IF = 0;
}
```

**Acceptance:** Motor runs closed-loop sensorless from ~500 mechanical RPM to
max speed. Responds to throttle changes. Recovers from brief load transients.
Desync detection triggers on stall. Test with 3+ different motors.

---

### Phase 3: DShot Protocol

#### Task 3.1: DShot Receiver (Input Capture + DMA)

**`dshot.c`:**
```c
// DShot bit timing (in Input Capture timer ticks)
// Timer runs at 200 MHz (CLK1), resolution = 5ns
// DShot600: bit period = 1.667 us = 333 ticks
// T0H = 625ns = 125 ticks, T1H = 1250ns = 250 ticks
// Discrimination threshold = ~187 ticks (midpoint)

typedef struct {
    uint16_t captureBuffer[32];  // DMA captures (16 bits * 2 edges)
    uint16_t rawFrame;           // Decoded 16-bit DShot frame
    uint16_t throttle;           // 11-bit throttle (0-2047)
    uint8_t  telemetryBit;       // Telemetry request bit
    uint8_t  crc;                // 4-bit CRC
    uint8_t  valid;              // Frame validation result
    uint16_t rate;               // Auto-detected DShot rate (150/300/600/1200)
    uint16_t bitThreshold;       // T0/T1 discrimination threshold
    uint32_t lastFrameTimestamp;  // For signal loss detection
} DSHOT_STATE_T;

#define DSHOT_TIMER_FREQ     200000000UL  // 200 MHz

// Thresholds for each DShot rate (in timer ticks at 200 MHz)
#define DSHOT150_THRESHOLD   (DSHOT_TIMER_FREQ / 150000 * 3 / 8)   // ~500
#define DSHOT300_THRESHOLD   (DSHOT_TIMER_FREQ / 300000 * 3 / 8)   // ~250
#define DSHOT600_THRESHOLD   (DSHOT_TIMER_FREQ / 600000 * 3 / 8)   // ~125
#define DSHOT1200_THRESHOLD  (DSHOT_TIMER_FREQ / 1200000 * 3 / 8)  // ~62

// CRC validation (from spec §2.2.3)
uint8_t DSHOT_CalculateCRC(uint16_t value)
{
    return (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
}

// Decode captured pulse widths into DShot frame
uint8_t DSHOT_DecodeFrame(DSHOT_STATE_T *d)
{
    d->rawFrame = 0;

    // Each bit: measure high-time from rising-to-falling edge capture
    for(int i = 0; i < 16; i++) {
        uint16_t highTime = d->captureBuffer[i * 2 + 1] - d->captureBuffer[i * 2];
        if(highTime > d->bitThreshold) {
            d->rawFrame |= (1 << (15 - i));  // MSB first
        }
    }

    // Extract fields
    d->throttle = (d->rawFrame >> 5) & 0x07FF;    // Bits [15:5]
    d->telemetryBit = (d->rawFrame >> 4) & 0x01;  // Bit [4]
    d->crc = d->rawFrame & 0x0F;                   // Bits [3:0]

    // Validate CRC
    uint16_t value = d->rawFrame >> 4;  // 12-bit value (throttle + telem)
    uint8_t calcCrc = DSHOT_CalculateCRC(value);
    d->valid = (calcCrc == d->crc) ? 1 : 0;

    if(d->valid) {
        d->lastFrameTimestamp = HAL_Timer_GetTimestamp();
    }

    return d->valid;
}

// Auto-detect DShot rate from first frame bit period
void DSHOT_AutoDetectRate(DSHOT_STATE_T *d)
{
    // Measure first bit total period (rising to next rising)
    uint16_t bitPeriod = d->captureBuffer[2] - d->captureBuffer[0];

    // Classify rate
    if(bitPeriod > 1000) {       // >5us → DShot150
        d->rate = 150;
        d->bitThreshold = DSHOT150_THRESHOLD;
    } else if(bitPeriod > 500) { // >2.5us → DShot300
        d->rate = 300;
        d->bitThreshold = DSHOT300_THRESHOLD;
    } else if(bitPeriod > 250) { // >1.25us → DShot600
        d->rate = 600;
        d->bitThreshold = DSHOT600_THRESHOLD;
    } else {                     // DShot1200
        d->rate = 1200;
        d->bitThreshold = DSHOT1200_THRESHOLD;
    }
}
```

**Test vectors (runtime validation):**
```c
// DShot CRC is computed over the 12-bit value = (throttle << 1) | telemetryBit
// CRC = (val ^ (val >> 4) ^ (val >> 8)) & 0x0F
// Test in startup self-check:
//   value=0x000 → CRC=0x0, frame=0x0000  (throttle=0, telem=0)
//   value=0x060 → CRC=0x6, frame=0x0606  (throttle=48, telem=0)
//   value=0x061 → CRC=0x7, frame=0x0617  (throttle=48, telem=1)
void DSHOT_SelfTest(void)
{
    assert(DSHOT_CalculateCRC(0x000) == 0x0);
    assert(DSHOT_CalculateCRC(0x060) == 0x6);
    assert(DSHOT_CalculateCRC(0x061) == 0x7);
}
```

---

#### Task 3.2: Bidirectional DShot + eRPM Telemetry

**`dshot_bidir.c`:**
```c
// GCR (5-bit) encoding table
static const uint8_t gcr_encode[16] = {
    0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
    0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F
};

// Encode eRPM for bidirectional DShot response
// eRPM encoded as: mantissa << exponent (compressed floating point)
// Response: 12-bit value → 4 nibbles → 4×5=20 GCR bits + checksum
uint32_t DSHOT_EncodeERPM(uint32_t erpm)
{
    // Convert eRPM to period (us)
    // period_us = 60000000 / erpm  (or use direct eRPM encoding)
    uint16_t period = (erpm > 0) ? (60000000UL / erpm) : 0xFFF;
    if(period > 0xFFF) period = 0xFFF;

    // Encode as mantissa/exponent: find shift to fit in 9 bits
    uint8_t exponent = 0;
    uint16_t mantissa = period;
    while(mantissa > 0x1FF && exponent < 7) {
        mantissa >>= 1;
        exponent++;
    }

    // 12-bit telemetry value: [exp:3][mantissa:9]
    uint16_t telem = (exponent << 9) | (mantissa & 0x1FF);

    // Calculate checksum nibble
    uint8_t crc = (~(telem ^ (telem >> 4) ^ (telem >> 8))) & 0x0F;

    // 16-bit encoded value: [telem:12][crc:4]
    uint16_t encoded = (telem << 4) | crc;

    // GCR encode 4 nibbles → 20 bits
    uint32_t gcr = 0;
    for(int i = 3; i >= 0; i--) {
        uint8_t nibble = (encoded >> (i * 4)) & 0x0F;
        gcr = (gcr << 5) | gcr_encode[nibble];
    }

    return gcr;  // 20-bit GCR encoded response
}

// Transmit GCR response by bit-banging the DShot pin as output
// Must start 30us ±5us after FC frame ends
// Total response duration: ~28us at DShot600
void DSHOT_TransmitResponse(uint32_t gcrData)
{
    // Switch DShot pin to output mode
    TRISDbits.TRISD8 = 0;  // Output

    // Bit-bang 21 GCR bits (20 data + stop)
    // Each bit period matches DShot rate
    // This is timing-critical — may need to use PWM or timer output
    // instead of bit-bang for production

    // ... implementation depends on chosen output method ...

    // Switch back to input
    TRISDbits.TRISD8 = 1;  // Input
    // Re-enable Input Capture
}
```

---

#### Task 3.3: DShot Command Handler

**`dshot_commands.c`:**
```c
typedef enum {
    DSHOT_CMD_DISARM = 0,
    DSHOT_CMD_BEEP1 = 1,
    DSHOT_CMD_BEEP2 = 2,
    DSHOT_CMD_BEEP3 = 3,
    DSHOT_CMD_BEEP4 = 4,
    DSHOT_CMD_BEEP5 = 5,
    DSHOT_CMD_ESC_INFO = 6,
    DSHOT_CMD_SPIN_DIR_1 = 7,
    DSHOT_CMD_SPIN_DIR_2 = 8,
    DSHOT_CMD_3D_MODE_OFF = 9,
    DSHOT_CMD_3D_MODE_ON = 10,
    DSHOT_CMD_SETTINGS_REQ = 11,
    DSHOT_CMD_SAVE_SETTINGS = 12,
    DSHOT_CMD_EDT_ENABLE = 13,
    DSHOT_CMD_EDT_DISABLE = 14,
    // ... extended commands up to 47
    DSHOT_THROTTLE_MIN = 48,
    DSHOT_THROTTLE_MAX = 2047,
} DSHOT_CMD_T;

typedef struct {
    uint8_t  lastCommand;
    uint8_t  repeatCount;        // Some commands need 6x or 10x repeats
    uint16_t repeatTimeout;      // Timeout between repeats (ms)
    uint32_t lastCommandTime;
} DSHOT_CMD_STATE_T;

#define CMD_REPEAT_REQUIRED     6    // Settings commands need 6 repeats
#define CMD_REPEAT_TIMEOUT_MS   100  // Max time between repeats

void DSHOT_ProcessCommand(DSHOT_CMD_STATE_T *cmd, uint16_t throttle,
                           GARUDA_DATA_T *g)
{
    if(throttle >= DSHOT_THROTTLE_MIN) {
        // Throttle command (48-2047 → 0-100%)
        g->throttle = throttle - DSHOT_THROTTLE_MIN;
        cmd->repeatCount = 0;
        return;
    }

    // Command (0-47)
    uint32_t now = g->systemTickMs;

    // Track repeat count for multi-frame commands
    if(throttle == cmd->lastCommand &&
       (now - cmd->lastCommandTime) < CMD_REPEAT_TIMEOUT_MS) {
        cmd->repeatCount++;
    } else {
        cmd->repeatCount = 1;
    }
    cmd->lastCommand = throttle;
    cmd->lastCommandTime = now;

    switch(throttle) {
        case DSHOT_CMD_DISARM:
            g->throttle = 0;
            g->state = ESC_IDLE;
            break;

        case DSHOT_CMD_BEEP1: case DSHOT_CMD_BEEP2:
        case DSHOT_CMD_BEEP3: case DSHOT_CMD_BEEP4:
        case DSHOT_CMD_BEEP5:
            if(g->state == ESC_IDLE || g->state == ESC_ARMED)
                BEEP_Play(throttle);  // Motor as speaker
            break;

        case DSHOT_CMD_SPIN_DIR_1:
            if(cmd->repeatCount >= CMD_REPEAT_REQUIRED)
                g->config.direction = 0;
            break;

        case DSHOT_CMD_SPIN_DIR_2:
            if(cmd->repeatCount >= CMD_REPEAT_REQUIRED)
                g->config.direction = 1;
            break;

        case DSHOT_CMD_3D_MODE_OFF:
            if(cmd->repeatCount >= CMD_REPEAT_REQUIRED)
                g->config.mode3d = 0;
            break;

        case DSHOT_CMD_3D_MODE_ON:
            if(cmd->repeatCount >= CMD_REPEAT_REQUIRED)
                g->config.mode3d = 1;
            break;

        case DSHOT_CMD_SAVE_SETTINGS:
            if(cmd->repeatCount >= CMD_REPEAT_REQUIRED)
                EEPROM_SaveConfig(&g->config);
            break;

        case DSHOT_CMD_EDT_ENABLE:
            if(cmd->repeatCount >= CMD_REPEAT_REQUIRED)
                g->edtEnabled = 1;
            break;

        case DSHOT_CMD_EDT_DISABLE:
            if(cmd->repeatCount >= CMD_REPEAT_REQUIRED)
                g->edtEnabled = 0;
            break;
    }
}
```

---

#### Task 3.4: Extended DShot Telemetry (EDT)

**`edt.c`:**
```c
typedef enum {
    EDT_FRAME_ERPM = 0,       // Default: eRPM (not EDT, normal bidir response)
    EDT_FRAME_TEMPERATURE,
    EDT_FRAME_VOLTAGE,
    EDT_FRAME_CURRENT,
    EDT_FRAME_DEBUG,
    EDT_FRAME_STRESS,
    EDT_FRAME_STATUS,
} EDT_FRAME_TYPE_T;

typedef struct {
    uint8_t  enabled;
    uint8_t  nextFrameType;
    uint16_t baseIntervalMs;   // Default 1000ms between EDT frames
    uint32_t lastEdtTime;
    uint8_t  statusPending;     // Immediate status frame needed
    uint16_t edtFrameCount;     // Total EDT frames sent
    uint8_t  maxFramesPerSec;   // Rate limiting (default 10)
    // Telemetry data
    uint8_t  temperatureC;
    uint16_t voltageDeciV;      // Voltage in 0.1V units
    uint16_t currentDeciA;      // Current in 0.1A units
    uint8_t  statusByte;        // Error/warning/alert flags
    uint32_t stressCount;       // From desync module
} EDT_STATE_T;

// Called from bidirectional DShot response path
// Returns 12-bit EDT frame OR 12-bit eRPM frame
uint16_t EDT_GetNextResponse(EDT_STATE_T *edt, uint32_t erpm, uint32_t now)
{
    if(!edt->enabled) {
        // Standard eRPM response
        return DSHOT_EncodeERPM(erpm);
    }

    // Status frames are immediate (on error/warning)
    if(edt->statusPending) {
        edt->statusPending = 0;
        return EDT_EncodeStatusFrame(edt->statusByte);
    }

    // Rate-limited periodic EDT frames
    if((now - edt->lastEdtTime) >= edt->baseIntervalMs) {
        edt->lastEdtTime = now;
        edt->edtFrameCount++;

        uint16_t frame;
        switch(edt->nextFrameType) {
            case EDT_FRAME_TEMPERATURE:
                frame = EDT_EncodeTempFrame(edt->temperatureC);
                edt->nextFrameType = EDT_FRAME_VOLTAGE;
                break;
            case EDT_FRAME_VOLTAGE:
                frame = EDT_EncodeVoltageFrame(edt->voltageDeciV);
                edt->nextFrameType = EDT_FRAME_CURRENT;
                break;
            case EDT_FRAME_CURRENT:
                frame = EDT_EncodeCurrentFrame(edt->currentDeciA);
                edt->nextFrameType = EDT_FRAME_STRESS;
                break;
            case EDT_FRAME_STRESS:
                frame = EDT_EncodeStressFrame(edt->stressCount);
                edt->nextFrameType = EDT_FRAME_TEMPERATURE;
                break;
            default:
                frame = DSHOT_EncodeERPM(erpm);
                edt->nextFrameType = EDT_FRAME_TEMPERATURE;
                break;
        }
        return frame;
    }

    // Default: eRPM
    return DSHOT_EncodeERPM(erpm);
}
```

---

### Phase 4: Production Features

#### Task 4.1: EEPROM Configuration (64-byte block)

```c
// eeprom.h
#define EEPROM_MAGIC        0x4D50
#define EEPROM_SCHEMA_VER   1

typedef struct __attribute__((packed)) {
    uint16_t magic;              // 0x4D50                          2 bytes
    uint8_t  schemaVersion;      // Schema version                  1
    uint8_t  motorPoles;         // Number of motor poles (def 14)  1
    uint8_t  direction;          // 0=normal, 1=reversed            1
    uint8_t  timingAdvanceDeg;   // 0-30 degrees (default 15)       1
    uint16_t pwmFrequencyHz;     // PWM frequency (default 24000)   2
    uint16_t deadTimeNs;         // Dead time in ns (default 750)   2
    uint8_t  dshotRate;          // 0=auto,1=150,2=300,3=600,4=1200 1 (enum, not raw Hz)
    uint8_t  mode3d;             // 0=off, 1=on                     1
    uint8_t  edtEnabled;         // 0=off, 1=on                     1
    uint16_t startupDutyPct;     // Open-loop startup duty (def 5%) 2
    uint16_t alignTimeMs;        // Alignment time ms (default 200) 2
    uint8_t  brakeOnStop;        // 0=coast, 1=brake                1
    uint16_t maxCurrentDeciA;    // Current limit 0.1A (def 250=25A)2
    uint8_t  tempLimitC;         // Temp limit deg C (default 100)  1
    uint16_t undervoltageDeciV;  // Under-voltage 0.1V (def 100=10V)2 (was uint8_t)
    uint16_t overvoltageDeciV;   // Over-voltage 0.1V (def 520=52V) 2 (was uint8_t, 260 overflows)
    uint16_t signalTimeoutMs;    // Signal loss timeout (def 500)   2
    uint8_t  startupTune;        // Startup beep melody (0-3)       1
    uint8_t  reserved[34];       // Future use                     34
    uint16_t crc16;              // CRC-16-CCITT over preceding     2
} GARUDA_CONFIG_T;
// Fixed fields: 2+1+1+1+1+2+2+1+1+1+2+2+1+2+1+2+2+2+1 = 28 bytes
// reserved[34] + crc16(2) = 36, total = 28 + 36 = 64 bytes
_Static_assert(sizeof(GARUDA_CONFIG_T) == 64, "Config struct must be 64 bytes");

// DShot rate enum (dshotRate field):
// 0=auto-detect, 1=DShot150, 2=DShot300, 3=DShot600, 4=DShot1200
// Decode in code: uint16_t rateHz = (dshotRate == 0) ? 0 :
//                 (uint16_t[]){0, 150, 300, 600, 1200}[dshotRate];
```

#### Task 4.2: Protection Features

All adapted from reference fault detection pattern:

```c
// overcurrent.c — uses reference's PWM PCI mechanism
// CMP3 → PCI8 → PWM fault → _PWM1Interrupt
// Reference value: CMP_REF = (ILIMIT * ADC_HALF / IPEAK) + ADC_HALF

// thermal.c — NTC on ADC channel
void THERMAL_Update(GARUDA_DATA_T *g) {
    uint16_t adcTemp = HAL_ADC_ReadTemp();
    g->edt.temperatureC = NTC_ADCtoTemp(adcTemp);  // Lookup table
    if(g->edt.temperatureC > g->config.tempLimitC) {
        g->faultCode = FAULT_OVERTEMP;
        g->state = ESC_FAULT;
    }
    // Derating: reduce max duty above tempLimit - 20C
    if(g->edt.temperatureC > (g->config.tempLimitC - 20)) {
        float derating = 1.0f - (float)(g->edt.temperatureC -
                         (g->config.tempLimitC - 20)) / 20.0f;
        g->maxDuty = (uint16_t)(MAX_DUTY * derating);
    }
}

// signal_loss.c — DShot frame timeout
void SIGNAL_LOSS_Check(GARUDA_DATA_T *g) {
    uint32_t elapsed = g->systemTickMs - g->dshot.lastFrameTimestamp;
    if(elapsed > g->config.signalTimeoutMs) {
        g->throttle = 0;
        g->state = ESC_BRAKING;
        g->edt.statusPending = 1;
        g->edt.statusByte |= STATUS_SIGNAL_LOST;
    }
}
```

#### Task 4.3: Bootloader

Memory map and entry conditions as specified. The bootloader is a separate
MPLAB X project occupying 0x0000-0x0FFF (4KB). Uses GSP protocol subset
over UART for firmware upload. Entry via:
  - GSP `ENTER_BOOTLOADER` command (0x06) over UART
  - UART RX held LOW at power-on (hardware entry)
  - Application sets a flag in RAM and resets (software entry)
Note: DShot commands are 0-47 only per spec. Bootloader entry is NOT
possible via DShot — use GSP/UART exclusively.

#### Task 4.4: GSP (Garuda Serial Protocol)

```c
// gsp.c — packet-based serial protocol over UART1
#define GSP_START_BYTE      0x24  // '$'

typedef enum {
    GSP_CMD_GET_INFO = 0x01,
    GSP_CMD_GET_CONFIG = 0x02,
    GSP_CMD_SET_CONFIG = 0x03,
    GSP_CMD_SAVE_CONFIG = 0x04,
    GSP_CMD_RESET_DEFAULT = 0x05,
    GSP_CMD_ENTER_BOOTLOADER = 0x06,
    GSP_CMD_GET_TELEMETRY = 0x07,
    GSP_CMD_MOTOR_TEST = 0x08,
    GSP_CMD_BEEP = 0x09,
} GSP_CMD_T;

// Packet: [0x24][LEN][CMD][PAYLOAD...][CRC16_LO][CRC16_HI]
```

---

### Phase 5: Advanced Features (Optional)

These are performance optimizations, not critical for initial release:

1. **Flux linkage integration** — accumulate integral of BEMF for low-speed ZC
2. **Ternary hysteresis** — 3-state current control for ripple reduction
3. **Adaptive timing advance** — speed/current/temperature-dependent advance
4. **Mixed decay** — slow/fast/mixed PWM decay based on throttle

---

## ISR Priority Map (Final)

| Priority | ISR | Source | Timing Budget | Borrowed From |
|----------|-----|--------|--------------|---------------|
| 7 | `_PWM1Interrupt` | PWM Fault PCI (overcurrent) | <200ns | Reference (verbatim) |
| 6 | `GARUDA_ADC_INTERRUPT` (`_AD1CH0Interrupt`) | ADC complete (PWM center) | <2us | Reference pattern (via macro), new logic |
| 5 | `_CCT1Interrupt` | Commutation timer match | <1us | New (timer from reference) |
| 4 | `_IC1Interrupt` / DMA | DShot Input Capture | DMA-assisted | New |
| 3 | — | DShot frame complete | <5us | New |
| 2 | — | Reserved | — | — |
| 1 | `_T1Interrupt` | Timer1 (100µs period, 10 counts → 1ms systemTick) | non-critical | Reference (verbatim) |
| 0 | Background | Main loop | unbounded | — |

---

## Data Flow Summary

```
DShot Input ──→ IC1+DMA ──→ DSHOT_DecodeFrame() ──→ throttle (0-2047)
                                                         │
                                                         ▼
Throttle ──→ GARUDA_StateMachine() ──→ duty (0-MAX_DUTY)
                    │                          │
                    │                          ▼
                    │         HAL_PWM_SetCommutationStep(step, duty)
                    │                 │
                    │                 ▼
                    │         PGxIOCON override registers ──→ FET gate drives
                    │
                    ├── BEMF_ProcessSample() ←── ADC (floating phase)
                    │         │
                    │         ▼ (zero-crossing detected)
                    │   TIMING_OnZeroCrossing() ──→ Start commutation timer
                    │                                       │
                    │                                       ▼
                    │                              _CCT1Interrupt()
                    │                                       │
                    │                                       ▼
                    │                         Advance step, update overrides
                    │
                    ├── DESYNC_Check() ──→ Fault if desynced
                    │
                    ├── THERMAL_Update() ──→ Fault if overtemp
                    │
                    └── SIGNAL_LOSS_Check() ──→ Brake if timeout

Bidirectional: After DShot RX ──→ 30us delay ──→ GCR eRPM/EDT response TX
```

---

## Build & Test Sequence

| Step | What | How to Verify |
|------|------|--------------|
| 1 | Project compiles | `make` / MPLAB build succeeds |
| 2 | MCU boots at 200 MHz | Scope CLK output, LED blinks |
| 3 | PWM outputs at 24 kHz | Scope all 6 pins, measure frequency and dead time |
| 4 | Override control works | Scope shows correct 6-step patterns per step value |
| 5 | ADC reads BEMF | X2C-Scope or UART prints, manually rotate motor |
| 6 | Bootstrap charging | Scope low-side pulses during init |
| 7 | Open-loop spin | Motor spins when SW1 pressed |
| 8 | ZC detection | LED toggles on each ZC, UART prints ZC timestamps |
| 9 | Closed-loop runs | Motor follows throttle, stable across speed range |
| 10 | DShot decode | CRC test vectors pass, Betaflight FC commands motor |
| 11 | Bidir DShot | FC reads eRPM from ESC, matches expected |
| 12 | EDT telemetry | FC displays temperature, voltage, current |
| 13 | Protection | Stall motor → fault triggers, release → recovers |
| 14 | EEPROM | Config survives power cycle |
| 15 | Bootloader | Firmware update via UART, survives bad flash |
| 16 | Demag compensation | Intentional stall → demag metric rises, duty reduces |
| 17 | External rotation | Spin motor by hand while off → ESC detects direction |
| 18 | Configurator tool | Web UI reads/writes config, shows live telemetry |

---

## NEW SECTIONS — From Reference ESC Study (v2.1 Updates)

The following sections were added after deep study of BLHeli, VESC, ASAC-ESC,
Sapog, Bluejay, and AM32 reference firmware. See `reference_esc_comparison.md`
for the full comparison.

---

### NEW Task 2.5: Demagnetization Compensation

**Source:** BLHeli_S — `Demag_Detected_Metric`, `Demag_Pwr_Off_Thresh`

**Problem:** After commutation, the previously-energized phase undergoes
demagnetization. During demag, the body diode clamps the phase voltage,
making BEMF readings invalid. Sensing too early produces false zero-crossings
that cause desynchronization.

**`demag.c`:**
```c
typedef struct {
    uint8_t  demagActive;         // Currently in demag period
    uint16_t demagExtendCount;    // Times blanking was extended this step
    uint16_t demagMetric;         // Sliding average (0-255) of demag events
    uint16_t demagPowerThreshold; // Metric level that triggers power reduction
    uint16_t demagEventTotal;     // Lifetime counter for EDT stress telemetry
} DEMAG_STATE_T;

#define DEMAG_METRIC_DECAY      1     // Subtract per clean commutation
#define DEMAG_METRIC_INCREMENT  10    // Add per detected demag event
#define DEMAG_POWER_THRESHOLD   100   // Metric value → reduce duty
#define DEMAG_FAULT_THRESHOLD   200   // Metric value → FAULT
#define DEMAG_MAX_EXTENSIONS    3     // Max blanking extensions per step

// Called after blanking period ends, BEFORE ZC detection begins
// Returns: 0 = ready to sense BEMF, 1 = still demagnetizing
uint8_t DEMAG_CheckAndExtend(DEMAG_STATE_T *d, BEMF_STATE_T *b,
                              uint8_t floatingPhase,
                              uint8_t expectedPolarity, uint16_t stepPeriod)
{
    // Read BEMF — check if polarity matches expected AFTER blanking
    uint16_t bemfRaw = HAL_ADC_ReadBEMF(floatingPhase);
    uint16_t vbusHalf = HAL_ADC_ReadVbus() >> 1;
    uint8_t currentPolarity = (bemfRaw > vbusHalf) ? 1 : 0;

    // Expected: for a falling ZC step, BEMF should start HIGH (positive)
    //           for a rising ZC step, BEMF should start LOW (negative)
    // If polarity is ALREADY at the expected ZC end → demag is masking real BEMF
    if (currentPolarity == expectedPolarity && d->demagExtendCount < DEMAG_MAX_EXTENSIONS) {
        // Still demagnetizing — extend blanking
        d->demagActive = 1;
        d->demagExtendCount++;
        d->demagMetric += DEMAG_METRIC_INCREMENT;
        if (d->demagMetric > 255) d->demagMetric = 255;

        // Extend blanking by 15% of step period
        uint16_t extension = stepPeriod * 15 / 100;
        BEMF_StartBlanking(b, extension);
        return 1;  // Not ready
    }

    // Clean commutation — decay metric
    d->demagActive = 0;
    d->demagExtendCount = 0;
    if (d->demagMetric > DEMAG_METRIC_DECAY)
        d->demagMetric -= DEMAG_METRIC_DECAY;
    else
        d->demagMetric = 0;

    return 0;  // Ready to sense BEMF
}

// Called from main state machine — returns duty reduction factor (0.0-1.0)
float DEMAG_GetDutyFactor(DEMAG_STATE_T *d)
{
    if (d->demagMetric > DEMAG_FAULT_THRESHOLD)
        return 0.0f;  // Signal to enter FAULT state
    if (d->demagMetric > DEMAG_POWER_THRESHOLD)
        return 0.75f; // Reduce duty by 25%
    return 1.0f;      // No reduction
}
```

**Integration into closed-loop:**
Insert between blanking end and ZC detection in the ADC ISR.

---

### NEW Task 2.6: External Rotation Detection

**Source:** Sapog — `motor_forced_rotation_detection.c`

**Problem:** If the drone crashes and the motor is windmilling, the ESC
must detect the rotation direction and speed before re-engaging. Starting
in the wrong direction with a spinning motor can cause destructive currents.

**`rotation_detect.c`:**
```c
typedef struct {
    uint8_t  detected;           // External rotation detected
    uint8_t  direction;          // 0=forward, 1=reverse
    uint32_t estimatedERPM;      // Estimated speed from BEMF
    uint16_t bemfSamples[3];     // Latest BEMF on each phase
    uint8_t  sampleCount;        // Samples collected
    uint8_t  phaseSequence[6];   // Detected commutation order
    uint8_t  sequenceIndex;
} ROTATION_DETECT_T;

#define ROTATION_BEMF_THRESHOLD  100   // ADC counts above noise floor
#define ROTATION_SAMPLE_INTERVAL_MS  5  // Sample every 5ms when idle

// Called periodically during ESC_IDLE state (from 1ms system tick)
void ROTATION_Detect(ROTATION_DETECT_T *r)
{
    // Ensure all PWM outputs are disabled (high-Z)
    // Sample all 3 BEMF channels
    r->bemfSamples[0] = HAL_ADC_ReadBEMF(0);  // Phase A
    r->bemfSamples[1] = HAL_ADC_ReadBEMF(1);  // Phase B
    r->bemfSamples[2] = HAL_ADC_ReadBEMF(2);  // Phase C

    uint16_t vbusHalf = HAL_ADC_ReadVbus() >> 1;

    // Check if any phase has significant BEMF
    uint16_t maxBemf = 0;
    for (int i = 0; i < 3; i++) {
        int16_t centered = (int16_t)r->bemfSamples[i] - (int16_t)vbusHalf;
        uint16_t magnitude = (centered > 0) ? centered : -centered;
        if (magnitude > maxBemf) maxBemf = magnitude;
    }

    if (maxBemf > ROTATION_BEMF_THRESHOLD) {
        r->detected = 1;
        // Determine which phase is highest, lowest, floating
        // to identify current commutation step
        // Track sequence of steps to determine direction
        // Measure time between steps for speed estimation
        // (Full implementation: track 6-step sequence over ~12 samples)
    } else {
        r->detected = 0;
        r->estimatedERPM = 0;
    }
}

// Called when starting motor — returns recommended initial conditions
void ROTATION_GetStartConditions(ROTATION_DETECT_T *r,
                                  uint8_t *startStep,
                                  uint8_t *startDirection,
                                  uint32_t *startStepPeriod)
{
    if (r->detected && r->estimatedERPM > 500) {
        // Motor is spinning — sync to existing rotation
        *startStep = r->phaseSequence[r->sequenceIndex];
        *startDirection = r->direction;
        // Convert eRPM to step period in timer ticks
        *startStepPeriod = (60UL * COMMUTATION_TIMER_FREQ) /
                           (r->estimatedERPM * 6UL);
    } else {
        // Motor is stationary — normal startup
        *startStep = 0;
        *startDirection = 0;  // From config
        *startStepPeriod = INITIAL_STEP_PERIOD;
    }
}
```

---

### NEW Task 2.7: Updated Startup Sequence (Incorporating All References)

**Sources:** ASAC (±100µs transition), Sapog (voltage ramp, timeout, rotation detect)

Updated `ESC_ALIGN` and `ESC_OPEN_LOOP_RAMP` states:

```c
case ESC_IDLE:
    // NEW: Periodically check for external rotation (from Sapog)
    if (g->systemTickMs % ROTATION_SAMPLE_INTERVAL_MS == 0) {
        ROTATION_Detect(&g->rotation);
    }

    // Wait for arm signal
    if (g->throttle == 0 && g->armTimer >= ARM_TIME_COUNTS) {
        g->state = ESC_ARMED;
    }
    break;

case ESC_ARMED:
    // Keep monitoring rotation while armed
    if (g->systemTickMs % ROTATION_SAMPLE_INTERVAL_MS == 0) {
        ROTATION_Detect(&g->rotation);
    }

    if (g->throttle > THROTTLE_MIN) {
        // NEW: Get start conditions from rotation detector
        ROTATION_GetStartConditions(&g->rotation,
            &g->currentStep, &g->direction, &g->stepPeriod);

        if (g->rotation.detected && g->rotation.estimatedERPM > 1000) {
            // Motor already spinning — skip align, go direct to open-loop
            // with synced step and timing
            g->state = ESC_OPEN_LOOP_RAMP;
        } else {
            // Normal startup — align first
            g->alignTimer = 0;
            g->state = ESC_ALIGN;
        }
        g->startupTimer = 0;  // NEW: overall startup timeout
        g->startupAttempts++;  // NEW: consecutive attempt counter
    }
    break;

case ESC_ALIGN:
    HAL_PWM_SetCommutationStep(g->currentStep, g->alignDuty);
    g->alignTimer++;
    g->startupTimer++;

    if (g->alignTimer >= g->alignTimeCounts) {
        g->openLoopCommutations = 0;
        g->state = ESC_OPEN_LOOP_RAMP;
    }

    // NEW: Startup timeout (from Sapog — 5 seconds)
    if (g->startupTimer > STARTUP_TIMEOUT_COUNTS) {
        g->faultCode = FAULT_STARTUP_TIMEOUT;
        g->state = ESC_FAULT;
    }
    break;

case ESC_OPEN_LOOP_RAMP:
    g->stepTimer++;
    g->startupTimer++;

    if (g->stepTimer >= g->stepPeriod) {
        g->stepTimer = 0;
        g->currentStep = (g->currentStep + 1) % 6;
        HAL_PWM_SetCommutationStep(g->currentStep, g->rampDuty);
        g->openLoopCommutations++;

        // Accelerate
        if (g->stepPeriod > g->minStepPeriod)
            g->stepPeriod -= g->rampAcceleration;
    }

    // NEW: Monitor BEMF during open-loop (from ASAC)
    uint8_t floatPhase = commutation_table[g->currentStep].floatingPhase;
    uint8_t zcPol = commutation_table[g->currentStep].zcPolarity;
    uint16_t bemf = HAL_ADC_ReadBEMF(floatPhase);
    uint16_t vHalf = HAL_ADC_ReadVbus() >> 1;
    uint8_t pol = (bemf > vHalf) ? 1 : 0;

    // NEW: Transition criterion from ASAC (±100µs window)
    if (g->openLoopCommutations >= MIN_OL_COMMUTATIONS && pol == zcPol) {
        uint32_t now = HAL_Timer_GetTimestamp();
        uint32_t expectedZC = g->lastCommTimestamp +
                              (g->stepPeriod * TIMER_TICKS_PER_PWM / 2);
        int32_t dt_us = (int32_t)(now - expectedZC) *
                        1000000 / COMMUTATION_TIMER_FREQ;

        if (dt_us >= -100 && dt_us <= 100) {
            // ZC within ±100µs of expected — safe to switch
            g->timing.lastCommTimestamp = g->lastCommTimestamp;
            g->timing.stepPeriod = g->stepPeriod * 2;
            g->timing.speedValid = 1;
            g->startupAttempts = 0;  // Reset failure counter on success
            g->state = ESC_CLOSED_LOOP;
        }
    }

    // NEW: Startup timeout
    if (g->startupTimer > STARTUP_TIMEOUT_COUNTS) {
        g->faultCode = FAULT_STARTUP_TIMEOUT;
        g->state = ESC_FAULT;
    }
    break;
```

**NEW startup parameters:**
```c
#define MIN_OL_COMMUTATIONS       30      // From ASAC
#define ZC_TRANSITION_WINDOW_US   100     // From ASAC: ±100µs
#define STARTUP_TIMEOUT_MS        5000    // From Sapog: 5 second limit
#define STARTUP_TIMEOUT_COUNTS    (STARTUP_TIMEOUT_MS * 1000 / LOOPTIME_MICROSEC)
#define MAX_STARTUP_ATTEMPTS      5       // From Sapog: consecutive failure lockout
```

---

### NEW Task 4.3: Updated Protection Features

**Sources:** VESC (soft thermal derating), Sapog (P-control current limit,
failure lockout), BLHeli (demag power cutoff)

**`overcurrent.c` — Two-Tier Current Protection:**
```c
// Tier 1: Soft limit — proportional duty reduction (from Sapog/VESC)
// Runs every PWM cycle in ADC ISR
void OVERCURRENT_SoftLimit(GARUDA_DATA_T *g)
{
    float current = HAL_ADC_ReadCurrent_Amps();
    float softLimit = g->config.maxCurrentDeciA * 0.1f * 0.8f;  // 80% of max

    if (current > softLimit) {
        // Proportional reduction (from Sapog: mot_i_max_p = 0.2)
        float excess = current - softLimit;
        float reduction = excess * CURRENT_LIMIT_P_GAIN;  // P gain = 0.2
        g->dutyReduction = reduction;
        if (g->dutyReduction > g->duty)
            g->dutyReduction = g->duty;  // Clamp to prevent negative
    } else {
        // Ramp back up slowly
        if (g->dutyReduction > 0)
            g->dutyReduction -= CURRENT_RECOVERY_RATE;
    }
}

// Tier 2: Hard limit — hardware comparator → PWM PCI (instant)
// Already configured via PGxFPCI (from reference FOC code)
// Response time: <200ns (hardware, not software)
```

**`thermal.c` — Linear Derating Curve (from VESC):**
```c
void THERMAL_Update(GARUDA_DATA_T *g)
{
    uint16_t adcTemp = HAL_ADC_ReadTemp();
    g->edt.temperatureC = NTC_ADCtoTemp(adcTemp);

    uint8_t startTemp = g->config.tempLimitC - 20;  // Start derating 20°C below limit
    uint8_t endTemp = g->config.tempLimitC;

    if (g->edt.temperatureC >= endTemp) {
        // Complete shutdown
        g->faultCode = FAULT_OVERTEMP;
        g->state = ESC_FAULT;
    }
    else if (g->edt.temperatureC > startTemp) {
        // Linear derating (from VESC: smooth, not bang-bang)
        float factor = (float)(endTemp - g->edt.temperatureC) /
                       (float)(endTemp - startTemp);
        g->thermalDutyFactor = factor;  // 0.0 to 1.0
    }
    else {
        g->thermalDutyFactor = 1.0f;  // No derating
    }
}
```

**Startup Failure Lockout (from Sapog):**
```c
// In ESC_FAULT state handler:
case ESC_FAULT:
    HAL_PWM_DisableAllOutputs();

    if (g->startupAttempts >= MAX_STARTUP_ATTEMPTS) {
        // LOCKED — requires explicit reset
        g->locked = 1;
        // Only clear via: power cycle, or DShot CMD_DISARM held 2 seconds
    }

    // Normal fault clear: throttle = 0 for 500ms
    if (!g->locked && g->throttle == 0) {
        g->faultClearTimer++;
        if (g->faultClearTimer >= FAULT_CLEAR_TIME_COUNTS) {
            g->faultCode = FAULT_NONE;
            g->state = ESC_IDLE;
        }
    }
    break;
```

---

### NEW Phase 4 Addition: Task 4.6 — Garuda Configurator Tool

**Sources:** VESC Tool (Qt desktop), BLHeliSuite (serial passthrough),
Sapog (DroneCAN GUI Tool)

**Technology:** Web-based using WebSerial API (Chrome/Edge)

**Rationale:** No install required, cross-platform, modern web tech.
WebSerial API provides direct USB/UART access from browser.
Fallback: Electron wrapper for offline use.

**Architecture:**
```
┌─────────────────────────────────────────────┐
│           Web Browser (Chrome/Edge)          │
├─────────────────────────────────────────────┤
│  Garuda Configurator (HTML + JS)            │
│                                              │
│  ┌──────────┐ ┌──────────┐ ┌─────────────┐ │
│  │ Config   │ │ Telemetry│ │ Firmware    │ │
│  │ Editor   │ │ Dashboard│ │ Updater     │ │
│  └────┬─────┘ └────┬─────┘ └──────┬──────┘ │
│       │             │              │         │
│  ┌────▼─────────────▼──────────────▼──────┐ │
│  │  GSP Protocol Layer (JS)               │ │
│  │  - Packet encode/decode                │ │
│  │  - CRC-16 CCITT                        │ │
│  │  - Command queue + timeout             │ │
│  └────────────────┬───────────────────────┘ │
│                   │                          │
│  ┌────────────────▼───────────────────────┐ │
│  │  WebSerial API                         │ │
│  │  - Port selection dialog               │ │
│  │  - Baud rate: 115200                   │ │
│  │  - 8N1                                 │ │
│  └────────────────┬───────────────────────┘ │
└───────────────────┼─────────────────────────┘
                    │ USB / UART
┌───────────────────▼─────────────────────────┐
│  Garuda ESC (dsPIC33AK128MC106)             │
│  GSP handler → config/telemetry/bootloader  │
└─────────────────────────────────────────────┘
```

**UI Screens:**

**1. Dashboard (Main Screen)**
- Connection status + ESC info (firmware version, hardware ID)
- Live telemetry: eRPM gauge, voltage bar, current bar, temperature
- State indicator: IDLE / ARMED / RUNNING / FAULT
- Fault history log

**2. Configuration Editor**
- All EEPROM parameters in grouped sections:
  - Motor: poles, direction, timing advance, PWM frequency
  - Startup: align time, ramp rate, open-loop duty
  - Protection: current limit, temp limit, voltage limits, signal timeout
  - DShot: rate selection, EDT enable, bidir enable
  - Advanced: dead time, blanking %, demag compensation level
- Read / Write / Save / Factory Reset buttons
- Export/Import as JSON file

**3. Motor Setup Wizard**
- Step 1: Connect and identify ESC
- Step 2: Set motor pole count (or auto-detect)
- Step 3: Set rotation direction (spin test)
- Step 4: Verify BEMF waveform (debug ADC plot)
- Step 5: Save and arm

**4. Debug / BEMF Monitor**
- Real-time ADC waveform plot (BEMF, Vbus, comparator output)
- Zero-crossing event markers
- Commutation timing diagram
- Demag metric display
- Uses GSP `GET_DEBUG` command at ~100 Hz

**5. Firmware Updater**
- Drag-and-drop .bin file
- Progress bar
- Automatic reboot into bootloader
- Verify after flash
- Rollback option

**Implementation Timeline:**
- Phase 4: Basic config editor + telemetry dashboard
- Phase 5: Motor wizard + BEMF monitor + firmware updater

---

### Updated Directory Structure (v2.1)

```
garudaESC.X/
├── ... (unchanged from v2)
├── motor/
│   ├── ... (unchanged from v2)
│   ├── demag.c/.h             # NEW: Demagnetization compensation
│   └── rotation_detect.c/.h   # NEW: External rotation detection
├── protection/
│   ├── overcurrent.c/.h       # UPDATED: Two-tier (soft P-control + hard HW)
│   ├── thermal.c/.h           # UPDATED: Linear derating curve (VESC-style)
│   └── ... (unchanged)
└── ...

garuda-configurator/            # NEW: Web-based configuration tool
├── index.html
├── css/
│   └── style.css
├── js/
│   ├── main.js                # App entry point
│   ├── gsp.js                 # GSP protocol encode/decode
│   ├── serial.js              # WebSerial API wrapper
│   ├── config.js              # Configuration editor logic
│   ├── telemetry.js           # Real-time dashboard
│   ├── charts.js              # Chart.js waveform plots
│   └── firmware.js            # Bootloader/update logic
└── assets/
    └── garuda-logo.svg
```

---

### Updated Interrupt Priority Map (v2.1)

| Priority | ISR | Source | Budget | Notes |
|----------|-----|--------|--------|-------|
| 7 | `_PWM1Interrupt` | PWM Fault PCI (HW overcurrent) | <200ns | From reference, unchanged |
| 6 | `_AD1CH0Interrupt` | ADC complete (PWM center) | <3us | **Increased** — now includes demag check |
| 5 | `_CCT1Interrupt` | Commutation timer match | <1us | Unchanged |
| 4 | `_IC1Interrupt` / DMA | DShot Input Capture | DMA | Unchanged |
| 3 | — | DShot frame complete | <5us | Unchanged |
| 2 | — | Reserved | — | — |
| 1 | `_T1Interrupt` | Timer1 (1ms tick) | relaxed | **Added:** rotation detect, thermal update, signal loss check |
| 0 | Background | Main loop | unbounded | **Added:** EEPROM write, flash CRC check |

---

### Updated Data Flow (v2.1)

```
                   ┌──── External Rotation Detect (idle only) ◄── BEMF ADC
                   │
DShot Input ──→ IC1+DMA ──→ DSHOT_DecodeFrame() ──→ throttle
                                                         │
                    ┌────────────────────────────────────┘
                    ▼
              GARUDA_StateMachine()
                    │
      ┌─────────────┼──────────────────────┐
      ▼             ▼                      ▼
   Startup    Closed-Loop              Protection
   (align →   (ZC detect →            (overcurrent soft ←── ADC current
    OL ramp)   timing engine)          thermal derating ←── ADC NTC
      │             │                   signal loss ←── DShot timeout
      │             ├── BEMF sample     demag comp ←── BEMF polarity)
      │             ├── Demag check          │
      │             ├── ZC majority filter   │
      │             ├── Timing advance       │
      │             └── Desync detect        │
      │                     │                │
      └──────────┬──────────┘                │
                 ▼                           │
           duty = throttle_to_duty()         │
                 │                           │
                 ├── × thermalDutyFactor  ◄──┘
                 ├── × demagDutyFactor
                 ├── - dutyReduction (overcurrent P)
                 ▼
        HAL_PWM_SetCommutationStep(step, duty) ──→ FET gates

Bidirectional DShot: RX complete ──→ 30µs ──→ GCR eRPM/EDT TX
GSP Protocol: UART RX ──→ command dispatch ──→ UART TX response
Configurator: WebSerial ←──→ GSP ←──→ config/telemetry/bootloader
```
