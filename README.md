# Project Garuda

**Open-source drone ESC firmware for the Microchip dsPIC33AK128MC106**

6-step trapezoidal BLDC motor control with sensorless BEMF zero-crossing detection — built from scratch for high-performance multirotor applications.

---

## Current Status: Phase 1 Complete

Phase 1 firmware compiles and links with **zero errors, zero warnings**.

- 14 source files, 17 headers
- Program flash: **9,024 bytes (6%)** of 128 KB
- Data RAM: **88 bytes (<1%)** of 16 KB
- Open-loop motor control: IDLE -> ARMED -> ALIGN -> OL_RAMP -> steady-state hold
- Hardware fault protection via external PCI8 overcurrent pin

Phase 2 (BEMF closed-loop commutation) is next.

---

## Target Hardware

| Parameter | Value |
|-----------|-------|
| **MCU** | dsPIC33AK128MC106 |
| **Core** | 200 MHz DSP |
| **Flash / RAM** | 128 KB / 16 KB |
| **ADC** | Dual-core, 12-bit |
| **Comparators** | 3 with built-in DAC (for BEMF ZC) |
| **PWM** | 3 generators, 400 MHz timebase, complementary with dead-time |
| **Dev Board** | MCLV-48V-300W (Microchip) |
| **Compiler** | XC-DSC v3.30 |
| **IDE** | MPLAB X v6.30 |
| **DFP** | dsPIC33AK-MC_DFP v1.4.172 |

---

## Project Structure

```
dspic33AKESC/
├── main.c                        Entry point, init sequence, button polling
├── garuda_types.h                All data structures and enums
├── garuda_config.h               User-tunable parameters (knobs)
├── garuda_calc_params.h          Derived constants computed from config
├── garuda_service.c/.h           State machine + ADC/Timer1/PWM fault ISRs
├── util.h                        Math utilities (from AN1292 reference)
│
├── hal/
│   ├── clock.c/.h                PLL init: 200MHz sys, 400MHz PWM, 100MHz ADC
│   ├── device_config.c           Fuse pragmas (WDT, JTAG, protection bits)
│   ├── port_config.c/.h          GPIO pin mapping: PWM, BEMF, LED, buttons, UART, DShot
│   ├── hal_pwm.c/.h              24kHz center-aligned PWM, 6-step override control
│   ├── hal_adc.c/.h              BEMF/Vbus/Pot ADC channels, floating phase muxing
│   ├── hal_comparator.c/.h       CMP1/2/3 with DAC for BEMF ZC (Phase 2 integration)
│   ├── board_service.c/.h        Peripheral init, button debounce, PWM enable/disable
│   ├── timer1.c/.h               100us tick timer (prescaler 1:8, 100MHz clock)
│   ├── uart1.c/.h                UART1 8N1 (for future GSP/debug)
│   └── delay.h                   Blocking delay using libpic30
│
└── motor/
    ├── commutation.c/.h          6-step commutation table + step advance
    ├── startup.c/.h              Alignment hold + open-loop forced ramp
    └── pi.c/.h                   PI controller (from AN1292 reference)
```

### File Categories

| Category | Files | Origin |
|----------|-------|--------|
| **Verbatim from AN1292** | clock.c/.h, device_config.c, timer1.c/.h, uart1.c/.h, delay.h, util.h, pi.c/.h | Direct copy |
| **Adapted from AN1292** | port_config.c/.h, hal_pwm.c/.h, hal_adc.c/.h, hal_comparator.c/.h, board_service.c/.h | Modified |
| **New for Garuda** | garuda_types.h, garuda_config.h, garuda_calc_params.h, garuda_service.c/.h, commutation.c/.h, startup.c/.h, main.c | Written from scratch |

---

## Pin Allocation

```
dsPIC33AK128MC106 - Project Garuda Pin Map

 MOTOR DRIVE (6 pins)
  RD2 -- PWM1H -- Phase A High     RD3 -- PWM1L -- Phase A Low
  RD0 -- PWM2H -- Phase B High     RD1 -- PWM2L -- Phase B Low
  RC3 -- PWM3H -- Phase C High     RC4 -- PWM3L -- Phase C Low

 BEMF SENSING (3 pins)
  RA4 -- AD1AN1 / CMP1B -- Phase A    (ADC1 CH0, PINSEL=1)
  RB2 -- AD2AN4 / CMP2B -- Phase B    (ADC2 CH0, PINSEL=4)
  RB5 -- AD2AN2 / CMP3B -- Phase C    (ADC2 CH0, PINSEL=2, muxed with B)

 ANALOG INPUTS
  RA7  -- AD1AN6  -- DC bus voltage (Vbus)
  RA11 -- AD1AN10 -- Potentiometer (speed reference in Phase 1)

 PROTOCOL I/O
  RD8  -- RP57/ICM1 -- DShot input (Input Capture, Phase 3)
  RC10 -- RP43/U1TX -- UART TX
  RC11 -- RP44/U1RX -- UART RX

 FAULT / UI
  RB11 -- RP28/PCI8 -- External overcurrent fault input
  RD5  -- LED1      -- Heartbeat (2Hz blink)
  RC9  -- LED2      -- Motor running indicator
  RD9  -- SW1       -- Start/Stop motor
  RD10 -- SW2       -- Change direction (when idle)
```

### Clock Domains

```
8 MHz FRC -> PLL1 -> 800 MHz VCO

CLK1 = 200 MHz  |  System clock (FCY)
CLK5 = 400 MHz  |  PWM timebase
CLK6 = 100 MHz  |  ADC clock
CLK7 = 400 MHz  |  DAC / Comparator clock
CLK8 = 100 MHz  |  UART clock
```

---

## State Machine

```
           SW1 press
  [IDLE] -----------> [ARMED]
    ^                    |
    |                    | throttle=0 for 500ms
    |                    v
    |               [ALIGN]       hold step 0 at 5% duty for 200ms
    |                    |
    |                    v
    |              [OL_RAMP]      forced commutation, decreasing period
    |                    |
    |                    | rampStepPeriod <= MIN_STEP_PERIOD
    |                    v
    |            [CLOSED_LOOP]    Phase 1: holds at final ramp speed
    |                    |        Phase 2: BEMF ZC-driven commutation
    | SW1 press          |
    +--------------------+
    |
    |  overcurrent
    +--- [FAULT] ----> SW1 clears fault -> [IDLE]
```

### State Descriptions

| State | Entry Condition | Behavior |
|-------|----------------|----------|
| `ESC_IDLE` | Power-on / SW1 stop | All PWM outputs LOW, heartbeat LED blinks |
| `ESC_ARMED` | SW1 press from IDLE | Verifies throttle near zero for 500ms (`ARM_TIME_MS`) |
| `ESC_ALIGN` | Arm timer complete | Holds commutation step 0 at `ALIGN_DUTY` for 200ms |
| `ESC_OL_RAMP` | Alignment complete | Forced commutation with decreasing step period, increasing duty |
| `ESC_CLOSED_LOOP` | Ramp target reached | Phase 1: continues forced commutation at final speed |
| `ESC_FAULT` | PCI8 overcurrent | All outputs disabled, LED2 off, SW1 to clear |

---

## Important APIs

### PWM Control (`hal/hal_pwm.h`)

```c
void InitPWMGenerators(void);
// Initialize all 3 PWM generators: 24kHz center-aligned, complementary mode,
// dead time, bootstrap charging. PG1=master, PG2/PG3=slaves.

void HAL_PWM_SetCommutationStep(uint8_t step);
// Apply 6-step commutation pattern via PGxIOCON override registers.
// step 0-5. For each phase: PWM_ACTIVE releases override,
// LOW forces H=off/L=on (sink), FLOAT forces both off (high-Z).

void HAL_PWM_SetDutyCycle(uint32_t duty);
// Set duty cycle on all 3 generators. Clamped to [MIN_DUTY, MAX_DUTY].
// Only the active (non-overridden) phase produces output.
```

### ADC (`hal/hal_adc.h`)

```c
void InitializeADCs(void);
// Configure ADC1 (BEMF_A on CH0, Vbus on CH4, Pot on CH1)
// and ADC2 (BEMF_B/C on CH0, muxed via PINSEL).

void HAL_ADC_SelectBEMFChannel(uint8_t floatingPhase);
// Mux AD2CH0 PINSEL for the floating phase:
//   0=Phase A (AD1CH0, no mux needed)
//   1=Phase B (AD2CH0, PINSEL=4)
//   2=Phase C (AD2CH0, PINSEL=2)
```

Key macros:
- `ADCBUF_BEMF_A` / `ADCBUF_BEMF_BC` - ADC result registers
- `ADCBUF_VBUS` / `ADCBUF_POT` - Bus voltage and potentiometer
- `GARUDA_ADC_INTERRUPT` = `_AD1CH0Interrupt` (fires at PWM rate, 24kHz)

### Comparator (`hal/hal_comparator.h`)

```c
void InitializeCMPs(void);
// Initialize CMP1/2/3 for BEMF zero-crossing with DAC calibration from flash.
// CMP1B=RA4, CMP2B=RB2, CMP3B=RB5, all using INSEL=1 (B input).

void HAL_CMP_EnableFloatingPhase(uint8_t phase);
// Enable only the comparator for the current floating phase (0=A, 1=B, 2=C).
// Disables the other two to avoid false triggers. Turns on DACCTRL1.

void HAL_CMP_SetReference(uint16_t vbusHalf);
// Update DAC1/2/3 reference to Vbus/2 for zero-crossing threshold.
```

Note: Comparators are initialized but not wired into the commutation loop yet. Phase 2 will integrate ZC detection.

### Commutation (`motor/commutation.h`)

```c
extern const COMMUTATION_STEP_T commutationTable[6];
// 6-step BLDC commutation sequence:
//   Step 0: A=PWM,   B=LOW,   C=FLOAT  (sense C rising)
//   Step 1: A=FLOAT, B=LOW,   C=PWM    (sense A falling)
//   Step 2: A=LOW,   B=FLOAT, C=PWM    (sense B rising)
//   Step 3: A=LOW,   B=PWM,   C=FLOAT  (sense C falling)
//   Step 4: A=FLOAT, B=PWM,   C=LOW    (sense A rising)
//   Step 5: A=PWM,   B=FLOAT, C=LOW    (sense B falling)

void COMMUTATION_AdvanceStep(volatile GARUDA_DATA_T *pData);
// Increment step 0->1->...->5->0, apply PWM overrides,
// select ADC channel for the new floating phase.
```

### Startup (`motor/startup.h`)

```c
void STARTUP_Init(volatile GARUDA_DATA_T *pData);
// Initialize: step=0, alignCounter=ALIGN_TIME_COUNTS,
// rampStepPeriod=INITIAL_STEP_PERIOD, duty=ALIGN_DUTY.

bool STARTUP_Align(volatile GARUDA_DATA_T *pData);
// Hold step 0 at ALIGN_DUTY. Called every 100us from Timer1 ISR.
// Returns true when alignment time (200ms) has elapsed.

bool STARTUP_OpenLoopRamp(volatile GARUDA_DATA_T *pData);
// Forced commutation with decreasing step period.
// Returns true when rampStepPeriod <= MIN_STEP_PERIOD.
```

### Board Service (`hal/board_service.h`)

```c
void HAL_InitPeripherals(void);
// Master init: ADC -> CMP -> PWM -> Timer1. Called once from main().

void HAL_MC1PWMEnableOutputs(void);
void HAL_MC1PWMDisableOutputs(void);
// Enable/disable all 6 PWM outputs via override registers.

void HAL_MC1ClearPWMPCIFault(void);
// Clear latched PCI fault via software termination.

bool IsPressed_Button1(void);  // SW1: Start/Stop
bool IsPressed_Button2(void);  // SW2: Direction change
```

### ISR Map

| ISR | Source | Rate | Purpose |
|-----|--------|------|---------|
| `_AD1CH0Interrupt` | ADC1 CH0 complete | 24 kHz (PWM rate) | Read BEMF/Vbus/Pot, update duty |
| `_T1Interrupt` | Timer1 | 10 kHz (100us) | Heartbeat, button service, state machine (arm/align/ramp/hold) |
| `_PWM1Interrupt` | PWM fault PCI | On fault | Disable outputs, set ESC_FAULT |

---

## Configuration Parameters (`garuda_config.h`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `PWMFREQUENCY_HZ` | 24000 | PWM switching frequency |
| `DEADTIME_NS` | 750 | Dead time in nanoseconds |
| `MOTOR_POLE_PAIRS` | 7 | Motor pole pairs |
| `DIRECTION_DEFAULT` | 0 | 0=CW, 1=CCW |
| `ALIGN_TIME_MS` | 200 | Alignment hold time |
| `ALIGN_DUTY_PERCENT` | 5 | Duty during alignment |
| `INITIAL_ERPM` | 2000 | Starting eRPM for ramp |
| `RAMP_TARGET_ERPM` | 15000 | Target eRPM (end of ramp) |
| `ARM_TIME_MS` | 500 | Throttle-zero time to arm |

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

---

## Phase 1 Test Procedure

### Test 1: Build Verification

**Goal:** Confirm firmware compiles with no errors or warnings.

```
Expected output:
  Total "program" memory used (bytes): ~9024 (6%)
  Total "data" memory used (bytes): 88 (<1%)
  Three ISR handlers registered: __AD1CH0Interrupt, __T1Interrupt, __PWM1Interrupt
```

### Test 2: Boot and Heartbeat (MPLAB SIM or Hardware)

**Goal:** Verify clock init and Timer1 ISR.

1. Load firmware in MPLAB SIM (or program to MCLV-48V-300W board)
2. Run
3. **Watch:** `heartbeatCounter` in `garuda_service.c` increments every 100us
4. **Watch:** `LED1` (RD5/LATD5) toggles every 250ms (2500 Timer1 ticks)
5. **Watch:** `garudaData.systemTick` increments every 1ms (10 Timer1 ticks)
6. **Watch:** `garudaData.state` == `ESC_IDLE` (value 0)

**Pass criteria:** LED1 toggles at 2Hz, systemTick counts up, state stays IDLE.

### Test 3: Button Press and Arming

**Goal:** Verify button debounce and ARMED state transition.

1. Set breakpoint at `garuda_service.c` Timer1 ISR, `ESC_ARMED` case
2. Simulate SW1 press: set `PORTDbits.RD9 = 0` (active low) in stimulus
3. **Watch:** `garudaData.state` transitions IDLE -> ARMED
4. **Watch:** `garudaData.armCounter` increments every 100us
5. Keep `garudaData.throttle` < 50 (pot stimulus near zero)
6. After 5000 counts (500ms): state transitions ARMED -> ALIGN
7. **Watch:** `LED2` (RC9/LATC9) goes HIGH

**Pass criteria:** IDLE -> ARMED -> ALIGN transition in 500ms with throttle at zero. If throttle > 50 during arming, armCounter resets.

### Test 4: Alignment Phase

**Goal:** Verify motor locks to step 0 at low duty.

1. Set breakpoint at `STARTUP_Align()` in `startup.c`
2. **Watch:** `PG1IOCONbits.OVRENH` == 0, `PG1IOCONbits.OVRENL` == 0 (Phase A = PWM active)
3. **Watch:** `PG2IOCONbits.OVRDAT` == 0b01 (Phase B = LOW sink)
4. **Watch:** `PG3IOCONbits.OVRDAT` == 0b00 (Phase C = FLOAT)
5. **Watch:** `PG1DC` == `ALIGN_DUTY` value
6. **Watch:** `garudaData.alignCounter` counts down from `ALIGN_TIME_COUNTS`
7. After 2000 counts (200ms): state transitions ALIGN -> OL_RAMP

**Pass criteria:** Step 0 pattern held for 200ms, correct override register values.

### Test 5: Open-Loop Ramp

**Goal:** Verify forced commutation with accelerating speed.

1. Set breakpoint at `COMMUTATION_AdvanceStep()` in `commutation.c`
2. **Watch:** `garudaData.currentStep` cycles 0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 0
3. **Watch:** `garudaData.rampStepPeriod` decreases by 1 each step
4. **Watch:** `garudaData.duty` increases gradually
5. **Watch:** PGxIOCON override registers change per commutation table at each step
6. When `rampStepPeriod` <= `MIN_STEP_PERIOD + 1`: state transitions OL_RAMP -> CLOSED_LOOP

**Pass criteria:** All 6 commutation steps cycle correctly, speed increases, transition occurs.

On hardware with scope:
- Probe PWM1H/PWM1L (RD2/RD3): 24kHz complementary with 750ns dead time
- Probe all 6 PWM pins: correct 6-step pattern, one phase PWM, one LOW, one floating
- Frequency of commutation cycle increases during ramp

### Test 6: Steady-State Hold (CLOSED_LOOP in Phase 1)

**Goal:** Verify motor continues running at final ramp speed.

1. After OL_RAMP -> CLOSED_LOOP transition
2. **Watch:** `garudaData.state` == `ESC_CLOSED_LOOP` (value 4)
3. **Watch:** Commutation continues at fixed `rampStepPeriod` (MIN_STEP_PERIOD)
4. **Watch:** `garudaData.duty` stays constant
5. Press SW1 (simulate RD9 low): state -> IDLE, all PWM outputs LOW

**Pass criteria:** Motor holds speed, SW1 stops it cleanly.

### Test 7: Fault Response

**Goal:** Verify overcurrent fault path.

1. Simulate PCI8 fault: drive `PORTBbits.RB11` (or trigger in stimulus)
2. **Watch:** `_PWM1Interrupt` fires
3. **Watch:** All PGxIOCON OVRENH/OVRENL = 1, OVRDAT = 0 (all outputs LOW)
4. **Watch:** `garudaData.state` == `ESC_FAULT`, `garudaData.faultCode` == `FAULT_OVERCURRENT`
5. **Watch:** `LED2` goes LOW
6. Press SW1: fault clears, state -> IDLE

**Pass criteria:** Immediate shutdown on fault, clean recovery on SW1.

### Test 8: Direction Change

**Goal:** Verify direction toggle only works in IDLE.

1. In IDLE state, press SW2 (simulate RD10 low)
2. **Watch:** `garudaData.direction` toggles 0 -> 1
3. Start motor (SW1), then press SW2 during ramp
4. **Watch:** `garudaData.direction` does NOT change (only changes in IDLE)

**Pass criteria:** Direction only changes when motor is stopped.

### Test 9: Static Assert Verification

**Goal:** Confirm config struct size guard.

1. Temporarily change `reserved[48]` to `reserved[49]` in `garuda_types.h`
2. Build -> should fail with: `static assertion failed: GARUDA_CONFIG_T must be 64 bytes`
3. Revert the change

**Pass criteria:** Compile-time failure on struct size mismatch.

---

## Implementation Roadmap

### Phase 1: Foundation (COMPLETE)
- Clock init (200 MHz PLL), GPIO config, peripheral init
- 24 kHz center-aligned PWM with dead-time and 6-step override control
- ADC reads BEMF phase voltages, Vbus, potentiometer
- Open-loop forced commutation: align -> ramp -> steady-state hold
- External overcurrent fault protection via PCI8
- Button control: start/stop, direction change
- Heartbeat LED, motor running indicator

### Phase 2: Closed-Loop BEMF Control (NEXT)
- Wire comparators into commutation loop (APIs already exist)
- ADC-based zero-crossing detection with majority filter
- Commutation timing engine with 30-degree delay
- Open-loop -> closed-loop transition
- Demagnetization compensation
- Desync detection and recovery

### Phase 3: DShot Protocol
- DShot150/300/600/1200 decoding via Input Capture (ICM1 pin already mapped)
- Auto-rate detection
- Full command set (CMD 0-47)
- Bidirectional DShot with GCR encoding

### Phase 4: Protection, Config & Communication
- Thermal monitoring with linear derating
- Under/overvoltage protection
- Signal loss failsafe
- 64-byte EEPROM config with CRC-16
- Garuda Serial Protocol (GSP) over UART

### Phase 5: Advanced Features
- Adaptive timing advance
- External rotation detection
- Motor auto-detection
- Web-based configurator (WebSerial API)

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
