# Project Garuda — Claude Code Implementation Plan
## dsPIC33AK Drone ESC Firmware Build Strategy

**Reference Template:** `mclv48v300w-33ak128mc106-pmsm-an1292-foc-pll` (branch 2.0.0)
**Target Device:** dsPIC33AK128MC106 (200 MHz, HW FPU, 40 MSPS ADC, HRPWM)
**Specification:** Project Garuda v1.3 — Clean-Room ESC Firmware

---

## 1. Architecture Overview & Reference Repo Mapping

The AN1292 FOC-PLL reference project (`pmsm.X`) provides a complete MPLAB X project for the **same dsPIC33AK128MC106** device on the MCLV-48V-300W board. Its peripheral initialization and MCC-generated code serve as our template for all hardware abstraction. Project Garuda repurposes the same peripheral set but replaces the FOC control algorithm with 6-step trapezoidal/BEMF sensorless commutation optimized for drone ESCs.

### What We Reuse from the Reference Repo

| Reference Component | Garuda Usage | Notes |
|---|---|---|
| `mcc_generated_files/` — PWM (PG1, PG2, PG3) | Reuse PWM init, adapt for 6-step override control | Same 3 half-bridge PWM generators; change from center-aligned SVM to 6-step override mode |
| `mcc_generated_files/` — ADC (multi-core) | Reuse ADC init for BEMF sampling, Vbus, temperature | Reference uses ADC for current sensing; we add BEMF floating-phase sampling |
| `mcc_generated_files/` — UART | Reuse for GSP protocol & X2C-Scope debug | Same UART1 peripheral config |
| `mcc_generated_files/` — OP-AMP, CMP, DAC | Reuse comparator for hardware ZC detection | Reference uses op-amp for current amplification; we repurpose for BEMF conditioning |
| `mc1_user_params.h` | Replace with ESC-specific parameters | Motor poles, timing advance, DShot config instead of FOC PI gains |
| Clock configuration (200 MHz) | Direct reuse | Same oscillator + PLL config |
| GPIO / Port mapping | Adapt for ESC board pinout | Same device, different board layout |

### What We Build New (Clean-Room)

| New Module | Specification Section | Priority |
|---|---|---|
| 6-Step Commutation Engine | §3.1, §3.2 | Phase 1 |
| BEMF Zero-Crossing Detection | §3.2.3 | Phase 2 |
| DShot Protocol Decoder | §2.1 | Phase 3 |
| Bidirectional DShot + GCR Telemetry | §2.2 | Phase 3 |
| EDT Telemetry Scheduler | §2.3 | Phase 3 |
| State Machine (8-state) | §3.1.1-3.1.2 | Phase 1-2 |
| EEPROM/DEE Configuration System | §4.1 | Phase 4 |
| Garuda Serial Protocol (GSP) | §7.2 | Phase 4 |
| Bootloader (4KB) | §5 | Phase 4 |
| CLC-Accelerated Commutation | §3.3.6 | Phase 2 |
| Advanced Algorithms (Flux, Ternary, etc.) | §3.3 | Phase 5 |

---

## 2. Project Directory Structure for Claude Code

```
project-garuda/
├── CLAUDE.md                          # Claude Code project instructions
├── garuda.X/                          # MPLAB X project
│   ├── mcc_generated_files/           # Copied & adapted from reference
│   │   ├── system/                    # Clock, oscillator, pin config
│   │   ├── pwm/                       # PG1, PG2, PG3 init
│   │   ├── adc/                       # ADC core init (BEMF + Vbus + Temp)
│   │   ├── uart/                      # UART1 for GSP/debug
│   │   ├── cmp/                       # Comparator for ZC detection
│   │   ├── dac/                       # DAC for ZC threshold (Vbus/2)
│   │   ├── clc/                       # CLC for HW commutation (Phase 2)
│   │   ├── tmr/                       # Timers: commutation, system tick
│   │   └── dee/                       # Data EEPROM Emulation
│   ├── garuda/                        # Application firmware (clean-room)
│   │   ├── garuda_main.c             # Main entry, state machine loop
│   │   ├── garuda_main.h
│   │   ├── garuda_config.h           # Build-time configuration macros
│   │   ├── motor/                     # Motor control core
│   │   │   ├── commutation.c/.h      # 6-step commutation table & execution
│   │   │   ├── bemf.c/.h             # BEMF ZC detection (ADC + comparator)
│   │   │   ├── startup.c/.h          # ALIGN → OPEN_LOOP_RAMP sequence
│   │   │   ├── timing.c/.h           # Commutation timing, advance angle
│   │   │   ├── desync.c/.h           # Desync detection & recovery
│   │   │   └── pi_control.c/.h       # Throttle-to-duty PI controller
│   │   ├── protocol/                  # Communication protocols
│   │   │   ├── dshot.c/.h            # DShot150/300/600/1200 decoder
│   │   │   ├── dshot_bidir.c/.h      # Bidirectional DShot + GCR encoder
│   │   │   ├── edt.c/.h             # Extended DShot Telemetry scheduler
│   │   │   ├── dshot_commands.c/.h   # DShot command handler (§2.1.3)
│   │   │   └── gsp.c/.h             # Garuda Serial Protocol (§7.2)
│   │   ├── config/                    # Configuration management
│   │   │   ├── eeprom.c/.h          # EEPROM layout (§4.1) + DEE interface
│   │   │   ├── defaults.h           # Factory default values
│   │   │   └── calibration.c/.h     # Dead-time auto-calibration (§7.6)
│   │   ├── protection/               # Safety & protection
│   │   │   ├── overcurrent.c/.h     # HW comparator-based fault
│   │   │   ├── thermal.c/.h         # Temperature monitoring & derating
│   │   │   ├── voltage.c/.h         # Under/over voltage detection
│   │   │   └── signal_loss.c/.h     # Input signal timeout failsafe
│   │   ├── telemetry/                # Telemetry system
│   │   │   ├── telemetry.c/.h       # Centralized telemetry data store
│   │   │   └── beep.c/.h            # Motor-as-speaker beep generation
│   │   └── hal/                       # Hardware Abstraction Layer
│   │       ├── hal_pwm.c/.h         # PWM override control for 6-step
│   │       ├── hal_adc.c/.h         # ADC channel routing for BEMF
│   │       ├── hal_clc.c/.h         # CLC truth table for commutation
│   │       ├── hal_timer.c/.h       # Commutation timer, system tick
│   │       ├── hal_comparator.c/.h  # ZC comparator control
│   │       └── hal_gpio.c/.h        # LED, button, misc I/O
│   ├── bootloader/                    # Separate bootloader project
│   │   ├── bl_main.c
│   │   ├── bl_protocol.c/.h         # BL_PING, BL_WRITE, etc. (§7.4)
│   │   └── bl_flash.c/.h            # Flash erase/write routines
│   └── tests/                         # Unit test stubs (PC-hosted)
│       ├── test_dshot_crc.c          # CRC test vectors from §2.2.3
│       ├── test_edt_framing.c
│       └── test_state_machine.c
├── docs/
│   └── Project_Garuda_Spec_v1.3.docx
└── tools/
    └── garuda_configurator/           # PC tool (Electron/WebSerial) — Phase 5
```

---

## 3. Claude Code CLAUDE.md Instructions

The following `CLAUDE.md` file should be placed in the project root. This is the master instruction file Claude Code reads for every task.

```markdown
# Project Garuda - Claude Code Instructions

## Project Overview
Drone ESC firmware for dsPIC33AK128MC106 (200MHz, HW FPU, 40MSPS ADC).
6-step trapezoidal BLDC control with DShot protocol, bidirectional telemetry, EDT.

## Reference Code
The `reference/` directory contains the Microchip AN1292 FOC-PLL project for the
SAME dsPIC33AK128MC106 device. Use its MCC-generated peripheral init code as
the canonical template for:
- PWM generator setup (PG1/PG2/PG3) — registers, clock dividers, dead time
- ADC configuration — core assignment, trigger sources, channel mapping
- UART1 — baud rate, pin mapping
- Comparator (CMP) and DAC — for BEMF zero-crossing detection
- CLC — configurable logic cell setup
- System clock — 200MHz PLL configuration
- Pin Manager — I/O port assignments

## CRITICAL: Clean-Room Rules
- NEVER reference, copy, or derive from BLHeli, AM32, Bluejay, or KISS source code
- ALL motor control algorithms must be original, based ONLY on:
  - Microchip datasheets (dsPIC33AK FRM) and application notes (AN1160, AN1292)
  - Public protocol specifications (DShot from Betaflight docs, EDT from public spec)
  - Motor control textbook theory (6-step, BEMF ZC detection)
  - The Garuda specification document (docs/Project_Garuda_Spec_v1.3.docx)

## Build System
- MPLAB X IDE v6.25 with XC-DSC compiler v3.20+
- Device: dsPIC33AK128MC106
- DFP: dsPIC33AK-MC_DFP v1.1.109
- Project file: garuda.X/

## Code Style
- C11, no dynamic allocation (bare-metal real-time)
- All ISR handlers < timing budgets in spec §6.2
- Use stdint.h types (uint8_t, uint16_t, etc.)
- Prefix all globals with module name: dshot_frame, bemf_zc_count, etc.
- Hardware registers accessed via MCC-generated macros where possible
- Comments must cite specification section: /* §3.2.1 Step 1: A=HIGH, B=LOW, C=Float */

## Interrupt Priority Scheme (from spec §6.1)
Priority 7: PWM Fault (overcurrent) — <200ns
Priority 6: Comparator ZC — <500ns
Priority 5: Commutation Timer — <1µs
Priority 4: ADC Complete — <2µs
Priority 3: DShot Input Capture — DMA-assisted
Priority 2: DShot Frame Complete — <5µs
Priority 1: 1ms System Tick
Priority 0: Background (EEPROM, diagnostics)

## Testing
Run test vectors from spec §2.2.3 against dshot_crc() implementation.
Verify state machine transitions match §3.1.2 table exactly.
```

---

## 4. Phase-by-Phase Implementation Plan

### Phase 1: Foundation (Weeks 1-4)
**Goal:** Motor spins with manual/open-loop commutation

#### Task 1.1: Project Scaffolding
```
Claude Code prompt:
"Clone the reference project's mcc_generated_files/ directory structure.
Set up the garuda.X MPLAB X project targeting dsPIC33AK128MC106.
Create the directory structure from the plan. Initialize all .c/.h files
with header guards and module documentation citing spec sections."
```

**Files to create:** All directory structure, stub files with proper headers.

**Reference files to copy & adapt:**
- `reference/project/pmsm.X/mcc_generated_files/system/` → Clock init (200 MHz PLL)
- `reference/project/pmsm.X/mcc_generated_files/pwm/` → PWM peripheral init
- `reference/project/pmsm.X/nbproject/` → Project configuration (adapt for new source tree)

#### Task 1.2: Clock & GPIO Configuration
```
Claude Code prompt:
"Using the reference project's system clock configuration as template,
configure the dsPIC33AK128MC106 for 200MHz operation. Set up GPIO for:
- 3x PWM high/low pairs (6 outputs) for half-bridge driving
- 1x DShot input capture pin
- 1x UART TX/RX
- 1x LED, 1x button
Reference: dsPIC33AK FRM chapters on oscillator and I/O ports."
```

#### Task 1.3: PWM Setup for 6-Step Control
```
Claude Code prompt:
"Adapt the reference project's PWM generator init (PG1, PG2, PG3) for
6-step trapezoidal control. Key differences from FOC reference:
- Need PWM OVERRIDE mode (PGxIOCONH.OVRENx bits) for direct gate control
- Each step: one phase = PWM modulated, one = always LOW, one = floating
- Dead time: 300ns default (§7.6.2), configurable via EEPROM
- PWM frequency: 24 kHz default (§4.1 PWM_FREQ_KHZ)
- Center-aligned PWM with ADC trigger at center for BEMF sampling
Build the commutation table per §3.2.1 six-step table.
Create hal_pwm.c with function: void hal_pwm_set_step(uint8_t step, uint16_t duty)"
```

**Critical register mapping from reference:**
```c
// From reference mcc_generated_files/pwm/ — adapt these patterns:
// PG1CON, PG2CON, PG3CON — PWM generator config
// PGxDC — Duty cycle register
// PGxIOCONH — Override control (THIS IS KEY FOR 6-STEP)
//   bit OVRENx = 1 to enable override
//   bit OVRDATx = override output value
// PGxDTH/PGxDTL — Dead time high/low side
// PGxTRIGA — ADC trigger position (set to center for BEMF sampling)
```

#### Task 1.4: Basic Commutation Table
```
Claude Code prompt:
"Implement commutation.c per spec §3.2.1 six-step commutation table.
Each step must configure PWM override registers for the three half-bridges.
Function: void commutation_execute_step(uint8_t step)
The step value (0-5) maps to the truth table from the spec.
Include the BEMF sensing phase and expected ZC polarity for each step.
Timer interrupt drives commutation at a fixed frequency for open-loop testing."
```

#### Task 1.5: Open-Loop Ramp
```
Claude Code prompt:
"Implement startup.c with the ALIGN and OPEN_LOOP_RAMP states from §3.1.
- ALIGN: Apply fixed commutation state for 50-500ms (configurable)
- OPEN_LOOP_RAMP: Step through commutation table at increasing frequency
- Use a timer ISR to drive commutation stepping
- Ramp from ~500 RPM to target open-loop speed
- NO BEMF feedback in this phase (pure forced commutation)
Success: motor spins smoothly in open loop."
```

**Phase 1 Deliverable:** Motor spins open-loop from the dsPIC33AK.

---

### Phase 2: Sensorless Closed-Loop (Weeks 5-8)
**Goal:** Reliable sensorless BEMF-based commutation

#### Task 2.1: ADC-Based BEMF Sampling
```
Claude Code prompt:
"Using the reference project's ADC initialization as template, configure
ADC channels for:
- 3x BEMF voltage (one per motor phase, sample floating phase)
- 1x DC bus voltage (Vbus)
- 1x Temperature (NTC or internal sensor)
ADC trigger from PWM center (PGxTRIGA) — sample during PWM ON time.
Only the FLOATING phase ADC is relevant each step.
Implement hal_adc.c to route ADC to correct phase based on commutation step."
```

#### Task 2.2: Zero-Crossing Detection
```
Claude Code prompt:
"Implement bemf.c per §3.2.3 BEMF Sampling Strategy:
1. At PWM center, ADC samples floating phase voltage
2. Compare against virtual neutral (Vbus/2 from DAC)
3. Majority filter: 3 consecutive same-polarity samples = valid ZC
4. Detect polarity transition per commutation step table
5. Timestamp ZC event for timing calculation

Also implement hardware comparator ZC detection (§3.3.2 hybrid approach):
- Configure comparator to compare BEMF against DAC output (Vbus/2)
- Comparator interrupt captures ZC edge
- ADC integration validates to reject noise"
```

#### Task 2.3: Commutation Timing Engine
```
Claude Code prompt:
"Implement timing.c per §3.2.2:
- After ZC detected, calculate commutation delay: T_next = T_zc + (30° - advance)
- Timing advance: configurable 0-30° (default 15° for drone motors)
- Track commutation period for speed calculation
- Blanking time: 5-20% of step period (ignore comparator after commutation)
- Demagnetization time: 10-35% of step period
Use SCCP timer capture for precise ZC timestamping."
```

#### Task 2.4: Desync Detection
```
Claude Code prompt:
"Implement desync.c per §3.2.4:
- Track commutation period variance over 6 steps
- If variance > 25% of average → warning
- If no ZC within 150% expected time → force commutation
- If 3+ consecutive forced commutations → desync confirmed
- Recovery: reduce duty 50%, attempt resync, 3 failures → FAULT
- Track stress metric for EDT telemetry"
```

#### Task 2.5: CLC Hardware Commutation (§3.3.6)
```
Claude Code prompt:
"Using the reference project's CLC peripheral init as template, implement
hal_clc.c to offload the commutation truth table to hardware:
- 3-bit sector register as CLC input
- CLC outputs drive PWM override directly
- CPU only writes sector value; CLC handles all 6 gate signals
- Latency: <10ns from sector update to gate drive change
This eliminates ISR jitter for commutation above 100k eRPM.
Refer to spec §3.3.6 CLC Configuration truth table."
```

#### Task 2.6: Full State Machine
```
Claude Code prompt:
"Implement the complete 8-state machine in garuda_main.c per §3.1:
States: POWER_ON_RESET, IDLE, ARMED, ALIGN, OPEN_LOOP_RAMP, 
        CLOSED_LOOP, BRAKING, FAULT
Transition table per §3.1.2. Run in 1ms system tick ISR.
Each state has: entry action, periodic action, exit action.
Throttle input initially from potentiometer (ADC) for testing."
```

**Phase 2 Deliverable:** Motor runs closed-loop sensorless on 3+ motor types.

---

### Phase 3: DShot Protocol (Weeks 9-12)
**Goal:** Full DShot600 + EDT compatibility with Betaflight

#### Task 3.1: DShot Receiver
```
Claude Code prompt:
"Implement dshot.c for DShot150/300/600/1200 decoding per §2.1:
- Use Input Capture peripheral (from reference) + DMA
- Capture rising and falling edges of each bit
- T0H vs T1H discrimination per timing table in §2.1.1
- 16-bit frame: [Throttle:11][Telemetry:1][CRC:4]
- CRC: (value ^ (value >> 4) ^ (value >> 8)) & 0x0F
- Auto-detect DShot rate from first frame timing
Validate against test vectors in §2.2.3."
```

#### Task 3.2: DShot Command Handler
```
Claude Code prompt:
"Implement dshot_commands.c per §2.1.3 command table:
- Commands 0-14 with repeat requirements (6x for settings)
- Throttle range 48-2047 (2000 steps)
- DISARM (0), BEEP (1-5), SPIN_DIRECTION (7/8), 3D_MODE (9/10),
  SAVE_SETTINGS (12), EDT_ENABLE/DISABLE (13/14)
- Repeat counter with timeout for multi-frame commands"
```

#### Task 3.3: Bidirectional DShot
```
Claude Code prompt:
"Implement dshot_bidir.c per §2.2:
- Detect bidirectional mode by idle HIGH polarity
- Invert received bits before CRC validation
- After receiving frame, switch pin to output
- Encode eRPM telemetry as GCR (5-bit encoding)
- eRPM = Mantissa << Exponent format
- Response starts 30µs ±5µs after FC frame end
- 21-bit GCR response duration ~28µs at DShot600
Checksum handling per §2.2.2 (critical compatibility detail)."
```

#### Task 3.4: EDT Telemetry
```
Claude Code prompt:
"Implement edt.c per §2.3:
- EDT frame detection: exponent != 0 AND mantissa bit 8 == 0
- Frame types: Temperature, Voltage, Current, Debug, Stress, Status
- Scheduling per §2.3.3: Status/Temp/Voltage/Current on offset timers
- Base interval 1000ms default, configurable
- Status frame: immediate on error/warning/alert
- Rate cap: max 10 EDT frames/second default
- eRPM is default/filler when no EDT pending"
```

**Phase 3 Deliverable:** Full DShot600 + EDT working with Betaflight.

---

### Phase 4: Production Features (Weeks 13-16)
**Goal:** Production-ready firmware

#### Task 4.1: EEPROM Configuration System
```
Claude Code prompt:
"Implement eeprom.c per §4.1 binary layout:
- 64-byte config block with CRC-16-CCITT
- Magic: 0x4D50, schema version, all fields per table
- Use dsPIC33AK DEE library (MCC-generated) for flash emulation
- Write coalescing: buffer changes, commit on SAVE_SETTINGS
- CRC failure → factory defaults + error log
- Wear leveling: 4KB reserved, 4x redundancy"
```

#### Task 4.2: Garuda Serial Protocol (GSP)
```
Claude Code prompt:
"Implement gsp.c per §7.2:
- Packet: START(0x24) + LENGTH + COMMAND + PAYLOAD + CRC16
- Commands: GET_INFO, GET_CONFIG, SET_CONFIG, SAVE_CONFIG,
  RESET_DEFAULT, ENTER_BOOTLOADER, GET_TELEMETRY, MOTOR_TEST, BEEP
- INFO_RESP: 32-byte payload with magic 'GRDA', capabilities
- TELEMETRY_RESP: 16-byte with eRPM, V, I, T, duty, state
- Use UART from reference project, half-duplex for 1-wire mode"
```

#### Task 4.3: Protection Features
```
Claude Code prompt:
"Implement protection modules per spec:
- overcurrent.c: Hardware comparator at Priority 7, <1µs trip
- thermal.c: NTC ADC reading, derating curve, shutdown threshold
- voltage.c: Vbus monitoring, under/over voltage cutoff
- signal_loss.c: 100ms timeout, transition to BRAKING state
All faults → FAULT state, require throttle=0 for 500ms to clear."
```

#### Task 4.4: Dead Time Auto-Calibration
```
Claude Code prompt:
"Implement calibration.c per §7.6:
- Start at 50ns dead time, 1% duty
- Ramp duty to 5%, monitor DC bus current
- Current spike > 2x expected → shoot-through → increase DT by 50ns
- Optimal = detected minimum + 50ns margin
- Store to EEPROM, mark calibrated
- Pre-calibration: 300ns default, 90% max duty, EDT warning"
```

#### Task 4.5: Bootloader
```
Claude Code prompt:
"Implement bootloader/ per §5 and §7.4:
- 4KB at 0x0000-0x0FFF, application starts at 0x1000
- Commands: BL_PING, BL_GET_INFO, BL_ERASE, BL_WRITE (256-byte pages),
  BL_READ, BL_VERIFY, BL_BOOT
- Entry: DShot CMD 255 x10, or UART LOW at power-up, or GSP command
- CRC-32 validation before boot
- Dual-image (A/B) with rollback on 3 consecutive boot failures
- Anti-brick: bootloader always accessible via UART"
```

**Phase 4 Deliverable:** Production-ready firmware passing all tests in Appendix B.

---

### Phase 5: Advanced Features (Weeks 17-20)
**Goal:** Performance optimization and advanced algorithms

#### Task 5.1: Flux Linkage Integration (§3.3.1)
```
"Implement low-speed enhancement: FPU accumulates flux integral,
commutation on threshold crossing instead of instantaneous ZC.
Target: sensorless down to 2% throttle."
```

#### Task 5.2: Ternary Hysteresis Control (§3.3.3)
```
"Three-state torque control: Active/Zero/Reverse vectors.
40-60% torque ripple reduction without FOC overhead.
CLC implements 3-state comparator."
```

#### Task 5.3: Adaptive Lead Angle (§3.3.4)
```
"Dynamic timing advance based on speed, current, temperature.
advance = base + (eRPM/10000) + (I_phase/20) - ((T-25)/50)
Range 0-45°, update every commutation cycle."
```

#### Task 5.4: Mixed Decay Mode (§3.3.5)
```
"Adaptive PWM decay: slow (<20% throttle), fast (>80%), 
mixed (20-80%) based on throttle rate of change."
```

#### Task 5.5: MSP Passthrough (§7.3)
```
"MSP v1 frame support for Betaflight passthrough:
Custom MSP commands 126-128 for ESC info/config/flash."
```

---

## 5. Key Peripheral Register Patterns from Reference

These patterns are extracted from the AN1292 FOC reference project and adapted for Garuda's 6-step control. Claude Code should reference these when implementing HAL modules.

### PWM Configuration (from reference, adapted for 6-step)
```c
// Reference uses PWM for SVM (space vector modulation)
// Garuda adapts for 6-step override control

// Key registers per PWM Generator (PG1, PG2, PG3):
// PGxCON[H/L] — Mode, clock source, trigger
// PGxDC       — Duty cycle (used for active phase modulation)
// PGxPHASE    — Phase offset (typically 0 for ESC)
// PGxPER      — Period (determines PWM frequency)
// PGxDTH/DTL  — Dead time high/low (300ns default)
// PGxIOCONH   — OUTPUT OVERRIDE CONTROL (critical for 6-step)
//   OVRENH = 1: Override high-side output
//   OVRENL = 1: Override low-side output  
//   OVRDAT<1:0>: Override values (HIGH/LOW/Float)
// PGxTRIGA    — ADC trigger A position (set to ~50% for center sampling)

// 6-step override pattern example:
// Step 1: A=PWM, B=LOW, C=Float
//   PG1: OVRENH=0, OVRENL=0 (PWM active)
//   PG2: OVRENH=1, OVRENL=1, OVRDAT=0b01 (high=off, low=on)
//   PG3: OVRENH=1, OVRENL=1, OVRDAT=0b00 (both off = float)
```

### ADC Configuration (from reference, adapted for BEMF)
```c
// Reference samples phase currents; Garuda samples BEMF voltages
// Same ADC hardware, different channel routing per commutation step

// Key: ADC trigger from PWM center ensures sampling during ON-time
// when BEMF voltage is present on floating phase
// ADC cores: typically 4 cores, map to channels dynamically
// Trigger source: PGxTRIGA from active PWM generator
```

### Comparator for ZC Detection (from reference, adapted)
```c
// Reference uses comparator for overcurrent protection
// Garuda adds ZC detection use:
// CMP input: BEMF floating phase voltage
// DAC output: Vbus/2 (virtual neutral point)
// Interrupt on edge transition (rising or falling per step polarity)
```

---

## 6. Critical Implementation Notes for Claude Code

### A. ISR Timing Budgets — Non-Negotiable
Every ISR must meet the timing budget in §6.2. Claude Code should:
- Count instructions in each ISR
- Use `__attribute__((interrupt, shadow))` for fast context save
- Minimize ISR work — set flags, let background process

### B. DShot CRC Test Vectors — Gate Check
Before any DShot work ships, verify against §2.2.3:
```
Throttle=0,    Telem=0 → CRC=0x0, Frame=0x0000
Throttle=48,   Telem=0 → CRC=0x3, Frame=0x0603
Throttle=1047, Telem=0 → CRC=0x7, Frame=0x82F7
Throttle=2047, Telem=0 → CRC=0xF, Frame=0xFFEF
Throttle=1047, Telem=1 → CRC=0x6, Frame=0x82E6
```

### C. Clean-Room Compliance — Every File
Every source file must include the header:
```c
/*
 * Project Garuda — dsPIC33AK Drone ESC Firmware
 * Copyright (c) 2026 Microchip Technology Inc.
 * 
 * Clean-room implementation. See Appendix A of specification.
 * Source classification: [MCHP-DS] / [PROTO-DSHOT] / [ORIGINAL]
 */
```

### D. Memory Map
```
0x0000 - 0x0FFF : Bootloader (4KB, protected)
0x1000 - 0x1E7FF: Application (120KB max)
0x1E800- 0x1EFFF: EEPROM emulation area (4KB)
0x1F000- 0x1FFFF: Config backup / reserved
```

### E. Pin Mapping (to be finalized per ESC board)
```
PWM1H/L — Phase A high/low gate drive
PWM2H/L — Phase B high/low gate drive
PWM3H/L — Phase C high/low gate drive
AN0-AN2 — BEMF voltage sensing (A, B, C)
AN3     — DC bus voltage
AN4     — Current sense (if available)
AN5     — Temperature (NTC)
RPI     — DShot input (Input Capture)
TX/RX   — UART (GSP / debug)
CMP1IN  — Comparator input (BEMF for HW ZC)
```

---

## 7. Claude Code Session Workflow

For each development session, the recommended workflow is:

1. **Start session:** `claude` in project-garuda/ directory
2. **Specify task:** Reference the task number (e.g., "Implement Task 2.2: Zero-Crossing Detection")
3. **Claude Code reads:** CLAUDE.md + specification sections + reference peripheral code
4. **Implementation:** Claude Code writes .c/.h files with proper headers and spec citations
5. **Verify:** Run test vectors where applicable, check ISR timing estimates
6. **Commit:** Descriptive commit message referencing spec section

### Example Claude Code Prompts (Ready to Use)

**Session 1:**
> "Read CLAUDE.md. Create the complete project directory structure with all stub files.
> Copy relevant MCC peripheral init patterns from reference/ for PWM, ADC, UART, CMP, DAC, CLC.
> Initialize garuda_main.c with the state machine skeleton from spec §3.1."

**Session 2:**
> "Implement hal_pwm.c and commutation.c. The 6-step commutation table is in spec §3.2.1.
> Use PWM override registers (PGxIOCONH) to control each half-bridge per step.
> Reference the PWM init from reference/mcc_generated_files/pwm/ for register patterns.
> PWM frequency = 24kHz, dead time = 300ns."

**Session 3:**
> "Implement dshot.c DShot receiver using Input Capture + DMA.
> Spec §2.1.1 has timing tables. §2.1.2 has frame structure and CRC.
> Validate against test vectors in §2.2.3. Support auto-detection of DShot rate."

---

## 8. Risk Mitigations & Contingencies

| Risk | Mitigation | Contingency |
|---|---|---|
| XC-DSC compiler unavailable in Claude Code | Write portable C, validate register addresses from datasheet | Test compilation on developer workstation |
| CLC configuration complexity | Start with software commutation, add CLC in Phase 2 | Software-only mode is fully functional |
| DShot timing precision | Use hardware Input Capture + DMA, not bit-banging | Start with DShot300 (more relaxed timing) |
| BEMF noise at low speed | Implement majority filter first, add flux integration in Phase 5 | Higher minimum throttle for initial release |
| Reference repo network unavailable | All register patterns documented in this plan | Use dsPIC33AK datasheet directly |

---

## 9. Verification Checklist per Phase

### Phase 1 ✓ Criteria
- [ ] dsPIC33AK boots at 200MHz
- [ ] PWM outputs toggling at 24kHz on all 3 phases
- [ ] 6-step commutation sequence verified on oscilloscope
- [ ] Motor spins open-loop with manual step timing

### Phase 2 ✓ Criteria
- [ ] ADC sampling BEMF at PWM center
- [ ] Zero-crossing detection working (LED toggle on ZC)
- [ ] Closed-loop commutation stable at multiple speeds
- [ ] Desync detection and recovery functional
- [ ] 3+ different motors tested successfully

### Phase 3 ✓ Criteria
- [ ] DShot CRC test vectors pass (all 5 from §2.2.3)
- [ ] DShot600 decode from Betaflight FC
- [ ] Bidirectional DShot eRPM telemetry verified
- [ ] EDT frames decoded correctly on FC side
- [ ] All DShot commands functional

### Phase 4 ✓ Criteria
- [ ] EEPROM read/write with CRC validation
- [ ] GSP protocol: all commands functional from PC tool
- [ ] Overcurrent fault trips in <1ms
- [ ] Signal loss failsafe stops motor in <500ms
- [ ] Bootloader: firmware update via 1-wire passthrough
- [ ] Dead time calibration runs successfully

### Phase 5 ✓ Criteria
- [ ] Sensorless operation down to 2% throttle (flux linkage)
- [ ] Measurable torque ripple reduction (ternary hysteresis)
- [ ] Adaptive lead angle improves efficiency across speed range
- [ ] Acoustic noise reduced at low speeds (mixed decay)
