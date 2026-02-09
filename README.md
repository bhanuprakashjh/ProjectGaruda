# Project Garuda

**Open-source drone ESC firmware for the Microchip dsPIC33AK128MC106**

6-step trapezoidal BLDC motor control with DShot protocol, sensorless BEMF zero-crossing detection, and a full protection suite — built from scratch for high-performance multirotor applications.

---

## Key Features

### Motor Control
- **6-step trapezoidal commutation** with PWM override registers for glitch-free phase switching
- **Hybrid BEMF zero-crossing detection** — ADC threshold with 3-sample majority filter + hardware comparators (CMP1/CMP2/CMP3, one per phase)
- **Open-loop startup** with alignment phase, decreasing-period ramp, and automatic transition to closed-loop when ZC is stable within ±100us
- **Demagnetization compensation** — tracks demag events, extends blanking, reduces duty under excessive demag stress (inspired by BLHeli)
- **External rotation detection** — senses BEMF before starting to determine if motor is already spinning (crash recovery)
- **Adaptive timing advance** (Phase 5) — linear interpolation between min/max advance based on eRPM (inspired by Sapog)

### DShot Protocol
- **DShot150/300/600/1200** input via Input Capture + DMA — auto-rate detection
- **Bidirectional DShot** — GCR-encoded eRPM telemetry sent back to flight controller
- **Extended DShot Telemetry (EDT)** — reports temperature, voltage, current, and stress level
- **Full DShot command set** (CMD 0-47) — direction change, beep, save settings, LED control

### Protection Suite
- **Hardware overcurrent** — external fault pin (PCI8/RB11) triggers instant PWM shutdown (<200ns)
- **Software overcurrent** — proportional duty reduction at soft limit, hard fault at absolute limit
- **Thermal derating** — linear power reduction from (limit - 20C) to limit, shutdown above limit
- **Under/overvoltage monitoring** — configurable thresholds in 0.1V resolution
- **Signal loss failsafe** — 250ms DShot timeout, 500ms UART timeout, configurable brake-on-stop
- **Desync detection** — monitors commutation timing variance, faults on repeated missed ZC
- **Consecutive startup failure lockout** — 5 failed starts triggers fault requiring disarm or power cycle

### Configuration & Communication
- **64-byte EEPROM config** with CRC-16 integrity check and factory defaults
- **Garuda Serial Protocol (GSP)** over UART — VESC-inspired packet format for configuration, telemetry, motor test, and firmware update
- **Web-based configurator** (planned) — WebSerial API, real-time telemetry plots, config backup/restore

---

## Target Hardware

| Parameter | Value |
|-----------|-------|
| **MCU** | dsPIC33AK128MC106 |
| **Core** | 200 MHz DSP, hardware FPU |
| **Flash / RAM** | 128 KB / 16 KB |
| **ADC** | Dual-core, 12-bit, 40 MSPS combined |
| **Comparators** | 3 with built-in DAC (used for BEMF ZC) |
| **PWM** | 6 generators, 400 MHz timebase, complementary with dead-time |
| **Dev Board** | MCLV-48V-300W (Microchip) |
| **Production** | Custom ESC board (planned) |

---

## Architecture

### Pin Allocation

```
dsPIC33AK128MC106 — Project Garuda Pin Map
═══════════════════════════════════════════

 MOTOR DRIVE (6 pins)
  RD2 ── PWM1H ── Phase A High     RD3 ── PWM1L ── Phase A Low
  RD0 ── PWM2H ── Phase B High     RD1 ── PWM2L ── Phase B Low
  RC3 ── PWM3H ── Phase C High     RC4 ── PWM3L ── Phase C Low

 BEMF SENSING (3 pins — shared with comparator inputs)
  RA4 ── AD1AN1 / CMP1B ── Phase A    (ADC + comparator)
  RB2 ── AD2AN4 / CMP2B ── Phase B    (ADC + comparator)
  RB5 ── AD2AN2 / CMP3B ── Phase C    (ADC + comparator)

 PROTOCOL I/O
  RD8  ── IC1  ── DShot input (Input Capture + DMA)
  RC10 ── U1TX ── UART TX (GSP / debug)
  RC11 ── U1RX ── UART RX (GSP / debug)

 MONITORING
  RA7  ── AD1AN6  ── DC bus voltage (Vbus)
  RA11 ── AD1AN10 ── Temperature NTC / Pot (dev)
  RB11 ── PCI8    ── External overcurrent fault input
```

### Clock Domains

```
8 MHz FRC → PLL1 → 800 MHz VCO → 200 MHz system clock

CLK1  = 200 MHz  │ System clock (FCY)
CLK5  = 400 MHz  │ PWM timebase
CLK6  = 100 MHz  │ ADC clock
CLK7  = 400 MHz  │ DAC / Comparator clock
CLK8  = 100 MHz  │ UART clock
```

### ISR Priority Map

| Priority | ISR | Source | Budget |
|----------|-----|--------|--------|
| 7 | `_PWM1Interrupt` | PWM Fault PCI (overcurrent) | <200ns |
| 6 | `_AD1CH0Interrupt` | ADC complete (PWM center) | <2us |
| 5 | `_CCT1Interrupt` | Commutation timer match | <1us |
| 4 | `_IC1Interrupt` / DMA | DShot Input Capture | DMA-assisted |
| 1 | `_T1Interrupt` | Timer1 (100us → 1ms tick) | non-critical |
| 0 | Background | Main loop | unbounded |

### Data Flow

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
                   │   TIMING_OnZeroCrossing() ──→ Commutation timer
                   │                                       │
                   │                                       ▼
                   │                              _CCT1Interrupt()
                   │                                       │
                   │                                       ▼
                   │                         Advance step, update overrides
                   │
                   ├── DESYNC_Check() ──→ Fault if desynced
                   ├── THERMAL_Update() ──→ Fault if overtemp
                   └── SIGNAL_LOSS_Check() ──→ Brake if timeout

Bidirectional: After DShot RX ──→ 30us delay ──→ GCR eRPM/EDT response TX
```

---

## Project Structure

```
garudaESC.X/
├── main.c                     # Entry point
├── garuda_service.c/.h        # Motor control state machine
├── garuda_types.h             # All data structures
├── hal/
│   ├── clock.c/.h             # 200 MHz PLL (from Microchip AN1292)
│   ├── device_config.c        # Fuses
│   ├── port_config.c/.h       # GPIO + pin mapping
│   ├── hal_pwm.c/.h           # PWM for 6-step commutation
│   ├── hal_adc.c/.h           # ADC for BEMF + Vbus
│   ├── hal_comparator.c/.h    # CMP1/2/3 for ZC detection
│   ├── hal_timer.c/.h         # SCCP commutation timer
│   ├── hal_ic.c/.h            # Input Capture for DShot
│   ├── timer1.c/.h            # 1ms system tick
│   ├── uart1.c/.h             # UART for GSP protocol
│   └── board_service.c/.h     # Board support
├── motor/
│   ├── commutation.c/.h       # 6-step table + execution
│   ├── bemf.c/.h              # BEMF zero-crossing detection
│   ├── startup.c/.h           # Align + open-loop ramp
│   ├── timing.c/.h            # Commutation timing engine
│   ├── demag.c/.h             # Demagnetization compensation
│   └── desync.c/.h            # Desync detection + recovery
├── protocol/
│   ├── dshot.c/.h             # DShot receiver
│   ├── dshot_bidir.c/.h       # Bidirectional DShot telemetry
│   ├── dshot_commands.c/.h    # DShot command handler
│   ├── edt.c/.h               # Extended DShot Telemetry
│   └── gsp.c/.h               # Garuda Serial Protocol
├── config/
│   ├── eeprom.c/.h            # EEPROM config storage
│   └── defaults.h             # Factory defaults
├── protection/
│   ├── overcurrent.c/.h       # PCI8 fault + ADC software threshold
│   ├── thermal.c/.h           # Temperature monitoring + derating
│   ├── voltage.c/.h           # Under/over voltage protection
│   └── signal_loss.c/.h       # Signal timeout failsafe
└── telemetry/
    ├── telemetry.c/.h         # Centralized telemetry
    └── beep.c/.h              # Motor beep generation
```

---

## Implementation Roadmap

### Phase 1: Foundation — Motor Spins Open-Loop
- MPLAB X project setup targeting dsPIC33AK128MC106
- Clock init (200 MHz), GPIO config, peripheral init
- PWM at 24 kHz with dead-time and override control
- ADC reads BEMF phase voltages
- Open-loop forced commutation — motor spins

### Phase 2: Closed-Loop BEMF Control
- ADC-based zero-crossing detection with majority filter
- Hardware comparator ZC (hybrid mode)
- Commutation timing engine with 30-degree delay
- Open-loop → closed-loop transition
- Demagnetization compensation
- Desync detection and recovery
- Fixed 15-degree timing advance

### Phase 3: DShot Protocol
- DShot150/300/600/1200 decoding via Input Capture + DMA
- Auto-rate detection
- Full command set (0-47)
- Bidirectional DShot with GCR encoding
- Extended DShot Telemetry (EDT)

### Phase 4: Protection, Config & Bootloader
- Overcurrent protection (hardware PCI8 + software threshold)
- Thermal monitoring with linear derating
- Under/overvoltage protection
- Signal loss failsafe
- 64-byte EEPROM config with CRC-16
- GSP protocol over UART
- Dual-image bootloader with UART firmware update

### Phase 5: Advanced Features
- Least-squares ZC regression (Sapog-inspired)
- Adaptive timing advance
- External rotation detection
- Motor auto-detection (pole count, direction, timing)
- Web-based configurator tool (WebSerial API)
- Flash CRC integrity monitoring

---

## Build & Test

| Step | Milestone | Verification |
|------|-----------|-------------|
| 1 | Project compiles | MPLAB build succeeds |
| 2 | MCU boots at 200 MHz | Scope CLK output, LED blinks |
| 3 | PWM outputs at 24 kHz | Scope all 6 pins, check dead time |
| 4 | Override control works | Correct 6-step patterns on scope |
| 5 | ADC reads BEMF | X2CScope / UART, manually rotate motor |
| 6 | Open-loop spin | Motor spins on button press |
| 7 | Closed-loop runs | Motor follows throttle, stable |
| 8 | DShot decode | Betaflight FC commands motor |
| 9 | Bidir DShot | FC reads eRPM from ESC |
| 10 | Protection | Stall → fault → release → recovers |
| 11 | EEPROM | Config survives power cycle |
| 12 | Bootloader | Firmware update via UART |

---

## Reference Firmware Studied

This project incorporates lessons learned from studying these open-source ESC firmwares:

| Firmware | Key Lesson for Garuda |
|----------|----------------------|
| **BLHeli_S** | Demag compensation, comparator-based ZC, DShot bit-timing |
| **VESC** | Proportional current limiting, thermal derating, packet protocol |
| **ASAC-ESC** | Clean startup sequence, ADC threshold ZC, code style reference |
| **Sapog** | Least-squares ZC regression, adaptive timing advance, external rotation detection |
| **Bluejay** | Bidirectional DShot + EDT implementation |
| **AM32** | Multi-MCU DShot support patterns |

See [`reference_esc_comparison.md`](reference_esc_comparison.md) for detailed comparison notes.

---

## Tools & Requirements

- **IDE:** MPLAB X v6.20+
- **Compiler:** XC-DSC v3.21+
- **DFP:** dsPIC33AK-MC_DFP v1.1.109
- **Programmer:** PICkit On Board (PKOBv4) or ICD 4
- **Dev Board:** [MCLV-48V-300W](https://www.microchip.com/en-us/development-tool/EV18H47A) (Microchip)
- **Reference:** Microchip AN1292 (FOC-PLL for dsPIC33AK — HAL layer reused)

---

## License

TBD

---

## Status

**Active development** — Currently in planning phase. Implementation plan v2.2 is complete and reviewed. Code implementation starting with Phase 1.
