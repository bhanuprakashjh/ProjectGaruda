# Garuda ESC — CK Board (dsPIC33CK + ATA6847)

Sensorless 6-step trapezoidal BLDC ESC firmware for the **EV43F54A** evaluation board (dsPIC33CK64MP205 + ATA6847 gate driver). Hardware BEMF zero-crossing detection using CLC D-Flip-Flop noise filtering, SCCP2 Input Capture timestamps, and PWM-synchronous sampling.

## Status

| Metric | A2212 12V | 2810 18V | 2810 24V |
|--------|-----------|----------|----------|
| **ZC Success (prop)** | **100%** | **100%** | 99.84% |
| **Max eRPM (prop)** | 48k | 56k | 62k |
| **Max eRPM (no-load)** | 102k | 103k | 103k |
| **Max continuous run** | 120s | 25s | 35s |
| **Max current tested** | 11.7A | 21.4A | 25.5A |

## Quick Start

### Prerequisites
- MPLAB X IDE v6.20+
- XC16 compiler v2.10
- EV43F54A evaluation board
- USB-UART adapter (for telemetry)
- 12-24V power supply

### Build & Flash
```bash
# Option 1: MPLAB X
# Open garuda_6step_ck.X in MPLAB X → Build & Program
# Motor profile set in garuda_config.h line 17

# Option 2: CLI
cd garuda_6step_ck.X
make -f Makefile-cli.mk MOTOR_PROFILE=1  # 0=Hurst, 1=A2212, 2=2810
# Flash hex: dist/default/production/garuda_6step_ck.X.production.hex
```

### Run
1. Connect power supply (12V for A2212, 18-24V for 2810)
2. Press SW1 to start motor
3. Turn pot to increase speed
4. Press SW2 to stop

### Telemetry
```bash
# Python capture tool
python3 tools/pot_capture.py --port /dev/ttyACM1

# GUI (WebSerial)
cd ../gui && npm run dev
# Open http://localhost:5173/ProjectGaruda/ in Chrome
```

## Architecture

See full documentation: [`docs/zc_detection_architecture_v3.md`](garuda_6step_ck.X/docs/zc_detection_architecture_v3.md)

```
ATA6847 BEMF comparator → CLC D-FF (samples at PWM mid-ON)
                         → SCCP2 IC (captures edge, 640ns timestamp)
                         → Poll ISR (reads CLC, confirms, schedules)
                         → SCCP4 OC (fires commutation at IC-precise time)
```

### Key Features
- **CLC D-FF noise filter**: 3 independent D-FFs sample BEMF at PWM mid-ON, hold clean value
- **SCCP2 IC capture**: hardware edge timestamp with 640ns precision
- **Hybrid detection**: IC provides timing, CLC provides validation, poll confirms
- **AM32-style 15° advance**: fixed fraction of interval, scales with speed
- **Non-integer poll frequency**: 210.5kHz breaks PWM aliasing
- **Mode state machine**: ACQUIRE (full deglitch) → TRACK (1-read CLC) → RECOVER

## Motor Profiles

| Profile | Motor | Poles | KV | Voltage | Startup |
|---------|-------|-------|-----|---------|---------|
| 0 | Hurst DMB2424B | 10 (5PP) | 149 | 24V | Gentle ramp |
| 1 | A2212 1400KV | 14 (7PP) | 1400 | 12V | Fast ramp |
| 2 | 2810 1350KV | 14 (7PP) | 1350 | 18-24V | Moderate ramp |

### Adding a New Motor
1. Add a new `#elif MOTOR_PROFILE == N` section in `garuda_config.h`
2. Set motor-specific parameters:
   - `MOTOR_POLE_PAIRS`, `MOTOR_KV`, `MOTOR_RS_MILLIOHM`, `MOTOR_LS_MICROH`
   - Startup: `ALIGN_TIME_MS`, `ALIGN_DUTY`, `RAMP_ACCEL_ERPM_S`, `RAMP_DUTY_CAP`
   - Voltage: `VBUS_OV_THRESHOLD`, `VBUS_UV_THRESHOLD`
   - ZC blanking: `ZC_BLANK_FLOOR_US` (lower for low-Ls motors)
3. Universal parameters (advance, CLC, IC, poll freq) apply automatically
4. Build with `make -f Makefile-cli.mk MOTOR_PROFILE=N`

## File Structure
```
garuda_6step_ck.X/
├── main.c                  # Entry point, peripheral init
├── garuda_config.h         # ALL configuration
├── garuda_types.h          # Data structures
├── garuda_service.c        # State machine, ISRs
├── motor/
│   ├── bemf_zc.c           # ZC detection (FastPoll, RecordZcTiming)
│   ├── commutation.c       # 6-step commutation table
│   └── startup.c           # OL ramp
├── hal/
│   ├── hal_pwm.c           # PWM + commutation patterns
│   ├── hal_clc.c           # CLC D-FF noise filter
│   ├── hal_ic.c            # SCCP1 poll timer + SCCP2 IC
│   ├── hal_com_timer.c     # SCCP4 commutation timer
│   ├── hal_ata6847.c       # Gate driver SPI config
│   ├── hal_adc.c           # ADC (current, voltage, pot)
│   └── port_config.c       # GPIO + PPS routing
├── gsp/
│   ├── gsp.c               # GSP binary protocol (UART)
│   ├── gsp_commands.c      # Command handlers
│   ├── gsp_snapshot.c      # Telemetry snapshot
│   └── gsp_ck_params.c     # Runtime parameters
├── docs/
│   └── zc_detection_architecture_v3.md  # Full architecture doc
├── tools/
│   └── pot_capture.py      # Bench test capture
├── nbproject/              # MPLAB X project files
├── Makefile-cli.mk         # CLI build
└── CLAUDE.md               # AI assistant context + roadmap
```

## GUI
WebSerial-based configurator at [`gui/`](../gui/). Auto-detects CK vs AK board.

```bash
cd gui && npm install && npm run dev
# Open http://localhost:5173/ProjectGaruda/
```

Live at: https://bhanuprakashjh.github.io/ProjectGaruda/

Features: Dashboard (eRPM, current, ZC diagnostics), Scope, Motor Setup, Parameters.
