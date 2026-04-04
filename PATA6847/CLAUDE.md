# Garuda ESC — CK Board (dsPIC33CK + ATA6847)

## Project Context
6-step trapezoidal BLDC ESC firmware for the EV43F54A evaluation board. Sensorless BEMF zero-crossing detection using ATA6847 integrated gate driver with digital comparator outputs. Targets drone motors (1000-2000KV, 7PP) at 12-24V.

## Current State (2026-04-04)
- **ZC Architecture**: CLC D-FF + SCCP2 IC capture + poll hybrid (see `docs/zc_detection_architecture_v3.md`)
- **Best Results**: 100% ZC success with prop on both A2212 (12V) and 2810 (18V)
- **PWM**: 24kHz complementary, 210.5kHz non-integer poll
- **Advance**: AM32-style fixed 15° (TIMING_ADVANCE_LEVEL=2)
- **Commit**: `4d34fd5`

## Build Instructions
```bash
# CLI build (specify motor profile)
cd garuda_6step_ck.X
make -f Makefile-cli.mk MOTOR_PROFILE=1  # 0=Hurst, 1=A2212, 2=2810

# MPLAB build
# Uses MOTOR_PROFILE default from garuda_config.h line 17
# Open garuda_6step_ck.X in MPLAB X, Build & Program
```

## Key Files
- `garuda_config.h` — ALL configuration (profiles, features, tuning)
- `garuda_service.c` — State machine, all ISRs
- `motor/bemf_zc.c` — ZC detection core (FastPoll, OnCommutation, RecordZcTiming)
- `hal/hal_clc.c` — CLC D-FF noise filter
- `hal/hal_ic.c` — SCCP1 poll timer + SCCP2 IC capture
- `hal/hal_pwm.c` — PWM + 6-step commutation
- `hal/hal_ata6847.c` — Gate driver SPI register config
- `tools/pot_capture.py` — Bench test telemetry capture
- `docs/zc_detection_architecture_v3.md` — Full architecture documentation

## Critical Hardware Knowledge
- **ATA6847 comparator is INVERTED**: output HIGH when BEMF < neutral
- **Rising ZC → falling comparator edge**, falling ZC → rising edge
- **SCCP IC MOD encoding** (DS70005349 Table 22-3): 0001=rising, 0010=falling
- **PG1TRIGA=0** fires at center-aligned PWM valley (center of ON-time for current polarity config)
- **PWM1H=RB10, PWM2H=RB12, PWM3H=RB14** — can be read as GPIO for PWM state
- **MPLAB build uses MOTOR_PROFILE default from garuda_config.h**, NOT command-line -D flag
- **XC16 `__builtin_disi` NOT supported** on dsPIC33CK — use double-read or seqlocks
- **UPDREQ=1 required for ALL PWM generators** in 6-step (not just master)

## ATA6847 Register Tuning
- **EGBLT** (edge blanking): 15 = 3.75µs max. Higher voltage needs more blanking.
- **SCTHSEL** (VDS threshold): 7 = 2000mV for 24V operation. Lower voltages can use 5 (1500mV).
- **GDUCR3** (slew rate): HS=12.5%, LS=full. Don't change — tested alternatives all worse.
- **ILIM**: Disabled (ILIMEN=0). Cycle-by-cycle chopping limits prop performance.

## Feature Flags
```
FEATURE_IC_ZC           1  — SCCP1 fast poll + SCCP4 HR timer
FEATURE_IC_ZC_CAPTURE   1  — SCCP2 IC for hardware timestamps
FEATURE_CLC_BLANKING    1  — CLC D-FF noise filter
PWM_DRIVE_UNIPOLAR      0  — 0=complementary (default), 1=unipolar
FEATURE_SPEED_PD        0  — Speed PID (broken, needs per-ZC rewrite)
FEATURE_GSP             1  — GSP binary protocol on UART1
```

## Known Working Configurations
1. **Complementary + CLC + IC** (default): proper braking, 100% ZC with prop, noise at 50%/80% duty no-load
2. **Unipolar + IC** (PWM_DRIVE_UNIPOLAR=1): 99.96% ZC no-load, no braking (needs speed PID)

## Production ESC Feature Roadmap

### Phase 1: Core Reliability (Current Focus)
- [ ] **PTG edge-relative sampling** — fix CLC clock position issue for duty-independent clean reads
  - Use PWM2/3 Trigger for PTG (NOT PWM1 — that's ADC)
  - Single detector front end (raw GPIO in PTG ISR)
  - Edge-relative delay (6-7µs after switching edge, not counter-relative)
  - Replaces CLC D-FF as primary conditioner at high speed
- [ ] **PG1TRIGA position tuning** — alternative to PTG, move CLC clock away from edges
- [ ] **Desync recovery hardening** — fast pot transients still cause phantom at 1-read confirm
- [ ] **24V bench testing with LiPo** — eliminate Vbus swing artifacts from bench supply

### Phase 2: Speed Control
- [ ] **Per-ZC interval-based speed PID** (AM32 architecture)
  - Run PID per ZC event (not fixed 1ms rate)
  - Error on commutation INTERVAL (not eRPM)
  - Accumulated input_override clamped 0-MAX_DUTY
  - Integral reset until 100 ZCs after startup
  - Essential for unipolar mode (no braking → duty doesn't control speed)
- [ ] **Stall protection PID** — boost duty when RPM drops under load
- [ ] **Current limit PID** — software duty cap on overcurrent

### Phase 3: Input & Communication
- [ ] **DShot input** — replace pot with DShot protocol for flight controller integration
  - DShot150/300/600 via SCCP IC capture
  - Bidirectional telemetry (eRPM, temperature, voltage, current)
- [ ] **Serial input** — UART-based throttle for ground vehicles
- [ ] **Arming sequence** — throttle-at-zero check before motor enable
- [ ] **Signal loss failsafe** — timeout → motor stop → MCU reset

### Phase 4: Protection & Safety
- [ ] **Software current limiting** — PID-based duty cap using ADC current measurement
- [ ] **Temperature monitoring** — ATA6847 or MCU die temperature → derate duty
- [ ] **Low voltage cutoff** — auto cell count, per-cell threshold, hysteresis
- [ ] **Desync current protection** — fast duty cut on current spike during desync
- [ ] **Watchdog timer** — MCU WDT for hang recovery

### Phase 5: Motor Tuning & Configuration
- [ ] **TIMING_ADVANCE_LEVEL as runtime GSP param** — GUI-tunable per motor
- [ ] **EEPROM parameter storage** — save tuned params across power cycles
- [ ] **Motor auto-detection** — measure Rs/Ls at startup, select profile
- [ ] **Variable PWM frequency** — AM32-style speed-adaptive PWM (smooth IIR transition)
- [ ] **Configurable PWM mode** — runtime switch between complementary/unipolar
- [ ] **Per-step CLC/sample offset tables** — vector-specific noise tuning

### Phase 6: Telemetry & GUI
- [ ] **Extended DShot telemetry** — temperature, current, voltage packets
- [ ] **GUI scope improvements** — real-time waveform capture, trigger modes
- [ ] **GUI parameter editor** — live tuning of advance, PID gains, blanking
- [ ] **GUI motor test** — automated sweep with data logging
- [ ] **Flight data recorder** — on-board logging for post-flight analysis

### Phase 7: Advanced Features
- [ ] **Bidirectional operation** — forward/reverse with direction commands
- [ ] **Brake on stop** — proportional braking via low-side PWM (complementary mode)
- [ ] **Sinusoidal startup** — smooth low-speed startup for heavy props
- [ ] **Field weakening** — extend speed range beyond BEMF limit
- [ ] **Active freewheeling** — complementary L-FET during off-time for efficiency

## Conventions
- Motor profiles are compile-time (MOTOR_PROFILE 0/1/2). Universal params in shared section.
- All duty calculations in LOOPTIME_TCY range. PWM-mode-dependent scaling at output only.
- HR timestamps via SCCP4 (640ns/tick). Timer1 (50µs) for timeouts and system tick.
- CLC reads via `ReadBEMFComparator()` which auto-selects CLC or raw GPIO based on feature flag.
- IC timestamp stored in `zcCandidateHR/T1`. Poll checks `icArmed` to decide if IC captured.
- Commit messages end with `Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>`

## Testing Checklist
Before any release:
- [ ] All 3 profiles build clean (Hurst, A2212, 2810)
- [ ] Both PWM modes build (complementary + unipolar)
- [ ] A2212 12V no-load: full pot sweep, check eRPM/timeouts/IC%
- [ ] A2212 12V prop: idle stability, full throttle, pot release
- [ ] 2810 24V no-load: full sweep, check rough spots above 55k
- [ ] 2810 18V prop: idle stability, full throttle
- [ ] GUI connects and shows eRPM, current, ZC diagnostics
- [ ] Fast pot transients don't cause permanent desync (recovery works)
- [ ] Motor stop → restart works without board reset
