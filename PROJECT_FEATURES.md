# Project Garuda — Feature Overview

**A self-learning drone ESC that adapts to its motor, monitors health, and tunes itself over time.**

Built from scratch on Microchip dsPIC33AK128MC106 (200 MHz DSP, 128 KB flash, 24 KB RAM).

---

## What Makes Garuda Different

Most open-source ESC firmware (BLHeli, AM32, Bluejay) runs fixed parameters set by the pilot. If the motor changes, wears out, or the props get damaged, the ESC doesn't know and doesn't adapt. The pilot might notice vibrations — or might not, until a crash.

Garuda is designed to:

1. **Self-commission** — automatically measure motor parameters (resistance, inductance, Kv, pole count, optimal timing) on first run, no manual tuning required.
2. **Continuously monitor health** — track ZC jitter, timing asymmetry, resistance drift, and thermal trends in real-time, producing an explainable health score with five sub-categories.
3. **Adapt within safe bounds** — automatically adjust timing advance, startup duty, and ramp acceleration based on quality metrics, with hard safety envelopes and automatic rollback on degradation.
4. **Persist learned knowledge** — store commissioned parameters, adapted settings, last-known-good snapshots, and wear counters in dual-page EEPROM with CRC integrity.

All of this runs alongside a conventional 6-step trapezoidal BLDC sensorless commutation engine with DShot protocol support.

---

## Target Hardware

| | |
|---|---|
| **MCU** | dsPIC33AK128MC106 — 200 MHz DSP, 128 KB flash, 24 KB RAM |
| **PWM** | 3 generators, 400 MHz timebase, complementary with programmable dead-time |
| **ADC** | Dual-core 12-bit, 40 MSPS aggregate |
| **Comparators** | 3 with built-in DAC (hardware BEMF zero-crossing) |
| **CLC** | 4 configurable logic cells (hardware signal conditioning) |
| **Dev Board** | MCLV-48V-300W (Microchip) |
| **Production** | Custom ESC PCB (planned) |

---

## Full Feature Set (Final Vision)

### 1. Motor Control Core

**6-step trapezoidal BLDC commutation** with sensorless back-EMF zero-crossing detection.

| Feature | Description |
|---------|-------------|
| **Open-loop startup** | Alignment hold (configurable time/duty) followed by forced commutation ramp with configurable acceleration |
| **Closed-loop BEMF** | ADC-sampled BEMF with majority-vote digital filter, or hardware comparator ZC via CMP1/2/3 + DAC reference at Vbus/2 |
| **Commutation timing** | 30-degree advance from ZC event, with configurable timing advance (0-30 deg) |
| **Demagnetization blanking** | PWM switching noise suppression via configurable blanking window after each commutation |
| **Desync detection** | Monitors for missed/late ZC events; automatic recovery by dropping to open-loop ramp if too many consecutive misses |
| **Direction control** | CW/CCW selectable (commutation table reversal), changeable only when stopped |
| **PWM** | 24 kHz center-aligned, 750 ns dead-time (configurable), complementary mode on all 3 half-bridges |

### 2. CLC Hardware Acceleration (Optional)

Configurable Logic Cells provide sub-microsecond signal conditioning in hardware, zero CPU overhead.

| CLC | Function | Description |
|-----|----------|-------------|
| **CLC1** | Commutation qualifier | AND-OR gate combining active comparator output + blanking gate + polarity control. Output feeds PWM sync PCI (PGxSPCI) for deterministic ZC-to-commutation latency. |
| **CLC2** | DShot conditioner | Invert/glitch gate for DShot input signal. Output feeds SCCP capture via CCPxCON2.ICS (no PPS remap needed). |
| CLC3-4 | Reserved | Available for future use (e.g., current limiting, brake logic) |

CLC1 replaces software-polled comparator reading with a hardware event path:
```
CMP output -> CLC1 (AND with blanking + polarity) -> PGxSPCI -> instant commutation
```

### 3. DShot Protocol

Full digital protocol support replacing traditional analog PWM throttle signals.

| Feature | Description |
|---------|-------------|
| **DShot rates** | DShot150, DShot300, DShot600, DShot1200 |
| **Auto-detection** | Automatic baud rate detection on first valid frame |
| **Command set** | Full command support (CMD 0-47): beep, direction, settings, save, LED control |
| **Bidirectional** | GCR-encoded eRPM telemetry sent back on the same wire during idle period |
| **EDT** | Extended DShot Telemetry — multiplexed voltage, current, temperature, stress, status over successive bidir frames |
| **Input capture** | Hardware timer-based decoding via SCCP Input Capture on RD8 (RP57) |

### 4. Self-Commissioning

Automated motor parameter detection — no manual entry of Kv, pole count, or timing required.

**Sequence:** `IDLE -> STATIC_R -> STATIC_L -> DYNAMIC_KV -> DYNAMIC_POLES -> DYNAMIC_TIMING -> VALIDATE -> COMMIT -> COMPLETE`

| Stage | Motor State | Measures | Method |
|-------|------------|----------|--------|
| **Static R** | Stopped | Phase resistance (mOhm) | DC injection, V/I measurement |
| **Static L** | Stopped | Phase inductance (uH) | Voltage step response, di/dt |
| **Dynamic Kv** | Low-speed spin | Back-EMF constant (RPM/V) | BEMF sampling during controlled spin |
| **Dynamic Poles** | Low-speed spin | Pole pair count | Electrical cycles per mechanical revolution |
| **Dynamic Timing** | Low-speed spin | Optimal timing advance (deg) | Sweep timing, measure ZC quality at each angle |
| **Validate** | Run at learned params | Confidence score | Must reach 200/255 to pass |
| **Commit** | — | — | Write to EEPROM, set health baselines |

Progress reported as 0-100% via telemetry. Abort-safe: motor outputs disabled immediately on abort or error. Triggered via DShot command or GSP protocol.

### 5. ZC Quality Estimator

Real-time assessment of commutation quality from telemetry data.

| Metric | Type | Description |
|--------|------|-------------|
| **ZC miss rate** | float (0.0-1.0) | Fraction of commutation steps where no ZC event was detected |
| **False cross rate** | float (0.0-1.0) | Fraction of ZC events with wrong polarity (noise-triggered) |
| **Timeout rate** | float (0.0-1.0) | Fraction of steps where ZC timed out (forced commutation used) |
| **ZC jitter** | float (std dev) | Standard deviation of ZC latency in timer ticks — computed via Welford's online algorithm (no sample storage) |
| **Confidence score** | 0-255 | Composite: `255 - (missRate*127.5 + falseRate*127.5 + timeoutRate*255 + desyncs*50)` |

Operates on a sliding time window (default 1000 ms, configurable). Drains telemetry samples from a lock-free ring buffer (ISR -> main loop, 13 bytes/sample, <35 ns write time).

### 6. Bounded Adaptation

Automatic parameter tuning that improves performance while guaranteeing safety.

**What it tunes:**
- Timing advance (0-30 degrees, configurable envelope)
- Startup duty cycle
- Ramp acceleration rate

**Safety rules:**
- All parameters clamped to compile-time `[min, max]` bounds — cannot exceed envelope
- Only one parameter changed per evaluation cycle (100 ms default)
- Hysteresis: metric must cross threshold by a margin AND hold for multiple cycles before adjustment
- **Last-known-good (LKG) snapshot** saved before any change
- **Auto-rollback** if confidence drops below threshold within 5 seconds of a change
- **Lock-out** after 3 consecutive failures — requires explicit unlock via GSP or DShot command
- Changes applied ONLY at safe boundaries: ESC idle, arming, or closed-loop with throttle < 10%

### 7. Explainable Health Scoring

Five sub-scores that map to physical failure modes a pilot can understand.

| Sub-Score | Weight | What It Detects | Physical Meaning |
|-----------|--------|----------------|------------------|
| **Bearing** | 25% | ZC jitter vs baseline | Vibration from worn bearings |
| **Balance** | 25% | Step-to-step timing asymmetry | Magnet or winding imbalance, bent shaft |
| **Connection** | 20% | Resistance drift from commissioned value | Loose bullet connector, corroded solder joint |
| **Thermal** | 20% | Temperature trend (duty-to-speed ratio) | Overheating motor or ESC |
| **Electrical** | 10% | Vbus ripple, current anomalies | Failing capacitor, battery degradation |

Each sub-score is 0-255 (255 = healthy). **Composite health** is the weighted average. **Trend** indicator (+1/0/-1) shows whether health is improving, stable, or declining.

Thresholds:
- **Degraded**: any sub-score < 128 (50%)
- **Critical**: any sub-score < 64 (25%)

Baselines established during commissioning or first stable run. Health data persisted to EEPROM for cross-session trending.

### 8. Persistent Storage (EEPROM v2)

128-byte dual-page flash storage with power-loss safety.

```
EEPROM Image (128 bytes)
+------------------------------+-------------------------------+
|  GARUDA_CONFIG_T (64 bytes)  |  EEPROM_LEARNED_T (64 bytes)  |
|  User configuration          |  Learned motor data            |
+------------------------------+-------------------------------+

Learned block layout:
  [0-3]   Header: magic=0x4752 ('GR'), schema=2, flags
  [4-13]  Commissioned: R, L, Kv, poles, timing, startup duty
  [14-19] Active adapted: timing, duty, ramp accel, confidence
  [20-25] Last-known-good: same fields as active
  [26-31] Health baselines: jitter, asymmetry, resistance
  [32-39] Wear stats: write count, rollback count, commission count,
          operating hours, startup count
  [40-61] Reserved (future expansion)
  [62-63] CRC-16-CCITT over bytes 0-61
```

**Dual-page ping-pong:** Writes alternate between page 0 and page 1. On boot, the page with the higher write count and valid CRC wins. If both are corrupt, factory defaults are loaded.

**Write throttle:** Minimum 60 seconds between writes to prevent flash wear. Typical flash endurance: 100,000 cycles = 190+ years at one write per minute.

### 9. Protection Systems

| Protection | Trigger | Response |
|------------|---------|----------|
| **Overcurrent** | External comparator on PCI8 pin (hardware) | Immediate PWM shutdown via PCI fault latch, sub-microsecond response |
| **Overvoltage** | ADC Vbus > configurable threshold | Disable outputs, set FAULT state |
| **Undervoltage** | ADC Vbus < configurable threshold | Disable outputs, set FAULT state |
| **Stall** | No ZC events for N consecutive steps | Open-loop restart attempt, then FAULT if repeated |
| **Desync** | ZC quality confidence drops below critical | Drop to open-loop, attempt re-sync, FAULT after max retries |
| **Signal loss** | No valid DShot frame for configurable timeout | Configurable: brake, coast, or disarm |
| **Thermal** | Duty-to-speed ratio exceeds threshold | Linear derating (reduce max duty proportionally) |

### 10. Communication Protocols

#### DShot (Primary — via Input Capture)
- Digital throttle protocol, 150-1200 kbit/s
- 16-bit frame: 11-bit throttle + telemetry request + 4-bit CRC
- Bidirectional: eRPM telemetry on same wire
- EDT: voltage, current, temperature, stress level in successive frames

#### GSP — Garuda Serial Protocol (Secondary — via UART)
- Binary protocol over UART (115200 8N1 default)
- Read/write all configuration parameters
- Trigger commissioning, read health scores, read telemetry
- Firmware version query, factory reset
- Packet format: `[SYNC][LEN][CMD][PAYLOAD][CRC16]`

#### Future: Configurator GUI
- Web-based (WebSerial API) or desktop application
- Real-time health dashboard with sub-score breakdown
- Commissioning wizard with progress bar
- Parameter editor with safety limit validation
- Firmware update capability

---

## Architecture

### Data Flow

```
                        ISR Domain (24 kHz / 10 kHz)
                        ============================
PWM rate ADC ISR ──> Read BEMF, Vbus, Pot ──> State machine
                                                    |
Timer1 ISR (100us) ──> Heartbeat, buttons ──> Commutation timing
                                                    |
                     COMMUTATION_AdvanceStep() ─────┘
                              |
                              v
                    RingBuffer_Write(sample)     <── 13 bytes, <35 ns
                              |                      lock-free SPSC
                              v
                        Main Loop (1 ms / 10 ms / 100 ms)
                        =================================
                    RingBuffer_Read() batch drain
                              |
              +---------------+---------------+
              |               |               |
              v               v               v
      QUALITY_Update()  HEALTH_Update()  ADAPT_Evaluate()
         @ 1 ms            @ 10 ms          @ 100 ms
              |               |               |
              v               v               v
      confidence score  5 sub-scores    action decision
              |               |               |
              +-------+-------+               |
                      |                       v
                      v                 ADAPT_Apply()
                  EEPROM_Save()      (at safe boundary only)
                  (throttled 60s)
```

### ISR / Main Loop Separation

The control loop (24 kHz ADC ISR + 10 kHz Timer1 ISR) never blocks. The learning loop (main loop) never touches SFRs directly. Communication between them is a single lock-free ring buffer — the ISR writes 13-byte telemetry samples, the main loop drains them in batches.

Adaptation parameters are only applied at **safe boundaries**: ESC idle, during arming, or in closed-loop with throttle below 10%. This prevents mid-flight parameter jumps.

### Module Map

```
dspic33AKESC/
├── main.c                          Entry point, main loop, button handling
├── garuda_types.h                  All data structures (30+ types)
├── garuda_config.h                 User knobs + feature flags
├── garuda_calc_params.h            Derived compile-time constants
├── garuda_service.c/.h             State machine + ISRs (ADC, Timer1, PWM fault)
├── util.h                          Math utilities
│
├── hal/                            Hardware Abstraction Layer
│   ├── clock.c/.h                  200 MHz PLL configuration
│   ├── device_config.c             Fuse settings (WDT, JTAG, protection)
│   ├── port_config.c/.h            GPIO + PPS mapping (PWM, BEMF, DShot, UART)
│   ├── hal_pwm.c/.h                24 kHz PWM, 6-step override, duty control
│   ├── hal_adc.c/.h                BEMF/Vbus/Pot ADC, floating phase muxing
│   ├── hal_comparator.c/.h         CMP1/2/3 + DAC for hardware ZC
│   ├── hal_clc.c/.h                [PLANNED] CLC1 commutation, CLC2 DShot
│   ├── board_service.c/.h          Peripheral init, PWM enable/disable, buttons
│   ├── timer1.c/.h                 100 us system tick
│   ├── uart1.c/.h                  UART1 for GSP/debug
│   ├── eeprom.c/.h                 NVM dual-page storage with CRC
│   └── delay.h                     Blocking delay
│
├── motor/                          Motor Control
│   ├── commutation.c/.h            6-step table + step advance + telemetry inject
│   ├── startup.c/.h                Alignment + open-loop ramp
│   ├── pi.c/.h                     PI controller
│   ├── bemf.c/.h                   [PLANNED] BEMF ZC detection (ADC + CMP)
│   ├── timing.c/.h                 [PLANNED] Commutation timing engine
│   └── desync.c/.h                 [PLANNED] Desync detection + recovery
│
├── learn/                          Self-Learning Modules
│   ├── ring_buffer.c/.h            Lock-free SPSC telemetry ring buffer
│   ├── quality.c/.h                ZC quality metrics (Welford's algorithm)
│   ├── health.c/.h                 5-axis health scoring
│   ├── adaptation.c/.h             Bounded parameter adaptation
│   ├── commission.c/.h             Self-commissioning state machine
│   └── learn_service.c/.h          Rate-divided dispatcher
│
└── protocol/                       [PLANNED] Communication
    ├── dshot.c/.h                  DShot decoder (IC1-based)
    ├── dshot_bidir.c/.h            Bidirectional DShot + GCR
    ├── edt.c/.h                    Extended DShot Telemetry
    ├── dshot_commands.c/.h         Command handler (CMD 0-47)
    └── gsp.c/.h                    Garuda Serial Protocol
```

### Feature Flags

All non-core features are gated behind compile-time flags. When disabled, the code compiles to zero overhead — no extra flash, no extra RAM.

| Flag | Default | Depends On | Controls |
|------|---------|------------|----------|
| `FEATURE_LEARN_MODULES` | 0 | — | Ring buffer, quality estimator, health scoring |
| `FEATURE_ADAPTATION` | 0 | `FEATURE_LEARN_MODULES` | Bounded parameter adaptation |
| `FEATURE_COMMISSION` | 0 | `FEATURE_LEARN_MODULES` | Self-commissioning sequence |
| `FEATURE_EEPROM_V2` | 0 | Any of the above | Persistent learned data storage |
| `FEATURE_CLC_COMMUTATION` | 0 | — | CLC1 hardware ZC qualification |
| `FEATURE_CLC_DSHOT` | 0 | — | CLC2 DShot signal conditioning |

### Resource Budget

| Component | RAM | Flash | Status |
|-----------|-----|-------|--------|
| Phase 1 core (open-loop) | 88 B | 9,024 B | Built |
| Learn modules (all flags on) | ~1,088 B | ~4,900 B | Interfaces built |
| DShot decoder | ~200 B | ~2,000 B | Planned |
| GSP protocol | ~300 B | ~2,500 B | Planned |
| Closed-loop BEMF | ~100 B | ~3,000 B | Planned |
| CLC HAL | ~20 B | ~500 B | Planned |
| **Total estimate** | **~1,800 B (7.5%)** | **~22,000 B (17%)** | |
| **Available** | **24,576 B** | **131,072 B** | |

Plenty of headroom for future features.

---

## Phase Plan

### Phase 1: Open-Loop Foundation — COMPLETE

Everything needed to spin a motor without feedback.

- 200 MHz PLL, GPIO, peripheral initialization
- 24 kHz center-aligned PWM with dead-time and 6-step override control
- ADC reads BEMF phases, Vbus, potentiometer
- State machine: IDLE -> ARMED -> ALIGN -> OL_RAMP -> steady-state hold
- Hardware overcurrent fault protection (PCI8)
- Button control (start/stop, direction)
- Heartbeat LED, UART stub
- **Build: 0 errors, 0 warnings, 9,024 bytes flash, 88 bytes RAM**

### Phase 1.5: Self-Learning Interfaces — COMPLETE

Module interfaces and types for the learning system, all behind disabled feature flags.

- 14 new source files (7 headers + 7 implementations) across `learn/` and `hal/`
- Ring buffer, quality estimator, health scoring, adaptation, commissioning, EEPROM
- ISR integration points in commutation.c, garuda_service.c, main.c
- All conditionally compiled — zero overhead with flags off
- **Build: 0 errors, 0 warnings, identical flash/RAM to Phase 1**

### Phase 2: Closed-Loop BEMF — NEXT

The critical transition from forced commutation to sensorless feedback.

- Wire comparator outputs into commutation loop (CMP1/2/3 already initialized)
- ADC-based ZC detection with configurable majority-vote filter
- Commutation timing engine: capture ZC timestamp, compute 30-degree delay, fire commutation
- Demagnetization blanking window (suppress false ZC from PWM switching noise)
- Open-loop to closed-loop transition (handoff when ZC confidence is sufficient)
- Desync detection: count consecutive missed ZCs, drop to open-loop if threshold exceeded
- Recovery: re-align and re-ramp after desync
- Enable `FEATURE_LEARN_MODULES` — telemetry ring buffer starts feeding real ZC data to quality estimator
- **Exit criteria:** Motor runs stably at varying throttle with automatic desync recovery

### Phase 3: DShot Protocol

Replace potentiometer with digital throttle.

- DShot150/300/600/1200 frame decoding via SCCP Input Capture on RD8
- Auto-rate detection (measure first few frames to determine baud)
- CRC-4 validation, throttle extraction (11-bit, 0-2047)
- DShot command handler (CMD 0-47): beep sequences, direction, settings save, LED control
- Bidirectional DShot: GCR-encoded eRPM telemetry on same wire
- EDT multiplexing: voltage, current, temperature, stress in successive bidir frames
- Signal-loss failsafe: configurable timeout -> brake / coast / disarm
- Optional: CLC2 glitch filter for DShot input conditioning
- **Exit criteria:** ESC responds correctly to BLHeliSuite / Betaflight DShot commands

### Phase 4: Protection, Configuration & Communication

Production hardening and user-facing features.

- Thermal monitoring with linear derating (reduce max duty as temperature rises)
- Under/overvoltage protection with configurable thresholds
- GSP binary protocol over UART: read/write config, trigger commissioning, read health/telemetry
- Enable `FEATURE_EEPROM_V2` — config + learned data persisted across power cycles
- Enable `FEATURE_COMMISSION` — full self-commissioning via DShot command or GSP
- Enable `FEATURE_ADAPTATION` — bounded auto-tuning with rollback safety
- Factory reset command (via DShot CMD or GSP)
- **Exit criteria:** ESC self-commissions on new motor, adapts timing over 100+ flights, recovers from adaptation failures via rollback

### Phase 5: Advanced Features & Configurator

Polish and user experience.

- CLC hardware commutation (sub-microsecond ZC-to-commutation latency)
- Web-based configurator via WebSerial API
  - Real-time health dashboard with 5-axis spider chart
  - Commissioning wizard with live progress
  - Parameter editor with safety limit visualization
  - Flight log viewer with health trends
- External rotation detection (spin direction before startup)
- Sine-wave startup for ultra-smooth low-speed (optional, motor-dependent)
- Bootloader (4 KB) for field firmware updates via UART

---

## Summary

Project Garuda aims to be the first open-source drone ESC firmware with **built-in motor intelligence**. Instead of requiring expert tuning, it learns what it needs to know from the motor itself, continuously validates that knowledge against real-time quality metrics, and adapts within safe bounds as conditions change. If something goes wrong — a bearing wears, a connection loosens, a prop gets damaged — the health scoring system flags it with a human-readable explanation before it becomes a crash.

The architecture separates concerns cleanly: the ISR domain runs the time-critical control loop at 24 kHz, the learning loop processes telemetry at 1-100 ms rates in the main loop, and all communication between them flows through a single lock-free ring buffer. Feature flags ensure every advanced module compiles to zero overhead until explicitly enabled, so the firmware can ship in configurations ranging from a simple open-loop ESC to a fully self-learning system.

Current status: Phase 1 and 1.5 are built and verified. Phase 2 (closed-loop BEMF) is next.
