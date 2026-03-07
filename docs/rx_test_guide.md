# RX Input Testing & Debugging Guide

## Overview

The RX input subsystem handles RC receiver signals (PWM servo pulses and DShot digital protocol) to control motor throttle. The signal path is:

```
Physical signal (RD8 pin)
  → SCCP4 Input Capture (IC4 ISR / DMA)
    → Seqlock mailbox (rxMailbox)
      → RX_Service() main-loop FSM
        → rxCachedThrottleAdc (0-4095)
          → ADC ISR throttle mux
            → garudaData.throttle → motor control
```

For testing **without physical hardware signals**, `GSP_CMD_RX_INJECT` (0x27) writes directly into the seqlock mailbox, bypassing SCCP4. This lets you test the entire downstream path from a Python script over USB serial.

---

## 1. Test Tools

### `tools/gsp_rx_test.py` — Automated Test Suite + Motor Test

**Requirements:** Python 3, `pyserial` (`pip install pyserial`)

#### Automated Tests (30 assertions, 12 tests)

```bash
python3 tools/gsp_rx_test.py [--port /dev/ttyACM0] [--baud 115200]
```

| Test | Description | What It Verifies |
|------|-------------|------------------|
| T0 | PING | Basic GSP connectivity |
| T1 | RX_INJECT ACK | Inject command accepted, link transitions to LOCKING |
| T2 | PWM Lock FSM | 10 valid PWM frames → LOCKING → LOCKED |
| T3 | PWM Throttle Conversion | 1000us=0, 1500us=1023, 2000us=4095 (±1 ADC) |
| T4 | DShot Lock + Conversion | DShot600 lock, value 0→0 ADC, 1000→1948, 2047→4095 |
| T5 | Invalid Frames Reset Lock | CRC failures reset lock count during LOCKING |
| T6 | Timeout → LOST | No frames for 200ms → RX_LINK_LOST + FAULT_RX_LOSS |
| T7 | Auto-Arm | LOCKED + zero throttle → ESC_ARMED |
| T8 | CRC Error Counter | Invalid frames increment `rxCrcErrors` |
| T9 | PWM Full Range Sweep | 1000-2100us in 50us steps, monotonic + accurate |
| T10 | DShot Full Range Sweep | 0-2047 in 100 steps, monotonic + accurate |
| T11 | Re-detect After LOST | LOST → clear fault → IDLE → re-DETECTING |

Use `--skip-timeout` to skip T6/T11 (they need 200ms+ RX timeout each).

#### Interactive Motor Test

```bash
# PWM mode
python3 tools/gsp_rx_test.py --motor-test

# DShot600 mode
python3 tools/gsp_rx_test.py --motor-test --dshot
```

**Controls:**
- Arrow Up / `+` / `]` — increase throttle (PWM: +50us, DShot: +100)
- Arrow Down / `-` / `[` — decrease throttle
- `0` — zero throttle (PWM: 1000us, DShot: 0=disarm)
- `1`-`9` — throttle presets (PWM: 1100-1900us, DShot: spread across 48-2047)
- `q` / Ctrl+C — stop motor and exit safely

**Live telemetry at 5 Hz:**
```
PWM=1500us (  50%) | State=CL       | Duty= 64% | Thr=1023 | Iq= 2.34A Id=-0.03A  4100RPM
```

**Safety:** Ctrl+C always triggers cleanup — injects zero throttle 10x then sends STOP_MOTOR.

### `tools/gsp_detect.py` — Motor Auto-Commissioning CLI

```bash
python3 tools/gsp_detect.py [/dev/ttyACM0]
```

Sends `GSP_CMD_AUTO_DETECT` (0x20) and monitors the detection sequence:
1. Rs (resistance) measurement
2. Ls (inductance) measurement
3. Lambda (flux linkage) measurement
4. PI auto-tune

Reads back measured params and optionally saves to EEPROM.

---

## 2. Firmware Architecture

### Feature Flags (`garuda_config.h`)

```c
#define FEATURE_RX_PWM    1   // RC PWM capture (1000-2000us)
#define FEATURE_RX_DSHOT  1   // DShot digital protocol (150/300/600)
#define FEATURE_RX_AUTO   1   // Auto-detect DShot vs PWM
```

All three must be enabled for full test coverage. The inject handler and test script check feature flags at runtime via `GSP_CMD_GET_INFO` (feature bits 20-22).

### Key Files

| File | Role |
|------|------|
| `input/rx_decode.c` | RX_Service() — main loop FSM, mailbox consumer, throttle conversion |
| `input/rx_decode.h` | Mailbox struct, cached throttle externs |
| `hal/hal_input_capture.c` | SCCP4 IC hardware driver, ISR, DMA, mailbox writer |
| `hal/hal_input_capture.h` | HAL API for detect/PWM/DShot modes |
| `gsp/gsp_commands.c` | HandleRxInject() — test injection into mailbox |
| `gsp/gsp_commands.h` | GSP_CMD_RX_INJECT = 0x27 |

### RX Link State Machine

```
UNLOCKED → DETECTING → LOCKING → LOCKED → (timeout) → LOST
                ↑                                        |
                └────────── (IDLE + re-detect) ──────────┘
```

- **DETECTING**: Captures edges, classifies protocol (PWM vs DShot rate)
- **LOCKING**: Counting consecutive valid frames (need `RX_LOCK_COUNT=10`)
- **LOCKED**: Throttle values flow to motor control; auto-arm if zero throttle
- **LOST**: No valid frame for `RX_TIMEOUT_MS=200`; sets FAULT_RX_LOSS if motor was running

### Throttle Conversion

**PWM** (`PwmUsToThrottleAdc`):
- Input: 1000-2000us pulse width
- Deadband: 1000 ± 25us = zero throttle
- Output: 0-4095 ADC value
- Formula: `(min(pulseUs-1000, 2000) * 4095) / 2000`

**DShot** (`DshotToThrottleAdc`):
- Input: 0-2047 DShot value
- Values 0-47 = commands (mapped to 0)
- Values 48-2047 = throttle
- Output: 0-4095 ADC value
- Formula: `(min(dshotVal-48, 1999) * 4095) / 1999`

### Seqlock Mailbox Protocol

The ISR writes to `rxMailbox` using a seqlock pattern:
```c
rxMailbox.seqNum++;      // odd = writing
rxMailbox.value = val;
rxMailbox.valid = ok;
rxMailbox.seqNum++;      // even = complete
```

RX_Service() reads with torn-read detection:
```c
seq1 = rxMailbox.seqNum;
value = rxMailbox.value;
valid = rxMailbox.valid;
seq2 = rxMailbox.seqNum;
if (seq1 != seq2 || (seq1 & 1)) skip;  // torn read
```

### GSP_CMD_RX_INJECT (0x27)

**Request:** 4 bytes `[protocol:u8, value:u16 LE, valid:u8]`
- `protocol`: 0=NONE, 1=PWM, 2=DSHOT150, 3=DSHOT300, 4=DSHOT600
- `value`: pulse width in us (PWM) or 0-2047 (DShot)
- `valid`: 1=good frame, 0=CRC fail / invalid

**Response:** Empty ACK (cmd echo, no payload)

**Behavior:**
1. First call disables IC4 ISR + DMA (prevents RD8 noise race)
2. Bootstraps link state from UNLOCKED/DETECTING/LOST to LOCKING
3. Writes seqlock mailbox (same pattern as ISR)
4. RX_Service() processes on next main loop iteration

---

## 3. Bugs Found & Fixed During Testing

Three real firmware bugs were discovered through inject testing:

### Bug 1: Mailbox Re-consumption

**Symptom:** Lock count filled instantly (1 inject → LOCKED), timeouts never fired.

**Root cause:** `RX_Service()` re-processed the same mailbox frame on every main loop call when `seqNum` hadn't changed. With the main loop running at ~100kHz and inject at ~50Hz, the same frame was consumed thousands of times.

**Fix:** Added `if (seq2 == lastSeqNum) goto rx_timeout_check;` — skip to timeout check when no new frame.

### Bug 2: LOST State Stuck on Stale faultCode

**Symptom:** After timeout → LOST → FAULT_RX_LOSS, clearing the fault returned to IDLE but the LOST handler wouldn't re-detect because it checked `faultCode == FAULT_NONE`. The fault code was cleared but re-detection still failed in some sequences.

**Root cause:** The LOST→DETECTING transition required both `state == ESC_IDLE` AND `faultCode == FAULT_NONE`. But fault code could persist between state transitions.

**Fix:** Removed `faultCode` check — `state == ESC_IDLE` alone is sufficient. If the motor was running when signal was lost, the timeout sets `state = ESC_FAULT`. User must CLEAR_FAULT → IDLE, which is the right gate.

### Bug 3: DShot Scaling Off-by-One

**Symptom:** DShot value 2047 mapped to 4093 instead of 4095.

**Root cause:** Mapping was `(dshotVal-48) * 4095 / 2000` but the range is 48-2047 = 1999 values, not 2000.

**Fix:** Changed to `(dshotVal-48) * 4095 / 1999`.

---

## 4. FOC Auto-Commissioning Changes

### What Changed (`foc_v2_detect.c`)

Complete rewrite of the auto-detection sequence:

| Phase | Duration | What It Does |
|-------|----------|--------------|
| Rs | ~1.2s | Ramp Id to 0.5A at θ=0, measure Vd/Id → resistance |
| Ls | ~4s | Voltage step pulses (10 pulses, averaged), measure dI/dt → inductance |
| Re-align | 200ms | Re-lock rotor at θ=0 before spinning |
| d→q transition | 120ms | Smooth transition from Id to Iq while θ advances |
| Lambda | ~4s | I/f spin-up, steady-state Vq = Rs·Iq + ω·λ → λ = (Vq-Rs·Iq)/ω |
| Auto-tune | instant | Kp = ωbw·Ls, Ki = ωbw·Rs (1 kHz bandwidth) |

### Key Design Decisions

- **Test current 0.5A** (conservative) — U25B on MCLV-48V-300W has no LEB, trips on switching transients at higher currents
- **d→q transition WHILE spinning** — commanding Iq at static θ=0 creates 90° torque offset, BEMF appears in Vd instead of Vq → λ ≈ 0
- **Multiple Ls pulses (10)** — averaged for noise rejection, with 40ms coast time between pulses for current decay + U25B recovery
- **Re-alignment phase** — rotor drifts during Ls voltage steps, must re-lock before spinning

### GUI Auto-Detect (`MotorTuningPanel.tsx`)

The GUI Motor Setup tab now has a "Run Auto-Detect" button that:
1. Sends `GSP_CMD_AUTO_DETECT`
2. Polls snapshot for progress (shows sub-state)
3. Reads back measured Rs, Ls, λ params via `GSP_CMD_GET_PARAM`
4. Displays results and offers to save to EEPROM

### Other FOC Changes (`foc_v2_control.c`)

- Improved startup sequence timing
- Better voltage limiting during detection
- Guard against PI windup during measurement phases

### Motor Profile Update

Switched default profile from short Hurst DMB0224C10002 to long Hurst DMB2424B10002:
- DMB2424B: Rs=534mΩ, Ls=471µH (auto-detected values)
- DMB0224C: Rs=2.54Ω, Ls=2.3mH (old manual values)

---

## 5. Physical PWM/DShot Testing Guide

### Hardware Setup

**PWM Source Options:**
- RC transmitter + receiver (bind first, channel 3 = throttle typical)
- Servo tester / PWM signal generator
- Arduino: `Servo.writeMicroseconds(1000-2000)` on a GPIO

**DShot Source Options:**
- Flight controller (Betaflight/ArduPilot) with DShot output
- Arduino + [DShot library](https://github.com/gueei/DShot-Arduino)
- STM32 timer DMA DShot generator

**Wiring:**
- Signal wire → **RD8** (RP57, SCCP4 input capture pin)
- Ground → shared ground with signal source
- Do NOT connect 5V/VCC from receiver to board (separate power domains)

### Step-by-Step Physical Test Procedure

#### A. PWM Test

1. **Connect** PWM signal source to RD8 + GND
2. **Set transmitter/generator to 1000us** (zero throttle) before powering ESC
3. **Power ESC** — should show IDLE in GUI snapshot
4. **Monitor via GUI or Python:**
   ```bash
   # Quick status check
   python3 tools/gsp_rx_test.py  # runs automated tests (will fail T1+ since no inject needed)
   ```
   Or use the GUI Dashboard — RX status shows in telemetry.

5. **Verify detection:**
   - Link state should go: DETECTING → LOCKING → LOCKED
   - Protocol should show PWM
   - `rxPulseUs` should read ~1000

6. **Verify arming:**
   - With zero throttle (1000us), ESC should auto-arm (state = ARMED)
   - Slowly increase throttle — motor should spin

7. **Verify timeout:**
   - Disconnect signal wire while motor is running
   - Within 200ms: state → FAULT, faultCode = FAULT_RX_LOSS (9)
   - Motor should stop

#### B. DShot Test

1. **Connect** DShot signal source to RD8 + GND
2. **Send DShot value 0** (disarm) before powering ESC
3. **Power ESC**
4. **Verify detection:**
   - Protocol should show DSHOT150/300/600 (auto-detected from bit timing)
   - `rxDshotRate` = 1/3/6
5. **Send throttle values 48-2047** — motor should spin proportionally
6. **Send value 0** — motor should stop (DShot disarm)

#### C. Auto-Detect Test (PWM→DShot switch)

With `FEATURE_RX_AUTO=1`:
1. Start with PWM, verify LOCKED
2. Disconnect PWM — should go to LOST after 200ms
3. Clear fault, wait for IDLE
4. Connect DShot source — should auto-detect new protocol
5. Verify protocol changed in RX status

### What to Watch For

- **Signal integrity:** Use short wires (< 30cm). Long wires + DShot = bit errors
- **Ground loop:** Shared ground between signal source and ESC is critical
- **DShot CRC errors:** Check `rxCrcErrors` in RX status. Some CRC failures are normal during alignment, but continuous failures indicate signal integrity issues
- **PWM pulse width:** Valid range is 900-2100us. Outside this range, frames are marked invalid
- **Timing:** DShot detection examines edge-to-edge delta:
  - < 200 counts (< 2us @ 100MHz) → DShot600
  - 200-400 counts → DShot300
  - 400-500 counts → DShot150
  - \> 50000 counts (> 500us) → PWM

---

## 6. Debugging

### GSP Commands for RX Debugging

| Command | ID | Description |
|---------|-----|-------------|
| `GET_RX_STATUS` | 0x26 | 12-byte response: linkState, protocol, dshotRate, throttle, pulseUs, crcErrors, droppedFrames |
| `GET_SNAPSHOT` | 0x02 | Full ESC state including throttle, state, faultCode |
| `RX_INJECT` | 0x27 | Test injection (disables hardware capture) |

### Using the GUI

The GUI Dashboard shows:
- ESC state (IDLE/ARMED/ALIGN/OL_RAMP/CL/FAULT)
- Throttle value (0-4095 ADC)
- Duty cycle (%)
- FOC telemetry: Iq, Id, speed (RPM), Vbus

### Common Issues & Solutions

| Problem | Likely Cause | Solution |
|---------|-------------|----------|
| Link stuck in DETECTING | No signal on RD8 / wrong pin | Check wiring, verify RD8 connectivity |
| DETECTING but never LOCKING | Signal detected but invalid frames | Check CRC errors, signal integrity |
| LOCKING but never LOCKED | Intermittent invalid frames resetting lock count | Improve signal quality, check for noise |
| LOCKED but no arming | Throttle not at zero | Ensure 1000us (PWM) or 0 (DShot) |
| FAULT_RX_LOSS while signal present | Main loop too slow, frames timing out | Check for blocking code in main loop |
| DShot CRC errors climbing | Signal integrity / timing mismatch | Shorter wires, check DShot rate vs wire length |
| Inject tests fail with UNKNOWN_CMD | RX features disabled in config | Set FEATURE_RX_PWM/DSHOT/AUTO=1, rebuild |
| Motor doesn't spin in motor test | Not armed / FOC startup failed | Check ESC state, press SW1 if needed |

### Debug Logging via Python

```python
import serial, struct

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Read RX status
from tools.gsp_rx_test import send_cmd, get_rx_status, get_snapshot
st = get_rx_status(ser)
print(f"Link: {st['linkState']}, Proto: {st['protocol']}, "
      f"Throttle: {st['throttle']}, CRC errors: {st['crcErrors']}")
```

### MDB Debugger (MPLAB X)

For low-level debugging with the hardware debugger:
```
# In MDB CLI
program dspic33AKESC.X/dist/default/production/dspic33AKESC.X.production.elf
run
# Set breakpoint in RX_Service() or IC4 ISR
```

Key variables to watch:
- `rxMailbox.seqNum` — should increment by 2 per frame
- `rxMailbox.value` — raw decoded value
- `rxCachedThrottleAdc` — converted throttle (0-4095)
- `rxCachedLocked` — 1 when link is LOCKED
- `garudaData.rxLinkState` — current FSM state
- `garudaData.rxCrcErrors` — cumulative CRC failures
- `garudaData.rxDroppedFrames` — missed frames (seqNum gaps)

### Build Configurations

Three build configurations verified:
1. **RX=0** (all features off) — baseline, no RX code compiled
2. **RX_PWM=1** — PWM only, DShot code excluded
3. **RX_PWM+DSHOT=1** — full RX support (current default)

To change, edit `garuda_config.h` and rebuild in MPLAB X.

---

## 7. File Inventory

### Firmware (dspic33AKESC/)

| File | Lines | Role |
|------|-------|------|
| `input/rx_decode.c` | 402 | RX service FSM, throttle conversion, lock state machine |
| `input/rx_decode.h` | 63 | Mailbox struct, externs, API |
| `hal/hal_input_capture.c` | ~350 | SCCP4 IC driver, PWM pulse measurement, DShot DMA |
| `hal/hal_input_capture.h` | ~80 | HAL API |
| `gsp/gsp_commands.c` | +70 lines | RX_INJECT handler, RX_STATUS handler |
| `gsp/gsp_commands.h` | +5 lines | Command IDs |
| `foc/foc_v2_detect.c` | ~550 | Auto-commissioning (Rs, Ls, lambda, PI tune) |
| `foc/foc_v2_detect.h` | ~80 | Detect state machine types + API |
| `garuda_config.h` | modified | RX feature flags enabled |

### Tools (tools/)

| File | Lines | Role |
|------|-------|------|
| `gsp_rx_test.py` | 987 | Automated RX tests + interactive motor test |
| `gsp_detect.py` | 285 | Auto-commissioning CLI monitor |

### GUI (gui/src/)

| File | Role |
|------|------|
| `components/MotorTuningPanel.tsx` | Auto-detect button, live progress, param readback |
| `components/ConnectionBar.tsx` | RX status display |
| `components/ProfileSelector.tsx` | Motor profile selection |
| `protocol/gsp.ts` | GSP_CMD_AUTO_DETECT added |
| `protocol/types.ts` | ESC_DETECT state, feature flag |
