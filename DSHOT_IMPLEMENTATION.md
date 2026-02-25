# DShot & PWM Input Implementation — Project Garuda Phase H

## Overview

Project Garuda is a drone ESC firmware for the **dsPIC33AK128MC106** microcontroller, implementing 6-step trapezoidal BLDC commutation. Phase H adds **flight controller (FC) input support** — both traditional **RC PWM** (1000-2000us analog) and **DShot** (150/300/600 digital) protocols on a single input pin.

**Target MCU**: dsPIC33AK128MC106 — 200 MHz CPU, 100 MHz peripheral bus, 32-bit with hardware FPU

**Target Board**: MCLV-48V-300W (development), custom drone ESC (production)

---

## Implementation Status at a Glance

### Implemented (Phase H — Complete)

| Feature | Status | Details |
|---------|--------|---------|
| DShot150/300/600 frame reception | Done | DMA ping-pong capture, 64-edge ring buffer |
| DShot CRC-4 validation | Done | XOR nibble check, rolling alignment recovery |
| DShot throttle decode (48-2047) | Done | 11-bit extraction, mapped to 0-4095 ADC scale |
| DShot rate auto-detection | Done | Min-delta edge classification (150/300/600) |
| RC PWM capture (1000-2000 us) | Done | ISR rise/fall pairing, seqlock mailbox |
| Auto-detect (DShot vs PWM) | Done | Single-pin protocol identification |
| Signal loss fail-safe | Done | 200 ms timeout → zero throttle + FAULT_RX_LOSS |
| Lock FSM (detect → lock → lost) | Done | 10 consecutive valid frames to lock |
| Seqlock mailbox (ISR → main) | Done | Tear-free, lock-free communication |
| Write-ordered ADC ISR mux | Done | Safe cached volatile consumption |
| Throttle source transition matrix | Done | All transitions IDLE-only, feature-gated |
| GSP telemetry (GET_RX_STATUS) | Done | Link state, CRC errors, dropped frames |
| WebSerial GUI integration | Done | Source selector, RX status panel |
| CLC hardware deglitch filter | Deferred | Planned if EMI noise causes issues on-target |

### Not Yet Implemented (Future Phases)

| Feature | Priority | Description |
|---------|----------|-------------|
| **Bidirectional DShot (EDT)** | High | ESC responds with eRPM on same wire when FC requests telemetry. Required for Betaflight/INAV RPM filtering. Requires pin direction flip + bit-bang response in inter-frame gap. |
| **DShot commands (1-47)** | Medium | Special throttle values for motor direction, 3D mode, beep, save settings, LED control, ESC info request. Currently all ignored (treated as zero throttle). |
| **KISS/serial telemetry** | Medium | Separate UART wire sending voltage, current, temperature, eRPM back to FC. Standard on BLHeli_32/AM32 ESCs. Would need a second UART or time-share with GSP. |
| **DShot extended telemetry (EDT v2)** | Low | Enhanced bidirectional protocol carrying voltage, current, temperature, status in addition to eRPM. Newer Betaflight feature. |

### What This Means for Flight

The current implementation is **sufficient for basic flight controller throttle control** — an FC running Betaflight/INAV can arm and fly a drone using DShot600 with our ESC. The throttle path from FC to motor is complete with full fail-safe protection.

However, for **full FC integration** comparable to BLHeli_32 or AM32:
- **Bidirectional DShot** is the critical missing piece — without it, the FC cannot read motor RPM for its RPM filter, which significantly impacts flight smoothness and noise rejection
- **DShot commands** are needed for features like motor direction change, beep-on-lost-model, and 3D (reversible) mode
- **KISS telemetry** provides the FC with battery voltage, current draw, and ESC temperature for OSD display and battery monitoring

---

## The Problem

Prior to Phase H, the ESC only accepted throttle from two sources:
1. **ADC potentiometer** — bench-only, not usable on a drone
2. **GSP (Garuda Serial Protocol)** — our custom UART protocol for tuning

Real drone operation requires the ESC to accept throttle commands from a flight controller via industry-standard protocols: **RC PWM** or **DShot**.

### Design Constraints
- Both protocols must share a **single input pin** (RD8/RP57)
- **Auto-detection** must identify which protocol the FC is sending
- **Signal loss** must immediately zero throttle and latch a fault
- ISR budget is tight: commutation runs at priority 7, ADC at priority 6
- DShot600 generates **~1.2 million edges/second** — too fast for per-edge ISRs

---

## Protocol Primer

### RC PWM
| Parameter | Value |
|-----------|-------|
| Signal type | Analog pulse width |
| Pulse range | 1000 us (zero) to 2000 us (full) |
| Frame rate | 50-400 Hz (2.5-20 ms period) |
| Resolution | ~1000 steps |
| Edge rate | ~100-800 edges/sec |

A standard servo pulse: high time encodes throttle, low time is dead space.

### DShot (Digital Shot)
| Parameter | DShot150 | DShot300 | DShot600 |
|-----------|----------|----------|----------|
| Bit period | 6.667 us | 3.333 us | 1.667 us |
| Bit-1 duty | ~75% high | ~75% high | ~75% high |
| Bit-0 duty | ~37% high | ~37% high | ~37% high |
| Frame rate | ~8 kHz | ~18 kHz | ~37 kHz |
| Edge rate | ~256K/s | ~576K/s | ~1.18M/s |

Each DShot frame is 16 bits:
```
[11-bit throttle][1-bit telemetry request][4-bit CRC]
```
- **Throttle 0** = disarm
- **Throttle 1-47** = special commands (ignored by ESC)
- **Throttle 48-2047** = actual throttle (2000 steps)
- **CRC** = XOR of nibbles: `(val ^ (val >> 4) ^ (val >> 8)) & 0x0F`

Bit encoding uses duty cycle, not voltage level:
```
  Bit-1 (75% duty):     ████████████░░░░
  Bit-0 (37% duty):     ██████░░░░░░░░░░
                         |-- period ---|
```

---

## Hardware Resources

### Input Pin
```
RD8 (RP57) ─── PPS ICM4R=57 ─── SCCP4 Input Capture
```

### SCCP4 Configuration
| Register | Setting | Purpose |
|----------|---------|---------|
| CCP4CON1.CCSEL | 1 | Input Capture mode |
| CCP4CON1.T32 | 1 | 32-bit timer |
| CCP4CON1.MOD | 0b0011 | Capture every edge (rise + fall) |
| CCP4CON1.CLKSEL | 0b000 | SCCP bus clock = 100 MHz |

**Timer resolution**: 10 ns per count. 32-bit timer wraps every ~42.9 seconds.

| Measurement | Timer Counts |
|-------------|-------------|
| 1000 us PWM pulse | 100,000 |
| 2000 us PWM pulse | 200,000 |
| DShot600 bit period | ~167 |
| DShot300 bit period | ~333 |
| DShot150 bit period | ~667 |

### DMA Channel 0 (DShot only)
- **Source**: CCP4BUF (IC4 capture register)
- **Destination**: Ping-pong double buffers (2 x 32 x 4 bytes)
- **Count**: 32 transfers per block (one DShot frame = 16 bits x 2 edges)
- **ISR**: `_DMA0Interrupt` at priority 4

---

## System Architecture

```
  Flight Controller
       │
       │  PWM pulse (1000-2000us) OR DShot frame (16-bit digital)
       │
       ▼
  RD8 (RP57) ── PPS ── SCCP4 Input Capture (every rise/fall, 32-bit @ 100 MHz)
                              │
              ┌───────────────┼────────────────┐
              │               │                │
        [DETECTING]     [PWM MODE]       [DSHOT MODE]
        ISR per-edge    ISR per-edge     Ping-pong DMA
        (short burst    (rise/fall       DMACNT=32
         for classify)   pairing)        per block
              │               │                │
              │          Seqlock             DMA-TC ISR
              │          mailbox             O(1) decode
              │          write               + mailbox
              │               │                │
              └───────────────┼────────────────┘
                              │
                     RX_Service() [main loop]
                     ├── Seqlock mailbox read
                     ├── Protocol-specific decode
                     ├── Lock FSM management
                     ├── DShot alignment search
                     └── Write cached throttle
                              │
                              ▼
                    ADC ISR Throttle Mux (24 kHz)
                    ├── Read rxCachedLocked
                    ├── Read rxCachedThrottleAdc
                    └── Route to motor control
```

---

## Data Flow Pipeline

### Stage 1: Edge Capture (Hardware + ISR/DMA)

**PWM Mode** — ISR-based rise/fall pairing:
```
Rising Edge → save timestamp
Falling Edge → width = fall - rise
  Validate: 950 us <= width <= 2050 us
  Write to seqlock mailbox (pulse width in us)
```

**DShot Mode** — DMA-based bulk capture:
```
SCCP4 captures edge → DMA transfers timestamp to buffer
  32 transfers = 1 complete DShot frame
  DMA-TC interrupt fires → copy to 64-edge ring buffer
  O(1) decode attempt at current alignment offset
  Write throttle value to seqlock mailbox
```

### Stage 2: Seqlock Mailbox (ISR-to-Main Communication)

The mailbox uses a **seqlock** pattern for tear-free communication between ISR (producer) and main loop (consumer):

```c
typedef struct {
    volatile uint16_t seqNum;  // odd = writing, even = complete
    uint16_t value;            // pulse width (PWM) or throttle (DShot)
    uint8_t  valid;            // 1 = CRC ok / width in range
} RX_MAILBOX_T;
```

**Producer (ISR/DMA-TC)**:
```
seqNum++          // now odd — "I'm writing"
value = data
valid = flag
seqNum++          // now even — "write complete"
```

**Consumer (RX_Service)**:
```
seq1 = seqNum
copy value, valid
seq2 = seqNum
if (seq1 != seq2 || seq1 is odd):
    skip — torn read
else:
    use data — guaranteed consistent
```

**Dropped frame detection**: each update increments seqNum by 2. Gap > 2 between reads = dropped frames.

### Stage 3: RX_Service (Main Loop Decode)

Runs in `main.c while(1)` alongside GSP_Service() and BoardService():
- Reads seqlock mailbox
- PWM: converts pulse width to 12-bit ADC-scale throttle (0-4095)
- DShot: throttle already decoded by DMA-TC ISR
- Manages lock FSM state transitions
- Performs DShot alignment search (deferred from ISR)
- Writes cached throttle values for ADC ISR consumption

### Stage 4: ADC ISR Throttle Mux (24 kHz)

The ADC ISR reads the cached values using **reverse write ordering** for safety:

```c
// RX_Service writes:   value FIRST, lock SECOND  (prevents stale-high)
// ADC ISR reads:        lock FIRST, value SECOND  (matches ordering)

switch (throttleSource) {
    case THROTTLE_SRC_PWM:
    case THROTTLE_SRC_DSHOT:
    case THROTTLE_SRC_AUTO:
        throttle = rxCachedLocked ? rxCachedThrottleAdc : 0;
        break;
    // ... ADC pot, GSP cases ...
    default:
        throttle = 0;  // safe fallback
}
```

---

## DShot Deep Dive

### Why DMA is Mandatory

DShot600 generates **~1.18 million edges per second**. A per-edge ISR at this rate would consume the entire CPU. The solution: **DMA transfers edge timestamps directly from the SCCP4 capture register to memory** with zero CPU involvement.

| Approach | ISR Rate | CPU Load |
|----------|----------|----------|
| Per-edge ISR | 1.18M/s | >100% (impossible) |
| **DMA + TC ISR** | **37K/s** | **< 5%** |

### Ping-Pong Double Buffering

Two DMA buffers alternate to ensure zero edge loss between consecutive frames:

```
Frame N:    DMA writes to Buffer A [32 edges]
            ↓ DMA-TC fires
Frame N+1:  DMA writes to Buffer B [32 edges]
            ↓ Meanwhile, ISR processes Buffer A
Frame N+2:  DMA writes to Buffer A [32 edges]
            ↓ ISR processes Buffer B
            ... and so on
```

Each DMA transfer-complete (TC) ISR:
1. Copies 32 completed edges into a **64-edge ring buffer**
2. Swaps active ping-pong buffer
3. Reloads DMA count register
4. Attempts O(1) decode at the current alignment offset

### 64-Edge Overlap Ring Buffer

DMA delivers disjoint 32-edge chunks. But alignment search needs to examine data across chunk boundaries. The ring buffer provides a sliding 64-edge window:

```
Ring Buffer (64 entries, circular):
┌──────────────────────────────────────────────────────────────────┐
│ ... old edges ... │ DMA chunk N-1 (32 edges) │ DMA chunk N (32) │
└──────────────────────────────────────────────────────────────────┘
                    └──── overlap window ─────┘
                    Alignment search across 16 possible bit offsets
```

### Rolling CRC Frame Alignment

DShot frames are 16 bits = 32 edges. If DMA starts mid-frame, the 32-edge chunk spans two partial frames — CRC will fail. The **rolling CRC alignment** algorithm finds the correct frame boundary:

```
Step 1: DMA-TC ISR attempts decode at current offset
        CRC pass? → aligned, write to mailbox
        CRC fail? → increment failure counter

Step 2: After 5 consecutive CRC failures → flag "pendingShift"

Step 3: RX_Service() (main loop) performs shift search:
        - Try offset+1, offset+2, ... (max 4 per call)
        - At each offset: decode 16 bits from ring, check CRC
        - CRC pass → new alignment found, lock it
        - All 16 offsets exhausted → give up, re-detect protocol

Step 4: Why 16 offsets? 16 bits × 2 edges = 32 edges per frame.
        Each "shift" = 1 bit = 2 edges. So 16 possible alignments.
```

**Key design choice**: alignment search is **deferred to the main loop** (max 4 shifts per call). The DMA-TC ISR stays O(1) — it never searches. This prevents ISR starvation of the commutation and ADC interrupt paths.

### DShot Frame Decode Algorithm

```c
bool DshotDecodeFrame(edges[64], offset, &throttle, &telemetry)
{
    raw = 0;
    baseIdx = offset * 2;  // each bit = 2 edges

    for (bit = 0..15) {
        idx = (baseIdx + bit*2) & 63;      // rise edge
        highTime = edges[idx+1] - edges[idx]; // fall - rise
        period = edges[idx+2] - edges[idx];   // next rise - this rise

        // Duty cycle threshold: >62.5% = bit 1, <50% = bit 0
        if (highTime * 16 > period * 10)
            raw |= (1 << (15 - bit));
    }

    // CRC check: XOR nibbles
    crc = (raw ^ (raw >> 4) ^ (raw >> 8)) & 0x0F;
    if (crc != (raw & 0x0F)) return false;

    throttle = (raw >> 5) & 0x07FF;   // bits 15..5
    telemetry = (raw >> 4) & 0x01;     // bit 4
    return true;
}
```

### DShot Rate Detection

During auto-detection, the minimum edge-to-edge delta classifies the DShot rate:

| Min Delta (counts @ 100 MHz) | Protocol |
|-------------------------------|----------|
| < 200 (~2 us) | DShot600 |
| 200-400 (~2-4 us) | DShot300 |
| 400-500 (~4-5 us) | DShot150 |

---

## CLC Hardware Signal Conditioning (Planned — Milestone E)

### The Problem: EMI Noise on the RX Pin

During motor commutation, the ESC's MOSFET bridge switches high current at 24 kHz. This generates electromagnetic interference (EMI) that couples into nearby signal traces — including the RX input pin (RD8). At DShot600 bit periods of ~1.67 us, even nanosecond-scale glitches can create false edges that corrupt frame alignment or cause spurious CRC failures.

### The Solution: CLC as a Hardware Deglitch Filter

The dsPIC33AK128MC106 has **Configurable Logic Cells (CLC)** — programmable hardware logic blocks that operate at the peripheral clock rate (100 MHz) with zero CPU involvement. CLC1 can be configured as a **D-flip-flop synchronizer** to filter the raw RX signal before it reaches SCCP4.

### Proposed CLC1 Configuration

```
            RD8 (raw signal with possible glitches)
              │
              ▼
  ┌───────────────────────┐
  │       CLC1            │
  │                       │
  │  D-FF Sync Filter     │
  │  ┌─────┐   ┌─────┐   │
  │  │ D Q ├──►│ D Q ├──►├──── Clean signal
  │  │   > │   │   > │   │
  │  └──┬──┘   └──┬──┘   │
  │     │         │       │
  │    CLK       CLK      │
  │   (100 MHz peripheral │
  │    bus clock)          │
  └───────────────────────┘
              │
              ▼
         Spare RPn pin (loopback)
              │
              ▼
         ICM4R PPS ── SCCP4 Input Capture
```

**How it works**:
1. Raw RX signal enters CLC1 as the D input
2. Two cascaded D-flip-flops sample the signal at 100 MHz
3. Glitches shorter than **one clock period (10 ns)** are rejected — they can't propagate through two FF stages
4. The clean output is routed to a spare RPn pin via internal loopback
5. SCCP4's PPS is remapped from the raw pin to the CLC-filtered pin

### Why Two Flip-Flops?

A single D-FF synchronizes the signal to the clock domain but doesn't filter. The second FF creates a **metastability barrier** — if the first FF captures a marginal edge (signal transitioning exactly at the clock edge), the second FF resolves it to a clean 0 or 1 before it reaches SCCP4. This is the standard **2-FF synchronizer** pattern used in all clock domain crossings.

### Filter Characteristics

| Parameter | Value |
|-----------|-------|
| Minimum pulse width passed | 10-20 ns (1-2 clock periods) |
| Latency added | 20 ns (2 clock periods) |
| DShot600 bit period | 1,667 ns |
| Latency as % of bit period | ~1.2% |
| CPU overhead | Zero (pure hardware) |

The 20 ns latency is negligible — it shifts all edge timestamps by a constant offset that cancels out in the duty-cycle ratio calculation.

### Why It's Deferred (Not Implemented Yet)

The current software-only approach handles noise through:
- **PWM**: pulse width validation rejects glitch-induced short/long pulses
- **DShot**: CRC-4 detects corrupted frames; rolling alignment recovers from persistent glitches
- **Lock FSM**: requires 10 consecutive valid frames — sporadic glitches don't cause false lock

CLC becomes necessary only if **real-world testing** reveals:
- Persistent CRC failure rates > 0.1% during motor operation
- False edge captures that cause repeated alignment loss
- EMI-induced noise that software filtering can't handle

This will be evaluated during hardware DShot testing with a real flight controller and motor running under load (Milestone C onward). If needed, CLC1 can be enabled via `FEATURE_RX_CLC` flag with zero impact on the existing signal path.

### Implementation Notes (When Needed)

```c
/* garuda_config.h */
#define FEATURE_RX_CLC  0  /* 1 = enable CLC1 deglitch filter */

/* hal_input_capture.c — in HAL_IC4_Init() */
#if FEATURE_RX_CLC
    /* Configure CLC1 as 2-FF synchronizer */
    CLC1CONL = 0;
    CLC1CONLbits.MODE = 0b010;  /* D-FF mode */
    /* Input: RD8 via data select mux */
    /* Output: routed to spare RPn for PPS loopback */
    /* Remap ICM4R from raw pin to CLC output pin */
    CLC1CONLbits.LCEN = 1;      /* Enable CLC1 */
#endif
```

The `FEATURE_RX_CLC` flag keeps CLC code compiled out by default — zero binary impact when not used.

---

## Link State Machine

The RX link FSM manages the lifecycle from signal detection to loss:

```
                    ┌──────────────┐
                    │  UNLOCKED    │ (power-on state)
                    └──────┬───────┘
                           │ IC4 enabled, start capturing
                           ▼
                    ┌──────────────┐
              ┌────►│  DETECTING   │◄──────────────────────────┐
              │     └──────┬───────┘                            │
              │            │ Protocol identified (min-delta)    │
              │            ▼                                    │
              │     ┌──────────────┐                            │
              │     │   LOCKING    │ Count valid frames         │
              │     └──────┬───────┘                            │
              │            │ N=10 consecutive valid frames      │
              │            ▼                                    │
              │     ┌──────────────┐                            │
              │     │   LOCKED     │ Normal operation           │
              │     └──────┬───────┘                            │
              │            │ No valid frame for 200 ms          │
              │            ▼                                    │
              │     ┌──────────────┐                            │
              │     │    LOST      │ Zero throttle + FAULT      │
              │     └──────┬───────┘                            │
              │            │ CLEAR_FAULT + ESC_IDLE             │
              └────────────┘                                    │
                                                                │
              DShot: 16 offsets exhausted → ─────────────────────┘
```

### Lock Criteria

**PWM**: 10 consecutive pulses with valid width (950-2050 us)

**DShot**: 10 consecutive CRC-valid decoded frames

### Signal Loss Response

When no valid frame is received for **200 ms** (`RX_TIMEOUT_MS`):

1. Set link state to `RX_LINK_LOST`
2. Zero the cached throttle (write ordering: lock=0 first, value=0 second)
3. If motor was running: latch `FAULT_RX_LOSS` and transition to `ESC_FAULT`
4. Motor stops immediately — requires explicit `CLEAR_FAULT` command to restart

This is a **hard safety requirement**: a drone with a lost RC link must not continue flying with stale throttle.

---

## Auto-Detection Strategy

When `THROTTLE_SRC_AUTO` is selected, the system auto-detects between DShot and PWM:

```
1. Start in DETECTING mode (IC4 ISR captures raw edges)
2. Accumulate >= 4 edges in detection buffer
3. RX_Service() calls DetectProtocol():
   - Compute minimum edge-to-edge delta
   - min < 500 counts (< 5 us) → DShot candidate
     - Subclassify rate: <200 = DShot600, <400 = DShot300, else = DShot150
   - min > 50,000 counts (> 500 us) → PWM candidate
   - Neither → keep detecting
4. Once identified:
   - PWM → switch IC4 to rise/fall pairing mode
   - DShot → arm DMA, switch to DMA mode
5. Enter LOCKING state, require N valid frames
```

AUTO means **DShot or PWM only**. The ADC potentiometer is never part of auto-detection — it's a separate, explicit bench-only source.

---

## ISR Priority Scheme

| Priority | ISR | Function | Rate |
|----------|-----|----------|------|
| 7 | AD1CMP5, AD2CMP1, PWMFault | Commutation + HWZC | Per-ZC event |
| 7 | _CCT1Interrupt | HWZC timer | Per-ZC event |
| 6 | AD1CH0 | ADC control loop + throttle mux | 24 kHz |
| 5 | Timer1 | Heartbeat, service tick | 1 kHz |
| **4** | **_CCT4Interrupt** | **PWM edge pair / detection** | **Per-edge** |
| **4** | **_DMA0Interrupt** | **DShot decode + mailbox** | **~37 kHz** |

Key principle: **RX input processing never preempts motor control**. Commutation (7) and ADC (6) always run first. Input capture (4) yields to everything critical.

---

## Throttle Conversion

### PWM: Pulse Width to ADC Scale

```
Input:  1000 us (zero) to 2000 us (full)
Output: 0 to 4095 (12-bit ADC scale)

Deadband: 1000 +/- 25 us = zero throttle
Mapping:  1025 us → 0,  1500 us → ~1024,  2000 us → 4095
Formula:  adc = (pulseUs - 1000) * 4095 / 2000
```

### DShot: Throttle Value to ADC Scale

```
Input:  0-2047 (DShot throttle field)
Output: 0 to 4095 (12-bit ADC scale)

DShot 0 = disarm → 0
DShot 1-47 = commands → 0 (ignored)
DShot 48-2047 = throttle
Mapping:  48 → 0,  1048 → ~2048,  2047 → 4095
Formula:  adc = (dshotVal - 48) * 4095 / 2000
```

Both conversions output the same 0-4095 range, allowing the ADC ISR throttle mux to treat all sources uniformly.

---

## Throttle Source Management

### Source Transition Matrix

All source switches are **IDLE-only** — the ESC must be stopped before changing input source.

| From \ To | ADC | GSP | PWM | DSHOT | AUTO |
|-----------|-----|-----|-----|-------|------|
| Any state (IDLE) | Allow* | Allow | Allow* | Allow* | Allow* |
| Any state (running) | WRONG_STATE | WRONG_STATE | WRONG_STATE | WRONG_STATE | WRONG_STATE |

\* Only if the corresponding feature flag is compiled in. Otherwise: OUT_OF_RANGE.

### Feature Flags

| Flag | Default | Purpose |
|------|---------|---------|
| `FEATURE_ADC_POT` | 1 | ADC potentiometer (bench boards) |
| `FEATURE_RX_PWM` | 0 | RC PWM input |
| `FEATURE_RX_DSHOT` | 0 | DShot digital input |
| `FEATURE_RX_AUTO` | 0 | Auto-detect (requires PWM or DSHOT) |

Compile-time static assertions prevent invalid configurations:
- At least one throttle source must be enabled
- `FEATURE_RX_AUTO` requires `FEATURE_RX_PWM` or `FEATURE_RX_DSHOT`

### Boot Source Selection

On power-up, the firmware automatically selects the default throttle source based on compile-time configuration (priority order): ADC_POT > RX_AUTO > RX_PWM > RX_DSHOT > GSP.

---

## Safety Design

### Fail-Safe Principles

1. **Zero throttle is always safe** — the `default:` case in the ADC ISR mux always returns zero
2. **No pot fallback** — flight boards have a floating ADC pin; reading it would produce random throttle
3. **Write ordering prevents transient states** — the ADC ISR never sees stale-high throttle with locked=true
4. **FAULT_RX_LOSS is latched** — requires explicit CLEAR_FAULT, preventing automatic restart after signal loss
5. **IDLE-only source switching** — prevents mid-flight throttle source changes

### Write Ordering Detail

```
RX_Service (producer):          ADC ISR (consumer):
─────────────────               ──────────────────
rxCachedThrottleAdc = value;    lock = rxCachedLocked;      // read lock FIRST
rxCachedLocked = 1;             if (lock)
                                    thr = rxCachedThrottleAdc; // read value SECOND
```

If interrupted between the two writes:
- Lock is still 0 → ISR uses 0 throttle (safe)
- Value is updated but lock not yet → ISR uses 0 throttle (safe)

On signal loss (reverse ordering):
```
rxCachedLocked = 0;             // unlock FIRST
rxCachedThrottleAdc = 0;        // zero value SECOND
```

If interrupted between:
- Lock is 0 → ISR uses 0 regardless of stale value (safe)

---

## File Structure

```
dspic33AKESC/
├── garuda_config.h          # Feature flags + RX config constants
├── garuda_types.h           # THROTTLE_SOURCE_T, RX_LINK_STATE_T,
│                            # RX_PROTOCOL_T, FAULT_RX_LOSS
├── garuda_service.c         # ADC ISR throttle mux (switch on source)
├── main.c                   # RX_Init() + RX_Service() in while(1)
├── hal/
│   ├── hal_input_capture.h  # HAL API: init, enable, disable, modes, DMA
│   ├── hal_input_capture.c  # SCCP4 IC + _CCT4Interrupt + _DMA0Interrupt
│   │                        # Seqlock mailbox, ping-pong DMA, ring buffer,
│   │                        # DShot decode, C.0 gate test infrastructure
│   └── port_config.c        # Pin config (PPS moved to HAL_IC4_Init)
├── input/
│   ├── rx_decode.h          # RX_MAILBOX_T, cached throttle externs
│   └── rx_decode.c          # RX_Service(): protocol detect, decode,
│                            # lock FSM, alignment search, caching
├── gsp/
│   ├── gsp_commands.h       # GSP_RX_STATUS_T (GET_RX_STATUS 0x26)
│   └── gsp_commands.c       # Feature flags bits 19-22, source handler
└── gui/
    └── src/
        ├── protocol/types.ts   # GspRxStatus type, FAULT_CODES, FEATURE_NAMES
        ├── protocol/decode.ts  # decodeRxStatus() parser
        ├── protocol/gsp.ts     # GET_RX_STATUS command + polling
        └── components/
            ├── ControlPanel.tsx  # Source selector (feature-gated buttons)
            └── StatusPanel.tsx   # RX link status + telemetry display
```

---

## GSP Telemetry Integration

### GET_RX_STATUS (Command 0x26)

12-byte packed response:

| Offset | Type | Field | Description |
|--------|------|-------|-------------|
| 0 | uint8 | linkState | 0=UNLOCKED, 1=DETECTING, 2=LOCKING, 3=LOCKED, 4=LOST |
| 1 | uint8 | protocol | 0=NONE, 1=PWM, 2=DShot150, 3=DShot300, 4=DShot600 |
| 2 | uint8 | dshotRate | 0/1/3/6 (rate indicator) |
| 3 | uint8 | (padding) | — |
| 4 | uint16 | throttle | Current decoded throttle (0-2047 DShot or 0-2000 PWM) |
| 6 | uint16 | pulseUs | PWM pulse width in microseconds (0 for DShot) |
| 8 | uint16 | crcErrors | Cumulative DShot CRC error count |
| 10 | uint16 | droppedFrames | Frames lost (seqlock gap detection) |

### Feature Flag Bits (GET_INFO)

| Bit | Feature |
|-----|---------|
| 19 | ADC_POT |
| 20 | RX_PWM |
| 21 | RX_DSHOT |
| 22 | RX_AUTO |

The WebSerial GUI uses these bits to show/hide throttle source selector buttons.

---

## Verification & Testing

### Compile-Time Verification
- `FEATURE_RX_*=0` (default): identical binary to pre-Phase H — **zero regression** confirmed
- Build tested with: RX=0, RX_PWM=1, RX_PWM+DSHOT=1 — all clean
- Hardware regression test: 24/25 GSP tests pass (pre-existing test issue, not regression)

### Milestone Roadmap

| Milestone | Status | Description |
|-----------|--------|-------------|
| **0: Compile Probe** | Pending | Verify DMA register symbols, count semantics, IFS clear method |
| **A: PWM Capture** | Code Complete | ISR rise/fall + seqlock + lock FSM |
| **B: Throttle Mux** | Code Complete | ADC ISR switch, FAULT_RX_LOSS, source matrix |
| **C.0: DMA Gate Test** | Pending | Logic analyzer validation of ping-pong DMA |
| **C.1-C.3: DShot DMA** | Code Complete | Ping-pong + ring buffer + rolling CRC + seqlock |
| **D: Auto-Detection** | Code Complete | Min-delta classification + mode switching |
| **E: CLC Deglitch** | Deferred | Optional hardware noise filter |
| **F: GSP/GUI Integration** | Code Complete | GET_RX_STATUS + GUI panels + feature flags |

### Hardware Test Plan
1. **PWM**: Signal generator at 50 Hz and 400 Hz, pulse widths 1000/1500/2000 us
2. **DShot**: Flight controller or signal generator outputting DShot600
3. **Signal loss**: Disconnect during motor run → verify zero throttle within 200 ms
4. **C.0 DMA gate test**: 10s continuous DShot600, logic analyzer edge count must match `frames x 32` exactly

---

## Future: Bidirectional DShot & Command Handling

### Bidirectional DShot (EDT) — How It Works

In standard DShot, the FC sends and the ESC only listens. In **bidirectional DShot**, the ESC responds with telemetry data on the **same wire** after each FC frame:

```
  FC sends frame        ESC responds          FC sends next frame
  ───────────┐    ┌─────────────────┐    ┌───────────
  DShot frame │    │  eRPM telemetry │    │  DShot frame
  (16 bits)   │    │  (16 bits, inv) │    │  (16 bits)
  ────────────┘    └─────────────────┘    └───────────
              │◄──►│                 │◄──►│
              30us   ~26.7us frame    30us
              guard    (inverted)     guard
              time                    time
```

**Implementation challenges**:
1. **Pin direction flip**: RD8 must switch from input to output within the ~30 us guard time after receiving a frame. On dsPIC33AK, this means reconfiguring TRIS and disabling IC4 temporarily.
2. **Bit-bang transmission**: The ESC must generate DShot-encoded edges with precise timing (~1.67 us per bit at DShot600). Options: timer-driven GPIO toggle, or use a PWM output channel.
3. **Inverted encoding**: Bidirectional DShot uses inverted signal levels — bit-1 is low-duty, bit-0 is high-duty.
4. **eRPM encoding**: The 16-bit response carries eRPM in a specific format: `(eRPM period in us) / 2`, with its own CRC. The ESC must compute this from commutation timing.
5. **Timing budget**: At DShot600 37 kHz, the entire receive + flip + transmit + flip-back cycle must complete in ~27 us. This is extremely tight.

**Possible dsPIC33AK approaches**:
- Use SCCP4 output-compare mode for the response (hardware-timed edges)
- Use a spare PWM generator to bit-bang the response
- DMA-driven output from a pre-computed edge buffer

### DShot Commands (Throttle Values 1-47)

When the FC sends throttle values 1-47, these are **special commands**, not throttle requests. The command is repeated multiple times by the FC (typically 10 frames) to ensure reception.

| Command | Value | Description |
|---------|-------|-------------|
| DSHOT_CMD_MOTOR_STOP | 0 | Disarm |
| DSHOT_CMD_BEEP1-5 | 1-5 | Beep sequences (lost model finder) |
| DSHOT_CMD_ESC_INFO | 6 | Request ESC info (requires bidir) |
| DSHOT_CMD_SPIN_DIRECTION_1 | 7 | Set rotation CW |
| DSHOT_CMD_SPIN_DIRECTION_2 | 8 | Set rotation CCW |
| DSHOT_CMD_3D_MODE_OFF | 9 | Disable 3D (reversible) mode |
| DSHOT_CMD_3D_MODE_ON | 10 | Enable 3D mode |
| DSHOT_CMD_SETTINGS_REQUEST | 11 | Request settings (requires bidir) |
| DSHOT_CMD_SAVE_SETTINGS | 12 | Save settings to EEPROM |
| DSHOT_CMD_SPIN_DIRECTION_NORMAL | 20 | Normal direction |
| DSHOT_CMD_SPIN_DIRECTION_REVERSED | 21 | Reversed direction |
| DSHOT_CMD_LED0-3_ON/OFF | 22-29 | LED control |
| DSHOT_CMD_AUDIO_STREAM | 30 | Audio stream mode |
| DSHOT_CMD_SILENT_MODE | 31 | Silent mode |
| DSHOT_CMD_EDT_ARM | 35 | Arm EDT (extended telemetry) |
| DSHOT_CMD_EDT_VOLT/TEMP/CURR/DEBUG | 36-43 | EDT channel config |

**Implementation approach**: add a command dispatcher in `rx_decode.c` that counts consecutive identical command frames (require N=6 repetitions), then executes the command. Most commands map to existing Garuda features (direction = `DIRECTION_DEFAULT`, save = EEPROM write, beep = PWM tone generation).

### KISS Telemetry (Separate Wire)

KISS telemetry is a **unidirectional UART stream** from ESC to FC on a dedicated telemetry wire (not the DShot signal wire):

| Byte | Field | Description |
|------|-------|-------------|
| 0 | Temperature | ESC temperature in Celsius |
| 1-2 | Voltage | Battery voltage x 100 |
| 3-4 | Current | Motor current x 100 |
| 5-6 | Consumption | mAh consumed |
| 7-8 | eRPM | Electrical RPM x 100 |
| 9 | CRC8 | XOR checksum |

**10 bytes at 115200 baud**, sent once per DShot frame (~37 kHz at DShot600, though typically throttled to a lower rate).

This would require either a **second UART** or **time-sharing UART1** with GSP. On the MCLV-48V-300W dev board, UART1 is used for GSP/X2CScope. A production ESC board could dedicate a second UART to telemetry.

### Memory Budget Estimate (Future Features)

| Feature | Flash Estimate | RAM Estimate |
|---------|---------------|-------------|
| Current Phase H | 2,696 bytes | 620 bytes |
| Bidirectional DShot | ~1,500-2,000 bytes | ~128 bytes (response buffer) |
| DShot commands | ~800-1,200 bytes | ~32 bytes (command state) |
| KISS telemetry | ~500-800 bytes | ~16 bytes (TX buffer) |
| **Total projected** | **~5,500-6,700 bytes** | **~800 bytes** |

With 71% flash and 88% RAM free, all future features fit comfortably.

---

## Key Design Decisions Summary

| Decision | Rationale |
|----------|-----------|
| Single pin for both protocols | Drone FC typically uses one signal wire per ESC |
| DMA for DShot (not ISR) | 1.2M edges/sec impossible with per-edge ISR |
| Ping-pong double buffering | Zero edge loss between consecutive frames |
| 64-edge ring + rolling CRC | Align without requiring inter-frame gap detection |
| Alignment search in main loop | Keep DMA-TC ISR O(1), prevent commutation starvation |
| Seqlock mailbox | Tear-free ISR-to-main communication, no mutexes |
| Write-ordered cached volatiles | Safe ADC ISR consumption without locks |
| FAULT_RX_LOSS latch | Hard safety: lost link = motor stop, manual restart |
| IDLE-only source switching | Prevent mid-flight input confusion |
| Features default OFF | Zero regression when RX not needed |
