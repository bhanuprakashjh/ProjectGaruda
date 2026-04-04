# Zero-Crossing Detection Architecture V3
## dsPIC33CK64MP205 + ATA6847 — Garuda 6-Step BLDC ESC

### Document Version
- Date: 2026-04-04
- Commit: 5286b2e
- Authors: Bhanu Prakash, Claude Opus 4.6

---

## 1. System Overview

### Hardware
- **MCU**: dsPIC33CK64MP205 (100 MIPS, 16-bit DSP)
- **Gate Driver**: ATA6847 QFN40 (integrated 3-phase driver + BEMF comparators)
- **Board**: EV43F54A evaluation board
- **PWM**: 24 kHz center-aligned complementary, 3 generators (PG1/2/3)
- **BEMF Comparator**: ATA6847 internal, digital outputs on RC6/RC7/RD10

### Motors Tested
| Motor | KV | Poles | Voltage | Max eRPM Achieved |
|-------|-----|-------|---------|-------------------|
| A2212 1400KV | 1400 | 14 (7PP) | 12V | 102k |
| 2810 1350KV | 1350 | 14 (7PP) | 18-24V | 103k |

### Architecture Diagram
```
ATA6847 BEMF Comparators (digital output)
    ├── RC6 (Phase A) ─── CLCINA ──→ CLC1 D-FF ──→ LCOUT (clean)
    ├── RC7 (Phase B) ─── CLCINB ──→ CLC2 D-FF ──→ LCOUT (clean)
    └── RD10 (Phase C) ── CLCINC ──→ CLC3 D-FF ──→ LCOUT (clean)
                                          ↑ CLK
                                     PWM Event A
                                     (mid-ON sample)

    ├── RC6 ── PPS ──→ SCCP2 ICM2 (edge capture, 640ns timestamp)
    ├── RC7 ── PPS ──→ (routed per step)
    └── RD10 ─ PPS ──→

SCCP4: Free-running timer (640ns/tick) + Output Compare (commutation)
SCCP1: 210.5 kHz periodic timer (ZC poll ISR)
Timer1: 20 kHz system tick (duty control, timeout, state machine)
```

---

## 2. Detection Paths

### 2.1 CLC D-Flip-Flop (Signal Conditioning)
- **Purpose**: Eliminate PWM switching noise from BEMF comparator output
- **Mode**: D-FF (CLC MODE=100), one per phase (CLC1/2/3)
- **Clock**: PWM Event A (PG1TRIGA=0, fires at center of PWM ON-time)
- **Operation**: Samples comparator at mid-ON (clean moment), holds value until next clock
- **Output**: Stable, noise-free BEMF state — only updates once per PWM cycle

**Why it works**: ATA6847 comparator output has switching noise at every PWM edge (~2-6µs ringing at 24V). The D-FF only captures the state during mid-ON when current is flowing and comparator is settled. Between clock edges, all noise is invisible.

**Limitation**: One observation per PWM cycle. At 24kHz = 42µs period. At 100k eRPM (100µs step) = 2.4 observations per step. The ZC can happen between observations → up to 42µs detection delay.

### 2.2 SCCP2 Input Capture (Precision Timestamp)
- **Purpose**: Capture the exact ZC edge time with 640ns precision
- **Mode**: IC capture, single-polarity edge (ATA6847 inverted comparator)
- **Clock**: Fp/64 = 1.5625 MHz (same as SCCP4 for backdating)
- **PPS**: Dynamically routed to floating phase comparator pin at each commutation

**Operation**:
1. Armed after blanking expires in FastPoll
2. Captures first edge matching configured polarity
3. ISR latches backdated SCCP4 timestamp into `zcCandidateHR`
4. Does NOT call RecordZcTiming (prevents estimator corruption from noise edges)
5. Sets `icArmed = false` to signal poll path

**Edge polarity** (ATA6847 inverted — HIGH when BEMF < neutral):
- Rising ZC (BEMF goes up) → comparator falls → capture falling edge (MOD=0010)
- Falling ZC → capture rising edge (MOD=0001)

**50% interval rejection**: IC ISR rejects edges arriving before 50% of refIntervalHR from last ZC. Prevents noise edges from providing wrong timestamps.

### 2.3 Poll Path (Detection + Validation)
- **Frequency**: 210.5 kHz (SCCP1 timer, non-integer ratio with 24kHz PWM)
- **Signal source**: CLC D-FF output (clean) when FEATURE_CLC_BLANKING=1
- **Deglitch**: Mode-dependent:
  - ACQUIRE (CL entry): full filterLevel (3-4 consecutive reads)
  - TRACK (stable): 1 read (CLC signal is stable, no deglitch needed)

**Operation**:
1. Blanking check (12% of step period from commutation, floor 25µs)
2. 50% interval rejection (elapsed since last ZC > half of refIntervalHR)
3. Read CLC output for floating phase
4. If matches expected state: increment pollFilter
5. If pollFilter >= required level: ZC confirmed
6. Call RecordZcTiming with IC's precise timestamp (if IC captured) or poll timestamp
7. Call ScheduleCommutation

### 2.4 Combined Flow
```
Commutation fires (SCCP4 OC ISR)
    ↓
BEMF_ZC_OnCommutation:
    → Configure SCCP2 IC (PPS route, edge polarity)
    → ForcePreZcState (CLC D-FF reset to pre-ZC state)
    → Set blanking end time
    → phase = IC_ZC_BLANKING
    ↓
FastPoll (210.5 kHz):
    → Check blanking: if expired → phase = IC_ZC_ARMED, arm IC
    → 50% interval rejection
    → Read CLC output (clean)
    → Deglitch filter (ACQUIRE: 3-4 reads, TRACK: 1 read)
    → If confirmed: RecordZcTiming + ScheduleCommutation
    ↓
IC Capture (_CCP2Interrupt):
    → Fires on raw comparator edge (before CLC updates)
    → 50% interval check on backdated timestamp
    → Stores precise zcCandidateHR (640ns)
    → Does NOT call RecordZcTiming
    → Poll path uses IC timestamp when it confirms
    ↓
SCCP4 OC fires at computed target → Commutation
```

---

## 3. Timing Advance

### AM32-Style Fixed Fraction
```c
advance = (interval / 8) * TIMING_ADVANCE_LEVEL
waitTime = interval / 2 - advance
```

- **TIMING_ADVANCE_LEVEL = 2** (15° at all speeds)
- Scales automatically with speed — no eRPM-based ramp
- Previous linear ramp (0-30°) caused current waste at mid-speed

**Why fixed fraction works**: The advance as a fraction of the commutation interval is speed-independent. 15° advance = 25% of the half-interval. This compensates for detection delay proportionally at any speed.

---

## 4. PWM Modes

### Complementary (PWM_DRIVE_UNIPOLAR=0, default)
- H and L FETs alternate with dead time
- Active braking during OFF-time → linear duty→speed relationship
- 2 switching edges per cycle → more comparator noise
- CLC D-FF required for clean ZC

### Unipolar H-PWM/L-OFF (PWM_DRIVE_UNIPOLAR=1)
- Only H-side switches, L-side forced OFF
- No braking → motor free-wheels, duty controls power not speed
- 1 switching edge per cycle → **34x fewer ZC timeouts** vs complementary
- Needs different duty calibration (lower idle, speed PID for regulation)

### Mode-Dependent Parameters
| Parameter | Complementary | Unipolar |
|-----------|--------------|----------|
| MAX_DUTY | LOOPTIME_TCY - 200 | LOOPTIME_TCY / 2 |
| MIN_DUTY | 200 | 30 |
| CL_IDLE_DUTY | 10% | 6% |
| DUTY_RAMP_ERPM | 60000 | 60000 |

---

## 5. Anti-Aliasing

### Non-Integer Poll Frequency
- Poll at **210.5 kHz** (not 200 kHz)
- PWM at 24 kHz: ratio = 8.77 (non-integer)
- Polls drift across PWM cycle → no systematic edge hits
- Eliminates speed-band-specific aliasing at 60k/80k eRPM

### ATA6847 Edge Blanking (EGBLT)
- EGBLT = 15 (3.75µs maximum) — suppresses comparator output during switching edges
- At 24V, ringing extends to 5-6µs → EGBLT only covers first 3.75µs
- CLC D-FF samples at mid-ON (5-7µs from edge) → catches remaining ringing tail at high voltage

---

## 6. Estimator

### Protected Reference Interval (refIntervalHR)
- IIR-filtered step period with independent shrink/growth clamps
- Shrink: max 25% per update (prevents false short intervals from tightening)
- Growth: max 50% per update (allows deceleration tracking)
- Seeded from OL ramp stepPeriodHR at CL entry

### Two-Step Averaging
- `stepPeriodHR = (3 × old + (interval + prevInterval) / 2) >> 2`
- Smooths step-to-step jitter while tracking speed changes

### Checkpoint Desync Detection
- Every 6 steps (one revolution): compare stepPeriodHR to checkpoint
- If changed by >40% → desync detected → RECOVER mode

---

## 7. Mode State Machine (ZC V2)

### ZC_MODE_ACQUIRE
- Entered at CL start and after recovery
- Full deglitch filter (3-4 reads for CLC, filterLevel for raw GPIO)
- IC stores timestamp but doesn't schedule
- Duty capped at ramp exit duty
- Transitions to TRACK after 20 consecutive good ZCs

### ZC_MODE_TRACK
- Normal operation
- 1-read CLC confirm (clean signal, fast validation)
- IC timestamp used for precise scheduling
- Full duty range available
- Transitions to RECOVER on timeout

### ZC_MODE_RECOVER
- Entered on missed ZCs or desync detection
- Expands refIntervalHR (+12.5% per timeout)
- Duty held (no increase)
- Transitions to ACQUIRE after 10 consecutive good ZCs
- Max 5 recovery attempts before desync fault

---

## 8. Test Results Summary

### A2212 1400KV at 12V

| Test | Mode | ZC Success | Max eRPM | Duration | Timeouts |
|------|------|-----------|----------|----------|----------|
| No-load, complementary 20kHz | Poll only | 99.5% | 102k | 49s | 1687 |
| No-load, unipolar 24kHz | IC+Poll | 99.96% | 102k | 119s | 66 |
| No-load, CLC+IC 24kHz comp | CLC+IC hybrid | 99.84% | 82k | 120s | 202 |
| **Prop, CLC+IC 24kHz comp** | **CLC+IC hybrid** | **100%** | **48k** | **70s** | **0** |

### 2810 1350KV at 18V

| Test | Mode | ZC Success | Max eRPM | Duration | Timeouts |
|------|------|-----------|----------|----------|----------|
| No-load, CLC+IC 24kHz comp | CLC+IC hybrid | 99.4% | 103k | 35s | 712 |
| **Prop, CLC+IC 24kHz comp** | **CLC+IC hybrid** | **100%** | **56k** | **25s** | **0** |

### 2810 1350KV at 24V

| Test | Mode | ZC Success | Max eRPM | Max Current | Timeouts |
|------|------|-----------|----------|-------------|----------|
| No-load | CLC+IC hybrid | 99.4% | 103k | 34A | 1224 |
| Prop | CLC+IC hybrid | 99.84% | 62k | 25A | 178 |

### Key Findings
1. **With prop: 100% ZC** on both motors (12V and 18V)
2. **No-load noise at 50%/80% duty**: CLC clock position issue (counter-relative, not edge-relative)
3. **24V Vbus swings**: bench supply sags 20-28V under 25A → comparator threshold shift
4. **Unipolar gives 34x fewer timeouts** but needs speed PID for proper control
5. **CLC+IC hybrid** combines complementary braking with clean ZC detection

---

## 9. Known Issues

### 9.1 CLC Clock Position (Duty-Dependent Noise)
- PG1TRIGA=0 fires at a fixed PWM counter position
- At 50% duty: clock lands near the switching edge → CLC samples noise
- At 70-80% duty: similar alignment issue
- **Does not affect prop operation** (duty stays below problem zone)
- **Fix**: PTG edge-relative sampling (see Section 10)

### 9.2 24V Vbus Swings
- Bench supply internal resistance causes 20-28V swings under 25A load
- Shifts ATA6847 BEMF comparator threshold → false/missed ZCs
- **Fix**: Use LiPo battery (20mΩ vs 300mΩ bench supply)

### 9.3 Fast Pot Transients
- Rapid pot changes can outrun the estimator
- 1-read CLC confirm accepts stale state during transients
- ACQUIRE mode full deglitch prevents this at CL entry
- **Fix**: Duty slew rate limiting (DUTY_SLEW_UP=3 already active)

### 9.4 No Speed PID
- Direct pot→duty mapping (no closed-loop speed control)
- Unipolar mode especially needs speed PID (no braking = speed depends on load)
- **Fix**: AM32-style per-ZC interval-based PID (future implementation)

---

## 10. PTG Improvement Plan

### Problem
CLC D-FF clock (PWM Event A) is **counter-relative** — its distance from the switching edge changes with duty. At certain duties, the clock samples during ringing.

### Solution
PTG provides **edge-relative** timing — sample always N µs after the switching edge, regardless of duty.

### Architecture
```
PWM switching edge
    ↓
PTG detects edge (via PWM trigger input)
    ↓
PTG Timer0 delays 6-7µs (past all ringing)
    ↓
PTG fires ISR (PTG0IF)
    ↓
ISR reads raw BEMF comparator (guaranteed clean)
    ↓
If matches expected → ZC confirmed, use IC timestamp
```

### Implementation Constraints (from codex review)
1. Use **PWM2 or PWM3** Trigger 2 for PTG cadence (PWM1 Trigger2 = ADC)
2. **Single detector front end** (all raw GPIO, no CLC/GPIO mixing)
3. **Redesign confirmation** for one-sample-per-PWM operation
4. Scope as **supplementary** to poll, not global replacement
5. One observation per PWM cycle = 2.4 per step at 100k eRPM at 24kHz

### Expected Benefit
- Eliminates duty-dependent noise (50%/80% problem)
- Clean reads at any voltage (edge-relative delay covers ringing)
- Maintains IC timestamp precision (640ns)
- Compatible with complementary PWM (proper braking)

### PTG Registers (dsPIC33CK64MP205)
- PTGI0: PWM1 ADC Trigger 2 (cadence source)
- PTGT0LIM: Timer 0 limit (blanking/delay)
- PTGQUE0-15: Step queue (3 steps: wait-trigger, wait-timer, fire-IRQ)
- PTG0IF: Interrupt to CPU

---

## 11. File Map

| File | Purpose |
|------|---------|
| `garuda_config.h` | All configuration (profiles, features, tuning) |
| `garuda_types.h` | Data structures (GARUDA_DATA_T, ZC states) |
| `garuda_service.c` | State machine, ISRs (Timer1, ADC, IC, commutation) |
| `motor/bemf_zc.c` | ZC detection (FastPoll, OnCommutation, RecordZcTiming) |
| `motor/bemf_zc.h` | ZC API |
| `hal/hal_ic.c` | SCCP1 poll timer + SCCP2 IC capture |
| `hal/hal_com_timer.c` | SCCP4 free-running timer + output compare |
| `hal/hal_clc.c` | CLC D-FF noise filter (3 per phase) |
| `hal/hal_pwm.c` | PWM init + 6-step commutation + unipolar/comp mode |
| `hal/hal_ata6847.c` | ATA6847 SPI register configuration |
| `gsp/gsp_snapshot.c` | Telemetry snapshot capture |
| `tools/pot_capture.py` | Bench test data capture tool |

---

## 12. Configuration Quick Reference

```c
// Feature flags
FEATURE_IC_ZC           1    // SCCP1 fast poll + SCCP4 HR timer
FEATURE_IC_ZC_CAPTURE   1    // SCCP2 IC capture for precise timestamps
FEATURE_CLC_BLANKING    1    // CLC D-FF noise filter
PWM_DRIVE_UNIPOLAR      0    // 0=complementary (default), 1=unipolar

// PWM
PWMFREQUENCY_HZ         24000   // 24 kHz (AM32 default)
ZC_POLL_FREQ_HZ         210526  // Non-integer ratio with PWM

// Timing advance
TIMING_ADVANCE_LEVEL    2       // 15° (AM32 default)

// ATA6847
EGBLT                   15      // 3.75µs edge blanking (maximum)
SCTHSEL                 7       // 2000mV VDS threshold (for 24V)

// Deglitch
ZC_DEGLITCH_MIN         3       // Min consecutive reads (high speed)
ZC_DEGLITCH_MAX         8       // Max consecutive reads (low speed)
// CLC mode: ACQUIRE=filterLevel, TRACK=1 read
```
