# Zero-Crossing Detection Architecture V3
## dsPIC33CK64MP205 + ATA6847 — Garuda 6-Step BLDC ESC

### Document Version
- Date: 2026-04-09
- Commit: (Phase 2 — detector-agreement acceptance)
- Authors: Bhanu Prakash, Claude Opus 4.6

---

## 0. Visual Guide — How It Works

### 0.1 The Problem: PWM Switching Noise on BEMF Comparator

The ATA6847 has internal BEMF comparators with digital outputs. Every time a PWM FET switches, the comparator output rings for 2-6µs (depending on voltage):

```
                    PWM H-FET gate signal
                    ┌──────────┐          ┌──────────┐
           OFF      │    ON    │   OFF    │    ON    │   OFF
         ──────────┘          └──────────┘          └──────────

                    Real BEMF comparator output (what ATA6847 gives us)
                    ↓ ringing  ↓ clean   ↓ ringing  ↓ clean
         ──╥╨╥╨╥───┤──────────├─╥╨╥╨╥───┤──────────├─╥╨╥╨╥──
           noise   │  VALID   │  noise  │  VALID   │  noise
           2-6µs   │  BEMF    │  2-6µs  │  BEMF    │  2-6µs
                    │  signal  │         │  signal  │
```

At 12V: ringing lasts ~2-3µs (manageable)
At 24V: ringing lasts ~5-6µs (overlaps with detection window)

The 200kHz poll reads the comparator every 5µs — some reads hit noise, some hit clean signal. The deglitch filter (3 consecutive matching reads = 15µs) helps but can't fully solve it at high speed where the step period is only 100-170µs.

### 0.2 The Solution: CLC D-Flip-Flop (Stroboscopic Sampling)

Instead of reading the noisy signal continuously, sample it ONCE per PWM cycle at the cleanest moment (mid-ON-time):

```
   PWM switching edges
   ↓                    ↓                    ↓
   ┌────────────────────┐                    ┌────────────
   │       ON time      │     OFF time       │
   └────────────────────┘                    └────────────
   ↑noise    ↑ CLEAN    ↑noise

   CLC D-FF clock (PWM Event A)
                ↑ (fires HERE at mid-ON)

   Raw BEMF:  ╥╨╥╨──1111111111──╥╨╥╨──0000000──╥╨╥╨──
              noise   clean      noise  clean    noise

   D-FF captures "1" at mid-ON
              ↓
   CLC output: ────────1111111111111111111111111111──────
                        ↑ captured     ↑ held until
                        clean "1"       next clock edge

   Next PWM cycle: D-FF captures "0" (ZC happened!)
              ↓
   CLC output: ────────────────0000000000000000000──────
                               ↑ transition = ZC detected!
```

The D-FF acts like a camera flash — it "photographs" the comparator at one clean moment, then holds that picture until the next flash. All noise between flashes is invisible.

### 0.3 SCCP2 IC Capture: Precise Timestamp

The CLC tells us IF a ZC happened. The IC tells us WHEN:

```
   Real ZC occurs at t=0 (comparator edge)
   ↓
   ├──── IC captures edge ──→ timestamp = t=0 (640ns precision)
   │
   ├──── CLC D-FF hasn't clocked yet (waits for next PWM Event A)
   │
   │     20µs later: CLC clocks ──→ output transitions
   │                                    ↓
   │     21µs later: Poll reads CLC ──→ "yes, expected state!" ──→ CONFIRMED
   │
   └──── RecordZcTiming uses IC timestamp (t=0), NOT poll time (t=21µs)
         ScheduleCommutation targets t=0 + advance_delay
         Commutation fires at precise time despite CLC delay
```

### 0.4 Why IC Alone Doesn't Work

The IC captures EVERY comparator edge — real ZC AND noise:

```
   Noise edge at t=5µs (during ringing)
   ↓
   IC captures ──→ timestamp = t=5µs
   IC stores zcCandidateHR = t=5µs

   If IC calls RecordZcTiming directly:
     → Estimator updated with WRONG interval
     → Commutation scheduled at WRONG time
     → Phantom commutation cascade!

   With CLC validation:
     → CLC hasn't transitioned (noise = temporary, comparator bounced back)
     → Poll reads CLC ──→ still shows pre-ZC state ──→ NOT confirmed
     → IC timestamp ignored, no damage to estimator
```

### 0.5 Complete Detection Flow Chart

```
              ┌─────────────────────┐
              │   COMMUTATION FIRES │
              │   (SCCP4 OC ISR)    │
              └──────────┬──────────┘
                         ↓
              ┌──────────────────────┐
              │  OnCommutation:      │
              │  • Route PPS to      │
              │    floating phase    │
              │  • Force CLC D-FF    │
              │    to pre-ZC state   │
              │  • Set blanking time │
              │  • Arm IC after      │
              │    blanking          │
              └──────────┬───────────┘
                         ↓
              ╔══════════════════════╗
              ║   BLANKING PERIOD    ║
              ║   (12% of step,     ║
              ║    min 25µs)         ║
              ║   Poll: skip reads   ║
              ║   IC: not armed yet  ║
              ╚══════════╤═══════════╝
                         ↓
          ┌──────────────┴──────────────┐
          ↓                             ↓
   ┌──────────────┐            ┌───────────────┐
   │  IC ARMED    │            │  POLL ARMED   │
   │  (SCCP2 IE)  │            │  (210.5kHz)   │
   └──────┬───────┘            └───────┬───────┘
          ↓                            ↓
   ┌──────────────┐            ┌───────────────┐
   │ IC captures  │            │ Poll reads    │
   │ raw edge     │            │ CLC D-FF      │
   │              │            │ output (clean)│
   │ 50% interval │            │               │
   │ check:       │            │ Matches       │
   │ too early? ──┼─YES→ skip  │ expected? ────┼─NO→ reset filter,
   │   ↓ NO       │            │              │     rawCoro,
   │ Store        │            │              │     rawFirstMatch
   │ zcCandidateHR│            │   ↓ YES      │
   │ (640ns       │            │ pollFilter++ │
   │  precision)  │            │ + raw corro- │
   │              │            │   boration   │
   │ icArmed=false│            │              │
   │ icCandidate  │            │ ACQUIRE: ≥FL │
   │  Valid=true  │            │ TRACK: fast  │
   └──────────────┘            │  or fallback │
                               │   ↓ YES      │
                               │ ┌─────────────┤
                               │ │ Timestamp   │
                               │ │ selection:  │
                               │ │ IC→raw→CLC  │
                               │ │  →poll      │
                               │ └──────┬──────┘
                               │        ↓
                               │ RecordZcTiming()
                               │ • Update IIR
                               │ • Update refInterval
                               │ • Mode transitions
                               │        ↓
                               │ ScheduleCommutation()
                               │ • Compute advance
                               │ • Set SCCP4 OC target
                               │ • Enable CCP4IE
                               └───────┬───────┘
                                       ↓
                               ┌───────────────┐
                               │ SCCP4 OC fires│
                               │ at target time│
                               │       ↓       │
                               │ COMMUTATION!  │
                               └───────────────┘
```

### 0.6 Mode State Machine

```
                    Motor Start (CL entry)
                           ↓
                  ┌─────────────────┐
                  │    ACQUIRE      │
                  │ • Full deglitch │
                  │ • Duty capped   │
                  │ • Building      │
                  │   estimator     │
                  └────────┬────────┘
                           │ 20 good ZCs
                           ↓
                  ┌─────────────────┐
           ┌──→   │     TRACK       │
           │      │ • Detector-     │
           │      │   agreement    │
           │      │   acceptance   │
           │      │ • Raw corrobo- │
           │      │   ration gate  │
           │      │ • Full duty     │
           │      └────────┬────────┘
           │               │ timeout/desync
           │               ↓
           │      ┌─────────────────┐
           │      │    RECOVER      │
           │      │ • Expand        │
           │      │   refInterval   │
           │      │ • Duty held     │
           │      │ • Max 5         │
           │      │   attempts      │
           │      └────────┬────────┘
           │               │ 10 good ZCs
           └───────────────┘
```

### 0.7 Noise Comparison: Complementary vs Unipolar PWM

```
   COMPLEMENTARY (2 switching edges per cycle):

   H-FET: ────┐████████████┌────┐████████████┌────
              ↓            ↑   ↓            ↑
   L-FET: ████┘            └████┘            └████
              ↑ edge 1     ↑ edge 2

   Comparator: ╥╨╥──────────╥╨╥──╥╨╥──────────╥╨╥──
               noise1      noise2 noise1      noise2

   Result: 2 noise spikes per PWM cycle


   UNIPOLAR H-PWM/L-OFF (1 switching edge per cycle):

   H-FET: ────┐████████████┌──────────────────────
              ↓            ↑
   L-FET: ────────────────────────────────────────  (always OFF)
              ↑ edge 1     (no edge 2!)

   Comparator: ╥╨╥──────────────────────────────────
               noise1 only

   Result: 1 noise spike per PWM cycle → 34x fewer ZC timeouts!
           But: no braking → motor free-wheels → needs speed PID
```

### 0.8 The CLC Clock Position Problem (Known Issue)

```
   Center-aligned PWM at different duty ratios:

   30% duty:
   Counter: ──/\──/\──/\──
                ↑ peak
   ON:      ___╱██╲___╱██╲___
   CLC CLK:    ↑ (far from edges = CLEAN sample)

   50% duty:
   Counter: ──/\──/\──/\──
   ON:      _╱████╲_╱████╲_
   CLC CLK:   ↑ (ON/OFF edge RIGHT HERE = NOISY sample!)

   80% duty:
   Counter: ──/\──/\──/\──
   ON:      ╱██████╲╱██████╲
   CLC CLK:  ↑ (near edge again = NOISY)

   The CLC clock fires at a FIXED counter position (PG1TRIGA=0).
   At some duty ratios, this position coincides with a switching edge.

   FIX: PTG edge-relative sampling (always N µs after edge, duty-independent)
```

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
- **Frequency**: 210.5 kHz (SCCP1 timer, non-integer ratio with PWM)
- **Signal source**: CLC D-FF output (clean) when FEATURE_CLC_BLANKING=1
- **Raw corroboration**: Raw GPIO read in parallel (sanity check, not co-equal detector)
- **Acceptance**: Mode-dependent:
  - ACQUIRE/RECOVER: full adaptive filterLevel (3-8 consecutive reads)
  - TRACK: detector-agreement acceptance (see below)

**TRACK Acceptance (Phase 2 — no speed gates)**:

Two paths, no `stepPeriodHR` thresholds:
- **Fast accept**: `pollFilter >= 1` AND raw stable (`rawCoro >= 2`, or `rawCoro >= 1` with `rawAge >= pollPeriodHR`)
  - Primary gate: two consecutive raw matches confirm CLC is not stale
  - Jitter insurance: single raw match that persisted for one poll period
- **Safe fallback**: `pollFilter >= 2` AND `candidateAge >= pwmPeriodHR`
  - CLC persisted through a fresh D-FF update cycle (guarantees D-FF re-sampled)

All candidate state (`rawCoro`, `rawFirstMatchHR`, `firstClcMatchHR`) is candidate-local — resets on any CLC mismatch. `rawFirstMatchHR` also resets on any raw mismatch to prevent spanning chatter.

**Timestamp selection** (after acceptance, never affects whether ZC is accepted):
1. IC timestamp — if `icCandidateValid` and IC doesn't lead raw by more than `IC_LEAD_MAX_HR`
2. `rawFirstMatchHR` — first stable raw observation within the CLC candidate
3. `firstClcMatchHR` — first CLC match (fallback)
4. Current poll time (last resort)

**Operation**:
1. Blanking check (12% of step period from commutation, floor 25µs)
2. 50% interval rejection (elapsed since last ZC > half of refIntervalHR)
3. Read CLC output for floating phase + raw GPIO in parallel
4. If CLC matches expected: record firstClcMatchHR, update rawCoro/rawFirstMatchHR
5. If raw mismatches: reset rawCoro and rawFirstMatchHR
6. IC age validation (PWM-aware budget: `pwmPeriodHR + pollPeriodHR + margin`)
7. Check acceptance criteria (fast or fallback path)
8. Select best timestamp, call RecordZcTiming + ScheduleCommutation

### 2.4 Combined Flow
```
Commutation fires (SCCP4 OC ISR)
    ↓
BEMF_ZC_OnCommutation:
    → Configure SCCP2 IC (PPS route, edge polarity)
    → ForcePreZcState (CLC D-FF reset to pre-ZC state)
    → Clear rawCoro, rawFirstMatch, firstClcMatch
    → Set blanking end time
    → phase = IC_ZC_BLANKING
    ↓
FastPoll (210.5 kHz):
    → Check blanking: if expired → phase = IC_ZC_ARMED, arm IC
    → 50% interval rejection
    → Read CLC output (clean) + raw GPIO (corroboration)
    → If CLC matches: track rawCoro, rawFirstMatchHR, firstClcMatchHR
    → If raw mismatches: reset rawCoro + rawFirstMatchHR
    → IC age validation (PWM-aware budget)
    → TRACK: fast accept (CLC+raw stable) or fallback (FL=2+pwmAge)
    → ACQUIRE: full adaptive FL (3-8)
    → Timestamp selection: IC → raw → CLC → poll
    → If accepted: RecordZcTiming + ScheduleCommutation
    ↓
IC Capture (_CCP2Interrupt):
    → Fires on raw comparator edge (before CLC updates)
    → 50% interval check on backdated timestamp
    → Stores precise zcCandidateHR (640ns), sets icCandidateValid
    → Does NOT call RecordZcTiming
    → Poll path validates IC freshness and lead-vs-raw before using
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
- Normal operation, full duty range available
- **Detector-agreement acceptance** (no speed gates):
  - Fast path: CLC + raw GPIO both match expected (rawCoro >= 2 or rawAge >= pollPeriodHR)
  - Fallback: pollFilter >= 2 AND CLC candidate persisted >= 1 PWM cycle
- **Timestamp hierarchy**: IC (if valid, not leading raw) → raw → CLC → poll
- **IC age validation**: PWM-aware budget (pwmPeriod + pollPeriod + margin)
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

### A2212 1400KV at 12V, 40kHz PWM (Phase 2 — detector-agreement)

| Test | Acceptance | ZC Success | Max eRPM | Timeouts | icFalse |
|------|-----------|-----------|----------|----------|---------|
| No-load, Phase 1 (rawCoro>=1) | CLC+raw(1) | 100% | 132k (inflated) | 24 | 92 |
| **No-load, Phase 2 (rawStable)** | **CLC+raw(stable)** | **100%** | **112k** | **0** | **15** |

Phase 2 diagnostic counters (60,202 ZC accepts):
- rawVeto: 42,863 (stale D-FF blocked)
- rawStableBlock: 63,235 (noise coincidence blocked — primary Phase 2 fix)
- trackFallback: 1,144 (PWM-aged safe path)
- icLeadReject: 44,297 (IC bounce timestamps downgraded)
- tsFromIc: 8,314 (13.8%), tsFromRaw: 51,877 (86.1%)

### Key Findings
1. **With prop: 100% ZC** on both motors (12V and 18V)
2. **Phase 2 eliminates all timeouts and forced commutations** at 40kHz
3. **50-54k eRPM false IC spike eliminated** (Phase 1 removed speed gate, Phase 2 tightened acceptance)
4. **Raw timestamp is dominant source** (86%) — IC is opportunistic precision, not primary
5. **eRPM readings converge** at high duty (100.8k at 95% duty, consistent across runs)
6. **No-load noise at 50%/80% duty**: CLC clock position issue (counter-relative, not edge-relative)
7. **24V Vbus swings**: bench supply sags 20-28V under 25A → comparator threshold shift
8. **Unipolar gives 34x fewer timeouts** but needs speed PID for proper control
9. **CLC+IC hybrid** combines complementary braking with clean ZC detection

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
- Phase 2 raw stability gate significantly reduces false accepts during transients
- ACQUIRE mode full deglitch prevents this at CL entry
- **Fix**: Duty slew rate limiting (DUTY_SLEW_UP=3 already active)

### 9.5 IC Timestamp Utilization at High Speed
- At high speed with moderate duty (70-90%), IC timestamp is mostly rejected:
  - icAgeReject: IC fires during blanking/noise, timestamp too old by poll time
  - icLeadReject: IC catches bounce edge, timestamp leads raw stability by too much
- Result: tsFromRaw dominates (86%), IC provides timing in only 14% of ZC accepts
- IC still valuable for 50% interval rejection in ISR (prevents early noise edges)
- **Potential fix**: Scan-window state machine (explicit ZC search model, not level-agreement)

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
PWMFREQUENCY_HZ         40000   // 40 kHz (zero timeouts at 100k eRPM)
ZC_POLL_FREQ_HZ         210526  // Non-integer ratio with PWM

// Timing advance
TIMING_ADVANCE_LEVEL    2       // 15° (AM32 default)

// ATA6847
EGBLT                   15      // 3.75µs edge blanking (maximum)
SCTHSEL                 7       // 2000mV VDS threshold (for 24V)

// Deglitch
ZC_DEGLITCH_MIN         3       // Min consecutive reads (high speed)
ZC_DEGLITCH_MAX         8       // Max consecutive reads (low speed)
// ACQUIRE: full adaptive filterLevel (3-8)
// TRACK: detector-agreement (raw stable + CLC, or FL=2 + PWM age)

// Phase 2: PWM-aware timing budgets (derived, not magic numbers)
IC_AGE_MAX_HR           // pwmPeriodHR + pollPeriodHR + margin (~56 HR @ 40kHz)
IC_LEAD_MAX_HR          // pollPeriodHR + margin (~17 HR @ 210kHz)
RAW_STABLE_AGE_HR       // pollPeriodHR (~7 HR @ 210kHz)
PWM_PERIOD_HR           // pwmPeriodHR (~39 HR @ 40kHz)
```
