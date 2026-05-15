# V4 Scheduler — AK Port Internals & Tuning Reference

> **Hardware**: dsPIC33AK128MC106 + ATA6847L on EV43F54A
> **Active config**: `FEATURE_V4_MIDPOINT_ZC=1`, `FEATURE_V5_SCHEDULER=0`, `FEATURE_V4_SECTOR_PI=1`
> **Verified peak**: 226k eRPM bare 2810 motor @ 24V
> **Doc covers**: the actual code paths that run on AK as of 2026-05-15, every load-bearing global, and what changing each knob does on the bench.

This complements `v4_architecture.md` (CK board) by documenting **AK-specific**
findings and the fixes that landed in May 2026. Read the CK doc first if you
want the high-level design intent; this doc is for someone who has the AK
board on their desk and needs to know which variable controls what.

---

## Table of Contents

1. [Code path overview](#1-code-path-overview)
2. [The actors and their priorities](#2-the-actors-and-their-priorities)
3. [The hot-path globals](#3-the-hot-path-globals)
4. [The PI math (what `timerPeriod` really is)](#4-the-pi-math-what-timerperiod-really-is)
5. [Blanking — why it's load-bearing](#5-blanking--why-its-load-bearing)
6. [Advance — the TAL bands](#6-advance--the-tal-bands)
7. [The 2T:ε pacing reality](#7-the-2tε-pacing-reality)
8. [Tuning knobs and what they do](#8-tuning-knobs-and-what-they-do)
9. [Diagnostic counters — how to read them](#9-diagnostic-counters--how-to-read-them)
10. [Known races](#10-known-races)
11. [Feature flag cheat sheet](#11-feature-flag-cheat-sheet)

---

## 1. Code path overview

A full electrical revolution looks like this:

```
            ┌─────────────────────────────── 1 electrical revolution ────────────────────────────────┐
            │  Sector 0     Sector 1     Sector 2     Sector 3     Sector 4     Sector 5             │

PWM         ┌─┐_┌─┐_┌─┐  ┌─┐_┌─┐_┌─┐  ┌─┐_┌─┐_┌─┐  ...
            └─┘ └─┘ └─┘  └─┘ └─┘ └─┘  └─┘ └─┘ └─┘
                                                                ┌── physical ZC of float phase
                                                                │
PTG fires      ↑    ↑    ↑    ↑    ↑    ↑    ↑    ↑    ↑    ↑    ↑    ↑    ↑    ↑
(60 kHz)       └────────────────────────────── samples BEMF ───┴──── transition ─┘

Commutate                ↑                       ↑                       ↑
(driven                  └── consumes capture    └── fires on schedule   └── ...
 by CCP3)
```

Two ISRs do all the load-bearing work:

| ISR | Source | Priority | Job |
|---|---|---|---|
| `_PTG0Interrupt` (`hal/hal_ptg.c`) | PTG step IRQ on PG1TRIGB match (per PWM cycle, 60 kHz) | **5** | Call `V4_ProcessBemfSample()`. Read BEMF, decide if it's a ZC, set `v4_captureValid` + `v4_lastCaptureHR`. |
| `_CCT3Interrupt` (`garuda_service.c`) | SCCP3 timer period match (scheduled per Commutate) | **6** | Call `SectorPI_Commutate()`. Advance PWM step, consume capture, update PI, schedule next CCT3 fire. |

**Priority 6 > 5 means Commutate can preempt PTG mid-flight.** That's the
race that bit the May 15 session and produced the
`memory/ak_v4_writes_atomic_2026_05_15.md` fix.

PTG cannot preempt Commutate — but Commutate can interrupt PTG, run to
completion, then PTG resumes mid-statement with some globals already
updated. This is the structural reason why all the per-sector globals
(`v4_currentSector`, `v4_floatingPhase`, `v5_ptgExpectedComp`) must be
written **back-to-back** in Commutate so PTG can never see a mid-update
snapshot.

---

## 2. The actors and their priorities

| Module | Priority | Fires when | Touches |
|---|---|---|---|
| **SCCP3** (`_CCT3Interrupt`) | 6 | Scheduled. Each Commutate arms next fire at `thisCommHR + sched_Tsector` (fallback) or `lastCapture + delayHR` (capture path). | Calls `SectorPI_Commutate` |
| **PTG0** (`_PTG0Interrupt`) | 5 | PG1TRIGB match — once per PWM cycle (60 kHz). | Calls `V4_ProcessBemfSample` |
| **CCT1** (`_CCT1Interrupt`) | 4 | 40 kHz Timer1-style tick, phase-offset from PTG. | OFF-mid sampler / falling-ZC alt path (`FEATURE_V4_OFFMID_ZC`) |
| **T1** (`_T1Interrupt`) | 3 | 1 kHz system tick. | TimeTick: speed measurement, smoothing, GUI telemetry timestamping, OL ramp counters. |
| **AD1CH4** (`_AD1CH4Interrupt`) | 3 | ADC channel-4 done (Vbus). | Vbus reading only on AK — current uses different channels. |

PTG=5 and Commutate=6 sharing the same "every PWM cycle" domain is the
key fact. Don't change either priority casually — many subtle behaviours
fall out of the preemption order.

---

## 3. The hot-path globals

### Set by `Commutate`, read by `PTG`

These three globals are written **atomically as one block** at the top
of step-3 in both `SectorPI_Commutate` and `CommutateV5_3` (sector_pi.c
~lines 877–894 and 654–671). If you add a new per-sector global, write
it inside the same block.

| Global | Type | Meaning | Twisting it does what |
|---|---|---|---|
| `v4_currentSector` | uint8_t (0..5) | Software's current commutation step. Even (0/2/4) = rising-BEMF sectors. Odd (1/3/5) = falling. | Set by `position++` in Commutate. Do not assign elsewhere. Reading it from PTG is safe **only** when paired with the next two. |
| `v4_floatingPhase` | uint8_t (0=A, 1=B, 2=C) | Which physical motor phase is floating in this sector. Used by `ReadBEMFComp()` to pick the right BEMF GPIO. | Updated atomically with `v4_currentSector` — never write separately. If you see staleness rise in `v4_fpStaleCount`, this pairing has been broken. |
| `v5_ptgExpectedComp` | uint8_t (0 or 1) | Expected POST-ZC comparator state for this sector. `0` = rising sector (post-ZC ATA6847 output goes LOW). `1` = falling sector. | Computed as `position & 1u`. Used by PTG to decide rising-vs-falling without calling `HAL_Capture_IsRisingZc()` (which has a stuck-true bug in ISR context — historical, never root-caused). |

### Set by `PTG`, read by `Commutate`

| Global | Type | Meaning | Twisting it does what |
|---|---|---|---|
| `v4_captureValid` | bool | "PTG accepted a sample this sector — there's a fresh capture to consume." | Set true by PTG when the deglitched comp matches expectation past blanking. Cleared by Commutate after consumption. Sticky within a sector (only first accept per sector matters). |
| `v4_lastCaptureHR` | uint16_t (HR ticks, 640 ns each) | Timestamp of the accepted capture, in SCCP4 free-running domain. | Set alongside `v4_captureValid=true`. Commutate uses this as the anchor for the next scheduled target. |

### PI internal state — only `SectorPI_Commutate` writes these

| Global | Type | Meaning | Twisting it does what |
|---|---|---|---|
| `timerPeriod` | uint16_t (HR ticks) | **The PI's current best estimate of one electrical sector's wall-clock duration.** This drives blanking width, expected ZC position, and fallback schedule. | Decreasing it = motor commutates faster (assumes higher speed). The PI updates this every Commutate as a function of `capValue` (consumed elapsed) and `setValue` (target advance). |
| `v4_timerPeriod` | uint16_t | Mirror of `timerPeriod` exported for CCP-ISR speed checks. | Identical value, just non-static. Don't write to this directly. |
| `integrator` | uint16_t | PI integral accumulator (in `timerPeriod` units). | Floor-clamped in CL_SETTLE to prevent the "death spiral" failure mode (see `seedIntegrator`). |
| `seedIntegrator` | uint16_t | The OL→CL handoff seed value. Floor for `integrator` until `commandEnabled` goes true (after CL_SETTLE_MS = 500 ms). | If a rotor lags handoff, sparse early-CL captures all look like "rotor is ahead" → PI shrinks period → faster fires → rotor falls further behind → stall. Floor stops this cascade. |
| `actualStepPeriodHR` | uint16_t | Truth: wallclock time between the last two Commutate calls. Used for blanking calc (NOT PI). | Updated unconditionally each Commutate from `thisCommHR - prevCommHR`. Independent of capture validity. |
| `position` | uint8_t (static, 0..5) | The software sector index. `v4_currentSector` is a volatile mirror for ISR consumers. | Increments by 1 per Commutate. Wraps 5→0. Reset to 0 on `SectorPI_Start`. |
| `lastCommHR` | uint16_t | Timestamp of the previous Commutate call. | Used to compute `measuredCommPeriod` at the next Commutate. |

### Timing and scheduling

| Global | Type | Meaning | Twisting it does what |
|---|---|---|---|
| `v4_blankingEndHR` | uint16_t | "PTG samples before this timestamp are ignored." Computed as `thisCommHR + blankPct% of sectorHR`. | Increasing `blankPct` (`v4Params.blankingPct`, GSP param `0xF3`) widens the rejection window. Wider = more noise immunity, but if it exceeds the ZC crossing the rotor commutation lags and motor stalls. |
| `v4_expectedZcHR` | uint16_t | Predicted timestamp of the physical ZC, used by some experimental capture paths. | Cosmetic in the current build; touch this only if you turn on the predictor flags. |
| `sched_Tsector` | uint16_t | Capture-anchored period estimate (V5.3 only). | Unused when `FEATURE_V5_SCHEDULER=0`. |

---

## 4. The PI math (what `timerPeriod` really is)

The V4 PI lives at the bottom of `SectorPI_Commutate` (sector_pi.c
~line 1018 to 1090). It's a set-point PI on **commutation interval**,
not on speed directly.

```
setValue = (timerPeriod >> 1) - advance_compensation   // target elapsed
capValue = lastCaptureHR - prevCommHR                  // measured elapsed
error    = setValue - capValue                          // signed
integrator += (Ki * error) >> 12                        // bounded
timerPeriod = integrator + (Kp * error) >> 8            // bounded
```

Key facts about `capValue`:
- It's the **time from the previous Commutate to the latest accepted capture**, not to "now". So a capture mid-sector gives `capValue ≈ T/2` if the timing is right.
- For the legacy V4 path, **only one polarity captures** (rising sectors stay stuck at `cR=4` after the May 15 fix). The other polarity returns `CAP_SENTINEL = 0xFFFF`, which the PI skips. That means the PI runs at half the Commutate rate.
- `capValue < setValue` means "rotor is ahead of where the PI expected" → integrator drops → `timerPeriod` shrinks → next Commutate fires sooner.
- `capValue > setValue` means "rotor is lagging" → integrator climbs → `timerPeriod` grows → next Commutate fires later.

**`timerPeriod` is the most important global to understand.** When you
see `Tp=` in the telemetry that's `timerPeriod`. At 24 V on bare 2810
you'll see it walk smoothly from `3906` (idle, ~17k eRPM CL entry)
down to `~135` (226k eRPM at full pot, block commutation engaged).

### Where the PI sees `measuredCommPeriod = 2T`

`measuredCommPeriod` (locally scoped in `SectorPI_Commutate`) is the
raw Commutate-to-Commutate interval. **It reads twice the true
physical sector period** because the V4 reactive scheduler fires
Commutate twice per physical sector — see section 7. The PI doesn't
consume `measuredCommPeriod` directly, but anything that does (like
`actualStepPeriodHR` for blanking calc) inherits this 2× scaling.
That's not a bug; it's the scheduler's "normal".

---

## 5. Blanking — why it's load-bearing

After each Commutate, PWM polarities change abruptly. The motor
phases ring for a while before the BEMF stabilizes — typical 5-15 µs
on this hardware. Blanking is the time window after Commutate during
which PTG samples are **discarded** (`v4_adcBlankReject++`).

Formula (sector_pi.c ~line 920):

```
sectorHR  = actualStepPeriodHR  // truth — wallclock duration of the previous Commutate cycle
blankPct  = v4Params.blankingPct  // EEPROM param, default ~25 (GSP code 0xF3)
blankHR   = sectorHR * blankPct / 100
v4_blankingEndHR = thisCommHR + blankHR
```

| Knob | Effect of increasing | Effect of decreasing |
|---|---|---|
| `blankPct` | Wider rejection. Tolerates more ringing. Late blanking can engulf ZC at high speed → motor stalls when blank > 50%. | Tighter window. Catches ZC earlier but risks accepting ring artifacts as ZC. Below ~15% on this hardware = phantom captures, motor desyncs. |

The default 25% works idle-to-block on bare 2810. Adjust this **first**
if a new motor desyncs at a specific speed band — the band tells you
whether ringing is too long (raise) or blanking is eating ZC (lower).

---

## 6. Advance — the TAL bands

"TAL" = Timing Advance Level. Speed-banded multiplier on the
electrical advance (in degrees of one sector) that the PI uses both for
its setpoint and for the next-schedule computation.

```c
if      (schedPeriod >= 260U) tal = 2U;     // ≤60k eRPM:    15.0°
else if (schedPeriod >= 130U) tal = 3U;     // 60-120k eRPM: 22.5°
else if (schedPeriod >= 100U) tal = 4U;     // 120-156k eRPM: 30.0°
else                          tal = 5U;     // >156k eRPM:    37.5°
```

`advHR = (schedPeriod >> 3) * tal` is the advance in HR ticks.

`delayHR = halfHR - advHR` is the time from capture to next Commutate.
When `advHR > halfHR` (at very high speed with `tal=5`), `delayHR`
clamps to 2 ticks — the schedule fires nearly immediately after
capture. **That clamp is part of the 2T:ε mechanism** — see section 7.

| Knob | Effect of increasing TAL | Effect of decreasing TAL |
|---|---|---|
| Per-band TAL | Fires Commutate earlier (more advance). Needed at high speed because inductance delays the BEMF crossing. Too much = early Commutate, motor pulls torque before the rotor is in position → loses sync. | Fires Commutate later. Smoother low-speed running. Too little at high speed = BEMF dies, motor stalls. |
| Band thresholds | Higher threshold = TAL stays low up to faster speeds. | Lower threshold = TAL ramps in sooner. |

The current bands are tuned for bare 2810 @ 24 V. For a different
motor, the **switching point** (where you need to jump from TAL=2 to
TAL=3) usually correlates with where the no-load running current
starts to climb steeply.

---

## 7. The 2T:ε pacing reality

This is the single most important architectural fact about V4. Read
this section twice if you're new to the code.

### The mechanism

Pretend we're at idle (T = 3906 HR ticks for one physical sector).

1. **t = 0**: Commutate fires for sector N. Computes `targetHR = thisCommHR + schedPeriod ≈ T`. Calls `HAL_ComTimer_ScheduleAbsolute(T)`. CCP3 will fire at `t = T`.
2. **t = T/2**: rotor crosses the BEMF zero-crossing. PTG ISR runs, deglitched read passes, sets `v4_captureValid = true`, `v4_lastCaptureHR = T/2`. **PTG does not touch CCP3**.
3. **t = T**: CCP3 fires (the schedule from step 1). Commutate B runs. Sees captureValid, computes new target = `v4_lastCaptureHR + delayHR = T/2 + delayHR`. At idle delayHR ≈ T/4 → target = 3T/4. **3T/4 is in the past** (now is T). `HAL_ComTimer_ScheduleAbsolute` falls into its ASAP path (margin < 4 ticks) and fires CCT3IF immediately.
4. **t = T + ε**: Commutate C fires (the ASAP). captureValid was already consumed. `position` advances again. New target = `thisCommHR + schedPeriod ≈ 2T + ε`.
5. Wait for next physical ZC. Repeat.

Result: **two Commutates fire per physical sector**. One at the
fallback (t=T, 2T, 3T, …) and one ε later as ASAP. The software's
sector index runs **twice as fast as the rotor's physical sector
index**.

### Why it doesn't break the motor

- The PWM step does advance on each Commutate, but because each pair is essentially "fire then immediately fire again", the rotor experiences the *second* fire as the effective sector boundary (the first one is briefly mid-update).
- `measuredCommPeriod` (Commutate-to-Commutate) reads ε once and T-ε once, averaging ~T/2. The PI sees half the true sector period. That's why `Tp` settles to ~half what you'd naively expect for a given eRPM.
- The "free advance" comes from the timing: the ASAP-fire Commutate effectively starts the new sector ~T/2 before the rotor reaches its natural sector start. At high speed this is exactly the advance the motor needs.

### Why only one polarity captures

`v4_captureValid` is consumed by Commutate B and reset. Commutate C
fires without a fresh capture (CAP_SENTINEL path). The next sector
(electrically N+2 in software, but the rotor only moved one physical
sector) will see the next ZC… and the cycle repeats. The result is
that captures only ever feed one polarity into the PI — bare 2810 sees
falling-sector captures (`cF` climbs); rising-sector captures `cR`
stay at 4 (the first few CL frames before the scheduler settles into
the 2T:ε pattern).

### What's been tried (so future-you doesn't repeat it)

| Attempt | What | Result | Memory file |
|---|---|---|---|
| Pair v4_currentSector/v4_floatingPhase/v5_ptgExpectedComp writes | Eliminate the race where PTG sees a mid-update snapshot | `cR` 0 → 4 (first rising captures ever); fpStale 7% → 4%. **Landed (commit 0b610d2).** | `ak_v4_writes_atomic_2026_05_15.md` |
| Reschedule CCP3 from PTG ISR | When PTG sets captureValid, also pull CCP3 to fire at `captureHR + delayHR`. Goal: kill the ASAP-pair so it's 1:1. | `cR` flows continuously (7619 vs 4) but PI desyncs because `measuredCommPeriod` doubles → PI thinks motor sped up 2× → period collapses. Peak: 225k → 109k. **Built but defaulted OFF** (`FEATURE_V4_PTG_RESCHEDULE=0`). | `ak_v4_ptg_reschedule_2026_05_15.md` |

To genuinely fix this without losing peak speed you need V5.3 — the
1:1 scheduler that's gated by `FEATURE_V5_SCHEDULER`. That flag is OFF
because the motor runs chaotically with it — needs a real session of
debugging.

---

## 8. Tuning knobs and what they do

These are the load-bearing parameters you actually adjust at the
bench. All visible from GSP (the protocol in `gsp/`), most also via
the GUI.

### EEPROM / runtime (`v4Params.*`)

| GSP code | Field | Default | What it does |
|---|---|---|---|
| 0xF0 | `kP` | varies per profile | PI proportional gain. Higher = faster tracking but more ripple. Too high = oscillation. |
| 0xF1 | `kI` | varies | PI integral gain. Higher = aggressive setpoint chase. Too high = wind-up + stall. |
| 0xF3 | `blankingPct` | 25 | Blanking window as % of `actualStepPeriodHR`. See section 5. |
| 0xF4 | `piFeedPolarity` | 1 (rising-only) | 0 = feed both polarities (assumes 1:1 scheduler), 1 = rising sectors only, 2 = falling only. Currently 1 because legacy V4 only ever captures one polarity anyway. |

### Compile-time (`garuda_config.h`)

| Macro | Default | What it does |
|---|---|---|
| `FEATURE_V4_MIDPOINT_ZC` | 1 | 1 = PTG mid-cycle BEMF sample (active path). 0 = CCP edge-triggered (dormant). 2 = mode-2 ZC (experimental, mostly off). |
| `FEATURE_V4_SECTOR_PI` | 1 | Core enable for V4 scheduler. Off = no commutation. |
| `FEATURE_BEMF_VIA_PTG` | 1 | 1 = PTG ISR calls `V4_ProcessBemfSample`. 0 = ADC ISR does it (older path). |
| `FEATURE_PER_SECTOR_PTG` | 1 | 1 = duty-adaptive PG1TRIGB switching (MID-OFF at low duty, MID-ON at high duty, with hysteresis). 0 = MID-OFF only. |
| `PTG_DUTY_ADAPT_THRESHOLD` | 50% | Duty value at which PTG switches between MID-OFF and MID-ON sampling. Tuned per motor; affects current draw vs ZC quality balance. |
| `PTG_DUTY_ADAPT_HYST` | ±5% | Hysteresis band around the threshold to prevent chatter when duty hunts near the boundary. |
| `FEATURE_V5_POST_ZC_OWN` | 0 | 1 = use V5's POST-ZC capture convention. Off because it cascade-locks on AK hardware. |
| `FEATURE_V5_POST_ZC_ACCEPT` | 1 | Shadow-only: counts what V5 POST-ZC accept logic *would* do. Diagnostic, doesn't affect runtime. |
| `FEATURE_V5_PTG_ZC` | 1 | Reserved, currently writes the `v5_ptgExpectedComp` flag. Required for the diagnostic shadows. |
| `FEATURE_V5_SCHEDULER` | 0 | 1:1 scheduler (CommutateV5_3 path). Off because chaotic. |
| `FEATURE_V4_PTG_RESCHEDULE` | 0 | 2026-05-15 experiment. See section 7 and `ak_v4_ptg_reschedule_2026_05_15.md`. |
| `OL_HANDOFF_SETTLE_MS` | 200 | How long the OL ramp keeps firing at MIN_STEP_PERIOD before transitioning to CL. Increase if rotor falls behind during handoff. |
| `CL_SETTLE_MS` | 500 | After CL entry, integrator stays floored at `seedIntegrator` for this long. Increase if motor has "first-second wobble". |

---

## 9. Diagnostic counters — how to read them

Everything in this section is visible in the `gsp_ak_test.py mon`
telemetry. The host tool reads `snap[]` over GSP and prints these
values once per telemetry frame (default ~50 ms).

### Capture-rate counters

| Field | Meaning | Read it like |
|---|---|---|
| `cR` (`v4_adcSetRising`) | Total captures accepted in rising sectors (even position). | On bare 2810, stays at 4 across the run (the first few CL frames before 2T:ε locks in). After commit 0b610d2's pairing fix this is the *correct* count, not zero. |
| `cF` (`v4_adcCaptureSet - v4_adcSetRising`) | Falling-sector captures. | Climbs continuously with motor speed. Typical 100k+ over a 30 s sweep. |
| `cE` (`commandEnabled`) | 1 = CL_SETTLE has passed, PI is free-running. 0 = CL_SETTLE still floored. | Should be 1 within 500 ms of CL entry. |
| `pR%` | `cR / sectorHits[rising]` — rising-sector capture acceptance ratio. | Stays at 0% in current V4 path (only 4 captures ever). |
| `pF%` | Falling-sector capture acceptance ratio. | Climbs to 80-99% across the speed range. Drops below 50% indicates desync. |

### Shadow tallies (deglitched comp via floating phase)

| Field | Meaning |
|---|---|
| `preR%` (`v4_compRising_High / total`) | Of all post-blanking samples in rising sectors, what fraction show comp=1 (pre-ZC). Bare 2810 baseline: ~96-100%. |
| `postF%` (`v4_compFalling_High / total`) | Of all post-blanking samples in falling sectors, what fraction show comp=1 (post-ZC for falling). Climbs from ~50% (idle) to ~80% (block commutation). |

### Sample-rate counters

| Field | Meaning |
|---|---|
| `skip` (`v5_ptgSkipped`) | Number of PTG fires that bypassed `V4_ProcessBemfSample` due to postscaling (when `PTG_POSTSCALE_N > 1`). Default 0. |
| `sect[a/b/c/d/e/f]` (`v4_sectorHits[0..5]`) | Per-sector Commutate fire counts. Should be near-equal across sectors (±1). |

### Multi-phase BEMF tally (2026-05-15 — `gsp_ak_test.py` end-of-run table)

For each (sector, phase) tuple, the firmware counts how many
post-blanking PTG fires saw comp=1 on each of the three BEMF GPIOs.
Reset per snapshot, so the host tool sums across frames. Look for:

- **Floating phase ([F] in the printout) at 0% or 100%** → phase-mapping bug. Should be transitioning (10-90%).
- **Driven phase at the opposite extreme** → confirms the floating phase is correctly identified.
- **One rising sector at 0% and another at 80%** → that's the 2T:ε scheduler asymmetry. Same waveform, different sample times because the two "rising sectors" land at different rotor phases.

### Race counters

| Field | Meaning |
|---|---|
| `fpStale` (`v4_fpStaleCount`) | Per-frame count of PTG fires where `v4_floatingPhase` disagreed with `commutationTable[v4_currentSector].floatingPhase`. Baseline (post commit 0b610d2): ~4%. >10% indicates a regression in the Commutate-writes-atomic invariant. |
| `v4_ptgRescheduleCount` | (When `FEATURE_V4_PTG_RESCHEDULE=1`) Count of PTG-side CCP3 reschedules. Used to confirm the reschedule path is being hit. |

---

## 10. Known races

These all stem from the priority 5 / 6 difference. Listed in
decreasing severity.

| Race | Status | Fix |
|---|---|---|
| Commutate updates `v4_currentSector` then writes other per-sector globals later; PTG mid-flight can see the gap. | **FIXED** in commit 0b610d2. All three globals written in one back-to-back block. | Pairing block in `SectorPI_Commutate` and `CommutateV5_3`. |
| PTG reads `v4_currentSector` and `v4_floatingPhase` non-atomically. Commutate preemption between the two reads makes the probe (and only the probe — the deglitch is later in the ISR) see a stale combo. | KNOWN, ~4% rate. Doesn't affect motor behaviour. | `v4_fpStaleCount` counter exposes the rate. To eliminate would need disabling interrupts around the read pair. Not worth the cost. |
| `v4_captureValid` is sticky within a sector. If PTG sets it again during the same Commutate cycle (post-blanking, multiple ZC-like edges), only the first one matters. | Intentional. Single capture per sector is correct. | Cleared in Commutate. |
| `actualStepPeriodHR` is read by PTG for blanking — Commutate updates it mid-cycle. | Tolerated. The change is small (one Commutate's interval delta) and blanking is bounded. | If you see telemetry oscillation tied to blanking, snapshot atomically in PTG. |

---

## 11. Feature flag cheat sheet

Quick-reference table of what each flag costs/buys. **Default config
is what's in `garuda_config.h`** as of commit `0b610d2`. All measured
on bare 2810 @ 24V.

| Flag | Default | ON cost | ON benefit |
|---|---|---|---|
| `FEATURE_V4_MIDPOINT_ZC=1` | 1 | Baseline. | Baseline (PTG mid-cycle ZC works). |
| `FEATURE_BEMF_VIA_PTG=1` | 1 | None vs the ADC-ISR alt. | Cleaner sample alignment. |
| `FEATURE_PER_SECTOR_PTG=1` | 1 | One extra branch per PTG ISR. | ~5x reduction in idle current (~12 A → ~5 A) by sampling MID-OFF at low duty. |
| `FEATURE_V5_POST_ZC_ACCEPT=1` | 1 | A few cycles in PTG ISR. | Shadow counters (`v5_postZc*Acc/Rej`). |
| `FEATURE_V5_POST_ZC_OWN=1` | 0 | Replaces legacy V4 detect. | Should be 1:1 with rotor (theoretically). DON'T enable on AK — cascades. |
| `FEATURE_V4_PTG_RESCHEDULE=1` | 0 | Helper inline + one extra ScheduleAbsolute per accepted capture. | `cR` flows continuously, both polarities feed PI. **Currently desyncs** because PI gains expect 2T:ε scale. Future: pair with a `>>1` compensator on measuredCommPeriod. |
| `FEATURE_V5_SCHEDULER=1` | 0 | Replaces SectorPI_Commutate with CommutateV5_3. | True 1:1 scheduling, no ASAP-pair. **Motor runs chaotically on AK.** Real fix would land here. |
| `FEATURE_MIDON_DIAG_PROBE=1` | 0 | Busy-waits ~8 µs inside PTG ISR. | Falling-sector MID-ON shadow counters. Diagnostic only. |
| `FEATURE_V5_PTG_ZC=1` | 1 | None — just enables the shared globals. | Required for V5 shadows to write/read coherently. |
| `FEATURE_V5_MEAS_PI` | varies | Tracks halved-measuredCommPeriod (which IS the true sector period in 2T:ε pattern). | Cleaner telemetry, available for future tracker. |
| `FEATURE_V5_MEAS_PI_OWN=1` | 0 | Replaces PI's timerPeriod with smoothed measurement. | Untested ownership — was a 2026-04 experiment. |

---

## Quick reference: where things live

| Concept | File | Approx line |
|---|---|---|
| V4 Commutate ISR body | `motor/sector_pi.c` | 721 (`SectorPI_Commutate`) |
| V5.3 Commutate (alt) | `motor/sector_pi.c` | 647 (`CommutateV5_3`) |
| PTG-driven BEMF processor | `garuda_service.c` | 524 (`V4_ProcessBemfSample`) |
| CCP3 scheduler primitive | `hal/hal_com_timer.c` | 85 (`HAL_ComTimer_ScheduleAbsolute`) |
| Capture pin GPIO macros | `hal/port_config.h` | 69 (`BEMF_A_GetValue` etc.) |
| Commutation table | `motor/commutation.c` | 18 (`commutationTable[6]`) |
| PI math | `motor/sector_pi.c` | 1018-1090 |
| TAL advance bands | `motor/sector_pi.c` | 1110-1115 |
| ATA6847 register config | `hal/hal_ata6847.c` | — |
| Reschedule helper | `garuda_service.c` | ~530 (`V4_NextTargetHR`) |

---

## Next-session bookmarks

- **`memory/ak_v4_writes_atomic_2026_05_15.md`** — why the three globals must be paired
- **`memory/ak_v4_ptg_reschedule_2026_05_15.md`** — why PTG-reschedule alone won't fix the 2T:ε without PI retune
- **`docs/v4_architecture.md`** — CK board original architecture intent
- **`docs/v4_motor_tuning_guide.md`** — symptom → knob lookup
- **`docs/v4_high_speed_ceiling_debug.md`** — investigation of the high-speed wall
