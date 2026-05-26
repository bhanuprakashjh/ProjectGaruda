# V4 Prop-Startup Investigation — Capture Pathology & Race Fix

**Status:** in progress (last build untested as of session end).
**Bench:** A2212 1400KV @ 12V, MOTOR_PROFILE=1, with prop.
**Goal:** make CL handoff stable under prop load. Bare-motor handoff worked; prop case ran away to phantom 600k–800k eRPM and tripped SP/IDLE within ~100 ms of CL entry.

This file captures the full debug arc: the diagnostic instrumentation we
added, what the captured data revealed, and the two firmware fixes in
flight at session close.  Read this before resuming the prop fix.

---

## Symptom (entering this investigation)

After a long sequence of config-level attempts on prop startup (PWM
frequency, MIN_STEP, RAMP_ACCEL, RAMP_DUTY_CAP, V4_MIN_AMPLITUDE,
ILIM_DAC, V4_PHASE_ADVANCE_DEG, CCP ISR re-enable) and a per-capture
PI delta clamp, the prop run still went:

```
ALIGN → OL_RAMP (3 s, eRPM tracking) → CL @ ~5–8k eRPM
       → eRPM jumps 503k / 873k (phantom) → SP trip → IDLE
```

User feedback: *"no progress with prop"*.  Six prior fixes had failed.

I proposed two paths: (1) instrument the capture stream so we could see
exactly what the PI was being fed, or (2) blindly add a "skip first N
captures" gate.  User picked **(1) instrument first**.

---

## Diagnostic instrumentation

### 1. PI capture-log ring buffer (firmware)

`motor/sector_pi.c` — a 30-entry ring captures every PI iteration after
CL entry, then freezes once full.  Re-armed on each `EnterCL()`.  Per
entry: 8 bytes.

```c
typedef struct __attribute__((packed)) {
    uint16_t timerPeriod;   /* PI input (before update) */
    uint16_t setValue;      /* PI's expected ZC position */
    uint16_t capValue;      /* what the capture reported */
    int16_t  deltaClamped;  /* post-±25% clamp */
} PI_LOG_ENTRY_T;
```

Accessor: `SectorPI_GetCaptureLog(uint8_t *buf, uint8_t *entriesOut)`
in `motor/sector_pi.h`.

### 2. GSP command (firmware)

`gsp/gsp_commands.h` + `gsp/gsp_commands.c`:

```c
GSP_CMD_PI_LOG = 0x30,
```

Response: `[count(u8)][n × 8B entries]` — total ≤241 bytes, fits in one
GSP frame (max 249).

### 3. pot_capture.py — auto-fetch + dump (host)

`tools/pot_capture.py`:

- After Ctrl+C / motor stops, sends `CMD_PI_LOG`.
- Parses + prints the log as a table.
- Writes a CSV next to the main capture: `<csv>_pi_log.csv`.

The log holds the most recent CL entry, so a prop run that fails fast
still has the failure boundary captured.

---

## Run #1 — pre-fix data ("no progress with prop", session 1)

Build: pre-investigation HEAD with the per-capture delta clamp added.
File: `pot_capture_20260429_173721_pi_log.csv`.

```
  #   tPer  setVal  capVal   delta
  0   3906   2612    3647     976
  1   4211   2815    2773     -42
  2   3953   2643    3050     407
  3   4090   2734    1962    -772
  4   3747   2505    1510    -936
  5   3647   2439    1700    -739
  6   3649   2440    3558     912
  7   4119   2754    2253    -501
  8   3733   2496    2146    -350
  9   3749   2507    3366     859
 10   4104   2744    3848    1026
 11   4210   2815    2723     -92
 12   3925   2624    3668     981
 13   4254   2844    2274    -570
 14   3830   2561      20    -957   ← capture at start of sector
 15   3673   2456    1079    -918
 16   3625   2424    3486     906
 17   4137   2766    1239   -1034
 18   3587   2399    3007     608
 19   4036   2698    1589   -1009
 20   3567   2385    3083     698
 21   4037   2699    1488   -1009
 22   3546   2371    3106     735
 23   4027   2692    1311   -1006
 24   3529   2360    3164     804
 25   4032   2696    1200   -1008
 26   3516   2351      20    -879   ← again at start of sector
 27   3493   2336    2498     162
 28   3763   2516    2706     190
 29   3781   2528    3273     745
```

Live telemetry alongside this run:

```
Cap%   Bnk%   Mis%   Set%    R/F%
26%    19%    69%    11%    3/97   ← R/F = rising/falling capture split
```

### Analysis — what this told us

1. **PI clamp is doing its job.**  No runaway in this run; period
   stays bounded in 3500–4250.  But the motor still decays.

2. **capValue is bimodal:**
   - "real ZC" cluster around 50% of T (idx 3, 4, 5, 8 — 1962/4090=48%,
     1510/3747=40%, 1700/3647=47%, 2146/3733=58%)
   - "wrong" cluster around 80–95%+ of T (idx 0, 6, 12 — 3647/3906=93%,
     3558/3649=97%, 3668/3925=93%)
   - Two captures at `capVal=20` (idx 14, 26) — captures slipping past
     blanking.

3. **From idx 18–25 the alternation is mechanical:** 3007, 1589, 3083,
   1488, 3106, 1311, 3164, 1200.  PI is locked in a **2-step limit
   cycle** — doesn't converge, just oscillates.  Clamp prevents
   runaway but can't break the cycle.

4. **R/F = 3/97 in live telemetry** — 97% of captures are tagged as
   falling-sector polarity, only 3% rising.  Not a real polarity
   distribution; one of the capture paths is silently broken.

### First hypothesis (correct conclusion, partially correct mechanism)

`garuda_service.c:450` — V4 ADC midpoint ISR:

```c
uint8_t expected = isRising ? 1 : 0;   // pre-ZC convention
if (comp != expected) v4_adcStateMismatch++;
else                  v4_adcCaptureSet++;
```

Confirmed by the comment block at `garuda_config.h:962–977`:

> The V4 ADC ISR uses `expected = isRising ? 1 : 0` — this is the
> **pre-ZC state** for both polarities (rising sector pre-ZC has
> comp=1, falling sector pre-ZC has comp=0 on the inverted ATA6847
> comp).  V4 only ever accepts captures in rising sectors because
> the pre-ZC sample for falling sectors lands too late within the
> sector and gets rejected by the half-period filter in Commutate.

The `FEATURE_V5_POST_ZC_ACCEPT` / `FEATURE_V5_POST_ZC_OWN` flags
already existed for the post-ZC convention but the OWN code path was
**stubbed**, never actually wired to drive `v4_captureValid`.  Shadow
counters showed both polarities had ~67% post-ZC accept rate — the
signal was there, the firmware just wasn't using it.

---

## Fix #1 — Post-ZC ADC capture convention

### `garuda_service.c` — implement the OWN code path

Inside `_ADCInterrupt`, replaced the V4 expected-pre-ZC logic with a
post-ZC implementation gated by `FEATURE_V5_POST_ZC_OWN`:

```c
#if FEATURE_V5_POST_ZC_OWN
{
    extern volatile uint8_t v5_ptgExpectedComp;
    uint8_t expectedPost = v5_ptgExpectedComp;
    bool    sectorRising = (expectedPost == 0u);
    if (comp != expectedPost) {
        v4_adcStateMismatch++;
    } else {
        v4_adcCaptureSet++;
        if (sectorRising) v4_adcSetRising++;
        v4_lastCaptureHR = nowHR;
        v4_captureValid  = true;
    }
}
#else
    /* original V4 pre-ZC logic preserved for rollback */
#endif
```

Notes on the choice of `v5_ptgExpectedComp`:

- It's written in `SectorPI_Commutate` as `position & 1u` (even=0
  rising, odd=1 falling).  That's exactly the post-ZC state on the
  inverted ATA6847 (rising-BEMF ZC: comp drops to 0; falling-BEMF
  ZC: comp rises to 1).
- Reading the volatile global bypasses
  `HAL_Capture_IsRisingZc()`, which the codebase documents as
  "stuck-TRUE in ISR context" (see Apr 18 investigation notes
  referenced at `motor/sector_pi.c:760–766`).
- The V5.1 shadow path uses the same global and reports correct rates
  per polarity — so it's the trusted source.

### `garuda_config.h` — turn the flags on

```c
#ifndef FEATURE_V5_POST_ZC_ACCEPT
#define FEATURE_V5_POST_ZC_ACCEPT  1   // was 0
#endif
#ifndef FEATURE_V5_POST_ZC_OWN
#define FEATURE_V5_POST_ZC_OWN     1   // was 0
#endif
```

`FEATURE_V5_POST_ZC_ACCEPT=1` is required so the Commutate write of
`v5_ptgExpectedComp` is unconditionally enabled (`sector_pi.c:767`
gate).

---

## Run #2 — post Fix #1 ("worse than before")

Build: post-fix-#1.  Files: `pot_capture_20260429_204519_pi_log.csv`
and `..._204550_pi_log.csv`.

```
  #   tPer  setVal  capVal   delta
  0   3906   2612    1167    -976
  1   3601   2408     313    -900
  2   3563   2382     275    -890
  3   3509   2346     275    -877
  4   3457   2312     329    -864
  5   3407   2278     435    -851
  6   3356   2244     591    -839
  7   3306   2211     286    -826
  ...
 13   3023   2022      19    -755
 14   2978   1992     529    -744
 ...
 25   2525   1689      76    -631
 ...
 29   2379   1592     353    -594
```

Run-2 second start (worse):

```
3.6  CL  831500   ← phantom eRPM in <0.5s, SP trip, IDLE
```

PI log second start showed many `capVal` values at 19–73 (essentially
the start of the sector) interleaved with mid-sector values:

```
  1   3601   2408      21    -900
  2   3563   2382      63    -890
  4   3457   2312      24    -864
  5   3407   2278      19    -851
  6   3356   2244    2533     289   ← occasional real ZC slips through
  ...
 13   3166   2117      47    -791
 15   3071   2054      24    -767
 ...
```

### Analysis — Fix #1 exposed a deeper bug

The post-ZC convention matches MORE often than pre-ZC — comp=0 (or
comp=1 for falling sectors) holds for the entire post-ZC half of the
sector, vs the narrower pre-ZC window.  That higher match rate
unmasked a **race condition** that the pre-ZC convention had been
hiding.

### Root cause: cross-sector ADC race

In `SectorPI_Commutate`, the ordering was:

1. Disable CCP IEs (`_CCP2IE = 0; _CCP5IE = 0`).
2. Read `thisCommHR`, compute `measuredCommPeriod`, update
   `actualStepPeriodHR`.
3. **Consume previous capture; set `v4_captureValid = false`** (line 658).
4. Advance `position`, reconfigure CCP, flush FIFOs.
5. **Set new `v4_blankingEndHR = thisCommHR + blank`** (line 743).

The ADC ISR fires at 40 kHz and is **NOT disabled**.  Between step 3
and step 5, the ADC ISR can fire and:

- See `v4_captureValid == false` (just cleared).
- See `(nowHR - v4_blankingEndHR) >= 0` — the *old* sector's blanking
  end is now in the past, so the gate trivially passes.
- Read comp; with post-ZC convention it matches almost any sample.
- Accept: `v4_lastCaptureHR = nowHR; v4_captureValid = true`.

`nowHR` here is just after `thisCommHR`, so the next Commutate
computes `elapsed = nowHR - prevCommHR ≈ 20–700 HR` — exactly the
"start of sector" cluster we see.

PI consumes this tiny `capValue`, computes a huge negative delta
(clamped at –tPer/4), integrates `tPer` downward, motor commutates
faster and faster against a stationary rotor → phantom eRPM → SP trip.

Pre-ZC convention had been hiding this because `comp == pre-ZC` is
only true for the first ~half of the sector and only in narrow
windows; the race rarely produced a bogus accept.  Post-ZC matches
broadly → race fires almost every sector.

---

## Fix #2 — close the cross-sector race

### `motor/sector_pi.c` — `SectorPI_Commutate`

Set `v4_blankingEndHR` for the *new* sector **before** clearing
`v4_captureValid`, so the ADC ISR's blanking gate rejects samples
during the critical section:

```c
/* 2.0 Close the cross-sector ADC race. ... */
{
    extern volatile uint16_t v4_blankingEndHR;
    uint16_t guardSectorHR = (actualStepPeriodHR >= V4_MIN_PERIOD)
                             ? actualStepPeriodHR : timerPeriod;
    uint8_t  guardPct      = v4Params.blankingPct;
    uint16_t guardBlankHR;
    if (guardPct == 0U || guardPct > 100U) {
        guardBlankHR = guardSectorHR >> 2;
    } else {
        guardBlankHR = (uint16_t)(((uint32_t)guardSectorHR * guardPct) / 100U);
    }
    v4_blankingEndHR = (uint16_t)(thisCommHR + guardBlankHR);
}
```

Width re-uses `actualStepPeriodHR` because sector width is independent
of position parity; the later (existing) write at line ~743 is now
redundant for the global but does no harm.

### `motor/sector_pi.c` — `EnterCL`

Also seed `v4_blankingEndHR` directly (not just the static
`hal_capture.c:blankingEndHR` via `HAL_Capture_SetBlanking`).  Without
this, the very first CL sector inherited `v4_blankingEndHR = 0` from
init, so every ADC sample passed the gate:

```c
/* Seed v4_blankingEndHR so the ADC ISR's blanking gate is sane
 * during the first CL sector ... */
{
    extern volatile uint16_t v4_blankingEndHR;
    uint8_t  pct      = v4Params.blankingPct;
    uint16_t blankHR;
    if (pct == 0U || pct > 100U) {
        blankHR = timerPeriod >> 2;
    } else {
        blankHR = (uint16_t)(((uint32_t)timerPeriod * pct) / 100U);
    }
    v4_blankingEndHR = (uint16_t)(lastCommHR + blankHR);
}
```

---

## State at session end

- **Build:** clean (post Fix #1 + Fix #2).
- **NOT YET TESTED on motor.**  Next bench session: flash this build
  and run prop startup again.

### What to expect on retest

| Signal | Pre-fix value | Predicted post-fix value |
|---|---|---|
| `capVal` cluster at <50 HR | many entries | none (race blocked) |
| `capVal` cluster overall | bimodal 50% / 85%+ | single cluster ~50% T |
| Live R/F% | 2/98 to 3/97 | toward 50/50 |
| Live `p2R%` / `p2F%` | ~6% / ~2% | both ≥ ~60% (already valid in shadow) |
| PI deltas | mostly clamped at ±tPer/4 | mostly small, well inside clamp |
| Prop CL handoff | runaway/decay | should converge |

### Verification checklist

1. **Prop A2212 @ 12V (profile 1)** — primary test.  Capture log
   should show single-cluster `capVal` near 50% T; CL handoff should
   not run away.
2. **Bare A2212 @ 12V** — must still start.  If it doesn't, the post-ZC
   convention has a flaw at low BEMF.
3. **Bare 2810 @ 25V (profile 1 settings on 2810)** — high-speed
   regression check.  Prior baseline 196k–238k eRPM under block-comm.
   Post-ZC convention shouldn't regress this; if it does, look for
   speed-dependent issues in the capture path.
4. **Block-comm engagement** — check at full pot.  BLK indicator
   should appear in telemetry at the configured per-profile threshold.

---

## Other changes still in the working tree (uncommitted)

These predate this investigation but may want to ship together with
the prop fix once verified:

- **Per-capture PI delta clamp** (`sector_pi.c`, ~line 840):
  ```c
  int32_t deltaCap = (int32_t)timerPeriod >> 2;
  if (delta >  deltaCap) delta =  deltaCap;
  if (delta < -deltaCap) delta = -deltaCap;
  ```
  Belt-and-suspenders against any single noisy capture.  Run #1 showed
  it kept the system from runaway; keep it.

- **CCP2/CCP5 ISRs re-enabled** in `hal/hal_capture.c:HAL_Capture_Start`.
  Was disabled in commit `9df971b` to save FIFO-drain CPU; suspected
  prop-startup regressor and re-enabled this session.  Inconclusive on
  prop alone, doesn't seem to hurt — keep enabled.

- **Profile 3 (HiZ1460 16Ω @ 30V) startup config** — `garuda_config.h`,
  matched to colleague's working code at
  `/media/bhanu1234/Development/ProjectGaruda/garuda-6step-ckhtitesh/`.
  ALIGN_TIME_MS=200, ALIGN_DUTY=LOOPTIME_TCY/20, INITIAL_STEP_PERIOD=1000,
  RAMP_ACCEL_ERPM_S=500, RAMP_DUTY_CAP=LOOPTIME_TCY/8.  Awaiting
  colleague verification.

- **Profile 2 startup** — aligned to profile 1's proven cadence
  (RAMP_ACCEL=1500, RAMP_DUTY_CAP=/6, ALIGN_DUTY=/40, ALIGN_TIME=150,
  INITIAL_STEP=800).  Profile 2's original values were never validated.

- **Per-profile V4_BLOCK_ENTER/EXIT_ERPM** — sized to each motor/Vbus.

---

## Files touched this session

```
garuda_6step_ck.X/garuda_config.h          (V5_POST_ZC_* flags → 1)
garuda_6step_ck.X/garuda_service.c         (post-ZC OWN code path)
garuda_6step_ck.X/motor/sector_pi.c        (PI log + race fix + EnterCL seed)
garuda_6step_ck.X/motor/sector_pi.h        (capture-log accessor proto)
garuda_6step_ck.X/gsp/gsp_commands.h       (GSP_CMD_PI_LOG = 0x30)
garuda_6step_ck.X/gsp/gsp_commands.c       (PI_LOG handler)
PATA6847/tools/pot_capture.py              (auto-fetch + dump PI log)
```

Captured CSVs (in repo root):

```
pot_capture_20260429_173721.csv          + _pi_log.csv   (Run #1, pre Fix #1)
pot_capture_20260429_204519.csv          + _pi_log.csv   (Run #2, post Fix #1)
pot_capture_20260429_204550.csv          + _pi_log.csv   (Run #2 retry)
```

---

## When you resume

1. Flash the latest build (Fix #1 + Fix #2 in place).
2. Prop-test with A2212 @ 12V (profile 1).
3. Pull the PI log via `pot_capture.py`; compare `capVal` distribution
   to the predictions above.
4. If `capVal` is now single-cluster near 50% of `tPer` and prop CL
   handoff converges → fix is good, run regression checks (bare A2212,
   bare 2810, block-comm engagement).
5. If `capVal` is still bimodal — the cluster shape will tell us where
   the next bug lives:
   - Both clusters now in ≥40% T region → blanking is honest, but a
     second capture path (CCP IC) is still firing on bouncy edges.
     Investigate `HAL_Capture_IsRisingZc()` stuck-TRUE in CCP ISRs.
   - Cluster persists at <40% T → `v4_blankingEndHR` write-ordering
     in Commutate didn't fully close the race.  Possibly need to
     temporarily disable `_AD1IE` across the critical section.
6. Once prop is solid, commit Fix #1 + Fix #2 + the trailing
   uncommitted improvements as a coherent set.
