# ZC V2 Execution Plan

Based on `reference_based_sensorless_zc_redesign.md` (architecture spec) and codex 9-phase
commit strategy. Incremental replacement — preserve working 2810 behavior until each stage
is verified.

## Starting Point

- Commit `200949e` (IIR clamp, bypass recovery, diagnostics)
- Design doc: `8a3347d` (architecture + this plan)
- Working: 2810 stable to 100k eRPM with slow pot, A2212 full speed
- Broken: 2810 desyncs on fast pot at 45-60k eRPM (bypass cascade)

---

## Phase 1: Telemetry Semantics + Scaffolding

**Goal**: Better observability. Zero behavior change.

**Commit**: One commit, all telemetry changes.

### Files and Changes

**`garuda_types.h`**

Add `ZC_MODE_T` enum (scaffolding for Phase 2, reported in telemetry now):
```c
typedef enum {
    ZC_MODE_ACQUIRE = 0,
    ZC_MODE_TRACK   = 1,
    ZC_MODE_RECOVER = 2
} ZC_MODE_T;
```

Add `ZC_CTRL_T` (Phase 2 will populate, Phase 1 just declares):
```c
typedef struct {
    ZC_MODE_T mode;
    uint16_t  acquireGoodCount;
    uint16_t  recoverGoodCount;
    uint8_t   recoverAttempts;
    uint16_t  refIntervalHR;        /* Phase 4 uses this for scheduling */
    uint16_t  rawIntervalHR;        /* Phase 4 uses this for raw capture */
    uint16_t  originalTimeoutHR;    /* captured at commutation for latency calc */
    uint16_t  demagMetric;          /* Phase 6 uses this */
} ZC_CTRL_T;
```

Extend `ZC_DIAG_T`:
```c
typedef struct {
    /* Existing (keep) */
    uint8_t  zcLatencyPct;
    uint16_t lastBlankingHR;
    uint16_t diagBypassAccepted;
    uint16_t diagRisingZcCount;      /* existing from 200949e */
    uint16_t diagFallingZcCount;     /* existing from 200949e */
    uint16_t diagIntervalReject;     /* existing from 200949e */
    /* New */
    uint16_t actualForcedComm;       /* ONLY incremented on real timeout */
    uint16_t zcTimeoutCount;         /* total timeouts (rising + falling) */
    uint16_t diagRisingTimeouts;     /* timeouts on rising ZC steps */
    uint16_t diagFallingTimeouts;    /* timeouts on falling ZC steps */
    uint16_t diagRisingRejects;      /* false ZCs rejected on rising steps */
    uint16_t diagFallingRejects;     /* false ZCs rejected on falling steps */
} ZC_DIAG_T;
```

Add `ZC_CTRL_T zcCtrl;` to `GARUDA_DATA_T`.

**`motor/bemf_zc.c`**

In `BEMF_ZC_Init()`:
- Zero all new ZC_DIAG_T and ZC_CTRL_T fields
- Set `zcCtrl.mode = ZC_MODE_ACQUIRE`

In `BEMF_ZC_CheckTimeout()`:
- Increment `zcDiag.actualForcedComm` when `forcedCountdown == 0`
- Increment `zcDiag.zcTimeoutCount`
- Increment `diagRisingTimeouts` or `diagFallingTimeouts` based on
  `commutationTable[currentStep].zcPolarity`

In `BEMF_ZC_OnCommutation()`:
- Capture `zcCtrl.originalTimeoutHR` = `forcedCountdown × COM_TIMER_T1_NUMER / COM_TIMER_T1_DENOM`
  at commutation time (before countdown starts)

In `RecordZcTiming()`:
- Fix `zcLatencyPct` to use `originalTimeoutHR` instead of current `forcedCountdown`

In `RecordZcTiming()` false-ZC rejection paths:
- Increment `diagRisingRejects` or `diagFallingRejects` based on polarity

**`garuda_service.c`**

At OL→CL entry: set `zcCtrl.mode = ZC_MODE_ACQUIRE` (no behavioral effect yet).

**`gsp/gsp_commands.h`**

Extend `GSP_CK_SNAPSHOT_T` (52 → 58 bytes):
```c
/* Replace forcedSteps with actualForcedComm */
uint8_t  actualForcedComm;    /* was: forcedSteps (stepsSinceLastZc) */
/* Keep existing: zcLatencyPct, zcBlankPct, zcBypassCount */
/* New: */
uint8_t  zcMode;              /* ZC_MODE_ACQUIRE/TRACK/RECOVER */
uint16_t zcTimeoutCount;      /* total ZC timeouts */
uint16_t risingZcCount;       /* rising ZC accepts */
uint16_t fallingZcCount;      /* falling ZC accepts */
```

Note: Keep old `missedSteps` field (consecutiveMissedSteps) — it's still valid.

**`gsp/gsp_snapshot.c`**

Capture new fields. Keep length-guarded for backward compatibility.

**`gui/src/protocol/decode.ts`**, **`types.ts`**

Add new fields to CkSnapshot, length-guarded.

**`tools/motor_sweep_test.py`**, **`tools/pot_capture.py`**

Update parsers. Display `zcMode` as `ACQ`/`TRK`/`RCV`. Display `actualForcedComm` instead
of `forced`. Keep `forced` (stepsSinceLastZc) in CSV for backward analysis but don't show
in live display.

### Validation

- Build all 3 profiles (Hurst, A2212, 2810)
- Flash 2810, run `pot_capture.py` — verify new counters appear and increment correctly
- Verify `actualForcedComm` is low in good runs (< 1/sec)
- Verify per-polarity counters: `risingZcCount ≈ fallingZcCount` or document gap
- Verify `zcMode` shows `ACQ` (stays there since no transitions implemented yet)
- A2212: run motor_sweep_test — no behavioral regression

---

## Phase 2: Mode Shell (No Behavior Change)

**Goal**: Mode state machine exists and transitions, but all modes use the SAME detector logic as today. Observable via telemetry. Zero risk.

**Commit**: One commit.

### Files and Changes

**`motor/bemf_zc.c`**

Add mode transition helpers:
```c
static void ZcEnterAcquire(volatile GARUDA_DATA_T *pData) {
    pData->zcCtrl.mode = ZC_MODE_ACQUIRE;
    pData->zcCtrl.acquireGoodCount = 0;
    /* Don't change filter, advance, timeout — same as today */
}

static void ZcEnterTrack(volatile GARUDA_DATA_T *pData) {
    pData->zcCtrl.mode = ZC_MODE_TRACK;
    /* Don't change anything yet — same detector */
}

static void ZcEnterRecover(volatile GARUDA_DATA_T *pData) {
    pData->zcCtrl.mode = ZC_MODE_RECOVER;
    pData->zcCtrl.recoverGoodCount = 0;
    pData->zcCtrl.recoverAttempts++;
    /* Don't change anything yet — same detector */
}
```

Add transition logic (observation only — fires transitions but behavior unchanged):

In `RecordZcTiming()` (fast poll path), after a good ZC:
```c
if (pData->zcCtrl.mode == ZC_MODE_ACQUIRE) {
    pData->zcCtrl.acquireGoodCount++;
    if (pData->zcCtrl.acquireGoodCount >= ZC_ACQUIRE_GOOD_ZC)
        ZcEnterTrack(pData);
} else if (pData->zcCtrl.mode == ZC_MODE_RECOVER) {
    pData->zcCtrl.recoverGoodCount++;
    if (pData->zcCtrl.recoverGoodCount >= ZC_RECOVER_GOOD_ZC)
        ZcEnterAcquire(pData);
}
```

In `BEMF_ZC_CheckTimeout()`, on timeout:
```c
if (pData->zcCtrl.mode == ZC_MODE_TRACK)
    ZcEnterRecover(pData);
/* ACQUIRE and RECOVER: stay in current mode on timeout */
```

In per-revolution desync check (existing), on desync:
```c
if (pData->zcCtrl.mode == ZC_MODE_TRACK)
    ZcEnterRecover(pData);
```

**`garuda_service.c`**

At OL→CL entry: call `ZcEnterAcquire()`.

**`garuda_config.h`**

Add per-profile knobs (shared section):
```c
#define ZC_ACQUIRE_GOOD_ZC      20U   /* ZCs to exit ACQUIRE → TRACK */
#define ZC_RECOVER_GOOD_ZC      10U   /* ZCs to exit RECOVER → ACQUIRE */
#define ZC_RECOVER_MAX_ATTEMPTS  5U   /* RECOVER entries before desync fault */
```

### Validation

- Build all 3 profiles
- 2810 pot_capture: verify `zcMode` transitions ACQ→TRK during normal operation
- 2810 pot_capture with fast pot: verify `zcMode` shows TRK→RCV on the desync event
- Verify behavior is **identical** to Phase 1 — the mode changes but nothing else does
- A2212: no regression

---

## Phase 3: Remove Bypass from TRACK

**Goal**: The critical change. In ZC_TRACK, never accept wrong-polarity ZC. This is what AM32/ESCape32 do.

**Commit**: One commit. This is the highest-risk change.

### Files and Changes

**`motor/bemf_zc.c`**, in `BEMF_ZC_FastPoll()`:

Replace:
```c
uint8_t expected = pData->bemf.cmpExpected;
if (pData->timing.stepsSinceLastZc >= 2)
{
    expected = cmp;
    if (pData->icZc.pollFilter == 0)
        pData->zcDiag.diagBypassAccepted++;
}
```

With:
```c
uint8_t expected = pData->bemf.cmpExpected;
/* No polarity bypass in any mode. AM32/ESCape32 never do this.
 * If ZC can't be detected with correct polarity, the timeout fires
 * and the mode state machine handles recovery. */
```

Remove `diagBypassAccepted` increment (counter stays in struct for CSV compatibility but
will always be 0).

Remove bypass desync detection code from `garuda_service.c` (the `bypassCheckDiv` /
`prevBypassCount` block) — it's no longer needed.

### Validation

- Build all 3 profiles
- **Critical test**: 2810 pot_capture with slow pot — motor MUST reach CL and track
  - If motor stalls at idle: the ATA6847 comparator truly can't detect one polarity.
    → Fall back: add AM32-style polling mode in RECOVER with per-polarity thresholds.
    → This is the contingency plan, not the default.
  - If motor works: the bypass was never needed. Every cascade was a software bug.
- 2810 fast pot: should see TRACK→RECOVER→ACQUIRE cycle instead of cascade
- A2212: no regression
- Check `actualForcedComm` rate — may be higher than before (timeouts where bypass
  would have accepted). This is correct behavior.

---

## Phase 4: Protected Estimator

**Goal**: Separate raw interval from protected reference. Prevent one false short interval from collapsing scheduling.

**Commit**: One commit.

### Files and Changes

**`garuda_types.h`**

The `refIntervalHR` and `rawIntervalHR` are already in `ZC_CTRL_T` from Phase 1.

**`motor/bemf_zc.c`**, in `RecordZcTiming()`:

Refactor into three stages:
1. **Candidate validation**: existing bounds checks (50% checkpoint, 1.5× max)
2. **Raw capture**: `rawIntervalHR = hrTick - lastZcTickHR`
3. **Protected reference update**:
   ```c
   uint16_t clampedHR = rawIntervalHR;
   uint16_t minRef = refIntervalHR - (refIntervalHR >> 2);  // max 25% shrink
   if (clampedHR < minRef) clampedHR = minRef;
   uint16_t maxRef = refIntervalHR + (refIntervalHR >> 1);  // max 50% growth
   if (clampedHR > maxRef) clampedHR = maxRef;
   refIntervalHR = (uint16_t)(((uint32_t)refIntervalHR * 3 + clampedHR) >> 2);
   ```
4. **Checkpoint**: update only after 6 consecutive good ZCs (one electrical revolution)

Same for Timer1 path (`refIntervalT1`).

**`motor/bemf_zc.c`**, in `BEMF_ZC_ScheduleCommutation()`:

Replace:
```c
uint16_t sp = pData->timing.stepPeriod;
if (pData->timing.zcInterval > 0 && pData->timing.prevZcInterval > 0) {
    uint16_t avgInterval = (...) / 2;
    if (avgInterval < sp) sp = avgInterval;  // ← THIS IS DANGEROUS
}
```

With:
```c
uint16_t sp = pData->zcCtrl.refIntervalHR;  // schedule from protected ref only
/* No min-of-recent shortcut. Acceleration comes from speed governor (Phase 9). */
```

Same for HR scheduling path.

**`garuda_service.c`**

At OL→CL entry: seed `zcCtrl.refIntervalHR` from Timer1 step period (same as existing
`stepPeriodHR` seeding).

### Validation

- 2810 fast pot: verify `refIntervalHR` does NOT halve on desync. Should stay within 25%
  of previous value.
- 2810 slow pot: verify motor still accelerates to 100k (ref tracks real speed, just slower)
- Check that commutation scheduling feels smooth (not jerky from clamped ref)
- A2212: no regression

---

## Phase 5: Explicit Scan Window

**Goal**: Replace blanking-percentage model with structured scan windows. Closer to BLHeli/Bluejay.

**Commit**: One commit.

### Files and Changes

**`garuda_types.h`** or `motor/bemf_zc.c` (local struct):

```c
typedef struct {
    uint16_t switchBlankHR;   // absolute post-commutation blank
    uint16_t scanStartHR;     // earliest ZC candidate acceptance
    uint16_t scanTimeoutHR;   // forced comm if no ZC by here
    uint16_t commDelayHR;     // ZC → next commutation
} ZC_WINDOW_T;
```

Add `ZC_WINDOW_T window;` to `IC_ZC_STATE_T`.

**`motor/bemf_zc.c`**, in `BEMF_ZC_OnCommutation()`:

Replace adaptive blanking computation with window computation:
```c
/* t_switch_blank: absolute hardware floor */
window.switchBlankHR = ZC_BLANK_FLOOR_HR;

/* t_scan_start: later of switch_blank or fraction of ref interval */
uint16_t scanPctDelay;
switch (pData->zcCtrl.mode) {
    case ZC_MODE_ACQUIRE: scanPctDelay = refIntervalHR * ZC_SCAN_DELAY_PCT_ACQUIRE / 100; break;
    case ZC_MODE_RECOVER: scanPctDelay = refIntervalHR * ZC_SCAN_DELAY_PCT_ACQUIRE / 100; break;
    default:              scanPctDelay = refIntervalHR * ZC_SCAN_DELAY_PCT_TRACK / 100; break;
}
window.scanStartHR = (scanPctDelay > window.switchBlankHR) ? scanPctDelay : window.switchBlankHR;

/* t_scan_timeout: mode-dependent multiplier */
uint8_t tmult;
switch (pData->zcCtrl.mode) {
    case ZC_MODE_ACQUIRE: tmult = ZC_SCAN_TIMEOUT_MULT_ACQUIRE; break;
    case ZC_MODE_RECOVER: tmult = ZC_SCAN_TIMEOUT_MULT_RECOVER; break;
    default:              tmult = ZC_SCAN_TIMEOUT_MULT_TRACK; break;
}
window.scanTimeoutHR = refIntervalHR * tmult;

/* t_comm_delay: half-interval minus advance */
window.commDelayHR = refIntervalHR / 2 - advanceHR;
```

**`motor/bemf_zc.c`**, in `BEMF_ZC_FastPoll()`:

Replace blanking check + half-interval check with:
```c
uint16_t elapsed = HAL_ComTimer_ReadTimer() - pData->icZc.lastCommHR;

/* Before scan window: wait */
if ((int16_t)(elapsed - pData->icZc.window.scanStartHR) < 0)
    return false;

/* Past timeout: handled by CheckTimeout */

/* Within scan window: apply polarity check + deglitch (existing) */
```

**`garuda_config.h`**

Add per-profile window knobs:
```c
#define ZC_SCAN_DELAY_PCT_TRACK     10U   /* 10% of ref interval */
#define ZC_SCAN_DELAY_PCT_ACQUIRE   15U   /* 15% — more conservative at startup */
#define ZC_SCAN_TIMEOUT_MULT_TRACK   2U   /* 2× ref interval */
#define ZC_SCAN_TIMEOUT_MULT_ACQUIRE 3U   /* 3× — wider window for acquisition */
#define ZC_SCAN_TIMEOUT_MULT_RECOVER 4U   /* 4× — widest for recovery */
```

Keep existing adaptive blanking knobs alive until window model is proven. Remove them
in a cleanup commit after bench validation.

### Validation

- Verify scan window timing across speed range (check via zcLatencyPct)
- Compare to Phase 4 runs — should be similar or better
- A2212: no regression

---

## Phase 6: Demag Classification

**Goal**: Make demag an observable state, not an inferred side effect of blanking.

**Commit**: One commit.

### Files and Changes

**`motor/bemf_zc.c`**, in `BEMF_ZC_FastPoll()`:

In the early scan window region (after `scanStartHR` but before `refIntervalHR / 2`):
```c
if (elapsed < refIntervalHR / 2) {
    if (cmp == expected && pData->icZc.pollFilter < filterLevel) {
        /* Comparator shows expected state but can't pass deglitch →
         * probably demag tail masquerading as valid BEMF */
        pData->zcCtrl.demagMetric++;
    }
}
```

Decay on clean ZC acceptance:
```c
if (pData->zcCtrl.demagMetric > 0)
    pData->zcCtrl.demagMetric--;
```

Use in mode transitions:
```c
if (pData->zcCtrl.demagMetric > ZC_DEMAG_RECOVER_THRESH)
    ZcEnterRecover(pData);
```

**`garuda_config.h`**

```c
#define ZC_DEMAG_INC             1U
#define ZC_DEMAG_DEC             1U
#define ZC_DEMAG_RECOVER_THRESH 20U   /* demag events before RECOVER */
```

### Validation

- Monitor `demagMetric` in telemetry during 2810 runs
- Verify demag doesn't trigger RECOVER during normal operation
- Verify demag DOES trigger RECOVER during demag-heavy conditions (high duty, low speed)

---

## Phase 7: Recovery Rewrite

**Goal**: Make recovery explicit, debuggable, and mode-specific.

**Commit**: One commit.

### Files and Changes

**`motor/bemf_zc.c`**, in `BEMF_ZC_CheckTimeout()`:

Mode-specific timeout behavior:
```c
switch (pData->zcCtrl.mode) {
    case ZC_MODE_ACQUIRE:
        /* Conservative: wider timeout already set by window. Just count. */
        break;

    case ZC_MODE_TRACK:
        /* First timeout → enter RECOVER immediately */
        ZcEnterRecover(pData);
        break;

    case ZC_MODE_RECOVER:
        /* Expand period conservatively (existing 12.5% increase) */
        pData->timing.stepPeriod += pData->timing.stepPeriod >> 3;
        /* Update refIntervalHR to match */
        pData->zcCtrl.refIntervalHR += pData->zcCtrl.refIntervalHR >> 3;
        /* After repeated failures: desync */
        if (pData->zcCtrl.recoverAttempts >= ZC_RECOVER_MAX_ATTEMPTS)
            return ZC_TIMEOUT_DESYNC;
        break;
}
```

Move existing forced-step deceleration logic under RECOVER only (currently applies
unconditionally when `!zcSynced`).

**`garuda_service.c`**

Mode-specific duty behavior in CL:
```c
switch (gData.zcCtrl.mode) {
    case ZC_MODE_ACQUIRE:
        /* Cap duty at ramp exit level — don't accelerate during acquisition */
        if (target > rampExitDuty) target = rampExitDuty;
        break;
    case ZC_MODE_TRACK:
        /* Normal duty mapping */
        break;
    case ZC_MODE_RECOVER:
        /* Hold current duty — don't change during recovery */
        target = slewedDuty;
        break;
}
```

### Validation

- 2810: verify RECOVER mode produces stable low-current state (not 34A)
- 2810: verify RECOVER→ACQUIRE→TRACK cycle completes after disturbance
- 2810: verify desync fault triggers after ZC_RECOVER_MAX_ATTEMPTS
- A2212: no regression

---

## Phase 8: Per-Polarity Instrumentation

**Goal**: Hard evidence for polarity asymmetry. No tuning changes yet.

**Commit**: One commit.

### Files and Changes

Most counters already exist from Phase 1. Add:

**`motor/bemf_zc.c`**

Per-polarity average latency (optional, if RAM allows):
```c
/* In RecordZcTiming, accumulate: */
if (zcPolarity > 0) {
    risingLatencySum += latencyHR;
    risingLatencyCount++;
} else {
    fallingLatencySum += latencyHR;
    fallingLatencyCount++;
}
```

Expose in snapshot if space allows (optional — can also compute offline from CSV).

### Validation

- Run 2810 at mid-speed, capture 60+ seconds of data
- Compare `risingZcCount` vs `fallingZcCount`
- Compare `risingTimeouts` vs `fallingTimeouts`
- If asymmetry > 20%: document and plan per-polarity tuning
- If symmetric: polarity asymmetry hypothesis is rejected

---

## Phase 9: Speed Governance

**Goal**: Prevent motor from accelerating faster than detector can follow. Phase-late on purpose — must not mask detector bugs.

**Commit**: One commit.

### Files and Changes

**`garuda_service.c`**

ESCape32-style duty cap by eRPM:
```c
uint32_t measuredErpm;
if (gData.timing.stepPeriodHR > 0)
    measuredErpm = 15625000UL / gData.timing.stepPeriodHR;
else
    measuredErpm = 0;

uint32_t maxDuty;
if (measuredErpm < DUTY_RAMP_ERPM)
    maxDuty = CL_IDLE_DUTY + (uint32_t)(MAX_DUTY - CL_IDLE_DUTY)
              * measuredErpm / DUTY_RAMP_ERPM;
else
    maxDuty = MAX_DUTY;

if (target > maxDuty) target = maxDuty;
```

**`garuda_config.h`** (per-profile):
```c
/* 2810: duty reaches 100% at 80k eRPM */
#define DUTY_RAMP_ERPM  80000U
```

### Validation

- 2810 fast pot: motor should accelerate smoothly without desync
- 2810: verify full speed range reachable with slow pot
- A2212: no regression (set DUTY_RAMP_ERPM high enough)

---

## Commit Order Summary

| # | Commit | Risk | Behavior Change |
|---|--------|------|-----------------|
| 1 | Telemetry semantics + scaffolding | None | No |
| 2 | Mode shell (observe only) | None | No |
| 3 | **Remove bypass from TRACK** | **High** | **Yes — the critical fix** |
| 4 | Protected estimator | Low | Subtle — scheduling changes |
| 5 | Explicit scan window | Medium | Replaces blanking model |
| 6 | Demag classification | Low | New mode transition trigger |
| 7 | Recovery rewrite | Medium | Mode-specific timeout/duty |
| 8 | Per-polarity instrumentation | None | No |
| 9 | Speed governance | Low | Limits acceleration rate |

---

## Validation Gates

After EACH commit, run:

| Test | Tool | Pass Criteria |
|------|------|---------------|
| Build all profiles | `make` with MOTOR_PROFILE=0,1,2 | Clean, no warnings |
| A2212 sweep | `motor_sweep_test.py --max-pct 50` | Same as commit `7622dad` |
| 2810 slow pot | `pot_capture.py` | Stable to 100k, low current |
| 2810 fast pot | `pot_capture.py` | No 34A cascade |
| 2810 start/stop ×5 | Manual SW1/SW2 | Every CL entry succeeds |
| Telemetry | Check CSV | New fields populated correctly |

### Final Acceptance (after Phase 9)

| Test | Pass Criteria |
|------|---------------|
| 2810 fast pot sweep ×10 | Zero cascades, TRACK→RECOVER→ACQUIRE cycle works |
| 2810 with prop, full range | Stable, recoverable |
| Per-polarity balance | rising ≈ falling (within 20%) or documented |
| `actualForcedComm` | < 1/sec in steady state |
| `zcMode` telemetry | Shows transitions, no stuck states |
| Max current at 100k eRPM | < 5A phase current (was 15A) |

---

## Config Knobs (all phases)

Added to `garuda_config.h` shared section with per-profile overrides where noted:

```c
/* Phase 2: Mode thresholds */
#define ZC_ACQUIRE_GOOD_ZC           20U
#define ZC_RECOVER_GOOD_ZC           10U
#define ZC_RECOVER_MAX_ATTEMPTS       5U

/* Phase 4: Estimator protection */
#define ZC_REF_SHRINK_LIMIT_PCT      25U   /* max shrink per ZC */
#define ZC_REF_GROW_LIMIT_PCT        50U   /* max growth per ZC */

/* Phase 5: Scan window */
#define ZC_SCAN_DELAY_PCT_TRACK      10U
#define ZC_SCAN_DELAY_PCT_ACQUIRE    15U
#define ZC_SCAN_TIMEOUT_MULT_TRACK    2U
#define ZC_SCAN_TIMEOUT_MULT_ACQUIRE  3U
#define ZC_SCAN_TIMEOUT_MULT_RECOVER  4U

/* Phase 6: Demag */
#define ZC_DEMAG_INC                  1U
#define ZC_DEMAG_DEC                  1U
#define ZC_DEMAG_RECOVER_THRESH      20U

/* Phase 9: Speed governance (per-profile) */
#define DUTY_RAMP_ERPM           80000U   /* 2810 */
```

---

## File Change Map

| File | Phases | Total Changes |
|------|--------|---------------|
| `garuda_types.h` | 1,2,4,5 | ZC_MODE_T, ZC_CTRL_T, ZC_DIAG_T, ZC_WINDOW_T, raw/ref split |
| `motor/bemf_zc.c` | 1-8 | Mode machine, bypass removal, windowed scan, protected estimator, demag |
| `motor/bemf_zc.h` | 2 | New mode API |
| `garuda_service.c` | 1,2,7,9 | CL entry → ACQUIRE, mode-specific duty, speed governor |
| `garuda_config.h` | 2,5,6,9 | Mode thresholds, window params, demag, duty ramp |
| `gsp/gsp_commands.h` | 1 | Snapshot expansion |
| `gsp/gsp_snapshot.c` | 1 | Capture new fields |
| `gui/src/protocol/decode.ts` | 1 | CkSnapshot decoder |
| `gui/src/protocol/types.ts` | 1 | CkSnapshot type |
| `tools/motor_sweep_test.py` | 1 | Parser + display |
| `tools/pot_capture.py` | 1 | Parser + display |
