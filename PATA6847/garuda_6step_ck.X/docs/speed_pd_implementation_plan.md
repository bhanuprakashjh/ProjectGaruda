# Speed PD Controller — Implementation Plan

## Context

ZC V2 Phases 1-4, 7, 9 are complete. The ZC detection architecture is
sound — bypass removed, protected estimator, mode state machine. The
remaining failure is desync when the motor is driven faster than the
estimator can track. The current firmware is open-loop: pot → duty.
AM32 uses a closed-loop speed controller: pot → target_eRPM → PD → duty.

## Dependency Order

Fix structural bugs FIRST, then add the PD loop on top.

---

## Step 1: Fix Structural Bugs (one commit)

### 1a. ACQUIRE no-op in duty path

**File**: `garuda_service.c:559`

Current:
```c
case ZC_MODE_ACQUIRE:
    /* Allow pot-driven duty but don't increase faster than slew */
    break;
```

Fix: cap duty at ramp exit level during ACQUIRE. This prevents
accelerating before the estimator has locked.

```c
case ZC_MODE_ACQUIRE:
    /* Cap duty at ramp exit level — don't accelerate until
     * ACQUIRE → TRACK transition confirms estimator is locked. */
    if (target > rampExitDuty)
        target = rampExitDuty;
    break;
```

Need to capture `rampExitDuty` at CL entry (already have `slewedDuty`
seeded from ramp — use that).

### 1b. Advance eRPM from wrong estimator

**File**: `bemf_zc.c:623` (in `BEMF_ZC_ScheduleCommutation`)

Current:
```c
if (pData->timing.stepPeriodHR > 0 && pData->timing.hasPrevZcHR)
    eRPM = 15625000UL / pData->timing.stepPeriodHR;
```

Fix: use `refIntervalHR` consistently.

```c
if (pData->zcCtrl.refIntervalHR > 0)
    eRPM = 15625000UL / pData->zcCtrl.refIntervalHR;
```

### 1c. ZC_RECOVER_MAX_ATTEMPTS not enforced

**File**: `bemf_zc.c` in `BEMF_ZC_CheckTimeout()`

Add check in RECOVER timeout path:
```c
case ZC_MODE_RECOVER:
    /* ... existing expand logic ... */
    if (pData->zcCtrl.recoverAttempts >= ZC_RECOVER_MAX_ATTEMPTS)
        return ZC_TIMEOUT_DESYNC;
    break;
```

### 1d. Telemetry cleanup

**File**: `gsp_commands.h`
- Rename `forcedSteps` comment to `stepsSinceLastZc (legacy)`
- Change `actualForcedComm` from uint8_t to uint16_t (or keep 8-bit
  but document saturation)
- Fix snapshot size comment (says 48 bytes, actual is 64)

**File**: `gsp_snapshot.c`
- Add comment that `forcedSteps` is legacy and `actualForcedComm`
  is the real signal

### Build and verify
- Build all 3 profiles
- 2810 pot_capture: verify ACQUIRE caps duty
- Verify advance uses refIntervalHR
- A2212 regression

---

## Step 2: Add Speed PD Controller (one commit)

### 2a. Add PD state to garuda_types.h

```c
typedef struct {
    int32_t  override;      /* accumulated duty override (PWM counts) */
    int32_t  lastError;     /* previous error for derivative */
    uint32_t targetErpm;    /* from pot mapping */
} SPEED_PD_T;
```

Add `SPEED_PD_T speedPd;` to `GARUDA_DATA_T`.

### 2b. Add config knobs to garuda_config.h

Per-profile (2810 section):
```c
#define SPEED_PD_KP          5U     /* proportional gain (÷10000) */
#define SPEED_PD_KD         50U     /* derivative gain (÷10000) */
#define SPEED_PD_SCALE   10000UL    /* output divider */
#define MIN_TARGET_ERPM   3000U     /* pot at zero → idle target */
#define MAX_TARGET_ERPM  80000U     /* pot at max → prop-safe max */
#define FEATURE_SPEED_PD     1      /* enable speed PD for 2810 */
```

Shared defaults:
```c
#ifndef FEATURE_SPEED_PD
#define FEATURE_SPEED_PD     0      /* disabled for Hurst/A2212 */
#endif
#ifndef MIN_TARGET_ERPM
#define MIN_TARGET_ERPM   1000U
#endif
#ifndef MAX_TARGET_ERPM
#define MAX_TARGET_ERPM 100000U
#endif
```

### 2c. Implement PD in garuda_service.c

Location: inside CL duty mapping, Timer1 ISR.
Rate: every 20 ticks (1ms) using existing systemTick divider.

```c
#if FEATURE_SPEED_PD
/* Speed PD controller — runs at 1kHz inside 20kHz Timer1 ISR.
 * Accumulated override replaces MapThrottleToDuty as duty target.
 * Feeds INTO existing pipeline: mode limits → governor → slew → PWM. */
{
    static uint16_t pdDiv = 0;
    if (++pdDiv >= 20)  /* 1ms rate */
    {
        pdDiv = 0;

        /* 1. Target from pot */
        uint16_t potVal = gData.gspThrottleActive
                        ? gData.gspThrottleValue : gData.potRaw;
        uint32_t tgtErpm;
        if (potVal < POT_DEADZONE)
            tgtErpm = MIN_TARGET_ERPM;
        else if (potVal > POT_MAX)
            tgtErpm = MAX_TARGET_ERPM;
        else
            tgtErpm = MIN_TARGET_ERPM +
                (uint32_t)(MAX_TARGET_ERPM - MIN_TARGET_ERPM)
                * (potVal - POT_DEADZONE) / (POT_MAX - POT_DEADZONE);
        gData.speedPd.targetErpm = tgtErpm;

        /* 2. Measured speed from protected estimator */
        uint32_t measErpm = 0;
        bool feedbackValid = (gData.zcCtrl.refIntervalHR > 0 &&
                              gData.timing.zcSynced);
        if (feedbackValid)
            measErpm = 15625000UL / gData.zcCtrl.refIntervalHR;

        /* 3. PD computation (only when feedback is valid) */
        if (feedbackValid)
        {
            int32_t error = (int32_t)tgtErpm - (int32_t)measErpm;
            int32_t dError = error - gData.speedPd.lastError;
            gData.speedPd.lastError = error;

            gData.speedPd.override +=
                (error * (int32_t)SPEED_PD_KP +
                 dError * (int32_t)SPEED_PD_KD) / (int32_t)SPEED_PD_SCALE;

            /* Clamp override to valid duty range */
            if (gData.speedPd.override > (int32_t)MAX_DUTY)
                gData.speedPd.override = (int32_t)MAX_DUTY;
            if (gData.speedPd.override < (int32_t)CL_IDLE_DUTY)
                gData.speedPd.override = (int32_t)CL_IDLE_DUTY;
        }
        /* If feedback invalid: freeze override (don't accumulate garbage) */

        target = (uint32_t)gData.speedPd.override;
    }
    else
    {
        /* Between PD updates: hold previous target */
        target = (uint32_t)gData.speedPd.override;
    }
}
#else
    /* Original open-loop: pot → duty */
    target = MapThrottleToDuty(
        gData.gspThrottleActive ? gData.gspThrottleValue : gData.potRaw);
#endif
```

### 2d. Anti-windup resets

In garuda_service.c:

**CL entry** (after `gData.state = ESC_CLOSED_LOOP`):
```c
gData.speedPd.override = (int32_t)gData.duty;  /* seed from ramp exit */
gData.speedPd.lastError = 0;
```

**RECOVER entry** (in ZcEnterRecover or service code):
```c
/* Freeze override at current value — don't change duty during recovery */
/* (override is already at current duty, just stop accumulating) */
gData.speedPd.lastError = 0;  /* reset derivative */
```

**Motor stop** (ESC_IDLE transition):
```c
gData.speedPd.override = 0;
gData.speedPd.lastError = 0;
```

### 2e. Mode interaction

The PD output is the `target` for duty. It still passes through:

```
speedPd.override → target
    ↓
mode limits (ACQUIRE: cap at rampExitDuty, RECOVER: hold)
    ↓
DUTY_RAMP_ERPM governor (cap by measured eRPM)
    ↓
duty slew (DUTY_SLEW_UP/DOWN per tick)
    ↓
HAL_PWM_SetDutyCycle()
```

All existing safety layers remain authoritative.

### Build and verify
- Build all 3 profiles (FEATURE_SPEED_PD=0 default for Hurst/A2212)
- 2810 no-load:
  - Slow pot: motor should track pot smoothly to 100k eRPM
  - Fast pot slam: motor should accelerate gradually (PD limits rate)
  - Pot to zero: motor should decelerate smoothly
- 2810 with props:
  - Full pot: motor should reach max achievable speed and hold
  - Fast pot: no desync (PD won't overcommand duty)
- A2212: no regression (FEATURE_SPEED_PD=0)

---

## Step 3: Tune Gains (bench iteration, no commit until stable)

Start with Kp=5, Kd=50. Observe:

| Symptom | Fix |
|---------|-----|
| Slow response to pot changes | Increase Kp |
| Speed oscillation / hunting | Increase Kd or decrease Kp |
| Overshoot on step input | Increase Kd |
| Can't reach target speed | Check MAX_TARGET_ERPM and DUTY_RAMP_ERPM |
| Jerky at idle | Increase MIN_TARGET_ERPM or add deadband |

Commit final gains once stable on both no-load and prop tests.

---

## Verification Checklist

| Test | Pass Criteria |
|------|---------------|
| Build all profiles | Clean, no warnings |
| 2810 slow pot sweep | Smooth acceleration to 100k, FC=0 |
| 2810 fast pot slam | Gradual acceleration, no desync |
| 2810 pot to zero | Smooth deceleration |
| 2810 props full pot | Reaches max achievable speed, holds stable |
| 2810 props fast sweep | No desync, no OV fault |
| 2810 start/stop ×5 | Every CL entry works |
| ACQUIRE caps duty | Duty held at ramp exit until TRACK |
| RECOVER holds duty | Override frozen during recovery |
| A2212 no-load sweep | No regression |
| Hurst low-speed test | No regression |

---

## File Change Summary

| File | Step | Changes |
|------|------|---------|
| `garuda_types.h` | 2 | Add SPEED_PD_T, add to GARUDA_DATA_T |
| `garuda_service.c` | 1,2 | Fix ACQUIRE duty cap, add PD loop, anti-windup resets |
| `motor/bemf_zc.c` | 1 | Fix advance eRPM source, enforce RECOVER_MAX_ATTEMPTS |
| `garuda_config.h` | 2 | PD gains, target eRPM range, FEATURE_SPEED_PD |
| `gsp/gsp_commands.h` | 1 | Telemetry comments, actualForcedComm size |
| `gsp/gsp_snapshot.c` | 1 | Legacy field comments |

## Timer1 ISR Rate Note

Timer1 ISR runs at **20kHz** (50µs), not 1kHz. The PD loop must run
at a divided rate: every 20 ticks = 1ms. Use a simple counter divider.
Do NOT run PD at 20kHz — the gains would need to be 20× smaller and
the computation would be wasteful.
