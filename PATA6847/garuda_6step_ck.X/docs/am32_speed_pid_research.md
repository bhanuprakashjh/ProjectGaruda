# AM32 Speed PID & Commutation Research

Research for implementing speed PI loop in garuda_6step_ck.X.
Source: `/media/bhanu1234/Development/ProjectGaruda/AM32/Src/main.c`

## Speed PID Architecture

### PID Declaration (line 225)
```c
fastPID speedPid = {
    .Kp = 10,        // proportional
    .Ki = 0,          // integral (disabled by default!)
    .Kd = 100,        // derivative (10× higher than Kp)
    .integral_limit = 10000,
    .output_limit = 50000
};
```

Note: Ki=0 by default — AM32 uses **PD control**, not PI. The derivative
term provides damping against speed oscillation.

### Control Variable
- **Input (actual)**: `e_com_time` — electrical commutation period in µs
- **Setpoint (target)**: `target_e_com_time` — from throttle
- **Output**: modifies `input_override` which replaces throttle input

### Throttle → Target Conversion (line 1914)
```c
target_e_com_time = 60000000 / map(adjusted_input, 47, 2047,
    MINIMUM_RPM_SPEED_CONTROL, MAXIMUM_RPM_SPEED_CONTROL) / (motor_poles/2);
```
- Throttle 47-2047 → RPM 500-5000 (configurable)
- RPM → e_com_time: `60000000 / RPM / pole_pairs`
- For 2810 (7PP): RPM 5000 → e_com_time = 60000000/5000/7 = 1714 µs

### PID Execution (line 900)
```c
if(use_speed_control_loop && running){
    input_override += doPidCalculations(&speedPid, e_com_time, target_e_com_time) / 10000;
    if(input_override > 2047) input_override = 2047;
    if(input_override < 0) input_override = 0;
    if(zero_crosses < 100) speedPid.integral = 0;  // startup guard
}
```
- Runs **per commutation** (not per ms)
- PID output is **accumulated** into input_override
- Integral reset until 100 good ZCs (prevents windup during startup)

### How PID Output Becomes Duty
```
pot → target_e_com_time
e_com_time (measured) vs target_e_com_time → PID error
PID output / 10000 → Δinput_override
input_override (0-2047) → duty_cycle via map()
```

## Commutation Scheduling Comparison

### AM32 (line 918-919)
```c
advance = (commutation_interval >> 3) * advance_level;
waitTime = (commutation_interval >> 1) - advance;
```
- advance_level = 0-3 → 0°, 7.5°, 15°, 22.5°
- **Fixed advance** — not speed-dependent
- Simple integer math

### Garuda (bemf_zc.c ScheduleCommutation)
```c
advDeg = lerp(MIN_DEG, MAX_DEG, eRPM, START_ERPM, MAX_ERPM);
delay = sp * (30 - advDeg) / 60;
```
- 2810: 2°-25° over 3000-150000 eRPM range
- **Speed-dependent advance** — adaptive
- More complex but better for wide speed range

### Mathematical Equivalence
Both formulas produce the same result for the same advance angle:
- AM32 level=2: delay = interval/2 - interval/4 = interval × 15/60 ✓
- Garuda advDeg=15: delay = interval × (30-15)/60 = interval × 15/60 ✓

## Key Differences: AM32 vs Garuda

| Aspect | AM32 | Garuda (current) |
|--------|------|------------------|
| Throttle model | pot → target RPM → PD → duty | pot → duty (direct) |
| Speed feedback | e_com_time (per-comm) | refIntervalHR (per-ZC) |
| Advance | Fixed 0-22.5° (4 levels) | Speed-dependent 2-25° |
| IIR | 3:1 on commutation_interval | 3:1 on stepPeriod + refIntervalHR clamp |
| Scheduling | From IIR only | From refIntervalHR (protected) |
| Min-of-recent | No | Removed in Phase 4 |
| Recovery | old_routine polling fallback | ZC_RECOVER mode (expand + hold) |
| Acceleration limit | max_duty_cycle_change + fast_accel | Duty slew (3 counts/tick) |

## What Garuda Needs for Speed PI

### 1. PID Structure
```c
typedef struct {
    int32_t  error;
    int32_t  integral;
    int32_t  lastError;
    int32_t  output;
    uint16_t Kp;       // × 1/10000
    uint16_t Ki;       // × 1/10000
    uint16_t Kd;       // × 1/10000
    int32_t  integralLimit;
    int32_t  outputLimit;
} SPEED_PID_T;
```

### 2. Target Mapping
```c
// In Timer1 ISR (1kHz):
uint32_t targetErpm = map(potRaw, POT_DEADZONE, POT_MAX,
                          MIN_TARGET_ERPM, MAX_TARGET_ERPM);
// Convert to refInterval for comparison:
uint16_t targetIntervalHR = 15625000UL / targetErpm;
```

### 3. PID Execution — CORRECTED (Codex Review)

**Critical: must use accumulated override, NOT direct duty command.**

AM32 accumulates: `input_override += PID_output / 10000`
If Garuda uses `duty = CL_IDLE_DUTY + output`, when error reaches zero
the duty falls to CL_IDLE_DUTY — motor stalls. The accumulated form
maintains the last working duty at steady state.

**Corrected design: feed PID into existing duty target path**

```c
// In Timer1 ISR (1kHz), NOT per-commutation:
// 1. Compute target from pot
uint32_t targetErpm = map(potRaw, POT_DEADZONE, POT_MAX,
                          MIN_TARGET_ERPM, MAX_TARGET_ERPM);

// 2. Get measured speed from latest valid refIntervalHR
uint32_t measuredErpm = 0;
if (gData.zcCtrl.refIntervalHR > 0)
    measuredErpm = 15625000UL / gData.zcCtrl.refIntervalHR;

// 3. PD controller (accumulated override, like AM32)
int32_t error = (int32_t)targetErpm - (int32_t)measuredErpm;
static int32_t lastError = 0;
static int32_t speedOverride = 0;
speedOverride += (error * SPEED_PID_KP +
                  SPEED_PID_KD * (error - lastError)) / 10000;
lastError = error;
// Clamp
if (speedOverride > (int32_t)MAX_DUTY) speedOverride = MAX_DUTY;
if (speedOverride < (int32_t)CL_IDLE_DUTY) speedOverride = CL_IDLE_DUTY;

// 4. Use speedOverride as the duty target (replaces MapThrottleToDuty)
target = (uint32_t)speedOverride;

// 5. Still passes through existing pipeline:
//    - mode-specific limiting (ACQUIRE/RECOVER)
//    - DUTY_RAMP_ERPM governor (if kept)
//    - duty slew
```

**Why accumulated**: At steady state, error=0, derivative=0, so
`speedOverride` stays at its current value — duty holds constant.
This is how AM32 maintains speed.

**Why Timer1 ISR not per-commutation**: Per-comm stops updating when
detector is weakest (RECOVER mode, timeouts). Timer1 always runs,
using the latest valid refIntervalHR as feedback.

### 4. Key Design Decisions (Corrected)
- **Run in Timer1 ISR** (1kHz), not per-commutation — always updates
- **Start with PD (Ki=0)** — avoids integral windup
- **Accumulated override** — maintains steady-state duty (AM32 pattern)
- **Feeds INTO existing duty pipeline** — does not replace mode limits, slew, etc.
- **Uses refIntervalHR as feedback** — protected estimator, always valid
- **Reset speedOverride on CL entry** — seed from ramp exit duty
- **Reset on RECOVER entry** — prevent stale override from overdriving

### 5. Suggested Initial Gains (2810)

Note: gains need tuning because Garuda uses eRPM (not e_com_time like
AM32) and outputs duty counts (not 0-2047 override). Start conservative
and tune on bench.

```c
#define SPEED_PID_KP      5U     // Conservative — lower than AM32's 10
#define SPEED_PID_KI      0U     // PD only initially
#define SPEED_PID_KD     50U     // Conservative — lower than AM32's 100
#define MIN_TARGET_ERPM  3000U   // idle target
#define MAX_TARGET_ERPM 80000U   // max target (prop-safe)
```

### 6. Integration with Existing Duty Pipeline

The PD output replaces `MapThrottleToDuty()` as the `target` source,
but everything downstream stays:

```
pot → target_eRPM → PD accumulated override → target
                                                  ↓
                              mode limits (ACQUIRE/RECOVER)
                                                  ↓
                              DUTY_RAMP_ERPM cap (Phase 9)
                                                  ↓
                              duty slew (DUTY_SLEW_UP/DOWN)
                                                  ↓
                              HAL_PWM_SetDutyCycle()
```

This preserves all existing safety layers while adding speed regulation.

## How This Fixes the Desync Problem

Current: pot → duty → motor accelerates → outpaces estimator → desync

With speed PD:
- pot → target_eRPM (e.g., 50k)
- PD compares measured eRPM to target
- If measured < target: PD increases override (accelerate)
- If measured ≈ target: PD holds override (steady state)
- If measured > target: PD decreases override (decelerate)
- **Motor can't outpace estimator** because PD won't increase duty
  until the measured speed confirms the motor is tracking

If motor stalls or can't reach target (prop load):
- error stays positive (too slow)
- PD output grows but is clamped by MAX_DUTY
- duty saturates — no overdriving
- if motor physically can't go faster, it holds at max achievable speed
