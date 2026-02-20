/**
 * @file startup.c
 *
 * @brief Motor alignment and open-loop forced commutation ramp.
 *
 * STARTUP_Align(): Holds the first commutation step at low duty for
 *   ALIGN_TIME_MS to lock the rotor position.
 *
 * STARTUP_OpenLoopRamp(): Forces commutation with decreasing step period
 *   and increasing duty. Returns true when stepPeriod <= MIN_STEP_PERIOD
 *   (ready to transition to closed-loop).
 *
 * Component: STARTUP
 */

#include "startup.h"
#include "commutation.h"
#include "../garuda_calc_params.h"
#include "../hal/hal_pwm.h"

#if FEATURE_SINE_STARTUP
/* 256-entry sine table, Q15 format: sin(2*pi*i/256) * 32767.
 * 512 bytes flash. Indices 0..255 map to 0..~360 degrees. */
static const int16_t sineTable256[256] = {
        0,    804,   1608,   2410,   3212,   4011,   4808,   5602,
     6393,   7179,   7962,   8739,   9512,  10278,  11039,  11793,
    12539,  13279,  14010,  14732,  15446,  16151,  16846,  17530,
    18204,  18868,  19519,  20159,  20787,  21403,  22005,  22594,
    23170,  23731,  24279,  24811,  25329,  25832,  26319,  26790,
    27245,  27683,  28105,  28510,  28898,  29268,  29621,  29956,
    30273,  30571,  30852,  31113,  31356,  31580,  31785,  31971,
    32137,  32285,  32412,  32521,  32609,  32678,  32728,  32757,
    32767,  32757,  32728,  32678,  32609,  32521,  32412,  32285,
    32137,  31971,  31785,  31580,  31356,  31113,  30852,  30571,
    30273,  29956,  29621,  29268,  28898,  28510,  28105,  27683,
    27245,  26790,  26319,  25832,  25329,  24811,  24279,  23731,
    23170,  22594,  22005,  21403,  20787,  20159,  19519,  18868,
    18204,  17530,  16846,  16151,  15446,  14732,  14010,  13279,
    12539,  11793,  11039,  10278,   9512,   8739,   7962,   7179,
     6393,   5602,   4808,   4011,   3212,   2410,   1608,    804,
        0,   -804,  -1608,  -2410,  -3212,  -4011,  -4808,  -5602,
    -6393,  -7179,  -7962,  -8739,  -9512, -10278, -11039, -11793,
   -12539, -13279, -14010, -14732, -15446, -16151, -16846, -17530,
   -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
   -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790,
   -27245, -27683, -28105, -28510, -28898, -29268, -29621, -29956,
   -30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971,
   -32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757,
   -32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285,
   -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
   -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683,
   -27245, -26790, -26319, -25832, -25329, -24811, -24279, -23731,
   -23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868,
   -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
   -12539, -11793, -11039, -10278,  -9512,  -8739,  -7962,  -7179,
    -6393,  -5602,  -4808,  -4011,  -3212,  -2410,  -1608,   -804
};
#endif

/**
 * @brief Initialize startup parameters.
 * Called when entering ESC_ALIGN state.
 */
void STARTUP_Init(volatile GARUDA_DATA_T *pData)
{
    pData->currentStep = 0;
    pData->alignCounter = ALIGN_TIME_COUNTS;
    pData->rampStepPeriod = INITIAL_STEP_PERIOD;
    pData->rampCounter = INITIAL_STEP_PERIOD;
    pData->duty = ALIGN_DUTY;

    /* Clear stale timing state from any previous run. STARTUP_Init is called
     * at every ESC_ALIGN entry (including desync recovery restarts). Without
     * this, zcSynced/goodZcCount survive from a prior run and can trigger
     * stale hot-handoff paths or bypass pre-sync. */
    pData->timing.zcSynced = false;
    pData->timing.goodZcCount = 0;
    pData->timing.risingZcWorks = false;
    pData->timing.fallingZcWorks = false;
    pData->timing.deadlineActive = false;
    pData->timing.stepsSinceLastZc = 0;
    pData->timing.hasPrevZc = false;

#if FEATURE_SINE_STARTUP
    STARTUP_SineInit(pData);
#endif
}

/**
 * @brief Hold alignment position at low duty.
 * Called every Timer1 tick (100us) while in ESC_ALIGN state.
 *
 * @return true when alignment is complete
 */
bool STARTUP_Align(volatile GARUDA_DATA_T *pData)
{
    if (pData->alignCounter == ALIGN_TIME_COUNTS)
    {
        /* First call — apply step 0 and set duty */
        HAL_PWM_SetCommutationStep(0);
        HAL_PWM_SetDutyCycle(ALIGN_DUTY);
    }

    if (pData->alignCounter > 0)
    {
        pData->alignCounter--;
        return false;
    }

    return true; /* Alignment complete */
}

/**
 * @brief Open-loop forced commutation ramp.
 * Called every Timer1 tick (100us) while in ESC_OL_RAMP state.
 * Advances commutation steps at decreasing intervals and gradually
 * increases duty cycle.
 *
 * Acceleration is driven by RAMP_ACCEL_ERPM_PER_S:
 *   Each commutation step spans stepPeriod * 100us seconds.
 *   delta_eRPM = ACCEL * stepPeriod / 10000
 *   new_eRPM = current_eRPM + delta_eRPM
 *   new_stepPeriod = 100000 / new_eRPM
 *
 * @return true when ramp target speed is reached
 */
bool STARTUP_OpenLoopRamp(volatile GARUDA_DATA_T *pData)
{
    /* Decrement the step counter */
    if (pData->rampCounter > 0)
    {
        pData->rampCounter--;
    }

    /* Time to advance to next commutation step */
    if (pData->rampCounter == 0)
    {
        COMMUTATION_AdvanceStep(pData);

        /* Current-gated acceleration: only decrease step period (speed up)
         * when bus current is below the gate threshold. High forward current
         * means the rotor is lagging — accelerating further will brake it.
         * Low/zero current (ibusRaw near or below 2048 bias) is normal at
         * low duty — ADC mostly samples during PWM OFF time. Allow accel. */
#if RAMP_CURRENT_GATE_MA > 0 && FEATURE_HW_OVERCURRENT
        if (pData->ibusRaw < RAMP_CURRENT_GATE_ADC)
        {
#endif
        /* Compute new step period from RAMP_ACCEL_ERPM_PER_S */
        uint32_t curPeriod = pData->rampStepPeriod;
        if (curPeriod == 0) curPeriod = INITIAL_STEP_PERIOD;
        uint32_t curErpm = 100000UL / curPeriod;
        uint32_t deltaErpm = ((uint32_t)RAMP_ACCEL_ERPM_PER_S * curPeriod) / 10000UL;
        if (deltaErpm < 1) deltaErpm = 1;
        uint32_t newErpm = curErpm + deltaErpm;
        uint32_t newPeriod = 100000UL / newErpm;
        if (newPeriod < MIN_STEP_PERIOD)
            newPeriod = MIN_STEP_PERIOD;
        pData->rampStepPeriod = newPeriod;
#if RAMP_CURRENT_GATE_MA > 0 && FEATURE_HW_OVERCURRENT
        }
#endif

        pData->rampCounter = pData->rampStepPeriod;

        /* Gradually increase duty toward RAMP_DUTY_CAP.
         * Increment ~0.5% of LOOPTIME per step for smooth torque ramp. */
        if (pData->duty < RAMP_DUTY_CAP)
        {
            pData->duty += (LOOPTIME_TCY / 200);
            if (pData->duty > RAMP_DUTY_CAP)
                pData->duty = RAMP_DUTY_CAP;
        }
        HAL_PWM_SetDutyCycle(pData->duty);
    }

    /* Check if we've reached the target speed */
    if (pData->rampStepPeriod <= MIN_STEP_PERIOD)
    {
        return true; /* Ready for closed-loop transition */
    }

    return false;
}

#if FEATURE_SINE_STARTUP
/* Helper: look up sine table for a Q16 angle, return Q15 value (-32767..+32767) */
static inline int16_t sineLookup(uint16_t angle)
{
    /* Top 8 bits of Q16 angle select the 256-entry table index */
    return sineTable256[angle >> 8];
}

/**
 * @brief Initialize sine startup state.
 * Called from STARTUP_Init when FEATURE_SINE_STARTUP=1.
 * Seeds angle at 90 deg (Phase A peak), zero rotation speed.
 * Pre-computes initial 3-phase duties, writes them, then releases
 * all PWM overrides so complementary mode drives all 3 phases.
 * Sets sine.active = true last (ADC ISR gate).
 *
 * NOTE: pData->alignCounter is already set by STARTUP_Init before
 * this is called — reused, not duplicated (Fix 6).
 */
void STARTUP_SineInit(volatile GARUDA_DATA_T *pData)
{
    pData->sine.angle = SINE_ALIGN_ANGLE_Q16;
    pData->sine.angleIncrement = 0;
    pData->sine.amplitude = SINE_MIN_AMPLITUDE;
    pData->sine.erpmFrac = (uint32_t)INITIAL_ERPM << 16;

    /* Pre-compute and write 3-phase duties before releasing overrides */
    uint32_t dA, dB, dC;
    STARTUP_SineComputeDuties(pData, &dA, &dB, &dC);
    HAL_PWM_SetDutyCycle3Phase(dA, dB, dC);

    /* Release all overrides — complementary mode on all 3 phases */
    HAL_PWM_ReleaseAllOverrides();

    /* Gate: ADC ISR may now run sine waveform updates */
    pData->sine.active = true;
}

/**
 * @brief Sine alignment phase — hold rotor at fixed angle.
 * Called from Timer1 ISR in ESC_ALIGN state.
 * Counts down pData->alignCounter (shared with trap path, Fix 6).
 * On completion: seeds angleIncrement for INITIAL_ERPM.
 *
 * @return true when alignment is complete
 */
bool STARTUP_SineAlign(volatile GARUDA_DATA_T *pData)
{
    if (pData->alignCounter > 0)
    {
        pData->alignCounter--;
        return false;
    }

    /* Alignment done — seed initial rotation speed */
    pData->sine.angleIncrement =
        (uint16_t)(((uint32_t)INITIAL_ERPM * SINE_ERPM_TO_ANGLE_Q16) >> 16);

    return true;
}

/**
 * @brief Sine V/f ramp — accelerate from INITIAL_ERPM to RAMP_TARGET_ERPM.
 * Called from Timer1 ISR in ESC_OL_RAMP state at 10 kHz.
 * Accumulates eRPM via Q16 fractional accumulator.
 * Amplitude scales linearly with frequency (V/f constant).
 *
 * @return true when target speed is reached
 */
bool STARTUP_SineRamp(volatile GARUDA_DATA_T *pData)
{
    /* Accumulate fractional eRPM */
    pData->sine.erpmFrac += SINE_ERPM_RAMP_RATE_Q16;
    uint32_t currentErpm = pData->sine.erpmFrac >> 16;

    /* Clamp at target */
    if (currentErpm >= RAMP_TARGET_ERPM)
        currentErpm = RAMP_TARGET_ERPM;

    /* Update angle increment for current eRPM (keep spinning during fadeout) */
    pData->sine.angleIncrement =
        (uint16_t)(((uint32_t)currentErpm * SINE_ERPM_TO_ANGLE_Q16) >> 16);

    /* V/f amplitude ramp: linear interpolation from MIN to MAX */
    if (currentErpm <= INITIAL_ERPM)
    {
        pData->sine.amplitude = SINE_MIN_AMPLITUDE;
    }
    else if (currentErpm >= RAMP_TARGET_ERPM)
    {
        pData->sine.amplitude = SINE_MAX_AMPLITUDE;
    }
    else
    {
        uint32_t range = RAMP_TARGET_ERPM - INITIAL_ERPM;
        uint32_t progress = currentErpm - INITIAL_ERPM;
        pData->sine.amplitude = (uint16_t)(SINE_MIN_AMPLITUDE +
            ((uint32_t)(SINE_MAX_AMPLITUDE - SINE_MIN_AMPLITUDE) * progress) / range);
    }

    return (currentErpm >= RAMP_TARGET_ERPM);
}

/**
 * @brief Compute 3-phase sine duties from current angle.
 * Called from ADC ISR at 24 kHz. Direction-aware.
 * Advances angle, looks up sine for 3 phases at 120 deg offsets,
 * computes duty = CENTER + (sin * amplitude) >> 15.
 *
 * Phase offsets: A = angle, B = angle - 120 deg, C = angle + 120 deg
 * (120 deg = 21845 in Q16, 240 deg = 43691 in Q16)
 */
void STARTUP_SineComputeDuties(volatile GARUDA_DATA_T *pData,
                                uint32_t *dutyA, uint32_t *dutyB, uint32_t *dutyC)
{
    /* Advance angle (direction-aware) */
    if (pData->direction == 0)
        pData->sine.angle += pData->sine.angleIncrement;
    else
        pData->sine.angle -= pData->sine.angleIncrement;

    uint16_t angle = pData->sine.angle;
    uint16_t amp = pData->sine.amplitude;

    /* Sine lookup for each phase (120 deg apart).
     * Commutation table CW sequence is A->C->B, so C leads A by 120 deg
     * and B lags A by 120 deg (i.e. leads by 240 deg). */
    int16_t sinA = sineLookup(angle);
    int16_t sinB = sineLookup(angle + 21845u);  /* +120 deg (= -240 deg, lags) */
    int16_t sinC = sineLookup(angle - 21845u);  /* -120 deg (leads) */

    /* duty = SINE_CENTER_DUTY + (sin * amplitude) >> 15 */
    int32_t dA = (int32_t)SINE_CENTER_DUTY + (((int32_t)sinA * amp) >> 15);
    int32_t dB = (int32_t)SINE_CENTER_DUTY + (((int32_t)sinB * amp) >> 15);
    int32_t dC = (int32_t)SINE_CENTER_DUTY + (((int32_t)sinC * amp) >> 15);

    /* Clamp to valid range */
    if (dA < (int32_t)MIN_DUTY) dA = MIN_DUTY;
    if (dA > (int32_t)MAX_DUTY) dA = MAX_DUTY;
    if (dB < (int32_t)MIN_DUTY) dB = MIN_DUTY;
    if (dB > (int32_t)MAX_DUTY) dB = MAX_DUTY;
    if (dC < (int32_t)MIN_DUTY) dC = MIN_DUTY;
    if (dC > (int32_t)MAX_DUTY) dC = MAX_DUTY;

    *dutyA = (uint32_t)dA;
    *dutyB = (uint32_t)dB;
    *dutyC = (uint32_t)dC;
}

/**
 * @brief Map current sine angle to the nearest 6-step commutation step.
 * Called once during sine->trap transition. Direction-aware, safe 0-5.
 *
 * Uses multiply-then-shift: ((adj * 6) >> 16) always produces 0..5.
 * For CCW: mirrors with (6 - rawStep) % 6.
 *
 * @return commutation step index 0-5
 */
uint8_t STARTUP_SineGetTransitionStep(volatile GARUDA_DATA_T *pData)
{
    uint16_t adj = pData->sine.angle + SINE_PHASE_OFFSET_Q16;
    uint8_t rawStep = (uint8_t)(((uint32_t)adj * 6) >> 16);

    if (pData->direction != 0)
        rawStep = (6 - rawStep) % 6;

    return rawStep;
}

/**
 * @brief Initialize morph state for sine->trap transition.
 * Called on OL_RAMP → MORPH transition.
 * Syncs rampStepPeriod from the sine ramp's actual eRPM.
 */
void STARTUP_MorphInit(volatile GARUDA_DATA_T *pData)
{
    pData->morph.subPhase = MORPH_CONVERGE;
    pData->morph.alpha = 0;
    pData->morph.sectorCount = 0;
    pData->morph.entryTick = pData->systemTick;
    pData->morph.lastZcTick = 0;
    pData->morph.morphZcCount = 0;

    /* Sync rampStepPeriod from the sine ramp's actual eRPM.
     * STARTUP_SineRamp() uses sine.erpmFrac (Q16) internally and never
     * touches rampStepPeriod — it's still at INITIAL_STEP_PERIOD.
     * Without this, TIMER1_TO_ADC_TICKS(rampStepPeriod) would produce
     * a 10× too-slow forced commutation period → instant desync. */
    uint32_t currentErpm = pData->sine.erpmFrac >> 16;
    if (currentErpm < INITIAL_ERPM) currentErpm = INITIAL_ERPM;
    if (currentErpm > RAMP_TARGET_ERPM) currentErpm = RAMP_TARGET_ERPM;
    pData->rampStepPeriod = (uint16_t)((100000UL + currentErpm / 2) / currentErpm);

    uint8_t step = STARTUP_SineGetTransitionStep(pData);
    pData->morph.morphStep = step;
    pData->morph.prevMorphStep = step;
    /* sine.active stays true — angle keeps advancing in SineComputeDuties */
}

/**
 * @brief Detect 60° sector boundary crossing.
 * Called each ADC ISR tick during morph sub-phase A.
 * Recomputes α from sectorCount with rounding.
 *
 * @return true on boundary crossing
 */
bool STARTUP_MorphCheckSectorBoundary(volatile GARUDA_DATA_T *pData)
{
    uint8_t newStep = STARTUP_SineGetTransitionStep(pData);
    if (newStep != pData->morph.morphStep)
    {
        pData->morph.prevMorphStep = pData->morph.morphStep;
        pData->morph.morphStep = newStep;
        pData->morph.sectorCount++;

        /* Compute α from sectorCount with rounding (not repeated add).
         * This guarantees α=256 exactly at MORPH_CONVERGE_SECTORS. */
        uint16_t sc = pData->morph.sectorCount;
        uint16_t a = (uint16_t)((sc * 256u + (MORPH_CONVERGE_SECTORS / 2))
                                / MORPH_CONVERGE_SECTORS);
        if (a > 256) a = 256;
        pData->morph.alpha = a;

        return true;
    }
    return false;
}

/**
 * @brief Compute blended sine/6-step duties during morph convergence.
 * Called at 24kHz during sub-phase A. Direction-aware.
 * Uses sector-coherent trap targets from current sine angle.
 */
void STARTUP_MorphComputeDuties(volatile GARUDA_DATA_T *pData,
                                 uint32_t *dutyA, uint32_t *dutyB, uint32_t *dutyC)
{
    /* Pure sine duties (advances angle normally) */
    uint32_t sineA, sineB, sineC;
    STARTUP_SineComputeDuties(pData, &sineA, &sineB, &sineC);

    /* Get sector from CURRENT angle (after advance by SineComputeDuties).
     * This ensures trap targets are coherent with the sine angle on
     * boundary-crossing ticks. */
    uint8_t stepNow = STARTUP_SineGetTransitionStep(pData);

    /* 6-step target duties for current sector */
    uint32_t trapDuty = ((uint32_t)pData->sine.amplitude
                         * SINE_TRAP_DUTY_NUM + SINE_TRAP_DUTY_DEN / 2)
                        / SINE_TRAP_DUTY_DEN;
    if (trapDuty < MIN_DUTY)    trapDuty = MIN_DUTY;
    if (trapDuty > RAMP_DUTY_CAP) trapDuty = RAMP_DUTY_CAP;

    const COMMUTATION_STEP_T *s = &commutationTable[stepNow];

    uint32_t trapA = (s->phaseA == PHASE_PWM_ACTIVE) ? trapDuty :
                     (s->phaseA == PHASE_LOW) ? MIN_DUTY : SINE_CENTER_DUTY;
    uint32_t trapB = (s->phaseB == PHASE_PWM_ACTIVE) ? trapDuty :
                     (s->phaseB == PHASE_LOW) ? MIN_DUTY : SINE_CENTER_DUTY;
    uint32_t trapC = (s->phaseC == PHASE_PWM_ACTIVE) ? trapDuty :
                     (s->phaseC == PHASE_LOW) ? MIN_DUTY : SINE_CENTER_DUTY;

    /* Blend: duty = sine*(256-α)/256 + trap*α/256 */
    uint16_t a = pData->morph.alpha;
    if (a > 256) a = 256;
    uint16_t inv = 256 - a;

    /* +128 rounding reduces limit-cycle noise at partial α */
    *dutyA = ((sineA * inv + 128) >> 8) + ((trapA * a + 128) >> 8);
    *dutyB = ((sineB * inv + 128) >> 8) + ((trapB * a + 128) >> 8);
    *dutyC = ((sineC * inv + 128) >> 8) + ((trapC * a + 128) >> 8);

    /* Clamp */
    if (*dutyA < MIN_DUTY) *dutyA = MIN_DUTY;
    if (*dutyB < MIN_DUTY) *dutyB = MIN_DUTY;
    if (*dutyC < MIN_DUTY) *dutyC = MIN_DUTY;
    if (*dutyA > MAX_DUTY) *dutyA = MAX_DUTY;
    if (*dutyB > MAX_DUTY) *dutyB = MAX_DUTY;
    if (*dutyC > MAX_DUTY) *dutyC = MAX_DUTY;
}
#endif /* FEATURE_SINE_STARTUP */
