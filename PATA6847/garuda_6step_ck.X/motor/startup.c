/**
 * @file startup.c
 * @brief Motor alignment and open-loop forced commutation ramp.
 *
 * STARTUP_Align(): Holds step 0 at low duty for ALIGN_TIME_MS
 *   to lock the rotor position.
 *
 * STARTUP_OpenLoopRamp(): Forces commutation with decreasing step
 *   period and increasing duty. Returns true when stepPeriod <=
 *   MIN_STEP_PERIOD (ready for closed-loop transition).
 */

#include "startup.h"
#include "commutation.h"
#include "../hal/hal_pwm.h"
#include "../garuda_config.h"

void STARTUP_Init(volatile GARUDA_DATA_T *pData)
{
    pData->currentStep = 0;
    pData->alignCounter = ALIGN_TIME_COUNTS;
    pData->rampStepPeriod = INITIAL_STEP_PERIOD;
    pData->rampCounter = INITIAL_STEP_PERIOD;
    pData->duty = ALIGN_DUTY;

    /* Clear stale timing state from any previous run */
    pData->timing.zcSynced = false;
    pData->timing.goodZcCount = 0;
    pData->timing.deadlineActive = false;
    pData->timing.stepsSinceLastZc = 0;
    pData->timing.hasPrevZc = false;
    pData->timing.consecutiveMissedSteps = 0;
}

bool STARTUP_Align(volatile GARUDA_DATA_T *pData)
{
    if (pData->alignCounter == ALIGN_TIME_COUNTS)
    {
        /* First call — apply step 0, start with zero duty */
        HAL_PWM_SetCommutationStep(0);
        HAL_PWM_SetDutyCycle(MIN_DUTY);
    }

    if (pData->alignCounter > 0)
    {
        pData->alignCounter--;

        /* Gradually ramp duty during alignment to avoid inrush.
         * Guard: if ALIGN_DUTY <= MIN_DUTY, just hold MIN_DUTY
         * (prevents unsigned underflow on low-Rs motors). */
        uint32_t rampDuty;
        if (ALIGN_DUTY > MIN_DUTY)
        {
            uint32_t elapsed = ALIGN_TIME_COUNTS - pData->alignCounter;
            rampDuty = MIN_DUTY +
                ((uint32_t)(ALIGN_DUTY - MIN_DUTY) * elapsed) / ALIGN_TIME_COUNTS;
        }
        else
        {
            rampDuty = (uint32_t)ALIGN_DUTY;
        }
        pData->duty = rampDuty;
        HAL_PWM_SetDutyCycle(rampDuty);

        return false;
    }

    return true;  /* Alignment complete */
}

bool STARTUP_OpenLoopRamp(volatile GARUDA_DATA_T *pData)
{
    if (pData->rampCounter > 0)
        pData->rampCounter--;

    if (pData->rampCounter == 0)
    {
        COMMUTATION_AdvanceStep(pData);

        /* Compute new step period from acceleration.
         * eRPM = TIMER1_FREQ_HZ * 60 / 6 / stepPeriod = TIMER1_FREQ_HZ * 10 / stepPeriod */
        #define ERPM_CONST ((uint32_t)TIMER1_FREQ_HZ * 10UL)
        uint32_t curPeriod = pData->rampStepPeriod;
        if (curPeriod == 0) curPeriod = INITIAL_STEP_PERIOD;
        uint32_t curErpm = ERPM_CONST / curPeriod;
        uint32_t deltaErpm = ((uint32_t)RAMP_ACCEL_ERPM_S * curPeriod) / TIMER1_FREQ_HZ;
        if (deltaErpm < 1) deltaErpm = 1;
        uint32_t newErpm = curErpm + deltaErpm;
        uint32_t newPeriod = ERPM_CONST / newErpm;
        if (newPeriod < MIN_STEP_PERIOD)
            newPeriod = MIN_STEP_PERIOD;
        pData->rampStepPeriod = newPeriod;

        pData->rampCounter = pData->rampStepPeriod;

        /* Gradually increase duty toward RAMP_DUTY_CAP */
        if (pData->duty < RAMP_DUTY_CAP)
        {
            pData->duty += (LOOPTIME_TCY / 200);
            if (pData->duty > RAMP_DUTY_CAP)
                pData->duty = RAMP_DUTY_CAP;
        }
        HAL_PWM_SetDutyCycle(pData->duty);
    }

    if (pData->rampStepPeriod <= MIN_STEP_PERIOD)
        return true;  /* Ready for closed-loop */

    return false;
}
