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
