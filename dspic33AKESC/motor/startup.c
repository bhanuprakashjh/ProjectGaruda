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

        /* Decrease step period (increase speed) */
        /* Acceleration: reduce period by a fixed amount per step
         * RAMP_ACCEL_ERPM_PER_S converts to period decrement:
         * At each step, reduce by: (stepPeriod^2 * ACCEL) / 100000
         * Simplified: decrement by 1 tick per step until min reached */
        if (pData->rampStepPeriod > MIN_STEP_PERIOD + 1)
        {
            pData->rampStepPeriod--;
        }

        pData->rampCounter = pData->rampStepPeriod;

        /* Gradually increase duty from ALIGN_DUTY toward a higher value */
        if (pData->duty < (MAX_DUTY / 4))
        {
            pData->duty += 1;
        }
        HAL_PWM_SetDutyCycle(pData->duty);
    }

    /* Check if we've reached the target speed */
    if (pData->rampStepPeriod <= MIN_STEP_PERIOD + 1)
    {
        return true; /* Ready for closed-loop transition */
    }

    return false;
}
