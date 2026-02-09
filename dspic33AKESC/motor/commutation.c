/**
 * @file commutation.c
 *
 * @brief 6-step commutation table and step advance logic.
 *
 * Standard BLDC 6-step commutation sequence:
 *   Step 0: A=PWM,  B=LOW,   C=FLOAT  (A->B, sense C rising)
 *   Step 1: A=FLOAT, B=LOW,  C=PWM    (C->B, sense A falling)
 *   Step 2: A=LOW,   B=FLOAT, C=PWM   (C->A, sense B rising)
 *   Step 3: A=LOW,   B=PWM,  C=FLOAT  (B->A, sense C falling)
 *   Step 4: A=FLOAT, B=PWM,  C=LOW    (B->C, sense A rising)
 *   Step 5: A=PWM,   B=FLOAT, C=LOW   (A->C, sense B falling)
 *
 * Component: COMMUTATION
 */

#include "commutation.h"
#include "../hal/hal_pwm.h"
#include "../hal/hal_adc.h"

#if FEATURE_LEARN_MODULES
#include "../learn/ring_buffer.h"
#include "../learn/learn_service.h"
#include <string.h>
#endif

/* 6-step commutation table */
const COMMUTATION_STEP_T commutationTable[6] =
{
    /* Step 0: A=PWM, B=LOW, C=FLOAT — floating=C, ZC rising */
    { PHASE_PWM_ACTIVE, PHASE_LOW,        PHASE_FLOAT,      2, +1 },
    /* Step 1: A=FLOAT, B=LOW, C=PWM — floating=A, ZC falling */
    { PHASE_FLOAT,      PHASE_LOW,        PHASE_PWM_ACTIVE, 0, -1 },
    /* Step 2: A=LOW, B=FLOAT, C=PWM — floating=B, ZC rising */
    { PHASE_LOW,        PHASE_FLOAT,      PHASE_PWM_ACTIVE, 1, +1 },
    /* Step 3: A=LOW, B=PWM, C=FLOAT — floating=C, ZC falling */
    { PHASE_LOW,        PHASE_PWM_ACTIVE, PHASE_FLOAT,      2, -1 },
    /* Step 4: A=FLOAT, B=PWM, C=LOW — floating=A, ZC rising */
    { PHASE_FLOAT,      PHASE_PWM_ACTIVE, PHASE_LOW,        0, +1 },
    /* Step 5: A=PWM, B=FLOAT, C=LOW — floating=B, ZC falling */
    { PHASE_PWM_ACTIVE, PHASE_FLOAT,      PHASE_LOW,        1, -1 },
};

/**
 * @brief Advance to the next commutation step (0→1→2→3→4→5→0).
 * Updates PWM override registers and selects the BEMF ADC channel
 * for the new floating phase.
 */
void COMMUTATION_AdvanceStep(volatile GARUDA_DATA_T *pData)
{
#if FEATURE_LEARN_MODULES
    /* Build telemetry sample from the COMPLETED step (before advancing) */
    {
        TELEM_SAMPLE_T sample;
        sample.timestamp = pData->systemTick;
        sample.step = pData->currentStep;
        sample.zcPolarity = commutationTable[pData->currentStep].zcPolarity;
        sample.zcLatencyTicks = 0;  /* Phase 2: fill from ZC timing */
        sample.duty = (uint16_t)pData->duty;
        sample.vbusRaw = pData->vbusRaw;
        sample.faultFlags = 0;      /* Phase 2: set from BEMF detection */
        RingBuffer_Write(&telemRing, &sample);
    }
#endif

    pData->currentStep++;
    if (pData->currentStep >= 6)
    {
        pData->currentStep = 0;
    }

    /* Apply the new commutation pattern to PWM overrides */
    HAL_PWM_SetCommutationStep(pData->currentStep);

    /* Select the ADC channel for the floating phase */
    HAL_ADC_SelectBEMFChannel(commutationTable[pData->currentStep].floatingPhase);
}
