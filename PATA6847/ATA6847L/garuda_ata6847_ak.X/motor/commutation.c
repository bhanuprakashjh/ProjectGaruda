/**
 * @file commutation.c
 * @brief 6-step commutation table.
 *
 * Standard BLDC 6-step commutation sequence:
 *   Step 0: A=PWM,  B=LOW,   C=FLOAT  (A->B, sense C rising)
 *   Step 1: A=FLOAT, B=LOW,  C=PWM    (C->B, sense A falling)
 *   Step 2: A=LOW,   B=FLOAT, C=PWM   (C->A, sense B rising)
 *   Step 3: A=LOW,   B=PWM,  C=FLOAT  (B->A, sense C falling)
 *   Step 4: A=FLOAT, B=PWM,  C=LOW    (B->C, sense A rising)
 *   Step 5: A=PWM,   B=FLOAT, C=LOW   (A->C, sense B falling)
 */

#include "commutation.h"

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
