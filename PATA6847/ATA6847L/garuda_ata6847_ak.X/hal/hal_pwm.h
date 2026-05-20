/**
 * @file hal_pwm.h
 * @brief PWM HAL for dsPIC33AK 6-step commutation.
 */
#ifndef HAL_PWM_H
#define HAL_PWM_H

#include <stdint.h>
#include <stdbool.h>
#include "../garuda_types.h"

void     HAL_PWM_Init(void);
void     HAL_PWM_EnableOutputs(void);
void     HAL_PWM_DisableOutputs(void);
void     HAL_PWM_SetCommutationStep(uint8_t step);
void     HAL_PWM_SetDutyCycle(uint32_t duty);
void     HAL_PWM_SetDutyCyclePeriod(uint32_t duty, uint16_t per);

/* Last duty value the hardware actually saw, AFTER clamping in
 * HAL_PWM_SetDutyCycle / HAL_PWM_SetDutyCyclePeriod. Use this for
 * telemetry "% of period" so the displayed duty reflects what's
 * really at the gate, not the upstream commanded value. */
extern volatile uint16_t g_pwmActualDuty;

/* Block-commutation flag — when true, the active-phase H gate is
 * driven solid ON via override (no PWM chopping during the active
 * sector). Set/cleared by sector_pi.c TimeTick based on duty-
 * saturation hysteresis. ApplyPhaseState() reads this in the
 * PHASE_PWM_ACTIVE case. */
extern volatile bool g_blockCommActive;
void     HAL_PWM_ChargeBootstrap(void);
void     HAL_PWM_ForceAllFloat(void);
void     HAL_PWM_ForceAllLow(void);

#if FEATURE_FOC_AN1078
/* SVPWM helpers — used by the AN1078 FOC path. The 6-step block
 * commutation code never calls these; they're a separate output mode
 * that drops the override layer and writes three independent duties. */
void     HAL_PWM_ReleaseAllOverrides(void);
void     HAL_PWM_SetDutyFloat3Phase(float da, float db, float dc);
#endif

#endif
