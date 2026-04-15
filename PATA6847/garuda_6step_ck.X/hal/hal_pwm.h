/**
 * @file hal_pwm.h
 * @brief PWM HAL for dsPIC33CK 6-step commutation.
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
void     HAL_PWM_ChargeBootstrap(void);
void     HAL_PWM_ForceAllFloat(void);
void     HAL_PWM_ForceAllLow(void);

/* Single-pulse mode: above SP_ENTER_ERPM, PWM period = sector period.
 * No switching edges mid-sector → clean BEMF for comparator.
 * amplitude controls on-time fraction within the sector. */
void     HAL_PWM_SetSinglePulse(uint16_t sectorPeriodTCY, uint32_t duty);
void     HAL_PWM_ExitSinglePulse(void);
bool     HAL_PWM_IsSinglePulse(void);
void     HAL_PWM_SetSPFlag(bool on);

#define SP_ENTER_ERPM   90000UL
#define SP_EXIT_ERPM    75000UL

#endif
