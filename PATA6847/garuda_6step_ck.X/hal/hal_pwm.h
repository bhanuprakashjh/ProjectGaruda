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

#endif
