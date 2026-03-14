/**
 * @file hal_opa.h
 * @brief Internal op-amp initialization for current sensing.
 *
 * dsPIC33CK256MP503 has 3 internal op-amps (OPA1, OPA2, OPA3).
 * EV43F54A board uses OPA2 and OPA3 for phase current sensing.
 */
#ifndef HAL_OPA_H
#define HAL_OPA_H

void HAL_OPA_Init(void);
void HAL_OPA_Enable(void);
void HAL_OPA_Disable(void);

#endif
