/**
 * @file hal_pwm.h
 *
 * @brief PWM module definitions and interface for 6-step BLDC commutation.
 * Adapted from AN1292 reference — changed to 24kHz, added override control
 * for 6-step commutation, removed SVM/single-shunt logic.
 *
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: PWM
 */

#ifndef _HAL_PWM_H
#define _HAL_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <xc.h>
#include <stdint.h>

#include "clock.h"
#include "../garuda_calc_params.h"

/* Duty cycle register macros */
#define PWM_PDC1                    PG1DCbits.DC
#define PWM_PDC2                    PG2DCbits.DC
#define PWM_PDC3                    PG3DCbits.DC

#define PWM_PHASE1                  PG1PHASEbits.PHASE
#define PWM_PHASE2                  PG2PHASEbits.PHASE
#define PWM_PHASE3                  PG3PHASEbits.PHASE

#define PWM_TRIGA                   PG1TRIGAbits.TRIGA

/* PWM Fault PCI — DISABLED for initial motor testing.
 * Reference uses CMP3 output (PSS=0b11101) as fault source, but our
 * code had RPn/PCI8R (PSS=0b01000) mapped to floating RB11 pin,
 * causing continuous false fault triggers.
 * TODO: Re-enable with correct CMP3 overcurrent detection. */
/* #define ENABLE_PWM_FAULT_PCI */
#define PCI_FAULT_ACTIVE_STATUS     PG1STATbits.FLTACT
#define _PWMInterrupt               _PWM1Interrupt
#define ClearPWMIF()                _PWM1IF = 0
#define EnablePWMIF()               _PWM1IE = 1
#define DisablePWMIF()              _PWM1IE = 0

void InitPWMGenerators(void);
void InitPWMGenerator1(void);
void InitPWMGenerator2(void);
void InitPWMGenerator3(void);
void InitDutyPWM123Generators(void);
void ChargeBootstrapCapacitors(void);

/* 6-step commutation interface */
void HAL_PWM_SetCommutationStep(uint8_t step);
void HAL_PWM_SetDutyCycle(uint32_t duty);

#ifdef __cplusplus
}
#endif

#endif /* _HAL_PWM_H */
