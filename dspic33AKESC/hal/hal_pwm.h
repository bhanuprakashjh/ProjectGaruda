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

/* PWM Fault PCI — board-level OC+OV fault via DIM040/RP28/RB11.
 * MCLV-48V-300W has external comparators (U25A OV, U25B OC) combined
 * through AND gate U27 into M1_FAULT_OC_OV → DIM:040 → RP28.
 * PCI8R=28 mapped in port_config.c. PSS=0b01000 (RPn). PPS=1 (active-low). */
#define ENABLE_PWM_FAULT_PCI
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

#if FEATURE_SINE_STARTUP
void HAL_PWM_SetDutyCycle3Phase(uint32_t dutyA, uint32_t dutyB, uint32_t dutyC);
void HAL_PWM_ReleaseAllOverrides(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _HAL_PWM_H */
