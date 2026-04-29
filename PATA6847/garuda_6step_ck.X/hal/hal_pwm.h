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

/* Single-pulse mode: above SP_ENTER_ERPM, PWM period = sector period.
 * No switching edges mid-sector → clean BEMF for comparator.
 * amplitude controls on-time fraction within the sector. */
void     HAL_PWM_SetSinglePulse(uint16_t sectorPeriodTCY, uint32_t duty);
void     HAL_PWM_ExitSinglePulse(void);
bool     HAL_PWM_IsSinglePulse(void);
void     HAL_PWM_SetSPFlag(bool on);

/* SP mode dormant — set above any reachable eRPM so SP never engages.
 * SP machinery (unipolar switch, actualStepPeriodHR, PI freeze, dynamic
 * blanking, CCP capture path) is preserved in the codebase for future
 * work. Pivoted to lower PWM carrier (20 kHz) instead — gives cleaner
 * BEMF at high speed without SP's hardware-coupling issues.
 * To re-enable SP: lower threshold to 70000UL/55000UL. */
#define SP_ENTER_ERPM   250000UL
#define SP_EXIT_ERPM    200000UL

#endif
