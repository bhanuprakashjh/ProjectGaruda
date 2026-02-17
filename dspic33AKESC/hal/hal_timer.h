/**
 * @file hal_timer.h
 *
 * @brief SCCP timer HAL for hardware ZC detection.
 * SCCP1: 32-bit one-shot timer for blanking + commutation scheduling.
 * SCCP2: 32-bit free-running timer for high-resolution timestamps.
 *
 * Component: HAL_TIMER
 */

#ifndef HAL_TIMER_H
#define HAL_TIMER_H

#include <stdint.h>
#include "../garuda_config.h"

#if FEATURE_ADC_CMP_ZC

#ifdef __cplusplus
extern "C" {
#endif

void HAL_SCCP1_Init(void);
void HAL_SCCP1_StartOneShot(uint32_t ticks);
void HAL_SCCP1_Stop(void);

void HAL_SCCP2_Init(void);
uint32_t HAL_SCCP2_ReadTimestamp(void);

#ifdef __cplusplus
}
#endif

#endif /* FEATURE_ADC_CMP_ZC */
#endif /* HAL_TIMER_H */
