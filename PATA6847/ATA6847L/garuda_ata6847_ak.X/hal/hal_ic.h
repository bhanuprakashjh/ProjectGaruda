/**
 * @file hal_ic.h
 * @brief V3 input-capture HAL stub (AK port — V4 ADC midpoint sampler is the
 * production ZC path; V3 IC mode is not used on this board).
 */
#ifndef HAL_IC_H
#define HAL_IC_H
#include <stdint.h>
#include <stdbool.h>
static inline void HAL_IC_Init(void) { }
static inline void HAL_IC_Start(void) { }
static inline void HAL_IC_Stop(void) { }
#endif
