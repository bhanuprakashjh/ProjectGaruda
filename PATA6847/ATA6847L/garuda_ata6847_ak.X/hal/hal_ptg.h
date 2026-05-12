/**
 * @file hal_ptg.h
 * @brief PTG HAL stub (AK port — FEATURE_V5_PTG_ZC=0 by default).
 * When V5 PTG sampling is re-enabled, this needs full AK rewrite —
 * AK PTG SFRs differ from CK.
 */
#ifndef HAL_PTG_H
#define HAL_PTG_H
#include <stdint.h>
static inline void HAL_PTG_Init(void) { }
static inline void HAL_PTG_Start(void) { }
static inline void HAL_PTG_Stop(void) { }
static inline void HAL_PTG_SetPeakDelay(uint16_t ticks) { (void)ticks; }
static inline void HAL_PTG_SetValleyDelay(uint16_t ticks) { (void)ticks; }
#endif
