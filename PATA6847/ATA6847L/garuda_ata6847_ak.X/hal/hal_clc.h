/**
 * @file hal_clc.h
 * @brief CLC HAL stub (AK port — CLC not used in V4 path).
 * FEATURE_CLC_BLANKING auto-disabled when FEATURE_V4_SECTOR_PI=1.
 */
#ifndef HAL_CLC_H
#define HAL_CLC_H
static inline void HAL_CLC_Init(void) { }
#endif
