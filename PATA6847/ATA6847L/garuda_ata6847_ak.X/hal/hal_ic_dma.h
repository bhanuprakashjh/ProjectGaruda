/**
 * @file hal_ic_dma.h
 * @brief Dual-CCP+DMA shadow ring HAL stub (AK port — not used in V4 path).
 * FEATURE_IC_DMA_SHADOW is auto-disabled when FEATURE_V4_SECTOR_PI=1.
 */
#ifndef HAL_IC_DMA_H
#define HAL_IC_DMA_H
#include <stdint.h>
#include <stdbool.h>
static inline void HAL_ICDMA_Init(void) { }
static inline void HAL_ICDMA_Start(void) { }
static inline void HAL_ICDMA_Stop(void) { }
#endif
