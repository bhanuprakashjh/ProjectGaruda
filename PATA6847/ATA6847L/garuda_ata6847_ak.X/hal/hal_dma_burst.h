/**
 * @file hal_dma_burst.h
 * @brief DMA burst capture HAL stub (AK port — V3 path, not used in V4).
 */
#ifndef HAL_DMA_BURST_H
#define HAL_DMA_BURST_H
#include <stdint.h>
static inline void HAL_DMABurst_Init(void) { }
static inline void HAL_DMABurst_Arm(void) { }
static inline bool HAL_DMABurst_Done(void) { return false; }
#endif
