/**
 * @file startup.h
 *
 * @brief Motor alignment and open-loop ramp interface.
 *
 * Component: STARTUP
 */

#ifndef _STARTUP_H
#define _STARTUP_H

#include <stdint.h>
#include <stdbool.h>
#include "../garuda_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void STARTUP_Init(volatile GARUDA_DATA_T *pData);
bool STARTUP_Align(volatile GARUDA_DATA_T *pData);
bool STARTUP_OpenLoopRamp(volatile GARUDA_DATA_T *pData);

#if FEATURE_SINE_STARTUP
void STARTUP_SineInit(volatile GARUDA_DATA_T *pData);
bool STARTUP_SineAlign(volatile GARUDA_DATA_T *pData);
bool STARTUP_SineRamp(volatile GARUDA_DATA_T *pData);
void STARTUP_SineComputeDuties(volatile GARUDA_DATA_T *pData,
                                uint32_t *dutyA, uint32_t *dutyB, uint32_t *dutyC);
uint8_t STARTUP_SineGetTransitionStep(volatile GARUDA_DATA_T *pData);

void STARTUP_MorphInit(volatile GARUDA_DATA_T *pData);
void STARTUP_MorphComputeDuties(volatile GARUDA_DATA_T *pData,
                                 uint32_t *dutyA, uint32_t *dutyB, uint32_t *dutyC);
bool STARTUP_MorphCheckSectorBoundary(volatile GARUDA_DATA_T *pData);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _STARTUP_H */
