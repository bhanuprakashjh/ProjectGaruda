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

#ifdef __cplusplus
}
#endif

#endif /* _STARTUP_H */
