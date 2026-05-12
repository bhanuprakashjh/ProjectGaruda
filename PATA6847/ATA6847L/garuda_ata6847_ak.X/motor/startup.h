/**
 * @file startup.h
 * @brief Motor alignment and open-loop ramp interface.
 */

#ifndef STARTUP_H
#define STARTUP_H

#include <stdint.h>
#include <stdbool.h>
#include "../garuda_types.h"

void STARTUP_Init(volatile GARUDA_DATA_T *pData);
bool STARTUP_Align(volatile GARUDA_DATA_T *pData);
bool STARTUP_OpenLoopRamp(volatile GARUDA_DATA_T *pData);

#endif /* STARTUP_H */
