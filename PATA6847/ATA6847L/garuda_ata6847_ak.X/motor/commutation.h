/**
 * @file commutation.h
 * @brief 6-step commutation table and interface.
 */

#ifndef COMMUTATION_H
#define COMMUTATION_H

#include <stdint.h>
#include "../garuda_types.h"

extern const COMMUTATION_STEP_T commutationTable[6];

void COMMUTATION_AdvanceStep(volatile GARUDA_DATA_T *pData);
void COMMUTATION_ApplyStep(volatile GARUDA_DATA_T *pData, uint8_t step);

#endif /* COMMUTATION_H */
