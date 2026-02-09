/**
 * @file commutation.h
 *
 * @brief 6-step commutation table and interface for BLDC motor control.
 *
 * Component: COMMUTATION
 */

#ifndef _COMMUTATION_H
#define _COMMUTATION_H

#include <stdint.h>
#include "../garuda_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 6-step commutation table (defined in commutation.c) */
extern const COMMUTATION_STEP_T commutationTable[6];

void COMMUTATION_AdvanceStep(volatile GARUDA_DATA_T *pData);

#ifdef __cplusplus
}
#endif

#endif /* _COMMUTATION_H */
