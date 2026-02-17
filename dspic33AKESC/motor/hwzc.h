/**
 * @file hwzc.h
 *
 * @brief Hardware ZC detection module using ADC digital comparators and SCCP timers.
 * State machine: BLANKING -> WATCHING -> COMM_PENDING -> (commutate) -> BLANKING.
 *
 * Component: HWZC
 */

#ifndef HWZC_H
#define HWZC_H

#include "../garuda_types.h"

#if FEATURE_ADC_CMP_ZC

#ifdef __cplusplus
extern "C" {
#endif

void HWZC_Init(volatile GARUDA_DATA_T *pData);
void HWZC_Enable(volatile GARUDA_DATA_T *pData);
void HWZC_Disable(volatile GARUDA_DATA_T *pData);
void HWZC_OnCommutation(volatile GARUDA_DATA_T *pData);
void HWZC_OnZcDetected(volatile GARUDA_DATA_T *pData);
void HWZC_OnBlankingExpired(volatile GARUDA_DATA_T *pData);
void HWZC_OnCommDeadline(volatile GARUDA_DATA_T *pData);
void HWZC_OnTimeout(volatile GARUDA_DATA_T *pData);

#ifdef __cplusplus
}
#endif

#endif /* FEATURE_ADC_CMP_ZC */
#endif /* HWZC_H */
