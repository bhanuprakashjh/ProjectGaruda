/**
 * @file bemf_zc.h
 *
 * @brief BEMF zero-crossing detection module for closed-loop commutation.
 * All functions called exclusively from ADC ISR context.
 *
 * Component: BEMF_ZC
 */

#ifndef _BEMF_ZC_H
#define _BEMF_ZC_H

#include "../garuda_types.h"
#include "../garuda_config.h"

#if FEATURE_BEMF_CLOSED_LOOP

#ifdef __cplusplus
extern "C" {
#endif

void                 BEMF_ZC_Init(volatile GARUDA_DATA_T *pData, uint16_t initialStepPeriod);
void                 BEMF_ZC_OnCommutation(volatile GARUDA_DATA_T *pData, uint16_t now);
bool                 BEMF_ZC_Poll(volatile GARUDA_DATA_T *pData, uint16_t now);
bool                 BEMF_ZC_CheckDeadline(volatile GARUDA_DATA_T *pData, uint16_t now);
ZC_TIMEOUT_RESULT_T  BEMF_ZC_CheckTimeout(volatile GARUDA_DATA_T *pData, uint16_t now);
void                 BEMF_ZC_HandleUndetectableStep(volatile GARUDA_DATA_T *pData, uint16_t now);

#ifdef __cplusplus
}
#endif

#endif /* FEATURE_BEMF_CLOSED_LOOP */
#endif /* _BEMF_ZC_H */
