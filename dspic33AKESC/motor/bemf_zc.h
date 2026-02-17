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

#if FEATURE_ADC_CMP_ZC && FEATURE_BEMF_INTEGRATION
void BEMF_INTEG_ObserverOnComm(volatile GARUDA_DATA_T *pData, uint16_t stepPeriod);
void BEMF_INTEG_ObserverTick(volatile GARUDA_DATA_T *pData, uint16_t now,
                              uint16_t lastCommTick);
#endif

#ifdef __cplusplus
}
#endif

#endif /* FEATURE_BEMF_CLOSED_LOOP */
#endif /* _BEMF_ZC_H */
