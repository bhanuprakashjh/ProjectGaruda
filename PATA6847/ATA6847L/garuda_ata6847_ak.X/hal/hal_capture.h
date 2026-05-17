/**
 * @file hal_capture.h
 * @brief Per-sector capture state for the BEMF midpoint sampler.
 */

#ifndef HAL_CAPTURE_H
#define HAL_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>
#include <xc.h>

#define BEMF_A_RP   0x0036U
#define BEMF_B_RP   0x0037U
#define BEMF_C_RP   0x004AU

extern volatile uint16_t lastCaptureHR_g;
extern volatile bool     captureValid;

void     HAL_Capture_Init(void);
void     HAL_Capture_Start(void);
void     HAL_Capture_Stop(void);
void     HAL_Capture_Configure(uint8_t rpPin, bool risingZc);
void     HAL_Capture_SetBlanking(uint16_t sectorPeriodHR);
bool     HAL_Capture_IsRisingZc(void);
uint16_t HAL_Capture_GetCcp2Offset(void);
uint16_t HAL_Capture_GetCcp5Offset(void);
uint16_t HAL_Capture_GetBlankingEnd(void);

#endif
