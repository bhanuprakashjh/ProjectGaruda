/**
 * @file hal_capture.c
 * @brief Per-sector capture state used by the BEMF midpoint sampler.
 *
 * No SCCP hardware capture on AK — commutation is driven by the ADC-ISR
 * midpoint sampler in ProcessBemfSample. This file holds the shared
 * state (last capture HR, per-sector polarity flag) the sampler reads.
 * The Init/Start/Stop/RouteForSector entry points are empty stubs kept
 * so existing sector_pi call sites don't need to change shape.
 */

#include "hal_capture.h"
#include "../motor/motor_params.h"

volatile uint16_t lastCaptureHR_g = 0;
volatile bool     captureValid  = false;

static bool currentRisingZc = true;

void HAL_Capture_Init(void) { }
void HAL_Capture_Start(void) { }
void HAL_Capture_Stop(void) { }
void HAL_Capture_Configure(uint8_t rpPin, bool risingZc)
{
    (void)rpPin;
    currentRisingZc = risingZc;
}
void HAL_Capture_RouteForSector(uint8_t sector) { (void)sector; }
void HAL_Capture_OnCommutation(void) { }
void HAL_Capture_SetBlanking(uint16_t sectorPeriodHR) { (void)sectorPeriodHR; }
bool HAL_Capture_IsRisingZc(void) { return currentRisingZc; }
uint16_t HAL_Capture_GetCcp2Offset(void) { return 0; }
uint16_t HAL_Capture_GetCcp5Offset(void) { return 0; }
uint16_t HAL_Capture_GetBlankingEnd(void) { return 0; }
