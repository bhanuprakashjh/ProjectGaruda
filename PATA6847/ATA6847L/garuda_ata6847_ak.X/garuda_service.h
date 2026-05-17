/**
 * @file garuda_service.h
 * @brief Main ESC service: state machine, ISRs, throttle.
 */

#ifndef GARUDA_SERVICE_H
#define GARUDA_SERVICE_H

#include "garuda_types.h"

extern volatile bool gStateChanged;

void GarudaService_Init(void);
void GarudaService_MainLoop(void);
void GarudaService_StartMotor(void);
void GarudaService_StopMotor(void);
void GarudaService_Tasks(void);
void GarudaService_ClearFault(void);

/* V4 BEMF detection — runs once per PTG fire (PWM mid-OFF or mid-ON,
 * selected by the duty-adaptive sampler in hal_ptg.c). Reads the
 * floating-phase comparator GPIO, deglitches, classifies pre/post-ZC,
 * updates the shadow counters, and on a real edge feeds
 * v4_lastCaptureHR + v4_captureValid for the sector PI.
 *
 * Defined in garuda_service.c, called from _PTG0Interrupt. Counter
 * names retain their `adc*` prefix for diagnostic continuity — they
 * aren't tied to the ADC peripheral. */
void V4_ProcessBemfSample(void);

#endif /* GARUDA_SERVICE_H */
