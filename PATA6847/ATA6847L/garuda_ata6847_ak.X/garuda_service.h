/**
 * @file garuda_service.h
 * @brief Main ESC service: state machine, ISRs, throttle.
 */

#ifndef GARUDA_SERVICE_H
#define GARUDA_SERVICE_H

#include "garuda_types.h"

#if !FEATURE_V4_SECTOR_PI
extern volatile GARUDA_DATA_T gData;
#endif
extern volatile bool gStateChanged;

void GarudaService_Init(void);
void GarudaService_MainLoop(void);
void GarudaService_StartMotor(void);
void GarudaService_StopMotor(void);
void GarudaService_Tasks(void);
void GarudaService_ClearFault(void);

/* V4 BEMF detection — runs once per PWM mid-OFF event.  Reads the
 * floating-phase comparator GPIO, deglitches, classifies pre/post-ZC,
 * updates the legacy V4 + V5 shadow counters, and on a real edge feeds
 * v4_lastCaptureHR + v4_captureValid for the sector PI.
 *
 * Defined in garuda_service.c.  Called from either:
 *   - the ADC ISR (default — FEATURE_BEMF_VIA_PTG=0)
 *   - the PTG _PTG0Interrupt (Phase 2 — FEATURE_BEMF_VIA_PTG=1)
 *
 * Same logic in both call sites; only the trigger source / latency
 * changes.  Counter names retain their `adc*` prefix for diagnostic
 * continuity — they aren't tied to the ADC peripheral. */
void V4_ProcessBemfSample(void);

#endif /* GARUDA_SERVICE_H */
