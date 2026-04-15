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

#endif /* GARUDA_SERVICE_H */
