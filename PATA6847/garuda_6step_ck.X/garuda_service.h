/**
 * @file garuda_service.h
 * @brief Main ESC service: state machine, ISRs, throttle.
 */

#ifndef GARUDA_SERVICE_H
#define GARUDA_SERVICE_H

#include "garuda_types.h"

extern volatile GARUDA_DATA_T gData;
extern volatile bool gStateChanged;

void GarudaService_Init(void);
void GarudaService_MainLoop(void);
void GarudaService_StartMotor(void);
void GarudaService_StopMotor(void);

#endif /* GARUDA_SERVICE_H */
