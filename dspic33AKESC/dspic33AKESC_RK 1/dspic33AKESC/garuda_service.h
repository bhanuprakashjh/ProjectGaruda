/**
 * @file garuda_service.h
 *
 * @brief ESC state machine and ADC ISR interface.
 *
 * Component: GARUDA SERVICE
 */

#ifndef _GARUDA_SERVICE_H
#define _GARUDA_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "garuda_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Global ESC data — volatile: shared between ISRs and main loop */
extern volatile GARUDA_DATA_T garudaData;

void GARUDA_ServiceInit(void);

#ifdef __cplusplus
}
#endif

#endif /* _GARUDA_SERVICE_H */
