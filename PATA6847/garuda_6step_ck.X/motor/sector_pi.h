/**
 * @file sector_pi.h
 * @brief V4 Sector PI motor control — AVR-style synchronizer.
 *
 * Ground-up rewrite modeled on Microchip AVR high-speed motor control.
 * PI always owns commutation scheduling. No poll, no DMA, no modes.
 */

#ifndef SECTOR_PI_H
#define SECTOR_PI_H

#include "../garuda_config.h"

#if FEATURE_V4_SECTOR_PI

#include <stdint.h>
#include <stdbool.h>

/* Motor status events (matches AVR motor_status_t) */
#define V4_EVENT_STALL      (1U << 1)
#define V4_EVENT_FAULT      (1U << 2)

/* V4 telemetry snapshot */
typedef struct {
    uint16_t timerPeriod;
    uint16_t integrator;
    uint16_t lastCapValue;
    uint16_t actualAmplitude;
    uint16_t measuredSpeed;
    uint8_t  position;
    uint8_t  stallCounter;
    uint32_t sectorCount;
    uint16_t statusEvents;
    bool     running;
    bool     commandEnabled;
    uint16_t diagCaptures;
    uint16_t diagPiRuns;
    uint16_t diagLastCapValue;
    int16_t  diagDelta;
} V4_TELEM_T;

void     SectorPI_Init(void);
void     SectorPI_Start(uint16_t vbusRaw);
void     SectorPI_Stop(void);
void     SectorPI_OlTick(void);       /* Call from Timer1 ISR at 20kHz during ALIGN+OL_RAMP */
void     SectorPI_Commutate(void);    /* Call from SCCP3 ISR during CL */
void     SectorPI_TimeTick(void);     /* Call at 1ms for speed measurement + amplitude */
void     SectorPI_CommandSet(uint16_t amplitude);
uint32_t SectorPI_ErpmGet(void);
void     SectorPI_TelemGet(V4_TELEM_T *out);
bool     SectorPI_IsRunning(void);
uint8_t  SectorPI_GetPhase(void);     /* 0=OFF, 1=ALIGN, 2=OL_RAMP, 3=CL */

#endif /* FEATURE_V4_SECTOR_PI */
#endif /* SECTOR_PI_H */
