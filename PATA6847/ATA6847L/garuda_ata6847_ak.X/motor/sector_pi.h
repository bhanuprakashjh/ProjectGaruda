/**
 * @file sector_pi.h
 * @brief Sector PI motor control — AVR-style synchronizer.
 *
 * Ground-up rewrite modeled on Microchip AVR high-speed motor control.
 * PI always owns commutation scheduling. No poll, no DMA, no modes.
 */

#ifndef SECTOR_PI_H
#define SECTOR_PI_H

#include "../garuda_config.h"


#include <stdint.h>
#include <stdbool.h>

/* Motor status events (matches AVR motor_status_t) */
#define EVENT_STALL      (1U << 1)
#define EVENT_FAULT      (1U << 2)

/* Telemetry snapshot */
typedef struct {
    uint16_t timerPeriod;
    uint16_t integrator;
    uint16_t lastCapValue;
    uint16_t actualAmplitude;
    uint16_t measuredSpeed;
    uint8_t  position;
    uint16_t stallCounter;   /* widened 2026-05-21 so STALL_THRESHOLD can exceed 255 */
    uint32_t sectorCount;
    uint16_t statusEvents;
    bool     running;
    bool     commandEnabled;
    uint16_t diagCaptures;
    uint16_t diagPiRuns;
    uint16_t diagLastCapValue;
    int16_t  diagDelta;
    bool     spMode;        /* single-pulse mode active */
    bool     spRequest;     /* single-pulse mode requested (may lag spMode) */
    uint32_t erpmNow;       /* instantaneous eRPM from timerPeriod */
    /* ADC midpoint ZC capture-rate diagnostics (Mode 1).
     * 32-bit — ADC fires at 40kHz so uint16 wraps every ~1.6s. */
    uint32_t adcBlankReject;        /* ADC fired pre-blanking-end */
    uint32_t adcStateMismatch;      /* past blanking, GPIO != expected */
    uint32_t adcCaptureSet;         /* set captureValid (candidate) */
    uint32_t commutateNoCapture;    /* Commutate found capValue == SENTINEL */
    uint32_t ptgFires;              /* _PTG0Interrupt fire count (heartbeat) */
    uint32_t postZcRisingAcc;       /* ADC ISR post-ZC shadow counters */
    uint32_t postZcRisingRej;
    uint32_t postZcFallingAcc;
    uint32_t postZcFallingRej;
    uint16_t tMeasHR;               /* smoothed commutation interval */
    uint16_t actualStepPeriodHR;    /* raw measured comm-to-comm HR (1× real) */
} TELEM_T;

void     SectorPI_Init(void);
void     SectorPI_Start(uint16_t vbusRaw);
void     SectorPI_Stop(void);
void     SectorPI_OlTick(void);       /* Call from Timer1 ISR at 20kHz during ALIGN+OL_RAMP */
void     SectorPI_Commutate(void);    /* Call from SCCP3 ISR during CL */
void     SectorPI_TimeTick(void);     /* Call at 1ms for speed measurement + amplitude */
void     SectorPI_CommandSet(uint16_t amplitude);
uint32_t SectorPI_ErpmGet(void);
void     SectorPI_TelemGet(TELEM_T *out);
bool     SectorPI_IsRunning(void);
uint8_t  SectorPI_GetPhase(void);     /* 0=OFF, 1=ALIGN, 2=OL_RAMP, 3=CL */

/* Capture-log accessor — returns up to *entriesOut entries (8B each:
 * timerPeriod, setValue, capValue, delta_clamped) into buf, freezes
 * after first 30 PI runs post-CL-entry. *entriesOut updated to actual
 * count copied. Caller must size buf >= 30*8 = 240 bytes. */
void     SectorPI_GetCaptureLog(uint8_t *buf, uint8_t *entriesOut);


/* ── Single-Pulse mode shared state ────────────────────────────── */
/* g_pwmPer: current PWM period in TCY units.
 * Equals LOOPTIME_TCY in normal mode, or MPER (sector-matched) in SP mode.
 * Updated by SectorPI_TimeTick — NOT by the Commutate ISR.
 * The ISR reads it for duty scaling with zero branch overhead. */
extern volatile uint16_t g_pwmPer;

/* ── Atomic per-sector snapshot (PTG ↔ Commutate race-safe) ─────
 * Single 16-bit word written atomically by SectorPI_Commutate (IPL 6),
 * read once at the top of ProcessBemfSample. PTG (IPL 4) cannot
 * preempt itself, so a single 16-bit load yields a coherent triple.
 *
 *   bits 0..2  currentSector  (0..5)
 *   bits 3..4  floatingPhase  (0..2)
 *   bit  5     ptgExpectedComp (0..1) */
extern volatile uint16_t sectorSnap;

/* SCCP4-domain timestamp of the most recent commutation. Read-only
 * outside SectorPI_Commutate. */
extern volatile uint16_t lastCommHR;

/* Latest accepted BEMF capture (SCCP4 HR domain). Written by
 * ProcessBemfSample when its deglitch + polarity gate accepts a
 * sample; consumed by SectorPI_OlTick and SectorPI_Commutate. The
 * captureValid flag re-arms (cleared) after each Commutate consumes
 * the timestamp. */
extern volatile uint16_t lastCaptureHR_g;
extern volatile bool     captureValid;

#define SECTOR_SNAP_SECT(w)  ((uint8_t)((w) & 0x7u))
#define SECTOR_SNAP_FP(w)    ((uint8_t)(((w) >> 3) & 0x3u))
#define SECTOR_SNAP_EXP(w)   ((uint8_t)(((w) >> 5) & 0x1u))

#endif /* SECTOR_PI_H */

