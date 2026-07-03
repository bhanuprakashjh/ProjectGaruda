/**
 * @file speed_pi.h
 *
 * @brief Per-ZC interval-based speed PID (CL state).
 *
 * Throttle ADC → target stepPeriodHR (period domain, not eRPM, saves a
 * runtime division). PI runs in HWZC ISR per sector event. Output
 * replaces the direct throttle→duty mapping in garuda_service CL state.
 *
 * Gated by FEATURE_SPEED_PI. When OFF (default), this module compiles
 * to no-ops and the existing direct-duty path is unchanged.
 *
 * Component: SPEED_PI
 */
#ifndef SPEED_PI_H
#define SPEED_PI_H

#include "../garuda_types.h"

/**
 * @brief Initialize speed PI state. Called from GarudaService_Init.
 */
void SPEED_PI_Init(volatile GARUDA_DATA_T *pData);

/**
 * @brief Enable PI on CL entry. Resets state to safe initial values.
 *
 * Integrator is seeded with the current garudaData.duty so the first
 * PI tick produces the same output as the prior direct-duty mapping
 * (bumpless transfer on enable).
 *
 * Counts ZCs from zero; integral is gated until SPEED_PI_INTEGRAL_DISABLE_ZCS
 * elapsed.
 */
void SPEED_PI_Enable(volatile GARUDA_DATA_T *pData);

/**
 * @brief Disable PI on CL exit (e.g., transition to IDLE/FAULT).
 */
void SPEED_PI_Disable(volatile GARUDA_DATA_T *pData);

/**
 * @brief Update PI on a ZC event. Call from HWZC_OnPiPeriodExpired.
 *
 * Reads pData->throttle and pData->hwzc.stepPeriodHR, writes
 * pData->speedPi.outputDuty for the CL state to consume.
 *
 * Does nothing if !pData->speedPi.enabled (e.g., before CL entry).
 */
void SPEED_PI_OnZcEvent(volatile GARUDA_DATA_T *pData);

/* ── WS4: nested speed→current→duty cascade (FEATURE_SPEED_CASCADE) ──────
 * A fresh implementation, independent of SPEED_PI above (it outputs a
 * current reference → inner current PI → duty, not duty directly).
 * Co-located in speed_pi.c only to avoid adding a new MPLAB translation
 * unit. No-op stubs when the feature is off. */
void     SPEED_CASCADE_Init(void);
void     SPEED_CASCADE_Reset(void);                            /* clear engage state on CL entry */
void     SPEED_CASCADE_Disable(void);
bool     SPEED_CASCADE_Enabled(void);
void     SPEED_CASCADE_OnZcEvent(volatile GARUDA_DATA_T *pData);            /* outer, per-ZC */
/* Inner step. Runs the loop only once the motor is ENGAGED (measured eRPM
 * above the engage gate); below the gate it passes `fallbackDuty` (the normal
 * throttle→duty map) straight through so the proven startup/accel is untouched
 * and the loop only takes over to HOLD/trim speed once established. */
uint32_t SPEED_CASCADE_InnerStep(volatile GARUDA_DATA_T *pData,
                                 uint32_t cap, uint32_t fallbackDuty);

#endif /* SPEED_PI_H */
