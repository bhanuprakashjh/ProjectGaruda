/**
 * @file back_emf_obs.h
 * @brief Voltage-model back-EMF observer for sensorless angle estimation.
 *
 * Model:
 *   ê_α = V_α − Rs·I_α − Ls·(dI_α/dt)
 *   ê_β = V_β − Rs·I_β − Ls·(dI_β/dt)
 *
 * dI/dt approximated by backward difference: (I[k] − I[k−1]) / Ts.
 * Output filtered by first-order EMA to suppress switching noise.
 *
 * Reliability note:
 *   The voltage model is unreliable at very low speed (R·I dominates).
 *   The state machine only hands off to the PLL above STARTUP_HANDOFF_RAD_S.
 */

#ifndef BACK_EMF_OBS_H
#define BACK_EMF_OBS_H

#include "foc_types.h"

/** @brief Reset observer state to zero (call on startup/fault clear). */
void bemf_obs_reset(BackEMFObs_t *obs);

/**
 * @brief Run one observer step (call every fast-loop tick).
 * @param obs      Observer state
 * @param v_alpha  Applied Vα (V) — Vα_duty × Vbus
 * @param v_beta   Applied Vβ (V)
 * @param i_alpha  Measured Iα (A) — from Clarke output
 * @param i_beta   Measured Iβ (A)
 * @param Ts       Sample period (s)
 */
void bemf_obs_update(BackEMFObs_t *obs,
                     float v_alpha, float v_beta,
                     float i_alpha, float i_beta,
                     float Ts);

#endif /* BACK_EMF_OBS_H */
