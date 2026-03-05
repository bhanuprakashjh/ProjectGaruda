/**
 * @file back_emf_obs.h
 * @brief Voltage-model back-EMF observer for sensorless angle estimation.
 *
 * Model:
 *   e_alpha = V_alpha - Rs*I_alpha - Ls*(dI_alpha/dt)
 *   e_beta  = V_beta  - Rs*I_beta  - Ls*(dI_beta/dt)
 *
 * dI/dt approximated by backward difference: (I[k] - I[k-1]) / Ts.
 * Output filtered by first-order EMA to suppress switching noise.
 *
 * Reliability note:
 *   The voltage model is unreliable at very low speed (R*I dominates).
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
 * @param v_alpha  Applied V_alpha (V) -- V_alpha_duty x Vbus
 * @param v_beta   Applied V_beta (V)
 * @param i_alpha  Measured I_alpha (A) -- from Clarke output
 * @param i_beta   Measured I_beta (A)
 * @param Ts       Sample period (s)
 */
void bemf_obs_update(BackEMFObs_t *obs,
                     float v_alpha, float v_beta,
                     float i_alpha, float i_beta,
                     float Ts);

#endif /* BACK_EMF_OBS_H */
