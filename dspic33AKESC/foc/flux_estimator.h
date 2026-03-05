/**
 * @file flux_estimator.h
 * @brief Flux-integration position estimator (parallel to PLL).
 *
 * Integrates back-EMF (from the voltage-model observer) to obtain
 * rotor flux linkage, then extracts angle via atan2.
 *
 * Uses high-pass filtered integration to prevent DC drift:
 *   psi[n] = (1 - hpf_alpha) * (psi[n-1] + e_bemf * Ts)
 *
 * This runs in parallel with the PLL estimator for comparison
 * and performance testing.
 *
 * Component: FOC
 */

#ifndef FLUX_ESTIMATOR_H
#define FLUX_ESTIMATOR_H

#include <stdint.h>

typedef struct {
    float psi_alpha;    /* Integrated flux alpha (V-s) */
    float psi_beta;     /* Integrated flux beta  (V-s) */
    float theta_est;    /* Estimated angle (rad, 0 to 2pi) */
    float omega_est;    /* Estimated speed (rad/s elec.) */
    float theta_prev;   /* Previous angle for speed differentiation */
} FluxEst_t;

/** Reset estimator state to zero. */
void flux_est_reset(FluxEst_t *est);

/**
 * Run one estimation step.
 * @param est       Estimator state
 * @param e_alpha   Back-EMF alpha from observer (V)
 * @param e_beta    Back-EMF beta from observer (V)
 * @param Ts        Sample period (s)
 */
void flux_est_update(FluxEst_t *est, float e_alpha, float e_beta, float Ts);

#endif /* FLUX_ESTIMATOR_H */
