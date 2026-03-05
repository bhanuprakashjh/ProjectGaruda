/**
 * @file flux_estimator.c
 * @brief Flux-integration position estimator.
 *
 * Integrates observer back-EMF to estimate rotor flux linkage,
 * then computes angle = atan2(-psi_alpha, psi_beta).
 *
 * High-pass filter on the integrator prevents DC drift that would
 * otherwise accumulate from offset errors in the voltage model.
 *
 * Component: FOC
 */

#include <math.h>
#include "flux_estimator.h"

/* High-pass filter coefficient.
 * Removes DC drift from the integrator.
 * Higher = more filtering (faster drift removal) but more phase lag.
 * 0.002 at 24 kHz -> HPF corner ~ 0.002 x 24000 / (2pi) ~ 7.6 Hz. */
#define FLUX_HPF_ALPHA  0.002f

#define TWO_PI  6.28318530718f

void flux_est_reset(FluxEst_t *est)
{
    est->psi_alpha  = 0.0f;
    est->psi_beta   = 0.0f;
    est->theta_est  = 0.0f;
    est->omega_est  = 0.0f;
    est->theta_prev = 0.0f;
}

void flux_est_update(FluxEst_t *est, float e_alpha, float e_beta, float Ts)
{
    /* High-pass filtered integration:
     * psi = (1 - alpha) x (psi_prev + e_bemf x Ts)
     * The (1-alpha) factor bleeds off accumulated DC offset. */
    float decay = 1.0f - FLUX_HPF_ALPHA;
    est->psi_alpha = decay * (est->psi_alpha + e_alpha * Ts);
    est->psi_beta  = decay * (est->psi_beta  + e_beta  * Ts);

    /* Angle from flux linkage.
     * For a PMSM: flux_alpha = Lm x cos(theta), flux_beta = Lm x sin(theta)
     * Back-EMF = d(flux)/dt, so integrated BEMF = flux (shifted by 90 deg).
     * theta = atan2(-psi_alpha, psi_beta) corrects the 90 deg phase. */
    float theta = atan2f(-est->psi_alpha, est->psi_beta);
    if (theta < 0.0f) theta += TWO_PI;

    /* Speed from angle differentiation (with wrap handling) */
    float dtheta = theta - est->theta_prev;
    if (dtheta > 3.14159f)  dtheta -= TWO_PI;
    if (dtheta < -3.14159f) dtheta += TWO_PI;
    est->omega_est = dtheta / Ts;

    est->theta_prev = theta;
    est->theta_est  = theta;
}
