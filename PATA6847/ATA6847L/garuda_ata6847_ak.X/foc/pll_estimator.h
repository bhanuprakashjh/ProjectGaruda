/**
 * @file pll_estimator.h
 * @brief PLL rotor angle and speed estimator from back-EMF.
 *
 * Phase discriminator (cross-product):
 *   epsilon = e_beta*cos(theta_hat) - e_alpha*sin(theta_hat)
 *   epsilon ~ sin(theta_true - theta_hat) ~ (theta_true - theta_hat) near lock
 *
 * Loop filter (PI):
 *   omega_hat = Kp_pll*epsilon + Ki_pll*integral(epsilon) dt
 *
 * Angle integration:
 *   theta_hat[k+1] = theta_hat[k] + omega_hat[k]*Ts   (wrapped to [0, 2pi))
 *
 * PLL bandwidth ~ 50 Hz (gains in garuda_foc_params.h).
 */

#ifndef PLL_ESTIMATOR_H
#define PLL_ESTIMATOR_H

#include "foc_types.h"

/** @brief Reset PLL to zero angle/speed (call on startup/fault). */
void pll_reset(PLL_t *pll);

/**
 * @brief Update PLL with new back-EMF estimates.
 * @param pll      PLL state (theta_est and omega_est updated in-place)
 * @param e_alpha  Observer e_alpha (V)
 * @param e_beta   Observer e_beta (V)
 * @param Ts       Sample period (s)
 */
void pll_update(PLL_t *pll, float e_alpha, float e_beta, float Ts);

#endif /* PLL_ESTIMATOR_H */
