/**
 * @file pll_estimator.h
 * @brief PLL rotor angle and speed estimator from back-EMF.
 *
 * Phase discriminator (cross-product):
 *   ε = eβ·cos(θ̂) − eα·sin(θ̂)
 *   ε ≈ sin(θ_true − θ̂) ≈ (θ_true − θ̂) near lock
 *
 * Loop filter (PI):
 *   ω̂ = Kp_pll·ε + Ki_pll·∫ε dt
 *
 * Angle integration:
 *   θ̂[k+1] = θ̂[k] + ω̂[k]·Ts   (wrapped to [0, 2π))
 *
 * PLL bandwidth ≈ 200 Hz (gains in garuda_foc_params.h).
 */

#ifndef PLL_ESTIMATOR_H
#define PLL_ESTIMATOR_H

#include "foc_types.h"

/** @brief Reset PLL to zero angle/speed (call on startup/fault). */
void pll_reset(PLL_t *pll);

/**
 * @brief Update PLL with new back-EMF estimates.
 * @param pll      PLL state (theta_est and omega_est updated in-place)
 * @param e_alpha  Observer eα (V)
 * @param e_beta   Observer eβ (V)
 * @param Ts       Sample period (s)
 */
void pll_update(PLL_t *pll, float e_alpha, float e_beta, float Ts);

#endif /* PLL_ESTIMATOR_H */
