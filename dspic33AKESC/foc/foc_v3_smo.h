/**
 * @file foc_v3_smo.h
 * @brief Sliding Mode Observer (SMO) for sensorless PMSM position estimation.
 *
 * Algorithm (AN1078 + sigmoid improvement):
 *   1. Current model: Î[k+1] = F·Î[k] + G·(V - E - Z)
 *      where F = 1 - Rs·Ts/Ls, G = Ts/Ls
 *   2. Error: err = Î - I_measured
 *   3. Switching: Z = K · sigmoid(err)  [sigmoid = x/(|x|+φ)]
 *   4. BEMF extraction: E = LPF(Z)  (two cascaded 1st-order IIR)
 *   5. Angle: θ = atan2(-Eα_filt, Eβ_filt)
 *
 * The sigmoid switching function replaces hard signum to reduce
 * chattering while preserving sliding mode convergence.
 *
 * Component: FOC V3
 */

#ifndef FOC_V3_SMO_H
#define FOC_V3_SMO_H

#include "foc_v3_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize SMO with motor parameters.
 * Precomputes F, G, K, phi, lpf_alpha.
 */
void v3_smo_init(SMO_Observer_t *smo,
              float Rs, float Ls, float lambda_pm,
              float vbus_nom, float dt);

/**
 * Reset SMO state (zero currents, BEMF, angle).
 * Preserves tuning parameters (F, G, K, phi, lpf_alpha).
 */
void v3_smo_reset(SMO_Observer_t *smo);

/**
 * Run one SMO tick (call at 24 kHz from ADC ISR).
 *
 * @param smo       Observer state
 * @param v_alpha   Applied voltage α (V)
 * @param v_beta    Applied voltage β (V)
 * @param i_alpha   Measured current α (A)
 * @param i_beta    Measured current β (A)
 * @param omega_est Current speed estimate (rad/s, for adaptive LPF)
 * @param dt        Sample period (s)
 */
void v3_smo_update(SMO_Observer_t *smo,
                float v_alpha, float v_beta,
                float i_alpha, float i_beta,
                float omega_est, float dt);

/**
 * Initialize PLL for SMO speed estimation.
 */
void v3_smo_pll_init(SMO_PLL_t *pll, float bw_hz, float omega_max);

/**
 * Reset PLL state (preserves gains).
 */
void v3_smo_pll_reset(SMO_PLL_t *pll);

/**
 * Run PLL on SMO angle output.
 * Returns smoothed speed estimate.
 */
float v3_smo_pll_update(SMO_PLL_t *pll, float theta_meas, float dt);

#ifdef __cplusplus
}
#endif

#endif /* FOC_V3_SMO_H */
