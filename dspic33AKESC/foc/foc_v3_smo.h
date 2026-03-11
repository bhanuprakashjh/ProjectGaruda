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
 * Initialize SMO from runtime configuration struct.
 * Precomputes F, G, stores all tuning params.
 */
void v3_smo_init(SMO_Observer_t *smo, const SMO_Config_t *cfg);

/**
 * Reset SMO state (zero currents, BEMF, angle).
 * Preserves tuning parameters (F, G, gains, LPF config).
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
 */
void v3_smo_update(SMO_Observer_t *smo,
                float v_alpha, float v_beta,
                float i_alpha, float i_beta,
                float omega_est);

/**
 * Compute SMO phase delay (radians) for commutation compensation.
 * Uses exact arctan model: φ = 2·arctan(ω/ωc) + ω·dt.
 * Call after v3_smo_update() — uses the alpha_now computed that tick.
 *
 * @param smo   Observer state (holds LPF cutoff ωc)
 * @param omega Current electrical speed (rad/s)
 * @return Phase delay in radians
 */
float v3_smo_phase_delay(const SMO_Observer_t *smo, float omega);

/**
 * Adapt Rs online from current prediction error.
 * Call during closed-loop at moderate+ speed only.
 * Updates smo->Rs_est and recomputes plant coefficient F.
 */
void v3_smo_adapt_rs(SMO_Observer_t *smo,
                     float i_alpha, float i_beta,
                     float Rs_init);

/**
 * Initialize PLL for SMO speed estimation.
 *
 * @param pll          PLL state
 * @param bw_min_hz    Minimum BW at low speed (Hz)
 * @param bw_max_hz    Maximum BW at high speed (Hz)
 * @param omega_bw_ref Speed at which BW = bw_max (rad/s)
 * @param omega_max    Speed clamp magnitude (rad/s)
 */
void v3_smo_pll_init(SMO_PLL_t *pll,
                     float bw_min_hz, float bw_max_hz,
                     float omega_bw_ref, float omega_max);

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
