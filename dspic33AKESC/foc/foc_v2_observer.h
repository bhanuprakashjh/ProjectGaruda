/**
 * @file foc_v2_observer.h
 * @brief MXLEMMING flux observer + PLL speed estimator for FOC v2.
 *
 * Component: FOC V2
 */

#ifndef FOC_V2_OBSERVER_H
#define FOC_V2_OBSERVER_H

#include "foc_v2_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Reset observer state.  Call before first use or on mode change.
 */
void foc_observer_reset(FOC_Observer_t *obs);

/**
 * Seed observer flux state (e.g. at end of alignment).
 * Sets x1 = lambda*cos(theta), x2 = lambda*sin(theta).
 */
void foc_observer_seed(FOC_Observer_t *obs, float lambda, float theta);

/**
 * Run one MXLEMMING observer tick.
 *
 * @param obs       Observer state
 * @param pll       PLL state (updated for speed estimation)
 * @param v_alpha   Applied voltage α (V)
 * @param v_beta    Applied voltage β (V)
 * @param i_alpha   Measured current α (A)
 * @param i_beta    Measured current β (A)
 * @param Rs        Phase resistance (ohm)
 * @param Ls        Phase inductance (H)
 * @param lambda    Nominal flux linkage (V·s/rad)
 * @param dt_comp_v Dead-time compensation voltage (V)
 * @param duty_abs  Absolute duty cycle [0..1] (for gain scheduling)
 * @param dt        Sample period (s)
 */
void foc_observer_update(FOC_Observer_t *obs, FOC_PLL_t *pll,
                         float v_alpha, float v_beta,
                         float i_alpha, float i_beta,
                         float Rs, float Ls, float lambda,
                         float dt_comp_v, float duty_abs,
                         float dt);

/**
 * BEMF angle correction (PLL_OBS hybrid).
 *
 * Computes d-axis BEMF residual to detect observer angle misalignment
 * and applies a corrective rotation to the flux integrator state.
 * Only active above ~500 elec rad/s — below this, the flux observer
 * tracks adequately with real current signal.
 *
 * Prevents the 180° attractor convergence at high speed with zero load.
 * Reference: MESC PLL_OBS (MESCfluxobs.c, David Molony).
 *
 * @param obs     Observer state (x1/x2 rotated in-place)
 * @param vd      Applied d-axis voltage (V), previous tick
 * @param vq      Applied q-axis voltage (V), previous tick
 * @param id      Measured d-axis current (A), previous tick
 * @param iq      Measured q-axis current (A), previous tick
 * @param omega   PLL speed estimate (elec rad/s)
 * @param Rs      Phase resistance (ohm)
 * @param Ls      Phase inductance (H)
 * @param lambda  PM flux linkage (V·s/rad)
 */
void foc_observer_bemf_correct(FOC_Observer_t *obs,
                                float vd, float vq,
                                float id, float iq,
                                float omega,
                                float Rs, float Ls, float lambda);

/**
 * Reset PLL state.
 */
void foc_pll_reset(FOC_PLL_t *pll);

/**
 * Initialize PLL gains.
 * @param pll   PLL state
 * @param bw_hz PLL bandwidth (Hz), e.g. 50
 */
void foc_pll_init(FOC_PLL_t *pll, float bw_hz);

#ifdef __cplusplus
}
#endif

#endif /* FOC_V2_OBSERVER_H */
