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
