/**
 * @file mxlemming_obs.h
 * @brief MXLEMMING flux observer for sensorless PMSM angle/speed estimation.
 *
 * Flux-integration observer based on VESC's MXLEMMING design:
 *   x1 += (Valpha - Rs*Ialpha)*dt - Ls*(Ialpha - Ialpha_prev)
 *   x2 += (Vbeta  - Rs*Ibeta )*dt - Ls*(Ibeta  - Ibeta_prev)
 *
 * Key advantages over voltage-model BEMF + EMA + PLL chain:
 *   - No dI/dt derivative (uses delta-I directly — no noise amplification)
 *   - No EMA filter needed (integration is inherently smooth)
 *   - No PLL needed (angle = atan2(-x1, x2), speed = d(angle)/dt)
 *   - Flux magnitude clamped to lambda = Ke (prevents integrator drift)
 *
 * Component: FOC
 */

#ifndef MXLEMMING_OBS_H
#define MXLEMMING_OBS_H

#include <stdint.h>

typedef struct {
    float x1;              /* Flux alpha (V-s) */
    float x2;              /* Flux beta  (V-s) */
    float i_alpha_prev;    /* Previous Ialpha for delta-I */
    float i_beta_prev;     /* Previous Ibeta for delta-I */
    float theta_est;       /* Estimated rotor angle (rad) */
    float omega_est;       /* Estimated speed (rad/s) */
    float theta_prev;      /* Previous angle for speed calc */
} MxlObs_t;

/** @brief Reset observer state to zero (call on startup/fault clear). */
void mxl_reset(MxlObs_t *obs);

/**
 * @brief Run one MXLEMMING observer step (call every fast-loop tick).
 * @param obs      Observer state
 * @param v_alpha  Applied voltage alpha (V) — from inverse Park output
 * @param v_beta   Applied voltage beta  (V)
 * @param i_alpha  Measured current alpha (A) — from Clarke output
 * @param i_beta   Measured current beta  (A)
 * @param dt       Sample period (s) — FOC_TS_FAST_S
 */
void mxl_update(MxlObs_t *obs,
                float v_alpha, float v_beta,
                float i_alpha, float i_beta,
                float dt);

#endif /* MXLEMMING_OBS_H */
