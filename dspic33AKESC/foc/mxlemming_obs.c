/**
 * @file mxlemming_obs.c
 * @brief MXLEMMING flux observer implementation.
 *
 * Flux integration with delta-I compensation:
 *   x1 += (Valpha - Rs*Ialpha)*dt - Ls*(Ialpha - Ialpha_prev)
 *   x2 += (Vbeta  - Rs*Ibeta )*dt - Ls*(Ibeta  - Ibeta_prev)
 *
 * After integration, flux magnitude is clamped to MOTOR_FLUX_LINKAGE
 * to prevent integrator drift. Angle is extracted via atan2f(-x1, x2)
 * which gives rotor angle directly (no pi/2 offset needed).
 *
 * Speed is computed as d(angle)/dt with wrap handling.
 *
 * Component: FOC
 */

#include "mxlemming_obs.h"
#include "../garuda_foc_params.h"
#include <math.h>

#define TWO_PI  6.28318530718f
#define PI_F    3.14159265359f

void mxl_reset(MxlObs_t *obs)
{
    obs->x1           = 0.0f;
    obs->x2           = 0.0f;
    obs->i_alpha_prev = 0.0f;
    obs->i_beta_prev  = 0.0f;
    obs->theta_est    = 0.0f;
    obs->omega_est    = 0.0f;
    obs->theta_prev   = 0.0f;
}

void mxl_update(MxlObs_t *obs,
                float v_alpha, float v_beta,
                float i_alpha, float i_beta,
                float dt)
{
    /* Flux integration with delta-I compensation.
     * The Ls*(I - I_prev) term replaces Ls*(dI/dt)*dt without dividing by dt,
     * eliminating quantization noise amplification from backward difference. */
    obs->x1 += (v_alpha - MOTOR_RS_OHM * i_alpha) * dt
             - MOTOR_LS_H * (i_alpha - obs->i_alpha_prev);
    obs->x2 += (v_beta  - MOTOR_RS_OHM * i_beta)  * dt
             - MOTOR_LS_H * (i_beta  - obs->i_beta_prev);

    /* Store current samples for next delta-I */
    obs->i_alpha_prev = i_alpha;
    obs->i_beta_prev  = i_beta;

    /* Flux magnitude clamping to MOTOR_FLUX_LINKAGE.
     * Prevents integrator drift/inflation from Rs errors, offsets, etc.
     * This is the key anti-drift mechanism — equivalent to VESC's
     * flux_linkage_gain correction. */
    float mag_sq = obs->x1 * obs->x1 + obs->x2 * obs->x2;
    float lambda = MOTOR_FLUX_LINKAGE;

    if (mag_sq > 1e-12f) {  /* Avoid division by zero at startup */
        float mag = sqrtf(mag_sq);
        if (mag > lambda) {
            float scale = lambda / mag;
            obs->x1 *= scale;
            obs->x2 *= scale;
        }
    }

    /* Angle extraction: atan2(-x1, x2) gives rotor angle directly.
     * Flux vector (x1, x2) is aligned with rotor d-axis, so
     * theta_rotor = atan2(x2, x1) rotated by convention.
     * The -x1, x2 convention matches VESC's MXLEMMING output. */
    float theta = atan2f(-obs->x1, obs->x2);
    if (theta < 0.0f) theta += TWO_PI;
    obs->theta_est = theta;

    /* Speed estimation: d(angle)/dt with wrap handling */
    float dtheta = theta - obs->theta_prev;
    if (dtheta >  PI_F) dtheta -= TWO_PI;
    if (dtheta < -PI_F) dtheta += TWO_PI;
    obs->omega_est = dtheta / dt;
    obs->theta_prev = theta;
}
