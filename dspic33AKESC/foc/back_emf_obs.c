/**
 * @file back_emf_obs.c
 * @brief Voltage-model back-EMF observer.
 *
 * Motor parameters (MOTOR_RS_OHM, MOTOR_LS_H, OBS_LPF_ALPHA) are taken
 * from garuda_foc_params.h.
 */

#include "back_emf_obs.h"
#include "../garuda_foc_params.h"

void bemf_obs_reset(BackEMFObs_t *obs)
{
    obs->e_alpha      = 0.0f;
    obs->e_beta       = 0.0f;
    obs->i_alpha_prev = 0.0f;
    obs->i_beta_prev  = 0.0f;
}

void bemf_obs_update(BackEMFObs_t *obs,
                     float v_alpha, float v_beta,
                     float i_alpha, float i_beta,
                     float Ts)
{
    float inv_Ts = 1.0f / Ts;

    /* Backward-difference current derivatives */
    float di_a = (i_alpha - obs->i_alpha_prev) * inv_Ts;
    float di_b = (i_beta  - obs->i_beta_prev)  * inv_Ts;

    /* Voltage-model: raw back-EMF */
    float ea_raw = v_alpha - MOTOR_RS_OHM * i_alpha - MOTOR_LS_H * di_a;
    float eb_raw = v_beta  - MOTOR_RS_OHM * i_beta  - MOTOR_LS_H * di_b;

    /* First-order EMA low-pass filter */
    float alpha = OBS_LPF_ALPHA;   /* per-profile: see garuda_foc_params.h */
    obs->e_alpha = alpha * ea_raw + (1.0f - alpha) * obs->e_alpha;
    obs->e_beta  = alpha * eb_raw + (1.0f - alpha) * obs->e_beta;

    /* Save for next derivative */
    obs->i_alpha_prev = i_alpha;
    obs->i_beta_prev  = i_beta;
}
