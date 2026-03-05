/**
 * @file smo_observer.c
 * @brief Sliding Mode Observer (SMO) implementation.
 *
 * Discrete-time sliding mode observer using sigmoid switching function.
 * All motor/timing parameters are compile-time constants from
 * garuda_foc_params.h -- no runtime division or parameter lookup.
 *
 * Computational cost: ~60-80 cycles per tick on dsPIC33AK hardware FPU.
 * (2x sigmoid: 2 fabsf + 2 add + 2 div = ~30 cycles)
 * (2x current model: 4 mul + 4 add = ~12 cycles)
 * (2x LPF: 4 mul + 2 add = ~10 cycles)
 * (overhead: ~10-20 cycles)
 *
 * Component: FOC
 */

#include "smo_observer.h"
#include "../garuda_foc_params.h"
#include <math.h>   /* fabsf -- single-cycle on hardware FPU */

/* Precomputed discrete-time system constants (compile-time).
 * Forward Euler: i[k+1] = A*i[k] + B*(V[k] - Z[k])
 *   A = 1 - Rs*Ts/Ls  (discrete system pole)
 *   B = Ts/Ls          (discrete input gain) */
#define SMO_A   (1.0f - MOTOR_RS_OHM * FOC_TS_FAST_S / MOTOR_LS_H)
#define SMO_B   (FOC_TS_FAST_S / MOTOR_LS_H)

/* Sliding gain -- must exceed maximum expected back-EMF magnitude.
 * Using Vbus_nom: at any operating point, BEMF < Vbus (motor can't
 * produce more EMF than the supply under normal operation). */
#define SMO_K   SMO_GAIN

void smo_reset(SMO_t *smo)
{
    smo->i_hat_alpha = 0.0f;
    smo->i_hat_beta  = 0.0f;
    smo->e_alpha     = 0.0f;
    smo->e_beta      = 0.0f;
    smo->z_alpha     = 0.0f;
    smo->z_beta      = 0.0f;
}

void smo_update(SMO_t *smo,
                float v_alpha, float v_beta,
                float i_alpha, float i_beta)
{
    /* 1. Current estimation error */
    float err_a = smo->i_hat_alpha - i_alpha;
    float err_b = smo->i_hat_beta  - i_beta;

    /* 2. Sigmoid switching function: F(x) = x / (|x| + phi)
     *    - Smooth approximation of signum: eliminates chattering
     *    - Near zero: linear with slope 1/phi (high gain)
     *    - Far from zero: saturates at +/-1 (like signum)
     *    - phi controls boundary layer width (amps)
     *    - Cost: 1 fabsf + 1 add + 1 div per axis */
    float sig_a = err_a / (fabsf(err_a) + SMO_SIGMOID_PHI);
    float sig_b = err_b / (fabsf(err_b) + SMO_SIGMOID_PHI);

    /* 3. Switching terms (converge to back-EMF when observer slides) */
    float z_a = SMO_K * sig_a;
    float z_b = SMO_K * sig_b;

    /* 4. Update current estimate (discrete motor model with correction)
     *    i_hat[k+1] = A * i_hat[k] + B * (V - Z)
     *    The switching term Z drives i_hat toward i_measured. */
    smo->i_hat_alpha = SMO_A * smo->i_hat_alpha + SMO_B * (v_alpha - z_a);
    smo->i_hat_beta  = SMO_A * smo->i_hat_beta  + SMO_B * (v_beta  - z_b);

    /* 5. Low-pass filter switching signal to extract back-EMF.
     *    EMA: e = alpha * z + (1-alpha) * e_prev
     *    The unfiltered Z contains BEMF + chattering remnants.
     *    LPF removes chattering; PLL handles remaining filtering. */
    smo->z_alpha = z_a;
    smo->z_beta  = z_b;
    smo->e_alpha = SMO_LPF_ALPHA * z_a + (1.0f - SMO_LPF_ALPHA) * smo->e_alpha;
    smo->e_beta  = SMO_LPF_ALPHA * z_b + (1.0f - SMO_LPF_ALPHA) * smo->e_beta;
}
