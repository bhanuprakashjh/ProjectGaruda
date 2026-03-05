/**
 * @file pll_estimator.c
 * @brief PLL estimator -- cross-product discriminator + PI loop filter.
 *
 * Gains (PLL_KP, PLL_KI, PLL_SPEED_CLAMP) are taken from
 * garuda_foc_params.h.
 */

#include "pll_estimator.h"
#include "../garuda_foc_params.h"
#include <math.h>   /* sinf, cosf */

#define TWO_PI  6.28318530717958647692f

void pll_reset(PLL_t *pll)
{
    pll->theta_est  = 0.0f;
    pll->omega_est  = 0.0f;
    pll->integrator = 0.0f;
}

void pll_update(PLL_t *pll, float e_alpha, float e_beta, float Ts)
{
    /* Phase error via cross-product discriminator.
     * Near lock: epsilon = e_beta*cos(theta_hat) - e_alpha*sin(theta_hat)
     *          ~ (theta_true - theta_hat)*|E| */
    float s = sinf(pll->theta_est);
    float c = cosf(pll->theta_est);
    float error = e_beta * c - e_alpha * s;

    /* PI loop filter: omega_hat = Kp*epsilon + Ki*integral(epsilon) dt */
    pll->integrator += PLL_KI * Ts * error;
    float omega = PLL_KP * error + pll->integrator;

    /* Clamp speed estimate to physical limit */
    if (omega >  PLL_SPEED_CLAMP) omega =  PLL_SPEED_CLAMP;
    if (omega < -PLL_SPEED_CLAMP) omega = -PLL_SPEED_CLAMP;
    pll->omega_est = omega;

    /* Integrate angle */
    pll->theta_est += omega * Ts;

    /* Wrap to [0, 2pi) */
    while (pll->theta_est >= TWO_PI) pll->theta_est -= TWO_PI;
    while (pll->theta_est <  0.0f)  pll->theta_est += TWO_PI;
}
