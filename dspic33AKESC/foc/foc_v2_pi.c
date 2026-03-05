/**
 * @file foc_v2_pi.c
 * @brief FOC v2 PI controller with Tustin (trapezoidal) integration.
 *
 * Anti-windup: integrator and output clamped to same limits.
 * Tustin gives better high-frequency accuracy than forward Euler.
 *
 * Component: FOC V2
 */

#include "foc_v2_pi.h"
#include "foc_v2_math.h"

void foc_pi_init(FOC_PI_t *pi, float kp, float ki, float lo, float hi)
{
    pi->kp         = kp;
    pi->ki         = ki;
    pi->integral   = 0.0f;
    pi->error_prev = 0.0f;
    pi->out_min    = lo;
    pi->out_max    = hi;
}

float foc_pi_run(FOC_PI_t *pi, float error, float dt)
{
    /* Tustin (trapezoidal) integration: integral += Ki * dt/2 * (e[n] + e[n-1]) */
    float integral_new = pi->integral + pi->ki * dt * 0.5f * (error + pi->error_prev);

    /* Compute output */
    float output = pi->kp * error + integral_new;

    /* Clamp output */
    output = foc_clampf(output, pi->out_min, pi->out_max);

    /* Anti-windup: clamp integrator to output limits.
     * Prevents integrator from growing beyond what the output can deliver. */
    integral_new = foc_clampf(integral_new, pi->out_min, pi->out_max);

    pi->integral   = integral_new;
    pi->error_prev = error;

    return output;
}

void foc_pi_preload(FOC_PI_t *pi, float value)
{
    pi->integral = foc_clampf(value, pi->out_min, pi->out_max);
}

void foc_pi_reset(FOC_PI_t *pi)
{
    pi->integral   = 0.0f;
    pi->error_prev = 0.0f;
}
