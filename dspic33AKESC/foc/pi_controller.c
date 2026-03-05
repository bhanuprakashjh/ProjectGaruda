/**
 * @file pi_controller.c
 * @brief Discrete PI with output-clamping anti-windup.
 */

#include "pi_controller.h"

void pi_init(PI_t *ctrl, float kp, float ki, float out_min, float out_max)
{
    ctrl->kp         = kp;
    ctrl->ki         = ki;
    ctrl->out_min    = out_min;
    ctrl->out_max    = out_max;
    ctrl->integrator = 0.0f;
    ctrl->output     = 0.0f;
}

void pi_reset(PI_t *ctrl)
{
    ctrl->integrator = 0.0f;
    ctrl->output     = 0.0f;
}

float pi_update(PI_t *ctrl, float error, float Ts)
{
    float integrator_new = ctrl->integrator + ctrl->ki * Ts * error;
    float out            = ctrl->kp * error + integrator_new;

    /* Clamping anti-windup:
     * Only commit the new integrator value if the output is in range,
     * OR if the integrator update is moving away from the rail. */
    if (out > ctrl->out_max) {
        out = ctrl->out_max;
        if (integrator_new < ctrl->integrator)   /* reducing -> accept */
            ctrl->integrator = integrator_new;
    } else if (out < ctrl->out_min) {
        out = ctrl->out_min;
        if (integrator_new > ctrl->integrator)   /* reducing -> accept */
            ctrl->integrator = integrator_new;
    } else {
        ctrl->integrator = integrator_new;
    }

    ctrl->output = out;
    return out;
}
