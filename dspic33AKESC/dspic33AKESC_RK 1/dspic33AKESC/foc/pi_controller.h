/**
 * @file pi_controller.h
 * @brief Generic discrete PI controller with output-clamping anti-windup.
 *
 * Discrete forward-Euler form:
 *   out = Kp·e + Ki·Ts·∑e
 *
 * Anti-windup: integrator update is accepted only when the output is NOT
 * saturated, or when the update would move the output away from saturation.
 * This prevents integrator wind-up without a separate tracking reference.
 */

#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include "foc_types.h"   /* PI_t */

/**
 * @brief Initialise PI state and gains.
 * @param ctrl     PI state struct
 * @param kp       Proportional gain
 * @param ki       Integral gain
 * @param out_min  Lower output clamp
 * @param out_max  Upper output clamp
 */
void pi_init(PI_t *ctrl, float kp, float ki, float out_min, float out_max);

/** @brief Zero the integrator (call on state transitions). */
void pi_reset(PI_t *ctrl);

/**
 * @brief Compute one PI step.
 * @param ctrl   PI state
 * @param error  setpoint − feedback
 * @param Ts     Sample period (s)
 * @return       Clamped output
 */
float pi_update(PI_t *ctrl, float error, float Ts);

#endif /* PI_CONTROLLER_H */
