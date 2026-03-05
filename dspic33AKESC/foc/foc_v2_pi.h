/**
 * @file foc_v2_pi.h
 * @brief FOC v2 PI controller with Tustin integration.
 *
 * Component: FOC V2
 */

#ifndef FOC_V2_PI_H
#define FOC_V2_PI_H

#include "foc_v2_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize PI controller.
 * @param pi    PI state
 * @param kp    Proportional gain
 * @param ki    Integral gain
 * @param lo    Output lower clamp
 * @param hi    Output upper clamp
 */
void foc_pi_init(FOC_PI_t *pi, float kp, float ki, float lo, float hi);

/**
 * Run one PI iteration (Tustin/trapezoidal integration).
 * @param pi    PI state
 * @param error Current error (reference - feedback)
 * @param dt    Sample period (s)
 * @return      Clamped output
 */
float foc_pi_run(FOC_PI_t *pi, float error, float dt);

/**
 * Preload integrator for bumpless transfer.
 * @param pi    PI state
 * @param value Integrator preload value
 */
void foc_pi_preload(FOC_PI_t *pi, float value);

/**
 * Reset PI state to zero.
 */
void foc_pi_reset(FOC_PI_t *pi);

#ifdef __cplusplus
}
#endif

#endif /* FOC_V2_PI_H */
