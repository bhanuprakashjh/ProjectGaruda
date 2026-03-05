/**
 * @file foc_v2_detect.h
 * @brief Motor auto-detection: R, L, lambda measurement and PI auto-tune.
 *
 * Component: FOC V2
 */

#ifndef FOC_V2_DETECT_H
#define FOC_V2_DETECT_H

#include "foc_v2_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start motor auto-detection sequence.
 * Sets foc->detect_state to DETECT_R.
 */
void foc_detect_start(FOC_State_t *foc);

/**
 * Run one detection tick (called from slow loop, 1kHz).
 * Drives the motor through R → L → Lambda → Tune sub-states.
 *
 * @param foc   FOC state (must be in FOC_MOTOR_DETECT mode)
 * @param ia    Measured phase A current (A)
 * @param ib    Measured phase B current (A)
 * @return      true when detection is complete (DETECT_DONE or DETECT_FAIL)
 */
bool foc_detect_tick(FOC_State_t *foc, float ia, float ib);

/**
 * Get detection results.
 * Valid only after foc_detect_tick returns true with detect_state == DETECT_DONE.
 */
FOC_MotorParams_t foc_detect_get_results(const FOC_State_t *foc);

#ifdef __cplusplus
}
#endif

#endif /* FOC_V2_DETECT_H */
