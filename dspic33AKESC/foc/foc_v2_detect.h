/**
 * @file foc_v2_detect.h
 * @brief Motor auto-commissioning: measure Rs, Ls, λ_pm and auto-tune PI.
 *
 * Uses existing FOC PI + SVPWM infrastructure — all measurements happen
 * through the normal fast-loop current control path.
 *
 * Sequence:
 *   DETECT_R:      Command Id at θ=0 (rotor locked), measure Vd/Id → Rs
 *   DETECT_L:      Voltage step on d-axis, measure dI/dt → Ls
 *   DETECT_ALIGN:  Re-align rotor at θ=0 + smooth d→q transition
 *   DETECT_LAMBDA: Spin with I/f at known ω, measure Vq-Rs*Iq → Ke*ω → λ_pm
 *   DETECT_TUNE:   Compute PI gains from measured Rs, Ls
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
 * Caller must set foc->mode = FOC_MOTOR_DETECT before calling.
 */
void foc_detect_start(FOC_State_t *foc);

/**
 * Fast-loop detect tick (24 kHz, called from foc_v2_fast_tick when mode==MOTOR_DETECT).
 *
 * Drives current/voltage and measures response.
 * Returns duty outputs via da/db/dc pointers.
 *
 * @param foc       FOC state (must be in FOC_MOTOR_DETECT mode)
 * @param ia        Measured phase A current (A, calibrated)
 * @param ib        Measured phase B current (A, calibrated)
 * @param i_alpha   Clarke α current (A)
 * @param i_beta    Clarke β current (A)
 * @param da_out    Output duty A [0..1]
 * @param db_out    Output duty B [0..1]
 * @param dc_out    Output duty C [0..1]
 * @return          true when detection is complete (DETECT_DONE or DETECT_FAIL)
 */
bool foc_detect_fast_tick(FOC_State_t *foc,
                          float ia, float ib,
                          float i_alpha, float i_beta,
                          float *da_out, float *db_out, float *dc_out);

/**
 * Apply detected params to FOC state (Rs, Ls, Ke, PI gains, observer).
 * Call after detect completes with DETECT_DONE.
 */
void foc_detect_apply(FOC_State_t *foc);

#ifdef __cplusplus
}
#endif

#endif /* FOC_V2_DETECT_H */
