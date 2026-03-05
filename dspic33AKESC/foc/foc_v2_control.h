/**
 * @file foc_v2_control.h
 * @brief FOC v2 control loop — state machine + per-tick pipeline.
 *
 * Public API called from garuda_service.c ADC ISR.
 *
 * Component: FOC V2
 */

#ifndef FOC_V2_CONTROL_H
#define FOC_V2_CONTROL_H

#include "foc_v2_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize FOC v2 state with motor parameters.
 * Call once at startup.
 */
void foc_v2_init(FOC_State_t *foc, const FOC_MotorParams_t *params);

/**
 * Fast-loop tick (24 kHz, called from ADC ISR).
 *
 * @param foc       FOC state
 * @param ia_raw    Phase A current ADC raw count
 * @param ib_raw    Phase B current ADC raw count
 * @param vbus_raw  Bus voltage ADC raw count
 * @param throttle  Throttle [0..4095]
 * @param da_out    Output duty A [0..1]
 * @param db_out    Output duty B [0..1]
 * @param dc_out    Output duty C [0..1]
 */
void foc_v2_fast_tick(FOC_State_t *foc,
                      uint16_t ia_raw, uint16_t ib_raw,
                      uint16_t vbus_raw, uint16_t throttle,
                      float *da_out, float *db_out, float *dc_out);

/**
 * Request motor start (ARMED → ALIGN).
 */
void foc_v2_start(FOC_State_t *foc);

/**
 * Request motor stop (any → ARMED).
 */
void foc_v2_stop(FOC_State_t *foc);

/**
 * Enter fault state.
 */
void foc_v2_fault(FOC_State_t *foc, uint16_t fault_code);

#ifdef __cplusplus
}
#endif

#endif /* FOC_V2_CONTROL_H */
