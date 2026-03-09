/**
 * @file foc_v3_control.h
 * @brief FOC v3 control loop — SMO observer + open-loop startup.
 *
 * Public API called from garuda_service.c ADC ISR.
 * Same interface as foc_v2_control.h for easy swapping.
 *
 * Component: FOC V3
 */

#ifndef FOC_V3_CONTROL_H
#define FOC_V3_CONTROL_H

#include "foc_v2_types.h"   /* FOC_MotorParams_t, FOC_PI_t */
#include "foc_v3_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize FOC v3 state with motor parameters.
 * Call once at startup.
 */
void foc_v3_init(V3_State_t *st, const FOC_MotorParams_t *params);

/**
 * Fast-loop tick (24 kHz, called from ADC ISR).
 *
 * @param st        FOC v3 state
 * @param ia_raw    Phase A current ADC raw count
 * @param ib_raw    Phase B current ADC raw count
 * @param vbus_raw  Bus voltage ADC raw count
 * @param throttle  Throttle [0..4095]
 * @param da_out    Output duty A [0..1]
 * @param db_out    Output duty B [0..1]
 * @param dc_out    Output duty C [0..1]
 */
void foc_v3_fast_tick(V3_State_t *st,
                      uint16_t ia_raw, uint16_t ib_raw,
                      uint16_t vbus_raw, uint16_t throttle,
                      float *da_out, float *db_out, float *dc_out);

/**
 * Request motor start (ARMED → ALIGN).
 */
void foc_v3_start(V3_State_t *st);

/**
 * Request motor stop (any → ARMED).
 */
void foc_v3_stop(V3_State_t *st);

/**
 * Enter fault state.
 */
void foc_v3_fault(V3_State_t *st, uint16_t fault_code);

#ifdef __cplusplus
}
#endif

#endif /* FOC_V3_CONTROL_H */
