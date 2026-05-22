/**
 * @file  foc_pi.h
 * @brief PI controller — shared between an1078_motor.{c,h}, fwc.{c,h},
 *        and any other FOC module that needs an anti-windup PI.
 *
 * Float port of AN1078's MC_ControllerPIUpdate_Assembly.  Extracted from
 * an1078_motor.h on 2026-05-22 to break the circular include between
 * motor.h and fwc.h.
 */
#ifndef FOC_PI_H
#define FOC_PI_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;          /* proportional gain         [out/in] */
    float ki;          /* integral gain             [out/(in·s)] */
    float kc;          /* anti-windup back-calc gain [-] (1.0 = none) */
    float outMax;      /* upper saturation limit */
    float outMin;      /* lower saturation limit */
    float integrator;  /* integral state */
} AN_PI_T;

/** One-tick PI update. Returns clamped output. */
static inline float an_pi_run(AN_PI_T *pi, float ref, float meas, float dt)
{
    float err = ref - meas;
    pi->integrator += pi->ki * err * dt;

    float pre_sat = pi->integrator + pi->kp * err;
    float out = pre_sat;
    if (out > pi->outMax) out = pi->outMax;
    if (out < pi->outMin) out = pi->outMin;

    /* Anti-windup back-calc: drain integrator by the saturation excess. */
    float excess = pre_sat - out;
    pi->integrator -= (1.0f - pi->kc) * excess;

    return out;
}

#ifdef __cplusplus
}
#endif

#endif /* FOC_PI_H */
