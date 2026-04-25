/**
 * @file  an1078_motor.h
 * @brief AN1078 motor controller — float port (header).
 *
 * Public API for AN1078-style sensorless FOC.
 *
 * State machine (mirrors AN1078 RunMotor / OpenLoop / ChangeMode flags):
 *   STOPPED → ALIGN (LOCK) → OPEN_LOOP_RAMP → CLOSED_LOOP
 *      ↑                                          │
 *      └─── on stop / fault ─────────────────────┘
 *
 * Caller is expected to:
 *   1. Call AN_MotorInit() once at boot
 *   2. Call AN_MotorStart() / AN_MotorStop() to arm / disarm
 *   3. Call AN_MotorFastTick() from the ADC ISR every PWM cycle,
 *      passing raw ADC counts and getting back duty cycles [0..1]
 */
#ifndef AN1078_MOTOR_H
#define AN1078_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "an1078_smc.h"

/* ── Mode byte for telemetry (mirror of AN1078 uGF.bits.OpenLoop) ─ */
typedef enum {
    AN_MODE_STOPPED   = 0,
    AN_MODE_LOCK      = 1,    /* d-axis alignment, no rotation */
    AN_MODE_OPEN_LOOP = 2,    /* forced-angle ramp */
    AN_MODE_CLOSED_LOOP = 3,  /* SMO-driven commutation */
    AN_MODE_FAULT     = 4
} AN_Mode_t;

/* ── PI controller — float port of AN1078 MC_ControllerPIUpdate ─ */
typedef struct {
    float kp;          /* proportional gain         [V/A or A·s/rad] */
    float ki;          /* integral gain             [V/A·s or A/rad] */
    float kc;          /* anti-windup back-calc gain [-] (1.0 = none) */
    float outMax;      /* upper saturation limit */
    float outMin;      /* lower saturation limit */
    float integrator;  /* integral state */
} AN_PI_T;

/* ── Top-level controller state ───────────────────────────────── */
typedef struct {
    /* High-level state */
    AN_Mode_t mode;
    uint16_t  faultCode;

    /* AN1078 RunMotor / OpenLoop / ChangeMode flags */
    bool runMotor;
    bool openLoop;
    bool changeMode;

    /* SMC observer */
    AN_SMC_T smc;

    /* PI controllers */
    AN_PI_T pi_d;   /* d-axis current */
    AN_PI_T pi_q;   /* q-axis current */
    AN_PI_T pi_spd; /* speed */

    /* Open-loop startup state */
    uint32_t startupLock;       /* ticks elapsed in LOCK phase */
    float    startupRamp;       /* OL ramp acceleration accumulator (rad/s) */
    float    thetaOpenLoop;     /* OL forced commutation angle (rad) */
    float    thetaError;        /* (thetaOL - smc.Theta) at handoff, bleeds in CL */
    uint16_t transCounter;      /* paces theta_error bleed (mod TRANSITION_STEPS) */
    uint32_t handoff_dwell;     /* consecutive ticks BEMF mag has been above gate */

    /* References */
    float velRef_rad_s;         /* speed reference in CL (rad/s electrical) */
    float vqRef;                /* PI Vq output target [V] */
    float vdRef;                /* PI Vd output target [V] */
    float targetSpeed_rad_s;    /* commanded speed (from throttle) */
    float speedRefRamp;         /* per-update speed change limit */
    int16_t speedRampCount;     /* counts ticks until next speed-PI update */

    /* Frame/measurement variables (telemetry-friendly; computed each tick) */
    float ia, ib;               /* phase currents [A] */
    float i_alpha, i_beta;      /* α-β currents */
    float id_meas, iq_meas;     /* measured d-q currents */
    float vd, vq;               /* commanded d-q voltages */
    float v_alpha, v_beta;      /* α-β voltages from inverse Park */
    float theta_drive;          /* commutation angle this tick */
    float vbus;                 /* measured DC bus [V] */

    /* ADC offset calibration */
    float ia_offset;
    float ib_offset;
    uint32_t cal_accum_a;
    uint32_t cal_accum_b;
    uint16_t cal_count;
    bool cal_done;

    /* Throttle (input) */
    uint16_t throttle;
} AN_Motor_T;

/* ── Public API ───────────────────────────────────────────────── */

/** One-time init (called at boot). */
void AN_MotorInit(AN_Motor_T *m);

/** Arm and start the motor (transitions STOPPED → LOCK). */
void AN_MotorStart(AN_Motor_T *m);

/** Stop the motor immediately (transitions any → STOPPED). */
void AN_MotorStop(AN_Motor_T *m);

/** Latch a fault and stop. */
void AN_MotorFault(AN_Motor_T *m, uint16_t code);

/**
 * Fast-loop tick (call from ADC ISR at AN_FS_HZ).
 *
 * @param ia_raw, ib_raw  Raw phase-current ADC counts (12-bit, midpoint ~2048)
 * @param vbus_raw        Raw bus-voltage ADC count
 * @param throttle        Throttle input [0..4095], 0=stop
 * @param da, db, dc      Output duty cycles [0..1] (50%=zero net voltage)
 */
void AN_MotorFastTick(AN_Motor_T *m,
                      uint16_t ia_raw, uint16_t ib_raw,
                      uint16_t vbus_raw, uint16_t throttle,
                      float *da, float *db, float *dc);

#ifdef __cplusplus
}
#endif

#endif /* AN1078_MOTOR_H */
