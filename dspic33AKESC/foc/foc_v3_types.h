/**
 * @file foc_v3_types.h
 * @brief FOC v3 data structures — SMO observer + open-loop startup.
 *
 * FOC v3 uses a Sliding Mode Observer (SMO) for sensorless position
 * estimation, based on Microchip AN1078. The SMO is a current-model
 * observer with sigmoid switching function and cascaded LPF for
 * back-EMF extraction.
 *
 * Key difference from v2 (MXLEMMING flux observer):
 *   - v2: integrates voltage to get flux, atan2(flux) for angle
 *   - v3: predicts current, error drives switching function,
 *         LPF extracts back-EMF, atan2(BEMF) for angle
 *   - SMO has inherent closed-loop correction (sliding mode)
 *   - v2 was open-loop integrator → diverged at high speed
 *
 * All quantities use IEEE-754 float32 (dsPIC33AK hardware FPU).
 *
 * Component: FOC V3
 */

#ifndef FOC_V3_TYPES_H
#define FOC_V3_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "foc_v2_types.h"    /* FOC_PI_t, FOC_MotorParams_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ── SMO observer state ──────────────────────────────────────── */
typedef struct {
    /* Estimated currents (α-β frame) */
    float i_hat_alpha;
    float i_hat_beta;

    /* Switching function output (raw) */
    float z_alpha;
    float z_beta;

    /* Back-EMF estimates — stage 1 (feedback to current model) */
    float e_alpha;
    float e_beta;

    /* Back-EMF estimates — stage 2 (filtered, used for angle) */
    float e_alpha_filt;
    float e_beta_filt;

    /* Angle and speed output */
    float theta_est;         /* atan2 of filtered BEMF (rad, [0, 2π)) */
    float omega_est;         /* Speed from delta-theta (rad/s) */

    /* Speed estimation internals */
    float theta_prev;        /* Previous theta for delta computation */
    float omega_lpf;         /* LP-filtered speed for smooth output */

    /* Discrete plant coefficients (precomputed) */
    float F;                 /* 1 - Rs*Ts/Ls (plant pole) */
    float G;                 /* Ts/Ls (plant gain) */

    /* Tuning parameters (cached from init) */
    float K_max;             /* Sliding gain ceiling (V) — Vbus */
    float K_base;            /* Min sliding gain (V) — 1.5·Ls/dt, controls chattering */
    float lambda_pm;         /* Flux linkage (V·s/rad) — for adaptive K */
    float phi;               /* Sigmoid boundary layer (A) */
    float lpf_alpha;         /* BEMF LPF coefficient (speed-adaptive base) */
} SMO_Observer_t;

/* ── PLL for SMO speed estimation ────────────────────────────── */
typedef struct {
    float theta_est;         /* PLL angle tracking (rad) */
    float omega_est;         /* PLL speed estimate (rad/s) */
    float kp, ki;            /* PLL PI gains */
    float omega_max;         /* Speed clamp magnitude (rad/s) */
} SMO_PLL_t;

/* ── FOC v3 state machine ────────────────────────────────────── */
typedef enum {
    V3_IDLE = 0,
    V3_ARMED,
    V3_ALIGN,                /* d-axis alignment (Id injection) */
    V3_OL_RAMP,              /* Open-loop forced-angle ramp */
    V3_CLOSED_LOOP,          /* SMO-driven commutation */
    V3_FAULT
} V3_Mode_t;

/* ── Top-level FOC v3 state ──────────────────────────────────── */
typedef struct {
    V3_Mode_t mode;

    /* SMO observer */
    SMO_Observer_t smo;
    SMO_PLL_t pll;

    /* PI controllers (reuse FOC_PI_t from v2) */
    /* Forward-declared — actual type from foc_v2_types.h */
    FOC_PI_t pid_d;
    FOC_PI_t pid_q;
    FOC_PI_t pid_spd;

    /* Commutation angle and speed */
    float theta;             /* Current commutation angle (rad) */
    float omega;             /* Current speed estimate (rad/s) */

    /* Current/voltage references */
    float iq_ref, id_ref;
    float vd, vq;            /* Last PI outputs (V) */

    /* Open-loop startup state */
    float omega_ol;          /* OL forced speed (rad/s) */
    float theta_ol;          /* OL forced angle (rad) */
    uint32_t align_ctr;      /* Alignment ticks elapsed */
    uint32_t ramp_ctr;       /* OL ramp ticks elapsed */
    uint32_t handoff_ctr;    /* Ticks at handoff speed (for dwell) */

    /* CL state */
    bool cl_active;          /* SMO driving commutation */

    /* Slow loop divider */
    uint16_t slow_div_ctr;

    /* Motor parameters */
    float Rs, Ls, lambda_pm, Ke;
    float dt_comp;           /* Dead-time compensation voltage (V) */

    /* ADC offset calibration */
    float ia_offset, ib_offset;
    uint32_t cal_accum_a, cal_accum_b;
    uint16_t cal_count;
    bool cal_done;

    /* Bus voltage */
    float vbus;

    /* Runtime startup/tuning params */
    float align_iq;
    float ramp_iq;
    uint32_t align_ticks;
    uint32_t iq_ramp_ticks;   /* d→q crossfade ticks at OL_RAMP entry */
    float ramp_rate;
    float handoff_rad_s;
    float fault_oc_a;
    float max_elec_rad_s;

    /* Overcurrent debounce */
    uint16_t oc_debounce_ctr;

    /* Fault code */
    uint16_t fault_code;

    /* Arming counter */
    uint32_t arm_ctr;

    /* Telemetry */
    float id_meas, iq_meas;
    float theta_obs;
    float omega_pll;
    uint8_t sub_state;
} V3_State_t;

#ifdef __cplusplus
}
#endif

#endif /* FOC_V3_TYPES_H */
