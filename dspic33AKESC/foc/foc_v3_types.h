/**
 * @file foc_v3_types.h
 * @brief FOC v3 data structures — correct classical SMO implementation.
 *
 * Refactored from initial V3 prototype following review:
 *   - All SMO tuning in runtime SMO_Config_t (no compile-time in observer)
 *   - Observer health metrics: residual, confidence, observable flag
 *   - PLL innovation tracking
 *   - Explicit applied voltage and delay compensation state
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

/* ── SMO configuration (runtime, from motor params) ──────────────
 *
 * AN1078-style classical SMO, float port.  Fixed sliding gain and
 * speed-proportional filter cutoff, no adaptive tuning layer. */
typedef struct {
    float Rs;
    float Ls;
    float lambda_pm;
    float vbus_nom;
    float dt;

    /* Sliding controller */
    float Kslide;            /* Sliding control gain (same units as V — see .c) */
    float MaxSMCError;       /* Boundary-layer half-width (A) — above this, Z saturates */

    /* BEMF low-pass filter (speed-proportional) */
    float theta_filter_cnst; /* Kslf scale — AN1078: 2π/60·Ts */
    float kslf_min;          /* Minimum Kslf (at and below end-speed) */

    /* Angle offset (AN1078 CONSTANT_PHASE_SHIFT, radians) */
    float theta_offset;

    /* Health thresholds — kept for handoff gating (non-algorithmic) */
    float conf_min;
    float residual_max;
} SMO_Config_t;

/* ── SMO observer state ──────────────────────────────────────────
 *
 * AN1078 classical SMO, float port.  Field names `e1_alpha` (stage 1,
 * fed back to current model) and `e2_alpha` (stage 2, used for angle)
 * follow our prior convention — they correspond to AN1078's `Ealpha`
 * and `EalphaFinal` respectively. */
typedef struct {
    /* Estimated currents (α-β frame) */
    float i_hat_alpha;
    float i_hat_beta;

    /* Switching function output (raw) */
    float z_alpha;
    float z_beta;

    /* Back-EMF stage 1 — feedback into current model (AN1078: Ealpha) */
    float e1_alpha;
    float e1_beta;

    /* Back-EMF stage 2 — used for angle (AN1078: EalphaFinal) */
    float e2_alpha;
    float e2_beta;

    /* Angle + speed output */
    float theta_est;         /* [0, 2π) */
    float omega_est;         /* rad/s (LP-filtered delta-θ) */

    /* Speed estimation internals */
    float theta_prev;
    float omega_lpf;

    /* Discrete plant coefficients */
    float F;                 /* 1 - Rs*dt/Ls */
    float G;                 /* dt/Ls */

    /* Motor / timing params (stored for Rs adaptation and diagnostics) */
    float Rs_est;
    float Ls;
    float dt;
    float lambda_pm;

    /* Sliding controller params (AN1078 style) */
    float Kslide;
    float MaxSMCError;

    /* BEMF LPF params (AN1078 style) */
    float theta_filter_cnst;
    float kslf_min;
    float alpha_now;         /* Last Kslf computed (telemetry + phase-delay stub) */

    /* Angle offset (AN1078 CONSTANT_PHASE_SHIFT) */
    float theta_offset;

    /* Back-compat aliases (kept so external telemetry/config still compile) */
    float k_base;            /* = Kslide (legacy snapshot field `focObsGain`) */

    /* Current estimation error — exposed for diagnostics */
    float err_alpha;
    float err_beta;

    /* Observer health metrics (diagnostic layer, not used by observer math) */
    float bemf_mag_filt;
    float confidence;
    float residual;
    bool  observable;
    float conf_min;
    float residual_max;
} SMO_Observer_t;

/* ── PLL for SMO speed estimation ────────────────────────────── */
typedef struct {
    float theta_est;         /* PLL angle tracking (rad) */
    float omega_est;         /* PLL speed estimate (rad/s) */
    float kp, ki;            /* PLL PI gains (current, speed-adaptive) */
    float omega_max;         /* Speed clamp magnitude (rad/s) */
    float bw_min_hz;         /* Min BW at low speed (Hz) */
    float bw_max_hz;         /* Max BW at high speed (Hz) */
    float omega_bw_ref;      /* Speed at which BW = bw_max (rad/s) */
    float innovation;        /* Current phase error magnitude (rad) */
    float innovation_lpf;    /* LP-filtered innovation (rad) */
} SMO_PLL_t;

/* ── FOC v3 state machine ────────────────────────────────────── */
typedef enum {
    V3_IDLE = 0,
    V3_ARMED,
    V3_ALIGN,                /* d-axis alignment (Id injection) */
    V3_OL_RAMP,              /* Open-loop forced-angle ramp */
    V3_CLOSED_LOOP,          /* SMO-driven commutation */
    V3_VF_ASSIST,            /* V/f low-speed assist (CL fallback) */
    V3_FAULT
} V3_Mode_t;

/* ── Top-level FOC v3 state ──────────────────────────────────── */
typedef struct {
    V3_Mode_t mode;

    /* SMO observer */
    SMO_Observer_t smo;
    SMO_PLL_t pll;

    /* PI controllers (reuse FOC_PI_t from v2) */
    FOC_PI_t pid_d;
    FOC_PI_t pid_q;
    FOC_PI_t pid_spd;

    /* Commutation angle and speed */
    float theta;             /* Current commutation angle (rad) — output of CL path */
    float omega;             /* LP-filtered speed for speed PI feedback (rad/s) */

    /* Applied voltage reconstruction (stored for next tick) */
    float da_prev, db_prev, dc_prev;  /* Previous tick duty cycles [0..1] */
    float v_alpha_applied;   /* Reconstructed Vα with dead-time comp (V) */
    float v_beta_applied;    /* Reconstructed Vβ with dead-time comp (V) */

    /* CL commutation state */
    float theta_delay_comp;  /* Current dynamic phase delay compensation (rad) */

    /* Smooth OL→CL transition (AN1078 pmsm.c trick).  Captured at
     * handoff as (theta_ol − smo.theta_est); added to the commutation
     * angle in CL and bled toward zero so the rotor never sees an
     * angle step. */
    float theta_error;

    /* Current/voltage outputs */
    float iq_ref, id_ref;
    float vd, vq;            /* Last PI outputs (V) */

    /* Open-loop startup state */
    float omega_ol;          /* OL forced speed (rad/s) */
    float theta_ol;          /* OL forced angle (rad) */
    uint32_t align_ctr;      /* Alignment ticks elapsed */
    uint32_t ramp_ctr;       /* OL ramp ticks elapsed */
    uint32_t handoff_ctr;    /* Ticks at handoff speed (for dwell) */
    uint32_t angle_ok_ctr;   /* Ticks where SMO angle ≈ theta_ol */

    /* CL state */
    bool cl_active;          /* SMO driving commutation */
    uint32_t blend_ctr;      /* Ticks since CL entry (for holdoff) */

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
    bool use_vf_startup;     /* 1=V/f (low-Rs), 0=I/f (PI current control) */

    /* Handoff thresholds (runtime) */
    float handoff_conf_min;
    float handoff_residual_max;
    float handoff_bemf_min;

    /* Protection counters */
    uint16_t oc_debounce_ctr;
    uint16_t observer_bad_ctr;  /* Observer health exit counter */
    uint16_t phantom_ctr;       /* Phantom commutation counter */
    uint16_t desync_ctr;        /* Desync (excessive |Id|) counter */
    uint16_t board_fault_ctr;   /* Board fault pin debounce counter */
    uint32_t rearm_ctr;         /* Re-arm timeout counter */

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
