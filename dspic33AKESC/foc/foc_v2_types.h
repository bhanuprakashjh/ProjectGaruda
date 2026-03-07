/**
 * @file foc_v2_types.h
 * @brief FOC v2 data structures — closed-loop current control with MXLEMMING observer.
 *
 * All quantities use IEEE-754 float32 (dsPIC33AK hardware FPU).
 *
 * Sign conventions:
 *   Currents : positive = current into motor terminal
 *   Voltages : positive = higher potential at motor terminal
 *   Angle    : electrical angle in radians, range [0, 2π)
 *   Speed    : electrical rad/s, positive = forward rotation
 *
 * Component: FOC V2
 */

#ifndef FOC_V2_TYPES_H
#define FOC_V2_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── FOC v2 PI controller ─────────────────────────────────────── */
typedef struct {
    float kp;           /* Proportional gain */
    float ki;           /* Integral gain */
    float integral;     /* Running integral state */
    float error_prev;   /* Previous error (for Tustin) */
    float out_min;      /* Lower output clamp */
    float out_max;      /* Upper output clamp */
} FOC_PI_t;

/* ── MXLEMMING flux observer ──────────────────────────────────── */
typedef struct {
    float x1, x2;           /* Flux alpha/beta integrator state (V·s) */
    float lambda_est;        /* Adaptive flux linkage estimate (V·s/rad) */
    float i_alpha_last;      /* Previous Iα (for L·dI term) */
    float i_beta_last;       /* Previous Iβ (for L·dI term) */
    float theta_est;         /* atan2 angle output (rad) */
    float gain;              /* Scheduled observer gain */
} FOC_Observer_t;

/* ── PLL speed estimator (smooth ω from observer angle) ──────── */
typedef struct {
    float theta_est;         /* PLL angle tracking (rad) */
    float omega_est;         /* PLL speed estimate (rad/s) */
    float kp, ki;            /* PLL PI gains */
    float omega_max;         /* Speed clamp magnitude (rad/s) */
} FOC_PLL_t;

/* ── Motor parameters (from profile or auto-detect) ──────────── */
typedef struct {
    float Rs;                /* Phase resistance (ohm) */
    float Ls;                /* Phase inductance (H) */
    float lambda_pm;         /* PM flux linkage (V·s/rad elec) */
    float Ke;                /* Back-EMF constant (V·s/rad elec) = lambda_pm */
    uint8_t pole_pairs;      /* Motor pole pairs */
    float max_current_a;     /* Peak phase current limit (A) */
    float max_elec_rad_s;    /* Max electrical speed (rad/s) */
    float vbus_nom_v;        /* Nominal bus voltage (V) */
    /* Runtime startup/tuning params (populated from GSP) */
    float kp_dq;             /* D/Q current loop Kp */
    float ki_dq;             /* D/Q current loop Ki */
    float obs_lpf_alpha;     /* Observer LPF coefficient */
    float align_iq_a;        /* Alignment Iq (A) */
    float ramp_iq_a;         /* OL ramp Iq (A) */
    uint32_t align_ticks;    /* Alignment dwell (24kHz ticks) */
    uint32_t iq_ramp_ticks;  /* Iq ramp duration (24kHz ticks) */
    float ramp_rate_rps2;    /* I/f ramp acceleration (rad/s²) */
    float handoff_rad_s;     /* CL handoff speed threshold (rad/s) */
    float fault_oc_a;        /* Software OC threshold (A) */
    float fault_stall_rad_s; /* Stall speed threshold (rad/s) */
} FOC_MotorParams_t;

/* ── FOC state machine ───────────────────────────────────────── */
typedef enum {
    FOC_IDLE = 0,
    FOC_ARMED,
    FOC_MOTOR_DETECT,
    FOC_ALIGN,
    FOC_IF_RAMP,
    FOC_CLOSED_LOOP,
    FOC_FAULT
} FOC_Mode_t;

/* ── Motor auto-detect sub-states ────────────────────────────── */
typedef enum {
    DETECT_IDLE = 0,
    DETECT_R,
    DETECT_L,
    DETECT_ALIGN,       /* Re-align rotor at θ=0 before spinning */
    DETECT_LAMBDA,
    DETECT_TUNE,
    DETECT_DONE,
    DETECT_FAIL
} FOC_DetectState_t;

/* ── Top-level FOC v2 state ──────────────────────────────────── */
typedef struct {
    FOC_Mode_t mode;

    /* Observers */
    FOC_Observer_t obs;
    FOC_PLL_t pll;

    /* PI controllers */
    FOC_PI_t pid_d;
    FOC_PI_t pid_q;
    FOC_PI_t pid_spd;

    /* Commutation angle and speed */
    float theta;             /* Current commutation angle (rad) */
    float omega;             /* Current speed estimate (rad/s) */

    /* Current/voltage references */
    float iq_ref, id_ref;
    float vd, vq;            /* Last PI outputs (V) */

    /* I/f startup state */
    float omega_if;          /* I/f forced speed (rad/s) */
    float theta_if;          /* I/f forced angle (rad) */
    uint32_t align_ctr;      /* Alignment ticks elapsed */
    uint32_t ramp_ctr;       /* I/f ramp ticks elapsed */
    uint32_t lock_ctr;       /* Consecutive angle-lock ticks */

    /* CL state */
    bool cl_active;          /* Observer driving commutation */

    /* Slow loop divider */
    uint16_t slow_div_ctr;

    /* Motor parameters (from profile or auto-detect) */
    float Rs, Ls, lambda_pm, Ke;
    float dt_comp;           /* Dead-time compensation voltage (V) */

    /* ADC offset calibration */
    float ia_offset, ib_offset;
    uint32_t cal_accum_a, cal_accum_b;
    uint16_t cal_count;
    bool cal_done;

    /* Bus voltage */
    float vbus;

    /* Runtime startup/tuning params (copied from FOC_MotorParams_t) */
    float align_iq;          /* Alignment current (A) */
    float ramp_iq;           /* OL ramp current (A) */
    uint32_t align_ticks;    /* Alignment dwell (ticks) */
    uint32_t iq_ramp_ticks;  /* Iq transition duration (ticks) */
    float ramp_rate;         /* Speed ramp (rad/s²) */
    float handoff_rad_s;     /* CL entry speed (rad/s) */
    float fault_oc_a;        /* OC threshold (A) */
    float fault_stall_rad_s; /* Stall threshold (rad/s) */
    float obs_lpf_alpha;     /* Observer LPF alpha */
    float max_elec_rad_s;    /* Max speed (rad/s) */

    /* Overcurrent debounce */
    uint16_t oc_debounce_ctr;

    /* Bus-loss detection (HW OC tripped, no current despite voltage) */
    uint16_t busloss_ctr;

    /* Fault code (set by FOC when entering FOC_FAULT) */
    uint16_t fault_code;

    /* Stall detection */
    uint32_t stall_ctr;

    /* Arming counter */
    uint32_t arm_ctr;

    /* Telemetry (read by GSP/GUI) */
    float id_meas, iq_meas;
    float theta_obs;         /* Observer angle for telemetry */
    float omega_pll;         /* PLL speed for telemetry */
    uint8_t sub_state;       /* 0=idle,1=armed,2=align,3=if_ramp,4=cl */

    /* Motor detect state (Phase 4) */
    FOC_DetectState_t detect_state;
    uint32_t detect_ctr;
    float detect_v_accum, detect_i_accum;
    uint16_t detect_samples;
    float detect_Rs, detect_Ls, detect_lambda;
} FOC_State_t;

#ifdef __cplusplus
}
#endif

#endif /* FOC_V2_TYPES_H */
