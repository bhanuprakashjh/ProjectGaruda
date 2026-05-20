/**
 * @file foc_types.h
 * @brief Shared data structures for the FOC pipeline.
 *
 * All quantities use IEEE-754 float32.
 * dsPIC33AK128MC106 hardware FPU handles float natively —
 * addition, multiply, divide, sinf, cosf are single instructions.
 *
 * Sign conventions:
 *   Currents : positive = current into motor terminal
 *   Voltages : positive = higher potential at motor terminal
 *   Angle    : electrical angle in radians, range [0, 2π)
 *   Speed    : electrical rad/s, positive = forward rotation
 */

#ifndef FOC_TYPES_H
#define FOC_TYPES_H

/* ── Three-phase (natural abc frame) ─────────────────────────── */
typedef struct {
    float a;
    float b;
    float c;
} ThreePhase_t;

/* ── Stationary frame (α-β, Clarke output) ───────────────────── */
typedef struct {
    float alpha;
    float beta;
} AlphaBeta_t;

/* ── Rotating frame (d-q, Park output) ──────────────────────── */
typedef struct {
    float d;   /* Direct axis  — flux component   */
    float q;   /* Quadrature axis — torque component */
} DQ_t;

/* ── PI controller state ─────────────────────────────────────── */
typedef struct {
    float kp;          /* Proportional gain */
    float ki;          /* Integral gain     */
    float integrator;  /* Running sum       */
    float out_min;     /* Lower output clamp */
    float out_max;     /* Upper output clamp */
    float output;      /* Last output value  */
} PI_t;

/* ── Back-EMF observer state ─────────────────────────────────── */
typedef struct {
    float e_alpha;        /* Filtered back-EMF α (V) */
    float e_beta;         /* Filtered back-EMF β (V) */
    float i_alpha_prev;   /* Iα at previous sample   */
    float i_beta_prev;    /* Iβ at previous sample   */
} BackEMFObs_t;

/* ── PLL estimator state ─────────────────────────────────────── */
typedef struct {
    float theta_est;    /* Estimated electrical angle (rad) */
    float omega_est;    /* Estimated electrical speed (rad/s) */
    float integrator;   /* PLL integral accumulator */
} PLL_t;

/* ── Top-level motor control state ──────────────────────────── */
typedef struct {
    ThreePhase_t I_abc;    /* Phase currents (A)              */
    AlphaBeta_t  I_ab;     /* After Clarke                    */
    DQ_t         I_dq;     /* After Park                      */
    DQ_t         V_dq;     /* Current PI outputs (V)          */
    AlphaBeta_t  V_ab;     /* After Inverse Park → SVPWM      */
    float        theta;    /* Angle used this cycle (rad)     */
    float        omega;    /* Speed estimate (rad/s elec.)    */
    float        Vbus;     /* DC bus voltage (V)              */
    float        Id_ref;   /* D-axis reference (A) — usually 0 */
    float        Iq_ref;   /* Q-axis reference (A) from speed PI */
    float        speed_ref;/* Speed setpoint (rad/s elec.)   */
} FOCState_t;

#endif /* FOC_TYPES_H */
