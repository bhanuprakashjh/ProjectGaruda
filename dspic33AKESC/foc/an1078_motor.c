/**
 * @file  an1078_motor.c
 * @brief AN1078 motor controller — float port (implementation).
 *
 * Direct float translation of the relevant parts of AN1078 `pmsm.c`:
 *   - InitControlParameters()         → AN_MotorInit() helpers
 *   - ResetParmeters()                → AN_MotorStop() / reset block
 *   - DoControl()                     → an_do_control()
 *   - CalculateParkAngle()            → an_calc_park_angle()
 *   - _ADCInterrupt body              → AN_MotorFastTick()
 *
 * Algorithm preserved exactly.  Where the original used assembly
 * primitives (MC_TransformClarke_Assembly, MC_TransformPark_Assembly,
 * MC_CalculateSpaceVectorPhaseShifted_Assembly), this file inlines
 * float versions.
 *
 * NOT ported (deliberately omitted):
 *   - Field weakening (FieldWeakening / fdweak.c)
 *   - Single-shunt reconstruction
 *   - TUNING / TORQUE_MODE conditional builds
 *   - PWM/ADC peripheral setup (handled by garuda_service.c)
 */

#include "an1078_motor.h"
#include "an1078_params.h"
#include "../gsp/gsp_params.h"   /* gspParams — live FW tuning */
#include <math.h>
#include <stddef.h>

/* ── Local constants ──────────────────────────────────────────── */

#define AN_TWO_PI          6.28318530717958647692f
#define AN_PI              3.14159265358979323846f
#define AN_INV_SQRT3       0.57735026918962576451f
#define AN_SQRT3_OVER_2    0.86602540378443864676f
#define AN_TWO_OVER_THREE  0.66666666666666666667f
#define AN_ONE_OVER_THREE  0.33333333333333333333f

/* Calibration sample count (1024 → 10-bit shift average). */
#define AN_CAL_SAMPLES     1024U

/* ── PI controller (port of MC_ControllerPIUpdate_Assembly) ─────
 *
 * Q15 reference behavior:
 *   err = ref - meas
 *   integrator += ki × err - kc × prev_excess
 *   pre_sat = integrator + kp × err
 *   out = clamp(pre_sat, outMin, outMax)
 *   prev_excess = pre_sat - out   (anti-windup back-calculation)
 *
 * Float port: identical.  ki and kp are both per-tick gains
 * (not continuous), matching AN1078 convention. */
static inline float an_pi_run(AN_PI_T *pi, float ref, float meas, float dt)
{
    float err = ref - meas;
    /* Integrator update: ki has units of (output / input) per second, so
     * multiply by dt for the per-tick step. */
    pi->integrator += pi->ki * err * dt;

    float pre_sat = pi->integrator + pi->kp * err;
    float out = pre_sat;
    if (out > pi->outMax) out = pi->outMax;
    if (out < pi->outMin) out = pi->outMin;

    /* Anti-windup back-calc: subtract excess from integrator, scaled by kc.
     * AN1078 kc ≈ 0.999 → near-disabled.  We implement the same form. */
    float excess = pre_sat - out;
    pi->integrator -= (1.0f - pi->kc) * excess;

    return out;
}

static inline void an_pi_preload(AN_PI_T *pi, float v) { pi->integrator = v; }

static void an_pi_init(AN_PI_T *pi, float kp, float ki, float outMin, float outMax)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->kc = AN_PI_KC;
    pi->outMin = outMin;
    pi->outMax = outMax;
    pi->integrator = 0.0f;
}

/* ── Clarke / Park transforms (float, 1:1 with mcb library) ──── */

/* Clarke: 3-phase → α-β (assumes ia + ib + ic = 0).
 *   α = ia
 *   β = (ia + 2·ib) / √3 */
static inline void an_clarke(float ia, float ib,
                             float *out_alpha, float *out_beta)
{
    *out_alpha = ia;
    *out_beta  = (ia + 2.0f * ib) * AN_INV_SQRT3;
}

/* Park: α-β → d-q at angle θ. */
static inline void an_park(float a, float b, float sin_t, float cos_t,
                           float *d, float *q)
{
    *d =  a * cos_t + b * sin_t;
    *q = -a * sin_t + b * cos_t;
}

/* Inverse Park: d-q → α-β at angle θ. */
static inline void an_inv_park(float d, float q, float sin_t, float cos_t,
                               float *a, float *b)
{
    *a = d * cos_t - q * sin_t;
    *b = d * sin_t + q * cos_t;
}

/* ── SVPWM (port of MC_CalculateSpaceVectorPhaseShifted_Assembly) ─
 *
 * Inputs Vα, Vβ in volts (0 = midpoint).  Outputs duty [0..1] for 3
 * phases, 0.5 = zero net voltage.  Standard min-max centering.  */
static void an_svpwm(float v_alpha, float v_beta, float vbus,
                     float *da, float *db, float *dc)
{
    /* Convert to phase voltages (inverse Clarke for 3-phase output) */
    float va = v_alpha;
    float vb = -0.5f * v_alpha + AN_SQRT3_OVER_2 * v_beta;
    float vc = -0.5f * v_alpha - AN_SQRT3_OVER_2 * v_beta;

    /* Common-mode shift (min-max) for SVM; centers output to use full bus */
    float vmin = va, vmax = va;
    if (vb < vmin) vmin = vb;
    if (vb > vmax) vmax = vb;
    if (vc < vmin) vmin = vc;
    if (vc > vmax) vmax = vc;
    float vcm = 0.5f * (vmin + vmax);

    /* Convert to [0..1] duty centred at 0.5; voltage / Vbus is the gain */
    float inv_vbus = (vbus > 1.0f) ? (1.0f / vbus) : 1.0f;
    *da = 0.5f + (va - vcm) * inv_vbus;
    *db = 0.5f + (vb - vcm) * inv_vbus;
    *dc = 0.5f + (vc - vcm) * inv_vbus;

    /* Clamp to [0..1] */
    if (*da < 0.0f) *da = 0.0f; if (*da > 1.0f) *da = 1.0f;
    if (*db < 0.0f) *db = 0.0f; if (*db > 1.0f) *db = 1.0f;
    if (*dc < 0.0f) *dc = 0.0f; if (*dc > 1.0f) *dc = 1.0f;
}

/* ── ADC helpers (signed amps from raw counts about a calibrated offset) */

static inline float an_raw_to_amps(uint16_t raw, float offset)
{
#if AN_CURRENT_INVERT
    return -(((float)raw - offset) * AN_CURRENT_A_PER_COUNT);
#else
    return  (((float)raw - offset) * AN_CURRENT_A_PER_COUNT);
#endif
}

static inline float an_raw_to_vbus(uint16_t raw)
{
    return (float)raw * AN_VBUS_V_PER_COUNT;
}

/* Wrap angle into [0, 2π). */
static inline float an_wrap_2pi(float th)
{
    while (th >= AN_TWO_PI) th -= AN_TWO_PI;
    while (th <  0.0f)      th += AN_TWO_PI;
    return th;
}

/* Wrap angle into (-π, π]. */
static inline float an_wrap_delta(float dth)
{
    if (dth >  AN_PI) dth -= AN_TWO_PI;
    if (dth < -AN_PI) dth += AN_TWO_PI;
    return dth;
}

static inline float an_abs(float x) { return (x >= 0.0f) ? x : -x; }

/* ── PI initialization helper ────────────────────────────────── */

static void an_init_pi_controllers(AN_Motor_T *m)
{
    /* AN1078 InitControlParameters: per-tick Q15 gains.
     * Our floats: kp/ki are continuous-time gains, an_pi_run multiplies
     * by dt where appropriate.  Voltage clamps based on Vbus computed
     * each tick during ISR (we set a wide initial clamp here). */
    float vmax_init = 24.0f * AN_INV_SQRT3 * AN_MAX_VOLTAGE_VECTOR_FRAC;

    an_pi_init(&m->pi_d, AN_KP_DQ, AN_KI_DQ, -vmax_init, vmax_init);
    an_pi_init(&m->pi_q, AN_KP_DQ, AN_KI_DQ, -vmax_init, vmax_init);

    /* Speed PI: output = Iq reference, so clamp in amps. */
    an_pi_init(&m->pi_spd, AN_KP_SPD, AN_KI_SPD,
               -AN_OVER_CURRENT_LIMIT, AN_OVER_CURRENT_LIMIT);
}

/* ── ResetParmeters (port of pmsm.c:228) ─────────────────────── */

static void an_reset_parameters(AN_Motor_T *m)
{
    /* Stop motor */
    m->runMotor = false;
    /* Speed reference 0 */
    m->velRef_rad_s = 0.0f;
    m->id_ref_fw   = 0.0f;
    /* Restart in open loop */
    m->openLoop = true;
    /* Mode change pending (for DoControl init block) */
    m->changeMode = true;

    /* Re-init PI controllers */
    an_init_pi_controllers(m);

    /* Re-init estimator */
    AN_SMCInit(&m->smc);

    /* Reset startup state */
    m->startupLock = 0;
    m->startupRamp = 0.0f;
    m->thetaOpenLoop = 0.0f;
    m->thetaError = 0.0f;
    m->transCounter = 0;
    m->handoff_dwell = 0;

    /* Reset speed-ramp pacing */
    m->speedRampCount = AN_SPEEDREFRAMP_COUNT;
    m->speedRefRamp = AN_SPEEDREF_RAMP_RAD_S;
    m->targetSpeed_rad_s = 0.0f;
    m->vqRef = 0.0f;
    m->vdRef = 0.0f;

    /* Reset commanded voltages so SVPWM outputs 50% on first tick */
    m->vd = 0.0f;
    m->vq = 0.0f;
    m->v_alpha = 0.0f;
    m->v_beta = 0.0f;
    m->theta_drive = 0.0f;
}

/* ── Public API ───────────────────────────────────────────────── */

void AN_MotorInit(AN_Motor_T *m)
{
    /* Zero the entire struct first */
    AN_Motor_T zero = {0};
    *m = zero;

    m->mode = AN_MODE_STOPPED;
    m->faultCode = 0;
    m->ia_offset = (float)AN_ADC_MIDPOINT;
    m->ib_offset = (float)AN_ADC_MIDPOINT;
    m->cal_done = false;

    an_reset_parameters(m);
}

void AN_MotorStart(AN_Motor_T *m)
{
    if (m->mode == AN_MODE_FAULT) return;
    an_reset_parameters(m);
    m->runMotor = true;
    m->mode = AN_MODE_LOCK;
}

void AN_MotorStop(AN_Motor_T *m)
{
    an_reset_parameters(m);
    m->mode = AN_MODE_STOPPED;
}

void AN_MotorFault(AN_Motor_T *m, uint16_t code)
{
    m->mode = AN_MODE_FAULT;
    m->faultCode = code;
    m->runMotor = false;
}

/* ── DoControl (port of pmsm.c:301) ─────────────────────────────
 *
 * Runs after observer.  In open loop:
 *   - Vq follows velRef directly (q-current PI driven to fixed Iq ref)
 *   - Vd via Id PI to zero
 * In closed loop:
 *   - Speed PI: ref = velRef, meas = SMC OmegaFltred → Iq ref
 *   - Iq PI:    ref = Iq_ref, meas = idq.q          → Vq
 *   - Id PI:    ref = 0,      meas = idq.d          → Vd
 *
 * AN1078 also implements field weakening (qVdRef from FieldWeakening()),
 * we hardcode Id_ref = 0 (no FW). */
static void an_do_control(AN_Motor_T *m, float dt)
{
    if (m->openLoop) {
        /* OPEN LOOP — first-tick init block (changeMode==true) */
        if (m->changeMode) {
            m->changeMode = false;

            m->vqRef = 0.0f;
            m->vdRef = 0.0f;

            AN_SMCReset(&m->smc);

            m->startupLock = 0;
            m->startupRamp = 0.0f;
        }

        /* PWM warmup phase: first AN_WARMUP_TICKS hold Vd=Vq=0 (zero net
         * motor voltage = balanced 50% duty) so gate drivers settle and
         * bootstrap caps top off without inrush.  Skip PI entirely so
         * its integrator stays at 0 — no wind-up during warmup. */
        if (m->startupLock < AN_WARMUP_TICKS) {
            m->vq = 0.0f;
            m->vd = 0.0f;
            /* Force PI integrators to zero so they don't drift on noise */
            m->pi_q.integrator = 0.0f;
            m->pi_d.integrator = 0.0f;
        } else {
            /* Soft-start: ramp iq_ref from 0 to AN_Q_CURRENT_REF_OPENLOOP
             * over AN_IQ_SOFT_START_TICKS counted from end of warmup. */
            uint32_t ramp_tick = m->startupLock - AN_WARMUP_TICKS;
            float iq_ref;
            if (ramp_tick < AN_IQ_SOFT_START_TICKS) {
                float frac = (float)ramp_tick / (float)AN_IQ_SOFT_START_TICKS;
                iq_ref = AN_Q_CURRENT_REF_OPENLOOP * frac;
            } else {
                iq_ref = AN_Q_CURRENT_REF_OPENLOOP;
            }
            float id_ref = 0.0f;

            m->vq = an_pi_run(&m->pi_q, iq_ref, m->iq_meas, dt);
            m->vd = an_pi_run(&m->pi_d, id_ref, m->id_meas, dt);
        }
    } else {
        /* CLOSED LOOP — proper speed PI with observer feedback.
         *
         * Speed control:
         *   throttle → setpoint (rad/s)
         *   speed PI: ref=setpoint, meas=observer.OmegaFltred → iq_ref
         *   iq_ref clamped at ±AN_OVER_CURRENT_LIMIT
         *
         * Current PI inner loop runs at AN_TS rate (24kHz).
         * Speed PI on top: also runs every tick (same dt, same kp/ki
         * tuning gives same closed-loop behavior). */

        /* Throttle-mapped speed setpoint */
        float speed_target;
        if (m->throttle <= AN_THROTTLE_DEADBAND) {
            speed_target = AN_END_SPEED_ELEC_RS;
        } else {
            float frac = (float)(m->throttle - AN_THROTTLE_DEADBAND)
                       / (4095.0f - (float)AN_THROTTLE_DEADBAND);
            speed_target = AN_END_SPEED_ELEC_RS
                         + frac * (AN_NOMINAL_SPEED_ELEC_RS - AN_END_SPEED_ELEC_RS);
        }

        /* Slew velRef toward target — uses CL-specific rate, faster
         * than OL ramp so throttle feels snappy without destabilizing
         * the OL→CL handoff. */
        {
            float diff = speed_target - m->velRef_rad_s;
            float max_step = AN_CL_VELREF_SLEW_RPS2 * AN_TS;
            if (diff >  max_step) m->velRef_rad_s += max_step;
            else if (diff < -max_step) m->velRef_rad_s -= max_step;
            else m->velRef_rad_s = speed_target;
        }
        /* Keep startupRamp synced for SMC LPF (used in OL bootstrap path
         * AND as the fallback Kslf source in step 4b). */
        m->startupRamp = m->velRef_rad_s;

        /* First-tick CL init: preload speed PI with the OL Iq to be
         * bumpless, AND reset the current PIs and FW integrator.
         *
         * Why reset pi_d/pi_q: during ALIGN+OL the q-PI integrator can
         * wind up significantly (rotor at θ=0 may not accept the
         * commanded Iq, integrator pumps Vq toward the rail).  At CL
         * handoff this stale integrator drives Vq strongly → Vd then
         * rails to compensate → vq_lim drops to zero → motor stuck in
         * rail state until BOARD_PCI fires (observed 2026-04-25 with
         * AN_Q_CURRENT_REF_OPENLOOP=8A and prop load).
         *
         * Why reset id_ref_fw: stale FW from any prior CL session must
         * not leak into a fresh start. */
        if (m->changeMode) {
            m->changeMode = false;
            an_pi_preload(&m->pi_spd, AN_Q_CURRENT_REF_OPENLOOP);
            m->pi_d.integrator = 0.0f;
            m->pi_q.integrator = 0.0f;
            m->id_ref_fw       = 0.0f;
        }

        /* Speed PI → Iq ref.  Feedback from observer's REAL Omega. */
        float iq_ref = an_pi_run(&m->pi_spd,
                                 m->velRef_rad_s,
                                 m->smc.OmegaFltred,
                                 dt);
        m->vqRef = iq_ref;

        float vmax = m->vbus * AN_INV_SQRT3 * AN_MAX_VOLTAGE_VECTOR_FRAC;

        /* ── Field weakening ─────────────────────────────────────
         *
         * At ~180k eRPM on this motor, |V| saturates at vmax.  Speed
         * PI keeps demanding more Iq but voltage limiter clamps Vq →
         * speed stops increasing → no path to motor's electrical max.
         *
         * Push Id negative: in rotor frame Vq = R·Iq + ω·L·Id + ω·λ,
         * so Id<0 reduces required Vq, freeing voltage headroom for
         * higher speed.  Field is "weakened" (rotor flux partially
         * cancelled by stator d-axis MMF).
         *
         * Simple integrator: when last tick's |V| exceeded threshold,
         * accumulate id_ref_fw negative.  Decay back to zero when |V|
         * is comfortably below threshold.  Clamped to ID_FW_MAX_NEG. */
        {
            const float FW_TRIGGER = 0.91f;   /* engage just below clamp */
            const float FW_KP_INT  = 400.0f;  /* A/s per (mod-thresh) unit */
            const float FW_DECAY   = 0.995f;  /* faster decay (was 0.9995) */
            /* Live-tunable: |Id_FW_max| × 10 from gspParams (decideci-amps).
             * 0 disables FW (motor will hit voltage ceiling).  Default is
             * the compile-time -12A if user hasn't configured. */
            float id_fw_max_user = (float)gspParams.an1078IdFwMaxDecia * 0.1f;
            float ID_FW_MAX_NEG = (id_fw_max_user > 0.01f) ? -id_fw_max_user : -12.0f;
            /* Gate: FW only when motor is actively accelerating forward
             * (iq_ref > some threshold).  Without this, FW pumps in
             * negative Id during stale-integrator startup or coasting,
             * which causes runaway: high Vd → vq_lim collapses → motor
             * stuck in rail (observed 2026-04-25).  Only engage when
             * the speed PI is genuinely demanding forward torque. */
            const float FW_IQ_GATE = 1.0f;

            float v_last = sqrtf(m->vd * m->vd + m->vq * m->vq);
            float mod_now = (vmax > 0.001f) ? v_last / vmax : 0.0f;

            if (mod_now > FW_TRIGGER && iq_ref > FW_IQ_GATE) {
                m->id_ref_fw -= FW_KP_INT * (mod_now - FW_TRIGGER) * dt;
                if (m->id_ref_fw < ID_FW_MAX_NEG) m->id_ref_fw = ID_FW_MAX_NEG;
            } else {
                m->id_ref_fw *= FW_DECAY;
                if (m->id_ref_fw > -0.005f) m->id_ref_fw = 0.0f;
            }
        }
        float id_ref = m->id_ref_fw;

        m->pi_d.outMax =  vmax;
        m->pi_d.outMin = -vmax;
        m->vd = an_pi_run(&m->pi_d, id_ref, m->id_meas, dt);

        float vd_sq = m->vd * m->vd;
        float vmax_sq = vmax * vmax;
        float vq_lim_sq = vmax_sq - vd_sq;
        float vq_lim = (vq_lim_sq > 0.0f) ? sqrtf(vq_lim_sq) : 0.0f;
        m->pi_q.outMax =  vq_lim;
        m->pi_q.outMin = -vq_lim;
        m->vq = an_pi_run(&m->pi_q, iq_ref, m->iq_meas, dt);
    }
}

/* ── CalculateParkAngle (port of pmsm.c:712) ────────────────────
 *
 * Open loop: lock then ramp; capture Theta_error at handoff.
 * Closed loop: bleed Theta_error toward zero at 0.05° per tick
 * subject to (transCounter == 0). */
static void an_calc_park_angle(AN_Motor_T *m)
{
    if (m->openLoop) {
        /* LOCK: align rotor at θ=0.  startupRamp held at zero. */
        if (m->startupLock < AN_LOCK_TIME) {
            m->startupLock++;
        }
        /* RAMP: accelerate forced angle until END_SPEED reached. */
        else if (m->startupRamp < AN_END_SPEED_ELEC_RS) {
            m->startupRamp += AN_OL_RAMP_RATE_RPS2 * AN_TS;
            if (m->startupRamp > AN_END_SPEED_ELEC_RS)
                m->startupRamp = AN_END_SPEED_ELEC_RS;
            m->mode = AN_MODE_OPEN_LOOP;
        }
        /* OL→CL handoff with proper observer gate.
         *
         * After tuning Kslide=2.5V + MaxSMCError=1A + KslfScale=3*Ts +
         * KslfMin=0.05, observer locks on 2810 in OL.  BEMF magnitude
         * matches λ·ω theory (~0.25 V at 366 rad/s).  Confidence stays
         * at 1.0 reliably during OL.
         *
         * Gate: BEMF magnitude must be ≥ 80% of theoretical (λ·ω) for
         * AN_HANDOFF_DWELL_TICKS consecutive ticks.  This filters brief
         * noise spikes and ensures observer is actually tracking before
         * we trust its angle. */
        else {
            float bemf_mag = sqrtf(m->smc.EalphaFinal * m->smc.EalphaFinal
                                 + m->smc.EbetaFinal  * m->smc.EbetaFinal);
            float bemf_expected = AN_MOTOR_LAMBDA * m->startupRamp;
            float bemf_min = bemf_expected * 0.8f;

            if (bemf_mag >= bemf_min) {
                m->handoff_dwell++;
            } else {
                m->handoff_dwell = 0;
            }

            if (m->handoff_dwell >= AN_HANDOFF_DWELL_TICKS) {
                m->changeMode = true;
                m->openLoop = false;
                m->thetaError = an_wrap_delta(m->thetaOpenLoop - m->smc.Theta);
                m->mode = AN_MODE_CLOSED_LOOP;
                m->handoff_dwell = 0;
            }
            /* else: stay in OL, hold at end-speed. */
        }

        /* Advance forced angle */
        m->thetaOpenLoop += m->startupRamp * AN_TS;
        m->thetaOpenLoop = an_wrap_2pi(m->thetaOpenLoop);
    } else {
        /* CL: bleed Theta_error, paced by transCounter (mod TRANS_STEPS) */
        if (m->transCounter == 0) {
            if (an_abs(m->thetaError) > AN_THETA_ERROR_BLEED_RAD) {
                if (m->thetaError < 0.0f) {
                    m->thetaError += AN_THETA_ERROR_BLEED_RAD;
                } else {
                    m->thetaError -= AN_THETA_ERROR_BLEED_RAD;
                }
            } else {
                m->thetaError = 0.0f;
            }
        }
    }

    /* transCounter wraps every TRANSITION_STEPS ticks */
    m->transCounter++;
    if (m->transCounter >= AN_TRANSITION_STEPS) m->transCounter = 0;
}

/* ── ADC offset calibration ────────────────────────────────── */

static void an_offset_cal(AN_Motor_T *m, uint16_t ia_raw, uint16_t ib_raw)
{
    if (m->cal_done) return;
    m->cal_accum_a += ia_raw;
    m->cal_accum_b += ib_raw;
    m->cal_count++;
    if (m->cal_count >= AN_CAL_SAMPLES) {
        m->ia_offset = (float)(m->cal_accum_a >> 10);  /* /1024 */
        m->ib_offset = (float)(m->cal_accum_b >> 10);
        m->cal_done = true;
    }
}

/* ── Fast-loop tick (port of _ADCInterrupt body, dual-shunt path) ─ */

void AN_MotorFastTick(AN_Motor_T *m,
                      uint16_t ia_raw, uint16_t ib_raw,
                      uint16_t vbus_raw, uint16_t throttle,
                      float *da, float *db, float *dc)
{
    /* Default: 50% duty (zero net voltage). */
    *da = 0.5f;
    *db = 0.5f;
    *dc = 0.5f;

    /* Stash inputs */
    m->throttle = throttle;
    m->vbus = an_raw_to_vbus(vbus_raw);

    /* When stopped, run offset cal and return idle. */
    if (!m->runMotor) {
        an_offset_cal(m, ia_raw, ib_raw);
        return;
    }

    if (m->mode == AN_MODE_FAULT) {
        return;  /* outputs at 50% (zero V) — caller may override to disable */
    }

    /* ── 1. Phase currents (ADC → amps) ──────────────────── */
    m->ia = an_raw_to_amps(ia_raw, m->ia_offset);
    m->ib = an_raw_to_amps(ib_raw, m->ib_offset);

    /* ── 2. Clarke ───────────────────────────────────────── */
    an_clarke(m->ia, m->ib, &m->i_alpha, &m->i_beta);

    /* ── 3. Park (using PREVIOUS tick's commutation angle) ─
     * AN1078 uses sincosTheta computed at the END of the previous
     * tick (line 620), so id/iq here is "what we measured given
     * the angle we WERE driving."  We replicate by carrying the
     * previous theta_drive in m->theta_drive. */
    {
        float sin_t = sinf(m->theta_drive);
        float cos_t = cosf(m->theta_drive);
        an_park(m->i_alpha, m->i_beta, sin_t, cos_t,
                &m->id_meas, &m->iq_meas);
    }

    /* ── 4. Feed observer ────────────────────────────────── */
    m->smc.Ialpha = m->i_alpha;
    m->smc.Ibeta  = m->i_beta;
    m->smc.Valpha = m->v_alpha;   /* from previous tick's inverse Park */
    m->smc.Vbeta  = m->v_beta;
    AN_SMC_Position_Estimation(&m->smc);

    /* ── 4b. SMC LPF tuning — pin Kslf to commanded speed (startupRamp).
     *
     * startupRamp tracks velRef (the slewed speed setpoint from
     * throttle).  Using it for Kslf gives a stable, predictable LPF
     * cutoff that scales with intended operating speed.
     *
     * Why not use observer's own OmegaFltred for Kslf?  Tested — caused
     * positive feedback at high speed: motor accelerates → observer
     * reports higher → Kslf grows → LPF lets through more BEMF →
     * observer angle improves → motor accelerates more → no control.
     *
     * OmegaFltred is also overridden in OL (bootstrap), but in CL we
     * leave the observer's native value alone for telemetry/PI
     * feedback. */
    if (m->openLoop) {
        m->smc.OmegaFltred = m->startupRamp;   /* OL bootstrap */
    }
    {
        float k = m->startupRamp * m->smc.KslfScale;
        if (k < m->smc.KslfMin)      k = m->smc.KslfMin;
        if (k > AN_SMC_KSLF_MAX)     k = AN_SMC_KSLF_MAX;
        m->smc.Kslf      = k;
        m->smc.KslfFinal = k;
    }

    /* ── 5. Run control (PI loops) ──────────────────────── */
    an_do_control(m, AN_TS);

    /* ── 6. Compute commutation angle (OL ramp or CL bleed) ─ */
    an_calc_park_angle(m);

    /* ── 7. Choose theta_drive: OL angle or (SMC + Theta_error) ─ */
    if (m->openLoop) {
        m->theta_drive = m->thetaOpenLoop;
    } else {
        m->theta_drive = an_wrap_2pi(m->smc.Theta + m->thetaError);
    }

    /* ── 8. Inverse Park: (vd, vq) → (vα, vβ) ──────────── */
    {
        float sin_t = sinf(m->theta_drive);
        float cos_t = cosf(m->theta_drive);
        an_inv_park(m->vd, m->vq, sin_t, cos_t, &m->v_alpha, &m->v_beta);
    }

    /* ── 9. SVPWM → duty cycles ─────────────────────────── */
    an_svpwm(m->v_alpha, m->v_beta, m->vbus, da, db, dc);
}
