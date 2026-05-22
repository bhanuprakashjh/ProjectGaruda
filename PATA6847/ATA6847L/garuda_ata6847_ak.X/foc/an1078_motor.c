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

#include "../garuda_config.h"

#if FEATURE_FOC_AN1078

#include "an1078_motor.h"
#include "an1078_params.h"
#include "foc_shim_gsp.h"        /* AK port: stub for gspParams runtime tuning */
#include <math.h>
#include <stddef.h>

/* Observer selection — AN1078 SMO (default) or ESMO drop-in. Both
 * operate on the same AN_SMC_T struct so the dozens of `m->smc.field`
 * reads below are unchanged across paths. See foc/esmo_observer.h. */
#if FEATURE_FOC_ESMO
#include "esmo_observer.h"
#define OBS_INIT(s)    ESMO_Init(s)
#define OBS_RESET(s)   ESMO_Reset(s)
#define OBS_UPDATE(s)  ESMO_Update(s)
#else
#define OBS_INIT(s)    AN_SMCInit(s)
#define OBS_RESET(s)   AN_SMCReset(s)
#define OBS_UPDATE(s)  AN_SMC_Position_Estimation(s)
#endif

/* ── Local constants ──────────────────────────────────────────── */

#define AN_TWO_PI          6.28318530717958647692f
#define AN_PI              3.14159265358979323846f
#define AN_INV_SQRT3       0.57735026918962576451f
#define AN_SQRT3_OVER_2    0.86602540378443864676f
#define AN_TWO_OVER_THREE  0.66666666666666666667f
#define AN_ONE_OVER_THREE  0.33333333333333333333f

/* Calibration sample count (1024 → 10-bit shift average). */
#define AN_CAL_SAMPLES     1024U

/* an_pi_run() lives in an1078_motor.h (header inline) so it can be
 * reused by other FOC modules — foc/fwc.c needs it for the angle PI. */
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
    OBS_INIT(&m->smc);

#if FEATURE_FWC
    /* FWC: angle-based field-weakening controller (foc/fwc.h).
     *
     * Bench tuning history:
     *   Init  : kp=0.05  ki=1.5  max=0.785 (π/4 / 45°)
     *           Result: PI hunting (42% engaged / 39% disengaged samples
     *           at top), Id excursions ±18 A.  Peak 207k.
     *   Now   : kp=0.02  ki=0.5  max=1.05  (60°)
     *           Calmer PI (less twitchy → no hunting) + deeper FW depth
     *           (Id can reach −0.87·Is at full angle vs −0.71 at 45°). */
    /* FW depth comes from garuda_config.h:FWC_ANGLE_MAX_RAD.
     *   0.0f       → FW DISABLED (Id_ref always 0)
     *   1.05f (60°) → deep FW (Id ≤ -0.87·|Is_ref|) */
    FWC_Init(&m->fwc, /*kp*/ 0.02f, /*ki*/ 0.5f,
             /*angle_fw_max*/ FWC_ANGLE_MAX_RAD);
#endif

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
    /* Reset FF current LPF state — must zero on every (re)start so a
     * previous run's filtered currents don't leak into the first FF
     * computation when ω is rapidly rising. */
    m->id_filt = 0.0f;
    m->iq_filt = 0.0f;
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

            OBS_RESET(&m->smc);

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

        /* Throttle-mapped speed setpoint.  Max speed comes from
         * gspParams.focMaxElecRadS (per-profile, GUI-editable) so the
         * pot range matches what the actual motor can physically reach.
         * Without this, switching profiles in the GUI worked for the
         * motor model but the pot still mapped 0-100% to the previous
         * motor's max — second half of pot had no effect on speed. */
        float max_speed_target = (gspParams.focMaxElecRadS > 0)
                               ? (float)gspParams.focMaxElecRadS
                               : AN_NOMINAL_SPEED_ELEC_RS;
        float speed_target;
        if (m->throttle <= AN_THROTTLE_DEADBAND) {
            speed_target = AN_END_SPEED_ELEC_RS;
        } else {
            float frac = (float)(m->throttle - AN_THROTTLE_DEADBAND)
                       / (4095.0f - (float)AN_THROTTLE_DEADBAND);
            speed_target = AN_END_SPEED_ELEC_RS
                         + frac * (max_speed_target - AN_END_SPEED_ELEC_RS);
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

        /* ── VelRef saturation guard ───────────────────────────────
         * When the motor is voltage-saturated (deep FW, modulation at
         * or above MOD_GUARD), the rotor physically cannot accelerate
         * further regardless of how high velRef goes.  Without this
         * guard, velRef keeps climbing → speed PI sees ever-larger
         * error → demands max Iq → bus current spikes → vbus dips →
         * USB EMI / brownout → MCU reset → motor desync.
         *
         * Symptom seen at the 227k peak (Phase A, foc_run_143856):
         *   t=33.46  velRef→max  Iq=+2.5     OK
         *   t=33.66  velRef→max  Iq=+16.7   ← spike (PI windup)
         *   t=33.86               Vbus=22.7  ← bus dipped → USB reset
         *
         * Cap velRef to (OmegaFltred + small lead) while saturated.
         * Below MOD_GUARD the guard is inactive — normal throttle
         * response.  Uses the PREVIOUS tick's vd/vq (no extra cost).
         *
         * Tuning notes:
         *  - MOD_GUARD = 0.93  triggers slightly before the FW corner
         *    at 0.91 so PI windup is caught early.
         *  - SPEED_LEAD = 500 rad/s ≈ 4.8 k eRPM headroom for the PI
         *    to still drive the motor up the remaining range without
         *    being able to run away. */
        {
            float vmax_guard = m->vbus * AN_INV_SQRT3 * AN_MAX_VOLTAGE_VECTOR_FRAC;
            float v_last  = sqrtf(m->vd * m->vd + m->vq * m->vq);
            float mod_now = (vmax_guard > 0.001f) ? v_last / vmax_guard : 0.0f;
            const float MOD_GUARD    = 0.93f;
            const float SPEED_LEAD   = 500.0f;  /* rad/s electrical */
            const float SAT_INT_MAX  = 5.0f;    /* A — tighter than ±18A
                                                 * outMax so integrator
                                                 * can't run away under
                                                 * sustained saturation */
            if (mod_now > MOD_GUARD) {
                /* (a) Cap velRef so speed-PI's P-term contribution stays
                 *     bounded — error capped at SPEED_LEAD,
                 *     Kp_spd × LEAD = 0.015 × 500 = 7.5 A worst case. */
                float vel_cap = m->smc.OmegaFltred + SPEED_LEAD;
                if (m->velRef_rad_s > vel_cap) m->velRef_rad_s = vel_cap;

                /* (b) Clamp the integrator to a tight envelope while
                 *     saturated.  Existing back-calc (kc=0.999) is too
                 *     gentle for sustained-saturation operation; this
                 *     hard-clamp prevents windup to the 18 A output
                 *     limit.  Total Iq ≤ P_max + SAT_INT_MAX ≈ 12.5 A,
                 *     well below the +13-16 A spikes that triggered USB
                 *     resets in earlier runs. */
                if (m->pi_spd.integrator >  SAT_INT_MAX) m->pi_spd.integrator =  SAT_INT_MAX;
                if (m->pi_spd.integrator < -SAT_INT_MAX) m->pi_spd.integrator = -SAT_INT_MAX;
            }
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
            /* Bumpless OL→CL transition.  Without this the speed PI sees
             * velRef=0 vs measured=ω_handoff (~1099 rad/s) at first CL
             * tick, computes a huge negative error, demands -12A Iq,
             * motor brakes hard immediately, observer can't keep up,
             * desync within 100 ms.  Setting velRef=measured makes the
             * initial error ~0 so Iq stays smooth across the boundary. */
            m->velRef_rad_s = m->smc.OmegaFltred;
            an_pi_preload(&m->pi_spd, AN_Q_CURRENT_REF_OPENLOOP);
            m->pi_d.integrator = 0.0f;
            m->pi_q.integrator = 0.0f;
            m->id_ref_fw       = 0.0f;
        }

        /* Speed PI → Iq ref.  Feedback from observer's REAL Omega.
         * iq_ref here is treated as the STATOR CURRENT MAGNITUDE Is_ref
         * when FWC is active (FWC splits it into Id_ref / Iq_ref via
         * angle).  When FWC is off, iq_ref maps directly to Iq with Id
         * driven separately by the legacy integrator below. */
        float iq_ref = an_pi_run(&m->pi_spd,
                                 m->velRef_rad_s,
                                 m->smc.OmegaFltred,
                                 dt);
        m->vqRef = iq_ref;

        float vmax = m->vbus * AN_INV_SQRT3 * AN_MAX_VOLTAGE_VECTOR_FRAC;

#if FEATURE_FWC
        /* ── FWC: angle-based field weakening ─────────────────────
         *
         * vs_now: |V| from previous tick's PI outputs.
         * vsRef:  voltage threshold at which FWC engages.  Use 95% of
         *         vmax — same trigger as the legacy 0.91 mod ratio
         *         but expressed in volts since that's FWC's unit. */
        {
            float vs_now = sqrtf(m->vd * m->vd + m->vq * m->vq);
            float vsRef  = 0.95f * vmax;

            /* Gate FWC on positive speed-PI demand (avoid pumping FW
             * during coast/brake — same protection the legacy path had). */
            const float FWC_IQ_GATE = 1.0f;
            if (iq_ref > FWC_IQ_GATE) FWC_Enable(&m->fwc);
            else                      FWC_Disable(&m->fwc);

            FWC_RunAngle(&m->fwc, vs_now, vsRef, dt);
        }

        /* Split iq_ref (= Is magnitude) into (Id_ref, Iq_ref) via angle.
         *   angle_fw = 0     → Id_ref = 0,             Iq_ref = iq_ref
         *   angle_fw > 0     → Id_ref < 0 (FW),        Iq_ref < iq_ref */
        float id_ref;
        {
            float id_split, iq_split;
            FWC_SplitCurrent(&m->fwc, iq_ref, &id_split, &iq_split);
            id_ref = id_split;
            iq_ref = iq_split;
        }
#else
        /* ── Legacy field weakening (id_ref_fw integrator) ──────
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
            const float FW_RELEASE = 0.86f;   /* hysteresis — release lower than trigger */
            const float FW_KP_INT  = 150.0f;  /* A/s per (mod-thresh) unit.  400 caused
                                                 * a ~30 Hz limit cycle near the voltage
                                                 * saturation boundary on A2212 (Id swung
                                                 * -1 to -6 A while speed bounced).
                                                 * 150 damps that without losing depth. */
            const float FW_DECAY   = 0.995f;
            /* Live-tunable: |Id_FW_max| × 10 from gspParams (deci-amps).
             *   value 0   → ID_FW_MAX_NEG = 0 → FW truly DISABLED (id_ref_fw
             *               clamps to 0, no field weakening current commanded).
             *   value 120 → ID_FW_MAX_NEG = -12 A → standard 2810 setup.
             *   value 200 → ID_FW_MAX_NEG = -20 A → max range; usually limited
             *               by bench supply or AN_OVER_CURRENT_LIMIT before
             *               this is actually reached.
             * NOTE: more FW does NOT always mean more top speed.  Past a
             * point, the supply current limit + Iq-clamp interaction causes
             * top speed to plateau or drop (observed 2026-04-26 with 2810). */
            float id_fw_max_user = (float)gspParams.an1078IdFwMaxDecia * 0.1f;
            float ID_FW_MAX_NEG = -id_fw_max_user;   /* 0 = disabled */
            /* Gate: FW only when motor is actively accelerating forward
             * (iq_ref > some threshold).  Without this, FW pumps in
             * negative Id during stale-integrator startup or coasting,
             * which causes runaway: high Vd → vq_lim collapses → motor
             * stuck in rail (observed 2026-04-25).  Only engage when
             * the speed PI is genuinely demanding forward torque. */
            const float FW_IQ_GATE = 1.0f;

            float v_last = sqrtf(m->vd * m->vd + m->vq * m->vq);
            float mod_now = (vmax > 0.001f) ? v_last / vmax : 0.0f;

            /* Hysteresis: engage above FW_TRIGGER, hold integrated value
             * down to FW_RELEASE, only decay below RELEASE.  Eliminates
             * the limit cycle where mod hovers near 0.91 and FW pumps
             * up/decays each tick.  When in CL operating regime with
             * voltage saturated, FW stays engaged steadily. */
            if (mod_now > FW_TRIGGER && iq_ref > FW_IQ_GATE) {
                m->id_ref_fw -= FW_KP_INT * (mod_now - FW_TRIGGER) * dt;
                if (m->id_ref_fw < ID_FW_MAX_NEG) m->id_ref_fw = ID_FW_MAX_NEG;
            } else if (mod_now < FW_RELEASE) {
                /* Below release threshold → decay.  Between RELEASE and
                 * TRIGGER → hold (no integration, no decay). */
                m->id_ref_fw *= FW_DECAY;
                if (m->id_ref_fw > -0.005f) m->id_ref_fw = 0.0f;
            }
        }
        float id_ref = m->id_ref_fw;
#endif /* !FEATURE_FWC */

#if FEATURE_FOC_FF_DECOUPLE
        /* ── BEMF-only feedforward ─────────────────────────────────
         *
         *   Vq_ff = ω · λ_est   (cancels BEMF — depends only on ω)
         *   Vd_ff = 0
         *
         * Vd cross-coupling FF was attempted three ways (Phase B with
         * iq_filt, Phase B' with iq_filt + integrator scaling, Vd FF
         * with iq_ref). The first two destabilized at the FW boundary;
         * iq_ref-based FF (bench-tested 2026-05-22, foc_run_160847.csv)
         * showed no measurable Id improvement vs no Vd FF (span 25.8A
         * vs 26.8A — within run-to-run noise).  The residual Id
         * swings at top come from observer angle noise + telemetry
         * aliasing of PWM-rate ripple, not from cross-coupling. */
        float ff_omega  = m->smc.OmegaFltred;
        float ff_lambda = (gspParams.focKeUvSRad > 0)
                        ? gspParams.focKeUvSRad * 1.0e-6f
                        : AN_MOTOR_LAMBDA;
        float vd_ff = 0.0f;
        float vq_ff = ff_omega * ff_lambda;
#else
        float vd_ff = 0.0f;
        float vq_ff = 0.0f;
#endif

        /* ── d-axis ─────────────────────────────────────────────────
         * Vd_ff = 0, so PI keeps the full ±vmax range. */
        float vmax_sq = vmax * vmax;
        m->pi_d.outMax =  vmax;
        m->pi_d.outMin = -vmax;
        float vd_pi = an_pi_run(&m->pi_d, id_ref, m->id_meas, dt);
        m->vd = vd_ff + vd_pi;

        /* ── q-axis (D-priority circle clamp) ──────────────────────
         * Given committed Vd, remaining Vq envelope is the circle
         * chord:  vq_avail = sqrt(vmax² - Vd²).  Total Vq ∈ ±vq_avail.
         *   Vq_pi ∈ [ -vq_avail - Vq_ff, +vq_avail - Vq_ff ]
         * Saturate Vq_ff to ±vq_avail first; FW will then pull
         * id_ref negative to reduce BEMF demand if it's clipping. */
        float vd_sq = m->vd * m->vd;
        float vq_avail_sq = vmax_sq - vd_sq;
        float vq_avail = (vq_avail_sq > 0.0f) ? sqrtf(vq_avail_sq) : 0.0f;

        if      (vq_ff >  vq_avail) vq_ff =  vq_avail;
        else if (vq_ff < -vq_avail) vq_ff = -vq_avail;

        m->pi_q.outMax =  vq_avail - vq_ff;
        m->pi_q.outMin = -vq_avail - vq_ff;
        float vq_pi = an_pi_run(&m->pi_q, iq_ref, m->iq_meas, dt);
        m->vq = vq_ff + vq_pi;

        /* Final circle clamp safety net. */
        {
            float vs_sq = m->vd * m->vd + m->vq * m->vq;
            if (vs_sq > vmax_sq) {
                float scale = sqrtf(vmax_sq / vs_sq);
                m->vd *= scale;
                m->vq *= scale;
            }
        }
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
        /* LOCK: align rotor.  startupRamp held at zero.
         *
         * IMPORTANT: AN1078 applies Iq (not Id) during alignment with
         * synth=0.  Iq at synth=0 means current is on stator β-axis →
         * rotor d-axis pulls to β = stator angle π/2.  After alignment,
         * rotor d sits at stator β, NOT stator α.
         *
         * To make OL ramp start with rotor d aligned to synth d (so the
         * Iq-q current generates maximum forward torque), the synth
         * angle must be set to π/2 at the LOCK→RAMP transition.  Then
         * synth d at stator angle π/2 = rotor d position → aligned.
         *
         * Without this offset, OL starts at synth=0 with rotor d at
         * synth q (90° offset).  Current is parallel to rotor d → ZERO
         * initial torque.  With prop drag, prop friction nudges rotor
         * out of this dead zone and motor catches up.  Without prop, low
         * inertia rotor stays stuck and OL ramp doesn't follow.  Fixed
         * 2026-04-26 after observing A2212 stuck-in-OL with no prop. */
        if (m->startupLock < AN_LOCK_TIME) {
            m->startupLock++;
            if (m->startupLock == AN_LOCK_TIME) {
                /* End of alignment — offset synth angle to match where
                 * rotor actually settled (stator β-axis). */
                m->thetaOpenLoop = AN_PI * 0.5f;
            }
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
            /* Handoff gate: SMC's BEMF magnitude must be ≥ threshold pct of
             * theoretical (λ·ω) and sustained for AN_HANDOFF_DWELL_TICKS.
             *
             * Threshold relaxed 80% → 50% on 2026-04-26 to handle the
             * prop-load case at higher idle (1500 RPM mech).  At low motor
             * freq the LPF still has significant attenuation, so observed
             * BEMF magnitude can dip below 0.8·λω even when motor tracks
             * the synth angle correctly.  50% is enough margin to confirm
             * observer lock without missing handoff under prop drag.
             *
             * Dwell counter still enforces the minimum sustained time —
             * a brief 50% dip during OL ramp won't trigger handoff. */
            float bemf_mag = sqrtf(m->smc.EalphaFinal * m->smc.EalphaFinal
                                 + m->smc.EbetaFinal  * m->smc.EbetaFinal);
            /* λ from gspParams (per-profile, GUI-editable) with fallback
             * to compile-time default if EEPROM not yet loaded. */
            float lambda = (gspParams.focKeUvSRad > 0)
                         ? gspParams.focKeUvSRad * 1.0e-6f
                         : AN_MOTOR_LAMBDA;
            float bemf_expected = lambda * m->startupRamp;
            /* Relaxed 50%→30% on 2026-04-26 for A2212.  Higher-Rs motors
             * have less OL torque margin → rotor slips synth angle under
             * prop drag → real BEMF lower than λ·ω_command.  Plus LPF
             * cascade attenuates ~20% at low motor freq.  30% still
             * rejects "no signal at all" while accepting partial slip. */
            /* AK port (2026-05-20): with OL end-speed bumped to 4000 RPM
             * mech, BEMF at handoff is ~1.7V, plenty of headroom. Gate
             * back to 25% so handoff only fires when SMO is actually
             * tracking, not just briefly above noise floor. */
            float bemf_min = bemf_expected * 0.25f;

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

    /* Single-pole LPF on id/iq for FF cross-coupling.  PI keeps using
     * raw id_meas/iq_meas — only the FF terms read id_filt/iq_filt.
     * Filters PWM ripple + ADC noise that otherwise gets amplified by
     * ω·L at high speed.  See AN_FF_I_LPF_TAU_S. */
    m->id_filt += AN_FF_I_LPF_ALPHA * (m->id_meas - m->id_filt);
    m->iq_filt += AN_FF_I_LPF_ALPHA * (m->iq_meas - m->iq_filt);

    /* ── 4. Feed observer ────────────────────────────────── */
    m->smc.Ialpha = m->i_alpha;
    m->smc.Ibeta  = m->i_beta;
    m->smc.Valpha = m->v_alpha;   /* from previous tick's inverse Park */
    m->smc.Vbeta  = m->v_beta;
    OBS_UPDATE(&m->smc);

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

#endif /* FEATURE_FOC_AN1078 */
