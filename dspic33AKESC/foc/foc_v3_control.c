/**
 * @file foc_v3_control.c
 * @brief FOC v3 control loop — correct classical SMO + V/f startup.
 *
 * State machine:
 *   IDLE → ARMED → ALIGN → OL_RAMP → CLOSED_LOOP → FAULT
 *                     ↑                    ↓ (low speed)
 *                     ↑                VF_ASSIST
 *                     ↑                    ↓ (speed rises + SMO ok)
 *                     ↑                CLOSED_LOOP
 *                     └── (throttle=0 >1s) ─┘
 *
 * Fast loop (24 kHz): Clarke → SMO → Park → PI_d/PI_q
 *   → circular clamp → InvPark → SVPWM → duty output
 *
 * Slow loop (1 kHz): Speed PI → Iq_ref, arming, fault detect
 *
 * The SMO runs free during OL ramp (receives real I, V but its
 * angle is not used for commutation). At handoff speed, if the
 * observer health metrics pass, commutation switches to SMO angle.
 *
 * Component: FOC V3
 */

#include <xc.h>
#include "foc_v3_control.h"
#include "foc_v3_smo.h"
#include "foc_v2_math.h"
#include "foc_v2_pi.h"
#include "../garuda_foc_params.h"
#include "../garuda_types.h"

/* ── V/f Open-Loop Startup ───────────────────────────────────
 * Voltage-mode startup with proper V/f curve.
 * Vq = VF_BOOST + Ke × ω — voltage tracks BEMF plus a torque
 * margin.  Boost provides lock-in torque at standstill, Ke×ω
 * keeps V above BEMF as speed rises. */
#define VF_BOOST_V           0.5f
#define VF_OL_RAMP_RATE      200.0f

/* ── Constants ───────────────────────────────────────────────── */
#define DT_FAST          FOC_TS_FAST_S
#define DT_SLOW          FOC_TS_SLOW_S
#define SLOW_DIV         FOC_SLOW_DIV

/* ADC calibration */
#define CAL_SAMPLES      1024U
#define CAL_SHIFT         10U

/* Overcurrent debounce (2ms at 24kHz) */
#define OC_DEBOUNCE_TICKS 48U

/* Board fault pin: RB11 = U25A(OV)+U25B(OC), active-low */
#define BOARD_FAULT_PIN      PORTBbits.RB11
#define BOARD_FAULT_ACTIVE   0
#define BOARD_FAULT_DEBOUNCE 48U

/* Handoff dwell at handoff speed before CL entry (cold startup) */
#define HANDOFF_DWELL_TICKS 3000U

/* Re-entry dwell from VF_ASSIST → CL (warm SMO/PLL, fast re-lock).
 * 480 ticks at 24kHz = 20ms — enough to confirm PLL re-lock. */
#define REENTRY_DWELL_TICKS 480U

/* CL holdoff: after CL entry, hold initial Iq while PLL/SMO stabilize */
#define CL_HOLDOFF_SLOW_TICKS 20U

/* AN1078-style post-handoff theta-error bleed rate.
 * AN1078 pmsm.c bleeds 0.05° every TRANSITION_STEPS ticks
 * (= IRP_PERCALC/4 = 5 ticks at 20kHz fast loop) → 175°/s ≈ 3 rad/s.
 * A 90° capture clears in ~0.5 s — fast but smooth. */
#define CL_THETA_BLEED_RAD_S  3.0f

/* Re-arm: throttle=0 for 1s in slow loop ticks */
#define REARM_TIMEOUT_TICKS 1000U

/* Observer health exit: bad ticks before CL → stop */
#define OBSERVER_BAD_LIMIT  500U    /* 500ms at 1kHz */

/* ── Helpers ─────────────────────────────────────────────────── */

static inline float raw_to_amps(uint16_t raw, float offset)
{
    return -((float)raw - offset) * CURRENT_SCALE_A_PER_COUNT;
}

static inline float raw_to_vbus(uint16_t raw)
{
    return (float)raw * VBUS_SCALE_V_PER_COUNT;
}

static void reset_startup(V3_State_t *st)
{
    st->omega_ol    = 0.0f;
    st->theta_ol    = 0.0f;
    st->align_ctr   = 0;
    st->ramp_ctr    = 0;
    st->handoff_ctr  = 0;
    st->angle_ok_ctr    = 0;
    st->cl_active       = false;
    st->blend_ctr       = 0;
    st->iq_ref      = 0.0f;
    st->id_ref      = 0.0f;
    st->vd          = 0.0f;
    st->vq          = 0.0f;
    st->da_prev     = 0.5f;
    st->db_prev     = 0.5f;
    st->dc_prev     = 0.5f;
    st->v_alpha_applied = 0.0f;
    st->v_beta_applied  = 0.0f;
    st->theta_delay_comp = 0.0f;
    st->theta_error     = 0.0f;
    st->oc_debounce_ctr = 0;
    st->observer_bad_ctr = 0;
    st->phantom_ctr = 0;
    st->desync_ctr = 0;
    st->board_fault_ctr = 0;
    st->rearm_ctr = 0;
    st->sub_state   = 0;

    foc_pi_reset(&st->pid_d);
    foc_pi_reset(&st->pid_q);
    foc_pi_reset(&st->pid_spd);
    v3_smo_reset(&st->smo);
    v3_smo_pll_reset(&st->pll);
}

/* ── Public API ──────────────────────────────────────────────── */

void foc_v3_init(V3_State_t *st, const FOC_MotorParams_t *params)
{
    /* Zero entire state */
    V3_State_t zero = {0};
    *st = zero;

    st->mode = V3_IDLE;

    /* Copy motor params */
    st->Rs        = params->Rs;
    st->Ls        = params->Ls;
    st->lambda_pm = params->lambda_pm;
    st->Ke        = params->Ke;

    /* Dead-time compensation */
    st->dt_comp = params->vbus_nom_v * FOC_DT_COMP_FRAC;

    /* Copy runtime startup params */
    st->align_iq       = params->align_iq_a;
    st->ramp_iq        = params->ramp_iq_a;
    st->align_ticks    = params->align_ticks;
    st->iq_ramp_ticks  = params->iq_ramp_ticks;
    st->ramp_rate      = params->ramp_rate_rps2;
    st->handoff_rad_s  = params->handoff_rad_s;
    st->fault_oc_a     = params->fault_oc_a;
    st->max_elec_rad_s = params->max_elec_rad_s;
    st->use_vf_startup = STARTUP_USE_VF;

    /* Handoff health thresholds (runtime, from params or defaults) */
    st->handoff_conf_min     = SMO_CONF_MIN_HANDOFF;
    st->handoff_residual_max = SMO_RESIDUAL_MAX_HANDOFF;
    st->handoff_bemf_min     = SMO_BEMF_MIN_HANDOFF;

    /* Build SMO configuration struct from motor params.
     * AN1078 float port: fixed Kslide, speed-proportional Kslf floored
     * at end-speed, constant angle offset.  See garuda_foc_params.h
     * SMO_* block. */
    {
        float end_speed = (params->handoff_rad_s > SMO_END_SPEED_ELEC_RS)
                        ? params->handoff_rad_s : SMO_END_SPEED_ELEC_RS;

        SMO_Config_t smo_cfg = {
            .Rs         = params->Rs,
            .Ls         = params->Ls,
            .lambda_pm  = params->lambda_pm,
            .vbus_nom   = params->vbus_nom_v,
            .dt         = DT_FAST,

            /* Sliding controller */
            .Kslide       = params->vbus_nom_v * 0.85f,   /* AN1078 Q15(0.85) */
            .MaxSMCError  = SMO_MAX_SMC_ERROR,

            /* BEMF LPF */
            .theta_filter_cnst = SMO_THETA_FILTER_CNST,
            .kslf_min          = end_speed * SMO_THETA_FILTER_CNST,

            /* Angle offset */
            .theta_offset = SMO_THETA_OFFSET,

            /* Health thresholds */
            .conf_min     = SMO_CONF_MIN,
            .residual_max = SMO_RESIDUAL_MAX,
        };
        v3_smo_init(&st->smo, &smo_cfg);
    }

    /* Initialize PLL (speed-adaptive BW from runtime tunables) */
    v3_smo_pll_init(&st->pll,
                    SMO_PLL_BW_MIN_HZ,
                    SMO_PLL_BW_MAX_HZ,
                    params->max_elec_rad_s * SMO_PLL_BW_REF_FRAC,
                    params->max_elec_rad_s);

    /* Current PI controllers */
    float vmax = params->vbus_nom_v * 0.95f * FOC_INV_SQRT3;
    foc_pi_init(&st->pid_d, params->kp_dq, params->ki_dq, -vmax, vmax);
    foc_pi_init(&st->pid_q, params->kp_dq, params->ki_dq, -vmax, vmax);

    /* Speed PI controller → Iq reference */
    foc_pi_init(&st->pid_spd, KP_SPD, KI_SPD, -1.0f, CL_IQ_MAX_A);
}

void foc_v3_start(V3_State_t *st)
{
    if (st->mode == V3_ARMED) {
        reset_startup(st);
        foc_pi_preload(&st->pid_d, 0.0f);
        foc_pi_preload(&st->pid_q, 0.0f);
        foc_pi_preload(&st->pid_spd, 0.0f);

        /* Hardcoded I/f startup overrides (2026-04-23).
         * EEPROM values were inconsistent across sessions and caused
         * startup to fail. These are conservative values for PRODRONE 2810
         * (1350KV, 7PP, ~22mΩ, ~10µH) at 24V no-prop.
         *
         * Fast-loop runs at 24 kHz → 1 ms = 24 ticks.
         *
         * Strategy: VERY slow concurrent Iq + omega ramp. Rotor stays
         * pinned to d-axis by align_iq while Iq grows to pull it forward.
         * omega ramps from zero so rotor never faces a step-speed kick.
         * 6-second total startup is fine for bench. */
        st->use_vf_startup  = 0;              /* force I/f mode */
        /* PORTED FROM v2 PROFILE 2 (proven on this 2810 + AK board).
         * v2 architecture: ALIGN uses Id (not Iq), then IF_RAMP smoothly
         * crossfades Id→Iq while ramping omega.  Handoff at 1000 rad/s
         * because BEMF = λ×ω = 0.583mV/rad·s × 1000 = 0.58V — that's
         * the floor where the observer can actually see signal above
         * the dt_comp noise.  Below ~500 rad/s BEMF is buried.
         *
         * Iq=3A → 18 mN·m, plenty for no-prop on this rotor.  Iq=8A
         * we tried earlier was overkill and made no difference because
         * the rotor was slipping the OL angle anyway. */
        st->align_iq        = 2.0f;           /* v2 STARTUP_ALIGN_IQ_A */
        st->ramp_iq         = 3.0f;           /* v2 STARTUP_RAMP_IQ_A */
        st->align_ticks     = 4800;           /* 200 ms — v2 STARTUP_ALIGN_TICKS */
        st->iq_ramp_ticks   = 4800;           /* 200 ms d→q crossfade */
        st->ramp_rate       = 800.0f;         /* rad/s² — v2 STARTUP_RAMP_RATE_RPS2 */
        st->handoff_rad_s   = 1000.0f;        /* rad/s — v2 STARTUP_HANDOFF_RAD_S */

        st->mode = V3_ALIGN;
    }
}

void foc_v3_stop(V3_State_t *st)
{
    st->mode = V3_ARMED;
    reset_startup(st);
}

void foc_v3_fault(V3_State_t *st, uint16_t fault_code)
{
    st->mode = V3_FAULT;
    st->fault_code = fault_code;
}

/* ── Fast-loop tick (24 kHz) ─────────────────────────────────── */

void foc_v3_fast_tick(V3_State_t *st,
                      uint16_t ia_raw, uint16_t ib_raw,
                      uint16_t vbus_raw, uint16_t throttle,
                      float *da_out, float *db_out, float *dc_out)
{
    /* Default: 50% duty (no net voltage) */
    *da_out = 0.5f;
    *db_out = 0.5f;
    *dc_out = 0.5f;

    /* Vbus conversion + dynamic dt_comp */
    st->vbus = raw_to_vbus(vbus_raw);
    st->dt_comp = st->vbus * FOC_DT_COMP_FRAC;

    /* ── IDLE / ARMED: ADC offset calibration ───────────────── */
    if (st->mode == V3_IDLE || st->mode == V3_ARMED) {
        if (!st->cal_done) {
            st->cal_accum_a += ia_raw;
            st->cal_accum_b += ib_raw;
            st->cal_count++;
            if (st->cal_count >= CAL_SAMPLES) {
                st->ia_offset = (float)(st->cal_accum_a >> CAL_SHIFT);
                st->ib_offset = (float)(st->cal_accum_b >> CAL_SHIFT);
                st->cal_done  = true;
            }
        }
        st->sub_state = (st->mode == V3_IDLE) ? 0 : 1;
        goto slow_loop;
    }

    /* ── FAULT: outputs disabled ─────────────────────────────── */
    if (st->mode == V3_FAULT) {
        st->sub_state = 0;
        *da_out = 0.5f;
        *db_out = 0.5f;
        *dc_out = 0.5f;
        goto slow_loop;
    }

    /* ── Motor running: ALIGN / OL_RAMP / VF_ASSIST / CLOSED_LOOP ── */

    /* Dynamic PI voltage clamp — tracks Vbus */
    {
        float vclamp = st->vbus * 0.95f * FOC_INV_SQRT3;
        st->pid_d.out_max =  vclamp;
        st->pid_d.out_min = -vclamp;
        st->pid_q.out_max =  vclamp;
        st->pid_q.out_min = -vclamp;
    }

    /* 1. Convert ADC to physical units */
    float ia = raw_to_amps(ia_raw, st->ia_offset);
    float ib = raw_to_amps(ib_raw, st->ib_offset);

    /* 2. Clarke transform */
    float i_alpha, i_beta;
    foc_clarke(ia, ib, &i_alpha, &i_beta);

    /* 3. Reconstruct applied voltage from previous PWM duty cycles.
     *    SMO_DT_COMP_MODE: 0=none, 1=per-phase ABC, 2=αβ approx */
    float v_alpha, v_beta;
    {
        float da = st->da_prev, db = st->db_prev, dc = st->dc_prev;

#if SMO_DT_COMP_MODE == 1
        /* Per-phase dead-time correction (physically correct model) */
        float va = da * st->vbus;
        float vb = db * st->vbus;
        float vc = dc * st->vbus;
        float ic = -(ia + ib);
        va -= st->dt_comp * foc_soft_sign(ia, 2.0f);
        vb -= st->dt_comp * foc_soft_sign(ib, 2.0f);
        vc -= st->dt_comp * foc_soft_sign(ic, 2.0f);
        v_alpha = (2.0f * va - vb - vc) * (1.0f / 3.0f);
        v_beta  = (vb - vc) * FOC_INV_SQRT3;
#elif SMO_DT_COMP_MODE == 2
        /* αβ-frame dead-time approximation */
        v_alpha = (2.0f * da - db - dc) * (1.0f / 3.0f) * st->vbus;
        v_beta  = (db - dc) * FOC_INV_SQRT3 * st->vbus;
        v_alpha -= st->dt_comp * foc_soft_sign(i_alpha, 2.0f);
        v_beta  -= st->dt_comp * foc_soft_sign(i_beta,  2.0f);
#else
        /* No dead-time compensation — raw duty reconstruction */
        v_alpha = (2.0f * da - db - dc) * (1.0f / 3.0f) * st->vbus;
        v_beta  = (db - dc) * FOC_INV_SQRT3 * st->vbus;
#endif
    }
    st->v_alpha_applied = v_alpha;
    st->v_beta_applied  = v_beta;

    /* 4. Run SMO observer (always, even during OL — let it converge).
     * Speed hint: forced omega during OL/VF_ASSIST, LP-filtered during CL. */
    float smo_omega_hint;
    if (st->mode == V3_ALIGN || st->mode == V3_OL_RAMP || st->mode == V3_VF_ASSIST)
        smo_omega_hint = st->omega_ol;
    else
        smo_omega_hint = st->omega;

    v3_smo_update(&st->smo,
               v_alpha, v_beta,
               i_alpha, i_beta,
               smo_omega_hint);

    /* 5. Run PLL on SMO angle */
    v3_smo_pll_update(&st->pll, st->smo.theta_est, DT_FAST);

    /* 6. Select commutation angle based on mode */
    float theta_drive;
    float iq_ref, id_ref;

    switch (st->mode) {
    case V3_ALIGN: {
        theta_drive = 0.0f;
        st->align_ctr++;

        if (st->use_vf_startup) {
            id_ref = 0.0f;
            iq_ref = 0.0f;
        } else {
            id_ref = st->align_iq;
            iq_ref = 0.0f;
        }

        if (st->align_ctr >= st->align_ticks) {
            st->mode = V3_OL_RAMP;
            st->ramp_ctr = 0;
            st->omega_ol = 0.0f;
            st->theta_ol = 0.0f;
            v3_smo_reset(&st->smo);
            v3_smo_pll_reset(&st->pll);
        }

        st->sub_state = 2;
        break;
    }

    case V3_OL_RAMP: {
        st->ramp_ctr++;

        if (st->use_vf_startup) {
            iq_ref = 0.0f;
            id_ref = 0.0f;
            /* V/f: ramp speed immediately (voltage already rotates field). */
            st->omega_ol += st->ramp_rate * DT_FAST;
            if (st->omega_ol > st->handoff_rad_s)
                st->omega_ol = st->handoff_rad_s;
        } else {
            /* v2-style smooth d→q crossfade.  ALIGN locked rotor at
             * d-axis with Id=align_iq.  Linearly fade Id down and Iq up
             * over iq_ramp_ticks so the current vector rotates from 0°
             * to 90° smoothly — rotor follows the rotating current
             * vector.  After crossfade, Id=0, Iq=ramp_iq. */
            if (st->ramp_ctr < st->iq_ramp_ticks) {
                float frac = (float)st->ramp_ctr / (float)st->iq_ramp_ticks;
                iq_ref = st->ramp_iq  * frac;
                id_ref = st->align_iq * (1.0f - frac);
            } else {
                iq_ref = st->ramp_iq;
                id_ref = 0.0f;
            }

            /* omega_ol ramps from tick 0 — rotor pulled along by Iq */
            st->omega_ol += st->ramp_rate * DT_FAST;
            if (st->omega_ol > st->handoff_rad_s)
                st->omega_ol = st->handoff_rad_s;
        }

        /* Advance forced angle */
        st->theta_ol += st->omega_ol * DT_FAST;
        st->theta_ol = foc_angle_wrap(st->theta_ol);
        theta_drive = st->theta_ol;

        /* Handoff condition: PLL tracking + BEMF quality.
         *
         * NOTE: smo.residual is NOT a convergence metric for SMO.
         * In sliding mode, current estimation error = G×K = (dt/Ls)×Vbus
         * per tick — the switching chattering IS the mechanism.
         * A2212: G×K = 16.7A → LP-filtered residual ≈ 12A always.
         * Residual is kept in telemetry but excluded from gating.
         *
         * Meaningful convergence signals:
         *   - PLL speed within ±50% of OL speed (PLL alive)
         *   - Confidence > threshold (BEMF magnitude reasonable)
         *   - BEMF magnitude above noise floor
         *   - PLL innovation settling (phase error decreasing)
         * All sustained for HANDOFF_DWELL_TICKS. */
        if (st->omega_ol >= st->handoff_rad_s) {
            float pll_speed = st->pll.omega_est;
            float ol_speed = st->omega_ol;
            bool pll_tracking = (pll_speed > ol_speed * 0.5f) &&
                                (pll_speed < ol_speed * 1.5f);

            bool health_ok = pll_tracking
                          && (st->smo.confidence > st->handoff_conf_min)
                          && (st->smo.bemf_mag_filt > st->handoff_bemf_min)
                          && (st->pll.innovation_lpf < 1.0f);

            if (health_ok) {
                st->handoff_ctr++;
            } else {
                st->handoff_ctr = 0;
            }

            if (st->handoff_ctr >= HANDOFF_DWELL_TICKS) {
                /* AN1078 handoff (faithful port of pmsm.c:734).
                 *
                 * Capture the angle gap, then leave PI integrators
                 * COMPLETELY ALONE.  The current PI was already running
                 * during OL with theta_drive = theta_ol, so its vd/vq
                 * outputs are valid for that frame.  In CL we'll
                 * commutate at smc.Theta + theta_error; on the very
                 * first CL tick that equals theta_ol exactly — no
                 * discontinuity in the commutation angle — so the PI
                 * continues seamlessly.  Each subsequent tick bleeds
                 * theta_error toward zero, smoothly migrating the
                 * commutation reference from OL to observer.
                 *
                 * Do NOT preload pid_d/pid_q (would inject a step).
                 * Do NOT seed st->theta separately (pll wasn't driving). */
                float terr = st->theta_ol - st->smo.theta_est;
                if (terr >  FOC_PI_F) terr -= FOC_TWO_PI;
                if (terr < -FOC_PI_F) terr += FOC_TWO_PI;
                st->theta_error = terr;

                /* Speed PI: preload with measured Iq so the speed loop
                 * starts from the actual operating point, no step. */
                float iq_actual = st->iq_meas > 0.0f ? st->iq_meas : 0.0f;
                foc_pi_preload(&st->pid_spd, iq_actual);
                st->iq_ref = iq_actual;

                st->omega = st->omega_ol;

                st->mode = V3_CLOSED_LOOP;
                st->cl_active = true;
                st->blend_ctr = 0;
            }
        }

        st->sub_state = 3;
        break;
    }

    case V3_CLOSED_LOOP: {
        /* AN1078 closed-loop commutation (faithful port of pmsm.c:616).
         *
         *   thetaElectrical = smc.Theta + Theta_error
         *
         * On the first CL tick Theta_error = (theta_ol − smc.Theta)
         * captured at handoff, so commutation == theta_ol exactly
         * (continuous with OL).  Each tick we bleed Theta_error toward
         * zero; commutation smoothly migrates to smc.Theta. */
        {
            float step = CL_THETA_BLEED_RAD_S * DT_FAST;
            if (st->theta_error >  step) st->theta_error -= step;
            else if (st->theta_error < -step) st->theta_error += step;
            else st->theta_error = 0.0f;
        }

        theta_drive = st->smo.theta_est + st->theta_error;
        theta_drive = foc_angle_wrap(theta_drive);
        st->theta = theta_drive;
        st->theta_delay_comp = 0.0f;

        iq_ref = st->iq_ref;
        id_ref = 0.0f;

        /* Speed for telemetry + speed PI feedback: SMO's own delta-θ
         * estimate (smo.omega_est).  AN1078 uses this directly. */
        {
            float omega_smo = st->smo.omega_est;
            if (omega_smo < 0.0f) omega_smo = 0.0f;
            st->omega += 0.05f * (omega_smo - st->omega);
        }

        st->sub_state = 4;
        break;
    }

    case V3_VF_ASSIST: {
        /* V/f low-speed assist: forced-angle drive while SMO keeps running.
         * Entered from CL when speed drops below CL_EXIT_RAD_S.
         * Returns to CL when speed rises above CL_ENTER_RAD_S with good SMO.
         *
         * omega_ol tracks current speed (seeded from CL omega at entry).
         * Throttle controls speed target via same V/f curve as OL_RAMP. */

        id_ref = 0.0f;
        iq_ref = 0.0f;  /* V/f mode: voltage, not current */

        /* Advance forced angle at current speed */
        st->theta_ol += st->omega_ol * DT_FAST;
        st->theta_ol = foc_angle_wrap(st->theta_ol);
        theta_drive = st->theta_ol;

        /* Re-entry to CL: check same health gates as OL_RAMP handoff */
        if (st->omega_ol >= CL_ENTER_RAD_S) {
            float pll_speed = st->pll.omega_est;
            float ol_speed = st->omega_ol;
            bool pll_tracking = (pll_speed > ol_speed * 0.5f) &&
                                (pll_speed < ol_speed * 1.5f);

            bool health_ok = pll_tracking
                          && (st->smo.confidence > st->handoff_conf_min)
                          && (st->smo.bemf_mag_filt > st->handoff_bemf_min)
                          && (st->pll.innovation_lpf < 1.0f);

            if (health_ok) {
                st->handoff_ctr++;
            } else {
                st->handoff_ctr = 0;
            }

            if (st->handoff_ctr >= REENTRY_DWELL_TICKS) {
                /* AN1078-style: capture theta_error, bleed during CL.
                 * VF_ASSIST drives forced angle; it doesn't run the
                 * d/q PI integrators (vd=0, vq=V/f), so we DO need to
                 * preload PI for current control. */
                float terr = st->theta_ol - st->smo.theta_est;
                if (terr >  FOC_PI_F) terr -= FOC_TWO_PI;
                if (terr < -FOC_PI_F) terr += FOC_TWO_PI;
                st->theta_error = terr;

                float vq_ss = VF_BOOST_V + st->Ke * st->omega_ol
                            + st->Rs * 0.3f;
                foc_pi_preload(&st->pid_q, vq_ss);
                foc_pi_preload(&st->pid_d, 0.0f);

                float iq_actual = st->iq_meas > 0.0f ? st->iq_meas : 0.0f;
                foc_pi_preload(&st->pid_spd, iq_actual);
                st->iq_ref = iq_actual;

                st->omega = st->omega_ol;

                st->mode = V3_CLOSED_LOOP;
                st->cl_active = true;
                st->blend_ctr = 0;
            }
        } else {
            st->handoff_ctr = 0;
        }

        st->sub_state = 5;  /* telemetry: VF_ASSIST */
        break;
    }

    default:
        return;
    }

    /* ── Common FOC pipeline (all running modes) ─────────────── */

    /* 7. Park transform at drive angle */
    float sin_t = sinf(theta_drive);
    float cos_t = cosf(theta_drive);
    float id_meas, iq_meas;
    foc_park(i_alpha, i_beta, sin_t, cos_t, &id_meas, &iq_meas);

    /* Store for telemetry */
    st->id_meas = id_meas;
    st->iq_meas = iq_meas;
    st->theta   = theta_drive;
    st->theta_obs = st->smo.theta_est;
    st->omega_pll = st->pll.omega_est;

    /* 8. Voltage generation */
    float vd_tot, vq_tot;
    bool use_pi = (st->mode == V3_CLOSED_LOOP)
               || ((st->mode == V3_ALIGN || st->mode == V3_OL_RAMP)
                   && !st->use_vf_startup);
    /* VF_ASSIST always uses V/f (voltage mode) */

    if (use_pi) {
        if (st->mode == V3_ALIGN) {
            foc_pi_preload(&st->pid_q, 0.0f);
        }

        float vd_pi = foc_pi_run(&st->pid_d, id_ref - id_meas, DT_FAST);
        float vq_pi = foc_pi_run(&st->pid_q, iq_ref - iq_meas, DT_FAST);

        vd_tot = vd_pi;
        vq_tot = vq_pi;

        /* Circular voltage clamp */
        float vmax = st->vbus * 0.95f * FOC_INV_SQRT3;
        if (st->mode == V3_ALIGN) {
            /* D-axis priority */
            vd_tot = foc_clampf(vd_tot, -vmax, vmax);
            float sq = vmax * vmax - vd_tot * vd_tot;
            if (sq < 0.0f) sq = 0.0f;
            float vq_lim = sqrtf(sq);
            vq_tot = foc_clampf(vq_tot, -vq_lim, vq_lim);
        } else {
            /* Q-axis priority */
            vq_tot = foc_clampf(vq_tot, -vmax, vmax);
            float sq = vmax * vmax - vq_tot * vq_tot;
            if (sq < 0.0f) sq = 0.0f;
            float vd_lim = sqrtf(sq);
            vd_tot = foc_clampf(vd_tot, -vd_lim, vd_lim);
        }
    } else {
        /* V/f: direct voltage at forced angle */
        vd_tot = 0.0f;
        if (st->mode == V3_ALIGN) {
            float vfrac = (float)st->align_ctr / (float)st->align_ticks;
            if (vfrac > 1.0f) vfrac = 1.0f;
            vq_tot = VF_BOOST_V * vfrac;
        } else {
            /* OL_RAMP and VF_ASSIST both use same V/f curve */
            vq_tot = VF_BOOST_V + st->Ke * st->omega_ol;
            float vq_max = st->vbus * 0.95f * FOC_INV_SQRT3;
            if (vq_tot > vq_max) vq_tot = vq_max;
        }
    }

    /* Store for telemetry and next-tick voltage reconstruction */
    st->vd = vd_tot;
    st->vq = vq_tot;

    /* 10. Inverse Park → αβ */
    float v_alpha_out, v_beta_out;
    foc_inv_park(vd_tot, vq_tot, sin_t, cos_t, &v_alpha_out, &v_beta_out);

    /* 11. SVPWM → duty cycles */
    foc_svpwm(v_alpha_out, v_beta_out, st->vbus, da_out, db_out, dc_out);

    /* Store actual duties for next-tick voltage reconstruction */
    st->da_prev = *da_out;
    st->db_prev = *db_out;
    st->dc_prev = *dc_out;

    /* ── Fast-loop protections ───────────────────────────────── */

    /* 12. Software overcurrent check */
    {
        float i_sq = id_meas * id_meas + iq_meas * iq_meas;
        float oc_sq = st->fault_oc_a * st->fault_oc_a;
        if (i_sq > oc_sq) {
            st->oc_debounce_ctr++;
            if (st->oc_debounce_ctr >= OC_DEBOUNCE_TICKS) {
                st->mode = V3_FAULT;
                st->fault_code = FAULT_FOC_INTERNAL;
                *da_out = 0.5f;
                *db_out = 0.5f;
                *dc_out = 0.5f;
            }
        } else {
            st->oc_debounce_ctr = 0;
        }
    }

    /* 13. Board fault pin check (U25B OC, active-low) */
    {
        if (BOARD_FAULT_PIN == BOARD_FAULT_ACTIVE) {
            st->board_fault_ctr++;
            if (st->board_fault_ctr >= BOARD_FAULT_DEBOUNCE) {
                st->mode = V3_FAULT;
                st->fault_code = FAULT_BOARD_PCI;
                *da_out = 0.5f;
                *db_out = 0.5f;
                *dc_out = 0.5f;
            }
        } else {
            st->board_fault_ctr = 0;
        }
    }

    /* ── Slow loop (every SLOW_DIV ticks → 1 kHz) ───────────── */
slow_loop:
    st->slow_div_ctr++;
    if (st->slow_div_ctr < SLOW_DIV) return;
    st->slow_div_ctr = 0;

    /* ARMED → auto-start after arm dwell */
    if (st->mode == V3_ARMED) {
        if (throttle <= THROTTLE_DEADBAND) {
            st->arm_ctr++;
            if (st->arm_ctr >= FOC_ARM_TIME_MS) {
                foc_v3_start(st);
            }
        } else {
            st->arm_ctr = 0;
        }
    }

    /* Closed-loop slow loop */
    if (st->mode == V3_CLOSED_LOOP) {
        st->blend_ctr++;

        if (st->blend_ctr <= CL_HOLDOFF_SLOW_TICKS) {
            /* Holdoff: keep initial Iq */
        } else {
            /* Speed reference from throttle */
            float omega_max_v = 0.70f * st->vbus / (FOC_SQRT3 * st->Ke + 1e-9f);
            if (omega_max_v > st->max_elec_rad_s)
                omega_max_v = st->max_elec_rad_s;

            float cl_idle = st->handoff_rad_s;
            float speed_target;
            if (throttle <= THROTTLE_DEADBAND) {
                speed_target = cl_idle;
            } else {
                float pot_frac = (float)(throttle - THROTTLE_DEADBAND)
                               / (4095.0f - (float)THROTTLE_DEADBAND);
                speed_target = cl_idle
                             + pot_frac * (omega_max_v - cl_idle);
            }

            float speed_ref = speed_target;
            float speed_err = speed_ref - st->omega;

            /* Anti-windup decay */
            if (speed_err < 0.0f && st->pid_spd.integral > 0.0f) {
                st->pid_spd.integral *= 0.97f;
            }

            st->iq_ref = foc_pi_run(&st->pid_spd, speed_err, DT_SLOW);
        }

        /* Desync detection: excessive |Id| */
        #define DESYNC_ID_THRESH   2.0f
        #define DESYNC_DEBOUNCE    200U
        {
            if (st->blend_ctr > CL_HOLDOFF_SLOW_TICKS
                && st->omega > st->handoff_rad_s * 1.5f)
            {
                float abs_id = st->id_meas;
                if (abs_id < 0.0f) abs_id = -abs_id;
                if (abs_id > DESYNC_ID_THRESH) {
                    st->desync_ctr++;
                    if (st->desync_ctr >= DESYNC_DEBOUNCE) {
                        st->mode = V3_FAULT;
                        st->fault_code = FAULT_DESYNC;
                        st->desync_ctr = 0;
                    }
                } else {
                    st->desync_ctr = 0;
                }
            } else {
                st->desync_ctr = 0;
            }
        }

        /* Phantom commutation detection */
        #define PHANTOM_RATIO       0.30f
        #define PHANTOM_SPEED_THRESH 1000.0f
        #define PHANTOM_DEBOUNCE    50U
        {
            if (st->blend_ctr > CL_HOLDOFF_SLOW_TICKS
                && st->omega > PHANTOM_SPEED_THRESH)
            {
                float vd = st->vd, vq = st->vq;
                float v_mag = sqrtf(vd * vd + vq * vq);
                float v_max = st->vbus * FOC_INV_SQRT3;
                float mod = (v_max > 0.1f) ? (v_mag / v_max) : 0.0f;
                float mod_expected = st->Ke * st->omega / (v_max + 0.01f);

                if (mod < mod_expected * PHANTOM_RATIO) {
                    st->phantom_ctr++;
                    if (st->phantom_ctr >= PHANTOM_DEBOUNCE) {
                        foc_v3_stop(st);
                        st->phantom_ctr = 0;
                        return;
                    }
                } else {
                    st->phantom_ctr = 0;
                }
            } else {
                st->phantom_ctr = 0;
            }
        }

        /* Observer health exit: confidence-based.
         * SMO residual is not a convergence metric (always ≈ G×K),
         * so health is judged by BEMF confidence only.
         * On failure: fall back to V/f assist (not full restart). */
        if (st->blend_ctr > CL_HOLDOFF_SLOW_TICKS
            && st->omega >= CL_EXIT_RAD_S) {
            bool obs_healthy = (st->smo.confidence > st->handoff_conf_min * 0.5f);
            if (!obs_healthy) {
                st->observer_bad_ctr++;
                if (st->observer_bad_ctr >= OBSERVER_BAD_LIMIT) {
                    st->mode = V3_VF_ASSIST;
                    st->omega_ol = st->omega;
                    st->theta_ol = st->theta;
                    st->handoff_ctr = 0;
                    st->cl_active = false;
                    st->observer_bad_ctr = 0;
                    st->theta_error = 0.0f;
                    return;
                }
            } else {
                st->observer_bad_ctr = 0;
            }
        }

        /* Low-speed CL exit → V/f assist (not full restart) */
        if (st->blend_ctr > CL_HOLDOFF_SLOW_TICKS
            && st->omega < CL_EXIT_RAD_S)
        {
            /* Transition to V/f assist — seed omega_ol from current speed */
            st->mode = V3_VF_ASSIST;
            st->omega_ol = st->omega;
            st->theta_ol = st->theta;
            st->handoff_ctr = 0;
            st->cl_active = false;
            st->theta_error = 0.0f;
            /* PI controllers will be re-preloaded on CL re-entry */
            return;
        }

        /* Re-arm: throttle=0 and low speed for 1s → back to ARMED */
        if (throttle <= THROTTLE_DEADBAND && st->omega < 200.0f) {
            st->rearm_ctr++;
            if (st->rearm_ctr >= REARM_TIMEOUT_TICKS) {
                foc_v3_stop(st);
                st->rearm_ctr = 0;
            }
        } else {
            st->rearm_ctr = 0;
        }
    }

    /* ── V/f Assist slow-loop ─────────────────────────────────── */
    if (st->mode == V3_VF_ASSIST) {
        /* Speed target from throttle.
         * Throttle > deadband: floor at handoff_rad_s (matches CL idle).
         * Throttle = 0: target 0 → ramp down to allow clean shutdown.
         * The CL→VF_ASSIST transition only happens when speed is already
         * near CL_EXIT_RAD_S, so ramping down is smooth. */
        float vf_idle = st->handoff_rad_s;
        float speed_target;
        if (throttle <= THROTTLE_DEADBAND) {
            speed_target = 0.0f;
        } else {
            float pot_frac = (float)(throttle - THROTTLE_DEADBAND)
                           / (4095.0f - (float)THROTTLE_DEADBAND);
            speed_target = vf_idle
                         + pot_frac * (st->max_elec_rad_s - vf_idle);
        }

        /* Ramp omega_ol toward target (rate-limited) */
        float speed_err = speed_target - st->omega_ol;
        float max_step = st->ramp_rate * DT_SLOW;
        if (speed_err >  max_step) speed_err =  max_step;
        if (speed_err < -max_step) speed_err = -max_step;
        st->omega_ol += speed_err;
        if (st->omega_ol < 0.0f) st->omega_ol = 0.0f;

        /* Re-arm: zero throttle + speed decayed below floor → clean shutdown */
        if (throttle <= THROTTLE_DEADBAND && st->omega_ol < VF_ASSIST_MIN_RAD_S) {
            st->rearm_ctr++;
            if (st->rearm_ctr >= REARM_TIMEOUT_TICKS) {
                foc_v3_stop(st);
                st->rearm_ctr = 0;
            }
        } else {
            st->rearm_ctr = 0;
        }
    }
}
