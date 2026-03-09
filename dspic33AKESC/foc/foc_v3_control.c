/**
 * @file foc_v3_control.c
 * @brief FOC v3 control loop — SMO observer + open-loop ramp startup.
 *
 * State machine:
 *   IDLE → ARMED → ALIGN → OL_RAMP → CLOSED_LOOP → FAULT
 *                                          ↓ (throttle=0 >1s)
 *                                        ARMED (clean re-arm)
 *
 * Fast loop (24 kHz): Clarke → SMO → Park → PI_d/PI_q
 *   → circular clamp → InvPark → SVPWM → duty output
 *
 * Slow loop (1 kHz): Speed PI → Iq_ref, arming, fault detect
 *
 * The SMO runs free during OL ramp (receives real I, V but its
 * angle is not used for commutation). At handoff speed, if the
 * PLL tracks consistently, commutation switches to SMO angle.
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
 * Low-Rs motors (A2212: 65mΩ) need voltage-mode at startup.
 * PI steady-state voltage (Rs×Iq = 0.26V) is insufficient to
 * maintain rotor lock-in.  V/f at 1.0V provides ~15A starting
 * torque, naturally reducing as BEMF builds with speed.
 * PI takes over only at CL handoff when BEMF is high enough. */
#define VF_OL_VOLTAGE        1.0f   /* Vq during V/f open-loop (V) */
#define VF_OL_RAMP_RATE      200.0f /* Electrical rad/s² during V/f */
#define VF_OL_SPEED_CAP      500.0f /* rad/s — proven V/f sync limit */

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

/* Handoff: dwell at handoff speed before CL entry.
 * 6000 ticks = 250ms — let SMO converge. */
#define HANDOFF_DWELL_TICKS 6000U

/* Re-arm: throttle=0 for 1s in slow loop ticks */
#define REARM_TIMEOUT_TICKS 1000U

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
    st->handoff_ctr = 0;
    st->cl_active   = false;
    st->iq_ref      = 0.0f;
    st->id_ref      = 0.0f;
    st->vd          = 0.0f;
    st->vq          = 0.0f;
    st->oc_debounce_ctr = 0;
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

    /* Initialize SMO observer */
    v3_smo_init(&st->smo, params->Rs, params->Ls, params->lambda_pm,
             params->vbus_nom_v, DT_FAST);

    /* Initialize PLL (100 Hz BW for speed estimation) */
    v3_smo_pll_init(&st->pll, 100.0f, params->max_elec_rad_s);

    /* Current PI controllers */
    float vmax = params->vbus_nom_v * 0.95f * FOC_INV_SQRT3;
    foc_pi_init(&st->pid_d, params->kp_dq, params->ki_dq, -vmax, vmax);
    foc_pi_init(&st->pid_q, params->kp_dq, params->ki_dq, -vmax, vmax);

    /* Speed PI controller */
    foc_pi_init(&st->pid_spd, KP_SPD, KI_SPD,
                -MOTOR_MAX_CURRENT_A, MOTOR_MAX_CURRENT_A);
}

void foc_v3_start(V3_State_t *st)
{
    if (st->mode == V3_ARMED) {
        reset_startup(st);
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

    /* ── Motor running: ALIGN / OL_RAMP / CLOSED_LOOP ──────── */

    /* Dynamic PI voltage clamp — tracks Vbus (matches v2) */
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

    /* 3. Reconstruct applied voltage from previous PI output.
     *    Transform last-tick Vd/Vq back to αβ at current commutation
     *    angle. This gives the SMO the voltage the motor actually saw. */
    float v_alpha, v_beta;
    {
        float sin_prev = sinf(st->theta);
        float cos_prev = cosf(st->theta);
        foc_inv_park(st->vd, st->vq, sin_prev, cos_prev, &v_alpha, &v_beta);
    }

    /* 4. Run SMO observer (always, even during OL — let it converge).
     * Speed hint for adaptive LPF: during V/f we know the real speed
     * (omega_ol), so use it directly. Using PLL omega creates a
     * chicken-and-egg: PLL needs SMO → SMO needs correct LPF → LPF
     * needs real speed → PLL is garbage. */
    float smo_omega_hint;
    if (st->mode == V3_ALIGN || st->mode == V3_OL_RAMP)
        smo_omega_hint = st->omega_ol;
    else
        smo_omega_hint = st->pll.omega_est;

    v3_smo_update(&st->smo,
               v_alpha, v_beta,
               i_alpha, i_beta,
               smo_omega_hint, DT_FAST);

    /* 5. Run PLL on SMO angle */
    v3_smo_pll_update(&st->pll, st->smo.theta_est, DT_FAST);

    /* 6. Select commutation angle based on mode */
    float theta_drive;
    float iq_ref, id_ref;

    switch (st->mode) {
    case V3_ALIGN: {
        /* V/f alignment: hold θ=0 with VF_OL_VOLTAGE on q-axis.
         * This locks rotor to d-axis. Voltage is applied as Vq
         * which at θ=0 produces a stationary stator field. */
        theta_drive = 0.0f;
        st->align_ctr++;

        /* Refs unused in V/f mode but set for telemetry */
        id_ref = 0.0f;
        iq_ref = 0.0f;

        if (st->align_ctr >= st->align_ticks) {
            st->mode = V3_OL_RAMP;
            st->ramp_ctr = 0;
            st->omega_ol = 0.0f;
            st->theta_ol = 0.0f;

            /* Seed SMO with alignment angle */
            v3_smo_reset(&st->smo);
            v3_smo_pll_reset(&st->pll);
        }

        st->sub_state = 2;  /* ALIGN */
        break;
    }

    case V3_OL_RAMP: {
        /* V/f open-loop: constant Vq at forced angle, ramp speed.
         * No current feedback — voltage drives the motor directly.
         * Current is self-limited: I = (Vq - BEMF) / Z_motor.
         * As speed increases, BEMF rises → current drops naturally. */
        st->ramp_ctr++;

        /* Refs unused in V/f mode but set for telemetry */
        iq_ref = 0.0f;
        id_ref = 0.0f;

        /* Speed ramp — cap at proven V/f sync limit */
        st->omega_ol += VF_OL_RAMP_RATE * DT_FAST;
        if (st->omega_ol > VF_OL_SPEED_CAP)
            st->omega_ol = VF_OL_SPEED_CAP;

        /* Advance forced angle */
        st->theta_ol += st->omega_ol * DT_FAST;
        st->theta_ol = foc_angle_wrap(st->theta_ol);

        theta_drive = st->theta_ol;

        /* Check handoff condition: speed above threshold + PLL tracking.
         * Dwell at handoff speed to let SMO converge. */
        if (st->omega_ol >= VF_OL_SPEED_CAP) {
            /* Check if PLL speed is in the right ballpark
             * (within 50% of OL speed, same direction) */
            float pll_speed = st->pll.omega_est;
            float ol_speed = st->omega_ol;
            bool pll_tracking = (pll_speed > ol_speed * 0.3f) &&
                                (pll_speed < ol_speed * 2.0f);

            if (pll_tracking) {
                st->handoff_ctr++;
            } else {
                st->handoff_ctr = 0;
            }

            if (st->handoff_ctr >= HANDOFF_DWELL_TICKS) {
                /* Transition to closed loop */
                st->mode = V3_CLOSED_LOOP;
                st->cl_active = true;

                /* Preload current PI integrators with expected voltages
                 * at handoff speed to prevent transient.
                 * Vq_ss ≈ Ke×ω + Rs×Iq_est (Iq_est from current Iq measurement)
                 * Vd_ss ≈ 0 */
                float iq_now = st->iq_meas;
                if (iq_now < 0.5f) iq_now = 0.5f; /* floor */
                float vq_preload = st->Ke * st->omega_ol
                                 + st->Rs * iq_now;
                foc_pi_preload(&st->pid_q, vq_preload);
                foc_pi_preload(&st->pid_d, 0.0f);

                /* Seed speed PI with current Iq */
                foc_pi_preload(&st->pid_spd, iq_now);
            }
        }

        st->sub_state = 3;  /* OL_RAMP */
        break;
    }

    case V3_CLOSED_LOOP: {
        /* SMO angle for commutation + phase lag compensation.
         * Half-sample transport delay: θ += ω × dt × 0.5 */
        theta_drive = st->smo.theta_est
                    + st->pll.omega_est * DT_FAST * FOC_PHASE_LAG_COMP;
        theta_drive = foc_angle_wrap(theta_drive);

        /* Speed/current references from slow loop */
        iq_ref = st->iq_ref;
        id_ref = 0.0f;  /* MTPA for non-salient motor */

        /* Track speed for telemetry */
        st->omega = st->pll.omega_est;

        st->sub_state = 4;  /* CL */
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

    /* 8. Voltage generation — V/f for OL modes, PI for CL */
    float vd_tot, vq_tot;

    if (st->mode == V3_ALIGN || st->mode == V3_OL_RAMP) {
        /* V/f: voltage at forced angle.
         * During ALIGN: ramp voltage from 0→VF_OL_VOLTAGE to limit
         * inrush current (1V/65mΩ = 15A without ramp!).
         * During OL_RAMP: full voltage as θ ramps. */
        vd_tot = 0.0f;
        if (st->mode == V3_ALIGN) {
            float vfrac = (float)st->align_ctr / (float)st->align_ticks;
            if (vfrac > 1.0f) vfrac = 1.0f;
            vq_tot = VF_OL_VOLTAGE * vfrac;
        } else {
            vq_tot = VF_OL_VOLTAGE;
        }
    } else {
        /* CL: PI current control */
        float vd_pi = foc_pi_run(&st->pid_d, id_ref - id_meas, DT_FAST);
        float vq_pi = foc_pi_run(&st->pid_q, iq_ref - iq_meas, DT_FAST);

        vd_tot = vd_pi;
        vq_tot = vq_pi;

        /* Circular voltage clamp (q-axis priority) */
        float vmax = st->vbus * 0.95f * FOC_INV_SQRT3;
        vq_tot = foc_clampf(vq_tot, -vmax, vmax);
        float sq = vmax * vmax - vq_tot * vq_tot;
        if (sq < 0.0f) sq = 0.0f;
        float vd_lim = sqrtf(sq);
        vd_tot = foc_clampf(vd_tot, -vd_lim, vd_lim);
    }

    /* Store for telemetry and next-tick voltage reconstruction */
    st->vd = vd_tot;
    st->vq = vq_tot;

    /* 10. Inverse Park → αβ */
    float v_alpha_out, v_beta_out;
    foc_inv_park(vd_tot, vq_tot, sin_t, cos_t, &v_alpha_out, &v_beta_out);

    /* 11. SVPWM → duty cycles */
    foc_svpwm(v_alpha_out, v_beta_out, st->vbus, da_out, db_out, dc_out);

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
        static uint16_t board_fault_ctr = 0;
        if (BOARD_FAULT_PIN == BOARD_FAULT_ACTIVE) {
            board_fault_ctr++;
            if (board_fault_ctr >= BOARD_FAULT_DEBOUNCE) {
                st->mode = V3_FAULT;
                st->fault_code = FAULT_BOARD_PCI;
                *da_out = 0.5f;
                *db_out = 0.5f;
                *dc_out = 0.5f;
            }
        } else {
            board_fault_ctr = 0;
        }
    }

    /* ── Slow loop (every SLOW_DIV ticks → 1 kHz) ───────────── */
slow_loop:
    st->slow_div_ctr++;
    if (st->slow_div_ctr < SLOW_DIV) return;
    st->slow_div_ctr = 0;

    /* Arming logic: garuda_service.c ISR syncs ESC_ARMED → V3_ARMED.
     * Main loop (SW1 or GSP) sets garudaData.state = ESC_ARMED,
     * ISR picks it up at the top of the v3 block.
     * Do NOT auto-arm here — that bypasses the user's start command. */

    /* ARMED → ALIGN: hold zero throttle for FOC_ARM_TIME_MS, then start.
     * Same as v2 / 6-step: SW1 arms, arm counter verifies zero throttle,
     * then auto-transitions to ALIGN. */
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

    /* Closed-loop slow loop: speed PI */
    if (st->mode == V3_CLOSED_LOOP) {
        /* Map throttle to speed reference */
        float speed_ref = 0.0f;
        if (throttle > THROTTLE_DEADBAND) {
            speed_ref = (float)(throttle - THROTTLE_DEADBAND) * THROTTLE_SPEED_SCALE;
        }

        /* Voltage-limited speed: prevent PI saturation */
        float omega_max_v = 0.80f * st->vbus / (FOC_SQRT3 * st->Ke + 1e-9f);
        if (omega_max_v > st->max_elec_rad_s)
            omega_max_v = st->max_elec_rad_s;
        if (speed_ref > omega_max_v)
            speed_ref = omega_max_v;

        st->iq_ref = foc_pi_run(&st->pid_spd,
                                 speed_ref - st->pll.omega_est,
                                 DT_SLOW);

        /* Re-arm: throttle=0 and low speed for 1s → back to ARMED */
        static uint32_t rearm_ctr = 0;
        float abs_omega = st->pll.omega_est;
        if (abs_omega < 0.0f) abs_omega = -abs_omega;

        if (throttle <= THROTTLE_DEADBAND && abs_omega < 200.0f) {
            rearm_ctr++;
            if (rearm_ctr >= REARM_TIMEOUT_TICKS) {
                foc_v3_stop(st);
                rearm_ctr = 0;
            }
        } else {
            rearm_ctr = 0;
        }
    }
}
