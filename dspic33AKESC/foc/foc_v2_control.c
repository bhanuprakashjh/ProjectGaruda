/**
 * @file foc_v2_control.c
 * @brief FOC v2 control loop — state machine, startup, closed-loop pipeline.
 *
 * State machine:
 *   IDLE → ARMED → ALIGN → IF_RAMP → CLOSED_LOOP → FAULT
 *                                          ↓ (throttle=0 >1s)
 *                                        ARMED (clean re-arm)
 *
 * Fast loop (24 kHz): Clarke → Observer → Park → PI_d/PI_q → decoupling
 *   → circular clamp → InvPark → SVPWM → duty output
 *
 * Slow loop (1 kHz): Speed PI → Iq_ref, arming, stall detect
 *
 * Component: FOC V2
 */

#include <xc.h>
#include "foc_v2_control.h"
#include "foc_v2_math.h"
#include "foc_v2_pi.h"
#include "foc_v2_observer.h"
#include "foc_v2_detect.h"
#include "../garuda_foc_params.h"
#include "../garuda_types.h"

/* ── Constants from garuda_foc_params.h ──────────────────────── */
#define DT_FAST          FOC_TS_FAST_S
#define DT_SLOW          FOC_TS_SLOW_S
#define SLOW_DIV         FOC_SLOW_DIV

/* Dead-time compensation disabled — see note at line ~460.
 * Microchip AN1292 reference achieves clean waveforms without it. */

/* ADC calibration */
#define CAL_SAMPLES      1024U
#define CAL_SHIFT         10U

/* Overcurrent debounce (2ms at 24kHz) */
#define OC_DEBOUNCE_TICKS 48U

/* Board fault pin: RB11 = U25A(OV)+U25B(OC) → U27 AND → active-low.
 * In FOC mode, FPCI is LEB-gated off (SVM false-trips), so we poll
 * the pin directly in the fast loop. Debounce 2ms (48 ticks). */
#define BOARD_FAULT_PIN      PORTBbits.RB11
#define BOARD_FAULT_ACTIVE   0                  /* Active-low */
#define BOARD_FAULT_DEBOUNCE 48U               /* 2ms at 24kHz */

/* Time-based handoff: dwell at handoff speed before CL entry.
 * 12000 ticks = 500ms at 24kHz — gives observer time to converge.
 * PLL omega is too noisy for a speed-based gate at low BEMF. */
#define HANDOFF_LOCK_TICKS 12000U

/* Re-arm: throttle=0 for 1s in slow loop ticks */
#define REARM_TIMEOUT_TICKS 1000U

/* ── Helpers ─────────────────────────────────────────────────── */

static inline float raw_to_amps(uint16_t raw, float offset)
{
    /* Negate: OA1/OA2 inverting topology on MCLV-48V-300W DIM */
    return -((float)raw - offset) * CURRENT_SCALE_A_PER_COUNT;
}

static inline float raw_to_vbus(uint16_t raw)
{
    return (float)raw * VBUS_SCALE_V_PER_COUNT;
}

static void reset_startup(FOC_State_t *foc)
{
    foc->omega_if    = 0.0f;
    foc->theta_if    = 0.0f;
    foc->align_ctr   = 0;
    foc->ramp_ctr    = 0;
    foc->lock_ctr    = 0;
    foc->cl_active   = false;
    foc->iq_ref      = 0.0f;
    foc->id_ref      = 0.0f;
    foc->vd          = 0.0f;
    foc->vq          = 0.0f;
    foc->oc_debounce_ctr = 0;
    foc->busloss_ctr = 0;
    foc->stall_ctr   = 0;
    foc->reverse_ctr = 0;
    foc->oscillation_ctr = 0;
    foc->flip_ctr    = 0;
    foc->flip_attempts = 0;
    foc->sub_state   = 0;

    foc_pi_reset(&foc->pid_d);
    foc_pi_reset(&foc->pid_q);
    foc_pi_reset(&foc->pid_spd);
    foc_observer_reset(&foc->obs);
    foc_pll_reset(&foc->pll);
}

/* ── Public API ──────────────────────────────────────────────── */

void foc_v2_init(FOC_State_t *foc, const FOC_MotorParams_t *params)
{
    /* Zero entire state */
    FOC_State_t zero = {0};
    *foc = zero;

    foc->mode = FOC_IDLE;

    /* Copy motor params */
    foc->Rs        = params->Rs;
    foc->Ls        = params->Ls;
    foc->lambda_pm = params->lambda_pm;
    foc->Ke        = params->Ke;

    /* Dead-time compensation for observer.
     * dt_comp = Vbus × 2×Td/Tpwm — accounts for dead-time voltage
     * distortion that the observer would otherwise integrate as
     * systematic angle error.  VESC always applies this.
     * Sign fix in foc_observer_update() (was +, now -).
     * At A2212 12V: 12 × 0.024 = 0.288V — well below BEMF at
     * handoff speed (1000 × 0.000563 = 0.56V). */
    foc->dt_comp   = params->vbus_nom_v * FOC_DT_COMP_FRAC;

    /* Copy runtime startup/tuning params */
    foc->align_iq        = params->align_iq_a;
    foc->ramp_iq         = params->ramp_iq_a;
    foc->align_ticks     = params->align_ticks;
    foc->iq_ramp_ticks   = params->iq_ramp_ticks;
    foc->ramp_rate       = params->ramp_rate_rps2;
    foc->handoff_rad_s   = params->handoff_rad_s;
    foc->fault_oc_a      = params->fault_oc_a;
    foc->fault_stall_rad_s = params->fault_stall_rad_s;
    foc->obs_lpf_alpha   = params->obs_lpf_alpha;
    foc->max_elec_rad_s  = params->max_elec_rad_s;

    /* Init PI controllers */
    float vclamp = params->vbus_nom_v * 0.95f * FOC_INV_SQRT3;
    foc_pi_init(&foc->pid_d, params->kp_dq, params->ki_dq, -vclamp, vclamp);
    foc_pi_init(&foc->pid_q, params->kp_dq, params->ki_dq, -vclamp, vclamp);
    foc_pi_init(&foc->pid_spd, KP_SPD, KI_SPD,
                -params->max_current_a, params->max_current_a);

    /* Init PLL — 100 Hz BW, used for speed estimation only.
     * Observer atan2 angle drives commutation (zero lag).
     * PLL smooths speed for the speed PI and handoff check. */
    foc_pll_init(&foc->pll, 100.0f);
    foc->pll.omega_max = params->max_elec_rad_s;

    /* Init observer */
    foc_observer_reset(&foc->obs);
    foc->obs.lambda_est = params->lambda_pm;

    /* ADC calibration */
    foc->ia_offset = (float)ADC_MIDPOINT;
    foc->ib_offset = (float)ADC_MIDPOINT;
    foc->cal_accum_a = 0;
    foc->cal_accum_b = 0;
    foc->cal_count = 0;
    foc->cal_done  = false;

    foc->vbus = params->vbus_nom_v;
    foc->slow_div_ctr = 0;
    foc->arm_ctr = 0;
}

void foc_v2_start(FOC_State_t *foc)
{
    if (foc->mode == FOC_ARMED) {
        reset_startup(foc);
        foc->mode = FOC_ALIGN;
    }
}

void foc_v2_stop(FOC_State_t *foc)
{
    reset_startup(foc);
    foc->mode = FOC_ARMED;
    foc->arm_ctr = 0;
}

void foc_v2_fault(FOC_State_t *foc, uint16_t fault_code)
{
    foc->mode = FOC_FAULT;
    (void)fault_code;  /* Fault code stored in garudaData.faultCode by caller */
}

/* ── Fast Loop ───────────────────────────────────────────────── */

void foc_v2_fast_tick(FOC_State_t *foc,
                      uint16_t ia_raw, uint16_t ib_raw,
                      uint16_t vbus_raw, uint16_t throttle,
                      float *da_out, float *db_out, float *dc_out)
{
    /* Default: center duty (safe) */
    *da_out = 0.5f;
    *db_out = 0.5f;
    *dc_out = 0.5f;

    /* Vbus conversion + dynamic dead-time compensation */
    foc->vbus = raw_to_vbus(vbus_raw);
    foc->dt_comp = foc->vbus * FOC_DT_COMP_FRAC;

    /* ── IDLE / ARMED: ADC offset calibration, no motor drive ── */
    if (foc->mode == FOC_IDLE || foc->mode == FOC_ARMED) {
        /* ADC zero-current offset calibration (motor off → no current) */
        if (!foc->cal_done) {
            foc->cal_accum_a += ia_raw;
            foc->cal_accum_b += ib_raw;
            if (++foc->cal_count >= CAL_SAMPLES) {
                foc->ia_offset = (float)(foc->cal_accum_a >> CAL_SHIFT);
                foc->ib_offset = (float)(foc->cal_accum_b >> CAL_SHIFT);
                foc->cal_done  = true;
            }
        }
        foc->sub_state = (foc->mode == FOC_ARMED) ? 1 : 0;
        foc->id_meas = 0.0f;
        foc->iq_meas = 0.0f;
        foc->omega   = 0.0f;
        goto slow_loop;
    }

    /* ── FAULT: outputs disabled ─────────────────────────────── */
    if (foc->mode == FOC_FAULT) {
        foc->sub_state = 0;
        *da_out = 0.5f;
        *db_out = 0.5f;
        *dc_out = 0.5f;
        goto slow_loop;
    }

    /* ── MOTOR_DETECT: auto-commissioning ────────────────────── */
    if (foc->mode == FOC_MOTOR_DETECT) {
        float ia = raw_to_amps(ia_raw, foc->ia_offset);
        float ib = raw_to_amps(ib_raw, foc->ib_offset);
        float i_alpha, i_beta;
        foc_clarke(ia, ib, &i_alpha, &i_beta);

        bool done = foc_detect_fast_tick(foc, ia, ib, i_alpha, i_beta,
                                         da_out, db_out, dc_out);
        if (done) {
            if (foc->detect_state == DETECT_DONE) {
                /* Apply measured params and transition to ARMED */
                foc_detect_apply(foc);
                reset_startup(foc);
                foc->mode = FOC_ARMED;
                foc->arm_ctr = 0;
            } else {
                /* Detection failed */
                foc->mode = FOC_FAULT;
            }
            *da_out = 0.5f;
            *db_out = 0.5f;
            *dc_out = 0.5f;
        }
        goto slow_loop;
    }

    /* ── Active modes: ALIGN, IF_RAMP, CLOSED_LOOP ──────────── */
    {
        /* Convert ADC to physical units */
        float ia = raw_to_amps(ia_raw, foc->ia_offset);
        float ib = raw_to_amps(ib_raw, foc->ib_offset);

        /* Dynamic PI voltage clamp — tracks Vbus.
         * No feedforward → PI gets full symmetric voltage budget.
         * PI will naturally produce BEMF voltage + correction. */
        float vclamp = foc->vbus * 0.95f * FOC_INV_SQRT3;
        foc->pid_d.out_max =  vclamp;
        foc->pid_d.out_min = -vclamp;
        foc->pid_q.out_max =  vclamp;
        foc->pid_q.out_min = -vclamp;

        /* 1. Clarke transform */
        float i_alpha, i_beta;
        foc_clarke(ia, ib, &i_alpha, &i_beta);

        /* 2. Observer update (uses previous tick's applied voltage) */
        if (foc->mode == FOC_CLOSED_LOOP || foc->mode == FOC_IF_RAMP) {
            /* Use reconstructed applied voltage for observer */
            float sin_t = sinf(foc->theta);
            float cos_t = cosf(foc->theta);
            float v_alpha, v_beta;
            foc_inv_park(foc->vd, foc->vq, sin_t, cos_t, &v_alpha, &v_beta);

            /* Approximate duty for observer gain scheduling:
             * |V| / Vbus gives effective modulation index. */
            float v_mag = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
            float duty_abs = (foc->vbus > 1.0f) ? (v_mag / foc->vbus) : 0.05f;

            foc_observer_update(&foc->obs, &foc->pll,
                                v_alpha, v_beta,
                                i_alpha, i_beta,
                                foc->Rs, foc->Ls, foc->lambda_pm,
                                foc->dt_comp, duty_abs,
                                DT_FAST);

            /* BEMF-d angle correction: provides closed-loop angle feedback.
             *
             * Without this, the observer is a pure open-loop flux integrator
             * with no mechanism to correct systematic angle errors from
             * dead-time, voltage reconstruction, and Rs mismatch.
             * At high speed, these errors accumulate faster (more electrical
             * cycles per second) and eventually cause observer divergence.
             *
             * The BEMFd correction uses the d-axis voltage equation
             * residual to detect and correct angle errors. Speed-gated:
             * off below 2000 rad/s (where startup current creates false
             * BEMFd), ramps to full at 4000 rad/s.
             *
             * This turns the observer from open-loop to closed-loop. */
            if (foc->mode == FOC_CLOSED_LOOP) {
                /* Park transform the measured current to get id/iq */
                float sin_obs = sinf(foc->obs.theta_est);
                float cos_obs = cosf(foc->obs.theta_est);
                float id_now, iq_now;
                foc_park(i_alpha, i_beta, sin_obs, cos_obs, &id_now, &iq_now);

                foc_observer_bemf_correct(&foc->obs,
                                          foc->vd, foc->vq,
                                          id_now, iq_now,
                                          foc->pll.omega_est,
                                          foc->Rs, foc->Ls, foc->lambda_pm);
            }
        }

        /* 3. Select commutation angle based on mode */
        float theta_drive;
        float id_ref = 0.0f;
        float iq_ref;

        switch (foc->mode) {
        case FOC_ALIGN: {
            /* Fixed angle θ=0, ramp Id from 0 to ALIGN_IQ.
             * MUST use Id (not Iq) for alignment!  At θ=0:
             *   Id → field along α-axis → rotor PM aligns at θ=0 ✓
             *   Iq → field along β-axis → rotor PM aligns at θ=90° ✗
             * Using Iq puts rotor 90° from expected angle — I/f ramp
             * then has zero real torque and loses sync under any load. */
            foc->align_ctr++;
            theta_drive = 0.0f;

            float align_frac = (float)foc->align_ctr / (float)foc->align_ticks;
            if (align_frac > 1.0f) align_frac = 1.0f;
            id_ref = foc->align_iq * align_frac;
            iq_ref = 0.0f;

            /* Transition: alignment complete → IF_RAMP */
            if (foc->align_ctr >= foc->align_ticks) {
                /* Seed observer: rotor aligned at θ=0 */
                foc_observer_seed(&foc->obs, foc->lambda_pm, 0.0f);
                foc->pll.theta_est = 0.0f;
                foc->pll.omega_est = 0.0f;

                foc->theta_if = 0.0f;
                foc->omega_if = 0.0f;
                foc->ramp_ctr = 0;
                foc->mode = FOC_IF_RAMP;
            }
            break;
        }

        case FOC_IF_RAMP: {
            /* I/f open-loop: forced angle + PI current control.
             * Simple linear speed ramp with time-based handoff.
             * No PLL/observer feedback — at low speed the BEMF is too
             * weak for reliable observer tracking (A2212 BEMF = 0.56V
             * at 1000 rad/s, below ADC noise floor at lower speeds).
             * High torque (5A) and slow ramp (500 rad/s²) ensure the
             * rotor follows the forced angle through prop load. */
            foc->ramp_ctr++;

            /* Smooth d→q current transition */
            if (foc->ramp_ctr < foc->iq_ramp_ticks) {
                float frac = (float)foc->ramp_ctr / (float)foc->iq_ramp_ticks;
                iq_ref = foc->ramp_iq * frac;
                id_ref = foc->align_iq * (1.0f - frac);
            } else {
                iq_ref = foc->ramp_iq;
            }

            /* Linear speed ramp — no feedback gating */
            float max_delta = foc->ramp_rate * DT_FAST;
            foc->omega_if += max_delta;
            if (foc->omega_if > foc->handoff_rad_s * 1.1f)
                foc->omega_if = foc->handoff_rad_s * 1.1f;

            /* Advance forced angle */
            foc->theta_if += foc->omega_if * DT_FAST;
            foc->theta_if = foc_angle_wrap(foc->theta_if);
            theta_drive = foc->theta_if;

            /* Handoff: time-based.  Once omega_if reaches handoff speed,
             * wait 500ms (12000 ticks) for observer to converge, then
             * switch to CL.  PLL omega is too noisy at low speed (A2212
             * BEMF = 0.56V at 1000 rad/s, observer jitter ±0.5 rad →
             * PLL speed jitter ±300 rad/s) to use as a gate.
             * If motor was actually desynced, CL stall detection catches
             * it within ~500ms and faults — clean retry path. */
            if (foc->omega_if >= foc->handoff_rad_s) {
                if (++foc->lock_ctr >= HANDOFF_LOCK_TICKS) {
                    /* Sanity: PLL must show forward rotation.  If observer
                     * is completely lost (ω < 0), skip CL — keep running
                     * I/f and retry next tick. */
                    if (foc->pll.omega_est <= 0.0f) {
                        foc->lock_ctr = HANDOFF_LOCK_TICKS - 1U;
                        break;
                    }

                    /* Bumpless transfer: transform PI integrator state
                     * from OL frame (theta_if) to CL frame (obs.theta_est).
                     * Rotates voltage vector to new d-q frame so PI
                     * integrators don't see a discontinuity. */
                    float delta_th = foc->theta_if - foc->obs.theta_est;
                    float cos_d = cosf(delta_th);
                    float sin_d = sinf(delta_th);
                    float vd_new =  foc->vd * cos_d + foc->vq * sin_d;
                    float vq_new = -foc->vd * sin_d + foc->vq * cos_d;
                    foc_pi_preload(&foc->pid_d, vd_new);
                    foc_pi_preload(&foc->pid_q, vq_new);

                    /* Speed PI preload with measured Iq */
                    float iq_actual = foc->iq_meas > 0.0f ? foc->iq_meas : 0.0f;
                    foc_pi_preload(&foc->pid_spd, iq_actual);

                    foc->cl_active = true;
                    foc->mode = FOC_CLOSED_LOOP;
                }
            }
            break;
        }

        case FOC_CLOSED_LOOP: {
            /* Observer atan2 angle for commutation + phase lag compensation.
             * The observer angle reflects rotor position at ADC sample time,
             * but PWM is applied 0.5 sample later. At high speed this
             * systematic lag causes angle error:
             *   4000 rad/s: 0.5 × 41.67µs × 4000 = 0.083 rad = 4.8°
             * VESC: phase += pll_speed × dt × (0.5 + offset)
             * Without this, observer accumulates systematic angle drift. */
            theta_drive = foc->obs.theta_est
                        + foc->pll.omega_est * DT_FAST * FOC_PHASE_LAG_COMP;
            theta_drive = foc_angle_wrap(theta_drive);
            float cl_omega = foc->pll.omega_est;

            /* Speed/current references from slow loop */
            iq_ref = foc->iq_ref;
            id_ref = foc->id_ref;  /* Observer-assist injection at high speed */

            /* Track speed for telemetry */
            foc->omega = cl_omega;
            break;
        }

        default:
            theta_drive = 0.0f;
            iq_ref = 0.0f;
            break;
        }

        /* 4. Park transform */
        float sin_t = sinf(theta_drive);
        float cos_t = cosf(theta_drive);
        float id_meas, iq_meas;
        foc_park(i_alpha, i_beta, sin_t, cos_t, &id_meas, &iq_meas);

        /* 5-6. PI controllers */
        float vd_pi = foc_pi_run(&foc->pid_d, id_ref - id_meas, DT_FAST);
        float vq_pi = foc_pi_run(&foc->pid_q, iq_ref - iq_meas, DT_FAST);

        /* 7. No feedforward — let PI controllers handle everything.
         * Feedforward (BEMF + cross-coupling) amplifies parameter errors
         * into systematic voltage distortion.  Microchip AN1292 reference
         * uses no feedforward and achieves clean sinusoidal currents.
         * Can be re-enabled once motor params are well-characterized. */
        float vd_tot = vd_pi;
        float vq_tot = vq_pi;

        /* 8. Circular voltage clamp (q-axis priority)
         * Vq carries the BEMF at high speed — preserving Vq budget
         * keeps the observer fed with the signal it needs for angle
         * tracking.  Vd gets whatever headroom remains. */
        float vmax = foc->vbus * 0.95f * FOC_INV_SQRT3;
        vq_tot = foc_clampf(vq_tot, -vmax, vmax);
        float sq = vmax * vmax - vq_tot * vq_tot;
        if (sq < 0.0f) sq = 0.0f;
        float vd_lim = sqrtf(sq);
        vd_tot = foc_clampf(vd_tot, -vd_lim, vd_lim);

        /* NOTE: Current PI anti-windup relies on the dynamic PI limits
         * set at lines 215-224 (vq_pi_budget tracks BEMF feedforward).
         * The circular voltage clamp may occasionally clip beyond these
         * limits, but the PI's internal Tustin anti-windup prevents
         * severe accumulation.  Speed PI windup (the main issue at the
         * voltage ceiling) is handled by the dynamic speed PI clamp
         * in the slow loop. */

        /* Save total voltage for observer on next tick */
        foc->vd = vd_tot;
        foc->vq = vq_tot;

        /* 9. Inverse Park (total voltage = PI + feedforward, clamped) */
        float v_alpha, v_beta;
        foc_inv_park(vd_tot, vq_tot, sin_t, cos_t, &v_alpha, &v_beta);

        /* 10-11. SVPWM */
        foc_svpwm(v_alpha, v_beta, foc->vbus, da_out, db_out, dc_out);

        /* Dead-time compensation disabled — Microchip AN1292 reference
         * does not use software DT comp and achieves clean waveforms.
         * DT comp (1.8% duty = 0.43V) was larger than Rs*I at low current
         * and any sign error created more distortion than it fixed. */

        /* 12. Update commutation angle for next tick */
        foc->theta = theta_drive;

        /* 13. Telemetry */
        foc->id_meas   = id_meas;
        foc->iq_meas   = iq_meas;
        foc->theta_obs = foc->obs.theta_est;
        foc->omega_pll = foc->pll.omega_est;
        foc->sub_state = (foc->mode == FOC_ALIGN)       ? 2 :
                         (foc->mode == FOC_IF_RAMP)      ? 3 :
                         (foc->mode == FOC_CLOSED_LOOP)  ? 4 : 0;

        /* 14. Software overcurrent check + bus-loss detection */
        float i_sq = id_meas * id_meas + iq_meas * iq_meas;
        if (i_sq > foc->fault_oc_a * foc->fault_oc_a) {
            if (++foc->oc_debounce_ctr >= OC_DEBOUNCE_TICKS) {
                foc->mode = FOC_FAULT;
                foc->fault_code = FAULT_OVERCURRENT;
                foc->oc_debounce_ctr = 0;
                *da_out = 0.5f;
                *db_out = 0.5f;
                *dc_out = 0.5f;
            }
        } else {
            foc->oc_debounce_ctr = 0;
        }

        /* 14b. Board fault detection: U25A(OV)+U25B(OC) → U27 AND → RB11.
         * Active-low. Poll pin directly (FPCI LEB-gated off in FOC mode). */
        if (BOARD_FAULT_PIN == BOARD_FAULT_ACTIVE) {
            if (++foc->busloss_ctr >= BOARD_FAULT_DEBOUNCE) {
                foc->mode = FOC_FAULT;
                foc->fault_code = FAULT_BOARD_PCI;
                foc->busloss_ctr = 0;
                *da_out = 0.5f;
                *db_out = 0.5f;
                *dc_out = 0.5f;
            }
        } else {
            foc->busloss_ctr = 0;
        }

        /* 15. Stall detection (CL only) */
        if (foc->mode == FOC_CLOSED_LOOP &&
            throttle >= THROTTLE_DEADBAND &&
            foc->iq_ref > 0.1f) {
            float abs_omega = foc->pll.omega_est;
            if (abs_omega < 0.0f) abs_omega = -abs_omega;
            if (abs_omega < foc->fault_stall_rad_s) {
                foc->stall_ctr++;
                if (foc->stall_ctr >= (uint32_t)(FAULT_STALL_TIMEOUT_MS * SLOW_DIV)) {
                    foc->mode = FOC_FAULT;
                    foc->fault_code = FAULT_STALL;
                    foc->stall_ctr = 0;
                    *da_out = 0.5f;
                    *db_out = 0.5f;
                    *dc_out = 0.5f;
                }
            } else {
                foc->stall_ctr = 0;
            }
        }

        /* 15b. Fast observer health checks (CL only, runs at 24kHz).
         * These must catch problems BEFORE the 3ms oscillation-to-HW-OC
         * window closes. */
        if (foc->mode == FOC_CLOSED_LOOP) {
            /* (a) Reverse speed: observer angle has flipped 180°.
             * Use pll.omega_est directly (omega_pll copy may be deferred
             * by compiler). 48 ticks (2ms) debounce. */
            if (foc->pll.omega_est < -50.0f) {
                foc->reverse_ctr++;
                if (foc->reverse_ctr >= 48) {
                    foc->mode = FOC_FAULT;
                    foc->fault_code = FAULT_OBSERVER;
                    foc->reverse_ctr = 0;
                    *da_out = 0.5f;
                    *db_out = 0.5f;
                    *dc_out = 0.5f;
                }
            } else {
                foc->reverse_ctr = 0;
            }

            /* (b) Decay flip attempts when stable (48000 ticks = 2s).
             * Previous oscillation detector (i_sq > OC²×0.25, +3/-1
             * counter) removed — false-triggered on normal 12A Iq
             * with A2212+prop because threshold (156) was within
             * operating range.  Real OC is handled by section 14. */
            if (foc->flip_attempts > 0) {
                if (++foc->flip_ctr >= 48000U) {
                    foc->flip_attempts--;
                    foc->flip_ctr = 0;
                }
            } else {
                foc->flip_ctr = 0;
            }
        }
    }

    /* ── Slow loop (every SLOW_DIV ticks → 1 kHz) ───────────── */
slow_loop:
    if (++foc->slow_div_ctr >= SLOW_DIV) {
        foc->slow_div_ctr = 0;

        if (foc->mode == FOC_ARMED) {
            /* Arming: throttle at zero for ARM_TIME_MS → ALIGN */
            if (throttle < THROTTLE_DEADBAND) {
                if (++foc->arm_ctr >= (uint32_t)FOC_ARM_TIME_MS) {
                    reset_startup(foc);
                    foc->mode = FOC_ALIGN;
                }
            } else {
                foc->arm_ctr = 0;
            }
        }
        else if (foc->mode == FOC_CLOSED_LOOP) {
            /* Speed PI: pot → speed ref → Iq ref.
             * Minimum idle speed = handoff speed to keep observer tracking.
             * Below this speed, BEMF is too weak for reliable sensorless. */
            float idle_speed = foc->handoff_rad_s;
            float speed_ref;
            if (throttle < THROTTLE_DEADBAND) {
                speed_ref = idle_speed;
            } else {
                float pot_frac = (float)(throttle - THROTTLE_DEADBAND)
                               / (4095.0f - (float)THROTTLE_DEADBAND);
                speed_ref = idle_speed + pot_frac * (foc->max_elec_rad_s - idle_speed);
            }

            /* Voltage-limited speed cap: prevent commanding speeds that
             * require more BEMF voltage than the bus can deliver.
             * Reserve 20% of Vq budget for current control headroom. */
            float vq_avail = foc->vbus * 0.80f * FOC_INV_SQRT3;
            float omega_max_v = vq_avail / foc->Ke;
            if (speed_ref > omega_max_v)
                speed_ref = omega_max_v;

            /* No-load speed limiter: prevent observer 180° flip.
             *
             * At no-load high speed, phase current → 0 and the flux
             * observer has no signal to distinguish 0° from 180°.
             * With 30µH inductance, a 180° flip causes 400kA/s current
             * slew → HW OC trips in <2 ticks (latching on MCLV board).
             * No software approach can react fast enough.
             *
             * Prevention: if |Iq| < 1.5A (no load) at speed > 4000 rad/s
             * for >250ms, cap speed to 5000 rad/s.  This keeps the motor
             * below the unstable region.  With prop load (Iq > 3A at
             * 3000+ rad/s), the limiter never activates.
             *
             * The limit is removed as soon as Iq exceeds the threshold
             * (load applied), allowing instant full-speed operation. */
            {
                float abs_iq = foc->iq_meas;
                if (abs_iq < 0.0f) abs_iq = -abs_iq;
                float abs_spd = foc->pll.omega_est;
                if (abs_spd < 0.0f) abs_spd = -abs_spd;

                /* Aggressive no-load limiter: observer has no angle correction
                 * signal at no-load (BEMF-d needs current), so it's a pure
                 * open-loop integrator that WILL diverge above ~4000 rad/s.
                 * Gate at 3000, cap at 3500, activate in 20ms.
                 * With prop load (Iq > 2A), limiter is inactive. */
                #define NOLOAD_IQ_THRESH  2.0f    /* A — below this = no-load */
                #define NOLOAD_SPD_GATE   3000.0f /* rad/s — start checking */
                #define NOLOAD_SPD_CAP    3500.0f /* rad/s — cap speed here */
                #define NOLOAD_DWELL_MS   20U     /* ms — fast activation */

                static uint16_t noload_ctr = 0;
                static bool noload_active = false;

                if (abs_iq < NOLOAD_IQ_THRESH && abs_spd > NOLOAD_SPD_GATE) {
                    if (noload_ctr < NOLOAD_DWELL_MS)
                        noload_ctr++;
                    if (noload_ctr >= NOLOAD_DWELL_MS)
                        noload_active = true;
                } else {
                    noload_ctr = 0;
                    noload_active = false;
                }

                if (noload_active && speed_ref > NOLOAD_SPD_CAP)
                    speed_ref = NOLOAD_SPD_CAP;
            }

            foc->iq_ref = foc_pi_run(&foc->pid_spd,
                                      speed_ref - foc->pll.omega_est,
                                      DT_SLOW);

            /* Modulation index soft-limit: when approaching voltage
             * saturation, reduce Iq to maintain current control authority
             * and prevent observer starvation.
             * At mod > 0.85 → linearly reduce to 0 at mod = 0.95.
             * This keeps ~15% voltage headroom for the current PI. */
            {
                float v_mag_sq = foc->vd * foc->vd + foc->vq * foc->vq;
                float vmax_sq  = vq_avail * vq_avail;  /* 80% bus */
                float mod_ratio = (vmax_sq > 0.01f) ? (v_mag_sq / vmax_sq) : 0.0f;
                /* mod_ratio > 1.0 means we're already past 80% bus.
                 * Scale down Iq when mod_ratio > 0.85^2/0.80^2 ≈ 1.13
                 * i.e., when actual mod_index > 0.85 */
                const float MOD_SOFT_START = 0.85f * 0.85f / (0.80f * 0.80f);
                const float MOD_HARD_LIMIT = 0.95f * 0.95f / (0.80f * 0.80f);
                if (mod_ratio > MOD_SOFT_START) {
                    float scale = 1.0f - (mod_ratio - MOD_SOFT_START)
                                       / (MOD_HARD_LIMIT - MOD_SOFT_START);
                    if (scale < 0.05f) scale = 0.05f;
                    foc->iq_ref *= scale;
                }
            }

            /* Id = 0 in CL (MTPA for non-salient motor).
             * Previous -1A injection for observer assist removed:
             * d-axis current doesn't break 0°/180° symmetry, and
             * the no-load speed limiter now prevents the flip. */
            foc->id_ref = 0.0f;

            /* Re-arm: throttle=0 and low speed for 1s → back to ARMED */
            static uint32_t rearm_ctr = 0;
            float abs_omega = foc->pll.omega_est;
            if (abs_omega < 0.0f) abs_omega = -abs_omega;
            if (throttle < THROTTLE_DEADBAND && abs_omega < foc->fault_stall_rad_s * 2.0f) {
                if (++rearm_ctr >= REARM_TIMEOUT_TICKS) {
                    foc_v2_stop(foc);
                    rearm_ctr = 0;
                }
            } else {
                rearm_ctr = 0;
            }
        }
    }
}
