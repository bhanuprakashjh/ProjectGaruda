/**
 * @file foc_v2_detect.c
 * @brief Motor auto-commissioning: Rs, Ls, λ_pm measurement + PI auto-tune.
 *
 * Runs inside the 24 kHz fast loop using the existing PI + SVPWM pipeline.
 *
 * Phase 1 — Rs (resistance):
 *   Command Id=TEST_CURRENT at θ=0, PI drives Vd to maintain it.
 *   After settling, average Vd/Id over SAMPLE window → Rs.
 *
 * Phase 2 — Ls (inductance):
 *   At θ=0, apply a voltage step (bypass PI, direct Vd command).
 *   Measure initial dI/dt: L = V_step / (dI/dt).
 *   Multiple pulses averaged for noise rejection.
 *
 * Phase 2.5 — Re-align:
 *   Re-apply Id at θ=0 to lock rotor (may have drifted during Ls pulses).
 *   Pure Id only — d→q transition happens in Phase 3 while θ advances.
 *
 * Phase 3 — λ_pm (flux linkage):
 *   Spin motor with I/f at known ω, PI holds Id=0, Iq=test current.
 *   At steady state: Vq = Rs*Iq + ω*λ_pm  →  λ_pm = (Vq - Rs*Iq) / ω.
 *
 * Phase 4 — Auto-tune:
 *   Kp_dq = ωbw * Ls,  Ki_dq = ωbw * Rs  (1 kHz BW, pole cancellation).
 *
 * Component: FOC V2
 */

#include "foc_v2_detect.h"
#include "foc_v2_math.h"
#include "foc_v2_pi.h"
#include "../garuda_foc_params.h"

/* ── Detect configuration ─────────────────────────────────────── */

/* Rs timing (24 kHz ticks) */
#define RS_RAMP_TICKS       12000U  /* 500ms: very slow ramp to avoid transient */
#define RS_SETTLE_TICKS     4800U   /* 200ms: let PI settle */
#define RS_SAMPLE_TICKS     12000U  /* 500ms: accumulate Vd/Id */

/* Ls timing */
#define LS_SETTLE_TICKS     2400U   /* 100ms: PI holds Id=0 first */
#define LS_STEP_TICKS       6U      /* 0.25ms: early inductive region only */
#define LS_COAST_TICKS      960U    /* 40ms: let current fully decay + U25B recover */
#define LS_NUM_PULSES       10U     /* Average over N pulses */

/* Re-alignment between Ls and Lambda phases */
#define REALIGN_TICKS       4800U   /* 200ms: lock rotor at θ=0 before spinning */

/* d→q transition at start of Lambda (WHILE θ is advancing).
 * Must NOT happen at static θ — commanding Iq at θ=0 creates a static
 * torque that pushes the rotor ~90° ahead of the reference angle,
 * causing all BEMF to appear in Vd instead of Vq → λ_measured ≈ 0. */
#define LAMBDA_DQ_TICKS     2880U   /* 120ms: smooth d→q while spinning */

/* Lambda timing */
#define LAMBDA_RAMP_TICKS   60000U  /* 2.5s: I/f spin-up (generous for low-speed+prop) */
#define LAMBDA_SETTLE_TICKS 12000U  /* 0.5s: speed settle */
#define LAMBDA_SAMPLE_TICKS 12000U  /* 0.5s: accumulate Vq/Iq/ω */

/* Auto-tune */
#define TUNE_CURRENT_BW_HZ  1000.0f
#define TUNE_CURRENT_BW_RAD (2.0f * FOC_PI_F * TUNE_CURRENT_BW_HZ)

/* Sanity limits */
#define RS_MIN  0.005f    /* 5 mohm */
#define RS_MAX  50.0f     /* 50 ohm */
#define LS_MIN  1e-6f     /* 1 uH */
#define LS_MAX  0.1f      /* 100 mH */
#define KE_MIN  1e-5f     /* Tiny motor */
#define KE_MAX  0.5f      /* Large motor */

#define DT_FAST FOC_TS_FAST_S

/* ── Internal sub-phase tracking ──────────────────────────────── */

/* Within each DETECT_x state, detect_ctr counts ticks.
 * detect_v_accum / detect_i_accum accumulate measurements.
 * detect_samples counts valid samples. */

/* Ls pulse state machine (within DETECT_L) */
typedef enum {
    LS_PHASE_SETTLE = 0,
    LS_PHASE_STEP,
    LS_PHASE_COAST
} ls_phase_t;

/* File-scope Ls pulse tracking */
static uint16_t ls_pulse_count;
static ls_phase_t ls_phase;
static uint32_t ls_phase_ctr;
static float ls_i_start;       /* Current at start of voltage step */
static float ls_step_voltage;  /* Adaptive step voltage based on measured Rs */
static float lambda_test_speed; /* Adaptive test speed for lambda phase */

/* ── Adaptive detect parameters (computed from loaded profile) ── */
static float detect_test_current;  /* Rs test current (A) */
static float detect_ls_peak_a;    /* Ls pulse target peak (A) */
static float detect_lambda_iq;    /* Lambda spin Iq (A) */
static float detect_lambda_speed; /* Lambda test speed (rad/s elec) */
static float detect_lambda_ramp;  /* Lambda ramp rate (rad/s²) */
static float detect_profile_oc;   /* Profile OC limit — floor for post-detect */
static float detect_profile_rs;   /* Profile Rs — used for lambda subtraction
                                   * instead of measured Rs, because deadtime
                                   * distortion inflates measured Rs on low-R motors */

void foc_detect_start(FOC_State_t *foc)
{
    foc->detect_state   = DETECT_R;
    foc->detect_ctr     = 0;
    foc->detect_v_accum = 0.0f;
    foc->detect_i_accum = 0.0f;
    foc->detect_samples = 0;
    foc->detect_Rs      = 0.0f;
    foc->detect_Ls      = 0.0f;
    foc->detect_lambda  = 0.0f;
    foc->mode           = FOC_MOTOR_DETECT;
    foc->sub_state      = 1;  /* Telemetry: detect active */

    /* Reset I/f state for lambda phase */
    foc->theta_if = 0.0f;
    foc->omega_if = 0.0f;

    /* ── Compute adaptive detect parameters from loaded profile ── */
    /* The profile Rs/Ke give us a hint about the motor class.
     * Low-Rs motors (drone: 50-200mΩ) need higher test current for
     * Rs measurement accuracy (Vd must exceed deadtime distortion).
     * High-KV motors (small Ke) need faster lambda spin for measurable BEMF. */

    float profile_rs = foc->Rs;
    float profile_ke = foc->Ke;

    /* Rs test current: target ≥ 0.5V across winding for measurability.
     * I = 0.5V / Rs, clamped [0.5A, 3A].  At 65mΩ: 7.7A → clamped 3A.
     * At 500mΩ: 1A.  At 2Ω: 0.25A → clamped 0.5A.
     * Cap at 3A (not 5A) to avoid U25B hardware OC trip on MCLV board
     * (no LEB on the comparator — switching transients at high current
     * cause latching fault even though RMS current is well within spec). */
    if (profile_rs > 0.01f) {
        detect_test_current = 0.5f / profile_rs;
    } else {
        detect_test_current = 2.0f;  /* Unknown motor — conservative default */
    }
    detect_test_current = foc_clampf(detect_test_current, 0.5f, 3.0f);

    /* Ls pulse peak: same as Rs test current (symmetric) */
    detect_ls_peak_a = detect_test_current;

    /* Lambda spin Iq: use full test current for maximum synchronization
     * torque.  Low-Ke motors (A2212: 0.000563 V·s/rad) produce very
     * little torque per amp (T = Ke×pp×Iq), so we need every amp
     * available to prevent desync during I/f spin.
     * Clamped [0.5A, 3A]. */
    detect_lambda_iq = detect_test_current;
    detect_lambda_iq = foc_clampf(detect_lambda_iq, 0.5f, 3.0f);

    /* Lambda test speed: need BEMF ≫ deadtime distortion for accuracy.
     * Low-Rs motors have large deadtime error (~0.3V), so we need enough
     * BEMF to dominate.  But speed is limited by I/f sync torque —
     * high-KV motors with props can't follow fast forced commutation.
     *
     * Compromise: cap at 500 rad/s (682 mech RPM for 7PP).
     * At this speed, even with prop, the drag is negligible and 3A Iq
     * provides enough sync torque (0.012 N·m >> prop drag).
     *
     * BEMF accuracy depends on using profile_rs (not measured) for the
     * lambda formula, so the 10×Rs/Ke formula is less critical.
     *   Hurst: min(676, 500) = 500 rad/s → BEMF 3.71V (excellent)
     *   A2212: min(3462, 500) = 500 rad/s → BEMF 0.28V (OK with profile Rs) */
    if (profile_ke > KE_MIN) {
        detect_lambda_speed = 10.0f * profile_rs * detect_lambda_iq / profile_ke;
    } else {
        detect_lambda_speed = 300.0f;
    }
    detect_lambda_speed = foc_clampf(detect_lambda_speed, 150.0f, 500.0f);

    /* Lambda ramp rate: reach test speed in ~2s (gentle for prop loads).
     * rate = speed / 2.0, clamped [50, 500] */
    detect_lambda_ramp = detect_lambda_speed / 2.0f;
    detect_lambda_ramp = foc_clampf(detect_lambda_ramp, 50.0f, 500.0f);

    /* Save profile OC limit — post-detect OC must never be lower */
    detect_profile_oc = foc->fault_oc_a;

    /* Save profile Rs for lambda subtraction.  On low-Rs motors (A2212:
     * 65mΩ), the 500ns deadtime creates ~0.29V of distortion that inflates
     * measured Rs by ~50-80mΩ.  Using the deadtime-corrupted Rs to subtract
     * from Vq eats the BEMF signal.  Using the known profile Rs gives a
     * much closer lambda estimate. */
    detect_profile_rs = profile_rs;

    /* Init PI for detect — use profile-aware gains.
     * Vclamp = Rs * test_current * 3 (headroom for transients).
     * Minimum 1.5V for low-Rs motors. */
    float vclamp = profile_rs * detect_test_current * 3.0f;
    if (vclamp < 1.5f) vclamp = 1.5f;
    if (vclamp > foc->vbus * 0.3f) vclamp = foc->vbus * 0.3f;

    /* PI gains: Kp from L/R dynamics, Ki for steady-state.
     * For unknown motors, use conservative soft gains. */
    float kp_detect, ki_detect;
    if (foc->Ls > LS_MIN && profile_rs > RS_MIN) {
        /* BW = 200 Hz (conservative during detect) */
        float bw = 2.0f * FOC_PI_F * 200.0f;
        kp_detect = foc->Ls * bw;
        ki_detect = profile_rs * bw;
    } else {
        kp_detect = 0.05f;
        ki_detect = 50.0f;
    }
    foc_pi_init(&foc->pid_d, kp_detect, ki_detect, -vclamp, vclamp);
    foc_pi_init(&foc->pid_q, kp_detect, ki_detect, -vclamp, vclamp);

    /* Reset Ls pulse state */
    ls_pulse_count = 0;
    ls_phase = LS_PHASE_SETTLE;
    ls_phase_ctr = 0;
    ls_i_start = 0.0f;
}

bool foc_detect_fast_tick(FOC_State_t *foc,
                          float ia, float ib,
                          float i_alpha, float i_beta,
                          float *da_out, float *db_out, float *dc_out)
{
    foc->detect_ctr++;

    /* Default: safe center duty */
    *da_out = 0.5f;
    *db_out = 0.5f;
    *dc_out = 0.5f;

    switch (foc->detect_state) {

    /* ════════════════════════════════════════════════════════════
     * PHASE 1: Rs — Command Id at θ=0, measure Vd/Id at steady state
     * ════════════════════════════════════════════════════════════ */
    case DETECT_R: {
        float sin_t = 0.0f;  /* sin(0) */
        float cos_t = 1.0f;  /* cos(0) */

        /* Park transform at θ=0 */
        float id_meas, iq_meas;
        foc_park(i_alpha, i_beta, sin_t, cos_t, &id_meas, &iq_meas);

        /* Ramp Id reference */
        float id_ref;
        if (foc->detect_ctr <= RS_RAMP_TICKS) {
            float frac = (float)foc->detect_ctr / (float)RS_RAMP_TICKS;
            id_ref = detect_test_current * frac;
        } else {
            id_ref = detect_test_current;
        }

        /* PI current control */
        float vd = foc_pi_run(&foc->pid_d, id_ref - id_meas, DT_FAST);
        float vq = foc_pi_run(&foc->pid_q, 0.0f - iq_meas, DT_FAST);

        /* Sampling phase: after ramp + settle */
        uint32_t sample_start = RS_RAMP_TICKS + RS_SETTLE_TICKS;
        uint32_t sample_end   = sample_start + RS_SAMPLE_TICKS;

        if (foc->detect_ctr > sample_start && foc->detect_ctr <= sample_end) {
            /* Accumulate Vd and Id for averaging.
             * At θ=0 steady state with no rotation: Vd = Rs * Id */
            foc->detect_v_accum += vd;
            foc->detect_i_accum += id_meas;
            foc->detect_samples++;
        }

        /* Inverse Park + SVPWM */
        float v_alpha, v_beta;
        foc_inv_park(vd, vq, sin_t, cos_t, &v_alpha, &v_beta);
        foc_svpwm(v_alpha, v_beta, foc->vbus, da_out, db_out, dc_out);

        /* Telemetry */
        foc->id_meas = id_meas;
        foc->iq_meas = iq_meas;
        foc->vd = vd;
        foc->vq = vq;

        /* Phase complete? */
        if (foc->detect_ctr > sample_end) {
            if (foc->detect_samples > 0 && foc->detect_i_accum > 0.1f) {
                foc->detect_Rs = foc->detect_v_accum / foc->detect_i_accum;
                foc->detect_Rs = foc_clampf(foc->detect_Rs, RS_MIN, RS_MAX);
            } else {
                foc->detect_state = DETECT_FAIL;
                return true;
            }

            /* Transition to Ls */
            foc->detect_state   = DETECT_L;
            foc->detect_ctr     = 0;
            foc->detect_v_accum = 0.0f;
            foc->detect_i_accum = 0.0f;
            foc->detect_samples = 0;
            foc->sub_state      = 2;

            /* Compute adaptive Ls step voltage: V = Rs * Ipeak.
             * Targets detect_ls_peak_a steady-state current.
             * Clamp [0.1V, 3V] for safety. */
            ls_step_voltage = foc->detect_Rs * detect_ls_peak_a;
            ls_step_voltage = foc_clampf(ls_step_voltage, 0.1f, 3.0f);

            /* Reset PI and Ls pulse state */
            foc_pi_reset(&foc->pid_d);
            foc_pi_reset(&foc->pid_q);
            ls_pulse_count = 0;
            ls_phase = LS_PHASE_SETTLE;
            ls_phase_ctr = 0;
        }
        break;
    }

    /* ════════════════════════════════════════════════════════════
     * PHASE 2: Ls — Voltage step, measure dI/dt
     * ════════════════════════════════════════════════════════════ */
    case DETECT_L: {
        float sin_t = 0.0f;
        float cos_t = 1.0f;

        float id_meas, iq_meas;
        foc_park(i_alpha, i_beta, sin_t, cos_t, &id_meas, &iq_meas);

        float vd = 0.0f;
        float vq = 0.0f;

        ls_phase_ctr++;

        switch (ls_phase) {
        case LS_PHASE_SETTLE:
            /* Let current decay to ~0 (PI holds Id=0) */
            vd = foc_pi_run(&foc->pid_d, 0.0f - id_meas, DT_FAST);
            vq = foc_pi_run(&foc->pid_q, 0.0f - iq_meas, DT_FAST);
            if (ls_phase_ctr >= (ls_pulse_count == 0 ? LS_SETTLE_TICKS : LS_COAST_TICKS)) {
                ls_phase = LS_PHASE_STEP;
                ls_phase_ctr = 0;
                ls_i_start = id_meas;
                foc_pi_reset(&foc->pid_d);
                foc_pi_reset(&foc->pid_q);
            }
            break;

        case LS_PHASE_STEP:
            /* Apply constant voltage step on d-axis, bypass PI.
             * L = V * dt / ΔI.  We measure ΔI over LS_STEP_TICKS. */
            vd = ls_step_voltage;
            vq = 0.0f;

            if (ls_phase_ctr >= LS_STEP_TICKS) {
                /* Measure current change */
                float di = id_meas - ls_i_start;
                if (di > 0.005f) {
                    /* L = (V - R*I_avg) * t / ΔI — corrected for R drop */
                    float t_step = (float)LS_STEP_TICKS * DT_FAST;
                    float i_avg = (ls_i_start + id_meas) * 0.5f;
                    float v_net = ls_step_voltage - foc->detect_Rs * i_avg;
                    if (v_net > 0.01f) {
                        float l_sample = v_net * t_step / di;
                        if (l_sample > LS_MIN && l_sample < LS_MAX) {
                            foc->detect_v_accum += l_sample;
                            foc->detect_samples++;
                        }
                    }
                }

                ls_pulse_count++;
                if (ls_pulse_count >= LS_NUM_PULSES) {
                    /* All pulses done — compute average Ls */
                    if (foc->detect_samples >= 2) {
                        foc->detect_Ls = foc->detect_v_accum / (float)foc->detect_samples;
                        foc->detect_Ls = foc_clampf(foc->detect_Ls, LS_MIN, LS_MAX);
                    } else {
                        foc->detect_state = DETECT_FAIL;
                        return true;
                    }

                    /* Lambda test speed: use default. */
                    lambda_test_speed = detect_lambda_speed;

                    /* Update PI gains with measured Rs, Ls for align + lambda.
                     * Vclamp = 0.45 × Vbus/√3 — leaves headroom vs linear limit. */
                    float vclamp = foc->vbus * 0.45f * FOC_INV_SQRT3;
                    float kp = foc->detect_Ls * TUNE_CURRENT_BW_RAD;
                    float ki = foc->detect_Rs * TUNE_CURRENT_BW_RAD;
                    foc_pi_init(&foc->pid_d, kp, ki, -vclamp, vclamp);
                    foc_pi_init(&foc->pid_q, kp, ki, -vclamp, vclamp);

                    /* Transition to re-alignment (NOT directly to lambda).
                     * After Ls pulses, rotor may have drifted from θ=0.
                     * Re-align firmly before spinning. */
                    foc->detect_state   = DETECT_ALIGN;
                    foc->detect_ctr     = 0;
                    foc->detect_v_accum = 0.0f;
                    foc->detect_i_accum = 0.0f;
                    foc->detect_samples = 0;
                    foc->sub_state      = 3;
                    foc->theta_if       = 0.0f;
                    foc->omega_if       = 0.0f;
                } else {
                    /* More pulses to go — coast and repeat */
                    ls_phase = LS_PHASE_SETTLE;
                    ls_phase_ctr = 0;
                }
            }
            break;

        case LS_PHASE_COAST:
            /* Handled by LS_PHASE_SETTLE with coast timing */
            break;
        }

        /* Inverse Park + SVPWM */
        float v_alpha, v_beta;
        foc_inv_park(vd, vq, sin_t, cos_t, &v_alpha, &v_beta);
        foc_svpwm(v_alpha, v_beta, foc->vbus, da_out, db_out, dc_out);

        foc->id_meas = id_meas;
        foc->iq_meas = iq_meas;
        foc->vd = vd;
        foc->vq = vq;
        break;
    }

    /* ════════════════════════════════════════════════════════════
     * PHASE 2.5: Re-align rotor at θ=0 before spinning.
     *
     * After Ls pulses, rotor may have drifted from θ=0 (no holding
     * current during coast periods).  Re-applying Id for 200ms
     * firmly locks the rotor position.  Then a 100ms smooth d→q
     * transition (like normal IF_RAMP) ensures the torque vector
     * rotates gradually — avoids the 90° current jump that could
     * cause rotor sync loss at lambda start.
     * ════════════════════════════════════════════════════════════ */
    case DETECT_ALIGN: {
        /* Pure Id alignment at θ=0 — NO d→q transition here.
         * The d→q transition must happen in DETECT_LAMBDA while θ
         * is advancing, otherwise Iq at static θ=0 pushes the rotor
         * 90° ahead, putting all BEMF into Vd instead of Vq. */
        float sin_t = 0.0f;
        float cos_t = 1.0f;

        float id_meas, iq_meas;
        foc_park(i_alpha, i_beta, sin_t, cos_t, &id_meas, &iq_meas);

        /* Ramp Id to detect_lambda_iq for firm alignment */
        float ramp = (float)foc->detect_ctr / (float)(REALIGN_TICKS / 2);
        if (ramp > 1.0f) ramp = 1.0f;
        float id_ref = detect_lambda_iq * ramp;

        float vd = foc_pi_run(&foc->pid_d, id_ref - id_meas, DT_FAST);
        float vq = foc_pi_run(&foc->pid_q, 0.0f - iq_meas, DT_FAST);

        float v_alpha, v_beta;
        foc_inv_park(vd, vq, sin_t, cos_t, &v_alpha, &v_beta);
        foc_svpwm(v_alpha, v_beta, foc->vbus, da_out, db_out, dc_out);

        foc->id_meas = id_meas;
        foc->iq_meas = iq_meas;
        foc->vd = vd;
        foc->vq = vq;

        /* Transition to lambda after alignment hold */
        if (foc->detect_ctr >= REALIGN_TICKS) {
            foc->detect_state   = DETECT_LAMBDA;
            foc->detect_ctr     = 0;
            foc->detect_v_accum = 0.0f;
            foc->detect_i_accum = 0.0f;
            foc->detect_samples = 0;
            foc->sub_state      = 4;
            foc->theta_if       = 0.0f;
            foc->omega_if       = 0.0f;
            /* Don't reset PI — Id integral helps bumpless d→q start */
        }
        break;
    }

    /* ════════════════════════════════════════════════════════════
     * PHASE 3: λ_pm — Spin with I/f, measure back-EMF from Vq
     * ════════════════════════════════════════════════════════════ */
    case DETECT_LAMBDA: {
        /* I/f ramp: accelerate to adaptive test speed */
        if (foc->omega_if < lambda_test_speed) {
            foc->omega_if += detect_lambda_ramp * DT_FAST;
            if (foc->omega_if > lambda_test_speed)
                foc->omega_if = lambda_test_speed;
        }

        /* Advance forced angle — MUST start from tick 1 */
        foc->theta_if += foc->omega_if * DT_FAST;
        foc->theta_if = foc_angle_wrap(foc->theta_if);

        float sin_t = sinf(foc->theta_if);
        float cos_t = cosf(foc->theta_if);

        /* Park transform */
        float id_meas, iq_meas;
        foc_park(i_alpha, i_beta, sin_t, cos_t, &id_meas, &iq_meas);

        /* d→q transition WHILE θ is advancing (like normal IF_RAMP).
         * This is critical: transitioning to Iq at static θ creates a
         * static torque that pushes the rotor ~90° ahead, causing BEMF
         * to appear in Vd instead of Vq → λ_measured ≈ 0.
         * By advancing θ simultaneously, the rotating field drags the
         * rotor along smoothly, maintaining synchronization. */
        float id_ref, iq_ref;
        if (foc->detect_ctr <= LAMBDA_DQ_TICKS) {
            float frac = (float)foc->detect_ctr / (float)LAMBDA_DQ_TICKS;
            id_ref = detect_lambda_iq * (1.0f - frac);
            iq_ref = detect_lambda_iq * frac;
        } else {
            id_ref = 0.0f;
            iq_ref = detect_lambda_iq;
        }

        float vd = foc_pi_run(&foc->pid_d, id_ref - id_meas, DT_FAST);
        float vq = foc_pi_run(&foc->pid_q, iq_ref - iq_meas, DT_FAST);

        /* Cross-coupling feedforward (uses measured Rs, Ls) */
        float ff_d = -foc->omega_if * foc->detect_Ls * iq_meas;
        float ff_q =  foc->omega_if * foc->detect_Ls * id_meas;
        float vd_tot = vd + ff_d;
        float vq_tot = vq + ff_q;

        /* Sampling: after d→q transition + speed ramp + settle */
        uint32_t sample_start = LAMBDA_DQ_TICKS + LAMBDA_RAMP_TICKS + LAMBDA_SETTLE_TICKS;
        uint32_t sample_end   = sample_start + LAMBDA_SAMPLE_TICKS;

        if (foc->detect_ctr > sample_start && foc->detect_ctr <= sample_end) {
            /* Accumulate total Vq and Iq.
             * At steady state with Id≈0:
             *   Vq_total = Rs*Iq + ω*λ_pm
             *   λ_pm = (Vq_total - Rs*Iq) / ω */
            foc->detect_v_accum += vq_tot;
            foc->detect_i_accum += iq_meas;
            foc->detect_samples++;
        }

        /* Inverse Park + SVPWM */
        float v_alpha, v_beta;
        foc_inv_park(vd_tot, vq_tot, sin_t, cos_t, &v_alpha, &v_beta);
        foc_svpwm(v_alpha, v_beta, foc->vbus, da_out, db_out, dc_out);

        foc->id_meas = id_meas;
        foc->iq_meas = iq_meas;
        foc->vd = vd_tot;
        foc->vq = vq_tot;
        foc->omega = foc->omega_if;

        /* Phase complete? */
        if (foc->detect_ctr > sample_end) {
            if (foc->detect_samples > 0 && foc->omega_if > 10.0f) {
                float avg_vq = foc->detect_v_accum / (float)foc->detect_samples;
                float avg_iq = foc->detect_i_accum / (float)foc->detect_samples;

                /* Use profile Rs (not measured) for subtraction.
                 * Measured Rs includes deadtime distortion (~50-80mΩ
                 * extra on low-R motors), which eats the BEMF signal
                 * and drives lambda toward zero. */
                foc->detect_lambda = (avg_vq - detect_profile_rs * avg_iq)
                                   / foc->omega_if;
                foc->detect_lambda = foc_clampf(foc->detect_lambda, KE_MIN, KE_MAX);
            } else {
                foc->detect_state = DETECT_FAIL;
                return true;
            }

            /* Transition to auto-tune */
            foc->detect_state = DETECT_TUNE;
            foc->detect_ctr   = 0;
            foc->sub_state    = 5;
        }
        break;
    }

    /* ════════════════════════════════════════════════════════════
     * PHASE 4: Auto-tune — Compute PI gains, apply all params
     * ════════════════════════════════════════════════════════════ */
    case DETECT_TUNE: {
        /* Single-tick computation */
        foc->detect_state = DETECT_DONE;
        foc->sub_state    = 6;
        return true;
    }

    case DETECT_DONE:
    case DETECT_FAIL:
        return true;

    default:
        foc->detect_state = DETECT_FAIL;
        return true;
    }

    return false;
}

void foc_detect_apply(FOC_State_t *foc)
{
    if (foc->detect_state != DETECT_DONE) return;

    /* Apply measured motor params */
    foc->Rs        = foc->detect_Rs;
    foc->Ls        = foc->detect_Ls;
    foc->lambda_pm = foc->detect_lambda;
    foc->Ke        = foc->detect_lambda;

    /* Compute and apply PI gains: pole cancellation at 1 kHz BW */
    float kp_dq = foc->detect_Ls * TUNE_CURRENT_BW_RAD;
    float ki_dq = foc->detect_Rs * TUNE_CURRENT_BW_RAD;
    float vclamp = foc->vbus * 0.95f * FOC_INV_SQRT3;
    foc_pi_init(&foc->pid_d, kp_dq, ki_dq, -vclamp, vclamp);
    foc_pi_init(&foc->pid_q, kp_dq, ki_dq, -vclamp, vclamp);

    /* Update observer initial lambda */
    foc->obs.lambda_est = foc->detect_lambda;

    /* Compute max electrical speed from Vbus and Ke */
    if (foc->detect_lambda > KE_MIN) {
        float omega_max = foc->vbus * 0.85f * FOC_INV_SQRT3 / foc->detect_lambda;
        foc->max_elec_rad_s = omega_max;
        foc->pll.omega_max  = omega_max;
    }

    /* Set reasonable startup params based on detected motor */
    foc->handoff_rad_s = lambda_test_speed;  /* Proven spin speed */
    foc->ramp_rate     = detect_lambda_ramp; /* Proven ramp rate */
    foc->align_iq      = detect_test_current;
    foc->ramp_iq       = detect_lambda_iq;

    /* Overcurrent limit: max of (4× detect current, profile OC limit).
     * Never reduce below the profile's known-safe OC threshold.
     * A2212: max(4×3=12, 25) = 25A.  Hurst: max(4×0.94=3.76, 10) = 10A. */
    float detect_oc = detect_test_current * 4.0f;
    if (detect_oc < detect_lambda_iq * 4.0f)
        detect_oc = detect_lambda_iq * 4.0f;
    foc->fault_oc_a = (detect_oc > detect_profile_oc)
                    ? detect_oc : detect_profile_oc;
}
