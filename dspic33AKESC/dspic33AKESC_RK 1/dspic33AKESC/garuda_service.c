/**
 * @file garuda_service.c
 *
 * @brief ESC FOC state machine and ADC ISR.
 *
 * State machine (driven from ADC ISR at 24 kHz):
 *   IDLE → ARMED (throttle=0 for ARM_TIME_MS) → STARTUP (open-loop I/F ramp)
 *   → ESC_RUNNING (FOC closed-loop via PLL)
 *
 * Timer1 ISR: 100us tick for heartbeat LED, board service, 1ms system tick.
 *
 * Component: GARUDA SERVICE
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>   /* fabsf — mapped to hardware FPU */

#include "garuda_service.h"
#include "garuda_types.h"
#include "garuda_config.h"
#include "garuda_calc_params.h"
#include "garuda_foc_params.h"
#include "hal/hal_adc.h"
#include "hal/hal_pwm.h"
#include "hal/board_service.h"
#include "hal/port_config.h"
#include "foc/clarke.h"
#include "foc/park.h"
#include "foc/svpwm.h"
#include "foc/pi_controller.h"
#include "foc/back_emf_obs.h"
#include "foc/pll_estimator.h"
#include "foc/flux_estimator.h"
#if FEATURE_SMO
#include "foc/smo_observer.h"
#endif

#include "x2cscope/diagnostics.h"

#if FEATURE_LEARN_MODULES
#include "learn/learn_service.h"
#endif

/* Global ESC runtime data — volatile: shared between ISRs and main loop */
volatile GARUDA_DATA_T garudaData;

/* ── Module-level FOC state (accessed only from ADC ISR) ─────────────── */
static PI_t         s_pid_d;        /* D-axis current PI */
static PI_t         s_pid_q;        /* Q-axis current PI */
static PI_t         s_pid_spd;      /* Speed PI (outer loop) */
static BackEMFObs_t s_obs;          /* Back-EMF voltage-model observer */
static PLL_t        s_pll;          /* PLL angle/speed estimator */
static FluxEst_t    s_flux_est;     /* Flux-integration angle estimator (parallel) */
#if FEATURE_SMO
static SMO_t        s_smo;          /* Sliding Mode Observer (parallel) */
static PLL_t        s_pll_smo;      /* PLL driven by SMO back-EMF estimates */
#endif

static float        s_iq_ref;       /* Q-axis current reference (A) */
static float        s_vbus;         /* Last measured bus voltage (V) */
static uint16_t     s_slow_ctr;     /* Slow loop counter (0 → FOC_SLOW_DIV) */

/* Open-loop startup ramp state */
static float        s_theta_ol;     /* Open-loop angle (rad) */
static float        s_omega_ol;     /* Open-loop speed (rad/s elec.) */
static bool         s_ovr_released; /* True after overrides released this run */
static uint32_t     s_iq_ramp_ctr;  /* Ticks elapsed since STARTUP entry (Iq ramp) */
static uint32_t     s_align_ctr;    /* Ticks elapsed during alignment dwell */

/* PLL angle correction gain (per fast-loop tick): 2π × BW × Ts.
 * At 2 Hz BW: ≈0.000524/tick → imperceptible per-tick angle nudge. */
#define PLL_CORRECTION_GAIN  (6.28318530718f * PLL_CORRECTION_BW_HZ * FOC_TS_FAST_S)

/* ADC zero-current offset calibration (sampled while motor is off) */
#define CAL_SAMPLES  1024U          /* Power of 2 for shift division */
#define CAL_SHIFT    10U            /* log2(CAL_SAMPLES) */
static uint32_t     s_ia_accum;     /* Running sum of raw Ia during calibration */
static uint32_t     s_ib_accum;     /* Running sum of raw Ib during calibration */
static uint16_t     s_cal_count;    /* Samples collected so far */
static uint16_t     s_ia_offset;    /* Calibrated zero-current offset for Ia */
static uint16_t     s_ib_offset;    /* Calibrated zero-current offset for Ib */
static bool         s_cal_done;     /* true after calibration complete */

/* Stall timer (slow loop ticks) */
static uint32_t     s_stall_ctr;

/* Heartbeat / tick sub-counters (Timer1 ISR) */
static uint16_t heartbeatCounter = 0;
static uint8_t  msSubCounter = 0;

/* Helper: get PLL rotor angle (corrected for BEMF→rotor offset) */
static inline float pll_rotor_angle(void)
{
    float a = s_pll.theta_est - PLL_ANGLE_OFFSET;
    if (a < 0.0f)            a += 6.28318530718f;
    if (a >= 6.28318530718f) a -= 6.28318530718f;
    return a;
}

#if FEATURE_SMO
/* Helper: get SMO PLL rotor angle (same offset correction) */
static inline float smo_rotor_angle(void)
{
    float a = s_pll_smo.theta_est - PLL_ANGLE_OFFSET;
    if (a < 0.0f)            a += 6.28318530718f;
    if (a >= 6.28318530718f) a -= 6.28318530718f;
    return a;
}
#endif

/* ── Forward declarations ────────────────────────────────────────────── */
static void startup_reset(void);
static float counts_to_vbus(uint16_t raw);

/* =========================================================================
 * GARUDA_ServiceInit
 * ========================================================================= */
void GARUDA_ServiceInit(void)
{
    /* Core state */
    garudaData.state            = ESC_IDLE;
    garudaData.faultCode        = FAULT_NONE;
    garudaData.runCommandActive = false;
    garudaData.systemTick       = 0;
    garudaData.armCounter       = 0;
    garudaData.desyncRestartAttempts = 0;

    /* Legacy fields kept for GSP snapshot */
    garudaData.throttle     = 0;
    garudaData.currentStep  = 0;
    garudaData.direction    = 0;
    garudaData.duty         = 0;
    garudaData.vbusRaw      = 0;
    garudaData.potRaw       = 0;

    /* FOC telemetry */
    garudaData.id_a       = 0.0f;
    garudaData.iq_a       = 0.0f;
    garudaData.theta_rad  = 0.0f;
    garudaData.omega_rad_s = 0.0f;
    garudaData.vbus_v     = 0.0f;
#if FEATURE_SMO
    garudaData.smo_theta_rad  = 0.0f;
    garudaData.smo_omega_rad_s = 0.0f;
#endif
    garudaData.pot_raw    = 0;

    /* FOC algorithm state */
    pi_init(&s_pid_d,   KP_DQ,  KI_DQ,  -CLAMP_VDQ,       CLAMP_VDQ);
    pi_init(&s_pid_q,   KP_DQ,  KI_DQ,  -CLAMP_VDQ,       CLAMP_VDQ);
    pi_init(&s_pid_spd, KP_SPD, KI_SPD, -CLAMP_IQ_REF_A,  CLAMP_IQ_REF_A);

    bemf_obs_reset(&s_obs);
    pll_reset(&s_pll);
    flux_est_reset(&s_flux_est);
#if FEATURE_SMO
    smo_reset(&s_smo);
    pll_reset(&s_pll_smo);
#endif
    startup_reset();

    s_iq_ref   = 0.0f;
    s_vbus     = MOTOR_VBUS_NOM_V;
    s_slow_ctr = 0;
    s_stall_ctr = 0;

    /* ADC offset calibration — will be sampled during IDLE before first run */
    s_ia_accum  = 0;
    s_ib_accum  = 0;
    s_cal_count = 0;
    s_ia_offset = ADC_MIDPOINT;  /* fallback until calibration completes */
    s_ib_offset = ADC_MIDPOINT;
    s_cal_done  = false;

#if FEATURE_GSP
    garudaData.throttleSource    = THROTTLE_SRC_ADC;
    garudaData.gspThrottle       = 0;
    garudaData.lastGspPacketTick = 0;
    garudaData.gspStartIntent    = false;
    garudaData.gspStopIntent     = false;
    garudaData.gspFaultClearIntent = false;
#endif

    /* Enable ADC interrupt to start the control loop */
    GARUDA_ClearADCIF();
    GARUDA_EnableADCInterrupt();

#if FEATURE_LEARN_MODULES
    LEARN_ServiceInit(&garudaData);
#endif
}

/* =========================================================================
 * Helper: reset open-loop startup state
 * ========================================================================= */
static void startup_reset(void)
{
    s_theta_ol     = 0.0f;
    s_omega_ol     = 0.0f;
    s_ovr_released = false;
    s_iq_ramp_ctr  = 0;
    s_align_ctr    = 0;
    pi_reset(&s_pid_d);
    pi_reset(&s_pid_q);
    pi_reset(&s_pid_spd);
    s_iq_ref    = 0.0f;
    s_stall_ctr = 0;
    flux_est_reset(&s_flux_est);
#if FEATURE_SMO
    smo_reset(&s_smo);
    pll_reset(&s_pll_smo);
#endif
}

/* =========================================================================
 * Helper: convert ADC count to phase current (A)
 * Uses calibrated zero-current offset instead of fixed ADC_MIDPOINT.
 * ========================================================================= */
static inline float counts_to_amps_cal(uint16_t raw, uint16_t offset)
{
    return ((float)(int16_t)(raw - offset)) * CURRENT_SCALE_A_PER_COUNT;
}

/* =========================================================================
 * Helper: convert ADC count to bus voltage (V)
 * Assumes 68k/4.7k divider: VBUS_SCALE_V_PER_COUNT.
 * ========================================================================= */
static float counts_to_vbus(uint16_t raw)
{
    return (float)raw * VBUS_SCALE_V_PER_COUNT;
}

/* =========================================================================
 * ADC ISR — FOC fast loop at 24 kHz
 * Triggered by AD1CH0 (Phase A current) completion from PWM1 trigger.
 * ========================================================================= */
float I_alpha, I_beta; 
void __attribute__((__interrupt__, no_auto_psv)) GARUDA_ADC_INTERRUPT(void)
{
    /* ── 1. Read ADC results ─────────────────────────────────────────────── */
    uint16_t raw_ia   = ADCBUF_IA; 
    uint16_t raw_ib   = ADCBUF_IB; 
    /* raw_ic not needed: Kirchhoff gives Ic = -(Ia+Ib) */
    uint16_t raw_vbus = ADCBUF_VBUS;
    uint16_t raw_pot  = ADCBUF_POT;

    /* ── 2. Convert ADC counts → physical units (calibrated offsets) ────── */
    float ia    = counts_to_amps_cal(raw_ia, s_ia_offset);
    float ib    = counts_to_amps_cal(raw_ib, s_ib_offset);
    s_vbus      = counts_to_vbus(raw_vbus);
    
    I_alpha=ia;
    I_beta=ib;

    /* ── 3. Dynamic PI voltage clamp — tracks actual bus voltage ─────────── *
     * Prevents integrator windup when Vbus < MOTOR_VBUS_NOM_V (e.g. 12V supply).
     * Without this, PI commands up to 22.8V → SVPWM saturates → uncontrolled
     * current flows through Rs (Vbus/2Rs ≈ 3A at 12V). */
    {
        float vclamp = s_vbus * 0.95f;
        s_pid_d.out_max =  vclamp;
        s_pid_d.out_min = -vclamp;
        s_pid_q.out_max =  vclamp;
        s_pid_q.out_min = -vclamp;
    }

    /* ── 4. Update shared telemetry ──────────────────────────────────────── */
    garudaData.pot_raw  = raw_pot;
    garudaData.potRaw   = raw_pot;   /* legacy */
    garudaData.throttle = raw_pot;   /* legacy — GSP reads throttle field */
    garudaData.vbusRaw  = raw_vbus;  /* legacy */
    garudaData.vbus_v   = s_vbus;

    /* Throttle source mux */
    uint16_t throttle_raw = raw_pot;
#if FEATURE_GSP
    if (garudaData.throttleSource == THROTTLE_SRC_GSP) {
        throttle_raw = (uint16_t)((uint32_t)garudaData.gspThrottle * 4095U / 2000U);
    }
#endif

    /* ── 4. State machine ────────────────────────────────────────────────── */
    ESC_STATE_T state = garudaData.state;

    if (state == ESC_IDLE || state == ESC_ARMED)
    {
        /* Outputs off — overrides already asserted by InitDutyPWM123Generators */
        s_ovr_released = false;

        /* ADC zero-current offset calibration (motor off → no current) */
        if (!s_cal_done) {
            s_ia_accum += raw_ia;
            s_ib_accum += raw_ib;
            if (++s_cal_count >= CAL_SAMPLES) {
                s_ia_offset = (uint16_t)(s_ia_accum >> CAL_SHIFT);
                s_ib_offset = (uint16_t)(s_ib_accum >> CAL_SHIFT);
                s_cal_done  = true;
            }
        }

        goto isr_slow_loop;
    }
    else if (state == ESC_STARTUP)
    {
        /* ── Open-loop startup ──────────────────────────────────────────── */

        /* Release PWM overrides once at STARTUP entry */
        if (!s_ovr_released) {
            HAL_PWM_ReleaseAllOverrides();
            s_ovr_released = true;
        }

        /* Alignment / pot-controlled speed */
        bool in_align = (s_align_ctr < STARTUP_ALIGN_TICKS);

        if (in_align) {
            /* Phase 1: Alignment dwell at θ=0 */
            s_align_ctr++;
            s_theta_ol = 0.0f;
            s_omega_ol = 0.0f;
        } else {
            /* Phase 2: POT controls 0 → full RPM, slew-rate limited. */
            float pot_target;
            if (throttle_raw < THROTTLE_DEADBAND) {
                pot_target = 0.0f;
            } else {
                float pot_frac = (float)(throttle_raw - THROTTLE_DEADBAND)
                               / (4095.0f - (float)THROTTLE_DEADBAND);
                pot_target = pot_frac * STARTUP_MAX_OL_RAD_S;
            }
            float max_delta = STARTUP_RAMP_RATE_RPS2 * FOC_TS_FAST_S;
            if (s_omega_ol < pot_target) {
                s_omega_ol += max_delta;
                if (s_omega_ol > pot_target) s_omega_ol = pot_target;
            } else if (s_omega_ol > pot_target) {
                s_omega_ol -= max_delta;
                if (s_omega_ol < pot_target) s_omega_ol = pot_target;
            }

            /* Advance open-loop angle */
            s_theta_ol += s_omega_ol * FOC_TS_FAST_S;
            if (s_theta_ol >= 6.28318530718f) s_theta_ol -= 6.28318530718f;

#if FEATURE_CLOSED_LOOP
            /* PLL angle correction: gently nudge OL angle toward PLL.
             * Active only above handoff speed where BEMF gives PLL signal.
             * Gain ≈ 0.0005/tick at 2Hz BW — imperceptible per tick.
             * Motor always drives from s_theta_ol, so there is NO sudden
             * angle switch, blend, or state change → no vibration. */
            if (s_omega_ol >= STARTUP_HANDOFF_RAD_S) {
                float pll_rotor = pll_rotor_angle();
                float err = pll_rotor - s_theta_ol;
                if (err >  3.14159265f) err -= 6.28318530718f;
                if (err < -3.14159265f) err += 6.28318530718f;
                s_theta_ol += PLL_CORRECTION_GAIN * err;
                if (s_theta_ol < 0.0f)            s_theta_ol += 6.28318530718f;
                if (s_theta_ol >= 6.28318530718f) s_theta_ol -= 6.28318530718f;
            }
#endif
        }

        /* Drive angle: always OL (PLL-corrected above threshold) */
        float theta = s_theta_ol;

        AlphaBeta_t vab;
        float da, db, dc;

#if STARTUP_VOLTAGE_MODE
        /* ── Voltage-mode V/f open-loop (debug) ─────────────────────── *
         * Bypasses PI controllers entirely. Directly commands Vd=0, Vq=V.
         * Tests: SVPWM sector mapping, PWM output wiring, motor rotation. */
        float vq_cmd;
        if (in_align) {
            /* Ramp voltage from 0 → ALIGN_V during alignment */
            float align_frac = (float)s_align_ctr / (float)STARTUP_ALIGN_TICKS;
            vq_cmd = STARTUP_ALIGN_V * align_frac;
        } else {
            /* V/f based on motor Ke: V = boost + Ke × ω.
             * Matches back-EMF exactly; only the fixed boost (STARTUP_ALIGN_V)
             * drives current for torque.  Eliminates excess current at speed. */
            vq_cmd = STARTUP_ALIGN_V + MOTOR_KE_VPEAK * s_omega_ol;
            float vq_max = s_vbus * 0.95f;  /* dynamic clamp at actual Vbus */
            if (vq_cmd > vq_max) vq_cmd = vq_max;
        }

        DQ_t vdq;
        vdq.d = 0.0f;
        vdq.q = vq_cmd;
        inv_park_transform(&vdq, theta, &vab);
        svpwm_update(vab.alpha, vab.beta, s_vbus, &da, &db, &dc);
        HAL_PWM_SetDutyFloat3Phase(da, db, dc);

        /* Telemetry — voltage mode: report actual phase currents */
        garudaData.id_a       = ia;      /* Phase A current (A) */
        garudaData.iq_a       = ib;      /* Phase B current (A) */

#else
        /* ── Current-mode I/F open-loop ─────────────────────────────── *
         * PI controllers regulate Id=0, Iq=commanded.  Current is actively
         * limited — no excess current from V/f mismatch or load angle. */
        AlphaBeta_t iab;
        clarke_transform(ia, ib, &iab);
        DQ_t idq;
        park_transform(&iab, theta, &idq);

        float iq_cmd;
        if (in_align) {
            /* Gentle ramp 0 → ALIGN_IQ during alignment */
            if (s_iq_ramp_ctr < STARTUP_IQ_RAMP_TICKS) {
                iq_cmd = STARTUP_ALIGN_IQ_A
                       * ((float)s_iq_ramp_ctr / (float)STARTUP_IQ_RAMP_TICKS);
                s_iq_ramp_ctr++;
            } else {
                iq_cmd = STARTUP_ALIGN_IQ_A;
            }
        } else {
            /* Ramp phase: scale Iq from align to ramp value.
             * Above handoff speed (pot-control): hold at RAMP_IQ. */
            float speed_frac = s_omega_ol / STARTUP_HANDOFF_RAD_S;
            if (speed_frac > 1.0f) speed_frac = 1.0f;
            iq_cmd = STARTUP_ALIGN_IQ_A
                   + (STARTUP_RAMP_IQ_A - STARTUP_ALIGN_IQ_A) * speed_frac;
        }

        DQ_t vdq;
        vdq.d = pi_update(&s_pid_d, 0.0f - idq.d, FOC_TS_FAST_S);
        vdq.q = pi_update(&s_pid_q, iq_cmd - idq.q, FOC_TS_FAST_S);
        inv_park_transform(&vdq, theta, &vab);
        svpwm_update(vab.alpha, vab.beta, s_vbus, &da, &db, &dc);
        HAL_PWM_SetDutyFloat3Phase(da, db, dc);

        garudaData.id_a       = idq.d;
        garudaData.iq_a       = idq.q;
#endif

        /* Estimated DC bus current: Idc = da×Ia + db×Ib + dc×Ic (both modes) */
        {
            float ic_phase = -(ia + ib);
            garudaData.idc_est = da * ia + db * ib + dc * ic_phase;
        }

        /* Warm up observer and PLL regardless of mode */
        {
            AlphaBeta_t iab_obs;
            clarke_transform(ia, ib, &iab_obs);
            bemf_obs_update(&s_obs,
                            vab.alpha, vab.beta,
                            iab_obs.alpha, iab_obs.beta,
                            FOC_TS_FAST_S);
            pll_update(&s_pll, s_obs.e_alpha, s_obs.e_beta, FOC_TS_FAST_S);
            flux_est_update(&s_flux_est, s_obs.e_alpha, s_obs.e_beta, FOC_TS_FAST_S);
#if FEATURE_SMO
            /* SMO runs in parallel — same inputs, separate BEMF estimate + PLL */
            smo_update(&s_smo,
                       vab.alpha, vab.beta,
                       iab_obs.alpha, iab_obs.beta);
            pll_update(&s_pll_smo, s_smo.e_alpha, s_smo.e_beta, FOC_TS_FAST_S);
#endif
        }

        /* Update telemetry (id_a, iq_a set inside voltage/current mode above) */
        garudaData.theta_rad   = theta;          /* OL angle (PLL-corrected above threshold) */
        garudaData.omega_rad_s = s_omega_ol;
        garudaData.theta2_rad  = pll_rotor_angle(); /* Corrected PLL angle for GUI */
#if FEATURE_SMO
        garudaData.smo_theta_rad  = smo_rotor_angle();
        garudaData.smo_omega_rad_s = s_pll_smo.omega_est;
#endif

        /* FEATURE_CLOSED_LOOP: PLL angle correction applied above (inside
         * speed control). Motor stays in STARTUP with corrected OL angle —
         * no state transition to RUNNING, no vibration from angle switching.
         * The PLL continuously improves angle quality without any handoff. */
    }
    else if (state == ESC_RUNNING)
    {
        /* ── Voltage-mode FOC: PLL angle + V/f voltage ──────────────────── *
         * Uses PLL-estimated angle (closed-loop tracking) but sets voltage
         * directly via V/f formula (no PI current controllers).
         * This avoids PI bang-bang oscillation at 12V while still getting
         * the benefit of sensorless angle tracking.
         * POT controls speed via V/f: V = ALIGN_V + Ke × ω_pll. */

        /* Release overrides if returning from fault */
        if (!s_ovr_released) {
            HAL_PWM_ReleaseAllOverrides();
            s_ovr_released = true;
        }

        float theta = pll_rotor_angle();  /* Corrected PLL angle */

        /* POT → speed target (same mapping as STARTUP) */
        float pot_target;
        if (throttle_raw < THROTTLE_DEADBAND) {
            pot_target = 0.0f;
        } else {
            float pot_frac = (float)(throttle_raw - THROTTLE_DEADBAND)
                           / (4095.0f - (float)THROTTLE_DEADBAND);
            pot_target = pot_frac * STARTUP_MAX_OL_RAD_S;
        }

        /* Slew-rate limit speed changes (same as STARTUP) */
        float max_delta = STARTUP_RAMP_RATE_RPS2 * FOC_TS_FAST_S;
        if (s_omega_ol < pot_target) {
            s_omega_ol += max_delta;
            if (s_omega_ol > pot_target) s_omega_ol = pot_target;
        } else if (s_omega_ol > pot_target) {
            s_omega_ol -= max_delta;
            if (s_omega_ol < pot_target) s_omega_ol = pot_target;
        }

        /* V/f voltage command based on commanded speed */
        float vq_cmd = STARTUP_ALIGN_V + MOTOR_KE_VPEAK * s_omega_ol;
        float vq_max = s_vbus * 0.95f;
        if (vq_cmd > vq_max) vq_cmd = vq_max;

        DQ_t vdq;
        vdq.d = 0.0f;   /* MTPA: Id=0 for surface-mount PMSM (no reluctance torque) */
        vdq.q = vq_cmd;

        AlphaBeta_t vab;
        inv_park_transform(&vdq, theta, &vab);

        float da, db, dc;
        svpwm_update(vab.alpha, vab.beta, s_vbus, &da, &db, &dc);
        HAL_PWM_SetDutyFloat3Phase(da, db, dc);

        /* Observer and PLL update */
        {
            AlphaBeta_t iab_obs;
            clarke_transform(ia, ib, &iab_obs);
            bemf_obs_update(&s_obs,
                            vab.alpha, vab.beta,
                            iab_obs.alpha, iab_obs.beta,
                            FOC_TS_FAST_S);
            pll_update(&s_pll, s_obs.e_alpha, s_obs.e_beta, FOC_TS_FAST_S);
            flux_est_update(&s_flux_est, s_obs.e_alpha, s_obs.e_beta, FOC_TS_FAST_S);
#if FEATURE_SMO
            smo_update(&s_smo,
                       vab.alpha, vab.beta,
                       iab_obs.alpha, iab_obs.beta);
            pll_update(&s_pll_smo, s_smo.e_alpha, s_smo.e_beta, FOC_TS_FAST_S);
#endif
        }

        /* Telemetry */
        garudaData.id_a        = ia;
        garudaData.iq_a        = ib;
        garudaData.theta_rad   = s_pll.theta_est;
        garudaData.omega_rad_s = s_pll.omega_est;
        garudaData.theta2_rad  = s_flux_est.theta_est;
#if FEATURE_SMO
        garudaData.smo_theta_rad  = smo_rotor_angle();
        garudaData.smo_omega_rad_s = s_pll_smo.omega_est;
#endif

        /* Estimated DC bus current */
        {
            float ic_phase = -(ia + ib);
            garudaData.idc_est = da * ia + db * ib + dc * ic_phase;
        }
    }
    else /* ESC_FAULT */
    {
        InitDutyPWM123Generators();
    }

isr_slow_loop:
    /* ── 5. Slow loop (every FOC_SLOW_DIV ticks → 1 kHz) ────────────────── */
    if (++s_slow_ctr >= FOC_SLOW_DIV)
    {
        s_slow_ctr = 0;
        state = garudaData.state;   /* Re-read after fast loop may have changed it */

        /* ── Arming counter ── */
        if (state == ESC_ARMED)
        {
            if (throttle_raw < THROTTLE_DEADBAND) {
                if (++garudaData.armCounter >= (uint32_t)FOC_ARM_TIME_MS) {
                    /* Throttle held at zero for ARM_TIME_MS → enter startup */
                    startup_reset();
                    bemf_obs_reset(&s_obs);
                    pll_reset(&s_pll);
#if FEATURE_SMO
                    smo_reset(&s_smo);
                    pll_reset(&s_pll_smo);
#endif
                    garudaData.state = ESC_STARTUP;
                }
            } else {
                garudaData.armCounter = 0; /* throttle not zero — keep waiting */
            }
        }

        /* ── Speed reference and stall detection (RUNNING only) ── */
        if (state == ESC_RUNNING)
        {
            /* Throttle → speed reference.
             * Throttle=0 at RUNNING entry is normal (arming requires throttle=0
             * for ARM_TIME_MS, so throttle is still 0 right after STARTUP handoff).
             * Do NOT stop the motor here — just demand zero torque and let the
             * user raise the POT.  Explicit stop is SW1 or GSP stop intent. */
            if (throttle_raw < THROTTLE_DEADBAND) {
                s_iq_ref = 0.0f;   /* coast — no torque, motor freewheels */
            } else {
                /* Use same POT→speed mapping as STARTUP to avoid
                 * speed reference discontinuity at handoff.
                 * STARTUP: pot_frac × MAX_OL_RAD_S (0→735)
                 * Must match so speed PI doesn't see a step change. */
                float pot_frac = (float)(throttle_raw - THROTTLE_DEADBAND)
                               / (4095.0f - (float)THROTTLE_DEADBAND);
                float spd_ref = pot_frac * STARTUP_MAX_OL_RAD_S;
                s_iq_ref = pi_update(&s_pid_spd,
                                     spd_ref - s_pll.omega_est,
                                     FOC_TS_SLOW_S);
            }

            /* Stall detection — only when throttle demands motion.
             * Suppressed at throttle=0 to avoid false faults while coasting. */
            if (throttle_raw >= THROTTLE_DEADBAND &&
                fabsf(s_pll.omega_est) < FAULT_STALL_RAD_S) {
                if (++s_stall_ctr >= (uint32_t)FAULT_STALL_TIMEOUT_MS) {
                    garudaData.state    = ESC_FAULT;
                    garudaData.faultCode = FAULT_STALL;
                    InitDutyPWM123Generators();
                    s_stall_ctr = 0;
                }
            } else {
                s_stall_ctr = 0;
            }
        }
    }

isr_done:
    /* Clear interrupt flag AFTER reading all ADC buffers */
    GARUDA_ClearADCIF();
}

/* =========================================================================
 * PWM Fault ISR — board-level FPCI (OC+OV via PCI8R)
 * Without this handler, the enabled PWM interrupt (_PWM1IE=1, FLTIEN=1)
 * causes an unhandled interrupt trap → processor reset.
 * ========================================================================= */
void __attribute__((__interrupt__, no_auto_psv)) _PWM1Interrupt(void)
{
    _PWM1IF = 0;

    /* Disable PWM outputs immediately */
    InitDutyPWM123Generators();

    /* Signal fault to main loop */
    garudaData.state     = ESC_FAULT;
    garudaData.faultCode = FAULT_BOARD_PCI;
}

/* =========================================================================
 * Timer1 ISR — 100 us tick
 * Handles: heartbeat LED, board service button debounce, 1ms system tick.
 * ========================================================================= */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    _T1IF = 0;
    
#ifdef ENABLE_DIAGNOSTICS    
    DiagnosticsStepIsr();
#endif    
    
    /* Heartbeat LED toggle at ~2 Hz */
    if (++heartbeatCounter >= HEART_BEAT_LED_COUNT)
    {
        heartbeatCounter = 0;
        LED1 ^= 1;
    }

    /* Board service step (drives button debounce at 1 ms) */
    BoardServiceStepIsr();

    /* 1 ms system tick sub-counter (10 × 100 us = 1 ms) */
    if (++msSubCounter >= 10)
    {
        msSubCounter = 0;
        garudaData.systemTick++;
    }
}
