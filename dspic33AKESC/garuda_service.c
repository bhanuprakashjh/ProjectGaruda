/**
 * @file garuda_service.c
 *
 * @brief ESC state machine and ADC ISR.
 *
 * State machine (driven from ADC ISR at PWM rate):
 *   IDLE → ARMED (throttle=0 for 500ms) → ALIGN → OL_RAMP
 *   → (Phase 2: CLOSED_LOOP)
 *
 * Timer1 ISR: 100us tick for heartbeat, board service, and commutation timing.
 *
 * Component: GARUDA SERVICE
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "garuda_service.h"
#include "garuda_types.h"
#include "garuda_config.h"
#include "garuda_calc_params.h"
#include "hal/hal_adc.h"
#include "hal/hal_pwm.h"
#include "hal/board_service.h"
#include "hal/port_config.h"
#include "motor/commutation.h"
#include "motor/startup.h"
#if FEATURE_BEMF_CLOSED_LOOP
#include "motor/bemf_zc.h"
#endif
#if FEATURE_ADC_CMP_ZC
#include "motor/hwzc.h"
#include "motor/speed_pi.h"
#include "hal/hal_timer.h"
#endif

#if FEATURE_LEARN_MODULES
#include "learn/learn_service.h"
#endif

#if (FEATURE_RX_PWM || FEATURE_RX_DSHOT || FEATURE_RX_AUTO)
#include "input/rx_decode.h"
#endif

#if FEATURE_FOC
#include <math.h>
#include "garuda_foc_params.h"
#include "foc/foc_types.h"
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
#if FEATURE_MXLEMMING
#include "foc/mxlemming_obs.h"
#endif
#endif

#if FEATURE_FOC_V2
#include <math.h>
#include "garuda_foc_params.h"
#include "foc/foc_v2_types.h"
#include "foc/foc_v2_control.h"
#include "foc/foc_v2_detect.h"
#include "hal/hal_pwm.h"
#if FEATURE_GSP
#include "gsp/gsp_params.h"
#endif
#endif

#if FEATURE_FOC_V3
#include <math.h>
#include "garuda_foc_params.h"
#include "foc/foc_v3_types.h"
#include "foc/foc_v3_control.h"
#include "hal/hal_pwm.h"
#if FEATURE_GSP
#include "gsp/gsp_params.h"
#endif
#endif

#if FEATURE_FOC_AN1078
#include <math.h>
#include "garuda_foc_params.h"
#include "foc/an1078_motor.h"
#include "foc/an1078_params.h"
#include "hal/hal_pwm.h"
#if FEATURE_GSP
#include "gsp/gsp_params.h"
#endif
#endif

#if FEATURE_BURST_SCOPE
#include "scope/scope_burst.h"
#endif

#include "x2cscope/diagnostics.h"

/* Global ESC runtime data — volatile: shared between ISRs and main loop */
volatile GARUDA_DATA_T garudaData;

#if FEATURE_AM32_STARTUP
/* Set by Timer1 ARMED exit, consumed by the ADC-ISR CL entry-init: do the
 * one blind kick + immediate HWZC arm (AM32 startMotor() semantics). */
static volatile uint8_t g_am32EntryPending;
#endif

#if FEATURE_FOC_V2
/* FOC v2 state — accessed only from ADC ISR (not volatile) */
static FOC_State_t s_foc_v2;
/* Flag: set by profile load handler, checked by ISR to re-init FOC */
volatile bool gspFocReinitNeeded;
#elif FEATURE_FOC_V3
/* FOC v3 state — SMO observer, accessed only from ADC ISR */
static V3_State_t s_foc_v3;
volatile bool gspFocReinitNeeded;
#elif FEATURE_FOC_AN1078
/* AN1078 motor controller — accessed only from ADC ISR */
/* Non-static so gsp_commands.c can re-init the SMC observer when motor
 * model params (Rs/Ls/Ke) are changed via SET_PARAM — re-uses gspParams
 * values to recompute F_PLANT/G_PLANT without firmware recompile. */
AN_Motor_T s_foc_an;
volatile bool gspFocReinitNeeded;
#endif

#if FEATURE_FOC_V2 || FEATURE_FOC_V3
/** Build FOC_MotorParams_t from GSP runtime params (or compile-time fallback).
 *  AN1078 uses its own constants in an1078_params.h, doesn't call this. */
static FOC_MotorParams_t BuildFocMotorParams(void) __attribute__((unused));
static FOC_MotorParams_t BuildFocMotorParams(void)
{
    FOC_MotorParams_t mp;
#if FEATURE_GSP
    mp.Rs             = (float)gspParams.focRsMilliOhm * 0.001f;
    mp.Ls             = (float)gspParams.focLsMicroH * 1e-6f;
    mp.Ke             = (float)gspParams.focKeUvSRad * 1e-6f;
    mp.lambda_pm      = mp.Ke;  /* Ke = lambda_pm for PMSM */
    mp.pole_pairs     = gspParams.motorPolePairs;
    mp.vbus_nom_v     = (float)gspParams.focVbusNomCentiV * 0.01f;
    mp.max_current_a  = (float)gspParams.focMaxCurrentCentiA * 0.01f;
    mp.max_elec_rad_s = (float)gspParams.focMaxElecRadS;
    mp.kp_dq          = (float)gspParams.focKpDqMilli * 0.001f;
    mp.ki_dq          = (float)gspParams.focKiDq;
    mp.obs_lpf_alpha  = (float)gspParams.focObsLpfAlphaMilli * 0.001f;
    mp.align_iq_a     = (float)gspParams.focAlignIqCentiA * 0.01f;
    mp.ramp_iq_a      = (float)gspParams.focRampIqCentiA * 0.01f;
    mp.align_ticks    = (uint32_t)gspParams.focAlignTimeMs * 24U;  /* ms → 24kHz ticks */
    mp.iq_ramp_ticks  = (uint32_t)gspParams.focIqRampTimeMs * 24U;
    mp.ramp_rate_rps2 = (float)gspParams.focRampRateRps2;
    mp.handoff_rad_s  = (float)gspParams.focHandoffRadS;
    mp.fault_oc_a     = (float)gspParams.focFaultOcCentiA * 0.01f;
    mp.fault_stall_rad_s = (float)gspParams.focFaultStallDeciRadS * 0.1f;
#else
    mp.Rs             = MOTOR_RS_OHM;
    mp.Ls             = MOTOR_LS_H;
    mp.lambda_pm      = MOTOR_FLUX_LINKAGE;
    mp.Ke             = MOTOR_KE_VPEAK;
    mp.pole_pairs     = MOTOR_POLE_PAIRS_FOC;
    mp.max_current_a  = MOTOR_MAX_CURRENT_A;
    mp.max_elec_rad_s = MOTOR_MAX_ELEC_RAD_S;
    mp.vbus_nom_v     = MOTOR_VBUS_NOM_V;
    mp.kp_dq          = KP_DQ;
    mp.ki_dq          = KI_DQ;
    mp.obs_lpf_alpha  = OBS_LPF_ALPHA;
    mp.align_iq_a     = STARTUP_ALIGN_IQ_A;
    mp.ramp_iq_a      = STARTUP_RAMP_IQ_A;
    mp.align_ticks    = STARTUP_ALIGN_TICKS;
    mp.iq_ramp_ticks  = STARTUP_IQ_RAMP_TICKS;
    mp.ramp_rate_rps2 = STARTUP_RAMP_RATE_RPS2;
    mp.handoff_rad_s  = STARTUP_HANDOFF_RAD_S;
    mp.fault_oc_a     = FAULT_OC_A;
    mp.fault_stall_rad_s = FAULT_STALL_RAD_S;
#endif
    return mp;
}
#endif

#if FEATURE_FOC
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
#if FEATURE_MXLEMMING
static MxlObs_t     s_mxl;          /* MXLEMMING flux observer */
#endif

static float        s_iq_ref;       /* Q-axis current reference (A) */
static float        s_vbus;         /* Last measured bus voltage (V) */
static uint16_t     s_slow_ctr;     /* Slow loop counter (0 -> FOC_SLOW_DIV) */

/* Open-loop startup ramp state */
static float        s_theta_ol;     /* Open-loop angle (rad) */
static float        s_omega_ol;     /* Open-loop speed (rad/s elec.) */
static bool         s_ovr_released; /* True after overrides released this run */

static uint32_t     s_iq_ramp_ctr;  /* Ticks elapsed since entry (Iq ramp) */
static uint32_t     s_align_ctr;    /* Ticks elapsed during alignment dwell */

/* PLL angle correction gain (per fast-loop tick): 2pi x BW x Ts */
#define PLL_CORRECTION_GAIN  (6.28318530718f * PLL_CORRECTION_BW_HZ * FOC_TS_FAST_S)

/* ADC zero-current offset calibration (sampled while motor is off) */
#define CAL_SAMPLES  1024U
#define CAL_SHIFT    10U
static uint32_t     s_ia_accum;
static uint32_t     s_ib_accum;
static uint16_t     s_cal_count;
static uint16_t     s_ia_offset;
static uint16_t     s_ib_offset;
static bool         s_cal_done;

/* Stall timer (slow loop ticks) */
static uint32_t     s_stall_ctr;

/* I/f → closed-loop transition state */
static bool         s_cl_active;    /* True after PLL lock → speed PI + PLL angle */
static uint32_t     s_cl_lock_ctr;  /* Consecutive ticks PLL tracks OL within tolerance */

/* Phase current OC debounce (software OC since ibusRaw=0 at PWM valley) */
#define FOC_OC_DEBOUNCE  48U  /* 48 ticks @ 24kHz = 2ms */
static uint16_t     s_foc_oc_ctr;

/* Helper: get PLL rotor angle (corrected for BEMF->rotor offset) */
static inline float pll_rotor_angle(void)
{
    float a = s_pll.theta_est - PLL_ANGLE_OFFSET;
    if (a < 0.0f)            a += 6.28318530718f;
    if (a >= 6.28318530718f) a -= 6.28318530718f;
    return a;
}

#if FEATURE_SMO
static inline float smo_rotor_angle(void)
{
    float a = s_pll_smo.theta_est - PLL_ANGLE_OFFSET;
    if (a < 0.0f)            a += 6.28318530718f;
    if (a >= 6.28318530718f) a -= 6.28318530718f;
    return a;
}
#endif

/* Forward declarations */
static void foc_startup_reset(void);

static void foc_startup_reset(void)
{
    s_theta_ol     = 0.0f;
    s_omega_ol     = 0.0f;
    s_ovr_released = false;
    s_iq_ramp_ctr  = 0;
    s_align_ctr    = 0;
    s_cl_active    = false;
    s_cl_lock_ctr  = 0;
    s_foc_oc_ctr   = 0;
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
#if FEATURE_MXLEMMING
    mxl_reset(&s_mxl);
#endif
}

static inline float counts_to_amps_cal(uint16_t raw, uint16_t offset)
{
    /* Negate: OA1/OA2 inverting topology on MCLV-48V-300W DIM.
     * AN1292 reference uses (HALF_ADC_COUNT - ADC_DATA) for same reason. */
    return -((float)(int16_t)(raw - offset)) * CURRENT_SCALE_A_PER_COUNT;
}

static inline float counts_to_vbus(uint16_t raw)
{
    return (float)raw * VBUS_SCALE_V_PER_COUNT;
}
#endif /* FEATURE_FOC */

#if FEATURE_FOC_V2 || FEATURE_FOC_V3 || FEATURE_FOC_AN1078
/* Reuse same helper for v2/v3/AN1078 telemetry raw→amps conversion */
static inline float v2_counts_to_amps(uint16_t raw, uint16_t offset)
{
    return -((float)(int16_t)(raw - offset)) * CURRENT_SCALE_A_PER_COUNT;
}
#endif

#if FEATURE_IF_STARTUP
/* ── I-f current-controlled spin-up (Milestone 1) ────────────────────────────
 * Reuses the standalone FOC primitives (Clarke/Park/SVPWM/PI) to drive a
 * regulated current vector at a forced, ramping angle. Because SVPWM sets the
 * differential phase voltage, the effective drive goes BELOW MIN_DUTY — so the
 * motor spins up smoothly with Ibus bounded at ifCurrentCa (no 22A slam) and no
 * deadtime floor. M1 = align → ramp to IF_HANDOFF and HOLD (no 6-step handoff). */
#include "foc/foc_types.h"
#include "foc/clarke.h"
#include "foc/park.h"
#include "foc/svpwm.h"
#include "foc/pi_controller.h"

static PI_t     s_if_pid_d, s_if_pid_q;
static float    s_if_theta;        /* forced electrical angle (rad) */
static float    s_if_omega;        /* forced electrical speed (elec rad/s) */
static uint16_t s_if_alignCtr;     /* ticks holding angle before the speed ramp */
static bool     s_if_atHandoff;    /* reached IF_HANDOFF (telemetry/M2 trigger) */
static bool     s_if_bridgeUp;     /* false until overrides released (after latch) */
/* MON current-offset calibration. The fixed IF_ADC_MIDPOINT (2048) is wrong on
 * this board — the MON channels rest at ia~4085, ib~84 (ib is starved by the
 * 1MHz HWZC channels unless they're disabled). Sample the TRUE rest over
 * IF_CAL_TICKS at zero current and subtract that instead of 2048. */
/* Phase-current offset calibration. With the CMP1/CMP2 root-cause fix
 * (hal_comparator.c — their input muxes railed OA1/OA2 in the 6-step build),
 * the op-amps should rest near 2048; the cal measures the TRUE rest anyway. */
static float    s_if_ia_off, s_if_ib_off;
static int32_t  s_if_ia_acc, s_if_ib_acc;
static uint16_t s_if_calCtr;
#define IF_CAL_TICKS  48u          /* ~2ms of zero-current rest sampling */

/* Self-contained ADC conversions: the FEATURE_FOC counts_to_* helpers, the
 * calibrated current offset, AND garuda_foc_params.h itself are all compiled
 * out in the pure 6-step build, so define the scales here from the documented
 * shunt/divider values (shunt 3mΩ × OA gain 24.95, Vref 3.3, FS 4095, Vbus
 * divider 23.2). Fixed 2048 bias matches the 6-step convention; negation
 * matches the FOC sign convention the reused gains were tuned with. */
#define IF_ADC_MIDPOINT   2048.0f
#define IF_CURRENT_SCALE  (3.3f / (4095.0f * 0.003f * 24.95f))   /* A per count */
#define IF_VBUS_SCALE     (3.3f * 23.2f / 4095.0f)               /* V per count */
/* Sign for the MON phase-current channels. The 6-step path reads them as
 * (raw-2048) POSITIVE (no negation, unlike the FOC MAIN channels). Wrong sign
 * = positive feedback = runaway → voltage clamp → full-scale current spike.
 * Diagnostic proved the MON channels are INVERTING (like the FOC MAIN
 * channels): +vd produced -idmeas. So negate (-1), same as the FOC path. */
#define IF_CURRENT_SIGN   (-1.0f)
static inline float if_raw_to_amps(uint16_t raw)
{
    return IF_CURRENT_SIGN * ((float)raw - IF_ADC_MIDPOINT) * IF_CURRENT_SCALE;
}
static inline float if_raw_to_vbus(uint16_t raw)
{
    return (float)raw * IF_VBUS_SCALE;
}

static void IF_StartupInit(void)
{
    float vbus = if_raw_to_vbus(garudaData.vbusRaw);
    if (vbus < 1.0f) vbus = 1.0f;
    /* Clamp Vd/Vq. Cap at 3V for the first real-current run: plenty to spin to
     * ~11k (BEMF there is ~1.5V) yet bounds a worst-case wrong-sign current
     * (the firmware OC/UV tiers backstop). Raise toward vbus*0.95*IF_INV_SQRT3
     * once confirmed stable. */
    float vclamp = vbus * 0.95f * IF_INV_SQRT3;
    if (vclamp > 3.0f) vclamp = 3.0f;
    float kp = (float)gspParams.focKpDqMilli * 0.001f;  /* tuned for this motor */
    float ki = (float)gspParams.focKiDq;
    pi_init(&s_if_pid_d, kp, ki, -vclamp, vclamp);
    pi_init(&s_if_pid_q, kp, ki, -vclamp, vclamp);
    s_if_theta     = 0.0f;
    s_if_omega     = 0.0f;
    s_if_alignCtr  = 0;
    s_if_atHandoff = false;
    s_if_bridgeUp  = false;
    /* Offset cal: seed with the nominal midpoint; refined in the cal window. */
    s_if_ia_off = IF_ADC_MIDPOINT; s_if_ib_off = IF_ADC_MIDPOINT;
    s_if_ia_acc = 0; s_if_ib_acc = 0; s_if_calCtr = 0;
    /* ADC channels are FOC-configured from BOOT (hal_adc.c, before ADC ON):
     * AD1CH0=OA1(ia), AD2CH0=OA2(ib); no 1MHz HWZC / MON channels in this
     * build. Nothing to reconfigure here — the earlier runtime PINSEL swap
     * (with ADON=1) is exactly what read OA2 as railed garbage. */
#if FEATURE_SINE_STARTUP
    garudaData.sine.active = false;     /* I-f owns the modulator now (only exists with sine) */
#endif
    /* Clean bring-up, part 1 (in this T1-ISR call): force override-LOW — undoes
     * STARTUP_Init/SineInit's drive AND keeps the low-sides on so the bootstrap
     * caps stay charged — then load BALANCED 0V duties into the buffer. We do
     * NOT release the overrides here: doing so before the balanced duty has
     * LATCHED makes the bridge briefly drive a stale/unbalanced duty → the UV
     * glitch. Part 2 (first IF_StartupTick) releases them once latched. */
    HAL_MC1PWMDisableOutputs();
    HAL_PWM_SetDutyFloat3Phase(0.5f, 0.5f, 0.5f);
}

static void IF_StartupTick(uint16_t raw_ia, uint16_t raw_ib)
{
    /* Clean bring-up, part 2: on the first tick the balanced duty loaded in
     * IF_StartupInit has now latched (≥1 PWM boundary elapsed since that T1
     * call), so release the overrides — the bridge goes override-low → balanced
     * 0V with no stale-duty transient. Hold balanced (skip the loop) this one
     * tick so the released bridge settles at 0V before current is commanded. */
    if (!s_if_bridgeUp) {
        HAL_PWM_ReleaseAllOverrides();
        HAL_PWM_SetDutyFloat3Phase(0.5f, 0.5f, 0.5f);
        s_if_bridgeUp = true;
        return;
    }

    /* Offset-cal window: hold balanced 0V (zero current) and average the op-amp
     * rest values, so id/iq are measured against the TRUE offset. spi_zcs
     * latches the measured ib offset — THE diagnostic for the CMP1/CMP2 fix:
     * ~2048 = op-amps un-railed and healthy; ~60/84 = still railed. */
    if (s_if_calCtr < IF_CAL_TICKS) {
        HAL_PWM_SetDutyFloat3Phase(0.5f, 0.5f, 0.5f);
        s_if_ia_acc += raw_ia;
        s_if_ib_acc += raw_ib;
        if (++s_if_calCtr >= IF_CAL_TICKS) {
            s_if_ia_off = (float)s_if_ia_acc / (float)IF_CAL_TICKS;
            s_if_ib_off = (float)s_if_ib_acc / (float)IF_CAL_TICKS;
            garudaData.speedPi.zcsSinceEnable = (uint16_t)s_if_ib_off;
        }
        return;
    }

    float ia   = IF_CURRENT_SIGN * ((float)raw_ia - s_if_ia_off) * IF_CURRENT_SCALE;
    float ib   = IF_CURRENT_SIGN * ((float)raw_ib - s_if_ib_off) * IF_CURRENT_SCALE;
    float vbus = if_raw_to_vbus(garudaData.vbusRaw);
    if (vbus < 1.0f) vbus = 1.0f;

    /* measure: 3-phase → αβ → dq at the forced angle */
    AlphaBeta_t iab;  clarke_transform(ia, ib, &iab);
    DQ_t idq;         park_transform(&iab, s_if_theta, &idq);

    /* regulate the current vector onto the forced d-axis (rotating field).
     * Soft-start: ramp the current reference up over the align window so the
     * rotor pulls into the forced frame gently (no step / no spike). */
    float frac = (s_if_alignCtr < IF_ALIGN_TICKS)
               ? ((float)s_if_alignCtr / (float)IF_ALIGN_TICKS) : 1.0f;
    float id_ref = (float)RT_IF_CURRENT_CA * 0.01f * frac;
    DQ_t vdq;
    vdq.d = pi_update(&s_if_pid_d, id_ref - idq.d, IF_DT_S);
    vdq.q = pi_update(&s_if_pid_q, 0.0f   - idq.q, IF_DT_S);

    /* dq → αβ → SVPWM duties (centered; effective V can be < MIN_DUTY) */
    AlphaBeta_t vab;  inv_park_transform(&vdq, s_if_theta, &vab);
    float da, db, dc;
    svpwm_update(vab.alpha, vab.beta, vbus, &da, &db, &dc);
    HAL_PWM_SetDutyFloat3Phase(da, db, dc);

    /* DIAG: spi_target=id_ref(mA), spi_error=id_meas(mA), spi_integ=vd(V),
     * spi_output=omega(rad/s), spi_zcs=ib offset. */
    garudaData.speedPi.lastTarget  = (uint32_t)(id_ref  * 1000.0f);
    garudaData.speedPi.lastError   = (int32_t) (idq.d   * 1000.0f);
    garudaData.speedPi.integratorF = vdq.d;
    garudaData.speedPi.outputDuty  = (uint32_t)(s_if_omega);

    /* align first (hold angle so the rotor settles), then ramp speed */
    if (s_if_alignCtr < IF_ALIGN_TICKS) {
        s_if_alignCtr++;
    } else if (s_if_omega < IF_HANDOFF_RAD_S) {
        float omega_dot = IF_ERPM_TO_RAD_S(RT_IF_RAMP_ERPM_PER_S);
        s_if_omega += omega_dot * IF_DT_S;
        if (s_if_omega >= IF_HANDOFF_RAD_S) {
            s_if_omega     = IF_HANDOFF_RAD_S;
            s_if_atHandoff = true;       /* M1: hold here; M2 will hand to 6-step */
        }
    }
    s_if_theta += s_if_omega * IF_DT_S;
    if (s_if_theta >  3.14159265f) s_if_theta -= 6.28318531f;
    if (s_if_theta < -3.14159265f) s_if_theta += 6.28318531f;
}
#endif /* FEATURE_IF_STARTUP */

#if FEATURE_IBUS_PROBE
/* PROBE: count of CMP3 rising-edge fires latched via _CMP3IF (set per cycle,
 * polled+cleared in the ADC ISR). Surfaced as the hijacked eRPM column so we
 * see the comparator output directly, independent of the CLPCI chop chain. */
volatile uint32_t g_cmp3FireCount = 0;
#endif

#if FEATURE_CL_DIFF_IDLE || FEATURE_CL_COAST_VERIFY
/* ── Coast-listen lock acquisition (2026-06-10) ──────────────────────────
 * At CL entry, instead of trusting the morph's lock (bench: the morph can
 * drag a SLIPPING rotor while its gate claims 3k lock — coast-listen caught
 * it at ~900 real), CUT THE BRIDGE for up to ~2 electrical cycles and listen
 * to phase B's clean BEMF — no PWM means no threshold model at all. The
 * median of the crossing intervals is the true period (one B-crossing every
 * half e-cycle = 3 sector periods); the last crossing's polarity gives the
 * sector exactly (B floats rising in step 2, falling in step 5; ZC = sector
 * center). Engage SYNCED at the crossing instant — differential drive when
 * FEATURE_CL_DIFF_IDLE, the conventional waveform otherwise (verify mode).
 * Same primitive AM32/BLHeli use to catch a spinning motor. Windage decel
 * over the window is negligible (morph coast measurements). The first ~3ms
 * after bridge-cut freewheel through the body diodes and ring the phases —
 * settle past it and reject impossible intervals. direction!=0 → no coast. */
#define CL_COAST_MIN_CROSS      3     /* crossings needed (=> 2 valid intervals) */
#define CL_COAST_MAX_IV         4     /* intervals stored for the median */
#define CL_COAST_TIMEOUT_TICKS  4500  /* ~100ms @45kHz — then blind fallback engage */
#define CL_COAST_SETTLE_TICKS   150   /* ~3.3ms: at bridge-cut the winding current
                                       * FREEWHEELS through the body diodes, slamming
                                       * the phase to the rails — ring-crossings.
                                       * (Bench 2026-06-10: 12-tick settle let a
                                       * synced-morph cut measure HALF the true
                                       * interval → 2× engage → +22A OC.) */
#define CL_COAST_IV_MIN         150   /* reject intervals implying > ~9k eRPM —
                                       * impossible coming from the ~3k morph;
                                       * those are ringing/noise, not BEMF */
#define CL_COAST_HYST           8     /* ADC counts hysteresis about the mean */

static struct {
    uint8_t  active;
    uint8_t  side;          /* 1=above mean, 0=below, 0xFF=not yet primed */
    uint8_t  nCross;        /* crossings seen */
    uint8_t  nIv;           /* intervals captured */
    uint8_t  lastRising;    /* polarity of last crossing */
    uint16_t tick;          /* ticks since coast start */
    uint16_t lastCrossTick;
    uint16_t mean;          /* running mean of phase-B = rest level estimate */
    uint16_t iv[CL_COAST_MAX_IV];
} s_clCoast;

#if FEATURE_CL_ENTRY_GLIDE
/* Entry-glide state: after the coast engage, effective duty (diff waveform)
 * ramps linearly from the speed-matched level to MIN_DUTY, then the drive
 * force-swaps to the conventional waveform (baseline from there on). */
static uint8_t  s_glideActive;
static uint8_t  s_glideDivCtr;

static uint16_t s_glideDuty;
#endif

static void CL_CoastBegin(void)
{
#if FEATURE_CL_ENTRY_GLIDE
    s_glideActive = 0;
#endif
    s_clCoast.active = 1;
    s_clCoast.side = 0xFF;
    s_clCoast.nCross = 0;
    s_clCoast.nIv = 0;
    s_clCoast.tick = 0;
    s_clCoast.lastCrossTick = 0;
    s_clCoast.mean = 0;
    HAL_MC1PWMDisableOutputs();   /* true coast: all phases Hi-Z (clears diff flag) */
}

/* Re-engage drive at `sector`, period `p`, next commutation in `dl` ticks,
 * marked synced. Mirrors the morph→CL exit seeding. */
static void CL_CoastEngage(uint8_t sector, uint16_t p, uint16_t dl, uint16_t now)
{
#if FEATURE_CL_ENTRY_GLIDE
    /* Glide engage: diff waveform at effective volts MATCHED to the measured
     * speed (duty = MIN_DUTY × erpm / equilibrium-erpm) — torque step at
     * engage ≈ 0, then the per-tick glide ramps duty to MIN_DUTY. */
    {
        uint32_t erpm = ERPM_FROM_ADC_STEP_NUM / p;
        uint32_t d = ((uint32_t)MIN_DUTY * erpm) / CL_GLIDE_EQ_ERPM;
        if (d > (uint32_t)MIN_DUTY)      d = MIN_DUTY;
        if (d < (uint32_t)MIN_DUTY / 4u) d = (uint32_t)MIN_DUTY / 4u;
        g_pwmDiffLow = 1;
        COMMUTATION_ApplyStep(&garudaData, sector);
        garudaData.duty = (uint16_t)d;
        HAL_PWM_SetDutyCycle(garudaData.duty);
        s_glideDuty   = (uint16_t)d;
        s_glideDivCtr = 0;
        s_glideActive = 1;
    }
#elif FEATURE_CL_DIFF_IDLE
    g_pwmDiffLow = 1;
    COMMUTATION_ApplyStep(&garudaData, sector);
    /* Engage GENTLY at the idle floor: the rotor is already at speed and only
     * needs holding volts. (Engaging at MIN_DUTY — which the deadtime comp
     * turns into a ~16% active pulse — caused a −21A blip at engage.) */
    garudaData.duty = CL_DIFF_IDLE_FLOOR;
    HAL_PWM_SetDutyCycle(garudaData.duty);
#else
    /* Verify mode: conventional waveform, same entry duty the morph coast
     * used (MIN_DUTY) — identical volts to today's hot hand-off, but at the
     * MEASURED sector and period instead of the morph's possibly-fictional
     * lock state. */
    COMMUTATION_ApplyStep(&garudaData, sector);
    garudaData.duty = MIN_DUTY;
    HAL_PWM_SetDutyCycle(garudaData.duty);
#endif
    BEMF_ZC_OnCommutation(&garudaData, now);

    garudaData.timing.stepPeriod = p;
    garudaData.timing.zcSynced = true;
    garudaData.timing.goodZcCount = (uint16_t)RT_ZC_SYNC_THRESHOLD;
    garudaData.timing.hasPrevZc = false;
    garudaData.timing.stepsSinceLastZc = 0;
    garudaData.timing.commDeadline = (uint16_t)(now + dl);
    garudaData.timing.deadlineActive = true;
#if FEATURE_BEMF_INTEGRATION || FEATURE_ADC_CMP_ZC
    garudaData.timing.deadlineIsZc = true;
#endif
    s_clCoast.active = 0;
}

/* @return true while the coast owns the bridge (caller skips normal CL). */
static bool CL_CoastListenTick(uint16_t vB, uint16_t now)
{
    if (!s_clCoast.active) return false;
    s_clCoast.tick++;

    /* Running mean of the clean BEMF = the rest level (signal average). */
    if (s_clCoast.mean == 0)
        s_clCoast.mean = vB;
    else
        s_clCoast.mean = (uint16_t)((int16_t)s_clCoast.mean
                       + (((int16_t)vB - (int16_t)s_clCoast.mean) >> 4));

    if (s_clCoast.tick >= CL_COAST_SETTLE_TICKS)
    {
        uint16_t hi = (uint16_t)(s_clCoast.mean + CL_COAST_HYST);
        uint16_t lo = (s_clCoast.mean > CL_COAST_HYST)
                    ? (uint16_t)(s_clCoast.mean - CL_COAST_HYST) : 0u;
        uint8_t side = s_clCoast.side;
        if (vB > hi)      side = 1;
        else if (vB < lo) side = 0;

        if (s_clCoast.side == 0xFF)
        {
            s_clCoast.side = side;            /* prime; no event */
        }
        else if (side != s_clCoast.side && side != 0xFF)
        {
            /* Crossing of the rest level (rising if now above). */
            s_clCoast.side = side;
            s_clCoast.lastRising = side;
            if (s_clCoast.nCross > 0)
            {
                uint16_t ivNow =
                    (uint16_t)(s_clCoast.tick - s_clCoast.lastCrossTick);
                /* Plausibility: too-short intervals are diode-ring/noise,
                 * not BEMF — discard (the timer still re-anchors below, so
                 * a later clean crossing measures from the last event). */
                if (ivNow >= CL_COAST_IV_MIN)
                {
                    if (s_clCoast.nIv >= CL_COAST_MAX_IV)
                    {
                        /* Sliding window: early intervals can be mean-bias
                         * artifacts — keep the most recent ones. */
                        uint8_t i;
                        for (i = 1; i < CL_COAST_MAX_IV; i++)
                            s_clCoast.iv[i - 1u] = s_clCoast.iv[i];
                        s_clCoast.nIv = CL_COAST_MAX_IV - 1u;
                    }
                    s_clCoast.iv[s_clCoast.nIv++] = ivNow;
                }
            }
            s_clCoast.lastCrossTick = s_clCoast.tick;
            s_clCoast.nCross++;

            if (s_clCoast.nCross >= CL_COAST_MIN_CROSS && s_clCoast.nIv >= 3)
            {
                /* CONSISTENCY GATE (2026-06-10): while the running mean is
                 * still biased early in the coast, it crosses the BEMF
                 * asymmetrically — ALTERNATING short/long intervals (bench:
                 * the post-sine bridge-cut measured a deterministic 79-tick
                 * period = "5696 eRPM" from a 3k rotor, impossible — all six
                 * runs identical). True BEMF crossings of a converged mean
                 * are uniform. Require the last 3 intervals to agree within
                 * 25% before trusting them; inconsistent → keep listening
                 * (the mean converges, later intervals become uniform);
                 * never consistent → timeout fallback (pre-coast behavior). */
                uint16_t a = s_clCoast.iv[s_clCoast.nIv - 3u];
                uint16_t b = s_clCoast.iv[s_clCoast.nIv - 2u];
                uint16_t c = s_clCoast.iv[s_clCoast.nIv - 1u];
                uint16_t mn = a, mx = a;
                if (b < mn) mn = b;
                if (b > mx) mx = b;
                if (c < mn) mn = c;
                if (c > mx) mx = c;
                if ((uint16_t)(mx - mn) <= (uint16_t)(mn >> 2))
                {
                    uint16_t m = (uint16_t)(a + b + c - mn - mx); /* median */
                    /* B crosses every HALF e-cycle = 3 sector periods. */
                    uint16_t p = (uint16_t)(m / 3u);
                    if (p < MIN_CL_ADC_STEP_PERIOD) p = MIN_CL_ADC_STEP_PERIOD;
                    if (p > RT_INITIAL_ADC_STEP_PERIOD) p = RT_INITIAL_ADC_STEP_PERIOD;

                    /* Sector from polarity: B floats rising in step 2, falling
                     * in step 5; the crossing IS the sector center, so the next
                     * commutation is half a period out. Engaging at the
                     * crossing instant keeps the angle error ≤ one tick. */
                    uint8_t k = s_clCoast.lastRising ? 2u : 5u;
                    CL_CoastEngage(k, p, (uint16_t)(p / 2u), now);
                    return true;
                }
            }
        }
    }

    /* Timeout — couldn't hear enough crossings (too slow / too quiet):
     * blind fallback = the pre-coast behavior (morph-seeded period). */
    if (s_clCoast.tick >= CL_COAST_TIMEOUT_TICKS)
        CL_CoastEngage(garudaData.currentStep,
                       garudaData.timing.stepPeriod,
                       (uint16_t)(garudaData.timing.stepPeriod / 2u), now);
    return true;
}
#endif /* FEATURE_CL_DIFF_IDLE || FEATURE_CL_COAST_VERIFY */

#if FEATURE_OC_AUTOZERO
/* OC auto-zero (2026-06-10): measured bus-ADC rest bias. 2048 = uncalibrated
 * (legacy behavior). Measured during ARMED with the bridge off; re-measured on
 * every disarm→arm. Shared with hal_comparator.c (CMP3 DAC shift). */
volatile uint16_t g_ocBiasAdc = 2048;
static uint16_t s_ocZeroRaw;       /* last UNCORRECTED bus sample (cal input) */
static uint32_t s_ocZeroAcc;
static uint16_t s_ocZeroCount;
static uint16_t s_ocZeroMin;       /* window spread — quiescence gate */
static uint16_t s_ocZeroMax;
#endif

/* Heartbeat LED counter */
static uint16_t heartbeatCounter = 0;
/* Sub-counter for 1ms system tick from 100us Timer1 */
static uint8_t msSubCounter = 0;

#if FEATURE_BEMF_CLOSED_LOOP
/* File-scope statics for ZC — only accessed from ADC ISR, NOT Timer1 ISR */
static uint16_t adcIsrTick = 0;
static ESC_STATE_T prevAdcState = ESC_IDLE;

#if FEATURE_SINE_STARTUP
/* Helper macro: write trap duties for active/float/low phases.
 * Uses virtual neutral (midpoint of active and low) for float phase. */
#define MORPH_WRITE_TRAP_DUTIES(step, activeDuty) do { \
    uint32_t _tf = ((activeDuty) + MIN_DUTY) / 2; \
    const COMMUTATION_STEP_T *_s = &commutationTable[(step)]; \
    uint32_t _dA = (_s->phaseA == PHASE_PWM_ACTIVE) ? (activeDuty) : \
                   (_s->phaseA == PHASE_FLOAT) ? _tf : MIN_DUTY; \
    uint32_t _dB = (_s->phaseB == PHASE_PWM_ACTIVE) ? (activeDuty) : \
                   (_s->phaseB == PHASE_FLOAT) ? _tf : MIN_DUTY; \
    uint32_t _dC = (_s->phaseC == PHASE_PWM_ACTIVE) ? (activeDuty) : \
                   (_s->phaseC == PHASE_FLOAT) ? _tf : MIN_DUTY; \
    HAL_PWM_SetDutyCycle3Phase(_dA, _dB, _dC); \
} while(0)
#endif
#endif

/**
 * @brief Initialize ESC service data to safe defaults.
 */
void GARUDA_ServiceInit(void)
{
    garudaData.state = ESC_IDLE;
    garudaData.throttle = 0;
    garudaData.currentStep = 0;
    garudaData.direction = DIRECTION_DEFAULT;
    garudaData.duty = 0;
    garudaData.vbusRaw = 0;
    garudaData.potRaw = 0;
    garudaData.faultCode = FAULT_NONE;
    garudaData.alignCounter = 0;
    garudaData.rampStepPeriod = RT_INITIAL_STEP_PERIOD;
    garudaData.rampCounter = 0;
    garudaData.systemTick = 0;
    garudaData.armCounter = 0;
    garudaData.runCommandActive = false;
    garudaData.desyncRestartAttempts = 0;
    garudaData.recoveryCounter = 0;

#if !FEATURE_FOC && !FEATURE_FOC_V2 && !FEATURE_FOC_V3 && !FEATURE_FOC_AN1078
    /* Phase-current monitor — start with empty max/min. iaMin initialized
     * to 0xFFFF so the first real sample wins the "less than" comparison. */
    garudaData.phaseCurrent.iaRaw = 0;
    garudaData.phaseCurrent.ibRaw = 0;
    garudaData.phaseCurrent.iaMax = 0;
    garudaData.phaseCurrent.iaMin = 0xFFFF;
    garudaData.phaseCurrent.ibMax = 0;
    garudaData.phaseCurrent.ibMin = 0xFFFF;
    garudaData.phaseCurrent.ibusWinMax = 0;
    garudaData.phaseCurrent.ibusWinMin = 0xFFFF;
    garudaData.phaseCurrent.iaAtFault = 0;
    garudaData.phaseCurrent.ibAtFault = 0;
    garudaData.phaseCurrent.iaMaxAtFault = 0;
    garudaData.phaseCurrent.iaMinAtFault = 0;
    garudaData.phaseCurrent.ibMaxAtFault = 0;
    garudaData.phaseCurrent.ibMinAtFault = 0;
    garudaData.phaseCurrent.ibusAtFault = 0;
    garudaData.phaseCurrent.ibusMaxAtFault = 0;
    garudaData.phaseCurrent.ibusMinAtFault = 0;
    garudaData.phaseCurrent.faultCaptured = 0;
#endif

#if FEATURE_FOC
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
#if FEATURE_MXLEMMING
    mxl_reset(&s_mxl);
#endif
    foc_startup_reset();
    s_iq_ref    = 0.0f;
    s_vbus      = MOTOR_VBUS_NOM_V;
    s_slow_ctr  = 0;
    s_stall_ctr = 0;
    s_ia_accum  = 0;
    s_ib_accum  = 0;
    s_cal_count = 0;
    s_ia_offset = ADC_MIDPOINT;
    s_ib_offset = ADC_MIDPOINT;
    s_cal_done  = false;
#endif

#if FEATURE_FOC_V2
    {
        FOC_MotorParams_t mp = BuildFocMotorParams();
        foc_v2_init(&s_foc_v2, &mp);
        gspFocReinitNeeded = false;
    }
#elif FEATURE_FOC_V3
    {
        FOC_MotorParams_t mp = BuildFocMotorParams();
        foc_v3_init(&s_foc_v3, &mp);
        gspFocReinitNeeded = false;
    }
#elif FEATURE_FOC_AN1078
    {
        AN_MotorInit(&s_foc_an);
        gspFocReinitNeeded = false;
    }
#endif

    /* Throttle source init — unconditional (Finding 42/54).
     * Priority: ADC_POT > RX_AUTO > RX_PWM > RX_DSHOT > GSP */
#if FEATURE_ADC_POT
    garudaData.throttleSource = THROTTLE_SRC_ADC;
#elif FEATURE_RX_AUTO
    garudaData.throttleSource = THROTTLE_SRC_AUTO;
#elif FEATURE_RX_PWM
    garudaData.throttleSource = THROTTLE_SRC_PWM;
#elif FEATURE_RX_DSHOT
    garudaData.throttleSource = THROTTLE_SRC_DSHOT;
#elif FEATURE_GSP
    garudaData.throttleSource = THROTTLE_SRC_GSP;
#endif

#if FEATURE_HW_OVERCURRENT
    garudaData.ibusRaw = 0;
    garudaData.ibusMax = 0;
    garudaData.clpciTripCount = 0;
    garudaData.fpciTripCount = 0;
#endif

    garudaData.bemf.bemfRaw = 0;
    garudaData.bemf.zcThreshold = 0;
    garudaData.bemf.zcAmpForFilterComp = 0;
    garudaData.bemf.fallOffBemfMin = 0xFFFF;  /* falling OFF-center envelope */
    garudaData.bemf.fallOffBemfMax = 0;
    garudaData.bemf.zeroCrossDetected = false;
    garudaData.bemf.cmpPrev = 0xFF;
    garudaData.bemf.cmpExpected = 0;
    garudaData.bemf.filterCount = 0;
    garudaData.bemf.ad2SettleCount = 0;
    garudaData.bemf.bemfSampleValid = true;
    garudaData.bemf.phaseBHigh = 0;
    garudaData.bemf.phaseBLow = 0;
    garudaData.bemf.phaseBHighValid = false;
    garudaData.bemf.phaseBLowValid = false;
    garudaData.bemf.measuredNeutral = 0;
    garudaData.bemf.neutralValid = false;
    garudaData.bemf.zcNeutral[0] = 0;
    garudaData.bemf.zcNeutral[1] = 0;
    garudaData.bemf.zcNeutral[2] = 0;
    garudaData.bemf.zcNeutralCount[0] = 0;
    garudaData.bemf.zcNeutralCount[1] = 0;
    garudaData.bemf.zcNeutralCount[2] = 0;

    garudaData.timing.stepPeriod = 0;
    garudaData.timing.lastCommTick = 0;
    garudaData.timing.lastZcTick = 0;
    garudaData.timing.prevZcTick = 0;
    garudaData.timing.zcInterval = 0;
    garudaData.timing.commDeadline = 0;
    garudaData.timing.forcedCountdown = 0;
    garudaData.timing.goodZcCount = 0;
    garudaData.timing.consecutiveMissedSteps = 0;
    garudaData.timing.stepsSinceLastZc = 0;
    for (uint8_t i = 0; i < 6; i++)
        garudaData.timing.stepMissCount[i] = 0;
    garudaData.timing.risingZcWorks = false;
    garudaData.timing.fallingZcWorks = false;
    garudaData.timing.zcSynced = false;
    garudaData.timing.deadlineActive = false;
    garudaData.timing.hasPrevZc = false;
#if FEATURE_BEMF_INTEGRATION || FEATURE_ADC_CMP_ZC
    garudaData.timing.deadlineIsZc = false;
#endif

    /* ISR priority setup */
#if FEATURE_ADC_CMP_ZC
#if GARUDA_TARGET_AK512
    GARUDA_ADC_IP = 6;  /* ADC ISR (VB conv = AD1CH3) lowered from 7 to 6 when HW ZC available */
    _AD1CMP1IP = 7;     /* AD1 comparator CH1 (VA): highest priority */
    _AD1CMP2IP = 7;     /* AD1 comparator CH2 (VB): highest priority */
    _AD2CMP2IP = 7;     /* AD2 comparator CH2 (VC): highest priority */
    _CCT1IP = 7;        /* SCCP1 timer: highest priority */
#else
    _AD1CH0IP = 6;      /* ADC ISR lowered from 7 to 6 when HW ZC available */
    _AD1CMP5IP = 7;     /* AD1 comparator CH5: highest priority */
    _AD2CMP1IP = 7;     /* AD2 comparator CH1: highest priority */
    _CCT1IP = 7;        /* SCCP1 timer: highest priority */
#endif /* GARUDA_TARGET_AK512 */
#endif

    /* Clear any latched PCI fault from previous run (U25B is latching).
     * Must happen before enabling ADC ISR, otherwise the first PWM
     * edge re-triggers the fault immediately. */
    HAL_MC1ClearPWMPCIFault();
    HAL_MC1PWMDisableOutputs();

    /* Enable ADC interrupt to start the control loop */
    GARUDA_ClearADCIF();
    GARUDA_EnableADCInterrupt();

#if FEATURE_ADC_CMP_ZC
    HWZC_Init(&garudaData);
    SPEED_PI_Init(&garudaData);
    HAL_ADC_InitHighSpeedBEMF();
    HAL_SCCP1_Init();
    HAL_SCCP2_Init();
    HAL_SCCP3_InitPeriodic(HWZC_SCCP3_PERIOD);  /* Start high-speed ADC trigger */
#endif

#if FEATURE_LEARN_MODULES
    LEARN_ServiceInit(&garudaData);
#endif
}

/**
 * @brief ADC ISR — runs at PWM rate (24kHz).
 * Reads BEMF/Vbus, runs state machine, updates PWM.
 * Phase 2: sole commutation owner for CLOSED_LOOP state
 * (bypassed when DIAGNOSTIC_MANUAL_STEP=1).
 */
void __attribute__((__interrupt__, no_auto_psv)) GARUDA_ADC_INTERRUPT(void)
{
    /* Read all ADC buffers. MUST read AD1CH0DATA first — interrupt source.
     * Reading clears data-ready condition on dsPIC33AK. */
#if FEATURE_FOC || FEATURE_FOC_V2 || FEATURE_FOC_V3 || FEATURE_FOC_AN1078
    /* FOC: AD1CH0 = Ia (OA1OUT), AD2CH0 = Ib (OA2OUT) — raw uint16_t */
    uint16_t raw_ia = ADCBUF_IA;
    uint16_t raw_ib = ADCBUF_IB;
#else
    /* 6-step: AD1CH0 = Phase B voltage (RB8), AD2CH0 = Phase A/C (muxed) */
    uint16_t phaseB_val = ADCBUF_PHASE_B;
    uint16_t phaseAC_val = ADCBUF_PHASE_AC;
#endif
    garudaData.vbusRaw = ADCBUF_VBUS;
    garudaData.potRaw = ADCBUF_POT;

    /* Throttle source mux — unconditional switch (Finding 53/56) */
    switch (garudaData.throttleSource) {
#if FEATURE_GSP
        case THROTTLE_SRC_GSP:
            garudaData.throttle = (uint16_t)((uint32_t)garudaData.gspThrottle * 4095 / 2000);
            break;
#endif
#if (FEATURE_RX_PWM || FEATURE_RX_DSHOT || FEATURE_RX_AUTO)
        case THROTTLE_SRC_PWM:
        case THROTTLE_SRC_DSHOT:
        case THROTTLE_SRC_AUTO:
            garudaData.throttle = rxCachedLocked ? rxCachedThrottleAdc : 0;
            break;
#endif
#if FEATURE_ADC_POT
        case THROTTLE_SRC_ADC:
            garudaData.throttle = garudaData.potRaw;
            break;
#endif
        default:
            /* Safety fallback — zero throttle for corrupted enum or disabled source.
             * Never reads floating ADC. (Finding 56) */
            garudaData.throttle = 0;
            break;
    }

#if FEATURE_BEMF_CLOSED_LOOP
    /* Capture entry state for transition detection. Must be saved before
     * any state changes (morph→CL etc.) so the NEXT tick sees the transition. */
    ESC_STATE_T entryState = garudaData.state;

    /* P1 DISABLED: Phase B rail-based measuredNeutral gives correct Phase B
     * midpoint (24 at low duty) but is 30% lower than the duty-proportional
     * value (34). Since HWZC uses zcThreshold directly (hwzc.c:169) for its
     * ADC comparator, the lower threshold pushes the comparator near the noise
     * floor → massive noise rejections (NW6: 75k rejects vs NW4: 20k) →
     * HWZC miss rate 26.7% → latch-off → software ZC fallback → failure.
     *
     * Phase B tracking still runs for diagnostic visibility in watch data. */
    if (garudaData.state == ESC_CLOSED_LOOP
#if FEATURE_SINE_STARTUP
        || (garudaData.state == ESC_MORPH
            && (garudaData.morph.subPhase == MORPH_HIZ
                || garudaData.morph.subPhase == MORPH_WINDOWED_HIZ))
#endif
       )
    {
        PHASE_STATE_T bRole = commutationTable[garudaData.currentStep].phaseB;
        if (bRole == PHASE_PWM_ACTIVE)
        {
            garudaData.bemf.phaseBHigh = phaseB_val;
            garudaData.bemf.phaseBHighValid = true;
            if (garudaData.bemf.phaseBLowValid)
            {
                garudaData.bemf.measuredNeutral =
                    (garudaData.bemf.phaseBHigh + garudaData.bemf.phaseBLow) >> 1;
                garudaData.bemf.neutralValid = true;
            }
        }
        else if (bRole == PHASE_LOW)
        {
            garudaData.bemf.phaseBLow = phaseB_val;
            garudaData.bemf.phaseBLowValid = true;
            if (garudaData.bemf.phaseBHighValid)
            {
                garudaData.bemf.measuredNeutral =
                    (garudaData.bemf.phaseBHigh + garudaData.bemf.phaseBLow) >> 1;
                garudaData.bemf.neutralValid = true;
            }
        }
        /* Steps 2,5: B=FLOAT — no update, use cached measuredNeutral */
    }

    /* ZC threshold: duty-proportional (P0) with symmetric IIR (P3).
     * P0: Exact division replaces >>18 shift (+1.7% bias fix).
     * P1 measured neutral disabled — see comment above. */
    static uint16_t zcThreshSmooth = 0;
    {
        /* P0: Duty-proportional threshold — always used.
         * Exact division replaces >>18 shift (+1.7% bias fix). */
#if FEATURE_CL_DIFF_IDLE
        /* Differential-low CL: BOTH driven phases carry the MIN_DUTY base
         * pulse (active = duty+MIN_DUTY, low = MIN_DUTY), so the float's
         * ON-center level — what this duty-proportional model tracks — is
         * raised by 2×MIN_DUTY. Bench 2026-06-10: at duty=2.2% the float swung
         * 47..136 while a flat-floored threshold (35) sat BELOW the whole
         * swing → zero ZC confirms → frozen pre-sync → OC. Midpoint model
         * (duty+2×MIN_DUTY → thr≈84) centers the threshold in that swing. */
        uint32_t thrDuty = garudaData.duty;
        if (g_pwmDiffLow) thrDuty += 3u * MIN_DUTY;   /* active=duty+2·MIN(+DT comp),
                                                       * low=MIN → midpoint duty/2+1.5·MIN */
        uint16_t rawThresh = (uint16_t)(
            ((uint32_t)garudaData.vbusRaw * thrDuty) / ZC_DUTY_DIVISOR);
#else
        uint16_t rawThresh = (uint16_t)(
            ((uint32_t)garudaData.vbusRaw * garudaData.duty) / ZC_DUTY_DIVISOR);
#endif
#if FEATURE_VIRTUAL_NEUTRAL
        /* Measured virtual neutral overrides the duty-model threshold while
         * the bridge is driving (CL/OL/morph). All three channels sampled at
         * the same PG1TRIGA instant this very tick. Downstream consumers
         * (smoothing IIR, live CMPLO refresh, falling-SW, filter comp)
         * inherit it unchanged. Bridge-off states keep the duty model: with
         * no drive the three dividers just read rest level (~equal), which
         * would park the threshold at the rest bias instead of near 0. */
        if (garudaData.state == ESC_CLOSED_LOOP
            || garudaData.state == ESC_OL_RAMP
            || garudaData.state == ESC_MORPH)
        {
            rawThresh = (uint16_t)(((uint32_t)ADCBUF_BEMF_VA
                                  + (uint32_t)ADCBUF_BEMF_VB
                                  + (uint32_t)ADCBUF_BEMF_VC) / 3u);
        }
#endif

        if (garudaData.state == ESC_CLOSED_LOOP
#if FEATURE_SINE_STARTUP
            || (garudaData.state == ESC_MORPH
                && (garudaData.morph.subPhase == MORPH_HIZ
                    || garudaData.morph.subPhase == MORPH_WINDOWED_HIZ))
#endif
           )
        {
            if (prevAdcState != ESC_CLOSED_LOOP
#if FEATURE_SINE_STARTUP
                && !(garudaData.state == ESC_MORPH
                     && (garudaData.morph.subPhase == MORPH_HIZ
                         || garudaData.morph.subPhase == MORPH_WINDOWED_HIZ)
                     && prevAdcState == ESC_MORPH)
#endif
               )
            {
                zcThreshSmooth = rawThresh;

                /* P1: Reset measured neutral on CL entry — start fresh each run */
                garudaData.bemf.neutralValid = false;
                garudaData.bemf.phaseBHighValid = false;
                garudaData.bemf.phaseBLowValid = false;
                garudaData.bemf.phaseBHigh = 0;
                garudaData.bemf.phaseBLow = 0;
                garudaData.bemf.zcNeutralCount[0] = 0;
                garudaData.bemf.zcNeutralCount[1] = 0;
                garudaData.bemf.zcNeutralCount[2] = 0;
            }
#if FEATURE_SINE_STARTUP
            /* Guardrail #7: only consume in windowed context — stale flag
             * from a prior run can't accidentally reseed during normal CL. */
            else if (garudaData.morph.forceThreshSeed
                     && garudaData.state == ESC_MORPH
                     && garudaData.morph.subPhase == MORPH_WINDOWED_HIZ)
            {
                zcThreshSmooth = rawThresh;
                garudaData.morph.forceThreshSeed = false;
            }
#endif
            else
            {
                /* P3: Symmetric IIR — 1/4 gain both directions, tau ~4 ticks = 0.17ms.
                 * Replaces asymmetric (fast rise 0.33ms, slow fall 10.7ms) that
                 * caused threshold lag during deceleration. */
                int16_t delta = (int16_t)rawThresh - (int16_t)zcThreshSmooth;
                zcThreshSmooth += (delta + 2) >> 2;
            }
            garudaData.bemf.zcThreshold = zcThreshSmooth;
        }
        else
        {
            zcThreshSmooth = rawThresh;  /* Non-CL: instant tracking */
            garudaData.bemf.zcThreshold = rawThresh;
        }

#if HWZC_THRESH_BIAS_DOWN
        /* TEMP (VEX/1407 @10V): pull the published detection threshold DOWN by a
         * fixed bias toward the true neutral. Applied to the OUTPUT only — the IIR
         * state (zcThreshSmooth) stays unbiased so the bias doesn't compound. */
        garudaData.bemf.zcThreshold =
            (garudaData.bemf.zcThreshold > HWZC_THRESH_BIAS_DOWN)
                ? (uint16_t)(garudaData.bemf.zcThreshold - HWZC_THRESH_BIAS_DOWN) : 0;
#endif

#if FEATURE_HWZC_FILTER_COMP
#if GARUDA_TARGET_AK512
        /* AK512: filter-comp amplitude = TRUE measured BEMF swing, NOT
         * zcThreshold. zcThreshold is the duty*Vbus neutral DC level; feeding
         * it made the comp offset scale with DUTY -> positive feedback (timing
         * error -> current -> duty -> bigger offset -> more advance -> more
         * error) that ran the 510 into regen above ~120k. The peak
         * |float - neutral| (bemfDevPeak, accumulated in the bemfRaw block
         * below) is duty-independent and tracks the real rotor BEMF, so
         * offset = amp*omega*tau stays the pure RC-lag correction at all
         * speeds. Latch once per sector on step change; IIR ~32 sectors. */
        {
            uint8_t step = garudaData.currentStep;
            if (garudaData.bemf.ampPrevStep != step) {
                garudaData.bemf.ampPrevStep = step;
                uint16_t pk = garudaData.bemf.bemfDevPeak;
                garudaData.bemf.bemfDevPeak = 0;   /* reset for the new sector */
                if (pk) {
                    if (garudaData.bemf.zcAmpForFilterComp == 0)
                        garudaData.bemf.zcAmpForFilterComp = pk;
                    else {
                        int32_t dAmp = (int32_t)pk
                                     - (int32_t)garudaData.bemf.zcAmpForFilterComp;
                        garudaData.bemf.zcAmpForFilterComp = (uint16_t)(
                            (int32_t)garudaData.bemf.zcAmpForFilterComp + (dAmp >> 5));
                    }
                }
            }
        }
#else
        /* 106 (UNCHANGED, bench-proven to 232k): slow IIR of zcThreshold. */
        if (garudaData.bemf.zcAmpForFilterComp == 0) {
            garudaData.bemf.zcAmpForFilterComp = garudaData.bemf.zcThreshold;
        } else {
            int32_t ampDelta = (int32_t)garudaData.bemf.zcThreshold
                             - (int32_t)garudaData.bemf.zcAmpForFilterComp;
            garudaData.bemf.zcAmpForFilterComp =
                (uint16_t)((int32_t)garudaData.bemf.zcAmpForFilterComp
                           + (ampDelta >> 11));   /* ~46 ms time constant */
        }
        if (garudaData.state != ESC_CLOSED_LOOP) {
            garudaData.bemf.zcAmpForFilterComp = garudaData.bemf.zcThreshold;
        }
#endif
#endif

#if FEATURE_ADC_CMP_ZC
        /* Live CMPLO refresh while HWZC is actively watching for a crossing.
         * Without this, CMPLO is only written at OnCommutation, so it stays
         * stale for up to one sector period (~91 µs at 100k eRPM, much longer
         * at low speed). Updating every 24 kHz tick tracks Vbus sag and duty
         * ramp mid-sector, which matters under load / during pot slew.
         *
         * Safety:
         *   - Gated on HWZC_WATCHING — during BLANKING/COMM_PENDING the
         *     comparator IE is disabled anyway.
         *   - CMPLO write is atomic (single 32-bit SFR).
         *   - CMPMOD is left untouched (set per-commutation in OnCommutation).
         *   - Deadband applied per-polarity, matching OnCommutation semantics.
         */
        if (garudaData.hwzc.enabled
            && garudaData.hwzc.phase == HWZC_WATCHING)
        {
            int8_t pol = commutationTable[garudaData.currentStep].zcPolarity;
            uint16_t t = garudaData.bemf.zcThreshold;
            /* Filter-lag pre-distortion mirrors what HWZC_OnCommutation applied
             * — keep the live refresh in lockstep so CMPLO stays compensated
             * as zcThreshold drifts with Vbus/duty mid-sector. */
            t = HWZC_ApplyFilterComp(&garudaData, t, (pol > 0));
            if (pol > 0)
                t = (t + HWZC_CMP_DEADBAND < 4095) ? t + HWZC_CMP_DEADBAND : 4095;
            else
                t = (t > HWZC_CMP_DEADBAND) ? t - HWZC_CMP_DEADBAND : 0;
            HAL_ADC_UpdateComparatorThreshold(garudaData.hwzc.activeCore, t);
        }
#endif
    }
#else
    garudaData.bemf.zcThreshold = garudaData.vbusRaw >> 1;
#endif

#if FEATURE_BEMF_CLOSED_LOOP
    /* Store floating phase ADC value in bemfRaw with validity tracking */
    {
        uint8_t fp = commutationTable[garudaData.currentStep].floatingPhase;
        if (fp == FLOATING_PHASE_B)
        {
            garudaData.bemf.bemfRaw = phaseB_val;
            garudaData.bemf.bemfSampleValid = true;
        }
        else if (garudaData.bemf.ad2SettleCount > 0)
        {
            garudaData.bemf.bemfRaw = phaseAC_val;
            garudaData.bemf.bemfSampleValid = false;
            garudaData.bemf.ad2SettleCount--;
        }
        else
        {
            garudaData.bemf.bemfRaw = phaseAC_val;
            garudaData.bemf.bemfSampleValid = true;
        }

#if FEATURE_HWZC_FILTER_COMP && GARUDA_TARGET_AK512
        /* Accumulate the TRUE BEMF swing for the filter-comp amplitude (latched
         * per sector by the block above): peak |float - neutral| while WATCHING,
         * BOTH polarities. Duty-independent -> no positive-feedback runaway. */
        if (garudaData.hwzc.enabled
            && garudaData.hwzc.phase == HWZC_WATCHING
            && garudaData.bemf.bemfSampleValid)
        {
            uint16_t b  = garudaData.bemf.bemfRaw;
            uint16_t th = garudaData.bemf.zcThreshold;
            uint16_t dev = (b > th) ? (uint16_t)(b - th) : (uint16_t)(th - b);
            if (dev > garudaData.bemf.bemfDevPeak)
                garudaData.bemf.bemfDevPeak = dev;
        }
#endif

#if FEATURE_ADC_CMP_ZC
        /* Diagnostic: falling-sector OFF-center BEMF envelope. Capture bemfRaw
         * (OFF-center floating sample) only while a FALLING sector is in the
         * WATCHING window — proves whether the falling crossing is visible at
         * OFF-center (envelope brackets zcThreshold) vs the silent ON-time
         * comparator. Cheap: 2 compares per tick, diagnostic only. */
        if (garudaData.hwzc.enabled
            && garudaData.hwzc.phase == HWZC_WATCHING
            && garudaData.bemf.bemfSampleValid
            && commutationTable[garudaData.currentStep].zcPolarity < 0)
        {
            uint16_t vfo = garudaData.bemf.bemfRaw;
            if (vfo < garudaData.bemf.fallOffBemfMin) garudaData.bemf.fallOffBemfMin = vfo;
            if (vfo > garudaData.bemf.fallOffBemfMax) garudaData.bemf.fallOffBemfMax = vfo;

#if FEATURE_HWZC_FALLING_SW && FEATURE_HWZC_SECTOR_PI
            /* Hybrid falling detector: the rotor's falling ZC is invisible to
             * the ON-time comparator but present here at OFF-center. Detect the
             * downward crossing of zcThreshold (one accept per sector) and record
             * it into the sector-PI path exactly as the HW comparator does
             * (hwzc.c:549) — OnPiPeriodExpired then uses it. Plausibility floor:
             * only past 1/4 of the period (the falling ZC sits ~mid-sector;
             * rejects early demag dips). Optional speed cap for coarse-resolution
             * top end. */
            if (!garudaData.hwzc.captureValid)
            {
                /* RC-lag compensation, falling branch (added 2026-06-07) — the
                 * HW rising path applies this (hwzc.c:251 / live refresh :726)
                 * but the falling SW path historically didn't, so falling drifted
                 * late vs rising as ω·τ grew. Apply the same per-polarity offset
                 * (falling: thresh + offset) then the falling deadband, so both
                 * polarities are phase-consistent. Magnitude tunes via
                 * HWZC_FILTER_AMP_PCT_FALLING. */
                uint16_t fth = HWZC_ApplyFilterComp(&garudaData,
                                                    garudaData.bemf.zcThreshold,
                                                    false /* falling */);
                fth = (fth > HWZC_CMP_DEADBAND) ? (uint16_t)(fth - HWZC_CMP_DEADBAND) : 0;
                uint32_t zc = HAL_SCCP2_ReadTimestamp();
                uint32_t intoSector = zc - garudaData.hwzc.lastCommStamp;
                bool plausible = intoSector > (garudaData.hwzc.timerPeriod >> 2);
#if HWZC_FALLING_SW_MAX_ERPM > 0
                uint32_t erpmNow = garudaData.hwzc.stepPeriodHR
                    ? HWZC_TICKS_TO_ERPM(garudaData.hwzc.stepPeriodHR) : 0;
                if (erpmNow > HWZC_FALLING_SW_MAX_ERPM) plausible = false;
#endif
                if (plausible && vfo < fth)
                {
                    garudaData.hwzc.lastCaptureHR = zc;
                    garudaData.hwzc.captureValid  = true;
                    garudaData.hwzc.lastZcStamp   = zc;
                    if (garudaData.hwzc.goodZcCount < 0xFFFE)
                        garudaData.hwzc.goodZcCount++;
                    garudaData.hwzc.missCount = 0;
                    garudaData.hwzc.totalZcCount++;
                }
            }
#endif
        }

#if FEATURE_HWZC_LOWSPD_OFFCTR && FEATURE_HWZC_FALLING_SW && FEATURE_HWZC_SECTOR_PI
        /* Phase 1 (OL->CL smooth-handoff plan): detect RISING ZC on the SAME
         * RC-filtered OFF-center (PWM-OFF) sample at low speed, so neither
         * polarity needs the PWM-ON comparator window. This is the enabler for a
         * low-duty hand-off (it removes the reason for the ~6% duty floor).
         * Mirrors the falling detector above with opposite sign. ADDITIVE: the
         * rising HW comparator still runs; first capture per sector wins
         * (!captureValid). Speed-gated (OFF-center res is ~1 PWM period). */
        if (garudaData.hwzc.enabled
            && garudaData.hwzc.phase == HWZC_WATCHING
            && garudaData.bemf.bemfSampleValid
            && commutationTable[garudaData.currentStep].zcPolarity > 0
            && !garudaData.hwzc.captureValid)
        {
            uint32_t erpmNow = garudaData.hwzc.stepPeriodHR
                ? HWZC_TICKS_TO_ERPM(garudaData.hwzc.stepPeriodHR) : 0;
            if (erpmNow > 0 && erpmNow < HWZC_LOWSPD_OFFCTR_MAX_ERPM)
            {
                uint16_t vfo = garudaData.bemf.bemfRaw;
                /* rising threshold: filter-comp + deadband on the HIGH side
                 * (mirror of the falling fth - deadband). Clamp to ADC range. */
                uint16_t rth = HWZC_ApplyFilterComp(&garudaData,
                                                    garudaData.bemf.zcThreshold,
                                                    true /* rising */);
                {
                    uint32_t r = (uint32_t)rth + HWZC_CMP_DEADBAND;
                    rth = (r > 4095u) ? 4095u : (uint16_t)r;
                }
                uint32_t zc = HAL_SCCP2_ReadTimestamp();
                uint32_t intoSector = zc - garudaData.hwzc.lastCommStamp;
                bool plausible = intoSector > (garudaData.hwzc.timerPeriod >> 2);
                if (plausible && vfo > rth)
                {
                    garudaData.hwzc.lastCaptureHR = zc;
                    garudaData.hwzc.captureValid  = true;
                    garudaData.hwzc.lastZcStamp   = zc;
                    if (garudaData.hwzc.goodZcCount < 0xFFFE)
                        garudaData.hwzc.goodZcCount++;
                    garudaData.hwzc.missCount = 0;
                    garudaData.hwzc.totalZcCount++;
                }
            }
        }
#endif
#endif
    }
    adcIsrTick++;

#if FEATURE_ADC_CMP_ZC && HWZC_USE_SW_COMPARE
    /* Software HWZC path: run the ZC compare on the just-captured mid-ON
     * sample. Mid-ON sampling (PG1TRIGA valley) avoids the ~48 kHz phantom
     * rate the HW digital comparator sees on this board (5.5 kHz RC filter
     * can't smooth 24 kHz PWM → ripple crosses threshold every cycle). */
    HWZC_OnSoftwareSample(&garudaData);
#endif

#if !FEATURE_FOC && !FEATURE_FOC_V2 && !FEATURE_FOC_V3 && !FEATURE_FOC_AN1078
    /* Phase-current peak tracking (diagnostic). AD1CH3 / AD2CH2 convert at
     * 24 kHz (PG1TRIGA, mid-ON valley). max/min are the per-sample window
     * peaks; they're reset after each GSP snapshot read (see gsp_snapshot.c).
     * So each telemetry row shows the peaks in the most recent ~20 ms.
     *
     * On CL entry: clear the "frozen-at-fault" snapshot so a new CL run
     * gets a fresh fault capture when/if it trips. */
    {
        if (entryState == ESC_CLOSED_LOOP && prevAdcState != ESC_CLOSED_LOOP) {
            garudaData.phaseCurrent.faultCaptured = 0;
            garudaData.phaseCurrent.iaAtFault = 0;
            garudaData.phaseCurrent.ibAtFault = 0;
            garudaData.phaseCurrent.iaMaxAtFault = 0;
            garudaData.phaseCurrent.iaMinAtFault = 0;
            garudaData.phaseCurrent.ibMaxAtFault = 0;
            garudaData.phaseCurrent.ibMinAtFault = 0;
            garudaData.phaseCurrent.ibusAtFault = 0;
            garudaData.phaseCurrent.ibusMaxAtFault = 0;
            garudaData.phaseCurrent.ibusMinAtFault = 0;
        }
        uint16_t ia = ADCBUF_IA_MON;
        uint16_t ib = ADCBUF_IB_MON;
        garudaData.phaseCurrent.iaRaw = ia;
        garudaData.phaseCurrent.ibRaw = ib;
        if (ia > garudaData.phaseCurrent.iaMax) garudaData.phaseCurrent.iaMax = ia;
        if (ia < garudaData.phaseCurrent.iaMin) garudaData.phaseCurrent.iaMin = ia;
        if (ib > garudaData.phaseCurrent.ibMax) garudaData.phaseCurrent.ibMax = ib;
        if (ib < garudaData.phaseCurrent.ibMin) garudaData.phaseCurrent.ibMin = ib;

        /* Bus-current window tracking — uses the existing garudaData.ibusRaw
         * which is captured later in this ISR, but at this point still holds
         * the PREVIOUS ISR's value (which is fine for window-aggregating). */
#if FEATURE_HW_OVERCURRENT
        uint16_t ibus = garudaData.ibusRaw;
        if (ibus > garudaData.phaseCurrent.ibusWinMax) garudaData.phaseCurrent.ibusWinMax = ibus;
        if (ibus < garudaData.phaseCurrent.ibusWinMin) garudaData.phaseCurrent.ibusWinMin = ibus;
#endif

        /* Freeze a snapshot on the first BOARD_PCI transition of this run.
         * faultCaptured is cleared at CL entry, set once here, so we keep
         * the VERY FIRST fault's currents (not any later re-trip). */
        if (!garudaData.phaseCurrent.faultCaptured
            && garudaData.faultCode == FAULT_BOARD_PCI)
        {
            garudaData.phaseCurrent.iaAtFault    = ia;
            garudaData.phaseCurrent.ibAtFault    = ib;
            garudaData.phaseCurrent.iaMaxAtFault = garudaData.phaseCurrent.iaMax;
            garudaData.phaseCurrent.iaMinAtFault = garudaData.phaseCurrent.iaMin;
            garudaData.phaseCurrent.ibMaxAtFault = garudaData.phaseCurrent.ibMax;
            garudaData.phaseCurrent.ibMinAtFault = garudaData.phaseCurrent.ibMin;
#if FEATURE_HW_OVERCURRENT
            garudaData.phaseCurrent.ibusAtFault    = garudaData.ibusRaw;
            garudaData.phaseCurrent.ibusMaxAtFault = garudaData.phaseCurrent.ibusWinMax;
            garudaData.phaseCurrent.ibusMinAtFault = garudaData.phaseCurrent.ibusWinMin;
#endif
            garudaData.phaseCurrent.faultCaptured = 1;
        }

#if FEATURE_BURST_SCOPE
        /* Stream 6-step diagnostic channels into burst scope ring (24 kHz).
         *
         * Reuses the FOC-oriented SCOPE_SAMPLE_T fields:
         *   ia  = Phase A current (OA1 → AD1CH3) in mA     [accurate]
         *   ib  = Phase B current (OA2 → AD2CH2) in mA     [broken on this
         *         MCLV+EV68M17A combo — reads ~0; kept as sanity channel]
         *   id  = Bus current (OA3/M1_IBUS_FILT) in mA     [REPURPOSED
         *         for 6-step; what U25B actually trips on]
         *   vd  = Vbus in centivolts (raw/10, approx)
         *   vq  = zcThreshold raw
         *   theta = currentStep (sector 0-5)
         *   omega = eRPM / 10 (fits int16 up to 327 kRPM)
         *   mod_index = dutyPct × 100 (0-10000)
         *   flags:  bit0=HWZC enabled, bit1=fault
         *   state:  garudaData.state
         *
         * Scale (shared with phase-current monitor): ~93 ADC counts/A, bias 2048.
         * mA = (raw - 2048) × 1000 / 93. Clamped to int16 range.
         */
        {
            SCOPE_SAMPLE_T ss;
            int32_t ma;

            ma = ((int32_t)garudaData.phaseCurrent.iaRaw - 2048) * 1000 / 93;
            if (ma > 32767)  ma = 32767;
            if (ma < -32768) ma = -32768;
            ss.ia = (int16_t)ma;

            ma = ((int32_t)garudaData.phaseCurrent.ibRaw - 2048) * 1000 / 93;
            if (ma > 32767)  ma = 32767;
            if (ma < -32768) ma = -32768;
            ss.ib = (int16_t)ma;

#if FEATURE_HW_OVERCURRENT
            ma = ((int32_t)garudaData.ibusRaw - 2048) * 1000 / 93;
            if (ma > 32767)  ma = 32767;
            if (ma < -32768) ma = -32768;
            ss.id = (int16_t)ma;
#else
            ss.id = 0;
#endif
            ss.iq       = 0;
            ss.vd       = (int16_t)(garudaData.vbusRaw);      /* raw ADC */
            ss.vq       = (int16_t)(garudaData.bemf.zcThreshold);
            ss.theta    = (int16_t)(garudaData.currentStep);
            ss.obs_x1   = (int16_t)(garudaData.bemf.bemfRaw);
            ss.obs_x2   = 0;
            {
                uint32_t erpm_hr = (garudaData.hwzc.enabled && garudaData.hwzc.stepPeriodHR)
                                 ? HWZC_TICKS_TO_ERPM(garudaData.hwzc.stepPeriodHR)
                                 : 0;
                int32_t eRPMscaled = (int32_t)(erpm_hr / 10);
                if (eRPMscaled > 32767) eRPMscaled = 32767;
                ss.omega = (int16_t)eRPMscaled;
            }
            {
                uint32_t dutyPctX100 = (garudaData.duty * 10000U) / LOOPTIME_TCY;
                if (dutyPctX100 > 32767) dutyPctX100 = 32767;
                ss.mod_index = (int16_t)dutyPctX100;
            }
            ss.flags  = (garudaData.hwzc.enabled ? 0x01 : 0x00)
                      | ((garudaData.state == ESC_FAULT) ? 0x02 : 0x00);
            ss.state  = (uint8_t)garudaData.state;
            ss.tick_lsb = (uint16_t)(garudaData.systemTick & 0xFFFF);
            Scope_WriteSample(&ss);
        }
#endif /* FEATURE_BURST_SCOPE */
    }
#endif
#else
#if !FEATURE_FOC && !FEATURE_FOC_V2 && !FEATURE_FOC_V3 && !FEATURE_FOC_AN1078
    garudaData.bemf.bemfRaw = phaseB_val;
    (void)phaseAC_val;  /* AD2CH0DATA must be read; suppress unused warning */
#endif
#endif

    /* Bus voltage fault enforcement (OV/UV) */
#if FEATURE_VBUS_FAULT
    {
        static uint8_t vbusOvCount = 0, vbusUvCount = 0;

        if ((garudaData.state >= ESC_ALIGN && garudaData.state <= ESC_CLOSED_LOOP)
            || garudaData.state == ESC_IF_RAMP)   /* I-f spin-up: keep OV/UV active */
        {
            if (garudaData.vbusRaw > RT_VBUS_OVERVOLTAGE_ADC)
            {
                if (++vbusOvCount >= VBUS_FAULT_FILTER)
                {
#if FEATURE_ADC_CMP_ZC
                    if (garudaData.hwzc.enabled)
                        HWZC_Disable(&garudaData);
                    garudaData.hwzc.fallbackPending = false;
#endif
#if FEATURE_HW_OVERCURRENT
                    HAL_CMP3_SetThreshold(RT_OC_CMP3_STARTUP_DAC);
#endif
                    HAL_MC1PWMDisableOutputs();
                    garudaData.state = ESC_FAULT;
                    garudaData.faultCode = FAULT_OVERVOLTAGE;
                    garudaData.runCommandActive = false;
                    LED2 = 0;
                }
            }
            else { vbusOvCount = 0; }

            {
                /* Determine UV threshold: relaxed during pre-sync startup to
                 * tolerate bus sag from CC-limited bench supply. Normal
                 * threshold resumes after ZC sync is achieved. */
                uint16_t uvThreshold = RT_VBUS_UNDERVOLTAGE_ADC;
#if FEATURE_PRESYNC_RAMP
                if (garudaData.state <= ESC_OL_RAMP
                    || (garudaData.state == ESC_CLOSED_LOOP
                        && !garudaData.timing.zcSynced))
                    uvThreshold = RT_VBUS_UV_STARTUP_ADC;
#endif

            if (garudaData.vbusRaw < uvThreshold)
            {
                if (++vbusUvCount >= VBUS_FAULT_FILTER)
                {
#if FEATURE_ADC_CMP_ZC
                    if (garudaData.hwzc.enabled)
                        HWZC_Disable(&garudaData);
                    garudaData.hwzc.fallbackPending = false;
#endif
#if FEATURE_HW_OVERCURRENT
                    HAL_CMP3_SetThreshold(RT_OC_CMP3_STARTUP_DAC);
#endif
                    HAL_MC1PWMDisableOutputs();
                    garudaData.state = ESC_FAULT;
                    garudaData.faultCode = FAULT_UNDERVOLTAGE;
                    garudaData.runCommandActive = false;
                    LED2 = 0;
                }
            }
            else { vbusUvCount = 0; }
            } /* uvThreshold scope */
        }
        else
        {
            vbusOvCount = 0;
            vbusUvCount = 0;
        }
    }
#endif

    /* Bus current sensing and overcurrent protection */
#if FEATURE_HW_OVERCURRENT
#if FEATURE_OC_AUTOZERO
    /* Read AD1CH2DATA — clears data-ready (mandatory on dsPIC33AK) — then
     * re-center into the 2048 frame that every threshold, the window tracker
     * and the host decoder assume. The chain's TRUE rest is ~78 counts, so
     * the correction is ≈ +1970 (no-op until the ARMED cal measures it).
     * Saturates at 4095 ≈ +22A — beyond that is CMP3 hardware territory. */
    {
        uint16_t rawBus = ADCBUF_IBUS;
        s_ocZeroRaw = rawBus;                       /* for the ARMED cal */
#if FEATURE_IBUS_ONCENTER
        /* Pulse-center (active-vector) conduction current swings OA3 BELOW the
         * rest bias, so the deviation is inverted vs the old freewheel sample.
         * Mirror it back into the 2048 frame: positive = motoring conduction. */
        int32_t corr = (2048 + (int32_t)g_ocBiasAdc) - (int32_t)rawBus;
#else
        int32_t corr = (int32_t)rawBus + (2048 - (int32_t)g_ocBiasAdc);
#endif
        if (corr < 0)    corr = 0;
        if (corr > 4095) corr = 4095;
        garudaData.ibusRaw = (uint16_t)corr;
    }
#else
    /* Read AD1CH2DATA — clears data-ready (mandatory on dsPIC33AK) */
    garudaData.ibusRaw = ADCBUF_IBUS;
#endif

    /* Track peak for diagnostics */
    if (garudaData.ibusRaw > garudaData.ibusMax)
        garudaData.ibusMax = garudaData.ibusRaw;

    /* IIR low-pass (EMA, alpha=1/256 -> ~5.7ms TC @45kHz) of the instantaneous
     * conduction sample -> ibusAvg: a smooth trend for the host to display
     * alongside ibusInst. Q8 accumulator seeded at the 2048 bias (=0 A). */
    {
        static int32_t s_ibusAvgAcc = (int32_t)2048 << 8;
        s_ibusAvgAcc += (int32_t)garudaData.ibusRaw - (s_ibusAvgAcc >> 8);
        garudaData.ibusAvg = (uint16_t)(s_ibusAvgAcc >> 8);
    }

    /* Count CLPCI activity via CLEVT latched event flags.
     * Poll all 3 generators — active PWM phase rotates with commutation.
     * Coarse counter: one 41.7us ADC tick may collapse multiple chop events. */
#if (OC_PROTECT_MODE == 0 || OC_PROTECT_MODE == 2) && OC_CLPCI_ENABLE
    if (PCI_CLIMIT_EVT_PG1 || PCI_CLIMIT_EVT_PG2 || PCI_CLIMIT_EVT_PG3)
    {
        garudaData.clpciTripCount++;
        /* Clear W1C event flags via direct register write (not bitfield RMW)
         * to avoid accidentally clearing other W1C bits in PGxSTAT. */
        PG1STAT = PCI_CLIMIT_EVT_MASK;
        PG2STAT = PCI_CLIMIT_EVT_MASK;
        PG3STAT = PCI_CLIMIT_EVT_MASK;
    }
#endif

#if FEATURE_IBUS_PROBE
    /* Live CMP3 output LEVEL. With the probe DAC forced below the OA3 rest, this
     * should read 1 continuously if CMP3 is wired to OA3 — independent of motor
     * current or the freewheel sample point. 0 = comparator blind to OA3. */
    g_cmp3FireCount = (uint32_t)DAC3CMPbits.CMPSTAT;
#endif

#ifdef ENABLE_PWM_FAULT_PCI
    /* Count transient FPCI trips via FLTEVT latched event flags.
     * With TERM=1 (auto-terminate), board FPCI trips that resolve within
     * one PWM cycle never set FLTACT by the time the ISR runs — but
     * FLTEVT latches the event. Non-zero count = duty being chopped. */
    if (PCI_FAULT_EVT_PG1 || PCI_FAULT_EVT_PG2 || PCI_FAULT_EVT_PG3)
    {
        garudaData.fpciTripCount++;
        PG1STAT = PCI_FAULT_EVT_MASK;
        PG2STAT = PCI_FAULT_EVT_MASK;
        PG3STAT = PCI_FAULT_EVT_MASK;
    }
#endif

    /* Software hard fault — immediate shutdown (Mode 2 only).
     * DEBOUNCED 3 consecutive samples (2026-06-10): a single garbage bus-ADC
     * sample (the readout spikes to the rail at random transition moments —
     * bench: +22A reads with the bridge OFF) was latching phantom OC_SW and
     * killing healthy runs. Real overcurrent is sustained for ≫3 ADC ticks
     * (~67µs), and the truly fast events belong to the CMP3 HARDWARE
     * comparator layer anyway — the software check is the slow backstop. */
#if OC_PROTECT_MODE == 2
    {
        static uint8_t s_ocFaultDebounce = 0;
        if (garudaData.ibusRaw > OC_FAULT_ADC_VAL)
        {
            if (s_ocFaultDebounce < 255u) s_ocFaultDebounce++;
        }
        else
            s_ocFaultDebounce = 0;

    if (s_ocFaultDebounce >= 3u
        && ((garudaData.state >= ESC_ALIGN
             && garudaData.state <= ESC_CLOSED_LOOP)
            || garudaData.state == ESC_IF_RAMP))   /* protect I-f spin-up too */
    {
#if FEATURE_ADC_CMP_ZC
        if (garudaData.hwzc.enabled)
            HWZC_Disable(&garudaData);
        garudaData.hwzc.fallbackPending = false;
#endif
        HAL_MC1PWMDisableOutputs();
        garudaData.state = ESC_FAULT;
        garudaData.faultCode = FAULT_OVERCURRENT;
        garudaData.runCommandActive = false;
        LED2 = 0;
    }
    }  /* debounce scope */
#endif
#endif /* FEATURE_HW_OVERCURRENT */

#if FEATURE_FOC
    /* ── FOC control path — replaces ENTIRE 6-step state machine ──
     * Architecture from RK1 (proven on A2212 1400KV):
     *   - Inline modular calls (no MCAPP_FOCStateMachine black box)
     *   - V/f voltage-mode with gradual PLL angle correction
     *   - POT=0 → motor coasts to stop; POT>0 → resumes (no re-arm)
     *   - No ESC_RUNNING state — PLL correction makes CL implicit
     */
    {
        /* Convert ADC counts → physical units using calibrated offsets */
        float ia    = counts_to_amps_cal(raw_ia, s_ia_offset);
        float ib    = counts_to_amps_cal(raw_ib, s_ib_offset);
        s_vbus      = counts_to_vbus(garudaData.vbusRaw);

        /* Dynamic PI voltage clamp — must match circular limiter (Vbus/sqrt(3))
         * so anti-windup engages at the real output ceiling, not 1.73x above it. */
        {
            float vclamp = s_vbus * 0.95f * 0.577350269f;
            s_pid_d.out_max =  vclamp;
            s_pid_d.out_min = -vclamp;
            s_pid_q.out_max =  vclamp;
            s_pid_q.out_min = -vclamp;
        }

        /* Throttle source: use garudaData.throttle (already muxed above) */
        uint16_t throttle_raw = garudaData.throttle;

        /* State dispatch */
        ESC_STATE_T state = garudaData.state;

        if (state == ESC_IDLE || state == ESC_ARMED)
        {
            /* Outputs off — PWM overrides already asserted */
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

            goto foc_slow_loop;
        }
        else if (state == ESC_CLOSED_LOOP)
        {
            /* ── I/f (current-forced) sensorless FOC ─────────────────────
             * PI current controllers active from tick 0.
             * Phase 1 (alignment): θ=0 fixed, Iq ramps 0→ALIGN_IQ.
             * Phase 2 (I/f OL):    θ forced at pot speed, Id=0 + Iq=RAMP_IQ.
             * Phase 3 (CL):        PLL angle, speed PI → Iq reference.
             *
             * Key advantage over V/f: on low-Rs motors (A2212, 0.065Ω),
             * any angle error θ_err causes Id_waste = V*sin(θ_err)/Rs.
             * V/f cannot correct this (voltage is set, not current).
             * I/f commands Id=0 and PI instantly cancels d-axis waste. */

            /* Release PWM overrides once at CLOSED_LOOP entry */
            if (!s_ovr_released) {
                HAL_PWM_ReleaseAllOverrides();
                s_ovr_released = true;
            }

#if FOC_DIAG_PWM_TEST == 1
            /* ── DIAG 1: PWM-only — 50% duty, no FOC math ──────── */
            HAL_PWM_SetDutyFloat3Phase(0.5f, 0.5f, 0.5f);
            garudaData.focSubState = 99;
            goto foc_slow_loop;
#elif FOC_DIAG_PWM_TEST == 2
            /* ── DIAG 2: Open-loop voltage — tests current sensing ─
             * Applies fixed Vq=0.2V at θ=0 (no PI, no feedback).
             * Expected: Iq ≈ Vq/Rs = 0.2/0.065 = 3.1A (POSITIVE).
             * If Iq is NEGATIVE → current sense polarity is inverted.
             * If Iq ≈ 0 → current sensing not working.
             * focSubState=98 confirms this code path. */
            {
                float vq_test = 0.2f;  /* 0.2V → ~3A on A2212 */
                float theta_test = 0.0f;

                /* Read + convert currents for telemetry */
                AlphaBeta_t iab_diag;
                clarke_transform(ia, ib, &iab_diag);
                DQ_t idq_diag;
                park_transform(&iab_diag, theta_test, &idq_diag);

                /* Fixed voltage — no PI */
                DQ_t vdq_diag = { .d = 0.0f, .q = vq_test };
                AlphaBeta_t vab_diag;
                inv_park_transform(&vdq_diag, theta_test, &vab_diag);
                float da2, db2, dc2;
                svpwm_update(vab_diag.alpha, vab_diag.beta, s_vbus, &da2, &db2, &dc2);
                HAL_PWM_SetDutyFloat3Phase(da2, db2, dc2);

                /* Telemetry: pack id/iq into focIa/focIb for easy reading */
                garudaData.focIa    = idq_diag.d;  /* expect ~0 */
                garudaData.focIb    = idq_diag.q;  /* expect ~+3A if polarity OK */
                garudaData.focTheta = theta_test;
                garudaData.focVbus  = s_vbus;
                garudaData.focSubState = 98;
            }
            goto foc_slow_loop;
#elif FOC_DIAG_PWM_TEST == 3
            /* ── DIAG 3: FOC with PI, no feedforward ────────────── */
            /* Falls through to normal FOC but feedforward is zeroed below */
#endif

            /* ── Angle + Current Reference Management ──────────────── */
            bool in_align = (s_align_ctr < STARTUP_ALIGN_TICKS);
            float theta_drive;
            float id_ref = 0.0f;
            float iq_ref;

            if (in_align) {
                /* Phase 1: Alignment at θ=0 with gradual Iq ramp.
                 * Ramp prevents L/R transient spike (L/R=0.46ms for A2212).
                 * Iq at θ=0 creates field at 90° elec → rotor locks there. */
                s_align_ctr++;
                s_theta_ol = 0.0f;
                s_omega_ol = 0.0f;
                theta_drive = 0.0f;
                float align_frac = (float)s_align_ctr / (float)STARTUP_ALIGN_TICKS;
                iq_ref = STARTUP_ALIGN_IQ_A * align_frac;

            } else if (!s_cl_active) {
                /* Phase 2: I/f open-loop — forced angle + PI current control.
                 * POT controls speed target, slew-rate limited. PI maintains
                 * Id=0 and Iq=RAMP_IQ. No V/f formula — PI computes voltage. */
                float pot_target;
                if (throttle_raw < THROTTLE_DEADBAND) {
                    pot_target = 0.0f;
                } else {
                    float pot_frac = (float)(throttle_raw - THROTTLE_DEADBAND)
                                   / (4095.0f - (float)THROTTLE_DEADBAND);
                    pot_target = pot_frac * STARTUP_MAX_OL_RAD_S;
                }

                /* Slew-rate limited speed ramp */
                float max_delta = STARTUP_RAMP_RATE_RPS2 * FOC_TS_FAST_S;
                if (s_omega_ol < pot_target) {
                    s_omega_ol += max_delta;
                    if (s_omega_ol > pot_target) s_omega_ol = pot_target;
                } else if (s_omega_ol > pot_target) {
                    s_omega_ol -= max_delta;
                    if (s_omega_ol < pot_target) s_omega_ol = pot_target;
                }

                /* Advance forced angle */
                s_theta_ol += s_omega_ol * FOC_TS_FAST_S;
                if (s_theta_ol >= 6.28318530718f) s_theta_ol -= 6.28318530718f;
                theta_drive = s_theta_ol;

                /* I/f current: ramp from ALIGN_IQ to RAMP_IQ over IQ_RAMP_TICKS */
                if (s_iq_ramp_ctr < STARTUP_IQ_RAMP_TICKS) {
                    s_iq_ramp_ctr++;
                    float frac = (float)s_iq_ramp_ctr / (float)STARTUP_IQ_RAMP_TICKS;
                    iq_ref = STARTUP_ALIGN_IQ_A
                           + (STARTUP_RAMP_IQ_A - STARTUP_ALIGN_IQ_A) * frac;
                } else {
                    iq_ref = STARTUP_RAMP_IQ_A;
                }

#if FEATURE_CLOSED_LOOP
                /* PLL angle correction + CL transition check.
                 * Observer lag compensation: BEMF EMA introduces phase lag
                 * that makes PLL trail the true rotor. Compensate forward. */
                if (s_omega_ol >= STARTUP_HANDOFF_RAD_S) {
                    float pll_rotor = pll_rotor_angle();

                    /* Compensate observer EMA phase lag */
                    float lag_comp = OBS_LAG_COEFF * s_omega_ol * FOC_TS_FAST_S;
                    if (lag_comp > OBS_LAG_COMP_MAX_RAD) lag_comp = OBS_LAG_COMP_MAX_RAD;
                    pll_rotor += lag_comp;
                    if (pll_rotor >= 6.28318530718f) pll_rotor -= 6.28318530718f;

                    float err = pll_rotor - s_theta_ol;
                    if (err >  3.14159265f) err -= 6.28318530718f;
                    if (err < -3.14159265f) err += 6.28318530718f;

                    /* Nudge forced angle toward PLL */
                    s_theta_ol += PLL_CORRECTION_GAIN * err;
                    if (s_theta_ol < 0.0f)            s_theta_ol += 6.28318530718f;
                    if (s_theta_ol >= 6.28318530718f) s_theta_ol -= 6.28318530718f;

                    /* CL transition: PLL tracks OL within tolerance */
                    if (fabsf(err) < CL_ANGLE_TOL_RAD) {
                        if (++s_cl_lock_ctr >= CL_LOCK_COUNT) {
                            s_cl_active = true;
                            /* Bumpless transfer: seed speed PI integrator
                             * so its output matches current Iq reference.
                             * PI output = Kp*err + integrator, err≈0 at lock
                             * → integrator ≈ output ≈ iq_ref. */
                            s_pid_spd.integrator = iq_ref;
                        }
                    } else {
                        s_cl_lock_ctr = 0;
                    }
                }
#endif
            } else {
                /* Phase 3: Full closed-loop — angle + speed PI.
                 * MXLEMMING provides angle (no PLL pi/2 offset, no EMA lag).
                 * PLL provides speed (inherently filtered by PI loop filter;
                 * MXLEMMING dtheta/dt is too noisy for speed PI feedback). */
#if FEATURE_MXLEMMING
                theta_drive = s_mxl.theta_est;
#else
                theta_drive = pll_rotor_angle();
#endif
                float cl_omega = s_pll.omega_est;

                /* Speed reference from pot */
                float speed_ref;
                if (throttle_raw < THROTTLE_DEADBAND) {
                    speed_ref = 0.0f;
                } else {
                    float pot_frac = (float)(throttle_raw - THROTTLE_DEADBAND)
                                   / (4095.0f - (float)THROTTLE_DEADBAND);
                    speed_ref = pot_frac * MOTOR_MAX_ELEC_RAD_S;
                }

                /* Speed PI → Iq reference */
                iq_ref = pi_update(&s_pid_spd, speed_ref - cl_omega,
                                   FOC_TS_FAST_S);

                /* Track speed for telemetry */
                s_omega_ol = cl_omega;
            }

            /* ── FOC Pipeline: Clarke → Park → PI → InvPark → SVPWM ── */
            AlphaBeta_t iab;
            clarke_transform(ia, ib, &iab);

            DQ_t idq;
            park_transform(&iab, theta_drive, &idq);

            DQ_t vdq;
            vdq.d = pi_update(&s_pid_d, id_ref - idq.d, FOC_TS_FAST_S);
            vdq.q = pi_update(&s_pid_q, iq_ref - idq.q, FOC_TS_FAST_S);

            /* ── BEMF + cross-coupling feedforward ──────────────────── */
            {
#if FOC_DIAG_PWM_TEST == 3
                float omega_ff = 0.0f;  /* DIAG 3: feedforward disabled */
#else
                /* Always use PLL omega for feedforward — it's inherently
                 * filtered (PI loop filter), so no voltage spikes from
                 * MXLEMMING dtheta/dt noise at low flux. */
                float omega_ff = s_cl_active ? s_pll.omega_est : s_omega_ol;
#endif

                /* BEMF feedforward on q-axis */
                float bemf_ff = omega_ff * MOTOR_KE_VPEAK;
                /* Cross-coupling decoupling */
                float vd_decouple = -idq.q * omega_ff * MOTOR_LS_H;
                float vq_decouple = +idq.d * omega_ff * MOTOR_LS_H;

                vdq.d += vd_decouple;
                vdq.q += bemf_ff + vq_decouple;

                /* Circular voltage limiting (d-axis priority, Vbus/sqrt(3)) */
                float vmax = s_vbus * 0.95f * 0.577350269f;
                if (vdq.d >  vmax) vdq.d =  vmax;
                if (vdq.d < -vmax) vdq.d = -vmax;
                float sq = vmax * vmax - vdq.d * vdq.d;
                if (sq < 0.0f) sq = 0.0f;  /* FP rounding safety */
                float vq_lim = sqrtf(sq);
                if (vdq.q >  vq_lim) vdq.q =  vq_lim;
                if (vdq.q < -vq_lim) vdq.q = -vq_lim;
            }

            AlphaBeta_t vab;
            inv_park_transform(&vdq, theta_drive, &vab);

            float da, db, dc;
            svpwm_update(vab.alpha, vab.beta, s_vbus, &da, &db, &dc);
            HAL_PWM_SetDutyFloat3Phase(da, db, dc);

            /* Telemetry */
            garudaData.focIa = ia;
            garudaData.focIb = ib;
            {
                float ic_phase = -(ia + ib);
                garudaData.focIdcEst = da * ia + db * ib + dc * ic_phase;
            }

            /* Observer chain — uses shared iab and vab.
             * PLL always runs (needed for OL→CL lock detection).
             * MXLEMMING runs in parallel; drives CL angle when active. */
            {
                bemf_obs_update(&s_obs,
                                vab.alpha, vab.beta,
                                iab.alpha, iab.beta,
                                FOC_TS_FAST_S);
                pll_update(&s_pll, s_obs.e_alpha, s_obs.e_beta, FOC_TS_FAST_S);
                flux_est_update(&s_flux_est, s_obs.e_alpha, s_obs.e_beta, FOC_TS_FAST_S);
#if FEATURE_SMO
                smo_update(&s_smo,
                           vab.alpha, vab.beta,
                           iab.alpha, iab.beta);
                pll_update(&s_pll_smo, s_smo.e_alpha, s_smo.e_beta, FOC_TS_FAST_S);
#endif
#if FEATURE_MXLEMMING
                mxl_update(&s_mxl,
                           vab.alpha, vab.beta,
                           iab.alpha, iab.beta,
                           FOC_TS_FAST_S);
#endif
            }

            /* Update telemetry for GSP/GUI.
             * focSubState: 0=align, 1=OL ramp, 2=OL+PLL tracking, 3=CL */
            garudaData.focTheta    = theta_drive;
            garudaData.focOmega    = s_pll.omega_est;
            garudaData.focVbus     = s_vbus;
#if FEATURE_MXLEMMING
            garudaData.focTheta2   = s_mxl.theta_est;
#else
            garudaData.focTheta2   = pll_rotor_angle();
#endif
            garudaData.focSubState = in_align ? 0
                                   : s_cl_active ? 3
                                   : (s_omega_ol >= STARTUP_HANDOFF_RAD_S ? 2 : 1);
            garudaData.focOffsetIa = s_ia_offset;
            garudaData.focOffsetIb = s_ib_offset;
#if FEATURE_SMO
            garudaData.focSmoTheta = smo_rotor_angle();
            garudaData.focSmoOmega = s_pll_smo.omega_est;
#endif

            /* Phase current overcurrent protection (ibusRaw=0 at PWM valley).
             * Use |Ia|+|Ib| > FAULT_OC_A as fast proxy for peak phase current.
             * Debounce 2ms to reject switching transients. */
            {
                float i_mag = fabsf(ia) + fabsf(ib);
                if (i_mag > FAULT_OC_A) {
                    if (++s_foc_oc_ctr >= FOC_OC_DEBOUNCE) {
                        HAL_MC1PWMDisableOutputs();
                        garudaData.state     = ESC_FAULT;
                        garudaData.faultCode = FAULT_OVERCURRENT;
                        garudaData.runCommandActive = false;
                        LED2 = 0;
                        s_foc_oc_ctr = 0;
                    }
                } else {
                    s_foc_oc_ctr = 0;
                }
            }

            /* Stall detection — only when throttle demands motion.
             * In I/f: stall = PLL speed near zero while forced angle advancing. */
            if (throttle_raw >= THROTTLE_DEADBAND &&
                !in_align &&
                fabsf(s_pll.omega_est) < FAULT_STALL_RAD_S &&
                s_omega_ol > STARTUP_HANDOFF_RAD_S) {
                if (++s_stall_ctr >= (uint32_t)(FAULT_STALL_TIMEOUT_MS * FOC_SLOW_DIV)) {
                    HAL_MC1PWMDisableOutputs();
                    garudaData.state     = ESC_FAULT;
                    garudaData.faultCode = FAULT_STALL;
                    garudaData.runCommandActive = false;
                    LED2 = 0;
                    s_stall_ctr = 0;
                }
            } else {
                s_stall_ctr = 0;
            }
        }
        else if (state == ESC_FAULT)
        {
            HAL_MC1PWMDisableOutputs();
        }
        /* ESC_IDLE, ESC_ARMED: handled above (offset cal + goto slow loop) */

    foc_slow_loop:
        /* ── Slow loop (every FOC_SLOW_DIV ticks → 1 kHz) ────────────── */
        if (++s_slow_ctr >= FOC_SLOW_DIV)
        {
            s_slow_ctr = 0;
            state = garudaData.state;  /* Re-read after fast loop may have changed it */

            /* Arming counter — moved from Timer1 ISR to avoid cross-ISR races */
            if (state == ESC_ARMED)
            {
                if (throttle_raw < THROTTLE_DEADBAND) {
                    if (++garudaData.armCounter >= (uint32_t)FOC_ARM_TIME_MS) {
                        /* Throttle held at zero for ARM_TIME_MS → enter FOC */
                        foc_startup_reset();
                        bemf_obs_reset(&s_obs);
                        pll_reset(&s_pll);
#if FEATURE_SMO
                        smo_reset(&s_smo);
                        pll_reset(&s_pll_smo);
#endif
                        /* Pre-load center duty before releasing overrides */
                        HAL_PWM_SetDutyFloat3Phase(0.5f, 0.5f, 0.5f);
                        garudaData.state = ESC_CLOSED_LOOP;
                        LED2 = 1;
                    }
                } else {
                    garudaData.armCounter = 0;
                }
            }
        }
    } /* end FOC scope */
#elif FEATURE_FOC_V2
    /* ── FOC v2 control path — closed-loop current control ──────────
     * Architecture: MXLEMMING flux observer + Tustin PI + SVPWM.
     * True Id/Iq current control from tick 0. No V/f waste.
     * State machine: IDLE → ARMED → ALIGN → IF_RAMP → CLOSED_LOOP
     */
    {
        /* Sync mode from main loop state changes */
        if (garudaData.state == ESC_DETECT && s_foc_v2.mode == FOC_IDLE) {
            /* Auto-detect triggered from main loop (GSP command) */
            foc_detect_start(&s_foc_v2);
        } else if (garudaData.state == ESC_ARMED && s_foc_v2.mode == FOC_IDLE) {
            s_foc_v2.mode = FOC_ARMED;
            s_foc_v2.arm_ctr = 0;
        } else if (garudaData.state == ESC_IDLE &&
                   s_foc_v2.mode != FOC_IDLE &&
                   s_foc_v2.mode != FOC_MOTOR_DETECT) {
            s_foc_v2.mode = FOC_IDLE;
            /* Reset calibration for next run */
            s_foc_v2.cal_done = false;
            s_foc_v2.cal_count = 0;
            s_foc_v2.cal_accum_a = 0;
            s_foc_v2.cal_accum_b = 0;
        }

        /* Re-init FOC with updated motor params (after profile load) */
        if (gspFocReinitNeeded && s_foc_v2.mode == FOC_IDLE) {
            FOC_MotorParams_t mp = BuildFocMotorParams();
            foc_v2_init(&s_foc_v2, &mp);
            gspFocReinitNeeded = false;
        }

        /* FOC v2 uses raw_ia/raw_ib already read at ISR entry */
        uint16_t throttle_v2 = garudaData.throttle;

        float da_v2, db_v2, dc_v2;
        foc_v2_fast_tick(&s_foc_v2,
                         raw_ia, raw_ib,
                         garudaData.vbusRaw, throttle_v2,
                         &da_v2, &db_v2, &dc_v2);

        /* Release PWM overrides when entering active modes */
        {
            static bool v2_ovr_released = false;
            if ((s_foc_v2.mode >= FOC_MOTOR_DETECT && s_foc_v2.mode <= FOC_CLOSED_LOOP)) {
                if (!v2_ovr_released) {
                    HAL_PWM_ReleaseAllOverrides();
                    v2_ovr_released = true;
                }
                HAL_PWM_SetDutyFloat3Phase(da_v2, db_v2, dc_v2);
            } else if (s_foc_v2.mode == FOC_FAULT && v2_ovr_released) {
                /* Fault: force 50% duty (zero net voltage) to stop motor */
                HAL_PWM_SetDutyFloat3Phase(0.5f, 0.5f, 0.5f);
            } else {
                v2_ovr_released = false;
            }
        }

        /* After detect completes: write ALL detected + derived params back
         * to gspParams BEFORE state mapping (ESC_DETECT → ESC_ARMED).
         * This ensures SAVE_CONFIG persists a complete, self-consistent set
         * that produces the same FOC behavior after MCU reset. */
        if (s_foc_v2.mode == FOC_ARMED && garudaData.state == ESC_DETECT) {
            /* Motor params */
            gspParams.focRsMilliOhm    = (uint16_t)(s_foc_v2.Rs * 1000.0f + 0.5f);
            gspParams.focLsMicroH      = (uint16_t)(s_foc_v2.Ls * 1e6f + 0.5f);
            gspParams.focKeUvSRad      = (uint16_t)(s_foc_v2.Ke * 1e6f + 0.5f);
            /* PI gains (pole cancellation from detected Rs, Ls) */
            gspParams.focKpDqMilli     = (uint16_t)(s_foc_v2.pid_d.kp * 1000.0f + 0.5f);
            gspParams.focKiDq          = (uint16_t)(s_foc_v2.pid_d.ki + 0.5f);
            /* Max speed from Vbus/Ke */
            gspParams.focMaxElecRadS   = (uint16_t)(s_foc_v2.max_elec_rad_s + 0.5f);
            /* Startup params */
            gspParams.focHandoffRadS   = (uint16_t)(s_foc_v2.handoff_rad_s + 0.5f);
            gspParams.focRampRateRps2  = (uint16_t)(s_foc_v2.ramp_rate + 0.5f);
            gspParams.focAlignIqCentiA = (uint16_t)(s_foc_v2.align_iq * 100.0f + 0.5f);
            gspParams.focRampIqCentiA  = (uint16_t)(s_foc_v2.ramp_iq * 100.0f + 0.5f);
            gspParams.focFaultOcCentiA = (uint16_t)(s_foc_v2.fault_oc_a * 100.0f + 0.5f);
        }

        /* Map FOC v2 mode to ESC state for main loop compatibility.
         * Don't overwrite ESC_DETECT when FOC is still IDLE — main loop
         * sets ESC_DETECT, ISR must not clobber it before processing. */
        switch (s_foc_v2.mode) {
            case FOC_IDLE:
                if (garudaData.state != ESC_DETECT)
                    garudaData.state = ESC_IDLE;
                break;
            case FOC_ARMED:        garudaData.state = ESC_ARMED; break;
            case FOC_MOTOR_DETECT: garudaData.state = ESC_DETECT; break;
            case FOC_ALIGN:        garudaData.state = ESC_ALIGN; break;
            case FOC_IF_RAMP:      garudaData.state = ESC_OL_RAMP; break;
            case FOC_CLOSED_LOOP:  garudaData.state = ESC_CLOSED_LOOP; break;
            case FOC_FAULT:        garudaData.state = ESC_FAULT; break;
            default:               garudaData.state = ESC_FAULT; break;
        }

        /* Fault propagation */
        if (s_foc_v2.mode == FOC_FAULT) {
            HAL_MC1PWMDisableOutputs();
            if (garudaData.faultCode == FAULT_NONE)
                garudaData.faultCode = s_foc_v2.fault_code ? s_foc_v2.fault_code : FAULT_FOC_INTERNAL;
            garudaData.runCommandActive = false;
            LED2 = 0;
        }

        /* Telemetry update */
#if FEATURE_FOC_V2
        garudaData.focIdMeas   = s_foc_v2.id_meas;
        garudaData.focIqMeas   = s_foc_v2.iq_meas;
        garudaData.focTheta    = (s_foc_v2.mode == FOC_MOTOR_DETECT)
                                   ? s_foc_v2.theta_if : s_foc_v2.theta;
        garudaData.focOmega    = (s_foc_v2.mode == FOC_MOTOR_DETECT)
                                   ? s_foc_v2.omega_if : s_foc_v2.omega_pll;
        garudaData.focVbus     = s_foc_v2.vbus;
        garudaData.focIa       = v2_counts_to_amps(raw_ia, (uint16_t)s_foc_v2.ia_offset);
        garudaData.focIb       = v2_counts_to_amps(raw_ib, (uint16_t)s_foc_v2.ib_offset);
        garudaData.focThetaObs = s_foc_v2.theta_obs;
        garudaData.focVd       = s_foc_v2.vd;
        garudaData.focVq       = s_foc_v2.vq;
        /* Observer internals */
        garudaData.focFluxAlpha   = s_foc_v2.obs.x1;
        garudaData.focFluxBeta    = s_foc_v2.obs.x2;
        garudaData.focLambdaEst   = s_foc_v2.obs.lambda_est;
        garudaData.focObsGain     = s_foc_v2.obs.gain;
        /* PI controller internals */
        garudaData.focPidDInteg   = s_foc_v2.pid_d.integral;
        garudaData.focPidQInteg   = s_foc_v2.pid_q.integral;
        garudaData.focPidSpdInteg = s_foc_v2.pid_spd.integral;
        /* Derived diagnostics */
        {
            float vd = s_foc_v2.vd, vq = s_foc_v2.vq;
            float vbus = s_foc_v2.vbus;
            float v_mag = sqrtf(vd * vd + vq * vq);
            float v_max = vbus * 0.57735027f;  /* 1/sqrt(3) */
            garudaData.focModIndex = (v_max > 0.1f) ? (v_mag / v_max) : 0.0f;

            float fx = s_foc_v2.obs.x1, fy = s_foc_v2.obs.x2;
            float flux_mag = sqrtf(fx * fx + fy * fy);
            float lam = s_foc_v2.lambda_pm;
            garudaData.focObsConfidence = (lam > 0.0f)
                ? (1.0f - fabsf(flux_mag - lam) / lam) : 0.0f;
            if (garudaData.focObsConfidence < 0.0f)
                garudaData.focObsConfidence = 0.0f;
        }
        garudaData.focSubState = (s_foc_v2.mode == FOC_MOTOR_DETECT)
                                   ? (uint8_t)s_foc_v2.detect_state
                                   : s_foc_v2.sub_state;
        garudaData.focOffsetIa = (uint16_t)s_foc_v2.ia_offset;
        garudaData.focOffsetIb = (uint16_t)s_foc_v2.ib_offset;

        /* Populate duty for telemetry: max of 3 SVPWM phase duties */
        {
            float dmax = da_v2;
            if (db_v2 > dmax) dmax = db_v2;
            if (dc_v2 > dmax) dmax = dc_v2;
            garudaData.duty = (uint32_t)(dmax * LOOPTIME_TCY);
        }

#if FEATURE_BURST_SCOPE
        /* Write burst scope sample from ISR-local FOC state */
        {
            SCOPE_SAMPLE_T ss;
            ss.ia    = (int16_t)(garudaData.focIa * 1000.0f);
            ss.ib    = (int16_t)(garudaData.focIb * 1000.0f);
            ss.id    = (int16_t)(s_foc_v2.id_meas * 1000.0f);
            ss.iq    = (int16_t)(s_foc_v2.iq_meas * 1000.0f);
            ss.vd    = (int16_t)(s_foc_v2.vd * 100.0f);
            ss.vq    = (int16_t)(s_foc_v2.vq * 100.0f);
            ss.theta = (int16_t)(s_foc_v2.theta * 10000.0f);
            ss.obs_x1 = (int16_t)(s_foc_v2.obs.x1 * 100000.0f);
            ss.obs_x2 = (int16_t)(s_foc_v2.obs.x2 * 100000.0f);
            /* Omega ×1 scaling (1 rad/s resolution, range ±32767).
             * ×10 overflows int16 at 3276 rad/s — A2212 reaches 6000+. */
            {
                float omega_clamped = s_foc_v2.omega_pll;
                if (omega_clamped > 32767.0f) omega_clamped = 32767.0f;
                if (omega_clamped < -32768.0f) omega_clamped = -32768.0f;
                ss.omega = (int16_t)(omega_clamped);
            }
            ss.mod_index = (int16_t)(garudaData.focModIndex * 10000.0f);
            ss.flags  = (s_foc_v2.cl_active ? 0x01 : 0x00)
                      | ((garudaData.state == ESC_FAULT) ? 0x02 : 0x00)
                      | (((uint8_t)s_foc_v2.mode & 0x07) << 2);
            ss.state  = (uint8_t)garudaData.state;
            ss.tick_lsb = (uint16_t)(garudaData.systemTick & 0xFFFF);
            Scope_WriteSample(&ss);
        }
#endif
#endif
    } /* end FOC v2 scope */
#elif FEATURE_FOC_V3
    /* ── FOC v3 control path — SMO observer + OL ramp startup ──────
     * Architecture: Sliding Mode Observer + Tustin PI + SVPWM.
     * State machine: IDLE → ARMED → ALIGN → OL_RAMP → CLOSED_LOOP
     */
    {
        /* Sync mode from main loop state changes */
        if (garudaData.state == ESC_ARMED && s_foc_v3.mode == V3_IDLE) {
            s_foc_v3.mode = V3_ARMED;
            s_foc_v3.arm_ctr = 0;
        } else if (garudaData.state == ESC_IDLE &&
                   s_foc_v3.mode != V3_IDLE) {
            s_foc_v3.mode = V3_IDLE;
            s_foc_v3.cal_done = false;
            s_foc_v3.cal_count = 0;
            s_foc_v3.cal_accum_a = 0;
            s_foc_v3.cal_accum_b = 0;
        }

        /* Re-init FOC with updated motor params (after profile load) */
        if (gspFocReinitNeeded && s_foc_v3.mode == V3_IDLE) {
            FOC_MotorParams_t mp = BuildFocMotorParams();
            foc_v3_init(&s_foc_v3, &mp);
            gspFocReinitNeeded = false;
        }

        uint16_t throttle_v3 = garudaData.throttle;

        float da_v3, db_v3, dc_v3;
        foc_v3_fast_tick(&s_foc_v3,
                         raw_ia, raw_ib,
                         garudaData.vbusRaw, throttle_v3,
                         &da_v3, &db_v3, &dc_v3);

        /* Release PWM overrides when entering active modes */
        {
            static bool v3_ovr_released = false;
            if (s_foc_v3.mode >= V3_ALIGN && s_foc_v3.mode <= V3_VF_ASSIST) {
                if (!v3_ovr_released) {
                    HAL_PWM_ReleaseAllOverrides();
                    v3_ovr_released = true;
                }
                HAL_PWM_SetDutyFloat3Phase(da_v3, db_v3, dc_v3);
            } else if (s_foc_v3.mode == V3_FAULT && v3_ovr_released) {
                HAL_PWM_SetDutyFloat3Phase(0.5f, 0.5f, 0.5f);
            } else {
                v3_ovr_released = false;
            }
        }

        /* Map FOC v3 mode to ESC state for main loop compatibility */
        switch (s_foc_v3.mode) {
            case V3_IDLE:         garudaData.state = ESC_IDLE; break;
            case V3_ARMED:        garudaData.state = ESC_ARMED; break;
            case V3_ALIGN:        garudaData.state = ESC_ALIGN; break;
            case V3_OL_RAMP:      garudaData.state = ESC_OL_RAMP; break;
            case V3_CLOSED_LOOP:  garudaData.state = ESC_CLOSED_LOOP; break;
            case V3_VF_ASSIST:    garudaData.state = ESC_OL_RAMP; break;  /* Map to OL for compat */
            case V3_FAULT:        garudaData.state = ESC_FAULT; break;
            default:              garudaData.state = ESC_FAULT; break;
        }

        /* Fault propagation */
        if (s_foc_v3.mode == V3_FAULT) {
            HAL_MC1PWMDisableOutputs();
            if (garudaData.faultCode == FAULT_NONE)
                garudaData.faultCode = s_foc_v3.fault_code ? s_foc_v3.fault_code : FAULT_FOC_INTERNAL;
            garudaData.runCommandActive = false;
            LED2 = 0;
        }

        /* Telemetry update */
        garudaData.focIdMeas   = s_foc_v3.id_meas;
        garudaData.focIqMeas   = s_foc_v3.iq_meas;
        garudaData.focTheta    = s_foc_v3.theta;
        garudaData.focOmega    = s_foc_v3.omega_pll;
        garudaData.focVbus     = s_foc_v3.vbus;
        garudaData.focIa       = v2_counts_to_amps(raw_ia, (uint16_t)s_foc_v3.ia_offset);
        garudaData.focIb       = v2_counts_to_amps(raw_ib, (uint16_t)s_foc_v3.ib_offset);
        garudaData.focThetaObs = s_foc_v3.theta_obs;
        garudaData.focVd       = s_foc_v3.vd;
        garudaData.focVq       = s_foc_v3.vq;
        /* SMO internals → flux telemetry fields (repurposed) */
        garudaData.focFluxAlpha   = s_foc_v3.smo.e2_alpha;
        garudaData.focFluxBeta    = s_foc_v3.smo.e2_beta;
        garudaData.focLambdaEst   = s_foc_v3.omega;  /* LP-filtered speed (speed PI feedback) */
        garudaData.focObsGain     = s_foc_v3.smo.k_base;
        garudaData.focPidDInteg   = s_foc_v3.pid_d.integral;
        garudaData.focPidQInteg   = s_foc_v3.pid_q.integral;
        garudaData.focPidSpdInteg = s_foc_v3.pid_spd.integral;
        /* Derived diagnostics */
        {
            float vd = s_foc_v3.vd, vq = s_foc_v3.vq;
            float vbus = s_foc_v3.vbus;
            float v_mag = sqrtf(vd * vd + vq * vq);
            float v_max = vbus * 0.57735027f;
            garudaData.focModIndex = (v_max > 0.1f) ? (v_mag / v_max) : 0.0f;
            garudaData.focObsConfidence = s_foc_v3.smo.confidence;
        }
        garudaData.focSubState = s_foc_v3.sub_state;
        garudaData.focOffsetIa = (uint16_t)s_foc_v3.ia_offset;
        garudaData.focOffsetIb = (uint16_t)s_foc_v3.ib_offset;

        /* V4 observer diagnostics */
        garudaData.smoResidual    = s_foc_v3.smo.residual;
        garudaData.pllInnovation  = s_foc_v3.pll.innovation_lpf;
        garudaData.pllOmega       = s_foc_v3.pll.omega_est;
        garudaData.omegaOl        = (s_foc_v3.mode == V3_CLOSED_LOOP)
                                  ? s_foc_v3.omega : s_foc_v3.omega_ol;
        garudaData.handoffCtr     = (uint16_t)s_foc_v3.handoff_ctr;
        garudaData.smoObservable  = s_foc_v3.smo.observable ? 1 : 0;

        /* Duty for telemetry */
        {
            float dmax = da_v3;
            if (db_v3 > dmax) dmax = db_v3;
            if (dc_v3 > dmax) dmax = dc_v3;
            garudaData.duty = (uint32_t)(dmax * LOOPTIME_TCY);
        }

#if FEATURE_BURST_SCOPE
        /* Burst scope sample from v3 state */
        {
            SCOPE_SAMPLE_T ss;
            ss.ia    = (int16_t)(garudaData.focIa * 1000.0f);
            ss.ib    = (int16_t)(garudaData.focIb * 1000.0f);
            ss.id    = (int16_t)(s_foc_v3.id_meas * 1000.0f);
            ss.iq    = (int16_t)(s_foc_v3.iq_meas * 1000.0f);
            ss.vd    = (int16_t)(s_foc_v3.vd * 100.0f);
            ss.vq    = (int16_t)(s_foc_v3.vq * 100.0f);
            ss.theta = (int16_t)(s_foc_v3.theta * 10000.0f);
            ss.obs_x1 = (int16_t)(s_foc_v3.smo.e2_alpha * 100000.0f);
            ss.obs_x2 = (int16_t)(s_foc_v3.smo.e2_beta * 100000.0f);
            {
                float omega_clamped = s_foc_v3.omega_pll;
                if (omega_clamped > 32767.0f) omega_clamped = 32767.0f;
                if (omega_clamped < -32768.0f) omega_clamped = -32768.0f;
                ss.omega = (int16_t)(omega_clamped);
            }
            ss.mod_index = (int16_t)(garudaData.focModIndex * 10000.0f);
            ss.flags  = (s_foc_v3.cl_active ? 0x01 : 0x00)
                      | ((garudaData.state == ESC_FAULT) ? 0x02 : 0x00)
                      | (((uint8_t)s_foc_v3.mode & 0x07) << 2);
            ss.state  = (uint8_t)garudaData.state;
            ss.tick_lsb = (uint16_t)(garudaData.systemTick & 0xFFFF);
            Scope_WriteSample(&ss);
        }
#endif
    } /* end FOC v3 scope */
#elif FEATURE_FOC_AN1078
    /* ── AN1078 control path — float port of Microchip reference ──
     * State machine: STOPPED → LOCK → OPEN_LOOP → CLOSED_LOOP
     * Direct port of pmsm.c + smcpos.c.  No PLL, no v2/v3 hybridization.
     */
    {
        uint16_t throttle_an = garudaData.throttle;

        /* Arming logic — mirror v3:
         *   SW1 / GSP-run cmd → main loop sets garudaData.state = ESC_ARMED
         *   We see ESC_ARMED + cal_done + no fault → AN_MotorStart
         *   SW1 / GSP-stop or fault → main loop sets state = ESC_IDLE / ESC_FAULT
         *   We see those → AN_MotorStop
         * Throttle alone NEVER arms.  This matches v3 semantics. */

        /* Hardware fault recovery: if board PCI tripped, force-stop AN1078
         * so PI doesn't wind up against blocked PWM. */
        if (garudaData.faultCode == FAULT_BOARD_PCI &&
            s_foc_an.mode != AN_MODE_STOPPED) {
            AN_MotorStop(&s_foc_an);
        }

        /* Main-loop wants us idle → stop. */
        if ((garudaData.state == ESC_IDLE || garudaData.state == ESC_FAULT) &&
            s_foc_an.mode != AN_MODE_STOPPED) {
            AN_MotorStop(&s_foc_an);
        }

        /* Main-loop pressed SW1 / sent GSP run → state=ESC_ARMED →
         * start when calibration done and no latched board fault. */
        if (garudaData.state == ESC_ARMED &&
            s_foc_an.mode == AN_MODE_STOPPED &&
            s_foc_an.cal_done &&
            garudaData.faultCode != FAULT_BOARD_PCI) {
            AN_MotorStart(&s_foc_an);
        }

        float da_an, db_an, dc_an;
        AN_MotorFastTick(&s_foc_an,
                         raw_ia, raw_ib,
                         garudaData.vbusRaw, throttle_an,
                         &da_an, &db_an, &dc_an);

        /* PWM enable: match V2/V3 pattern (proven on this hardware).
         * Release overrides first, then write duty.  V2/V3's first PWM
         * cycle sees duty register state from prior HAL_MC1PWMDisableOutputs
         * (PWM_PDC*=0, gates LOW), then SetDuty kicks in next cycle. */
        {
            static bool an_ovr_released = false;
            if (s_foc_an.mode >= AN_MODE_LOCK && s_foc_an.mode <= AN_MODE_CLOSED_LOOP) {
                if (!an_ovr_released) {
                    HAL_PWM_ReleaseAllOverrides();
                    an_ovr_released = true;
                }
                HAL_PWM_SetDutyFloat3Phase(da_an, db_an, dc_an);
            } else if (s_foc_an.mode == AN_MODE_FAULT && an_ovr_released) {
                HAL_PWM_SetDutyFloat3Phase(0.5f, 0.5f, 0.5f);
            } else if (s_foc_an.mode == AN_MODE_STOPPED && an_ovr_released) {
                HAL_MC1PWMDisableOutputs();
                an_ovr_released = false;
            } else {
                an_ovr_released = false;
            }
        }

        /* Map AN1078 mode → ESC state.  DO NOT overwrite ESC_FAULT set
         * externally by the hardware PCI ISR — main loop needs to see
         * that fault state so SW1 fault-clear works. */
        if (garudaData.state != ESC_FAULT) {
            switch (s_foc_an.mode) {
                case AN_MODE_STOPPED:     garudaData.state = ESC_IDLE; break;
                case AN_MODE_LOCK:        garudaData.state = ESC_ALIGN; break;
                case AN_MODE_OPEN_LOOP:   garudaData.state = ESC_OL_RAMP; break;
                case AN_MODE_CLOSED_LOOP: garudaData.state = ESC_CLOSED_LOOP; break;
                case AN_MODE_FAULT:       garudaData.state = ESC_FAULT; break;
                default:                  garudaData.state = ESC_FAULT; break;
            }
        }

        /* Fault propagation */
        if (s_foc_an.mode == AN_MODE_FAULT) {
            HAL_MC1PWMDisableOutputs();
            if (garudaData.faultCode == FAULT_NONE)
                garudaData.faultCode = s_foc_an.faultCode
                                     ? s_foc_an.faultCode : FAULT_FOC_INTERNAL;
            garudaData.runCommandActive = false;
            LED2 = 0;
        }

        /* Telemetry — reuse v2/v3 fields */
        garudaData.focIdMeas   = s_foc_an.id_meas;
        garudaData.focIqMeas   = s_foc_an.iq_meas;
        garudaData.focTheta    = s_foc_an.theta_drive;
        garudaData.focOmega    = s_foc_an.smc.OmegaFltred;
        garudaData.focVbus     = s_foc_an.vbus;
        garudaData.focIa       = s_foc_an.ia;
        garudaData.focIb       = s_foc_an.ib;
        garudaData.focThetaObs = s_foc_an.smc.Theta;
        garudaData.focVd       = s_foc_an.vd;
        garudaData.focVq       = s_foc_an.vq;
        garudaData.focFluxAlpha   = s_foc_an.smc.EalphaFinal;
        garudaData.focFluxBeta    = s_foc_an.smc.EbetaFinal;
        garudaData.focLambdaEst   = s_foc_an.thetaError;
        garudaData.focObsGain     = s_foc_an.smc.Kslf;
        garudaData.focPidDInteg   = s_foc_an.pi_d.integrator;
        garudaData.focPidQInteg   = s_foc_an.pi_q.integrator;
        garudaData.focPidSpdInteg = s_foc_an.pi_spd.integrator;

        {
            float v_mag = sqrtf(s_foc_an.vd * s_foc_an.vd
                              + s_foc_an.vq * s_foc_an.vq);
            float v_max = s_foc_an.vbus * 0.57735027f;
            garudaData.focModIndex = (v_max > 0.1f) ? (v_mag / v_max) : 0.0f;

            /* Confidence-like ratio: observed BEMF magnitude vs expected */
            float bemf_mag = sqrtf(s_foc_an.smc.EalphaFinal * s_foc_an.smc.EalphaFinal
                                 + s_foc_an.smc.EbetaFinal  * s_foc_an.smc.EbetaFinal);
            float omega_abs = (s_foc_an.smc.OmegaFltred >= 0)
                            ?  s_foc_an.smc.OmegaFltred : -s_foc_an.smc.OmegaFltred;
            float bemf_exp = AN_MOTOR_LAMBDA * omega_abs;
            float conf = (bemf_exp > 0.01f) ? (bemf_mag / bemf_exp) : 0.0f;
            if (conf > 1.0f) conf = 1.0f;
            garudaData.focObsConfidence = conf;
        }
        garudaData.focSubState = (uint8_t)s_foc_an.mode;
        garudaData.focOffsetIa = (uint16_t)s_foc_an.ia_offset;
        garudaData.focOffsetIb = (uint16_t)s_foc_an.ib_offset;

        /* Duty for telemetry */
        {
            float dmax = da_an;
            if (db_an > dmax) dmax = db_an;
            if (dc_an > dmax) dmax = dc_an;
            garudaData.duty = (uint32_t)(dmax * LOOPTIME_TCY);
        }
    }
#else
    /* ── 6-step state machine ── */
    /* State machine */
    switch (garudaData.state)
    {
        case ESC_IDLE:
#if FEATURE_OC_AUTOZERO
            s_ocZeroAcc = 0;            /* re-calibrate on the next arm */
            s_ocZeroCount = 0;
#endif
            break;
        case ESC_ARMED:
#if FEATURE_OC_AUTOZERO
            /* Bridge is off in ARMED → the bus ADC reads its true rest bias.
             * Average OC_AUTOZERO_SAMPLES (~1.4ms) and latch; ALIGN starts
             * 500ms later, so calibration always completes first.
             * QUIESCENCE GATE (2026-06-10): a quick re-arm with the rotor
             * still spinning down puts regen ripple on the bus — bench showed
             * whole runs with Ibus telemetry (and OC thresholds) offset ~7A
             * from a bias latched mid-spin. Latch only if the sample window
             * is tight; otherwise discard and retry — the rotor stops and a
             * later window passes. The previous bias holds meanwhile. */
            if (s_ocZeroCount < OC_AUTOZERO_SAMPLES)
            {
                uint16_t v = s_ocZeroRaw;
                if (s_ocZeroCount == 0)
                {
                    s_ocZeroAcc = 0;
                    s_ocZeroMin = v;
                    s_ocZeroMax = v;
                }
                if (v < s_ocZeroMin) s_ocZeroMin = v;
                if (v > s_ocZeroMax) s_ocZeroMax = v;
                s_ocZeroAcc += v;
                if (++s_ocZeroCount == OC_AUTOZERO_SAMPLES)
                {
                    if ((uint16_t)(s_ocZeroMax - s_ocZeroMin)
                            <= OC_AUTOZERO_MAX_SPREAD)
                        g_ocBiasAdc =
                            (uint16_t)(s_ocZeroAcc / OC_AUTOZERO_SAMPLES);
                    else
                        s_ocZeroCount = 0;   /* ripple — retry the window */
                }
            }
#endif
            break;

#if FEATURE_SINE_STARTUP
        case ESC_ALIGN:
        case ESC_OL_RAMP:
            if (garudaData.sine.active)
            {
                uint32_t dA, dB, dC;
                STARTUP_SineComputeDuties(&garudaData, &dA, &dB, &dC);
                HAL_PWM_SetDutyCycle3Phase(dA, dB, dC);
            }
            break;

#endif /* FEATURE_SINE_STARTUP — close so ESC_IF_RAMP is handled regardless of sine */

#if FEATURE_IF_STARTUP
        case ESC_IF_RAMP:
            /* I-f current-controlled spin-up. IF_StartupInit pointed AD1CH0/
             * AD2CH0 at the OA1/OA2 current op-amps, so phaseB_val/phaseAC_val
             * hold ia/ib here. The CMP1/CMP2 removal (hal_comparator.c) un-rails
             * the op-amps that defeated the earlier attempts. */
            IF_StartupTick(phaseB_val, phaseAC_val);
            break;
#endif

#if FEATURE_SINE_STARTUP   /* reopen: the MORPH cases below are sine-specific */
        case ESC_MORPH:
        {
            if (garudaData.morph.subPhase == MORPH_CONVERGE)
            {
                /* Sub-phase A: blended duties, all 3 phases driven */
                uint32_t dA, dB, dC;
                STARTUP_MorphComputeDuties(&garudaData, &dA, &dB, &dC);

#if FEATURE_HW_OVERCURRENT
                /* Current-proportional duty reduction during convergence.
                 * Same logic as CL SW OC limiter but applied to all 3 phases. */
                if (garudaData.ibusRaw > OC_SW_LIMIT_ADC)
                {
                    uint16_t excess = garudaData.ibusRaw - OC_SW_LIMIT_ADC;
                    uint16_t range = RT_OC_CMP3_DAC_VAL - OC_SW_LIMIT_ADC;
                    if (range == 0) range = 1;
                    /* Scale factor: 256 = no reduction, 0 = full cut */
                    uint32_t scale = 256;
                    uint32_t reduction = ((uint32_t)excess * 256u) / range;
                    if (reduction >= scale)
                        scale = 0;
                    else
                        scale -= reduction;
                    dA = (dA * scale) >> 8;
                    dB = (dB * scale) >> 8;
                    dC = (dC * scale) >> 8;
                    if (dA < MIN_DUTY) dA = MIN_DUTY;
                    if (dB < MIN_DUTY) dB = MIN_DUTY;
                    if (dC < MIN_DUTY) dC = MIN_DUTY;
                }
#endif
                HAL_PWM_SetDutyCycle3Phase(dA, dB, dC);

                if (STARTUP_MorphCheckSectorBoundary(&garudaData))
                {
                    if (garudaData.morph.alpha >= 256)
                    {
                        /* Convergence complete → enter Windowed Hi-Z */
                        garudaData.morph.alpha = 256;
                        garudaData.morph.subPhase = MORPH_WINDOWED_HIZ;
                        garudaData.morph.sectorCount = 0;
                        garudaData.morph.tickInStep = 0;
                        garudaData.morph.floatIsHiZ = false;
                        garudaData.morph.morphZcCount = 0;
                        garudaData.morph.forceThreshSeed = true;
                        garudaData.sine.active = false;

                        /* Apply 6-step via commutation module, then release float */
                        COMMUTATION_ApplyStep(&garudaData,
                                              garudaData.morph.morphStep);
                        HAL_PWM_ReleaseFloatPhase(garudaData.currentStep);

                        /* COAST: hold duty at MIN_DUTY during MORPH sub-B (windowed
                         * Hi-Z) so the bridge applies near-zero voltage to the rotor
                         * while HWZC captures clean ZCs. Was: garudaData.duty = trap_duty
                         * (= sineRampModPct × 16/5 clamped to RAMP_DUTY_CAP = 8%), but
                         * 8% Vbus (1.92V) applied to a rotor at 3k eRPM (BEMF=0.31V)
                         * = 22A across 0.05Ω. Coasting at MIN_DUTY for sub-B (~13ms
                         * at 3k eRPM through 4 sectors) lets the rotor stay near 3k
                         * (windage decel negligible over 13ms) while HWZC PI converges.
                         * Then CL takes over with mappedDuty slewing up from MIN_DUTY.
                         * The trap-duty `td` is still computed below for the float-
                         * phase virtual-neutral, which still needs the trap reference. */
                        uint32_t td = ((uint32_t)garudaData.sine.amplitude
                            * SINE_TRAP_DUTY_NUM + SINE_TRAP_DUTY_DEN / 2)
                            / SINE_TRAP_DUTY_DEN;
                        if (td < MIN_DUTY) td = MIN_DUTY;
                        if (td > RT_RAMP_DUTY_CAP) td = RT_RAMP_DUTY_CAP;
                        garudaData.duty = MIN_DUTY;

                        /* Float driven at trapFloat — virtual neutral */
                        uint32_t trapFloat = (td + MIN_DUTY) / 2;
                        const COMMUTATION_STEP_T *s =
                            &commutationTable[garudaData.currentStep];
                        uint32_t dA = (s->phaseA == PHASE_PWM_ACTIVE) ? td :
                                      (s->phaseA == PHASE_FLOAT) ? trapFloat : MIN_DUTY;
                        uint32_t dB = (s->phaseB == PHASE_PWM_ACTIVE) ? td :
                                      (s->phaseB == PHASE_FLOAT) ? trapFloat : MIN_DUTY;
                        uint32_t dC = (s->phaseC == PHASE_PWM_ACTIVE) ? td :
                                      (s->phaseC == PHASE_FLOAT) ? trapFloat : MIN_DUTY;
                        HAL_PWM_SetDutyCycle3Phase(dA, dB, dC);

                        /* Seed forced timing + snapshot */
                        uint16_t handoff =
                            TIMER1_TO_ADC_TICKS(garudaData.rampStepPeriod);
                        garudaData.timing.stepPeriod = handoff;
                        garudaData.timing.forcedCountdown = handoff;
                        garudaData.morph.stepPeriodSnap = handoff;

                        BEMF_ZC_Init(&garudaData, handoff);
                        BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
                    }
                }
            }
            else if (garudaData.morph.subPhase == MORPH_WINDOWED_HIZ)
            {
                static const uint8_t windowSchedule[] = MORPH_WINDOW_SCHEDULE;

                garudaData.morph.tickInStep++;

                /* --- Window bounds from SNAPSHOT period --- */
                uint8_t schedIdx = garudaData.morph.sectorCount;
                if (schedIdx >= MORPH_WINDOW_SECTORS)
                    schedIdx = MORPH_WINDOW_SECTORS - 1;
                uint8_t pct = windowSchedule[schedIdx];
                uint16_t sp = garudaData.morph.stepPeriodSnap;
                uint16_t width = (uint16_t)(((uint32_t)sp * pct + 50) / 100);

                /* fix #4: enforce minimum absolute window width */
                if (width < MORPH_WINDOW_MIN_TICKS && pct < 100)
                    width = MORPH_WINDOW_MIN_TICKS;
                if (width > sp)
                    width = sp;

                uint16_t winOpen = (sp - width) / 2;
                uint16_t winClose = winOpen + width;

                /* --- Window state transitions --- */
                bool inWindow = (garudaData.morph.tickInStep >= winOpen
                              && garudaData.morph.tickInStep < winClose);

                /* At 100%, keep Hi-Z for entire step — no window-close */
                if (pct >= 100)
                    inWindow = true;

                bool justOpenedWindow = false;

                if (inWindow && !garudaData.morph.floatIsHiZ)
                {
                    HAL_PWM_FloatPhaseToHiZ(garudaData.currentStep);
                    garudaData.morph.floatIsHiZ = true;
                    garudaData.bemf.ad2SettleCount = ZC_AD2_SETTLE_SAMPLES;
                    justOpenedWindow = true;
                }
                else if (!inWindow && garudaData.morph.floatIsHiZ)
                {
                    HAL_PWM_ReleaseFloatPhase(garudaData.currentStep);
                    garudaData.morph.floatIsHiZ = false;
                }

                /* --- OC limiter BEFORE duty write --- */
#if FEATURE_HW_OVERCURRENT
                if (garudaData.ibusRaw > OC_SW_LIMIT_ADC)
                {
                    uint16_t excess = garudaData.ibusRaw - OC_SW_LIMIT_ADC;
                    uint16_t range = RT_OC_CMP3_DAC_VAL - OC_SW_LIMIT_ADC;
                    if (range == 0) range = 1;
                    uint32_t reduction = ((uint32_t)excess
                        * (garudaData.duty - MIN_DUTY)) / range;
                    if (reduction >= garudaData.duty - MIN_DUTY)
                        garudaData.duty = MIN_DUTY;
                    else
                        garudaData.duty -= reduction;
                }
#endif

                /* --- Duty write EVERY tick (after OC) --- */
                if (garudaData.morph.floatIsHiZ)
                    HAL_PWM_SetDutyCycle(garudaData.duty);
                else
                    MORPH_WRITE_TRAP_DUTIES(garudaData.currentStep, garudaData.duty);

                /* --- BEMF polling inside Hi-Z window (skip open tick) --- */
                if (garudaData.morph.floatIsHiZ && !justOpenedWindow)
                {
                    bool wasDetected = garudaData.bemf.zeroCrossDetected;
                    uint8_t spacing = garudaData.timing.stepsSinceLastZc;
                    BEMF_ZC_Poll(&garudaData, adcIsrTick);
                    bool newZcThisTick = (!wasDetected
                        && garudaData.bemf.zeroCrossDetected);

                    if (newZcThisTick)
                    {
                        garudaData.morph.morphZcCount++;
                        if (garudaData.morph.morphZcCount >= 2 && spacing == 1)
                        {
                            uint16_t measured = (uint16_t)(adcIsrTick
                                - garudaData.morph.lastZcTick);
                            uint16_t mHandoff =
                                TIMER1_TO_ADC_TICKS(garudaData.rampStepPeriod);
                            if (measured < mHandoff) measured = mHandoff;
                            uint16_t maxM = mHandoff + (mHandoff >> 1);
                            if (measured > maxM) measured = maxM;
                            int32_t delta = (int32_t)measured
                                - (int32_t)garudaData.timing.stepPeriod;
                            garudaData.timing.stepPeriod = (uint16_t)(
                                (int32_t)garudaData.timing.stepPeriod
                                + (delta >> 3));
                        }
                        garudaData.morph.lastZcTick = adcIsrTick;
                    }
                }

                /* --- Forced commutation --- */
                if (garudaData.timing.forcedCountdown > 0)
                    garudaData.timing.forcedCountdown--;

                if (garudaData.timing.forcedCountdown == 0)
                {
                    /* fix #1: detect terminal sector BEFORE touching overrides.
                     * On terminal step, go straight to MORPH_HIZ with Hi-Z
                     * intact — no driven→Hi-Z transient on the float phase. */
                    bool isTerminal =
                        (garudaData.morph.sectorCount + 1 >= MORPH_WINDOW_SECTORS);

                    COMMUTATION_AdvanceStep(&garudaData);
                    /* AdvanceStep → ApplyStep sets float to Hi-Z via
                     * SetCommutationStep. On non-terminal steps: release
                     * float to driven for next window. On terminal step:
                     * KEEP Hi-Z (skip release + driven write). */

                    if (!isTerminal)
                    {
                        HAL_PWM_ReleaseFloatPhase(garudaData.currentStep);
                        MORPH_WRITE_TRAP_DUTIES(garudaData.currentStep,
                                                garudaData.duty);
                        garudaData.morph.floatIsHiZ = false;
                    }
                    /* else: float stays Hi-Z from AdvanceStep → ApplyStep */

                    garudaData.morph.tickInStep = 0;
                    garudaData.timing.forcedCountdown =
                        garudaData.timing.stepPeriod;
                    garudaData.morph.stepPeriodSnap =
                        garudaData.timing.stepPeriod;
                    garudaData.morph.sectorCount++;

                    BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);

                    /* --- Schedule complete → enter full MORPH_HIZ --- */
                    if (garudaData.morph.sectorCount >= MORPH_WINDOW_SECTORS)
                    {
                        garudaData.morph.subPhase = MORPH_HIZ;
                        garudaData.morph.sectorCount = 0;
                        garudaData.morph.floatIsHiZ = true; /* fix #5 */
                        HAL_PWM_SetDutyCycle(garudaData.duty);
                    }
                }

                /* Staleness decay */
                if (garudaData.timing.stepsSinceLastZc > ZC_STALENESS_LIMIT)
                {
                    garudaData.timing.goodZcCount = 0;
                    garudaData.timing.risingZcWorks = false;
                    garudaData.timing.fallingZcWorks = false;
                }
            }
            else /* MORPH_HIZ */
            {
                /* Sub-phase C: real 6-step, BEMF on float phase */
                bool wasDetected = garudaData.bemf.zeroCrossDetected;
                uint8_t spacing = garudaData.timing.stepsSinceLastZc;
                BEMF_ZC_Poll(&garudaData, adcIsrTick);

                /* Latch "ZC confirmed this tick" BEFORE any forced
                 * commutation runs — immune to stepsSinceLastZc race. */
                bool newZcThisTick = (!wasDetected
                    && garudaData.bemf.zeroCrossDetected);

                /* Track ZC timestamps for IIR period adaptation.
                 * Guard: only trust single-step intervals (spacing==1). */
                if (newZcThisTick)
                {
                    garudaData.morph.morphZcCount++;
                    if (garudaData.morph.morphZcCount >= 2 && spacing == 1)
                    {
                        uint16_t measured = (uint16_t)(adcIsrTick
                            - garudaData.morph.lastZcTick);
                        uint16_t mHandoff =
                            TIMER1_TO_ADC_TICKS(garudaData.rampStepPeriod);
                        /* Clamp to [handoff, 1.5× handoff] */
                        if (measured < mHandoff)
                            measured = mHandoff;
                        uint16_t maxMeasured = mHandoff + (mHandoff >> 1);
                        if (measured > maxMeasured)
                            measured = maxMeasured;
                        /* IIR: 7/8 old + 1/8 measured */
                        int32_t delta = (int32_t)measured
                            - (int32_t)garudaData.timing.stepPeriod;
                        garudaData.timing.stepPeriod = (uint16_t)(
                            (int32_t)garudaData.timing.stepPeriod
                            + (delta >> 3));

#if FEATURE_MORPH_LOCK_GATE
                        /* Strict-lock tracking: count CONSECUTIVE Hi-Z ZC
                         * intervals that are STABLE (within tol% of the smoothed
                         * period) and NOT a half-period harmonic. CL must engage
                         * on a trustworthy angle, else it drives the wrong sector
                         * and regen-sloshes the rotor (the ~22A hand-off pulse).
                         * Use the RAW interval (pre-clamp) so the [handoff,1.5×]
                         * clamp above can't mask a harmonic. */
                        {
                            uint16_t rawIv = (uint16_t)(adcIsrTick
                                - garudaData.morph.lastZcTick);
                            uint16_t ref = garudaData.timing.stepPeriod;
                            uint16_t diff = (rawIv > ref)
                                ? (uint16_t)(rawIv - ref)
                                : (uint16_t)(ref - rawIv);
                            /* half-period harmonic: raw interval well below the
                             * OL hand-off period (rotor can't be 2× the OL speed) */
                            bool harmonic = (rawIv < (uint16_t)(mHandoff
                                                                - (mHandoff >> 2)));
                            bool stable = !harmonic
                                && ((uint32_t)diff * 100u
                                    <= (uint32_t)ref * RT_MORPH_LOCK_TOL_PCT);
                            if (stable) {
                                if (garudaData.morph.stableZcCount < 255u)
                                    garudaData.morph.stableZcCount++;
                            } else {
                                garudaData.morph.stableZcCount = 0;
                            }
                        }
#endif
                    }
                    garudaData.morph.lastZcTick = adcIsrTick;
                }

                /* Forced commutation (open-loop timing) */
                bool forcedStepThisTick = false;
                if (garudaData.timing.forcedCountdown > 0)
                    garudaData.timing.forcedCountdown--;

                if (garudaData.timing.forcedCountdown == 0)
                {
                    COMMUTATION_AdvanceStep(&garudaData);
                    BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
                    garudaData.timing.forcedCountdown =
                        garudaData.timing.stepPeriod;
                    garudaData.morph.sectorCount++;
                    forcedStepThisTick = true;
                }

                /* Staleness decay: mirror pre-sync pattern */
                if (garudaData.timing.stepsSinceLastZc > ZC_STALENESS_LIMIT)
                {
                    garudaData.timing.goodZcCount = 0;
                    garudaData.timing.risingZcWorks = false;
                    garudaData.timing.fallingZcWorks = false;
                }

                /* ZC confidence gate → exit morph */
                if (garudaData.timing.goodZcCount >= MORPH_ZC_THRESHOLD
                    && garudaData.timing.risingZcWorks
                    && garudaData.timing.fallingZcWorks
                    && newZcThisTick
#if FEATURE_MORPH_LOCK_GATE
                    /* ...and a trustworthy, stable, non-harmonic period lock */
                    && garudaData.morph.stableZcCount >= RT_MORPH_LOCK_ZC_COUNT
#endif
                    )
                {
                    garudaData.timing.zcSynced = true;

                    /* Seed first post-sync commutation deadline */
                    if (forcedStepThisTick)
                    {
                        garudaData.timing.commDeadline = (uint16_t)(
                            adcIsrTick + garudaData.timing.stepPeriod);
                    }
                    else if (garudaData.morph.morphZcCount >= 1)
                    {
                        garudaData.timing.commDeadline = (uint16_t)(
                            garudaData.morph.lastZcTick
                            + garudaData.timing.stepPeriod / 2);
                    }
                    else
                    {
                        garudaData.timing.commDeadline = (uint16_t)(
                            adcIsrTick + garudaData.timing.stepPeriod);
                    }
                    garudaData.timing.deadlineActive = true;
#if FEATURE_BEMF_INTEGRATION || FEATURE_ADC_CMP_ZC
                    /* Only mark as ZC-timed when deadline is anchored
                     * to an actual ZC event (not a forced-step fallback).
                     * Downstream HWZC handoff gating uses this flag. */
                    garudaData.timing.deadlineIsZc = !forcedStepThisTick;
#endif

                    garudaData.state = ESC_CLOSED_LOOP;
                    break;
                }

                /* Sector timeout */
                if (garudaData.morph.sectorCount >= MORPH_HIZ_MAX_SECTORS)
                {
                    if (garudaData.timing.goodZcCount >= 3)
                    {
                        /* Partial lock — let CL pre-sync finish.
                         * Sync rampStepPeriod from IIR-adapted stepPeriod
                         * so CL entry re-init doesn't discard morph's
                         * speed tracking and jump back to ramp-end rate.
                         * Inverse of TIMER1_TO_ADC_TICKS (rounded):
                         * t1 = adc * 10000 / PWMFREQUENCY_HZ.
                         * FIXED 2026-06-11: was *5/12 (stale 24 kHz factor). */
                        garudaData.rampStepPeriod = (uint16_t)(
                            ((uint32_t)garudaData.timing.stepPeriod * 10000UL
                             + PWMFREQUENCY_HZ / 2) / PWMFREQUENCY_HZ);
                        garudaData.state = ESC_CLOSED_LOOP;
                    }
                    else
                    {
                        HAL_MC1PWMDisableOutputs();
#if FEATURE_HW_OVERCURRENT
                        HAL_CMP3_SetThreshold(RT_OC_CMP3_STARTUP_DAC);
#endif
                        garudaData.state = ESC_FAULT;
                        garudaData.faultCode = FAULT_MORPH_TIMEOUT;
                        garudaData.runCommandActive = false;
                        LED2 = 0;
                    }
                }


#if FEATURE_HW_OVERCURRENT
                /* SW OC soft limiter — same as CL (proportional duty cut) */
                if (garudaData.ibusRaw > OC_SW_LIMIT_ADC)
                {
                    uint16_t excess = garudaData.ibusRaw - OC_SW_LIMIT_ADC;
                    uint16_t range = RT_OC_CMP3_DAC_VAL - OC_SW_LIMIT_ADC;
                    if (range == 0) range = 1;
                    uint32_t reduction = ((uint32_t)excess
                        * (garudaData.duty - MIN_DUTY)) / range;
                    if (reduction >= garudaData.duty - MIN_DUTY)
                        garudaData.duty = MIN_DUTY;
                    else
                        garudaData.duty -= reduction;
                }
#endif
                HAL_PWM_SetDutyCycle(garudaData.duty);
            }

            /* Absolute timeout — covers BOTH sub-phases.
             * If motor stalls during CONVERGE (e.g. prop overload),
             * this is the only exit path. */
            if ((garudaData.systemTick
                - garudaData.morph.entryTick) > MORPH_TIMEOUT_MS)
            {
                HAL_MC1PWMDisableOutputs();
#if FEATURE_HW_OVERCURRENT
                HAL_CMP3_SetThreshold(RT_OC_CMP3_STARTUP_DAC);
#endif
                garudaData.state = ESC_FAULT;
                garudaData.faultCode = FAULT_MORPH_TIMEOUT;
                garudaData.runCommandActive = false;
                LED2 = 0;
            }
            break;
        }
#else
        case ESC_ALIGN:
        case ESC_OL_RAMP:
            break;
#endif

#if DIAGNOSTIC_MANUAL_STEP
        case ESC_CLOSED_LOOP:
            /* Manual step mode: ADC ISR just holds duty. No automatic commutation. */
            HAL_PWM_SetDutyCycle(garudaData.duty);
            break;
#elif FEATURE_BEMF_CLOSED_LOOP
        case ESC_CLOSED_LOOP:
        {
#if FEATURE_HANDOFF_CHOP
            /* Handoff-chop arm latch: set at CL entry (below) BEFORE the
             * coast-listen block can `break` and skip the rest of the case,
             * then consumed by the handoff-chop block once driving resumes.
             * Fixes the chop never arming on the coast-listen entry path
             * (2810) — the old `prevAdcState != CL` arm was eaten by the break. */
            static bool hcArmPending = false;
#endif
            /* Throttle-zero shutdown: if pot returns to zero after being raised,
             * gracefully stop. Don't wait for desync — at low duty the HW ZC
             * comparator can trigger on noise indefinitely, keeping the motor
             * in a stalled-but-"running" state (audible buzz).
             *
             * hasSeenThrottle prevents false shutdown at CL entry — the arming
             * gate requires pot=0, so pot is still at zero when CL starts.
             * Only arm the shutdown after the user has raised the pot once. */
            {
                static bool hasSeenThrottle = false;
                static uint16_t zeroThrottleCount = 0;

                if (prevAdcState != ESC_CLOSED_LOOP)
                {
                    hasSeenThrottle = false;
                    zeroThrottleCount = 0;
#if FEATURE_HANDOFF_CHOP
                    hcArmPending = true;   /* arm entry chop before any coast-listen break */
#endif
                }

                if (garudaData.throttle >= ARM_THROTTLE_ZERO_ADC)
                    hasSeenThrottle = true;

#if FEATURE_THROTTLE_ZERO_AUTO_DISARM
                if (hasSeenThrottle && garudaData.throttle < ARM_THROTTLE_ZERO_ADC)
                {
                    if (++zeroThrottleCount >= (PWMFREQUENCY_HZ / 20))  /* 50ms */
                    {
#if FEATURE_ADC_CMP_ZC
                        if (garudaData.hwzc.enabled)
                            HWZC_Disable(&garudaData);
                        garudaData.hwzc.fallbackPending = false;
#endif
#if FEATURE_HW_OVERCURRENT
                        HAL_CMP3_SetThreshold(RT_OC_CMP3_STARTUP_DAC);
#endif
                        HAL_MC1PWMDisableOutputs();
                        garudaData.runCommandActive = false;
                        garudaData.state = ESC_IDLE;
                        LED2 = 0;
                        zeroThrottleCount = 0;
                        break;
                    }
                }
                else
                {
                    zeroThrottleCount = 0;
                }
#else
                /* Throttle-zero auto-disarm disabled — motor keeps running at
                 * CL_IDLE_DUTY when pot is at zero. Use GSP stop or power cycle
                 * to stop the motor. */
                (void)hasSeenThrottle;
                (void)zeroThrottleCount;
#endif
            }

            /* Detect first entry into CLOSED_LOOP (state transition) */
            if (prevAdcState != ESC_CLOSED_LOOP)
            {
#if FEATURE_ADC_CMP_ZC && FEATURE_HWZC_HANDOFF_DAMP
                /* Arm the hand-off period-collapse damp for the first N
                 * commutations (prevents the half-period phantom at entry). */
                garudaData.hwzc.handoffDamp = HWZC_HANDOFF_DAMP_EVENTS;
#endif
#if FEATURE_SINE_STARTUP
                /* Morph hot handoff: ZC already locked, skip re-init.
                 * Gate on prevAdcState == ESC_MORPH (not just zcSynced)
                 * to prevent stale state from a previous run triggering
                 * the hot-handoff path from OL_RAMP. */
                if (prevAdcState == ESC_MORPH
                    && garudaData.timing.zcSynced)
                {
                    garudaData.desyncRestartAttempts = 0;
#if FEATURE_HW_OVERCURRENT
                    HAL_CMP3_SetThreshold(RT_OC_CMP3_DAC_VAL);
#endif
#if FEATURE_ADC_CMP_ZC
                    garudaData.hwzc.fallbackPending = false;
                    garudaData.hwzc.enablePending = false;
                    garudaData.hwzc.dbgLatchDisable = false;
#endif
                }
                else
#endif
                {
                    /* Normal CL entry (from OL_RAMP or morph without lock) */
                    uint16_t initPeriod = TIMER1_TO_ADC_TICKS(garudaData.rampStepPeriod);
                    if (initPeriod < MIN_ADC_STEP_PERIOD)
                        initPeriod = MIN_ADC_STEP_PERIOD;
                    if (initPeriod > RT_INITIAL_ADC_STEP_PERIOD)
                        initPeriod = RT_INITIAL_ADC_STEP_PERIOD;
                    BEMF_ZC_Init(&garudaData, initPeriod);
                    BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
                    garudaData.bemf.ad2SettleCount = ZC_AD2_SETTLE_SAMPLES;
#if FEATURE_ADC_CMP_ZC
                    /* Clear stale HWZC state from any prior run */
                    garudaData.hwzc.fallbackPending = false;
                    garudaData.hwzc.enablePending = false;
                    garudaData.hwzc.dbgLatchDisable = false;

#if FEATURE_PRESYNC_RAMP
                    /* Defensively disable HWZC: stale hwzc.enabled=true from a
                     * prior run (e.g. desync recovery) would skip the entire SW
                     * pre-sync block (line ~479 gates on !hwzc.enabled). Force
                     * HWZC off so pre-sync runs. Clear fallbackPending to prevent
                     * stale HWZC state from re-seeding SW ZC. */
                    if (garudaData.hwzc.enabled)
                        HWZC_Disable(&garudaData);
                    garudaData.hwzc.fallbackPending = false;
#else
                    /* Immediate HWZC enable for motors with reliable OL ramp
                     * delivery speed (non-presync-ramp path). */
                    {
                        uint32_t curErpm = ERPM_FROM_ADC_STEP_NUM / initPeriod;
                        if (curErpm >= RT_HWZC_CROSSOVER_ERPM
#if FEATURE_CL_DIFF_IDLE
                            && !g_pwmDiffLow   /* HWZC invalid under diff drive */
#endif
                           )
                        {
                            HWZC_Enable(&garudaData);
                        }
                    }
#endif

#if FEATURE_AM32_STARTUP
                    if (g_am32EntryPending)
                    {
                        g_am32EntryPending = 0;
#if FEATURE_SINE_STARTUP
                        garudaData.sine.active = false;
#endif
                        /* One blind commutation from the UNKNOWN rotor angle
                         * (AM32 does exactly this — worst case zero torque on
                         * the first step; the listener sorts it out). */
                        COMMUTATION_ApplyStep(&garudaData,
                            (uint8_t)((garudaData.currentStep + 1u) % 6u));
                        garudaData.duty = MIN_DUTY;
                        HAL_PWM_SetDutyCycle(garudaData.duty);
                        if (!garudaData.hwzc.enabled)
                            HWZC_Enable(&garudaData);
                        /* Trust the listener from event 1: capture-driven PI
                         * immediately, no blind phase, no sync gate. */
                        garudaData.timing.zcSynced = true;
                        garudaData.timing.goodZcCount =
                            (uint16_t)RT_ZC_SYNC_THRESHOLD;
                    }
#endif
#if FEATURE_PLL_STARTUP
                    if (garudaData.hwzc.pllStartActive)
                    {
                        /* Engage 6-step from the align angle and start the
                         * blind PLL schedule: comparator armed from the
                         * first commutation, captures gated in hwzc.c. */
                        /* rotor is parked AT the align vector = step s0's
                         * center → applying s0 gives ~zero torque. Lead by
                         * one step (60°) so the schedule pulls forward. */
#if FEATURE_SINE_STARTUP
                        garudaData.sine.active = false;
                        uint8_t s0 = STARTUP_SineGetTransitionStep(&garudaData);
#else
                        uint8_t s0 = 0;   /* classic STARTUP_Align parks at step 0 */
#endif
                        COMMUTATION_ApplyStep(&garudaData, (uint8_t)((s0 + 1u) % 6u));
                        garudaData.duty = MIN_DUTY;
                        HAL_PWM_SetDutyCycle(garudaData.duty);
                        if (!garudaData.hwzc.enabled)
                            HWZC_Enable(&garudaData);
                    }
#endif
#endif
                }
            }

#if FEATURE_CL_DIFF_IDLE || FEATURE_CL_COAST_VERIFY
            /* CL entry: coast-listen first (clean-BEMF measurement of the TRUE
             * rotor sector/period), then the per-tick gate owns the bridge
             * until engage. Entry-init above has already run. HWZC stays off
             * during the coast (bridge-off signals would be garbage); in
             * verify mode it re-enables via the normal crossover logic right
             * after engage. CCW: sector math not implemented — no coast. */
            if (prevAdcState != ESC_CLOSED_LOOP)
            {
#if FEATURE_ADC_CMP_ZC
                garudaData.hwzc.enablePending = false;
                if (garudaData.hwzc.enabled)
                    HWZC_Disable(&garudaData);
#endif
                if (garudaData.direction == 0)
                    CL_CoastBegin();
#if FEATURE_CL_DIFF_IDLE && !FEATURE_CL_ENTRY_GLIDE
                else
                {
                    /* CCW: blind diff engage (steady-state diff idle only;
                     * glide mode falls through to the baseline hot hand-off) */
                    g_pwmDiffLow = 1;
                    HAL_PWM_SetCommutationStep(garudaData.currentStep);
                }
#endif
            }
            if (CL_CoastListenTick(phaseB_val, adcIsrTick))
                break;   /* coasting: bridge off, skip normal CL this tick */
#endif

#if FEATURE_ADC_CMP_ZC
            /* HW->SW fallback re-seed (Rule 9) */
            if (garudaData.hwzc.fallbackPending)
            {
                garudaData.hwzc.fallbackPending = false;
                garudaData.hwzc.enablePending = false;
                uint16_t swPeriod = HWZC_SCCP2_TO_ADC(
                    garudaData.hwzc.stepPeriodHR);
                if (swPeriod < MIN_ADC_STEP_PERIOD)
                    swPeriod = MIN_ADC_STEP_PERIOD;
                if (swPeriod > RT_INITIAL_ADC_STEP_PERIOD)
                    swPeriod = RT_INITIAL_ADC_STEP_PERIOD;
                BEMF_ZC_Init(&garudaData, swPeriod);
                BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
                garudaData.bemf.ad2SettleCount = ZC_AD2_SETTLE_SAMPLES;

                if (garudaData.hwzc.goodZcCount >= RT_ZC_SYNC_THRESHOLD)
                {
                    /* HW ZC had good lock — seed as synced to avoid
                     * pre-sync forced-commutation jerk at transition */
                    garudaData.timing.zcSynced = true;
                    garudaData.timing.goodZcCount = RT_ZC_SYNC_THRESHOLD;
                    garudaData.timing.forcedCountdown =
                        swPeriod * ZC_TIMEOUT_MULT;
#if FEATURE_HW_OVERCURRENT
                    /* Lower CMP3 from startup to operational threshold */
                    HAL_CMP3_SetThreshold(RT_OC_CMP3_DAC_VAL);
#endif
                }
                else
                {
                    /* HW ZC was struggling — conservative pre-sync */
                    garudaData.timing.zcSynced = false;
                    garudaData.timing.forcedCountdown = swPeriod;
                }
            }

            if (!garudaData.hwzc.enabled)
            {
#endif /* FEATURE_ADC_CMP_ZC */

            if (!garudaData.timing.zcSynced)
            {
                /* === PRE-SYNC: Forced commutation + passive ZC detection === */

                if (garudaData.timing.forcedCountdown > 0)
                    garudaData.timing.forcedCountdown--;

                if (garudaData.timing.forcedCountdown == 0)
                {
                    COMMUTATION_AdvanceStep(&garudaData);
                    BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);

#if FEATURE_PRESYNC_RAMP
                    /* Feedback-gated pre-sync ramp: accelerate forced commutation
                     * only when ZC evidence is strong. Requires goodZcCount >= 3
                     * AND risingZcWorks — not a single noisy edge. If motor is
                     * stale (no recent ZC), hold current speed until motor catches
                     * up and ZC resumes. Same eRPM formula as OL_RAMP. */
                    if (garudaData.timing.stepPeriod > RT_MIN_ADC_STEP_PERIOD
                        && garudaData.timing.goodZcCount >= 3
                        && garudaData.timing.risingZcWorks
                        && garudaData.timing.stepsSinceLastZc <= ZC_STALENESS_LIMIT)
                    {
                        uint32_t sp = garudaData.timing.stepPeriod;
                        uint32_t curErpm = ERPM_FROM_ADC_STEP_NUM / sp;
                        uint32_t deltaErpm = ((uint32_t)RT_RAMP_ACCEL_ERPM_PER_S * sp)
                                             / PWMFREQUENCY_HZ;
                        if (deltaErpm < 1) deltaErpm = 1;
                        uint32_t newErpm = curErpm + deltaErpm;
                        uint32_t newPeriod = ERPM_FROM_ADC_STEP_NUM / newErpm;
                        if (newPeriod < RT_MIN_ADC_STEP_PERIOD)
                            newPeriod = RT_MIN_ADC_STEP_PERIOD;
                        garudaData.timing.stepPeriod = (uint16_t)newPeriod;
                    }
#endif

                    garudaData.timing.forcedCountdown = garudaData.timing.stepPeriod;
                    garudaData.zcDiag.forcedStepPresyncCount++;
                }

#if FEATURE_PRESYNC_RAMP
                /* Pre-sync timeout: fault if ZC never achieved */
                {
                    static uint32_t presyncEntryTick = 0;
                    if (prevAdcState != ESC_CLOSED_LOOP)
                        presyncEntryTick = garudaData.systemTick;

                    if ((garudaData.systemTick - presyncEntryTick) > PRESYNC_TIMEOUT_MS)
                    {
#if FEATURE_ADC_CMP_ZC
                        if (garudaData.hwzc.enabled)
                            HWZC_Disable(&garudaData);
                        garudaData.hwzc.fallbackPending = false;
#endif
#if FEATURE_HW_OVERCURRENT
                        HAL_CMP3_SetThreshold(RT_OC_CMP3_STARTUP_DAC);
#endif
                        HAL_MC1PWMDisableOutputs();
                        garudaData.state = ESC_FAULT;
                        garudaData.faultCode = FAULT_STARTUP_TIMEOUT;
                        garudaData.runCommandActive = false;
                        LED2 = 0;
                    }
                }
#endif

                /* Passive ZC detection (builds goodZcCount, no commutation trigger) */
                BEMF_ZC_Poll(&garudaData, adcIsrTick);

                if (garudaData.timing.stepsSinceLastZc > ZC_STALENESS_LIMIT)
                    garudaData.timing.goodZcCount = 0;

                if (garudaData.timing.goodZcCount >= (uint16_t)RT_ZC_SYNC_THRESHOLD
                    && garudaData.timing.risingZcWorks)
                {
                    garudaData.timing.zcSynced = true;
                    garudaData.desyncRestartAttempts = 0;
#if FEATURE_HW_OVERCURRENT
                    /* Lower CMP3 from startup to operational threshold */
                    HAL_CMP3_SetThreshold(RT_OC_CMP3_DAC_VAL);
#endif
#if FEATURE_BEMF_INTEGRATION
                    garudaData.integ.bemfPeakSmooth = 0;
                    garudaData.integ.integral = 0;
                    garudaData.integ.stepDevMax = 0;
                    garudaData.integ.shadowFired = false;
#endif
                    if (garudaData.bemf.zeroCrossDetected)
                    {
#if FEATURE_TIMING_ADVANCE
                        uint16_t sp0 = garudaData.timing.stepPeriod;
                        uint32_t eRPM0 = ERPM_FROM_ADC_STEP_NUM / sp0;
                        uint16_t adv0;
                        if (eRPM0 <= RT_RAMP_TARGET_ERPM)
                            adv0 = TIMING_ADVANCE_MIN_DEG;
                        else if (eRPM0 >= RT_MAX_CLOSED_LOOP_ERPM)
                            adv0 = RT_TIMING_ADV_MAX_DEG;
                        else
                        {
                            uint32_t r0 = RT_MAX_CLOSED_LOOP_ERPM - RT_RAMP_TARGET_ERPM;
                            uint32_t p0 = eRPM0 - RT_RAMP_TARGET_ERPM;
                            adv0 = TIMING_ADVANCE_MIN_DEG +
                                (uint16_t)((uint32_t)(RT_TIMING_ADV_MAX_DEG - TIMING_ADVANCE_MIN_DEG)
                                           * p0 / r0);
                        }
                        uint16_t d0 = (uint16_t)((uint32_t)sp0 * (30 - adv0) / 60);
                        garudaData.timing.commDeadline = (uint16_t)(adcIsrTick + d0);
#else
                        garudaData.timing.commDeadline = (uint16_t)(
                            adcIsrTick + garudaData.timing.stepPeriod / 2);
#endif
                        garudaData.timing.deadlineActive = true;
#if FEATURE_BEMF_INTEGRATION || FEATURE_ADC_CMP_ZC
                        garudaData.timing.deadlineIsZc = true;
#endif
                    }
                    else
                    {
                        BEMF_ZC_HandleUndetectableStep(&garudaData, adcIsrTick);
                    }
                }
            }
            else
            {
                /* === POST-SYNC: ZC-driven commutation === */

#if FEATURE_ADC_CMP_ZC
                /* Crossover check + enablePending management (Rule 12).
                 * dbgLatchDisable: after first HW ZC failure, permanently
                 * block re-enable so motor stays on SW ZC and diagnostics
                 * are preserved for debugger reading. */
                if (!garudaData.hwzc.dbgLatchDisable
#if FEATURE_CL_DIFF_IDLE
                    /* Differential-low drive shifts the float's PWM-ON level up
                     * by the MIN_DUTY base — HWZC's ON-window comparator model
                     * (CMPLO from the conventional duty midpoint) is invalid
                     * there: rising fires constantly, falling never → phantom
                     * captures → lock collapse (bench 2026-06-10). Keep HWZC off
                     * in diff mode; the SW valley detector (unaffected by diff —
                     * both driven phases are grounded at the sampling point, and
                     * morph-proven at these speeds) owns ZC until the swap to
                     * conventional drive. */
                    && !g_pwmDiffLow
#endif
                   )
                {
                    uint32_t curErpm = ERPM_FROM_ADC_STEP_NUM /
                        garudaData.timing.stepPeriod;
                    if (curErpm >= RT_HWZC_CROSSOVER_ERPM)
                        garudaData.hwzc.enablePending = true;
                    else if (garudaData.hwzc.enablePending
                             && curErpm < (RT_HWZC_CROSSOVER_ERPM
                                           - HWZC_HYSTERESIS_ERPM))
                        garudaData.hwzc.enablePending = false;
                }
#if FEATURE_CL_DIFF_IDLE
                else if (g_pwmDiffLow)
                    garudaData.hwzc.enablePending = false;
#endif
#endif

                /* Poll for ZC */
                BEMF_ZC_Poll(&garudaData, adcIsrTick);

                /* Check commutation deadline */
                if (BEMF_ZC_CheckDeadline(&garudaData, adcIsrTick))
                {
#if FEATURE_BEMF_INTEGRATION
                    if (!garudaData.timing.deadlineIsZc)
                    {
                        garudaData.integ.shadowSkipCount++;
                    }
                    else if (garudaData.integ.bemfPeakSmooth == 0)
                    {
                        garudaData.integ.shadowUnseededSkip++;
                    }
                    else
                    {
                        uint16_t tol = garudaData.timing.stepPeriod / INTEG_HIT_DIVISOR;
                        if (tol < 1) tol = 1;

                        garudaData.integ.shadowSampleCount++;
                        garudaData.integ.shadowStepPeriodSum +=
                            garudaData.timing.stepPeriod;

                        if (garudaData.integ.shadowFired)
                        {
                            int16_t diff = (int16_t)(garudaData.integ.shadowFireTick
                                                   - adcIsrTick);
                            garudaData.integ.shadowVsActual = diff;

                            {
                                int64_t wide = (int64_t)garudaData.integ.shadowErrorSum + diff;
                                if (wide < INT32_MIN) wide = INT32_MIN;
                                else if (wide > INT32_MAX) wide = INT32_MAX;
                                garudaData.integ.shadowErrorSum = (int32_t)wide;
                            }

                            int16_t absDiff = (diff < 0) ? -diff : diff;
                            garudaData.integ.shadowAbsErrorSum += (uint32_t)absDiff;

                            if (absDiff <= (int16_t)tol)
                                garudaData.integ.shadowHitCount++;
                            else
                                garudaData.integ.shadowMissCount++;
                        }
                        else
                        {
                            garudaData.integ.shadowNoFireCount++;
                            garudaData.integ.shadowMissCount++;
                            garudaData.integ.shadowVsActual = SHADOW_NO_FIRE_SENTINEL;
                        }
                    }
#endif
#if FEATURE_ADC_CMP_ZC
                    /* Snapshot deadlineIsZc BEFORE AdvanceStep clears it (Rule 14) */
                    bool wasZcDeadline = garudaData.timing.deadlineIsZc;
#endif
                    COMMUTATION_AdvanceStep(&garudaData);
                    BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
                    BEMF_ZC_HandleUndetectableStep(&garudaData, adcIsrTick);

#if FEATURE_ADC_CMP_ZC
                    /* enablePending hook: hand off to hardware ZC at true-ZC
                     * commutation boundary only (Rule 12) */
                    if (garudaData.hwzc.enablePending && wasZcDeadline)
                    {
                        HWZC_Enable(&garudaData);
                        goto cl_duty_control;
                    }
#endif
                }

                /* Timeout watchdog */
                ZC_TIMEOUT_RESULT_T toResult = BEMF_ZC_CheckTimeout(&garudaData, adcIsrTick);
                if (toResult == ZC_TIMEOUT_DESYNC)
                {
                    garudaData.zcDiag.zcDesyncCount++;
#if FEATURE_HW_OVERCURRENT
                    HAL_CMP3_SetThreshold(RT_OC_CMP3_STARTUP_DAC);
#endif
#if FEATURE_DESYNC_RECOVERY
                    if (garudaData.runCommandActive
#if !FEATURE_THROTTLE_ZERO_AUTO_DISARM
                        || true   /* AUTO_DISARM=0: stay in the run loop even
                                   * if some earlier path cleared the flag.
                                   * Re-arm here so the recovery exit can
                                   * restart instead of falling to IDLE. */
#endif
                       )
                    {
#if !FEATURE_THROTTLE_ZERO_AUTO_DISARM
                        garudaData.runCommandActive = true;
#endif
                        HAL_MC1PWMDisableOutputs();
                        garudaData.state = ESC_RECOVERY;
                        garudaData.recoveryCounter = RT_DESYNC_COAST_COUNTS;
                        LED2 = 0;
                    }
                    else
                    {
                        HAL_MC1PWMDisableOutputs();
                        garudaData.desyncRestartAttempts = 0;
                        garudaData.state = ESC_IDLE;
                        LED2 = 0;
                    }
#else
                    garudaData.state = ESC_FAULT;
                    garudaData.faultCode = FAULT_DESYNC;
                    HAL_MC1PWMDisableOutputs();
                    LED2 = 0;
#endif
                }
                else if (toResult == ZC_TIMEOUT_FORCE_STEP)
                {
#if FEATURE_BEMF_INTEGRATION
                    garudaData.integ.shadowSkipCount++;
#endif
                    garudaData.zcDiag.zcTimeoutForceCount++;
                    {
                        uint8_t s = garudaData.currentStep;
                        if (garudaData.timing.stepMissCount[s] < 255)
                            garudaData.timing.stepMissCount[s]++;
                    }
                    COMMUTATION_AdvanceStep(&garudaData);
                    BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
                    BEMF_ZC_HandleUndetectableStep(&garudaData, adcIsrTick);

                    if (garudaData.timing.goodZcCount == 0)
                    {
                        garudaData.timing.zcSynced = false;
                        garudaData.timing.hasPrevZc = false;
                        uint16_t initPeriod = TIMER1_TO_ADC_TICKS(garudaData.rampStepPeriod);
                        if (initPeriod < RT_MIN_ADC_STEP_PERIOD)
                            initPeriod = RT_MIN_ADC_STEP_PERIOD;
                        garudaData.timing.stepPeriod = initPeriod;
                        garudaData.timing.forcedCountdown = initPeriod;
#if FEATURE_PRESYNC_RAMP
                        /* Reset duty to ramp level to prevent ratchet:
                         * post-sync CL_IDLE floor inflates duty, and pre-sync
                         * holds mappedDuty=garudaData.duty. Without reset,
                         * each sync→unsync cycle pumps duty higher, driving
                         * massive current through low-R motor. */
                        garudaData.duty = RT_RAMP_DUTY_CAP;
#endif
                    }
                }
            }

#if FEATURE_ADC_CMP_ZC
            }
            else
            {
                /* hwzc.enabled=true: software ZC block skipped.
                 * SCCP1 ISR + comparator ISR handle commutation. */

                /* No speed-drop fallback: once HW ZC activates, it stays
                 * active at all speeds. The ADC comparator works fine at low
                 * eRPM (24kHz samples → plenty per step). Only miss-limit
                 * triggers fallback (error condition). This eliminates the
                 * HW→SW transition jerk during normal deceleration. */

                /* Promote zcSynced once HWZC has confirmed good ZCs.
                 * This unlocks pot-mapped duty (CL_IDLE_DUTY floor)
                 * and MAX_DUTY cap instead of RAMP_DUTY_CAP. */
                if (!garudaData.timing.zcSynced
#if FEATURE_PLL_STARTUP
                    /* during the PLL blind ramp, sync is declared ONLY by
                     * HWZC_PllStartTick's gated-capture handover — raw
                     * goodZcCount here counts low-speed phantoms */
                    && !garudaData.hwzc.pllStartActive
#endif
                    && garudaData.hwzc.goodZcCount >= RT_ZC_SYNC_THRESHOLD)
                {
                    garudaData.timing.zcSynced = true;
                    garudaData.desyncRestartAttempts = 0;
#if FEATURE_HW_OVERCURRENT
                    HAL_CMP3_SetThreshold(RT_OC_CMP3_DAC_VAL);
#endif
                }

                /* Plausibility stall check: if stepPeriodHR is at the IIR
                 * floor (motor apparently at max eRPM) but duty is low,
                 * the motor is physically stalled and HWZC is tracking PWM
                 * switching noise. Debounced to avoid false triggers during
                 * brief transients. Gated on zcSynced to skip post-handoff. */
                {
                    static uint16_t hwzcStallCount = 0;
                    if (prevAdcState != ESC_CLOSED_LOOP)
                        hwzcStallCount = 0;

                    if (garudaData.hwzc.stepPeriodHR <= RT_HWZC_MIN_STEP_TICKS
                        && garudaData.duty < HWZC_STALL_DUTY_LIMIT
                        && garudaData.timing.zcSynced)
                    {
                        if (++hwzcStallCount >= HWZC_STALL_DEBOUNCE_TICKS)
                        {
                            garudaData.hwzc.goodZcCount = 0;
                            garudaData.hwzc.dbgLatchDisable = true;
                            HWZC_Disable(&garudaData);
                            hwzcStallCount = 0;
                        }
                    }
                    else
                    {
                        hwzcStallCount = 0;
                    }
                }

                /* No-capture watchdog: independent of the period/duty stall
                 * check above. If hwzc.totalZcCount hasn't moved for N ticks
                 * while zcSynced is set, the motor is physically stalled
                 * (autonomous SCCP1 timer keeps commutating on the last
                 * known period but no real BEMF edges arrive). The standard
                 * BEMF_ZC_CheckTimeout doesn't see this because lastCommTick
                 * is updated by the autonomous timer's HWZC_OnCommutation
                 * call. Force DESYNC → RECOVERY here so the pot-zero idle
                 * path can recover via the regular restart cycle. */
                {
                    static uint32_t lastZcCount = 0;
                    static uint16_t noCapCount = 0;
                    if (prevAdcState != ESC_CLOSED_LOOP) {
                        lastZcCount = garudaData.hwzc.totalZcCount;
                        noCapCount = 0;
                    }
                    if (garudaData.hwzc.enabled && garudaData.timing.zcSynced)
                    {
                        if (garudaData.hwzc.totalZcCount != lastZcCount)
                        {
                            lastZcCount = garudaData.hwzc.totalZcCount;
                            noCapCount = 0;
                        }
                        else if (++noCapCount >= HWZC_NO_CAPTURE_TICKS)
                        {
                            /* Mirror the desync-handler path so the pot-zero
                             * recovery loop kicks in cleanly. */
                            garudaData.zcDiag.zcDesyncCount++;
#if FEATURE_HW_OVERCURRENT
                            HAL_CMP3_SetThreshold(RT_OC_CMP3_STARTUP_DAC);
#endif
                            HAL_MC1PWMDisableOutputs();
#if !FEATURE_THROTTLE_ZERO_AUTO_DISARM
                            garudaData.runCommandActive = true;
#endif
                            garudaData.state = ESC_RECOVERY;
                            garudaData.recoveryCounter = RT_DESYNC_COAST_COUNTS;
                            LED2 = 0;
                            noCapCount = 0;
                        }
                    }
                    else
                    {
                        noCapCount = 0;
                    }
                }

#if FEATURE_BEMF_INTEGRATION
                /* Read-only observer: keep shadow integration warm (Rule 11) */
                {
                    bool newComm = (garudaData.hwzc.commSeq
                                    != garudaData.hwzc.obsCommSeq);
                    if (newComm)
                    {
                        garudaData.hwzc.obsCommSeq =
                            garudaData.hwzc.commSeq;
                        garudaData.hwzc.obsLastCommTick = adcIsrTick;
                        /* Seqlock read of stepPeriodHR (Rule 13) */
                        uint32_t sp;
                        uint16_t s1, s2;
                        do {
                            s1 = garudaData.hwzc.writeSeq;
                            sp = garudaData.hwzc.stepPeriodHR;
                            s2 = garudaData.hwzc.writeSeq;
                        } while (s1 != s2 || (s1 & 1));
                        uint16_t obsPeriod = HWZC_SCCP2_TO_ADC(sp);
                        if (obsPeriod < 1) obsPeriod = 1;
                        BEMF_INTEG_ObserverOnComm(&garudaData, obsPeriod);
                        garudaData.hwzc.shadowHwzcSkipCount++;
                    }
                    BEMF_INTEG_ObserverTick(&garudaData, adcIsrTick,
                        garudaData.hwzc.obsLastCommTick);
                }
#endif /* FEATURE_BEMF_INTEGRATION */
            }
#endif /* FEATURE_ADC_CMP_ZC */

#if FEATURE_ADC_CMP_ZC
        cl_duty_control:
#endif

            /* Duty cycle control. */
            {
                uint32_t cap = garudaData.timing.zcSynced ? MAX_DUTY : RT_RAMP_DUTY_CAP;
                uint32_t mappedDuty;

#if FEATURE_DUTY_SLEW
                static uint16_t postSyncCounter = 0;
#endif

                if (!garudaData.timing.zcSynced)
                {
                    /* Pre-sync: hold duty at CL entry level.
                     * Don't let pot changes affect duty while forced
                     * commutation is trying to lock ZC — changing duty
                     * shifts the threshold and confuses detection. */
                    mappedDuty = garudaData.duty;
#if FEATURE_DUTY_SLEW
                    postSyncCounter = 0;
#endif
                }
                else
                {
#if FEATURE_DUTY_SLEW
                    if (postSyncCounter < RT_POST_SYNC_SETTLE_TICKS)
                        postSyncCounter++;
#endif
#if FEATURE_SPEED_PI
                    /* Speed PI: throttle controls TARGET SPEED via target
                     * stepPeriodHR; PI regulates duty to track. Replaces
                     * the direct throttle→duty scaling below. The output
                     * is updated per HWZC event (see SPEED_PI_OnZcEvent).
                     *
                     * SPEED_PI_Enable on first sync tick — bumpless transfer
                     * (integrator seeded from garudaData.duty). */
                    if (!garudaData.speedPi.enabled)
                        SPEED_PI_Enable(&garudaData);
                    mappedDuty = garudaData.speedPi.outputDuty;
#else
#if FEATURE_CL_DIFF_IDLE
                    /* Differential-low idle: throttle maps from the (sub-
                     * MIN_DUTY) diff floor — idle equilibrium ~3k, meeting the
                     * morph hand-off speed instead of slamming up to ~10k. */
                    mappedDuty = CL_DIFF_IDLE_FLOOR +
                        ((uint32_t)garudaData.throttle * (cap - CL_DIFF_IDLE_FLOOR)) / 4096;
#else
                    mappedDuty = RT_CL_IDLE_DUTY +
                        ((uint32_t)garudaData.throttle * (cap - RT_CL_IDLE_DUTY)) / 4096;
#endif
#endif
                }
#if FEATURE_CL_DIFF_IDLE
                if (mappedDuty < CL_DIFF_IDLE_FLOOR) mappedDuty = CL_DIFF_IDLE_FLOOR;
#elif FEATURE_CL_LOW_IDLE
                /* Lower CL idle floor (deadtime unchanged → no shoot-through risk;
                 * only shortens the H-pulse). Drops idle equilibrium + handoff gap. */
                if (mappedDuty < CL_LOW_IDLE_FLOOR) mappedDuty = CL_LOW_IDLE_FLOOR;
#else
                if (mappedDuty < MIN_DUTY) mappedDuty = MIN_DUTY;
#endif
                if (mappedDuty > cap) mappedDuty = cap;

#if FEATURE_CL_ENTRY_SOFTSTART
                /* CL-ENTRY SOFT-START: cap duty to a ceiling that ramps linearly
                 * from CL_ENTRY_START_DUTY up to the requested (idle) duty over
                 * CL_ENTRY_RAMP_TICKS. The phase current then builds gradually
                 * with the rising BEMF instead of stepping to full idle duty
                 * against ~0 BEMF at entry -> smaller inrush PEAK + longer ramp,
                 * final idle speed unchanged. Stops limiting once the ceiling
                 * reaches the idle floor. */
                {
                    static uint16_t clEntryTick = CL_ENTRY_RAMP_TICKS;
                    if (prevAdcState != ESC_CLOSED_LOOP)
                        clEntryTick = 0;                 /* arm at CL entry */
                    if (clEntryTick < CL_ENTRY_RAMP_TICKS) {
                        uint32_t span = (RT_CL_IDLE_DUTY > CL_ENTRY_START_DUTY)
                                      ? (RT_CL_IDLE_DUTY - CL_ENTRY_START_DUTY) : 0u;
                        uint32_t entryCeil = CL_ENTRY_START_DUTY +
                            ((span * (uint32_t)clEntryTick) / CL_ENTRY_RAMP_TICKS);
                        if (mappedDuty > entryCeil) mappedDuty = entryCeil;
                        clEntryTick++;
                    }
                }
#endif

#if FEATURE_DUTY_SLEW
                {
                    static uint32_t prevDuty = 0;
                    if (prevAdcState != ESC_CLOSED_LOOP)
#if FEATURE_CL_ENTRY_SOFTSTART
                        prevDuty = CL_ENTRY_START_DUTY;  /* low baseline so the soft-start ramp isn't fought by a down-slew */
#else
                        prevDuty = garudaData.duty;
#endif

                    /* Post-sync settle: use reduced slew-up rate for
                     * POST_SYNC_SETTLE_MS after ZC lock. This prevents
                     * the motor from accelerating faster than the
                     * stepPeriod IIR can track on low-inertia motors.
                     * Normal rate: 2%/ms. Settle rate: 0.25%/ms (1/8). */
                    uint32_t upRate = RT_DUTY_SLEW_UP_RATE;
                    if (postSyncCounter < RT_POST_SYNC_SETTLE_TICKS)
                        upRate = RT_DUTY_SLEW_UP_RATE / RT_POST_SYNC_SLEW_DIVISOR;


                    int32_t delta = (int32_t)mappedDuty - (int32_t)prevDuty;
                    if (delta > 0)
                    {
                        if ((uint32_t)delta > upRate)
                            mappedDuty = prevDuty + upRate;
                    }
                    else if (delta < 0)
                    {
#if FEATURE_VBUS_REGEN_BRAKE
                        /* Vbus-aware regen brake — sticky version. Once engaged
                         * (Vbus rose above ON threshold during a regen event),
                         * stays engaged for minimum hold ticks even if Vbus
                         * briefly dips, then releases only when Vbus falls all
                         * the way to OFF threshold (wide hysteresis). This
                         * prevents the chatter that let duty keep dropping
                         * across regen pulses in the prior implementation. */
                        static bool regenBrakeActive = false;
                        static uint16_t regenBrakeHoldTicks = 0;
                        if (regenBrakeActive) {
                            if (regenBrakeHoldTicks > 0)
                                regenBrakeHoldTicks--;
                            if (regenBrakeHoldTicks == 0
                                && garudaData.vbusRaw < VBUS_REGEN_BRAKE_OFF_ADC)
                                regenBrakeActive = false;
                        } else {
                            if (garudaData.vbusRaw > VBUS_REGEN_BRAKE_ON_ADC) {
                                regenBrakeActive = true;
                                regenBrakeHoldTicks = VBUS_REGEN_BRAKE_MIN_TICKS;
                            }
                        }
                        uint32_t effectiveDownRate = RT_DUTY_SLEW_DOWN_RATE;
                        if (regenBrakeActive) {
                            /* Slow the slew-down rather than freezing entirely.
                             * Motor still decelerates during regen but at a rate
                             * the bus capacitor can absorb. Avoids the clumsy
                             * "can't decelerate" feel of a hard freeze. */
                            effectiveDownRate = RT_DUTY_SLEW_DOWN_RATE
                                              / VBUS_REGEN_BRAKE_SLEW_DIVISOR;
                            if (effectiveDownRate == 0) effectiveDownRate = 1;
                        }
#if FEATURE_HIGH_RPM_SLEW_DOWN
                        /* Proactive: at high eRPM, slew-down is limited
                         * regardless of Vbus state. Prevents the regen
                         * pulse before it spikes the bus. Reads HWZC's
                         * stepPeriodHR — at high RPM HWZC owns the period.
                         * Below the threshold or before HWZC engages,
                         * no penalty applied. */
                        if (garudaData.hwzc.enabled
                            && garudaData.hwzc.stepPeriodHR > 0
                            && garudaData.hwzc.stepPeriodHR
                               < HIGH_RPM_SLEW_THRESHOLD_HR) {
                            uint32_t highRpmRate = RT_DUTY_SLEW_DOWN_RATE
                                                 / HIGH_RPM_SLEW_DIVISOR;
                            if (highRpmRate == 0) highRpmRate = 1;
                            if (effectiveDownRate > highRpmRate)
                                effectiveDownRate = highRpmRate;
                        }
#endif
#if FEATURE_VBUS_EMERGENCY_HOLD
                        /* Emergency tier: above regen brake (28V), if
                         * Vbus hits 30V, FREEZE slew-down entirely so
                         * the rotor stops dumping regen energy into bus.
                         * Sticky hysteresis (30→27V) + 20ms min hold
                         * to prevent chatter. Releases when Vbus settles
                         * back below 27V AND min-hold has elapsed. */
                        static bool emergencyHoldActive = false;
                        static uint16_t emergencyHoldTicks = 0;
                        if (emergencyHoldActive) {
                            if (emergencyHoldTicks > 0)
                                emergencyHoldTicks--;
                            if (emergencyHoldTicks == 0
                                && garudaData.vbusRaw < VBUS_EMERGENCY_HOLD_OFF_ADC)
                                emergencyHoldActive = false;
                        } else {
                            if (garudaData.vbusRaw > VBUS_EMERGENCY_HOLD_ON_ADC) {
                                emergencyHoldActive = true;
                                emergencyHoldTicks = VBUS_EMERGENCY_HOLD_MIN_TICKS;
                            }
                        }
                        if (emergencyHoldActive)
                            effectiveDownRate = 0;
#endif
                        if ((uint32_t)(-delta) > effectiveDownRate)
                            mappedDuty = prevDuty - effectiveDownRate;
#else
                        if ((uint32_t)(-delta) > RT_DUTY_SLEW_DOWN_RATE)
                            mappedDuty = prevDuty - RT_DUTY_SLEW_DOWN_RATE;
#endif
                    }
                    prevDuty = mappedDuty;
                }
#endif
                if (garudaData.timing.stepPeriod <= RT_MIN_CL_ADC_STEP_PERIOD
                    && mappedDuty > garudaData.duty)
                {
                    mappedDuty = garudaData.duty;
                }

#if FEATURE_VBUS_SAG_LIMIT
                {
                    static uint16_t vbusFiltered = 0;
                    static bool vbusSagActive = false;

                    if (prevAdcState != ESC_CLOSED_LOOP)
                    {
                        vbusFiltered = garudaData.vbusRaw;
                        vbusSagActive = false;
                    }

                    /* IIR filter always runs (keeps filter warm during pre-sync) */
                    vbusFiltered = (uint16_t)(
                        ((uint32_t)vbusFiltered * 7 + garudaData.vbusRaw) >> 3);

#if FEATURE_PRESYNC_RAMP
                    /* Bypass sag limiter during pre-sync: at 12V CC-limited supply,
                     * vbus sags to ~636 ADC (8.9V) under startup load. With
                     * VBUS_SAG_THRESHOLD_ADC=900, the sag limiter would reduce
                     * RAMP_DUTY_CAP by ~26%, killing startup torque. Allow full
                     * startup duty until ZC sync is achieved. */
                    if (garudaData.timing.zcSynced)
                    {
#endif
                    if (!vbusSagActive && vbusFiltered < VBUS_SAG_THRESHOLD_ADC)
                        vbusSagActive = true;
                    else if (vbusSagActive && vbusFiltered > VBUS_SAG_RECOVERY_ADC)
                        vbusSagActive = false;

                    if (vbusSagActive)
                    {
                        uint32_t sagDepth = (vbusFiltered < VBUS_SAG_THRESHOLD_ADC) ?
                            (VBUS_SAG_THRESHOLD_ADC - vbusFiltered) : 0;
                        uint32_t reduction = (sagDepth * VBUS_SAG_GAIN) >> 4;
                        if (reduction >= mappedDuty)
                            mappedDuty = MIN_DUTY;
                        else
                            mappedDuty -= reduction;
                    }
#if FEATURE_PRESYNC_RAMP
                    }
                    else
                    {
                        vbusSagActive = false;  /* Reset so it re-evaluates on sync */
                    }
#endif

#if FEATURE_CL_LOW_IDLE
                    /* respect the lowered CL idle floor (this sag block's
                     * re-floor would otherwise clobber CL_LOW_IDLE_FLOOR
                     * back up to MIN_DUTY every tick) */
                    if (mappedDuty < CL_LOW_IDLE_FLOOR)
                        mappedDuty = CL_LOW_IDLE_FLOOR;
#elif FEATURE_CL_DIFF_IDLE
                    /* differential-low idle: don't re-floor up to MIN_DUTY */
                    if (mappedDuty < CL_DIFF_IDLE_FLOOR)
                        mappedDuty = CL_DIFF_IDLE_FLOOR;
#else
                    if (mappedDuty < MIN_DUTY)
                        mappedDuty = MIN_DUTY;
#endif
                }
#endif

#if FEATURE_HW_OVERCURRENT
                /* Software bus current soft limiter — proportional duty reduction.
                 * Ramps down duty smoothly BEFORE CMP3/CLPCI hardware trips. */
                if (garudaData.ibusRaw > OC_SW_LIMIT_ADC
                    && mappedDuty > MIN_DUTY)   /* guard: below MIN_DUTY (diff idle)
                                                 * the subtraction would underflow
                                                 * unsigned and RAISE duty under OC */
                {
                    uint16_t excess = garudaData.ibusRaw - OC_SW_LIMIT_ADC;
                    uint16_t range = RT_OC_CMP3_DAC_VAL - OC_SW_LIMIT_ADC;
                    if (range == 0) range = 1;
                    uint32_t reduction = ((uint32_t)excess * (mappedDuty - MIN_DUTY)) / range;
                    if (reduction >= mappedDuty - MIN_DUTY)
                        mappedDuty = MIN_DUTY;
                    else
                        mappedDuty -= reduction;
                }
#endif

#if FEATURE_CL_SOFT_ENTRY
                /* Option A — soft OL->CL hand-off. MORPH leaves the duty already
                 * at operating level (~6%), so a slew-RATE limiter has no upward
                 * step to catch. Instead OWN an explicit cap that starts at
                 * MIN_DUTY on CL entry and ramps up gently, so the motor walks
                 * from the low-BEMF hand-off speed (~3k) to the idle equilibrium
                 * (~14k) with bounded current instead of the ~22A torque slam.
                 * Final cap (after OC limiter) so protections still win; only
                 * limits the UPPER bound during the entry window. */
                {
                    static uint32_t softCap = 0;
                    static uint16_t softCtr = 0;
                    if (prevAdcState != ESC_CLOSED_LOOP) {
                        softCtr  = 0;
                        softCap  = MIN_DUTY;        /* begin LOW, ignore morph-exit duty */
                    }
                    if (softCtr < CL_SOFT_ENTRY_TICKS) {
                        softCtr++;
                        uint32_t step = RT_DUTY_SLEW_UP_RATE / CL_SOFT_ENTRY_DIVISOR;
                        if (step == 0) step = 1;
                        softCap += step;
                        if (mappedDuty > softCap)
                            mappedDuty = softCap;
                    }
                }
#endif
#if FEATURE_IF_BRIDGE
                /* Option D — I-f current-limited OL->CL hand-off bridge.
                 * Ramp duty up from MIN_DUTY, but back off whenever bus current
                 * exceeds the cap, so the motor accelerates from the (low-BEMF)
                 * hand-off speed to the idle equilibrium at BOUNDED current
                 * instead of the structural ~22A slam. Motor-agnostic: the cap
                 * is the only knob. Only ever LOWERS mappedDuty (exits, leaving
                 * normal control, once the regulated duty catches up to demand),
                 * so OC/regen/OV protections above still win. */
                {
                    static uint32_t ifDuty = 0;
                    static uint16_t ifCtr  = 0;
                    static uint16_t ifPeak = 0;
                    static bool ifActive   = false;
                    if (prevAdcState != ESC_CLOSED_LOOP) {
                        ifActive = true;
                        ifDuty   = MIN_DUTY;
                        ifCtr    = 0;
                        ifPeak   = 0;
                    }
                    if (ifActive) {
                        ifCtr++;
                        /* back off on current MAGNITUDE (both motoring + and regen -
                         * excursions); the bus current swings negative during the
                         * unlocked hand-off, which a signed compare would miss.
                         * ibusRaw is sampled at the PWM valley (~0 there), so the real
                         * hand-off current registers only as a RECURRING spike that a
                         * single per-tick read mostly misses — PEAK-HOLD it (decaying)
                         * so the back-off actually sees the −22A and reacts. */
                        int32_t iInst = (int32_t)garudaData.ibusRaw
                                      - (int32_t)OC_BIAS_COUNTS;
                        if (iInst < 0) iInst = -iInst;
                        if ((uint16_t)iInst > ifPeak)
                            ifPeak = (uint16_t)iInst;
                        else
                            ifPeak -= (ifPeak >> IF_BRIDGE_PEAK_DECAY_SHIFT);
                        int32_t iMag = (int32_t)ifPeak;
                        if (iMag > IF_BRIDGE_LIMIT_DELTA) {
                            /* over the current cap — back off fast */
                            if (ifDuty > MIN_DUTY + IF_BRIDGE_DOWN_RATE)
                                ifDuty -= IF_BRIDGE_DOWN_RATE;
                            else
                                ifDuty = MIN_DUTY;
                        } else {
                            /* under the cap — ramp up toward normal demand */
                            ifDuty += IF_BRIDGE_UP_RATE;
                        }
                        /* Never exceed the normal demand, but DON'T exit just
                         * because the ramp caught up — a single PWM-gated low
                         * current sample must not let the bridge bail before the
                         * real overcurrent develops. Stay active for the whole
                         * window; once the motor reaches idle, ifDuty simply sits
                         * at the demand (harmless) until the window expires. */
                        if (ifDuty > mappedDuty)
                            ifDuty = mappedDuty;
                        if (ifCtr >= IF_BRIDGE_TICKS)
                            ifActive = false;        /* window elapsed -> hand back */
                        mappedDuty = ifDuty;         /* apply the bounded-current duty */
                    }
                }
#endif

#if FEATURE_CL_ENTRY_GLIDE
                /* Entry glide: ramp effective duty linearly from the matched
                 * engage level to MIN_DUTY (~320ms), overriding the mapping
                 * (placed LAST so the MIN_DUTY floors above can't undo the
                 * sub-MIN cap). Speed follows quasi-statically → near-linear
                 * climb to the MIN_DUTY equilibrium at a few amps instead of
                 * the ballistic 22A slam. At MIN_DUTY, force-swap to the
                 * conventional waveform — bit-identical baseline after. */
                if (s_glideActive)
                {
                    if (++s_glideDivCtr >= CL_GLIDE_DIV)
                    {
                        s_glideDivCtr = 0;
                        s_glideDuty++;
                    }
                    if (s_glideDuty >= MIN_DUTY)
                    {
                        s_glideActive = 0;
                        if (g_pwmDiffLow)
                        {
                            /* hand back to the baseline waveform (HWZC then
                             * re-enables via the normal crossover logic) */
                            g_pwmDiffLow = 0;
                            HAL_PWM_SetCommutationStep(garudaData.currentStep);
                        }
                    }
                    else if (mappedDuty > s_glideDuty)
                    {
                        mappedDuty = s_glideDuty;
                    }
                }
#endif
                garudaData.duty = mappedDuty;

#if FEATURE_CL_DIFF_IDLE
                /* Drive-mode management. garudaData.duty is EFFECTIVE duty in
                 * both modes (identical line-line volts for duty ≥ MIN_DUTY),
                 * so the swaps are voltage-seamless; only the low phase's
                 * waveform class changes. Hysteresis prevents IOCON churn. */
                {
                    /* (CL entry is handled by the coast-listen block at the
                     * top of the case — it engages diff mode synced.) */
                    if (g_pwmDiffLow
                             && garudaData.duty >= MIN_DUTY
                                + (MIN_DUTY / CL_DIFF_EXIT_HYST_DIV))
                    {
                        /* throttle demand exceeds the conventional floor —
                         * swap to the proven override-LOW waveform (HWZC
                         * re-enables via the normal crossover logic) */
                        g_pwmDiffLow = 0;
                        HAL_PWM_SetCommutationStep(garudaData.currentStep);
                    }
                    else if (!g_pwmDiffLow && garudaData.duty < MIN_DUTY)
                    {
                        /* demand back below the conventional floor — re-enter */
                        g_pwmDiffLow = 1;
                        HAL_PWM_SetCommutationStep(garudaData.currentStep);
#if FEATURE_ADC_CMP_ZC
                        garudaData.hwzc.enablePending = false;
                        if (garudaData.hwzc.enabled)
                        {
                            /* hand ZC back to the SW valley detector, re-seeded
                             * from HWZC's period (the proven fallback path) */
                            HWZC_Disable(&garudaData);
                            garudaData.hwzc.fallbackPending = true;
                        }
#endif
                    }
                }
#endif

#if FEATURE_HANDOFF_CHOP
                /* Sub-MIN_DUTY current bound via the CMP3 HARDWARE chop, not duty.
                 * Hold a LOW chop threshold for a window at CL entry so the
                 * cycle-by-cycle CLPCI truncates each pulse at OC_CMP3_HANDOFF_MA
                 * — bounding the phase current (and its −freewheel bus pulse)
                 * regardless of duty/MIN_DUTY, with no regen oscillation. Other
                 * state transitions also write the CMP3 DAC (zcSync etc.), so we
                 * RE-ASSERT every tick to own the threshold for the whole window,
                 * then restore the operational value once. Armed only at CL entry;
                 * align/OL/morph keep STARTUP_DAC so their torque isn't chopped. */
                {
                    static uint16_t hcCtr = 0;
                    static bool hcActive = false;
                    if (hcArmPending) {        /* armed at CL entry; survives the coast-listen break */
                        hcArmPending = false;
                        hcActive = true;
                        hcCtr = 0;
                    }
                    if (hcActive) {
                        HAL_CMP3_SetThreshold(OC_CMP3_HANDOFF_DAC);
                        if (++hcCtr >= HANDOFF_CHOP_TICKS) {
                            hcActive = false;
                            HAL_CMP3_SetThreshold(RT_OC_CMP3_DAC_VAL);
                        }
                    }
                }
#endif
            }
            HAL_PWM_SetDutyCycle(garudaData.duty);
            break;
        }
#else  /* !FEATURE_BEMF_CLOSED_LOOP — Phase 1 open-loop path */
        case ESC_CLOSED_LOOP:
            HAL_PWM_SetDutyCycle(garudaData.duty);
            break;
#endif

        case ESC_BRAKING:
            break;

        case ESC_RECOVERY:
            /* PWM already disabled on entry — coast-down handled in Timer1 */
            break;

        case ESC_FAULT:
            HAL_MC1PWMDisableOutputs();
            break;
    }
#endif /* !FEATURE_FOC && !FEATURE_FOC_V2 && !FEATURE_FOC_V3 && !FEATURE_FOC_AN1078 — end of 6-step state machine */

#if FEATURE_BEMF_CLOSED_LOOP
    /* Track state for transition detection (must be last before flag clear).
     * Use entryState (not garudaData.state) so mid-ISR state changes
     * (e.g. morph→CL) are visible on the NEXT tick. */
#if FEATURE_SPEED_PI
    /* Disable speed PI when leaving CL state. Cleanup so SPEED_PI_OnZcEvent
     * (called from HWZC ISR) no-ops until next CL re-entry. Bumpless on
     * re-enable because Enable seeds integrator from current duty. */
    if (entryState == ESC_CLOSED_LOOP
        && garudaData.state != ESC_CLOSED_LOOP
        && garudaData.speedPi.enabled)
        SPEED_PI_Disable(&garudaData);
#endif
    prevAdcState = entryState;
#endif

    /* X2CScope: sample variables at ADC rate (24kHz) */
#ifdef ENABLE_DIAGNOSTICS
    DiagnosticsStepIsr();
#endif

    /* Clear interrupt flag AFTER reading all buffers (matches reference) */
    GARUDA_ClearADCIF();
}

/**
 * @brief Timer1 ISR — 100us tick.
 * Handles: heartbeat LED, board service, commutation timing for
 * align/ramp states, 1ms system tick.
 */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    /* Heartbeat LED — toggle at ~2Hz (250ms) */
    heartbeatCounter++;
    if (heartbeatCounter >= HEART_BEAT_LED_COUNT)
    {
        heartbeatCounter = 0;
        if (LED1 == 1)
            LED1 = 0;
        else
            LED1 = 1;
    }

    /* Board service step (drives button debounce at 1ms) */
    BoardServiceStepIsr();

    /* 1ms system tick sub-counter (10 x 100us = 1ms) */
    msSubCounter++;
    if (msSubCounter >= 10)
    {
        msSubCounter = 0;
        garudaData.systemTick++;
    }

    /* State-specific processing at 100us rate */
    switch (garudaData.state)
    {
        case ESC_IDLE:
            /* Nothing — waiting for button press in main loop */
            break;

        case ESC_ARMED:
#if FEATURE_FOC || FEATURE_FOC_V2 || FEATURE_FOC_V3 || FEATURE_FOC_AN1078
            /* FOC: arming handled in ADC ISR slow loop (1kHz) to avoid
             * cross-ISR races. Timer1 is a no-op for ESC_ARMED with FOC. */
            break;
#else
#if FEATURE_ARM_BEEP
            /* Arm melody: plays AFTER the quiet arm window (OC auto-zero has
             * already latched its bias) and BEFORE startup. Three sequential
             * pitches over ARM_BEEP_TICKS; the arm-complete transition below
             * is pushed out by the same amount. Throttle-up during the
             * melody resets the countdown (same rule as arming itself). */
            if (garudaData.throttle < ARM_THROTTLE_ZERO_ADC
                && garudaData.armCounter >= ARM_TIME_COUNTS)
            {
                static const uint16_t halfTbl[3] = {
                    (uint16_t)(10000u / (2u * ARM_BEEP_FREQ1_HZ)),
                    (uint16_t)(10000u / (2u * ARM_BEEP_FREQ2_HZ)),
                    (uint16_t)(10000u / (2u * ARM_BEEP_FREQ3_HZ)),
                };
                uint32_t t = garudaData.armCounter - ARM_TIME_COUNTS;
                uint32_t noteLen = ARM_BEEP_TICKS / 3u;
                uint8_t note = (uint8_t)(t / noteLen);
                if (note > 2u) note = 2u;
                if (((t / halfTbl[note]) & 1u) == 0u)
                {
                    HAL_PWM_SetCommutationStep(0);
                    HAL_PWM_SetDutyCycle((uint16_t)(((uint32_t)LOOPTIME_TCY
                                          * ARM_BEEP_DUTY_PCT) / 100u));
                }
                else
                {
                    HAL_MC1PWMDisableOutputs();
                }
            }
#endif
            /* Verify throttle stays at zero for ARM_TIME */
            if (garudaData.throttle < ARM_THROTTLE_ZERO_ADC)
            {
                garudaData.armCounter++;
#if FEATURE_ARM_BEEP
                if (garudaData.armCounter >= ARM_TIME_COUNTS + ARM_BEEP_TICKS)
#else
                if (garudaData.armCounter >= ARM_TIME_COUNTS)
#endif
                {
                    /* Armed successfully — transition to ALIGN.
                     * Init before state change: ADC ISR (prio 6) can
                     * preempt Timer1 (prio 5) between the two lines. */
                    STARTUP_Init(&garudaData);
#if FEATURE_IBUS_PROBE
                    /* Bus-current chop probe: keep the sine align ACTIVE (the
                     * proven bridge drive — STARTUP_Init already set sine.active
                     * + released overrides). Hold it in ESC_ALIGN (don't advance)
                     * and sweep the CMP3 chop threshold there. The earlier
                     * SetCommutationStep path never energized the bridge from
                     * this entry → zero current; the sine drive does. */
                    garudaData.state = ESC_ALIGN;
#elif FEATURE_AM32_STARTUP
                    /* AM32-style: no align, no ramp. Seed the listener's
                     * period guess and enter CL; the ADC-ISR entry block
                     * does the one blind kick + HWZC arm (startMotor()
                     * semantics, AM32 main.c:977). */
                    {
                        uint32_t seedErpm = AM32_START_SEED_ERPM;
                        if (seedErpm == 0u)   /* derive from active profile */
                            seedErpm = (2u * RT_RAMP_TARGET_ERPM) / 3u;
                        if (seedErpm < 300u) seedErpm = 300u;
                        garudaData.rampStepPeriod =
                            (uint16_t)ERPM_TO_STEP_TICKS(seedErpm);
                    }
                    g_am32EntryPending = 1;
#if FEATURE_HW_OVERCURRENT
                    HAL_CMP3_SetThreshold(RT_OC_CMP3_DAC_VAL);
#endif
                    garudaData.state = ESC_CLOSED_LOOP;
#elif FEATURE_IF_STARTUP
                    /* I-f self-aligns from the clean override-low bridge state.
                     * STARTUP_Init's SineInit briefly wrote sine duties + released
                     * overrides; IF_StartupInit (same tick) overwrites to balanced
                     * 0V before any sine duty latches, so the sine HOLD never runs
                     * and there's no sine→SVPWM handover. */
                    IF_StartupInit();
                    garudaData.state = ESC_IF_RAMP;
#else
                    garudaData.state = ESC_ALIGN;
#endif
                    LED2 = 1; /* Motor running indicator */
                }
            }
            else
            {
                garudaData.armCounter = 0; /* Reset if throttle not zero */
            }
            break;
#endif

        case ESC_ALIGN:
#if FEATURE_IBUS_PROBE
            /* Hold the PROVEN sine align (ignore its completion return so the
             * bridge keeps driving real current ~1.5A), and sweep the CMP3 chop
             * threshold over a FINE 0..3A range (the align current is small).
             * As the pot rises the threshold drops; when it crosses the real
             * current the chop fires → eRPM (trip-rate, hijacked) JUMPS and Ibus
             * DROPS toward the threshold. Trip current: A ≈ 3 × (4095 − thr)/4095.
             * eRPM lighting + Ibus falling = the hardware current chop WORKS. */
            (void)STARTUP_SineAlign(&garudaData);
            PG1LEBbits.LEB = 80u;
            /* INPSEL SCAN (2026-06-13): force DAC3 to 30 (below the OA3 ~78 rest)
             * so the comparator + input should read HIGH on whichever input the
             * op-amp actually lands on. SWEEP THE POT through 5 zones; eRPM
             * (= live CMPSTAT, 10000/0) lights in the zone whose input sees OA3:
             *   pot 0-20%   INPSEL=0  CMP_A = RA5 = OA3OUT   (expected winner)
             *   pot 20-40%  INPSEL=1  CMP_B = RB5 = OA3IN+
             *   pot 40-60%  INPSEL=2  CMP_C = RA6 = OA3IN-
             *   pot 60-80%  INPSEL=3  CMP_D = RA1            (AN957's value)
             *   pot 80-100% INPSEL=4  Bandgap 0.8V — CHAIN SELF-TEST: this MUST
             *               read 10000 (993 > 30). If even THIS stays 0, the
             *               comparator/DAC/CMPSTAT chain is broken, not the input. */
            {
                uint16_t z = garudaData.throttle;
                uint8_t  sel = (z < 819u) ? 0u : (z < 1638u) ? 1u
                             : (z < 2457u) ? 2u : (z < 3276u) ? 3u : 4u;
                DAC3CMPbits.INPSEL = sel;
                DAC3DATbits.DACDAT = 30u;
            }
            break;
#elif FEATURE_FOC || FEATURE_FOC_V2 || FEATURE_FOC_V3 || FEATURE_FOC_AN1078
            /* FOC: alignment handled in ADC ISR. Timer1 is a no-op. */
            break;
#elif FEATURE_SINE_STARTUP
            /* I-f path bypasses ALIGN entirely (ARMED→IF_RAMP), so this only
             * runs in the non-IF build. */
            if (STARTUP_SineAlign(&garudaData))
            {
#if FEATURE_PLL_STARTUP
                /* PLL-from-align: no OL ramp, no morph. Seed the slow
                 * initial period, flag the blind schedule, and enter CL —
                 * the ADC-ISR entry block engages 6-step + HWZC there. */
                garudaData.rampStepPeriod =
                    (uint16_t)ERPM_TO_STEP_TICKS(PLL_START_ERPM0);
                garudaData.hwzc.pllStartActive = 1;
                garudaData.hwzc.pllStartGood = 0;
                garudaData.hwzc.pllPrevCap = 0;
#if FEATURE_HW_OVERCURRENT
                HAL_CMP3_SetThreshold(RT_OC_CMP3_DAC_VAL);
#endif
                garudaData.state = ESC_CLOSED_LOOP;
#else
                garudaData.state = ESC_OL_RAMP;
#endif
            }
            break;
#else
            if (STARTUP_Align(&garudaData))
            {
#if FEATURE_PLL_STARTUP
                /* PLL-from-align over the CLASSIC align: same exit as the
                 * sine-align path — seed slow period, flag the blind
                 * schedule, enter CL (ADC-ISR entry block engages). */
                garudaData.rampStepPeriod =
                    (uint16_t)ERPM_TO_STEP_TICKS(PLL_START_ERPM0);
                garudaData.hwzc.pllStartActive = 1;
                garudaData.hwzc.pllStartGood = 0;
                garudaData.hwzc.pllPrevCap = 0;
#if FEATURE_HW_OVERCURRENT
                HAL_CMP3_SetThreshold(RT_OC_CMP3_DAC_VAL);
#endif
                garudaData.state = ESC_CLOSED_LOOP;
#else
                garudaData.state = ESC_OL_RAMP;
                garudaData.rampCounter = garudaData.rampStepPeriod;
#endif
            }
            break;
#endif

#if FEATURE_IF_STARTUP
        case ESC_IF_RAMP:
            /* I-f spin-up is driven entirely in the ADC ISR; Timer1 is a no-op. */
            break;
#endif

        case ESC_OL_RAMP:
#if FEATURE_FOC || FEATURE_FOC_V2 || FEATURE_FOC_V3 || FEATURE_FOC_AN1078
            /* FOC: I/f ramp handled in ADC ISR. Timer1 is a no-op. */
            break;
#elif DIAGNOSTIC_MANUAL_STEP
            garudaData.state = ESC_CLOSED_LOOP;
            break;
#elif FEATURE_SINE_STARTUP
            if (STARTUP_SineRamp(&garudaData))
            {
                /* Sine ramp complete → enter waveform morph.
                 * rampStepPeriod synced from sine eRPM in MorphInit. */
                STARTUP_MorphInit(&garudaData);
#if FEATURE_HW_OVERCURRENT
                /* Lower CMP3 from startup (22A) to operational (12A).
                 * Morph convergence can spike current above the board PCI
                 * threshold (~15A) but below CMP3 startup. Without this,
                 * CMP3 never trips and the board PCI hard-faults. */
                HAL_CMP3_SetThreshold(RT_OC_CMP3_DAC_VAL);
#endif
#if FEATURE_SKIP_MORPH && FEATURE_CL_COAST_VERIFY
                if (garudaData.direction == 0)
                {
                    /* Skip the morph: the CL-entry coast-listen measures the
                     * TRUE sector/period from clean coast BEMF — no trap
                     * converge, no windowed-Hi-Z grind (deletes the morph
                     * current kick). MorphInit above already did the critical
                     * rampStepPeriod sync; the morph fields it set go unused.
                     * Next ADC tick: normal CL entry init (prev==OL_RAMP →
                     * BEMF_ZC_Init path), then CL_CoastBegin cuts the bridge. */
                    garudaData.sine.active = false;
                    garudaData.state = ESC_CLOSED_LOOP;
                    break;
                }
#endif
                garudaData.state = ESC_MORPH;
            }
            break;
#else
#if FEATURE_PRESYNC_RAMP
            /* Skip blind forced ramp. Enter CL directly with slow initial
             * step period. ADC ISR pre-sync handles feedback-gated
             * acceleration with ZC detection in parallel. */
            garudaData.duty = RT_RAMP_DUTY_CAP;
            garudaData.state = ESC_CLOSED_LOOP;
#else
            if (STARTUP_OpenLoopRamp(&garudaData))
            {
                garudaData.state = ESC_CLOSED_LOOP;
            }
#endif
            break;
#endif

        case ESC_MORPH:
#if FEATURE_FOC || FEATURE_FOC_V2 || FEATURE_FOC_V3 || FEATURE_FOC_AN1078
            /* FOC: no morph phase. Unreachable. */
            break;
#elif FEATURE_SINE_STARTUP
            /* All morph logic runs in ADC ISR (24kHz). Timer1 is idle. */
            break;
#else
            /* Morph only used with SINE_STARTUP — unreachable here */
            break;
#endif

#if FEATURE_FOC || FEATURE_FOC_V2 || FEATURE_FOC_V3 || FEATURE_FOC_AN1078
        case ESC_CLOSED_LOOP:
            /* FOC: all control in ADC ISR. Timer1 does nothing. */
            break;
#elif DIAGNOSTIC_MANUAL_STEP
        case ESC_CLOSED_LOOP:
            /* Manual step mode: hold current step. SW2 advances from main loop. */
            break;
#elif FEATURE_BEMF_CLOSED_LOOP
        case ESC_CLOSED_LOOP:
            /* Phase 2: All commutation handled in ADC ISR. Timer1 does nothing. */
            break;
#else
        case ESC_CLOSED_LOOP:
            /* Phase 1: keep forced commutation at final ramp speed. */
            if (garudaData.rampCounter > 0)
            {
                garudaData.rampCounter--;
            }
            if (garudaData.rampCounter == 0)
            {
                COMMUTATION_AdvanceStep(&garudaData);
                garudaData.rampCounter = garudaData.rampStepPeriod;
            }
            break;
#endif

        case ESC_BRAKING:
            break;

        case ESC_RECOVERY:
#if FEATURE_DESYNC_RECOVERY
            if (garudaData.recoveryCounter > 0)
            {
                garudaData.recoveryCounter--;
            }
            else
            {
#if FEATURE_THROTTLE_ZERO_AUTO_DISARM
                bool restartGateThrottle =
                    (garudaData.throttle >= ARM_THROTTLE_ZERO_ADC);
#else
                /* Throttle-zero auto-disarm disabled — recovery always
                 * attempts restart while runCommandActive is still set,
                 * regardless of throttle level. Pot-at-zero is treated as
                 * "run at idle duty", not "stop". */
                bool restartGateThrottle = true;
#endif

#if !FEATURE_THROTTLE_ZERO_AUTO_DISARM
                /* AUTO_DISARM=0: never run out of restart attempts.
                 * Reset the counter when it would otherwise cap, so the
                 * motor keeps restarting indefinitely at any throttle. */
                if (garudaData.desyncRestartAttempts >= RT_DESYNC_MAX_RESTARTS)
                    garudaData.desyncRestartAttempts = 0;
                garudaData.runCommandActive = true;
#endif

                if (garudaData.runCommandActive &&
                    garudaData.desyncRestartAttempts < RT_DESYNC_MAX_RESTARTS &&
                    restartGateThrottle)
                {
                    garudaData.desyncRestartAttempts++;
                    STARTUP_Init(&garudaData);
                    garudaData.state = ESC_ALIGN;
                    LED2 = 1;
                }
#if FEATURE_THROTTLE_ZERO_AUTO_DISARM
                else if (garudaData.throttle < ARM_THROTTLE_ZERO_ADC)
                {
                    /* Throttle is zero — user wants motor stopped.
                     * Don't restart, go to IDLE. */
                    garudaData.runCommandActive = false;
                    garudaData.desyncRestartAttempts = 0;
                    garudaData.state = ESC_IDLE;
                    LED2 = 0;
                }
#endif
                else if (!garudaData.runCommandActive)
                {
                    /* User pressed stop during coast — graceful idle */
                    garudaData.desyncRestartAttempts = 0;
                    garudaData.state = ESC_IDLE;
                    LED2 = 0;
                }
                else
                {
                    /* Max restarts exhausted — permanent fault */
                    garudaData.runCommandActive = false;
                    garudaData.state = ESC_FAULT;
                    garudaData.faultCode = FAULT_DESYNC;
                    LED2 = 0;
                }
            }
#endif
            break;

        case ESC_FAULT:
            break;
    }

    TIMER1_InterruptFlagClear();
}

#ifdef ENABLE_PWM_FAULT_PCI
/**
 * @brief PWM Fault ISR — handles PCI fault events.
 */
void __attribute__((__interrupt__, no_auto_psv)) _PWMInterrupt(void)
{
    if (PCI_FAULT_ACTIVE_STATUS)
    {
        /* Board-level FPCI fault via PCI8R (RP28/DIM040).
         * Source: U25A (overvoltage) + U25B (overcurrent) → U27 AND gate.
         * This is a combined OC+OV signal — cannot distinguish which triggered. */
#if FEATURE_ADC_CMP_ZC
        if (garudaData.hwzc.enabled)
            HWZC_Disable(&garudaData);
#endif
#if FEATURE_HW_OVERCURRENT
        HAL_CMP3_SetThreshold(RT_OC_CMP3_STARTUP_DAC);
#endif
        HAL_MC1ClearPWMPCIFault();
        HAL_MC1PWMDisableOutputs();
        garudaData.state = ESC_FAULT;
        garudaData.faultCode = FAULT_BOARD_PCI;
        garudaData.runCommandActive = false;
#if FEATURE_ADC_CMP_ZC
        garudaData.hwzc.fallbackPending = false;
#endif
        LED2 = 0;
    }
    ClearPWMIF();
}
#endif /* ENABLE_PWM_FAULT_PCI */

#if FEATURE_ADC_CMP_ZC
#if GARUDA_TARGET_AK512
/**
 * @brief MC510: one comparator ISR per dedicated BEMF channel.
 * VA = AD1CH1 → AD1CMP1, VB = AD1CH2 → AD1CMP2, VC = AD2CH2 → AD2CMP2.
 * Same stub semantics as the 106 pair: disable own IE, clear own CMPSTAT
 * flag, dispatch HWZC_OnZcDetected (Rule 10: no data re-read here).
 */
void __attribute__((__interrupt__, no_auto_psv)) _AD1CMP1Interrupt(void)
{
    _AD1CMP1IE = 0;              /* Disable immediately to prevent re-trigger */
    AD1CMPSTATbits.CH1FLG = 0;   /* Clear comparator status */
    if (garudaData.hwzc.enabled)
        HWZC_OnZcDetected(&garudaData);
    _AD1CMP1IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _AD1CMP2Interrupt(void)
{
    _AD1CMP2IE = 0;
    AD1CMPSTATbits.CH2FLG = 0;
    if (garudaData.hwzc.enabled)
        HWZC_OnZcDetected(&garudaData);
    _AD1CMP2IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _AD2CMP2Interrupt(void)
{
    _AD2CMP2IE = 0;
    AD2CMPSTATbits.CH2FLG = 0;
    if (garudaData.hwzc.enabled)
        HWZC_OnZcDetected(&garudaData);
    _AD2CMP2IF = 0;
}
#else
/**
 * @brief ADC1 Comparator CH5 ISR — ZC detected on Phase B.
 * Do NOT re-read AD1CH5DATA for validation (Rule 10).
 */
void __attribute__((__interrupt__, no_auto_psv)) _AD1CMP5Interrupt(void)
{
    _AD1CMP5IE = 0;              /* Disable immediately to prevent re-trigger */
    AD1CMPSTATbits.CH5CMP = 0;  /* Clear comparator status */
    if (garudaData.hwzc.enabled)
        HWZC_OnZcDetected(&garudaData);
    _AD1CMP5IF = 0;
}

/**
 * @brief ADC2 Comparator CH1 ISR — ZC detected on Phase A or C.
 */
void __attribute__((__interrupt__, no_auto_psv)) _AD2CMP1Interrupt(void)
{
    _AD2CMP1IE = 0;
    AD2CMPSTATbits.CH1CMP = 0;
    if (garudaData.hwzc.enabled)
        HWZC_OnZcDetected(&garudaData);
    _AD2CMP1IF = 0;
}
#endif /* GARUDA_TARGET_AK512 */

/**
 * @brief SCCP1 Timer ISR — blanking expired, commutation deadline, or timeout.
 * Action determined by hwzc.phase (Rule 3: phase is set BEFORE timer starts).
 */
void __attribute__((__interrupt__, no_auto_psv)) _CCT1Interrupt(void)
{
    if (!garudaData.hwzc.enabled)
    {
        /* HWZC was disabled (fault, button stop, throttle-zero shutdown)
         * while a timer event was pending. Discard — do not commutate. */
        _CCT1IF = 0;
        return;
    }

    switch (garudaData.hwzc.phase)
    {
        case HWZC_BLANKING:
            HWZC_OnBlankingExpired(&garudaData);
            break;
        case HWZC_WATCHING:
#if FEATURE_HWZC_SECTOR_PI
            /* PI mode: SCCP1 firing in WATCHING phase is the autonomous
             * period-expired event (NOT a timeout). Run PI math + commutate. */
            HWZC_OnPiPeriodExpired(&garudaData);
#else
            HWZC_OnTimeout(&garudaData);
#endif
            break;
        case HWZC_COMM_PENDING:
            HWZC_OnCommDeadline(&garudaData);
            break;
        default:
            break;
    }
    _CCT1IF = 0;
}
#endif /* FEATURE_ADC_CMP_ZC */
