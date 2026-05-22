/**
 * @file  esmo_observer.c
 * @brief Enhanced SMO observer — skeleton (math identical to AN1078).
 *
 * Phase 1 step 0: copy AN1078 SMO math byte-for-byte so the FEATURE_FOC_ESMO
 * flag can be flipped and the motor behaves identically. Subsequent commits
 * swap one piece at a time (Ed/|E| → adaptive Kp → closed-form offset →
 * adaptive Kslide → speed LPF) so any regression is bisectable.
 */

#include "../garuda_config.h"

#if FEATURE_FOC_ESMO

#include "esmo_observer.h"
#include "an1078_params.h"
#include "pll_estimator.h"
#include "foc_shim_gsp.h"
#include <math.h>

#define ESMO_TWO_PI    6.28318530717958647692f
#define ESMO_PI        3.14159265358979323846f

/* Adaptive Kslide ramp (task 120).
 *   MODE 0 (STATIC): Kslide held at esmo_tune_kslide() value each tick.
 *                    GUI changes take effect immediately.  (Default.)
 *   MODE 1 (RAMP):   Kslide starts at KslideMin on reset, +STEP per tick
 *                    until KslideMax = esmo_tune_kslide().  GUI sets the
 *                    ceiling; ramp gives smoother startup with lower Z
 *                    when BEMF is small.  Reset → restart ramp.
 *
 * Step sized so full ramp completes in ~50 ms at 30 kHz:
 *   (KslideMax - KslideMin)/STEP = 50ms × 30kHz = 1500 ticks
 *   For typical (KslideMin=0.5, KslideMax=2.5) → STEP ≈ 0.00133 V/tick.
 * TI uses 0.000002 at 10kHz (would take 5 minutes!) — we're 1000× faster. */
#ifndef ESMO_KSLIDE_MODE
#define ESMO_KSLIDE_MODE    0      /* 0=STATIC, 1=ADAPTIVE RAMP */
#endif
#ifndef ESMO_KSLIDE_MIN
#define ESMO_KSLIDE_MIN     0.5f   /* startup floor, V */
#endif
#ifndef ESMO_KSLIDE_STEP
#define ESMO_KSLIDE_STEP    0.0013f /* V per tick */
#endif

/* Speed LPF on OmegaFltred (the value fed to the outer speed PI in
 * an1078_motor.c).  Does NOT touch theta integration — pure speed-
 * loop input smoothing.  Enabled 2026-05-22 after bench showed
 * residual ω wobble at top end was driving Iq spikes in the speed PI,
 * which cross-coupled into Id excursions to ±22 A.  Filtering ω here
 * lets the speed PI see a stable feedback signal without slowing
 * theta tracking.
 *   MODE 0: OmegaFltred = pll.omega_est directly.
 *   MODE 1 (default now): OmegaFltred = b0·ω + a1·OmegaFltred,
 *                         a1 = 1/(1+2π·fc·Ts), b0 = 1-a1.
 * fc = 200 Hz at 30 kHz Ts → a1 ≈ 0.96, b0 ≈ 0.04.  Bump fc higher
 * (300–500 Hz) if throttle feels sluggish. */
#ifndef ESMO_SPEED_LPF_MODE
#define ESMO_SPEED_LPF_MODE 1       /* 0=passthrough, 1=LPF (default ON) */
#endif
#ifndef ESMO_SPEED_LPF_FC_HZ
#define ESMO_SPEED_LPF_FC_HZ 200.0f
#endif

/* ── Live tuning helpers (mirror an1078_smc.c, shared semantics) ─── */
static inline float esmo_tune_theta_base(void)
{
    uint16_t v = gspParams.an1078ThetaBaseDegX10;
    return (v != 0) ? ((float)v * 0.1f * (ESMO_PI / 180.0f))
                    : AN_SMC_THETA_OFFSET_BASE;
}
static inline float esmo_tune_theta_k(void)
{
    uint16_t v = gspParams.an1078ThetaKE7;
    return (float)v * 1.0e-7f;
}
static inline float esmo_tune_kslide(void)
{
    uint16_t v = gspParams.an1078KslideMv;
    return (v != 0) ? ((float)v * 0.001f) : AN_SMC_KSLIDE;
}

static inline float esmo_abs(float x) { return (x >= 0.0f) ? x : -x; }
static inline float esmo_wrap_2pi(float th)
{
    while (th >= ESMO_TWO_PI) th -= ESMO_TWO_PI;
    while (th <  0.0f)        th += ESMO_TWO_PI;
    return th;
}
static inline float esmo_wrap_delta(float dth)
{
    if (dth >  ESMO_PI) dth -= ESMO_TWO_PI;
    if (dth < -ESMO_PI) dth += ESMO_TWO_PI;
    return dth;
}

/* ── Init / Reset (identical to AN_SMC for skeleton) ─────────── */

void ESMO_Init(AN_SMC_T *s)
{
    float Rs = (gspParams.focRsMilliOhm > 0)
             ? gspParams.focRsMilliOhm * 1.0e-3f
             : AN_MOTOR_RS;
    float Ls = (gspParams.focLsMicroH > 0)
             ? gspParams.focLsMicroH * 1.0e-6f
             : AN_MOTOR_LS;

    s->Fsmopos = 1.0f - Rs * AN_TS / Ls;
    s->Gsmopos = AN_TS / Ls;

#if ESMO_KSLIDE_MODE == 1
    s->Kslide = ESMO_KSLIDE_MIN;           /* ramp starts at floor */
#else
    s->Kslide = esmo_tune_kslide();        /* static — GUI-tunable each tick */
#endif
    s->MaxSMCError = AN_SMC_MAX_LINEAR_ERR;

    s->KslfScale = AN_SMC_KSLF_SCALE;
    s->KslfMin   = AN_SMC_KSLF_MIN;
    s->Kslf      = s->KslfMin;
    s->KslfFinal = s->KslfMin;

    s->ThetaOffset = esmo_tune_theta_base();

    ESMO_Reset(s);
}

void ESMO_Reset(AN_SMC_T *s)
{
    s->Valpha = 0.0f;
    s->Ealpha = 0.0f;
    s->EalphaFinal = 0.0f;
    s->Zalpha = 0.0f;
    s->EstIalpha = 0.0f;
    s->Vbeta = 0.0f;
    s->Ebeta = 0.0f;
    s->EbetaFinal = 0.0f;
    s->Zbeta = 0.0f;
    s->EstIbeta = 0.0f;
    s->Ialpha = 0.0f;
    s->IalphaError = 0.0f;
    s->Ibeta = 0.0f;
    s->IbetaError = 0.0f;
    s->Theta = 0.0f;
    s->Omega = 0.0f;
    s->OmegaFltred = 0.0f;

    s->PrevTheta = 0.0f;
    s->AccumTheta = 0.0f;
    s->AccumThetaCnt = 0;

    s->Kslf      = s->KslfMin;
    s->KslfFinal = s->KslfMin;

#if ESMO_KSLIDE_MODE == 1
    s->Kslide = ESMO_KSLIDE_MIN;           /* restart ramp on reset */
#endif

    pll_reset(&s->pll);
}

/* ── CalcEstI: current observer + sliding switching ───────────── */
static inline void EsmoCalcEstI(AN_SMC_T *s)
{
    s->EstIalpha = s->Fsmopos * s->EstIalpha
                 + s->Gsmopos * (s->Valpha - s->Ealpha - s->Zalpha);
    s->EstIbeta  = s->Fsmopos * s->EstIbeta
                 + s->Gsmopos * (s->Vbeta  - s->Ebeta  - s->Zbeta);

    s->IalphaError = s->EstIalpha - s->Ialpha;
    s->IbetaError  = s->EstIbeta  - s->Ibeta;

    if (esmo_abs(s->IalphaError) < s->MaxSMCError) {
        s->Zalpha = (s->Kslide * s->IalphaError) / s->MaxSMCError;
    } else if (s->IalphaError > 0.0f) {
        s->Zalpha =  s->Kslide;
    } else {
        s->Zalpha = -s->Kslide;
    }

    if (esmo_abs(s->IbetaError) < s->MaxSMCError) {
        s->Zbeta = (s->Kslide * s->IbetaError) / s->MaxSMCError;
    } else if (s->IbetaError > 0.0f) {
        s->Zbeta =  s->Kslide;
    } else {
        s->Zbeta = -s->Kslide;
    }
}

/* ── CalcBEMF: 2-stage LPF on Z to extract back-EMF ───────────── */
static inline void EsmoCalcBEMF(AN_SMC_T *s)
{
    s->Ealpha      += s->Kslf      * (s->Zalpha - s->Ealpha);
    s->EalphaFinal += s->KslfFinal * (s->Ealpha - s->EalphaFinal);

    s->Ebeta       += s->Kslf      * (s->Zbeta  - s->Ebeta);
    s->EbetaFinal  += s->KslfFinal * (s->Ebeta  - s->EbetaFinal);
}

/* ── ESMO PLL: Ed/|E| projection with full-quadrant pull-in + adaptive Kp
 *
 * Discriminator:
 *   Ed = Eα·cos + Eβ·sin (parallel — drive to 0)
 *   Eq = Eβ·cos − Eα·sin (perpendicular, used only for sign-flipping)
 *   |E| = √(Eα² + Eβ²)
 *   err = ±Ed / |E|   (sign flipped when Eq ≥ 0 → full-quadrant pull-in)
 *
 * Adaptive Kp (task 118):
 *   Kp = clamp(KpMin + KpSF·|ω_est|, KpMin, KpMax)
 *
 * The Ed/|E| normalization removes the BEMF-amplitude dependency from
 * the loop gain, so Kp can be scaled with speed (which is what we want
 * for tracking at higher fundamental frequencies).  Defaulted to
 * KpMin=KpMax with KpSF=0 → CONSTANT Kp (matches the old PLL fixed-
 * gain behavior).  Bump KpSF in `an1078_params.h` to engage adaptive
 * gain — start at ESMO_PLL_KP_SF=0.025 (2× at top of speed range). */
#define ESMO_BEMF_MAG_MIN  0.001f   /* 1 mV — below this, error → 0 */

/* Adaptive Kp defaults — enabled 2026-05-22, then KpMax dialed back from
 * 5× to 3× when first bench showed clean acceleration up to 219k eRPM
 * peak but underdamped Id oscillation (±20A) above 175k.  The 5× gain
 * at top end was tracking rotor well but amplifying BEMF noise; the
 * PI loop was underdamped at that gain.
 *
 * 3× cap saturates the ramp at ω≈10k rad/s (~100k eRPM).  Below that:
 * progressively higher gain helps tracking.  Above: gain stays at 3×
 * — enough for the speed range without amplifying noise into Id.
 *
 * Bench-tune by overriding any of these defines via MPLAB. */
#ifndef ESMO_PLL_KP_MIN
#define ESMO_PLL_KP_MIN    PLL_KP            /* 628 — low-speed Kp */
#endif
#ifndef ESMO_PLL_KP_MAX
#define ESMO_PLL_KP_MAX    (3.0f * PLL_KP)   /* 1884 — high-speed Kp.
                                              * Bench tuning history:
                                              *   5× + 100 Hz LPF: idle
                                              *     unstable (Id ±9A,
                                              *     Iq ±23A, ω 18-32k).
                                              *   4× + 200 Hz LPF: no
                                              *     measurable improvement
                                              *     (Id span 17.1 → 18.5 A,
                                              *     well within run-to-run
                                              *     noise).
                                              * 3× is the right answer; the
                                              * TI 2-stage LPF (ESMO_TI_PLL=1)
                                              * is the real gain. */
#endif
#ifndef ESMO_PLL_KP_SF
#define ESMO_PLL_KP_SF     0.125f            /* same slope; saturates at lower ω */
#endif

/* TI-style PLL: integrate LPF'd omega into theta instead of raw omega.
 * Port of esmo.c:291-297 (ti-c2000-motor-control-sdk) — TI inserts a
 * single-pole LPF on PLL output before using it for θ integration.
 * Theory: per-tick (Kp·err + integrator) has high-frequency content from
 * BEMF noise and PWM ripple.  Filtering before integrating gives a
 * cleaner θ trajectory → cleaner Park transform → less spurious Id/Iq
 * mixing.  Trade-off: small θ lag (≈ 1/(2π·fc) seconds) during ω
 * transients.  At fc=200Hz the lag is ~0.8ms = 1.5° of theta at top
 * speed (23000 rad/s).
 *
 * Default OFF so first builds match prior behavior — flip to 1 to bench. */
#ifndef ESMO_TI_PLL
#define ESMO_TI_PLL                 1
#endif
#ifndef ESMO_TI_PLL_LPF_FC_HZ
#define ESMO_TI_PLL_LPF_FC_HZ       200.0f
#endif

static inline void EsmoPllUpdate(PLL_t *pll, float e_alpha, float e_beta)
{
    float c = cosf(pll->theta_est);
    float sv = sinf(pll->theta_est);

    float Ed   = e_alpha * c + e_beta * sv;
    float Eq   = e_beta  * c - e_alpha * sv;
    float Emag = sqrtf(e_alpha * e_alpha + e_beta * e_beta);

    float err;
    if (Emag < ESMO_BEMF_MAG_MIN) {
        err = 0.0f;                 /* BEMF below noise floor — don't move */
    } else {
        float sign = (Eq >= 0.0f) ? -1.0f : 1.0f;
        err = sign * Ed / Emag;
    }

    /* Adaptive Kp: scale with current |ω_est|.  When KpSF=0 (default
     * until tuned), reduces to constant Kp = KpMin = KpMax = PLL_KP. */
    float omega_abs = (pll->omega_est >= 0.0f) ? pll->omega_est : -pll->omega_est;
    float Kp = ESMO_PLL_KP_MIN + ESMO_PLL_KP_SF * omega_abs;
    if (Kp < ESMO_PLL_KP_MIN) Kp = ESMO_PLL_KP_MIN;
    if (Kp > ESMO_PLL_KP_MAX) Kp = ESMO_PLL_KP_MAX;

    pll->integrator += PLL_KI * AN_TS * err;
    float omega = Kp * err + pll->integrator;

    if (omega >  PLL_SPEED_CLAMP) omega =  PLL_SPEED_CLAMP;
    if (omega < -PLL_SPEED_CLAMP) omega = -PLL_SPEED_CLAMP;
    pll->omega_est = omega;

#if ESMO_TI_PLL
    /* TI pattern: LPF the PLL output, integrate the FILTERED ω into θ.
     * Coefficients are compile-time const-folded by XC-DSC to a single
     * MAC.  speed_flt converges to omega in steady state, so locked
     * tracking is unaffected; only per-tick jitter is suppressed. */
    {
        const float two_pi_fc_ts = 2.0f * ESMO_PI * ESMO_TI_PLL_LPF_FC_HZ * AN_TS;
        const float a1 = 1.0f / (1.0f + two_pi_fc_ts);
        const float b0 = 1.0f - a1;
        pll->speed_flt = b0 * omega + a1 * pll->speed_flt;
        pll->theta_est += pll->speed_flt * AN_TS;
    }
#else
    pll->theta_est += omega * AN_TS;
#endif

    while (pll->theta_est >= ESMO_TWO_PI) pll->theta_est -= ESMO_TWO_PI;
    while (pll->theta_est <  0.0f)        pll->theta_est += ESMO_TWO_PI;
}

/* ── Main per-tick update ─────────────────────────────────────── */
void ESMO_Update(AN_SMC_T *s)
{
    EsmoCalcEstI(s);
    EsmoCalcBEMF(s);

    /* ESMO PLL — Ed/|E| projection + Eq sign flip for full-quadrant
     * pull-in.  Replaces AN1078's cross-product discriminator. */
    EsmoPllUpdate(&s->pll, s->EalphaFinal, s->EbetaFinal);

    /* Theta offset — two modes selectable at compile time:
     *   MODE 0 (LINEAR, default): BASE + K·|ω|.  Matches AN1078 baseline.
     *   MODE 1 (ATAN2): BASE + atan2(|ω|·offsetSF, Kslf).  Closed-form
     *     LPF phase-lag compensation — accounts for Kslf saturation
     *     automatically (linear K can't handle the high-speed knee).
     *
     * Switching to mode 1 requires picking offsetSF.  Starting guess
     * derived from the current K=1e-4: the linear K·ω equals atan(K·ω)
     * for small angles, and atan2(ω·SF, Kslf) ≈ atan(ω·SF/Kslf).  At
     * ω=10000, Kslf=0.5, K=1e-4 gives K·ω=1.0 rad — too big to be
     * linear-region.  offsetSF = K·Kslf/1 ≈ 5e-5 ish; will need bench
     * tuning. */
#ifndef ESMO_THETA_OFFSET_MODE
#define ESMO_THETA_OFFSET_MODE  0       /* 0=LINEAR, 1=ATAN2 */
#endif
#ifndef ESMO_OFFSET_SF
#define ESMO_OFFSET_SF          5.0e-5f /* atan2 scale, mode 1 only */
#endif

    {
        float omega_abs = esmo_abs(s->pll.omega_est);
        float dyn_offset = esmo_tune_theta_base();
#if ESMO_THETA_OFFSET_MODE == 1
        /* atan2 form — closed-form LPF lag.  Use the larger of Kslf
         * and KslfMin so the denominator is never zero at startup. */
        float kslf = (s->Kslf > s->KslfMin) ? s->Kslf : s->KslfMin;
        dyn_offset += atan2f(omega_abs * ESMO_OFFSET_SF, kslf);
#else
        /* linear form — original AN1078 behavior */
        dyn_offset += esmo_tune_theta_k() * omega_abs;
#endif
        /* CRITICAL: ESMO's Ed/|E| PLL tracks ROTOR angle directly
         * (Park-frame convention: Ed=0 at lock when theta_pll=theta_rotor).
         * The old cross-product PLL tracked BEMF angle (rotor + π/2), and
         * needed PLL_ANGLE_OFFSET = π/2 subtraction to get rotor angle.
         * For ESMO the subtraction is WRONG — produces 90° lag → motor
         * runs backwards in CL (bug found 2026-05-22).  Skip the offset. */
        s->Theta = s->pll.theta_est + dyn_offset;
        s->Theta = esmo_wrap_2pi(s->Theta);

#if ESMO_KSLIDE_MODE == 1
        /* Ramp Kslide up toward GUI ceiling (esmo_tune_kslide()).  Once
         * we hit the cap, ramp stops; reset restarts it. */
        float kmax = esmo_tune_kslide();
        if (s->Kslide < kmax) {
            s->Kslide += ESMO_KSLIDE_STEP;
            if (s->Kslide > kmax) s->Kslide = kmax;
        } else if (s->Kslide > kmax) {
            /* GUI lowered ceiling — track down immediately, no ramp */
            s->Kslide = kmax;
        }
#else
        s->Kslide = esmo_tune_kslide();
#endif
    }

    {
        float dth = esmo_wrap_delta(s->Theta - s->PrevTheta);
        s->AccumTheta += dth;
        s->PrevTheta = s->Theta;
    }
    s->AccumThetaCnt++;
    if (s->AccumThetaCnt >= AN_IRP_PERCALC) {
        s->Omega = s->AccumTheta / ((float)AN_IRP_PERCALC * AN_TS);
        s->AccumTheta = 0.0f;
        s->AccumThetaCnt = 0;
    }

#if ESMO_SPEED_LPF_MODE == 1
    {
        /* a1, b0 computed once via compile-time const fold — XC-DSC
         * resolves these at compile time so this is a single MAC. */
        const float two_pi_fc_ts = 2.0f * ESMO_PI * ESMO_SPEED_LPF_FC_HZ * AN_TS;
        const float a1 = 1.0f / (1.0f + two_pi_fc_ts);
        const float b0 = 1.0f - a1;
        s->OmegaFltred = b0 * s->pll.omega_est + a1 * s->OmegaFltred;
    }
#else
    s->OmegaFltred = s->pll.omega_est;
#endif

    {
        float o = esmo_abs(s->OmegaFltred);
        float k = o * s->KslfScale;
        if (k < s->KslfMin)      k = s->KslfMin;
        if (k > AN_SMC_KSLF_MAX) k = AN_SMC_KSLF_MAX;
        s->Kslf      = k;
        s->KslfFinal = k;
    }
}

#endif /* FEATURE_FOC_ESMO */
