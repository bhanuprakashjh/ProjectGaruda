/**
 * @file  an1078_smc.h
 * @brief AN1078 Sliding-Mode Observer — float port (header).
 *
 * Direct field-by-field translation of AN1078 `smcpos.h` SMC struct
 * to single-precision float.  All fields renamed Q-style → camelCase
 * to match modern style, but each one corresponds 1:1 to a Q15 field
 * in the original.
 *
 * Comments map names to AN1078 originals where divergent.
 */
#ifndef AN1078_SMC_H
#define AN1078_SMC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── SMC observer state (mirrors AN1078 SMC struct) ─────────── */
typedef struct {
    /* Inputs (filled by caller before each tick) */
    float Valpha;        /* Stationary α-axis stator voltage    [V] */
    float Vbeta;         /* Stationary β-axis stator voltage    [V] */
    float Ialpha;        /* Stationary α-axis stator current    [A] */
    float Ibeta;         /* Stationary β-axis stator current    [A] */

    /* Internal state */
    float EstIalpha;     /* Estimated α current                 [A] */
    float EstIbeta;      /* Estimated β current                 [A] */
    float IalphaError;   /* α current estimation error          [A] */
    float IbetaError;    /* β current estimation error          [A] */

    float Ealpha;        /* Stage-1 α back-EMF (raw, fed to model) [V] */
    float Ebeta;         /* Stage-1 β back-EMF                  [V] */
    float EalphaFinal;   /* Stage-2 α back-EMF (filtered, → angle) [V] */
    float EbetaFinal;    /* Stage-2 β back-EMF                  [V] */

    float Zalpha;        /* α sliding output (Z signal)         [V] */
    float Zbeta;         /* β sliding output                    [V] */

    /* Configuration / tuning */
    float Fsmopos;       /* Plant pole       1 - Rs·Ts/Ls       [-] */
    float Gsmopos;       /* Plant gain       Ts/Ls              [s/H] */
    float Kslide;        /* Sliding gain (max |Z|)              [V] */
    float MaxSMCError;   /* Linear-region half-width            [A] */

    float Kslf;          /* Stage-1 LPF coefficient (this tick) [-] */
    float KslfFinal;     /* Stage-2 LPF coefficient (= Kslf)    [-] */
    float KslfScale;     /* Scale: Kslf = |ω| × KslfScale       [s] */
    float KslfMin;       /* Floor for Kslf                      [-] */

    float ThetaOffset;   /* Constant phase shift                [rad] */

    /* Outputs */
    float Theta;         /* Estimated rotor electrical angle    [rad, 0..2π) */
    float Omega;         /* Speed estimate (per IRP_PERCALC)    [rad/s elec] */
    float OmegaFltred;   /* LP-filtered speed for Kslf scaling  [rad/s elec] */

    /* Speed-estimator running accumulators */
    float PrevTheta;     /* θ at previous tick                  [rad] */
    float AccumTheta;    /* Σ Δθ over AccumThetaCnt ticks       [rad] */
    uint16_t AccumThetaCnt;  /* tick counter for averaging */
} AN_SMC_T;

/* ── Initialization ────────────────────────────────────────── */

/**
 * Initialize SMC.  Computes Fsmopos, Gsmopos from supplied
 * Rs, Ls, Ts and stores tuning constants from an1078_params.h.
 */
void AN_SMCInit(AN_SMC_T *s);

/**
 * Reset all estimator state to zero (preserves Fsmopos, Gsmopos,
 * Kslide, MaxSMCError, etc.).  Called at OL→CL handoff and on motor
 * stop, mirroring AN1078's pmsm.c:319 reset block. */
void AN_SMCReset(AN_SMC_T *s);

/* ── Per-tick update ───────────────────────────────────────── */

/**
 * Run one observer step.  Caller fills Valpha, Vbeta, Ialpha, Ibeta
 * BEFORE calling.  Updates EstI*, IerrorErr*, Z*, E*, EFinal*, Theta.
 * Updates Omega/OmegaFltred every IRP_PERCALC ticks.
 *
 * Direct port of AN1078 SMC_Position_Estimation_Inline().
 */
void AN_SMC_Position_Estimation(AN_SMC_T *s);

#ifdef __cplusplus
}
#endif

#endif /* AN1078_SMC_H */
