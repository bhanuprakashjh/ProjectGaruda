/**
 * @file garuda_types.h
 * @brief Core data structures for 6-step BLDC on dsPIC33CK + ATA6847.
 */

#ifndef GARUDA_TYPES_H
#define GARUDA_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "garuda_config.h"

/* ESC state machine */
typedef enum {
    ESC_IDLE = 0,
    ESC_ARMED,
    ESC_ALIGN,
    ESC_OL_RAMP,
    ESC_CLOSED_LOOP,
    ESC_RECOVERY,
    ESC_FAULT
} ESC_STATE_T;

/* Phase output states for 6-step commutation */
typedef enum {
    PHASE_PWM_ACTIVE = 0,
    PHASE_LOW,
    PHASE_FLOAT
} PHASE_STATE_T;

/* Commutation step descriptor */
typedef struct {
    PHASE_STATE_T phaseA;
    PHASE_STATE_T phaseB;
    PHASE_STATE_T phaseC;
    uint8_t floatingPhase;   /* 0=A, 1=B, 2=C */
    int8_t  zcPolarity;      /* +1=rising, -1=falling */
} COMMUTATION_STEP_T;

/* BEMF state (ATA6847 digital comparator) */
typedef struct {
    bool     zeroCrossDetected;
    uint8_t  cmpPrev;         /* Previous comparator state (0 or 1, 0xFF=unknown) */
    uint8_t  cmpExpected;     /* Expected post-ZC state */
    uint8_t  filterCount;     /* Consecutive matching reads */
} BEMF_STATE_T;

/* Commutation timing — Timer1 ticks (50 us) + high-res SCCP4 ticks (640 ns) */
typedef struct {
    uint16_t stepPeriod;            /* Current step period in Timer1 ticks */
    uint16_t lastCommTick;          /* Timer1 tick at last commutation */
    uint16_t lastZcTick;            /* Timer1 tick at last ZC */
    uint16_t prevZcTick;
    uint16_t zcInterval;
    uint16_t prevZcInterval;        /* Previous zcInterval for 2-step averaging */
    uint16_t commDeadline;          /* Timer1 tick for next commutation (fallback) */
    uint16_t forcedCountdown;
    uint16_t goodZcCount;
    uint16_t checkpointStepPeriod;       /* Per-revolution checkpoint (ESCape32-style) */
#if FEATURE_IC_ZC
    uint16_t checkpointStepPeriodHR;    /* HR checkpoint for high-speed desync detection */
#endif
    uint8_t  revStepCount;               /* Steps since last revolution checkpoint (0-5) */
    uint8_t  consecutiveMissedSteps;
    uint8_t  stepsSinceLastZc;
    bool     zcSynced;
    bool     deadlineActive;
    bool     hasPrevZc;
    bool     bypassSuppressed;     /* True from CL entry until first confirmed ZC.
                                    * Prevents polarity bypass from arming before
                                    * the estimator has a valid reference. */
#if FEATURE_IC_ZC
    /* High-resolution timing via SCCP4 free-running timer (640 ns/tick).
     * stepPeriodHR provides 78x better resolution than Timer1-based stepPeriod.
     * At Tp:9 (22k eRPM): Timer1 = 9 ticks, HR = 703 ticks. */
    uint16_t lastZcTickHR;          /* SCCP4 tick at last ZC */
    uint16_t prevZcTickHR;
    uint16_t zcIntervalHR;
    uint16_t prevZcIntervalHR;
    uint16_t stepPeriodHR;          /* Step period in SCCP4 ticks (640 ns each) */
    bool     hasPrevZcHR;
#endif
} TIMING_STATE_T;

/* ZC timeout result */
typedef enum {
    ZC_TIMEOUT_NONE = 0,
    ZC_TIMEOUT_FORCE_STEP,
    ZC_TIMEOUT_DESYNC
} ZC_TIMEOUT_RESULT_T;

/* ZC fast-poll state (FEATURE_IC_ZC) — SCCP1 periodic timer model */
#if FEATURE_IC_ZC
typedef enum {
    IC_ZC_BLANKING = 0, /* Waiting for blanking period to expire */
    IC_ZC_ARMED,        /* Polling active, waiting for ZC */
    IC_ZC_DONE          /* ZC accepted, polling idle for this step */
} IC_ZC_PHASE_T;

typedef struct {
    IC_ZC_PHASE_T phase;        /* State machine phase */
    uint8_t  activeChannel;     /* Which phase to poll: 0=A, 1=B, 2=C */
    uint8_t  pollFilter;        /* Consecutive matching reads (deglitch) */
    uint8_t  filterLevel;       /* Current deglitch threshold (adaptive) */
    uint16_t blankingEndHR;     /* SCCP4 tick when blanking expires */
    uint16_t lastCommHR;        /* SCCP4 tick at last commutation */
    uint16_t zcCandidateHR;     /* SCCP4 tick at first matching read */
    uint16_t zcCandidateT1;     /* Timer1 tick at first matching read */
    /* Diagnostics */
    uint16_t diagDmaPrimaryAccept; /* ZCs accepted via DMA-primary path */
    uint16_t diagDmaPrimaryMiss;   /* DMA-primary: no qualifying cluster, fell through to poll */
    uint16_t diagAccepted;      /* ZCs accepted via fast poll */
    uint16_t diagLcoutAccepted; /* ZCs accepted via ADC ISR backup */
    uint16_t diagFalseZc;       /* Rejected by RecordZcTiming */
    uint16_t diagPollCycles;    /* Total poll ISR invocations */
    /* Raw corroboration (candidate-local, TRACK mode) */
    uint8_t  rawCoro;           /* Saturating 0..2: raw GPIO agrees with CLC during candidate */
    bool     hasFirstClcMatch;  /* true after CLC first matches expected this candidate */
    uint16_t firstClcMatchHR;   /* HR timestamp of first CLC==expected this candidate */
    /* Phase 2: raw stability tracking (candidate-local) */
    bool     hasRawFirstMatch;  /* true after raw first matches expected within current CLC candidate */
    uint16_t rawFirstMatchHR;   /* HR timestamp of first raw==expected within CLC candidate */
    uint16_t rawFirstMatchT1;   /* Timer1 tick at first raw==expected (for timestamp selection) */
    /* Raw comparator edge trace */
    uint8_t  lastCmpState;      /* previous raw comparator read (0 or 1) */
    uint16_t stepFlips[6];      /* comparator transitions per step (glitch count) */
    uint16_t stepPolls[6];      /* total polls per step (for rate calculation) */
    /* Diagnostics — raw corroboration & IC age */
    uint16_t diagRawVeto;       /* CLC matched but raw didn't corroborate (stale D-FF) */
    uint16_t diagIcAgeReject;   /* IC timestamp discarded as stale */
    uint16_t diagTrackFallback; /* ZCs accepted via PWM-aged FL=2 fallback */
    uint16_t diagRawStableBlock; /* Fast accept blocked: CLC candidate but raw not stable */
    /* Phase 2: timestamp source counters */
    uint16_t diagTsFromIc;      /* ZC accepted using IC timestamp */
    uint16_t diagTsFromRaw;     /* ZC accepted using rawFirstMatchHR */
    uint16_t diagTsFromClc;     /* ZC accepted using firstClcMatchHR */
    uint16_t diagTsFromPoll;    /* ZC accepted using current poll time */
    uint16_t diagIcLeadReject;  /* IC timestamp downgraded: IC led raw by too much */
#if FEATURE_IC_ZC_CAPTURE
    /* SCCP2 IC capture state */
    uint16_t icCommStamp;       /* SCCP2 timer at last commutation (IC domain) */
    bool     icArmed;           /* true when IC is armed for capture */
    bool     icCandidateValid;  /* true when IC has captured and timestamp is usable */
    uint16_t diagIcAccepted;    /* ZCs accepted via IC capture */
    uint16_t diagIcBounce;      /* IC fires rejected (bounce after blanking) */
#endif
#if FEATURE_PTG_ZC
    /* PTG edge-relative sampling (supplements CLC D-FF) */
    uint8_t  ptgSample;        /* Raw BEMF from PTG ISR (0 or 1) */
    uint8_t  ptgSampleValid;   /* 1 when fresh PTG sample available */
#endif
} IC_ZC_STATE_T;
#endif

/* ── ZC V2 Mode Enum (Phase 1 scaffolding) ──────────────────────────── */
typedef enum {
    ZC_MODE_ACQUIRE = 0,   /* CL entry / resync: conservative, building trust */
    ZC_MODE_TRACK   = 1,   /* Normal CL: strict polarity, tight timing */
    ZC_MODE_RECOVER = 2    /* Lost sync: wider timeout, no advance, hold duty */
} ZC_MODE_T;

/* ── ZC Controller State (Phase 2+ populates, Phase 1 declares) ────── */
typedef struct {
    ZC_MODE_T mode;
    uint16_t  acquireGoodCount;    /* consecutive good ZCs in ACQUIRE */
    uint16_t  recoverGoodCount;    /* consecutive good ZCs in RECOVER */
    uint8_t   recoverAttempts;     /* RECOVER entries since last stable TRACK */
    uint16_t  refIntervalHR;      /* protected reference interval (Phase 4+) */
    uint16_t  rawIntervalHR;      /* last accepted raw interval (Phase 4+) */
    uint16_t  refIntervalT1;      /* Timer1 protected reference (Phase 4+) */
    uint16_t  rawIntervalT1;      /* Timer1 raw interval (Phase 4+) */
    uint16_t  originalTimeoutHR;  /* timeout captured at commutation for latency */
    uint16_t  demagMetric;        /* demag classification metric (Phase 6+) */
} ZC_CTRL_T;

/* ── PLL Step Predictor State ──────────────────────────────────────── */
#if FEATURE_IC_ZC
typedef struct {
    /* Predictor core */
    uint16_t predStepHR;        /* Predicted 60° step period (HR ticks) */
    uint16_t predNextCommHR;    /* Predicted next commutation time (HR) */
    uint16_t predZcHR;          /* Predicted ZC time for current sector (HR) */
    uint16_t predZcOffsetHR;    /* Adaptive comm-to-ZC phase offset (HR ticks).
                                 * Seeded from lastRealZcDelayHR, then IIR-updated
                                 * from accepted ZCs. Replaces fixed half+advance. */
    int16_t  phaseErrHR;        /* Last phase error: actual - predicted ZC (HR) */
    bool     locked;            /* true when predictor is tracking reliably */
    uint8_t  missCount;         /* Consecutive steps without valid ZC correction */
    uint16_t lastCommHR;        /* HR timestamp of last commutation (predictor domain) */

    /* Scan window (derived from predZcHR) */
    uint16_t scanOpenHR;        /* Earliest acceptable ZC candidate */
    uint16_t scanCloseHR;       /* Latest acceptable ZC candidate */

    /* Telemetry (Step 1: shadow mode — compute but don't schedule) */
    uint16_t diagPredCommCount; /* Commutations where predictor was computed */
    uint16_t diagPhaseErrAccum; /* Sum of |phaseErrHR| for averaging (wrapping) */
    uint16_t diagZcInWindow;    /* ZC fell within predicted scan window */
    uint16_t diagZcOutWindow;   /* ZC fell outside predicted scan window */
    int16_t  diagMinMarginHR;   /* Minimum predNextComm margin observed */

    /* Dual phase-error diagnostic */
    uint16_t lastRealZcDelayHR; /* Delay from last comm to last real ZC (measured) */
    int16_t  phaseErrReactiveHR;/* Phase error vs reactive model B */

    /* Step 2: gate readiness instrumentation.
     * Counters at the exact future veto point (after timestamp selection,
     * before estimator update). gateActive has hysteresis separate from
     * locked: enable after N consecutive locked+in-window, disable on
     * timeout or M out-of-window in one revolution. */
    bool     gateActive;           /* true when gate would be armed */
    uint8_t  gateArmCount;         /* Consecutive locked+in-window ZCs toward arming */
    uint8_t  gateRevRejects;       /* Out-of-window rejects in current revolution */
    /* Shadow counters at the veto site */
    uint16_t diagWinCandInGated;   /* In-window while gateActive */
    uint16_t diagWinCandOutGated;  /* Out-of-window while gateActive */
    uint16_t diagWinOutEarly;      /* ZC before scanOpen (all states) */
    uint16_t diagWinOutLate;       /* ZC after scanClose (all states) */
    /* Live gate counters (FEATURE_PRED_WINDOW_GATE) */
    uint16_t diagWindowReject;     /* ZC vetoed by window gate */
    uint16_t diagWindowRecovered;  /* Vetoed step got a later valid ZC */

    /* Step 3: Predictor-driven commutation scheduling */
    bool     predictiveMode;       /* true when predictor owns commutation scheduling */
    bool     handoffPending;       /* true: let current reactive comm fire, then own next */
    uint16_t handoffTargetHR;      /* Pre-computed first predictor target (set in RecordZcTiming) */
    uint16_t lastPredCommHR;       /* Exact HR time of last predictor-owned commutation */
    uint16_t pendingPredCommHR;    /* Next scheduled predictor commutation target */
    bool     pendingPredValid;     /* true when pendingPredCommHR is programmed */
    /* Reactive scheduling state captured for handoff continuity */
    uint16_t lastReactiveTargetHR; /* Most recent reactive targetHR — seed for
                                    * first predictive target to avoid phase jump */
    uint8_t  lastReactiveTAL;      /* TAL used by reactive — predictor must match */
    /* Shadow delta: predictor vs reactive target comparison */
    int16_t  predVsReactiveDelta;  /* predNextComm - reactiveTarget (HR ticks) */
    uint8_t  deltaOkCount;         /* Consecutive steps with |delta| < threshold */
    /* Entry score: evaluated after RecordZcTiming with current ZC's data.
     * Decoupled from gateActive (which is for supervision only).
     * GREEN + small phaseErr: +4, YELLOW + moderate: -1, RED/timeout: clear. */
    uint8_t  entryScore;           /* 0-255, saturating. Entry at >= 24 (~1 rev) */
    /* Step 3 telemetry */
    uint16_t diagPredCommOwned;    /* Commutations scheduled by predictor */
    uint16_t diagPredEnter;        /* Successful predictive mode entries */
    uint16_t diagPredEntryLate;    /* Handoff aborted: first target already past */
    uint16_t diagPredIsrFired;     /* _CCP4Interrupt entered while predictiveMode already true */
    uint16_t diagPredIsrEntries;   /* Total _CCP4Interrupt entries with deadlineActive */
    uint16_t diagPredExitRed;      /* Predictor exits due to RED zone */
    uint16_t diagPredExitMiss;     /* Predictor exits due to missCount */
    uint16_t diagPredExitYellow;   /* Predictor exits due to repeated YELLOW */
    uint16_t diagPredExitPhaseErr; /* Predictor exits due to large phaseErr */
    uint16_t diagPredExitTimeout;  /* Predictor exits due to timeout */

#if FEATURE_6STEP_DPLL
    /* ── 6-step DPLL state ─────────────────────────────────────────
     * Separates T_hat (predStepHR), measurement bias, and advance.
     * The reactive path's delayHR = halfHR - advHR tangles them.
     *
     * Model:
     *   t_comm[k+1]  = t_comm[k] + T_hat
     *   t_zc_pred[k] = t_comm[k] + T_hat/2 + A_cmd + phaseBiasHR
     *   e[k]         = t_meas[k] - t_zc_pred[k]
     *
     * In V1 (A_cmd=0), phaseBiasHR absorbs poll latency + advance
     * + comparator delay — a combined bias. */
    int16_t  phaseBiasHR;         /* B_hat: combined measurement/phase bias.
                                   * Updated each ZC: += e/8.
                                   * V1: absorbs latency+advance+phase. */
    uint16_t advanceCmdHR;        /* A_cmd: commanded advance (V1: 0) */
    int16_t  advanceTrimHR;       /* Slow trim on advance (future, init 0) */
    uint16_t lastMeasTsHR;        /* Last accepted ZC timestamp used for DPLL */
    uint16_t measDeadlineHR;      /* Measurement timeout: expected ZC + 1.5× step.
                                   * Independent of deadlineActive. */
    uint8_t  fallbackReason;      /* Why we exited predictive mode:
                                   * 0=none, 1=missCount, 2=phaseErr,
                                   * 3=measTimeout, 4=targetPast, 5=notCL */
    uint8_t  graceCount;          /* Steps remaining after entry where
                                   * phaseErr exits are suppressed. The
                                   * bias IIR needs ~12 steps to absorb
                                   * the entry discontinuity. */
    int16_t  dpllErrHR;           /* Last DPLL phase error (tMeas - predZc) */
    uint16_t dmaMeasUsedCount;    /* Steps where DMA passed plausibility gate */
    uint16_t dmaMeasRejectCount;  /* Steps where DMA failed gate → poll used */
    uint16_t predCloseAgreeCount;   /* Predicted-close found same cluster as poll-close */
    uint16_t predCloseDisagreeCount; /* Predicted-close found different or missing */
#endif
} ZC_PRED_T;
#endif

/* ── Sector PI synchronizer state (FEATURE_SECTOR_PI) ──────────────── */
#if FEATURE_SECTOR_PI
typedef struct {
    /* ── Control state ─────────────────────────────────────────────
     * PI synchronizer modeled on Microchip AVR high-speed example.
     * Measures ZC position within sector (elapsed from commutation),
     * compares to expected position, drives sector period via PI.
     *
     * Model:
     *   capValueHR  = t_meas - lastCommHR        (where ZC fell)
     *   setValueHR  = T_hat/2 + detDelay + advCmd (where ZC expected)
     *   syncErrHR   = capValue - setValue
     *   syncIntHR  += syncErr >> KI_SHIFT
     *   T_hatHR     = syncIntHR + (syncErr >> KP_SHIFT)
     *   nextCommHR  = lastCommHR + T_hatHR
     */
    uint16_t T_hatHR;             /* Sector period estimate (60° electrical) */
    uint16_t syncIntHR;           /* PI integrator state */
    uint16_t lastCommHR;          /* HR timestamp of last commutation */
    uint16_t lastMeasHR;          /* HR timestamp of last accepted ZC measurement */
    bool     prevStepRisingZc;    /* Polarity of the step that just completed */

    /* Per-sector measurement (from DMA) */
    uint16_t capValueHR;          /* Elapsed: measHR - lastCommHR */
    uint16_t setValueHR;          /* Expected: T_hat/2 + detDelay + advCmd */
    int16_t  syncErrHR;           /* Phase error: capValue - setValue */

    /* Explicit advance/delay (NOT mixed, NOT estimated online) */
    uint16_t detDelayHR;          /* Fixed detector pipeline delay (HR ticks) */
    uint16_t advanceCmdHR;        /* Speed-dependent torque advance (HR ticks) */

    /* Cluster info from extended DMA selector */
    uint16_t clusterMidHR;        /* Selected cluster midpoint */
    uint8_t  clusterCount;        /* Edges in selected cluster */
    uint16_t clusterWidthHR;      /* Last - first edge in cluster */
    uint8_t  clusterRejectReason; /* 0=accepted, 1=no_edges, 2=no_cluster,
                                   * 3=out_of_corridor, 4=single_edge */

    /* Mode control */
    uint8_t  mode;                /* 0=OFF, 1=SHADOW, 2=OWNED */
    uint8_t  goodStreak;          /* Consecutive sectors with |syncErr| < threshold */
    uint8_t  missStreak;          /* Consecutive sectors with no DMA ZC */
    uint8_t  fallbackReason;      /* Why ownership exited (0=none, 1=miss,
                                   * 2=syncErr, 3=speed, 4=notCL) */

    /* Shadow comparison */
    int16_t  syncVsReactiveDelta; /* nextCommHR(sync) - targetHR(reactive) */

    /* Telemetry counters */
    uint16_t diagSyncAccepts;     /* Sectors with valid DMA measurement */
    uint16_t diagSyncMisses;      /* Sectors with no DMA measurement */
    uint16_t diagSyncEntries;     /* Times ownership was entered */
    uint16_t diagSyncExits;       /* Times ownership was exited */
} ZC_SYNC_T;
#endif

/* ── DMA shadow capture state ───────────────────────────────────────── */
#if FEATURE_IC_DMA_SHADOW
typedef struct {
    /* Per-step probe results, overwritten each commit in RecordZcTiming.
     * uint32 so the counters don't wrap at ~13s at 60k eRPM. */
    uint32_t  stepCount;            /* total probe invocations */
    uint32_t  matchCount;           /* steps where a capture was found in window */
    uint32_t  edgesInWindowSum;     /* running sum for average edges/step */
    uint32_t  ringOverflowCount;    /* ring near-full events (>3/4 consumed) */

    /* Most recent signed deltas (HR ticks) */
    int16_t   lastEarliestVsPoll;      /* DMA earliest − poll timestamp */
    int16_t   lastEarliestVsExpected;  /* DMA earliest − predicted ZC */
    int16_t   lastClosestVsExpected;   /* DMA closest − predicted ZC */
    int16_t   lastPollVsExpected;      /* poll timestamp − predicted ZC (baseline) */

    uint8_t   lastEdgeCount;        /* captures found in most recent step */
    bool      lastFound;            /* true if most recent probe matched */

    /* DMA-direct substitution counters (only meaningful when
     * FEATURE_DMA_ZC_DIRECT=1). Incremented in RecordZcTiming. */
    uint32_t  substituteCount;      /* times hrTick was replaced by DMA edge */
    uint32_t  substituteSkipGated;  /* below speed threshold, left alone */
    uint32_t  substituteSkipRange;  /* DMA edge outside sanity window */
    int16_t   lastCorrectionHR;     /* most recent (refined - poll) HR delta.
                                     * Zeroed each step; only set when DMA is
                                     * actually used this step. 0 = no DMA. */
    int16_t   minCorrectionHR;      /* most negative correction seen so far */
    int16_t   maxCorrectionHR;      /* most positive correction seen so far */
    uint16_t  smoothedLatencyHR;    /* IIR-averaged poll latency (positive HR ticks) */
    uint8_t   measSource;           /* What fed the DPLL this step:
                                     * 0=none, 1=poll, 2=DMA-gated */
} DMA_SHADOW_T;
#endif

/* ── ZC Diagnostics ─────────────────────────────────────────────────── */
typedef struct {
    /* Existing (keep for backward compat) */
    uint8_t  zcLatencyPct;          /* 0-255: ZC window position (0xFF=timeout) */
    uint16_t lastBlankingHR;        /* Layer 1 blanking applied (HR ticks) */
    uint16_t diagBypassAccepted;    /* ZCs accepted via polarity bypass (legacy) */
    uint16_t diagRisingZcCount;     /* ZCs accepted on rising polarity steps */
    uint16_t diagFallingZcCount;    /* ZCs accepted on falling polarity steps */
    uint16_t diagIntervalReject;    /* ZC intervals rejected by clamp */
    /* V2 counters */
    uint16_t actualForcedComm;      /* ONLY incremented on real timeout-forced comm */
    uint16_t zcTimeoutCount;        /* total ZC timeouts */
    uint16_t diagRisingTimeouts;    /* timeouts on rising ZC steps */
    uint16_t diagFallingTimeouts;   /* timeouts on falling ZC steps */
    uint16_t diagRisingRejects;     /* false ZCs rejected on rising steps */
    uint16_t diagFallingRejects;    /* false ZCs rejected on falling steps */
    /* Per-step 0..5 counters — distinguishes phase-pair vs polarity-class */
    uint16_t stepAccepted[6];       /* ZCs accepted per commutation step */
    uint16_t stepTimeouts[6];       /* timeouts per commutation step */
    /* Scheduler margin diagnostics */
    uint16_t diagTargetPast;        /* Commutation target already in past (fire ASAP) */
    int16_t  lastScheduleMarginHR;  /* Last margin: positive=on-time, negative=late */
} ZC_DIAG_T;

/* ── Speed PD Controller State ──────────────────────────────────────── */
typedef struct {
    int32_t  override;      /* accumulated duty override (PWM counts) */
    int32_t  lastError;     /* previous error for derivative */
    uint32_t targetErpm;    /* from pot mapping (diagnostic) */
} SPEED_PD_T;

/* Fault codes */
typedef enum {
    FAULT_NONE = 0,
    FAULT_OVERVOLTAGE,
    FAULT_UNDERVOLTAGE,
    FAULT_STALL,
    FAULT_DESYNC,
    FAULT_STARTUP_TIMEOUT,
    FAULT_ATA6847
} FAULT_CODE_T;

/* Main ESC runtime data */
typedef struct {
    ESC_STATE_T  state;
    uint8_t      currentStep;       /* 0-5 */
    uint8_t      direction;         /* 0=CW, 1=CCW */
    uint32_t     duty;              /* PWM duty cycle count */
    uint16_t     vbusRaw;           /* Vbus ADC reading (12-bit) */
    uint16_t     potRaw;            /* Pot ADC reading (12-bit) */
    uint16_t     throttle;          /* Mapped throttle 0-4095 */
    FAULT_CODE_T faultCode;

    /* Timing counters */
    uint32_t alignCounter;
    uint32_t rampStepPeriod;
    uint32_t rampCounter;
    uint32_t systemTick;            /* 1 ms counter */
    uint16_t armCounter;
    uint16_t timer1Tick;            /* 50 µs counter @ 20 kHz (wraps at 65535) */

    /* Run command / recovery */
    bool     runCommandActive;
    uint8_t  desyncRestartAttempts;
    uint32_t recoveryCounter;

    /* GSP throttle override — when gspThrottleActive, potRaw is replaced
     * with gspThrottleValue in MapThrottleToDuty. GUI motor testing. */
    bool     gspThrottleActive;
    uint16_t gspThrottleValue;      /* 0-65535 (same scale as potRaw) */

    /* ATA6847 fault monitoring */
    bool     ataFaultPending;       /* Set by Timer1 ISR when nIRQ asserted */
    bool     ataIlimActive;         /* Set when ILIM chopping detected */
    uint8_t  ataLastSIR1;           /* Last SIR1 value for diagnostics */

    /* Current sensing (ADC, 20kHz PWM-center triggered, signed 12-bit fractional)
     * Shunt: 3mΩ (RS1/RS2/RS3 on EV43F54A inverter sheet)
     * Phase A (IS1): OA2 Gt=16 → AN1 (ADCBUF1)
     * Phase B (IS2): OA3 Gt=16 → AN4 (ADCBUF4)
     * DC Bus (IBus): ATA6847 OPO3 Gt=8 → OA1 → AN0 (ADCBUF0) */
    int16_t  iaRaw;             /* Phase A current (AN1, OA2, signed) */
    int16_t  ibRaw;             /* Phase B current (AN4, OA3, signed) */
    int16_t  ibusRaw;           /* DC bus current  (AN0, OA1, signed) */

    /* Sub-structures */
    BEMF_STATE_T   bemf;
    TIMING_STATE_T timing;
#if FEATURE_IC_ZC
    IC_ZC_STATE_T  icZc;
#endif
    ZC_DIAG_T      zcDiag;
    ZC_CTRL_T      zcCtrl;
#if FEATURE_IC_ZC
    ZC_PRED_T      zcPred;
#endif
#if FEATURE_IC_DMA_SHADOW
    DMA_SHADOW_T   dmaShadow;
#endif
#if FEATURE_SECTOR_PI
    ZC_SYNC_T      zcSync;
#endif
    SPEED_PD_T     speedPd;

} GARUDA_DATA_T;

#endif /* GARUDA_TYPES_H */
