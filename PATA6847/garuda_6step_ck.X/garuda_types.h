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
    SPEED_PD_T     speedPd;

} GARUDA_DATA_T;

#endif /* GARUDA_TYPES_H */
