/**
 * @file gsp_commands.h
 * @brief GSP v2 command IDs and CK board wire-format structures.
 *
 * Protocol-compatible with dsPIC33AK board. Same packet format, CRC,
 * and command IDs. Different snapshot struct (CK 6-step vs AK FOC).
 * Board detection via boardId field in GSP_INFO_T.
 */

#ifndef GSP_COMMANDS_H
#define GSP_COMMANDS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Firmware version */
#define GSP_FW_MAJOR    1
#define GSP_FW_MINOR    0
#define GSP_FW_PATCH    0

/* GSP protocol version (same as AK board) */
#define GSP_PROTOCOL_VERSION  2

/* Board IDs — GUI uses this to select CK vs AK UI */
#define GSP_BOARD_MCLV48V300W  0x0001   /* dsPIC33AK + MCLV-48V-300W */
#define GSP_BOARD_EV43F54A     0x0002   /* dsPIC33CK + ATA6847 (this board) */

/* Command IDs — same as AK board for protocol compatibility */
typedef enum {
    GSP_CMD_PING            = 0x00,
    GSP_CMD_GET_INFO        = 0x01,
    GSP_CMD_GET_SNAPSHOT    = 0x02,
    GSP_CMD_START_MOTOR     = 0x03,
    GSP_CMD_STOP_MOTOR      = 0x04,
    GSP_CMD_CLEAR_FAULT     = 0x05,
    GSP_CMD_SET_THROTTLE    = 0x06,
    GSP_CMD_HEARTBEAT       = 0x08,
    GSP_CMD_GET_PARAM       = 0x10,
    GSP_CMD_SET_PARAM       = 0x11,
    GSP_CMD_SAVE_CONFIG     = 0x12,
    GSP_CMD_LOAD_DEFAULTS   = 0x13,
    GSP_CMD_TELEM_START     = 0x14,
    GSP_CMD_TELEM_STOP      = 0x15,
    GSP_CMD_GET_PARAM_LIST  = 0x16,
    GSP_CMD_LOAD_PROFILE    = 0x17,
    /* DMA burst capture research tool */
    GSP_CMD_BURST_ARM       = 0x20,   /* arm the burst; no payload */
    GSP_CMD_BURST_STATUS    = 0x21,   /* response: {state, count} 2 bytes */
    GSP_CMD_BURST_GET_STEP  = 0x22,   /* payload: {stepIdx}; response: DMA_BURST_STEP_T */
    GSP_CMD_TELEM_FRAME     = 0x80,
    GSP_CMD_ERROR           = 0xFF
} GSP_CMD_ID_T;

/* Error codes */
typedef enum {
    GSP_ERR_UNKNOWN_CMD      = 0x01,
    GSP_ERR_BAD_LENGTH       = 0x02,
    GSP_ERR_BUSY             = 0x03,
    GSP_ERR_WRONG_STATE      = 0x04,
    GSP_ERR_OUT_OF_RANGE     = 0x05,
    GSP_ERR_UNKNOWN_PARAM    = 0x06,
    GSP_ERR_CROSS_VALIDATION = 0x07,
} GSP_ERR_CODE_T;

/* GSP_INFO_T — 20 bytes, same as AK board (protocol-compatible) */
typedef struct __attribute__((packed)) {
    uint8_t  protocolVersion;
    uint8_t  fwMajor;
    uint8_t  fwMinor;
    uint8_t  fwPatch;
    uint16_t boardId;           /* 0x0002 for CK board */
    uint8_t  motorProfile;      /* 0=Hurst, 1=A2212 */
    uint8_t  motorPolePairs;
    uint32_t featureFlags;      /* CK-specific feature bits */
    uint32_t pwmFrequency;
    uint32_t maxErpm;
} GSP_INFO_T;

/* CK Feature flag bits */
#define GSP_FEATURE_IC_ZC       (1UL << 0)   /* SCCP1 fast poll ZC */
#define GSP_FEATURE_VBUS_FAULT  (1UL << 1)   /* Vbus OV/UV monitoring */
#define GSP_FEATURE_DESYNC_REC  (1UL << 2)   /* Desync auto-recovery */
#define GSP_FEATURE_DUTY_SLEW   (1UL << 3)   /* Asymmetric duty slew */
#define GSP_FEATURE_TIM_ADVANCE (1UL << 4)   /* Linear timing advance */
#define GSP_FEATURE_ATA6847     (1UL << 25)  /* ATA6847 gate driver */
#define GSP_FEATURE_ILIM_HW     (1UL << 26)  /* Hardware current limit */
#define GSP_FEATURE_CURRENT_SNS (1UL << 27)  /* Phase current sensing */
#define GSP_FEATURE_GSP         (1UL << 16)  /* GSP protocol active */

/**
 * GSP_CK_SNAPSHOT_T — 64 bytes, CK board telemetry snapshot.
 *
 * V1: 48 bytes (core + electrical + speed + ZC diag + system)
 * V2: 52 bytes (+zcLatencyPct, zcBlankPct, zcBypassCount)
 * V3: 64 bytes (+zcMode, actualForcedComm, per-polarity counters)
 */
typedef struct __attribute__((packed)) {
    /* Core state (8B) */
    uint8_t  state;             /* ESC_STATE_T */
    uint8_t  faultCode;         /* FAULT_CODE_T */
    uint8_t  currentStep;       /* 0-5 */
    uint8_t  ataStatus;         /* SIR1 summary + ILIM flag */
    uint16_t potRaw;            /* Pot ADC (speed reference) */
    uint8_t  dutyPct;           /* 0-100% */
    uint8_t  zcSynced;          /* 1=SYN, 0=--- */

    /* Electrical (12B) */
    uint16_t vbusRaw;           /* Bus voltage ADC */
    int16_t  iaRaw;             /* Phase A current (signed, fractional) */
    int16_t  ibRaw;             /* Phase B current (signed, fractional) */
    int16_t  ibusRaw;           /* Reconstructed bus current (abs value) */
    uint16_t duty;              /* Raw duty count */

    /* Speed/timing (14B) */
    uint16_t stepPeriod;        /* Timer1 ticks */
    uint16_t stepPeriodHR;      /* SCCP4 HR ticks (640ns) */
    uint32_t eRpm;              /* Computed eRPM (full 32-bit, A2212 reaches 100k+) */
    uint16_t goodZcCount;
    uint16_t zcInterval;
    uint16_t prevZcInterval;

    /* ZC diagnostics (8B) */
    uint16_t icAccepted;        /* Fast poll accepted ZCs */
    uint16_t icFalse;           /* Rejected ZCs */
    uint8_t  filterLevel;       /* Current deglitch FL */
    uint8_t  missedSteps;       /* consecutiveMissedSteps */
    uint8_t  forcedSteps;       /* LEGACY: stepsSinceLastZc (NOT real forced comm count).
                                 * Use actualForcedComm below for real timeout count. */
    uint8_t  ilimActive;        /* ATA6847 ILIM chopping */

    /* System (8B) */
    uint32_t systemTick;        /* 1ms counter */
    uint32_t uptimeSec;

    /* ZC diagnostics v1 (4B) */
    uint8_t  zcLatencyPct;      /* 0-255: ZC position in window, 0xFF=timeout */
    uint8_t  zcBlankPct;        /* Layer 1 blanking as % of step period */
    uint16_t zcBypassCount;     /* ZCs accepted via polarity bypass (legacy) */

    /* ZC V2 diagnostics (12B) */
    uint8_t  zcMode;            /* ZC_MODE_ACQUIRE/TRACK/RECOVER */
    uint8_t  actualForcedComm;  /* true timeout-forced commutations (saturated 255) */
    uint16_t zcTimeoutCount;    /* total ZC timeouts */
    uint16_t risingZcCount;     /* rising polarity ZC accepts */
    uint16_t fallingZcCount;    /* falling polarity ZC accepts */
    uint16_t risingTimeouts;    /* rising polarity timeouts */
    uint16_t fallingTimeouts;   /* falling polarity timeouts */

    /* Per-step 0..5 counters (24B) — phase-pair vs polarity-class test */
    uint16_t stepAccepted[6];   /* ZCs accepted per commutation step */
    uint16_t stepTimeouts[6];   /* timeouts per commutation step */
    /* Raw comparator edge trace (24B) */
    uint16_t stepFlips[6];      /* comparator transitions per step (glitch count) */
    uint16_t stepPolls[6];      /* total polls per step */

    /* Raw corroboration & IC age diagnostics (6B) */
    uint16_t rawVetoCount;      /* CLC matched but raw didn't corroborate */
    uint16_t icAgeRejectCount;  /* IC timestamp discarded as stale */
    uint16_t trackFallbackCount; /* ZCs accepted via PWM-aged FL=2 fallback */

    /* Phase 2: raw stability & timestamp source diagnostics (14B) */
    uint16_t rawStableBlock;    /* Fast accept blocked: raw not stable enough */
    uint16_t tsFromIc;          /* ZCs using IC timestamp */
    uint16_t tsFromRaw;         /* ZCs using raw first-match timestamp */
    uint16_t tsFromClc;         /* ZCs using CLC first-match timestamp */
    uint16_t tsFromPoll;        /* ZCs using poll timestamp */
    uint16_t icLeadReject;      /* IC timestamp downgraded (IC led raw too much) */

    /* Scheduler margin diagnostics (4B) */
    uint16_t targetPastCount;   /* Commutation target already in past */
    int16_t  schedMarginHR;     /* Last scheduler margin (HR ticks, neg=late) */

    /* PLL predictor telemetry (18B) */
    int16_t  predPhaseErrHR;    /* Phase error model A: nominal (comm+half+adv) */
    int16_t  predPhaseErrRxHR;  /* Phase error model B: reactive (comm+lastDelay) */
    uint16_t predStepHR;        /* Predicted step period */
    uint16_t predZcInWindow;    /* ZC fell within scan window */
    uint16_t predZcOutWindow;   /* ZC fell outside scan window */
    uint8_t  predLocked;        /* Predictor locked (tracking) */
    uint8_t  predMissCount;     /* Steps without ZC correction */
    int16_t  predMinMarginHR;   /* Min predictor margin observed */
    uint16_t predRealZcDelayHR; /* Last measured comm-to-ZC delay */
    uint16_t predZcOffsetHR;    /* Adaptive phase offset (IIR-tracked) */

    /* Step 2: gate readiness (8B) */
    uint16_t winCandInGated;    /* In-window while gateActive */
    uint16_t winCandOutGated;   /* Out-of-window while gateActive */
    uint16_t winOutEarly;       /* ZC before scanOpen */
    uint16_t winOutLate;        /* ZC after scanClose */
    uint8_t  gateActive;        /* Gate armed */
    uint8_t  _pad1;             /* alignment */
    uint16_t windowReject;      /* ZCs vetoed by live gate */
    uint16_t windowRecovered;   /* Vetoed steps that got later valid ZC */

    /* Step 3: predictive scheduling (12B) */
    uint16_t predCommOwned;     /* Commutations scheduled by predictor */
    uint8_t  predictiveMode;    /* Predictor owns scheduling */
    uint8_t  handoffPending;    /* Handoff in progress */
    uint16_t predExitMiss;      /* Exits: missCount */
    uint16_t predExitTimeout;   /* Exits: timeout */
    uint16_t predEnter;         /* Successful predictive entries */
    uint16_t predEntryLate;     /* Handoff aborted: target past */
    int16_t  predVsReactiveDelta; /* Shadow: pred target - reactive target */
    uint8_t  deltaOkCount;      /* Consecutive small-delta steps */
    uint8_t  entryScore;        /* Predictive entry readiness (0-255) */
    uint16_t predIsrFired;      /* ISR entries while predictiveMode active */
    uint16_t predIsrEntries;    /* Total ISR entries with deadlineActive */

    /* DPLL state (FEATURE_6STEP_DPLL) */
    int16_t  dpllPhaseBiasHR;  /* B_hat: combined measurement/phase bias */
    int16_t  dpllErrHR;       /* Last DPLL phase error (tMeas - predZc) */
    uint16_t dmaMeasUsed;     /* Steps where DMA passed plausibility gate */
    uint16_t dmaMeasReject;   /* Steps where DMA failed gate → poll used */
    uint16_t predCloseAgree;  /* Predicted-close matched poll-close */
    uint16_t predCloseDisagree; /* Predicted-close differed from poll-close */
    uint8_t  dpllFallbackReason; /* Why predictive mode exited (0=none) */
    uint8_t  measSource;       /* What fed DPLL this step: 0=none 1=poll 2=DMA */

    /* Sector PI synchronizer telemetry (14B) */
    int16_t  syncErrHR;       /* Phase error: capValue - setValue */
    uint16_t syncT_hatHR;     /* Current sector period estimate */
    int16_t  syncVsReactive;  /* nextComm(sync) - targetHR(reactive) */
    uint8_t  syncMode;        /* 0=OFF, 1=SHADOW, 2=OWNED */
    uint8_t  syncGoodStreak;  /* Consecutive small-error sectors */
    uint8_t  syncMissStreak;  /* Consecutive missed DMA clusters */
    uint8_t  syncClusterCount;/* Edges in last selected cluster */
    uint16_t syncAccepts;     /* Total sectors with valid DMA measurement */
    uint16_t syncMisses;      /* Total sectors with no DMA measurement */

    /* IC capture diagnostics (2B) — added to debug high-speed wall.
     * icBounce climbs when the SCCP2 IC capture is rejected by the
     * 50% interval gate in _CCP2Interrupt — i.e. the IC fired before
     * (estimated step / 2) elapsed since the last ZC. During
     * acceleration the IIR-averaged refIntervalHR lags reality, so
     * the actual ZC arrives at the *real* 50% mark which is BEFORE
     * the gate's threshold, and the IC capture is silently dropped. */
    uint16_t icBounce;

    /* DMA shadow telemetry (26B) — dual-CCP + DMA ring experiment.
     * Only populated when FEATURE_IC_DMA_SHADOW=1 at compile time,
     * otherwise all zero. Fields describe how well a shadow-only
     * hardware-precise capture ring (CCP2+DMA0 for rising ZCs,
     * CCP5+DMA1 for falling) tracks the live poll-accepted ZC.
     *
     * 32-bit counters grouped at the front — at 60k eRPM the original
     * uint16 step/match counters wrapped in ~13 seconds and caused
     * display garbage in the telemetry. The per-step average
     * (dmaEdgesAvgX16) stays 16-bit — it never accumulates. */
    uint32_t dmaStepCount;            /* probe invocations (total steps observed) */
    uint32_t dmaMatchCount;           /* steps where DMA found capture in window */
    uint32_t dmaRingOverflow;         /* ring near-full events */
    uint16_t dmaEdgesAvgX16;          /* avg captures/step × 16 (fixed-point) */
    int16_t  dmaLastEarliestVsPoll;   /* DMA earliest vs poll-accepted (HR ticks) */
    int16_t  dmaLastEarliestVsExp;    /* DMA earliest vs predicted ZC */
    int16_t  dmaLastClosestVsExp;     /* DMA best-fit vs predicted ZC */
    int16_t  dmaLastPollVsExp;        /* baseline: poll-accepted vs predicted ZC */
    uint8_t  dmaLastEdgeCount;        /* captures found in most recent step */
    uint8_t  dmaLastFound;            /* 1 = most recent probe matched */

    /* Hybrid DMA-direct ZC substitution telemetry. Populated when
     * FEATURE_DMA_ZC_DIRECT=1. Shows how often the poll timestamp was
     * replaced with a hardware-precise DMA edge, plus the observed
     * correction range (how much earlier the real edge was vs poll). */
    uint32_t dmaSubCount;             /* total DMA→direct substitutions */
    uint32_t dmaSubSkipGated;         /* below speed threshold, skipped */
    uint32_t dmaSubSkipRange;         /* DMA edge out of sanity window */
    int16_t  dmaLastCorrectionHR;     /* most recent (refined - poll) HR delta */
    int16_t  dmaMinCorrectionHR;      /* min (most negative) correction seen */
    int16_t  dmaMaxCorrectionHR;      /* max (most positive) correction seen */
} GSP_CK_SNAPSHOT_T;

/* XC16 doesn't support _Static_assert. Verify size at compile time:
 * sizeof(GSP_CK_SNAPSHOT_T) should be 48 bytes. */

/* Command handler prototype */
typedef void (*GSP_CMD_HANDLER_T)(const uint8_t *payload, uint8_t payloadLen);

void GSP_DispatchCommand(uint8_t cmdId, const uint8_t *payload, uint8_t payloadLen);
void GSP_TelemTick(void);

#ifdef __cplusplus
}
#endif

#endif /* GSP_COMMANDS_H */
