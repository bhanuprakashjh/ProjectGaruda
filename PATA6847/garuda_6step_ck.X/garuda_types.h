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

/* Commutation timing — all in Timer1 ticks (20 kHz = 50 µs) */
typedef struct {
    uint16_t stepPeriod;            /* Current step period in Timer1 ticks */
    uint16_t lastCommTick;          /* Timer1 tick at last commutation */
    uint16_t lastZcTick;            /* Timer1 tick at last ZC */
    uint16_t prevZcTick;
    uint16_t zcInterval;
    uint16_t prevZcInterval;        /* Previous zcInterval for 2-step averaging */
    uint16_t commDeadline;          /* Timer1 tick for next commutation */
    uint16_t forcedCountdown;
    uint16_t goodZcCount;
    uint8_t  consecutiveMissedSteps;
    uint8_t  stepsSinceLastZc;
    bool     zcSynced;
    bool     deadlineActive;
    bool     hasPrevZc;
} TIMING_STATE_T;

/* ZC timeout result */
typedef enum {
    ZC_TIMEOUT_NONE = 0,
    ZC_TIMEOUT_FORCE_STEP,
    ZC_TIMEOUT_DESYNC
} ZC_TIMEOUT_RESULT_T;

/* IC ZC detection state (FEATURE_IC_ZC) */
#if FEATURE_IC_ZC
typedef enum {
    IC_ZC_BLANKING = 0, /* Waiting for blanking period to expire */
    IC_ZC_ARMED,        /* IC enabled, waiting for first capture */
    IC_ZC_PENDING,      /* First capture received, guard window open */
    IC_ZC_CONFIRMED,    /* Guard passed, ZC accepted */
    IC_ZC_DONE          /* IC disabled for this step */
} IC_ZC_PHASE_T;

typedef struct {
    IC_ZC_PHASE_T phase;        /* State machine phase */
    uint16_t blankingEndTick;   /* Timer1 tick when blanking expires */
    uint16_t captureTick;       /* Raw SCCP capture value (640 ns ticks) */
    uint16_t captureTimer1;     /* Timer1 tick at capture ISR entry */
    uint8_t  guardCountdown;    /* Timer1 ticks remaining in guard window */
    uint8_t  activeChannel;     /* Which SCCP is active: 0=A, 1=B, 2=C */
    /* Diagnostics — counters for test/debug (no ISR overhead beyond increment) */
    uint16_t diagAccepted;      /* ZCs accepted (guard + comparator passed) */
    uint16_t diagChatter;       /* Chatter: stale FIFO entries in PENDING */
    uint16_t diagCaptures;      /* Total IC captures processed */
    uint16_t diagFalseZc;       /* Comparator validation failures at guard expiry */
} IC_ZC_STATE_T;
#endif

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

    /* Sub-structures */
    BEMF_STATE_T   bemf;
    TIMING_STATE_T timing;
#if FEATURE_IC_ZC
    IC_ZC_STATE_T  icZc;
#endif
} GARUDA_DATA_T;

#endif /* GARUDA_TYPES_H */
