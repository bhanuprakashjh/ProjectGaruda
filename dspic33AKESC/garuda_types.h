/**
 * @file garuda_types.h
 *
 * @brief Core data structures and enumerations for Project Garuda ESC firmware.
 *
 * Component: TYPES
 */

#ifndef GARUDA_TYPES_H
#define GARUDA_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ESC state machine states */
typedef enum
{
    ESC_IDLE = 0,
    ESC_ARMED,
    ESC_ALIGN,
    ESC_OL_RAMP,
    ESC_CLOSED_LOOP,
    ESC_BRAKING,
    ESC_FAULT
} ESC_STATE_T;

/* Phase output states for 6-step commutation */
typedef enum
{
    PHASE_PWM_ACTIVE = 0,   /* PWM driven (H-side modulated, L-side complementary) */
    PHASE_LOW,              /* Forced LOW (both H and L off, L pulled low) */
    PHASE_FLOAT             /* Floating (both H and L off via override) */
} PHASE_STATE_T;

/* Commutation step descriptor */
typedef struct
{
    PHASE_STATE_T phaseA;
    PHASE_STATE_T phaseB;
    PHASE_STATE_T phaseC;
    uint8_t floatingPhase;  /* 0=A, 1=B, 2=C */
    int8_t zcPolarity;      /* +1 = rising ZC, -1 = falling ZC */
} COMMUTATION_STEP_T;

/* BEMF sensing state (stubbed for Phase 1, filled in Phase 2) */
typedef struct
{
    uint16_t bemfRaw;
    uint16_t vbusHalf;
    bool zeroCrossDetected;
} BEMF_STATE_T;

/* Commutation timing state (stubbed for Phase 1, filled in Phase 2) */
typedef struct
{
    uint32_t stepPeriod;        /* current commutation step period in timer counts */
    uint32_t lastZcTimestamp;
    uint32_t zcInterval;
    uint16_t filterCount;
} TIMING_STATE_T;

/* Fault codes */
typedef enum
{
    FAULT_NONE = 0,
    FAULT_OVERVOLTAGE,
    FAULT_UNDERVOLTAGE,
    FAULT_OVERCURRENT,
    FAULT_STALL,
    FAULT_DESYNC
} FAULT_CODE_T;

/* Main ESC runtime data */
typedef struct
{
    ESC_STATE_T state;
    uint16_t throttle;          /* 0-2000 (DShot range), or ADC pot value */
    uint8_t currentStep;        /* 0-5 commutation step index */
    uint8_t direction;          /* 0=CW, 1=CCW */
    uint32_t duty;              /* PWM duty cycle count */
    uint16_t vbusRaw;           /* raw ADC reading of DC bus voltage */
    uint16_t potRaw;            /* raw ADC reading of potentiometer */
    FAULT_CODE_T faultCode;

    /* Timing counters */
    uint32_t alignCounter;      /* counts down during alignment phase */
    uint32_t rampStepPeriod;    /* current forced commutation period during OL ramp */
    uint32_t rampCounter;       /* counts down within current step */
    uint32_t systemTick;        /* 1ms tick counter */
    uint16_t armCounter;        /* counts up during arming phase */

    /* Sub-structures */
    BEMF_STATE_T bemf;
    TIMING_STATE_T timing;
} GARUDA_DATA_T;

/* 64-byte packed configuration structure (for flash storage / GSP protocol) */
typedef struct __attribute__((packed))
{
    uint16_t pwmFrequency;      /* Hz */
    uint16_t deadtimeNs;        /* nanoseconds */
    uint8_t motorPolePairs;
    uint8_t direction;          /* 0=CW, 1=CCW */
    uint16_t alignTimeMs;
    uint8_t alignDutyPercent;
    uint8_t dshotRate;          /* 0=DShot150..4=DShot1200 */
    uint16_t initialErpm;
    uint16_t rampTargetErpm;
    uint16_t rampAccelErpmPerS;
    uint8_t reserved[48];       /* pad to 64 bytes (16 used + 48 reserved) */
} GARUDA_CONFIG_T;

_Static_assert(sizeof(GARUDA_CONFIG_T) == 64, "GARUDA_CONFIG_T must be 64 bytes");

#ifdef __cplusplus
}
#endif

#endif /* GARUDA_TYPES_H */
