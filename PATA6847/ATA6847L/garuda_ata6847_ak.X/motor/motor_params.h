/**
 * @file motor_params.h
 * @brief Sector-PI runtime tunable parameters.
 *
 * Architecture (mirrors gsp_params for V3):
 *   - Always-allocate single global struct `escParams`
 *   - Reads from struct happen everywhere the code used to read #defines
 *   - GSP GET_PARAM/SET_PARAM/GET_PARAM_LIST handlers read/write the struct
 *   - Compile-time #defines remain as defaults (PHASE_ADVANCE_DEG etc)
 *
 * Phase A scope: only the 5 highest-leverage HOT params for live tuning.
 * Cold params (PWM freq, ATA6847 regs, motor profile constants) come in
 * later phases. Keeping the param ID space sparse so cold-param IDs can
 * slot in without renumbering.
 *
 * ID space reserved for V4: 0xF0–0xFF (V3 uses 0xC0–0xE5 already).
 */

#ifndef PARAMS_H
#define PARAMS_H

#include "../garuda_config.h"


#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Parameter IDs — Phase A (HOT params only) ───────────────── */
/* 0xF0–0xF7 reserved for HOT params (changeable while running).
 * 0xF8–0xFF reserved for COLD params (require motor stop).
 * Keeping 0xC0–0xE5 free for cold params shared with V3. */

/* Group 0: PI loop tuning */
#define PARAM_PHASE_ADVANCE_X10   0xF0  /* 0.1° units: 100 = 10.0° */
#define PARAM_PI_KP_SHIFT         0xF1  /* Kp = 1 / 2^N */
#define PARAM_PI_KI_SHIFT         0xF2  /* Ki = 1 / 2^N */

/* Group 1: Capture / blanking */
#define PARAM_BLANKING_PCT        0xF3  /* % of sector period */

/* Group 2: Limits */
#define PARAM_MIN_PERIOD_HR       0xF4  /* Speed ceiling — timerPeriod floor */

/* Group 3: Diagnostic / AK-port tuning */
#define PARAM_TRIGA_POS           0xF6  /* PG1TRIGA position for ADC trigger.
                                            * Bits 0-14: TRIGA value (0..LOOPTIME_TCY-1).
                                            * Bit 15:    CAHALF (0=cycle1, 1=cycle2).
                                            * Written to PG1TRIGA on Set. Read returns
                                            * current PG1TRIGA decoded into this format. */

#define PARAM_COUNT  6

/* ── Group IDs ─────────────────────────────────────────────────── */
/* Numbered 8/9/10 to avoid GUI metadata collision with V3 groups 0-7
 * (Motor Identity, Startup & Ramp, Closed-Loop, etc). The GUI's
 * CK_PARAM_GROUPS table maps these to "PI Loop", "Capture",
 * "Limits". */
#define GROUP_PI_LOOP    8   /* phase advance, Kp, Ki */
#define GROUP_CAPTURE    9   /* blanking, deglitch */
#define GROUP_LIMITS     10  /* min period, stall threshold */
#define GROUP_COUNT      3

/* ── Type codes (match V3 wire format) ─────────────────────────── */
#define TYPE_U8   0
#define TYPE_U16  1
#define TYPE_U32  2

/* ── Runtime parameter struct ──────────────────────────────────── */
typedef struct {
    /* Group 0: PI loop tuning */
    uint16_t phaseAdvanceDegX10; /* 0.1° units: 0–300 (= 0.0–30.0°) */
    uint8_t  piKpShift;          /* 0–8: Kp = 1/2^N */
    uint8_t  piKiShift;          /* 0–8: Ki = 1/2^N */

    /* Group 1: Capture / blanking */
    uint8_t  blankingPct;        /* 10–60: % of sectorPeriodHR */

    /* Group 2: Limits */
    uint16_t minPeriodHr;        /* 5–500 HR ticks */
} ESC_PARAMS_T;

/* ── Derived values — precomputed for ISR hot path ─────────────── */
typedef struct {
    uint16_t advancePlus30Fp8;   /* (advance + 30°) × 256/60, 8.8 fixed */
    /* blanking factor stays as raw %, scaled where consumed */
} ESC_DERIVED_T;

/* ── Descriptor table entry (matches V3 12-byte wire format) ───── */
typedef struct {
    uint16_t id;
    uint8_t  type;
    uint8_t  group;
    uint32_t min;
    uint32_t max;
} PARAM_DESC_T;

/* ── Globals ───────────────────────────────────────────────────── */
extern volatile ESC_PARAMS_T  escParams;
extern volatile ESC_DERIVED_T escDerived;

/* ── API ───────────────────────────────────────────────────────── */

/** Initialize params from compile-time defaults (PHASE_ADVANCE_DEG etc). */
void Params_InitDefaults(void);

/** Get a parameter value by ID. Returns 0 on unknown ID, sets *ok = false. */
uint32_t Params_Get(uint16_t paramId, bool *ok);

/** Set a parameter value by ID. Returns false if unknown ID or out of range. */
bool Params_Set(uint16_t paramId, uint32_t value);

/** Recompute derived values. Called automatically from Params_Set. */
void Params_RecomputeDerived(void);

/** Get descriptor table for GET_PARAM_LIST. */
const PARAM_DESC_T* Params_GetDescriptorTable(uint8_t *count);

#ifdef __cplusplus
}
#endif


#endif /* PARAMS_H */
