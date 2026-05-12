/**
 * @file v4_params.h
 * @brief V4 sector-PI runtime tunable parameters.
 *
 * Architecture (mirrors gsp_ck_params for V3):
 *   - Always-allocate single global struct `v4Params`
 *   - Reads from struct happen everywhere V4 code used to read #defines
 *   - GSP GET_PARAM/SET_PARAM/GET_PARAM_LIST handlers read/write the struct
 *   - Compile-time #defines remain as defaults (V4_PHASE_ADVANCE_DEG etc)
 *
 * Phase A scope: only the 5 highest-leverage HOT params for live tuning.
 * Cold params (PWM freq, ATA6847 regs, motor profile constants) come in
 * later phases. Keeping the param ID space sparse so cold-param IDs can
 * slot in without renumbering.
 *
 * ID space reserved for V4: 0xF0–0xFF (V3 uses 0xC0–0xE5 already).
 */

#ifndef V4_PARAMS_H
#define V4_PARAMS_H

#include "../garuda_config.h"

#if FEATURE_V4_SECTOR_PI

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── V4 Parameter IDs — Phase A (HOT params only) ───────────────── */
/* 0xF0–0xF7 reserved for HOT params (changeable while running).
 * 0xF8–0xFF reserved for COLD params (require motor stop).
 * Keeping 0xC0–0xE5 free for cold params shared with V3. */

/* Group 0: PI loop tuning */
#define V4_PARAM_PHASE_ADVANCE_X10   0xF0  /* 0.1° units: 100 = 10.0° */
#define V4_PARAM_PI_KP_SHIFT         0xF1  /* Kp = 1 / 2^N */
#define V4_PARAM_PI_KI_SHIFT         0xF2  /* Ki = 1 / 2^N */

/* Group 1: Capture / blanking */
#define V4_PARAM_BLANKING_PCT        0xF3  /* % of sector period */
#define V4_PARAM_PI_FEED_POLARITY    0xF5  /* 0=both, 1=rising only, 2=falling only */

/* Group 2: Limits */
#define V4_PARAM_MIN_PERIOD_HR       0xF4  /* Speed ceiling — timerPeriod floor */

#define V4_PARAM_COUNT  6

/* ── Group IDs ─────────────────────────────────────────────────── */
/* Numbered 8/9/10 to avoid GUI metadata collision with V3 groups 0-7
 * (Motor Identity, Startup & Ramp, Closed-Loop, etc). The GUI's
 * CK_PARAM_GROUPS table maps these to "V4 PI Loop", "V4 Capture",
 * "V4 Limits". */
#define V4_GROUP_PI_LOOP    8   /* phase advance, Kp, Ki */
#define V4_GROUP_CAPTURE    9   /* blanking, deglitch */
#define V4_GROUP_LIMITS     10  /* min period, stall threshold */
#define V4_GROUP_COUNT      3

/* ── Type codes (match V3 wire format) ─────────────────────────── */
#define V4_TYPE_U8   0
#define V4_TYPE_U16  1
#define V4_TYPE_U32  2

/* ── Runtime parameter struct ──────────────────────────────────── */
typedef struct {
    /* Group 0: PI loop tuning */
    uint16_t phaseAdvanceDegX10; /* 0.1° units: 0–300 (= 0.0–30.0°) */
    uint8_t  piKpShift;          /* 0–8: Kp = 1/2^N */
    uint8_t  piKiShift;          /* 0–8: Ki = 1/2^N */

    /* Group 1: Capture / blanking */
    uint8_t  blankingPct;        /* 10–60: % of sectorPeriodHR */
    uint8_t  piFeedPolarity;     /* 0=both, 1=rising-only, 2=falling-only */

    /* Group 2: Limits */
    uint16_t minPeriodHr;        /* 5–500 HR ticks */
} V4_PARAMS_T;

/* ── Derived values — precomputed for ISR hot path ─────────────── */
typedef struct {
    uint16_t advancePlus30Fp8;   /* (advance + 30°) × 256/60, 8.8 fixed */
    /* blanking factor stays as raw % — HAL_Capture_SetBlanking does the scale */
} V4_DERIVED_T;

/* ── Descriptor table entry (matches V3 12-byte wire format) ───── */
typedef struct {
    uint16_t id;
    uint8_t  type;
    uint8_t  group;
    uint32_t min;
    uint32_t max;
} V4_PARAM_DESC_T;

/* ── Globals ───────────────────────────────────────────────────── */
extern volatile V4_PARAMS_T  v4Params;
extern volatile V4_DERIVED_T v4Derived;

/* ── API ───────────────────────────────────────────────────────── */

/** Initialize params from compile-time defaults (V4_PHASE_ADVANCE_DEG etc). */
void V4Params_InitDefaults(void);

/** Get a parameter value by ID. Returns 0 on unknown ID, sets *ok = false. */
uint32_t V4Params_Get(uint16_t paramId, bool *ok);

/** Set a parameter value by ID. Returns false if unknown ID or out of range. */
bool V4Params_Set(uint16_t paramId, uint32_t value);

/** Recompute derived values. Called automatically from V4Params_Set. */
void V4Params_RecomputeDerived(void);

/** Get descriptor table for GET_PARAM_LIST. */
const V4_PARAM_DESC_T* V4Params_GetDescriptorTable(uint8_t *count);

#ifdef __cplusplus
}
#endif

#endif /* FEATURE_V4_SECTOR_PI */

#endif /* V4_PARAMS_H */
