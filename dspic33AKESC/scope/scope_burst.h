/**
 * @file scope_burst.h
 * @brief Triggered high-rate burst scope — 128 samples at 24kHz ISR rate.
 *
 * Ring buffer captures FOC internals at full ISR rate (41.6µs).
 * Trigger modes: manual, fault, state-change, threshold.
 * Frozen buffer is read out via GSP chunked protocol.
 *
 * Component: SCOPE
 */

#ifndef SCOPE_BURST_H
#define SCOPE_BURST_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Configuration ─────────────────────────────────────────────── */

#define SCOPE_BUF_SIZE      128     /* Power of 2 for fast masking */
#define SCOPE_BUF_MASK      (SCOPE_BUF_SIZE - 1)
#define SCOPE_SAMPLE_SIZE   26      /* Bytes per sample */
#define SCOPE_MAX_CHUNK     9       /* Max samples per GSP read (9×26+2=236 < 249 GSP max) */

/* ── Sample format ─────────────────────────────────────────────── */

/**
 * 26-byte sample packed for RAM efficiency.
 * All values int16_t with known scaling factors:
 *   ia, ib, id, iq: ×1000 (milliamps, ±32.767A range)
 *   vd, vq:         ×100  (centivolts, ±327.67V range)
 *   theta:          ×10000 (0.0001 rad resolution, ±3.2767)
 *   obs_x1, obs_x2: ×100000 (flux in 10µV·s units)
 *   omega:          ×10   (0.1 rad/s, ±3276.7 rad/s)
 *   mod_index:      ×10000 (0.0001 resolution, 0-1 mapped)
 *   flags:          bit0=CL, bit1=fault, bit2-4=mode
 *   tick_lsb:       lower 16 bits of ISR tick counter
 */
typedef struct __attribute__((packed)) {
    int16_t  ia;            /* Phase A current ×1000 */
    int16_t  ib;            /* Phase B current ×1000 */
    int16_t  id;            /* D-axis current ×1000 */
    int16_t  iq;            /* Q-axis current ×1000 */
    int16_t  vd;            /* D-axis voltage ×100 */
    int16_t  vq;            /* Q-axis voltage ×100 */
    int16_t  theta;         /* Commutation angle ×10000 */
    int16_t  obs_x1;        /* Observer flux alpha ×100000 */
    int16_t  obs_x2;        /* Observer flux beta ×100000 */
    int16_t  omega;         /* Electrical speed ×10 */
    int16_t  mod_index;     /* Modulation index ×10000 */
    uint8_t  flags;         /* bit0=CL, bit1=fault, bit2-4=mode(3bit) */
    uint8_t  state;         /* ESC state */
    uint16_t tick_lsb;      /* ISR tick counter LSB */
} SCOPE_SAMPLE_T;

_Static_assert(sizeof(SCOPE_SAMPLE_T) == SCOPE_SAMPLE_SIZE,
               "SCOPE_SAMPLE_T size mismatch");

/* ── Trigger modes ─────────────────────────────────────────────── */

typedef enum {
    SCOPE_TRIG_MANUAL       = 0,    /* Armed, waits for GSP trigger cmd */
    SCOPE_TRIG_FAULT        = 1,    /* Triggers on any fault */
    SCOPE_TRIG_STATE_CHANGE = 2,    /* Triggers on ESC state transition */
    SCOPE_TRIG_THRESHOLD    = 3     /* Triggers when channel crosses threshold */
} SCOPE_TrigMode_t;

/* ── Scope states ──────────────────────────────────────────────── */

typedef enum {
    SCOPE_IDLE   = 0,   /* Not armed, not capturing */
    SCOPE_ARMED  = 1,   /* Armed, filling pre-trigger */
    SCOPE_FILLING = 2,  /* Triggered, filling post-trigger */
    SCOPE_READY  = 3    /* Buffer frozen, ready for readout */
} SCOPE_State_t;

/* ── Threshold channel IDs ─────────────────────────────────────── */

typedef enum {
    SCOPE_CH_IA     = 0,
    SCOPE_CH_IB     = 1,
    SCOPE_CH_ID     = 2,
    SCOPE_CH_IQ     = 3,
    SCOPE_CH_VD     = 4,
    SCOPE_CH_VQ     = 5,
    SCOPE_CH_THETA  = 6,
    SCOPE_CH_OMEGA  = 9,
    SCOPE_CH_MOD    = 10
} SCOPE_Channel_t;

/* ── Threshold edge ────────────────────────────────────────────── */

typedef enum {
    SCOPE_EDGE_RISING  = 0,
    SCOPE_EDGE_FALLING = 1
} SCOPE_Edge_t;

/* ── Arm configuration (from GSP command) ──────────────────────── */

typedef struct {
    SCOPE_TrigMode_t trigMode;
    uint8_t          preTrigPct;     /* 0-100: % of buffer before trigger */
    SCOPE_Channel_t  trigChannel;    /* For THRESHOLD mode */
    SCOPE_Edge_t     trigEdge;       /* For THRESHOLD mode */
    int16_t          threshold;      /* Scaled value (same as sample field) */
} SCOPE_ArmConfig_t;

/* ── Status (for GSP status response) ──────────────────────────── */

typedef struct {
    SCOPE_State_t    state;
    SCOPE_TrigMode_t trigMode;
    uint8_t          preTrigPct;
    uint8_t          trigIdx;        /* Index in buffer where trigger occurred */
    uint8_t          sampleCount;    /* Total valid samples */
    uint8_t          sampleSize;     /* sizeof(SCOPE_SAMPLE_T) */
} SCOPE_Status_t;

/* ── API ───────────────────────────────────────────────────────── */

/**
 * Initialize scope module. Call once at startup.
 */
void Scope_Init(void);

/**
 * Arm the scope with given configuration.
 * Transitions: IDLE/READY → ARMED.
 */
void Scope_Arm(const SCOPE_ArmConfig_t *cfg);

/**
 * Disarm / stop capture.
 * Transitions: any → IDLE.
 */
void Scope_Disarm(void);

/**
 * Force trigger (for MANUAL mode). No-op if not ARMED.
 */
void Scope_ForceTrigger(void);

/**
 * Write one sample to the ring buffer. Called from ISR at 24kHz.
 * Handles trigger detection and state transitions internally.
 * ~330ns overhead when armed (no-op when IDLE/READY).
 */
void Scope_WriteSample(const SCOPE_SAMPLE_T *sample);

/**
 * Get current scope status.
 */
SCOPE_Status_t Scope_GetStatus(void);

/**
 * Read samples from frozen buffer.
 * @param offset  Sample index (0 = oldest)
 * @param count   Number of samples to read (max SCOPE_MAX_CHUNK)
 * @param out     Output buffer (must hold count × SCOPE_SAMPLE_SIZE bytes)
 * @return        Actual number of samples copied
 */
uint8_t Scope_ReadSamples(uint8_t offset, uint8_t count,
                          SCOPE_SAMPLE_T *out);

#ifdef __cplusplus
}
#endif

#endif /* SCOPE_BURST_H */
