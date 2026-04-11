/**
 * @file hal_dma_burst.h
 * @brief Research tool — capture raw DMA edge sequences for N consecutive steps.
 *
 * Pure observer. Zero effect on motor control.
 *
 * Workflow:
 *   1. Motor is running normally in closed loop.
 *   2. User arms via HAL_DmaBurst_Arm().
 *   3. On the NEXT commutation, OnCommutation() starts recording.
 *   4. At each ZC acceptance (or timeout), OnZc() snapshots:
 *        - commHR, pollHR, predictedHR
 *        - the raw DMA edges captured between commutation and ZC
 *      and stores them into steps[nextIdx], then increments nextIdx.
 *   5. When nextIdx == DMA_BURST_STEPS, state → FULL. Further
 *      OnCommutation/OnZc calls are ignored until re-armed.
 *   6. Host reads the buffer via GSP (one step per packet) and
 *      renders it as a timeline.
 *
 * Memory cost: DMA_BURST_STEPS × sizeof(DMA_BURST_STEP_T)
 *            = 12 × 50  = 600 bytes.
 */

#ifndef HAL_DMA_BURST_H
#define HAL_DMA_BURST_H

#include "../garuda_config.h"

#if FEATURE_DMA_BURST_CAPTURE

#include <stdint.h>
#include <stdbool.h>

#define DMA_BURST_STEPS             12u
#define DMA_BURST_EDGES_PER_STEP    32u   /* covers whole step at low speed too */

/* One captured step — all timestamps in HR (SCCP4) domain, 640 ns/tick. */
typedef struct {
    uint16_t commHR;          /* HR tick at commutation edge */
    uint16_t pollHR;          /* HR tick when poll accepted (0 if timeout) */
    uint16_t predictedHR;     /* predictor's expected ZC time */
    uint16_t edges[DMA_BURST_EDGES_PER_STEP]; /* raw DMA edges since commHR */
    uint8_t  edgeCount;       /* actual edges stored (0..20) */
    uint8_t  stepIndex;       /* commutation step 0..5 */
    uint8_t  risingZc;        /* 1 = CCP2 ring (rising ZC), 0 = CCP5 (falling) */
    uint8_t  wasTimeout;      /* 1 if this step ended in a timeout (no ZC) */
} DMA_BURST_STEP_T;            /* sizeof = 50 bytes */

typedef enum {
    DMA_BURST_IDLE      = 0,   /* not armed */
    DMA_BURST_CAPTURING = 1,   /* armed, filling steps */
    DMA_BURST_FULL      = 2    /* capture complete, ready for readout */
} DMA_BURST_STATE_T;

/**
 * @brief Arm the capture. Next commutation begins filling the buffer.
 * Call this from any context. Does not allocate or block.
 */
void HAL_DmaBurst_Arm(void);

/**
 * @brief Query state.
 * @return 0 = idle, 1 = capturing, 2 = full
 */
uint8_t HAL_DmaBurst_GetState(void);

/**
 * @brief How many steps have been captured so far (0..DMA_BURST_STEPS).
 */
uint8_t HAL_DmaBurst_GetStepCount(void);

/**
 * @brief Access a captured step by index. Returns NULL if idx is out of range.
 */
const DMA_BURST_STEP_T *HAL_DmaBurst_GetStep(uint8_t idx);

/**
 * @brief Called from BEMF_ZC_OnCommutation for each new step.
 * @param commHR     HR tick at commutation edge
 * @param stepIdx    commutation step (0..5)
 * @param risingZc   polarity of the NEXT ZC for this step
 */
void HAL_DmaBurst_OnCommutation(uint16_t commHR, uint8_t stepIdx, bool risingZc);

/**
 * @brief Called from RecordZcTiming immediately after the probe.
 * Snapshots the edge ring into the current step slot and advances.
 *
 * @param pollHR      hrTick the poll path just accepted
 * @param predictedHR predictor's expected ZC time for this step
 * @param wasTimeout  true if this step ended in a timeout
 */
void HAL_DmaBurst_OnZc(uint16_t pollHR, uint16_t predictedHR, bool wasTimeout);

#endif /* FEATURE_DMA_BURST_CAPTURE */
#endif /* HAL_DMA_BURST_H */
