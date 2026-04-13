/**
 * @file hal_ic_dma.h
 * @brief Dual-CCP + DMA shadow input capture ring (experiment).
 *
 * Gated by FEATURE_IC_DMA_SHADOW. When OFF, this file produces no code
 * and the production path is untouched. When ON, the live CCP2 IC path
 * in hal_ic.c is stubbed (its <1% timestamp contribution is negligible
 * and already dominated by raw poll), and this module takes exclusive
 * ownership of CCP2 and CCP5 to produce hardware-precise ZC timestamps
 * in circular RAM rings via DMA0 and DMA1 — with zero CPU involvement
 * per capture.
 *
 * Architecture:
 *   CCP2 (MOD=0b0010, falling edge)  → captures every rising BEMF ZC
 *   CCP5 (MOD=0b0001, rising edge)   → captures every falling BEMF ZC
 *     (ATA6847 comparator is inverted)
 *
 *   DMA0 triggers on SCCP2 capture (CHSEL=0x0C), source=&CCP2BUFL,
 *     destination=dmaRingRising[] (128 entries, circular wrap),
 *     one 16-bit word per trigger.
 *
 *   DMA1 triggers on SCCP5 capture (CHSEL=0x16), source=&CCP5BUFL,
 *     destination=dmaRingFalling[] (128 entries, circular wrap),
 *     one 16-bit word per trigger.
 *
 *   Same floating-phase RP pin routed to both ICM2R and ICM5R each
 *   commutation (PPS selectors are independent, no conflict).
 *
 *   Both timers free-running forever. CCPON never toggled after init.
 *   _CCP2IE and _CCP5IE remain 0 always — DMA owns the FIFO drain.
 *
 * Shadow-only: HAL_ZcDma_Probe() is called from RecordZcTiming AFTER
 * the poll path has committed lastZcTickHR. It scans the appropriate
 * ring for captures in [windowOpenHR, currentHR] and reports deltas
 * against the poll timestamp and the predicted ZC. The results live
 * in GARUDA_DATA_T::dmaShadow and are exposed via GSP telemetry.
 * The live scheduler is never affected.
 */

#ifndef HAL_IC_DMA_H
#define HAL_IC_DMA_H

#include "../garuda_config.h"

#if FEATURE_IC_ZC && FEATURE_IC_DMA_SHADOW

#include <stdint.h>
#include <stdbool.h>

#define DMA_ZC_RING_SIZE   128u   /* 256 bytes per ring, power of 2 for mask */
#define DMA_ZC_RING_MASK   (DMA_ZC_RING_SIZE - 1u)

/**
 * @brief Per-step probe result. Populated by HAL_ZcDma_Probe().
 */
typedef struct {
    bool      found;              /* true if at least one capture in window */
    uint8_t   edgeCount;          /* captures found in window (0..~8 typical) */
    uint16_t  earliestHR;         /* earliest capture after windowOpenHR, HR domain */
    uint16_t  closestHR;          /* capture closest to expectedHR */
    int16_t   earliestVsPoll;     /* earliestHR − pollAcceptedHR (signed HR ticks) */
    int16_t   earliestVsExpected; /* earliestHR − expectedHR */
    int16_t   closestVsExpected;  /* closestHR − expectedHR */
    bool      ringWrappedSinceMark;   /* true if ring wrapped past mark */
} HAL_ZcDma_Result;

/**
 * @brief One-time init. Must be called AFTER HAL_ComTimer_Init()
 * (which starts CCP4) so the time-base offset can be measured.
 *
 * Configures CCP2 and CCP5 as free-running captures, initializes
 * DMA0 and DMA1 with the ring buffers as circular destinations,
 * and establishes the CCP2→CCP4 and CCP5→CCP4 HR domain offsets.
 */
void HAL_ZcDma_Init(void);

/**
 * @brief Route the currently-floating BEMF phase's RP pin to both
 * ICM2R and ICM5R. Called once per commutation from OnCommutation.
 * Captures the current DMA write heads into the shadow state for
 * bounded ring scans on the next probe.
 *
 * @param rpPin  RP pin number of the floating phase comparator.
 * @param risingZc  true if next ZC is rising polarity (uses CCP2 ring),
 *                  false if falling (uses CCP5 ring).
 */
void HAL_ZcDma_OnCommutation(uint16_t rpPin, bool risingZc);

/**
 * @brief Scan the appropriate ring for captures in the current step's
 * valid window. Called from RecordZcTiming after lastZcTickHR commits.
 *
 * @param windowOpenHR  HR timestamp when the 50% interval gate opens.
 * @param expectedHR    HR timestamp of the predicted ZC
 *                      (lastZcTickHR of previous step + zcIntervalHR).
 * @param pollAcceptedHR The HR timestamp the poll path just committed.
 * @param risingZc      Which ring to scan.
 * @param result        Out: populated with scan results.
 */
void HAL_ZcDma_Probe(uint16_t windowOpenHR,
                     uint16_t expectedHR,
                     uint16_t pollAcceptedHR,
                     bool     risingZc,
                     HAL_ZcDma_Result *result);

/**
 * @brief Refine a poll-detected ZC timestamp using the DMA ring.
 *
 * Scans the appropriate ring for the DMA-captured edge closest (in
 * absolute HR-tick distance) to the poll-accepted timestamp, bounded
 * by a lookback window. If a matching capture exists within
 * [pollHR - maxLookbackHR, pollHR + maxLookbackHR], returns that
 * capture's HR tick — which is hardware-precise and therefore
 * earlier/cleaner than the poll time. If no capture is in range,
 * returns pollHR unchanged.
 *
 * The ring scan is bounded by the commutation-head marker captured
 * at the last HAL_ZcDma_OnCommutation call, so only captures from the
 * current step are eligible.
 *
 * @param pollHR          Poll-detected ZC timestamp (HR domain)
 * @param maxLookbackHR   Max absolute deviation allowed (HR ticks)
 * @param risingZc        Which ring (true = rising ZC / CCP2)
 * @param correctionOut   Optional out-param: signed delta (refined - poll).
 *                        Pass NULL if not needed.
 * @return Refined HR timestamp, or pollHR if no capture in range.
 */
uint16_t HAL_ZcDma_RefineTimestamp(uint16_t  pollHR,
                                   uint16_t  maxLookbackHR,
                                   bool      risingZc,
                                   int16_t  *correctionOut);

/**
 * @brief Dump all edges captured since the last commutation marker.
 *
 * Walks the appropriate ring from commHead to current head, converts
 * each capture into the HR (SCCP4) domain, and writes them in order
 * into the caller's buffer. Intended for diagnostic/research tools
 * that want to see the raw edge pattern within a single commutation
 * step (e.g. the DMA burst capture module).
 *
 * @param risingZc   true → dmaRingRising (CCP2), false → dmaRingFalling (CCP5)
 * @param outBuf     caller buffer receiving HR-domain timestamps (oldest first)
 * @param maxCount   max entries to write (oldest are preserved if more exist)
 * @return number of entries actually written (0..maxCount)
 */
uint8_t HAL_ZcDma_DumpSinceCommutation(bool      risingZc,
                                       uint16_t *outBuf,
                                       uint8_t   maxCount);

/**
 * @brief Detect ZC from the DMA ring using a WINDOWED cluster rule.
 *
 * Scans the appropriate ring from commutation marker to current head.
 * Finds the LAST cluster with ≥2 edges (gap ≤ ~6.4 µs) whose start
 * is at or after windowOpenHR AND whose end is at or before
 * windowCloseHR.
 *
 * The close bound is critical: without it the detector picks post-ZC
 * noise clusters. In the bootstrap phase, close = pollHR. Once the
 * DPLL locks, close = predZcHR + margin.
 *
 * @param windowOpenHR   50% gate: ignore clusters before this time
 * @param windowCloseHR  Upper bound: ignore clusters after this time
 * @param risingZc       Which ring to scan
 * @param zcTimestampOut Out: cluster midpoint HR tick (only if found)
 * @return true if a qualifying cluster was found
 */
bool HAL_ZcDma_DetectZc(uint16_t  windowOpenHR,
                        uint16_t  windowCloseHR,
                        bool      risingZc,
                        uint16_t *zcTimestampOut);

#endif /* FEATURE_IC_ZC && FEATURE_IC_DMA_SHADOW */
#endif /* HAL_IC_DMA_H */
