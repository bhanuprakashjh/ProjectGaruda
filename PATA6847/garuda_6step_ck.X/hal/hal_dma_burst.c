/**
 * @file hal_dma_burst.c
 * @brief One-shot DMA edge burst capture — see hal_dma_burst.h.
 */

#include "hal_dma_burst.h"

#if FEATURE_DMA_BURST_CAPTURE && !FEATURE_V4_SECTOR_PI

#include <string.h>
#include "hal_ic_dma.h"

/* ── Capture buffer ──────────────────────────────────────────────── */
static DMA_BURST_STEP_T burstSteps[DMA_BURST_STEPS];
static volatile uint8_t burstNextIdx;
static volatile uint8_t burstState;
static volatile bool    slotOpen;   /* true after OnCommutation, false after OnZc */

/* ── API ─────────────────────────────────────────────────────────── */

void HAL_DmaBurst_Arm(void)
{
    /* Clear whole buffer so readout of unused slots is well-defined. */
    memset(burstSteps, 0, sizeof(burstSteps));
    burstNextIdx = 0;
    slotOpen     = false;
    burstState   = DMA_BURST_CAPTURING;
}

uint8_t HAL_DmaBurst_GetState(void)
{
    return burstState;
}

uint8_t HAL_DmaBurst_GetStepCount(void)
{
    return burstNextIdx;
}

const DMA_BURST_STEP_T *HAL_DmaBurst_GetStep(uint8_t idx)
{
    if (idx >= DMA_BURST_STEPS) return 0;
    return &burstSteps[idx];
}

/* ── Hooks ───────────────────────────────────────────────────────── */

/* NOTE ON CALL ORDER:
 *
 * HAL_DmaBurst_OnCommutation MUST be called BEFORE HAL_ZcDma_OnCommutation
 * in BEMF_ZC_OnCommutation. Reason: we dump the prior step's edges using
 * HAL_ZcDma_DumpSinceCommutation, which reads the commHead marker inside
 * hal_ic_dma. That marker is updated (to the new step) by
 * HAL_ZcDma_OnCommutation. If the update runs first, the dump returns an
 * empty ring. Calling burst first means the dump uses the marker from
 * the step that just ended — which is what we want.
 *
 * Likewise the slot is closed at the NEXT commutation, not at ZC
 * acceptance, so the captured edge list spans the FULL step including
 * post-poll ringing tail. HAL_DmaBurst_OnZc only records pollHR/
 * predictedHR into the currently-open slot — it does NOT close it.
 */

void HAL_DmaBurst_OnCommutation(uint16_t commHR, uint8_t stepIdx, bool risingZc)
{
    if (burstState != DMA_BURST_CAPTURING) return;

    /* Close the previously-open slot (if any). The ring's commHead is
     * still pointing at the START of the prior step because
     * HAL_ZcDma_OnCommutation has not yet run for this new commutation
     * (see call order note above). */
    if (slotOpen)
    {
        DMA_BURST_STEP_T *s = &burstSteps[burstNextIdx];
        /* If OnZc never fired for this step, it's a timeout. pollHR and
         * predictedHR were initialised to 0 when the slot opened. */
        if (s->pollHR == 0 && s->predictedHR == 0)
            s->wasTimeout = 1u;
        s->edgeCount = HAL_ZcDma_DumpSinceCommutation(
            (s->risingZc != 0u),
            s->edges,
            DMA_BURST_EDGES_PER_STEP);
        burstNextIdx++;
        slotOpen = false;
        if (burstNextIdx >= DMA_BURST_STEPS)
        {
            burstState = DMA_BURST_FULL;
            return;
        }
    }

    if (burstNextIdx >= DMA_BURST_STEPS) return;

    /* Open a new slot for this step. edges are dumped at the NEXT
     * commutation so this slot's edge list will include every edge up
     * to and after pollHR. */
    DMA_BURST_STEP_T *s = &burstSteps[burstNextIdx];
    s->commHR      = commHR;
    s->stepIndex   = stepIdx;
    s->risingZc    = risingZc ? 1u : 0u;
    s->edgeCount   = 0;
    s->pollHR      = 0;
    s->predictedHR = 0;
    s->wasTimeout  = 0;
    slotOpen = true;
}

void HAL_DmaBurst_OnZc(uint16_t pollHR, uint16_t predictedHR, bool wasTimeout)
{
    if (burstState != DMA_BURST_CAPTURING) return;
    if (!slotOpen)                         return;
    if (burstNextIdx >= DMA_BURST_STEPS)   return;

    /* Record the poll acceptance into the CURRENTLY-OPEN slot, but do
     * NOT close it. The slot is closed at the next commutation so the
     * post-poll edge tail ends up in the capture too. */
    DMA_BURST_STEP_T *s = &burstSteps[burstNextIdx];
    s->pollHR      = pollHR;
    s->predictedHR = predictedHR;
    s->wasTimeout  = wasTimeout ? 1u : 0u;
}

#endif /* FEATURE_DMA_BURST_CAPTURE && !FEATURE_V4_SECTOR_PI */
