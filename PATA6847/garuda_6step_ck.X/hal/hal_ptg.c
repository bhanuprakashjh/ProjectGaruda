/**
 * @file hal_ptg.c
 * @brief PTG edge-relative BEMF sampling for clean ZC detection.
 *
 * Solves the CLC clock position problem: PG1TRIGA=0 is counter-relative,
 * so at certain duty ratios the CLC D-FF samples near switching edges.
 * PTG samples at a FIXED DELAY after the edge — always clean.
 *
 * Hardware chain:
 *   PG1TRIGB (set to duty value) fires at ON→OFF edge
 *   → PTGBTE routes trigger to PTG broadcast input
 *   → PTG step queue: WAIT trigger → delay Timer0 → fire IRQ
 *   → PTG0 ISR reads raw GPIO (BEMF comparator pin)
 *   → FastPoll uses this clean sample instead of CLC output
 *
 * CPU cost: ~15 instructions per ISR × 24kHz (or 48kHz in center-aligned)
 *           = 0.4-0.7% CPU. Negligible.
 */

#include <xc.h>
#include "hal_ptg.h"
#include "hal_clc.h"
#include "port_config.h"
#include "../garuda_config.h"
#include "../garuda_types.h"

#if FEATURE_PTG_ZC && !FEATURE_V4_SECTOR_PI

extern volatile GARUDA_DATA_T gData;

/* PTG step queue command encoding (8-bit: CMD[7:4] | OPT[3:0]) */
#define PTG_CMD_WAIT_T0     0x01u   /* PTGCTRL: wait for Timer 0 expiry */
#define PTG_CMD_WHI(n)      (0x30u | ((n) & 0x0Fu))  /* Wait trigger n HIGH */
#define PTG_CMD_IRQ(n)      (0x50u | ((n) & 0x03u))  /* Fire PTG interrupt n */
#define PTG_CMD_JMP(n)      (0x70u | ((n) & 0x0Fu))  /* Jump to step n */

/* Pack two 8-bit steps into one 16-bit queue register */
#define PTG_PACK(step_lo, step_hi)  (((uint16_t)(step_hi) << 8) | (step_lo))

void HAL_PTG_Init(void)
{
    /* Disable PTG during configuration */
    PTGCST = 0x0000;

    /* PTG clock: Fosc/2 = 100MHz (10ns/tick), no additional divider.
     * PTGCLK[14:12] = 000 = Fosc/2
     * PTGDIV[11:7]  = 00000 = /1
     * PTGPWD[6:3]   = 0000
     * PTGWDT[2:0]   = 000 */
    PTGCON = 0x0000;

    /* Timer 0 limit: delay after switching edge for ringing to settle.
     * At 100MHz: 1µs = 100 ticks. 7µs = 700 ticks. */
    PTGT0LIM = (uint16_t)((uint32_t)PTG_DELAY_US * 100u);

    /* Broadcast trigger enable: route PG1 ADC Trigger 2 to PTG.
     *
     * PTGBTE bit mapping is device-specific (DS70005178 Table 2).
     * For dsPIC33CK64MP205, the mapping is believed to be:
     *   Bit 8: PG1 ADC Trigger 1 (TRIGA)
     *   Bit 9: PG1 ADC Trigger 2 (TRIGB)  ← we use this
     *
     * If PTG doesn't trigger: try PTG_BTE_BIT = 1, 2, or check
     * the device datasheet broadcast trigger table. */
    /* Route trigger: low 16 bits in PTGBTE, high 16 in PTGBTEH */
#if PTG_BTE_BIT < 16u
    PTGBTE = (1u << PTG_BTE_BIT);
#else
    PTGBTEH = (1u << (PTG_BTE_BIT - 16u));
#endif

    /* Step queue (4 steps, continuous loop):
     *
     * Step 0: PTGWHI(PTG_BTE_BIT) — wait for PG1 TRIGB to go high
     *         (fires at ON→OFF switching edge)
     *
     * Step 1: PTGCTRL wait Timer 0 — delay PTG_DELAY_US µs
     *         (ringing settles, comparator output is clean)
     *
     * Step 2: PTGIRQ(0) — fire PTG0 interrupt
     *         (ISR reads raw BEMF GPIO pin)
     *
     * Step 3: PTGJMP(0) — loop back to step 0
     *         (repeat every PWM cycle) */
    PTGQUE0 = PTG_PACK(PTG_CMD_WHI(PTG_BTE_BIT), PTG_CMD_WAIT_T0);
    PTGQUE1 = PTG_PACK(PTG_CMD_IRQ(0),           PTG_CMD_JMP(0));

    /* Clear remaining queue entries */
    PTGQUE2 = 0;
    PTGQUE3 = 0;
    PTGQUE4 = 0;
    PTGQUE5 = 0;
    PTGQUE6 = 0;
    PTGQUE7 = 0;

    /* Configure PTG0 interrupt */
    _PTG0IF = 0;
    _PTG0IP = 5;    /* Same priority as ZC poll — no preemption race */
    _PTG0IE = 1;

    /* Reset queue pointer to step 0 */
    PTGQPTR = 0;
}

void HAL_PTG_Start(void)
{
    /* Reset queue pointer and clear stale samples */
    PTGQPTR = 0;
    gData.icZc.ptgSampleValid = 0;

    /* Enable and start PTG execution */
    PTGCSTbits.PTGEN = 1;
    PTGCSTbits.PTGSTRT = 1;
}

void HAL_PTG_Stop(void)
{
    PTGCSTbits.PTGEN = 0;
    _PTG0IE = 0;
    _PTG0IF = 0;
    gData.icZc.ptgSampleValid = 0;
}

/**
 * @brief PTG0 ISR — fires PTG_DELAY_US µs after each PWM switching edge.
 *
 * Reads raw BEMF GPIO and FORCES the CLC D-FF to match via async R/S.
 * This gives the CLC the correct value at an edge-relative moment,
 * fixing the counter-relative clock position issue.
 *
 * Key insight: CLC D-FF latch-and-hold is what makes it work — poll
 * reads a stable held value, not a noisy instantaneous one. PTG just
 * ensures the CLC is loaded at the RIGHT TIME (edge-relative), while
 * the regular PWMEVTA clock may load it at a noisy time at certain
 * duty ratios. PTG's update overrides the potentially bad PWMEVTA
 * sample, and CLC holds the clean value until next update.
 *
 * FastPoll reads CLC output as before — no change to the poll path.
 */
void __attribute__((interrupt, no_auto_psv)) _PTG0Interrupt(void)
{
    _PTG0IF = 0;

    uint8_t ch = gData.icZc.activeChannel;
    uint8_t raw;

    switch (ch)
    {
        case 0:  raw = BEMF_A_GetValue() ? 1 : 0; break;
        case 1:  raw = BEMF_B_GetValue() ? 1 : 0; break;
        case 2:  raw = BEMF_C_GetValue() ? 1 : 0; break;
        default: return;
    }

    /* Force the floating phase CLC D-FF Q output to match raw BEMF.
     * R/S is async — takes effect immediately, CLC holds until next update. */
    HAL_CLC_ForceState(ch, raw);
}

#endif /* FEATURE_PTG_ZC && !FEATURE_V4_SECTOR_PI */
