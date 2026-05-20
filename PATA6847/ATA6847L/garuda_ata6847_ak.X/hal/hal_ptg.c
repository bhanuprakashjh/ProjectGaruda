/**
 * @file hal_ptg.c
 * @brief PTG implementation — drives the BEMF midpoint sampler.
 *
 * See hal_ptg.h for the architectural overview.
 *
 * Step queue layout (4 bytes packed into PTGQUE0 — STEP0 = LSB):
 *   STEP0: PTGWHI | 0       0x40    wait for PTG Input 0 rising edge
 *                                   (= PWM1 ADC Trigger 2 = PG1TRIGB match)
 *   STEP1: PTGIRQ | 0       0x70    generate _PTG0Interrupt
 *   STEP2: PTGJMP | 0       0xA0    jump back to STEP0
 *   STEP3: (NOP, unreached) 0x00
 * Packed: PTGQUE0 = 0x00A07040.
 *
 * Opcode encoding verified against DS70005539 Table 26-5.
 *
 * PTGCON:
 *   ON       = 1   peripheral enabled
 *   PTGDIV   = 0   clock = FCY/1 = 200 MHz → 5 ns/PTG-tick
 *   PTGITM   = 0   continuous, all input triggers visible
 *   PTGSSEN  = 0   no single-step
 *   PTGSTRT  = 1   begin queue execution
 */

#include <xc.h>
#include <stdint.h>
#include "hal_ptg.h"
#include "port_config.h"           /* BEMF_x_GetValue() macros */
#include "../garuda_config.h"
#include "../garuda_service.h"     /* ProcessBemfSample() */

/* ── Globals (defined here, externed from hal_ptg.h) ──────────────── */
volatile uint32_t ptgFires      = 0;
volatile uint32_t ptgSkipped    = 0;

/* Per-sector expected post-ZC comparator state — written by Commutate
 * in sector_pi.c when POST_ZC_ACCEPT is enabled.  Read in the BEMF
 * ISR paths to classify accept vs reject. */
volatile uint8_t  ptgExpectedComp = 0;

/* ── PTG step-command opcodes (DS70005539 Table 26-5) ─────────────── */
#define PTG_OP_CTRL     (0x0u << 4)   /* 0x00 — PTGCTRL  */
#define PTG_OP_WHI      (0x4u << 4)   /* 0x40 — wait for trigger rising  */
#define PTG_OP_WLO      (0x5u << 4)   /* 0x50 — wait for trigger falling */
#define PTG_OP_IRQ      (0x7u << 4)   /* 0x70 — generate IRQ (operand selects PTG0..3) */
#define PTG_OP_JMP      (0xAu << 4)   /* 0xA0 — jump (operand = step index) */

/* ISR priority 5 — above ADC (3) and Timer1 (4), below CCT3 (6).
 * Codex review flagged the hardcoded 5 as ignoring PTG_ISR_PRIORITY=4 in
 * garuda_config.h, but lowering PTG to 4 puts it at the SAME level as
 * Timer1 — they then serialize via natural-vector priority instead of
 * preempting. Under any meaningful motor load this causes PTG to miss
 * fires while Timer1 is mid-ISR, and the MCU goes silent (no trap, no
 * LED) shortly after entering CL. Keep this at 5; the garuda_config.h
 * value is unused for PTG (left in place only for documentation). */
#ifndef HAL_PTG_ISR_PRIORITY
#define HAL_PTG_ISR_PRIORITY    5U
#endif

void HAL_PTG_Init(void)
{
    /* Disable everything before configuring.  Writing PTGCON.ON = 0
     * with PTGSTRT = 0 puts the peripheral into a known idle state. */
    PTGCON   = 0x00000000UL;
    PTGT0LIM = 0;
    PTGT1LIM = 0;
    PTGSDLIM = 0;       /* no per-step delay */
    PTGC0LIM = 0;
    PTGC1LIM = 0;
    PTGBTE   = 0;
    PTGHOLD  = 0;
    PTGQPTR  = 0;       /* start at STEP0 */

    /* Step queue — see file header for the assembled bytes. */
    PTGQUE0  = ((uint32_t)(PTG_OP_WHI | 0x0))            /* STEP0 */
             | ((uint32_t)(PTG_OP_IRQ | 0x0) << 8)       /* STEP1 */
             | ((uint32_t)(PTG_OP_JMP | 0x0) << 16);     /* STEP2 */
    PTGQUE1  = 0;
    PTGQUE2  = 0;
    PTGQUE3  = 0;
    PTGQUE4  = 0;
    PTGQUE5  = 0;
    PTGQUE6  = 0;
    PTGQUE7  = 0;

    /* IRQ output 0 → _PTG0Interrupt.  Configure priority and clear the
     * flag, leave the IE disabled until HAL_PTG_Start(). */
    _PTG0IF  = 0;
    _PTG0IP  = HAL_PTG_ISR_PRIORITY;
    _PTG0IE  = 0;
}

void HAL_PTG_Start(void)
{
    /* Re-init each start for clean post-desync state. */
    HAL_PTG_Init();

    ptgFires        = 0;

    _PTG0IF = 0;
    _PTG0IE = 1;

    PTGCONbits.ON      = 1;     /* enable peripheral */
    PTGCONbits.PTGSTRT = 1;     /* begin step-queue execution */
}

void HAL_PTG_Stop(void)
{
    PTGCONbits.PTGSTRT = 0;
    PTGCONbits.ON      = 0;
    _PTG0IE = 0;
    _PTG0IF = 0;
}

/* ── PTG0 ISR ──────────────────────────────────────────────────────
 * Fires at every PG1TRIGB match (PWM mid-OFF, period boundary, by
 * default — same instant the BEMF GPIO is stable for sampling).
 *
 *  - ptgFires++ runs every fire (heartbeat counter for diagnostics).
 *  - ProcessBemfSample() runs here (not in the ADC ISR). The ADC
 *    ISR retains POT/Vbus/current responsibilities only. Latency drops
 *    from ~1.5 µs (ADC scan complete) to ~50 ns (ISR vector). */
void __attribute__((interrupt, no_auto_psv)) _PTG0Interrupt(void)
{
    _PTG0IF = 0;
    ptgFires++;
#if PTG_POSTSCALE_N > 1U
    /* Process 1 of every PTG_POSTSCALE_N fires (rate experiment). */
    static uint8_t postscaler = 0;
    if (++postscaler < PTG_POSTSCALE_N) {
        ptgSkipped++;
        PG1TRIGB = PTG_TRIG_MID_ON_POS;   /* keep next fire armed at MID-ON */
        return;
    }
    postscaler = 0;
#endif
    /* Duty-adaptive sample position.
     *
     * Center-aligned PWM: OFF window is (1-duty)·period wide, ON window
     * is duty·period. Below 50% duty the OFF window is wider → MID-OFF
     * is farthest from switching edges. Above 50% the ON window is
     * wider → MID-ON is cleanest. Above ~80% duty MID-OFF desyncs (OFF
     * window too narrow for PTG ISR latency + 3-read deglitch to fit).
     *
     * One position per fire (not both simultaneously). Hysteresis band
     * ±5% around 50% to avoid chatter when PI hunts across the boundary. */
    ProcessBemfSample();

    {
        extern volatile uint16_t g_pwmActualDuty;
        static uint8_t lastWasMidOn = 0;   /* sticky for hysteresis */
        uint16_t duty = g_pwmActualDuty;
        if (lastWasMidOn) {
            if (duty < (PTG_DUTY_ADAPT_THRESHOLD - PTG_DUTY_ADAPT_HYST)) {
                PG1TRIGB = PTG_TRIG_MID_OFF_POS;
                lastWasMidOn = 0;
            } else {
                PG1TRIGB = PTG_TRIG_MID_ON_POS;
            }
        } else {
            if (duty > (PTG_DUTY_ADAPT_THRESHOLD + PTG_DUTY_ADAPT_HYST)) {
                PG1TRIGB = PTG_TRIG_MID_ON_POS;
                lastWasMidOn = 1;
            } else {
                PG1TRIGB = PTG_TRIG_MID_OFF_POS;
            }
        }
    }
}
