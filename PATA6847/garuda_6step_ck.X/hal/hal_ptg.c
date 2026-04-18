/**
 * @file hal_ptg.c
 * @brief V5.0 Peripheral Trigger Generator — implementation.
 *
 * Command queue layout (packed 2×8-bit steps per PTGQUE16):
 *   STEP0: PTGWHI | 0x0   wait for PTGI0 (= PWM1 ADC Trigger 2, fires
 *                         at PG1TRIGA match, i.e. PWM valley)
 *   STEP1: PTGCTRL| 0x8   start PTGT0, wait for match (= PTGT0LIM ticks)
 *   STEP2: PTGIRQ | 0x0   generate _PTG0Interrupt
 *   STEP3: PTGJMP | 0x0   loop back to STEP0
 *
 * PTG clock: FCY/PTGDIV. PTGCON=0 → FCY=100 MHz, /1 = 100 MHz, 10 ns/tick.
 *
 * Opcode values verified against dsPIC33CK256MP508 Family Data Sheet
 * DS70005349 Table 24-1 (PTG Step Command Format). The earlier V3-era
 * file in this location used incorrect opcodes (0x30/0x50/0x70) which
 * would have executed reserved/wrong commands — that code is removed.
 */

#include "hal_ptg.h"

/* Counters defined unconditionally so telemetry code can read them
 * without a flag gate. They stay at 0 when FEATURE_V5_PTG_ZC=0 because
 * the ISR that increments them is also only compiled with the flag on. */
volatile uint32_t v5_ptgFires      = 0;
volatile uint32_t v5_ptgRisingAcc  = 0;
volatile uint32_t v5_ptgRisingRej  = 0;
volatile uint32_t v5_ptgFallingAcc = 0;
volatile uint32_t v5_ptgFallingRej = 0;

#if FEATURE_V5_PTG_ZC

#include <xc.h>

/* Per-sector expected post-ZC comp state, written by Commutate:
 *   0 = rising sector (inverted comp drops to 0 after ZC)
 *   1 = falling sector (inverted comp rises to 1 after ZC)
 * Reading this instead of calling HAL_Capture_IsRisingZc() saves a
 * function call in the hot ISR path. */
volatile uint8_t v5_ptgExpectedComp = 0;

/* The floating-phase GPIO-read logic lives in garuda_service.c as a
 * static inline. Replicate here so we can read BEMF from the PTG ISR
 * without exposing the static symbol. v4_floatingPhase itself is an
 * extern volatile global written in Commutate. */
extern volatile uint8_t v4_floatingPhase;

static inline uint8_t ReadBemfCompLocal(void)
{
    switch (v4_floatingPhase) {
        case 0:  return _RC6;
        case 1:  return _RC7;
        case 2:  return _RD10;
        default: return 0;
    }
}

/* ── PTG step-command opcodes (CMD<3:0> in upper nibble) ─────────── */
#define PTG_OP_CTRL     (0x0u << 4)   /* 0x00 — PTGCTRL */
#define PTG_OP_WHI      (0x4u << 4)   /* 0x40 — wait for trigger rising edge */
#define PTG_OP_WLO      (0x5u << 4)   /* 0x50 — wait for trigger falling edge */
#define PTG_OP_IRQ      (0x7u << 4)   /* 0x70 — generate interrupt */
#define PTG_OP_JMP      (0xAu << 4)   /* 0xA0 — jump to step index */

/* PTGCTRL sub-options (lower nibble) */
#define PTG_CTRL_WAIT_T0    0x8u      /* Start PTGT0, wait for PTGT0LIM match */

/* Helper: pack two 8-bit steps into a 16-bit PTGQUE register
 * (low-index step in low byte, high-index step in high byte). */
#define PTG_PACK(lo, hi)    ((uint16_t)(((uint16_t)(hi) << 8) | (uint8_t)(lo)))

void HAL_PTG_Init(void)
{
    /* Ensure PTG is off while we configure. */
    PTGCST   = 0x0000;
    PTGCON   = 0x0000;          /* clock = FCY, divider = 1 (10 ns/tick) */
    PTGT0LIM = V5_PTG_VALLEY_DELAY;
    PTGT1LIM = 0x0000;
    PTGSDLIM = 0x0000;          /* no extra per-step delay */
    PTGC0LIM = 0x0000;
    PTGC1LIM = 0x0000;
    PTGBTE   = 0x0000;
    PTGBTEH  = 0x0000;
    PTGQPTR  = 0x0000;

    /* Step queue — see file header for the command sequence. */
    PTGQUE0  = PTG_PACK(PTG_OP_WHI | 0x0,
                        PTG_OP_CTRL| PTG_CTRL_WAIT_T0);
    PTGQUE1  = PTG_PACK(PTG_OP_IRQ | 0x0,
                        PTG_OP_JMP | 0x0);
    PTGQUE2  = 0x0000;
    PTGQUE3  = 0x0000;

    /* Interrupt: clear flag, set priority, leave disabled until Start. */
    _PTG0IF  = 0;
    _PTG0IP  = V5_PTG_ISR_PRIORITY;
    _PTG0IE  = 0;
}

void HAL_PTG_Start(void)
{
    /* Full re-init on every motor start so post-desync restarts get a
     * clean PTG state. The boot-time HAL_PTG_Init alone leaves residual
     * register values from the previous run that can pin the state
     * machine in odd places after a desync/restart cycle. */
    HAL_PTG_Init();

    /* Reset diagnostic counters so each run shows fresh ratios. */
    v5_ptgFires      = 0;
    v5_ptgRisingAcc  = 0;
    v5_ptgRisingRej  = 0;
    v5_ptgFallingAcc = 0;
    v5_ptgFallingRej = 0;
    v5_ptgExpectedComp = 0;

    /* DIAGNOSTIC step 2d: keep PTG hardware running but DO NOT enable
     * the IRQ. This isolates the cost of PTG state machine activity
     * from the cost of the ISR fire itself. If the motor reaches V4
     * speeds (~107k) here, the ISR latency on ADC was the culprit; if
     * it still desyncs at ~78k, the PTG hardware itself is the source
     * (perhaps SFR-bus contention, or some PWM trigger interaction). */
    _PTG0IF = 0;
    _PTG0IE = 0;                /* TEMPORARILY DISABLED for isolation */
    PTGCST  = 0x8000;           /* PTGEN = 1, PTGSTRT = 0 */
    PTGCSTbits.PTGSTRT = 1;     /* start queue execution */
}

void HAL_PTG_Stop(void)
{
    PTGCSTbits.PTGSTRT = 0;
    PTGCST  = 0x0000;           /* PTGEN = 0 */
    _PTG0IE = 0;
    _PTG0IF = 0;
}

void HAL_PTG_SetDelay(uint16_t ptgTicks)
{
    /* PTGT0LIM is read on each PTGCTRL-start-T0 step. A mid-run write
     * takes effect on the next iteration — no stop/restart needed. */
    PTGT0LIM = ptgTicks;
}

/* ── PTG0 ISR ─────────────────────────────────────────────────────── */
/* Fires at (PWM valley + PTGT0LIM PTG ticks).
 *
 * Step-2c body: bare minimum — count fires, exit. The earlier body that
 * read BEMF GPIO and tallied accept/reject was already proven (pR≈67,
 * pF≈67 in step 2b bench data) but cost enough CPU at high speed to
 * cap the motor at 77 kRPM vs V4's 107 kRPM. Shadow accept counters
 * (v5_ptgRising/FallingAcc/Rej) are kept defined so telemetry still
 * decodes; they just don't increment from this ISR anymore.
 *
 * To reintroduce per-polarity sampling later without the speed hit,
 * either (a) halve PTG fire rate via the step queue (WHI twice) or
 * (b) feed PTG samples into the PI directly so the work pays off in
 * better PI tracking, not just diagnostic counters. */
void __attribute__((interrupt, no_auto_psv)) _PTG0Interrupt(void)
{
    _PTG0IF = 0;
    v5_ptgFires++;
}

#endif /* FEATURE_V5_PTG_ZC */
