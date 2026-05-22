/**
 * @file hal_ioc.c
 * @brief Edge-triggered BEMF zero-crossing via dsPIC33AK CN/IOC.
 *
 * See hal_ioc.h + docs/ioc_bemf_detection_plan.md for the architecture.
 *
 * Wholly inert when FEATURE_IOC_BEMF=0. Mutually exclusive with
 * FEATURE_FOC_AN1078 (FOC doesn't sense BEMF).
 */

#include "../garuda_config.h"

#if FEATURE_IOC_BEMF

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "hal_ioc.h"
#include "port_config.h"
#include "../garuda_types.h"
#include "../motor/sector_pi.h"
#include "../motor/commutation.h"

/* ── BEMF pin layout (see port_config.h verified) ───────────────── */
#define BEMF_A_BIT      9U          /* RB9 — Port B */
#define BEMF_B_BIT      8U          /* RB8 — Port B */
#define BEMF_C_BIT      10U         /* RA10 — Port A */
#define BEMF_PB_MASK    ((1UL << BEMF_A_BIT) | (1UL << BEMF_B_BIT))
#define BEMF_PA_MASK    (1UL << BEMF_C_BIT)

/* ISR priority 5 — matches the PTG ISR slot we're replacing. Above
 * Timer1 (4) and ADC (3); below CCT3 commutation (6). */
#ifndef HAL_IOC_ISR_PRIORITY
#define HAL_IOC_ISR_PRIORITY    5U
#endif

/* Speed gate (Plan §3.6.2 Layer 5 — speed-adaptive). Skip IOC arming
 * when the motor is slow enough that BEMF can't reliably toggle the
 * ATA6847L comparator past the noise floor (~0.85 V peak at 14 k eRPM).
 * In that regime the comp output drifts across threshold without clean
 * edges → IOC stays silent for seconds and the motor runs on timeouts
 * anyway. Gating skips the wasted re-arm work and keeps startup clean.
 *
 * tMeasHR is the smoothed sector period in HR ticks (640 ns/tick).
 *   tMeasHR = 0           → first sectors post-handoff, arm anyway
 *   tMeasHR > IOC_MAX_TM  → too slow, skip arm
 *   tMeasHR <= threshold  → fast enough, arm normally
 *
 * 2400 HR ticks ≈ 1.5 ms/sector ≈ ~25 k eRPM. Empirically the
 * comparator chatter stops being a problem above this point. */
#ifndef HAL_IOC_MAX_PERIOD_HR
#define HAL_IOC_MAX_PERIOD_HR   2400U
#endif

/* L1 demag-blanking window in HR ticks (640 ns each). Bench 2026-05-21
 * showed scaling-with-T (was tMeasHR >> 2 = 25% of sector) rejected
 * ~97% of edges INCLUDING real ZCs across the whole 13–100k eRPM range
 * — because demag is a fixed physical time (set by motor L and Vbus),
 * not a fraction of sector. Source-board HWZC uses a fixed SCCP1
 * one-shot value for the same reason. 16 HR ticks ≈ 10 µs, which is
 * ~2× the worst-case demag overshoot on the 2810 at 24V. At 100k eRPM
 * (sector ≈ 100 µs) this is 10% of sector; at 10k eRPM (sector ≈
 * 1.5 ms) it's <1%. Tune down/up via this define if accept rate is
 * still poor / if polarity rejects spike (transient leakage). */
#ifndef HAL_IOC_DEMAG_BLANK_HR
#define HAL_IOC_DEMAG_BLANK_HR  16U
#endif

/* ── Diagnostic counters (exported via hal_ioc.h) ───────────────── */
volatile uint32_t iocAcceptCount    = 0;
volatile uint32_t iocDemagReject    = 0;
volatile uint32_t iocBlankReject    = 0;
volatile uint32_t iocFilterReject   = 0;
volatile uint32_t iocPolarityReject = 0;
volatile uint32_t iocIntervalReject = 0;
volatile uint32_t iocPolUnanimous   = 0;
volatile uint32_t iocFiresPortA     = 0;
volatile uint32_t iocFiresPortB     = 0;

/* Last ACCEPTED ZC timestamp (HR ticks). Layer 6 gates against this:
 * if a new candidate fires too soon after the last accept, it's noise.
 * Distinct from lastCaptureHR_g (which sector_pi consumes/clears). */
static volatile uint16_t lastAcceptedZcHR = 0;

/* ── Externals from sector_pi.c / commutation.c / garuda_service.c */
extern volatile uint8_t  currentSector;
extern volatile uint16_t lastCaptureHR_g;
extern volatile bool     captureValid;
extern volatile uint16_t lastCommHR;
extern volatile uint16_t blankingEndHR;
extern volatile uint16_t tMeasHR;          /* most recent sector period, HR */
extern const COMMUTATION_STEP_T commutationTable[6];

/* Floating phase of the currently-armed sector. Stashed by
 * HAL_Ioc_ArmForSector so the ISR can read it without re-deriving. */
static volatile uint8_t s_currentFp = 0;
/* Expected post-ZC comparator state (0 or 1) — Layer 4 polarity gate. */
static volatile uint8_t s_expectedComp = 0;

/* ── BEMF GPIO read (inline, mirrors garuda_service.c:ReadBEMFComp) */
static inline uint8_t IocReadBemf(uint8_t fp)
{
    switch (fp) {
        case 0: return BEMF_A_GetValue();
        case 1: return BEMF_B_GetValue();
        case 2: return BEMF_C_GetValue();
        default: return 0;
    }
}

/* ── Public API ─────────────────────────────────────────────────── */

void HAL_Ioc_Init(void)
{
    /* Port-level config — leave ON=0 until first arm. CNCONx.ON is
     * required for any per-pin enable to take effect.
     *
     * CRITICAL: CNSTYLE=1 selects **edge-detect mode**. With CNSTYLE=0
     * (mismatch mode, the reset default) CNEN1x is ignored, so only
     * rising edges (CNEN0x) would arm — every falling-edge sector
     * stays silent. This bug ate ~50% of real ZCs prior to 2026-05-21.
     * In edge mode: CNEN0x[n]=1 enables rising-edge interrupt for pin
     * n, CNEN1x[n]=1 enables falling-edge — datasheet DS70005539 §
     * 11.4.10.1, step 8. */
    CNCONA = 0;
    CNCONB = 0;
    CNCONAbits.CNSTYLE = 1;
    CNCONBbits.CNSTYLE = 1;

    /* Clear every per-pin enable on both ports. */
    CNEN0A = 0;  CNEN1A = 0;
    CNEN0B = 0;  CNEN1B = 0;

    /* No pull-ups — ATA6847L drives push-pull. */
    CNPUA = 0;  CNPDA = 0;

    /* Clear flag registers + IFS bits. */
    CNFA = 0;  CNFB = 0;
    _CNAIF = 0;  _CNBIF = 0;

    /* Set ISR priority but leave IE off. */
    _CNAIP = HAL_IOC_ISR_PRIORITY;
    _CNBIP = HAL_IOC_ISR_PRIORITY;
    _CNAIE = 0;
    _CNBIE = 0;
}

void HAL_Ioc_Disarm(void)
{
    /* Mask port enables first to silence the vectors. */
    _CNAIE = 0;
    _CNBIE = 0;

    /* Clear all per-pin enables. */
    CNEN0A = 0;  CNEN1A = 0;
    CNEN0B = 0;  CNEN1B = 0;

    /* Clear any pending flags so the next arm starts fresh. */
    CNFA = 0;  CNFB = 0;
    _CNAIF = 0;  _CNBIF = 0;

    /* Reset L6 anchor so the first ZC of next run isn't compared to a
     * stale timestamp from the previous run. */
    lastAcceptedZcHR = 0;
}

void HAL_Ioc_ArmForSector(uint8_t sector)
{
    if (sector >= 6) sector %= 6;
    const COMMUTATION_STEP_T *s = &commutationTable[sector];

    /* Tear down first — clean slate every sector. */
    _CNAIE = 0;
    _CNBIE = 0;
    CNEN0A = 0;  CNEN1A = 0;
    CNEN0B = 0;  CNEN1B = 0;

    /* Bench tested speed gate (skip arming below threshold) on
     * 2026-05-21 — introduced a discontinuity in sector_pi's control
     * loop when IOC switched on/off around the threshold. Caused fast
     * desync. Continuous arming (let L1/L2/L4 filter noise at all
     * speeds) was empirically better — keep this disabled. */

    /* Cache for the ISR. */
    s_currentFp = s->floatingPhase;

    /* ATA6847L inverts: rising-BEMF arrives as a FALLING comp edge,
     * post-ZC comp settles LOW. So:
     *   zcPolarity = +1 (rising BEMF)  → arm CNEN1 (falling), expect 0
     *   zcPolarity = -1 (falling BEMF) → arm CNEN0 (rising),  expect 1 */
    const bool rising_bemf = (s->zcPolarity > 0);
    s_expectedComp = rising_bemf ? 0u : 1u;

    switch (s->floatingPhase) {
        case 0:  /* Phase A → RB9, Port B */
            if (rising_bemf) CNEN1B = (1UL << BEMF_A_BIT);
            else             CNEN0B = (1UL << BEMF_A_BIT);
            CNCONBbits.ON = 1;
            CNFB = 0;  _CNBIF = 0;  _CNBIE = 1;
            break;
        case 1:  /* Phase B → RB8, Port B */
            if (rising_bemf) CNEN1B = (1UL << BEMF_B_BIT);
            else             CNEN0B = (1UL << BEMF_B_BIT);
            CNCONBbits.ON = 1;
            CNFB = 0;  _CNBIF = 0;  _CNBIE = 1;
            break;
        case 2:  /* Phase C → RA10, Port A */
            if (rising_bemf) CNEN1A = (1UL << BEMF_C_BIT);
            else             CNEN0A = (1UL << BEMF_C_BIT);
            CNCONAbits.ON = 1;
            CNFA = 0;  _CNAIF = 0;  _CNAIE = 1;
            break;
        default:
            /* Unknown floating phase — leave disarmed. */
            break;
    }
}

/* ── Shared accept body (five-layer rejection) ──────────────────── */

static inline __attribute__((always_inline))
void IocOnEdge(void)
{
    /* 0. Timestamp the edge. CCP4TMR is the HR timer (640 ns/tick).
     * dsPIC33AK SFR is 32-bit; low 16 carry the count we use. */
    const uint16_t now_hr = (uint16_t)CCP4TMR;

    /* Layer 1: demag-interval gate. Fixed absolute time, not a fraction
     * of T. See HAL_IOC_DEMAG_BLANK_HR comment above for rationale and
     * the 2026-05-21 trace that drove this change.
     *
     * Previous scaling-with-T (tMeasHR >> 2) was rejecting real ZCs at
     * all speeds because it widened with sector while the actual demag
     * overshoot stays around a few microseconds. */
    const uint16_t since_comm = (uint16_t)(now_hr - lastCommHR);
    if (since_comm < HAL_IOC_DEMAG_BLANK_HR) {
        iocDemagReject++;
        return;
    }

    /* Layer 2: static blanking gate. Seeded by Commutate to cover the
     * ringing+demag window at OL where tMeasHR is still being learned. */
    if ((int16_t)(now_hr - blankingEndHR) < 0) {
        iocBlankReject++;
        return;
    }

    /* Layer 6: ZC-interval filter (port of source-board HWZC pattern).
     * Reject candidates whose interval since the last ACCEPTED ZC is
     * implausibly short — switching transients can pass L1/L2/L3/L4 if
     * they land outside blanking with correct polarity, but they fire
     * way too soon after the last real ZC. The smoothed sector period
     * (tMeasHR) is the expected ZC-to-ZC interval; demand at least
     * 70 % of that before accepting. */
    if (lastAcceptedZcHR != 0u && tMeasHR > 0u) {
        const uint16_t since_last_zc = (uint16_t)(now_hr - lastAcceptedZcHR);
        const uint16_t min_interval  = (uint16_t)(((uint32_t)tMeasHR * 7U) / 10U);
        if (since_last_zc < min_interval) {
            iocIntervalReject++;
            return;
        }
    }

    /* Layer 3: 3-read consensus. Sub-µs PWM ringing settles by the time
     * we get here; three reads with 4 NOPs (~20 ns at FCY=200 MHz)
     * confirm a stable line state. */
    const uint8_t fp = s_currentFp;
    uint8_t r1 = IocReadBemf(fp);
    Nop(); Nop(); Nop(); Nop();
    uint8_t r2 = IocReadBemf(fp);
    Nop(); Nop(); Nop(); Nop();
    uint8_t r3 = IocReadBemf(fp);
    uint8_t comp = ((uint8_t)(r1 + r2 + r3) >= 2u) ? 1u : 0u;

    /* Layer 4: polarity gate. Reject edges in the wrong direction
     * (switching artifacts produce both polarities).
     *
     * Diagnostic split (2026-05-21): when all three reads agreed but
     * disagreed with s_expectedComp, the comparator was firmly settled
     * in the OPPOSITE direction — that's a real edge with the arm
     * polarity wrong (sector mismatch). Split votes are just noise.
     * iocPolUnanimous is the "sector mismatch" signal; the difference
     * iocPolarityReject - iocPolUnanimous is the noise signal. */
    if (comp != s_expectedComp) {
        if (r1 == r2 && r2 == r3) {
            iocPolUnanimous++;
        }
        iocPolarityReject++;
        return;
    }
    (void)iocFilterReject;   /* reserved for adaptive Layer 3 expansion */

    /* ACCEPT — publish to sector_pi via the existing protocol. */
    lastCaptureHR_g = now_hr;
    captureValid    = true;
    iocAcceptCount++;
    lastAcceptedZcHR = now_hr;       /* feeds Layer 6 next sector */

    /* Disarm both ports until Commutate re-arms. Prevents follow-up
     * edges from the same sector firing the ISR again. */
    _CNAIE = 0;  _CNBIE = 0;
    CNEN0A = 0;  CNEN1A = 0;
    CNEN0B = 0;  CNEN1B = 0;
}

/* ── ISR vectors. One body, two wrappers — only one is "armed" per
 *     sector via _CNxIE so the other can't fire. ─────────────────── */

void __attribute__((interrupt, no_auto_psv)) _CNAInterrupt(void)
{
    /* Latch + clear flags first to acknowledge the IRQ source. */
    CNFA = 0;
    _CNAIF = 0;
    iocFiresPortA++;
    IocOnEdge();
}

void __attribute__((interrupt, no_auto_psv)) _CNBInterrupt(void)
{
    CNFB = 0;
    _CNBIF = 0;
    iocFiresPortB++;
    IocOnEdge();
}

#endif /* FEATURE_IOC_BEMF */
