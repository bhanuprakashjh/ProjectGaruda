/**
 * @file hal_capture.c
 * @brief V4 Capture HAL (AK port — diag-only, motor not driven by it).
 *
 * Forked from CK `../../garuda_6step_ck.X/hal/hal_capture.c`. The V4
 * milestone (228k eRPM @ commit 24703f6) drives commutation from the
 * ADC ISR midpoint sampler (FEATURE_V4_MIDPOINT_ZC=1, garuda_service.c
 * line ~360) — CCP capture is INERT in the production path. This file
 * lives behind FEATURE_V4_CCP_DIAG for future research only.
 *
 * dsPIC33AK has SCCP1/2/3/4 only — there is no SCCP5 (unlike CK).
 * Every `CCP5*` reference below must be re-mapped to a spare SCCP slot
 * (SCCP3 is the only one free in our system: SCCP1 = poll, SCCP2 = IC
 * diag, SCCP4 = HR timer).  CCP5 was the falling-edge capture on CK;
 * on AK, re-purpose SCCP3 for that role.
 *
 * [AK PORT] open items:
 *  - Replace every `CCP5*` SFR with the SCCP3 equivalent.
 *  - SCCP4 stays as HR timer (640 ns ticks at FCY/64) — interface
 *    unchanged.
 *  - PPS routing of comparator GPIO → CCP input differs (use the AK
 *    PPS input table from DS70005527).
 *  - Until rewritten, this whole file is gated by FEATURE_V4_SECTOR_PI
 *    AND a new FEATURE_V4_CCP_DIAG (off by default) so the AK build
 *    can still produce a working ELF.
 */

#include "hal_capture.h"

#if FEATURE_V4_SECTOR_PI

#include <xc.h>
#include "../motor/v4_params.h"

/* AK port: keep just the shared state visible; rest of file is gated
 * by FEATURE_V4_CCP_DIAG (defined off in garuda_config.h until SCCP3
 * re-map is implemented for AK). */
volatile uint16_t v4_lastCaptureHR = 0;
volatile bool     v4_captureValid  = false;

#ifndef FEATURE_V4_CCP_DIAG
#define FEATURE_V4_CCP_DIAG 0
#endif

#if FEATURE_V4_CCP_DIAG

/* Polarity for current sector */
static bool currentRisingZc = true;

/* CCP→HR offsets (computed once at init) */
static uint16_t ccp2HrOffset;
static uint16_t ccp5HrOffset;

/* Blanking: HR timestamp at commutation + expected sector period */
static volatile uint16_t commTimeHR;
static volatile uint16_t blankingEndHR;

void HAL_Capture_Init(void)
{
    /* ── Enable DMA module (required for CCP FIFO access) ──────── */
    /* On dsPIC33CK, CCP2BUFL (@0x0994) and CCP5BUFL (@0x0A00) are
     * in the DMA-capable SFR range. The FIFO mechanism requires
     * DMAEN=1 even for CPU-only access (no DMA channels needed). */
    DMACONbits.DMAEN  = 1;
    DMACONbits.PRSSEL = 0;
    DMAL = 0x0800u;     /* Include DMA-capable SFR range */
    DMAH = 0x2FFFu;     /* End of data RAM */

    /* ── CCP2: falling edge = rising ZC ─────────────────────────── */
    CCP2CON1L = 0; CCP2CON1H = 0; CCP2CON2L = 0; CCP2CON2H = 0;
    CCP2CON1Lbits.CCSEL  = 1;
    CCP2CON1Lbits.T32    = 0;
    CCP2CON1Lbits.CLKSEL = 0b000;
    CCP2CON1Lbits.TMRPS  = 0b11;       /* 1:64 → 640 ns/tick */
    CCP2CON1Lbits.MOD    = 0b0010;     /* Falling edge — FIXED */
    CCP2PRL = 0xFFFFu;
    _CCP2IP = V4_CAPTURE_ISR_PRIORITY;
    _CCP2IP = V4_CAPTURE_ISR_PRIORITY; _CCP2IF = 0; _CCP2IE = 0;
    _CCT2IP = 1; _CCT2IF = 0; _CCT2IE = 0;
    CCP2CON1Lbits.ON = 1;           /* Start — NEVER toggled */

    /* ── CCP5: rising edge = falling ZC ─────────────────────────── */
    CCP5CON1L = 0; CCP5CON1H = 0; CCP5CON2L = 0; CCP5CON2H = 0;
    CCP5CON1Lbits.CCSEL  = 1;
    CCP5CON1Lbits.T32    = 0;
    CCP5CON1Lbits.CLKSEL = 0b000;
    CCP5CON1Lbits.TMRPS  = 0b11;
    CCP5CON1Lbits.MOD    = 0b0001;     /* Rising edge — FIXED */
    CCP5PRL = 0xFFFFu;
    _CCP5IP = V4_CAPTURE_ISR_PRIORITY; _CCP5IF = 0; _CCP5IE = 0;
    _CCT5IP = 1; _CCT5IF = 0; _CCT5IE = 0;
    CCP5CON1Lbits.ON = 1;

    /* ── CCP→HR offsets ─────────────────────────────────────────── */
    {
        uint16_t hr = CCP4TMRL;
        uint16_t c2 = CCP2TMRL;
        uint16_t c5 = CCP5TMRL;
        ccp2HrOffset = (uint16_t)(hr - c2);
        ccp5HrOffset = (uint16_t)(hr - c5);
    }

    /* ── SCCP1: 40 kHz periodic timer for OFF-mid falling ZC ──────
     * Fires CCT1IF at PWM peak (12.5 µs offset from valley).
     * SCCP CLKSEL=000 → Standard Speed Peripheral Clock = 100 MHz on
     * both CK and AK (CK: FPB=100MHz, AK: FPB/2=100MHz).
     * TMRPS=01 → 1:4 → 25 MHz, 40 ns/tick. Period 625 = 25 µs = PWM period.
     * Counter seeded at half-period in HAL_Capture_Start to align.
     *
     * [AK PORT] CCP1PRL formula below uses FCY/4 — on CK that yields
     * 25M (correct) because FCY = FPB = 100MHz. On AK FCY = 200MHz so
     * the formula yields 50M which is WRONG by 2x. When this block
     * goes live (FEATURE_V4_CCP_DIAG=1), replace `FCY` with the literal
     * 100000000UL Standard Speed Peripheral Clock value. */
    CCP1CON1L = 0; CCP1CON1H = 0; CCP1CON2L = 0; CCP1CON2H = 0;
    CCP1CON1Lbits.CCSEL  = 0;          /* Timer mode */
    CCP1CON1Lbits.T32    = 0;          /* 16-bit */
    CCP1CON1Lbits.CLKSEL = 0b000;      /* Standard Speed Periph Clk = 100 MHz */
    CCP1CON1Lbits.TMRPS  = 0b10;       /* 1:4 → 25 MHz, 40 ns/tick */
    CCP1CON1Lbits.MOD    = 0b0000;     /* Auto-reload on period match */
    CCP1PRL = (FCY / 4 / PWMFREQUENCY_HZ) - 2;  /* 623 = 24.96 µs (CK only — see [AK PORT] note above) */
    _CCT1IP = V5_SCCP1_ISR_PRIORITY;    /* V4 baseline 2; V5 bumps to 5 */
    _CCT1IF = 0;
    _CCT1IE = 0;                        /* Enabled at motor start */
    _CCP1IF = 0; _CCP1IE = 0;
    CCP1CON1Lbits.ON = 1;           /* Start free-running */
}

void HAL_Capture_Start(void)
{
    /* Flush FIFOs */
    (void)CCP2BUFL; (void)CCP2BUFL; (void)CCP2BUFL; (void)CCP2BUFL;
    (void)CCP5BUFL; (void)CCP5BUFL; (void)CCP5BUFL; (void)CCP5BUFL;
    v4_captureValid = false;
    _CCP2IF = 0; _CCP5IF = 0;
    _CCP2IE = 1;                /* RE-ENABLED 2026-04-29 — testing as prop-startup
                                 * regression suspect. Earlier this session disabled
                                 * to save FIFO-drain CPU; comments before that
                                 * called the ISRs "load-bearing" for low-speed BEMF
                                 * detection (FIFO drain side effects). */
    _CCP5IE = 1;                /* RE-ENABLED — same reason as CCP2. */

    /* Seed SCCP1 counter at half-period so it fires at PWM peak.
     * ADC fires at valley (counter=0); we want SCCP1 12.5 µs later.
     *
     * Phase B (2026-04-21): SCCP1 ISR disabled. Previously disabling
     * caused trips at 107k vs 196k baseline because its side effects
     * (ReadBEMFComp polls + ISR jitter) were load-bearing. Phase A
     * moved those side effects into the ADC ISR (FIFO drain + 3-read
     * deglitch on the GPIO) so they're now explicit. SCCP1 should be
     * safe to kill — pure CPU waste at this point.
     * Recovers ~120 ms/sec of CPU at 40 kHz ADC ISR rate. */
    CCP1TMRL = CCP1PRL / 2;
    _CCT1IF = 0;
    _CCT1IE = 0;                /* Phase B: was 1 — SCCP1 ISR disabled */
}

void HAL_Capture_Stop(void)
{
    _CCP2IE = 0; _CCP5IE = 0;
    _CCP2IF = 0; _CCP5IF = 0;
    _CCT1IE = 0; _CCT1IF = 0;  /* Stop off-mid ISR */
    v4_captureValid = false;
}

void HAL_Capture_Configure(uint8_t rpPin, bool risingZc)
{
    /* Route pin to BOTH CCP2 and CCP5 */
    __builtin_write_RPCON(0x0000);
    RPINR4bits.ICM2R = rpPin;
    RPINR7bits.ICM5R = rpPin;
    __builtin_write_RPCON(0x0800);

    currentRisingZc = risingZc;
    v4_captureValid = false;


    /* Store commutation time for blanking */
    commTimeHR = CCP4TMRL;
}

void HAL_Capture_SetBlanking(uint16_t sectorPeriodHR)
{
    /* Blanking: ignore captures in the first v4Params.blankingPct% of the
     * sector. Demagnetization typically lasts ~30-35% of the sector and
     * true ZC is around 50%. Default 40% rejects demag while preserving
     * the ZC window — runtime tunable via GUI for per-board / per-motor
     * adjustment. */
    uint8_t pct = v4Params.blankingPct;
    blankingEndHR = (uint16_t)(commTimeHR + ((uint32_t)sectorPeriodHR * pct / 100));
}

bool HAL_Capture_IsRisingZc(void) { return currentRisingZc; }
uint16_t HAL_Capture_GetCcp2Offset(void) { return ccp2HrOffset; }
uint16_t HAL_Capture_GetCcp5Offset(void) { return ccp5HrOffset; }
uint16_t HAL_Capture_GetBlankingEnd(void) { return blankingEndHR; }

#else /* !FEATURE_V4_CCP_DIAG — AK port stubs */

/* Stubs so the V4 sector PI code links; ADC-ISR midpoint path drives
 * the motor.  When FEATURE_V4_CCP_DIAG is wired up for AK SCCP3, the
 * full implementation above takes over.
 *
 * IMPORTANT: HAL_Capture_IsRisingZc() must still return the per-sector
 * polarity (set via HAL_Capture_Configure) even though the hardware
 * capture path is stubbed — the V4 ADC-ISR midpoint sampler uses it to
 * decide whether the comparator GPIO read matches the expected ZC
 * direction for the current sector. A "stuck true" value rejected
 * every falling-ZC sector (1, 3, 5) and prevented CL lock. */
static bool stubCurrentRisingZc = true;

void HAL_Capture_Init(void) { }
void HAL_Capture_Start(void) { }
void HAL_Capture_Stop(void) { }
void HAL_Capture_Configure(uint8_t rpPin, bool risingZc)
{
    (void)rpPin;
    stubCurrentRisingZc = risingZc;
}
void HAL_Capture_RouteForSector(uint8_t sector) { (void)sector; }
void HAL_Capture_OnCommutation(void) { }
void HAL_Capture_SetBlanking(uint16_t sectorPeriodHR) { (void)sectorPeriodHR; }
bool HAL_Capture_IsRisingZc(void) { return stubCurrentRisingZc; }
uint16_t HAL_Capture_GetCcp2Offset(void) { return 0; }
uint16_t HAL_Capture_GetCcp5Offset(void) { return 0; }
uint16_t HAL_Capture_GetBlankingEnd(void) { return 0; }

#endif /* FEATURE_V4_CCP_DIAG */

#endif /* FEATURE_V4_SECTOR_PI */
