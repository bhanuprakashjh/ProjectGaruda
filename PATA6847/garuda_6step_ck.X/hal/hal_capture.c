/**
 * @file hal_capture.c
 * @brief V4 Capture HAL — CCP2/CCP5 with CPU ISR, last-edge-wins.
 *
 * V3-proven config: CCP2 (falling edge) + CCP5 (rising edge),
 * both always on, CCPON never toggled, PPS remapped at commutation.
 * CPU ISRs drain FIFOs and store last edge. No DMA.
 */

#include "hal_capture.h"

#if FEATURE_V4_SECTOR_PI

#include <xc.h>
#include "../motor/v4_params.h"

/* Shared state */
volatile uint16_t v4_lastCaptureHR = 0;
volatile bool     v4_captureValid  = false;

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
    CCP2CON1Lbits.CCPON = 1;           /* Start — NEVER toggled */

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
    CCP5CON1Lbits.CCPON = 1;

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
     * Fp/4 = 25 MHz → 40 ns/tick, period 625 = 25 µs = PWM period.
     * Counter seeded at half-period in HAL_Capture_Start to align. */
    CCP1CON1L = 0; CCP1CON1H = 0; CCP1CON2L = 0; CCP1CON2H = 0;
    CCP1CON1Lbits.CCSEL  = 0;          /* Timer mode */
    CCP1CON1Lbits.T32    = 0;          /* 16-bit */
    CCP1CON1Lbits.CLKSEL = 0b000;      /* Fp = 100 MHz */
    CCP1CON1Lbits.TMRPS  = 0b10;       /* 1:4 → 25 MHz, 40 ns/tick */
    CCP1CON1Lbits.MOD    = 0b0000;     /* Auto-reload on period match */
    CCP1PRL = (FCY / 4 / PWMFREQUENCY_HZ) - 2;  /* 623 = 24.96 µs — deliberate drift vs PWM */
    _CCT1IP = V5_SCCP1_ISR_PRIORITY;    /* V4 baseline 2; V5 bumps to 5 */
    _CCT1IF = 0;
    _CCT1IE = 0;                        /* Enabled at motor start */
    _CCP1IF = 0; _CCP1IE = 0;
    CCP1CON1Lbits.CCPON = 1;           /* Start free-running */
}

void HAL_Capture_Start(void)
{
    /* Flush FIFOs */
    (void)CCP2BUFL; (void)CCP2BUFL; (void)CCP2BUFL; (void)CCP2BUFL;
    (void)CCP5BUFL; (void)CCP5BUFL; (void)CCP5BUFL; (void)CCP5BUFL;
    v4_captureValid = false;
    _CCP2IF = 0; _CCP5IF = 0;
    _CCP2IE = 1; _CCP5IE = 1;   /* ISR drains FIFO continuously.
                                 * Phase C tried disabling these in
                                 * non-SP — regressed to ~100k trip.
                                 * Inline drain at PWM center isn't
                                 * equivalent to edge-triggered preempt. */

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

#endif /* FEATURE_V4_SECTOR_PI */
