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
}

void HAL_Capture_Start(void)
{
    /* Flush FIFOs */
    (void)CCP2BUFL; (void)CCP2BUFL; (void)CCP2BUFL; (void)CCP2BUFL;
    (void)CCP5BUFL; (void)CCP5BUFL; (void)CCP5BUFL; (void)CCP5BUFL;
    v4_captureValid = false;
    _CCP2IF = 0; _CCP5IF = 0;
    _CCP2IE = 1; _CCP5IE = 1;   /* ISR drains FIFO continuously */
}

void HAL_Capture_Stop(void)
{
    _CCP2IE = 0; _CCP5IE = 0;
    _CCP2IF = 0; _CCP5IF = 0;
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
    /* Blanking: ignore captures in first 40% of sector.
     * Demagnetization on the floating phase lasts ~30-35% of
     * the sector. True ZC is at ~50%. 40% blanking rejects
     * all demagnetization noise while keeping the ZC window. */
    blankingEndHR = (uint16_t)(commTimeHR + ((uint32_t)sectorPeriodHR * 40 / 100));
}

bool HAL_Capture_IsRisingZc(void) { return currentRisingZc; }
uint16_t HAL_Capture_GetCcp2Offset(void) { return ccp2HrOffset; }
uint16_t HAL_Capture_GetCcp5Offset(void) { return ccp5HrOffset; }
uint16_t HAL_Capture_GetBlankingEnd(void) { return blankingEndHR; }

#endif /* FEATURE_V4_SECTOR_PI */
