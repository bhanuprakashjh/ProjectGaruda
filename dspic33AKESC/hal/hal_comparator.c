/**
 * @file hal_comparator.c
 *
 * @brief Comparator/DAC module configuration for BEMF zero-crossing detection.
 * Adapted from AN1292 reference — initializes all 3 CMP modules.
 *
 * Phase 1: Comparators initialized and DACs set to Vbus/2, but ZC detection
 * is not yet wired into commutation (that's Phase 2).
 *
 * Component: COMPARATOR
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "hal_comparator.h"

static void CMP1_Initialize(void);
static void CMP2_Initialize(void);
static void CMP3_Initialize(void);

/**
 * @brief Initialize all comparator modules with DAC calibration.
 */
void InitializeCMPs(void)
{
    uint32_t *FPDMDACaddress;
    uint32_t FPDMDACdata, POSINLADJ, NEGINLADJ, DNLADJ;

    /* DAC Calibration from flash at 0x7F20B0 */
    FPDMDACaddress = (uint32_t *)(0x7F20B0);
    FPDMDACdata = (uint32_t)(*FPDMDACaddress);
    POSINLADJ = (FPDMDACdata & 0x00FF0000) >> 16;
    NEGINLADJ = (FPDMDACdata & 0x0000FF00) >> 8;
    DNLADJ = (FPDMDACdata & 0x000000FF);

    DACCTRL1 = 0;
    DACCTRL1bits.NEGINLADJ = NEGINLADJ;
    DACCTRL1bits.DNLADJ = DNLADJ;
    DACCTRL1bits.POSINLADJ = POSINLADJ;

    DACCTRL1bits.ON = 0;
    DACCTRL1bits.SIDL = 0;
    DACCTRL1bits.FCLKDIV = 0b111;   /* Filter clock divide by 8 */

    DACCTRL2 = 0;
    DACCTRL2bits.TMODTIME = 0;
    DACCTRL2bits.SSTIME = 0;

    /* Initialize all 3 comparators */
    CMP1_Initialize();
    CMP2_Initialize();
    CMP3_Initialize();
}

/**
 * @brief Configure CMP1 for Phase A BEMF zero-crossing.
 * Input: CMP1B = RA4 (INSEL=1), DAC1 reference
 */
static void CMP1_Initialize(void)
{
    DAC1CON = 0;
    DAC1CONbits.DACEN = 0;
    DAC1CONbits.IRQM = 0;          /* Interrupts disabled */
    DAC1CONbits.CBE = 0;
    DAC1CONbits.DACOEN = 0;
    DAC1CONbits.FLTREN = 0;
    DAC1CONbits.CMPSTAT = 0;
    DAC1CONbits.CMPPOL = 0;        /* Non-inverted */
    DAC1CONbits.INSEL = 1;         /* B input: CMP1B = RA4 */
    DAC1CONbits.HYSPOL = 0;
    DAC1CONbits.HYSSEL = 0b01;     /* 15mV hysteresis */
    DAC1CONbits.TMCB = 0;

    DAC1DAT = 0;

    DAC1SLPCON = 0;
    DAC1SLPCONbits.SLOPEN = 0;
    DAC1SLPCONbits.HME = 0;
    DAC1SLPCONbits.TWME = 0;
    DAC1SLPCONbits.PSE = 0;

    DAC1SLPDAT = 0;
}

/**
 * @brief Configure CMP2 for Phase B BEMF zero-crossing.
 * Input: CMP2B = RB2 (INSEL=1), DAC2 reference
 */
static void CMP2_Initialize(void)
{
    DAC2CON = 0;
    DAC2CONbits.DACEN = 0;
    DAC2CONbits.IRQM = 0;
    DAC2CONbits.CBE = 0;
    DAC2CONbits.DACOEN = 0;
    DAC2CONbits.FLTREN = 0;
    DAC2CONbits.CMPSTAT = 0;
    DAC2CONbits.CMPPOL = 0;
    DAC2CONbits.INSEL = 1;         /* B input: CMP2B = RB2 */
    DAC2CONbits.HYSPOL = 0;
    DAC2CONbits.HYSSEL = 0b01;
    DAC2CONbits.TMCB = 0;

    DAC2DAT = 0;

    DAC2SLPCON = 0;
    DAC2SLPCONbits.SLOPEN = 0;
    DAC2SLPCONbits.HME = 0;
    DAC2SLPCONbits.TWME = 0;
    DAC2SLPCONbits.PSE = 0;

    DAC2SLPDAT = 0;
}

/**
 * @brief Configure CMP3 for Phase C BEMF zero-crossing.
 * Input: CMP3B = RB5 (INSEL=1), DAC3 reference
 */
static void CMP3_Initialize(void)
{
    DAC3CON = 0;
    DAC3CONbits.DACEN = 0;
    DAC3CONbits.IRQM = 0;
    DAC3CONbits.CBE = 0;
    DAC3CONbits.DACOEN = 0;
    DAC3CONbits.FLTREN = 0;
    DAC3CONbits.CMPSTAT = 0;
    DAC3CONbits.CMPPOL = 0;
    DAC3CONbits.INSEL = 1;         /* B input: CMP3B = RB5 */
    DAC3CONbits.HYSPOL = 0;
    DAC3CONbits.HYSSEL = 0b01;
    DAC3CONbits.TMCB = 0;

    DAC3DAT = 0;

    DAC3SLPCON = 0;
    DAC3SLPCONbits.HCFSEL = 0;
    DAC3SLPCONbits.SLPSTOPA = 0;
    DAC3SLPCONbits.SLPSTOPB = 0;
    DAC3SLPCONbits.SLPSTRT = 0;
    DAC3SLPCONbits.SLOPEN = 0;
    DAC3SLPCONbits.HME = 0;
    DAC3SLPCONbits.TWME = 0;
    DAC3SLPCONbits.PSE = 0;

    DAC3SLPDAT = 0;
}

/**
 * @brief Enable only the comparator for the current floating phase.
 * Disables the other two to avoid false triggers.
 *
 * @param phase 0=A (CMP1), 1=B (CMP2), 2=C (CMP3)
 */
void HAL_CMP_EnableFloatingPhase(uint8_t phase)
{
    /* Disable all first */
    DAC1CONbits.DACEN = 0;
    DAC2CONbits.DACEN = 0;
    DAC3CONbits.DACEN = 0;

    switch (phase)
    {
        case 0: /* Phase A floating — enable CMP1 */
            DAC1CONbits.DACEN = 1;
            break;
        case 1: /* Phase B floating — enable CMP2 */
            DAC2CONbits.DACEN = 1;
            break;
        case 2: /* Phase C floating — enable CMP3 */
            DAC3CONbits.DACEN = 1;
            break;
    }

    /* Ensure common DAC module is enabled */
    DACCTRL1bits.ON = 1;
}

/**
 * @brief Update the DAC reference value on all 3 comparators.
 * Typically set to Vbus/2 for BEMF zero-crossing detection.
 *
 * @param vbusHalf Half of bus voltage ADC reading, scaled for DAC
 */
void HAL_CMP_SetReference(uint16_t vbusHalf)
{
    DAC1DATbits.DACDAT = vbusHalf;
    DAC2DATbits.DACDAT = vbusHalf;
    DAC3DATbits.DACDAT = vbusHalf;
}
