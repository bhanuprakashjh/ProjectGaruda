/**
 * @file hal_comparator.c
 *
 * @brief Comparator/DAC module configuration.
 *
 * NOTE: NOT used for BEMF ZC detection — phase voltage pins are not routable
 * to CMP inputs on MCLV-48V-300W DIM. Retained for future overcurrent PCI
 * fault use (CMP3). InitializeCMPs() is not called at startup; see
 * board_service.c HAL_InitPeripherals() for re-enable instructions.
 *
 * Component: COMPARATOR
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "hal_comparator.h"
#include "../garuda_config.h"
#if FEATURE_HW_OVERCURRENT
#include "../garuda_calc_params.h"
#endif

static void CMP1_Initialize(void);
static void CMP2_Initialize(void);
#if FEATURE_HW_OVERCURRENT
static void CMP3_InitOvercurrent(void);
#else
static void CMP3_Initialize(void);
#endif

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
#if FEATURE_HW_OVERCURRENT
    CMP3_InitOvercurrent();
#else
    CMP3_Initialize();
#endif
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

#if !FEATURE_HW_OVERCURRENT
/**
 * @brief Configure CMP3 for Phase C BEMF zero-crossing.
 * Input: CMP3B = RB5 (INSEL=1), DAC3 reference
 * Not used when FEATURE_HW_OVERCURRENT — CMP3_InitOvercurrent() replaces it.
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
#endif /* !FEATURE_HW_OVERCURRENT */

#if FEATURE_HW_OVERCURRENT
/**
 * @brief Configure CMP3 for bus overcurrent detection via OA3.
 * Input: CMP3A = RA5 = OA3OUT (INSEL=0).
 *
 * All modes start with elevated startup threshold (OC_CMP3_STARTUP_DAC).
 * On low-R motors (A2212: 0.065 ohm), stall current during align/ramp
 * easily exceeds the operational threshold, causing CLPCI chopping that
 * robs the motor of startup torque.
 * Lowered to OC_CMP3_DAC_VAL after ZC sync via HAL_CMP3_SetThreshold().
 *
 * Called from InitializeCMPs() after DAC factory calibration is applied.
 */
static void CMP3_InitOvercurrent(void)
{
    DAC3CON = 0;
    DAC3CONbits.FLTREN = OC_CMP_FILTER_EN;
    DAC3CONbits.CMPPOL = 0;         /* Non-inverted: HIGH when OA3OUT > DAC */
    DAC3CONbits.INSEL = 0;          /* CMP3A = RA5 = OA3OUT */
    DAC3CONbits.HYSPOL = 0;         /* Hysteresis on rising edge */
    DAC3CONbits.HYSSEL = OC_CMP_HYSTERESIS;

    /* All modes start with elevated startup threshold. On low-R motors
     * (A2212: 0.065 ohm), stall current during align/ramp easily exceeds
     * the operational threshold, causing CLPCI to chop away startup torque.
     * Lowered to OC_CMP3_DAC_VAL after ZC sync via HAL_CMP3_SetThreshold(). */
    DAC3DATbits.DACDAT = OC_CMP3_STARTUP_DAC;

    DAC3SLPCON = 0;
    DAC3SLPDAT = 0;
}

/**
 * @brief Enable CMP3 and DAC module for overcurrent protection.
 * Call after CMP3_InitOvercurrent() has configured the threshold.
 */
void HAL_CMP3_EnableOvercurrent(void)
{
    DACCTRL1bits.ON = 1;
    DAC3CONbits.DACEN = 1;
}

/**
 * @brief Update CMP3/DAC3 threshold at runtime.
 * @param dacVal 12-bit DAC value (0-4095)
 */
void HAL_CMP3_SetThreshold(uint16_t dacVal)
{
    DAC3DATbits.DACDAT = dacVal;
}
#endif /* FEATURE_HW_OVERCURRENT */

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

/**
 * @brief Read comparator output status for given phase.
 * @param phase 0=A (CMP1), 1=B (CMP2), 2=C (CMP3)
 * @return CMPSTAT bit: 1 if BEMF > DAC ref, 0 if BEMF < DAC ref
 */
uint8_t HAL_CMP_ReadStatus(uint8_t phase)
{
    switch (phase)
    {
        case 0: return DAC1CONbits.CMPSTAT;
        case 1: return DAC2CONbits.CMPSTAT;
        case 2: return DAC3CONbits.CMPSTAT;
        default: return 0;
    }
}
