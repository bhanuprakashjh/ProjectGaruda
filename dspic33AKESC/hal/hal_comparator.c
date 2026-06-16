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

#if 0   /* see InitializeCMPs — CMP1/CMP2 removed (railed the OA1/OA2 op-amps) */
static void CMP1_Initialize(void);
static void CMP2_Initialize(void);
#endif
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

    /* DAC factory calibration (POSINLADJ/NEGINLADJ/DNLADJ) from flash.
     * 2026-06-13: was 0x7F20B0 — WRONG, loaded garbage INL/DNL trim into the
     * DAC, leaving the CMP3 reference corrupt (bench: the internal-bandgap CMP3
     * self-test never asserted CMPSTAT, proving the comparator CORE was dead,
     * not the input mux). Microchip AN957 on this exact MC510 reads 0x7F20E0. */
    FPDMDACaddress = (uint32_t *)(0x7F20E0);
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

    /* Initialize comparators */
#if 0   /* 2026-06-10: CMP1/CMP2 init REMOVED — root cause of the railed phase-
         * current op-amps in the 6-step build. These are DEAD CODE (file header:
         * comparators are NOT used for BEMF on this DIM; HAL_CMP_EnableFloating-
         * Phase has zero call sites), but their input muxes sit on CMP1B=RA4=
         * OA1IN+ and CMP2B=RB2=OA2IN+. Initializing them ONLY in the 6-step
         * build correlated exactly with OA1/OA2 resting at opposite rails
         * (4085/84) while the FOC build — which skips them citing "pin conflict"
         * — reads the same op-amps at ~2048. CMP3 (overcurrent) is untouched. */
    CMP1_Initialize();
    CMP2_Initialize();
#endif
#if FEATURE_HW_OVERCURRENT
    CMP3_InitOvercurrent();
#else
    CMP3_Initialize();
#endif
}

#if 0   /* CMP1/CMP2 disabled — see InitializeCMPs */
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
#endif  /* CMP1/CMP2 disabled */

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
#if GARUDA_TARGET_AK512
    /* MC510: comparator-config fields moved from DACxCON to DACxCMP
     * (INSEL renamed INPSEL). Zero DAC3CMP to mirror the DAC3CON=0 reset
     * of these fields on AK128. */
    DAC3CON = 0;
    DAC3CMP = 0;
    DAC3CMPbits.FLTREN = OC_CMP_FILTER_EN;
    DAC3CMPbits.CMPPOL = 0;         /* Non-inverted: HIGH when OA3OUT > DAC */
    /* ROOT CAUSE of the dead chop (2026-06-13): on MC510 the OA3 output reaches
     * CMP3 on input D (INPSEL=3), NOT input A. INPSEL=0 watched the wrong node
     * -> CMP3 never tripped -> no chop, and the on-chip OC fault never worked
     * (hence the board-U25B fallback). Authority: Microchip AN957 on this exact
     * MCLV-48V-300W + MC510 DIM (internal op-amp) sets INPSEL=3 for the DC-bus
     * OC comparator. Ibus telemetry stayed correct because the ADC reads OA3 on
     * its own channel (AD3CH1), independent of the comparator input mux. */
    DAC3CMPbits.INPSEL = 3;         /* CMP_D = OA3 output -> CMP3 (AN957) */
    DAC3CMPbits.HYSPOL = 0;         /* Hysteresis on rising edge */
    DAC3CMPbits.HYSSEL = OC_CMP_HYSTERESIS;
#if FEATURE_IBUS_PROBE
    /* PROBE diagnostic: latch every rising edge of the comparator output in the
     * _CMP3IF interrupt flag (IEC stays 0 -> no ISR, just the latch). The ADC
     * ISR polls/clears it, so we see CMP3 fire even though the ADC samples at
     * freewheel-center where the bus current is ~0. Isolates "comparator sees
     * the current" from "CLPCI chops the output". */
    DAC3CONbits.IRQM = 0b01;        /* event/IF-latch on rising edge */
#endif
#else
    DAC3CON = 0;
    DAC3CONbits.FLTREN = OC_CMP_FILTER_EN;
    DAC3CONbits.CMPPOL = 0;         /* Non-inverted: HIGH when OA3OUT > DAC */
    DAC3CONbits.INSEL = 0;          /* CMP3A = RA5 = OA3OUT */
    DAC3CONbits.HYSPOL = 0;         /* Hysteresis on rising edge */
    DAC3CONbits.HYSSEL = OC_CMP_HYSTERESIS;
#endif

    /* All modes start with elevated startup threshold. On low-R motors
     * (A2212: 0.065 ohm), stall current during align/ramp easily exceeds
     * the operational threshold, causing CLPCI to chop away startup torque.
     * Lowered to OC_CMP3_DAC_VAL after ZC sync via HAL_CMP3_SetThreshold(). */
#if AK512_CHOP_ALWAYS_TEST
    DAC3DATbits.DACDAT = 40;   /* BRINGUP DIAG: below OA3 rest -> always tripped */
#else
    DAC3DATbits.DACDAT = OC_CMP3_STARTUP_DAC;
#endif

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
 * @param dacVal 12-bit DAC value (0-4095), in the 2048-bias frame
 */
void HAL_CMP3_SetThreshold(uint16_t dacVal)
{
#if AK512_CHOP_ALWAYS_TEST
    /* BRINGUP DIAG: force trip below OA3 rest (~78) so CMP3 is always tripped.
     * If CLPCI is wired the motor cannot spin. STRIP before merge. */
    (void)dacVal;
    DAC3DATbits.DACDAT = 40;
    return;
#endif
#if FEATURE_OC_AUTOZERO
    /* This board's OA3 output rests at ~g_ocBiasAdc (~78 counts), NOT the 2048
     * mid-rail the OC_TRIP_MV math assumes (AN957's board is biased at 2048; this
     * one isn't — that's why ibus ADC rests near 0). So the comparator input is
     * in the 78-frame and the 2048-frame threshold MUST be shifted down by
     * (2048 - bias) to land on the real signal. 2026-06-13: removing this shift
     * (mistaken 2048-frame assumption) put the threshold ~1970 counts too high so
     * CMP3 never tripped — the "cap" seen was just the structural CL-idle current. */
    extern volatile uint16_t g_ocBiasAdc;
    int32_t v = (int32_t)dacVal - (2048 - (int32_t)g_ocBiasAdc);
    if (v < 1)    v = 1;
    if (v > 4095) v = 4095;
    DAC3DATbits.DACDAT = (uint16_t)v;
#else
    DAC3DATbits.DACDAT = dacVal;
#endif
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
#if GARUDA_TARGET_AK512
        case 0: return DAC1CMPbits.CMPSTAT;
        case 1: return DAC2CMPbits.CMPSTAT;
        case 2: return DAC3CMPbits.CMPSTAT;
#else
        case 0: return DAC1CONbits.CMPSTAT;
        case 1: return DAC2CONbits.CMPSTAT;
        case 2: return DAC3CONbits.CMPSTAT;
#endif
        default: return 0;
    }
}
