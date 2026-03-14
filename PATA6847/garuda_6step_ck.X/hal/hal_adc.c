/**
 * @file hal_adc.c
 * @brief ADC initialization for dsPIC33CK on EV43F54A.
 *
 * Register values from EV43F54A reference firmware (verified working).
 *
 * ADC trigger: PWM1 Trigger1 (at center) for currents (AN0,AN1,AN4),
 *              PWM1 Trigger2 for Vbus (AN9) and pot (AN6).
 *
 * Dedicated cores: Core0=AN0, Core1=AN1.
 * Shared core: AN4, AN6, AN9.
 * 12-bit resolution, fractional format.
 */

#include <xc.h>
#include "hal_adc.h"

static void ADC_Core0PowerEnable(void)
{
    ADCON5Lbits.C0PWR = 1;
    while (ADCON5Lbits.C0RDY == 0);
    ADCON3Hbits.C0EN = 1;
}

static void ADC_Core1PowerEnable(void)
{
    ADCON5Lbits.C1PWR = 1;
    while (ADCON5Lbits.C1RDY == 0);
    ADCON3Hbits.C1EN = 1;
}

static void ADC_SharedCorePowerEnable(void)
{
    ADCON5Lbits.SHRPWR = 1;
    while (ADCON5Lbits.SHRRDY == 0);
    ADCON3Hbits.SHREN = 1;
}

void HAL_ADC_Init(void)
{
    ADCON1L = (0x8000 & 0x7FFF);  /* ADON disabled initially */

    ADCON1H = 0x00E0;    /* FORM=Fractional, SHRRES=12-bit */
    ADCON2L = 0x0002;    /* SHRADCS=2 */
    ADCON2H = 0x0010;
    ADCON3L = 0x0000;
    ADCON3H = (0x0083 & 0xFF00);  /* Disable C0EN,C1EN,SHREN initially */

    ADCON4L = 0x0003;
    ADCON4H = 0x0000;    /* C0CHS=AN0, C1CHS=AN1 */

    /* Signed mode for current channels (AN0,AN1,AN4 only).
     * AN6 (pot) and AN9 (Vbus) must be unsigned. */
    ADMOD0L = 0x0505;
    ADMOD0H = 0x0000;
    ADMOD1L = 0x0000;
    ADMOD1H = 0x0000;

    /* Interrupt enable: AN9 only (last to complete — shared core, Trig2).
     * When AN9 ISR fires, all earlier channels (AN0,AN1,AN4,AN6) are ready.
     * Reading their buffers in the ISR clears data-ready flags.
     * Original ref had 0x0073/0x0300 but ISR must read ALL enabled channels
     * or the common ADCIF re-asserts immediately → infinite ISR loop. */
    ADIEL = 0x0200;   /* AN9 only */
    ADIEH = 0x0000;

    /* All comparators disabled */
    ADCMP0ENL = ADCMP1ENL = ADCMP2ENL = ADCMP3ENL = 0x00;
    ADCMP0ENH = ADCMP1ENH = ADCMP2ENH = ADCMP3ENH = 0x00;
    ADCMP0LO = ADCMP1LO = ADCMP2LO = ADCMP3LO = 0x00;
    ADCMP0HI = ADCMP1HI = ADCMP2HI = ADCMP3HI = 0x00;
    ADCMP0CON = ADCMP1CON = ADCMP2CON = ADCMP3CON = 0x00;
    ADFL0CON = ADFL1CON = ADFL2CON = ADFL3CON = 0x0400;
    ADLVLTRGL = ADLVLTRGH = 0x00;

    /* Core configuration: 12-bit, ADCS=2 */
    ADCORE0L = 0x0000;
    ADCORE0H = 0x0300;
    ADCORE1L = 0x0000;
    ADCORE1H = 0x0300;

    ADEIEL = ADEIEH = 0x00;

    /* Warmtime */
    ADCON5H = (0x0F00 & 0xF0FF);
    ADCON5Hbits.WARMTIME = 0x0F;

    /* Enable ADC module */
    ADCON1Lbits.ADON = 1;

    /* Power up cores */
    ADC_SharedCorePowerEnable();
    ADC_Core0PowerEnable();
    ADC_Core1PowerEnable();

    /* Trigger sources:
     * AN0 → no trigger (sampled by Core0 on PWM trigger)
     * AN1 → PWM1 Trigger1
     * AN4 → PWM1 Trigger1
     * AN6 → PWM1 Trigger2
     * AN9 → PWM1 Trigger2 */
    ADTRIG0L = 0x0400;   /* TRGSRC1=PWM1 Trig1 */
    ADTRIG0H = 0x0000;
    ADTRIG1L = 0x0004;   /* TRGSRC4=PWM1 Trig1 */
    ADTRIG1H = 0x0005;   /* TRGSRC6=PWM1 Trig2 */
    ADTRIG2L = 0x0500;   /* TRGSRC9=PWM1 Trig2 */
    ADTRIG2H = 0x0000;
    ADTRIG3L = ADTRIG3H = 0x0000;
    ADTRIG4L = ADTRIG4H = 0x0000;

    /* ADC interrupt: priority 3 (below Timer1 priority 5) */
    IPC22bits.ADCIP = 3;
    IFS5bits.ADCIF = 0;
    IEC5bits.ADCIE = 0;

    /* Keep ADC ON — pot/Vbus readable during IDLE via software trigger.
     * ISR is disabled until motor starts (no overhead). */
}

void HAL_ADC_Enable(void)
{
    ADCON1Lbits.ADON = 1;
}

void HAL_ADC_Disable(void)
{
    ADCON1Lbits.ADON = 0;
}

void HAL_ADC_InterruptEnable(void)
{
    IFS5bits.ADCIF = 0;
    IEC5bits.ADCIE = 1;
}

void HAL_ADC_InterruptDisable(void)
{
    IEC5bits.ADCIE = 0;
}

/**
 * @brief Software-triggered read of pot (AN6) and Vbus (AN9).
 * Used during IDLE when PWM triggers aren't running.
 * Call from main loop, NOT from ISR.
 */
void HAL_ADC_PollPotVbus(uint16_t *pot, uint16_t *vbus)
{
    /* AN6 (pot) — software individual channel trigger */
    ADCON3Lbits.CNVCHSEL = 6;
    ADCON3Lbits.CNVRTCH = 1;
    while (!ADSTATLbits.AN6RDY);
    *pot = (uint16_t)ADCBUF6;

    /* AN9 (Vbus) */
    ADCON3Lbits.CNVCHSEL = 9;
    ADCON3Lbits.CNVRTCH = 1;
    while (!ADSTATLbits.AN9RDY);
    *vbus = (uint16_t)ADCBUF9;
}
