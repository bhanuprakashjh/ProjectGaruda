/**
 * @file port_config.c
 *
 * @brief GPIO pin initialization for Project Garuda ESC.
 * Adapted from AN1292 reference:
 *   - PWM outputs, LEDs, buttons, UART PPS, PCI8 fault
 *   - OA1/OA2/OA3 pins configured as analog inputs for ADC (both int/ext modes)
 *   - HAL_OA3_Init() enables internal op-amps (called only when USE_INTERNAL_OPAMP=1)
 *   - DShot IC1 pin (RD8)
 *
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: PORTS
 */

#include <xc.h>

#include "port_config.h"
#include "../garuda_config.h"

/**
 * @brief Initialize all ports as input/digital, then map hardware functions.
 */
void SetupGPIOPorts(void)
{
    /* Reset all PORTx register (all inputs) */

    #ifdef TRISA
        TRISA = 0xFFFF;
        LATA  = 0x0000;
    #endif
    #ifdef ANSELA
        ANSELA = 0x0000;
    #endif

    #ifdef TRISB
        TRISB = 0xFFFF;
        LATB  = 0x0000;
    #endif
    #ifdef ANSELB
        ANSELB = 0x0000;
    #endif

    #ifdef TRISC
        TRISC = 0xFFFF;
        LATC  = 0x0000;
    #endif
    #ifdef ANSELC
        ANSELC = 0x0000;
    #endif

    #ifdef TRISD
        TRISD = 0xFFFF;
        LATD  = 0x0000;
    #endif
    #ifdef ANSELD
        ANSELD = 0x0000;
    #endif

    MapGPIOHWFunction();
}

/**
 * @brief Map port pins as input/output, analog/digital, and PPS assignments.
 */
void MapGPIOHWFunction(void)
{
    /* ================================================================
     * OA1 — Phase A current sensing (FOC 3-shunt)
     * OA1IN- : RA3 (M1_SHUNT_IA_N)
     * OA1IN+ : RA4 (M1_SHUNT_IA_P)
     * OA1OUT : RA2 → AD1AN0 (PINSEL=0 on AD1CH0)
     * ================================================================ */
    ANSELAbits.ANSELA3 = 1;  TRISAbits.TRISA3 = 1;  /* OA1IN- */
    ANSELAbits.ANSELA4 = 1;  TRISAbits.TRISA4 = 1;  /* OA1IN+ */
    ANSELAbits.ANSELA2 = 1;  TRISAbits.TRISA2 = 1;  /* OA1OUT → ADC input */

    /* ================================================================
     * OA2 — Phase B current sensing (FOC 3-shunt)
     * OA2IN- : RB1 (M1_SHUNT_IB_N)
     * OA2IN+ : RB2 (M1_SHUNT_IB_P)
     * OA2OUT : RB0 → AD2AN1 (PINSEL=1 on AD2CH0)
     * ================================================================ */
    ANSELBbits.ANSELB1 = 1;  TRISBbits.TRISB1 = 1;  /* OA2IN- */
    ANSELBbits.ANSELB2 = 1;  TRISBbits.TRISB2 = 1;  /* OA2IN+ */
    ANSELBbits.ANSELB0 = 1;  TRISBbits.TRISB0 = 1;  /* OA2OUT → ADC input */

    /* ================================================================
     * Potentiometer input (POT1) - used as Speed Reference
     * DIM:028 - PIN #06: AD1AN10/RP12/RA11
     * ================================================================ */
    ANSELAbits.ANSELA11 = 1;
    TRISAbits.TRISA11 = 1;

    /* ================================================================
     * DC Bus Voltage (VBUS)
     * DIM:039 - PIN #02: AD1AN6/RP8/IOMF1/RA7
     * ================================================================ */
    ANSELAbits.ANSELA7 = 1;
    TRISAbits.TRISA7 = 1;

    /* ================================================================
     * Inverter Control - PWM Outputs
     * PWM1L : DIM:003 - PIN #54  TDI/RP52/PWM1L/IOMD5/RD3
     * PWM1H : DIM:001 - PIN #53  TDO/RP51/PWM1H/IOMD4/RD2
     * PWM2L : DIM:007 - PIN #52  TCK/RP50/PWM2L/IOMD3/RD1
     * PWM2H : DIM:005 - PIN #51  RP49/PWM2H/IOMD2/RD0
     * PWM3L : DIM:004 - PIN #44  RP37/PWM3L/IOMD1/RC4
     * PWM3H : DIM:002 - PIN #43  PGD3/RP36/PWM3H/IOMD0/RC3
     * ================================================================ */
    TRISDbits.TRISD3 = 0;
    TRISDbits.TRISD2 = 0;
    TRISDbits.TRISD1 = 0;
    TRISDbits.TRISD0 = 0;
    TRISCbits.TRISC4 = 0;
    TRISCbits.TRISC3 = 0;

    /* ================================================================
     * Debug LEDs
     * LED1 : DIM:030 - PIN #55 : RP54/ASCL1/RD5
     * LED2 : DIM:032 - PIN #34 : RP42/IOMD10/SDO2/IOMF10/PCI19/RC9
     * ================================================================ */
    TRISDbits.TRISD5 = 0;
    TRISCbits.TRISC9 = 0;

    /* ================================================================
     * Push button Switches
     * SW1 : DIM:034 - PIN #49 : RP58/IOMF7/RD9
     * SW2 : DIM:036 - PIN #50 : RP59/RD10
     * ================================================================ */
    TRISDbits.TRISD9 = 1;
    TRISDbits.TRISD10 = 1;

    /* ================================================================
     * PWM Fault PCI8 input — board OC+OV fault (M1_FAULT_OC_OV)
     * DIM:040 - Pin #32 : RP28/SDI2/RB11
     * TRISB11 = 1 (input) from TRISB=0xFFFF init above.
     * ANSELB11 = 0 (digital) from ANSELB=0x0000 init above.
     * ================================================================ */
    _PCI8R = 28;

    /* ================================================================
     * UART PPS mapping
     * UART_RX : DIM:054 - PIN #46 : RP44/IOMD8/IOMF8/RC11 (Input)
     * UART_TX : DIM:052 - PIN #45 : RP43/IOMD9/IOMF9/RC10 (Output)
     * ================================================================ */
    _U1RXR = 44;
    _RP43R = 9;

    /* ================================================================
     * DShot Input Capture (IC1) pin
     * RD8 = RP57 → IC1 PPS mapping
     * ================================================================ */
    TRISDbits.TRISD8 = 1;      /* Input */
    _ICM1R = 57;                /* Map RP57 to IC1 input */

    /* ================================================================
     * OA3 — Phase C current sensing (FOC 3-shunt)
     * OA3IN+ : DIM:029 - RB5  (M1_SHUNT_IC_P)
     * OA3IN- : DIM:031 - RA6  (M1_SHUNT_IC_N)
     * OA3OUT : RA5 → AD1AN3 (PINSEL=3 on AD1CH2)
     * (Previously used for bus current OC — now phase C current)
     * ================================================================ */
    ANSELBbits.ANSELB5 = 1;  TRISBbits.TRISB5 = 1;   /* OA3IN+ = RB5 */
    ANSELAbits.ANSELA6 = 1;  TRISAbits.TRISA6 = 1;    /* OA3IN- = RA6 */
    ANSELAbits.ANSELA5 = 1;  TRISAbits.TRISA5 = 1;    /* OA3OUT = RA5 (input to ADC) */
}

/**
 * @brief Initialize OA1, OA2, OA3 internal op-amps for 3-phase current sensing.
 * OA1: Phase A current (RA2 output → AD1CH0, PINSEL=0)
 * OA2: Phase B current (RB0 output → AD2CH0, PINSEL=1)
 * OA3: Phase C current (RA5 output → AD1CH2, PINSEL=3)
 * External gain resistors on MCLV-48V-300W DIM provide gain ×8.
 * Only called when USE_INTERNAL_OPAMP=1 (internal op-amp mode).
 * When USE_INTERNAL_OPAMP=0, board's external op-amps feed the same ADC pins.
 */
void HAL_OA3_Init(void)
{
    /* OA1 — Phase A current */
    AMP1CON1 = 0x0000;
    AMP1CON1bits.HPEN = 1;       /* High-power mode (high bandwidth) */
    AMP1CON1bits.UGE = 0;        /* External resistor gain (not unity) */
    AMP1CON1bits.DIFFCON = 0;    /* Both differential pairs active */
    AMP1CON1bits.OMONEN = 1;     /* Internal ADC connection enabled */
    AMP1CON1bits.AMPEN = 1;      /* Enable op-amp — begins settling (~10us) */

    /* OA2 — Phase B current */
    AMP2CON1 = 0x0000;
    AMP2CON1bits.HPEN = 1;
    AMP2CON1bits.UGE = 0;
    AMP2CON1bits.DIFFCON = 0;
    AMP2CON1bits.OMONEN = 1;
    AMP2CON1bits.AMPEN = 1;

    /* OA3 — Phase C current */
    AMP3CON1 = 0x0000;
    AMP3CON1bits.HPEN = 1;
    AMP3CON1bits.UGE = 0;
    AMP3CON1bits.DIFFCON = 0;
    AMP3CON1bits.OMONEN = 1;
    AMP3CON1bits.AMPEN = 1;
}
