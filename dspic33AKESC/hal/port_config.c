/**
 * @file port_config.c
 *
 * @brief GPIO pin initialization for Project Garuda ESC.
 * Adapted from AN1292 reference:
 *   - Keeps: PWM outputs, LEDs, buttons, UART PPS, PCI8 fault
 *   - Removes: Op-amp configuration (OA1/OA2/OA3 not used)
 *   - Adds: Phase voltage ADC pins (RB9, RB8, RA10), DShot IC1 pin (RD8)
 *
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: PORTS
 */

#include <xc.h>

#include "port_config.h"

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
     * Phase Voltage Feedback (MCLV-48V-300W via EV68M17A DIM)
     * M1_VA : DIM:009 - RB9  (AD2AN10)
     * M1_VB : DIM:011 - RB8  (AD1AN11)
     * M1_VC : DIM:022 - RA10 (AD2AN7)
     * ================================================================ */
    ANSELBbits.ANSELB9 = 1;
    TRISBbits.TRISB9 = 1;

    ANSELBbits.ANSELB8 = 1;
    TRISBbits.TRISB8 = 1;

    ANSELAbits.ANSELA10 = 1;
    TRISAbits.TRISA10 = 1;

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
     * PWM Fault PCI8 input
     * DIM:040 - Pin #32 : RP28/SDI2/RB11
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
}
