// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file clock.c
 *
 * @brief This module initializes the clock/oscillator module
 * 
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: CLOCK/OSCILLATOR
 *
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">

/*******************************************************************************
* SOFTWARE LICENSE AGREEMENT
* 
* © [2024] Microchip Technology Inc. and its subsidiaries
* 
* Subject to your compliance with these terms, you may use this Microchip 
* software and any derivatives exclusively with Microchip products. 
* You are responsible for complying with third party license terms applicable to
* your use of third party software (including open source software) that may 
* accompany this Microchip software.
* 
* Redistribution of this Microchip software in source or binary form is allowed 
* and must include the above terms of use and the following disclaimer with the
* distribution and accompanying materials.
* 
* SOFTWARE IS "AS IS." NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
* APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,
* MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL 
* MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR 
* CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO
* THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY
* LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL
* NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR THIS
* SOFTWARE
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <xc.h>
#include <stdint.h>

#include "clock.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
/**
* <B> Function: InitOscillator() </B>
*
* @brief Function to initialize the oscillator/clock module.
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitOscillator(); </CODE>
*
*/
void InitOscillator (void)
{
    /** System Clock Control Register */
    OSCCTRL = 0x0000;
    /* Bit 0 = FRC_EN 8 MHz FRC Clock Enable bit
     * 1 Enable FRC Oscillator
       0 Disable FRC Oscillator */
    OSCCTRLbits.FRCEN = 1;
    /* Bit 2 = POSC_EN Primary Crystal Clock Enable bit 
       1 Enable Primary Crystal/Resonator Oscillator
       0 Disable Primary Crystal/Resonator Oscillator  */
    OSCCTRLbits.POSCEN = 0;
 
    /** OSCCFG : Oscillator Configuration Register */
    OSCCFG = 0;
    /** Bit 0 = POSCMD0 Primary Oscillator Configuration bit(1)
        1 Oscillator mode selected (3.5 MHz-32 MHz)
        0 External clock mode selected */
    OSCCFGbits.POSCMD = 0;
    /** Bit 5 = POSCIOFNC Primary CLKO Enable Configuration bit
        1 CLKO output signal active on the OSC2 pin; 
        0 CLKO output disabled */
    OSCCFGbits.POSCIOFNC = 0;
       
/* In this device Internal RC Oscillator is 8MHz
     * Also,In all Motor Control Development boards primary oscillator or 
     * Crystal oscillator output frequency is  8MHz
     * Hence, FPLLI (PLL Input frequency)is 8MHz in the application
     * 
     * FOSC (Oscillator output frequency),FCY (Device Operating Frequency),
     * FVCO (VCO Output Frequency )is:
     *         ( FPLLI * M)     (8 * 100)           
     * FVCO = -------------- = -----------  = 800 MHz
     *               N1             1    
     *
     *         (FPLLI * M)         (8 * 100)          
     * FPLL = --------------  = -----------    = 200 MHz
     *        (N1 * N2 * N3)     (1 * 4 * 1)     
     *
     * Fsys  = 200 MHz 
     *
     * where,
     * N1 = PLL1DIVbits.REFDIV = 1 
     * N2 = PLL1DIVbits.POSTDIV1 = 4
     * N3 = PLL1DIVbits.POSTDIV2 = 1 
     * M = PLL1DIVbits.FBDIV = 100
     */
    
    /* PLL Feedback Divider bits (also denoted as 'M', PLL multiplier)
     * M = (PLLFBDbits.PLLFBDIV)= 150     */
      PLL1DIVbits.PLLFBDIV = 100;

    /* PLL Phase Detector I/P Divider Select bits(denoted as 'N1',PLL pre-scaler)
     * N1 = PLL1DIVbits.REFDIV = 1        */
      PLL1DIVbits.PLLPRE = 1;

    /* PLL Output Divider #1 Ratio bits((denoted as 'N2' or POSTDIV#1)
     * N2 = PLL1DIVbits.POSTDIV1 = 3      */
    PLL1DIVbits.POSTDIV1 = 4;
    
    /* PLL Output Divider #2 Ratio bits((denoted as 'N3' or POSTDIV#2)
     * N3 = PLL1DIVbits.POSTDIV2 = 1      */
    PLL1DIVbits.POSTDIV2 = 1;
    /* Bit 6 = PLL1_EN PLL1 Enable bit
      1 Enable PLL1
      0 Disable PLL1 */
    OSCCTRLbits.PLL1EN = 1;
    PLL1CONbits.NOSC = 1;
    PLL1CONbits.ON = 1;
    PLL1CONbits.OE = 0;
    PLL1CONbits.OSWEN = 1;
    VCO1DIV = 1;
    while (PLL1CONbits.OSWEN);
    
    PLL1CONbits.PLLSWEN = 1;
    while (PLL1CONbits.PLLSWEN == 1);
    PLL1CONbits.FOUTSWEN = 1;
    while (PLL1CONbits.PLLSWEN == 1);
    while(PLL1CONbits.CLKRDY == 0);
    
    /* Configuring Clock for Individual Peripherals*/
    
    /** System Clock  
    * Input Clock Selection (NOSC) = 5 :PLL1 FOUT output = 200 MHz
    * Clock Division (INTDIV) = (0*2) - No Division
    * Final Clock for System = 200 MHz */
    CLK1DIVbits.INTDIV = 0;
    CLK1CONbits.OE = 1;
    CLK1CONbits.ON = 1;
    CLK1CONbits.NOSC = 5; 
    CLK1CONbits.OSWEN = 1; 
    while (CLK1CONbits.OSWEN);
    CLK1CONbits.DIVSWEN =1;
    while (CLK1CONbits.DIVSWEN);
    
    /** Clock used for PWM  
    * Input Clock Selection (NOSC) = 7 :PLL1 VCO DIV output = 800 MHz
    * Clock Division (INTDIV) = (1*2)
    * Final Clock for PWM = 400 MHz */
    CLK5DIVbits.INTDIV = 1;
    CLK5CONbits.OE = 1;
    CLK5CONbits.ON = 1;
    CLK5CONbits.NOSC = 7;
    CLK5CONbits.OSWEN = 1;
    while (CLK5CONbits.OSWEN);
    CLK5CONbits.DIVSWEN =1;
    while (CLK5CONbits.DIVSWEN);
    
    /** Clock used for ADC  
    * Input Clock Selection (NOSC) = 5 :PLL1 FOUT output = 200 MHz
    * Clock Division (INTDIV) = (1*2)
    * Final Clock for ADC = 100 MHz */
    CLK6DIVbits.INTDIV = 1;
    CLK6CONbits.OE = 1;
    CLK6CONbits.ON = 1;
    CLK6CONbits.NOSC = 5;
    CLK6CONbits.OSWEN = 1; 
    while (CLK6CONbits.OSWEN);
    CLK6CONbits.DIVSWEN =1;
    while (CLK6CONbits.DIVSWEN);
    
    /** Clock used for DAC and CMP  
    * Input Clock Selection (NOSC) = 7 :PLL1 VCO DIV output = 800 MHz
    * Clock Division (INTDIV) = (1*2)
    * Final Clock for DAC = 400 MHz */
    CLK7DIVbits.INTDIV = 1;
    CLK7CONbits.OE = 1;
    CLK7CONbits.ON = 1;
    CLK7CONbits.NOSC = 7; 
    CLK7CONbits.OSWEN = 1; 
    while (CLK7CONbits.OSWEN);
    CLK7CONbits.DIVSWEN =1;
    while (CLK7CONbits.DIVSWEN);
 
    /** Clock used for UART  
    * Input Clock Selection (NOSC) = 5 :PLL1 FOUT output = 200 MHz
    * Clock Division (INTDIV) = (1*2)
    * Final Clock for UART = 100 MHz */
    CLK8DIVbits.INTDIV = 1;
    CLK8CONbits.OE = 1;
    CLK8CONbits.ON = 1;
    CLK8CONbits.NOSC = 5; 
    CLK8CONbits.OSWEN = 1; 
    while (CLK8CONbits.OSWEN);
    CLK8CONbits.DIVSWEN =1;
    while (CLK8CONbits.DIVSWEN);
    
    /** Reference Clock Generator REFO1 output through PPS (PPS to be 22)
    * Input Clock Selection (NOSC) = 5 :PLL1 FOUT output = 200 MHz 
    * Clock Division (INTDIV) = (4000*2)
    * Final Clock for REFO = 25 kHz*/
    CLK12DIVbits.INTDIV = 4000;
    CLK12CONbits.OE = 1;
    CLK12CONbits.ON = 1;
    CLK12CONbits.NOSC = 5; 
    CLK12CONbits.OSWEN = 1; 
    while (CLK12CONbits.OSWEN);
    CLK12CONbits.DIVSWEN =1;
    while (CLK12CONbits.DIVSWEN);
    
}

// </editor-fold>