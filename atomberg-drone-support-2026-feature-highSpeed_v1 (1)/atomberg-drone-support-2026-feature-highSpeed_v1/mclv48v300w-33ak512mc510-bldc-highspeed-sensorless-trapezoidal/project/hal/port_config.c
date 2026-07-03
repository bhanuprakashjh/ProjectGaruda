// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file port_config.c
 *
 * @brief This module initializes the GPIO pins as analog/digital,input or 
 * output etc. Also to PPS functionality to Re-mappable input or output pins.
 * 
 * Definitions in this file are for dsPIC33AK512MC510
 *
 * Component: PORTS
 *
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">

/*
© [2025] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/


// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <xc.h>

#include "port_config.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: SetupGPIOPorts() </B>
*
* @brief Function initialize all ports as input and digital pins
*        
* @param none.
* @return none.
* 
* @example
* <CODE> SetupGPIOPorts(); </CODE>
*
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

    #ifdef TRISE
        TRISE = 0xFFFF;
        LATE  = 0x0000;
    #endif
    #ifdef ANSELE
        ANSELE = 0x0000;
    #endif

    #ifdef TRISF
        TRISF = 0xFFFF;
        LATF  = 0x0000;
    #endif
    #ifdef ANSELF
        ANSELF = 0x0000;
    #endif

    #ifdef TRISG
        TRISG = 0xFFFF;
        LATG  = 0x0000;
    #endif
    #ifdef ANSELG
        ANSELG = 0x0000;
    #endif

    MapGPIOHWFunction();

}
/**
* <B> Function: MapGPIOHWFunction() </B>
*
* @brief Function maps port pins as input or output, analog or digital
*        
* @param none.
* @return none.
* 
* @example
* <CODE> MapGPIOHWFunction(); </CODE>
*
*/
void MapGPIOHWFunction(void)
{  
    /* Amplifier input and output pins */
    /* IA : OA1
     * Positive Input   : DIM:013 - PIN23: OA1IN+/AD1AN1/AD5AN4/CVDAN4/CMP1B/RP5/RA4
     * Negative Input   : DIM:015 - PIN22: OA1IN-/AD1ANN2/AD1AN2/CVDAN3/CMPCN/CMP1C/RP4/RA3
     * Amplifier Output : DIM:017 - PIN21: OA1OUT/AD1AN0/CVDAN2/CMP1A/RP3/RA2       */
    ANSELAbits.ANSELA4 = 1;
    TRISAbits.TRISA4 = 1;   
    ANSELAbits.ANSELA3 = 1;
    TRISAbits.TRISA3 = 1;   
    ANSELAbits.ANSELA2 = 1;
    TRISAbits.TRISA2 = 0;
    
    /* IB : OA2
     * Positive Input   : DIM:021 - PIN33: OA2IN+/AD2AN1/CVDAN18/CVDTX2/CMP2B/RP19/RB2
     * Negative Input   : DIM:023 - PIN32: TMS/OA2IN-/AD2ANN2/AD2AN2/CVDAN17/CVDTX1/CMP2C/RP18/RB1
     * Amplifier Output : DIM:025 - PIN31: OA2OUT/AD2AN0/CVDAN16/CVDTX0/CMP2A/RP17/RB0   */
    ANSELBbits.ANSELB2 = 1;
    TRISBbits.TRISB2 = 1;   
    ANSELBbits.ANSELB1 = 1;
    TRISBbits.TRISB1 = 1;     
    ANSELBbits.ANSELB0 = 1;
    TRISBbits.TRISB0 = 0;
    
    /* IBUS : OA3
     * Positive Input   : DIM:029 - PIN26: OA3IN+/AD3AN1/AD4AN5/CVDAN21/CVDTX5/CMP3B/RP22/RB5
     * Negative Input   : DIM:031 - PIN25: OA3IN-/AD3ANN2/AD3AN2/CVDAN6/CMPDN/CMP3C/RP7/RA6
     * Amplifier Output : DIM:033 - PIN24: OA3OUT/AD3AN0/CVDAN5/CMP3A/RP6/INT0/RA5 */
    ANSELBbits.ANSELB5 = 1;
    TRISBbits.TRISB5 = 1;   
    ANSELAbits.ANSELA6 = 1;
    TRISAbits.TRISA6 = 1;   
    ANSELAbits.ANSELA5 = 1;
    TRISAbits.TRISA5 = 0;
    
#ifndef INTERNAL_OPAMP_CONFIG
    /* External Amplifier mode - Amplifier output pins are configured as
     analog input channels
     * IA   : DIM:019 - PIN21: OA1OUT/AD1AN0/CVDAN2/CMP1A/RP3/RA2   
     * IB   : DIM:027 - PIN31: OA2OUT/AD2AN0/CVDAN16/CVDTX0/CMP2A/RP17/RB0
     * IBUS : DIM:035 - PIN24: OA3OUT/AD3AN0/CVDAN5/CMP3A/RP6/INT0/RA5 */
    TRISAbits.TRISA2 = 1;   
    TRISBbits.TRISB0 = 1;  
    TRISAbits.TRISA5 = 1;  
#endif
    /* IC from external amplifier on MCLV-48V-300W development board 
     * IC   : DIM:020 - PIN30: AD3ANN1/AD3AN3/CVDAN29/CVDTX13/RP30/RB13*/
    ANSELBbits.ANSELB13 = 1;
    TRISBbits.TRISB13 = 1;  
    
    /* Potentiometer  input (POT1) - used as Speed Reference 
     * DIM:028 - PIN #36: AD2AN5/CVDAN31/CVDTX15/RP32/RB15 */
    ANSELBbits.ANSELB15 = 1;
    TRISBbits.TRISB15 = 1;   
    
	/* Phase Voltages (VA, VB, and VC)
	 * VA   : DIM:009 - PIN #14: UREF/AD1AN3/CVDAN9/IBIAS2/ISRC2/RP10/RA9  
     * VB   : DIM:011 - PIN #15: AD1ANN1/AD1AN4/CVDAN10/CMPEN/RP11/RA10
     * VC   : DIM:022 - PIN #34: AD2AN4/CVDAN24/CVDTX8/CMP2D/RP25/RB8 */
    ANSELAbits.ANSELA9 = 1;
    TRISAbits.TRISA9 = 1;    
    ANSELAbits.ANSELA10 = 1;
    TRISAbits.TRISA10 = 1;    
    ANSELBbits.ANSELB8 = 1;
    TRISBbits.TRISB8 = 1;    
	
    /* DC Bus Voltage (VBUS) 
     * DIM:039 -  PIN #37: AD3AN4/CVDTX29/RP81/RF0 */
    ANSELFbits.ANSELF0 = 1;
    TRISFbits.TRISF0 = 1;   
    
    /* Digital SIGNALS */   
    /* Inverter Control - PWM Outputs
     * PWM1L : DIM:003 - PIN #83  TDI/RP52/PWM1L/IOMAD0/RD3
     * PWM1H : DIM:001 - PIN #82  TDO/RP51/PWM1H/IOMAD1/RD2
     * PWM2L : DIM:007 - PIN #79  TCK/RP50/PWM2L/IOMAD2/RD1
     * PWM2H : DIM:005 - PIN #78  RP49/PWM2H/IOMAD3/RD0
     * PWM3L : DIM:004 - PIN #67  RP37/PWM3L/IOMAD4/RC4
     * PWM3H : DIM:002 - PIN #66  RP36/PWM3H/IOMAD5/RC3      */
    TRISDbits.TRISD3 = 0 ;          
    TRISDbits.TRISD2 = 0 ;         
    TRISDbits.TRISD1 = 0 ;          
    TRISDbits.TRISD0 = 0 ;           
    TRISCbits.TRISC4 = 0 ;          
    TRISCbits.TRISC3 = 0 ;         
     
    /* Debug LEDs */
    /* LED1 : DIM:030 - PIN #10 : CVDTX21/RP67/RE2   */
    TRISEbits.TRISE2 = 0;
    /* LED2 : DIM:032 - PIN #06 : CVDTX22/RP68/RE3  */
    TRISEbits.TRISE3 = 0;

    /* Push button Switches */
    /* SW1 : DIM:034 - PIN #03 : CVDAN12/RP13/RA12   */
    TRISAbits.TRISA12 = 1;            
    /* SW2 : DIM:036 - PIN #02 : CVDTX20/RP66/RE1  */
    TRISEbits.TRISE1 = 1;            
	
    /* Configuring FLTLAT_OC_OV (DIM:040) - Pin #01 : CVDTX19/RP65/RE0 as PCI8 
     Please note PWM fault PCI is configured with Comparator output, not PCI pin*/
	_PCI8R = 65;
    
	/** Diagnostic Interface
        Re-map UART Channels to the device pins connected to the following 
        pins on the Motor Control Development Board.
        UART_RX : DIM:054 - PIN #74 : RP47/APWM3H/IOMBD9/RC14 (Input)
        UART_TX : DIM:052 - PIN #75 : RP48/APWM3L/IOMBD8/RC15(Output)   */
    _U1RXR = 47;
    _RP48R = 19;
 
    /* GPIO for debugging */
    /* SRM control board with R2*/
    
    /* MCLV with R2 */
    
    /* MCLV with R1 */
    /* DIM83 [44] PGC1/AD4AN0/CVDAN20/CVDTX4/CMP4B/RP21/SCL1/RB4 XPRO2_ADC(-) (4) */
    TRISBbits.TRISB4 = 0;
    /* DIM85 [52] RP98/APWM4H/IOMBD11/RG1 XPRO2_GPIO1 (5) */
    TRISGbits.TRISG1 = 0; 
    /* DIM87 [49] RP84/RF3 XPRO2_GPIO2 (6) */
    TRISFbits.TRISF3 = 0;
    /* DIM89 [76] RP58/PWM6H/IOMBD3/IOMBF7/RD9 XPRO2_PWM(+) (7) */
    TRISDbits.TRISD9 = 0;
    _RP58R = 63;
    /* DIM91 [70] RP45/PWM8H/IOMBD7/RC12 XPRO2_PWM(-) (8) */
    TRISCbits.TRISC12 = 0;
 
}

/**
* <B> Function: OpampConfig() </B>
*
* @brief Function to configure and enable the Op-Amp Module
*        
* @param none.
* @return none.
* 
* @example
* <CODE> OpampConfig(); </CODE>
*
*/
void OpampConfig (void)
{
    /** AMP1CON1 :AMP1 Control Register 1*/
    AMP1CON1 = 0x0000;
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP1CON1bits.AMPEN = 0;
    /** Bit 14 = HPEN High-Power Enable bit  
        1 Enables Op Amp High-Power (high bandwidth) mode ; 
        0 Disables Op Amp High-Power mode */    
    AMP1CON1bits.HPEN = 1;
    /** Bit 13 = UGE Unity Gain Buffer Enable bit 
        1 Enables Unity Gain mode ; 
        0 Disables Unity Gain mode */
    AMP1CON1bits.UGE = 0;
    /** Bits 12:11 = DIFFCON[1:0] Differential Input Mode Control bits
        11 Reserved, do not use; 
        10 Turn NMOS differential input pair off
        01 Turn PMOS differential input pair off 
        00 Use both NMOS and PMOS differential input pair */
    AMP1CON1bits.DIFFCON = 0;
    /** Bit 10 = REFEN Input Reference Enable bit
        1 Noninverting input connected to internal device voltage source
        0 Noninverting input of op amp connected to user pin */
    AMP1CON1bits.REFEN = 0;
    
    /** AMP1CON2 :AMP1 Control Register 2*/
    /** bits 28-24 POFFSETHP[4:0]
        Offset Correction for PMOS differential input pair (high power mode)*/
    AMP1CON2bits.POFFSETHP = 0;
    /** bits 20-16 = NOFFSETHP[4:0]
        Offset Correction for NMOS differential input pair (high power mode)*/
    AMP1CON2bits.NOFFSETHP = 0;
    /** bits 12-8 = POFFSETLP[4:0]
        Offset Correction for PMOS differential input pair (low power mode)*/
    AMP1CON2bits.POFFSETLP = 0;
   /** bits 4-0 =  NOFFSETLP[4:0]
       Offset Correction for NMOS differential input pair (low power mode)*/
    AMP1CON2bits.NOFFSETLP = 0;
    
    /** AMP2CON1 :AMP2 Control Register 1*/
    AMP2CON1 = 0x0000;
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP2CON1bits.AMPEN = 0;
    /** Bit 14 = HPEN High-Power Enable bit  
        1 Enables Op Amp High-Power (high bandwidth) mode ; 
        0 Disables Op Amp High-Power mode */    
    AMP2CON1bits.HPEN = 1;
    /** Bit 13 = UGE Unity Gain Buffer Enable bit 
        1 Enables Unity Gain mode ; 
        0 Disables Unity Gain mode */
    AMP2CON1bits.UGE = 0;
    /** Bits 12:11 = DIFFCON[1:0] Differential Input Mode Control bits
        11 Reserved, do not use; 
        10 Turn NMOS differential input pair off
        01 Turn PMOS differential input pair off 
        00 Use both NMOS and PMOS differential input pair */
    AMP2CON1bits.DIFFCON = 0;
    /** Bit 10 = REFEN Input Reference Enable bit
        1 Noninverting input connected to internal device voltage source
        0 Noninverting input of op amp connected to user pin */
    AMP2CON1bits.REFEN = 0;
    
    /** AMP2CON2 :AMP2 Control Register 2*/
    /** bits 28-24 = POFFSETHP[4:0]
        Offset Correction for PMOS differential input pair (high power mode)*/
    AMP2CON2bits.POFFSETHP = 0;
    /** bits 20-16 = NOFFSETHP[4:0]
        Offset Correction for NMOS differential input pair (high power mode)*/
    AMP2CON2bits.NOFFSETHP = 0;
    /** bits 12-8 = POFFSETLP[4:0]
        Offset Correction for PMOS differential input pair (low power mode)*/
    AMP2CON2bits.POFFSETLP = 0;
   /** bits 4-0 = NOFFSETLP[4:0]
       Offset Correction for NMOS differential input pair (low power mode)*/
    AMP2CON2bits.NOFFSETLP = 0;
    
    /** AMP3CON1 :AMP3 Control Register 1*/
    AMP3CON1 = 0x0000;
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP3CON1bits.AMPEN = 0;
    /** Bit 14 = HPEN High-Power Enable bit  
        1 Enables Op Amp High-Power (high bandwidth) mode ; 
        0 Disables Op Amp High-Power mode */    
    AMP3CON1bits.HPEN = 1;
    /** Bit 13 = UGE Unity Gain Buffer Enable bit 
        1 Enables Unity Gain mode ; 
        0 Disables Unity Gain mode */
    AMP3CON1bits.UGE = 0;
    /** Bits 12:11 = DIFFCON[1:0] Differential Input Mode Control bits
        11 Reserved, do not use; 
        10 Turn NMOS differential input pair off
        01 Turn PMOS differential input pair off 
        00 Use both NMOS and PMOS differential input pair */
    AMP3CON1bits.DIFFCON = 0;
    /** Bit 10 = REFEN Input Reference Enable bit
        1 Noninverting input connected to internal device voltage source
        0 Noninverting input of op amp connected to user pin */
    AMP3CON1bits.REFEN = 0;
    
    /** AMP3CON2 :AMP3 Control Register 2*/
    /** bits 28-24 = POFFSETHP[4:0]
        Offset Correction for PMOS differential input pair (high power mode)*/
    AMP3CON2bits.POFFSETHP = 0;
    /** bits 20-16 = NOFFSETHP[4:0]
        Offset Correction for NMOS differential input pair (high power mode)*/
    AMP3CON2bits.NOFFSETHP = 0;
    /** bits 12-8 = POFFSETLP[4:0]
        Offset Correction for PMOS differential input pair (low power mode)*/
    AMP3CON2bits.POFFSETLP = 0;
   /** bits 4-0 = NOFFSETLP[4:0]
       Offset Correction for NMOS differential input pair (low power mode)*/
    AMP3CON2bits.NOFFSETLP = 0;
    
	/* Enabling the amplifiers */
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP1CON1bits.AMPEN = 1;
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP2CON1bits.AMPEN = 1;
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP3CON1bits.AMPEN = 1;
}

// </editor-fold>