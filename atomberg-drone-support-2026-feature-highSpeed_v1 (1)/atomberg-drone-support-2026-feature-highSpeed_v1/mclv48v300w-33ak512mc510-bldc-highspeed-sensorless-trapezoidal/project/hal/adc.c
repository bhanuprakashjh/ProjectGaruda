// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file adc.c
 *
 * @brief This module configures and enables the ADC Module 
 * 
 * Definitions in this file are for dsPIC33AK512MC510
 *
 * Component: ADC
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
#include <stdint.h>

#include "adc.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
/**
* <B> Function: InitializeADCs() </B>
*
* @brief Function initializes and enable the ADC Module
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitializeADCs(); </CODE>
*
*/
void InitializeADCs(void)
{    
    /* Channel Configuration for IA*/
    /*ADC1 Channel 2 for IA : OA1OUT/AD1AN0/CVDAN2/CMP1A/RP3/RA2 */
    AD1CH2CON1bits.PINSEL = 0;       
    AD1CH2CON1bits.SAMC = 3;              
    AD1CH2CON1bits.FRAC = 0;
    AD1CH2CON1bits.DIFF = 0;
	
	/* Channel Configuration for IB*/
    /*ADC2 Channel 1 for IB : OA2OUT/AD2AN0/CVDAN16/CVDTX0/CMP2A/RP17/RB0 */    
    AD2CH1CON1bits.PINSEL = 0;       
    AD2CH1CON1bits.SAMC = 3;              
    AD2CH1CON1bits.FRAC = 0;
    AD2CH1CON1bits.DIFF = 0;
	
    /* Channel Configuration for IC*/
    /*ADC3 Channel 1 for IC : AD3ANN1/AD3AN3/CVDAN29/CVDTX13/RP30/RB13 */ 
    AD3CH1CON1bits.PINSEL = 3;      
    AD3CH1CON1bits.SAMC = 3;      
    AD3CH1CON1bits.FRAC = 0;
    AD3CH1CON1bits.DIFF = 0;
    
    /* Channel Configuration for POT*/
    /*ADC2 Channel 2 for POT : AD2AN5/CVDAN31/CVDTX15/RP32/RB15 */    
    AD2CH2CON1bits.PINSEL = 5;       
    AD2CH2CON1bits.SAMC = 5;              
    AD2CH2CON1bits.FRAC = 0;
    AD2CH2CON1bits.DIFF = 0;
    
    /* Channel Configuration for IBUS*/
    /*ADC3 Channel 0 for IBUS : OA3OUT/AD3AN0/CVDAN5/CMP3A/RP6/INT0/RA5 */ 
    AD3CH0CON1bits.PINSEL = 0;      
    AD3CH0CON1bits.SAMC = 3;      
    AD3CH0CON1bits.FRAC = 0;
    AD3CH0CON1bits.DIFF = 0;

    /* Channel Configuration for VBUS*/
    /*ADC3 Channel 2 for VBUS : AD3AN4/CVDTX29/RP81/RF0 */     
    AD3CH2CON1bits.PINSEL = 4;   
    AD3CH2CON1bits.SAMC = 5;         
    AD3CH2CON1bits.FRAC = 0;
    AD3CH2CON1bits.DIFF = 0;
    
    /* Channel Configuration for VA*/
    /*ADC1 Channel 0 for VA : UREF/AD1AN3/CVDAN9/IBIAS2/ISRC2/RP10/RA9 */     
    AD1CH0CON1bits.PINSEL = 3;   
    AD1CH0CON1bits.SAMC = 3;         
    AD1CH0CON1bits.FRAC = 0;
    AD1CH0CON1bits.DIFF = 0;
    
    /* Channel Configuration for VB*/
    /*ADC1 Channel 1 for VB : AD1ANN1/AD1AN4/CVDAN10/CMPEN/RP11/RA10 */     
    AD1CH1CON1bits.PINSEL = 4;   
    AD1CH1CON1bits.SAMC = 3;         
    AD1CH1CON1bits.FRAC = 0;
    AD1CH1CON1bits.DIFF = 0;
    
    /* Channel Configuration for VC*/
    /*ADC2 Channel 0 for VC : AD2AN4/CVDAN24/CVDTX8/CMP2D/RP25/RB8 */     
    AD2CH0CON1bits.PINSEL = 4;   
    AD2CH0CON1bits.SAMC = 3;         
    AD2CH0CON1bits.FRAC = 0;
    AD2CH0CON1bits.DIFF = 0;
    
    /* Turn on the ADC Core 1 */   
    AD1CONbits.ON = 1;     
    /* Waiting till the ADC Core 1 is ready*/
    while(AD1CONbits.ADRDY == 0);  
    
    /* Turn on the ADC Core 2 */
    AD2CONbits.ON = 1;             
    /* Waiting till the ADC Core 2 is ready*/
    while(AD2CONbits.ADRDY == 0);   
    
    /* Turn on the ADC Core 3 */   
    AD3CONbits.ON = 1;     
    /* Waiting till the ADC Core 3 is ready*/
    while(AD3CONbits.ADRDY == 0);  

    /*AD3CH2 - VBUS used for ADC Interrupt */
    /* Set ADC interrupt priority IPL 6  */ 
    _AD3CH2IP = 7;
    /* Clear ADC interrupt flag */
    _AD3CH2IF = 0;
    /* Disable the AD1CH1 interrupt  */
    _AD3CH2IE = 0;
    
    /*Selecting the Trigger Sources for ADC Channels*/   
       
    /*Software trigger for VA - AD1CH0*/
    AD1CH0CON1bits.TRG1SRC = 0b000001;   
    /*Software trigger for VB - AD1CH1*/
    AD1CH1CON1bits.TRG1SRC = 0b000001;    
     
    /*Software trigger for VC - AD2CH0*/
    AD2CH0CON1bits.TRG1SRC = 0b000001; 
    /*PWM1 ADC Trigger 1 for POT - AD2CH2*/
    AD2CH2CON1bits.TRG1SRC = 5;  
    
    /*PWM1 ADC Trigger 1 for IBUS  - AD3CH0*/
    AD3CH0CON1bits.TRG1SRC = 0b000111;   
    /*PWM1 ADC Trigger 1 for VBUS - AD3CH2*/
    AD3CH2CON1bits.TRG1SRC = 5;    
    
    /* Set ADC interrupt priority IPL 6  */ 
    _AD1CH1IP = 6;
    /* Clear ADC interrupt flag */
    _AD1CH1IF = 0;
    /* Disable the AD1CH1 interrupt  */
    _AD1CH1IE = 0;
#ifdef ENABLE_PHASE_CURRENT_MEASUREMENT
    /* Trigger Phase current sensing ADC channels */
    /*PWM1 ADC Trigger 1 for IA - AD1CH2*/
    AD1CH2CON1bits.TRG1SRC = 4;
    /*PWM1 ADC Trigger 1 for IB - AD2CH1*/
    AD2CH1CON1bits.TRG1SRC = 4;  
    /*PWM1 ADC Trigger 1 for IC - AD3CH1*/
    AD3CH1CON1bits.TRG1SRC = 4;
    
    /*AD1CH2 - IA used for ADC Interrupt */
    /* Set ADC interrupt priority IPL 6  */ 
    _AD1CH2IP = 6;
    /* Clear ADC interrupt flag */
    _AD1CH2IF = 0;
    /* Disable the AD1CH2 interrupt  */
    _AD1CH2IE = 0;
#endif
}

// </editor-fold>