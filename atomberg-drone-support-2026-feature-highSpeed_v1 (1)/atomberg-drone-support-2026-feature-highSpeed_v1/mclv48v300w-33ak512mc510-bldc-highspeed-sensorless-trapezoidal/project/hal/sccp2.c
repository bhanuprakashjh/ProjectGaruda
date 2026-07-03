// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file sccp2.c
 *
 * @brief This module configures and enables the SCCP2 Module 
 * 
 * Definitions in this file are for dsPIC33AK512MC510
 *
 * Component: SCCP2
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

// <editor-fold defaultstate="collapsed" desc="Header Files ">
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "sccp2.h"
// </editor-fold> 

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
/**
* <B> Function: SCCP2_Timer_Initialize() </B>
*
* @brief Function configures SCCP2 Module in 32bit timer mode
*        
* @param none.
* @return none.
* 
* @example
* <CODE> SCCP2_Timer_Initialize(); </CODE>
*
*/
void SCCP2_Timer_Initialize(void)
{
    /* Set SCCP2 operating OFF */
    CCP2CON1bits.ON = 0;   
    /* Set timebase width (32-bit = 1) */
    CCP2CON1bits.T32 = 1;    
    /* Module operates as an output compare peripheral */
    CCP2CON1bits.CCSEL = 0;     
    /* Set mode to 16/32 bit timer mode features to Output Timer Mode */
    CCP2CON1bits.MOD = 0b0000;  
    /* No external synchronization; timer rolls over at FFFFh or matches with the Timer Period register */
    CCP2CON1bits.SYNC = 0b00000;
    /* Set timebase synchronization (Synchronized) */
    CCP2CON1bits.TMRSYNC = 0;   
    /* Set the clock source (Tcy) */
    CCP2CON1bits.CLKSEL = 0b000;
    /* Set the clock pre-scaler 1:1 prescaler */
    CCP2CON1bits.TMRPS = 0b00;  
    /* Set Sync/Triggered mode (Synchronous) */
    CCP2CON1bits.TRIGEN = 0;    

    /* CCP2 Control Register 2 */
    CCP2CON2 = 0x00000000;
    /* Initialize timer prior to enable module. */
    CCP2TMR = 0x00000000;
    /* Set timer period register */
    CCP2PR = 0xFFFFFFFF;  

    /* Interrupt Priority set */
    IPC6bits.CCT2IP = 0;        
    /* Clear Interrupt flag */
    IFS1bits.CCT2IF = 0;         
    /* Disable Interrupt */
    IEC1bits.CCT2IE = 0; 

    /* Disable CCP/input capture */
    CCP2CON1bits.ON = 0;     
}

void SCCP2_SetTimerPeriod(uint32_t timerPeriod) 
{ 
    CCP2PR = timerPeriod;             
}

void SCCP2_Timer_Start(void)
{
    /* Start the Timer */
    CCP2CON1bits.ON = true;

}

void SCCP2_Timer_Stop(void)
{
    /* Stop the Timer */
    CCP2CON1bits.ON = false;
}

uint32_t SCCP2_TimerDataRead(void) 
{ 
    uint32_t timervalue;

    timervalue =  CCP2TMR;
    return timervalue;
}

void SCCP2_TimerDataSet(uint32_t value) 
{ 
   /* Update the counter values */
    CCP2TMR = value;           
}
