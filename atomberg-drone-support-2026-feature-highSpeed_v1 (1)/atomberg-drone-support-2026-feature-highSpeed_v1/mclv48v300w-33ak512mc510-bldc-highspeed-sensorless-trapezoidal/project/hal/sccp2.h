// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file sccp2.h
 *
 * @brief This header file lists the functions and definitions - to configure 
 * and enable SCCP2 Module and its features 
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

#ifndef SCCP2_H
#define    SCCP2_H

#ifdef    __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">      
#include <xc.h>
#include <stdint.h>
#include "clock.h"
// </editor-fold>

// <editor-fold defaultstate="expanded" desc="TYPE DEFINITIONS ">  
#define SCCP2_FCY_STANDARD         StandardClockFrequencyGet
// </editor-fold> 

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void SCCP2_Timer_Initialize(void);

/**
 * Read timer counters.
 * Summary: Read timer high and low counters.
 * @example
 * <code>
 * SCCP2_TimerDataRead();
 * </code>
 */
uint32_t SCCP2_TimerDataRead(void); 


/**
 * Set timer counters.
 * Summary: Set timer high and low counters.
 * @example
 * <code>
 * SCCP2_TimerDataSet();
 * </code>
 */
void SCCP2_TimerDataSet(uint32_t ); 


/**
 * Set Timer period register.
 * Summary: Set timer period register.
 * @example
 * <code>
 * SCCP2_SetTimerPeriod();
 * </code>
 */
void SCCP2_SetTimerPeriod(uint32_t ); 

/**
 * Starts SCCP2 Timer module.
 * Summary: Starts SCCP2 Timer module.
 * @example
 * <code>
 * SCCP2_Timer_Start();
 * </code>
 */
void SCCP2_Timer_Start();

void SCCP2_Timer_Stop();

/**
 * Clears Capture/Compare/Time 2 interrupt request flag.
 * Summary: Clears CCT2 interrupt request flag.
 * @example
 * <code>
 * CCT2_InterruptFlagClear();
 * </code>
 */
inline static void CCT2_InterruptFlagClear(void) {_CCT2IF = 0; }

/**
 * Enable Capture/Compare/Time Compare 2 interrupt.
 * Summary: Enable CCT2 interrupt.
 * @example
 * <code>
 * CCT2_InterruptEnable();
 * </code>
 */
inline static void CCT2_InterruptEnable (void) {_CCT2IE = 1; }
/**
 * Disable Capture/Compare/Time Compare 2 interrupt.
 * Summary: Disable CCT2 interrupt.
 * @example
 * <code>
 * CCT2_InterruptDisable();
 * </code>
 */
inline static void CCT2_InterruptDisable (void) {_CCT2IE = 0; }
/**
 * Sets the priority level for Capture/Compare/Time Compare 2 interrupt.
 * @param priorityValue desired priority level between 0 and 7
 * @example
 * <code>
 * CCT2_InterruptPrioritySet(5);
 * </code>
 */
inline static void CCT2_InterruptPrioritySet(uint16_t priorityValue)
{
    _CCT2IP = 0x7&priorityValue;
}
// </editor-fold>
#ifdef    __cplusplus
}
#endif

#endif    /* SCCP2_H */
