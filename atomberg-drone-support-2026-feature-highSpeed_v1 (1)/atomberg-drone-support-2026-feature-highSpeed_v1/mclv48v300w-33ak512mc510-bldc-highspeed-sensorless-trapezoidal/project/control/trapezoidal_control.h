// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file trapezoidal_control.h
 *
 * @brief This header file lists data type definitions and interface functions 
 * of the Trapezoidal control for BLDC motor.
 *
 * Component: Trapezoidal Control
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
#ifndef __TRAPEZOIDAL_CONTROL_H
#define	__TRAPEZOIDAL_CONTROL_H

#ifdef	__cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include "trapezoidal_types.h"
#include "board_service.h"
#include "variable_trigger.h"  
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="DEFINITIONS ">  

#define MCAPP_CONTROL_SCHEME_T              MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T
    
#define SetCurrentReference HAL_MC1SetCurrentLimitDACRef
// </editor-fold>
    
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_TrapezoidalControlInit(MCAPP_CONTROL_SCHEME_T *);
void MCAPP_TrapezoidalControlStateMachine (MCAPP_CONTROL_SCHEME_T *); 
// </editor-fold>

#ifdef	__cplusplus
}
#endif

#endif	/* end of __TRAPEZOIDAL_CONTROL_H */

