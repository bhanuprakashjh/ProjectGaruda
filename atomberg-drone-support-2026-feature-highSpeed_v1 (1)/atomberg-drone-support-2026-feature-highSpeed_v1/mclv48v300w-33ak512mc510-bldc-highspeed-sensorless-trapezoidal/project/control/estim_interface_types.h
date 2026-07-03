// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file estim_interface_types.h
 *
 * @brief This module initializes data structure variable type definitions of 
 * Estimator interface structure and enumerations
 * 
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

#ifndef ESTIM_INTERFACE_TYPES_H
#define	ESTIM_INTERFACE_TYPES_H

#ifdef	__cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
        
#include "estim_bemf_types.h"
#include "motor_types.h"

// </editor-fold>
    
// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS">

 /* Description:
    This structure will host parameters related to angle/speed estimator
    parameters. */
        
typedef struct
{
    /* Back EMF based estimator */
    MCAPP_ESTIMATOR_BEMF_T *pEstimBEMF;
    
} MCAPP_ESTIM_INTERFACE_T;        
        
        
// </editor-fold>


#ifdef	__cplusplus
}
#endif

#endif	/* ESTIM_INTERFACE_TYPES_H */

