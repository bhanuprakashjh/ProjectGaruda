// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file fault.h
 *
 * @brief This module has the definitions related faults.
 *
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
#ifndef FAULT_H
#define	FAULT_H

#ifdef	__cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>
#include "measure.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="DEFINITIONS ">


// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS ">

/**
 * Fault States
 */
typedef enum 
{
    MCAPP_FAULT_NONE = 0,               /** No Fault  */
    MCAPP_OVERCURRENT_FAULT_PHASE = 1,  /** Over Current fault on phase - software */
    MCAPP_OVERCURRENT_FAULT_DCBUS = 2,  /** Over Current fault on DC Bus - Comparator + PCI */        

} MCAPP_FAULT_STATE_T;
   
    
typedef struct
{
    float 
        overCurrentFaultLimit;    /* Over current fault limit */
    
    MCAPP_FAULT_STATE_T
        faultState;             /* Fault state*/
    
}MCAPP_FAULT_T;

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

bool MCAPP_OverCurrentFault_Detect(MCAPP_MEASURE_T *, MCAPP_FAULT_T*);
    
// </editor-fold>


#ifdef	__cplusplus
}
#endif

#endif	/* FAULT_H */

