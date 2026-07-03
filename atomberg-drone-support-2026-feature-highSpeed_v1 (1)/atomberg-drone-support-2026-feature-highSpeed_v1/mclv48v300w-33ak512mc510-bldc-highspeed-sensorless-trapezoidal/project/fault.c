// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file fault.c
 *
 * @brief This module implements the fault detection routines.
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
#include <stdbool.h>

#include "fault.h"

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Function Declarations ">



// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Global Variables  ">



// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/**
 * <B> Function: MCAPP_OverCurrentFault_Detect(MCAPP_MEASURE_T *pMotorInputs, MCAPP_FAULT_T * pFault)  </B>
 * 
 * @brief Function to detect overcurrent fault
 * @param Pointer to the data structure containing measured motor parameters 
 * and fault data structure
 * @return fault status
 */
bool MCAPP_OverCurrentFault_Detect(MCAPP_MEASURE_T *pMotorInputs, MCAPP_FAULT_T * pFault)
{
    MCAPP_MEASURE_CURRENT_T *pCurrent = &pMotorInputs->measureCurrent;
    
    bool faultStatus;
    
    if(( pCurrent->Ibus_actual > pFault->overCurrentFaultLimit))
    {
        pFault->faultState = MCAPP_OVERCURRENT_FAULT_PHASE;
        faultStatus = 1;
    }
    else{
        pFault->faultState = MCAPP_FAULT_NONE;
        faultStatus = 0;
    }
    
    return faultStatus;
}

// </editor-fold>