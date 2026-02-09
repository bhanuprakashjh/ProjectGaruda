// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file pi.c
 *
 * @brief This module implements Proportional Integral Control (PI).
 *
 * Component: PI CONTROLLER
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

#include <stdint.h>
#include "pi.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: MC_ControllerPIUpdate(MC_PI_T *)  </B>
*
* @brief Function implementing PI Controller.
*        
* @param Pointer to the data structure containing PI Controller inputs.
* @return none.
* 
* @example
* <CODE> MC_ControllerPIUpdate(&piInputId); </CODE>
*
*/
void MC_ControllerPIUpdate(MC_PI_T *pPI)
{
    float error;
    float outUnsat;

    MC_PIPARAMS_T *pParam = &pPI->param;
    MC_PISTATE_T *pstateVar= &pPI->stateVar;
    
    /* Parallel form implementation of PI controller */
    error  = pPI->inReference - pPI->inMeasure;
    
    outUnsat  = pstateVar->integrator + pParam->kp * error ;

    if( outUnsat > pParam->outMax )
    {
        pPI->output = pParam->outMax;
    }
    else if( outUnsat < pParam->outMin )
    {
        pPI->output = pParam->outMin;
    }
    else
    {
        pstateVar->integrator = pstateVar->integrator + pParam->ki * error ;
        pPI->output = outUnsat;
    }
    
}

/**
* <B> Function: MC_ControllerPIReset(MC_PI_T *, float)  </B>
*
* @brief Function to reset the integrator output from PI Controller.
*        
* @param Pointer to the data structure containing PI Controller inputs.
* @param reset value
* @return none.
* 
* @example
* <CODE> MC_ControllerPIReset(&pFOC->piId, 0); </CODE>
*
*/
void MC_ControllerPIReset(MC_PI_T *pPI, float resetValue)
{
    pPI->stateVar.integrator = resetValue;
}

// </editor-fold>
