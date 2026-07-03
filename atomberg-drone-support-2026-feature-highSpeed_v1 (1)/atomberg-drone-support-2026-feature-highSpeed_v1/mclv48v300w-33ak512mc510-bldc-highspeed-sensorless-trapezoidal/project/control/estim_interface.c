// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file estim_interface.c
 *
 * @brief This module implements Estimator.
 * This is a sensor-less speed observer based on motor back EMF.
 *
 * Component: ESTIMATOR
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
#include <stdbool.h>

#include <math.h>

#include "estim_interface.h"
#include "mc1_calc_params.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Definitions ">

// </editor-fold>

/**
* <B> Function: MCAPP_EstimatorInit( MCAPP_ESTIM_INTERFACE_T *) </B>
*
* @brief Function to initialize estimator parameters
*        
* @param Pointer to the data structure containing estimator parameters
* @return none.
* 
* @example
* <CODE> MCAPP_EstimatorInit(&pFOC->estimatorInterface);  </CODE>
*
*/
void MCAPP_EstimatorInit(MCAPP_ESTIM_INTERFACE_T *pEstimInt )
{
    MCAPP_EstimatorBEMFInit(pEstimInt->pEstimBEMF); 
}

/**
* <B> Function: MCAPP_EstimatorStep( MCAPP_ESTIM_INTERFACE_T *) </B>
*
* @brief Function to interface estimator parameters
*        
* @param Pointer to the data structure containing estimator parameters
* @return none.
* 
* @example
* <CODE> MCAPP_EstimatorStep(&pFOC->estimatorInterface);   </CODE>
*
*/
void MCAPP_EstimatorStep(MCAPP_ESTIM_INTERFACE_T *pEstimInt )
{
   MCAPP_EstimatorBEMF(pEstimInt->pEstimBEMF); 
}




