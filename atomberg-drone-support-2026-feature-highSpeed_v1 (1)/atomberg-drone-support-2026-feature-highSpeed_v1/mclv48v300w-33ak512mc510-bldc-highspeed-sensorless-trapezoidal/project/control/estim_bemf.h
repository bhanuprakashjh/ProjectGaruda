// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file estim_bemf.h
 *
 * @brief This module implements Speed and Position Back EMF Estimator interface.
 *
 * Component: BEMF ESTIMATOR
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

#ifndef ESTIM_BEMF_H
#define	ESTIM_BEMF_H

#ifdef	__cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include "estim_bemf_types.h"
#include "sccp2.h"
    
// </editor-fold>
    
/* Macro to start the commutation timer */
#define COMMUTATION_TIMER_START   SCCP2_Timer_Start

/* Macro to stop the commutation timer */
#define COMMUTATION_TIMER_STOP    SCCP2_Timer_Stop    

/* Macro to read the commutation timer value */
#define COMMUTATION_TIMER_READ    SCCP2_TimerDataRead

/* Macro to set the commutation timer value */
#define COMMUTATION_TIMER_SET     SCCP2_TimerDataSet
    
/* Macro for commutation counter FCY */
#define COMMUTATION_COUNTER_FCY     SCCP3_FCY_STANDARD

// </editor-fold> 
    
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_EstimatorBEMFInit (MCAPP_ESTIMATOR_BEMF_T *);
void MCAPP_EstimatorBEMF (MCAPP_ESTIMATOR_BEMF_T *);
void MapBemfPhaseSequence(MCAPP_ESTIMATOR_BEMF_T *);

/**
Stops and resets all commutation timing resources.
Used to ensure a clean timing state before starting or restarting commutation.
*/
static inline void Commutation_ResetTiming(void)
{
    COMMUTATION_TIMER_STOP();
    COMMUTATION_TIMER_SET(0);
}

/**
* <B> Function: inline void MCAPP_CalculateSpeed(&speed) </B>
*
* @brief Function to calculate speed using timer value.
*        
* @param none.
* @return none.
* 
* @example
* <CODE> MCAPP_CalculateSpeed(&speed); </CODE>
*
*/ 
inline void MCAPP_CalculateSpeed(MCAPP_CALC_SPEED_T *pCalculateSpeed)
{    
    if(pCalculateSpeed->enableSpeedMeas == true)
    {
        /* Calculating Speed using the period*/
        if(pCalculateSpeed->avgPeriod != 0)
        {
            pCalculateSpeed->speed = (pCalculateSpeed->multiplier/pCalculateSpeed->avgPeriod);
            /*  Set the motor stall counter value for half of the measured speed */
            pCalculateSpeed->motorStallCounter = (uint32_t)(pCalculateSpeed->motorStallMultiplier / (0.5*pCalculateSpeed->speed));
            /* Speed has been measured, clear the flag */
            pCalculateSpeed->enableSpeedMeas = false;
        }
    }
    else
    {
        /* If sector transition doesn't occur for X ,then decrement speed */
		/* Stall detection 
         * At lower speeds, the time interval for detecting change is decremented,
           to determine whether the motor has stalled or is operating below the minimum speed threshold 
         * If the motor stalled is detected then the measured speed value is decremented */
        if(pCalculateSpeed->motorStallCounter == 0)
        {
            /* Decrement speed */
            pCalculateSpeed->speed--;
            if(pCalculateSpeed->speed < 0)
            {
                pCalculateSpeed->speed = 0;
            }
        }
        else
        {
            /* Decrement Motor stop counter */
            pCalculateSpeed->motorStallCounter--;
        }
    }
    /* Speed Filter*/
    pCalculateSpeed->LPF_omega.X = pCalculateSpeed->speed;
    pCalculateSpeed->omegaFiltered = MCAPP_Filter(&pCalculateSpeed->LPF_omega);
}


inline static float UpdateBlankingTime(MCAPP_ESTIMATOR_BEMF_T *pData,float maxSpeed)
{
    static int blankingDisabled = 0;
    float ratio = pData->calculateSpeed.omegaFiltered / maxSpeed;

    if (ratio >= 0.65f)
        blankingDisabled = 1;
    else if (ratio <= 0.60f)
        blankingDisabled = 0;

    return blankingDisabled ? 0.0f : pData->closedLoopBlankingTime;
}

// </editor-fold>

#ifdef	__cplusplus
}
#endif

#endif	/* ESTIM_BEMF_H */

