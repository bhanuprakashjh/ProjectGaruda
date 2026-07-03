// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file estim_bemf.c
 *
 * @brief This module implements Back EMF Estimator for BLDC.
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

#include "estim_bemf.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

inline static void MCAPP_CommutationChangeHandler(MCAPP_ESTIMATOR_BEMF_T *pEstim);

// </editor-fold>

/**
* <B> Function: void MCAPP_EstimatorBEMFInit(MCAPP_ESTIMATOR_BEMF_T *)  </B>
*
* @brief Function to reset BEMF Estimator Data Structure variables.
*
* @param    pointer to the data structure containing BEMF Estimator parameters.
* @return   none.
* @example
* <CODE> MCAPP_EstimatorBEMFInit(&estimator); </CODE>
*
*/
void MCAPP_EstimatorBEMFInit(MCAPP_ESTIMATOR_BEMF_T *pEstim)
{
    pEstim->commutationSector = 0;
    pEstim->reconstructedNeutralVoltage = 0;
    pEstim->commutationSector = 0;
    pEstim->presentCommutationSector = 0;
    pEstim->previousCommutationSector = 0;
    pEstim->closedLoopStartingFlag = 0;
    pEstim->closedLoopFlag = 0;
    pEstim->blankingTimeLimit = 0;
    pEstim->blankingTimeCounter = 0;
    pEstim->commutationInterval = 0;
    pEstim->timerPeriod = 0;
    pEstim->commutationSectorChanged = 0;
    pEstim->startBemfPhaseMapping = 0;
    pEstim->isBemfPhaseMappingComplete = 0;
    
    pEstim->commutationPeriod = 0;
    pEstim->updateInverterSwitchingPattern = 0;
            
    pEstim->calculateSpeed.avgPeriod = 0;
    pEstim->calculateSpeed.motorStallCounter = 0;
    pEstim->calculateSpeed.enableSpeedMeas = 0;
    pEstim->calculateSpeed.speed = 0;
    pEstim->calculateSpeed.omegaFiltered = 0;
    
    MCAPP_MovingAvgFilterInit();
    MCAPP_FilterInit(&pEstim->calculateSpeed.LPF_omega);
    MCAPP_MajorityFilterInit();
}

/**
* <B> Function: void MCAPP_EstimatorBEMF(MCAPP_ESTIMATOR_BEMF_T *)  </B>
*
* @brief BEMF based estimator to determine rotor speed and position based on
* motor parameters and feedbacks.
*
* @param    pointer to the data structure containing BEMF Estimator parameters.
* @return   none.
* @example
* <CODE> MCAPP_EstimatorBEMF(&estimator); </CODE>
*
*/
void MCAPP_EstimatorBEMF(MCAPP_ESTIMATOR_BEMF_T *pEstim)
{
    /* Calculate the reconstructed neutral voltage as the average of the measured BEMF values for phases A, B, and C. */
    pEstim->reconstructedNeutralVoltage = (*pEstim->pBemfA + *pEstim->pBemfB + *pEstim->pBemfC) / 3.0;
    
    /* Check if commutation blanking time has expired to prevent false BEMF detection */
    if(pEstim->blankingTimeCounter > pEstim->blankingTimeLimit)
    {
        uint8_t estimatedSector = 0; /* Estimated commutation sector based on BEMF comparison */
        
        /* Set bit 0 if Phase C voltage is higher than reconstructed neutral voltage */
        if (((*pEstim->pBemfC - pEstim->reconstructedNeutralVoltage) * pEstim->directionCmd) > 0)
            estimatedSector |= 0x01;

        /* Set bit 1 if Phase B voltage is higher than reconstructed neutral voltage */
        if (((*pEstim->pBemfB - pEstim->reconstructedNeutralVoltage) * pEstim->directionCmd) > 0)
            estimatedSector |= 0x02;

        /* Set bit 2 if Phase A voltage is higher than reconstructed neutral voltage */
        if (((*pEstim->pBemfA - pEstim->reconstructedNeutralVoltage) * pEstim->directionCmd) > 0)
            estimatedSector |= 0x04;
        
        /* Update the estimator commutation sector */
        
        pEstim->commutationSector = MCAPP_MajorityFilter(estimatedSector);
    }
    
//    /* Increment blanking time counter */
//    pEstim->blankingTimeCounter++;
        
    /* Update present commutation sector */
    pEstim->presentCommutationSector = pEstim->commutationSector;
    /* Check if commutation sector has changed */
    if(pEstim->presentCommutationSector != pEstim->previousCommutationSector)
    {
        LATCbits.LATC12 = 0;
        /* Set the commutation sector changed flag */
        pEstim->commutationSectorChanged = true;
        /* Commutation sector has changed */
        MCAPP_CommutationChangeHandler(pEstim);
        /* Update previous commutation sector */
        pEstim->previousCommutationSector = pEstim->presentCommutationSector;
		/* Set flag to update inverter switching pattern */
        pEstim->updateInverterSwitchingPattern = true;
        LATCbits.LATC12 = 1;
    }
    else
    {
        /* Do nothing */
    }
}

/**
* <B> Function: inline static void ReadCommutationInterval(MCAPP_ESTIMATOR_BEMF_T *pData) </B>
*
* @brief Reads the timer value for the last commutation interval and stores it in the estimator data structure.
*        Used to capture the elapsed time for a 60-degree commutation in sensorless BLDC control.
*
* @param pData Pointer to the estimator structure where the interval will be stored.
* @return none.
* @example
* <CODE> ReadCommutationInterval(&estimatorData); </CODE>
*
*/
inline static void ReadCommutationInterval(MCAPP_ESTIMATOR_BEMF_T *pData)
{
    /* Buffer variable to read the commutation interval */
    uint32_t commutationIntervalBuff = COMMUTATION_TIMER_READ(); 
    
    /* Check if the commutation interval is greater that timer counts per sector.
     This is to avoid invalid timer values caused due to glitches in the back EMF feedback */
    if(commutationIntervalBuff > pData->timerCountsPerSector)
    {
        /* Update the timer period */
        pData->timerPeriod = commutationIntervalBuff; 
        COMMUTATION_TIMER_SET(0);
    }
    else
    {
//        COMMUTATION_TIMER_SET(0);
    }
    /* Calculate moving average of period */
    pData->commutationInterval = MCAPP_MovingAvgFilter(pData->timerPeriod);
    
    /* Calculate the commutation period for the next 30-degree sector based on measured 60-degree interval */
     pData->commutationPeriod = (uint32_t)(pData->commutationInterval/2);    
}

/**
* <B> Function: inline static void MCAPP_CommutationChangeHandler(MCAPP_ESTIMATOR_BEMF_T *) </B>
*
* @brief Handles actions required after a commutation sector change in sensorless BLDC control.
*        Reads the commutation interval, resets the timer, and updates timing for the next sector.
*
* @param none.
* @return none.
* @example
* <CODE> MCAPP_CommutationChangeHandler(); </CODE>
*
*/
inline static void MCAPP_CommutationChangeHandler(MCAPP_ESTIMATOR_BEMF_T *pEstim)
{
    /* GSP telemetry: accepted-commutation tally (wraps; host diffs it) */
    { extern volatile uint16_t g_gspZcCount; g_gspZcCount++; }
    /* Read commutation interval */
    ReadCommutationInterval(pEstim);
    /* Update the average timer period value for calculating speed */
    pEstim->calculateSpeed.avgPeriod = pEstim->commutationInterval;
    /* Enable speed measurement */
    pEstim->calculateSpeed.enableSpeedMeas = true;
}

/** Function: void MapBemfPhaseSequence(MCAPP_ESTIMATOR_BEMF_T *) 
*
@brief Maps the Back-EMF (BEMF) phase sequence during sensorless motor startup.
Identifies the correct phase order by observing BEMF signals and updates
estimator data required for commutation.
*
@param pEstim Pointer to the BEMF estimator structure.
@return none.
@example
 MapBemfPhaseSequence(pEstim); 
*
*/
void MapBemfPhaseSequence(MCAPP_ESTIMATOR_BEMF_T *pEstim) 
{
    /* Calculate the reconstructed neutral voltage as the average of the measured BEMF values for phases A, B, and C. */
    pEstim->reconstructedNeutralVoltage = (*pEstim->pBemf1 + *pEstim->pBemf2 + *pEstim->pBemf3) / 3.0;

    if(pEstim->blankingTimeCounter > pEstim->blankingTimeLimit)
    {        
        /* Get input states */
        int8_t input1 =
            (( *pEstim->pBemf1 > pEstim->reconstructedNeutralVoltage ) -
             ( *pEstim->pBemf1 < pEstim->reconstructedNeutralVoltage )) * pEstim->directionCmd;

        int8_t input2 =
            (( *pEstim->pBemf2 > pEstim->reconstructedNeutralVoltage ) -
             ( *pEstim->pBemf2 < pEstim->reconstructedNeutralVoltage )) * pEstim->directionCmd;

        int8_t input3 =
            (( *pEstim->pBemf3 > pEstim->reconstructedNeutralVoltage ) -
             ( *pEstim->pBemf3 < pEstim->reconstructedNeutralVoltage )) * pEstim->directionCmd;

        /* Set flag */
        pEstim->isBemfPhaseMappingComplete = true;
        
        /* Identify sequence */
        if ((input1 == -1) && (input2 == 1) && (input3 == -1))
        {
            if(pEstim->directionCmd == 1)
            {
                pEstim->pBemfA = pEstim->pBemf2;
                pEstim->pBemfB = pEstim->pBemf1;
                pEstim->pBemfC = pEstim->pBemf3;
            }
            else if(pEstim->directionCmd == -1)
            {
                pEstim->pBemfA = pEstim->pBemf3;
                pEstim->pBemfB = pEstim->pBemf1;
                pEstim->pBemfC = pEstim->pBemf2;
            }
            else
            {
                /* Do nothing */
            }
        }
        else if ((input2 == -1) && (input1 == 1) && (input3 == -1))
        {
            if(pEstim->directionCmd == 1)
            {
                pEstim->pBemfA = pEstim->pBemf1;
                pEstim->pBemfB = pEstim->pBemf2;
                pEstim->pBemfC = pEstim->pBemf3;
            }
            else if(pEstim->directionCmd == -1)
            {
                pEstim->pBemfA = pEstim->pBemf3;
                pEstim->pBemfB = pEstim->pBemf2;
                pEstim->pBemfC = pEstim->pBemf1;
            }
            else
            {
                /* Do nothing */
            }
        }
        else if ((input3 == -1) && (input1 == 1) && (input2 == -1))
        {
            if(pEstim->directionCmd == 1)
            {
                pEstim->pBemfA = pEstim->pBemf1;
                pEstim->pBemfB = pEstim->pBemf3;
                pEstim->pBemfC = pEstim->pBemf2;
                }
            else if(pEstim->directionCmd == -1)
            {
                pEstim->pBemfA = pEstim->pBemf2;
                pEstim->pBemfB = pEstim->pBemf3;
                pEstim->pBemfC = pEstim->pBemf1;
            }
            else
            {
                /* Do nothing */
            }
        }
        else
        {
            /* Reset flag */
            pEstim->isBemfPhaseMappingComplete = false;
        }
    }
}

