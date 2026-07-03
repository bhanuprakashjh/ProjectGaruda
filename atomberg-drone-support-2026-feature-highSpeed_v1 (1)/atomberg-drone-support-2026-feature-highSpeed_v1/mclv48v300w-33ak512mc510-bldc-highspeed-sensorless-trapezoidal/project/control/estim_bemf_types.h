// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file estim_bemf_types.h
 *
 * @brief This module initializes data structure variable type definitions of 
 * Back Estimator interface structure and enumerations
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


#ifndef ESTIM_BEMF_TYPES_H
#define	ESTIM_BEMF_TYPES_H

#ifdef	__cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include "filter.h"    
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS">  
  
    typedef struct
{ 
    uint32_t
		motorStallCounter,   /* Counter for motor stall */
        motorStallMultiplier;/* Multiplier of motor stall counter */
         
    float    
        multiplier,         /* Speed Multiplier */
        avgPeriod,          /* Moving average output of period */
        speed;              /* Measured speed */
    bool
        enableSpeedMeas;  /* FLag to start speed calculation */
    
    MCAPP_LPF         LPF_omega;                      /** Omega LPF Filter */
    float             omegaFiltered;    /** Filtered Estimated Velocity */
    
}MCAPP_CALC_SPEED_T;

 /* Description:
    This structure will host parameters related to angle/speed estimator
    parameters. */
        
typedef struct
{
    float *pBemf1;   /* Pointer for mapping estimated back EMF 1 */
    float *pBemf2;   /* Pointer for mapping estimated back EMF 2 */
    float *pBemf3;   /* Pointer for mapping estimated back EMF 3 */
    float *pBemfA;   /* Pointer for estimated back EMF for phase A */
    float *pBemfB;   /* Pointer for estimated back EMF for phase B */
    float *pBemfC;   /* Pointer for estimated back EMF for phase C */
    int8_t directionCmd; /* Variable for direction command */
    uint8_t commutationSector; /* Variable for sector to commutate */ 
    uint8_t presentCommutationSector; /* Variable for present commuting sector */
    uint8_t previousCommutationSector; /* Variable for previous commutation sector */
    float reconstructedNeutralVoltage; /* Reconstructed neutral voltage */
    uint16_t blankingTimeCounter; /* Variable for blanking time counter */
    uint16_t blankingTimeLimit; /* Variable for blanking time limit */
    uint16_t startupBlankingTime; /* Variable for startup blanking time limit */
    uint16_t closedLoopBlankingTime; /* Variable for closed loop blanking time limit */
    bool closedLoopStartingFlag; /* Set: transition to closed starting  */  
    bool closedLoopFlag; /* Set: transition to closed loop; Reset: transition to startup */ 
    uint32_t commutationInterval; /* Timer value on every commutation sequence change */
    uint32_t timerPeriod; /* Buffer variable for timer value */
    uint32_t timerCountsPerSectorMultiplier; /* Multiplier for timer counts per sector */
    uint32_t timerCountsPerSector; /* Variable for timer counts per sector */
    bool commutationSectorChanged; /* Flag to indicate commutation sector has changed */
    /* Flag to indicate completion of BEMF phase sequence mapping */
    bool isBemfPhaseMappingComplete;
    /* Flag to start the back EMF phase mapping */
    bool startBemfPhaseMapping;
    uint32_t commutationPeriod; /* Variable for commutation period value after BEMF zero crossing is detected */
    bool updateInverterSwitchingPattern; /* Flag to update the inverter switching pattern */
    MCAPP_CALC_SPEED_T calculateSpeed;
    
} MCAPP_ESTIMATOR_BEMF_T;

// </editor-fold>

#ifdef	__cplusplus
}
#endif

#endif	/* ESTIM_BEMF_TYPES_H */

