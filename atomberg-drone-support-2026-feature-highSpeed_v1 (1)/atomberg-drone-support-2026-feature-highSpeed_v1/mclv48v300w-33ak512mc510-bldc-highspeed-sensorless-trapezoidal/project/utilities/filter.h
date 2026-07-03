// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file filter.h
 *
 * @brief This header file lists definitions and interface functions 
 * for filters.
 * 
 * 
 * Component: FILTER 
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

#ifndef FILTER_H
#define	FILTER_H

#ifdef	__cplusplus
extern "C" {
#endif
// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include "filter_types.h"
// </editor-fold>
    
// <editor-fold defaultstate="expanded" desc="DEFINITIONS/CONSTANTS ">
/* Cut-off frequency for Low pass filter */    
#define LFP_CUTOFF_FREQUENCY 0.1
#define AVGFILTER_SCALER  4

// </editor-fold> 
    
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_LowPassFilterInit (void);
float MCAPP_LowPassFilter (float);
/**
 * Summary: Executes the Low Pass Filter
 * @param LPF state variable structure
 * @return filtered output
 */
float MCAPP_Filter(MCAPP_LPF *pLPF);

/**
 * Summary: Initialized the Low Pass Filter
 * @param LPF state variable structure
 * @return 
 */
void MCAPP_FilterInit(MCAPP_LPF *pLPF);
void MCAPP_MovingAvgFilterInit (void);
uint32_t MCAPP_MovingAvgFilter (uint32_t);
void MCAPP_MajorityFilterInit (void);
uint8_t MCAPP_MajorityFilter(uint8_t );

// </editor-fold> 


#ifdef	__cplusplus
}
#endif

#endif	/* FILTER_H */

