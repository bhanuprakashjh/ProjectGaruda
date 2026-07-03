// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file filter_types.h
 *
 * @brief This header file lists data type for filters.
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

#ifndef FILTER_TYPES_H
#define	FILTER_TYPES_H

#ifdef	__cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="TYPE DEFINITIONS ">
    
/**
 * Low pass filter data type
*/
typedef struct
{
    float output;        /* Output of LPF */
}MCAPP_FILTER_LPF_T;

/**
 * Low pass filter data type
*/
typedef struct tagMCAPP_LPF
{
    float Y;				// Output
    float Y_prev;			// Previous Value of Output
    float X;				// Input
    float X_prev;			// Previous Value of Input
    float Wc;				// Cutoff Frequency
    float Ts;				// sampling time
}MCAPP_LPF;

/**
 * Moving average filter data type
*/
typedef struct
{
    uint32_t mean;           /* mean value */
    uint32_t average;        /* average filtered data */
    uint64_t accumalator;    /*Variable to accumulate previous output */
}MCAPP_FILTER_AVG_T;

/**
 * Majority function filter data type
 */
typedef struct
{
    uint8_t samples[3];   /* Last 3 input samples */
    uint8_t index;        /* Circular buffer index */
    uint8_t output;       /* Majority-voted output */
} MCAPP_FILTER_MAJ_T;

// </editor-fold>
#ifdef	__cplusplus
}
#endif

#endif	/* FILTER_TYPES_H */

