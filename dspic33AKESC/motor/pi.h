// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file pi.h
 *
 * @brief This header file lists data type definitions and interface functions 
 * of the PI controller module
 *
 * Component: PI Controller
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

#ifndef __PI_H
#define __PI_H

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="TYPE DEFINITIONS ">

/**
 * PI Controller parameters data type
*/
typedef struct
{
    /* Proportional gain co-efficient term */
    float kp;

    /* Integral gain co-efficient term */
    float ki;     
    
    /* Maximum output limit */
    float outMax;

    /* Minimum output limit */
    float outMin;
    
}MC_PIPARAMS_T ;     

/**
 * PI Controller state variable datatype
*/
typedef struct
{
    /* Integrator sum */
    float integrator;

} MC_PISTATE_T;

/**
 * PI Controller Input data type
*/
typedef struct
{
    /* Parameters to the PI controller */
    MC_PIPARAMS_T param;
    /* State variables to the PI controller */
    MC_PISTATE_T stateVar;
    /* Input reference to the PI controller */
    float   inReference;
    /* Input measured value */
    float   inMeasure;
    /* Output of the PI controller */
    float   output;
} MC_PI_T;


// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MC_ControllerPIUpdate(MC_PI_T *);
void MC_ControllerPIReset(MC_PI_T *, float resetValue);
// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of __PI_H
