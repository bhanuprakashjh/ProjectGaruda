// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file linix36zwn.h
 *
 * @brief This file has definitions to be configured by the user for spinning
 * motor using field oriented control.
 *
 * Motor : LINIC 36ZWN24 
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

#ifndef __LINIX36ZWN24_H
#define __LINIX36ZWN24_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <math.h>
#include <stdint.h>

#include "../mc1_user_params.h"
// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/CONSTANTS ">
    
/*Parameters of LINIX36ZWN24 */
/** Motor Name Plate Parameters */
/* No.of pole pairs*/
#define POLE_PAIRS                                      4
/* Motor Back EMF Constant (unit : line voltage peak / kRPM) */
#define MOTOR_BEMF_CONSTANT_MECH                        3.445f
/* Maximum Speed (unit : RPM)*/
#define MAXIMUM_SPEED_RPM                               4000.0f
/* Motor Rated Phase Current in RMS (unit : amps) */
#define NOMINAL_CURRENT_PHASE_RMS                       1.85f
/* Motor Inertia (unit : Kg-m2) 
 * Needed for speed controller gain calculation */
#define MOTOR_INERTIA                                   0.00003f
   
// </editor-fold>
    
// <editor-fold defaultstate="collapsed" desc="Advance parameters ">    
/* The stator resistance and inductance are required for inner current control gain calculations */
/* per phase resistance (unit : ohm) */
#define MOTOR_PER_PHASE_RESISTANCE                      1.3205f
/* per phase inductance (unit : henry) */
#define MOTOR_PER_PHASE_INDUCTANCE                      0.00139f
    
/* Rotor locking parameters */
/* Lock time for Motor's poles alignment (unit : seconds)*/
#define ROTOR_LOCK_TIME_SEC     0.5f
    
/* Startup Ramp rate (unit : Second)  */
#define STARTUP_RAMP_TIME_S     5.0f

/*  Speed Reference Ramp rate (unit : RPM per second) */
#define SPEED_REF_RAMP_RATE     1500.0f   
    
/*PI Controller Parameters*/
/* Current Controller Zeta Constant */
#define IBUSPI_ZETA     1.0f 
/* Speed Feedback Filter Constant */ 
#define FILTER_CUTOFF   50.0f
/* Speed Controller Phase Margin */
#define PHASE_MARGIN    85.0f
    
// </editor-fold>
    
#ifdef __cplusplus
}
#endif

#endif	/* end of __LINIX36ZWN24_H */
