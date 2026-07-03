// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc1_user_params.h
 *
 * @brief This file has definitions to be configured by the user for spinning
 * 		  motor 1 using Trapezoidal control.
 *
 * Component: APPLICATION (motor control 1 - mc1)
 * Motor : To be selected from '#define MOTOR  1'
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

#ifndef __MC1_USER_PARAMS_H
#define __MC1_USER_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <math.h>

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="MACROS ">

/* Define macros for Operational Modes */
        
/*Control Loop Selection : 
                        0 = Voltage (duty) control loop
                        1 = Closed-loop outer speed and inner current control using a PI controller
                        2 = Closed-loop current control using a PI controller */
#define CLOSED_LOOP 1
    
/* 
 * This configuration should be defined based on hardware capability (presence of current feedbacks).
 * or to disable inner current loop  
 * Note: If the user intends to enable current control, the advance parameters 
 * of the motor in motor parameter file needs to be updated.
 */
 #define ENABLE_CURRENT_CONTROL  

/* --------------------------------------------------------------------------
 * PWM Current Limiter (PCI) using PWM CLPCI feature
 *
 * When defined:
 *   - Limits the DC bus current to the configured maximum value
 *     using the PWM CLPCI hardware feature.
 *
 * When undefined:
 *   - No PWM-based bus current limiting is applied.
 *
 * Note:
 *   - This feature is automatically disabled when current control
 *     (ENABLE_CURRENT_CONTROL) is enabled.
 *   - It is recommended to enable this feature when current control
 *     is disabled to ensure bus and motor currents remain within
 *     board and motor limits.
 * -------------------------------------------------------------------------- */
#define ENABLE_PWM_CURRENT_LIMITER
// <editor-fold defaultstate="collapsed" desc="PWM CURRENT LIMITER SELECTION ">
    #ifdef ENABLE_CURRENT_CONTROL

    /* Current control enabled: PWM current limiter not used */
    #undef ENABLE_PWM_CURRENT_LIMITER

#else

    /* Current control disabled: enable PWM current limiter by default */
    #ifndef ENABLE_PWM_CURRENT_LIMITER
        #define ENABLE_PWM_CURRENT_LIMITER
    #endif

#endif
// </editor-fold>  

/* --------------------------------------------------------------------------
*Phase Current Measurement (ADC) Enable
*
*When defined:
* - ADC channels for motor phase currents are triggered and converted.
* - Phase current values are available for observation and debugging.
*
*Note:
* - This firmware does NOT use phase current feedback in any control
    algorithm. Enabling this feature is intended for monitoring and
    evaluation purposes only.
* - When enabled, the ADC interrupt occurs after phase current
    conversion completion, which increases the latency between the
    ADC trigger and interrupt execution.
*-------------------------------------------------------------------------- */
#undef ENABLE_PHASE_CURRENT_MEASUREMENT 
    
/* --------------------------------------------------------------------------
 * GSP (Garuda Serial Protocol) - garuda-gui compatibility over UART1@115.2k.
 * MUTUALLY EXCLUSIVE with X2CScope diagnostics (both own UART1): when
 * ENABLE_GSP is defined, main.c skips DiagnosticsInit/StepMain and runs
 * GSP_Init/GSP_Service instead. Protocol + bridge live in gsp_ak.c/h
 * (#included by main.c - no MPLAB project changes needed).
 * GUI functions available: connect/info, live telemetry stream + charts,
 * start/stop/clear-fault, throttle source POT vs GUI slider.
 * -------------------------------------------------------------------------- */
#define ENABLE_GSP

/* Define INTERNAL_OPAMP_CONFIG to use internal op-amp outputs(default), 
 * Undefine INTERNAL_OPAMP_CONFIG to use external op-amp outputs from the 
   development board;Ensure the jumper resistors are modified on DIM  */
#define INTERNAL_OPAMP_CONFIG

/* Motor Selection : 1 = Sparrow Series APM 3115-Kv900 Brushless Motor
                     2 = Linix 36ZWN24 Brushless Motor 
                     3 = Alex 2807 Kv1300 Brushless Motor
                     4 = Tmotor U3 Kv700 Brushless Motor */
#define MOTOR  4
      
// </editor-fold>
    
// <editor-fold defaultstate="collapsed" desc="MOTOR SELECTION HEADER FILES ">
    
#if MOTOR == 1
    #include "sparrow_3115-kv900.h"
#elif MOTOR == 2
    #include "linix36zwn24.h"
#elif MOTOR == 3
    #include "alex_2807-kv1300.h"
#elif MOTOR == 4
    #include "tmotor_u3_kv700.h"
#else
    #include "sparrow_3115-kv900.h"
#endif
// </editor-fold> 

// <editor-fold defaultstate="expanded" desc="DEFINITIONS ">    
/** Board Parameters */
/* Peak measurement voltage of the board (unit : volts)*/
#define MC1_PEAK_VOLTAGE                75.9f   
/* Peak measurement current of the board (unit : amps)
   Peak Current  = Vref / (op-amp Gain * Rshunt) 
   where op-amp Gain = 8.3, Vref = 1.65V and Rshunt = 0.003ohm */
#define MC1_PEAK_CURRENT                22.0f //66.265f = Atomberg-modified board (8.3 gain / 3mOhm); 22.0f = STOCK board   
/* Nominal DC Bus Voltage required by the motor (unit : volts)*/ 
#define DC_LINK_VOLTAGE                 24.0f     

// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif	/* end of __MC1_USER_PARAMS_H */
