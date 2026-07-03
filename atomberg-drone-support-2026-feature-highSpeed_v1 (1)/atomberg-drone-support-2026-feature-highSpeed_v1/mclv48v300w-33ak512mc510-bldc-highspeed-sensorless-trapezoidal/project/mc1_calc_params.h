// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc1_calc_params.h
 *
 * @brief This file has definitions used in the application to run motor 1,
 *        calculated based on associated user parameter header file
 *        mc1_user_params.h.
 *
 * Component: BOARD
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

#ifndef __MC1_CALC_PARAMS_H
#define __MC1_CALC_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
#include <stdint.h>
 
#include "mc1_user_params.h"
#include "util.h"
#include "pwm.h"
#include "adc.h"
#include "clock.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/MACROS ">
#define MC1_LOOPTIME_SEC            (float)(1.0f/PWMFREQUENCY_HZ)
    
#define SPEED_LOOP_HZ               1000U      // 1 kHz speed loop
    
#define SPEED_LOOP_COUNTS           (uint16_t) (PWMFREQUENCY_HZ / SPEED_LOOP_HZ)

#define ADC_CURRENT_SCALE           (float)(MC1_PEAK_CURRENT/32768.0f)
    
#define ADC_VOLTAGE_SCALE           (float)(MC1_PEAK_VOLTAGE/MAX_ADC_COUNT)
    
#define MOTOR_BEMF_CONSTANT_ELEC    (float)(((((MOTOR_BEMF_CONSTANT_MECH/SQRT_3)*60.0f)/1000.0f)/(2.0f*M_PI))/POLE_PAIRS)  

/* Motor Peak Current per phase (unit : amps) */
#define NOMINAL_CURRENT_PEAK      (float) (NOMINAL_CURRENT_PHASE_RMS * M_SQRT2)

/* Maximum Speed (unit : RPM)*/
#define MAXIMUM_SPEED_RPM         (float)(DC_LINK_VOLTAGE * VELOCITY_CONSTANT_KV)
    
/* SPEED MULTIPLIER CALCULATION = (((FCY/2)*60)/(TIMER_PRESCALER*POLEPAIRS*6))  */
#define SPEED_MULTIPLIER     (uint32_t)(((float)(FCY/2.0f)/(float)(1.0f * POLE_PAIRS *6.0f))*(float)60.0f)     
        
/* User-configurable delay before allowing motor direction change (seconds) */
#define DIRECTION_CHANGE_DELAY_SEC     (1.0f)
/* Direction change delay converted to control loop counts */
#define DIRECTION_CHANGE_DELAY_COUNTS  ((uint32_t)(DIRECTION_CHANGE_DELAY_SEC / MC1_LOOPTIME_SEC))
   
/* Motor stall detect counts
 * sector transitions counts at 1Hz electrical 
 * Counts for sector transition at 1rpm.
 * If a Hall sector transition does not occur within these many counts,
 * Then the motor speed is less than 1 rpm hence stalled.
 */
#define MOTOR_STALL_DETECT_COUNTS   (uint32_t)(60.0f / (MC1_LOOPTIME_SEC * 6.0f * POLE_PAIRS))
    
/* Speed Reference Ramp rate (unit : RPM per Speed loop interval)  */
#define SPEED_REF_RAMP_VALUE      SPEED_REF_RAMP_RATE / (float) SPEED_LOOP_HZ 
    
/* Rotor Lock parameters*/
/* Locking Time in counts of PWM switching interval*/      
#define ROTOR_LOCK_TIME_COUNTS            ROTOR_LOCK_TIME_SEC/MC1_LOOPTIME_SEC
/* Locking Current, set to 10% of motors rated current, minimum 0.5A (unit : amps)*/ 
#define ROTOR_LOCK_CURRENT ((0.1f * NOMINAL_CURRENT_PEAK) < 0.5f ? 0.5f : (0.1f * NOMINAL_CURRENT_PEAK))
/* Max PI output voltage during rotor lock current control 
 *   V_line = I_lock × R_phase × 2 */
#define ROTOR_LOCK_MAX_VOLTAGE   (ROTOR_LOCK_CURRENT * MOTOR_PER_PHASE_RESISTANCE * 2.0f)
    
/* Forced commutation startup parameters */
/* Macro for maximum startup current, minimum 1A */
#define MAX_STARTUP_CURRENT_PK ((1.25f * ROTOR_LOCK_CURRENT) < 1.0f ? 1.0f : (1.25f * ROTOR_LOCK_CURRENT))
/* Back EMF peak at final startup frequency */
#define MAX_STARTUP_BEMF_VOLTAGE_PK (1.0 * M_SQRT2)
/* Macro for Starting electrical frequency (Hz) for startup BLDC ramp */
#define INITIAL_STARTUP_FREQ_HZ   1.0   
/* Macro for Final startup electrical frequency (Hz) 
  For a Back EMF voltage of 1V, and E = V/2*/
#define FINAL_STARTUP_FREQ_HZ   (MAX_STARTUP_BEMF_VOLTAGE_PK / (2.0 * M_PI * MOTOR_BEMF_CONSTANT_ELEC))
/* V/F ratio */
#define V_F_RATIO ((2.0 * MAX_STARTUP_BEMF_VOLTAGE_PK + (2.0 * MOTOR_PER_PHASE_RESISTANCE * MAX_STARTUP_CURRENT_PK))/FINAL_STARTUP_FREQ_HZ)
/* Minimum startup Duty */    
#define MIN_STARTUP_DUTY 0.05 /* 5% */
/* Startup Frequency Ramp Step Rate */
#define STARTUP_FREQ_RAMP_STEP (((INITIAL_STARTUP_FREQ_HZ * FINAL_STARTUP_FREQ_HZ) / STARTUP_RAMP_TIME_S) * MC1_LOOPTIME_SEC)
/* Target startup speed */   
#define TARGET_STARTUP_SPEED_RPM ((60.0 * FINAL_STARTUP_FREQ_HZ) / POLE_PAIRS)
   
/* Blanking time for BEMF based sector estimation to avoid false detection
 * The BEMF contains a spike immediately after switching. */
/* Startup blanking time 
 * 75% of the commutation sector time at the final electrical frequency is blanked */
#define OL_BLANKING_TIME (uint16_t)((0.75f * (1.0f / (12.0f * FINAL_STARTUP_FREQ_HZ))) / MC1_LOOPTIME_SEC)
/* Closed-loop blanking time 
 * 20% of the commutation sector time at the maximum electrical frequency is blanked, minimum value is 1 */
#define CL_BLANKING_TIME (uint16_t)(((0.2f * 60.0 )/ (MC1_LOOPTIME_SEC * 6.0 * MAXIMUM_SPEED_RPM * POLE_PAIRS)) + 1.0)

/* Comparator reference for PWM Current Limit PCI from DC Bus current, set to 125% of rotor locking current */ 
#define CMP_REF_OP_IBUS       (uint16_t)(((MAX_STARTUP_CURRENT_PK * 2047.0f)/MC1_PEAK_CURRENT)+HALF_ADC_COUNT)

/* Overcurrent fault limit(software) - phase current (unit : amps)
    1.5 times the nominal peak current */
#define OC_FAULT_LIMIT_PHASE    1.5f*NOMINAL_CURRENT_PEAK

/* PWM period in terms of Timer counts */    
#define PWM_PERIOD_TIMER_COUNTS (uint32_t)(MC1_LOOPTIME_SEC*StandardClockFrequencyGet)
    
/* BEMF RC filter rise time in microseconds */
#define BEMF_RC_RISE_TIME_MICROSEC     2.0f
/* BEMF RC rise time in terms of PWM clock period */
#define BEMF_RC_RISE_TIME_COUNTS ((uint32_t)(BEMF_RC_RISE_TIME_MICROSEC * 16.0f * PWM_CLOCK_MHZ))
    
#define POT_RAMP_TIME_SEC       2.0f   /* Time to ramp 0 ? full scale */  
#define POT_RAMP_COUNTS_PER_SEC (MAX_ADC_COUNT / POT_RAMP_TIME_SEC)
#define POT_RAMP_STEP_PER_ISR (float)(POT_RAMP_COUNTS_PER_SEC * MC1_LOOPTIME_SEC)    
// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif	/* end of __MC1_CALC_PARAMS_H */
