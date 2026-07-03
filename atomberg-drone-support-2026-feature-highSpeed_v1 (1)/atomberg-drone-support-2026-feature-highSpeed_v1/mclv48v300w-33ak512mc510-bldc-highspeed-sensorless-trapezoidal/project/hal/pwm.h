// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file pwm.h
 *
 * @brief This header file lists the functions and definitions to configure the 
 * PWM Module
 * 
 * Definitions in this file are for dsPIC33AK512MC510
 *
 * Component: PWM
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

#ifndef _PWM_H
#define _PWM_H

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <xc.h>
#include <stdint.h>
        
#include "clock.h"
#include "mc1_calc_params.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/CONSTANTS ">
        
/* Define to enable PWM Fault PCI*/         
#undef ENABLE_PWM_FAULT
/* Define to enable PWM Current Limit PCI*/ 
#undef CYCLE_BY_CYCLE_CURRENT_LIMIT
        
#define PWM_PDC1                    PG1DCbits.DC
#define PWM_PDC2                    PG2DCbits.DC
#define PWM_PDC3                    PG3DCbits.DC
        
#define PWM_PHASE1                  PG1PHASEbits.PHASE 
#define PWM_PHASE2                  PG2PHASEbits.PHASE
#define PWM_PHASE3                  PG3PHASEbits.PHASE  
        
#define PCI_FAULT_ACTIVE_STATUS      PG1STATbits.FLTACT        
#define _PWMInterrupt               _PWM1Interrupt
#define ClearPWMIF()                _PWM1IF = 0 
#define EnablePWMIF()               _PWM1IE = 1
#define DisablePWMIF()              _PWM1IE = 0
              
/*Specify PWM Switching Frequency in Hertz*/
#define PWMFREQUENCY_HZ                     40000
/*Specify PWM Module Clock in Mega Hertz*/
#define PWM_CLOCK_MHZ                       400   
/* Specify PWM module dead time in micro seconds*/
#define DEADTIME_MICROSEC                   0.75f
/*Dead time in terms of PWM clock period*/  
#define DEADTIME                            (uint32_t)(DEADTIME_MICROSEC*16*PWM_CLOCK_MHZ)
/* Loop Time in micro seconds*/
#define LOOPTIME_MICROSEC                   (MC1_LOOPTIME_SEC * 1000000.0f)
/*Loop time in terms of PWM clock period*/
#define LOOPTIME_TCY                        (uint32_t)((LOOPTIME_MICROSEC*16*PWM_CLOCK_MHZ)-16)
/*Specify ADC Triggering Point w.r.t PWM Output for sensing Analog Inputs*/ 
#define ADC_SAMPLING_POINT                  (uint32_t)(((LOOPTIME_MICROSEC-6)*16*PWM_CLOCK_MHZ)-16)
//#define ADC_SAMPLING_POINT2                  (uint32_t)((24*16*PWM_CLOCK_MHZ)-16)
        
/****Bootstrap Capacitor Charging Parameters*/
/*Specify bootstrap charging time in Seconds (mention at least 0.01Secs)*/
#define BOOTSTRAP_CHARGING_TIME_SECS        0.015
/*Specify bootstrap Capacitor Tickle Charge Time in Micro Seconds
 * Minimum Time = 1uSec and Maximum Time = 5uSec */
#define TICKLE_CHARGE_TIME_MICROSEC         1.0
/*Calculate Bootstrap charging time in number of PWM Half Cycles*/
#define BOOTSTRAP_CHARGING_COUNTS           (uint32_t)(BOOTSTRAP_CHARGING_TIME_SECS/MC1_LOOPTIME_SEC)
/*Calculate Bootstrap Capacitor Tickle Charge duty*/
#define TICKLE_CHARGE_DUTY                  (LOOPTIME_TCY - (uint32_t)(TICKLE_CHARGE_TIME_MICROSEC*16*PWM_CLOCK_MHZ))

#define MIN_DUTY            0
#define MAX_DUTY            (uint32_t) LOOPTIME_TCY - 1

/* CLPCI1 configuration:

- Latched current-limit PCI
- Inverted PCI input (active-low)
- No acceptance qualifier
*/
#define CLPCI1_CUR_LIMIT_CFG   (0x03001000U)
// </editor-fold>      

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
        
void InitPWMGenerators(void);  
void InitPWMGenerator1 (void);
void InitPWMGenerator2 (void);
void InitPWMGenerator3 (void);
void InitDutyPWM123Generators(void);
void InitPWMGenerators(void);   
void ChargeBootstrapCapacitors(void);
// </editor-fold>
        
#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of PWM_H


