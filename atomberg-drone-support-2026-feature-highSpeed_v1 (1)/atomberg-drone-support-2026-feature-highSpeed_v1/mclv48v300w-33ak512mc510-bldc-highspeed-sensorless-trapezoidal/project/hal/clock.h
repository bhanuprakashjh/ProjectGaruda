// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file clock.h
 *
 * @brief This header file lists the functions and definitions - to configure 
 * and enable Clock/Oscillator/PLL Module and its features
 * 
 * Definitions in this file are for dsPIC33AK512MC510
 * 
 * Component: CLOCK/OSCILLATOR
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

#ifndef _CLOCK_H
#define _CLOCK_H

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
        
// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
        
#include <xc.h>
#include <stdint.h>
        
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="DEFINITIONS/MACROS ">
// Instruction cycle frequency (Hz) - 200,000,000 Hz
#define FCY                     200000000UL 
// Standard Clock Frequency  (Hz) - 100,000,000 Hz
#define StandardClockFrequencyGet   100000000UL 
// </editor-fold> 
        
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
        
void InitOscillator(void);

// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
    
#endif      // end of __CLOCK_H

