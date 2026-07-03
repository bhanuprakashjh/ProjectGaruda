// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file port_config.h
 *
 * @brief This header file lists the functions and definitions for initializing 
 * GPIO pins as analog/digital,input or output etc. Also to PPS functionality to 
 * Re-mappable input or output pins.
 * 
 * Definitions in this file are for dsPIC33AK512MC510
 *
 * Component: PORTS
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

#ifndef _PORTCONFIG_H
#define _PORTCONFIG_H

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <xc.h>

// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatibility
    extern "C" {
#endif
       
// <editor-fold defaultstate="expanded" desc="DEFINITIONS ">

/* Digital I/O definitions */
        
/*Push buttons*/
/* SW1 : DIM:034 - PIN #03 : CVDAN12/RP13/RA12 */
#define SW1                   PORTAbits.RA12
/* SW2 : DIM:036 - PIN #02 : CVDTX20/RP66/RE1  */
#define SW2                   PORTEbits.RE1
        
/* Used as START/STOP button of Motor */
#define BUTTON_START_STOP              SW1
/* Used as Direction Change button of Motor */
#define BUTTON_DIRECTION_CHANGE        SW2	

/* Debug LEDs */
/* LED1(LD2) : DIM:030 - PIN #10 : CVDTX21/RP67/RE2 */
#define LED1                    LATEbits.LATE2
/* LED2(LD3) : DIM:032 - PIN #06 : CVDTX22/RP68/RE3 */
#define LED2                    LATEbits.LATE3

#define DEBUG_EVTC_DC_ADC_ISR LATBbits.LATB4
#define DEBUG_VC LATGbits.LATG1
#define DEBUG_EVTA_SOC LATFbits.LATF3
#define DEBUG_SCCP LATCbits.LATC12
// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
        
void SetupGPIOPorts(void);
void OpampConfig (void);
void MapGPIOHWFunction (void);

// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif

#endif      // end of __PORT_CONFIG_H
