// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file main.c
 *
 * @brief This is the main entry to the application.
 *
 * Component: APPLICATION
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

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

#include <xc.h>

#include "board_service.h"
#include "diagnostics.h"

#include "mc1_service.h" 
#include "mc1_init.h"
#include "mc1_user_params.h"
#ifdef ENABLE_GSP
#include "gsp_ak.h"
#endif


// </editor-fold>

/**
* <B> Function: int main (void)  </B>
*
* @brief main() function,entry point of the application.
*
*/
int main (void)
{
    InitOscillator();
    SetupGPIOPorts();
    
    /* Initialize Peripherals */
    HAL_InitPeripherals();
    
#if defined(ENABLE_DIAGNOSTICS) && !defined(ENABLE_GSP)
    /* Diagnostics using X2CScope Plugin */
    DiagnosticsInit();
#endif
#ifdef ENABLE_GSP
    /* garuda-gui serial protocol on UART1 (replaces X2CScope) */
    GSP_Init();
#endif

    MCAPP_MC1ServiceInit(); 
    
    
    while(1)
    {
        
#if defined(ENABLE_DIAGNOSTICS) && !defined(ENABLE_GSP)
        DiagnosticsStepMain();
#endif
#ifdef ENABLE_GSP
        GSP_Service();
#endif
        
    }
    
    return 0;
}

#ifdef ENABLE_GSP
/* Single-translation-unit include: the GSP engine+bridge compiles HERE.
 * gsp_ak.c is also registered in the MPLAB project for tree visibility,
 * but its standalone compile is an empty object (see the
 * GSP_AK_IMPLEMENTATION guard inside it). */
#define GSP_AK_IMPLEMENTATION
#include "gsp_ak.c"
#endif

// </editor-fold>

