// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file diagnostics.c
 *
 * @brief This module implements the diagnostics routine using X2C Scope
 * 
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: DIAGNOSTICS
 *
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">

/*******************************************************************************
* SOFTWARE LICENSE AGREEMENT
* 
* � [2024] Microchip Technology Inc. and its subsidiaries
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

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include "garuda_config.h"

#if FEATURE_X2CSCOPE

#include <stdint.h>

#include "X2CScope.h"
#include "../hal/uart1.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/CONSTANTS ">

#define X2C_DATA __attribute__((section("x2cscope_data_buf")))
#define X2C_BAUDRATE_DIVIDER 54 /** 100M/(16*54) = 115.7 kbps */
#define X2C_BUFFER_SIZE 4900
X2C_DATA static uint8_t X2C_BUFFER[X2C_BUFFER_SIZE];
    /*
     * UART uses Clock Module #8 (100MHz)
     * baud rate = 100MHz/16/(1+baudrate_divider) for highspeed = false
     * baud rate = 100MHz/4/(1+baudrate_divider) for highspeed = true
     * 
     * 6250kbaud => 0 (for DIAG_BAUDRATE_DIVIDER with highspeed = false)
     * 3125kbaud => 1
     * 2083kbaud => 2
     * 1562kbaud => 3
     *  1250kbaud => 4
     *  1041kbaud => 5
     *  892kbaud => 6
     *  781kbaud => 7
     *  694kbaud => 8
     *  625kbaud => 9
     *  568kbaud => 10
     *  520.8kbaud => 11
     *  480.76kbaud => 12
     *  446.4kbaud => 13
     *  416.6kbaud => 14
     *  390.62kbaud => 15
     *  367.6kbaud => 16
     *  347.2kbaud => 17
     *  328.9kbaud => 18
     *  115.7kbaud => 54
     *   57.87kbaud => 107
     */
// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: DiagnosticsInit() </B>
*
* @brief Function to initialize the UART1 and X2C Scope
*        
* @param none.
* @return none.
* 
* @example
* <CODE> DiagnosticsInit(); </CODE>
*
*/
void DiagnosticsInit(void)
{
    UART1_InterruptReceiveDisable();
    UART1_InterruptReceiveFlagClear();
    UART1_InterruptTransmitDisable();
    UART1_InterruptTransmitFlagClear();
    UART1_Initialize();
    UART1_BaudRateDividerSet(X2C_BAUDRATE_DIVIDER); 
    UART1_SpeedModeStandard();
    UART1_ModuleEnable();  
    
    X2CScope_Init();
}
/**
* <B> Function: DiagnosticsStepMain() </B>
*
* @brief Function to send/receive data from X2C Scope
*        
* @param none.
* @return none.
* 
* @example
* <CODE> DiagnosticsStepMain(); </CODE>
*
*/
void DiagnosticsStepMain(void)
{
    X2CScope_Communicate();
}
/**
* <B> Function: DiagnosticsStepIsr() </B>
*
* @brief Function to update variables to latest sample
*        
* @param none.
* @return none.
* 
* @example
* <CODE> DiagnosticsStepIsr(); </CODE>
*
*/
void DiagnosticsStepIsr(void)
{
    X2CScope_Update();
}
/**
* <B> Function: X2CScope_sendSerial() </B>
*
* @brief Function to write data through UART transmission
*        
* @param data write for UART transmission.
* @return none.
* 
* @example
* <CODE> X2CScope_sendSerial(data); </CODE>
*
*/
static void X2CScope_sendSerial(uint8_t data)
{
    UART1_DataWrite(data);
}
/**
* <B> Function: X2CScope_receiveSerial() </B>
*
* @brief Function to read data received through UART
*        
* @param none.
* @return data received through UART receive register.
* 
* @example
* <CODE> X2CScope_receiveSerial(); </CODE>
*
*/
static uint8_t X2CScope_receiveSerial()
{
    return UART1_DataRead();
}
/**
* <B> Function: X2CScope_isReceiveDataAvailable() </B>
*
* @brief Function to check the receive buffer of UART
*        
* @param none.
* @return status bit to read the receive buffer in UART.
* 
* @example
* <CODE> X2CScope_isReceiveDataAvailable(); </CODE>
*
*/
static uint8_t X2CScope_isReceiveDataAvailable()
{
    return UART1_IsReceiveBufferDataReady();
}
/**
* <B> Function: X2CScope_isSendReady() </B>
*
* @brief Function to check the transmit buffer of UART is empty or full
*        
* @param none.
* @return status bit to check transmit buffer is empty or full
* 
* @example
* <CODE> X2CScope_isSendReady(); </CODE>
*
*/
static uint8_t X2CScope_isSendReady()
{
    return !UART1_StatusBufferFullTransmitGet();
}
/**
* <B> Function: X2CScope_Init() </B>
*
* @brief Function to initialize X2C Scope : to hook UART functions,
* Size of X2C buffer
*        
* @param none.
* @return none.
* 
* @example
* <CODE> X2CScope_Init(); </CODE>
*
*/
void X2CScope_Init(void)
{
    X2CScope_HookUARTFunctions(
        X2CScope_sendSerial,
        X2CScope_receiveSerial,
        X2CScope_isReceiveDataAvailable,
        X2CScope_isSendReady);
    X2CScope_Initialise(X2C_BUFFER,sizeof(X2C_BUFFER));
}

#endif /* FEATURE_X2CSCOPE */

// </editor-fold>