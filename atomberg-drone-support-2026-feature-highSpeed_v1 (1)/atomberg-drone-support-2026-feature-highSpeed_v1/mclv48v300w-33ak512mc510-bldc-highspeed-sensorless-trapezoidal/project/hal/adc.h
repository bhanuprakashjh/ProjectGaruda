// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file adc.h
 *
 * @brief This header file lists the functions and definitions - to configure 
 * and enable ADC Module and its features 
 * 
 * Definitions in this file are for dsPIC33AK512MC510
 *
 * Component: ADC
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

#ifndef _ADC_H
#define _ADC_H
        
// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
        
#include <xc.h>
#include <stdint.h>
        
#include "mc1_user_params.h"
        
// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
        
// <editor-fold defaultstate="expanded" desc="DEFINITIONS/CONSTANTS ">
/*Maximum count in 12-bit ADC*/
#define MAX_ADC_COUNT     4096.0f
#define HALF_ADC_COUNT    2048    
        
/*Converting 2^11 format to 2^15 format  */
#define MC1_ADCBUF_IA    (int16_t)((AD1CH2DATA - HALF_ADC_COUNT)<<4)
#define MC1_ADCBUF_IB    (int16_t)((AD2CH1DATA - HALF_ADC_COUNT)<<4)
#define MC1_ADCBUF_IC    (int16_t)((AD3CH1DATA- HALF_ADC_COUNT)<<4) 
#define ADCBUF_IBUS      (int16_t)((AD3CH0DATA- HALF_ADC_COUNT)<<4) 
        
#define MC1_ADCBUF_POT   (int16_t)AD2CH2DATA
#define MC_ADCBUF_VDC    (int16_t)AD3CH2DATA 

#define ADCBUF_INV_A_VA           AD1CH0DATA
#define ADCBUF_INV_A_VB           AD1CH1DATA
#define ADCBUF_INV_A_VC           AD2CH0DATA  

#define EnableSamplingBEMF() _AD1CH1IE = 1 
#define DisableSamplingBEMF() _AD1CH1IE = 0 
#define ADC_PhaseVoltageSampleISR _AD1CH1Interrupt 
#define ClearADC_PhaseVoltageSampleFlag()     _AD1CH1IF = 0
        
/* ADC1 Interrupt */
 #ifndef ENABLE_PHASE_CURRENT_MEASUREMENT       
#define MC1_EnableADCInterrupt()            _AD3CH2IE = 1
#define MC1_DisableADCInterrupt()           _AD3CH2IE = 0
#define MC1_ADC_INTERRUPT                   _AD3CH2Interrupt  
#define MC1_ClearADCIF()                    _AD3CH2IF = 0 
#define MC1_ClearADCIF_ReadADCBUF()         MC_ADCBUF_VDC
#else
#define MC1_EnableADCInterrupt()            _AD1CH2IE = 1
#define MC1_DisableADCInterrupt()           _AD1CH2IE = 0
#define MC1_ADC_INTERRUPT                   _AD1CH2Interrupt  
#define MC1_ClearADCIF()                    _AD1CH2IF = 0 
#define MC1_ClearADCIF_ReadADCBUF()         MC1_ADCBUF_IA
#endif        
// </editor-fold>
        
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
        
void InitializeADCs(void);

// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
    
#endif      // end of __ADC_H

