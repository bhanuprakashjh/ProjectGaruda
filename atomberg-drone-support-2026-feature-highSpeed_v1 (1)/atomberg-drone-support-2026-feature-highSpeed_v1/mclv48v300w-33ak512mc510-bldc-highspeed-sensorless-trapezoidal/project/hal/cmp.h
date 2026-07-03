// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file cmp.h
 *
 * @brief This header file lists the functions and definitions - to configure 
 * and enable DAC Module and its features 
 * 
 * Definitions in this file are for dsPIC33AK512MC510
 *
 * Component: DAC
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

#ifndef CMP_H
#define	CMP_H

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
        
#include <xc.h>
#include <stdint.h>

// </editor-fold>

#ifdef	__cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
            
void InitializeCMPs(void);
void CMP3_ModuleEnable(bool);
void CMP3_ReferenceSet(uint16_t );

/**
 * Enable Comparator3 module.
 * Summary: Enable Comparator3 module.
 * @example
 * <code>
 * EnableComparatorModule3();
 * </code>
 */
inline static void EnableComparatorModule3 (void) {DAC3CONbits.DACEN = 1;}
/**
 * Disable Comparator3 module.
 * Summary: Disable Comparator3 module.
 * @example
 * <code>
 * DisableComparatorModule3();
 * </code>
 */
inline static void DisableComparatorModule3 (void) {DAC3CONbits.DACEN = 0;}

// </editor-fold> 


#ifdef	__cplusplus
}
#endif

#endif	/* CMP_H */

