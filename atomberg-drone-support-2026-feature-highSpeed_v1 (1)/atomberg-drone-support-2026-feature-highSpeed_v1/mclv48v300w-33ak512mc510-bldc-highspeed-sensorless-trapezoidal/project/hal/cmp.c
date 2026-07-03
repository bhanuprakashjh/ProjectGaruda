// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file cmp.c
 *
 * @brief This module configures and enables the DAC Module 
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

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "cmp.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">
static void CMP3_Initialize(void);

// </editor-fold> 

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
/**
* <B> Function: InitializeCMPs() </B>
*
* @brief Function initializes and enables the CMP Module
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitializeCMPs(); </CODE>
*
*/
void InitializeCMPs (void)
{

//    uint32_t *FPDMDACaddress;
//    uint32_t FPDMDACdata,POSINLADJ,NEGINLADJ,DNLADJ;
//    
//    /** Start of DAC Calibration **/
//    /* The calibration register FPDMDAC is located in Flash at 0x7F20E0 with the POSINLADJ, NEGINLADJ
//       and DNLADJ bit fields. The location should be copied and written to the corresponding bit fields in
//       the DACCTRL1 SFR at start-up.    */
//    /* Copying data from the given address in the Flash */
//    FPDMDACaddress = (uint32_t*)(0x7F20E0);
//    FPDMDACdata = (uint32_t)(*FPDMDACaddress);
//    /* POSINLADJ available at bit field 23:16 */
//    POSINLADJ = (FPDMDACdata & 0x00FF0000)>>16;
//    /* NEGINLADJ available at bit field 15:8 */
//    NEGINLADJ = (FPDMDACdata & 0x0000FF00)>>8;
//    /* DNLADJ available at bit field 7:0 */
//    DNLADJ = (FPDMDACdata & 0x000000FF);
//	
//    /* Initialize DACCTRL1 REGISTER */
//    DACCTRL1 = 0;
//    /* Writing the calibration data copied from calibration register to DAC Control Register */
//    DACCTRL1bits.NEGINLADJ = NEGINLADJ;
//    DACCTRL1bits.DNLADJ = DNLADJ;
//    DACCTRL1bits.POSINLADJ = POSINLADJ;
    /** End of DAC calibration **/

    /* Common DAC Module Enable bit
        1 = Enables DAC modules
        0 = Disables DAC modules */
    DACCTRL1bits.ON = 0;
    /* DAC Stop in Idle Mode bit
        1 = Discontinues module operation when device enters Idle mode
        0 = Continues module operation in Idle mode */
    DACCTRL1bits.SIDL = 0;
    /* Comparator Filter Clock Divider bits
        111 = Divide by 8
             ......
        000 = 1x */
    DACCTRL1bits.FCLKDIV = 0b111;
    
    /* Initialize DACCTRL2 REGISTER */
    DACCTRL2 = 0;
    /* Transition Mode Duration bits
        The value for TMODTIME<9:0> should be less than the SSTIME<9:0>.*/
    DACCTRL2bits.TMODTIME = 0;
    
    /* Time from Start of Transition Mode till Steady-State Filter is Enabled 
        The value for SSTIME<9:0> should be greater than the TMODTIME<9:0>.*/
    DACCTRL2bits.SSTIME = 0;

    /* Function to initialize Comparator3 Module */
    CMP3_Initialize();   
}

/**
* <B> Function: InitializeCMP3() </B>
*
* @brief Function to initialize CMP3 module 
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitializeCMP3(); </CODE>
*
*/
void CMP3_Initialize (void)
{
    /* Initialize DAC3CON REGISTER */
    DAC3CON = 0;
    /* Individual DACx Module Enable bit
        1 = Enables DACx module
        0 = Disables DACx module and disables FSCM clock */
    DAC3CONbits.DACEN = 0;
    /* Interrupt Mode select bits
        11 = Generates an interrupt on either a rising or falling edge detect
        10 = Generates an interrupt on a falling edge detect
        01 = Generates an interrupt on a rising edge detect
        00 = Interrupts are disabled */
    DAC3CONbits.IRQM = 0;
    /* DACx Output Buffer Enable bit
        1 = DACx analog voltage is connected to the DACOUT1 pin
        0 = DACx analog voltage is not connected to the DACOUT1 pin */
    DAC3CONbits.DACOEN = 0; 
    /* DACx Leading-Edge Blanking bits
        These register bits specify the blanking period for the comparator 
        following changes to the DAC output during Change-of-State (COS) for the
        input signal selected by the HCFSEL<3:0> bits */
    DAC3CONbits.TMCB = 0;

    /* Initialize DAC3CMP REGISTER */
    DAC3CMP = 0;
    /* Comparator Hysteresis Polarity Select bit
        1 = Hysteresis is applied to the falling edge of comparator output
        0 = Hysteresis is applied to the rising edge of comparator output */
    DAC3CMPbits.HYSPOL = 0; 
    /* Comparator Hysteresis Select bits
        0b11 = 45 mv hysteresis, 0b10 = 30 mv hysteresis
        0b01 = 15 mv hysteresis,0b00 = No hysteresis is selected */
    DAC3CMPbits.HYSSEL = 0b11;     
    /* Comparator Blank Enable bit
    1 = Enables the analog comparator output to be blanked 
    0 = Disables the blanking signal to the analog comparator; */
    DAC3CMPbits.CBE = 0;
   /* Comparator Digital Filter Enable bit
        1 = Digital filter is enabled
        0 = Digital filter is disabled */
    DAC3CMPbits.FLTREN = 0;
    /* Comparator Status bits -The current state of the comparator output 
        including the CMPPOL selection */
    DAC3CMPbits.CMPSTAT = 0;
    /* Comparator Output Polarity Control bit
        1 = Output is inverted
        0 = Output is non-inverted */
    DAC3CMPbits.CMPPOL = 0;
    /* Comparator Positive Input Source Select bits - Refer Data sheet for selection (CMP3A)*/
    DAC3CMPbits.INPSEL = 0; 
    /* Comparator Negative Input Source Select bits - Refer Data sheet for selection (DAC3)*/
    DAC3CMPbits.INNSEL = 0;    
    /* Initialize DAC3DAT REGISTER */
    /* DACx Data bits - In Hysteretic mode, Slope Generator mode and 
        Triangle mode, this register specifies the low data value
        and/or limit for the DACx module. */
    DAC3DAT = 0;
   
    /* Initialize DAC3SLPCON REGISTER */
    DAC3SLPCON = 0;
    /* Hysteretic Comparator Function Input Select bits
        The selected input signal controls the switching between the 
        DACx high limit (DACxDATH) and the DACx low limit (DACxDATL) as the 
        data source for the PDM DAC */
    DAC3SLPCONbits.HCFSEL = 0;
    /* Slope Stop A Signal Select bits
        The selected Slope Stop A signal is logically OR?d with the selected 
        Slope Stop B signal to terminate the slope function.*/
    DAC3SLPCONbits.SLPSTOPA = 0 ;
    /* Slope Stop B Signal Select bits
        The selected Slope Stop B signal is logically OR?d with the selected 
        Slope Stop A signal to terminate the slope function.*/
    DAC3SLPCONbits.SLPSTOPB = 0 ;  
    /* Slope Start Signal Select bits */
    DAC3SLPCONbits.SLPSTRT = 0 ;    
    /* Slope Function Enable/On bit
        1 = Enables slope function
        0 = Disables slope function */
    DAC3SLPCONbits.SLOPEN = 0;
    /* Hysteretic Mode Enable bit
        1 = Enables Hysteretic mode for DACx
        0 = Disables Hysteretic mode for DACx 
        HME mode requires the user to disable the slope function (SLOPEN = 0).*/
    DAC3SLPCONbits.HME = 0 ;
    /* Triangle Wave Mode Enable bit(2)
        1 = Enables Triangle Wave mode for DACx
        0 = Disables Triangle Wave mode for DACx 
        TWME mode requires the user to enable the slope function (SLOPEN = 1).*/
    DAC3SLPCONbits.TWME = 0 ;  
    /* Positive Slope Mode Enable bit
        1 = Slope mode is positive (increasing)
        0 = Slope mode is negative (decreasing) */
    DAC3SLPCONbits.PSE = 0 ;   
    
    /* Initialize DAC3SLPDAT REGISTER */
    /* Slope Ramp Rate Value bits */
    DAC3SLPDAT = 0;
}


/**
* <B> Function: CMP3_ReferenceSet(uint16_t) </B>
*
* @brief Function to write data to the DACDAT register
*        
* @param DAC reference value.
* @return none.
* 
* @example
* <CODE> CMP3_ReferenceSet(reference); </CODE>
*
*/
void CMP3_ReferenceSet(uint16_t data)
{
    /** Initialize DAC3DATH REGISTER */
    /** DACx Data bits - This register specifies the high DACx data value. */
    DAC3DATbits.DACDAT = data;
}

/**
* <B> Function: CMP3_ModuleEnable() </B>
*
* @brief Function to enable/disable DAC3 Module and its output to DAC3OUT pin.
* Ensure that other Slave DACx are disabled, before setting DAC3CONLbits.DACOEN
* to '1' allowing DACx analog voltage to the DACOUT1 pin.
*        
* @param none.
* @return none.
* 
* @example
* <CODE> CMP3_ModuleEnable(); </CODE>
*
*/
void CMP3_ModuleEnable( bool state)
{
    if (state == true)
    {
        /** Individual DACx Module Enable bit
            1 = Enables DACx module
            0 = Disables DACx module and disables FSCM clock */
        DAC3CONbits.DACEN = 1;
        /** Common DAC Module Enable bit
            1 = Enables DAC modules
            0 = Disables DAC modules */
        DACCTRL1bits.ON = 1;
    }
    else
    {
        /** DACx Output Buffer Enable bit
            1 = DACx analog voltage is connected to the DACOUT1 pin
            0 = DACx analog voltage is not connected to the DACOUT1 pin */
        DAC3CONbits.DACOEN = 0;
        /** Individual DACx Module Enable bit
            1 = Enables DACx module
            0 = Disables DACx module and disables FSCM clock */
        DAC3CONbits.DACEN = 0;
        /** Common DAC Module Enable bit
            1 = Enables DAC modules
            0 = Disables DAC modules */
        DACCTRL1bits.ON = 0;
    }
}
// </editor-fold>