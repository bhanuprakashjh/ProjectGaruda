// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file board_service.c
 *
 * @brief This module implements the board service routines 
 * 
 * Definitions in this file are for dsPIC33AK512MC510
 *
 * Component: BOARD SERVICE
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
#include "port_config.h"
#include "board_service.h"
#include "mc1_user_params.h"
#include "mc1_calc_params.h"
#include "adc.h"
#include "pwm.h"
#include "cmp.h"
#include "delay.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLES">

BUTTON_T buttonStartStop;
BUTTON_T buttonDirectionChange;

uint16_t boardServiceISRCounter = 0;

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

static void ButtonGroupInitialize(void);
static void ButtonScan(BUTTON_T * ,bool);

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
/**
* <B> Function: IsPressed_Button1() </B>
*
* @brief Function monitors the actuation of push button
*        
* @param none.
* @return boolean for button actuation.
* 
* @example
* <CODE> IsPressed_Button1(); </CODE>
*
*/
bool IsPressed_Button1(void)
{
    if(buttonStartStop.status)
    {
        buttonStartStop.status = false;
        return true;
    }
    else
    {
        return false;
    }
}
/**
* <B> Function: IsPressed_Button2() </B>
*
* @brief Function monitors the actuation of push button
*        
* @param none.
* @return boolean for button actuation.
* 
* @example
* <CODE> IsPressed_Button2(); </CODE>
*
*/
bool IsPressed_Button2(void)
{
    if (buttonDirectionChange.status)
    {
        buttonDirectionChange.status = false;
        return true;
    }
    else
    {
        return false;
    }
}

/**
* <B> Function: BoardServiceStepIsr() </B>
*
* @brief Function to increment the board service routine counter
*        
* @param none.
* @return none.
* 
* @example
* <CODE> BoardServiceStepIsr(); </CODE>
*
*/
void BoardServiceStepIsr(void)
{
    if (boardServiceISRCounter <  BOARD_SERVICE_TICK_COUNT)
    {
        boardServiceISRCounter += 1;
    }
}

/**
* <B> Function: BoardService() </B>
*
* @brief Function to reset the board service routine counter 
*        
* @param none.
* @return none.
* 
* @example
* <CODE> BoardService(); </CODE>
*
*/
void BoardService(void)
{
    if (boardServiceISRCounter ==  BOARD_SERVICE_TICK_COUNT)
    {
        /* Button scanning loop for Button 1 to start Motor A */
        ButtonScan(&buttonStartStop,BUTTON_START_STOP);

        /* Button scanning loop for SW2 to enter into filed
            weakening mode */
        ButtonScan(&buttonDirectionChange,BUTTON_DIRECTION_CHANGE);

        boardServiceISRCounter = 0;
    }
}

/**
* <B> Function: BoardServiceInit() </B>
*
* @brief Function to initialize the board service routine
*        
* @param none.
* @return none.
* 
* @example
* <CODE> BoardServiceInit(); </CODE>
*
*/
void BoardServiceInit(void)
{
    ButtonGroupInitialize();
    boardServiceISRCounter = BOARD_SERVICE_TICK_COUNT;
}

/**
* <B> Function: ButtonScan() </B>
*
* @brief Function to monitor the push button actuation with de-bounce delay
*        
* @param none.
* @return none.
* 
* @example
* <CODE> ButtonScan(); </CODE>
*
*/
void ButtonScan(BUTTON_T *pButton,bool button) 
{
    if (button == true) 
    {
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT) 
        {
            pButton->debounceCount--;
            pButton->state = BUTTON_DEBOUNCE;
        }
    } 
    else 
    {
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT) 
        {
            pButton->state = BUTTON_NOT_PRESSED;
        } 
        else 
        {
            pButton->state = BUTTON_PRESSED;
            pButton->status = true;
        }
        pButton->debounceCount = 0;
    }
}

/**
* <B> Function: ButtonGroupInitialize() </B>
*
* @brief Function to initialize the push button states
*        
* @param none.
* @return none.
* 
* @example
* <CODE> ButtonGroupInitialize(); </CODE>
*
*/
void ButtonGroupInitialize(void)
{
    buttonStartStop.state = BUTTON_NOT_PRESSED;
    buttonStartStop.debounceCount = 0;
    buttonStartStop.state = false;

    buttonDirectionChange.state = BUTTON_NOT_PRESSED;
    buttonDirectionChange.debounceCount = 0;
    buttonDirectionChange.state = false;
}

/**
* <B> Function: HAL_InitPeripherals() </B>
*
* @brief Function to initialize the peripherals(Op-Amp, ADC, CMP, DAC and PWM)
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_InitPeripherals(); </CODE>
*
*/
void HAL_InitPeripherals(void)
{                
#ifdef INTERNAL_OPAMP_CONFIG    
    OpampConfig();
#endif
    
    InitializeADCs();
    
#ifdef ENABLE_PWM_CURRENT_LIMITER    
    InitializeCMPs();  
    CMP3_ReferenceSet(CMP_REF_OP_IBUS);
    CMP3_ModuleEnable(true);
    
    HAL_MC1EnableCurrentLimitDACRef();
#else 
    HAL_MC1DisableCurrentLimitDACRef();
#endif

    /*400ms POR delay for IBUS_EXT signal coming from MCP651S in Dev Board*/
    __delay_ms(400);
    
    /* Make sure ADC does not generate interrupt while initializing parameters*/
    MC1_DisableADCInterrupt();  
    
    InitPWMGenerators(); 
    
    /*Clearing and enabling PWM Interrupt to handle PCI Fault*/
    ClearPWMIF();
    EnablePWMIF();
    
    SCCP2_Timer_Initialize();
    
}

/**
* <B> Function: HAL_ResetPeripherals() </B>
*
* @brief Function to reset the clear ADC interrupt and disable PWM outputs
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_ResetPeripherals(); </CODE>
*
*/
void HAL_ResetPeripherals(void)
{
    MC1_ClearADCIF();
    MC1_EnableADCInterrupt();
    HAL_MC1PWMDisableOutputs();
}

/**
* <B> Function: HAL_MC1PWMEnableOutputs() </B>
*
* @brief Function to enable the PWM outputs -  override is removed
* PWM generator controls the PWM outputs now
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC1PWMEnableOutputs(); </CODE>
*
*/
void HAL_MC1PWMEnableOutputs(void)
{
    /* Set PWM Duty Cycles */
    PWM_PDC3 = 0;
    PWM_PDC2 = 0;
    PWM_PDC1 = 0;
	
	/*  0 = PWM Generator provides data for the PWM3H pin */
    PG3IOCON2bits.OVRENH = 0; 
    /*  0 = PWM Generator provides data for the PWM3L pin */
    PG3IOCON2bits.OVRENL = 0; 
    
    /*  0 = PWM Generator provides data for the PWM2H pin */
    PG2IOCON2bits.OVRENH = 0;
    /*  0 = PWM Generator provides data for the PWM2L pin */
    PG2IOCON2bits.OVRENL = 0; 
    
    /*  0 = PWM Generator provides data for the PWM1H pin */
    PG1IOCON2bits.OVRENH = 0;  
    /*  0 = PWM Generator provides data for the PWM1L pin */
    PG1IOCON2bits.OVRENL = 0;     
}

/**
* <B> Function: HAL_MC1PWMDisableOutputs() </B>
*
* @brief Function to disable the PWM outputs -  override is activated
* OVRDAT<> register controls the PWM outputs now
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC1PWMDisableOutputs(); </CODE>
*
*/
void HAL_MC1PWMDisableOutputs(void)
{
    /* Set PWM Duty Cycles */
    PWM_PDC3 = 0;
    PWM_PDC2 = 0;
    PWM_PDC1 = 0; 
    
    /** Set Override Data on all PWM outputs */
    /* 0b00 = State for PWM3H,L, if Override is Enabled */
    PG3IOCON2bits.OVRDAT = 0;
    /* 0b00 = State for PWM2H,L, if Override is Enabled */
    PG2IOCON2bits.OVRDAT = 0; 
    /* 0b00 = State for PWM1H,L, if Override is Enabled */
    PG1IOCON2bits.OVRDAT = 0;  

    /* 1 = OVRDAT<1> provides data for output on PWM3H */
    PG3IOCON2bits.OVRENH = 1; 
    /* 1 = OVRDAT<0> provides data for output on PWM3L */
    PG3IOCON2bits.OVRENL = 1; 
    
    /* 1 = OVRDAT<1> provides data for output on PWM2H */
    PG2IOCON2bits.OVRENH = 1;
    /* 1 = OVRDAT<0> provides data for output on PWM2L */
    PG2IOCON2bits.OVRENL = 1; 
    
    /* 1 = OVRDAT<1> provides data for output on PWM1H */
    PG1IOCON2bits.OVRENH = 1;  
    /* 1 = OVRDAT<0> provides data for output on PWM1L */
    PG1IOCON2bits.OVRENL = 1;     
}

/**
* <B> Function: HAL_MC1PWMDutyCycleLimitCheck(uint32_t) </B>
*
* @brief Function to limit the PWM duty cycles between Min and Max duty
*        
* @param duty value.
* @return dutycycle.
* 
* @example
* <CODE> HAL_MC1PWMDutyCycleLimitCheck(dutycycle); </CODE>
*
*/
uint32_t HAL_MC1PWMDutyCycleLimitCheck(uint32_t dutycycle)
{
    if(dutycycle < MIN_DUTY)
    {
        dutycycle = MIN_DUTY;
    }
  
    if(dutycycle > MAX_DUTY)
    {
        dutycycle = MAX_DUTY;
    }
    
    return dutycycle;
}

/**
* <B> Function: HAL_PWM_DutyCycleRegister_Set(uint32_t) </B>
*
* @brief Function to set the duty cycle values to the PDC registers
*        
* @param duty value
* @return none.
* 
* @example
* <CODE> HAL_PWM_DutyCycleRegister_Set(pwmDC); </CODE>
*
*/
void HAL_PWM_DutyCycleRegister_Set(uint32_t pwmDC)
{
    uint32_t pwmDuty;
    
    pwmDuty = HAL_MC1PWMDutyCycleLimitCheck(pwmDC);
    PWM_PDC3 = (uint32_t)(pwmDuty);
    PWM_PDC2 = (uint32_t)(pwmDuty);
    PWM_PDC1 = (uint32_t)(pwmDuty);
}

/**
* <B> Function: HAL_TrapHandler(void) </B>
*
* @brief Function handle the traps
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_TrapHandler(); </CODE>
*
*/
void HAL_TrapHandler(void)
{
    HAL_MC1PWMDisableOutputs();
    while (1)
    {
        Nop();
        Nop();
        Nop();
    }
}

/* Functions for PWMs ON and OFF  using override */
/**
* <B> Function: PWM1_OverrideEnableDataSet(uint32_t) </B>
*
* @brief Function to override PWM1.
*        
* @param override value.
* @return none.
* 
* @example
* <CODE> PWM1_OverrideEnableDataSet(data); </CODE>
*
*/
void PWM1_OverrideEnableDataSet(uint32_t data)
{
    uint32_t dataBuffer;
    dataBuffer = PG1IOCON2 & 0xFFCFCFFF;
    PG1IOCON2 = dataBuffer | data;
}
/**
* <B> Function: PWM2_OverrideEnableDataSet(uint32_t) </B>
*
* @brief Function to override PWM2.
*        
* @param override value.
* @return none.
* 
* @example
* <CODE> PWM2_OverrideEnableDataSet(data); </CODE>
*
*/
void PWM2_OverrideEnableDataSet(uint32_t data)
{
    uint32_t dataBuffer;
    dataBuffer = PG2IOCON2 & 0xFFCFCFFF;
    PG2IOCON2 = dataBuffer | data;
}
/**
* <B> Function: PWM3_OverrideEnableDataSet(uint32_t) </B>
*
* @brief Function to override PWM3.
*        
* @param override value.
* @return none.
* 
* @example
* <CODE> PWM3_OverrideEnableDataSet(data); </CODE>
*
*/
void PWM3_OverrideEnableDataSet(uint32_t data)
{
    uint32_t dataBuffer;
    dataBuffer = PG3IOCON2 & 0xFFCFCFFF;
    PG3IOCON2 = dataBuffer | data;
}
/**
* <B> Function: HAL_MC1MotorInputsRead(MCAPP_MEASURE_T *)  </B>
*
* @brief Function to assign the variables with respective ADC buffers
*        
* @param Pointer to the data structure containing measured currents.
* @return none.
* 
* @example
* <CODE> HAL_MC1MotorInputsRead(pMotorInputs); </CODE>
*
*/
void HAL_MC1MotorInputsRead(MCAPP_MEASURE_T *pMotorInputs)
{
#ifdef ENABLE_PHASE_CURRENT_MEASUREMENT
    pMotorInputs->measureCurrent.Ia = MC1_ADCBUF_IA;
    pMotorInputs->measureCurrent.Ib = MC1_ADCBUF_IB;
    pMotorInputs->measureCurrent.Ic = MC1_ADCBUF_IC;
#endif
    pMotorInputs->measureCurrent.Ibus = ADCBUF_IBUS;
    pMotorInputs->measurePot = MC1_ADCBUF_POT;
    pMotorInputs->measureVdc.count = MC_ADCBUF_VDC;
    pMotorInputs->measureVdc.value = 
                (float)(pMotorInputs->measureVdc.count * ADC_VOLTAGE_SCALE);
//    pMotorInputs->measurePhaseVolt.Va = ADCBUF_INV_A_VA;
//    pMotorInputs->measurePhaseVolt.Vb = ADCBUF_INV_A_VB;
//    pMotorInputs->measurePhaseVolt.Vc = ADCBUF_INV_A_VC;
}

/**
* <B> Function: HAL_MC1ClearPWMPCIFault(void)  </B>
*
* @brief Function to terminate the PWM Fault PWM event through software
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC1ClearPWMPCIFault(); </CODE>
*
*/
void HAL_MC1ClearPWMPCIFault(void)
{
    /* write of '1' to SWTERM bit will produce a PCI Fault termination event */
    PG1F1PCI1bits.SWTERM = 1;
    PG2F1PCI1bits.SWTERM = 1;
    PG3F1PCI1bits.SWTERM = 1;  
}

/**
* <B> Function: HAL_MC1EnableCurrentLimitDACRef(void)  </B>
*
* @brief Function to enable DAC reference for current limit PCI.
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC1EnableCurrentLimitDACRef(); </CODE>
*
*/
void HAL_MC1EnableCurrentLimitDACRef(void)
{
    /* Enable comparator module */
    EnableComparatorModule3();
    /* PWM GENERATOR 1 Current Limit PCI REGISTER */
    PG1CLPCI1    = CLPCI1_CUR_LIMIT_CFG;;
    /* PWM GENERATOR 2 Current Limit PCI REGISTER */
    PG2CLPCI1    = CLPCI1_CUR_LIMIT_CFG;
    /* PWM GENERATOR 3 Current Limit PCI REGISTER */
    PG3CLPCI1    = CLPCI1_CUR_LIMIT_CFG;
}

/**
* <B> Function: HAL_MC1DisableCurrentLimitDACRef(void)  </B>
*
* @brief Function to disable DAC reference for current limit PCI.
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC1DisableCurrentLimitDACRef(); </CODE>
*
*/
void HAL_MC1DisableCurrentLimitDACRef(void)
{
    /* Enable comparator module */
    DisableComparatorModule3(); 
    /* PWM GENERATOR 1 Current Limit PCI REGISTER */
    PG1CLPCI1    = 0x0000;
    /* PWM GENERATOR 2 Current Limit PCI REGISTER */
    PG2CLPCI1    = 0x0000;
    /* PWM GENERATOR 3 Current Limit PCI REGISTER */
    PG3CLPCI1    = 0x0000;
}

/**
* <B> Function: HAL_MC1SetCurrentLimitDACRef(void)  </B>
*
* @brief Function to set DAC reference for current limit PCI.
*        
* @param Current value in A.
* @return none.
* 
* @example
* <CODE> HAL_MC1SetCurrentLimitDACRef(); </CODE>
*
*/
void HAL_MC1SetCurrentLimitDACRef(float currentValue)
{
    /* Convert current value in Amp to DAC value in counts */
    uint16_t refValue = (uint16_t)(((currentValue * 2047.0f)/MC1_PEAK_CURRENT)+HALF_ADC_COUNT);
    CMP3_ReferenceSet(refValue);
}
// </editor-fold>