// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc1_service.c
 *
 * @brief This module implements motor control.
 *
 * Component: MOTOR CONTROL APPLICATION
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

#include <libq.h>
#include "diagnostics.h"
#include "board_service.h"
#include "mc1_init.h"
#include "trapezoidal_control.h"
#include "estim_interface.h"
#include "mc1_user_params.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc=" Global Variables ">

uint16_t runCmdMC1=0;
/* GSP: accepted-commutation tally (incremented by estim_bemf.c on every
 * sector change; reported as goodZcCount). Defined unconditionally so the
 * symbol exists even when ENABLE_GSP is off. */
volatile uint16_t g_gspZcCount = 0;
#ifdef ENABLE_GSP
#include "gsp_ak.h"
#endif
uint16_t directionCmdMC1=0;
int16_t heartBeatCount=0;

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="VARIABLES ">

MC1APP_DATA_T mc1;
MC1APP_DATA_T *pMC1Data = &mc1;

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

static void MC1APP_StateMachine(MC1APP_DATA_T *);
static void MCAPP_MC1ReceivedDataProcess(MC1APP_DATA_T *);
inline static void BoardServiceStep(void);

// </editor-fold>

/**
* <B> Function: void MC1APP_StateMachine (MC1APP_DATA_T *)  </B>
*
* @brief Application state machine.
*
* @param Pointer to the data structure containing Application parameters.
* @return none.
* 
* @example
* <CODE> MC1APP_StateMachine(&mc); </CODE>
*
*/
static void MC1APP_StateMachine(MC1APP_DATA_T *pMCData)
{
    MCAPP_MEASURE_T *pMotorInputs = pMCData->pMotorInputs;
    MCAPP_CONTROL_SCHEME_T *pControlScheme = pMCData->pControlScheme;

    switch(pMCData->appState)
    {
    case MCAPP_INIT:

        HAL_MC1PWMDisableOutputs();

        /* Stop the motor */
        pMCData->runCmd = 0;       
        pMCData->directionChangeDelayCounter = pMCData->directionChangeDelayValue;
        
        MCAPP_TrapezoidalControlInit(pControlScheme);
        MCAPP_MeasureInit(pMotorInputs);      
        pMCData->appState = MCAPP_CMD_WAIT;

        break;
        
    case MCAPP_CMD_WAIT:
        
        if(pMCData->runCmd == 1)
        {
            /* Function call to charge Bootstrap capacitors*/
//            ChargeBootstrapCapacitors();
            pMCData->appState = MCAPP_OFFSET;
        }
       break;
       
    case MCAPP_OFFSET:
        
        /* Measure Initial Offsets */
        MCAPP_MeasureCurrentOffset(pMotorInputs);

        if(MCAPP_MeasureCurrentOffsetStatus(pMotorInputs))
        {
            /* Start the timer */
            COMMUTATION_TIMER_START();
            HAL_MC1PWMEnableOutputs();
            EnableSamplingBEMF();
            pMCData->appState = MCAPP_RUN;
        }

        break;
            
    case MCAPP_RUN:
        /* Check for change direction command flag */
        if(pMCData->directionCmdFlag == 1)
        {
            /* Disable PWM outputs while motor is slowing down for change direction*/
            HAL_MC1PWMDisableOutputs();
            /* Change run direction */
            pMCData->appState = MCAPP_DIRECTION_CHANGE;
            break;
        }
        
        /* Compensate motor current offsets */
        MCAPP_MeasureCurrentCalibrate(pMotorInputs);
        /* Check for over current fault */
        if (MCAPP_OverCurrentFault_Detect(pMotorInputs, &pMCData->fault) == 1)
        {
            pMCData->appState = MCAPP_FAULT;
            break;
        }
        /* Phase voltages in actual values */
        MCAPP_MeasureActualPhaseVoltage(pMotorInputs);
 
        MCAPP_TrapezoidalControlStateMachine(pControlScheme);
        
        /* Check for control scheme faults */
        if(pControlScheme->faultStatus == 1 ) 
        {
            pMCData->appState = MCAPP_FAULT;
            break;
        }

        if (pMCData->runCmd == 0)
        {
            /* Stop the timer */
            Commutation_ResetTiming();
            /* Exit loop if motor not run */
            pMCData->appState = MCAPP_STOP;
        }
        
        break;

    case MCAPP_DIRECTION_CHANGE:
        
        /* Disable PWM outputs while motor is slowing down for change direction*/
        HAL_MC1PWMDisableOutputs();
               
        /* Check if motor has stopped */
            if(pMCData->directionChangeDelayCounter == 0)
            {
                /* Change direction */
                pMCData->directionCmd = pMCData->directionCmdBuffer;
                /* Indicate direction change completed*/
                pMCData->directionCmdFlag = 0;
                /* Move state machine state to Init */
                pMCData->appState = MCAPP_INIT;
            }
            else
            {
                /* Decrement counter till motor stops running */
                pMCData->directionChangeDelayCounter--;
            }
        break;

    case MCAPP_STOP:
        HAL_MC1PWMDisableOutputs();
        DisableSamplingBEMF();
        pMCData->appState = MCAPP_INIT;
        
        break;
        
    case MCAPP_FAULT:
        /* Based on the application handle the fault state appropriately, 
           in this example there is no recovery from fault state */
        HAL_MC1PWMDisableOutputs();
        
        break;
        
    default:
        HAL_MC1PWMDisableOutputs();
        break;     

    } /* end of switch-case */
     
}

/**
* <B> Function: MC1_ADC_INTERRUPT()  </B>
*
* @brief ADC interrupt vector ,and it performs following actions:
*        (1) Increments DiagnosticsStepIsr for X2C Scope 
*        (2) Reads motor 1 bus current and phase voltage
*            feedbacks from ADC data buffers.
*        (3) Executes Trapezoidal Control based on the current and speed feedbacks.
*        (4) Loads duty cycle  to the registers of PWM Generators 
*             controlling motor 1.
* 
* @param none.
* @return none.
* 
* @example none
*
*/
void __attribute__((__interrupt__,no_auto_psv)) MC1_ADC_INTERRUPT()
{
    int16_t __attribute__((__unused__)) adcBuffer;
    DEBUG_EVTC_DC_ADC_ISR = 1;
    
    #ifdef ENABLE_DIAGNOSTICS
        DiagnosticsStepIsr();
    #endif
    
    BoardServiceStep();
#ifdef ENABLE_GSP
    GSP_IsrTick();          /* 1 ms timebase + failsafe for the GUI link */
#endif
                
    HAL_MC1MotorInputsRead(pMC1Data->pMotorInputs);

    MC1APP_StateMachine(pMC1Data);
    
    HAL_PWM_DutyCycleRegister_Set(pMC1Data->pControlScheme->pwmDuty);

    adcBuffer = MC1_ClearADCIF_ReadADCBUF();
    DEBUG_EVTC_DC_ADC_ISR = 0;
	MC1_ClearADCIF();
}

void __attribute__((__interrupt__,no_auto_psv)) ADC_PhaseVoltageSampleISR()
{
    int16_t __attribute__((__unused__)) adcBuffer;
    DEBUG_VC = 0;

    pMC1Data->pMotorInputs->measurePhaseVolt.Va = ADCBUF_INV_A_VA;
    pMC1Data->pMotorInputs->measurePhaseVolt.Vb = ADCBUF_INV_A_VB;
    pMC1Data->pMotorInputs->measurePhaseVolt.Vc = ADCBUF_INV_A_VC;

    /* Estimate Speed and rotor position */
    MCAPP_EstimatorStep(&pMC1Data->pControlScheme->estimatorInterface); 
    
    if(IsMultipleSamplingEnabled(&pMC1Data->pControlScheme->variableTrigger))
    {
        AD1SWTRGbits.CH0TRG = 1;
        AD1SWTRGbits.CH1TRG = 1;
        AD2SWTRGbits.CH0TRG = 1;
    }
    
    adcBuffer = ADCBUF_INV_A_VB;
    DEBUG_VC = 1;
    /* Clear ADC interrupt flag */
    ClearADC_PhaseVoltageSampleFlag();
}

void __attribute__((__interrupt__,no_auto_psv)) _PWMEVTAInterrupt()
{
    DEBUG_EVTA_SOC = 0;
    EnableMultipleSampling(&pMC1Data->pControlScheme->variableTrigger);
    PG2TRIGBbits.TRIGB = (0.75f * pMC1Data->pControlScheme->pwmDuty); 
    
    VariableTriggerUpdate(&pMC1Data->pControlScheme->variableTrigger,pMC1Data->pControlScheme->pwmDuty);
    if (VariableTriggerSample3(&pMC1Data->pControlScheme->variableTrigger))
    {
        /* Perform Sample 3 BEMF measurement */
        AD1SWTRGbits.CH0TRG = 1;
        AD1SWTRGbits.CH1TRG = 1;
        AD2SWTRGbits.CH0TRG = 1;
    }
    DEBUG_EVTA_SOC = 1;
    IFS1bits.PEVTAIF = 0;
}

void __attribute__((__interrupt__,no_auto_psv)) _PWMEVTCInterrupt()
{
    DEBUG_EVTC_DC_ADC_ISR = 1;
    
    DisableMultipleSampling(&pMC1Data->pControlScheme->variableTrigger);
    
    if ((VariableTriggerSample1(&pMC1Data->pControlScheme->variableTrigger)) || (VariableTriggerSample2(&pMC1Data->pControlScheme->variableTrigger)))
    {
        AD1SWTRGbits.CH0TRG = 1;
        AD1SWTRGbits.CH1TRG = 1;
        AD2SWTRGbits.CH0TRG = 1;
    }

    DEBUG_EVTC_DC_ADC_ISR = 0;
    IFS1bits.PEVTCIF = 0;
}
/**
* <B> Function: _PWMInterrupt()     </B>
*
* @brief Function to handle PWM Fault Interrupt from Fault PCI
*        
* @param none.
* @return none.
* 
* @example none
* 
*/
void __attribute__((__interrupt__,no_auto_psv)) _PWMInterrupt()
{
    HAL_MC1ClearPWMPCIFault();
    mc1.appState = MCAPP_FAULT;
    mc1.fault.faultState = MCAPP_OVERCURRENT_FAULT_DCBUS;
    ClearPWMIF(); 
}

/**
* <B> Function: void MCAPP_MC1ServiceInit (void)  </B>
*
* @brief Function to initialize the MC1 parameters
*
* @param Pointer to the data structure containing Application parameters.
* @return none.
* 
* @example
* <CODE> MCAPP_MC1ServiceInit(); </CODE>
*
*/
void MCAPP_MC1ServiceInit(void)
{
    MCAPP_MC1ParamsInit(pMC1Data);

    MC1_ClearADCIF();
    MC1_EnableADCInterrupt();
   
    IEC1bits.PEVTCIE = 1;
    IEC1bits.PEVTAIE = 1;
    HAL_MC1PWMDisableOutputs();
}

/**
* <B> Function: void MCAPP_MC1InputBufferSet (uint16_t, uint16_t)  </B>
*
* @brief Function store the run command and direction change command
*
* @param run and direction command
* @return none.
* 
* @example
* <CODE> MCAPP_MC1InputBufferSet(runCmdMC1,directionCmdMC1); </CODE>
*
*/
void MCAPP_MC1InputBufferSet(uint16_t runCmd, uint16_t directionCmd )
{ 
    pMC1Data->runCmdBuffer = runCmd;
    pMC1Data->directionCmdBuffer = directionCmd;
    
    MCAPP_MC1ReceivedDataProcess(pMC1Data);
}

/**
* <B> Function: void MCAPP_MC1ReceivedDataProcess (MC1APP_DATA_T *)  </B>
*
* @brief Function to process the received ADC and digital input data
*
* @param Pointer to the data structure containing Application parameters.
* @return none.
* 
* @example
* <CODE> MCAPP_MC1ReceivedDataProcess(&pMCData); </CODE>
*
*/
static void MCAPP_MC1ReceivedDataProcess(MC1APP_DATA_T *pMCData)
{
    MCAPP_CONTROL_SCHEME_T *pControlScheme = pMCData->pControlScheme;
    MCAPP_MEASURE_T *pMotorInputs = pMCData->pMotorInputs;
    
    /* Update the run command with run command buffer value */
    pMCData->runCmd = pMCData->runCmdBuffer;
        
    /* If there is a change direction command */
    if( pMCData->directionCmd != pMCData->directionCmdBuffer)
    {
        /* Indicates change in direction to state machine*/
        pMCData->directionCmdFlag = 1;
    }

    /* Control Input from pot - or from the GUI throttle when the GSP
     * source is selected (GSP_ThrottleTarget returns the pot unchanged
     * in ADC mode; slew limiting stays in force either way) */
    float currentValue = pControlScheme->ctrlParam.controlInput;
#ifdef ENABLE_GSP
    float targetValue = GSP_ThrottleTarget(pMotorInputs->measurePot);
#else
    float targetValue = pMotorInputs->measurePot;
#endif
    pControlScheme->ctrlParam.controlInput = RateLimitSym(currentValue, targetValue, POT_RAMP_STEP_PER_ISR);
}

/**
* <B> Function: BoardServiceStep()     </B>
*
* @brief Function to handle board service step. 
* This configured for 100 micro second.
* Board service routine is executed here.
* LED1 is toggled at a rate of 250 ms as a Heartbeat LED.
*        
* @param none.
* @return none.
* 
* @example
* <CODE> BoardServiceStep();        </CODE>
*
*/
inline static void BoardServiceStep(void)
{
    BoardService();
    if (IsPressed_Button1())
    {
        if(runCmdMC1 == 1)
        {
            runCmdMC1 = 0;
        }
        else
        {
            runCmdMC1 = 1;
        }
    }

    if (IsPressed_Button2())
    {
        if(directionCmdMC1 == 1)
        {
            directionCmdMC1 = 0;
        }
        else
        {
            directionCmdMC1 = 1;
        }
    }
    
    //    /*Heart Beat LED : LED1*/
    if (heartBeatCount < HEART_BEAT_LED_COUNT)
    {
        heartBeatCount += 1;
    }
    else
    {
        heartBeatCount = 0;
        if(LED1 == 1)
        {
            LED1 = 0;
        }
        else
        {
            LED1 = 1;
        }
    }
    
    /* LED2 status indicates the run command */
    LED2 = runCmdMC1;
    
    MCAPP_MC1InputBufferSet(runCmdMC1, directionCmdMC1);

    BoardServiceStepIsr();

}

/* ---- GSP accessors (garuda-gui bridge, see gsp_ak.c) ------------------- */
uint16_t MC1_GspAppState(void)
{
    return pMC1Data->appState;
}

void MC1_GspClearFault(void)
{
    if (pMC1Data->appState == MCAPP_FAULT)
    {
        runCmdMC1 = 0;
        pMC1Data->runCmd = 0;
        pMC1Data->runCmdBuffer = 0;
        pMC1Data->appState = MCAPP_INIT;
    }
}
