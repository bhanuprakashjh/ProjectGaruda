// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc1_init.c
 *
 * @brief This module initializes data structure holding motor control
 * parameters required to run motor 1 using field oriented control.
 * In this application to initialize variable required to run the application.
 *
 * Component: APPLICATION (Motor Control 1 - mc1)
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
#include <string.h>

#include "mc1_init.h"
#include "board_service.h"
#include "mc1_user_params.h"
#include "mc1_calc_params.h"
#include "filter_types.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

static void MCAPP_MC1ControlSchemeConfig(MC1APP_DATA_T *);

// </editor-fold>

/**
* <B> Function: MCAPP_MC1ParamsInit (MC1APP_DATA_T *)  </B>
*
* @brief Function to reset variables used for current offset measurement.
*
* @param Pointer to the Application data structure required for 
* controlling motor 1.
* @return none.
* 
* @example
* <CODE> MCAPP_MC1ParamsInit(&pMCData); </CODE>
*
*/
void MCAPP_MC1ParamsInit(MC1APP_DATA_T *pMCData)
{    
    /* Reset all variables in the data structure to '0' */
    memset(pMCData,0,sizeof(MC1APP_DATA_T));

    pMCData->pControlScheme = &pMCData->controlScheme;
    pMCData->pMotorInputs = &pMCData->motorInputs;
    
    /* Configure Control Scheme */
    MCAPP_MC1ControlSchemeConfig(pMCData);
    
    /* Set motor control state as 'MTR_INIT' */
    pMCData->appState = MCAPP_INIT;
}

/**
* <B> Function: MCAPP_MC1ControlSchemeConfig (MC1APP_DATA_T *)  </B>
*
* @brief Function to configure the control scheme and parameters
*
* @param Pointer to the Application data structure required for 
* controlling motor 1.
* @return none.
* 
* @example
* <CODE> MCAPP_MC1ControlSchemeConfig(&pMCData); </CODE>
*
*/
void MCAPP_MC1ControlSchemeConfig(MC1APP_DATA_T *pMCData)
{
    MCAPP_CONTROL_SCHEME_T *pControlScheme;
    MCAPP_MEASURE_T *pMotorInputs;

    pControlScheme = pMCData->pControlScheme;
    pMotorInputs = pMCData->pMotorInputs;
 
    /* Configure Inputs */    
    pControlScheme->pDirectionCmd = &pMCData->directionCmd;
    pControlScheme->pIbus = &pMotorInputs->measureCurrent.Ibus_actual;
    pControlScheme->pVdc= &pMotorInputs->measureVdc.value;
    
    /* Initialize Motor parameters */
    pControlScheme->motor.MaxSpeed        =  MAXIMUM_SPEED_RPM;
    pControlScheme->motor.MinSpeed        =  TARGET_STARTUP_SPEED_RPM;
    /* Set motor rated current to nominal peak current, limited to the 95% of peak current. */
    pControlScheme->motor.RatedCurrent    =  (NOMINAL_CURRENT_PEAK > MC1_PEAK_CURRENT) ?
                                                (0.95f * MC1_PEAK_CURRENT) : NOMINAL_CURRENT_PEAK;
    pControlScheme->motor.Ke              = MOTOR_BEMF_CONSTANT_ELEC;
    pControlScheme->motor.Ls              = MOTOR_PER_PHASE_INDUCTANCE;
    pControlScheme->motor.Rs              = MOTOR_PER_PHASE_RESISTANCE;
    pControlScheme->motor.Inertia         = MOTOR_INERTIA;
    pControlScheme->motor.polePairs       = (uint16_t)POLE_PAIRS;
            
    /* Initialize Trapezoidal control parameters */
#if CLOSED_LOOP == 0
    pControlScheme->ctrlParam.controlLoop = VOLTAGE_CONTROL;
#elif CLOSED_LOOP == 1
    pControlScheme->ctrlParam.controlLoop = SPEED_CONTROL;
#elif CLOSED_LOOP == 2
    #ifdef ENABLE_CURRENT_CONTROL 
        pControlScheme->ctrlParam.controlLoop = CURRENT_CONTROL;
    #else
        pControlScheme->ctrlParam.controlLoop = SPEED_CONTROL;
    #endif
#else 
    pControlScheme->ctrlParam.controlLoop = SPEED_CONTROL;
#endif       
    pControlScheme->ctrlParam.speedRampRate = SPEED_REF_RAMP_VALUE;
    pControlScheme->controlLoopRate = SPEED_LOOP_COUNTS;
    pMCData->directionChangeDelayValue = DIRECTION_CHANGE_DELAY_COUNTS;
    /* Initialize fault parameters */
    /* Set over current fault limit, if greater than measurable peak current set fault limit to peak measurable current  */
    pMCData->fault.overCurrentFaultLimit = (OC_FAULT_LIMIT_PHASE > (MC1_PEAK_CURRENT)) ?
                                                MC1_PEAK_CURRENT : OC_FAULT_LIMIT_PHASE;
    
    /* Initialize startup parameters */
    pControlScheme->startup.rotorLockTimeLimit = ROTOR_LOCK_TIME_COUNTS;
    pControlScheme->startup.rotorLockCurrentRef = ROTOR_LOCK_CURRENT;   
    pControlScheme->startup.finalFreq = FINAL_STARTUP_FREQ_HZ;
    pControlScheme->startup.initialFreq = INITIAL_STARTUP_FREQ_HZ;
    pControlScheme->startup.rampStepFreq = STARTUP_FREQ_RAMP_STEP;
    pControlScheme->startup.minstartupDuty = MIN_STARTUP_DUTY;
    pControlScheme->startup.targetSpeed = TARGET_STARTUP_SPEED_RPM;
    pControlScheme->startup.vfRatio = V_F_RATIO;
    pControlScheme->startup.Tcy = MC1_LOOPTIME_SEC;
    pControlScheme->startup.rotorLockMaxVoltage = ROTOR_LOCK_MAX_VOLTAGE;
    pControlScheme->startup.startupCurrent = MAX_STARTUP_CURRENT_PK;

// <editor-fold defaultstate="collapsed" desc="CALCULATE PI CONTROL PARAMETERS "> 
    float currentPI_Kp = (2.0*pControlScheme->motor.Ls)/(4.0*IBUSPI_ZETA*IBUSPI_ZETA*(1.5 * MC1_LOOPTIME_SEC));
    float currentPI_Ki = ((2.0*pControlScheme->motor.Rs)/(4.0*IBUSPI_ZETA*IBUSPI_ZETA*(1.5 * MC1_LOOPTIME_SEC)));
    
    float filterCutoff  = FILTER_CUTOFF*2.0f*M_PI;
    float Kmd = 1.5f*pControlScheme->motor.Ke * pControlScheme->motor.polePairs;
    float innerLoopTimeConst = (2.0f*pControlScheme->motor.Ls)/currentPI_Kp;
    float Tf = (1.0f/filterCutoff) + innerLoopTimeConst;
    float PM  = tan(PHASE_MARGIN*M_PI/180.0f);
    float x   = Tf*((4.0f*PM*PM)+2.0f);
    float T1  = (x + sqrt((x*x)-(4.0f*Tf*Tf)))/2.0f;
    float T2  = (x - sqrt((x*x)-(4.0f*Tf*Tf)))/2.0f;
    float Tw = 00.f;
    if(T1>Tf)
    {
        Tw = T1;
    }
    else if(T2>Tf)
    {
        Tw = T2;
    }
    else
    {
        Tw = 0.0f;
    }    
    float Wgc  = 1.0f/sqrt(Tw*Tf);
    float speedPI_Kp  = (pControlScheme->motor.Inertia*Wgc/Kmd)*(M_PI/30.0f);
    float speedPI_Ki  = (speedPI_Kp/Tw)*(M_PI/30.0f); 
// </editor-fold>
    /* Initialize PI controller used for current control */
    pControlScheme->maxVoltage                =   (DC_LINK_VOLTAGE * 0.95f);
#ifdef ENABLE_CURRENT_CONTROL
    pControlScheme->piCurrent.param.kp        =   currentPI_Kp;
    pControlScheme->piCurrent.param.ki        =   currentPI_Ki;
    pControlScheme->piCurrent.param.samTime   =   MC1_LOOPTIME_SEC;
    pControlScheme->piCurrent.param.outMin    =   (DC_LINK_VOLTAGE * 0.005f);
    
    /* Initialize PI controller used for speed control */
    pControlScheme->piSpeed.param.kp          =   speedPI_Kp;
    pControlScheme->piSpeed.param.ki          =   speedPI_Ki;
    pControlScheme->piSpeed.param.samTime     =   (1.0f/SPEED_LOOP_HZ);
    pControlScheme->piSpeed.param.outMax      =   pControlScheme->motor.RatedCurrent;
    pControlScheme->piSpeed.param.outMin      =   (pControlScheme->motor.RatedCurrent*0.005);
#else
    /* Initialize PI controller used for speed control */
    pControlScheme->piSpeed.param.kp          =   speedPI_Kp;
    pControlScheme->piSpeed.param.ki          =   speedPI_Ki;
    pControlScheme->piSpeed.param.samTime     =   (1.0f/SPEED_LOOP_HZ);
    pControlScheme->piSpeed.param.outMax      =   (DC_LINK_VOLTAGE * 0.999f);
    pControlScheme->piSpeed.param.outMin      =   (DC_LINK_VOLTAGE * 0.005f);
#endif
    
    /* Initialize BEMF Estimator */
    pControlScheme->estimator.pBemf1 = &pMotorInputs->measurePhaseVolt.Va_actual;
    pControlScheme->estimator.pBemf2 = &pMotorInputs->measurePhaseVolt.Vb_actual;
    pControlScheme->estimator.pBemf3 = &pMotorInputs->measurePhaseVolt.Vc_actual;
    pControlScheme->estimator.startupBlankingTime = OL_BLANKING_TIME;
    pControlScheme->estimator.closedLoopBlankingTime = CL_BLANKING_TIME;
    pControlScheme->estimator.timerCountsPerSectorMultiplier = SPEED_MULTIPLIER;
    pControlScheme->estimator.calculateSpeed.multiplier = SPEED_MULTIPLIER;
    pControlScheme->estimator.calculateSpeed.motorStallMultiplier = MOTOR_STALL_DETECT_COUNTS;
    pControlScheme->estimator.calculateSpeed.LPF_omega.Wc = FILTER_CUTOFF*2.0*M_PI;
    pControlScheme->estimator.calculateSpeed.LPF_omega.Ts = MC1_LOOPTIME_SEC;
    
    /* Initialize Estimator Interface */
    pControlScheme->estimatorInterface.pEstimBEMF  = &pControlScheme->estimator;
    
    /* Output Initializations */
    pControlScheme->pwmPeriod = LOOPTIME_TCY; 
}