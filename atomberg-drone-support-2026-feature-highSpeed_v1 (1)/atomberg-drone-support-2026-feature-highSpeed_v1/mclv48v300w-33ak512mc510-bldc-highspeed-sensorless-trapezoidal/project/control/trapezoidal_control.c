// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file trapezoidal_control.c
 *
 * @brief This module implements control state machine.
 *
 * Component: CONTROL ALGORITHIM
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
#include "board_service.h"
#include "trapezoidal_control.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Global Variables  ">

/* Commutation sector sequence for forward direction (ABC) */
//uint8_t commSectorSeqABC[6] = {4, 6, 2, 3, 1, 5}; 
uint8_t commSectorSeqABC[6] = {1, 5, 4, 6, 2, 3}; 
/* Commutation sector sequence for reverse direction (ACB) */
uint8_t commSectorSeqACB[6] = {4, 5, 1, 3, 2, 6}; 

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">
static void MCAPP_GetControlInputs(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *);
static void UpdateStartupCommutationStep(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *);
inline static bool ReadyforClosedLoopTransition(MCAPP_ESTIMATOR_T *);
inline static bool ReadyforClosedLoopStarting(MCAPP_ESTIMATOR_T *pData);
inline static void MCAPP_UpdateControlState(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pControl);
inline static void MCAPP_EstimatorCheckClosedLoopStarting(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pEstim);
inline static void MCAPP_EstimatorCheckClosedLoopTransition(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pData);
inline static void MCAPP_TrapezoidalControlUpdateInverterSwitching(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *); 
// </editor-fold>

/**
* <B> Function: MCAPP_TrapezoidalControlInit(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *)  </B>
*
* @brief Function to set variables for Control.
*        
* @param 
* @return none.
 * 
* @example
* <CODE> MCAPP_TrapezoidalControlInit(&pTrapezoidalControl); </CODE>
*
*/
void MCAPP_TrapezoidalControlInit(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pTrapezoidalControl)
{
    pTrapezoidalControl->piCurrent.inMeasure   = 0;
    pTrapezoidalControl->piCurrent.inReference = 0;
    pTrapezoidalControl->piCurrent.output        = 0;
    pTrapezoidalControl->piSpeed.inMeasure     = 0;
    pTrapezoidalControl->piSpeed.inReference   = 0;
    pTrapezoidalControl->piSpeed.output          = 0;
    MC_ControllerPIReset(&pTrapezoidalControl->piCurrent,0);
    MC_ControllerPIReset(&pTrapezoidalControl->piSpeed,0);
    
    pTrapezoidalControl->Ibus                       = 0;
    pTrapezoidalControl->commutationSector          = 0;
    pTrapezoidalControl->controlState               = 0;
    pTrapezoidalControl->directionCmd               = 0;
    pTrapezoidalControl->faultStatus                = 0;
    pTrapezoidalControl->pwmDuty                    = 0;
    pTrapezoidalControl->nextCommutationSectorIndex = 0;
    pTrapezoidalControl->voltageRaw = 0;
    pTrapezoidalControl->Vdc = 0;
    pTrapezoidalControl->nextCommutationSector = 0;
    pTrapezoidalControl->presentCommutationSector = 0;
    pTrapezoidalControl->previousCommutationSector = 0;
    pTrapezoidalControl->commutationSectorMissmatched = 0;

    pTrapezoidalControl->ctrlParam.targetCurrent    = 0;
    pTrapezoidalControl->ctrlParam.targetDuty       = 0;
    pTrapezoidalControl->ctrlParam.targetSpeed      = 0;
    pTrapezoidalControl->ctrlParam.speedRef         = 0;
    
    pTrapezoidalControl->startup.rotorLockTimeCounter = 0;
    pTrapezoidalControl->startup.sectorValidCount = 0;
    pTrapezoidalControl->startup.commutationSectorIndex = 0;
    pTrapezoidalControl->startup.prevCommutationSectorIndex = 0;
    pTrapezoidalControl->startup.startupDuty = 0;
    pTrapezoidalControl->startup.commutationSector = 0;
    pTrapezoidalControl->startup.commutationSectorChanged = 0;
    pTrapezoidalControl->startup.thetaSum = 0;
    pTrapezoidalControl->startup.thetaRadian = 0;
    pTrapezoidalControl->startup.omegaEle = 0;
    pTrapezoidalControl->startup.commutationFreq = 0;
    
    /* Initialize Estimator parameters */
    MCAPP_EstimatorInit(&pTrapezoidalControl->estimatorInterface);
    /* Initialize variable trigger */
    VariableTriggerInit(&pTrapezoidalControl->variableTrigger);
    
}


/**
* <B> Function: void MCAPP_GetControlInputs (MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *) </B>
*
* @brief Function read motor control inputs.
*
* @param Pointer to the data structure containing Control parameters.
* @return none.
* @example
* <CODE> MCAPP_GetControlInputs(&pControl); </CODE>
*
*/ 
static inline void MCAPP_GetControlInputs(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pControl)
{ 
    MCAPP_MOTOR_T *pMotor = &pControl->motor;
    
    /* Direction command input */
    pControl->directionCmd  = *(pControl->pDirectionCmd);
    /* Measured bus current */
    pControl->Ibus = *(pControl->pIbus);
    /* Measured DC bus voltage */
    pControl->Vdc = *(pControl->pVdc);
    /* directionCmd 0 -> +1 (forward), 1 -> -1 (reverse) */
    pControl->estimator.directionCmd = (pControl->directionCmd == 0U) ? 1 : -1;
    /* Set the commutation sector sequence pointer based on the direction command */
    if(pControl->directionCmd == 1)
    {
            pControl->pCommSectorSeq = commSectorSeqACB;
    }
    else
    {
            pControl->pCommSectorSeq = commSectorSeqABC;
    }
    
    if(pControl->ctrlParam.controlLoop == SPEED_CONTROL)
    {
        /* Speed Input from control input for speed control */
       pControl->ctrlParam.targetSpeed = pMotor->MinSpeed + 
            (((pMotor->MaxSpeed - pMotor->MinSpeed) * 
               pControl->ctrlParam.controlInput)/MAX_ADC_COUNT);
    }
    else if(pControl->ctrlParam.controlLoop == CURRENT_CONTROL)
    {
        /* Current Input from control input for current control */
        pControl->ctrlParam.targetCurrent = pMotor->RatedCurrent *
                (pControl->ctrlParam.controlInput/MAX_ADC_COUNT);
    } 
    else
    {
        
    }
}

inline static uint8_t getSectorIndex(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pControl, uint8_t currentSector)
{
    for (uint8_t arrayIndex = 0; arrayIndex < 6; arrayIndex++)
    {
        if (pControl->pCommSectorSeq[arrayIndex] == currentSector)
            return arrayIndex;
    }
    return 7; // error case
}

static inline void MCAPP_CheckAndUpdateCommutation(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pControl)
{
    MCAPP_ESTIMATOR_T *pEstim = &pControl->estimator;
    
    
    /* Check if update inverter switching pattern flag is set */
    if (pEstim->updateInverterSwitchingPattern == true)
    {
        /* Remaining time before commutation */
        int32_t remainingCounts = pEstim->commutationPeriod  - COMMUTATION_TIMER_READ();
        /* Clamp to minimum safe value */
        if (remainingCounts <= 0)
        {
            remainingCounts = 1;
        }
        /* Ensure update occurs within current PWM period */
        if (remainingCounts < PWM_PERIOD_TIMER_COUNTS)
        {
            #warning remove this line
            DEBUG_EVTC_DC_ADC_ISR = 0;

            /* Apply the inverter switching pattern
               corresponding to the commutation sector */
            MCAPP_TrapezoidalControlUpdateInverterSwitching(pControl);

            pEstim->updateInverterSwitchingPattern = false;
#warning remove this line
            DEBUG_EVTC_DC_ADC_ISR = 1;
        }
    }
    else
    {
        /* Do nothing */
    }
}

/**
* <B> Function: void MCAPP_TrapezoidalControlStateMachine(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *)  </B>
*
* @brief Executes Speed, Current and Duty Control Loops and performs actions
*        associated with Trapezoidal control 
*
* @param Pointer to the data structure containing control parameters.
* @return none.
* @example
* <CODE> MCAPP_TrapezoidalControlStateMachine(&pControl); </CODE>
*
*/
void MCAPP_TrapezoidalControlStateMachine(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pControl)
{    
    MCAPP_CONTROL_T *pCtrlParam = &pControl->ctrlParam;
    MCAPP_FORCED_COMMUTATION_STARTUP_T *pStartup = &pControl->startup;
    MCAPP_ESTIMATOR_T *pEstim = &pControl->estimator;
    
    switch (pControl->controlState)
    {
        case CONTROL_INIT:
            MCAPP_TrapezoidalControlInit(pControl);
#ifdef ENABLE_PWM_CURRENT_LIMITER 
            /* Set current limiter for rotor locking state */
            SetCurrentReference(pStartup->rotorLockCurrentRef);
#endif
            MCAPP_GetControlInputs(pControl);
            /* Load the initial commutation sector to lock the rotor */
            pControl->commutationSector = pControl->pCommSectorSeq[0];
            pControl->nextCommutationSector = pControl->pCommSectorSeq[0];
            /* Compute the minimum startup voltage based on the configured
               startup duty cycle and the maximum available DC bus voltage. */
            float minVoltage = (pStartup->minstartupDuty * pControl->maxVoltage);
            /* Check if rotor-lock voltage limit is not below the minimum startup voltage. */
            if(pStartup->rotorLockMaxVoltage < minVoltage)
            {
                /* Clamp rotor-lock maximum voltage to the minimum startup voltage */
                pStartup->rotorLockMaxVoltage = minVoltage;
            }
            else
            {
                /* Do nothing */
            }
#ifdef ENABLE_CURRENT_CONTROL
            /* Apply the rotor-lock voltage limit as the maximum output of the current PI controller. */
            pControl->piCurrent.param.outMax = pStartup->rotorLockMaxVoltage;
#endif
            pControl->controlState = ROTOR_LOCK;
            break;
            
        case ROTOR_LOCK:
            MCAPP_GetControlInputs(pControl);
            /* Apply the inverter switching pattern corresponding to the initial commutation sector. */
            MCAPP_TrapezoidalControlUpdateInverterSwitching(pControl);
         
//            /* Convert control input to target duty */
//            pControl->ctrlParam.targetDuty = (pControl->ctrlParam.controlInput / MAX_ADC_COUNT);            
//            /* Update PWM duty */
//            pControl->pwmDuty = (uint32_t)(pControl->ctrlParam.targetDuty * pControl->pwmPeriod);
            
#ifdef ENABLE_CURRENT_CONTROL            
            /* PI control in Current Loop */
            pControl->piCurrent.inReference = pStartup->rotorLockCurrentRef;
            pControl->piCurrent.inMeasure   = pControl->Ibus;
            MC_ControllerPIUpdate(&pControl->piCurrent);

            pControl->voltageRaw = (pControl->piCurrent.output/pControl->Vdc);
            pControl->pwmDuty = (uint32_t) ((float)(pControl->voltageRaw * 
                                                        pControl->pwmPeriod)); 
#else
            /* Fixed voltage for rotor locking */
            pControl->voltageRaw = (pStartup->rotorLockMaxVoltage/pControl->Vdc);
            pControl->pwmDuty = (uint32_t) ((float)(pControl->voltageRaw * 
                                                        pControl->pwmPeriod)); 
#endif
            /* Rotor lock time counter */
            if (pStartup->rotorLockTimeCounter < pStartup->rotorLockTimeLimit)
            {
                pStartup->rotorLockTimeCounter++;
            }
            else
            {
                /* Reset counter */
                pStartup->rotorLockTimeCounter = 0;
                /* Set  minimum allowable timer counts per sector for startup.
                 * Used to prevent invalid estimator timing values during startup. */
                pEstim->timerCountsPerSector =
                (uint32_t)(pControl->estimator.timerCountsPerSectorMultiplier /
                           (1.5f * pStartup->targetSpeed));
                 /* Reset current PI and restore limits */
                MC_ControllerPIReset(&pControl->piCurrent,0);
                pControl->piCurrent.param.outMax = pControl->maxVoltage;
                /* Set startup blanking time */
                pControl->estimator.blankingTimeLimit = pControl->estimator.startupBlankingTime;
#ifdef ENABLE_PWM_CURRENT_LIMITER 
                /* Set current limiter for startup state */
                SetCurrentReference(pStartup->startupCurrent);
#endif
                /* Transition to forced commutation startup */
                pControl->controlState = FORCED_COMMUTATION_STARTUP;
            }            
            break;
            
        case FORCED_COMMUTATION_STARTUP:

            MCAPP_GetControlInputs(pControl);
            
            /* Update startup commutation step and sector */
            UpdateStartupCommutationStep(pControl);
            
            /* Update commutation sector based on startup sector */
            pControl->commutationSector = pStartup->commutationSector; 
            /* Apply the inverter switching pattern corresponding to the initial commutation sector. */
            MCAPP_TrapezoidalControlUpdateInverterSwitching(pControl);

#ifndef ENABLE_CURRENT_CONTROL  
            /* Startup duty */
            pControl->pwmDuty = (uint32_t) ((float)(pStartup->startupDuty * 
                                                     pControl->pwmPeriod)); 
#else             
            /* PI control in Current Loop */
            pControl->piCurrent.inReference = pStartup->startupCurrent;
            pControl->piCurrent.inMeasure   = pControl->Ibus;
            MC_ControllerPIUpdate(&pControl->piCurrent);
            pControl->voltageRaw = (pControl->piCurrent.output/pControl->Vdc);
            pControl->pwmDuty = (uint32_t) ((float)(pControl->voltageRaw * 
                                                        pControl->pwmPeriod)); 
#endif
            /* Estimate Speed */
            MCAPP_CalculateSpeed(&pEstim->calculateSpeed);
            /* Increment blanking time counter */
            pEstim->blankingTimeCounter++;
            MCAPP_EstimatorCheckClosedLoopStarting(pControl);
            if(ReadyforClosedLoopStarting(pEstim)) 
            {
#ifndef ENABLE_CURRENT_CONTROL 
                /* Set speed PI integrator to startup duty for better response */
                pControl->piSpeed.stateVar.integrator = (pStartup->startupDuty*pControl->maxVoltage*0.75f);
#else
                /* Set current PI integrator to startup duty for smooth transition */
                pControl->piCurrent.stateVar.integrator = (pControl->voltageRaw*pControl->maxVoltage*0.75f);
                /* Set speed PI integrator to bus current for better response */
                pControl->piSpeed.stateVar.integrator = pStartup->rotorLockCurrentRef;
#endif
                /* Set timer counts per sector based on 1.5 times the maximum speed */
                pEstim->timerCountsPerSector = (uint32_t)((float)(pEstim->timerCountsPerSectorMultiplier/(1.5f *pControl->motor.MaxSpeed)));
#ifdef ENABLE_PWM_CURRENT_LIMITER 
                /* Set current limiter for closed-loop running  */
                SetCurrentReference(pControl->motor.RatedCurrent);
#endif
                /* Transition to closed-loop starting */
//                pControl->controlState = CLOSED_LOOP_STARTING;
                /* Update the speed reference with measured value */
                pCtrlParam->speedRef = pEstim->calculateSpeed.speed;
                pControl->controlLoopRateCounter = pControl->controlLoopRate;
                /* Update the commutation sectors */
                pControl->commutationSector = pEstim->commutationSector;
                pControl->nextCommutationSector = pControl->commutationSector;
                pControl->nextCommutationSectorIndex = getSectorIndex(pControl,pControl->nextCommutationSector);              
                /* Apply the inverter switching pattern corresponding to the initial commutation sector. */
                MCAPP_TrapezoidalControlUpdateInverterSwitching(pControl); 
                /* Set closed loop blanking time */
                pEstim->blankingTimeLimit = pEstim->closedLoopBlankingTime;
                /* Update the control state based on the control loop */
                MCAPP_UpdateControlState(pControl);
            }
            break;
        
        case CLOSED_LOOP_STARTING:

            HAL_MC1PWMDisableOutputs();
            
            MCAPP_GetControlInputs(pControl);
            
            /* Update startup commutation step and sector */
            UpdateStartupCommutationStep(pControl);
            
            MCAPP_EstimatorStep(&pControl->estimatorInterface); 
            
            MCAPP_EstimatorCheckClosedLoopTransition(pControl);
            
            if(ReadyforClosedLoopTransition(pEstim))
            {
                /* Update the speed reference with measured value */
                pCtrlParam->speedRef = pEstim->calculateSpeed.speed;
                /* Update the commutation sectors */
                pControl->commutationSector = pEstim->commutationSector;
                pControl->nextCommutationSector = pControl->commutationSector;
                pControl->nextCommutationSectorIndex = getSectorIndex(pControl,pControl->nextCommutationSector);              
                /* Apply the inverter switching pattern corresponding to the initial commutation sector. */
                MCAPP_TrapezoidalControlUpdateInverterSwitching(pControl); 
                /* Set closed loop blanking time */
                pEstim->blankingTimeLimit = pEstim->closedLoopBlankingTime;
                /* Update the control state based on the control loop */
                MCAPP_UpdateControlState(pControl);
            }
            else
            {
                /* Do nothing */
            }
            break;
            
        case VOLTAGE_CONTROL_LOOP:

            MCAPP_GetControlInputs(pControl);
            
            /* Estimate Speed */
            MCAPP_CalculateSpeed(&pEstim->calculateSpeed);
            
            /* Update blanking time limit */
            pEstim->blankingTimeLimit = UpdateBlankingTime(pEstim,pControl->motor.MaxSpeed);
            /* Increment blanking time counter */
            pEstim->blankingTimeCounter++;
            /* Update the commutation sector */
            pControl->commutationSector = pEstim->commutationSector;
            
            MCAPP_CheckAndUpdateCommutation(pControl);
            /* Convert control input to target duty */
            pControl->ctrlParam.targetDuty = (pControl->ctrlParam.controlInput / MAX_ADC_COUNT);
            
            /* Check if target duty is less than minimum duty */
            if(pControl->ctrlParam.targetDuty < pStartup->minstartupDuty)
            {
                /* Set target duty as minimum duty */
                pControl->ctrlParam.targetDuty = pStartup->minstartupDuty;
            }
            else
            {
                /* Do nothing */
            }
            
            /* Update PWM duty */
            pControl->pwmDuty = (uint32_t)(pControl->ctrlParam.targetDuty * pControl->pwmPeriod);
//            DEBUG_EVTC_DC_ADC_ISR = 1;
           
            break;  
          
        case SPEED_CONTROL_LOOP:
            
            MCAPP_GetControlInputs(pControl);
            /* Update estimator timer counts per sector */
            if (pCtrlParam->speedRef == pCtrlParam->targetSpeed)
            {
            /* Steady-state condition: calculate timing from target speed */
            pEstim->timerCountsPerSector =
                (uint32_t)(pEstim->timerCountsPerSectorMultiplier /
                           (1.5f * pCtrlParam->targetSpeed));
            }
            else
            {
            /* Speed ramp condition: limit timing using maximum motor speed */
            pEstim->timerCountsPerSector =
                (uint32_t)(pEstim->timerCountsPerSectorMultiplier /
                           (1.5f * pControl->motor.MaxSpeed));
            }

            /* Estimate Speed */
            MCAPP_CalculateSpeed(&pEstim->calculateSpeed);
            /* Update blanking time limit */
            pEstim->blankingTimeLimit = UpdateBlankingTime(pEstim,pControl->motor.MaxSpeed);
            /* Increment blanking time counter */
            pEstim->blankingTimeCounter++; 

            /* Update the commutation sector */
            pControl->commutationSector = pEstim->commutationSector;
            
            MCAPP_CheckAndUpdateCommutation(pControl);
            
            /* Increment speed control loop rate counter */
            pControl->controlLoopRateCounter++;
            /* Execute speed control loop at the configured rate */
            if(pControl->controlLoopRateCounter > pControl->controlLoopRate)
            {
                #warning remove this line
                DEBUG_SCCP = 0;
                /* Reset loop rate counter */
                pControl->controlLoopRateCounter = 0;
                /* Closed Loop Speed Ramp */
                float deltaSpeed = pCtrlParam->targetSpeed - pCtrlParam->speedRef;   
                if(deltaSpeed > pCtrlParam->speedRampRate)
                {
                     /* Increase speed reference by ramp rate */
                    pCtrlParam->speedRef = pCtrlParam->speedRef + 
                                                        pCtrlParam->speedRampRate;
                }
                else if(deltaSpeed < -pCtrlParam->speedRampRate)
                {
                     /* Decrease speed reference by ramp rate */
                    pCtrlParam->speedRef = pCtrlParam->speedRef - 
                                                        pCtrlParam->speedRampRate;
                }
                else
                {
                    /* Target speed reached: clamp reference to target */
                    pCtrlParam->speedRef = pCtrlParam->targetSpeed;
                }

                /* Execute Outer Speed Loop - Generate I Reference Generation */
                pControl->piSpeed.inMeasure   = pEstim->calculateSpeed.omegaFiltered;
                pControl->piSpeed.inReference = pCtrlParam->speedRef;
                MC_ControllerPIUpdate(&pControl->piSpeed);
                #warning remove this line
                DEBUG_SCCP = 1;
            }
#ifndef ENABLE_CURRENT_CONTROL 
            pControl->voltageRaw = (pControl->piSpeed.output/pControl->Vdc);
#else
			/* PI control in Current Loop */
            pControl->piCurrent.inReference = pControl->piSpeed.output;
            pControl->piCurrent.inMeasure   = pControl->Ibus;
            MC_ControllerPIUpdate(&pControl->piCurrent);
            pControl->voltageRaw = (pControl->piCurrent.output/pControl->Vdc);
#endif
            pControl->pwmDuty = (uint32_t) ((float)(pControl->voltageRaw * 
                                                        pControl->pwmPeriod)); 
            break;
            
        case CURRENT_CONTROL_LOOP:
            MCAPP_GetControlInputs(pControl);
            
            /* Estimate Speed and rotor position */
            MCAPP_EstimatorStep(&pControl->estimatorInterface); 
            
            /* Update the commutation sector */
            pControl->commutationSector = pEstim->commutationSector;
            
            /* PI control in Current Loop */
            pControl->piCurrent.inReference = pControl->ctrlParam.targetCurrent;
            pControl->piCurrent.inMeasure   = pControl->Ibus;
            MC_ControllerPIUpdate(&pControl->piCurrent);
            pControl->voltageRaw = (pControl->piCurrent.output/pControl->Vdc);
            pControl->pwmDuty = (uint32_t) ((float)(pControl->voltageRaw * 
                                                        pControl->pwmPeriod)); 
            break;

        case CONTROL_FAULT:
                    
            break;
        
        default:
            pControl->controlState = CONTROL_FAULT;
            break;

    } /* End Of switch - case */
}

/**
* <B> Function: inline static void MCAPP_EstimatorCheckClosedLoopTransition(MCAPP_ESTIMATOR_BEMF_T *)  </B>
*
 * @brief Starting closed loop operation based on estimated back EMF.
 *
 * @param pEstim Pointer to the estimator structure containing relevant variables and flags.
 * @return none.
* @example
* <CODE> MCAPP_EstimatorCheckClosedLoopTransition(&estimator); </CODE>
*
*/
inline static void MCAPP_EstimatorCheckClosedLoopTransition(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pData)
{
    MCAPP_FORCED_COMMUTATION_STARTUP_T *pStartup = &pData->startup;
    MCAPP_ESTIMATOR_T *pEstim = &pData->estimator;
    
    /* Check if a new commutation sector has been entered */
    if(pEstim->commutationSectorChanged == true)
    {
        /* Check if two fundamental cycles are completed  */
        if (pStartup->sectorValidCount >= 12)
        {
            pEstim->closedLoopFlag = 1; /* Set flag: transition to closed loop */
            pStartup->sectorValidCount = 0; /* Reset sector counter after switching */
        }
        else
        {
            /* Increment sector counter if condition is true */
            pStartup->sectorValidCount++;
        }
        pEstim->commutationSectorChanged = false;
    }
    else
    {
        
    }
}

/**
* <B> Function: inline static void MCAPP_EstimatorCheckClosedLoopStarting(MCAPP_ESTIMATOR_BEMF_T *)  </B>
*
 * @brief Checks conditions for transitioning from startup to closed loop operation
 *        in the estimator and sets the closedLoopStartingFlag accordingly.
 *
 * @param pEstim Pointer to the estimator structure containing relevant variables and flags.
 * @return none.
* @example
* <CODE> MCAPP_EstimatorCheckClosedLoopStarting(&estimator); </CODE>
*
*/
inline static void MCAPP_EstimatorCheckClosedLoopStarting(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pData)
{
    MCAPP_FORCED_COMMUTATION_STARTUP_T *pStartup = &pData->startup;
    MCAPP_ESTIMATOR_T *pEstim = &pData->estimator;
    
    /* Check if commutation frequency has reached its final value*/
    if(pStartup->commutationFreq == pStartup->finalFreq)
    { 
        /* Check if mapping is completed */
        if(pEstim->isBemfPhaseMappingComplete == true)
        {
            /* Check if a new commutation sector has been entered */
            if(pStartup->commutationSectorChanged == true)
            {
                /* Check if the calculated speed is within ±5% range of the set startup speed */
                if(pEstim->calculateSpeed.omegaFiltered > (0.95 * pStartup->targetSpeed) && 
                   (pEstim->calculateSpeed.omegaFiltered < (1.05 * pStartup->targetSpeed)))
                {
                    /* Increment sector counter if condition is true */
                    pStartup->sectorValidCount++;
                }
                else
                {
                    /* Reset sector counter if condition is false */
                    pStartup->sectorValidCount = 0;
                }

                /* If condition is true for 5 consecutive sectors, switch to estimated commutation */
                if (pStartup->sectorValidCount >= 5)
                {
                    pEstim->closedLoopStartingFlag = 1; /* Set flag: transition to closed loop */
                    pStartup->sectorValidCount = 0;       /* Reset sector counter after switching */
                }
                /* Reset commutation sector change flag for next cycle */
                pStartup->commutationSectorChanged = false;
            }
            else
            {
                /* Do nothing */
            }
        }
        else
        {
            /* Map the BEMF phase sequence */
            if(pEstim->startBemfPhaseMapping == true)
            {
                /* Check if sector has changed from the initial value */
                if(pData->nextCommutationSector != 5)
                {
                    MapBemfPhaseSequence(pEstim);
                }
                else
                {
                    /* Do nothing */
                }
            }
            else
            {
                /* Check if commutation sector is '5' */
                if(pData->nextCommutationSector == 5)
                {
                    /* Start mapping the BEMF phases */
                    pEstim->startBemfPhaseMapping = true;
                }
            }
        }
    }
    else
    {
        
    }
}

/**
* <B> Function: static inline void MCAPP_ComputeStartupReference(MCAPP_FORCED_COMMUTATION_STARTUP_T *pStartup) </B>
*
* @brief Computes startup electrical reference during startup.
*        Ramps the commutation frequency, computes electrical speed,
*        integrates the startup electrical angle, and wraps it
*        within the 0 to 2? range.
*
* @param Pointer to the startup parameter structure.
* @return none.
* @example
* <CODE> MCAPP_ComputeStartupReference(&pStartup); </CODE>
*
*/
static inline void MCAPP_ComputeStartupReference(MCAPP_FORCED_COMMUTATION_STARTUP_T *pStartup)
{
    /* Ramp commutation frequency */
    if (pStartup->commutationFreq < pStartup->finalFreq)
    {
        pStartup->commutationFreq += pStartup->rampStepFreq;
    }
    else
    {
        pStartup->commutationFreq = pStartup->finalFreq;
    }

    /* Compute electrical angular speed */
    pStartup->omegaEle = 2.0f * M_PI * pStartup->commutationFreq;

    /* Integrate startup electrical angle */
    pStartup->thetaSum =
        pStartup->thetaRadian +
        (pStartup->omegaEle * pStartup->Tcy);

    /* Wrap angle to [0, 2PI] */
    if (pStartup->thetaSum >= (2.0f * M_PI))
    {
        pStartup->thetaRadian = pStartup->thetaSum - (2.0f * M_PI);
    }
    else if (pStartup->thetaSum < 0.0f)
    {
        pStartup->thetaRadian = pStartup->thetaSum + (2.0f * M_PI);
    }
    else
    {
        pStartup->thetaRadian = pStartup->thetaSum;
    }
}

/**
* <B> Function: static inline void MCAPP_UpdateCommutationSectorFromAngle(
*               MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pControl) </B>
*
* @brief Determines and updates the commutation sector from
*        the startup electrical angle reference.
*
* @param pControl Pointer to the trapezoidal control parameter structure.
* @return none.
* @example
* <CODE> MCAPP_UpdateCommutationSectorFromAngle(&controlParams); </CODE>
*
*/
static inline void MCAPP_UpdateCommutationSectorFromAngle(
    MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pControl)
{
    MCAPP_FORCED_COMMUTATION_STARTUP_T *pStartup = &pControl->startup;

    /* Determine sector index from electrical angle (60° per sector) */
    pStartup->commutationSectorIndex =
        (uint8_t)(pStartup->thetaRadian / (M_PI / 3.0));

    /* Advance to next commutation sector */
    pStartup->commutationSectorIndex =
        (pStartup->commutationSectorIndex + 1U) % 6U;

    /* Update active commutation sector */
    pStartup->commutationSector =
        pControl->pCommSectorSeq[pStartup->commutationSectorIndex];
    
    /* Check for sector change */
    if(pStartup->prevCommutationSectorIndex != pStartup->commutationSectorIndex)
    {
       /* Commutation sector has changed */
        pStartup->commutationSectorChanged = true;
        /* Update previous commutation sector index */
        pStartup->prevCommutationSectorIndex = pStartup->commutationSectorIndex;
    }
    else
    {
        /* Do nothing */
    }
}

/**
* <B> Function: static inline void MCAPP_ComputeStartupVoltageFeedForward(
*               MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pControl) </B>
*
* @brief Computes startup voltage feed-forward during startup.
*        Estimates required phase voltage from bus current,
*        stator resistance, and back-EMF, then normalizes it
*        to the DC bus to generate a startup duty reference.
*
* @param pControl Pointer to the trapezoidal control parameter structure.
* @return none.
* @example
* <CODE> MCAPP_ComputeStartupVoltageFeedForward(&controlParams); </CODE>
*
*/
static inline void MCAPP_ComputeStartupVoltageFeedForward(
    MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pControl)
{
    MCAPP_FORCED_COMMUTATION_STARTUP_T *pStartup = &pControl->startup;

    /* Compute startup voltage feed-forward */
    float Vff = (pStartup->commutationFreq * pStartup->vfRatio);
    /* Normalize feed-forward voltage to DC bus */
    float dutyNorm = (Vff / pControl->Vdc);

    /* Clamp startup duty to minimum limit */
    if (dutyNorm < pStartup->minstartupDuty)
    {
        pStartup->startupDuty = pStartup->minstartupDuty;
    }
    else
    {
        pStartup->startupDuty = dutyNorm;
    }
}


/**
* <B> Function: void UpdateStartupCommutationStep(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pControl) </B>
*
* @brief Updates the commutation interval and sector during startup.
*        Handles the commutation counter, advances the commutation sector, and
*        ramps down the commutation interval to accelerate the motor until the
*        final interval is reached.
*
* @param pControl Pointer to the trapezoidal control parameter structure.
* @return none.
* @example
* <CODE> UpdateStartupCommutationStep(&controlParams); </CODE>
*
*/
static inline void UpdateStartupCommutationStep(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pControl)
{
    MCAPP_FORCED_COMMUTATION_STARTUP_T *pStartup = &pControl->startup;
    
    /* Compute startup angle reference */
    MCAPP_ComputeStartupReference(pStartup);
    
    /* Update commutation sector from startup electrical angle */
    MCAPP_UpdateCommutationSectorFromAngle(pControl);
    
    /* Compute startup voltage feed-forward duty */
    MCAPP_ComputeStartupVoltageFeedForward(pControl);
}

/**
* <B> Function: void MCAPP_TrapezoidalControlUpdateInverterSwitching(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pControl) </B>
*
* @brief Updates the inverter switching states based on the current commutation sector.
*
* @param pControl Pointer to the trapezoidal control parameter structure.
* @return none.
* @example
* <CODE> MCAPP_TrapezoidalControlUpdateInverterSwitching(&controlParams); </CODE>
*
*/
inline static void MCAPP_TrapezoidalControlUpdateInverterSwitching(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pControl)
{
    /** Check if the commutation sector is valid */
    if((pControl->commutationSector > 0)&&(pControl->commutationSector < 7))
    {
        /* Check if the next sector to commute matches the value obtained from the estimator */
        if(pControl->nextCommutationSector == pControl->commutationSector)
        {            
            /* It's a match, clear the mismatch flag */
            pControl->commutationSectorMissmatched = false;
            /* Update the present commutation sector to the next sector */
            pControl->presentCommutationSector = pControl->nextCommutationSector;
            /* Store the current sector as the previous sector for tracking */
            pControl->previousCommutationSector = pControl->presentCommutationSector;
            /* Increment the next commutation sector index */
            pControl->nextCommutationSectorIndex = (pControl->nextCommutationSectorIndex + 1) % 6;
            /* Update the next commutation sector */
            pControl->nextCommutationSector = pControl->pCommSectorSeq[pControl->nextCommutationSectorIndex];  
            pControl->estimator.blankingTimeCounter = 0;
        }
        else
        {
            /* Sectors do not match, set the mismatch flag */
            pControl->commutationSectorMissmatched = true;
            /* Revert present commutation sector to the previous sector */
            pControl->presentCommutationSector = pControl->previousCommutationSector;
        }

        /* Update the inverter switching pattern */
        switch(pControl->presentCommutationSector)   
        {
            
            case 1:
            /* Commute PhaseA FLOATING, PhaseB DC-, PhaseC DC+(Chopping) */                   
            HAL_PWM_OverrideConfigure_PhaseA(FLOATING);
            HAL_PWM_OverrideConfigure_PhaseB(PWM_OFF);
            HAL_PWM_OverrideConfigure_PhaseC(PWM_ON);
            break;
            case 2:
            /* Commute PhaseA DC-, PhaseB DC+(Chopping), PhaseC FLOATING */
            HAL_PWM_OverrideConfigure_PhaseC(FLOATING);
            HAL_PWM_OverrideConfigure_PhaseA(PWM_OFF);
            HAL_PWM_OverrideConfigure_PhaseB(PWM_ON);               
            break;
            case 3:
            /* Commute PhaseA DC-, PhaseB FLOATING, PhaseC DC+(Chopping) */
            HAL_PWM_OverrideConfigure_PhaseB(FLOATING);
            HAL_PWM_OverrideConfigure_PhaseA(PWM_OFF);
            HAL_PWM_OverrideConfigure_PhaseC(PWM_ON);
            break;
            case 4:
            /* Commute PhaseA DC+(Chopping), PhaseB FLOATING, PhaseC DC- */
            HAL_PWM_OverrideConfigure_PhaseB(FLOATING);
            HAL_PWM_OverrideConfigure_PhaseC(PWM_OFF);
            HAL_PWM_OverrideConfigure_PhaseA(PWM_ON);            
            break;
            case 5:
            /* Commute PhaseA DC+(Chopping), PhaseB DC-, PhaseC FLOATING */
            HAL_PWM_OverrideConfigure_PhaseC(FLOATING);
            HAL_PWM_OverrideConfigure_PhaseB(PWM_OFF);
            HAL_PWM_OverrideConfigure_PhaseA( PWM_ON);              
            break;
            case 6:
            /* Commute PhaseA FLOATING, PhaseB DC+(Chopping), PhaseC DC- */
            HAL_PWM_OverrideConfigure_PhaseA(FLOATING);
            HAL_PWM_OverrideConfigure_PhaseC(PWM_OFF);
            HAL_PWM_OverrideConfigure_PhaseB(PWM_ON);                
            break;
            default:
            /* Disable PWM outputs for other commutation sectors */
            HAL_MC1PWMDisableOutputs(); 
            break;   
        }
    }
    else
    {
        /* Disable PWM outputs for invalid commutation sectors */
        HAL_MC1PWMDisableOutputs();
    }
}

/**
* <B> Function: inline static bool ReadyforClosedLoopTransition(MCAPP_ESTIMATOR_T *pData)  </B>
*
* @brief Function to check the forced commutation startup to closed loop transition
*
* @param Pointer to the data structure containing Estimator parameters.
* @return Transition status.
* 
* @example
* <CODE> ReadyforClosedLoopTransition(&pData); </CODE>
*
*/
inline static bool ReadyforClosedLoopTransition(MCAPP_ESTIMATOR_T *pData)
{
    bool readyStatus;
        
    if(pData->closedLoopFlag == true)
    {
        readyStatus = true;
    }
    else
    {
        readyStatus = false;
    }
    
    return readyStatus;
}

/**
* <B> Function: inline static bool ReadyforClosedLoopStarting(MCAPP_ESTIMATOR_T *pData)  </B>
*
* @brief Function to check the startup to closed loop starting
*
* @param Pointer to the data structure containing Estimator parameters.
* @return Starting status.
* 
* @example
* <CODE> ReadyforClosedLoopStarting(&pData); </CODE>
*
*/
inline static bool ReadyforClosedLoopStarting(MCAPP_ESTIMATOR_T *pData)
{
    bool readyStatus;
        
    if(pData->closedLoopStartingFlag == true)
    {
        readyStatus = true;
    }
    else
    {
        readyStatus = false;
    }
    
    return readyStatus;
}

/**
* <B> Function: inline static void MCAPP_UpdateControlState(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pControl)  </B>
*
 * @brief Updates the control state based on the selected control loop mode.
 *        Sets the control state to current control, speed control, or voltage control
 *        depending on the value of the controlLoop parameter.
 *
 * @param pCtrlParam Pointer to the control parameter structure containing the control loop mode.
 * @param pControl Pointer to the control structure where the control state will be updated.
 * @return none.
* 
* @example
* <CODE> MCAPP_UpdateControlState(&pControl); </CODE>
*
*/
inline static void MCAPP_UpdateControlState(MCAPP_BLDC_TRAPEZOIDAL_CONTROL_T *pControl)
{
    MCAPP_CONTROL_T *pCtrlParam = &pControl->ctrlParam;
    
    /* Check if the selected control loop is current control */
    if( pCtrlParam->controlLoop == CURRENT_CONTROL )
    {
        pControl->controlState = CURRENT_CONTROL_LOOP; /* Set control state to current control loop  */
    }
    /* Check if the selected control loop is speed control */
    else if( pCtrlParam->controlLoop == SPEED_CONTROL )
    {
        pControl->controlState = SPEED_CONTROL_LOOP; /* Set control state to speed control loop  */
    }
    /* Default to open loop voltage control if neither current nor speed control is selected */
    else
    {
        pControl->controlState = VOLTAGE_CONTROL_LOOP; /* Set control state to voltage control loop  */
    }
}
