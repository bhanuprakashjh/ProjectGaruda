/* 
 * File:   variable_trigger.h
 * Author: I76316
 *
 * Created on May 20, 2026, 1:08 PM
 */

#ifndef VARIABLE_TRIGGER_H
#define	VARIABLE_TRIGGER_H

#ifdef	__cplusplus
extern "C" {
#endif
// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include "variable_trigger_types.h"
#include "mc1_calc_params.h"
// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/CONSTANTS ">

/* Trigger level 2 to level 1 threshold */
#define VARIABLE_TRIGGER_T2_TO_T1_THRESHOLD     (BEMF_RC_RISE_TIME_COUNTS)

/* Trigger level 1 to level 2 threshold */
#define VARIABLE_TRIGGER_T1_TO_T2_THRESHOLD     (1.25f * VARIABLE_TRIGGER_T2_TO_T1_THRESHOLD)
    
/* Trigger level 3 to level 2 threshold */
#define VARIABLE_TRIGGER_T3_TO_T2_THRESHOLD     (uint32_t)(2.5f * VARIABLE_TRIGGER_T2_TO_T1_THRESHOLD)
    
/* Trigger level 2 to level 3 threshold */
#define VARIABLE_TRIGGER_T2_TO_T3_THRESHOLD     (uint32_t)(3U * VARIABLE_TRIGGER_T2_TO_T1_THRESHOLD)
    
/* PWM generator trigger register used for ADC/BEMF sampling */
#define VARIABLE_TRIGGER_REG         PG2TRIGAbits.TRIGA

/* Trigger position scaling when PWM duty is low */
#define VARIABLE_TRIGGER_SCALE_STATE1   0.6f

/* Trigger position scaling when PWM duty is medium */
#define VARIABLE_TRIGGER_SCALE_STATE2   0.8f

/* Trigger position scaling when PWM duty is high */
#define VARIABLE_TRIGGER_SCALE_STATE3   0.9f
    
/* Variable trigger states */
typedef enum 
{ 
    VARIABLE_TRIGGER_STATE_1    = 0,
    VARIABLE_TRIGGER_STATE_2    = 1,
    VARIABLE_TRIGGER_STATE_3    = 2,
}VARIABLE_TRIGGER_STATES;    
        
// </editor-fold>    
    
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
    
inline static void VariableTriggerInit(MCAPP_VARIABLE_TRIGGER *pData)
{
    pData->state = 0;
    pData->multipleSampling = 0;
}

inline static void VariableTriggerUpdate(MCAPP_VARIABLE_TRIGGER *pData, uint32_t pwmDuty)
{
    switch (pData->state)
    {
        case VARIABLE_TRIGGER_STATE_1:
        {
            if (pwmDuty >= VARIABLE_TRIGGER_T1_TO_T2_THRESHOLD)
            {
                pData->state = VARIABLE_TRIGGER_STATE_2;
            }

            VARIABLE_TRIGGER_REG = (uint32_t)(VARIABLE_TRIGGER_SCALE_STATE1 * pwmDuty);
            break;
        }

        case VARIABLE_TRIGGER_STATE_2:
        {
            if (pwmDuty < VARIABLE_TRIGGER_T2_TO_T1_THRESHOLD)
            {
                pData->state = VARIABLE_TRIGGER_STATE_1;
            }
            else if (pwmDuty >= VARIABLE_TRIGGER_T2_TO_T3_THRESHOLD)
            {
                pData->state = VARIABLE_TRIGGER_STATE_3;
            }

            VARIABLE_TRIGGER_REG = (uint32_t)(VARIABLE_TRIGGER_SCALE_STATE2 * pwmDuty);
            break;
        }

        case VARIABLE_TRIGGER_STATE_3:
        {
            /* Allow transition back to LOW or MID based on duty */
            if (pwmDuty < VARIABLE_TRIGGER_T2_TO_T1_THRESHOLD)
            {
                pData->state = VARIABLE_TRIGGER_STATE_1;
            }
            else if (pwmDuty < VARIABLE_TRIGGER_T3_TO_T2_THRESHOLD)
            {
                pData->state = VARIABLE_TRIGGER_STATE_2;
            }

            VARIABLE_TRIGGER_REG =
                (uint32_t)(VARIABLE_TRIGGER_SCALE_STATE3 * pwmDuty);
            break;
        }

        default:
        {
            pData->state = VARIABLE_TRIGGER_STATE_1;
            VARIABLE_TRIGGER_REG = (uint32_t)(VARIABLE_TRIGGER_SCALE_STATE1 * pwmDuty);
            break;
        }
    }
}

static inline bool VariableTriggerSample1(const MCAPP_VARIABLE_TRIGGER *pTrigger)
{
    return (pTrigger->state == VARIABLE_TRIGGER_STATE_1);
}

static inline bool VariableTriggerSample2(const MCAPP_VARIABLE_TRIGGER *pTrigger)
{
    return (pTrigger->state == VARIABLE_TRIGGER_STATE_2);
}

static inline bool VariableTriggerSample3(const MCAPP_VARIABLE_TRIGGER *pTrigger)
{
    return (pTrigger->state == VARIABLE_TRIGGER_STATE_3);
}

static inline void EnableMultipleSampling(MCAPP_VARIABLE_TRIGGER *pData)
{
    pData->multipleSampling = true;
}

static inline void DisableMultipleSampling(MCAPP_VARIABLE_TRIGGER *pData)
{
    pData->multipleSampling = false;
}

static inline bool IsMultipleSamplingEnabled(const MCAPP_VARIABLE_TRIGGER *pData)
{
    return pData->multipleSampling;
}
// </editor-fold>

#ifdef	__cplusplus
}
#endif

#endif	/* VARIABLE_TRIGGER_H */

