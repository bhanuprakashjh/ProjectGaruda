// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file measure.c
 *
 * @brief This module has functions for signal conditioning of measured
 *        analog feedback signals.
 *
 * Component: MEASURE
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

#include "measure.h"
#include "filter.h"
#include "mc1_calc_params.h"
// </editor-fold>

/**
* <B> Function: MCAPP_MeasureInit(MCAPP_MEASURE_T *)  </B>
*
* @brief Function to reset variables used for measurements.
*        .
*
* @param Pointer to the data structure containing measured quantities.
* @return none.
* @example
* <CODE> MCAPP_MeasureInit(&pMotorInputs); </CODE>
*
*/
void MCAPP_MeasureInit(MCAPP_MEASURE_T *pMotorInputs)
{
    MCAPP_MeasureCurrentInit(pMotorInputs);
    MCAPP_MovingAvgFilterInit();
}

/**
* <B> Function: MCAPP_MeasureCurrentInit(MCAPP_MEASURE_T *)  </B>
*
* @brief Function to reset variables used for current offset measurement.
*        
*
* @param Pointer to the data structure containing measured currents.
* @return none.
*
* @example
* <CODE> MCAPP_MeasureCurrentInit(&pMotorInputs); </CODE>
*
*/
void MCAPP_MeasureCurrentInit(MCAPP_MEASURE_T *pMotorInputs)
{
    MCAPP_MEASURE_CURRENT_T *pCurrent;
    
    pCurrent = &pMotorInputs->measureCurrent;
    
    pCurrent->counter = 0;
    pCurrent->sumIa = 0;
    pCurrent->sumIb = 0;
    pCurrent->sumIc = 0;
    pCurrent->sumIbus = 0;
    pCurrent->status = 0;
}

/**
* <B> Function: MCAPP_MeasureCurrentOffset(MCAPP_MEASURE_CURRENT_T *)  </B>
*
* @brief Function to compute current offset after measuring specified number of
*        current samples and averaging them.
*        
* @param Pointer to the data structure containing measured current.
* @return none.
* 
* @example
* <CODE> MCAPP_MeasureCurrentOffset(&pMotorInputs); </CODE>
*
*/
void MCAPP_MeasureCurrentOffset(MCAPP_MEASURE_T *pMotorInputs)
{
    MCAPP_MEASURE_CURRENT_T *pCurrent;
    
    pCurrent = &pMotorInputs->measureCurrent;
#ifdef ENABLE_PHASE_CURRENT_MEASUREMENT    
    pCurrent->sumIa += pCurrent->Ia;
    pCurrent->sumIb += pCurrent->Ib;
    pCurrent->sumIc += pCurrent->Ic;
#endif
    pCurrent->sumIbus += pCurrent->Ibus;
    pCurrent->counter++;

    if (pCurrent->counter >= OFFSET_COUNT_MAX)
    {
        pCurrent->offsetIa = (int16_t)(pCurrent->sumIa >> OFFSET_COUNT_BITS);
        pCurrent->offsetIb = (int16_t)(pCurrent->sumIb >> OFFSET_COUNT_BITS);
        pCurrent->offsetIc = (int16_t)(pCurrent->sumIc >> OFFSET_COUNT_BITS);
        pCurrent->offsetIbus =
            (int16_t)(pCurrent->sumIbus >> OFFSET_COUNT_BITS);

        pCurrent->counter = 0;
        pCurrent->sumIa = 0;
        pCurrent->sumIb = 0;
        pCurrent->sumIc = 0;
        pCurrent->sumIbus = 0;
        pCurrent->status = 1;
    }

}
/**
* <B> Function: MCAPP_MeasureCurrentCalibrate(MCAPP_MEASURE_T *)  </B>
*
* @brief Function to compensate offset from measured current samples.
*        .
* @param Pointer to the data structure containing measured current.
* @return none.
* @example
* <CODE> MCAPP_MeasureCurrentCalibrate(&pMotorInputs); </CODE>
*
*/
void MCAPP_MeasureCurrentCalibrate(MCAPP_MEASURE_T *pMotorInputs)
{
    MCAPP_MEASURE_CURRENT_T *pCurrent;
    
    pCurrent = &pMotorInputs->measureCurrent;
    
    /* offset correction*/
#ifdef ENABLE_PHASE_CURRENT_MEASUREMENT
    pCurrent->Ia = pCurrent->Ia - pCurrent->offsetIa;
    pCurrent->Ib = pCurrent->Ib  - pCurrent->offsetIb;
    pCurrent->Ic = pCurrent->Ic  - pCurrent->offsetIc;
    /*Convert ADC Counts to real value*/
    pCurrent->Ia_actual = (float)(pCurrent->Ia * ADC_CURRENT_SCALE);
    pCurrent->Ib_actual = (float)(pCurrent->Ib * ADC_CURRENT_SCALE);
    pCurrent->Ic_actual = (float)(pCurrent->Ic * ADC_CURRENT_SCALE);
#endif
    
    pCurrent->Ibus = pCurrent->Ibus - pCurrent->offsetIbus;
    /*Convert ADC Counts to real value*/
    pCurrent->Ibus_actual = (float)(pCurrent->Ibus * ADC_CURRENT_SCALE);
}
/**
* <B> Function: MCAPP_MeasureCurrentOffsetStatus(MCAPP_MEASURE_T *)  </B>
*
* @brief Function to compensate offset from measured current samples.
*        
* @param Pointer to the data structure containing measured current.
* @return status bit after computing offset.
* 
* @example
* <CODE> MCAPP_MeasureCurrentOffsetStatus(&current); </CODE>
*
*/
int16_t MCAPP_MeasureCurrentOffsetStatus (MCAPP_MEASURE_T *pMotorInputs)
{
    return pMotorInputs->measureCurrent.status;
}

/**
* <B> Function: MCAPP_MeasureActualPhaseVoltage(MCAPP_MEASURE_T *)  </B>
*
* @brief Function to represent phase voltages in real values.
*        .
* @param Pointer to the data structure containing measured phase voltages.
* @return none.
* @example
* <CODE> MCAPP_MeasureActualPhaseVoltage(&pMotorInputs); </CODE>
*
*/
void MCAPP_MeasureActualPhaseVoltage(MCAPP_MEASURE_T *pMotorInputs)
{
    MCAPP_MEASURE_PHASEVOLT_T *pVoltage;
    
    pVoltage = &pMotorInputs->measurePhaseVolt;
    
    pVoltage->Va_actual = (float)(pVoltage->Va * ADC_VOLTAGE_SCALE);
    pVoltage->Vb_actual = (float)(pVoltage->Vb * ADC_VOLTAGE_SCALE);
    pVoltage->Vc_actual = (float)(pVoltage->Vc * ADC_VOLTAGE_SCALE);
}
/**
* <B> Function: MCAPP_MeasureAvgInit(MCAPP_MEASURE_AVG_T *, uint16_t )          </B>
*
* @brief Function to initialize the moving average filter
*
* @param Pointer to the data structure containing filter data. 
* @param moving average filter scaler 
* @return none.
* 
* @example
* <CODE> MCAPP_MeasureAvgInit(&filterData, scaler); </CODE>
*
*/
void MCAPP_MeasureAvgInit(MCAPP_MEASURE_AVG_T *pFilterData,uint16_t scaler)
{
    pFilterData->scaler = scaler;
    pFilterData->maxIndex = (uint16_t)((1 << scaler));
    pFilterData->index = 0;
    pFilterData->sum = 0;
    pFilterData->status = 0;
}

/**
* <B> Function: int16_t MCAPP_MeasureAvg(MCAPP_MEASURE_AVG_T *)         </B>
*
* @brief Function implementing moving average filter 
*
* @param Pointer to the data structure containing filter data.
* @return none.
* 
* @example
* <CODE> MC_MovingAvgFilter(&filterData);                          </CODE>
*
*/
int16_t MCAPP_MeasureAvg(MCAPP_MEASURE_AVG_T *pFilterData)
{    
    pFilterData->sum += pFilterData->input;

    if (pFilterData->index < pFilterData->maxIndex)
    {
        pFilterData->index++;
    }
    else
    {
        pFilterData->avg = pFilterData->sum >> pFilterData->scaler; 
        pFilterData->sum = 0;
        pFilterData->index = 0;
        pFilterData->status = 1;
    }
    return pFilterData->avg;
}

// </editor-fold>