// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file measure.h
 *
 * @brief This header file lists the functions and definitions
 * for signal conditioning of measured analog feedback signals.
 *
 * Component: ANALOG SIGNAL MEASURE
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

#ifndef __MEASURE_H
#define __MEASURE_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include "mc1_user_params.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="DEFINITIONS ">

#define OFFSET_COUNT_BITS   (int16_t)10
#define OFFSET_COUNT_MAX    (int16_t)(1 << OFFSET_COUNT_BITS)

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS ">

typedef struct
{
    float
        Ia_actual,
        Ib_actual,
        Ic_actual,
        Ibus_actual;
    
    int16_t
        offsetIa,       /* A phase current offset */
        offsetIb,       /* B phase current offset */
        offsetIc,       /* C phase current offset */
        offsetIbus,     /* BUS current offset */
        Ia,             /* A phase Current Feedback */
        Ib,             /* B phase Current Feedback */
        Ic,             /* C phase Current Feedback */
        Ibus,           /* Bus current Feedback */
        counter,        /* counter */
        status;         /* flag to indicate offset measurement completion */
            
    int32_t
        sumIa,          /* Accumulation of Ia */
        sumIb,          /* Accumulation of Ib */
        sumIc,          /* Accumulation of Ic */
        sumIbus;        /* Accumulation of Ibus */

} MCAPP_MEASURE_CURRENT_T;

typedef struct
{
    int16_t input;
    uint16_t index;
    uint16_t maxIndex;
    uint16_t scaler;
    int16_t avg;
    int32_t sum;
    uint16_t status;
    
}MCAPP_MEASURE_AVG_T;

typedef struct
{
    int16_t
        count;              /* Measured DC Bus Voltage value in counts. */
    
    float
        value;              /* Measured value of DC Bus Voltage. */
} MCAPP_MEASURE_VDC_T;

typedef struct
{
    float
        Va_actual,
        Vb_actual,
        Vc_actual;
    
    int16_t
        Va,                 /* A phase terminal voltage w.r.t. DC_Neg */
        Vb,                 /* B phase terminal voltage w.r.t. DC_Neg */
        Vc,                 /* C phase terminal voltage w.r.t. DC_Neg */
        status,             /* Status if phase voltages are available */
        samplingFactor;     /* Ratio of sampling time to ADC interrupt */
}MCAPP_MEASURE_PHASEVOLT_T;

typedef struct
{
    int16_t 
        sharedCoreChannelNumber;    /* Shared core channel number for switching */
    float
        measurePot;         /* Measure potentiometer */
    
    MCAPP_MEASURE_CURRENT_T
        measureCurrent;             /* Current measurement parameters */
        
    MCAPP_MEASURE_VDC_T
        measureVdc;                 /* DC Bus Sensing parameters */
    
    MCAPP_MEASURE_PHASEVOLT_T
        measurePhaseVolt;   /* Phase voltage measurement parameters */

}MCAPP_MEASURE_T;

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
void MCAPP_MeasureInit (MCAPP_MEASURE_T *);
void MCAPP_MeasureCurrentOffset (MCAPP_MEASURE_T *);
void MCAPP_MeasureCurrentCalibrate (MCAPP_MEASURE_T *);
void MCAPP_MeasureCurrentInit (MCAPP_MEASURE_T *);
int16_t MCAPP_MeasureCurrentOffsetStatus (MCAPP_MEASURE_T *);
void MCAPP_MeasureActualPhaseVoltage(MCAPP_MEASURE_T *);

void MCAPP_MeasureAvgInit(MCAPP_MEASURE_AVG_T *,uint16_t );
int16_t MCAPP_MeasureAvg(MCAPP_MEASURE_AVG_T *);
// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif /* end of __MEASURE_H */
