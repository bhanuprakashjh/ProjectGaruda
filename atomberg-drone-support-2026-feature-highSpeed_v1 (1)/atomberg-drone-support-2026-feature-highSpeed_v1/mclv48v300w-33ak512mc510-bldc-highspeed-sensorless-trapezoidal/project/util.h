// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file util.h
 *
 * @brief 
 *
 *
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">

/*******************************************************************************
* SOFTWARE LICENSE AGREEMENT
* 
* © [2024] Microchip Technology Inc. and its subsidiaries
* 
* Subject to your compliance with these terms, you may use this Microchip 
* software and any derivatives exclusively with Microchip products. 
* You are responsible for complying with third party license terms applicable to
* your use of third party software (including open source software) that may 
* accompany this Microchip software.
* 
* Redistribution of this Microchip software in source or binary form is allowed 
* and must include the above terms of use and the following disclaimer with the
* distribution and accompanying materials.
* 
* SOFTWARE IS "AS IS." NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
* APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,
* MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL 
* MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR 
* CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO
* THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY
* LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL
* NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR THIS
* SOFTWARE
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/
// </editor-fold>

#ifndef UTIL_H
#define	UTIL_H

#ifdef	__cplusplus
extern "C" {
#endif


// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <math.h>

// </editor-fold>    

    
// <editor-fold defaultstate="expanded" desc="Definitions ">
        
#define SQRT_3         1.732050807568877f  /* sqrt(3) */
#define Q15_MAX        32767.0f
#define Q14_MAX        16384.0f
#define Q30_MAX        1073741824.0f
  
#define DEGREE_TO_RADIAN        (float)(M_PI/180.0f)
#define RADIAN_TO_DEGREE        (float)(180.0f/M_PI)
#define RAD_PER_SEC_TO_RPM      (float)(60.0f/(2.0f*M_PI))
#define RPM_TO_RAD_PER_SEC      (float)((2.0f*M_PI)/60.0f)
#define Q15_TO_RADIAN           (float)(M_PI/Q15_MAX)     
#define RPM_TO_ELEC_RAD_PER_S   (float)(M_PI/30.0f)    
#define RADIAN_TO_Q15           (float)(Q15_MAX/M_PI) 
   
// </editor-fold>

/** Signed 16/32 bit alias union */
typedef union tagSX1632_t
{
    struct
    {
        uint16_t lo;    /** lower 16 bits */
        int16_t  hi;    /** upper 16 bits */ 
    } x16;              /** access as 16-bit values */
    int32_t x32;        /** access as 32-bit values */ 
} sx1632_t;
   

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/**
 *
 * 
 * @param a first input
 * @return (x * x)
 */
inline static float SquareFloat(const float x)
{
    return ( x * x );
}

/**
 *
 * 
 * @param a first input
 * @return (x * x)
 */
inline static float CubeFloat(const float x)
{
    return ( x * x * x );
}

/**
* <B> Function: SaturateFloat( float * const, const float, const float) </B>
*
* @brief Function saturates a float value between the minimum and maximum values.
* 
* @param pointer to the input.
* @param minimum value.
* @param maximum value.
* @return none.
* 
* @example
* <CODE> SaturateFloat( &x, -1.0f, 0.0f ); </CODE>
*
*/
inline static void SaturateFloat( float * const input, const float min, const float max  )
{
    if( max < (*input ))
    {
        *input = max;
    }
    else if( min > (*input ))
    {
        *input = min;
    }
    else
    {

    }
}

/**
* <B> Function:  LowPassFilter (float ,float ,float * ) </B>
*
* @brief Function runs 1st order backward Euler filter operation on the input.
*        
* @param input signal
* @param Filter coefficient value
* @param pointer to output signal
*
* @example
* <CODE> LowPassFilter (input, filterCoeff,*output); </CODE>
*
*/

inline static void LowPassFilter (float input, float filterCoeff, float* output)
{
	*output = *output+ ((input - *output) * filterCoeff) ;
}


/**
* <B> Function:  RampToTarget (float*, float, float) </B>
*
* @brief Function generates a linear ramp from the current value to the target value with a specified step size.
*        
* @param current: pointer to the current value, which will be updated.
* @param target: the target value to ramp towards.
* @param step: the maximum change per function call (step size).
*
* @example
* <CODE> RampToTarget(&currentValue, targetValue, stepSize); </CODE>
*
*/
inline static void RampToTarget(float *current, float target, float step) 
{
    float delta = target - *current;

    if (fabs(delta) <= step) {
        *current = target;  // Close enough ? snap to target
    } else {
        *current += (delta > 0) ? step : -step;  // Move in correct direction
    }
}

/**
 * @brief Symmetric rate limiter (same rate for increase and decrease).
 *
 * Limits how fast a value can change toward a target.
 * The allowed change per call is the same in both directions.
 *
 * @param current Current value
 * @param target  Desired target value
 * @param step    Maximum change per call (applies symmetrically)
 * @return        Updated value after symmetric rate limiting
 */
static inline float RateLimitSym(float current, float target, float step)
{
    float error = target - current;

    if (fabsf(error) <= step) {
        return target;   // Close enough ? snap
    }

    return current + ((error > 0.0f) ? step : -step);
}

/**
 * @brief Asymmetric rate limiter (different rates for increase and decrease).
 *
 * Allows rising and falling transitions to have different speeds.
 * Commonly used for fast stop / slow start behavior.
 *
 * @param current   Current value
 * @param target    Desired target value
 * @param stepUp    Maximum increase per call
 * @param stepDown  Maximum decrease per call
 * @return          Updated value after asymmetric rate limiting
 */
static inline float RateLimitAsym(float current,
                                  float target,
                                  float stepUp,
                                  float stepDown)
{
    float error = target - current;

    if (error > 0.0f) {
        return (error <= stepUp) ? target : current + stepUp;
    } else {
        return (-error <= stepDown) ? target : current - stepDown;
    }
}

// </editor-fold>

#ifdef	__cplusplus
}
#endif

#endif	/* UTIL_H */

