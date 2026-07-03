/* 
 * File:   variable_trigger_types.h
 * Author: I76316
 *
 * Created on May 20, 2026, 1:08 PM
 */

#ifndef VARIABLE_TRIGGER_TYPES_H
#define	VARIABLE_TRIGGER_TYPES_H

#ifdef	__cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS ">
   
typedef struct
{

    uint16_t state; /* Variable for trigger state */
    bool multipleSampling; /* Flag to indicate state of multiple sampling */
}MCAPP_VARIABLE_TRIGGER;
    
// </editor-fold>


#ifdef	__cplusplus
}
#endif

#endif	/* VARIABLE_TRIGGER_TYPES_H */

