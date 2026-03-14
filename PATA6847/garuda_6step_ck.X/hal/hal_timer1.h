/**
 * @file hal_timer1.h
 * @brief Timer1 for 100 µs tick on dsPIC33CK.
 */
#ifndef HAL_TIMER1_H
#define HAL_TIMER1_H

void HAL_Timer1_Init(void);
void HAL_Timer1_Start(void);
void HAL_Timer1_Stop(void);

#endif
