/**
 * @file hal_timer1.h
 * @brief Timer1 for 50 µs (20 kHz) system tick on dsPIC33AK.
 */
#ifndef HAL_TIMER1_H
#define HAL_TIMER1_H

void HAL_Timer1_Init(void);
void HAL_Timer1_Start(void);
void HAL_Timer1_Stop(void);

#endif
