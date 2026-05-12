/**
 * @file hal_timer1.h
 * @brief Timer1 for 50 µs (20 kHz) system tick on dsPIC33AK (AK port).
 *
 * Forked from CK `../../garuda_6step_ck.X/hal/hal_timer1.h` — API
 * preserved so `garuda_service.c` callers don't need to change.
 */
#ifndef HAL_TIMER1_H
#define HAL_TIMER1_H

void HAL_Timer1_Init(void);
void HAL_Timer1_Start(void);
void HAL_Timer1_Stop(void);

#endif
