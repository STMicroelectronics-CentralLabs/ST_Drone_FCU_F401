#ifndef __TIMER_H__
#define __TIMER_H__

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

typedef volatile struct _tUserTimer
{
    uint32_t    target_tick;
    uint32_t    interval;
    uint32_t    flag;
    uint32_t    flag2;
    uint32_t    event_cnt;
}tUserTimer;

void SetupTimer(tUserTimer *t, uint32_t interval);
void StartTimer(tUserTimer *t);
void StopTimer(tUserTimer *t);
void ClearTimer(tUserTimer *t);
void TimerProcess(tUserTimer *t);
uint32_t isTimerEventExist(tUserTimer *t);

void User_Timer_Callback(void);

#endif /* __TIMER_H__ */
