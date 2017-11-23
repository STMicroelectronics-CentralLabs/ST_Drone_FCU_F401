#include "timer.h"



tUserTimer tim;

void SetupTimer(tUserTimer *t, uint32_t interval)
{
    t->interval = interval;
    t->flag = 0;
    t->flag2 = 0;
    t->event_cnt = 0;
}

void StartTimer(tUserTimer *t)
{
    t->target_tick = HAL_GetTick() + t->interval;
    t->flag = 1;
}

void StopTimer(tUserTimer *t)
{
    t->flag = 0;
}

void ClearTimer(tUserTimer *t)
{
    t->event_cnt = 0;
}

void TimerProcess(tUserTimer *t)
{
    if (t->flag && HAL_GetTick() >= t->target_tick)
    {
        t->event_cnt++;
        t->target_tick = t->target_tick + t->interval;
    }
}

uint32_t isTimerEventExist(tUserTimer *t)
{
    return t->event_cnt;
}

void User_Timer_Callback(void)
{
    TimerProcess(&tim);
    // Add additional timer processing if more user timers
}