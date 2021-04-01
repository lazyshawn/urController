
#ifndef SYSTEM_TIME_H
#define SYSTEM_TIME_H

#include <ctime>
#include <cstdio>

#define NSEC_PER_SEC (1000000000)

double GetCurrentTime();
void ResetTime();
void ResetTimerCounter();
void TimeCounterAdd();
int GetTimerCounter();
double GetOffsetTime();
void SetStartTime(double time);

#endif
