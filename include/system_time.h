
#ifndef SYSTEM_TIME_H
#define SYSTEM_TIME_H

#include <ctime>
#include <cstdio>
#include "common.h"

double GetCurrentTime();
void ResetTime();
void ResetTimerCounter();
void TimeCounterAdd();
int GetTimerCounter();
double GetOffsetTime();
void SetStartTime(double time);

#endif
