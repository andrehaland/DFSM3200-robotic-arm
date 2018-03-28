#include "Timer.h"
#include <Arduino.h>

Timer::Timer()
{
    reset();
}

void Timer::reset()
{
    m_startTime = millis();
}

unsigned int Timer::millisSinceReset() const
{
    return millis() - m_startTime;
}