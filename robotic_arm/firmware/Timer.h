#ifndef TIMER_H
#define TIMER_H


class Timer
{
public:

    Timer();

    ~Timer(){}

    unsigned int millisSinceReset() const;

    void reset();

private:

    unsigned int m_startTime;

};
#endif