#include "SpeedServo.h"

SpeedServo::SpeedServo(Servo* servo_ptr): m_counter(0), m_positive(true), m_desired(90)
{
    m_servo = servo_ptr;
}


void SpeedServo::write(int position)
{
    m_current = position;
    m_servo->write(position);
}

void SpeedServo::setDesired(int desired)
{
    m_desired = desired;

    m_positive = ((m_desired - m_current) > 0) ? true : false;
}

void SpeedServo::increment()
{
    ++m_counter;
}

void SpeedServo::resetCounter()
{
    m_counter = 0;
}


void SpeedServo::update()
{
    if(m_desired != m_current)
    {
        int new_angle;
        if(m_positive)
            new_angle = m_current + 1;
        else
            new_angle = m_current - 1;

        this->write(new_angle);
    }

}