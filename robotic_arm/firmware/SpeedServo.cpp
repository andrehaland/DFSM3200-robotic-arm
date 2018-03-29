#include "SpeedServo.h"
#include "Conversion.h"

SpeedServo::SpeedServo(Servo* servo_ptr,ros::NodeHandle* nh_ptr, const char* topic)
: m_counter(0), m_positive(true), m_current(90), m_desired(90),
  m_jointSubscriber(topic, &SpeedServo::servoCallback, this)
{
    m_nodeHandlePtr = nh_ptr;
    m_servo = servo_ptr;
}


void SpeedServo::subscribe()
{
    m_nodeHandlePtr->subscribe(m_jointSubscriber);
}

void SpeedServo::write(int position)
{
    m_current = position;
    m_servo->write(position);
}

void SpeedServo::setDesired(int desired)
{
    m_desired = desired;

    // Find out which way the servo shall rotate -- positive meaning it will increment from current
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
    // If the servo is not in the desired positon 
    if(m_desired != m_current)
    {
        // Increment or decrement the servo position by 1
        int new_angle;
        if(m_positive)
            new_angle = m_current + 1;
        else
            new_angle = m_current - 1;

        this->write(new_angle);
    }

}

void SpeedServo::servoCallback(const std_msgs::Float64& angle)
{
    //this->setDesired(static_cast<int>(angle.data));
    this->setDesired(static_cast<int>(conversion::radToDeg(angle.data + M_PI / 2.0)));
}