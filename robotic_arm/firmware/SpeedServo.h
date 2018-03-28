#ifndef SPEED_SERVO_H
#define SPEED_SERVO_H

#include <Arduino.h>
#include <Servo.h>



class SpeedServo
{

public:

	SpeedServo(Servo* servo_ptr);

	~SpeedServo(){}

	void write(int position);

    int current() const { return m_current; }

    int desired() const { return m_desired; }

    bool positive() const { return m_positive; }

    unsigned int counter() const { return m_counter; }

    void setDesired(int desired);

    void update();

    void increment();

    void resetCounter();

private:

	Servo* m_servo;	

    int m_current;

    int m_desired;

    bool m_positive;

    unsigned int m_counter;

};
#endif