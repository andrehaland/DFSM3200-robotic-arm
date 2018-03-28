#ifndef SPEED_SERVO_H
#define SPEED_SERVO_H

#include<ros.h>
#include <std_msgs/Float64.h>

#include <Arduino.h>
#include <Servo.h>



class SpeedServo
{

public:

    /**
     * Constructor takes a pointer to a Servo object
     **/
	SpeedServo(Servo* servo_ptr, ros::NodeHandle* nh_ptr, const char* topic);

	~SpeedServo(){}

    /**
     * Must be called after ros::NodeHandle::initNode()
     **/
    void subscribe();

    /**
     * Wrapper of Servo::write() 
     **/
	void write(int position);

    /**
     * Returns the current position of the servo 
     **/
    int current() const { return m_current; }

    /**
     * Returns the desired position of the servo, 
     * meaning the last set angle
     **/
    int desired() const { return m_desired; }

    void setDesired(int desired);

    unsigned int counter() const { return m_counter; }

    bool positive() const { return m_positive; }

    void update();

    void increment();

    void resetCounter();

    void servoCallback(const std_msgs::Float64& angle);

private:

	Servo* m_servo;	

    ros::NodeHandle* m_nodeHandlePtr;

    ros::Subscriber<std_msgs::Float64, SpeedServo> m_jointSubscriber;

    int m_current;

    int m_desired;

    bool m_positive;

    unsigned int m_counter;



};
#endif