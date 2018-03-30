#ifndef SPEED_SERVO_H
#define SPEED_SERVO_H

#include<ros.h>
#include <std_msgs/Float64.h>

#include <Arduino.h>
#include <Servo.h>



/**
 * This class is used to better control the speed of the servo motors 
 * Each object remembers the last written angle, and uses this variable
 * to slowly increment the write-value
 **/
class SpeedServo
{

public:

    /**
     * Constructor takes a pointer to a Servo object
     **/
	SpeedServo(Servo* servo_ptr, ros::NodeHandle* nh_ptr, const char* topic);

    /**
     * Default destructor
     **/
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

    /**
     * Set the desired angle 
     **/
    void setDesired(int desired);

    /**
     * Returns the value of the counter 
     **/
    unsigned int counter() const { return m_counter; }

    /**
     * Returns a bool telling whether the motor is moving
     * in a positive direction, meaning next angle > current angle 
     **/
    bool positive() const { return m_positive; }

    /**
     * Check if motor is in desired position. If not, it should slowly
     * move towards the desired position until it reaches it 
     **/
    void update();

    /**
     * Increment counter 
     **/
    void increment();

    /**
     * Reset the counter
     **/
    void resetCounter();

    /**
     * Callback function listening to the joint topic
     * Used the recieved data to set desired angle
     **/
    void servoCallback(const std_msgs::Float64& angle);

private:

    /**
     * Pointer to Servo object
     **/
	Servo* m_servo;	

    /**
     * Pointer to NodeHandle used to subscribe to topics 
     **/
    ros::NodeHandle* m_nodeHandlePtr;

    /**
     * Subscribers to joint angles 
     **/
    ros::Subscriber<std_msgs::Float64, SpeedServo> m_jointSubscriber;

    /**
     * The last written joint angle 
     **/
    int m_current;

    /**
     * The desired joint angle 
     **/
    int m_desired;

    /**
     * What directions the joint should move 
     **/
    bool m_positive;

    /**
     * Counter used to adjust speed of the joint movement 
     **/
    unsigned int m_counter;
};
#endif