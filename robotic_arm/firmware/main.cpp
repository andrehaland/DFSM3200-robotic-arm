#include <ros.h>
#include <std_msgs/Float64.h>

#include "servo_msgs/JointAngles.h"

#include <Arduino.h>

#include <Servo.h>

#include "SpeedServo.h"
#include "Timer.h"

ros::NodeHandle nh;

// Sets the speed of the motors -- Lower numbers is faster
const unsigned int NUM_TICKS = 350;

// Initialize Servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

// Initialize SpeedServo objects
SpeedServo speed_servo1(&servo1, &nh, "/5dof_robot/joint2/command");
SpeedServo speed_servo2(&servo2, &nh, "/5dof_robot/joint3/command");
SpeedServo speed_servo3(&servo3, &nh, "/5dof_robot/joint4/command");
SpeedServo speed_servo4(&servo4, &nh, "/5dof_robot/joint5/command");
SpeedServo speed_servo5(&servo5, &nh, "/5dof_robot/joint6/command");


void setup()
{
  	nh.initNode();

	//Attach to pins
	servo1.attach(2);
	servo2.attach(3);
	servo3.attach(4);
	servo4.attach(5);
	servo5.attach(6);

	// Set servos to upright position
	/*speed_servo1.write(90);
	speed_servo2.write(90);
	speed_servo3.write(90);
	speed_servo4.write(90);
	speed_servo5.write(90);*/

	// Subscribe to joint topics
	speed_servo1.subscribe();
	speed_servo2.subscribe();
	speed_servo3.subscribe();
	speed_servo4.subscribe();
	speed_servo5.subscribe();
}


void loop()
{
	// Update counter of every servo
	speed_servo1.increment();
	speed_servo2.increment();
	speed_servo3.increment();
	speed_servo4.increment();
	speed_servo5.increment();
	
	// Update all servo motors
	if(speed_servo1.counter() >= NUM_TICKS)
	{
		speed_servo1.update();
		speed_servo1.resetCounter();
	}
	if(speed_servo2.counter() >= NUM_TICKS)
	{
		speed_servo2.update();
		speed_servo2.resetCounter();
	}
	if(speed_servo3.counter() >= NUM_TICKS)
	{
		speed_servo3.update();
		speed_servo3.resetCounter();
	}
	if(speed_servo4.counter() >= NUM_TICKS)
	{
	
		speed_servo4.update();
		speed_servo4.resetCounter();
	}
	if(speed_servo5.counter() >= NUM_TICKS)
	{
		speed_servo5.update();
		speed_servo5.resetCounter();
	}
	

	// Handle callbacks
  	nh.spinOnce();
}
