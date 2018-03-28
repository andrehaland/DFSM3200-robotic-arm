#include <ros.h>
#include <std_msgs/Float64.h>

#include "servo_msgs/JointAngles.h"

#include <Arduino.h>

#include <Servo.h>

#include "SpeedServo.h"

ros::NodeHandle nh;

const unsigned int NUM_TICKS = 500;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

SpeedServo speed_servo1(&servo1);
SpeedServo speed_servo2(&servo2);
SpeedServo speed_servo3(&servo3);
SpeedServo speed_servo4(&servo4);
SpeedServo speed_servo5(&servo5);


float radToDeg(float rad)
{
	return (rad * 180.0f) / PI;
}



void servo1Callback(const std_msgs::Float64& angle)
{
	speed_servo1.setDesired(static_cast<int>(angle.data));
	//speed_servo1.setDesired(static_cast<int>(radToDeg(angle.data + (PI / 2))));
}

void servo2Callback(const std_msgs::Float64& angle)
{
	speed_servo2.setDesired(static_cast<int>(angle.data));
	//speed_servo2.setDesired(static_cast<int>(radToDeg(angle.data + (PI / 2))));
}

void servo3Callback(const std_msgs::Float64& angle)
{
	speed_servo3.setDesired(static_cast<int>(angle.data));
	//speed_servo3.setDesired(static_cast<int>(radToDeg(angle.data + (PI / 2))));
}

void servo4Callback(const std_msgs::Float64& angle)
{
	speed_servo4.setDesired(static_cast<int>(angle.data));
	//speed_servo4.setDesired(static_cast<int>(radToDeg(angle.data + (PI / 2))));
}

void servo5Callback(const std_msgs::Float64& angle)
{
	speed_servo5.setDesired(static_cast<int>(angle.data));
	//speed_servo5.setDesired(static_cast<int>(radToDeg(angle.data + (PI / 2))));
}


ros::Subscriber<std_msgs::Float64> joint1("/simple_robotic_model/joint2_position_controller/command", &servo1Callback);
ros::Subscriber<std_msgs::Float64> joint2("/simple_robotic_model/joint3_position_controller/command", &servo2Callback);
ros::Subscriber<std_msgs::Float64> joint3("/simple_robotic_model/joint4_position_controller/command", &servo3Callback);
ros::Subscriber<std_msgs::Float64> joint4("/simple_robotic_model/joint5_position_controller/command", &servo4Callback);
ros::Subscriber<std_msgs::Float64> joint5("/simple_robotic_model/joint6_position_controller/command", &servo5Callback);



void setup()
{
	servo1.attach(2);
	servo2.attach(3);
	servo3.attach(4);
	servo4.attach(5);
	servo5.attach(6);

	speed_servo1.write(90);
	speed_servo2.write(90);
	speed_servo3.write(90);
	speed_servo4.write(90);
	speed_servo5.write(90);

  	nh.initNode();

	nh.subscribe(joint1);
	nh.subscribe(joint2);
	nh.subscribe(joint3);
	nh.subscribe(joint4);
	nh.subscribe(joint5);
}


void loop()
{
	speed_servo1.increment();
	speed_servo2.increment();
	speed_servo3.increment();
	speed_servo4.increment();
	speed_servo5.increment();
	



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

  	nh.spinOnce();
}
