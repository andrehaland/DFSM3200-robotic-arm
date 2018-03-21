#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>

#include "servo_msgs/JointAngles.h"

#include <Arduino.h>

#include <Servo.h>

ros::NodeHandle nh;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;


void servoCallback(const servo_msgs::JointAngles& angles)
{
	nh.loginfo("Recieved msg");
}

ros::Subscriber<servo_msgs::JointAngles> servo_subscriber("joints", &servoCallback);



void setup()
{
	/*servo1.attach(9);
	servo2.attach(10);
	servo3.attach(11);*/

  	nh.initNode();

	nh.subscribe(servo_subscriber);
}


void loop()
{
  nh.spinOnce();
}
