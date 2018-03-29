#include "KinematicHandler.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <math.h>

#define DEGREES_50 -0.698131701f
#define DEGREES_30 -1.0471975f
#define DEGREES_40  -0.872664626f
#define DEGREES_125 0.610865238f
#define DEGREES_135  0.785398163f
#define DEGREES_180 M_PI / 2.0
#define DEGREES_0 (M_PI / 2.0) * -1.0f
#define DEGREES_90 0.0f


KinematicHandler::KinematicHandler(ros::NodeHandle* nh) :from_camera(0), from_matlab(0), cartesion_recieved(false)
{
	node_ptr = nh;
    
	joint_publisher2 = node_ptr->advertise<std_msgs::Float64>("/5dof_robot/joint2/command", 100);
	joint_publisher3 = node_ptr->advertise<std_msgs::Float64>("/5dof_robot/joint3/command", 100);
	joint_publisher4 = node_ptr->advertise<std_msgs::Float64>("/5dof_robot/joint4/command", 100);
	joint_publisher5 = node_ptr->advertise<std_msgs::Float64>("/5dof_robot/joint5/command", 100);
	joint_publisher6 = node_ptr->advertise<std_msgs::Float64>("/5dof_robot/joint6/command", 100);

	cartesian_publisher = node_ptr->advertise<geometry_msgs::Vector3>("/MATLAB/cartesian_subscriber", 100);
}

void KinematicHandler::kinematicCallback(const std_msgs::Float64MultiArray& joints)
{
	if(!cartesion_recieved)
		return;

	from_matlab = ros::Time::now();

	double seconds = from_matlab.toSec() - from_camera.toSec();

	ROS_INFO("Controller recieved joint values from MATLAB after %f secs --> Publishing to joints!", seconds);

	// Publish to all joints --> "/5dof_robot/jointX/commad" topic
	std_msgs::Float64 joint;
	joint.data = joints.data[0];
	joint_publisher2.publish(joint);
	joint.data = joints.data[1];
	joint_publisher3.publish(joint);
	joint.data = joints.data[2];
	joint_publisher4.publish(joint);
	joint.data = joints.data[3];
	joint_publisher5.publish(joint);
	joint.data = joints.data[4];
	joint_publisher6.publish(joint);

	// Store last published angles
	angles.first = joints.data[0];
	angles.second = joints.data[1];
	angles.third = joints.data[2];
	angles.fourth = joints.data[3];
	angles.fifth = joints.data[4];

	// Wait 5s before setting arm to start position
	ros::Duration(5.0).sleep();

	setStartPosition();

	cartesion_recieved = false;


}

void KinematicHandler::cameraCallback(const geometry_msgs::Vector3& cartesian)
{
	//std_msgs::Header h = cartesian->header;
	from_camera = ros::Time::now();
	ROS_INFO("Controller recieved cartesian point from camera --> Sending them to MATLAB!");
	
	// Forward the cartesian coordinates to MATLAB
	cartesian_publisher.publish(cartesian);
	cartesion_recieved = true;
}

void KinematicHandler::setStartPosition()
{	
	ROS_INFO("Setting arm to start position!");

	Angles newAngles;
	std_msgs::Float64 joint;
	
	// If the second motor is below 50 degrees
    if(angles.second <= DEGREES_50)
    {
		// Set fourth joint oposite directen to shorten the length of arm
		newAngles.fourth = DEGREES_180;
		joint.data = newAngles.fourth;
		joint_publisher5.publish(joint);
		ros::Duration(1.0).sleep();

		newAngles.third = DEGREES_180;
		joint.data = newAngles.third;
		joint_publisher4.publish(joint);
		ros::Duration(1.0).sleep();

		// Set fourth joint oposite direction to shorten the length of arm
		newAngles.fourth = DEGREES_90;
		joint.data = newAngles.fourth;
		joint_publisher5.publish(joint);
		ros::Duration(0.5).sleep();

		newAngles.second = DEGREES_90;
		joint.data = newAngles.second;
		joint_publisher3.publish(joint);

		newAngles.third = DEGREES_90;
		joint.data = newAngles.third;
		joint_publisher4.publish(joint);
    }
    else if(angles.second >= DEGREES_125)
    {
		//Set fourth joint
		newAngles.fourth = DEGREES_0;
		joint.data = newAngles.fourth;
		joint_publisher5.publish(joint);
		ros::Duration(1.0).sleep();

		newAngles.third = DEGREES_0;
		joint.data = newAngles.third;
		joint_publisher4.publish(joint);
		ros::Duration(1.0).sleep();

		newAngles.fourth = DEGREES_90;
		joint.data = newAngles.fourth;
		joint_publisher5.publish(joint);
		ros::Duration(0.5).sleep();

		newAngles.second = DEGREES_90;
		joint.data = newAngles.second;
		joint_publisher3.publish(joint);

		newAngles.third = DEGREES_90;
		joint.data = newAngles.third;
		joint_publisher4.publish(joint);
    }
	else
	{
		joint.data = DEGREES_90;
		joint_publisher3.publish(joint);
		joint_publisher4.publish(joint);
		joint_publisher5.publish(joint);

	}

	// Set last joint
	newAngles.fifth = DEGREES_90;
	joint.data = newAngles.fifth;
	joint_publisher6.publish(joint);
}


void KinematicHandler::startPosCallback(const std_msgs::Int16& start)
{
	if(start.data == 1)
		angles.second = DEGREES_135;
	else if(start.data == 0)
		angles.second = DEGREES_40;
	else
		angles.second = DEGREES_90;
	
	angles.first = DEGREES_90;
	angles.third = DEGREES_90;
	angles.fourth = DEGREES_90;
	angles.fifth = DEGREES_90;
	setStartPosition();
}
