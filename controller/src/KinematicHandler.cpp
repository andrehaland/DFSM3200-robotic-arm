#include "KinematicHandler.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <math.h>

// Since the URDF model has limits of -PI/2 to PI/2, these macros are used to define
// the degrees of the servo motors. For instance: 60 degrees -> 60-90 = -30 = -0.52 radians 
#define DEGREES_60 -0.523598776f
#define DEGREES_50 -0.698131701f
#define DEGREES_30 -1.0471975f
#define DEGREES_40  -0.872664626f
#define DEGREES_110 0.349065865f
#define DEGREES_125 0.610865238f
#define DEGREES_135  0.785398163f
#define DEGREES_180 M_PI / 2.0
#define DEGREES_0 (M_PI / 2.0) * -1.0f
#define DEGREES_90 0.0f


KinematicHandler::KinematicHandler(ros::NodeHandle* nh) 
 :from_camera(0), from_matlab(0),
 cartesion_recieved(false), camera_ready(false)
{
	node_ptr = nh;
    
	// Advertise joint topics
	joint_publisher2 = node_ptr->advertise<std_msgs::Float64>("/5dof_robot/joint2/command", 100);
	joint_publisher3 = node_ptr->advertise<std_msgs::Float64>("/5dof_robot/joint3/command", 100);
	joint_publisher4 = node_ptr->advertise<std_msgs::Float64>("/5dof_robot/joint4/command", 100);
	joint_publisher5 = node_ptr->advertise<std_msgs::Float64>("/5dof_robot/joint5/command", 100);
	joint_publisher6 = node_ptr->advertise<std_msgs::Float64>("/5dof_robot/joint6/command", 100);

	// Advertise cartesian topic
	cartesian_publisher = node_ptr->advertise<geometry_msgs::Vector3>("/MATLAB/cartesian_subscriber", 100);
}

void KinematicHandler::kinematicCallback(const std_msgs::Float64MultiArray& joints)
{
	// To prevent undesirable callbacks
	if(!cartesion_recieved)
		return;

	// Calculate the time MATLAB used 
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
	if(!camera_ready)
		return;

	from_camera = ros::Time::now();
	ROS_INFO("Controller recieved cartesian point from camera --> Sending them to MATLAB!");
	
	// Forward the cartesian coordinates to MATLAB
	cartesian_publisher.publish(cartesian);
	cartesion_recieved = true;
	camera_ready = false;
}

void KinematicHandler::setStartPosition()
{	
	ROS_INFO("Setting arm to start position!");

	Angles newAngles;
	std_msgs::Float64 joint;
	
	// If the second motor is below 60 degrees
    if(angles.second <= DEGREES_60)
    {
		// Set fourth joint opposite direction to shorten the length of arm
		newAngles.fourth = DEGREES_180;
		joint.data = newAngles.fourth;
		joint_publisher5.publish(joint);
		ros::Duration(1.3).sleep();

		// Set third joint oposite direction to shorten the length of arm
		newAngles.third = DEGREES_180;
		joint.data = newAngles.third;
		joint_publisher4.publish(joint);
		ros::Duration(1.3).sleep();

		// Set fourth joint to start position
		newAngles.fourth = DEGREES_90;
		joint.data = newAngles.fourth;
		joint_publisher5.publish(joint);
		ros::Duration(0.5).sleep();

		// Set second joint to start position
		newAngles.second = DEGREES_90;
		joint.data = newAngles.second;
		joint_publisher3.publish(joint);

		// Set third joint to start position
		newAngles.third = DEGREES_90;
		joint.data = newAngles.third;
		joint_publisher4.publish(joint);
    }
    else if(angles.second >= DEGREES_110)
    {
		// Set fourth joint oposite direction to shorten the length of arm
		newAngles.fourth = DEGREES_0;
		joint.data = newAngles.fourth;
		joint_publisher5.publish(joint);
		ros::Duration(1.3).sleep();

		// Set third joint oposite direction to shorten the length of arm
		newAngles.third = DEGREES_0;
		joint.data = newAngles.third;
		joint_publisher4.publish(joint);
		ros::Duration(1.3).sleep();

		// Set fourth joint to start position
		newAngles.fourth = DEGREES_90;
		joint.data = newAngles.fourth;
		joint_publisher5.publish(joint);
		ros::Duration(0.5).sleep();

		// Set second joint to start position
		newAngles.second = DEGREES_90;
		joint.data = newAngles.second;
		joint_publisher3.publish(joint);

		// Set third joint to start position
		newAngles.third = DEGREES_90;
		joint.data = newAngles.third;
		joint_publisher4.publish(joint);
    }
	else
	{
		// If the second joint is between 60 and 110 degrees, all joints all set
		// directly to star position
		joint.data = DEGREES_90;
		joint_publisher3.publish(joint);
		joint_publisher4.publish(joint);
		joint_publisher5.publish(joint);

	}

	// Set first and last joint
	newAngles.fifth = DEGREES_90;
	joint.data = newAngles.fifth;
	joint_publisher6.publish(joint);
	newAngles.first = DEGREES_90;
	joint.data = newAngles.first;
	joint_publisher2.publish(joint);

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

void KinematicHandler::cameraReadyCallback(const std_msgs::Bool& ready)
{
	ROS_INFO("Camera ready to read coordinates!");
	camera_ready = ready.data;
}
