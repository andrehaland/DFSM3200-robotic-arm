#include "KinematicHandler.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>

KinematicHandler::KinematicHandler(ros::NodeHandle* nh) :from_camera(0), from_matlab(0)
{
	node_ptr = nh;
    
  	client = node_ptr->serviceClient<ik_srvs::CartesianToJoint>("inverse_kinematic");
	//joint_publisher = node_ptr->advertise<servo_msgs::JointAngles>("joint_angles", 100);
	joint_publisher2 = node_ptr->advertise<std_msgs::Float64>("/simple_robotic_model/joint2_position_controller/command", 100);
	joint_publisher3 = node_ptr->advertise<std_msgs::Float64>("/simple_robotic_model/joint3_position_controller/command", 100);
	joint_publisher4 = node_ptr->advertise<std_msgs::Float64>("/simple_robotic_model/joint4_position_controller/command", 100);
	joint_publisher5 = node_ptr->advertise<std_msgs::Float64>("/simple_robotic_model/joint5_position_controller/command", 100);
	joint_publisher6 = node_ptr->advertise<std_msgs::Float64>("/simple_robotic_model/joint6_position_controller/command", 100);

	// Erlend edit
	
	cartesian_publisher = node_ptr->advertise<geometry_msgs::Vector3>("/MATLAB/cartesian_subscriber", 100);

}
	// Erlend edit end

void KinematicHandler::kinematicCallback(const std_msgs::Float64MultiArray& joints)
{
	from_matlab = ros::Time::now();
	ros::Duration matlab_time = from_matlab- from_camera;

	ROS_INFO("Controller recieved joint values [%f, %f, %f, %f, %f] from MATLAB --> Publishing to joints!\n", joints.data[0],joints.data[1],joints.data[2],joints.data[3],joints.data[4]);

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

}

void KinematicHandler::cameraCallback(const geometry_msgs::Vector3& cartesian)
{
	from_camera = ros::Time::now();
	ROS_INFO("Controller recieved cartesian point from camera --> Sending them to MATLAB!");
	
	// Forward the cartesian coordinates to MATLAB
	cartesian_publisher.publish(cartesian);
}
