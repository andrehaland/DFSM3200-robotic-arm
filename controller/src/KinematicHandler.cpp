#include "KinematicHandler.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

KinematicHandler::KinematicHandler(ros::NodeHandle* nh)
{

  node_ptr = nh;
    
  client = node_ptr->serviceClient<ik_srvs::CartesianToJoint>("inverse_kinematic");
	joint_publisher = node_ptr->advertise<servo_msgs::JointAngles>("joint_angles", 100);
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
	ROS_INFO("Joint values:");
	ROS_INFO("1: [%f], 2: [%f], 3: [%f], 4: [%f], 5: [%f]", joints.data[0], joints.data[1], joints.data[2], joints.data[3], joints.data[4]);
	ROS_INFO("Publishing to joints!");

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
	ROS_INFO("Controller recieved message from camera");
	
	cartesian_publisher.publish(cartesian);
	/**
	srv.request.cartesian.x = cartesian.x;
	srv.request.cartesian.y = cartesian.y;
	srv.request.cartesian.z = cartesian.z;

	if(client.call(srv))
	{
		ROS_INFO("Controller successfully called kinematic server!");
		joints.first_joint = srv.response.joints.first_joint;
		joints.second_joint = srv.response.joints.second_joint;
		joints.third_joint = srv.response.joints.third_joint;
		joints.fourth_joint = srv.response.joints.fourth_joint;
		joints.fifth_joint = srv.response.joints.fifth_joint;

        ROS_INFO("j1 = %f, j2 = %f, j3 = %f, j4, = %f, j5 = %f", joints.first_joint, joints.second_joint, joints.third_joint, joints.fourth_joint, joints.fifth_joint);
		joint_publisher.publish(joints);

		std_msgs::Float64 joint;

		joint.data = joints.first_joint;
		joint_publisher2.publish(joint);

		joint.data = joints.second_joint;
		joint_publisher3.publish(joint);

		joint.data = joints.third_joint;
		joint_publisher4.publish(joint);

		joint.data = joints.fourth_joint;
		joint_publisher5.publish(joint);

		joint.data = joints.fifth_joint;
		joint_publisher6.publish(joint);
	}
	else
	{
		ROS_ERROR("Failed to call kinematic server!");
	}
	**/
}
