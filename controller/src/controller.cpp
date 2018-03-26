#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"

#include "KinematicHandler.h"
/**

{
	ROS_INFO("Hello World [%f]", msg->data[1]);
}
**/
int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller");	

	ros::NodeHandle node_handle;

	// Object responsible for camera callback and sending request to server
	KinematicHandler kinematic(&node_handle);


	// Setup subscriber
	ros::Subscriber camera = node_handle.subscribe("camera", 100, &KinematicHandler::cameraCallback, &kinematic);
	//ros::Subscriber kinematic_sub = node_handle.subscribe("/MATLAB/kinematic_publisher", 100, matlabCallback);

	ros::Subscriber kinematics = node_handle.subscribe("/MATLAB/kinematic_publisher", 100, &KinematicHandler::kinematicCallback, &kinematic);

	while(ros::ok())
	{
		ros::spinOnce();

	}
}
