#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"

#include "KinematicHandler.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller");	

	ros::NodeHandle node_handle;

	// Object responsible for camera callback and sending request to server
	KinematicHandler kinematic(&node_handle);


	// Setup subscriber to camera to get cartesian coordinates
	ros::Subscriber camera = node_handle.subscribe("camera", 100, &KinematicHandler::cameraCallback, &kinematic);

	// Setup subscriber to get the computed angles by MATLAB
	ros::Subscriber kinematics = node_handle.subscribe("/MATLAB/kinematic_publisher", 100, &KinematicHandler::kinematicCallback, &kinematic);

	while(ros::ok())
	{
		ros::spinOnce();
	}
}
