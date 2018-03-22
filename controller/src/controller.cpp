#include <ros/ros.h>

#include "KinematicHandler.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller");	

	ros::NodeHandle node_handle;

	// Object responsible for camera callback and sending request to server
	KinematicHandler kinematic(&node_handle);


	// Setup subscriber
	ros::Subscriber camera = node_handle.subscribe("camera", 100, &KinematicHandler::cameraCallback, &kinematic);


	
	while(ros::ok())
	{
		ros::spinOnce();

	}
}
