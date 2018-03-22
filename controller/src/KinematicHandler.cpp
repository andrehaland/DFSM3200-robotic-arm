#include "KinematicHandler.h"


KinematicHandler::KinematicHandler(ros::NodeHandle* nh)
{
    node_ptr = nh;
    
    client = node_ptr->serviceClient<ik_srvs::CartesianToJoint>("inverse_kinematic");
	joint_publisher = node_ptr->advertise<servo_msgs::JointAngles>("joint_angles", 100);

}

void KinematicHandler::cameraCallback(const geometry_msgs::Vector3& cartesian)
{
	ROS_INFO("Controller recieved message from camera");

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
	}
	else
	{
		ROS_ERROR("Failed to call kinematic server!");
	}
}