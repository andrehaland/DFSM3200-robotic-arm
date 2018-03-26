#ifndef KINEMATIC_HANDLER_H
#define KINEMATIC_HANDLER_H

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64MultiArray.h>
#include <servo_msgs/JointAngles.h>
#include "ik_srvs/CartesianToJoint.h"

class KinematicHandler
{
public:
    KinematicHandler(ros::NodeHandle* nh);

    ~KinematicHandler(){}

    void cameraCallback(const geometry_msgs::Vector3& cartesian);
		void kinematicCallback(const std_msgs::Float64MultiArray& joints);



private:

    ros::NodeHandle* node_ptr;

    ros::ServiceClient client;

		ros::Publisher cartesian_publisher;
    ros::Publisher joint_publisher;

		ros::Publisher joint_publisher2;
		ros::Publisher joint_publisher3;
		ros::Publisher joint_publisher4;
		ros::Publisher joint_publisher5;
		ros::Publisher joint_publisher6;

    ik_srvs::CartesianToJoint srv;

    servo_msgs::JointAngles joints;


};
#endif
