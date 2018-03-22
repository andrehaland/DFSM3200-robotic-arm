#ifndef KINEMATIC_HANDLER_H
#define KINEMATIC_HANDLER_H

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <servo_msgs/JointAngles.h>
#include "ik_srvs/CartesianToJoint.h"


class KinematicHandler
{
public:
    KinematicHandler(ros::NodeHandle* nh);

    ~KinematicHandler(){}

    void cameraCallback(const geometry_msgs::Vector3& cartesian);



private:

    ros::NodeHandle* node_ptr;

    ros::ServiceClient client;

    ros::Publisher joint_publisher;

    ik_srvs::CartesianToJoint srv;

    servo_msgs::JointAngles joints;


};
#endif
