#ifndef KINEMATIC_HANDLER_H
#define KINEMATIC_HANDLER_H

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64MultiArray.h>

#include "Angles.h"


class KinematicHandler
{
public:
    KinematicHandler(ros::NodeHandle* nh);

    ~KinematicHandler(){}

    void cameraCallback(const geometry_msgs::Vector3& cartesian);

		void kinematicCallback(const std_msgs::Float64MultiArray& joints);

    void startPosCallback(const std_msgs::Int16& start);

    void setStartPosition();

private:
    bool cartesion_recieved;

    ros::Time from_camera;
    ros::Time from_matlab;

    ros::NodeHandle* node_ptr;


		ros::Publisher cartesian_publisher;
    ros::Publisher joint_publisher;

		ros::Publisher joint_publisher2;
		ros::Publisher joint_publisher3;
		ros::Publisher joint_publisher4;
		ros::Publisher joint_publisher5;
		ros::Publisher joint_publisher6;

    // Holds the last published angles
    Angles angles;

};
#endif
