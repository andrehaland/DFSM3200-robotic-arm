#ifndef KINEMATIC_HANDLER_H
#define KINEMATIC_HANDLER_H

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include "Angles.h"


class KinematicHandler
{
public:
    /**
     * The constructor takes a pointer to the NodeHandle. It uses this pointer
     * to advertise different topics 
     **/
    KinematicHandler(ros::NodeHandle* nh);

    /**
     * Default destructor 
     **/
    ~KinematicHandler(){}

    /**
     *  This callback functions listens to input from the camera,
     *  in form of a Vector3, it further publishes the data to
     *  MATLAB
     **/
    void cameraCallback(const geometry_msgs::Vector3& cartesian);

    /**
     * This callback function listens to the /MATLAB/kinematic_publisher topic.
     * It contains the 5 joint angles calculated by MATLAB
     **/
		void kinematicCallback(const std_msgs::Float64MultiArray& joints);

    /**
     * Algorithm used to set the robot arm back to start position
     **/
    void setStartPosition();

    /**
     * Function used to test the set-start-position algorithm
     **/
    void startPosCallback(const std_msgs::Int16& start);

    /**
     * Subscriber used to set camera_ready member variable 
     **/
    void cameraReadyCallback(const std_msgs::Bool& ready);

private:
    /**
     * To avoid bugs with the kinematic callback fucntion being called
     * without having recieved cartesian coordinates from the camera 
     **/
    bool cartesion_recieved;

    /**
     * Used to record the time this node recieves cartesian coordinates from the camera
     **/
    ros::Time from_camera;

    /**
     * Used to record the time this node recieves joint angles from MATLAB
     **/
    ros::Time from_matlab;


    /**
     * Pointer to the NodeHandle
     * Used to advertise topics 
     **/
    ros::NodeHandle* node_ptr;

    /**
     * Publisher to send cartesian coordinates to MATLAB 
     **/
	ros::Publisher cartesian_publisher;


    /**
     * Topics used to publish to each joint 
     **/
		ros::Publisher joint_publisher2;
		ros::Publisher joint_publisher3;
		ros::Publisher joint_publisher4;
		ros::Publisher joint_publisher5;
		ros::Publisher joint_publisher6;

    /**
     * Holds the last published angles.
     * Used in set-start-position algorithm
     **/
    Angles angles;

    bool camera_ready;

};
#endif
