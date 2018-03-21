#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include "servo_msgs/JointAngles.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_publisher");

    ros::NodeHandle node_handle;

    ros::Publisher joint_pub = node_handle.advertise<servo_msgs::JointAngles>("joints", 1000);

    ros::Rate loop_rate(1);

    servo_msgs::JointAngles joints;

    while(ros::ok())
    {
        joints.first_joint = 1;
        joints.second_joint = 2;
        joints.third_joint = 3;
        joints.fourth_joint = 4;
        joints.fifth_joint = 5;

        joint_pub.publish(joints);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}