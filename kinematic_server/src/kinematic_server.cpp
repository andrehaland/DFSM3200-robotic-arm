#include <ros/ros.h>
#include <ik_srvs/CartesianToJoint.h>

#include <math.h>

float radToDeg(float rad)
{
    return (rad * 180.0f) / M_PI;
}

float getPositive(const float& first, const float& second)
{
    return first > second ? first : second;
}

bool inverseKinematic(ik_srvs::CartesianToJoint::Request &req,
                      ik_srvs::CartesianToJoint::Response &res)
{
    ROS_INFO("Request: X:%f, Y:%f, Z:%f\n", req.cartesian.x, req.cartesian.y, req.cartesian.z);

    float numerator;

    float p_x = 0.5f;

    float d_two_two = 0.5f;
    float p_x_two = 0.5f;
    float p_y_two = 0.5f;

    float d_two = 0.5f;
    float p_y = 0.01f;

    float numerator_plus = p_x + sqrt(-(d_two_two) + p_x_two + p_y_two);
    float numerator_minus = p_x - sqrt(-(d_two_two) + p_x_two + p_y_two);
    float denumerator = d_two - p_y;

    float theta_one_plus = 2 * atan(numerator_plus / denumerator);
    float theta_one_minus = 2 * atan(numerator_minus / denumerator);

    float theta_one = getPositive(theta_one_plus, theta_one_minus);

    //////////////////////////////////////////////////////

    float a_z = 0.3f;
    float alpha_z = 0.5f;
    float a_x_two = 0.4f;
    float cos_theta_one_two = cos(theta_one);
    float a_x = 0.5f;
    float a_y = 0.3f;
    float cos_theta_one = cos(theta_one);

    float a_y_two = 0.3f;
    float sin_theta_one_two = sin(theta_one);

    float a_z_two = 0.3f;

    float sin_theta_one = sin(theta_one);

    numerator_plus = a_z + sqrt(a_x_two*cos_theta_one_two + 2*a_x*a_y*cos_theta_one*sin_theta_one_two + a_y_two*sin_theta_one_two + a_z_two);
    numerator_minus = a_z - sqrt(a_x_two*cos_theta_one_two + 2*a_x*a_y*cos_theta_one*sin_theta_one_two + a_y_two*sin_theta_one_two + a_z_two);
    denumerator = a_x*cos_theta_one + a_y * sin_theta_one;

    float theta_two_plus = 2*atan(numerator_plus / denumerator);
    float theta_two_minus = 2*atan(numerator_minus / denumerator);

    float theta_two = getPositive(theta_two_minus, theta_two_plus);

    ////////////////////////////////////////////////////////

    float sin_two_theta_one = sin(theta_one);



    ROS_INFO("theta_one_plus = %f", radToDeg(theta_one_plus));
    ROS_INFO("theta_one_minus = %f", radToDeg(theta_one_minus));
    ROS_INFO("theta_two_plus = %f", radToDeg(theta_two_plus));
    ROS_INFO("theta_two_minus = %f", radToDeg(theta_two_minus));


    res.joints.first_joint = static_cast<int>(radToDeg(theta_one));
    res.joints.second_joint = static_cast<int>(radToDeg(theta_two));
    res.joints.third_joint = 7;
    res.joints.fourth_joint = 8;
    res.joints.fifth_joint = 9;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinematic_server");
    ros::NodeHandle node_handle;

    ros::ServiceServer service = node_handle.advertiseService("inverse_kinematic", inverseKinematic);
    ROS_INFO("Inverse kinematic server ready!");
    ros::spin();

    return 0;
}
