#include <ros/ros.h>
#include <ik_srvs/CartesianToJoint.h>

#include <math.h>

const float PI = 3.14159265359f;

float radToDeg(float rad)
{
    return (rad * 180.0f) / PI;
}

float degToRad(float deg)
{
    return (deg * PI) / 180.0f;
}


float getPositive(const float& first, const float& second)
{
    return first > second ? first : second;
}


float determineAngle(float first_degrees, float second_degrees)
{
    // If angles are negative, add one revolution
    if(first_degrees < 0.0f)
    {
        first_degrees += 360.0f;
    }
    if(second_degrees < 0.0f)
    {
        second_degrees += 360.0f;
    }

    ROS_INFO("First before std::trunc: %f", first_degrees);
    ROS_INFO("Second before std::trunc: %f", second_degrees);

    float trunced_first = std::trunc(1000 * first_degrees) / 1000;
    float trunced_second = std::trunc(1000 * second_degrees) / 1000;

    // Determine which angle is between 0 and 180 (0 and PI)
    if(trunced_first >= 0.0f && trunced_first <= 180.0f)
    {
        // Return radians for further use in inverse kinematic operations
        ROS_INFO("Returning first: %f\n", first_degrees);
        return degToRad(first_degrees);
    }
    else
    {
        // Return radians for further use in inverse kinematic operations
        ROS_INFO("Returning second: %f\n", second_degrees);
        return degToRad(second_degrees);
    }
}

/**
 * Arctan(x) defined for all x
 * Arcsin(x) defined for x in [1,1]
 **/
bool inverseKinematic(ik_srvs::CartesianToJoint::Request &req,
                      ik_srvs::CartesianToJoint::Response &res)
{
    ROS_INFO("Request: X:%f, Y:%f, Z:%f\n", req.cartesian.x, req.cartesian.y, req.cartesian.z);

    float numerator_plus, numerator_minus, denumerator = 0.0f;

    float x = req.cartesian.x;
    float y = req.cartesian.y;
    float z = req.cartesian.z;

    ////////////////////Theta1///////////////////////////////////

    float d_two = 0.5f;

    numerator_plus = x + sqrt(-(pow(d_two,2)) + pow(x,2) + pow(y,2));
    numerator_minus = x - sqrt(-(pow(d_two,2)) + pow(x,2) + pow(y,2));
    denumerator = d_two - y;

    float theta_one_plus = 2.0f * atan(numerator_plus / denumerator);
    float theta_one_minus = 2.0f * atan(numerator_minus / denumerator);

    float theta_one = determineAngle(radToDeg(theta_one_plus), radToDeg(theta_one_minus));

    /////////////////////Theta2/////////////////////////////////

    float a_z = 0.3f;
    float a_x = 0.4f;
    float a_y = 0.5f;

    float cos_theta_one = cos(theta_one);
    float sin_theta_one = sin(theta_one);

    numerator_plus = a_z + sqrt( pow(a_x, 2) * pow(cos_theta_one, 2) + 2 * a_x * a_y * cos_theta_one * sin_theta_one + pow(a_y, 2) * pow(sin_theta_one, 2) + pow(a_z, 2));
    numerator_minus = a_z - sqrt( pow(a_x, 2) * pow(cos_theta_one, 2) + 2 * a_x * a_y * cos_theta_one * sin_theta_one + pow(a_y, 2) * pow(sin_theta_one, 2) + pow(a_z, 2));
    denumerator = a_x * cos_theta_one + a_y * sin_theta_one;

    float theta_two_plus = 2.0f * atan(numerator_plus / denumerator);
    float theta_two_minus = 2.0f * atan(numerator_minus / denumerator);

    float theta_two = determineAngle(radToDeg(theta_two_minus), radToDeg(theta_two_plus));

    /////////////////////Theta3///////////////////////////////////

    float sin_two_theta_one = sin(2 * theta_one);
    float cos_theta_two = cos(theta_two);
    float sin_theta_two = sin(theta_two);

    numerator_minus = (sqrt(pow(a_x, 2) * pow(cos_theta_one, 2) + (sin_two_theta_one * a_x * a_y) - pow(a_y, 2) * pow(cos_theta_one, 2) + pow(a_y, 2) + pow(a_z, 2)) - (a_z * cos_theta_two) + (a_x*cos_theta_one * sin_theta_two) + (a_y*sin_theta_one * sin_theta_two));
    numerator_plus = (sqrt(pow(a_x, 2) * pow(cos_theta_one, 2) + (sin_two_theta_one * a_x * a_y) - pow(a_y, 2) * pow(cos_theta_one, 2) + pow(a_y, 2) + pow(a_z, 2)) + (a_z * cos_theta_two) - (a_x*cos_theta_one * sin_theta_two) - (a_y*sin_theta_one * sin_theta_two));

    denumerator = a_z * sin_theta_two + a_x * cos_theta_one*cos_theta_two + a_y * cos_theta_two * sin_theta_one;

    float theta_three_minus = -2.0f * atan(numerator_minus / denumerator);
    float theta_three_plus = 2.0f * atan(numerator_plus / denumerator);
    

    float theta_three = determineAngle(radToDeg(theta_three_minus), radToDeg(theta_three_plus));
    /////////////////////Theta4///////////////////////////////////////////

    float d_one = 0.2f;

    float n_z = 2.0f;

    float a_three = 0.3f;
    float a_four = 0.4f;
    float a_five = 0.5f;
    float a_six = 0.6f;

    float sin_theta_two_plus_three = sin(theta_two + theta_three);

    float numerator = d_one - z + a_six * n_z + a_four * sin_theta_two_plus_three + a_three * sin_theta_two;

    float theta_four_first = -1*theta_two - theta_three - asin(numerator / a_five);
    float theta_four_second = PI - theta_two - theta_three - asin(numerator / a_five);

    float theta_four = determineAngle(radToDeg(theta_four_first), radToDeg(theta_four_second));

    //////////////////////////Theta5//////////////////////////////////

    float n_x = 0.1f;
    float n_y = 0.1f;

    float inner_parenthesis = n_x * cos(theta_one) * sin(theta_two) - n_z*cos(theta_two) + n_y*sin(theta_one)*sin(theta_two);

    ROS_INFO("arcsin(%f)", inner_parenthesis);

    float theta_five_first = -1*theta_three - theta_four - asin(inner_parenthesis);
    float theta_five_second = PI - theta_four - theta_three - asin(inner_parenthesis);

    float theta_five = determineAngle(radToDeg(theta_five_first), radToDeg(theta_five_second));

    //////////////////////////////////////////////////////////////////////

    ROS_INFO("Theta_one: %f", theta_one);
    ROS_INFO("Theta_two: %f", theta_two);
    ROS_INFO("Theta_three: %f", theta_three);
    ROS_INFO("Theta_four: %f", theta_four);
    ROS_INFO("Theta_five: %f\n", theta_five);


    res.joints.first_joint = theta_one;
    res.joints.second_joint = theta_two;
    res.joints.third_joint = theta_three;
    res.joints.fourth_joint = theta_four;
    res.joints.fifth_joint = theta_five;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinematic_server");
    ros::NodeHandle node_handle;

    ros::ServiceServer service = node_handle.advertiseService("inverse_kinematic", inverseKinematic);
    ROS_INFO("Inverse kinematic server ready!\n");
    ros::spin();

    return 0;
}
