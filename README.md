# DFSM3200 Robotic Arm
This is the repository for group 1 - Robotic Arm. The following sections will describe each directory of this repo, and what they contain.

## Hand-ins
This directory contains some of the mandotory hand-ins during the course.

## Controller
This is a ROS package and contains the controller node. This node is the center of all actions. It recieves a Vector3 from the Camera mockup node, and furhters sends a request to the Kinematic server. The response from the server is a servo_msgs/JointAngles message. This message is further sent to two topics, one for the physical robot, and one for the simulated robot

## Ik_srvs
This is a ROS package containg a service message. This service has a geometry_msgs/Vector3 message as request, and returns a servo_msgs/JointAngles as response.

## Kinematic_server
This is a ROS package containg a server node. The server uses the ik_srvs/CartesianToJoint service and does the inverse kinematic to turn a cartersion point in space into joint angles for the robot.

## Robotic_arm
This is a ROS package used to control the physical robot. It depends on the rosserial_arduino package.

## Servo_msgs
This is a ROS package containg the servo_msgs/JointAngles message. This message holds five joint angles
