# DFSM3200 Robotic Arm
This is the repository for group 1 - Robotic Arm. The following sections will describe each directory of this repo, and what they contain.

## Hand-ins
This directory contains the mandotory hand-ins during the course.

## controller
This is a ROS package and contains the controller node. This node is the center of all actions. It recieves a Vector3 from the camera mockup node, and furhter sends these coordinates to a MATLAB script. The response from MATLAB is five joint angles. These angles are further published for the gazebo model and the physical arm to utilize the angles

## matlab
This directory contains a MATLAB script used to find the inverse kinematic of a given URDF file. The script subscribes to cartesian coordinates, and uses this data to publish joint angles after it has found the inverse kinematics

## robotic_arm
This is a ROS package used to control the physical robot. It depends on the rosserial_arduino package.

## simple_robotic_model
This directory contains the robot model written in xacro. It also contains launch files to start gazebo and load the model
