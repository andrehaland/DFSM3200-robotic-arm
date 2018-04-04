# DFSM3200 Robotic Arm
This is the repository for group 1 - Robotic Arm. 

## Dependencies
The dependencies for the software in this repository is listed in the table below.
![alt text](https://github.com/andrehaland/DFSM3200-robotic-arm/blob/develop/Hand-ins/Diagrams/Dependencies.PNG)

## Building
To build the packages in this repository, a catkin workspace is required. See http://wiki.ros.org/catkin/workspaces for information on catkin workspaces.

When in the catkin workspace, the command *catkin_make* will build all the packages of the workspace. If the user wants to upload the compiled Arduino code, the command *catkin_make robotic_arm_firmware_robotic-upload* must be run in the workspace

## Directories
The following sections will describe each directory of this repo, and what they contain.

### Hand-ins
This directory contains the mandotory hand-ins during the course.

### camera
This is a ROS package and containts the camera node. This node spots a red marker in the frame and publishes it on a topic.

### controller
This is a ROS package and contains the controller node. This node is the center of all actions. It recieves a Vector3 from the camera mockup node, and furhter sends these coordinates to a MATLAB script. The response from MATLAB is five joint angles. These angles are further published for the gazebo model and the physical arm to utilize the angles

### matlab
This directory contains a MATLAB script used to find the inverse kinematic of a given URDF file. The script subscribes to cartesian coordinates, and uses this data to publish joint angles after it has found the inverse kinematics

### robotic_arm
This is a ROS package used to control the physical robot. It depends on the rosserial_arduino package.

### simple_robotic_model
This directory contains the robot model written in xacro. It also contains launch files to start gazebo and load the model
