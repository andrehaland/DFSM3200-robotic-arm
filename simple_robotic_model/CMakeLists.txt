cmake_minimum_required(VERSION 2.8.3)
project(simple_robotic_model)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
	std_msgs
	gazebo_msgs
	gazebo_plugins
	gazebo_ros
	gazebo_ros_control
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_robotic_model
#  CATKIN_DEPENDS actionlib actionlib_msgs roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

