cmake_minimum_required(VERSION 2.8.3)
project(ik_srvs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  servo_msgs
  geometry_msgs
  std_msgs
  message_generation
)



add_service_files(
   FILES
   CartesianToJoint.srv
)

generate_messages(
   DEPENDENCIES
   servo_msgs std_msgs geometry_msgs
 )

catkin_package(
  CATKIN_DEPENDS roscpp servo_msgs std_msgs geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
