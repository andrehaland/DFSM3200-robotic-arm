cmake_minimum_required(VERSION 2.8.3)
project(servo_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
   FILES
   JointAngles.msg
)


generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
