cmake_minimum_required(VERSION 2.8.3)
project(joint_publisher)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  servo_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joint_publisher
#  CATKIN_DEPENDS roscpp servo_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joint_publisher src/main.cpp)
target_link_libraries(joint_publisher ${catkin_LIBRARIES})
