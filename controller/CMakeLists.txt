cmake_minimum_required(VERSION 2.8.3)
project(controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ik_srvs
  roscpp
  servo_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES controller
#  CATKIN_DEPENDS geometry_msgs inverse_kinematic_srvs roscpp servo_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(controller src/controller.cpp src/KinematicHandler.h src/KinematicHandler.cpp src/Angles.h)
target_link_libraries(controller ${catkin_LIBRARIES})
