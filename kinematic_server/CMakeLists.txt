cmake_minimum_required(VERSION 2.8.3)
project(kinematic_server)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ik_srvs
  roscpp
  servo_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kinematic_server
#  CATKIN_DEPENDS geometry_msgs inverse_kinematic_srvs roscpp servo_msgs
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(kinematic_server src/kinematic_server.cpp)
target_link_libraries(kinematic_server ${catkin_LIBRARIES})
