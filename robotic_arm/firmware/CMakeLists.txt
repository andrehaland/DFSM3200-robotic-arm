cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})
include (/opt/ros/kinetic/share/rosserial_arduino/arduino-cmake/cmake/ArduinoToolchain.cmake)
include (/opt/ros/kinetic/share/rosserial_arduino/arduino-cmake/cmake/Platform/Arduino.cmake)

generate_arduino_firmware(robotic
		SRCS main.cpp ${ROS_LIB_DIR}/time.cpp
		BOARD uno
  		PORT /dev/ttyUSB0
)
