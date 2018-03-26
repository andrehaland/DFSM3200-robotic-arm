#!/bin/bash

# Get IP adress of host
IP=$(hostname -I)

#Remove trailing whitespace
IP=${IP::-1}

echo "IP: ${IP}"

# Set URI of roscore
export ROS_MASTER_URI=http://158.36.17.26:11311

# Set IP of this computer
export ROS_IP=${IP}

echo ROS MASTER URI has been set to 158.36.17.26:11311

echo

env | grep ROS

