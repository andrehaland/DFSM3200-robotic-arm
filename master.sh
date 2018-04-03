#!/bin/bash

###############################################################
# This script is sourced on the Samsung to set the IP for the #
# ROS master. It must be sourced in every terminal before     #
# running any ROS target                                      #
###############################################################


# Get IP address of host
IP=$(hostname -I)

# Remove trailing whitespace
IP=${IP::-1}

echo IP: $IP

# Set this computer as ROS master
export ROS_MASTER_URI=http://$IP:11311
export ROS_IP=$IP

echo ROS MASTER URI has been set to http://$IP:11311

echo

env | grep ROS
