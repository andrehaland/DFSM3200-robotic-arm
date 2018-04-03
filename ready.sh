#!/bin/bash

rostopic pub -1 /camera_ready std_msgs/Bool -- 1
