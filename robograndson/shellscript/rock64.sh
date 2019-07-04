#!/bin/bash
source /home/sayter/ros_ws/devel/setup.bash
export ROS_IP=$(ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')
export ROSLAUNCH_SSH_UNKNOWN=1 
roslaunch "$1"
