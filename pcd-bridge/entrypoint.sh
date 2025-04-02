#!/usr/bin/bash

source /opt/ros/noetic/setup.bash && catkin_make && source /workspace/devel/local_setup.bash && rosrun pcd_bridge $1