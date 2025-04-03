#!/usr/bin/bash

source /opt/ros/humble/setup.bash && cd /workspace && colcon build
source /opt/ros/humble/setup.bash && source /workspace/install/local_setup.bash && ros2 run pcd_bridge_receiver $1