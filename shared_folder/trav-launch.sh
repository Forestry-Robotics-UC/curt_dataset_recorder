#!/bin/bash
set -e

#ROS 2 Middleware Implementation 

#Fastrtps
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

#Cyclonedds
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS2 Humble
source /opt/ros/jazzy/setup.bash

#Build workspace only with the packages descriminated on docker compose file
cd /root/ros2_ws/
touch $ROS2_WS/src/traversability_mapping/ThirdParty/Sophus/COLCON_IGNORE
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS2 Workspace
source /root/ros2_ws/install/setup.bash

#Run Hesai driver
ros2 launch traversability_mapping_ros global_gt_traversability_mapping.launch.py ros_params_file:=/root/ros2_ws/src/traversability_mapping/traversability_ros_interface/traversability_mapping_ros/params/traversability_gt_ros_params.yaml
