#!/bin/bash
set -e

#ROS 2 Middleware Implementation 

#Fastrtps
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

#Cyclonedds
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS2 Humble
source /opt/ros/kilted/setup.bash

#Build workspace only with the packages descriminated on docker compose file
cd /root/ros2_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS2 Workspace
source /root/ros2_ws/install/setup.bash

ros2 run nav2_map_server map_server \
  --ros-args \
  -p yaml_filename:=/root/shared_folder/kalhan_occupation_map.yaml \
  -p frame_id:=map_curt \
  -p use_lifecycle:=true &

#Run nav2 ROS" framework
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=/root/nav2/nav2_params.yaml