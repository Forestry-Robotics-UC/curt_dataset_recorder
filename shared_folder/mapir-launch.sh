#!/bin/bash
set -e

# ROS 2 Middleware Implementation

# Fast RTPS
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Build workspace only with the packages described on docker compose file
ROS2_WS="${ROS2_WS:-/workspaces/ros2_ws}"
cd "${ROS2_WS}"
colcon build --symlink-install \
  --base-paths src/mapir-camera-ros2 \
  --packages-select mapir_camera_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS2 Workspace
source "${ROS2_WS}/install/setup.bash"

# Run MAPIR camera launch
ros2 launch mapir_camera_ros2 mapir_camera.launch.py \
  namespace:=mapir \
  camera_impl:=cpp \
  enable_indices:=true \
  camera_info_url:=file:///root/sensor_configs/mapir/mapir3_ocn_camera_info.yaml

``