#!/bin/bash
set -e

#ROS 2 Middleware Implementation 

#Fastrtps
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

#Cyclonedds
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

#Build workspace
cd /root/ros2_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS2 Workspace
source /root/ros2_ws/install/setup.bash

#Run Madwick filter
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args \
    -p use_mag:=true \
    -p use_magnetic_field_msg:=false \
    -p publish_tf:=false \
    -p world_frame:=enu \
    -p base_link_frame:=imu \
    -p gain:=0.1 \
    -p zeta:=0.0 \
    --remap /imu/data_raw:=/imu/data \
    --remap /imu/data:=/imu/fused \
    --remap /imu/mag:=/mag &

#Run rm3100 driver
ros2 launch rm3100_driver rm3100_driver.launch.py

