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

#Run Imu fusing pkg to run Madwick filter
ros2 launch imu_mag_fusion imu_mag_fusion.launch.py &

#Run rm3100 driver
ros2 launch rm3100_driver rm3100_driver.launch.py

