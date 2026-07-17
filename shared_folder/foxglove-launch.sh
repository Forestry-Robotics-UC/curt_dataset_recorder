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

#ros2 bag play /root/rosbags/2026_06_19_18_19_06__kalhan-map-test-2_ --clock &
#ros2 bag play /root/rosbags/2026_06_22_16_29_43__lab-test_ &
ros2 bag play /root/rosbags/2026_07_06_18_09_02__curt_kalhan_coop_ --clock

# Run Foxglove server
#ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=9092

#rviz2 -d /root/shared_folder/isr.rviz
