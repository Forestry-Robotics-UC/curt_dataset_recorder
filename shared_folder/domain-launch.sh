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

#Build workspace only with the packages descriminated on docker compose file
cd /root/ros2_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS2 Workspace
source /root/ros2_ws/install/setup.bash

#sleep 9000000

#Launch domain_bridge
#ros2 run domain_bridge domain_bridge /root/ros2_ws/src/domain_bridge/examples/bridge_config.yaml
ros2 launch domain_bridge domain_bridge.launch.xml config:=/root/ros2_ws/src/domain_bridge/examples/bridge_config.yaml from_domain:=37 to_domain:=30
