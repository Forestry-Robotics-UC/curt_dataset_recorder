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

#Launch scovox driver
#ros2 launch scovox_mapping lidar_mapping.launch.py params_file:="/root/ros2_ws/src/scovox/config/scovox_lidar_raw_deskew.yaml" pointcloud_topic:=/ouster/points use_sim_time:=false &
#ros2 launch scovox_mapping scovox_multi_robot.launch.py robot_name:=curt use_sim_time:=false

#ros2 launch scovox_mapping dscovox_single_robot.launch.py \
#    robot:=curt cloud_topic:=/ouster/points imu_topic:=/curt/imu/data base_frame:=os_lidar integration_frame:=map_curt map_frame:=map_curt use_sim_time:=false

#ros2 launch scovox_mapping dscovox_multi_robot.launch.py

ros2 launch scovox_mapping dscovox_multi_robot.launch.py robot_name:=curt use_sim_time:=false


# ros2 launch explo_planner exploration_experiment.launch.py max_steps:=200 output_csv:=/root/shared_folder/exploration_eig.csv use_sim_time:=false 