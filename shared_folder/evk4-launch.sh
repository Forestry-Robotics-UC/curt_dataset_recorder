#!/bin/bash
set -e

#ROS 2 Middleware Implementation 

#Cyclonedds
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

#Build workspace only with the packages discriminated on docker compose file
cd /root/ros2_ws/
colcon build --symlink-install --cmake-args ---symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo #-DCMAKE_BUILD_TYPE=Release

# Source ROS2 Workspace
source /root/ros2_ws/install/setup.bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
export HDF5_PLUGIN_PATH=$HDF5_PLUGIN_PATH:/usr/local/lib/hdf5/plugin  # On Ubuntu 24.04

#Launch ouster driver
ros2 launch metavision_driver driver_composition.launch.py serial:="00051972"
