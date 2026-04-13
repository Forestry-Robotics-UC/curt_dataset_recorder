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
ROS2_WS="${ROS2_WS:-/root/ros2_ws}"
MAPIR_SRC_DIR="${ROS2_WS}/src/mapir-camera-ros2"
mkdir -p "${ROS2_WS}/src"
if [[ ! -d "${MAPIR_SRC_DIR}" ]]; then
  echo "MAPIR source tree not found, cloning into ${MAPIR_SRC_DIR}"
  git clone -b curt https://github.com/Forestry-Robotics-UC/mapir-camera-ros2 "${MAPIR_SRC_DIR}"
fi
cd "${ROS2_WS}"
colcon build --symlink-install \
  --base-paths src/mapir-camera-ros2 \
  --packages-select mapir_camera_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS2 Workspace
source "${ROS2_WS}/install/setup.bash"

# Run MAPIR camera launch
compress_mode="${MAPIR_COMPRESS_MODE:-none}"
compress_topics_csv="${MAPIR_COMPRESS_TOPICS:-/mapir/image_raw}"
compress_pid=""
driver_pid=""

cleanup() {
  kill "${compress_pid}" >/dev/null 2>&1 || true
  kill "${driver_pid}" >/dev/null 2>&1 || true
  wait "${compress_pid}" 2>/dev/null || true
  wait "${driver_pid}" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

ros2 launch mapir_camera_ros2 mapir_camera.launch.py \
  namespace:=mapir \
  camera_impl:=cpp \
  camera_info_url:=file:///root/sensor_configs/mapir/mapir3_ocn_camera_info.yaml &
driver_pid="$!"

if [[ "${compress_mode}" != "none" ]]; then
  IFS=',' read -r -a compress_topics <<< "${compress_topics_csv}"
  /root/shared_folder/image-transport-republish.sh "${compress_mode}" "${compress_topics[@]}" &
  compress_pid="$!"
fi

if [[ -n "${compress_pid}" ]]; then
  wait -n "${driver_pid}" "${compress_pid}"
else
  wait "${driver_pid}"
fi
