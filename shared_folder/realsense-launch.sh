#!/bin/bash
set -euo pipefail

#ROS 2 Middleware Implementation 

#Fastrtps
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

#Cyclonedds
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

#Build workspace only with the packages descriminated on docker compose file
ROS2_WS="${ROS2_WS:-/root/ros2_ws}"
cd "${ROS2_WS}"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS2 Workspace
source "${ROS2_WS}/install/setup.bash"

# Run realsense driver
LAUNCH_ARGS=()

# Override OEM CameraInfo with camera_calibration YAML when requested.
if [[ -n "${REALSENSE_COLOR_CAMERA_INFO_URL:-}" ]]; then
  LAUNCH_ARGS+=("color_camera_info_url:=${REALSENSE_COLOR_CAMERA_INFO_URL}")
fi

# Use the color-model intrinsics for aligned depth if you also want that topic replaced.
if [[ -n "${REALSENSE_ALIGNED_DEPTH_TO_COLOR_CAMERA_INFO_URL:-}" ]]; then
  LAUNCH_ARGS+=("aligned_depth_to_color_camera_info_url:=${REALSENSE_ALIGNED_DEPTH_TO_COLOR_CAMERA_INFO_URL}")
fi

compress_mode="${REALSENSE_COMPRESS_MODE:-none}"
compress_topics_csv="${REALSENSE_COMPRESS_TOPICS:-/camera/color/image_raw}"
compress_pid=""
driver_pid=""

cleanup() {
  kill "${compress_pid}" >/dev/null 2>&1 || true
  kill "${driver_pid}" >/dev/null 2>&1 || true
  wait "${compress_pid}" 2>/dev/null || true
  wait "${driver_pid}" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

ros2 launch realsense2_camera rs_launch.py "${LAUNCH_ARGS[@]}" &
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
