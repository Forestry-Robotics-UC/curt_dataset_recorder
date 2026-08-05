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

# Build only the recorder and custom message packages needed at runtime.
# This avoids pulling in full sensor driver builds like ouster_ros.
cd /root/ros2_ws/
colcon build --symlink-install \
  --packages-select \
    hector_recorder_msgs \
    hector_recorder \
    ouster_sensor_msgs \
    scovox_msgs \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source /root/ros2_ws/install/setup.bash
sleep 10

wait_for_topic() {
  local topic="$1"
  local timeout_s="${2:-30}"
  local i=0

  while [ "$i" -lt "$timeout_s" ]; do
    if ros2 topic list 2>/dev/null | grep -Fx "$topic" >/dev/null; then
      echo "Found topic: $topic"
      return 0
    fi
    sleep 1
    i=$((i + 1))
  done

  echo "Timed out waiting for topic: $topic"
  return 1
}

#Database of random names to bags
names_dir="/root/shared_folder"
name_1=$(shuf -n 1 "$names_dir/.names_1")

# Directory where the bags are saved
BAG_DIR="/root/rosbags"

# Prompt user to give a bag name, otherwise give a random name from database
default_label="${name_1}"
if [ -t 0 ]; then
  read -r -p "Enter bag name (leave empty for default name): " user_label
else
  user_label=""
fi
if [ -z "$user_label" ]; then
  label="$default_label"
else
  label="$(echo "$user_label" | tr ' ' '_' | tr -cd '[:alnum:]_.-')"
fi
BAG_NAME="$(date +%Y_%m_%d_%H_%M_%S)__${label}_"

#Topics to record
#Ouster Points /ouster/points /ouster/imu 
#TOPICS="/ouster/lidar_packets /ouster/imu_packets /ouster/metadata /curt/camera_curt/color/image_raw /curt/camera_curt/color/metadata /curt/camera_curt/color/camera_info /curt/camera_curt/imu /imu/data /imu/mag /imu/fused /event_camera/events /mapir/camera_info /mapir/image_raw/ffmpeg /fix /tf /tf_static /mag"
#TOPICS="/ouster/points /ouster/metadata /curt/camera_curt/color/image_raw/compressed /curt/camera_curt/aligned_depth_to_color/image_raw /curt/camera_curt/color/metadata /curt/camera_curt/depth/metadata /curt/camera_curt/extrinsics/depth_to_color /curt/camera_curt/extrinsics/depth_to_depth /curt/camera_curt/color/camera_info /curt/camera_curt/aligned_depth_to_color/camera_info /curt/imu/data /curt/imu/mag /curt/imu/fused /mapir/camera_info /mapir/image_raw/ffmpeg /mapir/indices/ndvi /mapir/indices_color/ndvi /curt/fix /curt/mag /tf /tf_static"
#TOPICS="/ouster/lidar_packets /ouster/imu_packets /ouster/metadata /camera/color/image_raw/compressed /camera/color/metadata /camera/color/camera_info /curt/imu/data /curt/imu/mag /curt/fix /tf /tf_static"
TOPICS="/ouster/points /ouster/imu /ouster/metadata /camera/color/image_raw/compressed /camera/color/metadata /camera/color/camera_info /camera/aligned_depth_to_color/image_raw/compressed /camera/aligned_depth_to_color/camera_info /curt/imu/data /curt/imu/mag  /mapir/image_raw/ffmpeg /diagnostics /tf /tf_static"
#TOPICS="/ouster/points /ouster/imu /ouster/metadata /imu/data"


mkdir -p "$BAG_DIR"

# Start ROS2 bag recording
echo "Starting ROS 2 bag recording..."
#Run hector_recorder
#ros2 run hector_recorder record -o "$BAG_DIR/$BAG_NAME" --max-bag-size-gb 5 --topics $TOPICS &
ros2 run hector_recorder record -o "$BAG_DIR/$BAG_NAME" --max-bag-size-gb 5 &


BAG_PID=$!

trap "echo 'Stopping recording...'; kill -2 $BAG_PID 2>/dev/null || true" SIGINT SIGTERM

# Wait for recorder without aborting on non-zero (e.g., SIGINT=130)
set +e
wait $BAG_PID
REC_EXIT=$?
set -e
sync
echo "Rosbag stopped"

# Generate bag info file
bag_path=$(ls -1dt "$BAG_DIR/${BAG_NAME}"* 2>/dev/null | head -n1)
if [ -z "$bag_path" ]; then
 echo "No bag path found for ${BAG_NAME}"
 exit 1
fi

#Wait briefly for metadata.yaml to appear (up to 30s)
for i in $(seq 1 30); do
 if [ -f "$bag_path/metadata.yaml" ]; then
   break
 fi
 sleep 1
done

#Generate info; don't abort the script if this fails
if ! ros2 bag info "$bag_path" > "$bag_path/info.txt"; then
 echo "ros2 bag info failed; info.txt not generated"
fi
sync
echo "Rosbag info saved"
