#!/usr/bin/env bash
set -e

# Default States
LIDAR=1
IMU=1
RTK=1
RGB=1
RECORD=1
VIZUALIZATION=1
QUIET=0
MAG=0
NAVIGATION=0
RGBD=0
EVENT=0
MAPIR=0
COMPRESSION=0

# Help message function
show_help() {
  echo "Usage: $0 [OPTIONS] [BAG_NAME]"
  echo ""
  echo "The default behaviour is to RECORD the LiDAR, IMU, RTK and Compressed RGBD from RealSense."
  echo "The vizualization by default is enable with a Foxglove websocket open on port 9092 to allow remote vizualization."
  echo ""
  echo "Options:"
  echo "  -a, --all          Enable all sensors"
  echo "  -v, --verbose      Enable verbose mode"
  echo "  -e, --enable       Enable list of sensors [l, i, r, rtk, mag, event, rgb, rgbd, mapir]"
  echo "  -d, --disable      Disable list of sensors [l, i, r, rtk, mag, event, rgb, rgbd, mapir]"
  echo "  -c, --compression  Enable compression of Image topics"
  echo "  -r, --record       Enable recording"
  echo "  -n, --nav          Enable Navigation"
  echo "  -nv, --no-viz      Disable Vizualization"
  echo "  --quiet            Suppress output"
  echo "  --help             Show this help message"
  echo ""
  echo "Arguments:"
  echo "  BAG_NAME           Optional bag file name. If not provided, a random name from .names file will be used."
  echo "                     Spaces will be replaced with underscores."
  echo ""
  echo "Examples:"
  echo "  $0                                 # Use random bag name"
  echo "  $0 my_bag_name                     # Use 'my_bag_name' as bag name"
  echo "  $0 -a my_bag_name                 # Enable all sensors with custom bag name"
  echo "  $0 -d l,i -c experiment_01        # Disable LiDAR and IMU, enable compression"
  exit 0
}

# Function to enable sensors
enable_sensors() {
  for sensor in $1; do
    case $sensor in
      l|LIDAR) LIDAR=1 ;;
      i|IMU) IMU=1 ;;
      r|RGB) RGB=1 ;;
      rtk|RTK) RTK=1 ;;
      event|EVENT) EVENT=1 ;;
      rgb|RGB) RGB=1 ;;
      rgbd|RGBD) RGBD=1 ;;
      mapir|MAPIR) MAPIR=1 ;;
      mag|MAG) MAG=1 ;;
      *) echo "Unknown sensor: $sensor" >&2 ;;
    esac
  done
}

# Function to disable sensors
disable_sensors() {
  for sensor in $1; do
    case $sensor in
      l|LIDAR) LIDAR=0 ;;
      i|IMU) IMU=0 ;;
      r|RGB) RGB=0 ;;
      rtk|RTK) RTK=0 ;;
      event|EVENT) EVENT=0 ;;
      mag|MAG) MAG=0 ;;
      rgb|RGB) RGB=0 ;;
      rgbd|RGBD) RGBD=0 ;;
      mapir|MAPIR) MAPIR=0 ;;
      *) echo "Unknown sensor: $sensor" >&2 ;;
    esac
  done
}

# Function to enable all sensors
enable_all_sensors() {
  LIDAR=1
  IMU=1
  RTK=1
  RGB=1
  RGBD=1
  EVENT=1
  MAPIR=1
  MAG=1
}

# Decide which TOPICS to record
build_topics() {
  TOPICS=""
  
  # TF is always recorded (needed for transforms)
  TOPICS="$TOPICS /tf /tf_static"
  
  # LiDAR topics
  if [ "$LIDAR" -eq 1 ]; then
    TOPICS="$TOPICS /ouster/points /ouster/metadata /ouster/imu"
  fi
  
  # IMU topics
  if [ "$IMU" -eq 1 ]; then
    TOPICS="$TOPICS /curt/imu/data"
  fi
  
  # Magnetometer topics
  if [ "$MAG" -eq 1 ]; then
    TOPICS="$TOPICS /curt/imu/mag /curt/mag"
  fi
  
  # Fused IMU data (record when either IMU or MAG is enabled)
  if [ "$IMU" -eq 1 ] || [ "$MAG" -eq 1 ]; then
    TOPICS="$TOPICS /curt/imu/fused"
  fi
  
  # RTK/GPS topics
  if [ "$RTK" -eq 1 ]; then
    TOPICS="$TOPICS /curt/fix"
  fi
  
  # RGB camera topics
  if [ "$RGB" -eq 1 ]; then
    if [ "$COMPRESSION" -eq 1 ]; then
      TOPICS="$TOPICS /curt/camera_curt/color/image_raw/compressed"
    else
      TOPICS="$TOPICS /curt/camera_curt/color/image_raw"
    fi
    TOPICS="$TOPICS /curt/camera_curt/color/metadata /curt/camera_curt/color/camera_info"
  fi
  
  # RGBD (depth) topics
  if [ "$RGBD" -eq 1 ]; then
    TOPICS="$TOPICS /curt/camera_curt/aligned_depth_to_color/image_raw /curt/camera_curt/depth/metadata /curt/camera_curt/aligned_depth_to_color/camera_info /curt/camera_curt/extrinsics/depth_to_color /curt/camera_curt/extrinsics/depth_to_depth"
  fi
  
  # Event camera topics
  if [ "$EVENT" -eq 1 ]; then
    TOPICS="$TOPICS /event_camera/events"
  fi
  
  # MAPIR camera topics
  if [ "$MAPIR" -eq 1 ]; then
    TOPICS="$TOPICS /mapir/camera_info /mapir/image_raw/ffmpeg /mapir/indices/ndvi /mapir/indices_color/ndvi"
  fi
  
  # Navigation topics
  if [ "$NAVIGATION" -eq 1 ]; then
    TOPICS="$TOPICS /goal_pose /nav2/cmd_vel_stamped /path /plan /goal_checker_selector"
  fi
  
  # Trim leading space
  TOPICS="${TOPICS# }"
  
  # If quiet mode is off, print the selected topics
  if [ "$QUIET" -eq 0 ]; then
    echo "Recording topics: $TOPICS"
  fi
}



# Parse arguments
while getopts ":aed:crnvq-:" opt; do
  case $opt in
    a) enable_all_sensors ;;
    e) ENABLE_LIST="$OPTARG" ;;
    d) DISABLE_LIST="$OPTARG" ;;
    c) COMPRESSION=1 ;;
    r) RECORD=1 ;;
    n) NAVIGATION=1 ;;
    nv) VIZUALIZATION=0 ;;
    v) VERBOSE=0 ;;
    q) QUIET=0 ;;
    -) # Handle long options
      case "${OPTARG}" in
        all) enable_all_sensors ;;
        enable) ENABLE_LIST="${!OPTIND}"; OPTIND=$((OPTIND+1)) ;;
        disable) DISABLE_LIST="${!OPTIND}"; OPTIND=$((OPTIND+1)) ;;
        compression) COMPRESSION=1 ;;
        record) RECORD=1 ;;
        nav) NAVIGATION=1 ;;
        verbose) VERBOSE=1 ;;
        quiet) QUIET=1 ;;
        no-viz) VIZUALIZATION=0 ;;
        help) show_help ;;
        *) echo "Unknown option: --${OPTARG}" >&2; exit 1 ;;
      esac ;;
    \?) echo "Invalid option: -$OPTARG" >&2; exit 1 ;;
    :) echo "Option -$OPTARG requires an argument." >&2; exit 1 ;;
  esac
done


# Process enable/disable lists if provided
if [ -n "$ENABLE_LIST" ]; then
  enable_sensors "$ENABLE_LIST"
fi
if [ -n "$DISABLE_LIST" ]; then
  disable_sensors "$DISABLE_LIST"
fi



#TOPICS="/ouster/points /ouster/metadata /curt/camera_curt/color/image_raw/compressed /curt/camera_curt/aligned_depth_to_color/image_raw /curt/camera_curt/color/metadata /curt/camera_curt/depth/metadata /curt/camera_curt/extrinsics/depth_to_color /curt/camera_curt/extrinsics/depth_to_depth /curt/camera_curt/color/camera_info /curt/camera_curt/aligned_depth_to_color/camera_info /curt/imu/data /curt/imu/mag /curt/imu/fused /mapir/camera_info /mapir/image_raw/ffmpeg /mapir/indices/ndvi /mapir/indices_color/ndvi /curt/fix /curt/mag /tf /tf_static"


build_topics


# After parsing options, check for positional argument
shift $((OPTIND-1))

if [ $# -gt 0 ]; then
    BAGFILE_NAME="$1"
    # Sanitize
    BAGFILE_NAME=$(echo "$BAGFILE_NAME" | tr ' ' '_' | tr -cd '[:alnum:]_.-')
fi


# Cleanup function
cleanup() {
    local exit_code=$?

    # Eliminate containers
    docker compose down

    #PROBABLY NOT NEEDED BUT I AM NOT SURE
    #docker rm mapir-ffmpeg mapir-png realsense-ffmpeg realsense-png >/dev/null 2>&1 || true
    
    return $exit_code
}

# Set trap only when not on quiet mode
if [ "$QUIET" -eq 0 ]; then
  trap cleanup EXIT
fi


#Restart Robot ROS2
#sudo systemctl restart ipa-ros-autostart.service # MAYBE NOT NEEDED

# Bring up the network connection (if needed)
#sudo nmcli connection up Ouster

# Kill the ROS2 openzen IMU node from host (if needed)
sudo pkill -f "openzen_node" || true

# --- CONFIGURATION ---
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
cd $SCRIPT_DIR/Docker

# STARTING SENSORS CONTAINERS
echo "Starting Sensors..."
docker compose up -d \
  --scale recorder=0 \
  --scale mapir_ffmpeg=0 \
  --scale ouster=$LIDAR \
  --scale openzen=$IMU \
  --scale evk4=$EVENT \
  --scale rm3100=$MAG \
  --scale emlid=$RTK \
  --scale realsense=$RGB \
  --scale foxglove=$VIZUALIZATION \
  --scale glim=$NAVIGATION \
  --scale traversability_ros2=$NAVIGATION \
  --scale nav2=$NAVIGATION


# Wait for container to be ready
sleep 2

# Run the recorder
if [ "$RECORD" -eq 1 ]; then
  if [ "$QUIET" -eq 1 ]; then
    docker compose run  -d --rm -e HEADLESS=1 -e TOPICS="$TOPICS" -e BAGFILE_NAME="$BAGFILE_NAME" recorder
  else
    docker compose run --rm -e TOPICS="$TOPICS" -e BAGFILE_NAME="$BAGFILE_NAME" recorder
  fi

fi