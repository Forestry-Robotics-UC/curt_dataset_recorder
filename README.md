This repository provides a pipeline to record datasets with CURTmini robot. For a detailed instruction manual, please check the [Wiki](https://github.com/Forestry-Robotics-UC/curt_dataset_recorder/wiki)!

## 0. Clone Repository

This repository uses submodules. To clone it with all submodules:

```bash
git clone --recursive https://github.com/Forestry-Robotics-UC/curt_dataset_recorder.git
```

## 1. System Architecture

The entire data-acquisition system for the CURTmini is organized under:

```
~/curt_dataset_recorder/
├── Docker/
│   ├── ouster/
│   ├── emlid/
│   ├── realsense/
│   ├── recorder/
│   ├── xsens/
│   ├── curt/
│   ├── evk4/
│   ├── mapir/
│   ├── openzen/
│   ├── rm3100/
│   ├── foxglove/
│   └── docker-compose.yml
├── ros2_ws/
│   ├── curt_description/
│   ├── imu_mag_fusion/
│   ├── realsense_camera_info_override/
│   ├── ouster-build/
│   ├── emlid-build/
│   ├── realsense-build/
│   ├── recorder-build/
│   ├── xsens-build/
│   ├── curt-build/
│   ├── evk4-build/
│   ├── mapir-build/
│   ├── openzen-build/
│   └── rm3100-build/
├── shared_folder/
│   ├── ouster-launch.sh
│   ├── emlid-launch.sh
│   ├── realsense-launch.sh
│   ├── recorder-launch.sh
│   ├── xsens-launch.sh
│   ├── curt-launch.sh
│   ├── evk4-launch.sh
│   ├── mapir-launch.sh
│   ├── openzen-launch.sh
│   ├── rm3100-launch.sh
│   └── foxglove-launch.sh
├── sensor_configs/
│   ├── ouster/
│   ├── emlid/
│   ├── realsense/
│   ├── xsens/
│   ├── mapir/
│   ├── openzen/
│   └── rm3100/
└── startup.sh
```

### 1.1 Docker Containers

Each sensor package has its own Dockerfile inside its corresponding directory:

- **ouster/** → Ouster O1 LiDAR driver
- **emlid/** → GNSS-RTK (Reach M2) driver
- **realsense/** → Intel Realsense camera driver
- **xsens/** → Xsens IMU
- **curt/** → CURTmini URDF
- **evk4/** → Event-Based Camera
- **mapir/** → Mapir Survey3 Camera
- **openzen/** → CURTmini internal IMU
- **rm3100/** → RM3100 Magnetometer
- **recorder/** → hector_recorder
- **foxglove/** → Foxglove and RViZ docker containers

A **docker-compose.yml** file creates all containers for the sensors and the recording.

### 1.2 Shared ROS 2 Workspace

The directory ```ros2_ws/``` is a workspace shared across all containers. Each container mounts:

- **ros2_ws/<sensor>-build/** → Build folder of each container This prevents each container from rebuilding the full workspace and allows faster startup. This directory also contains two packages that are being shared to the containers:

### 1.3 Shared Entry-Point Scripts

The folder **shared_folder/** contains launcher scripts used by each container:

- They source the ROS2 setup
- Build only the required packages
- And run the corresponding launch files or drivers

The recorder entry point runs the **hector_recorder** ROS2 command.

### 1.4 Sensor Configuration

This folder has every configuration needed to each sensor. Every sensor has its own directory and the files are linked to the respective containers.

### 1.5 CURTmini URDF Package

This folder, inside ```ros2_ws/```, has the package needed to launch the CURTmini URDF with all the sensors.

### 1.6 IMU - Magnetometer Fusion Package

This directory, inside ```ros2_ws/```, contains the package responsible for fusing data from the internal **OpenZen IMU** with the **RM3100 magnetometer** to compute the **Attitude and Heading Reference System (AHRS)**. The fused output is published to the ```/imu/fused``` topic at a frequency of ```500 Hz```. Configuration options, topic names, can be adjusted in the parameters.yaml file located within the config/ subdirectory.

---

## 2. System Startup Procedure

### 2.1 Connecting to the CURT-NUC

The hotspot automatically powers on when the CURTmini robot is turned on.

It hosts a Wi-Fi hotspot:

- **SSID:** nuc-curt-mini-sn3
- **IP Address:** 10.42.0.1
- **Connect via SSH:**
  ```
  ssh curt@10.42.0.1
  ```

### 2.2 Launching the Recording System

Run the script **startup.sh** to start all the system components. The script will tranfer the RM3100 CAN device to the container and run:

1. **docker compose up -d**
   → Starts all sensor containers in the background
2. **docker compose run -i --rm recorder**
   → Opens an interactive shell and launches the hector_recorder TUI
   The recorder will then:

- Ask for a bag name (leave empty to auto-generate)
- Start recording once confirmed
  All ROS2 bags are saved outside the containers. In the directory that you mount to the recorder container, inside the docker-compose file. By default, ouside the repo main directory.

#### To close the system:

1. Press **Ctrl+C** to close the hector_recording and stop the system.
---

## 3. Recording Configuration

Current recording topics:

```text
/ouster/lidar_packets /ouster/imu_packets /ouster/metadata /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /camera/color/metadata /camera/depth/metadata /camera/extrinsics/depth_to_color /camera/extrinsics/depth_to_depth /camera/color/camera_info /camera/aligned_depth_to_color/camera_info /camera/imu /imu/data /imu/mag /imu/fused /event_camera/events /mapir/camera_info /mapir/image_raw /fix /tf /tf_static /mag
```

To modify what is recorded:

1. Edit the **recorder entry-point script** in **shared_folder/recorder-launch.sh**
2. Update:
   - **TOPICS variable** → to add/remove ROS2 topics
   - **hector_recorder command** → configure:
     - Bag size limit
     - Storage format (MCAP, SQLite)
     - Compression
     - Performance parameters
     - etc…

---

## 4. Sensor Configuration

All sensor configuration live inside sensor_configs/.
Configuration files are located here:

### 4.1 Emlid

Config file:
`emlid/nmea_serial_driver.yaml`
Launch file:
`emlid/nmea_serial_driver.py`

### 4.2 Ouster O1 LiDAR

Config file:

`ouster/driver_params.yaml`

### 4.3 Realsense Camera

Launch file:
`realsense/rs_launch.py`

Calibrated `camera_info` override:

- Put the YAML produced by `camera_calibration` in `sensor_configs/realsense/`.
- Set `REALSENSE_COLOR_CAMERA_INFO_URL` in `Docker/docker-compose.yml` to a `file:///` URL visible inside the container, for example:
  `file:///root/sensor_configs/realsense/color_camera_info.yaml`
- If you also want to replace `/camera/aligned_depth_to_color/camera_info`, set
  `REALSENSE_ALIGNED_DEPTH_TO_COLOR_CAMERA_INFO_URL` too. In many setups this
  can point at the same color calibration YAML.
- The launch now remaps the OEM topics to `.../camera_info_oem` and republishes
  calibrated `camera_info` on the original topic names, so downstream nodes can
  keep subscribing to `/camera/color/camera_info`.

Compressed Compose variants:

- `docker compose -f Docker/docker-compose.yml --profile realsense-png up realsense_png`
- `docker compose -f Docker/docker-compose.yml --profile realsense-ffmpeg up realsense_ffmpeg`

Default compressed topic:

- `/camera/color/image_raw/compressed` for the PNG path
- `/camera/color/image_raw/ffmpeg` for the ffmpeg path

Notes:

- The PNG path uses `compressed_image_transport` with `png`, so it is lossless.
- The ffmpeg path uses `ffmpeg_image_transport` with `libx264` and `crf:0` for minimum loss, but it should still be treated as near-lossless rather than byte-identical.
- By default only `/camera/color/image_raw` is republished. Override `REALSENSE_COMPRESS_TOPICS` in `Docker/docker-compose.yml` if you also want additional raw image topics republished through the same transport.

### 4.4 Xsens IMU

Launch file:
`xsens/xsens_driver.launch.xml`

### 4.5 OpenZen IMU

To configure the OpenZen IMU, download the [official software](https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1138294814/LPMS+Data+Acquisition+Software). However, is only compatible with Windows.

### 4.6 Mapir

Config files:
`mapir/mapir3_ocn_camera_info.yaml`
`mapir/mapir_camera_params.yaml`
`mapir/mapir_indices_params.yaml`
`mapir/rviz_mapir_indices.rviz`

Compressed Compose variants:

- `docker compose -f Docker/docker-compose.yml --profile mapir-png up mapir_png`
- `docker compose -f Docker/docker-compose.yml --profile mapir-ffmpeg up mapir_ffmpeg`

Default compressed topic:

- `/mapir/image_raw/compressed` for the PNG path
- `/mapir/image_raw/ffmpeg` for the ffmpeg path

Notes:

- The PNG path uses `compressed_image_transport` with `png`, so it is lossless.
- The ffmpeg path uses `ffmpeg_image_transport` with `libx264` and `crf:0` for minimum loss, but it should still be treated as near-lossless rather than byte-identical.
- Override `MAPIR_COMPRESS_TOPICS` in `Docker/docker-compose.yml` if the MAPIR launch exposes additional raw image topics that you want republished.

### 4.7 RM3100

Config files:
`params.yaml`

---

## 5. Revert CURTmini Host Multi-Robot Changes

This section describes the host-side configuration changes made to the CURTmini robot for multi-robot operation and network communication.

### 5.1 Workspace and URDF Modifications

**Reverting to original workspace:**
The original workspace path is `~/workspace (original)` (not modified). The following changes were made to the CURTmini `~/workspace` workspace:

| File | Change | Purpose |
|------|--------|---------|
| `~/workspace/src/curt_mini/curt_mini/bringup/robot_base.launch.py` | Commented out `imu_lpresearch` (internal OpenZen IMU) | OpenZen IMU driver is launched separately from `curt_dataset_recorder` |
| `~/workspace/src/curt_mini/curt_mini/models/*.xacro` | Updated to custom xacro files | Contains external sensors mounted on CURTmini with modified frame names (e.g., `base_link` → `base_link_curt`) |
| `~/workspace/src/curt_mini/curt_mini/config/ros2_control.yaml` | Changed `odom_frame_id` to `odom_curt`, `base_frame_id` to `base_link_curt` | Enable multiple robots on the same network sharing `/tf` topic without interference |
| `~/workspace/src/curt_mini/curt_mini/config/twist_mux.yaml` | Added `/nav2/cmd_vel_stamped` topic | Required for nav2 to send goals to the robot |

**After modifying the workspace:**
```bash
cd ~/workspace
colcon build
source ~/workspace/install/setup.bash
```

### 5.2 CycloneDDS Configuration

**Network Interface Setup:**
The host ROS 2 uses CycloneDDS for communication. By default, controllers use the `lo` (loopback) network interface. To enable ROS 2 communication over WiFi, the CycloneDDS configuration file must be updated.

- **Backup file:** `/opt/ros/cyclonedds-config.xml.bak` (original configuration)
- **Action:** Replace the active configuration with the backup.
- **WiFi Interface:** Currently configured for `wlxf8d1110ce656`

### 5.3 System Services

Two systemd services are configured to start at boot:

#### Hotspot Service
- **Service name:** `start_ap.service`
- **Purpose:** Enables hotspot mode on CURTmini startup for field SSH access
- **SSID:** nuc-curt-mini-sn3
- **IP Address:** 10.42.0.1

#### WiFi Network Service
- **Service name:** `start_wifi_specific_network.service`
- **Purpose:** Automatically connects to the designated router for multi-robot network operation

**Service Management Commands:**

| Command | Description |
|---------|-------------|
| `sudo systemctl enable start_ap.service` | Enable service to start on boot |
| `sudo systemctl disable start_ap.service` | Disable service from starting on boot |
| `sudo systemctl start start_ap.service` | Start the service immediately |
| `sudo systemctl stop start_ap.service` | Stop the service immediately |

### 5.4 Environment Configuration (.bashrc)

The following entries are added to `~/.bashrc` for convenience:

```bash
# Launch aliases
alias start="bash /$HOME/Documents/Duarte/curt_dataset_recorder/startup.sh"

# ROS 2 Domain ID
export ROS_DOMAIN_ID=37
```

---
