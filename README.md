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
│   └── docker-compose.yml
├── ros2_ws/
│   ├── curt_description/
│   ├── imu_mag_fusion/
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
│   └── rm3100-launch.sh
├──sensor_configs/
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

A **docker-compose.yml** file creates all containers for the sensors and the recording.

### 1.2 Shared ROS 2 Workspace

The directory **ros2_ws/** is a workspace shared across all containers. Each container mounts:

- **ros2_ws/<sensor>-build/** → Build folder of each container

This prevents each container from rebuilding the full workspace and allows faster startup.

### 1.3 Shared Entry-Point Scripts

The folder **shared_folder/** contains launcher scripts used by each container:

- They source the ROS2 setup
- Build only the required packages
- And run the corresponding launch files or drivers

The recorder entry point runs the **hector_recorder** ROS2 command.

### 1.4 Sensor Configuration

This folder has every configuration needed to each sensor. Every sensor has its own directory and the files are linked to the respective containers.

### 1.5 CURTmini URDF Package

This folder, inside ros2_ws/, has the package needed to launch the CURTmini URDF with all the sensors.

### 1.6 IMU - Magnetometer Fusion Package

This directory, inside ros2_ws/, contains the package responsible for fusing data from the internal IMU (OpenZen) with the RM3100 magnetometer to compute the Attitude and Heading Reference System (AHRS). The fused output is published to the /imu/fused topic at a frequency of 500 Hz. Configuration options, topic names, can be adjusted in the parameters.yaml file located within the config/ subdirectory.

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
/ouster/lidar_packets /ouster/imu_packets /ouster/metadata /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /camera/color/metadata /camera/depth/metadata /camera/extrinsics/depth_to_color /camera/extrinsics/depth_to_depth /camera/color/camera_info /camera/aligned_depth_to_color/camera_info /camera/imu /imu/data /imu/mag /imu/fused /event_camera/events /mapir/camera_info /mapir/image_raw /mapir/indices/ndvi /fix /tf /tf_static /robot_description /mag
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

### 4.7 RM3100

Config files:
`params.yaml`

---
