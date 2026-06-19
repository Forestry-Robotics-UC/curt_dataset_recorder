cmake_minimum_required(VERSION 3.5)
project(realsense_camera_info_override)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

install(PROGRAMS
  src/camera_info_override.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
