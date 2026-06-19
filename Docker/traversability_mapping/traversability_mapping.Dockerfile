FROM ros:jazzy-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt-get update \
    && apt-get install -y \
    libtbb-dev \
    libgoogle-glog-dev \
    packagekit-gtk3-module \
    libyaml-cpp-dev \
    libpcap-dev \
    libpthread-stubs0-dev \
    libeigen3-dev \
    liblcm-dev \
    libpcl-dev \
    libboost-all-dev

#Install ROS Packages
RUN apt-get install -y ros-${ROS_DISTRO}-bondcpp \
    ros-${ROS_DISTRO}-grid-map-cmake-helpers \
    ros-${ROS_DISTRO}-grid-map-costmap-2d \
    ros-${ROS_DISTRO}-grid-map-demos \
    ros-${ROS_DISTRO}-grid-map-loader \
    ros-${ROS_DISTRO}-grid-map-octomap \
    ros-${ROS_DISTRO}-grid-map-pcl \
    ros-${ROS_DISTRO}-grid-map-sdf \
    ros-${ROS_DISTRO}-gz-cmake-vendor \
    ros-${ROS_DISTRO}-gz-math-vendor \
    ros-${ROS_DISTRO}-gz-utils-vendor \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-interactive-markers \
    ros-${ROS_DISTRO}-laser-geometry \
    ros-${ROS_DISTRO}-map-msgs \
    ros-${ROS_DISTRO}-nav2-common \
    ros-${ROS_DISTRO}-nav2-costmap-2d \
    ros-${ROS_DISTRO}-nav2-util \
    ros-${ROS_DISTRO}-nav2-voxel-grid \
    ros-${ROS_DISTRO}-octomap \
    ros-${ROS_DISTRO}-octomap-msgs \
    ros-${ROS_DISTRO}-octomap-ros \
    ros-${ROS_DISTRO}-octomap-rviz-plugins \
    ros-${ROS_DISTRO}-octomap-server \
    ros-${ROS_DISTRO}-point-cloud-transport \
    ros-${ROS_DISTRO}-rviz-default-plugins \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-smclib \
    ros-${ROS_DISTRO}-pcl-* \
    ros-${ROS_DISTRO}-grid-map-core \
    ros-${ROS_DISTRO}-grid-map-ros \
    ros-${ROS_DISTRO}-grid-map-filters \
    ros-${ROS_DISTRO}-grid-map-rviz-plugin \
    ros-${ROS_DISTRO}-grid-map-visualization \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-sophus \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp


#Configure catkin workspace
ENV ROS2_WS=/root/ros2_ws
RUN mkdir -p $ROS2_WS/src

# WORKDIR $ROS2_WS/src
# RUN git clone https://github.com/suchetanrs/traversability_mapping.git

# WORKDIR $ROS2_WS
# RUN rosdep install --from-paths src --ignore-src -r -y --skip-keys Sophus
# RUN touch $ROS2_WS/src/traversability_mapping/ThirdParty/Sophus/COLCON_IGNORE && \
#     . /opt/ros/jazzy/setup.bash && \
#     colcon build --symlink-install

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
