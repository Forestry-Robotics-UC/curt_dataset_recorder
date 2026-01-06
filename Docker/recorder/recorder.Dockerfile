FROM ros:jazzy-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt update \
    && apt install -y \
    libeigen3-dev \
    libjsoncpp-dev \
    libspdlog-dev \
    libcurl4-openssl-dev \
    libpcap-dev \
    libopencv-dev \
    libboost-all-dev \
    libncurses-dev

#Install ros2 pkg
RUN apt -y install ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-realsense2-camera-msgs \
    ros-jazzy-pcl-ros \
    ros-jazzy-tf2-eigen

#Configure catkin workspace
ENV CATKIN_WS=/root/ros2_ws
RUN mkdir -p $CATKIN_WS/src

WORKDIR $CATKIN_WS/src

RUN git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git

RUN git clone https://github.com/tu-darmstadt-ros-pkg/hector_recorder.git

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Clean-up
WORKDIR /root
RUN apt-get clean

CMD ["bash"]
