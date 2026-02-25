FROM ros:jazzy-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt-get update \
    && apt-get install -y \
    cmake \
    libeigen3-dev \
    libjsoncpp-dev \
    libspdlog-dev \
    libcurl4-openssl-dev \
    libpcap-dev \
    libopencv-dev

#Install ROS Packages
RUN apt-get install -y ros-jazzy-pcl-ros \
    ros-jazzy-tf2-eigen \
    ros-jazzy-rmw-cyclonedds-cpp

#Configure catkin workspace
ENV CATKIN_WS=/root/ros2_ws
RUN mkdir -p $CATKIN_WS/src

#Clone Ouster ROS2 pkg
WORKDIR $CATKIN_WS/src
#RUN git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
RUN git clone -b ros2 --recurse-submodules https://github.com/errorcodecritical/ouster-ros.git

#Build workspace
WORKDIR $CATKIN_WS
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Clean-up
WORKDIR /root
RUN apt-get clean

CMD ["bash"]
