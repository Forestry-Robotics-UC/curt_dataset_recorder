FROM ros:jazzy-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

#Install ROS Packages
RUN apt update \
    && apt install -y \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-librealsense* \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-rmw-cyclonedds-cpp

#Configure catkin workspace
ENV CATKIN_WS=/root/ros2_ws
RUN mkdir -p $CATKIN_WS/src

#Clone realsense pkg
WORKDIR $CATKIN_WS/src
RUN git clone https://github.com/IntelRealSense/realsense-ros.git

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Clean-up
WORKDIR /root
RUN apt-get clean

CMD ["bash"]
