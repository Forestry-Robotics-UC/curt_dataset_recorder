FROM ros:kilted-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

#Install ROS Packages
RUN apt update && apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-nav2-common


#Configure catkin workspace
ENV ROS2_WS=/root/ros2_ws
RUN mkdir -p $ROS2_WS/src

RUN echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
