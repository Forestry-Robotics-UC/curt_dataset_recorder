FROM ros:jazzy-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

#Configure catkin workspace
ENV CATKIN_WS=/root/ros2_ws
RUN mkdir -p $CATKIN_WS/src

#Install System Packages
RUN apt update && apt install -y \
    libbluetooth-dev

#Install ROS Packages
RUN apt install -y \
    ros-jazzy-rmw-cyclonedds-cpp

#Clone openzen pkg
WORKDIR $CATKIN_WS/src
RUN git clone --recurse-submodules https://bitbucket.org/lpresearch/openzenros2.git

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Clean-up
WORKDIR /root
RUN apt-get clean

CMD ["bash"]
