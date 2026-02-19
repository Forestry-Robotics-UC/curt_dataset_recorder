FROM ros:jazzy-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install some python packages
RUN apt update \
    && apt install -y \
    libgeographiclib-dev \
    libcxxopts-dev \
    libexpected-dev \
    libboost-dev \
    iproute2 \
    python3-pip

RUN pip install dronecan --break-system-packages

#Install ROS Packages
RUN apt install -y \
    ros-$ROS_DISTRO-angles \
    ros-$ROS_DISTRO-topic-tools \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-backward-ros \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-imu-tools

#Configure catkin workspace
ENV CATKIN_WS=/root/ros2_ws
RUN mkdir -p $CATKIN_WS/src

WORKDIR $CATKIN_WS/src

# Clone RM3100 magnetometer driver
RUN git clone https://github.com/Forestry-Robotics-UC/go1_magnetometer_rm3100.git

# Clone necessary repos
RUN git clone -b ros2 https://github.com/ctu-vras/compass.git
RUN git clone https://github.com/TartanLlama/expected.git
RUN git clone -b ros2 https://github.com/ctu-vras/ros-utils.git
RUN git clone https://github.com/ctu-vras/cras_msgs.git

# Build workspace
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Clean-up
WORKDIR /root
RUN apt-get clean

CMD ["bash"]
