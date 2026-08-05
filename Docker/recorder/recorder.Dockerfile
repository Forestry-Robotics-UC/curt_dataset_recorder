FROM ros:jazzy-ros-base

LABEL maintainer="Mário Cristóvão <mario.cristovao@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Define user and workspace
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=1000
ENV CATKIN_WS=/home/${USERNAME}/ros2_ws

# Create the user with custom UID/GID
RUN set -eux; \
    if ! getent group ${USER_GID} >/dev/null; then \
        groupadd --gid ${USER_GID} ${USERNAME}; \
    fi; \
    if id -u ${USERNAME} >/dev/null 2>&1; then \
        usermod --uid ${USER_UID} --gid ${USER_GID} --home /home/${USERNAME} --shell /bin/bash ${USERNAME}; \
    elif getent passwd ${USER_UID} >/dev/null; then \
        EXISTING_USER="$(getent passwd ${USER_UID} | cut -d: -f1)"; \
        usermod --login ${USERNAME} --gid ${USER_GID} --home /home/${USERNAME} --shell /bin/bash "${EXISTING_USER}"; \
    else \
        useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME}; \
    fi; \
    mkdir -p /home/${USERNAME}; \
    chown -R ${USER_UID}:${USER_GID} /home/${USERNAME}; \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

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
    libncurses-dev \
    libzip-dev \
    sudo

# Install ROS 2 packages
RUN apt -y install ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-realsense2-camera-msgs \
    ros-jazzy-pcl-ros \
    ros-jazzy-metavision-driver \
    ros-jazzy-tf2-eigen \
    ros-jazzy-compressed-image-transport \
    ros-jazzy-ffmpeg-image-transport \
    ros-${ROS_DISTRO}-cv-bridge

# Configure catkin workspace
RUN mkdir -p ${CATKIN_WS}/src
RUN chown -R ${USER_UID}:${USER_GID} ${CATKIN_WS}

WORKDIR ${CATKIN_WS}/src

# Clone repositories
RUN git clone -b ros2 --recurse-submodules https://github.com/errorcodecritical/ouster-ros.git
RUN git clone https://github.com/tu-darmstadt-ros-pkg/hector_recorder.git

# Set up .bashrc for the user
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source ${CATKIN_WS}/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.bashrc

# Clean-up
WORKDIR /home/${USERNAME}
RUN apt-get clean

# Switch to the user
USER ${USERNAME}
WORKDIR ${CATKIN_WS}

CMD ["bash"]