FROM ros:jazzy-ros-base

LABEL maintainer="Mario Cristovao <mario.cristovao@isr.uc.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install some packages
RUN apt update 
RUN apt -y install apt-utils build-essential software-properties-common wget unzip curl git cmake
RUN apt -y install libopencv-dev libboost-all-dev libusb-1.0-0-dev libprotobuf-dev protobuf-compiler
RUN apt -y install libhdf5-dev hdf5-tools libglew-dev libglfw3-dev libcanberra-gtk-module ffmpeg 

# Install some Python Dependencies
RUN apt -y install python3-venv python3-dev python3-vcstool

#Install ROS Packages
RUN apt-get install -y \
    ros-jazzy-ament-cmake-clang-format \
    ros-jazzy-tf-transformations \
    ros-jazzy-rqt-image-view \
    ros-jazzy-rmw-cyclonedds-cpp

#INSTALL OPENEB FROM SOURCE

WORKDIR /root/
RUN git clone https://github.com/prophesee-ai/openeb.git

#solve python env
RUN python3 -m venv /tmp/prophesee/py3venv --system-site-packages
RUN /tmp/prophesee/py3venv/bin/python -m pip install pip --upgrade
RUN /tmp/prophesee/py3venv/bin/python -m pip install -r /root/openeb/utils/python/requirements_openeb.txt

#Compile and install OpenEB
WORKDIR /root/openeb/build/
RUN cmake .. -DBUILD_TESTING=OFF
RUN cmake --build . --config Release -- -j 4
RUN sudo cmake --build . --target install


#Configure catkin workspace
ENV CATKIN_WS=/root/ros2_ws
RUN mkdir -p $CATKIN_WS/src

#Install OpenEBA pkg
WORKDIR $CATKIN_WS/src
RUN git clone https://github.com/ros-event-camera/metavision_driver.git
RUN vcs import < metavision_driver/metavision_driver.repos


#Build workspace
WORKDIR $CATKIN_WS
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Clean-up
WORKDIR /root
RUN apt-get clean

CMD ["bash"]
