ARG ROS_DISTRO=foxy
FROM ros:${ROS_DISTRO}

RUN apt-get update && rosdep update && rosdep fix-permissions
RUN apt-get update && apt-get install -y python3-pip python3-apt ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup -y

RUN pip3 install Adafruit-MotorHAT

WORKDIR /root
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git
RUN cd YDLidar-SDK/build && \
    cmake .. && \
    make && \
    make install
    
RUN apt-get install python3-pil python3-bson python3-tornado python3-twisted python3-opencv -y