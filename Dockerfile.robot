ARG ROS_DISTRO=foxy
FROM ros:${ROS_DISTRO}

RUN apt-get update && rosdep update && rosdep fix-permissions
RUN apt-get update && apt-get install -y python3-pip python3-apt

RUN pip3 install Adafruit-MotorHAT

WORKDIR /root
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git
RUN cd YDLidar-SDK/build && \
    cmake .. && \
    make && \
    make install

ADD robot_ws/src /root/ros2_ws/src
WORKDIR /root/ros2_ws/
RUN . /opt/ros/foxy/setup.sh && rosdep update && rosdep fix-permissions
RUN rosdep install --from src -i -y
RUN mkdir /opt/jetbot_ros/
RUN colcon build --install-base /opt/jetbot_ros/
