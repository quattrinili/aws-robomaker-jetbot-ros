ARG ROS_DISTRO=foxy
FROM ros:${ROS_DISTRO}

RUN apt-get update && rosdep update && rosdep fix-permissions
RUN apt-get install -y python3-pip python3-apt

RUN apt-get install ros-foxy-example-interfaces ros-foxy-navigation2 ros-foxy-vision-opencv -y

RUN pip3 install Adafruit-MotorHAT

WORKDIR /root
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git
RUN cd YDLidar-SDK/build && \
    cmake .. && \
    make && \
    make install

ADD robot_ws/ /root/ros2_ws/
WORKDIR /root/ros2_ws/
RUN . /opt/ros/foxy/setup.sh && rosdep update && rosdep fix-permissions

RUN mkdir src/deps && vcs import src/deps < deps.repos
RUN rosdep install --from src -i -y
RUN mkdir /opt/jetbot_ros/
RUN . /opt/ros/foxy/setup.sh && colcon build --install-base /opt/jetbot_ros/

