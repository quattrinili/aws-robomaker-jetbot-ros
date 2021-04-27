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

RUN apt-get install ros-foxy-rosauth python3-tornado python3-twisted python3-pil python3-bson -y

RUN apt-get install python3-opencv python3-pil -y
RUN pip3 install Adafruit_SSD1306

ADD robot_ws/ /root/ros2_ws/
WORKDIR /root/ros2_ws/

RUN mkdir -p src/deps
RUN vcs import src/deps < deps.repos
RUN mkdir /opt/jetbot_ros/
RUN . /opt/ros/foxy/setup.sh && colcon build --install-base /opt/jetbot_ros/
WORKDIR /
ENV ROS_DISTRO foxy
ENV APP_NAME jetbot_ros
ADD app_entrypoint.sh /app_entrypoint.sh
RUN chmod +x /app_entrypoint.sh
ENTRYPOINT ["./app_entrypoint.sh"]
