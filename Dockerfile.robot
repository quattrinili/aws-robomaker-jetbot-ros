# Distribution of ROS2
ARG ROS_DISTRO="foxy"
ARG BASE_IMAGE=ros-foxy-l4t-base:r32.4.4
FROM ${BASE_IMAGE}
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /root

RUN apt-get update && apt-get install -y python3-pip python3-apt

RUN apt-get install python3-tornado python3-twisted python3-pil python3-bson python3-opencv -y

RUN pip3 install Adafruit-MotorHAT Adafruit_SSD1306

RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git
RUN cd YDLidar-SDK/build && \
    cmake .. && \
    make && \
    make install

ADD robot_ws/ /root/ros2_ws/
WORKDIR /root/ros2_ws/

RUN mkdir -p src/deps
RUN vcs import src/deps < deps.repos
RUN mkdir /opt/jetbot_ros/
RUN . /opt/ros/foxy/setup.sh && colcon build --install-base /opt/jetbot_ros/

WORKDIR /
ENV APP_NAME jetbot_ros
ADD app_entrypoint.sh /app_entrypoint.sh
RUN chmod +x /app_entrypoint.sh
ENTRYPOINT ["./app_entrypoint.sh"]
