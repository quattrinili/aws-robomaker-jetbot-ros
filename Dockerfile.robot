# Distribution of ROS2
ARG ROS_DISTRO="foxy"
ARG BASE_IMAGE=ros-foxy-l4t-base:r32.4.4
FROM ${BASE_IMAGE}
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /root

COPY nvidia-l4t-apt-source.list /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
RUN sudo apt-key adv --fetch-key http://repo.download.nvidia.com/jetson/jetson-ota-public.asc
RUN apt-get update

RUN apt-get update && apt-get install -y libopencv-python libopencv-dev libboost-python-dev libboost-dev libssl-dev \
          && apt-get install -y --no-install-recommends \
          python3-pip \
          python3-dev \
          python3-apt \
          build-essential \
          python3-tornado \
          python3-twisted \
          python3-pil \
          python3-bson \
          python3-opencv \
          zlib1g-dev \
          zip \
          libjpeg8-dev && rm -rf /var/lib/apt/lists/*

RUN pip3 install Adafruit-MotorHAT Adafruit_SSD1306 Cython wheel

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
