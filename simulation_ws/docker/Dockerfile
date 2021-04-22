ARG ROS_DISTRO=foxy
FROM ros:${ROS_DISTRO}

RUN apt-get update && rosdep update && rosdep fix-permissions
RUN apt-get update && apt-get install -y python3-pip python3-apt ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup -y
    
RUN apt-get install python3-pil python3-bson python3-tornado python3-twisted python3-opencv -y
RUN apt-get install ros-foxy-gazebo-ros ros-foxy-gazebo-plugins ros-foxy-rosauth ros-foxy-example-interfaces ros-foxy-gazebo-ros-pkgs gazebo11 -y