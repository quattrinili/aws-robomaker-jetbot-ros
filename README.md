## NVidia Jetbot - ROS2 Sample Application

### Nodes

jetbot_base
- jetbot_motor_controller
- jetbot_camera

jetbot_move
- circle

## Build Steps

The best way is to build this in a container. First, follow the instructions (here)[https://github.com/dusty-nv/jetson-containers] to create a new ROS2 container on your NVidia Jetbot. Once you have a container, connect to your container:

```
git clone https://github.com/jerwallace/aws-robomaker-jetbot-ros.git -b ros2 src/aws-robomaker-jetbot-ros
docker build robot_ws/docker/. -t jetbot-ros:robot
docker build simulation_ws/docker/. -t jetbot-ros:simulation
```

In one tab, open the robot_ws docker container and build the application:
```
cd robot_ws
sudo docker run -v `pwd`:/ws --device /dev/ttyUSB0 --device /dev/i2c-1 --device /dev/video0 --volume /tmp/argus_socket:/tmp/argus_socket -it jetbot-ros:robot /bin/bash
source /opt/ros/foxy/install/setup.bash
cd /ws
rosdep install --from src -i
mkdir deps && vcs import src/deps < deps.repos 
colcon build
```

In another tab, open the simulation_ws docker container:
```
cd simulation_ws
sudo docker run -v `pwd`:/ws --network host -e DISPLAY=:0 --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -it jetbot-ros:simulation /bin/bash
source /opt/ros/foxy/install/setup.bash
cd /ws
rosdep install --from src -i
colcon build
```

## Running

In the robot container, start a simple application.
```
ros2 launch jetbot_move circle.launch.py
```

## Deploying with AWS Greengrass 2.0

TODO

## Dependencies

- 

