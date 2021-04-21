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
mkdir -p ros_ws/src && cd ros_ws
git clone https://github.com/jerwallace/aws-robomaker-jetbot-ros.git -b ros2 src/aws-robomaker-jetbot-ros
sudo docker run -v `pwd`:/ws --device /dev/ttyUSB0 --device /dev/i2c-1 --device /dev/video0 --volume /tmp/argus_socket:/tmp/argus_socket -it <YOUR_IMAGE_NAME> /bin/bash
```

Now, you should be inside the docker container. Run the following build steps:
```
source /opt/ros/foxy/install/setup.bash
cd /ws/src/aws-robomaker-jetbot-ros
mkdir deps && vcs import deps < deps.repos 
cd /ws
colcon build
```

## Running

Once in the container, you can run the sample application with the following launch:

```
ros2 launch jetbot_move circle.launch.py
```

## Deploying with AWS Greengrass 2.0

TODO

## Dependencies

- 

