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
git clone https://github.com/jerwallace/aws-robomaker-jetbot-ros.git -b ros2
cd aws-robomaker-jetbot-ros 
```

### For running on the Jetbot:
```
export MODE=robot
docker-compose -f $MODE.compose.yml build 
```

### For running in Simulation on a Mac:
```
export MODE=simulation_mac
docker-compose -f $MODE.compose.yml build 
```

## Running

Parameters for running different applications are stored in the `.env` file.

### Simple circle on the Jetbot

```
export MODE=robot
export ROBOT_ROS_PKG=jetbot_move
export ROBOT_ROS_LAUNCH=circle_with_bringup
docker-compose -f $MODE.compose.yml up
```

### Simulation on a Mac

In the robot container, start a simple application.

```
export MODE=simulation_mac
export IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}') 
echo IP=$IP >> .env
xhost +$IP
docker-compose -f $MODE.compose.yml up
```

On a mac
```


```

## Deploying with AWS Greengrass 2.0

TODO

## Dependencies

- 

