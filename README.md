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
docker-compose build
```

## Running

In the robot container, start a simple application.
```
docker-compose up
```

## Deploying with AWS Greengrass 2.0

TODO

## Dependencies

- 

