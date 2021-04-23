#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/opt/$APP_NAME/setup.bash"
exec "$@"