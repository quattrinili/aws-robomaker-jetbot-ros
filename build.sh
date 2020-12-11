#!/bin/bash
apt update
rosdep update
rosdep fix-permissions
rosdep install --from-paths src --ignore-src -r -y
colcon build