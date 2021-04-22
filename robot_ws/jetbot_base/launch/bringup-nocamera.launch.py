#!/usr/bin/env python
"""
Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software
is furnished to do so.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED,INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import os

DEFAULT_SPEED = os.environ.get('DEFAULT_SPEED', '0.1')
MAX_PWM =  os.environ.get('MAX_PWM', '115')
I2C_BUS = os.environ.get('I2C_BUS', '1')
ROBOT_ID = os.environ.get('ROBOT_ID', 'robo1')

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[ROBOT_ID, '_'],
            description='Prefix for node names'),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py'))),
        launch_ros.actions.Node(
             package='jetbot_base', executable='jetbot_motor_controller', output='screen',
             parameters=[
                {"default_speed": DEFAULT_SPEED},
                {"i2c_bus": I2C_BUS},
                {"max_pwm": MAX_PWM}
             ],
             name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'jetbot_motor_controller']),
    ])

if __name__ == '__main__':
    generate_launch_description()