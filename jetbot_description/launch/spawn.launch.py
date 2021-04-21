import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='x_pos',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            name='y_pos',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            name='z_pos',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            name='roll',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            name='pitch',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            name='yaw',
            default_value='0.0'
        ),
        Node(
            package='gazebo_ros',
            node_executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity',
                'robot',
                '-file',
                os.path.join(get_package_share_directory(
                    'jetbot_description'), 'urdf', 'jetbot.urdf'),
                '-x',
                LaunchConfiguration('x_pos'),
                '-y',
                LaunchConfiguration('y_pos'),
                '-z',
                LaunchConfiguration('z_pos'),
                '-R',
                LaunchConfiguration('roll'),
                '-P',
                LaunchConfiguration('pitch'),
                '-Y',
                LaunchConfiguration('yaw')
            ]
        ),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()