import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name='x_pos', default_value='-3.5'),
            launch.actions.DeclareLaunchArgument(
                name='y_pos', default_value='5.5'),
            launch.actions.DeclareLaunchArgument(
                name='z_pos', default_value='0.30'),
            launch.actions.DeclareLaunchArgument(
                name='gui', default_value='false'),
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(
                            'gazebo_dance_world'),
                        'launch',
                        'dance.launch.py',
                    )
                ),
                launch_arguments={
                    'gui': launch.substitutions.LaunchConfiguration('gui')
                }.items(),
            ),
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(
                            'jetbot_description'
                        ),
                        'launch',
                        'spawn.launch.py',
                    )
                ),
                launch_arguments={
                    'x_pos': launch.substitutions.LaunchConfiguration('x_pos'),
                    'y_pos': launch.substitutions.LaunchConfiguration('y_pos'),
                    'z_pos': launch.substitutions.LaunchConfiguration('z_pos'),
                }.items(),
            ),
        ]
    )
    return ld


if __name__ == '__main__':
    generate_launch_description()
