import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import xacro

pack_dir = get_package_share_directory('oit_minibot_light_01_ros2')


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen'
        )
    ])
