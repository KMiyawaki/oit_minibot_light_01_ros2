
# https://github.com/jdgalviss/realsense_ros2/blob/979350f8b5c1c70bea1d54182f893e8be6bc5e17/realsense_ros2/launch/cartographer_launch.py

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

import os

pack_dir = get_package_share_directory('oit_minibot_light_01_ros2')


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', choices=['true', 'false'])
    use_sim_time = LaunchConfiguration(use_sim_time_arg.name)

    cartographer_node = Node(package='cartographer_ros',
                             executable='cartographer_node',
                             output='both',
                             parameters=[{'use_sim_time': use_sim_time}],
                             arguments=['-configuration_directory', os.path.join(pack_dir, 'config'),
                                        '-configuration_basename', 'cartographer.lua'])
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='both',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'])

    return LaunchDescription([cartographer_node, cartographer_occupancy_grid_node])
