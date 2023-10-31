import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

pack_dir = get_package_share_directory('oit_minibot_light_01_ros2')


def generate_launch_description():
    teleop_arg = DeclareLaunchArgument('teleop', default_value='key',
                                       description='teleop device type', choices=['joy', 'key', 'mouse', 'none'])
    teleop = LaunchConfiguration(teleop_arg.name)

    devices = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pack_dir, 'launch', 'devices.launch.py')))
    teleop_select = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pack_dir, 'launch', 'teleop_select.launch.py')),
        launch_arguments={'teleop': teleop}.items())
    rviz = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pack_dir, 'launch', 'rviz.launch.py')),
        launch_arguments={'rviz_conf': 'simple', 'use_sim_time': 'false'}.items())
    timer = TimerAction(period=7.5, actions=[rviz])

    return LaunchDescription([teleop_arg, devices, teleop_select, timer])
