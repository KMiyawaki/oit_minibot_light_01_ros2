import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

pack_dir = get_package_share_directory('oit_minibot_light_01_ros2')
slam_toolbox_dir = get_package_share_directory('slam_toolbox')


def generate_launch_description():
    teleop_arg = DeclareLaunchArgument('teleop', default_value='key',
                                       description='teleop device type', choices=['joy', 'key', 'mouse'])
    teleop_conf = LaunchConfiguration(teleop_arg.name)

    slam_launch = os.path.join(
        slam_toolbox_dir, 'launch', 'online_async_launch.py')
    slam_conf = os.path.join(pack_dir, 'config', 'slam_toolbox_online.yaml')

    stage = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pack_dir, 'launch', 'stage.launch.py')))
    rviz = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pack_dir, 'launch', 'rviz.launch.py')), launch_arguments={'rviz_conf': 'mapping'}.items())
    teleop_select = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pack_dir, 'launch', 'teleop_select.launch.py')),
        launch_arguments={'teleop': teleop_conf}.items())
    slam = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        slam_launch), launch_arguments={'use_sim_time': 'true', 'slam_params_file': slam_conf}.items())

    return LaunchDescription([stage, rviz, teleop_arg, teleop_select, slam])
