import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

pack_dir = get_package_share_directory('oit_minibot_light_01_ros2')

def generate_launch_description():
    rviz_conf_arg = DeclareLaunchArgument('rviz_conf', default_value='none',
                                         description='select to use rviz', choices=['simple', 'none'])
    rviz_conf = LaunchConfiguration(rviz_conf_arg.name)

    csi_camera = Node(package='oit_minibot_light_01_ros2',
                            executable='csi_camera',
                            name='csi_camera',
                            output='screen')

    rviz_launch = PythonLaunchDescriptionSource(
        os.path.join(pack_dir, 'launch', 'rviz.launch.py'))
    rviz = IncludeLaunchDescription(rviz_launch,
                                    launch_arguments={'rviz_conf': rviz_conf}.items())

    return LaunchDescription([rviz_conf_arg, csi_camera, rviz])
