import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    pack_dir = get_package_share_directory('oit_minibot_light_01_ros2')
    parameter_file = os.path.join(pack_dir, 'config', 'X4.yaml')

    ydlidar = LifecycleNode(package='ydlidar_ros2_driver',
                            executable='ydlidar_ros2_driver_node',
                            name='ydlidar_ros2_driver_node',
                            output='screen',
                            emulate_tty=True,
                            namespace='/',
                            parameters=[parameter_file])

    return LaunchDescription([ydlidar])
