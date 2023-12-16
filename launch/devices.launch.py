import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

pack_dir = get_package_share_directory('oit_minibot_light_01_ros2')


def get_robot_desc():
    xacro_path = os.path.join(pack_dir, 'urdf', 'minibot_light_01.xacro')
    doc = xacro.process_file(xacro_path)
    return doc.toprettyxml(indent='  ')


def generate_launch_description():
    roboclaw = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pack_dir, 'launch', 'roboclaw.launch.py')))
    ydlidar = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pack_dir, 'launch', 'ydlidar.launch.py')))
    csi_camera = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pack_dir, 'launch', 'csi_camera.launch.py')))
    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               name='robot_state_publisher',
               output='both',
               parameters=[{'robot_description': get_robot_desc()}])

    return LaunchDescription([roboclaw, ydlidar, csi_camera, rsp])
