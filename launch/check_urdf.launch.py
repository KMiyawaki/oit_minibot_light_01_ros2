import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

pack_dir = get_package_share_directory('oit_minibot_light_01_ros2')


def get_robot_desc():
    xacro_path = os.path.join(pack_dir, 'urdf', 'minibot_light_01.xacro')
    doc = xacro.process_file(xacro_path)
    return doc.toprettyxml(indent='  ')


def generate_launch_description():
    rviz_conf = os.path.join(pack_dir, 'rviz', 'check_urdf.rviz')
    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               name='robot_state_publisher',
               output='both',
               parameters=[{'robot_description': get_robot_desc()}])
    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='both',
                arguments=['-d', rviz_conf])

    return LaunchDescription([rsp, rviz])
