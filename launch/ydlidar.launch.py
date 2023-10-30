import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch.substitutions import LaunchConfiguration
import xacro

pack_dir = get_package_share_directory('oit_minibot_light_01_ros2')


def get_robot_desc():
    xacro_path = os.path.join(pack_dir, 'urdf', 'minibot_light_01.xacro')
    doc = xacro.process_file(xacro_path)
    return doc.toprettyxml(indent='  ')


def generate_launch_description():
    use_robot_state_publisher_arg = DeclareLaunchArgument('use_robot_state_publisher', default_value='false',
                                                          description='select to use robot_state_publisher', choices=['true', 'false'])
    use_robot_state_publisher = LaunchConfiguration(
        use_robot_state_publisher_arg.name)

    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false',
                                         description='select to use rviz', choices=['true', 'false'])
    use_rviz = LaunchConfiguration(use_rviz_arg.name)

    X4_yaml = os.path.join(pack_dir, 'config', 'X4.yaml')
    laser_filter_yaml = os.path.join(pack_dir, 'config', 'laser_filter.yaml')

    ydlidar = LifecycleNode(package='ydlidar_ros2_driver',
                            executable='ydlidar_ros2_driver_node',
                            name='ydlidar_ros2_driver_node',
                            output='screen',
                            emulate_tty=True,
                            namespace='/',
                            parameters=[X4_yaml],
                            remappings=[('scan', 'ydlidar_scan')])
    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        output='screen',
        parameters=[laser_filter_yaml],
        remappings=[('scan', 'ydlidar_scan'), ('scan_filtered', 'scan')]
    )

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               name='robot_state_publisher',
               output='both',
               parameters=[{'robot_description': get_robot_desc()}],
               condition=IfCondition(use_robot_state_publisher))

    rviz_launch = PythonLaunchDescriptionSource(
        os.path.join(pack_dir, 'launch', 'rviz.launch.py'))
    rviz = IncludeLaunchDescription(rviz_launch,
                                    condition=IfCondition(use_rviz),
                                    launch_arguments={'rviz_conf': 'simple'}.items())

    return LaunchDescription([use_rviz_arg, use_robot_state_publisher_arg, ydlidar, laser_filter, rsp, rviz])
