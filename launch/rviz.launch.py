from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

pack_dir = get_package_share_directory('oit_minibot_light_01_ros2')


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument('rviz_conf', default_value='simple',
                                     description='rviz setting', choices=['simple', 'check_urdf', 'mapping', 'navigation'])
    rviz_conf = LaunchConfiguration(rviz_arg.name)
    rviz_conf_path = LaunchConfiguration(
        rviz_arg.name + '_path', default=[pack_dir, '/rviz/', rviz_conf, '.rviz'])

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', choices=['true', 'false'])
    use_sim_time_conf = LaunchConfiguration(use_sim_time_arg.name)

    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time_conf}],
                arguments=['-d', rviz_conf_path])

    # LogInfo(msg=['rviz_path_conf=', rviz_conf_path(rviz_path_conf)])
    return LaunchDescription([rviz_arg, use_sim_time_arg, rviz])
