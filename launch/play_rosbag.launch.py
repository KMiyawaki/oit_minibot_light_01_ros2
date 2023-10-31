import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

pack_dir = get_package_share_directory('oit_minibot_light_01_ros2')
ws_dir = pack_dir.replace('install/oit_minibot_light_01_ros2/share', 'src')
slam_toolbox_dir = get_package_share_directory('slam_toolbox')


def generate_launch_description():
    bag_arg = DeclareLaunchArgument(
        'bag', default_value='rosbag2_2023_10_30-16_34_39')
    bag = LaunchConfiguration(bag_arg.name)
    bag_path = LaunchConfiguration(
        bag_arg.name + '_path', default=[ws_dir, '/bags/', bag])

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true')
    use_rviz = LaunchConfiguration(use_rviz_arg.name)

    slam_arg = DeclareLaunchArgument(
        'slam', default_value='true')
    slam = LaunchConfiguration(slam_arg.name)

    rviz_conf = LaunchConfiguration('rviz_conf', default=PythonExpression(
        ["'simple'", " if '", slam, "' == 'false' else 'mapping'"]))

    slam_toolbox_offline_yaml = os.path.join(
        pack_dir, 'config', 'slam_toolbox_offline.yaml')

    slam_offline = Node(
        parameters=[slam_toolbox_offline_yaml],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(slam)
    )

    rviz = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pack_dir, 'launch', 'rviz.launch.py')),
        launch_arguments={'rviz_conf': rviz_conf,
                          'use_sim_time': 'false'}.items(),
        condition=IfCondition(use_rviz))

    rviz_mapping = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pack_dir, 'launch', 'rviz.launch.py')),
        launch_arguments={'rviz_conf': 'mapping',
                          'use_sim_time': 'false'}.items(),
        condition=IfCondition(PythonExpression(["'", slam, "' == 'true' and '", use_rviz, "' == 'true'"])))

    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path],
        output='screen'
    )

    return LaunchDescription([bag_arg, use_rviz_arg, slam_arg, slam_offline, rviz, bag_play])
