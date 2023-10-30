import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml, ReplaceString

pack_dir = get_package_share_directory('oit_minibot_light_01_ros2')


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False')
    use_sim_time = LaunchConfiguration(use_sim_time_arg.name)

    nav2_yaml = os.path.join(pack_dir, 'config', 'nav2.yaml')
    nav2_params = [ParameterFile(
        RewrittenYaml(
            source_file=nav2_yaml,
            root_key=None,
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True),
        allow_substs=True)]

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']
    navigation_nodes = GroupAction(
        actions=[
            Node(package='nav2_controller',
                 executable='controller_server',
                 name='controller_server',
                 output='screen',
                 respawn=True,
                 respawn_delay=2.0,
                 parameters=nav2_params,
                 remappings=[('cmd_vel', 'cmd_vel_nav')]),
            Node(package='nav2_smoother',
                 executable='smoother_server',
                 name='smoother_server',
                 output='screen',
                 respawn=True,
                 respawn_delay=2.0,
                 parameters=nav2_params),
            Node(package='nav2_planner',
                 executable='planner_server',
                 name='planner_server',
                 output='screen',
                 respawn=True,
                 respawn_delay=2.0,
                 parameters=nav2_params),
            Node(package='nav2_behaviors',
                 executable='behavior_server',
                 name='behavior_server',
                 output='screen',
                 respawn=True,
                 respawn_delay=2.0,
                 parameters=nav2_params),
            Node(package='nav2_bt_navigator',
                 executable='bt_navigator',
                 name='bt_navigator',
                 output='screen',
                 respawn=True,
                 respawn_delay=2.0,
                 parameters=nav2_params),
            Node(package='nav2_waypoint_follower',
                 executable='waypoint_follower',
                 name='waypoint_follower',
                 output='screen',
                 respawn=True,
                 respawn_delay=2.0,
                 parameters=nav2_params),
            Node(package='nav2_velocity_smoother',
                 executable='velocity_smoother',
                 name='velocity_smoother',
                 output='screen',
                 respawn=True,
                 respawn_delay=2.0,
                 parameters=nav2_params,
                 remappings=[('cmd_vel', 'cmd_vel_nav'),
                             ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(package='nav2_lifecycle_manager',
                 executable='lifecycle_manager',
                 name='lifecycle_manager_navigation',
                 output='screen',
                 parameters=[use_sim_time,
                             {'autostart': True},
                             {'node_names': lifecycle_nodes}]),
        ]
    )

    return LaunchDescription([use_sim_time_arg, navigation_nodes])
