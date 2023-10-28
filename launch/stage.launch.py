from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

pack_dir = get_package_share_directory('oit_minibot_light_01_ros2')


def concat(subs):
    arr = ["'", subs[0]]
    for i in range(1, len(subs)):
        arr = arr + ["' + '", subs[i]]
    arr.append("'")
    return arr


def stage_world_path(world_conf):
    path = PathJoinSubstitution([pack_dir, 'maps', world_conf])
    return PythonExpression(concat([path, '.world']))


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='HRC',
        description='World file relative to the project world file, without .world')
    world_conf = LaunchConfiguration(world_arg.name)

    return LaunchDescription([
        world_arg,
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            parameters=[{"world_file": stage_world_path(world_conf)}]
        )
    ])
