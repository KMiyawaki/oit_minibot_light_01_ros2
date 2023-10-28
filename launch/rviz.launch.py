from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

pack_dir = get_package_share_directory('oit_minibot_light_01_ros2')


def concat(subs):
    arr = ["'", subs[0]]
    for i in range(1, len(subs)):
        arr = arr + ["' + '", subs[i]]
    arr.append("'")
    return arr


def rviz_conf_path(rviz_conf):
    path = PathJoinSubstitution([pack_dir, 'rviz', rviz_conf])
    return PythonExpression(concat([path, '.rviz']))


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument('rviz_conf', default_value='simple',
                                     description='rviz stting', choices=['simple', 'check_urdf', 'mapping', 'navigation'])
    rviz_conf = LaunchConfiguration(rviz_arg.name)

    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='both',
                arguments=['-d', rviz_conf_path(rviz_conf)])

    # LogInfo(msg=['rviz_conf_path=', rviz_conf_path(rviz_conf)])
    return LaunchDescription([rviz_arg, rviz])


if __name__ == '__main__':
    rviz_conf_path(LaunchConfiguration('hhoge'))
