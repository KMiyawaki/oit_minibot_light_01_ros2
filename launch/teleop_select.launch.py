import math
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

pack_dir = get_package_share_directory('oit_minibot_light_01_ros2')


def if_condition(conf, op, value):
    if type(value) == str:
        return IfCondition(PythonExpression(["'", conf, "'", op, "'", value, "'"]))
    else:
        return IfCondition(PythonExpression([conf, op, value]))


def generate_launch_description():
    joy_conf = os.path.join(pack_dir, 'config', 'joy.yaml')

    teleop_arg = DeclareLaunchArgument('teleop', default_value='joy',
                                       description='teleop device type', choices=['joy', 'key', 'mouse'])
    teleop_conf = LaunchConfiguration(teleop_arg.name)

    joy = GroupAction(actions=[
        Node(package='joy',
             executable='joy_node',
             name='joy_node',
             output='screen'),
        Node(package='teleop_twist_joy',
             executable='teleop_node',
             name='teleop_node',
             output='screen',
             parameters=[joy_conf])],
        condition=if_condition(teleop_conf, '==', 'joy'))

    key = Node(package='key_teleop',
               executable='key_teleop',
               name='key_teleop',
               output='screen',
               prefix='xterm -e',
               remappings=[('key_vel', 'cmd_vel')],
               parameters=[{'forward_rate': 0.2, 'backward_rate': 0.2,
                            'rotation_rate': math.radians(60.0)}],
               condition=if_condition(teleop_conf, '==', 'key'))

    mouse = Node(package='mouse_teleop',
                 executable='mouse_teleop',
                 name='mouse_teleop',
                 output='screen',
                 remappings=[('mouse_vel', 'cmd_vel')],
                 condition=if_condition(teleop_conf, '==', 'mouse'))

    return LaunchDescription([teleop_arg, joy, key, mouse])
