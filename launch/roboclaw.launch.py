import math
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def calc_ticks_per_meter(encoder_cpr, gear_ratio, radius):
    output_cpr = encoder_cpr * gear_ratio
    return output_cpr / (2 * math.pi * radius)


def generate_launch_description():
    pack_dir = get_package_share_directory('oit_minibot_light_01_ros2')
    conf = os.path.join(pack_dir, 'config', 'roboclaw.yaml')

    return LaunchDescription([
        Node(
            package='oit_roboclaw_driver2',
            executable='oit_roboclaw_driver',
            output='screen',
            name='oit_roboclaw_driver',
            parameters=[conf]
        ),
        Node(
            package='oit_roboclaw_driver2',
            executable='oit_roboclaw_2wheels',
            output='screen',
            name='oit_roboclaw_2wheels',
            parameters=[{'process_rate': 20.0, 'tread': 0.18,
                         # 100:1 Micro Metal Gearmotor with 12 CPR Encoder
                         # https://www.pololu.com/product/3052/specs
                         # https://www.pololu.com/product/4760
                         'ticks_per_meter': calc_ticks_per_meter(12, 100.37, 0.03),
                         'linear_max': 0.5, 'angular_max': math.radians(90.0)}]
        )
    ])


if __name__ == '__main__':
    print(calc_ticks_per_meter(12, 100.37, 0.03))
