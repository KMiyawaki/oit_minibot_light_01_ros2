from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'oit_minibot_light_01_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miyawaki',
    maintainer_email='5770105+KMiyawaki@users.noreply.github.com',
    description='Minibot Light 01 ROS2 package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
