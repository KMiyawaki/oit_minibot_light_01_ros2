from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'oit_minibot_light_01_ros2'


def data_files_sub(dir):
    data_files = []
    for (path, _, files) in os.walk(dir):
        dst = os.path.join('share', package_name, path)
        if not dst:
            continue
        src = []
        for f in files:
            src.append(os.path.join(path, f))
        data_files.append((dst, src))
    return data_files


def data_files(dirs):
    data_files = [
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    for d in dirs:
        data_files = data_files + data_files_sub(d)
    return data_files


setup(
    name=package_name,
    version='0.9.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files(['config', 'launch', 'maps', 'urdf', 'rviz']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miyawaki',
    maintainer_email='5770105+KMiyawaki@users.noreply.github.com',
    description='Minibot Light 01 ROS2 package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'csi_camera = oit_minibot_light_01_ros2.csi_camera:main',
            'capture = oit_minibot_light_01_ros2.capture:main',
        ],
    },
)
