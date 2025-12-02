from setuptools import setup
import os
from glob import glob

package_name = 'robot_edge_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, 
              package_name + '.neural_network',
              package_name + '.mapping',
              package_name + '.navigation'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Developer',
    maintainer_email='user@example.com',
    description='ROS 2 package for robot navigation with edge computing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'distance_estimator = robot_edge_navigation.distance_estimator_node:main',
            'slam_mapper = robot_edge_navigation.slam_mapper_node:main',
            'robot_controller = robot_edge_navigation.robot_controller_node:main',
            'navigation_node = robot_edge_navigation.navigation_node:main',
            'movement_node = robot_edge_navigation.movement_node:main',
        ],
    },
)

