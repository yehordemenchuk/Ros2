#!/usr/bin/env python3
"""
Launch file for mapping only (building map without navigation)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    """
    Generate launch description for mapping system only
    """
    
    return LaunchDescription([
        LogInfo(msg='Starting Mapping System...'),
        
        # Distance Estimator Node (Neural Network)
        Node(
            package='robot_edge_navigation',
            executable='distance_estimator',
            name='distance_estimator_node',
            output='screen',
        ),
        
        # SLAM Mapper Node
        Node(
            package='robot_edge_navigation',
            executable='slam_mapper',
            name='slam_mapper_node',
            output='screen',
            parameters=[{
                'map_width': 400,
                'map_height': 400,
                'map_resolution': 0.05,
            }]
        ),
        
        LogInfo(msg='Mapping System started!'),
    ])

