#!/usr/bin/env python3
"""
Launch file for the complete robot navigation system
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Generate launch description for robot navigation system
    """
    
    return LaunchDescription([
        LogInfo(msg='Starting Robot Edge Navigation System...'),
        
        # Distance Estimator Node (Neural Network)
        Node(
            package='robot_edge_navigation',
            executable='distance_estimator',
            name='distance_estimator_node',
            output='screen',
            parameters=[{
                'use_neural_network': True,
            }]
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
        
        # Navigation Node
        Node(
            package='robot_edge_navigation',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
        ),
        
        # Movement Node (uses MovementController module)
        Node(
            package='robot_edge_navigation',
            executable='movement_node',
            name='movement_node',
            output='screen',
            parameters=[{
                'max_linear_vel': 0.5,
                'max_angular_vel': 1.0,
                'max_linear_accel': 0.2,
                'max_angular_accel': 0.5,
                'control_frequency': 10.0,
                'use_tf': False,  # Set to True if using TF transforms
                'robot_base_frame': 'base_link',
                'map_frame': 'map',
            }]
        ),
        
        LogInfo(msg='Robot Navigation System started successfully!'),
    ])

