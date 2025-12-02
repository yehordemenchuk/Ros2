#!/usr/bin/env python3
"""
Launch file for movement node only
Useful for testing movement functionality independently
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    """
    Generate launch description for movement node only
    """
    
    return LaunchDescription([
        LogInfo(msg='Starting Movement Node...'),
        
        # Movement Node
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
                'use_tf': False,  # Set to True if using TF
                'robot_base_frame': 'base_link',
                'map_frame': 'map',
            }]
        ),
        
        LogInfo(msg='Movement Node started!'),
        LogInfo(msg='Subscribe to /robot/cmd_vel_direct for direct velocity commands'),
        LogInfo(msg='Subscribe to /robot/cmd_path for path following'),
    ])

