#!/usr/bin/env python3
"""
Robot Controller Node
This node controls the robot's movement based on navigation commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from typing import Tuple
import math

class RobotControllerNode(Node):
    """
    ROS 2 Node for controlling robot movement
    Implements simple velocity control for differential drive robot
    """
    
    def __init__(self):
        super().__init__('robot_controller_node')
        
        # Robot parameters
        self.max_linear_vel = 0.5  # m/s
        self.max_angular_vel = 1.0  # rad/s
        self.linear_acceleration = 0.1  # m/s²
        self.angular_acceleration = 0.5  # rad/s²
        
        # Current velocities
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        # Target velocities
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        
        # Subscribers
        self.cmd_path_sub = self.create_subscription(
            Path,
            '/robot/cmd_path',
            self.path_callback,
            10
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        # Current path to follow
        self.current_path = None
        self.current_path_index = 0
        
        self.get_logger().info('Robot Controller Node initialized')
    
    def map_callback(self, msg: OccupancyGrid):
        """Callback for map updates"""
        # Store map for obstacle avoidance if needed
        pass
    
    def path_callback(self, msg: Path):
        """Callback for receiving navigation path"""
        if len(msg.poses) > 0:
            self.current_path = msg
            self.current_path_index = 0
            self.get_logger().info(f'Received new path with {len(msg.poses)} waypoints')
        else:
            self.current_path = None
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
    
    def calculate_velocities_to_waypoint(self, waypoint: PoseStamped) -> Tuple[float, float]:
        """
        Calculate required velocities to reach a waypoint
        Returns: (linear_vel, angular_vel)
        """
        # For now, simple proportional control
        # In production, use actual robot pose from odometry/tf
        
        # Placeholder: assume robot is at origin facing +x
        # Get waypoint position
        goal_x = waypoint.pose.position.x
        goal_y = waypoint.pose.position.y
        
        # Calculate angle to goal
        angle_to_goal = math.atan2(goal_y, goal_x)
        distance_to_goal = math.sqrt(goal_x**2 + goal_y**2)
        
        # Simple control law
        linear_vel = min(self.max_linear_vel, distance_to_goal * 0.5)
        angular_vel = math.clamp(angle_to_goal * 2.0, -self.max_angular_vel, self.max_angular_vel)
        
        # Stop if close to goal
        if distance_to_goal < 0.1:
            linear_vel = 0.0
            angular_vel = 0.0
        
        return linear_vel, angular_vel
    
    def control_loop(self):
        """
        Main control loop that updates robot velocities
        """
        # If we have a path to follow
        if self.current_path and self.current_path_index < len(self.current_path.poses):
            waypoint = self.current_path.poses[self.current_path_index]
            linear_vel, angular_vel = self.calculate_velocities_to_waypoint(waypoint)
            
            self.target_linear_vel = linear_vel
            self.target_angular_vel = angular_vel
            
            # Check if waypoint reached (simple distance check)
            goal_x = waypoint.pose.position.x
            goal_y = waypoint.pose.position.y
            distance = math.sqrt(goal_x**2 + goal_y**2)
            
            if distance < 0.2:  # Reached waypoint
                self.current_path_index += 1
                if self.current_path_index >= len(self.current_path.poses):
                    self.get_logger().info('Path completed')
                    self.current_path = None
                    self.target_linear_vel = 0.0
                    self.target_angular_vel = 0.0
        
        # Smoothly adjust velocities (acceleration limiting)
        linear_vel_diff = self.target_linear_vel - self.current_linear_vel
        angular_vel_diff = self.target_angular_vel - self.current_angular_vel
        
        dt = 0.1  # Timer period
        max_linear_change = self.linear_acceleration * dt
        max_angular_change = self.angular_acceleration * dt
        
        self.current_linear_vel += math.clamp(
            linear_vel_diff, 
            -max_linear_change, 
            max_linear_change
        )
        self.current_angular_vel += math.clamp(
            angular_vel_diff,
            -max_angular_change,
            max_angular_change
        )
        
        # Publish velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = self.current_linear_vel
        cmd_vel.angular.z = self.current_angular_vel
        
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

