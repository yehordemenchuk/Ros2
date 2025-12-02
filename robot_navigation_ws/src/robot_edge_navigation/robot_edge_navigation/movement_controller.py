#!/usr/bin/env python3
"""
Movement Controller Module
This module handles low-level robot movement control including:
- Velocity control with acceleration limiting
- Waypoint following
- Movement commands (forward, backward, turn, stop)
- Safety checks
"""

from geometry_msgs.msg import Twist, PoseStamped, Pose
from typing import Tuple, Optional
import math
import time

class MovementController:
    """
    Low-level robot movement controller
    Handles velocity commands, acceleration limiting, and movement primitives
    """
    
    def __init__(
        self,
        max_linear_vel: float = 0.5,
        max_angular_vel: float = 1.0,
        max_linear_accel: float = 0.2,
        max_angular_accel: float = 0.5,
        base_frame: str = 'base_link'
    ):
        """
        Initialize movement controller
        
        Args:
            max_linear_vel: Maximum linear velocity (m/s)
            max_angular_vel: Maximum angular velocity (rad/s)
            max_linear_accel: Maximum linear acceleration (m/s²)
            max_angular_accel: Maximum angular acceleration (rad/s²)
            base_frame: Base frame of the robot
        """
        # Velocity limits
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        
        # Acceleration limits
        self.max_linear_accel = max_linear_accel
        self.max_angular_accel = max_angular_accel
        
        # Current state
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        # Target state
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        
        # Last update time
        self.last_update_time = time.time()
        
        # Robot parameters
        self.base_frame = base_frame
        self.wheel_base = 0.3  # Distance between wheels (meters)
        
        # Safety parameters
        self.emergency_stop = False
        self.min_safe_distance = 0.2  # Minimum safe distance to obstacle (meters)
    
    def set_emergency_stop(self, stop: bool):
        """Enable or disable emergency stop"""
        self.emergency_stop = stop
        if stop:
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
    
    def set_target_velocity(self, linear: float, angular: float):
        """
        Set target velocity
        
        Args:
            linear: Target linear velocity (m/s)
            angular: Target angular velocity (rad/s)
        """
        # Clip to maximum velocities
        self.target_linear_vel = max(-self.max_linear_vel, 
                                    min(self.max_linear_vel, linear))
        self.target_angular_vel = max(-self.max_angular_vel,
                                     min(self.max_angular_vel, angular))
    
    def stop(self):
        """Stop the robot immediately"""
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
    
    def move_forward(self, speed: float = None):
        """
        Move forward at specified speed
        
        Args:
            speed: Speed in m/s (defaults to max_linear_vel)
        """
        if speed is None:
            speed = self.max_linear_vel
        self.set_target_velocity(speed, 0.0)
    
    def move_backward(self, speed: float = None):
        """
        Move backward at specified speed
        
        Args:
            speed: Speed in m/s (defaults to max_linear_vel)
        """
        if speed is None:
            speed = self.max_linear_vel
        self.set_target_velocity(-speed, 0.0)
    
    def turn_left(self, angular_speed: float = None):
        """
        Turn left in place
        
        Args:
            angular_speed: Angular speed in rad/s (defaults to max_angular_vel)
        """
        if angular_speed is None:
            angular_speed = self.max_angular_vel
        self.set_target_velocity(0.0, angular_speed)
    
    def turn_right(self, angular_speed: float = None):
        """
        Turn right in place
        
        Args:
            angular_speed: Angular speed in rad/s (defaults to max_angular_vel)
        """
        if angular_speed is None:
            angular_speed = self.max_angular_vel
        self.set_target_velocity(0.0, -angular_speed)
    
    def move_with_rotation(self, linear: float, angular: float):
        """
        Move with both linear and angular velocity
        
        Args:
            linear: Linear velocity (m/s)
            angular: Angular velocity (rad/s)
        """
        self.set_target_velocity(linear, angular)
    
    def compute_velocities_to_waypoint(
        self,
        robot_pose: Pose,
        waypoint: PoseStamped,
        lookahead_distance: float = 0.3,
        k_linear: float = 1.0,
        k_angular: float = 2.0
    ) -> Tuple[float, float]:
        """
        Compute velocities to reach a waypoint using simple proportional control
        
        Args:
            robot_pose: Current robot pose
            waypoint: Target waypoint
            lookahead_distance: Lookahead distance for control (meters)
            k_linear: Linear velocity gain
            k_angular: Angular velocity gain
            
        Returns:
            Tuple of (linear_velocity, angular_velocity)
        """
        # Extract positions
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y
        
        # Extract robot orientation from quaternion
        robot_q = robot_pose.orientation
        robot_theta = self.quaternion_to_yaw(robot_q)
        
        waypoint_x = waypoint.pose.position.x
        waypoint_y = waypoint.pose.position.y
        
        # Calculate distance and angle to waypoint
        dx = waypoint_x - robot_x
        dy = waypoint_y - robot_y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_waypoint = math.atan2(dy, dx)
        
        # Calculate angle error
        angle_error = self.normalize_angle(angle_to_waypoint - robot_theta)
        
        # Simple proportional control
        # Linear velocity based on distance
        if distance < lookahead_distance:
            linear_vel = k_linear * distance
        else:
            linear_vel = k_linear * lookahead_distance
        
        # Reduce linear velocity when angle error is large
        linear_vel *= (1.0 - abs(angle_error) / math.pi)
        linear_vel = max(0.0, min(linear_vel, self.max_linear_vel))
        
        # Angular velocity based on angle error
        angular_vel = k_angular * angle_error
        angular_vel = max(-self.max_angular_vel, 
                         min(angular_vel, self.max_angular_vel))
        
        # Stop if very close to waypoint
        if distance < 0.05:
            linear_vel = 0.0
            angular_vel = 0.0
        
        return linear_vel, angular_vel
    
    def update(self, dt: float = None) -> Twist:
        """
        Update movement controller and return velocity command
        
        Args:
            dt: Time delta since last update (seconds). If None, computed automatically
            
        Returns:
            Twist message with velocity command
        """
        # Calculate time delta
        current_time = time.time()
        if dt is None:
            dt = current_time - self.last_update_time
            if dt <= 0:
                dt = 0.1  # Default to 10 Hz
        self.last_update_time = current_time
        
        # Emergency stop check
        if self.emergency_stop:
            self.current_linear_vel = 0.0
            self.current_angular_vel = 0.0
        else:
            # Apply acceleration limiting
            linear_diff = self.target_linear_vel - self.current_linear_vel
            angular_diff = self.target_angular_vel - self.current_angular_vel
            
            max_linear_change = self.max_linear_accel * dt
            max_angular_change = self.max_angular_accel * dt
            
            # Limit changes
            if abs(linear_diff) > max_linear_change:
                self.current_linear_vel += max_linear_change if linear_diff > 0 else -max_linear_change
            else:
                self.current_linear_vel = self.target_linear_vel
            
            if abs(angular_diff) > max_angular_change:
                self.current_angular_vel += max_angular_change if angular_diff > 0 else -max_angular_change
            else:
                self.current_angular_vel = self.target_angular_vel
        
        # Create Twist message
        cmd_vel = Twist()
        cmd_vel.linear.x = self.current_linear_vel
        cmd_vel.angular.z = self.current_angular_vel
        
        return cmd_vel
    
    def get_current_velocity(self) -> Tuple[float, float]:
        """
        Get current velocity
        
        Returns:
            Tuple of (linear_velocity, angular_velocity)
        """
        return (self.current_linear_vel, self.current_angular_vel)
    
    def is_stopped(self, threshold: float = 0.01) -> bool:
        """
        Check if robot is stopped
        
        Args:
            threshold: Velocity threshold below which robot is considered stopped
            
        Returns:
            True if robot is stopped
        """
        return (abs(self.current_linear_vel) < threshold and 
                abs(self.current_angular_vel) < threshold)
    
    @staticmethod
    def quaternion_to_yaw(quaternion) -> float:
        """
        Convert quaternion to yaw angle (rotation around z-axis)
        
        Args:
            quaternion: Quaternion message
            
        Returns:
            Yaw angle in radians
        """
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalize angle to [-pi, pi]
        
        Args:
            angle: Angle in radians
            
        Returns:
            Normalized angle in radians
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def compute_differential_drive_velocities(
        self,
        left_wheel_vel: float,
        right_wheel_vel: float
    ) -> Tuple[float, float]:
        """
        Convert wheel velocities to linear and angular velocities
        (for differential drive robots)
        
        Args:
            left_wheel_vel: Left wheel velocity (m/s)
            right_wheel_vel: Right wheel velocity (m/s)
            
        Returns:
            Tuple of (linear_velocity, angular_velocity)
        """
        linear_vel = (left_wheel_vel + right_wheel_vel) / 2.0
        angular_vel = (right_wheel_vel - left_wheel_vel) / self.wheel_base
        return linear_vel, angular_vel
    
    def compute_wheel_velocities(
        self,
        linear_vel: float,
        angular_vel: float
    ) -> Tuple[float, float]:
        """
        Convert linear and angular velocities to wheel velocities
        (for differential drive robots)
        
        Args:
            linear_vel: Linear velocity (m/s)
            angular_vel: Angular velocity (rad/s)
            
        Returns:
            Tuple of (left_wheel_velocity, right_wheel_velocity)
        """
        left_wheel_vel = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_wheel_vel = linear_vel + (angular_vel * self.wheel_base / 2.0)
        return left_wheel_vel, right_wheel_vel
    
    def set_parameters(
        self,
        max_linear_vel: float = None,
        max_angular_vel: float = None,
        max_linear_accel: float = None,
        max_angular_accel: float = None,
        wheel_base: float = None
    ):
        """
        Update controller parameters
        
        Args:
            max_linear_vel: Maximum linear velocity (m/s)
            max_angular_vel: Maximum angular velocity (rad/s)
            max_linear_accel: Maximum linear acceleration (m/s²)
            max_angular_accel: Maximum angular acceleration (rad/s²)
            wheel_base: Distance between wheels (meters)
        """
        if max_linear_vel is not None:
            self.max_linear_vel = max_linear_vel
        if max_angular_vel is not None:
            self.max_angular_vel = max_angular_vel
        if max_linear_accel is not None:
            self.max_linear_accel = max_linear_accel
        if max_angular_accel is not None:
            self.max_angular_accel = max_angular_accel
        if wheel_base is not None:
            self.wheel_base = wheel_base

