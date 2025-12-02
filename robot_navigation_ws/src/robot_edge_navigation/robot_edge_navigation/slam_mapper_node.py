#!/usr/bin/env python3
"""
SLAM Mapper Node
This node builds a map of the environment using LiDAR data and distance estimates
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Twist
import numpy as np
from collections import deque
import math

class SLAMMapperNode(Node):
    """
    ROS 2 Node for SLAM mapping using LiDAR and camera-based distance estimates
    """
    
    def __init__(self):
        super().__init__('slam_mapper_node')
        
        # Map parameters
        self.map_width = 400  # cells
        self.map_height = 400  # cells
        self.map_resolution = 0.05  # meters per cell (5cm)
        self.map_origin_x = -10.0  # meters
        self.map_origin_y = -10.0  # meters
        
        # Initialize occupancy grid
        self.occupancy_grid = np.ones((self.map_height, self.map_width), 
                                     dtype=np.int8) * -1  # Unknown = -1
        
        # Robot pose (x, y, theta)
        self.robot_pose = [0.0, 0.0, 0.0]  # [x, y, theta]
        
        # Latest sensor data
        self.latest_lidar = None
        self.latest_camera_distances = None
        
        # Map update history for probabilistic mapping
        self.probability_map = np.ones((self.map_height, self.map_width), 
                                      dtype=np.float32) * 0.5  # Unknown = 0.5
        
        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        self.distance_sub = self.create_subscription(
            Float32MultiArray,
            '/robot/object_distances',
            self.distance_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            10
        )
        
        self.map_metadata_pub = self.create_publisher(
            MapMetaData,
            '/map_metadata',
            10
        )
        
        # Timer for periodic map publishing
        self.map_timer = self.create_timer(0.5, self.publish_map)  # 2 Hz
        
        self.get_logger().info('SLAM Mapper Node initialized')
        self.get_logger().info(f'Map size: {self.map_width}x{self.map_height} cells, '
                              f'Resolution: {self.map_resolution}m/cell')
    
    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to map cell coordinates
        """
        map_x = int((x - self.map_origin_x) / self.map_resolution)
        map_y = int((y - self.map_origin_y) / self.map_resolution)
        return map_x, map_y
    
    def map_to_world(self, map_x: int, map_y: int) -> Tuple[float, float]:
        """
        Convert map cell coordinates to world coordinates
        """
        x = map_x * self.map_resolution + self.map_origin_x
        y = map_y * self.map_resolution + self.map_origin_y
        return x, y
    
    def update_map_with_lidar(self, scan: LaserScan):
        """
        Update occupancy grid using LiDAR scan data
        """
        if scan is None:
            return
        
        robot_x, robot_y, robot_theta = self.robot_pose
        map_x, map_y = self.world_to_map(robot_x, robot_y)
        
        # Process each LiDAR ray
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        
        for i, range_val in enumerate(scan.ranges):
            if range_val < scan.range_min or range_val > scan.range_max:
                continue
            
            # Calculate angle of this ray
            angle = angle_min + i * angle_increment + robot_theta
            
            # Calculate endpoint in world coordinates
            end_x = robot_x + range_val * math.cos(angle)
            end_y = robot_y + range_val * math.sin(angle)
            
            # Convert to map coordinates
            end_map_x, end_map_y = self.world_to_map(end_x, end_y)
            
            # Mark occupied cell (obstacle)
            if (0 <= end_map_x < self.map_width and 
                0 <= end_map_y < self.map_height):
                self.occupancy_grid[end_map_y, end_map_x] = 100
                self.probability_map[end_map_y, end_map_x] = 0.9  # High probability of obstacle
            
            # Mark free cells along the ray (bresenham line)
            self.mark_free_cells(map_x, map_y, end_map_x, end_map_y)
    
    def mark_free_cells(self, x0: int, y0: int, x1: int, y1: int):
        """
        Mark cells as free along a line using Bresenham's algorithm
        """
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            if (0 <= x < self.map_width and 0 <= y < self.map_height):
                # Don't mark the endpoint as free (it's occupied)
                if x != x1 or y != y1:
                    if self.occupancy_grid[y, x] == -1:
                        self.occupancy_grid[y, x] = 0
                    self.probability_map[y, x] = 0.1  # Low probability of obstacle
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def update_map_with_camera_distances(self, distances: np.ndarray):
        """
        Update map using camera-based distance estimates
        This integrates neural network distance estimates
        """
        if distances is None or len(distances) == 0:
            return
        
        robot_x, robot_y, robot_theta = self.robot_pose
        
        # Camera is assumed to be facing forward (along robot's x-axis)
        # Process distance estimates from camera
        for i, distance in enumerate(distances):
            # Spread estimates in a cone in front of the robot
            angle_offset = (i - len(distances) / 2) * 0.1  # Spread over ~1 radian
            angle = robot_theta + angle_offset
            
            # Calculate obstacle position
            end_x = robot_x + distance * math.cos(angle)
            end_y = robot_y + distance * math.sin(angle)
            
            # Update map
            end_map_x, end_map_y = self.world_to_map(end_x, end_y)
            if (0 <= end_map_x < self.map_width and 
                0 <= end_map_y < self.map_height):
                # Blend with existing probability
                self.probability_map[end_map_y, end_map_x] = min(
                    0.95, 
                    self.probability_map[end_map_y, end_map_x] * 1.1
                )
                if self.probability_map[end_map_y, end_map_x] > 0.7:
                    self.occupancy_grid[end_map_y, end_map_x] = 100
    
    def lidar_callback(self, msg: LaserScan):
        """Callback for LiDAR scan data"""
        self.latest_lidar = msg
        self.update_map_with_lidar(msg)
    
    def distance_callback(self, msg: Float32MultiArray):
        """Callback for camera distance estimates"""
        if len(msg.data) > 0:
            self.latest_camera_distances = np.array(msg.data)
            self.update_map_with_camera_distances(self.latest_camera_distances)
    
    def odom_callback(self, msg: Twist):
        """
        Estimate robot pose from velocity commands
        In production, use actual odometry data
        """
        # Simple integration (not accurate, but useful for testing)
        dt = 0.1  # Approximate time step
        self.robot_pose[0] += msg.linear.x * math.cos(self.robot_pose[2]) * dt
        self.robot_pose[1] += msg.linear.x * math.sin(self.robot_pose[2]) * dt
        self.robot_pose[2] += msg.angular.z * dt
    
    def publish_map(self):
        """Publish the occupancy grid map"""
        # Create OccupancyGrid message
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.orientation.w = 1.0
        
        # Flatten the occupancy grid (row-major order)
        map_msg.data = self.occupancy_grid.flatten().tolist()
        
        self.map_pub.publish(map_msg)
        
        self.get_logger().debug('Map published')

def main(args=None):
    rclpy.init(args=args)
    node = SLAMMapperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

