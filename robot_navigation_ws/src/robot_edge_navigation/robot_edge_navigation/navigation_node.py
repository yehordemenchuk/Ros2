#!/usr/bin/env python3
"""
Navigation Node
This node plans paths using the generated map and sends commands to the robot controller
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
import numpy as np
import math
from typing import List, Tuple, Optional

class NavigationNode(Node):
    """
    ROS 2 Node for path planning and navigation
    Uses A* algorithm for path planning on the occupancy grid map
    """
    
    def __init__(self):
        super().__init__('navigation_node')
        
        # Map data
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        
        # Goal position
        self.current_goal = None
        
        # Path planning parameters
        self.inflation_radius = 0.3  # meters
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(
            Path,
            '/robot/cmd_path',
            10
        )
        
        self.get_logger().info('Navigation Node initialized')
    
    def map_callback(self, msg: OccupancyGrid):
        """Callback for map updates"""
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        
        self.get_logger().debug(f'Map updated: {self.map_width}x{self.map_height}')
        
        # If we have a goal, replan
        if self.current_goal:
            self.plan_path()
    
    def goal_callback(self, msg: PoseStamped):
        """Callback for navigation goal"""
        self.current_goal = msg
        self.get_logger().info(
            f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        
        if self.map_data is not None:
            self.plan_path()
        else:
            self.get_logger().warn('Map not available yet, will plan when map is received')
    
    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to map coordinates"""
        map_x = int((x - self.map_origin_x) / self.map_resolution)
        map_y = int((y - self.map_origin_y) / self.map_resolution)
        return map_x, map_y
    
    def map_to_world(self, map_x: int, map_y: int) -> Tuple[float, float]:
        """Convert map coordinates to world coordinates"""
        x = map_x * self.map_resolution + self.map_origin_x
        y = map_y * self.map_resolution + self.map_origin_y
        return x, y
    
    def is_valid_cell(self, map_x: int, map_y: int) -> bool:
        """Check if a map cell is valid and free"""
        if (map_x < 0 or map_x >= self.map_width or 
            map_y < 0 or map_y >= self.map_height):
            return False
        
        # Cell is free if occupancy value is low (< 50)
        return self.map_data[map_y, map_x] < 50
    
    def get_neighbors(self, map_x: int, map_y: int) -> List[Tuple[int, int]]:
        """Get valid neighboring cells (8-connected)"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                new_x, new_y = map_x + dx, map_y + dy
                if self.is_valid_cell(new_x, new_y):
                    neighbors.append((new_x, new_y))
        return neighbors
    
    def heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """Euclidean distance heuristic for A*"""
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    def a_star_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        A* path planning algorithm
        Returns list of (map_x, map_y) coordinates or None if no path found
        """
        start_x, start_y = start
        goal_x, goal_y = goal
        
        # Priority queue: (f_score, g_score, x, y, parent)
        open_set = [(0, 0, start_x, start_y, None)]
        closed_set = set()
        g_scores = {(start_x, start_y): 0}
        parents = {}
        
        while open_set:
            # Sort by f_score and get best node
            open_set.sort()
            f_score, g_score, current_x, current_y, parent = open_set.pop(0)
            
            if (current_x, current_y) in closed_set:
                continue
            
            closed_set.add((current_x, current_y))
            parents[(current_x, current_y)] = parent
            
            # Check if we reached the goal
            if current_x == goal_x and current_y == goal_y:
                # Reconstruct path
                path = []
                node = (current_x, current_y)
                while node is not None:
                    path.append(node)
                    node = parents.get(node)
                path.reverse()
                return path
            
            # Explore neighbors
            for neighbor_x, neighbor_y in self.get_neighbors(current_x, current_y):
                if (neighbor_x, neighbor_y) in closed_set:
                    continue
                
                # Cost to move to neighbor (diagonal costs more)
                dx = abs(neighbor_x - current_x)
                dy = abs(neighbor_y - current_y)
                move_cost = math.sqrt(2) if dx == 1 and dy == 1 else 1.0
                
                tentative_g = g_scores[(current_x, current_y)] + move_cost
                
                # Check if this path is better
                if (neighbor_x, neighbor_y) not in g_scores or \
                   tentative_g < g_scores[(neighbor_x, neighbor_y)]:
                    g_scores[(neighbor_x, neighbor_y)] = tentative_g
                    h = self.heuristic(neighbor_x, neighbor_y, goal_x, goal_y)
                    f = tentative_g + h
                    open_set.append((f, tentative_g, neighbor_x, neighbor_y, (current_x, current_y)))
        
        # No path found
        return None
    
    def plan_path(self):
        """Plan path from current robot pose to goal"""
        if self.map_data is None or self.current_goal is None:
            return
        
        # For now, assume robot starts at origin
        # In production, get actual pose from tf/odometry
        start_world = (0.0, 0.0)
        goal_world = (
            self.current_goal.pose.position.x,
            self.current_goal.pose.position.y
        )
        
        start_map = self.world_to_map(start_world[0], start_world[1])
        goal_map = self.world_to_map(goal_world[0], goal_world[1])
        
        # Check if start and goal are valid
        if not self.is_valid_cell(start_map[0], start_map[1]):
            self.get_logger().warn('Start position is in occupied cell')
            return
        
        if not self.is_valid_cell(goal_map[0], goal_map[1]):
            self.get_logger().warn('Goal position is in occupied cell')
            return
        
        # Plan path using A*
        path_map = self.a_star_path(start_map, goal_map)
        
        if path_map is None:
            self.get_logger().error('No path found to goal')
            return
        
        # Convert map path to world coordinates and create Path message
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        # Simplify path (remove unnecessary waypoints)
        simplified_path = self.simplify_path(path_map)
        
        for map_x, map_y in simplified_path:
            world_x, world_y = self.map_to_world(map_x, map_y)
            
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = world_x
            pose_stamped.pose.position.y = world_y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose_stamped)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Path planned with {len(path_msg.poses)} waypoints')
    
    def simplify_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """Simplify path by removing redundant waypoints"""
        if len(path) <= 2:
            return path
        
        simplified = [path[0]]
        
        for i in range(1, len(path) - 1):
            # Check if middle point is necessary
            prev = simplified[-1]
            curr = path[i]
            next_pt = path[i + 1]
            
            # If we can go directly from prev to next, skip curr
            # Simple check: if all cells in line are free
            if not self.is_line_free(prev, next_pt):
                simplified.append(curr)
        
        simplified.append(path[-1])
        return simplified
    
    def is_line_free(self, start: Tuple[int, int], end: Tuple[int, int]) -> bool:
        """Check if line between two points is free of obstacles"""
        x0, y0 = start
        x1, y1 = end
        
        # Use Bresenham's line algorithm to check all cells
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            if not self.is_valid_cell(x, y):
                return False
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return True

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

