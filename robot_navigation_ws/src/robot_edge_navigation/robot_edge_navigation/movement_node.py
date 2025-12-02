#!/usr/bin/env python3
"""
Movement Node
ROS 2 Node that uses the MovementController module for robot movement
This can be used standalone or integrated with other nodes
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Path
from tf2_ros import TransformListener, Buffer
import math
from .movement_controller import MovementController

class MovementNode(Node):
    """
    ROS 2 Node for robot movement control
    Uses MovementController module for low-level control
    """
    
    def __init__(self):
        super().__init__('movement_node')
        
        # Declare parameters
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('max_linear_accel', 0.2)
        self.declare_parameter('max_angular_accel', 0.5)
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('use_tf', False)
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        
        # Get parameters
        max_linear_vel = self.get_parameter('max_linear_vel').value
        max_angular_vel = self.get_parameter('max_angular_vel').value
        max_linear_accel = self.get_parameter('max_linear_accel').value
        max_angular_accel = self.get_parameter('max_angular_accel').value
        control_freq = self.get_parameter('control_frequency').value
        use_tf = self.get_parameter('use_tf').value
        robot_frame = self.get_parameter('robot_base_frame').value
        map_frame = self.get_parameter('map_frame').value
        
        # Initialize movement controller
        self.movement_controller = MovementController(
            max_linear_vel=max_linear_vel,
            max_angular_vel=max_angular_vel,
            max_linear_accel=max_linear_accel,
            max_angular_accel=max_angular_accel,
            base_frame=robot_frame
        )
        
        # TF for getting robot pose (optional)
        self.use_tf = use_tf
        if self.use_tf:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.robot_frame = robot_frame
            self.map_frame = map_frame
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Path,
            '/robot/cmd_path',
            self.path_callback,
            10
        )
        
        self.direct_vel_sub = self.create_subscription(
            Twist,
            '/robot/cmd_vel_direct',
            self.direct_velocity_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Timer for control loop
        dt = 1.0 / control_freq
        self.control_timer = self.create_timer(dt, self.control_loop)
        
        # Current path to follow
        self.current_path = None
        self.current_path_index = 0
        self.current_robot_pose = None
        
        self.get_logger().info('Movement Node initialized')
        self.get_logger().info(f'Max velocities: linear={max_linear_vel} m/s, angular={max_angular_vel} rad/s')
    
    def path_callback(self, msg: Path):
        """Callback for receiving navigation path"""
        if len(msg.poses) > 0:
            self.current_path = msg
            self.current_path_index = 0
            self.get_logger().info(f'Received new path with {len(msg.poses)} waypoints')
        else:
            self.current_path = None
            self.movement_controller.stop()
    
    def direct_velocity_callback(self, msg: Twist):
        """Callback for direct velocity commands (bypasses path following)"""
        self.movement_controller.set_target_velocity(msg.linear.x, msg.angular.z)
        self.current_path = None  # Cancel path following
    
    def get_robot_pose(self) -> Optional[Pose]:
        """
        Get current robot pose
        Uses TF if available, otherwise returns None
        """
        if not self.use_tf:
            return None
        
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time()
            )
            
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            
            return pose
        except Exception as e:
            self.get_logger().debug(f'Could not get robot pose from TF: {str(e)}')
            return None
    
    def control_loop(self):
        """Main control loop"""
        # Update robot pose if using TF
        if self.use_tf:
            self.current_robot_pose = self.get_robot_pose()
        
        # Follow path if we have one
        if self.current_path and self.current_path_index < len(self.current_path.poses):
            waypoint = self.current_path.poses[self.current_path_index]
            
            # Compute velocities to waypoint
            if self.current_robot_pose is not None:
                # Use actual robot pose from TF
                linear_vel, angular_vel = self.movement_controller.compute_velocities_to_waypoint(
                    self.current_robot_pose,
                    waypoint
                )
            else:
                # Fallback: use simple proportional control
                # This assumes robot starts at origin
                goal_x = waypoint.pose.position.x
                goal_y = waypoint.pose.position.y
                
                # Simple distance-based control
                distance = math.sqrt(goal_x**2 + goal_y**2)
                angle = math.atan2(goal_y, goal_x)
                
                linear_vel = min(self.movement_controller.max_linear_vel, distance * 0.5)
                angular_vel = max(-self.movement_controller.max_angular_vel,
                                min(self.movement_controller.max_angular_vel, angle * 2.0))
                
                if distance < 0.1:
                    linear_vel = 0.0
                    angular_vel = 0.0
            
            self.movement_controller.set_target_velocity(linear_vel, angular_vel)
            
            # Check if waypoint reached
            if self.current_robot_pose is not None:
                robot_x = self.current_robot_pose.position.x
                robot_y = self.current_robot_pose.position.y
                goal_x = waypoint.pose.position.x
                goal_y = waypoint.pose.position.y
                distance = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
            else:
                # Fallback distance check
                goal_x = waypoint.pose.position.x
                goal_y = waypoint.pose.position.y
                distance = math.sqrt(goal_x**2 + goal_y**2)
            
            if distance < 0.2:  # Reached waypoint
                self.current_path_index += 1
                if self.current_path_index >= len(self.current_path.poses):
                    self.get_logger().info('Path completed')
                    self.current_path = None
                    self.movement_controller.stop()
        
        # Update movement controller and get velocity command
        cmd_vel = self.movement_controller.update()
        
        # Publish velocity command
        self.cmd_vel_pub.publish(cmd_vel)
    
    def emergency_stop(self):
        """Emergency stop the robot"""
        self.movement_controller.set_emergency_stop(True)
        self.get_logger().warn('Emergency stop activated!')

def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

