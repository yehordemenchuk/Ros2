# Movement Module Documentation

## Overview

The movement module provides a dedicated, reusable module for robot movement control. It handles low-level velocity control, acceleration limiting, waypoint following, and movement primitives.

## Components

### 1. MovementController (`movement_controller.py`)

A standalone Python class that handles robot movement logic. It can be used independently or integrated into ROS 2 nodes.

**Key Features:**
- Velocity control with acceleration limiting
- Waypoint following
- Movement primitives (forward, backward, turn, stop)
- Emergency stop functionality
- Differential drive robot support
- Configurable parameters

### 2. MovementNode (`movement_node.py`)

A ROS 2 node that wraps the `MovementController` module for use in ROS 2 systems.

**Features:**
- Subscribes to `/robot/cmd_path` for path following
- Subscribes to `/robot/cmd_vel_direct` for direct velocity commands
- Publishes to `/cmd_vel` for robot control
- Optional TF integration for robot pose
- Configurable via ROS 2 parameters

## Usage

### Using MovementController Directly

```python
from robot_edge_navigation.movement_controller import MovementController

# Create controller
controller = MovementController(
    max_linear_vel=0.5,      # m/s
    max_angular_vel=1.0,     # rad/s
    max_linear_accel=0.2,    # m/s²
    max_angular_accel=0.5    # rad/s²
)

# Move forward
controller.move_forward(0.3)

# Turn left
controller.turn_left(0.5)

# Move with both linear and angular velocity
controller.move_with_rotation(0.4, 0.3)

# Stop
controller.stop()

# Update controller and get velocity command
cmd_vel = controller.update(dt=0.1)

# Waypoint following
robot_pose = Pose()  # Your robot pose
waypoint = PoseStamped()  # Target waypoint
linear, angular = controller.compute_velocities_to_waypoint(robot_pose, waypoint)
controller.set_target_velocity(linear, angular)
```

### Using MovementNode (ROS 2)

```bash
# Run the movement node
ros2 run robot_edge_navigation movement_node

# Or with parameters
ros2 run robot_edge_navigation movement_node \
  --ros-args \
  -p max_linear_vel:=0.8 \
  -p max_angular_vel:=1.5 \
  -p use_tf:=true
```

### Publishing Commands to MovementNode

```bash
# Direct velocity command
ros2 topic pub --once /robot/cmd_vel_direct geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Path following (published by navigation node)
ros2 topic pub --once /robot/cmd_path nav_msgs/msg/Path "{...}"
```

## API Reference

### MovementController Class

#### Initialization

```python
MovementController(
    max_linear_vel: float = 0.5,
    max_angular_vel: float = 1.0,
    max_linear_accel: float = 0.2,
    max_angular_accel: float = 0.5,
    base_frame: str = 'base_link'
)
```

#### Movement Methods

- `move_forward(speed: float = None)` - Move forward
- `move_backward(speed: float = None)` - Move backward
- `turn_left(angular_speed: float = None)` - Turn left in place
- `turn_right(angular_speed: float = None)` - Turn right in place
- `move_with_rotation(linear: float, angular: float)` - Move with both velocities
- `stop()` - Stop immediately
- `set_target_velocity(linear: float, angular: float)` - Set target velocities

#### Control Methods

- `update(dt: float = None) -> Twist` - Update controller and return velocity command
- `compute_velocities_to_waypoint(robot_pose, waypoint, ...) -> Tuple[float, float]` - Compute velocities to reach waypoint
- `get_current_velocity() -> Tuple[float, float]` - Get current velocity
- `is_stopped(threshold: float = 0.01) -> bool` - Check if stopped

#### Safety Methods

- `set_emergency_stop(stop: bool)` - Enable/disable emergency stop
- `set_parameters(...)` - Update controller parameters

#### Utility Methods

- `quaternion_to_yaw(quaternion) -> float` - Convert quaternion to yaw angle
- `normalize_angle(angle: float) -> float` - Normalize angle to [-π, π]
- `compute_differential_drive_velocities(left, right) -> Tuple[float, float]` - Convert wheel velocities
- `compute_wheel_velocities(linear, angular) -> Tuple[float, float]` - Convert to wheel velocities

## Parameters (MovementNode)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_linear_vel` | float | 0.5 | Maximum linear velocity (m/s) |
| `max_angular_vel` | float | 1.0 | Maximum angular velocity (rad/s) |
| `max_linear_accel` | float | 0.2 | Maximum linear acceleration (m/s²) |
| `max_angular_accel` | float | 0.5 | Maximum angular acceleration (rad/s²) |
| `control_frequency` | float | 10.0 | Control loop frequency (Hz) |
| `use_tf` | bool | false | Enable TF for robot pose |
| `robot_base_frame` | string | 'base_link' | Robot base frame |
| `map_frame` | string | 'map' | Map frame |

## Topics (MovementNode)

### Subscribed Topics

- `/robot/cmd_path` (nav_msgs/Path) - Path to follow
- `/robot/cmd_vel_direct` (geometry_msgs/Twist) - Direct velocity commands

### Published Topics

- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands to robot

## Integration Example

Here's how to use `MovementController` in your own ROS 2 node:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from robot_edge_navigation.movement_controller import MovementController

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        
        # Create movement controller
        self.movement_controller = MovementController(
            max_linear_vel=0.5,
            max_angular_vel=1.0
        )
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)
    
    def control_loop(self):
        # Use movement controller
        cmd_vel = self.movement_controller.update()
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Example: Move forward
        # self.movement_controller.move_forward(0.3)
        
        # Example: Follow waypoint
        # robot_pose = self.get_robot_pose()
        # waypoint = self.get_next_waypoint()
        # linear, angular = self.movement_controller.compute_velocities_to_waypoint(
        #     robot_pose, waypoint
        # )
        # self.movement_controller.set_target_velocity(linear, angular)
```

## Examples

### Example 1: Simple Movement

```python
controller = MovementController()

# Move forward for 2 seconds
import time
controller.move_forward(0.5)
start_time = time.time()
while time.time() - start_time < 2.0:
    cmd_vel = controller.update()
    # Publish cmd_vel to robot
    time.sleep(0.1)
controller.stop()
```

### Example 2: Waypoint Following

```python
controller = MovementController()

robot_pose = Pose()
robot_pose.position.x = 0.0
robot_pose.position.y = 0.0
robot_pose.orientation.w = 1.0

waypoint = PoseStamped()
waypoint.pose.position.x = 2.0
waypoint.pose.position.y = 1.0
waypoint.pose.orientation.w = 1.0

# Compute velocities to waypoint
linear, angular = controller.compute_velocities_to_waypoint(robot_pose, waypoint)
controller.set_target_velocity(linear, angular)

# Update controller
cmd_vel = controller.update()
```

### Example 3: Emergency Stop

```python
controller = MovementController()
controller.move_forward(0.5)

# Emergency stop
controller.set_emergency_stop(True)

# Later, resume
controller.set_emergency_stop(False)
```

## Notes

- The `MovementController` is thread-safe for basic operations, but for concurrent access, consider adding locks
- Acceleration limiting ensures smooth velocity changes
- Waypoint following uses proportional control - can be extended with PID control for better accuracy
- TF integration is optional - if disabled, the node uses simple distance-based control

## Future Enhancements

- PID controller for waypoint following
- Support for different robot types (holonomic, omnidirectional)
- Obstacle avoidance integration
- Trajectory smoothing
- Predictive control

