# Integration Guide: Movement Module and Navigation Module

## Yes, They Work Together!

The movement module and navigation module are designed to work together seamlessly through ROS 2 topics. Here's how:

## System Architecture

```
┌─────────────────┐
│  Navigation     │
│     Node        │───► /robot/cmd_path (nav_msgs/Path)
└─────────────────┘              │
                                 │
                                 ▼
                         ┌───────────────┐
                         │  Movement     │
                         │     Node      │
                         └───────────────┘
                                 │
                                 ▼
                          /cmd_vel (geometry_msgs/Twist)
                                 │
                                 ▼
                         ┌───────────────┐
                         │     Robot     │
                         └───────────────┘
```

## How They Work Together

### 1. Navigation Node → Movement Node

**Navigation Node** (`navigation_node.py`):
- Plans paths using A* algorithm
- Publishes planned paths to `/robot/cmd_path` (nav_msgs/Path)
- Each path contains waypoints (PoseStamped messages)

**Movement Node** (`movement_node.py`):
- Subscribes to `/robot/cmd_path` 
- Receives paths from navigation node
- Uses `MovementController` to follow waypoints
- Publishes velocity commands to `/cmd_vel`

### 2. Topic Flow

```
1. User sends goal → /goal_pose (geometry_msgs/PoseStamped)
2. Navigation node plans path → /robot/cmd_path (nav_msgs/Path)
3. Movement node receives path → Follows waypoints
4. Movement node publishes → /cmd_vel (geometry_msgs/Twist)
5. Robot moves!
```

## Complete Workflow

### Step-by-Step Process:

1. **Mapping Phase** (optional):
   - SLAM mapper builds map using LiDAR + camera
   - Map published to `/map`

2. **Navigation Request**:
   ```bash
   ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
     "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}"
   ```

3. **Path Planning** (Navigation Node):
   - Receives goal from `/goal_pose`
   - Uses map from `/map` to plan path
   - Publishes path to `/robot/cmd_path`

4. **Path Execution** (Movement Node):
   - Receives path from `/robot/cmd_path`
   - Computes velocities to reach each waypoint
   - Publishes velocity commands to `/cmd_vel`
   - Robot follows path automatically

5. **Completion**:
   - Movement node reaches final waypoint
   - Robot stops
   - Ready for next goal

## Running the Integrated System

### Option 1: Launch Everything Together

```bash
ros2 launch robot_edge_navigation robot_navigation.launch.py
```

This launches:
- Distance estimator
- SLAM mapper
- Navigation node
- Movement node

### Option 2: Launch Components Separately

```bash
# Terminal 1: Mapping
ros2 launch robot_edge_navigation mapping_only.launch.py

# Terminal 2: Navigation + Movement
ros2 run robot_edge_navigation navigation_node
ros2 run robot_edge_navigation movement_node
```

## Topic Configuration

Both modules use standard ROS 2 topics for communication:

| Topic | Type | Publisher | Subscriber | Purpose |
|-------|------|-----------|------------|---------|
| `/goal_pose` | geometry_msgs/PoseStamped | User/App | Navigation Node | Navigation goal |
| `/map` | nav_msgs/OccupancyGrid | SLAM Mapper | Navigation Node | Environment map |
| `/robot/cmd_path` | nav_msgs/Path | Navigation Node | Movement Node | Planned path |
| `/cmd_vel` | geometry_msgs/Twist | Movement Node | Robot | Velocity commands |

## Direct Movement Control

You can also control the robot directly, bypassing navigation:

```bash
# Send direct velocity command (bypasses path following)
ros2 topic pub /robot/cmd_vel_direct geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

The movement node subscribes to `/robot/cmd_vel_direct` for direct control.

## Integration Points

### 1. Path Topic
- **Navigation node publishes**: `/robot/cmd_path`
- **Movement node subscribes**: `/robot/cmd_path`
- **Message type**: `nav_msgs/Path`
- **Contains**: List of waypoints (PoseStamped)

### 2. Coordinate Frames
Both modules use the same coordinate frames:
- `map` - Global map frame
- `base_link` - Robot base frame
- Movement node can optionally use TF to get robot pose

### 3. Parameters
Both can be configured independently:
- Navigation: Path planning parameters
- Movement: Velocity limits, acceleration limits

## Testing Integration

### Test 1: Full Navigation

```bash
# 1. Launch system
ros2 launch robot_edge_navigation robot_navigation.launch.py

# 2. Send goal (in another terminal)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}"

# 3. Monitor topics
ros2 topic echo /robot/cmd_path      # See planned path
ros2 topic echo /cmd_vel             # See velocity commands
```

### Test 2: Direct Movement Control

```bash
# Launch movement node only
ros2 launch robot_edge_navigation movement_only.launch.py

# Send direct velocity
ros2 topic pub --once /robot/cmd_vel_direct geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

## Benefits of This Architecture

1. **Modularity**: Each module can work independently
2. **Loose Coupling**: Communication through topics, no code dependencies
3. **Flexibility**: Can use navigation or direct control
4. **Testability**: Easy to test each module separately
5. **Extensibility**: Easy to add new features or replace modules

## Advanced Integration

### Using TF for Accurate Pose Tracking

Enable TF in movement node for better waypoint following:

```python
# In launch file or parameter
use_tf: true
robot_base_frame: 'base_link'
map_frame: 'map'
```

### Custom Control Algorithms

You can create custom control nodes that:
- Subscribe to `/robot/cmd_path`
- Implement custom control logic
- Publish to `/cmd_vel`

### Multiple Navigation Sources

You can have multiple nodes publish to `/robot/cmd_path`:
- Navigation node (A* path planning)
- Manual path generator
- Behavior tree planner
- etc.

Movement node will follow the most recent path received.

## Troubleshooting

### Issue: Robot not moving

**Check:**
1. Is navigation node publishing paths? → `ros2 topic echo /robot/cmd_path`
2. Is movement node receiving paths? → Check logs
3. Is movement node publishing velocities? → `ros2 topic echo /cmd_vel`
4. Is robot subscribed to `/cmd_vel`?

### Issue: Robot not following path accurately

**Solutions:**
1. Enable TF in movement node (`use_tf: true`)
2. Adjust control gains in `MovementController`
3. Check coordinate frame transforms
4. Verify robot pose is accurate

### Issue: Path planning fails

**Check:**
1. Is map available? → `ros2 topic echo /map`
2. Is goal valid (not in obstacle)?
3. Check navigation node logs for errors

## Summary

**Yes, the movement module and navigation module work together perfectly!**

- Navigation node plans paths and publishes to `/robot/cmd_path`
- Movement node subscribes to paths and executes them
- They communicate through ROS 2 topics (loosely coupled)
- Both can work independently or together
- The system is modular, flexible, and extensible

This architecture follows ROS 2 best practices for modular, maintainable robot systems.

