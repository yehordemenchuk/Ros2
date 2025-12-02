# Robot Edge Navigation System - Overview

## System Architecture

This ROS 2 Humble package implements a complete robot navigation system with edge computing capabilities. The system combines LiDAR, camera, and neural network processing to build maps and navigate autonomously.

### Components

1. **Distance Estimator Node** (`distance_estimator_node.py`)
   - Subscribes to: `/camera/image_raw` (sensor_msgs/Image)
   - Publishes to: `/robot/object_distances` (std_msgs/Float32MultiArray)
   - Function: Uses neural network (or simple image processing) to estimate distances to objects in camera images
   - Edge computing: Performs neural network inference on-device

2. **SLAM Mapper Node** (`slam_mapper_node.py`)
   - Subscribes to:
     - `/scan` (sensor_msgs/LaserScan) - LiDAR data
     - `/robot/object_distances` (std_msgs/Float32MultiArray) - Camera distance estimates
     - `/cmd_vel` (geometry_msgs/Twist) - For odometry estimation
   - Publishes to: `/map` (nav_msgs/OccupancyGrid)
   - Function: Builds occupancy grid map by fusing LiDAR and camera distance data

3. **Navigation Node** (`navigation_node.py`)
   - Subscribes to:
     - `/map` (nav_msgs/OccupancyGrid) - Generated map
     - `/goal_pose` (geometry_msgs/PoseStamped) - Navigation goal
   - Publishes to: `/robot/cmd_path` (nav_msgs/Path)
   - Function: Plans paths using A* algorithm on the occupancy grid

4. **Robot Controller Node** (`robot_controller_node.py`)
   - Subscribes to:
     - `/robot/cmd_path` (nav_msgs/Path) - Planned path
     - `/map` (nav_msgs/OccupancyGrid) - Map for reference
   - Publishes to: `/cmd_vel` (geometry_msgs/Twist)
   - Function: Controls robot movement to follow planned paths

### Data Flow Diagram

```
┌─────────────┐
│   Camera    │──┐
└─────────────┘  │
                 ├──► distance_estimator_node ──► object_distances
┌─────────────┐  │                                      │
│    LiDAR    │──┼──────────────────────────────────────┼──► slam_mapper_node
└─────────────┘  │                                      │         │
                 │                                      │         ▼
┌─────────────┐  │                                      │       map
│  Odometry   │──┘                                      │         │
└─────────────┘                                         │         │
                                                        │         ▼
                                                        │  navigation_node
                                                        │         │
                                                        │         ▼
                                                        │    cmd_path
                                                        │         │
                                                        └─────────┼──► robot_controller_node
                                                                  │         │
                                                                  │         ▼
                                                                  │     cmd_vel
                                                                  │         │
                                                                  └─────────┘
```

## Workflow

### Phase 1: Map Building
1. Robot starts moving (manually or autonomously)
2. Camera captures images → Distance estimator processes images → Publishes distance estimates
3. LiDAR scans environment → Publishes scan data
4. SLAM mapper fuses LiDAR and camera data → Builds occupancy grid map
5. Map is continuously updated as robot moves

### Phase 2: Navigation
1. User sends navigation goal via `/goal_pose` topic
2. Navigation node receives goal and current map
3. Navigation node plans path using A* algorithm
4. Navigation node publishes planned path to `/robot/cmd_path`
5. Robot controller receives path and generates velocity commands
6. Robot follows path to goal

## Key Features

- **Edge Computing**: Neural network runs on-device for real-time distance estimation
- **Sensor Fusion**: Combines LiDAR and camera data for robust mapping
- **Real-time Mapping**: Continuous map updates as robot explores
- **Path Planning**: A* algorithm for optimal path planning
- **Obstacle Avoidance**: Map-based obstacle avoidance

## Neural Network Integration

The distance estimator includes a neural network architecture (`DistanceEstimationNet`) that can be:
1. **Trained offline** on your specific camera and environment
2. **Loaded at runtime** for real-time inference
3. **Currently uses placeholder** simple depth estimation (can be replaced with trained model)

### To Use a Trained Model:

1. Train your model and save weights (`.pth` file)
2. Update `distance_estimator_node.py`:
   ```python
   # In __init__ method, replace:
   self.model = DistanceEstimationNet().to(self.device)
   
   # With:
   self.model = DistanceEstimationNet().to(self.device)
   self.model.load_state_dict(torch.load('path/to/your/model.pth'))
   self.model.eval()
   ```
3. Update `estimate_depth_simple` to use the model:
   ```python
   # Preprocess image for neural network
   # Run inference
   # Postprocess results
   ```

## Topics Reference

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | Subscribed | Camera images |
| `/scan` | sensor_msgs/LaserScan | Subscribed | LiDAR scan data |
| `/robot/object_distances` | std_msgs/Float32MultiArray | Published | Distance estimates (50 values) |
| `/robot/processed_image` | sensor_msgs/Image | Published | Camera image with distance annotations |
| `/map` | nav_msgs/OccupancyGrid | Published/Subscribed | Occupancy grid map |
| `/goal_pose` | geometry_msgs/PoseStamped | Subscribed | Navigation goal |
| `/robot/cmd_path` | nav_msgs/Path | Published | Planned navigation path |
| `/cmd_vel` | geometry_msgs/Twist | Published/Subscribed | Robot velocity commands |

## Configuration

Edit `config/default.yaml` to customize:
- Robot velocity limits
- Map size and resolution
- Navigation parameters
- Neural network settings

## Testing

### 1. Test with Simulated Robot
Use Gazebo or another simulator with LiDAR and camera sensors.

### 2. Test Individual Nodes
```bash
# Terminal 1
ros2 run robot_edge_navigation distance_estimator

# Terminal 2
ros2 run robot_edge_navigation slam_mapper

# Terminal 3 - Publish test camera image
ros2 topic pub --once /camera/image_raw sensor_msgs/msg/Image "{...}"

# Terminal 4 - Publish test LiDAR scan
ros2 topic pub --once /scan sensor_msgs/msg/LaserScan "{...}"
```

### 3. Test Complete System
```bash
# Launch all nodes
ros2 launch robot_edge_navigation robot_navigation.launch.py

# Send navigation goal
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

## Notes

- **Odometry**: Current implementation uses velocity integration for pose estimation. For production, integrate actual odometry data from `/odom` topic or TF transforms.
- **Robot Pose**: Navigation assumes robot starts at origin. Update if your robot has a different starting pose.
- **Coordinate Frames**: Ensure proper TF transforms between `base_link`, `laser_frame`, `camera_frame`, and `map` frames.

## Future Improvements

- Integrate actual odometry/TF for better pose estimation
- Add loop closure detection for better SLAM
- Implement dynamic obstacle avoidance
- Add visualization tools (RViz2)
- Support for multiple robot types
- ROS 2 parameter server integration
- Action servers for navigation goals

