# Robot Edge Navigation Package

ROS 2 Humble package for robot navigation with edge computing capabilities using LiDAR, camera, and neural network-based distance estimation.

## Overview

This package implements a complete robot navigation system that:
1. **Builds a map** of the environment using LiDAR and camera data
2. **Estimates distances** to objects using a neural network processing camera images
3. **Plans paths** using the generated map
4. **Controls robot movement** to follow planned paths

## Architecture

### Nodes

1. **distance_estimator_node** - Processes camera images to estimate distances to objects using neural network
2. **slam_mapper_node** - Builds occupancy grid map using LiDAR scans and camera distance estimates
3. **navigation_node** - Plans paths using A* algorithm on the occupancy grid map
4. **robot_controller_node** - Controls robot movement based on navigation commands

### Data Flow

```
Camera (image_raw) → distance_estimator_node → object_distances
LiDAR (scan) ────────────────────┐
                                 ↓
object_distances ────────────────┼──→ slam_mapper_node → map
                                 ↓
Odometry ────────────────────────┘

map → navigation_node → cmd_path → robot_controller_node → cmd_vel → Robot
```

## Installation

### Prerequisites

- ROS 2 Humble
- Python 3.8+
- NumPy
- OpenCV (cv_bridge)
- PyTorch (optional, for neural network)

### Build Instructions

```bash
# Navigate to your ROS 2 workspace
cd robot_navigation_ws

# Install dependencies (Ubuntu/Debian)
sudo apt update
sudo apt install -y \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    python3-numpy \
    python3-opencv

# Optional: Install PyTorch for neural network
pip3 install torch torchvision

# Build the package
colcon build --packages-select robot_edge_navigation
source install/setup.bash
```

## Usage

### Running the Complete System

```bash
ros2 launch robot_edge_navigation robot_navigation.launch.py
```

### Running Only Mapping

```bash
ros2 launch robot_edge_navigation mapping_only.launch.py
```

### Running Individual Nodes

```bash
# Terminal 1: Distance estimator
ros2 run robot_edge_navigation distance_estimator

# Terminal 2: SLAM mapper
ros2 run robot_edge_navigation slam_mapper

# Terminal 3: Navigation
ros2 run robot_edge_navigation navigation_node

# Terminal 4: Robot controller
ros2 run robot_edge_navigation robot_controller
```

## Topics

### Subscribed Topics

- `/camera/image_raw` (sensor_msgs/Image) - Camera images
- `/scan` (sensor_msgs/LaserScan) - LiDAR scan data
- `/goal_pose` (geometry_msgs/PoseStamped) - Navigation goal
- `/cmd_vel` (geometry_msgs/Twist) - Robot velocity commands (for odometry estimation)

### Published Topics

- `/robot/object_distances` (std_msgs/Float32MultiArray) - Distance estimates from neural network
- `/robot/processed_image` (sensor_msgs/Image) - Processed camera image with distance annotations
- `/map` (nav_msgs/OccupancyGrid) - Occupancy grid map
- `/robot/cmd_path` (nav_msgs/Path) - Planned navigation path
- `/cmd_vel` (geometry_msgs/Twist) - Robot velocity commands

## Configuration

Edit `config/default.yaml` to customize:
- Robot velocity limits
- Map parameters (size, resolution)
- Navigation parameters
- Neural network settings

## Neural Network

The package includes a placeholder neural network architecture for distance estimation. In production:

1. Train a custom model for your specific camera and environment
2. Save the trained model weights
3. Update `distance_estimator_node.py` to load your trained model
4. The current implementation uses a simple depth estimation as a placeholder

## Notes

- The current implementation uses simple odometry estimation from velocity commands. For production, integrate actual odometry data.
- The neural network uses a placeholder implementation. Replace with your trained model.
- Map building assumes robot starts at origin. Adjust if needed.
- Path planning uses A* algorithm on the occupancy grid.

## License

Apache-2.0

