# Robot Navigation Workspace

ROS 2 Humble workspace for robot navigation with edge computing.

## Quick Start

### 1. Install ROS 2 Humble

Follow the official ROS 2 Humble installation guide: https://docs.ros.org/en/humble/Installation.html

### 2. Install Dependencies

```bash
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2-ros \
    ros-humble-cv-bridge \
    ros-humble-image-transport

pip3 install -r src/robot_edge_navigation/requirements.txt
```

### 3. Build the Workspace

```bash
cd robot_navigation_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Run the System

```bash
# Launch complete navigation system
ros2 launch robot_edge_navigation robot_navigation.launch.py

# Or launch only mapping
ros2 launch robot_edge_navigation mapping_only.launch.py
```

## Testing with Simulated Robot

If you want to test without a physical robot, you can use Gazebo:

```bash
# Install Gazebo
sudo apt install -y ros-humble-gazebo-ros-pkgs

# Launch robot simulation (example - adjust for your robot)
ros2 launch your_robot_description gazebo.launch.py
```

## Sending Navigation Goals

```bash
# Send a navigation goal (example)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

## Package Structure

- `robot_edge_navigation/` - Main ROS 2 package
  - `robot_edge_navigation/` - Python modules
  - `launch/` - Launch files
  - `config/` - Configuration files

## Documentation

See `src/robot_edge_navigation/README.md` for detailed package documentation.

