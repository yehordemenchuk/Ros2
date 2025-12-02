#!/bin/bash
# Build script for robot navigation workspace

set -e

echo "Building Robot Navigation Workspace..."

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS 2 is not sourced. Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install -q -r src/robot_edge_navigation/requirements.txt || echo "Warning: Some Python dependencies may need manual installation"

# Build the workspace
echo "Building workspace..."
colcon build --symlink-install --packages-select robot_edge_navigation

echo "Build complete!"
echo "To use the package, run: source install/setup.bash"

