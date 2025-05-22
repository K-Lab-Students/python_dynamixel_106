#!/bin/bash
set -e

# Create src directory if it doesn't exist
mkdir -p src

# Download ROS 2 source code
wget https://raw.githubusercontent.com/ros2/ros2/${ROS_DISTRO}/ros2.repos -O ros2.repos
vcs import src < ros2.repos

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO}

# Build ROS 2
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the setup file
source install/setup.bash

echo "ROS 2 build completed successfully!" 