#!/bin/bash
set -e

# Download ROS 2 source
wget -q https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos
vcs import src < ros2.repos

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -y \
    --skip-keys "rti-connext-dds-6.1.1 fastcdr urdfdom_headers python3-vcstool"

# Build ROS 2
colcon build --merge-install --symlink-install

# Source the setup file
source install/setup.bash

echo "ROS 2 build completed successfully!" 