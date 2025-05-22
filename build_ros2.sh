#!/bin/bash
set -e

# Create src directory if it doesn't exist
mkdir -p src

# Download ROS 2 source code
wget https://raw.githubusercontent.com/ros2/ros2/${ROS_DISTRO}/ros2.repos -O ros2.repos
vcs import src < ros2.repos

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
    --skip-keys "rti-connext-dds-6.0.1 rti-connext-dds-6.1.1 fastcdr urdfdom_headers python3-vcstool"

# Set Fast DDS as the default DDS implementation
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Build ROS 2
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
    -DCMAKE_CXX_FLAGS="-Wall -Wextra -Wpedantic" \
    -DCMAKE_C_FLAGS="-Wall -Wextra -Wpedantic"

# Source the setup file
source install/setup.bash

echo "ROS 2 build completed successfully!" 