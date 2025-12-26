#!/usr/bin/env bash

set -e

# Source ROS 2 Jazzy
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy not found at /opt/ros/jazzy"
  return 1
fi
source /opt/ros/jazzy/setup.bash

# Build workspace if needed (with symlinks for URDF/Xacro)
if [ ! -f install/setup.bash ]; then
  echo "install/ not found. Building workspace with symlink-install..."
  colcon build --symlink-install
fi

# Source workspace
source install/setup.bash

echo "ROS 2 Jazzy sourced"
echo "Workspace built (symlink-install enabled)"
