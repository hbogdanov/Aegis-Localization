#!/usr/bin/env bash

# Source ROS2 setup and workspace overlay
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "$(pwd)/ros2_ws/install/setup.bash" ]; then
  source "$(pwd)/ros2_ws/install/setup.bash"
fi
