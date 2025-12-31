#!/bin/bash
set -e

# Setup ROS humble environment
source "/opt/ros/humble/setup.bash"
source "/ros2_ws/install/setup.bash"

exec "$@"
