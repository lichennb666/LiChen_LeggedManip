#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
if [ -f /workspace/LeggedManip_Lab/ros2_ws/install/setup.bash ]; then
  source /workspace/LeggedManip_Lab/ros2_ws/install/setup.bash
fi
exec "$@"
