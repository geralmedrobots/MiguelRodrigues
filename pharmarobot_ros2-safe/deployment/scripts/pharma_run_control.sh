#!/usr/bin/env bash
set -euo pipefail
CONTAINER="${PHARMA_CONTAINER:-pharma_container}"

exec docker exec -i "$CONTAINER" bash -lc '
  source /opt/ros/humble/setup.bash
  source /ros_ws/install/setup.bash
  exec ros2 launch teleop_pharma control_only.launch.py
'
