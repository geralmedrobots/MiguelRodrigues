#!/usr/bin/env bash
set -euo pipefail
CONTAINER="${PHARMA_CONTAINER:-pharma_container}"
FRONT_PORT="${FRONT_PORT:-/dev/lidar_front}"
BACK_PORT="${BACK_PORT:-/dev/lidar_back}"
BAUDRATE="${BAUDRATE:-460800}"

exec docker exec -i "$CONTAINER" bash -lc "
  source /opt/ros/humble/setup.bash
  source /ros_ws/install/setup.bash
  exec ros2 launch teleop_pharma lidar_only.launch.xml \\
    front_port:=$FRONT_PORT \\
    back_port:=$BACK_PORT \\
    baudrate:=$BAUDRATE
"
