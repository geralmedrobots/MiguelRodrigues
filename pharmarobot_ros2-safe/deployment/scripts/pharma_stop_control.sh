#!/usr/bin/env bash
set -euo pipefail
CONTAINER="${PHARMA_CONTAINER:-pharma_container}"

docker exec "$CONTAINER" bash -lc '
  pkill -INT -f "[r]os2 launch teleop_pharma control_only.launch.py" || true
  pkill -INT -f "[c]ontrol_only.launch.py" || true
  pkill -TERM -f "[j]oy_linux_node" || true
  pkill -TERM -f "[j]oy_to_cmd_vel_node" || true
  pkill -TERM -f "[c]ommand_arbiter_node" || true
  pkill -TERM -f "[r]oboteq_ros2_driver_node" || true
  sleep 2
  pkill -KILL -f "[r]os2 launch teleop_pharma control_only.launch.py" || true
  pkill -KILL -f "[c]ontrol_only.launch.py" || true
  pkill -KILL -f "[j]oy_linux_node" || true
  pkill -KILL -f "[j]oy_to_cmd_vel_node" || true
  pkill -KILL -f "[c]ommand_arbiter_node" || true
  pkill -KILL -f "[r]oboteq_ros2_driver_node" || true
' || true
