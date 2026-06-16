#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CONTAINER="${CONTAINER:-pharma_container}"

restore_ignores() {
  printf 'Optional test package: excluded from normal colcon builds.\n' > "$REPO_ROOT/src/odom_test_logger/COLCON_IGNORE"
  printf 'Optional test package: excluded from normal colcon builds.\n' > "$REPO_ROOT/src/robot_test_logger/COLCON_IGNORE"
}
trap restore_ignores EXIT

rm -f "$REPO_ROOT/src/odom_test_logger/COLCON_IGNORE"
rm -f "$REPO_ROOT/src/robot_test_logger/COLCON_IGNORE"

docker exec -it "$CONTAINER" bash -lc '
  cd /ros_ws
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --packages-select robot_test_logger odom_test_logger
'

echo "Optional test nodes built. They are still excluded from future normal builds."
