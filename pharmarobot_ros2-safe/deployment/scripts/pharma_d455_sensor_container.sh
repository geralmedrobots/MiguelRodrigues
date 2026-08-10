#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="${PHARMA_WS_DIR:-/home/medrobots/pharmarobot/pharmarobot/pharmarobot_ros2-master}"
TOOL="$ROOT_DIR/src/realsense_imu/tools/d455_production_container.py"

if [[ ! -f "$TOOL" ]]; then
  echo "[d455-sensor] production lifecycle tool is missing: $TOOL" >&2
  exit 66
fi

action="${1:-status}"
shift || true

exec env PYTHONPATH="$ROOT_DIR/src/realsense_imu" \
  python3 "$TOOL" "$action" "$@"
