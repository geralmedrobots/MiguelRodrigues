#!/usr/bin/env bash
set -euo pipefail
CONTAINER="${PHARMA_CONTAINER:-pharma_container}"

docker exec "$CONTAINER" bash -lc '
  pkill -INT -f "[r]os2 launch teleop_pharma control_only.launch.py" || true
' || true
