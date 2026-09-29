#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=deployment/scripts/pharma_workspace_mount.sh
source "$SCRIPT_DIR/pharma_workspace_mount.sh"

ROOT_DIR=""
resolve_pharma_workspace_root ROOT_DIR
TOOL="$ROOT_DIR/src/realsense_imu/tools/d455_production_container.py"

if [[ ! -f "$TOOL" ]]; then
  echo "[d455-sensor] production lifecycle tool is missing: $TOOL" >&2
  exit 66
fi

action="${1:-status}"
shift || true

exec env PYTHONPATH="$ROOT_DIR/src/realsense_imu" \
  python3 "$TOOL" "$action" "$@"
