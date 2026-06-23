#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[pharma] Deprecated wrapper: use pharma-minimal-nodes.service or pharma_run_control.sh."
echo "[pharma] Delegating to the authoritative control launcher."

exec "$SCRIPT_DIR/pharma_run_control.sh"
