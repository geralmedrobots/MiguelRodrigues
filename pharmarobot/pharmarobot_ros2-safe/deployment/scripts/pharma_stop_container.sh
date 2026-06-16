#!/usr/bin/env bash
set -euo pipefail
CONTAINER="${PHARMA_CONTAINER:-pharma_container}"
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
