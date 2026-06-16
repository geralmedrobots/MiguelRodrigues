#!/usr/bin/env bash
set -euo pipefail
IMAGE="${PHARMA_IMAGE:-pharmarobot:clean}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
docker build -t "$IMAGE" "$ROOT_DIR"
