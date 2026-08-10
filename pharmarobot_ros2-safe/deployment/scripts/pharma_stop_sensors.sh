#!/usr/bin/env bash
set -euo pipefail

echo \
  "[pharma-sensors] deprecated compatibility wrapper; " \
  "stopping the production D455 sensor container" >&2
exec /usr/local/bin/pharma_d455_sensor_container.sh stop "$@"
