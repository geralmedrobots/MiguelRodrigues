#!/usr/bin/env bash
# Resolve the minimal read-only TeleDex reference bind used by pharma_container.

configure_teledex_reference_mount() {
  local output_name="$1"
  local -n output_args="$output_name"
  local enabled="${TELEDEX_REFERENCE_ENABLED:-1}"
  local host_dir="${TELEDEX_REFERENCE_HOST_DIR:-/home/medrobots/teledex_reference}"
  local container_dir="${TELEDEX_REFERENCE_CONTAINER_DIR:-/opt/teledex_reference}"

  if [[ "$enabled" != "0" && "$enabled" != "1" ]]; then
    echo \
      "[pharma-container] ERROR: TELEDEX_REFERENCE_ENABLED must be 0 or 1" >&2
    return 64
  fi
  if [[ "$enabled" == "0" ]]; then
    echo "[pharma-container] TeleDex reference mount is disabled"
    return 0
  fi
  if [[ "$container_dir" != /* || "$container_dir" == "/" ]]; then
    echo \
      "[pharma-container] ERROR: TELEDEX_REFERENCE_CONTAINER_DIR must be an absolute non-root path" >&2
    return 64
  fi
  if [[ ! -d "$host_dir" ]]; then
    echo \
      "[pharma-container] ERROR: TeleDex host directory is missing: $host_dir" >&2
    return 66
  fi

  local host_logger="$host_dir/teledex_logger.py"
  if [[ ! -f "$host_logger" ]]; then
    echo \
      "[pharma-container] ERROR: TeleDex logger is missing: $host_logger" >&2
    return 66
  fi
  if [[ ! -r "$host_logger" ]]; then
    echo \
      "[pharma-container] ERROR: TeleDex logger is not readable: $host_logger" >&2
    return 77
  fi

  local resolved_logger
  resolved_logger="$(readlink -f "$host_logger")"
  output_args+=(
    --mount
    "type=bind,src=$resolved_logger,dst=$container_dir/teledex_logger.py,readonly"
  )
  echo \
    "[pharma-container] Mounting TeleDex logger read-only at $container_dir/teledex_logger.py"
}
