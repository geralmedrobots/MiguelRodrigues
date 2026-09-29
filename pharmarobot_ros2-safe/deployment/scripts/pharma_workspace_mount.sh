#!/usr/bin/env bash
# Resolve and validate the repository bind mounts for pharma_container.

resolve_pharma_workspace_root() {
  local root_name="$1"
  local -n output_root="$root_name"
  local requested_root

  if [[ -v PHARMA_WS_DIR ]]; then
    requested_root="$PHARMA_WS_DIR"
    if [[ -z "$requested_root" ]]; then
      echo "[pharma-container] ERROR: PHARMA_WS_DIR is explicitly empty" >&2
      return 64
    fi
  else
    local helper_dir
    helper_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    requested_root="$(cd "$helper_dir/../.." && pwd)"
  fi

  if [[ "$requested_root" != /* ]]; then
    echo "[pharma-container] ERROR: PHARMA_WS_DIR must be an absolute path" >&2
    return 64
  fi

  if [[ ! -d "$requested_root" ]]; then
    echo \
      "[pharma-container] ERROR: PHARMA_WS_DIR does not exist: $requested_root" >&2
    return 66
  fi

  output_root="$(readlink -f "$requested_root")"
  local required_directory
  for required_directory in src deployment; do
    if [[ ! -d "$output_root/$required_directory" ||
          ! -r "$output_root/$required_directory" ||
          ! -x "$output_root/$required_directory" ]]; then
      echo \
        "[pharma-container] ERROR: required workspace directory is unavailable: $output_root/$required_directory" >&2
      return 66
    fi
  done
}

configure_pharma_workspace_mounts() {
  local args_name="$1"
  local root_name="$2"
  local -n output_args="$args_name"
  local -n output_root="$root_name"

  resolve_pharma_workspace_root "$root_name" || return $?

  local required_file
  for required_file in \
      src/odometry_validation/package.xml \
      src/command_arbiter/package.xml \
      src/sllidar_ros2/package.xml; do
    if [[ ! -f "$output_root/$required_file" ||
          ! -r "$output_root/$required_file" ]]; then
      echo \
        "[pharma-container] ERROR: required package file is unavailable: $output_root/$required_file" >&2
      return 66
    fi
  done

  output_args+=(
    --mount
    "type=bind,src=$output_root/src,dst=/ros_ws/src"
    --mount
    "type=bind,src=$output_root/deployment,dst=/ros_ws/deployment,readonly"
  )
  echo "[pharma-container] Using PHARMA_WS_DIR=$output_root"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  workspace_args=()
  workspace_root=""
  configure_pharma_workspace_mounts workspace_args workspace_root
  printf '<%s>\n' "${workspace_args[@]}"
fi
