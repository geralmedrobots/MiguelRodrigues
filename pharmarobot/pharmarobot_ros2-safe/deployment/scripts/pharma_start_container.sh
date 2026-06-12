#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${PHARMA_CONTAINER:-pharma_container}"
IMAGE="${PHARMA_IMAGE:-pharmarobot:clean}"
WS_DIR="${PHARMA_WS_DIR:-/home/medrobots/pharmarobot/pharmarobot/pharmarobot_ros2-master}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"

if [[ ! -d "$WS_DIR/src" ]]; then
  echo "[pharma-container] Workspace source not found: $WS_DIR/src" >&2
  exit 1
fi

docker rm -f "$CONTAINER" >/dev/null 2>&1 || true

ROBOTEQ_DEVICE="${ROBOTEQ_PORT:-/dev/roboteq}"
FRONT_DEVICE="${FRONT_PORT:-/dev/lidar_front}"
BACK_DEVICE="${BACK_PORT:-/dev/lidar_back}"
JOYSTICK_DEVICE="${JOYSTICK_PORT:-/dev/input/js0}"

DEVICE_ARGS=()
map_device() {
  local host_name="$1"
  local container_name="$2"
  local required="$3"

  if [[ ! -e "$host_name" ]]; then
    if [[ "$required" == "required" ]]; then
      echo "[pharma-container] ERROR: required device is missing: $host_name" >&2
      exit 1
    fi
    echo "[pharma-container] WARNING: optional device is not available: $host_name"
    return
  fi

  local resolved
  resolved="$(readlink -f "$host_name")"
  DEVICE_ARGS+=(--device "$resolved:$container_name")
  echo "[pharma-container] Mapping $host_name ($resolved) as $container_name"
}

# Motor control must never start against an ambiguous ttyUSB number.
map_device "$ROBOTEQ_DEVICE" /dev/roboteq required
map_device "$FRONT_DEVICE" /dev/lidar_front optional
map_device "$BACK_DEVICE" /dev/lidar_back optional
map_device "$JOYSTICK_DEVICE" /dev/input/js0 optional

docker run -d \
  --name "$CONTAINER" \
  --network host \
  "${DEVICE_ARGS[@]}" \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE" \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v "$WS_DIR/src:/ros_ws/src" \
  -v "$WS_DIR/deployment:/ros_ws/deployment:ro" \
  "$IMAGE" \
  bash -lc 'source /opt/ros/humble/setup.bash && tail -f /dev/null'

echo "[pharma-container] Rebuilding the mounted core source..."
docker exec "$CONTAINER" bash -lc '
  cd /ros_ws
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install \
    --packages-up-to serial sllidar_ros2 joy_to_cmdvel command_arbiter roboteq_ros2_driver teleop_pharma
'

echo "[pharma-container] Started $CONTAINER with ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VALUE"
