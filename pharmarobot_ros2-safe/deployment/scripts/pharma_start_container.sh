#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${PHARMA_CONTAINER:-pharma_container}"
IMAGE="${PHARMA_IMAGE:-pharmarobot:clean}"
WS_DIR="${PHARMA_WS_DIR:-/home/medrobots/pharmarobot/pharmarobot/pharmarobot_ros2-master}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ROS_LOCALHOST_ONLY_VALUE="${ROS_LOCALHOST_ONLY:-0}"

if [[ ! -d "$WS_DIR/src" ]]; then
  echo "[pharma-container] Workspace source not found: $WS_DIR/src" >&2
  exit 1
fi

if [[ ! "$ROS_DOMAIN_ID_VALUE" =~ ^[0-9]+$ ]] ||
   (( ROS_DOMAIN_ID_VALUE < 0 || ROS_DOMAIN_ID_VALUE > 232 )); then
  echo "[pharma-container] ERROR: ROS_DOMAIN_ID must be in 0..232" >&2
  exit 1
fi
if [[ "$RMW_IMPLEMENTATION_VALUE" != "rmw_fastrtps_cpp" ]] ||
   [[ "$ROS_LOCALHOST_ONLY_VALUE" != "0" ]]; then
  echo "[pharma-container] ERROR: unsupported production DDS settings" >&2
  exit 1
fi
ROBOTEQ_DEVICE="${ROBOTEQ_PORT:-/dev/roboteq}"
FRONT_DEVICE="${FRONT_PORT:-/dev/lidar_front}"
BACK_DEVICE="${BACK_PORT:-/dev/lidar_back}"

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

set +e
existing_inspect="$(docker inspect "$CONTAINER" 2>&1)"
inspect_status=$?
set -e
if [[ "$inspect_status" -ne 0 && "$inspect_status" -ne 1 ]]; then
  echo \
    "[pharma-container] ERROR: existing container identity could not be " \
    "verified; refusing replacement: $existing_inspect" >&2
  exit 75
fi
if [[ "$inspect_status" -eq 1 ]] &&
   [[ "$existing_inspect" != *"No such object"* &&
      "$existing_inspect" != *"No such container"* ]]; then
  echo \
    "[pharma-container] ERROR: ambiguous Docker inspect failure; refusing " \
    "replacement: $existing_inspect" >&2
  exit 75
fi
if [[ -n "$existing_inspect" ]] &&
   grep -Eq \
     'D455_IMU_AVAILABLE=|D455_IMU_|D455_SERIAL_NUMBER=|pharmarobot-d455-imu|/dev/bus/usb/|/dev/video|/dev/media|/dev/iio:device|HID-SENSOR|realsense2_camera|realsense_imu' \
     <<< "$existing_inspect"; then
  echo \
    "[pharma-container] ERROR: the existing main container has legacy D455 " \
    "access. It was not modified. Run " \
    "'pharma_d455_sensor_container.sh migration-check', record its exact " \
    "container ID, and obtain separate approval for an operator-performed " \
    "stop/removal before rerunning this launcher." >&2
  exit 78
fi

docker rm -f "$CONTAINER" >/dev/null 2>&1 || true

docker run -d \
  --name "$CONTAINER" \
  --network host \
  "${DEVICE_ARGS[@]}" \
  --mount type=bind,src=/dev/input,dst=/dev/input \
  --device-cgroup-rule 'c 13:* rwm' \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE" \
  -e ROS_LOCALHOST_ONLY="$ROS_LOCALHOST_ONLY_VALUE" \
  -e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION_VALUE" \
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

echo \
  "[pharma-container] Started $CONTAINER with " \
  "ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VALUE and no direct D455 hardware access"
