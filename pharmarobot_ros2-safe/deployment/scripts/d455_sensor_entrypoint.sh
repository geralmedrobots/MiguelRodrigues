#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /sensor_ws/install/setup.bash
set -u

if [[ ! "${D455_SERIAL_NUMBER:-}" =~ ^[0-9]+$ ]]; then
  echo "[d455-sensor] invalid D455_SERIAL_NUMBER" >&2
  exit 64
fi
if [[ ! "${ROS_DOMAIN_ID:-}" =~ ^[0-9]+$ ]]; then
  echo "[d455-sensor] invalid ROS_DOMAIN_ID" >&2
  exit 64
fi
if [[ "${ROS_LOCALHOST_ONLY:-}" != "0" ]]; then
  echo "[d455-sensor] ROS_LOCALHOST_ONLY must be 0 for production DDS" >&2
  exit 64
fi
if [[ "${RMW_IMPLEMENTATION:-}" != "rmw_fastrtps_cpp" ]]; then
  echo "[d455-sensor] unsupported RMW_IMPLEMENTATION" >&2
  exit 64
fi
if [[ "${FASTDDS_BUILTIN_TRANSPORTS:-}" != "UDPv4" ]]; then
  echo \
    "[d455-sensor] FASTDDS_BUILTIN_TRANSPORTS must be UDPv4 for " \
    "production DDS" >&2
  exit 64
fi

exec ros2 launch realsense_imu robot_sensors.launch.py \
  serial_number:="${D455_SERIAL_NUMBER}"
