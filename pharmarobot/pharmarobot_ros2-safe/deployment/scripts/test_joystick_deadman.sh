#!/usr/bin/env bash
set -eo pipefail

cd /ros_ws
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash

export ROS_DOMAIN_ID="${TEST_ROS_DOMAIN_ID:-92}"
export ROS_LOCALHOST_ONLY=1

LOG_FILE=/tmp/joystick_deadman_test_node.log

cleanup() {
  if [[ -n "${NODE_PID:-}" ]]; then
    kill -INT "$NODE_PID" >/dev/null 2>&1 || true
    wait "$NODE_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

ros2 run joy_to_cmdvel joy_to_cmd_vel_node \
  --ros-args \
  -p output_topic:=/test/cmd_vel/joy \
  -p enable_button_index:=4 \
  -p joy_timeout_s:=0.30 \
  >"$LOG_FILE" 2>&1 &
NODE_PID=$!

sleep 1
python3 /ros_ws/deployment/scripts/test_joystick_deadman.py

echo "Joy deadman node log: $LOG_FILE"
