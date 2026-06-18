#!/usr/bin/env bash
set -eo pipefail

cd /ros_ws
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash

# Isolate this software-only test from the live robot ROS graph.
export ROS_DOMAIN_ID="${TEST_ROS_DOMAIN_ID:-91}"
export ROS_LOCALHOST_ONLY=1

LOG_FILE=/tmp/command_arbiter_test_node.log

cleanup() {
  if [[ -n "${ARBITER_PID:-}" ]]; then
    kill -INT "$ARBITER_PID" >/dev/null 2>&1 || true
    wait "$ARBITER_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

ros2 run command_arbiter command_arbiter_node \
  --ros-args \
  -p joy_timeout_s:=0.25 \
  -p test_timeout_s:=0.25 \
  -p navigation_timeout_s:=0.25 \
  >"$LOG_FILE" 2>&1 &
ARBITER_PID=$!

sleep 1
python3 /ros_ws/deployment/scripts/test_command_arbiter.py

echo "Arbiter node log: $LOG_FILE"
