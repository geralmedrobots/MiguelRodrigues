#!/usr/bin/env bash
set -u

CONTAINER="${PHARMA_CONTAINER:-pharma_container}"

kill_match()
{
  local signal="$1"
  local pattern="$2"

  docker exec "$CONTAINER" \
    pkill "-${signal}" -f "$pattern" \
    >/dev/null 2>&1 || true
}

kill_match INT  'ros2 launch teleop_pharma control_only.launch.py'
kill_match TERM '/joy_linux_node'
kill_match TERM '/joy_to_cmd_vel_node'
kill_match TERM '/command_arbiter_node'
kill_match TERM '/roboteq_ros2_driver_node'

sleep 2

kill_match KILL 'ros2 launch teleop_pharma control_only.launch.py'
kill_match KILL '/joy_linux_node'
kill_match KILL '/joy_to_cmd_vel_node'
kill_match KILL '/command_arbiter_node'
kill_match KILL '/roboteq_ros2_driver_node'
