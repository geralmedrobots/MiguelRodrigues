#!/usr/bin/env bash
set -euo pipefail

cd /ros_ws
source /opt/ros/humble/setup.bash

rm -rf build install log
colcon build --symlink-install \
  --packages-up-to \
    serial \
    sllidar_ros2 \
    joy_to_cmdvel \
    command_arbiter \
    roboteq_ros2_driver \
    teleop_pharma
