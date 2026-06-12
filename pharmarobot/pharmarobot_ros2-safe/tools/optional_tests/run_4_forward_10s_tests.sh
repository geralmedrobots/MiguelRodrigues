#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
HOST_LOG_DIR="${HOST_LOG_DIR:-$REPO_ROOT/odom_test_logs}"
CONTAINER="${CONTAINER:-pharma_container}"

mkdir -p "$HOST_LOG_DIR"

echo "Starting 4 forward odometry tests."
echo "Command per trial: linear.x = 0.2 m/s, angular.z = 0.0, duration = 10 s"
echo ""

for i in 1 2 3 4; do
  TRIAL_NAME=$(printf "forward_10s_trial_%02d" "$i")
  TRIAL_DIR="$HOST_LOG_DIR/$TRIAL_NAME"

  echo "===================================================="
  echo "Trial: $TRIAL_NAME"
  echo "===================================================="

  rm -rf "$TRIAL_DIR"
  mkdir -p "$TRIAL_DIR"

  echo "[1/5] Clearing old odom logs..."
  docker exec "$CONTAINER" bash -lc '
    pkill -f "[o]dom_test_logger_node" || true
    rm -rf /root/odom_test_logs
    mkdir -p /root/odom_test_logs
  '

  echo "[2/5] Starting odom_test_logger..."
  docker exec -d "$CONTAINER" bash -lc "
    source /opt/ros/humble/setup.bash
    source /ros_ws/install/setup.bash

    ros2 run odom_test_logger odom_test_logger_node \
      --ros-args \
      -p test_name:=$TRIAL_NAME \
      -p wheel_radius_m:=0.0881 \
      -p wheelbase_m:=0.453 \
      -p encoder_cpr:=4096.0 \
      > /tmp/odom_test_logger.log 2>&1
  "

  sleep 2

  echo "[3/5] Running 10 s forward movement..."
  docker exec "$CONTAINER" bash -lc '
    source /opt/ros/humble/setup.bash
    source /ros_ws/install/setup.bash

    timeout --signal=SIGINT 10 ros2 topic pub --rate 20 /cmd_vel/test geometry_msgs/msg/Twist \
    "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" || true

    ros2 topic pub -1 /cmd_vel/test geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  '

  echo "[4/5] Stopping odom_test_logger..."
  docker exec "$CONTAINER" bash -lc '
    pkill -f "[o]dom_test_logger_node" || true
  '

  sleep 1

  echo "[5/5] Copying log to VS Code folder..."
  docker cp "$CONTAINER":/root/odom_test_logs/. "$TRIAL_DIR/"

  echo ""
  echo "Measure the real distance moved on the floor for $TRIAL_NAME."
  read -p "Enter measured distance in meters, then press ENTER: " DISTANCE

  echo "$TRIAL_NAME,$DISTANCE" >> "$HOST_LOG_DIR/forward_10s_measurements.csv"

  echo "Saved:"
  ls -lt "$TRIAL_DIR"
  echo ""
done

echo "All 4 trials finished."
echo "Measurements saved to:"
echo "$HOST_LOG_DIR/forward_10s_measurements.csv"
