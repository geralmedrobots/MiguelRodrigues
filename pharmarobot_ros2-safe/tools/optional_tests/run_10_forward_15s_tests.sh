#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
HOST_LOG_DIR="${HOST_LOG_DIR:-$REPO_ROOT/odom_test_logs}"
CONTAINER="${CONTAINER:-pharma_container}"

CMD_LINEAR="0.2"
CMD_ANGULAR="0.0"
DURATION="15"
WHEEL_RADIUS="0.0881"
WHEELBASE="0.453"
ENCODER_CPR="4096.0"

mkdir -p "$HOST_LOG_DIR"

MEASUREMENTS_FILE="$HOST_LOG_DIR/forward_15s_measurements.csv"

if [ ! -f "$MEASUREMENTS_FILE" ]; then
  echo "trial_name,tape_distance_m,video_file,video_straight_distance_m,video_path_length_m,notes" > "$MEASUREMENTS_FILE"
fi

echo "Starting 10 forward odometry tests."
echo "Command per trial: linear.x=${CMD_LINEAR} m/s, angular.z=${CMD_ANGULAR}, duration=${DURATION} s"
echo "Logger parameters: wheel_radius=${WHEEL_RADIUS}, wheelbase=${WHEELBASE}, encoder_cpr=${ENCODER_CPR}"
echo ""

for i in $(seq 1 10); do
  TRIAL_NAME=$(printf "forward_15s_trial_%02d" "$i")
  TRIAL_DIR="$HOST_LOG_DIR/$TRIAL_NAME"

  echo "===================================================="
  echo "Trial: $TRIAL_NAME"
  echo "===================================================="

  rm -rf "$TRIAL_DIR"
  mkdir -p "$TRIAL_DIR"

  echo ""
  echo "Prepare the robot:"
  echo "1. Put robot at the start mark."
  echo "2. Make sure the robot is ENABLED."
  echo "3. Start Video Physics recording now."
  echo ""
  read -p "Press ENTER when video is recording and robot is ready..."

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
      -p wheel_radius_m:=$WHEEL_RADIUS \
      -p wheelbase_m:=$WHEELBASE \
      -p encoder_cpr:=$ENCODER_CPR \
      > /tmp/odom_test_logger.log 2>&1
  "

  sleep 2

  echo "[3/5] Running ${DURATION}s forward movement..."
  docker exec "$CONTAINER" bash -lc "
    source /opt/ros/humble/setup.bash
    source /ros_ws/install/setup.bash

    timeout --signal=SIGINT $DURATION ros2 topic pub --rate 20 /cmd_vel/test geometry_msgs/msg/Twist \
    \"{linear: {x: $CMD_LINEAR, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $CMD_ANGULAR}}\" || true

    ros2 topic pub -1 /cmd_vel/test geometry_msgs/msg/Twist \
    \"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\"
  "

  echo "[4/5] Stopping odom_test_logger..."
  docker exec "$CONTAINER" bash -lc '
    pkill -f "[o]dom_test_logger_node" || true
  '

  sleep 1

  echo "[5/5] Copying log to VS Code folder..."
  docker cp "$CONTAINER":/root/odom_test_logs/. "$TRIAL_DIR/"

  echo ""
  echo "Stop Video Physics recording now."
  echo "Measure the same robot reference point from start to end."
  echo ""

  read -p "Tape measured straight-line distance [m]: " TAPE_DISTANCE
  read -p "Video filename/reference: " VIDEO_FILE
  read -p "Video straight-line distance [m] if known, else leave blank: " VIDEO_STRAIGHT
  read -p "Video path length [m] if known, else leave blank: " VIDEO_PATH
  read -p "Notes, e.g. drift/slip/yaw/enable issue: " NOTES

  echo "$TRIAL_NAME,$TAPE_DISTANCE,$VIDEO_FILE,$VIDEO_STRAIGHT,$VIDEO_PATH,$NOTES" >> "$MEASUREMENTS_FILE"

  echo ""
  echo "Saved log files:"
  ls -lt "$TRIAL_DIR"
  echo ""
done

echo "All 10 trials finished."
echo "Measurements saved to:"
echo "$MEASUREMENTS_FILE"
