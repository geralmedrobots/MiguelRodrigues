#!/usr/bin/env bash
set -uo pipefail

CONTAINER="${CONTAINER:-pharma_container}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
HOST_LOG_DIR="${HOST_LOG_DIR:-$REPO_ROOT/odom_test_logs}"

TARGET_DISTANCE="2.0"
SPEED="0.15"
WHEEL_RADIUS="0.0881"
WHEELBASE="0.453"
ENCODER_CPR="4096.0"

MEASUREMENTS_FILE="$HOST_LOG_DIR/inverse_2m_measurements.csv"

mkdir -p "$HOST_LOG_DIR"

if [ ! -f "$MEASUREMENTS_FILE" ]; then
  echo "trial_name,target_encoder_distance_m,measured_distance_m,error_m,error_percent,pass_fail,notes" > "$MEASUREMENTS_FILE"
fi

echo "Starting 5 inverse odometry tests."
echo "Target encoder distance: ${TARGET_DISTANCE} m"
echo "Speed: ${SPEED} m/s"
echo "Wheel radius: ${WHEEL_RADIUS} m"
echo "Pass condition: measured distance within ±2% of 2.0 m"
echo ""

for i in $(seq 1 5); do
  TRIAL_NAME=$(printf "inverse_2m_radius_0881_trial_%02d" "$i")
  TRIAL_DIR="$HOST_LOG_DIR/$TRIAL_NAME"

  echo "===================================================="
  echo "Trial: $TRIAL_NAME"
  echo "===================================================="

  rm -rf "$TRIAL_DIR"
  mkdir -p "$TRIAL_DIR"

  echo ""
  echo "Prepare robot:"
  echo "1. Put robot at start mark."
  echo "2. Make sure robot is ENABLED."
  echo "3. Clear the path for at least 2.5 m."
  echo ""
  read -p "Press ENTER when ready..."

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

  echo "[3/5] Creating and running 2 m encoder-target movement..."
  docker exec -i "$CONTAINER" bash <<PYBASH
cat > /tmp/move_2m_from_ticks.py <<'PY'
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from roboteq_ros2_driver.msg import WheelTicks


class MoveTargetFromTicks(Node):
    def __init__(self):
        super().__init__("move_2m_from_ticks")

        self.target_distance_m = float("${TARGET_DISTANCE}")
        self.speed_mps = float("${SPEED}")
        self.wheel_radius_m = float("${WHEEL_RADIUS}")
        self.encoder_cpr = abs(float("${ENCODER_CPR}"))
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius_m) / self.encoder_cpr

        self.distance_m = 0.0
        self.finished = False
        self.last_log_time = time.time()

        self.pub = self.create_publisher(Twist, "/cmd_vel/test", 10)
        self.sub = self.create_subscription(WheelTicks, "/wheel_ticks", self.ticks_callback, 50)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"Target={self.target_distance_m:.3f} m | speed={self.speed_mps:.3f} m/s | "
            f"wheel_radius={self.wheel_radius_m:.4f} m | meters_per_tick={self.meters_per_tick:.9f}"
        )

    def ticks_callback(self, msg):
        if self.finished:
            return

        left_delta_m = float(msg.left_ticks) * self.meters_per_tick
        right_delta_m = float(msg.right_ticks) * self.meters_per_tick
        center_delta_m = 0.5 * (left_delta_m + right_delta_m)

        self.distance_m += abs(center_delta_m)

    def stop(self):
        msg = Twist()
        self.pub.publish(msg)

    def control_loop(self):
        if self.finished:
            self.stop()
            return

        if self.distance_m >= self.target_distance_m:
            self.finished = True
            self.stop()
            self.get_logger().info(
                f"Target reached. Encoder distance={self.distance_m:.4f} m. Stopping."
            )
            time.sleep(0.5)
            rclpy.shutdown()
            return

        msg = Twist()
        msg.linear.x = self.speed_mps
        msg.angular.z = 0.0
        self.pub.publish(msg)

        now = time.time()
        if now - self.last_log_time > 0.5:
            self.last_log_time = now
            self.get_logger().info(
                f"Moving | encoder_distance={self.distance_m:.4f}/{self.target_distance_m:.4f} m"
            )


def main():
    rclpy.init()
    node = MoveTargetFromTicks()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()


if __name__ == "__main__":
    main()
PY
PYBASH

  docker exec -it "$CONTAINER" bash -lc '
    source /opt/ros/humble/setup.bash
    source /ros_ws/install/setup.bash
    timeout --signal=SIGINT --kill-after=2 30 python3 /tmp/move_2m_from_ticks.py
  '

  echo "[4/5] Stopping odom_test_logger..."
  docker exec "$CONTAINER" bash -lc '
    pkill -f "[o]dom_test_logger_node" || true
  '

  sleep 1

  echo "[5/5] Copying log to VS Code folder..."
  docker cp "$CONTAINER":/root/odom_test_logs/. "$TRIAL_DIR/"

  echo ""
  read -p "Measured real distance [m]: " MEASURED
  read -p "Notes: " NOTES

  ERROR=$(python3 - <<PY
target = float("${TARGET_DISTANCE}")
measured = float("${MEASURED}")
error = measured - target
print(f"{error:.4f}")
PY
)

  ERROR_PERCENT=$(python3 - <<PY
target = float("${TARGET_DISTANCE}")
measured = float("${MEASURED}")
error_percent = ((measured - target) / target) * 100.0
print(f"{error_percent:.2f}")
PY
)

  PASS_FAIL=$(python3 - <<PY
target = float("${TARGET_DISTANCE}")
measured = float("${MEASURED}")
error_percent = abs((measured - target) / target) * 100.0
print("PASS" if error_percent <= 2.0 else "FAIL")
PY
)

  echo "$TRIAL_NAME,$TARGET_DISTANCE,$MEASURED,$ERROR,$ERROR_PERCENT,$PASS_FAIL,$NOTES" >> "$MEASUREMENTS_FILE"

  echo ""
  echo "Result: measured=${MEASURED} m | error=${ERROR} m | error=${ERROR_PERCENT}% | $PASS_FAIL"
  echo "Saved log:"
  ls -lt "$TRIAL_DIR"
  echo ""
done

echo "All 5 inverse 2 m tests finished."
echo "Measurements saved to:"
echo "$MEASUREMENTS_FILE"
