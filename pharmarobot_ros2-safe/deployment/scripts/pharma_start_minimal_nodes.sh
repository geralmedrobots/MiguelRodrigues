#!/usr/bin/env bash
set -euo pipefail

CONTAINER="pharma_container"

echo "[pharma] Waiting for container..."

for i in $(seq 1 60); do
    if docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
        echo "[pharma] Container is running."
        break
    fi
    sleep 2
done

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
    echo "[pharma] ERROR: container is not running."
    exit 1
fi

echo "[pharma] Stopping old control launch/nodes..."

docker exec "$CONTAINER" bash -lc '
pkill -TERM -f "[c]ontrol_only.launch.py" || true
pkill -TERM -f "[r]os2 launch teleop_pharma control_only.launch.py" || true
pkill -TERM -f "[j]oy_linux_node" || true
pkill -TERM -f "[j]oy_to_cmd_vel_node" || true
pkill -TERM -f "[c]ommand_arbiter_node" || true
pkill -TERM -f "[r]oboteq_ros2_driver_node" || true
sleep 2
pkill -KILL -f "[c]ontrol_only.launch.py" || true
pkill -KILL -f "[r]os2 launch teleop_pharma control_only.launch.py" || true
pkill -KILL -f "[j]oy_linux_node" || true
pkill -KILL -f "[j]oy_to_cmd_vel_node" || true
pkill -KILL -f "[c]ommand_arbiter_node" || true
pkill -KILL -f "[r]oboteq_ros2_driver_node" || true
' || true

echo "[pharma] Starting teleop_pharma control_only.launch.py..."

docker exec -d "$CONTAINER" bash -lc '
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash

exec ros2 launch teleop_pharma control_only.launch.py \
  > /tmp/control_only.launch.log 2>&1
'

sleep 6

echo "[pharma] Control processes:"
docker exec "$CONTAINER" bash -lc '
ps -eo pid,args | grep -E "[c]ontrol_only.launch.py|[j]oy_linux_node|[j]oy_to_cmd_vel_node|[c]ommand_arbiter_node|[r]oboteq_ros2_driver_node" || true
'

echo "[pharma] Control launch log:"
docker exec "$CONTAINER" bash -lc '
tail -n 80 /tmp/control_only.launch.log
'

echo "[pharma] ROS graph check:"
docker exec "$CONTAINER" bash -lc '
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash

echo "--- Nodes ---"
ros2 node list || true

echo "--- /cmd_vel/joy ---"
ros2 topic info /cmd_vel/joy -v || true

echo "--- /cmd_vel/safe ---"
ros2 topic info /cmd_vel/safe -v || true
'

exit 0
