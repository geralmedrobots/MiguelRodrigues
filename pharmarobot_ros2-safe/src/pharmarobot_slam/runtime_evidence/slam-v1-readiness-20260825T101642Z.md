# SLAM v1 runtime readiness — 2026-08-25 10:16:42 UTC

## Scope and safety

Read-only ROS 2 Humble diagnostics in the already-running `pharma_container`.
No `slam_toolbox` launch, command publication, robot motion, Roboteq device
access, service restart, or production TF/configuration change was performed.

## Environment and preparation

Container: `pharma_container`, image `pharmarobot:clean`, running.

Initial exact command:

```bash
docker exec pharma_container bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && \
   ros2 run pharmarobot_slam slam_v1_readiness.py'
```

Initial result: `Package 'pharmarobot_slam' not found`. The mounted source was
present and discoverable by `colcon list`, but no install artifact existed. A
package-only build was performed:

```bash
docker exec pharma_container bash -lc \
  'source /opt/ros/humble/setup.bash; cd /ros_ws; \
   colcon build --symlink-install --packages-select pharmarobot_slam'
```

Build result: `1 package finished`. The next run proved an executable-bit
packaging defect (`No executable found`); only the two installed SLAM utility
scripts were marked executable. No runtime service was restarted.

## Live graph evidence

- `/front_scan`: `sensor_msgs/msg/LaserScan`, one publisher
  `/rplidar_front`; received frame `front_laser`, stamp
  `1787652881.035460475`.
- `/odom`: `nav_msgs/msg/Odometry`, one publisher
  `/roboteq_ros2_driver`; received frame `odom`, stamp
  `1787652881.142227058`.
- `/tf`: one publisher, `/roboteq_ros2_driver`.
- `/tf_static`: two publishers, `/base_to_front_laser` and
  `/base_to_back_laser`, owning distinct sensor transforms.
- `/back_scan` exists as `sensor_msgs/msg/LaserScan`, but SLAM v1 did not
  subscribe to or require it.
- No SICK node/topic and no scan-merging node were required.

Bounded `tf2_echo` evidence:

- `odom -> base_link`: available and updating.
- `base_link -> front_laser`: available; static translation
  `[0.410, 0.000, 0.290]`.
- `odom -> base_footprint`: unavailable; frame `base_footprint` does not exist.
- `base_footprint -> base_link`: unavailable; frame `base_footprint` does not
  exist.

## Final reproducible readiness command

```bash
docker exec pharma_container bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && \
   ros2 run pharmarobot_slam slam_v1_readiness.py'
```

Result, exit code `1`:

```text
BLOCKED: odometry child frame is 'base_link'; expected 'base_footprint'
BLOCKED: missing TF odom -> base_footprint
BLOCKED: missing TF base_footprint -> base_link
[ros2run]: Process exited with failure 1
```

## Verdict

FAIL. LaserScan, odometry freshness, the existing `odom -> base_link ->
front_laser` chain, and TF publisher ownership are healthy. The requested SLAM
contract `odom -> base_footprint -> base_link -> front_laser` is not present.
No transform was fabricated to make the check pass.
