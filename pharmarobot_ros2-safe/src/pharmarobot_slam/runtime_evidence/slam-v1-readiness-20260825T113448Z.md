# SLAM v1 base_footprint final runtime readiness — 2026-08-25 11:34:48 UTC

## Safety boundary

Read-only stationary observation of the already-running ROS 2 Humble graph.
No `slam_toolbox`, Twist publisher, motor command, Roboteq action, runtime
restart, Nav2, or robot motion was performed.

## Candidate hashes

```text
96a263fa81ea3a486b53a2814806e5bf37af315493b1117b12efe92ca9d1083a  src/roboteq_ros2_driver/config/roboteq.yaml
1dc6c9f23fd1ed3f41ba908095b10289a13896b827a7fec0ee59a8c82761c8ad  src/teleop_pharma/launch/lidar_only.launch.xml
73efe3aad1645037a69bf365805d593850bce7b0ea2c0ae6b4dee11fc236db77  src/pharmarobot_slam/tools/slam_v1_readiness.py
```

## Command

```bash
docker exec pharma_container bash -lc \
  'source /opt/ros/humble/setup.bash; source /ros_ws/install/setup.bash; \
   ros2 run pharmarobot_slam slam_v1_readiness.py'
```

The checker observed the complete bounded five-second window.

## Result: FAIL (exit 1)

```text
BLOCKED: odometry child frame is 'base_link'; expected 'base_footprint'
BLOCKED: missing TF odom -> base_footprint
BLOCKED: missing TF base_footprint -> base_link
BLOCKED: unexpected /tf_static publishers: ['/base_to_back_laser', '/base_to_front_laser']
BLOCKED: unexpected /tf frame pairs: [('odom', 'base_link')]; expected [('odom', 'base_footprint')]
BLOCKED: unexpected /tf_static frame pairs: [('base_link', 'back_laser'), ('base_link', 'front_laser')]; expected [('base_footprint', 'base_link'), ('base_link', 'back_laser'), ('base_link', 'front_laser')]
[ros2run]: Process exited with failure 1
```

## Exact blocker and next gate

The active odometry and lidar launch processes were started before the source
changes and retain the old startup-only contract. Runtime therefore still owns
`odom -> base_link` and lacks `base_footprint`.

A separately authorized, stationary controlled reload of the affected control
and lidar launch processes is required. The control reload was not inferred
from authorization to run the read-only checker because it accesses the
Roboteq runtime. After reload, rerun the same command. PASS requires fresh scan,
odometry, and dynamic-TF timestamps; exact frame IDs and base geometry; exactly
one expected dynamic publisher/edge; exactly three expected static
publishers/edges; and no existing `map -> odom`.
