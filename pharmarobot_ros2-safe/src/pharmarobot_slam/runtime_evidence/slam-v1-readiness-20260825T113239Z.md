# SLAM v1 base_footprint runtime readiness — 2026-08-25 11:32:39 UTC

## Scope and safety boundary

Read-only stationary observation of the already-running ROS 2 Humble graph.
No `slam_toolbox`, Twist publisher, motor command, Roboteq action, service
restart, container restart, Nav2, or robot motion was performed.

## Candidate hashes

```text
96a263fa81ea3a486b53a2814806e5bf37af315493b1117b12efe92ca9d1083a  src/roboteq_ros2_driver/config/roboteq.yaml
1dc6c9f23fd1ed3f41ba908095b10289a13896b827a7fec0ee59a8c82761c8ad  src/teleop_pharma/launch/lidar_only.launch.xml
c0206bd2885e03823159f30e3b4bbff521f419b235213d8fe157331caa153cb7  src/pharmarobot_slam/tools/slam_v1_readiness.py
```

## Reproduction command

```bash
docker exec pharma_container bash -lc \
  'source /opt/ros/humble/setup.bash; source /ros_ws/install/setup.bash; \
   ros2 run pharmarobot_slam slam_v1_readiness.py'
```

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

## Exact blocker

The active odometry and lidar launch processes predate the candidate changes.
They retain startup-only parameters/actions, so live runtime still owns
`odom -> base_link` and only the two laser static edges. A controlled stationary
reload of both affected production launch processes is required before the new
contract can exist live. That reload was intentionally not performed because
restarting the control stack accesses the Roboteq runtime and exceeds the
explicit no-motor-command boundary.

After separately authorized reload, PASS requires exactly:

```text
/tf publisher:        /roboteq_ros2_driver
/tf edge:             odom -> base_footprint
/tf_static publishers:/footprint_to_base_link, /base_to_front_laser,
                      /base_to_back_laser
/tf_static edges:     base_footprint -> base_link,
                      base_link -> front_laser,
                      base_link -> back_laser
```

It also requires fresh `/front_scan`, `/odom`, and dynamic TF timestamps, exact
`base_footprint -> base_link` geometry `(0, 0, 0.042, identity rotation)`, and
no pre-existing `map -> odom`.
