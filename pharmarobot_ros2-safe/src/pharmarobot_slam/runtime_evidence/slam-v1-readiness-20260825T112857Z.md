# SLAM v1 base_footprint runtime readiness — 2026-08-25 11:28:57 UTC

## Scope and safety boundary

This was a read-only stationary check of the already-running ROS 2 Humble
graph. `slam_toolbox` was not launched. No Twist was published, no motor or
Roboteq command was issued, and no service or container was restarted.

## Candidate source identity

```text
96a263fa81ea3a486b53a2814806e5bf37af315493b1117b12efe92ca9d1083a  src/roboteq_ros2_driver/config/roboteq.yaml
1dc6c9f23fd1ed3f41ba908095b10289a13896b827a7fec0ee59a8c82761c8ad  src/teleop_pharma/launch/lidar_only.launch.xml
c5c0a837e352abe5cb41f72d6504384e96978a029b14badbd7b9294d5eab7760  src/pharmarobot_slam/tools/slam_v1_readiness.py
```

The candidate files specify:

```text
/odom.header.frame_id = odom
/odom.child_frame_id = base_footprint
dynamic TF: odom -> base_footprint
static TF: base_footprint -> base_link, xyz=(0, 0, 0.042), rpy=(0, 0, 0)
static TFs preserved: base_link -> front_laser, base_link -> back_laser
```

## Command

```bash
docker exec pharma_container bash -lc \
  'source /opt/ros/humble/setup.bash; source /ros_ws/install/setup.bash; \
   ros2 run pharmarobot_slam slam_v1_readiness.py'
```

## Raw result

Exit status: `1` (`FAIL`)

```text
BLOCKED: odometry child frame is 'base_link'; expected 'base_footprint'
BLOCKED: missing TF odom -> base_footprint
BLOCKED: missing TF base_footprint -> base_link
BLOCKED: unexpected /tf_static publishers: ['/base_to_back_laser', '/base_to_front_laser']
[ros2run]: Process exited with failure 1
```

## Interpretation

The live processes were started before the candidate configuration and launch
changes. They still expose the old `odom -> base_link` graph and the two old
laser static publishers. The source/install paths are symlinked to the updated
workspace, but a running ROS process does not reload its startup parameters or
launch actions automatically.

The exact runtime blocker is therefore a controlled reload of both affected
launch processes. That reload was not performed because restarting the control
stack would access the Roboteq runtime and was not authorized by the task's
no-motor-command boundary. After an explicitly approved stationary reload,
rerun the same readiness command; PASS requires fresh `/front_scan`, fresh
`/odom` with child `base_footprint`, fresh `odom -> base_footprint`, the exact
static `base_footprint -> base_link`, both unchanged laser static publishers,
and no other `/tf` or `/tf_static` publishers.
