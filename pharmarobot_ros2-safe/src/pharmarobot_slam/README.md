# pharmarobot_slam

SLAM v1 for ROS 2 Humble using `slam_toolbox` and the existing differential
drive odometry. It is deliberately runtime-modular: the launch assumes lidar,
odometry and robot TF are already running and launches only the mapper.

## Configuration

`config/slam_toolbox_v1.yaml` selects `map`, `odom`, `base_footprint`, and
`/front_scan`. Production uses the repository's two SLLIDARs, identified by
the existing launch as `front_laser` and `back_laser`. SLAM v1 intentionally
uses only the front SLLIDAR.

`rviz/slam_v1.rviz` uses `map` as the fixed frame and includes TF, RobotModel,
LaserScan on `/front_scan`, Odometry on `/odom`, and Map on `/map`. RViz may be
run from a separate workstation. To start the reproducible local view, use:

```bash
ros2 launch pharmarobot_slam rviz_v1.launch.py
```

This starts RViz2 only; it does not start SLAM, lidar, odometry, or any motion
node. `/back_scan` remains outside SLAM v1.

## Readiness contract

Before mapping, validate that `/front_scan` is `sensor_msgs/msg/LaserScan`, has
`frame_id=front_laser`, nonzero timestamps, finite angle fields, and sensible
`range_min`/`range_max`. Validate the live chain
`odom -> base_footprint -> base_link -> front_laser` and that exactly one node
owns each edge. The static `base_footprint -> base_link` transform is
`x=0, y=0, z=0.042, rpy=0`.
Do not create a fake transform to satisfy this contract. `map -> odom` must be
free for slam_toolbox before launch. Until it publishes `map`, use `odom` as
the RViz fixed frame for diagnostics.

The read-only runtime checker is installed as
`pharmarobot_slam/slam_v1_readiness.py`; run it only after the existing lidar,
odometry, and static TF services are already running. It checks topic types,
LaserScan fields, odometry and dynamic-TF freshness, required transforms, the
exact base offset, exclusive TF publisher endpoints and frame pairs, and
rejects a pre-existing `map -> odom`. It observes for the complete bounded
timeout so late-discovered publishers within that window are included.
It does not publish commands, TF, or any other runtime data. The static checker
`validate_slam_v1.py` is also available for offline CI.

The production lidar contract is the repository's `sllidar_ros2` architecture:
primary `/front_scan` with frame `front_laser`, plus deferred `/back_scan` with
frame `back_laser`. No SICK dependency or scan-merging layer is required for
SLAM v1.

## Offline checks

This package is intentionally testable without ROS runtime or hardware: check
the YAML/launch text and package manifest, then use the repository's normal
ament/colcon validation when a sourced Humble environment is available.

## First manual moving trial recorder

Start the existing lidar, odometry, static-TF, and command-arbiter graph first.
Then run the recorder from the sourced ROS workspace:

```bash
ros2 run pharmarobot_slam slam_v1_trial_recorder.py
```

Drive only with the manual joystick through the existing `/cmd_vel` -> arbiter
-> `/cmd_vel/safe` chain. The recorder starts only `slam_v1.launch.py` and a
rosbag; it sends no commands and does not start a driver or joystick. Stop it
from a second sourced terminal with:

```bash
ros2 run pharmarobot_slam slam_v1_trial_stop.py
```

Each run is written to `slam-v1-trials/slam-v1-<UTC timestamp>/` (or the
`--evidence-root` supplied to the recorder). It contains the bag, mapper
stdout/stderr, topic freshness/count samples, start/end publisher ownership,
start/end TF-tree captures, final map and map metadata, bag info, process
snapshots, and `manifest.json`. Capture aborts on mapper exit, stale/invalid
`/cmd_vel/safe`, required-topic loss, or duplicate dynamic-topic publishers.
An offline recorder test does not establish graph or physical-runtime success;
inspect the manifest and abort reason after the run.
