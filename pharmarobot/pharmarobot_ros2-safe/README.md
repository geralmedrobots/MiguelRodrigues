# PharmaRobot ROS 2 — cleaned core workspace

This version intentionally contains only the components required for the current development phase:

- joystick input (`joy_linux`, installed from ROS packages);
- joystick-to-velocity conversion (`joy_to_cmdvel`);
- priority/timeout command arbitration (`command_arbiter`);
- Roboteq motor and wheel-odometry driver;
- dual SLLIDAR drivers;
- static `base_link -> front_laser/back_laser` transforms;
- optional, explicitly disabled odometry/test loggers.

SLAM, Nav2, Kalman filtering, RealSense, SICK scanner integration, TurtleBot packages and legacy combined launch files are not part of the active workspace.

## Active launch files

```text
teleop_pharma/control_only.launch.py
  joy_node
  joy_to_cmdvel          -> /cmd_vel/joy
  command_arbiter        -> /cmd_vel/safe
  roboteq_ros2_driver    <- /cmd_vel/safe

teleop_pharma/lidar_only.launch.xml
  rplidar_front
  rplidar_back
  base_to_front_laser
  base_to_back_laser
```

No normal launch or service starts test loggers.

## Build the Docker image

From the repository root:

```bash
./deployment/scripts/build_image.sh
```

Default image name:

```text
pharmarobot:clean
```

## Install the NUC services

Review `/etc/default/pharmarobot` values, especially workspace path, ports and ROS domain, then run:

```bash
./deployment/install_services.sh
```

Start the container and the two independent stacks:

```bash
sudo systemctl restart pharmarobot.service
sudo systemctl restart pharma-minimal-nodes.service
sudo systemctl restart pharma-lidar-nodes.service
```

Check health:

```bash
systemctl status pharma-minimal-nodes.service --no-pager -l
systemctl status pharma-lidar-nodes.service --no-pager -l

docker exec pharma_container bash -lc '
  source /opt/ros/humble/setup.bash
  source /ros_ws/install/setup.bash
  ros2 node list | sort
  ros2 node list | sort | uniq -d
'
```

Expected active nodes:

```text
/base_to_back_laser
/base_to_front_laser
/joy_node
/joy_to_cmdvel
/command_arbiter
/roboteq_ros2_driver
/rplidar_back
/rplidar_front
```

The duplicate-node command should print nothing.

## Clean rebuild inside the running container

```bash
docker exec -it pharma_container bash -lc '/ros_ws/deployment/scripts/build_core.sh'
```

Then restart the relevant service.


## Command-source arbitration

The motor driver no longer subscribes to the public source topics directly. Intended producers publish to separate inputs:

```text
/cmd_vel/joy   priority 100
/cmd_vel/test  priority 50
/cmd_vel/nav   priority 10
       |
       v
command_arbiter
       |
       v
/cmd_vel/safe -> roboteq_ros2_driver
```

Each input expires after 0.25 s. The arbiter publishes at 20 Hz and outputs zero when no source is fresh. This prevents intended command sources from fighting, but it does not authenticate publishers; ROS-domain isolation, firewalling and SROS2 remain separate Priority 1 work.

Run the software-only integration test inside the container:

```bash
docker exec -it pharma_container bash -lc '/ros_ws/deployment/scripts/test_command_arbiter.sh'
```

The test uses an isolated ROS domain and does not launch the Roboteq driver.

## Optional test/logging tools

The logger packages are excluded by `COLCON_IGNORE` and are not built during normal operation.

To enable them for a controlled calibration session:

```bash
./tools/optional_tests/build_optional_test_nodes.sh
```

See:

```text
tools/optional_tests/README.md
```

Historical CSV files remain under `odom_test_logs/` and `robot_logs/`, but are not copied into the Docker image and are not started by any service.

## Important safety limitations

This cleaned version improves process separation and adds command timeouts, but it is still a prototype. It does not yet include:

- a physical hold-to-run deadman mapped in software;
- SROS2 authentication/authorisation;
- Roboteq STO/fault/status gating;
- stable udev device names;
- dynamic `odom -> base_link` TF.

Do not use it for unattended or autonomous operation until Priority 1 and Priority 2 items in `PRIORITY_PLAN.md` are closed and hardware-tested.


## Persistent USB device names

The runtime uses stable names rather than enumeration-dependent `/dev/ttyUSB*` paths:

```text
/dev/roboteq
/dev/lidar_front
/dev/lidar_back
```

Install the rules once while the current known mapping is connected:

```bash
sudo ./deployment/scripts/install_usb_udev_rules.sh \
  --roboteq /dev/ttyUSB0 \
  --front /dev/ttyUSB2 \
  --back /dev/ttyUSB1
```

Validate at any time with:

```bash
./deployment/scripts/check_usb_devices.sh
```

The installer prefers a unique USB serial number. If a device does not expose a
unique serial number, it matches its physical USB port path; in that case keep
that device connected to the same physical port.
