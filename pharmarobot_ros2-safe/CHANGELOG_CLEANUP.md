# Cleanup performed

## Removed

- all generated `build/`, `install/` and `log/` trees;
- `src/kalman_odom`;
- legacy combined launch files that started Kalman, static odom TF, duplicated LiDARs and duplicated control nodes;
- vendored joystick driver repository (the Docker image now installs `ros-humble-joy-linux`);
- obsolete root Docker/start scripts;
- nested Git metadata, editor swap files and backup C++ files;
- obsolete `src/os_settings` service file;
- Nav2, TurtleBot, RealSense and SICK packages from the Dockerfile.

## Kept but disabled by default

- `src/odom_test_logger`;
- `src/robot_test_logger`;
- historical `odom_test_logs/` and `robot_logs/` data;
- motion/calibration scripts under `tools/optional_tests/`.

The optional logger packages contain `COLCON_IGNORE` files and are not built or launched during normal operation. They can only be enabled through the explicit helper script.

## Core changes

- removed runtime dependency on `robot_test_logger` from joystick and Roboteq nodes;
- added Joy message timeout stop;
- added Roboteq command timeout, non-finite command rejection, command clamping and stop-on-shutdown;
- replaced the undefined empty `std::optional` kinematics object with a concrete object;
- reduced high-rate control logs from INFO to DEBUG;
- created isolated control-only and LiDAR-only launches;
- added supervised systemd services and exact stop commands;
- removed `--privileged` from the proposed container deployment;
- added clean Docker and Git ignore rules.

## Command arbitration update

- Added the `command_arbiter` package.
- Separated command inputs into `/cmd_vel/joy`, `/cmd_vel/test` and `/cmd_vel/nav`.
- Changed the Roboteq input to `/cmd_vel/safe`.
- Added priority and freshness selection with a zero-output fallback.
- Updated optional motion tests to use `/cmd_vel/test`.
- Added an isolated software integration test.
