# PharmaRobot remediation priorities

Priority scale:

- **1 — Maximum priority:** must be resolved before normal mobile operation outside a controlled test area.
- **2 — High priority:** required before serious SLAM/Nav2 integration and repeated autonomous testing.
- **3 — Medium priority:** reliability and maintainability work for a stable prototype.
- **4 — Low priority:** engineering quality and deployment hardening.
- **5 — Optional optimisation:** performance, convenience and future productisation.

## Priority 1 — Maximum priority

| Item | Risk | Status in this cleaned version | Next action |
|---|---|---|---|
| Command-source arbitration | Multiple intended command sources could fight for the motor driver | **Solved in source; runtime validation required** | A priority/timeout arbiter now accepts `/cmd_vel/joy`, `/cmd_vel/test` and `/cmd_vel/nav`, and exclusively publishes `/cmd_vel/safe`; the Roboteq driver subscribes only to `/cmd_vel/safe`. Run the isolated integration test and verify the live graph. |
| Hold-to-run deadman | Joystick movement must require a dedicated held button | **Implemented in source** | L1 (button index 4 by default) is required for every non-zero joystick command; validate the physical mapping and release/timeout behaviour on the NUC. |
| Joystick-loss stop | Lost `/joy` messages could leave an old command active | **Improved** | A 0.5 s Joy freshness watchdog now publishes zero; validate on hardware and reduce toward 0.2 s if stable. |
| Driver command timeout | Driver could retain the last command after upstream failure | **Improved** | A configurable 0.5 s timeout sends zero commands; verify the Roboteq watchdog and fault response. |
| Driver shutdown stop | Node shutdown previously only closed the port | **Improved** | Explicit zero commands are now sent before closing; hardware-test abrupt process and USB loss. |
| Undefined kinematics object | Empty `std::optional` was dereferenced | **Solved in source** | Rebuild cleanly and run odometry regression tests. |
| Remote ROS access | Unsecured DDS allows remote command publication | **Not solved** | Use a dedicated ROS domain/firewall immediately; design SROS2 permissions before deployment. |
| Hardware fault/STO monitoring | ROS can send commands while Roboteq is disabled or faulted | **Not solved** | Read and publish controller fault/status/STO flags; inhibit motion unless healthy. |
| Autonomous test runaway | Encoder-target test could run indefinitely | **Improved** | Optional inverse test now has a 30 s wall-clock timeout; add stale-tick and safety-scanner aborts before reuse. |

## Priority 2 — High priority

| Item | Risk | Status in this cleaned version | Next action |
|---|---|---|---|
| One authoritative bring-up | Legacy launches produced duplicate joystick, TF, LiDAR, Kalman and driver nodes | **Solved structurally** | Only `control_only.launch.py` and `lidar_only.launch.xml` remain. Validate service restart order on the NUC. |
| Process supervision | Detached `docker exec -d` let systemd show healthy while nodes were dead | **Improved** | New services run foreground `docker exec` processes with `Restart=on-failure`; install and test them. |
| Broad `pkill` patterns | Restarting control could kill LiDAR or unrelated ROS launches | **Solved structurally** | New stop scripts target only the exact launch command. |
| Stable USB device names | `/dev/ttyUSB0/1/2` can change after reboot/replug | **Solved in source; deployment validation required** | Install the generated udev rules once, verify all three symlinks, then reboot/replug-test before closing the item. |
| Serial protocol robustness | `flushInput`, `stoi`, serial exceptions and blocking reads can hide/crash on faults | **Not solved** | Create a single serial transaction layer with bounded reads, parser validation and recovery. |
| Controller configuration validation | Runtime startup overwrites MMOD/PID/EPPR without readback | **Not solved** | Separate commissioning from runtime; read back and verify each required setting. |
| Dynamic odometry TF | SLAM needs valid `odom -> base_link`; legacy static transform was wrong | **Legacy fault removed; feature missing** | Implement dynamic TF from wheel odometry before SLAM. |
| Coupled wheel saturation | Combined linear/angular command can saturate one wheel asymmetrically | **Partially improved** | Final RPM/power is clamped, but proportional coupled scaling should replace independent clipping. |
| Clean reproducible builds | Old `build/install/log` trees caused stale binaries | **Solved in repository** | Generated trees removed; always perform a clean build after replacing the workspace. |

## Priority 3 — Medium priority

| Item | Status | Recommended work |
|---|---|---|
| Odometry twist output | Pending | Publish measured linear/angular velocity instead of zeros. |
| Odometry covariance | Pending | Derive realistic covariance from calibration and surface tests. |
| Serial I/O separation | Pending | Move encoder polling off the single-threaded command callback path. |
| Parameter validation | Pending | Reject non-positive radius, wheelbase, CPR magnitude, RPM and current limits at startup. |
| Sign convention | Pending | Replace hidden hard-coded sign inversions with explicit YAML parameters and unit tests. |
| Controller/ROS diagnostics | Pending | Publish diagnostic state for serial link, watchdog, faults, STO and encoder freshness. |
| LiDAR health checks | Pending | Add topic-frequency and serial health checks to the LiDAR service. |
| Rotation calibration | Pending | Calibrate wheelbase using repeated left/right 90°, 180° and 360° tests. |
| SLAM readiness | Pending | Validate TF, timestamps, scan frame IDs and odometry before enabling SLAM Toolbox. |

## Priority 4 — Low priority

| Item | Status | Recommended work |
|---|---|---|
| Non-root container user | Pending | Run the ROS processes as a dedicated UID/GID. |
| Reduced container privileges | Improved | `--privileged` was removed; add seccomp/AppArmor and read-only mounts where practical. |
| Image reproducibility | Pending | Pin the base/image and apt dependencies by version or digest. |
| Package metadata | Partially improved | Complete maintainers, licences, descriptions and repository ownership. |
| Automated tests | Pending | Add unit tests for wheel mixing, signs, timeout behaviour and serial parsing. |
| CI | Pending | Build and run static analysis on every pull request. |
| Log rotation | Core logging removed | When logging is re-enabled, use bounded asynchronous queues and rotation. |

## Priority 5 — Optional optimisation

- Migrate XML launch files to Python for consistent validation and parameters.
- Add Docker Compose profiles for `core`, `lidar`, `test` and later `slam`.
- Add RViz configuration files for LiDAR and odometry debugging.
- Add rosbag recording profiles and automated calibration reports.
- Optimise CPU usage, QoS and scan rates after correctness is established.
- Add SROS2 certificate provisioning and a production deployment pipeline.
