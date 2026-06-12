# PharmaRobot ROS 2 Static Code Audit

## Scope and limitations

Reviewed the uploaded `pharmarobot_ros2-master.tar.gz`, focusing on project-owned and integration code:

- `src/joy_to_cmdvel`
- `src/roboteq_ros2_driver`
- `src/kalman_odom`
- `src/odom_test_logger`
- `src/robot_test_logger`
- `src/teleop_pharma`
- top-level Dockerfiles and shell scripts
- the systemd/start scripts pasted in the conversation

The vendored upstream trees (`src/joystick_drivers`, `src/serial`, and the SLLIDAR SDK) were reviewed for integration and supply-chain risk, not exhaustively line-by-line. This is a static audit; it does not prove the absence of further defects and does not replace hardware-in-the-loop safety validation, dependency/CVE scanning, fuzzing, or penetration testing.

## Executive conclusion

The current stack is suitable for controlled prototyping only. It is not yet safe or secure enough for unattended operation or hospital deployment. The most urgent problems are unauthenticated remote motion commands, lack of a software deadman/watchdog, unsafe command arbitration, undefined behaviour in the Roboteq driver, broken Kalman code, invalid TF architecture, and unsupervised detached ROS processes.

---

# Critical findings

## C-01 — Any host on the ROS 2 domain can command robot motion

**Evidence:**
- `src/joy_to_cmdvel/src/joy_to_cmd_vel.cpp:117` publishes `cmd_vel` with no authentication.
- `src/roboteq_ros2_driver/src/driver_dev.cpp:72,135-138` subscribes directly to `/cmd_vel`.
- `run_docker_boot.sh:34-41` and `build_and_run_docker.sh:121-134` use host networking and no SROS2 security.

**Impact:** Any machine on the same DDS domain can publish a `geometry_msgs/Twist` to `/cmd_vel`, including accidental test nodes, a colleague’s machine, or a malicious host. This can cause unintended robot motion.

**Required fix:** Use a dedicated ROS domain plus host firewall immediately; then implement SROS2 governance/permissions or a secured gateway. Do not expose the motor driver directly to the general DDS graph.

## C-02 — No actual deadman switch and no joystick-loss stop

**Evidence:**
- `src/joy_to_cmdvel/src/joy_to_cmd_vel.cpp:166-222` converts joystick axes directly to movement.
- `src/joy_to_cmdvel/src/joy_to_cmd_vel.cpp:234-242` logs `deadman_active=true` although no deadman input is checked.
- `src/joy_to_cmdvel/src/joy_to_cmd_vel.cpp:148-155` returns on malformed input without publishing zero.
- `src/roboteq_ros2_driver/include/.../roboteq_ros2_driver.hpp:45-48` declares timeout state but it is never used anywhere.

**Impact:** If the joystick disconnects or messages stop after a non-zero command, the software does not immediately command zero. The system relies only on a Roboteq watchdog that is configured but never verified.

**Required fix:** Add a hold-to-run deadman button, a periodic output timer, a joystick freshness timeout below 200 ms, and a driver-level `/cmd_vel` timeout that actively sends zero.

## C-03 — Undefined behaviour from dereferencing an empty `std::optional`

**Evidence:**
- `src/roboteq_ros2_driver/include/.../roboteq_ros2_driver.hpp:50` default-constructs an empty `std::optional<DifferentialDriveKinematics>`.
- `src/roboteq_ros2_driver/src/driver_dev.cpp:88` immediately dereferences it with `->initParam(...)` without `emplace()`.

**Impact:** Undefined behaviour at node construction. It may appear to work by accident and fail unpredictably after compiler, optimisation, or memory-layout changes.

**Required fix:** Replace the optional with a concrete member or call `differential_drive_kinematics_.emplace(...)` before dereferencing.

## C-04 — Autonomous distance test can drive indefinitely if encoder messages stop

**Evidence:**
- `run_5_inverse_2m_tests.sh:96-137` continuously publishes forward commands.
- The stop condition depends only on accumulated `/wheel_ticks`.
- There is no maximum run duration, stale-tick timeout, obstacle stop, or external enable validation.

**Impact:** If `/wheel_ticks` stops, is remapped incorrectly, or the encoder node fails, the test continues commanding forward motion indefinitely.

**Required fix:** Add an independent wall-clock timeout, tick freshness timeout, maximum distance/time envelope, deadman, and safety scanner stop. Abort on any missing feedback.

## C-05 — Multiple `/cmd_vel` publishers are allowed with no arbitration

**Evidence:**
- Joystick publishes `cmd_vel` in `joy_to_cmd_vel.cpp:117,222`.
- Test scripts create additional `/cmd_vel` publishers, e.g. `run_5_inverse_2m_tests.sh:96` and `run_10_forward_15s_tests.sh:74-78`.
- The Roboteq driver subscribes directly to `/cmd_vel`.

**Impact:** Joystick, test scripts, Nav2, remote computers, or stale nodes can race. The last message wins, so stop commands and motion commands can overwrite each other unpredictably.

**Required fix:** Introduce `twist_mux` or a custom command arbiter with priorities, locks, source timeouts, and a single protected output topic to the motor driver.

## C-06 — Invalid TF architecture: static `odom -> base_footprint`

**Evidence:**
- `src/teleop_pharma/launch/teleop_launch.xml:37-39` publishes `odom -> base_footprint` as a static identity transform.
- `src/roboteq_ros2_driver/src/driver_dev.cpp:424-428` leaves dynamic odometry TF unimplemented.

**Impact:** The robot can move in odometry while TF says it is stationary. RViz, SLAM Toolbox, Nav2, and sensor filters receive contradictory transforms, causing dropped messages, distorted maps, or localisation failure.

**Required fix:** Remove the static odom transform. Publish a dynamic `odom -> base_link` or `odom -> base_footprint` from the odometry source at every update.

## C-07 — `kalman_odom` is fundamentally broken but is launched by the legacy stack

**Evidence:**
- Undeclared parameters are read at `src/kalman_odom/src/kalman_odom_node.cpp:19-25`.
- All state values, including wheel radius and wheelbase, are then overwritten with zero at `33-36`.
- Division by zero occurs at `84` because `b=x_[4]=0`.
- Subscription typo: `/wheel_tick` at `38-40`, while the driver publishes `/wheel_ticks`.
- `src/teleop_pharma/launch/teleop_launch.xml:135-139` launches this node.

**Impact:** The node can crash, output NaNs, or silently publish invalid odometry. It is not safe to use for fusion or SLAM.

**Required fix:** Disable/remove it immediately. Replace it with `robot_localization` after wheel odometry and IMU interfaces are correct.

## C-08 — Container has near-host-root privileges

**Evidence:**
- `run_docker_boot.sh:34-38` and `build_and_run_docker.sh:121-132` use `--privileged`, `--network=host`, device access, and a writable host workspace mount.
- The container runs as root.
- The service script shown in the conversation uses the mutable image `tiagobarrosisr/pharmarobot:latest`.

**Impact:** A compromised ROS dependency, image, or process can access host devices and modify the mounted repository; with privileged mode the isolation boundary is largely removed.

**Required fix:** Run a non-root user, remove `--privileged`, map only required devices, use read-only mounts where possible, drop capabilities, apply AppArmor/seccomp, and pin the image by digest.

## C-09 — systemd reports success while ROS processes may be dead

**Evidence:** The pasted `pharma-minimal-nodes.service` and `pharma-lidar-nodes.service` are `Type=oneshot` with `RemainAfterExit=yes`, while their scripts use detached `docker exec -d`.

**Impact:** systemd marks the service active/exited even if `joy_linux`, `joy_to_cmdvel`, Roboteq, or LiDAR nodes crash seconds later. Restarting the container also kills all nodes while the node services may remain marked active.

**Required fix:** Run a supervised foreground process per service, or use Docker Compose with restart policies and health checks. Add `PartOf=`/`BindsTo=` relationships to the container service.

---

# High-severity findings

## H-01 — Motor commands are not validated, finite-checked, or clamped

**Evidence:** `driver_dev.cpp:202-284` trusts arbitrary `Twist` values and casts calculations directly to `int32_t`.

**Impact:** NaN, infinity, extreme values, or a malicious publisher can cause undefined/out-of-range conversions or unsafe motor commands. Open-loop power is not clamped to the documented controller range.

**Fix:** Reject non-finite values; clamp linear/angular commands and final wheel RPM/power; enforce acceleration/deceleration limits at the driver boundary.

## H-02 — No active zero command on node shutdown or exception

**Evidence:** `driver_dev.cpp:691-696` closes the serial port without sending `!S 1 0`, `!S 2 0`, `!G 1 0`, and `!G 2 0`.

**Impact:** Motion can continue until the hardware watchdog expires; if watchdog configuration failed, motion may persist longer.

**Fix:** Add a noexcept emergency-stop routine called on shutdown, SIGINT, exceptions, and destructor; verify the controller watchdog at startup.

## H-03 — Serial write errors can terminate the node

**Evidence:** `driver_dev.cpp:297-302` performs serial writes in the command callback with no exception handling or recovery.

**Impact:** USB disconnects or write errors can crash the motor node without a controlled stop or fault publication.

**Fix:** Catch serial exceptions, enter a latched fault state, send zero if possible, close/reconnect safely, and expose diagnostics.

## H-04 — Failed serial connection does not stop construction cleanly

**Evidence:** `driver_dev.cpp:179-199` sleeps after failure but does not return an error or prevent `cmdvel_setup()`/`odom_setup()` from using a closed port.

**Impact:** Startup behaviour becomes exception-driven and inconsistent; systemd may still report success.

**Fix:** Throw a clear fatal exception after bounded retries or use a lifecycle node that stays inactive until connected.

## H-05 — Malformed serial response can crash the process

**Evidence:** `driver_dev.cpp:533-540` uses `std::stoi` without catching `invalid_argument` or `out_of_range`.

**Impact:** Electrical noise, partial replies, or protocol desynchronisation can crash the motor/odometry node.

**Fix:** Strictly validate the entire response, catch conversion errors, bound values, and count/report protocol faults.

## H-06 — Serial odometry polling can block command handling

**Evidence:**
- Single-threaded executor at `driver_dev.cpp:708-712`.
- 50 ms timer at `140-141`.
- Serial timeout set to 1000 ms at `113`.
- Timer callback sleeps and reads serial at `508-530`.

**Impact:** A delayed encoder response can block `/cmd_vel` processing for up to the serial timeout, creating stale controls and jerky or delayed stops.

**Fix:** Use separate callback groups/executor threads or a dedicated serial I/O thread with bounded non-blocking reads and thread-safe command queues.

## H-07 — `flushInput()` discards controller responses and faults

**Evidence:** `driver_dev.cpp:511` clears input before every encoder query.

**Impact:** Command acknowledgements, fault/status messages, and partial responses can be discarded, hiding controller problems and causing protocol desynchronisation.

**Fix:** Implement a single protocol parser/transaction layer; never blindly flush shared controller input.

## H-08 — No controller status/fault monitoring or command acknowledgement

**Evidence:** The driver sends configuration and movement commands but never reads/decodes fault flags, status flags, command acknowledgements, STO state, or loop errors.

**Impact:** ROS can report successful commands while the controller is disabled, in STO, faulted, or rejecting closed-loop commands—the exact failure mode observed during testing.

**Fix:** Poll and publish controller diagnostics; reject motion until configuration is acknowledged and all required enable/fault states are valid.

## H-09 — Driver overwrites controller configuration at every startup without validation

**Evidence:** `driver_dev.cpp:326-413` writes MMOD, watchdog, current limits, RPM limits, acceleration, PID gains, and EPPR every start.

**Impact:** A software restart can silently replace a known-good controller configuration with incorrect values. No readback confirms success. The configured `encoder_ppr` is negative (`roboteq.yaml:13`).

**Fix:** Separate commissioning configuration from runtime control; validate/range-check values; read back every setting; fail closed if mismatched.

## H-10 — Invalid kinematic parameters are accepted and used

**Evidence:** `differential_drive_kinematics.cpp:11-13,25-27` only prints an error; it does not throw or prevent division. The runtime config intentionally uses negative encoder resolution.

**Impact:** Division by zero, inverted odometry, and invalid pose estimates can propagate into SLAM/Nav2.

**Fix:** Require positive magnitudes and separate sign parameters; refuse activation on invalid radius, wheelbase, CPR/PPR, RPM, or current limits.

## H-11 — Hidden, duplicated sign corrections make regressions likely

**Evidence:**
- `driver_dev.cpp:50` hard-codes `kCommandAngularSign=-1`.
- Wheel formulas at `212-214` also encode direction.
- Encoder directions are hard-negated at `589-590`.
- Negative CPR/PPR values are also configured in YAML.

**Impact:** Multiple sign layers can cancel or double-invert each other, causing recurring left/right regressions and incorrect odometry.

**Fix:** Define one explicit hardware convention in configuration, with unit tests for forward/left/right and encoder signs.

## H-12 — Wheel speed envelope can exceed controller limits

**Evidence:** Joystick turbo allows 1.0 m/s and 0.9 rad/s (`joy_to_cmd_vel.cpp:37-39`), while the driver adds/subtracts angular wheel speed and sets `max_rpm=100`.

**Impact:** Combined translation and rotation can command one wheel beyond configured limits, causing saturation, asymmetric response, and odometry/heading drift.

**Fix:** Apply coupled saturation: scale both wheel commands proportionally so neither exceeds maximum wheel RPM.

## H-13 — High-rate synchronous logging is in the control callback

**Evidence:**
- Every Joy message logs at INFO and writes CSV (`joy_to_cmd_vel.cpp:224-243`).
- Logger default is flush-every-row (`robot_test_logger.hpp:16-21,314-349`).

**Impact:** Disk I/O and console formatting occur in the command path, increasing jitter; logs can grow without bound and fill the filesystem.

**Fix:** Use throttled/debug logging, an asynchronous bounded queue, log rotation, and disk-space limits.

## H-14 — Stale build/install trees can execute old code

**Evidence:** The archive contains `build/`, `install/`, `log/`, nested `src/serial/build/`, backup files, and thousands of generated artifacts despite `.gitignore`.

**Impact:** Sourcing `/ros_ws/install/setup.bash` can run an older executable/config than the edited source. This directly explains repeated “changed in VS Code but old behaviour remains” incidents.

**Fix:** Remove generated artefacts from version control/distribution; perform clean reproducible builds; verify source and binary hashes/commit IDs at startup.

## H-15 — Hard-coded `/dev/ttyUSB*` mappings are unstable

**Evidence:** `roboteq.yaml:8`, `lidar_only.launch.xml:7-8`, and `teleop_launch.xml:81,98` use ttyUSB numbers.

**Impact:** Rebooting or reconnecting USB devices can swap Roboteq and LiDAR ports. A driver may open the wrong device or fail unpredictably.

**Fix:** Create udev rules and use stable `/dev/serial/by-id` or named symlinks such as `/dev/roboteq`, `/dev/lidar_front`, `/dev/lidar_back`.

## H-16 — Legacy and new launch paths can run simultaneously

**Evidence:**
- `teleop_launch.xml` launches joystick, driver, LiDARs, TF, and Kalman.
- `lidar_only.launch.xml` launches the same LiDAR/TF names.
- `teleop.launch.py` launches a second control stack.
- `start_robot.sh` and `run_docker_boot.sh` launch different stacks.

**Impact:** Duplicate node names, duplicate TFs, serial-port contention, and multiple command publishers.

**Fix:** Define one authoritative bring-up architecture and remove/deprecate legacy launch entry points.

## H-17 — Broad process killing breaks unrelated subsystems

**Evidence:** The start script shown in the conversation used `pkill -f "[r]os2 launch"`; other scripts use broad pattern-based `pkill`.

**Impact:** Restarting control can kill LiDAR/SLAM; restarting another stack can leave orphaned child processes or kill unrelated nodes.

**Fix:** Supervise process groups/PIDs or containers; never use broad `pkill -f` for production process management.

## H-18 — Malformed parameter-file command in the pasted service script

**Evidence:** The pasted minimal-node script contained `--params-file/ros_ws/...` with no separating space.

**Impact:** The driver may start with defaults or fail argument parsing, causing old wheel radius/channel mappings or missing runtime configuration.

**Fix:** Use `--params-file /ros_ws/...` and assert runtime parameter values after startup.

---

# Medium-severity findings

## M-01 — Joystick mapping and turbo activation are hard-coded

**Evidence:** `joy_to_cmd_vel.cpp:14-39,83-99` assumes specific axes/button indices and trigger polarity.

**Impact:** A different controller mapping can produce unintended motion or permanent turbo.

**Fix:** Parameters plus controller-profile validation; require an explicit deadman distinct from turbo.

## M-02 — Acceleration/deceleration depends on Joy message timing

**Evidence:** Ramping is only executed inside the Joy callback (`joy_to_cmd_vel.cpp:186-207`).

**Impact:** Different joystick publication rates change acceleration behaviour; loss of messages stops the ramp rather than ramping safely to zero.

**Fix:** Run output control on a fixed-rate timer using the latest validated target.

## M-03 — `joystick_connected_` never returns to false

**Evidence:** It is set true at `joy_to_cmd_vel.cpp:158-164` and never reset.

**Impact:** Diagnostics can say the joystick is connected after disconnection.

**Fix:** Add freshness tracking and explicit connected/disconnected state transitions.

## M-04 — Odometry output publishes zero velocity

**Evidence:** `driver_dev.cpp:667-672` sets every twist component to zero.

**Impact:** EKF, Nav2, and diagnostics receive a moving pose with zero velocity, degrading prediction and control.

**Fix:** Compute velocity from displacement and a monotonic time delta; publish realistic covariance.

## M-05 — `pub_odom_tf` is a non-functional parameter

**Evidence:** `driver_dev.cpp:424-428` contains only a TODO.

**Impact:** Operators may enable the parameter and assume a required transform exists when it does not.

**Fix:** Implement it or remove the parameter until complete.

## M-06 — Encoder normalisation is mathematically mislabeled/inconsistent

**Evidence:** `driver_dev.cpp:626-627` divides ticks by CPR but comments that it converts to radians; it omits `2π` and mixes negative CPR/sign changes.

**Impact:** Downstream users can misinterpret the field and fuse wrong units.

**Fix:** Rename to revolutions or multiply by `2π`; document sign and units precisely.

## M-07 — System clock is used for elapsed timing

**Evidence:** `driver_dev.cpp:56-61` uses `system_clock`, truncates to `uint32_t`, and wraps; the computed `dt` is currently unused.

**Impact:** NTP/time adjustments and wraparound can break future velocity calculations.

**Fix:** Use `steady_clock` and 64-bit durations.

## M-08 — Odom logger permits path traversal/arbitrary truncation

**Evidence:** `odom_test_logger.cpp:25-26,156-166` concatenates `test_name` and `log_dir` directly and opens with `std::ios::trunc` while running as root.

**Impact:** A crafted parameter can write/truncate arbitrary accessible paths in the container, including mounted host paths.

**Fix:** Sanitize test names to a safe basename, restrict logs to a fixed directory, run non-root, and use exclusive-create semantics.

## M-09 — Odom logger does not validate calibration parameters

**Evidence:** `odom_test_logger.cpp:32-38,278` divides by CPR and wheelbase without zero/finite/range checks.

**Impact:** Invalid parameters produce infinities/NaNs or crashes and contaminate calibration results.

**Fix:** Validate and fail startup on invalid values.

## M-10 — `path_length_m` is not true path length

**Evidence:** `odom_test_logger.cpp:275-286` sets path length equal to signed accumulated centre displacement.

**Impact:** Reversing or oscillating can reduce/cancel the value. True path length should accumulate `abs(center_delta_m)`.

**Fix:** Maintain separate signed displacement and non-negative travelled path length.

## M-11 — CSV measurement scripts do not escape user input

**Evidence:** `run_5_inverse_2m_tests.sh:181-208` and `run_10_forward_15s_tests.sh:96-102` interpolate input directly into Python source/CSV.

**Impact:** Commas/newlines corrupt CSV; crafted input can inject spreadsheet formulas, and quotes in `MEASURED` can inject Python code into the here-document.

**Fix:** Pass values as argv/environment, validate numeric input, and write CSV through Python’s `csv` module with formula-injection protection.

## M-12 — Test scripts continue after failures

**Evidence:** All test scripts use `set -uo pipefail` but omit `-e`.

**Impact:** Logger startup, copy, or stop failures can be ignored while the robot still moves, producing invalid records or unsafe sequences.

**Fix:** Use explicit checked steps and a trap that always publishes zero and stops the test node.

## M-13 — Calibration scripts contain stale values

**Evidence:** `run_4_forward_10s_tests.sh:39` and `run_10_forward_15s_tests.sh:10,61` use radius `0.084`, while current configuration is `0.0881`.

**Impact:** Logs and test decisions can be internally inconsistent.

**Fix:** Load one robot calibration YAML as the single source of truth.

## M-14 — Legacy boot script does not mount the workspace it claims to mount

**Evidence:** `run_docker_boot.sh:30-32` prints a workspace mount, but the `docker run` command at `34-47` has no `-v` mount and launches the legacy all-in-one stack.

**Impact:** It runs stale image code and creates duplicates, while misleading the operator about the source in use.

**Fix:** Delete or rewrite the script; never keep conflicting boot paths.

## M-15 — Docker build is non-reproducible and overbroad

**Evidence:** `Dockerfile` uses unpinned `ubuntu:22.04`, current apt repositories, a downloaded key from GitHub, `pip3 install -U`, and `ros-humble-turtlebot3*`.

**Impact:** Rebuilding at a later date can produce different binaries and a much larger attack surface.

**Fix:** Pin image digest/package versions, use a lock/SBOM, remove unused packages, and scan dependencies in CI.

## M-16 — X11 access is weakened globally

**Evidence:** `build_and_run_docker.sh:98,107` runs `xhost +local:root` and mounts the X socket.

**Impact:** Local root processes can access/control the user’s X display and capture input.

**Fix:** Do not run RViz in the privileged robot container; run RViz locally or use narrowly scoped Xauthority with a non-root container.

## M-17 — Incomplete package metadata/dependencies

**Evidence:**
- `teleop_pharma/package.xml` omits runtime dependencies used by launch files (`tf2_ros`, `sllidar_ros2`, `kalman_odom`).
- Multiple packages have `TODO` licenses/descriptions and version `0.0.0`.

**Impact:** Clean deployments can fail; licensing/compliance and traceability are inadequate for a commercial or medical product.

**Fix:** Correct package manifests, licenses, maintainers, versions, and release provenance.

## M-18 — `teleop_pharma/setup.py` conflicts with the CMake package

**Evidence:** `src/teleop_pharma/setup.py:3` declares package name `teleop`, while package.xml/CMake declare `teleop_pharma`.

**Impact:** Confusing or accidental Python installation can create package/index collisions.

**Fix:** Remove unused setup.py or make the package consistently `ament_python`.

## M-19 — Logger files can collide and have no retention policy

**Evidence:** `robot_test_logger.hpp:36-38` uses second-resolution filenames and append mode; there is no rotation/size cap or interprocess lock.

**Impact:** Simultaneous nodes can append to the same CSV, and logs can fill disk.

**Fix:** Add node name/PID/nanoseconds/UUID to filenames, asynchronous rotation, maximum retention, and write-error diagnostics.

## M-20 — Kalman update lacks frame/time integrity

**Evidence:** It consumes unstamped `Pose2D`, does not normalise yaw innovation, ignores full covariance/state transition, and publishes no child frame/twist/covariance (`kalman_odom_node.cpp:100-127`).

**Impact:** Even after crash bugs are fixed, the output is not a defensible state estimator.

**Fix:** Replace with `robot_localization` using stamped, frame-correct odometry and IMU data.

---

# Low-severity / maintainability findings

## L-01 — Misleading encoder-stream log

`driver_dev.cpp:501-502` logs that `# C_?CR_# 33` was sent, but the actual write is commented out.

## L-02 — Dead fields and unfinished safety logic

`logger_`, `last_cmd_time_`, `received_first_cmd_`, `command_timeout_logged_`, and `cmd_timeout_s_` exist in the Roboteq header but are unused.

## L-03 — Compile warnings are tolerated

Build logs repeatedly show an unused `dt`; third-party SLLIDAR builds contain numerous pedantic/unused warnings. Warnings should be reviewed and project code built with `-Werror` in CI where appropriate.

## L-04 — Repository/distribution contains generated and private operational data

The archive includes build/install/log trees, robot/odometry CSV data, a swap file, backup source, and a nested `.git` directory. This increases size, leaks operational history, and complicates provenance.

## L-05 — Vendored dependencies are not centrally pinned or inventoried

`joystick_drivers`, `serial`, and `sllidar_ros2` are copied into source; only SLLIDAR contains nested repository metadata. There is no SBOM or automated dependency vulnerability scan.

## L-06 — `open_new_terminal_docker.sh` is unsafe/broken

The script uses unquoted user-supplied container names inside `bash -c`, tests with an unanchored grep, references undefined `$i`, and ignores `NUM_SESSIONS`.

## L-07 — ROS 1 environment variables are set in a ROS 2 system

`ROS_HOSTNAME` and `ROS_MASTER_URI` in Docker scripts are irrelevant to DDS and can confuse diagnostics. Use ROS 2 variables (`ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, `RMW_IMPLEMENTATION`) explicitly.

---

# Likely reason the joystick stack became unreliable after service changes

The most probable architectural cause is not the LiDAR node itself. It is the service/process model:

1. Detached ROS processes are launched by oneshot services, so crashes are invisible to systemd.
2. Restarting/recreating `pharma_container` kills all detached control nodes, but the node services can remain `active (exited)` unless explicitly restarted.
3. Earlier minimal scripts used broad `pkill -f "ros2 launch"`, which killed unrelated launches.
4. The pasted Roboteq command had a malformed `--params-file` argument.
5. Multiple legacy launch paths can start duplicate nodes and TF publishers.
6. The default DDS graph allows remote nodes on a colleague’s computer to appear and potentially publish commands.

---

# Recommended remediation order

## Immediate safety hotfix — 1 to 2 engineering days

1. Stop using `teleop_launch.xml`, `run_docker_boot.sh`, and autonomous test scripts until guarded.
2. Add driver command timeout and active zero command on timeout/shutdown.
3. Add joystick deadman and joystick freshness timeout.
4. Add `twist_mux`; allow exactly one final command source.
5. Fix the empty optional and all parameter validation.
6. Remove static `odom -> base_footprint`.
7. Disable `kalman_odom`.

## Bring-up/service hardening — 1 to 3 days

1. Replace detached oneshot services with supervised foreground processes/Compose.
2. Use udev stable device names.
3. Remove broad `pkill` and legacy entry points.
4. Clean build/install/log artefacts and perform a clean build.
5. Add startup assertions for node count, runtime parameters, ports, fault state, and command publisher count.

## Cybersecurity baseline — 2 to 5 days

1. Dedicated ROS domain and VLAN/firewall.
2. SROS2 identities and topic permissions.
3. Non-root, non-privileged container with pinned digest and restricted devices/capabilities.
4. SBOM, dependency scanning, signed images/releases.

## Verification — approximately 1 week

1. Unit tests for joystick mapping, deadman, timeout, wheel/channel signs, and saturation.
2. Hardware-in-loop tests for USB loss, encoder loss, joystick loss, process crash, controller fault/STO, and network command injection.
3. Fault-injection acceptance criteria: robot must command zero within a defined maximum stop latency.

