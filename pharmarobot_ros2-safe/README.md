# PharmaRobot ROS 2 safe workspace

This workspace contains the current ROS 2 Humble control and sensing stack for
the PharmaRobot differential-drive AMR.

It intentionally contains only the components required for the current
development phase:

- joystick input through `joy_linux`;
- joystick-to-velocity conversion in `joy_to_cmdvel`;
- priority and timeout command arbitration in `command_arbiter`;
- Roboteq motor, wheel-tick, odometry, TF, and diagnostics driver in
  `roboteq_ros2_driver`;
- dual SLLIDAR drivers;
- static `base_link -> front_laser/back_laser` transforms;
- optional, explicitly disabled odometry/test loggers.

SLAM, Nav2, Kalman filtering, RealSense, SICK scanner integration, TurtleBot
packages, and legacy combined launch files are not part of the active
workspace.

## Runtime topology

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

No normal launch or service starts the optional test loggers.

Expected active nodes after both control and lidar services are running:

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

`pharma-minimal-nodes.service` is the authoritative control bring-up path. It
uses `pharma_run_control.sh`, which stops stale control launches and known child
control nodes before starting `teleop_pharma/control_only.launch.py`. The legacy
`pharma_start_minimal_nodes.sh` command is only a compatibility wrapper and
delegates to the same launcher; do not use it to create a second detached
control launch alongside the service.

## Docker workflow

Build the ROS 2 Humble image from the repository root:

```bash
./deployment/scripts/build_image.sh
```

Default image name:

```text
pharmarobot:clean
```

Rebuild the active workspace inside the running container:

```bash
docker exec pharma_container bash -lc '/ros_ws/deployment/scripts/build_core.sh'
```

For focused Roboteq driver development inside `pharma_container`:

```bash
docker exec pharma_container bash -lc '
  cd /ros_ws
  source /opt/ros/humble/setup.bash
  colcon --log-base /tmp/roboteq_doc_log build \
    --merge-install --symlink-install \
    --packages-up-to roboteq_ros2_driver \
    --build-base /tmp/roboteq_doc_build \
    --install-base /tmp/roboteq_doc_install
'
```

Run the Roboteq unit tests that cover protocol parsing, command watchdog,
serial worker behavior, odometry, TF, covariance, diagnostics, command scaling,
and parameter validation:

```bash
docker exec pharma_container bash -lc '
  cd /ros_ws
  source /opt/ros/humble/setup.bash
  source /tmp/roboteq_doc_install/setup.bash
  colcon --log-base /tmp/roboteq_doc_log test \
    --merge-install \
    --packages-select roboteq_ros2_driver \
    --build-base /tmp/roboteq_doc_build \
    --install-base /tmp/roboteq_doc_install \
    --ctest-args -R "test_command_watchdog|test_roboteq_protocol|test_serial_worker|test_serial_transport|test_command_scaling|test_command_conversion|test_roboteq_configuration|test_roboteq_diagnostics|test_roboteq_odometry|test_driver_parameter_validation|test_odom_tf|test_odom_twist|test_odom_covariance"
  colcon --log-base /tmp/roboteq_doc_log test-result \
    --test-result-base /tmp/roboteq_doc_build --verbose
'
```

Run package lint and the full package test set:

```bash
docker exec pharma_container bash -lc '
  cd /ros_ws
  source /opt/ros/humble/setup.bash
  source /tmp/roboteq_doc_install/setup.bash
  colcon --log-base /tmp/roboteq_doc_log test \
    --merge-install \
    --packages-select roboteq_ros2_driver \
    --build-base /tmp/roboteq_doc_build \
    --install-base /tmp/roboteq_doc_install \
    --event-handlers console_direct+
  colcon --log-base /tmp/roboteq_doc_log test-result \
    --test-result-base /tmp/roboteq_doc_build --verbose
'
```

Current lint status: the functional Roboteq gtests pass, but full package
lint currently reports pre-existing `cpplint` include-order failures in
`include/roboteq_ros2_driver/roboteq_ros2_driver.hpp`.

Run the software-only command arbiter integration test in an isolated ROS
domain:

```bash
docker exec pharma_container bash -lc '/ros_ws/deployment/scripts/test_command_arbiter.sh'
```

The arbiter test does not launch the Roboteq driver. It starts only the
`command_arbiter` node with localhost-only ROS graph settings.

Before staging or committing documentation/code changes, run:

```bash
git diff --check
```

## Service installation

Review `/etc/default/pharmarobot` values, especially workspace path, serial
ports, and ROS domain, then install the NUC services:

```bash
./deployment/install_services.sh
```

The container start script bind-mounts the complete host `/dev/input` directory
and permits input character devices (`c 13:* rwm`). This keeps joystick access
across reconnects and container recreation without relying on a changing
`/dev/input/eventN` path or privileged mode.

Starting or restarting these services can operate robot runtime nodes and
requires explicit hardware/runtime approval:

```bash
sudo systemctl restart pharmarobot.service
sudo systemctl restart pharma-minimal-nodes.service
sudo systemctl restart pharma-lidar-nodes.service
```

Health checks:

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

The duplicate-node command should print nothing.

## Command-source arbitration

The Roboteq driver subscribes only to the safe command output topic. Intended
producers publish to separate inputs:

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

Each configured input expires after `0.25 s` in `control_only.launch.py`. The
arbiter publishes at `20 Hz` and outputs zero when no source is fresh. This
prevents intended command sources from fighting, but it does not authenticate
publishers; ROS-domain isolation, firewalling, and SROS2 remain separate work.

## Roboteq serial I/O architecture

`roboteq_ros2_driver` separates ROS callbacks from Roboteq serial I/O:

- ROS command callback: validates `/cmd_vel/safe`, converts twist to channel
  wheel speeds, applies channel mapping and signs, then submits only the latest
  desired command to the worker.
- Command watchdog timer: tracks command age and logs timeout state; the serial
  worker owns the actual timeout stop write.
- Odometry timer: consumes the latest validated encoder sample from the worker
  and publishes `/wheel_ticks`, `/odom`, and optionally dynamic TF. It does not
  touch the serial port.
- Serial worker thread: exclusively owns the serial transport. It opens and
  closes the connection, sends startup/reconnect/timeout/shutdown stops,
  validates controller configuration by readback, validates communication with
  `?FID`, sends motor commands, polls encoders with `?CR`, handles failures,
  and schedules reconnects.

Commands use latest-command-wins semantics. There is no unbounded command
queue. The serial transport performs bounded complete writes for commands such
as `!G` and `!S`; queries require bounded validated responses and reject
malformed, rejected, truncated, oversized, stale, wrong-prefix, or timed-out
responses.

Phase 5B hardware validation is complete for the production stop/write-accept
path. The validated Option E sequence is: startup drain, exact four-command
zero stop batch, ownership of exactly four `+\r` acknowledgements, post-ACK
quiet verification, startup `?FID\r` validation, transition to
`waiting_for_fresh_command`, measured runtime stop, and post-stop `?FF\r`
diagnostic verification. The final 30-attempt evidence batch is
`src/roboteq_ros2_driver/validation_evidence/roboteq-final-phase5b-stop-ff-20260715T134630Z/00-final-phase5b-stop-ff.jsonl`
with SHA-256
`d3c6750ca92b37bc540a16fff05ebf5f8fa9d54e09d924c099481b1a7a19223a`.

## Stop, timeout, and reconnect behavior

The driver has two related timeout paths:

- The ROS watchdog timer logs when no command has been refreshed within
  `cmd_timeout_s`.
- The serial worker sends one stop when the applied command exceeds the worker
  command timeout, then invalidates the stale desired command.

The worker also sends stop commands on startup/reconnect, serial failure, and
shutdown. The stop command set is:

```text
!G 1 0
!G 2 0
!S 1 0
!S 2 0
```

On serial failure, the worker logs the failure context, attempts a stop if the
transport is still open, closes the transport, marks the connection unhealthy,
invalidates pending commands, and schedules reconnect after
`serial_reconnect_interval_s`.

With `require_fresh_command_after_reconnect: true`, motion commands submitted
before or during a disconnect are invalidated. After reconnect and validation,
the worker enters `waiting_for_fresh_command`; non-zero motion requires a fresh
post-reconnect `/cmd_vel/safe` message.

Phase 5B stop latency is measured only from `SerialIoWorker::requestStop()` to
serial-library/OS write acceptance of the complete 28-byte zero batch. It is
not physical motor stop time and does not prove UART completion, controller
execution, STO actuation, or wheel standstill.

## Odometry and TF

The Roboteq driver publishes:

- `/wheel_ticks` using encoder-derived left/right ticks;
- `/odom` using integrated differential-drive wheel odometry;
- dynamic `odom -> base_link` TF when `pub_odom_tf` is true.

The production YAML enables TF with:

```yaml
pub_odom_tf: true
odom_frame: "odom"
base_frame: "base_link"
```

The dynamic transform uses the odometry message timestamp, `odom_frame` as the
parent frame, `base_frame` as the child frame, odometry `x/y` translation, zero
`z`, and the odometry yaw quaternion.

Odometry assumptions in the implementation:

- planar differential-drive motion;
- observed pose DOFs are `x`, `y`, and `yaw`;
- observed twist DOFs are `linear.x` and `angular.z`;
- non-planar pose/twist fields are not measured by wheel odometry;
- encoder samples must parse as strict `CR=<int>:<int>` responses before they
  can update odometry.

Encoder read failures or malformed encoder responses do not publish a
valid-looking zero-motion odometry sample. They are treated as serial worker
failures: the worker marks the connection unhealthy, sends a stop if possible,
closes the transport, invalidates pending commands, and schedules reconnect.

## Odometry covariance

The `/odom` publisher fills diagonal covariance entries for
`nav_msgs/msg/Odometry`. Pose covariance is ordered as
`[x, y, z, roll, pitch, yaw]`; twist covariance is ordered as
`[linear.x, linear.y, linear.z, angular.x, angular.y, angular.z]`. The diagonal
indices are `0`, `7`, `14`, `21`, `28`, and `35`; off-diagonal entries are zero.

Production defaults from `src/roboteq_ros2_driver/config/roboteq.yaml`:

```yaml
odom_pose_covariance_x: 0.05
odom_pose_covariance_y: 0.10
odom_pose_covariance_z: 1000000.0
odom_pose_covariance_roll: 1000000.0
odom_pose_covariance_pitch: 1000000.0
odom_pose_covariance_yaw: 0.25
odom_twist_covariance_linear_x: 0.10
odom_twist_covariance_linear_y: 1000000.0
odom_twist_covariance_linear_z: 1000000.0
odom_twist_covariance_angular_x: 1000000.0
odom_twist_covariance_angular_y: 1000000.0
odom_twist_covariance_angular_z: 0.50
```

The high covariance values mark unobserved DOFs. These defaults are not
recorded as calibrated robot-specific values. Negative, NaN, or infinite
covariance parameters are sanitized to conservative defaults at odometry setup.

## Diagnostics

The Roboteq driver publishes `diagnostic_msgs/msg/DiagnosticArray` on
`/diagnostics`.

Current diagnostic statuses:

```text
roboteq/serial_connection
roboteq/command_watchdog
roboteq/encoder_freshness
roboteq/controller_faults
roboteq/controller_sto
```

Severity meanings:

- `OK`: the checked item is currently fresh/ready/normal.
- `WARN`: startup or recovery state, no first command yet, command/encoder age
  approaching its threshold, no encoder samples yet, or controller fault/STO
  state is unsupported/unavailable/unknown.
- `ERROR`: serial transport disconnected, command timed out, encoder age
  exceeded the error threshold, or a controller safety signal reports an active
  unsafe state.

Publication behavior:

- A periodic heartbeat is published according to `diagnostics_publish_rate_hz`
  after the diagnostics state has already been published once.
- State changes publish immediately, including connection transitions, fresh
  command recovery, timeout transitions, and encoder freshness transitions.
- Published diagnostic records are also logged at severity mapped from the
  diagnostic status level.

Thread-safety notes:

- Serial worker status and encoder samples are protected by the worker mutex.
- Command age state is protected by the driver command-state mutex.
- Diagnostic publication fingerprint/timing state is protected by a dedicated
  diagnostics mutex.
- The serial worker invokes a diagnostics callback on important state changes
  so recovery and failure transitions do not wait only for the periodic timer.

Fault and STO handling are diagnostic-only today. The driver exposes
`roboteq/controller_faults` and `roboteq/controller_sto`, but the current state
defaults to `unsupported` because the controller fault/STO registers are not
polled by this implementation. These diagnostics do not gate motion yet.

## Important Roboteq parameters

Production values live in `src/roboteq_ros2_driver/config/roboteq.yaml`.

```yaml
cmdvel_topic: "/cmd_vel/safe"
odom_topic: "odom"
port: "/dev/roboteq"
baud: 115200
open_loop: false
cmd_timeout_s: 0.5
wheel_radius: 0.0881
wheelbase: 0.453
encoder_ppr: 1024
encoder_cpr: 4096
encoder_eppr: -1024
motor_sign_1: 1
motor_sign_2: 1
encoder_sign_1: 1
encoder_sign_2: 1
command_angular_sign: -1
max_amps: 5.0
max_rpm: 100
channel_1: "right"
channel_2: "left"
serial_read_timeout_ms: 50
serial_write_timeout_ms: 50
serial_transaction_timeout_ms: 500
serial_max_response_bytes: 256
serial_reconnect_interval_s: 1.0
encoder_poll_period_ms: 50
require_fresh_command_after_reconnect: true
diagnostics_publish_rate_hz: 1.0
encoder_freshness_warn_s: 0.25
encoder_freshness_error_s: 1.0
```

The node default for `serial_transaction_timeout_ms` is `100`; the production
YAML overrides it to `500`.

Safety-critical parameters are validated before ROS publishers/subscribers,
timers, serial transport, or the serial worker are created. Invalid values abort
node construction and cannot open the serial transport or send controller
commands.

## Controller startup validation

Runtime startup validates required Roboteq controller settings by readback
instead of silently recommissioning persistent controller configuration. The
worker sends conservative stop commands first, then queries settings derived
from ROS parameters and driver constants, including `MMOD`, `ALIM`, `MXRPM`,
`MAC`, `MDEC`, `KP`, `KI`, `KD`, `EPPR`, `ECHOF`, and `RWD`.

Each readback must parse as `<setting>=<int>` and match the expected value. If
a setting is missing, malformed, unreadable, rejected, or mismatched, startup
logs the failing setting, sends a stop where possible, closes the serial port,
and prevents normal runtime motion.

Commissioning tools must set persistent Roboteq parameters before this node is
started.

## Hardware validation checklist

The repository contains automated tests and staged hardware evidence. Phase 5B
now has a completed production serial-stop validation record, but that result
is narrower than full robot safety validation. Do not claim physical stop or
system-level safety behavior from Phase 5B alone.

Run hardware validation only with explicit hardware approval, wheels lifted and
secured where appropriate, the area clear, and the physical emergency stop/STO
path verified:

1. Start the normal approved control stack and confirm a single Roboteq node is
   present.
2. Confirm startup stop is sent, controller configuration readback succeeds,
   `/diagnostics` reports serial readiness, and `roboteq/controller_faults` and
   `roboteq/controller_sto` are understood as unsupported diagnostic signals
   unless separate STO/fault polling has been added.
3. Send a small forward command through the normal `/cmd_vel/safe` chain and
   confirm both wheels rotate in the expected direction.
4. Send a small reverse command and a small turn command and confirm channel
   mapping, signs, and command scaling are correct.
5. Stop refreshing commands and confirm the timeout stop occurs and motion does
   not resume without a fresh command.
6. Observe `/wheel_ticks`, `/odom`, and dynamic `odom -> base_link` TF while
   wheels turn; check direction, scale, timestamps, and plausible twist.
7. Disconnect or power-cycle the Roboteq serial device only under an approved
   safe lifted-wheel procedure. Confirm the driver logs the failure, reports
   diagnostics state changes, reconnects stopped, and does not replay stale
   pre-disconnect motion.
8. Send a fresh post-reconnect command through the normal safety chain and
   confirm motion resumes only after that fresh command.
9. Confirm diagnostics heartbeat timing, immediate state-change/recovery
   publication, command timeout severity, encoder freshness severity, and
   serial connection severity.

Record controller model, firmware, serial number, robot configuration,
calibration date, exact parameter file, test commands, observed results, and
operator approval with the hardware evidence.

## Persistent USB device names

The runtime uses stable names rather than enumeration-dependent `/dev/ttyUSB*`
paths:

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

## Optional test/logging tools

The logger packages are excluded by `COLCON_IGNORE` and are not built during
normal operation.

To enable them for a controlled calibration session:

```bash
./tools/optional_tests/build_optional_test_nodes.sh
```

See `tools/optional_tests/README.md`.

Historical CSV files may exist under `odom_test_logs/` and `robot_logs/`, but
they are not copied into the Docker image and are not started by any service.

## Known limitations and unfinished work

- Phase 5B is complete for the production serial stop/write-acceptance path,
  but it does not establish physical STO behavior or wheel stop time.
- Phase 4 remains blocked until the real LiDAR/OSSD/STO safety chain is
  completed and separately validated on hardware.
- Controller fault and STO state are published as diagnostics but are not
  polled from Roboteq registers and do not gate motion.
- ROS graph authentication/authorization is not implemented; SROS2 and network
  hardening remain separate work.
- The command arbiter prevents expected command-source conflicts but does not
  authenticate publishers.
- Odometry covariance defaults are conservative placeholders, not calibrated
  robot-specific measurements.
- Dynamic `odom -> base_link` TF is implemented and enabled in production YAML,
  but the live TF graph still requires robot validation before SLAM use.
- The driver validates controller configuration at startup but does not provide
  commissioning tools for writing persistent Roboteq settings.
- Full Roboteq package lint currently fails `cpplint` include-order checks in
  `include/roboteq_ros2_driver/roboteq_ros2_driver.hpp`.
- The software command-arbiter integration assertions pass in the container,
  but the wrapper may need manual cleanup if the background test node does not
  exit after SIGINT.
- Do not use this workspace for unattended or autonomous operation until the
  safety items in `PRIORITY_PLAN.md` are closed and hardware-tested.
