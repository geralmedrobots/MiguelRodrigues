# RealSense D455 IMU

This package starts the official `realsense2_camera` ROS 2 wrapper with only
the D455 gyro and accelerometer enabled. Color, depth, infrared, RGBD, point
cloud, alignment, and RealSense TF publishing are disabled.

The upstream wrapper combines the two motion streams using linear
interpolation and supplies the sample timestamp. `realsense_imu` relays that
message to a stable raw topic without changing angular velocity, linear
acceleration, timestamp, covariance, or the upstream sensor frame. A sample
whose frame differs from the configured expected D455 optical frame is
rejected; the relay never relabels untransformed vectors as `base_link`.
Orientation is not estimated: orientation covariance element zero is `-1`, as
required by `sensor_msgs/msg/Imu` for unavailable orientation.

The package-local `d455_imu_processor` subscribes to that raw interface,
subtracts a stationary raw-frame gyro-bias estimate, applies one explicit
validated 3D rotation, rotates both covariance matrices, and publishes
robot-aligned measurements. It has no motor-command or safety-action output.

The runtime interfaces are:

- librealsense/ROS serial number: `146222250608`;
- ASIC/USB descriptor serial number: `151223061922`;
- raw topic: `/imu/d455/data_raw` in `d455_imu_optical_frame`;
- processed topic: `/imu/data` in `d455_imu_link`;
- diagnostics: `/diagnostics`;
- angular-velocity covariance: `0.01` from the upstream wrapper;
- linear-acceleration covariance: `0.01` from the upstream wrapper.

The launch path adds the underscore prefix required internally by
`realsense2_camera` for digit-only serial numbers. Neither acquisition nor the
processor publishes TF or invents orientation. EKF and `robot_localization`
remain disabled.

## Normal robot runtime

`robot_sensors.launch.py` is the sensor-only entry point inside the persistent
production container `pharmarobot_d455_sensor`. It starts the official wrapper
in IMU-only mode, the raw-topic relay, and `d455_imu_processor`:

```bash
ros2 launch realsense_imu robot_sensors.launch.py \
  serial_number:=146222250608
```

The processor defaults are versioned in
`config/d455_imu_processor.yaml`. The explicit provisional quaternion
`[0.5, 0.5, 0.5, 0.5]` implements the complete right-handed cyclic mapping
`raw +X -> processed +Y`, `raw +Y -> processed +Z`, and
`raw +Z -> processed +X`. The observed positive optical `Y` yaw therefore
maps to positive processed `Z`; the other axes remain marked provisional until
physical validation.

Bias estimation requires a warmup followed by a continuous stationary window.
Stationarity requires low corrected gyro norm, gravity-consistent acceleration,
and, in the normal-runtime configuration, a fresh exact-zero
`/cmd_vel/safe` sample. A missing, stale, or nonzero safe command therefore
prevents bias learning; the raw topic remains available. Motion, nonfinite
input, timestamp reset, excessive sample gap, or gravity mismatch clears the
candidate window. Initial calibration gates `/imu/data`; later bias updates
are allowed only during another complete stationary window.

Angular-velocity and linear-acceleration covariances are transformed as
`R * covariance * R-transpose`. They remain marked uncalibrated in diagnostics;
this processor does not replace upstream placeholder values with invented
confidence.

Processor diagnostics expose five independent statuses: raw-input
receipt/rejections/dropouts, processed publication rate, the configured
quaternion/matrix/frame validation, bias state and residuals, and transformed
covariance state. The processor does not query or publish TF; its transform
diagnostic explicitly records that the versioned quaternion is the current
axis authority. A later mounting/TF commissioning step must establish the
translation and confirm the provisional non-yaw axes before fusion.

The production architecture uses three distinct ownership boundaries:

- `pharmarobot_d455_sensor` is persistent, sensor-only, and independently
  restartable. It alone owns the D455 USB, video, media, IIO, and required
  sysfs paths.
- `pharma_container` owns robot control and consumes IMU topics over DDS. It
  receives no D455 device or sysfs access and does not start sensors with
  `docker exec`.
- the validation container remains ephemeral, carries the validation ownership
  label, uses `network=none`, and is always removed after a validation run.

`pharma-d455-imu.service` depends only on Docker and networking. Its normal
process is `pharma_d455_sensor_container.sh run`, which prepares the exact
production container and attaches systemd to `docker start --attach`.
Docker `--init` reaps children, while the image entrypoint uses `exec` so stop
and restart signals reach the ROS launch process. Sensor failure therefore
does not stop, recreate, or restart `pharma_container`.

The production preflight reuses the reviewed serial selector, narrow AppArmor
generator, loaded/enforcing profile verification, and exact resource manifest.
It rejects an active validation container, multiple production ownership
labels, a foreign fixed-name container, any legacy D455 resource or process in
the existing `pharma_container`, wrong physical identity, broad device access,
and configuration drift. Profile reload and stopped-container
recreation are disabled by default and require separate explicit
authorization.

Before profile handling or container startup, the production preflight also
performs a read-only census of every running container. It compares only the
currently selected D455 device paths, IIO/sysfs mounts, and narrowly named
RealSense IMU processes. The official camera wrapper is matched as a D455
process only when its command line also carries the reviewed
`__node:=d455` identity. An unrelated generic camera, other RealSense wrapper,
USB, sysfs, or ROS reference is not sufficient to match. Any running foreign
privileged container is treated as ambiguous broad D455 access. A conflict
report includes the foreign container name, full ID, labels, exact matches,
and reason, then fails without stopping, restarting, or removing any
container. Only the fixed-name production container with the complete
reviewed ownership labels may be excluded from this census.

The foreign-owner census does not infer ownership from a
`DeviceCgroupRules` entry alone: a rule without the corresponding selected
device node or mount does not prove that the device is present in that
container. Exact selected paths, concrete mounts, privileged mode, and
D455-specific processes remain the fail-closed signals.

Production prepare/start and the complete validation preflight/runtime/cleanup
workflow hold the same non-blocking host lock at
`/run/lock/pharmarobot-d455.lock`. Contention fails closed. The access probe
runs while this lock is held, so a validation container cannot race between
the production exclusion check and sensor startup. A failed operation releases
the lock only after its owned cleanup completes.

The sensor container uses `--network=host`, `--cap-drop=ALL`,
`no-new-privileges`, Docker `--init`, and the enforcing
`pharmarobot-d455-imu` profile. It never receives `/dev/roboteq`, motor serial
devices, `/dev/input`, or a broad `/dev`, `/dev/bus/usb`, or `/sys` mapping.

## Production lifecycle

Build the sensor-only image after the reviewed ROS base image is locally
available:

```bash
deployment/scripts/pharma_build_d455_sensor_image.sh
```

Inspect migration state before recreating a legacy main container:

```bash
deployment/scripts/pharma_d455_sensor_container.sh migration-check
```

An exit status of `2` means the current `pharma_container` still has legacy
D455 environment, profile, device, or sysfs markers. It is not modified.
The report includes the immutable full container ID. The updated main launcher
always fails closed in this state. Stopping and removing that exact legacy
container is a separate operator procedure requiring explicit approval; no
persistent environment setting authorizes migration.

The production sensor lifecycle is:

```bash
deployment/scripts/pharma_d455_sensor_container.sh start
deployment/scripts/pharma_d455_sensor_container.sh status
deployment/scripts/pharma_d455_sensor_container.sh logs
deployment/scripts/pharma_d455_sensor_container.sh restart
deployment/scripts/pharma_d455_sensor_container.sh stop
```

`remove` additionally requires `--authorize-remove`. A stopped owned container
whose image, DDS settings, AppArmor resource fingerprint, or selected hardware
changed is not recreated unless that individual command includes
`--authorize-recreate`. AppArmor reload likewise requires
`--authorize-profile-reload` on the individual command. These approvals are
never read from persistent service environment. Foreign containers are never
stopped, renamed, or removed. An explicitly requested `restart` may stop only
the exact immutable-recorded owned production sensor container; the subsequent
prepare phase then performs the foreign-owner census before that owned
container is started again.

The production sensor container must be stopped before a command may include
`--authorize-profile-reload`. Supplying reload authorization while it is
running fails before image verification, preflight, or `apparmor_parser`, so a
live sensor process never has its confinement policy changed underneath it.

The image build records the verified local base-image ID, pruned source
manifest hash, and resulting immutable image ID in
`/var/lib/pharmarobot/d455-sensor-image.env`. The lifecycle verifies that
trusted manifest against Docker image labels before creation. Destructive
lifecycle operations additionally require the exact full container ID and
configuration hash in
`/var/lib/pharmarobot/d455-sensor-container.json`; matching labels alone are
not sufficient ownership proof.

Immediately before creating or starting a stopped sensor container, the
production preflight rechecks the selected serial and resolved resources,
correlates current AppArmor audit output, and runs a bounded, non-ROS,
production-owned access probe under the final enforcing policy. The probe is
removed and its absence proved before the ROS entrypoint may start.

`pharma_run_sensors.sh` and `pharma_stop_sensors.sh` remain deprecated
compatibility wrappers; both route to the production lifecycle and contain no
`docker exec`.

## Production DDS and QoS contract

Both production containers use the values from `/etc/default/pharmarobot`:

- host networking;
- `ROS_DOMAIN_ID=0` by default;
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`;
- `ROS_LOCALHOST_ONLY=0`.

The production sensor container additionally requires
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. This keeps its Fast DDS participants off
private-container shared-memory transport while preserving private IPC and
AppArmor isolation. Default Fast DDS participants in `pharma_container`
support UDPv4 and remain wire-compatible in both directions. The setting is
part of the sensor container's configuration fingerprint and immutable
inspection contract; a stopped container created without it requires
separately authorized recreation. The entrypoint fails before ROS launch if
the setting is absent or changed.

The domain may be changed to another reviewed value in `0..232`, but it must
match in both containers. Host networking assumes DDS multicast is permitted
on the selected host interfaces. Validation-only domain `91`,
`ROS_LOCALHOST_ONLY=1`, and `network=none` are not production settings.

`/imu/d455/data_raw` and `/imu/data` use ROS sensor-data QoS: best effort,
volatile, keep-last depth 5. `/diagnostics` uses reliable, volatile, keep-last
depth 10. The processor's `/cmd_vel/safe` subscription uses sensor-data QoS;
a reliable or best-effort publisher is compatible with that best-effort
subscription. Initial processed output remains gated until fresh exact-zero
commands arrive over production DDS.

## Dependency provenance

Use one librealsense installation source in each runtime environment. Do not
install Intel `librealsense2-*` packages alongside ROS
`ros-humble-librealsense2` packages in the same environment.

The selected split is:

- NUC host: Intel `librealsense2-utils` and its udev rules, used for physical
  enumeration and permissions only;
- robot/validation container: `ros-humble-librealsense2`,
  `ros-humble-realsense2-camera`, and
  `ros-humble-realsense2-description` from the ROS repository.

`deployment/docker/Dockerfile.d455_sensor` installs the sensor-container
dependencies; the main robot Dockerfile intentionally does not.
The launch file also reports a clear error if `realsense2_camera` or
`rs_launch.py` is unavailable. The official wrapper applies a five-second
device startup timeout.

Installing host packages, changing udev state, building an image, creating a
container, accessing the camera, and launching ROS each require explicit
approval under this repository's safety rules.

## Host tools and udev rules

Configure the official librealsense Debian repository, then install only the
host enumeration tools:

```bash
sudo apt-get install -y apt-transport-https ca-certificates curl gnupg lsb-release
sudo mkdir -p /etc/apt/keyrings

curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
  | gpg --dearmor \
  | sudo tee /etc/apt/keyrings/librealsenseai.gpg >/dev/null

printf 'deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo %s main\n' \
  "$(lsb_release -cs)" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt-get update
sudo apt-get install -y librealsense2-utils
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo udevadm settle
```

Reconnect the D455 after the rules are installed. `librealsense2-dkms` is not
part of this IMU-only procedure; consider that kernel-changing package only if
a separately reviewed hardware error proves it necessary.

## Host hardware preflight

Confirm the USB product and serial before Docker is allowed to access it:

```bash
lsusb -d 8086:0b5c
rs-enumerate-devices -s

PYTHONPATH=src/realsense_imu \
  python3 -m realsense_imu.usb_device \
  --serial-number 146222250608 \
  --usb-serial-number 151223061922
```

The D455 reports two identifiers: librealsense uses `146222250608`, while its
USB descriptor, ASIC serial, and firmware-update ID use `151223061922`. The
selector verifies both identities. It then discovers only device nodes below
that USB parent: the USB node, its video and media nodes, and the `accel_3d`
and `gyro_3d` IIO nodes. It also verifies the two exact IIO sysfs directories
and their writable scan, trigger, buffer, and sampling controls.
If the kernel exposes the exact per-device `in_accel_hysteresis` or
`in_anglvel_hysteresis` control, the selector also requires that file to be
writable and records it for profile generation. Kernels that do not expose a
hysteresis attribute remain supported; an attribute present during selection
but missing or inaccessible during validation fails closed.

Selection fails closed for missing, ambiguous, mismatched, malformed,
non-character, or inaccessible resources. The video/media nodes allow native
librealsense to identify the D455; the launch configuration still prevents
image, depth, infrared, and point-cloud streams from starting.

## Narrow AppArmor profile

The repository-managed `realsense_imu.apparmor_profile` generator consumes the
same validated `HostResources` selection as the Docker argument generator. It
does not accept hand-entered sysfs paths. Generate the profile only while the
serial-validated D455 is connected:

```bash
PYTHONPATH=src/realsense_imu \
  python3 -m realsense_imu.apparmor_profile \
  --serial-number 146222250608 \
  --usb-serial-number 151223061922 \
  > /tmp/pharmarobot-d455-imu.apparmor
```

Generation refuses a missing or ambiguous librealsense serial, a missing or
ambiguous USB/ASIC serial, missing or duplicate `accel_3d`/`gyro_3d` devices,
an IIO path outside the selected USB parent, and any missing or inaccessible
required IIO control. A disconnect or USB reset invalidates the generated
paths; regenerate and reload the profile before recreating the container.

Validate syntax without loading the profile into the kernel:

```bash
apparmor_parser -Q -T /tmp/pharmarobot-d455-imu.apparmor
```

After Agent 2 accepts the live diff and separate approval is given, install
and load the generated enforcing profile manually:

```bash
sudo install -o root -g root -m 0644 \
  /tmp/pharmarobot-d455-imu.apparmor \
  /etc/apparmor.d/pharmarobot-d455-imu
sudo apparmor_parser -r -W /etc/apparmor.d/pharmarobot-d455-imu
```

To unload and remove that exact profile later:

```bash
sudo apparmor_parser -R /etc/apparmor.d/pharmarobot-d455-imu
sudo unlink /etc/apparmor.d/pharmarobot-d455-imu
```

The profile uses a stable read/write split. Read-only discovery is bounded to:

- `/sys/devices/system/cpu/{,**}`;
- `/sys/devices/system/node/{,**}`;
- the NUC USB-controller roots
  `/sys/devices/pci0000:00/0000:00:0d.0/usb[12]/{,**}` and
  `/sys/devices/pci0000:00/0000:00:14.0/usb[34]/{,**}`;
- `/sys/bus/platform/drivers/hid_sensor_custom/{,**}`;
- exact serial-derived USB, video, media, and IIO discovery links.

Every bounded pattern is `r` only. There is no read or write rule for all of
`/sys`, `/sys/devices`, `/sys/bus`, or a whole PCI tree. This lets librealsense
enumerate harmless CPU, NUMA, USB-topology, and HID-driver metadata without an
ever-growing list of `uevent`, `busnum`, `devnum`, `speed`, and `descriptors`
files. A different host controller topology requires code review; it must not
be handled by broadening the rule to all sysfs.

The profile's only writable `/sys` rules are the validated accel/gyro `buffer/enable`,
`buffer/length`, `trigger/current_trigger`, sampling-frequency, three axis
scan-enable, and timestamp scan-enable files, plus each sensor's exact
`in_accel_hysteresis` or `in_anglvel_hysteresis` file when the selector found
and validated it. No wildcard IIO control is emitted. The profile has no broad
`/sys/**` or `/dev/bus/usb/**` rule. Docker still receives exact device nodes
and exact major/minor cgroup rules from the selector.

`apparmor=unconfined` is forbidden because it removes this mandatory-access
boundary and restores access beyond the selected D455 sysfs controls.
`--privileged` is forbidden because it broadly expands device access and
capabilities, defeating both serial-derived device cgroups and the container's
least-privilege isolation. Neither is an acceptable permission workaround.

The USB node number can change whenever the camera disconnects or resets. If
that occurs, the USB node and HID/IIO sysfs instance may both change. The
bounded read-only discovery rules remain valid, but the exact writable IIO
rules and exact Docker device/cgroup arguments become stale. Stop and remove
the validation container, rerun the serial-validated host preflight,
regenerate/reload the AppArmor profile, and create a new container with the
newly selected resources. Never reuse stale generated write paths or Docker
arguments.

### Policy design choice

The dedicated native-backend container with bounded read-only topology rules
was selected because it preserves the already validated ROS wrapper and keeps
all write and device access serial-derived. The alternatives were rejected for
this validation step:

- RSUSB would require a separately built and validated librealsense/ROS image,
  changes the camera backend, and still needs exact USB-device mapping;
- a host-side capture node removes the container boundary and gives the process
  ordinary host access;
- a broader or privileged container defeats the write/device isolation this
  profile exists to provide.

Routine librealsense discovery should therefore need one reviewed profile
load. A camera reset or replug still requires regeneration because the strict
write paths and device identities intentionally remain exact.

## Autonomous host preflight for isolated validation

`tools/d455_host_preflight.py` is the sole host-side orchestration entry point
for a fresh isolated no-motion validation. It reuses `usb_device.py` for both
stable serial checks and current resource discovery, then reuses
`apparmor_profile.py` for profile generation. It does not duplicate discovery
inside the ROS relay or processor.

The preflight records the librealsense and USB/ASIC serials as stable identity.
Current USB bus/device numbers, `iio:deviceN` numbers, and HID suffixes such as
`.0001` or `.0002` are recorded separately as resolved transient resources.
Every run rediscovers them. Zero matches, duplicate serial matches, duplicate
motion classes, an unexpected associated IIO class, a symlink escape, an
unbounded resource set, or any motor-facing device fails closed.

Each candidate run writes a fresh evidence directory containing:

- the exact serial/resource manifest and resource fingerprint;
- the generated profile, profile SHA256, generator-template SHA256 and tool
  identity;
- the installed/kernel profile state before and after any authorized reload;
- every bounded command, argument list, timeout, exit status, stdout and
  stderr;
- Docker image/container inspection, process censuses and cleanup result;
- the existing no-motion runtime wrapper evidence when ROS validation begins.

Profile syntax is always checked before comparison or reload. An unchanged
installed profile is a no-op only when its file hash and resource manifest
match and the exact profile is loaded in enforce mode with no conflicting D455
profile. A stale profile causes an approval-required failure. The tool contains
no password automation and never invokes `sudo`; an authorized reload must run
the entire host tool through a separately approved root command. It backs up
the previous installed profile and manifest into evidence, installs
atomically, reloads, verifies enforcing identity, and restores the previous
profile if reload or post-load verification fails.

Execution also requires separate acknowledgement flags for dedicated-container
recreation, stationary D455 access, and ROS no-motion validation. The generated
container command retains `--init`, `network=none`, `cap-drop=ALL`,
`no-new-privileges`, the enforcing D455 profile, and only exact serial-derived
devices/sysfs mounts. The tool refuses to remove a same-named container unless
it carries the dedicated validation label. It verifies PID 1, isolation,
absence of Roboteq, motor serial, joystick and control processes, and performs
read/access-mode checks without writing a sysfs control before ROS launch.
The generated AppArmor profile retains its existing `network,` and
`capability,` mediation rules because removing them has not yet been validated
against the RealSense runtime. They do not independently grant host network or
capability access: the dedicated container must also pass the exact
`network=none`, `cap-drop=ALL`, `no-new-privileges`, and AppArmor security
option census. Any additional Docker security option, including
`seccomp=unconfined`, is rejected rather than treated as harmless.

Operational profile, manifest, kernel-state and validation-wrapper paths are
fixed in the package-local tool. They cannot be replaced through command-line
arguments while the tool is running as root; dependency-injected alternate
paths exist only in the offline test seam.

The separately approved runtime form is:

```bash
sudo env PYTHONPATH=src/realsense_imu \
  python3 src/realsense_imu/tools/d455_host_preflight.py \
  --image pharmarobot:realsense-imu \
  --container-name pharma_realsense_imu_validation \
  --workspace /tmp/REVIEWED_D455_WORKSPACE \
  --evidence-dir src/realsense_imu/validation_evidence/NEW_RUN_ID \
  --execute \
  --authorize-profile-reload \
  --authorize-container-recreate \
  --authorize-stationary-d455 \
  --authorize-ros-no-motion
```

That command is documentation, not standing authorization. The four real
host/runtime operations still require the explicit approval described in
`AGENTS.md`. Omit `--authorize-profile-reload` when a reload has not been
approved; an unchanged, proven enforcing profile can then proceed without a
host policy mutation, while a stale profile stops before Docker creation.

### Immutable validation workspace orchestration

`tools/d455_container_orchestrator.py` prepares the reviewed source used by
the host preflight without adding D455 discovery, AppArmor management, device
selection, or ROS launch logic of its own. It creates a fresh deterministic
snapshot that preserves repository-relative paths under `/validation_ws`.
The snapshot contains the complete `src/realsense_imu` package, excluding
validation evidence and generated build/cache artifacts, plus the explicitly
reviewed root Docker and deployment contract files needed by package tests.
Every included regular file is bounded and hashed. Missing inputs, symlinks,
special files, an unlisted contract dependency, or a concurrent mutation fail
closed before an image build.

`tools/Dockerfile.d455_validation_workspace` derives a validation image from
the exact inspected base-image digest. The build runs without network access,
performs the focused `realsense_imu` colcon build, runs the package's complete
test suite, captures verbose `colcon test-result` output, and labels the image
with both the base digest and source-manifest SHA256. Before host preflight,
the orchestrator verifies those labels, the installed workspace, the copied
manifest, and the retained test-result evidence in a temporary no-network,
capability-dropped verifier container.

Only the resulting immutable image digest and `/validation_ws` are passed to
`d455_host_preflight.py`. The host preflight remains the sole authority for
serial-derived D455 discovery, AppArmor generation/reload, exact device
scope, dedicated-container creation, stationary access, and no-motion ROS
validation. The orchestrator never publishes Twist and does not authorize
Roboteq, joystick, arbiter, motor devices, production containers, or general
network access.

An optional one-time legacy-container migration requires the exact approved
legacy name and full 64-character container ID. It proves the reviewed base
image, no-network/AppArmor/capability/device isolation, a real PID 1 reaper,
absence of zombies and ROS/control processes, and an idle process census
before stopping anything. It then renames the stopped container into a
run-specific quarantine namespace; it never deletes or restarts it. If the
new workflow fails, only the original name is restored, and stale hardware
state is never restarted. For a running legacy container, `docker-init` is
proved live as PID 1. For an already stopped container, Docker's
`HostConfig.Init=true` is only configuration evidence that an init was
requested; it is not proof of a formerly live PID 1. A stopped legacy
container without even that configuration proof is rejected.

Stop and rename results are reconciled by inspecting the exact full container
ID, name, and running state even when Docker returns nonzero or times out.
Applied renames are recorded durably before further work and rolled back when
the stopped identity and original-name availability can be proved. If the
daemon-side result cannot be established, the evidence explicitly records an
unresolved migration and the workflow fails without claiming rollback.

Execution requires distinct acknowledgements for the derived image build,
dedicated-container recreation, stationary D455 access, and ROS no-motion
validation. AppArmor reload and legacy quarantine each retain their own
separate acknowledgement. The operational form is:

```bash
python3 src/realsense_imu/tools/d455_container_orchestrator.py \
  --base-image REVIEWED_BASE_IMAGE \
  --derived-tag pharmarobot:d455-validation-workspace-new-run-id \
  --target-container pharma_realsense_imu_validation_NEW_RUN_ID \
  --evidence-dir src/realsense_imu/validation_evidence/NEW_RUN_ID \
  --execute \
  --authorize-derived-image-build \
  --authorize-container-recreate \
  --authorize-stationary-d455 \
  --authorize-ros-no-motion
```

The derived tag must be fresh and must match the dedicated
`pharmarobot:d455-validation-workspace-*` namespace. Production, general,
pre-existing, and base-image-alias tags are rejected, so failure cleanup
cannot remove an unrelated image tag. Before removing a failed-build tag,
cleanup also requires the exact run's base-digest and source-manifest labels;
a namespace-matching but unowned tag is retained and reported as a failure.

Add `--authorize-profile-reload` only after separate approval when the host
preflight proves a reload is required. Add `--legacy-name`, `--legacy-id`, and
`--authorize-legacy-quarantine` only for a separately reviewed legacy
migration. This example is not standing authorization for Docker, AppArmor,
hardware, or ROS runtime operations.

## Historical isolated Docker validation runtime

This path is retained for historical evidence reproduction. New normal-runtime
integration uses the independently supervised
`pharmarobot_d455_sensor` production container described above.

The host inventory for this D455 showed six video nodes, two media nodes, one
USB node, `accel_3d`, and `gyro_3d`. Build the image, then allow the selector to
generate one validated Docker argument per exact resource:

```bash
docker build -t pharmarobot:realsense-imu .

if docker_args_output="$(
  PYTHONPATH=src/realsense_imu \
    python3 -m realsense_imu.usb_device \
    --serial-number 146222250608 \
    --usb-serial-number 151223061922 \
    --docker-args
)"; then
  mapfile -t d455_docker_args <<< "$docker_args_output"

  docker run -d \
    --name pharma_realsense_imu_validation \
    "${d455_docker_args[@]}" \
    pharmarobot:realsense-imu
else
  printf 'D455 preflight failed; container was not created.\n' >&2
fi
```

The generated arguments enforce:

- no host networking;
- no production ROS domain;
- no `--privileged` flag;
- the named enforcing `pharmarobot-d455-imu` AppArmor profile, never
  `apparmor=unconfined`;
- no broad `/dev/bus/usb` mount or `c 189:*` device rule;
- no Roboteq, LiDAR, joystick, or `/dev/input` mapping;
- only the serial-associated D455 USB, video, media, accel-IIO, and gyro-IIO
  device nodes;
- exact bind mounts and exact major/minor device-cgroup rules for the two IIO
  nodes, whose colon-bearing names cannot use Docker's `--device` syntax;
- writable bind mounts for only the associated accel and gyro sysfs
  directories;
- dropped Linux capabilities and `no-new-privileges`.

The generated arguments never include broad `/dev` or `/sys` mounts,
`--privileged`, wildcard character-device rules, or robot devices.
Container creation must wait until the matching generated AppArmor profile is
installed and loaded; Docker will fail closed if the named profile is absent.

Run all ROS launch and inspection commands inside this container so its DDS
graph remains isolated from the robot runtime.

## Launch and capture

After separate hardware approval, launch in one terminal:

```bash
docker exec -it pharma_realsense_imu_validation bash -lc '
  source /opt/ros/humble/setup.bash
  source /ros_ws/install/setup.bash
  ros2 launch realsense_imu d455_imu.launch.py \
    serial_number:=146222250608 \
    expected_frame_id:=d455_imu_optical_frame \
    topic_name:=/imu/d455/data_raw
'
```

In a second terminal, inspect and capture ten seconds:

```bash
docker exec pharma_realsense_imu_validation bash -lc '
  source /opt/ros/humble/setup.bash
  source /ros_ws/install/setup.bash
  ros2 topic list
  ros2 topic type /imu/d455/data_raw
  ros2 topic info /realsense/d455/imu --verbose
  ros2 topic info /imu/d455/data_raw --verbose
  timeout --signal=INT 10s ros2 topic echo /imu/d455/data_raw
'
```

Coordinate the ten-second capture with the person handling the camera. Use
three deliberate intervals and keep the motion smooth enough to avoid a USB
disconnect:

- seconds 0-3: camera stationary for the gravity/noise baseline;
- seconds 3-7: gentle yaw rotation with visible nonzero angular velocity;
- seconds 7-10: camera stationary again for the post-rotation baseline.

Record the interval timing in the evidence summary. Do not claim rotation
validation from stationary sensor noise alone. Analyze the captured YAML before
accepting the run; this requires the yaw window's gyro norm and integrated
angular motion to be at least three times the larger stationary baseline and
to exceed a positive `0.05 rad/s` gyro-norm floor:

```bash
d455_imu_capture_analysis camera-imu-10s.yaml
```

Pass criteria:

- `/imu/d455/data_raw` has type `sensor_msgs/msg/Imu`;
- timestamps strictly increase and `frame_id` is
  `d455_imu_optical_frame`;
- angular velocity clearly departs from both stationary baselines during the
  deliberate gentle-yaw interval;
- stationary linear acceleration has gravity-scale magnitude;
- orientation is not populated and `orientation_covariance[0] == -1`;
- gyro and accelerometer covariances are nonzero and conservative;
- the raw combined topic has the wrapper publisher and relay subscriber;
- `/imu/d455/data_raw` has only the relay publisher, with no relay
  subscription;
- no image, depth-image, infrared-image, point-cloud, VO, SLAM, or odometry
  stream is intentionally started.

Stop immediately on a missing dependency, wrong or ambiguous device, serial
mismatch, permission error, device disconnect/reset, unexpected ROS graph,
feedback loop, image/depth/pointcloud stream, or non-monotonic timestamp.

## Evidence

Create a new append-only directory before hardware execution:

```bash
evidence_dir="src/realsense_imu/validation_evidence/d455-imu-capture-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$evidence_dir"
```

Retain branch/status, `git diff --check`, dependency versions, host and
container USB checks, sysfs/device selection, image build output, isolated
package build/test/lint logs, exact Docker and launch commands, launch output,
topic list/type/graph, representative ten-second IMU messages, and a pass/fail
summary. A failed preflight is evidence and must not be overwritten or reused
as a successful capture directory.
