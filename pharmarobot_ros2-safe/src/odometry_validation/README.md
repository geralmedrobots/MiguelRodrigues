# Copyright 2026 Medrobots Engineering
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Odometry validation with TeleDex reference

The default workflow is interactive and uses TeleDex/ARKit as a read-only
candidate physical reference. It starts an existing TeleDex session, captures an explicit
initial reference before a trial, polls the reference during motion and zero
command settling, and accepts a final reference only after the existing ROS
stationarity check passes. Any TeleDex failure before or during a nonzero trial
raises a fail-closed validation error and uses the existing one-owner zero
cleanup path.

The adapter loads transform validation and rigid-transform math from the
standalone `/opt/teledex_reference/teledex_logger.py` by default; it does not
implement a second TeleDex transport or publish a ROS topic. Override that
explicit root with `--teledex-reference-root` or `TELEDEX_REFERENCE_ROOT`.
Preflight requires a readable directory and logger module, the expected logger
functions, and an installed TeleDex package exposing `Session.start()`,
`Session.get_latest_data()`, and `Session.stop()`. The installed TeleDex API has
no tracking-state field or ARKit source timestamp. Evidence therefore stores
local monotonic receive time beside ROS callback-time boundaries and documents
this synchronization uncertainty. Missing, malformed, polling-failed, source
updates absent beyond the configured timeout, or reconnect-interrupted data fail
closed. When the installed callback is available, each trial boundary requires
a new WebSocket update, not a numerically changed stationary pose; receive gaps
are measured only after that fresh boundary. The adapter cannot prove ARKit
tracking quality or detect a relocalisation world-frame reset that arrives as a
plausible new pose, so such a session is not calibration ground truth without
independent corroboration.

## Laser physical reference

In interactive calibration (not `--cli-trial-mode`), select either TeleDex / ARKit
or Laser before selecting the first movement. Laser mode retains every existing
preflight, confirmation, controlled-stop, stationarity, operator-verdict, and
evidence step; it does not read a laser topic or alter robot commands.

For translation, enter initial longitudinal-wall and side-wall ranges before
preflight/motion, then final ranges only after stationarity. With +X forward and
+Y left, `dx=initial_longitudinal-final_longitudinal`; `dy=initial_side-final_side`
for a left wall, or `final_side-initial_side` for a right wall. The report preserves
`dx`, `dy`, endpoint `d_xy`, and signed reference (+`d_xy` forward, -`d_xy`
backward). `d_xy` is explicitly not travelled path length. Lateral drift and an
IMU/odometry yaw quality aid reject unsuitable trials; that yaw aid is not laser
ground truth.

For rotation, configure the selected optical origin and beam yaw in `base_link`.
The active front-laser defaults are `(0.41, 0, 0.29)` m and `180` degrees. With
the beam initially normal to a flat reference wall, the solver uses
`(d_final+a)*cos(theta)-b*sin(theta)=d_initial+a`, where `(a,b)` is the laser
origin resolved along/left of the initial beam. The commanded CW/CCW direction
selects the near-zero signed root. Invalid/impossible geometry fails closed.

```text
--reference-mode laser
--laser-side-wall left|right
--laser-origin-base-m X Y Z
--laser-beam-yaw-deg DEG
--laser-max-lateral-drift-m 0.05
--laser-max-yaw-drift-deg 1.0
```

Laser-derived wheel-radius/odometry scale values are report-only candidates
requiring repeated-trial statistics; this tool never changes wheel radius or
track width.

## Stationary TeleDex stream diagnostic

Before any future motion validation, with the AMR stationary and no validation
process running, collect a 60-second read-only stream trace. This command opens
only the TeleDex WebSocket listener; it creates no ROS node and publishes no
`/cmd_vel` command:

```bash
ros2 run odometry_validation teledex_stream_diagnostic \
  --duration-s 60 \
  --poll-rate-hz 20 \
  --teledex-reference-root /opt/teledex_reference \
  --teledex-phone-to-base-rpy-deg -90 0 0 \
  --teledex-phone-to-base-translation-m 0.1425 -0.1000 -0.0075 \
  --teledex-stale-timeout-s 0.5 \
  --output src/odometry_validation/validation_evidence/teledex-stream-stationary-$(date -u +%Y%m%dT%H%M%SZ).json
```

The JSON distinguishes WebSocket callback arrivals from adapter polls and pose
changes. Review `longest_websocket_interarrival_s`, `pose_change_age_s`,
connection state, and source timestamp/sequence availability before considering
any stale-timeout change. Keep the current `0.5 s` limit unless repeated
stationary traces justify a new, reviewed bound.

Each valid TeleDex trial writes raw `teledex_pose.csv`, per-trial `report.json`
and `report.md`, plus campaign `campaign_report.json`, `campaign_report.md`,
and `campaign_summary.csv`. TeleDex 0.0.7 returns `T_world_phone`. The adapter
forms `T_world_base = T_world_phone * T_phone_base` and then computes
`inverse(T_world_base_start) * T_world_base_current`. All reported relative
translation is therefore in the initial AMR frame: +X forward, +Y left, +Z up;
positive yaw is ROS counter-clockwise about +Z. Translation path length is the
sum of successive planar increments, not endpoint-only distance.

Configure the fixed mount with `--teledex-phone-to-base-rpy-deg ROLL PITCH YAW`
and, once measured, `--teledex-phone-to-base-translation-m X Y Z`. The current
mount is `-90 0 0` degrees: phone +X is robot +X, phone -Z is robot +Y, and
phone +Y is robot +Z. The CLI vector is the `base_link` origin expressed in
phone coordinates. The measured phone position is currently only
`(-0.1425, -0.0075, z)` in base coordinates, which converts to
`(0.1425, -z, -0.0075)` for this mount. The validated phone height is
`z=0.1000 m`, so the configured translation is
`(0.1425, -0.1000, -0.0075) m`. A measured lever arm is
required to remove phone-origin motion during pure base rotation.

The optional `--teledex-axis-validation` workflow records two separate,
operator-supervised observations: straight forward translation and a small CCW
rotation after returning to the initial pose. It identifies the discrete signed
phone forward and yaw axes, constructs a right-handed `T_phone_base` rotation,
and writes `teledex_axis_validation.json`. Ambiguous observations or a detected
mapping that differs from the explicitly selected campaign mount fail closed;
the read-only workflow never silently replaces the selected mapping. It never
commands motion and must be used only during a separately approved supervised
procedure.

The main image pins the validated `teledex==0.0.7` environment. The container
launcher mounts only the host `teledex_logger.py` source file at
`/opt/teledex_reference/teledex_logger.py` with Docker's `readonly` bind option;
it deliberately does not expose the host virtual environment. Configure the
host and container roots with `TELEDEX_REFERENCE_HOST_DIR` and
`TELEDEX_REFERENCE_CONTAINER_DIR`. Set `TELEDEX_REFERENCE_ENABLED=0` only when
starting a container that must not offer TeleDex; TeleDex-mode validation will
then fail closed at adapter preflight.
