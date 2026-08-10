# Roboteq validation tools

## D455/Roboteq rotation validation harness

> **Frozen for new motion testing.** Existing validation evidence and the
> harness are preserved for audit and checked cleanup. This harness is no
> longer the recommended path for new velocity or rotation tests. Its CLI
> rejects the nonzero `motion` stage before state creation or any Docker/ROS
> command. Nonzero motion through this harness must not be requested unless a
> later, explicit approval authorizes a separately reviewed code change to
> re-enable it. Read-only `status` and checked zero/cleanup workflows such as
> `abort` remain available under the repository's normal runtime approval
> rules.

`d455_rotation_validation.py` replaces the ad-hoc nested-heredoc rotation
procedure. It is a host-side, fail-closed orchestrator with one checked stage
per invocation. The tool does not import ROS or open a device itself, but its
`prepare`, `motion`, `finalize`, and `abort` stages execute Docker and ROS
commands. Those stages remain operator-gated hardware actions: Codex must print
the reviewed invocation for the operator to run manually. The historical
nonzero `motion` documentation below describes preserved evidence semantics;
the current CLI guard blocks it.

The harness stores its authoritative state on the host in a new per-trial
evidence directory:

- `rotation-harness-state.json` records the trial status, pinned container IDs,
  and recorder identities;
- `rotation-harness-events.jsonl` is the append-only stage/event history;
- `.rotation-harness.lock` prevents concurrent stage invocations;
- numbered `*-publisher-evidence.json` and `*-publisher.log` artifacts record
  every prepare-zero, motion, and cleanup-zero child run, including its exit
  status, requested/actual count, monotonic timing, discovered endpoints,
  per-field endpoint QoS acceptance, and the pinned recorder-QoS override;
- `robot-recorder.log`, `imu-recorder.log`, and the matching
  `*-recorder-cleanup.json` artifacts preserve each recorder's final log,
  child exit status, pinned wrapper identity, and strict reap proof before a
  cleanup stage can report success;
- `*-recorder-launch-cleanup.json` preserves strict recovery proof for a
  recorder launch that failed or was interrupted before its full identity
  could be committed;
- finalized `robot-bag/` and `imu-bag/` directories are copied to the host only
  after all final checks pass.

Recorder identity is not an unchecked PID file. For each container the state
records the Docker container ID plus the recorder PID, process-group ID,
session ID, `/proc` start time, and command-line bytes. Every later operation
revalidates that identity before signalling the complete owned process group.
Each recorder wrapper is the foreground process of a detached Docker exec,
rather than a background child of a short-lived in-container shell. The
wrapper starts rosbag, waits for that child, atomically writes its exit status,
and exits under the Docker exec parent that owns reaping. Startup waits for the
bag database, validates and durably records the identity, then acknowledges
the recorder wrapper. Before the detached launch, the state durably registers
the attempt token and its receipt, exit, bag, and log paths. The wrapper writes
an atomic PID/PGID/SID/start-time/command-line receipt before it may start
`ros2 bag record`. A failed, timed-out, or interrupted launch/identity scan is
therefore never classified as `never_started`: cleanup must either prove a
receipt-free, token-free quiescent attempt or use the receipt to prove the
pinned wrapper and its complete process group were reaped. That recovery proof
is hash-pinned in the state. Stop is bounded: `SIGINT`, then `SIGTERM`, then
`SIGKILL` if necessary. Cleanup succeeds only when the complete pinned process
group is empty and `/proc/<wrapper-pid>` is absent. A live member, any zombie
member, a PID-1-owned zombie, an unknown state, identity drift, or incomplete
identity fails closed. Before signalling a live leader, the harness pins and
revalidates its container ID, PID, process-group ID, session ID, `/proc` start
time, and nonempty exact command line. Startup-token cleanup applies the same
strictly-empty rule.

ROS setup files are sourced with shell nounset disabled. The generated shell
enables `set -u` only after all requested setup files have completed, avoiding
the Humble `AMENT_TRACE_SETUP_FILES: unbound variable` failure while retaining
strict handling for the command body.

### Staged flow

Use a fresh evidence directory and fresh, non-existing bag/log paths for each
trial. The forms below document the CLI; they do not authorize Docker, hardware
access, or motion, and placeholders must be replaced with the reviewed current
container identities and paths.

1. `prepare` pins both containers and starts the robot and IMU recorders. It
   records `/cmd_vel/joy`, `/cmd_vel/test`, `/cmd_vel/nav`, `/cmd_vel/safe`,
   `/wheel_ticks`, `/odom`, `/diagnostics`, `/tf`, and `/tf_static` in the robot
   bag, and `/camera/imu` in the IMU bag.

   The robot recorder is started first with
   `config/d455_rotation_rosbag_qos.yaml` forced through
   `--qos-profile-overrides-path`. The package-local
   `d455_twist_publisher.py` then publishes exactly 20 exact-zero `Twist`
   messages over 1 second at 20 Hz. It uses the same explicit QoS contract as
   every later publisher and does not begin until both
   `/:command_arbiter` and `/:rosbag2_recorder` are visible with compatible,
   unambiguous QoS evidence. Reliability and durability must be reported and
   match exactly. History and depth must either match exactly or be reported
   by the middleware as the known unreported sentinels `unknown` and `0`;
   those sentinels are accepted only with an explicitly recorded
   `tolerated_unreported` result, while the recorder override file is pinned
   by path and SHA-256 before the child starts. A launch gate holds the
   publisher until its container ID, PID, PGID, SID, `/proc` start time, exact
   command line, helper path, and helper SHA-256 are pinned. The child writes
   its expected PID/PGID breadcrumb as its first shell action, while the parent
   independently writes the same spawned PID/expected-PGID breadcrumb. The
   child then writes an atomic full identity receipt before waiting on the
   gate; the parent writes `GATE_RELEASE_AUTHORIZED` before it can create the
   gate. A missing gate is terminal and cannot fall through to the protected
   publisher command. The launch shell waits for and reaps the short-lived
   wrapper instead of
   orphaning it to container PID 1. After that wait, the harness requires the
   wrapper wait status and atomic exit artifact to agree at zero, rejects any
   remaining process-group member including a zombie, persists stdout/stderr
   and JSON timing evidence, and requires the active robot bag to contain
   exactly those 20 `/cmd_vel/test` messages. It does not require a completed
   publisher to retain a live `/proc/<pid>/cmdline`. Only then is the IMU
   recorder started.
   Missing or mismatched QoS, count, timing, topic delivery, identity, exit, or
   reaping evidence invalidates prepare. Finally, prepare verifies at least 10
   exact-zero `/cmd_vel/safe` samples before entering the `prepared` state.
   This discovery step cannot construct or publish a nonzero command.

   The single `/cmd_vel/test` publisher/recorder QoS contract is:

   ```yaml
   history: keep_last
   depth: 1
   reliability: reliable
   durability: volatile
   ```

   ```bash
   python3 src/roboteq_ros2_driver/tools/d455_rotation_validation.py \
     --evidence-dir TRIAL_EVIDENCE_DIR prepare \
     --trial-id TRIAL_ID \
     --robot-container ROBOT_CONTAINER \
     --imu-container IMU_CONTAINER \
     --robot-bag ROBOT_BAG_PATH --imu-bag IMU_BAG_PATH \
     --robot-log ROBOT_LOG_PATH --imu-log IMU_LOG_PATH \
     --imu-setup D455_ISOLATED_INSTALL_SETUP
   ```

   `/opt/ros/humble/setup.bash` is included by default for both containers.
   Repeat `--robot-setup` or `--imu-setup` only for additional overlays.

   Before the first motion attempt with this recorder-discovery sequence, a
   fresh operator-gated zero-motion runtime smoke is required because offline
   tests cannot prove live ROS graph discovery. In a dedicated evidence
   directory run only `prepare`, `status`, then `abort`; do not run `motion`.
   Require successful preparation, exact-zero verification, clean recorder
   shutdown, and no remaining recorder process or zombie. Preserve that smoke evidence
   and use another fresh evidence directory and fresh recorder identities for
   any later motion trial; never reuse the aborted smoke state.

2. Before `motion`, create a fresh kernel-audit artifact. It must be a regular,
   non-symlink UTF-8 file, no more than 4096 bytes, containing exactly these two
   lines and nothing else:

   ```text
   apparmor_denials=0
   d455_usb_reset_or_disconnect=0
   ```

   It must postdate trial creation and be no more than 120 seconds old. The
   harness stores its absolute path and SHA-256. Audit collection itself is an
   operator-gated host action and must use the separately reviewed bounded
   kernel-log procedure.

3. After separate approval for this trial, run `motion`. The fixed allowlist is
   `linear.x=0.0`, `angular.z` exactly one of `-0.675`, `-0.45`, `-0.30`,
   `-0.15`, `0.15`, `0.30`, `0.45`, or `0.675` rad/s, 20 Hz, and duration 2
   or 5 seconds. Both rotation directions are explicitly supported. The
   literal acknowledgement is required:

   ```bash
   python3 src/roboteq_ros2_driver/tools/d455_rotation_validation.py \
     --evidence-dir TRIAL_EVIDENCE_DIR motion \
     --linear-x 0.0 --angular-z APPROVED_SIGN \
     --duration APPROVED_DURATION --rate-hz 20 \
     --acknowledge-motion robot-clear-estop-ready \
     --kernel-audit-artifact FRESH_PRE_MOTION_AUDIT
   ```

   Before nonzero publication the harness verifies both owned recorders, live
   IMU/wheel/odom messages, recorder subscriptions, exact Roboteq diagnostic
   message text `ready` for serial and `fresh` for encoders within one coherent
   capture window, and the fresh audit. A pair in the same `DiagnosticArray` is
   accepted; a pair split across `DiagnosticArray` messages must be no more
   than 2 seconds apart, using valid ROS header stamps when both exist and
   callback monotonic receive times otherwise. A disconnected or resyncing
   serial status, stale encoder status, or other non-OK required status is
   sticky and fails the gate. Diagnostic level zero is accepted as integer zero
   or a single zero byte. The subscriber explicitly requests best-effort,
   volatile QoS so it can match either reliable or best-effort publishers;
   discovery and message receipt have separate bounded 10-second and 8-second
   windows. It also requires the live command arbiter to report the expected
   `publish_rate_hz=20.0` and `test_timeout_s=0.25` parameters. It then
   uses the same package-local publisher to send bounded exact zero, proves the
   recorder is subscribed to `/cmd_vel/test`, and verifies at least 10
   exact-zero `/cmd_vel/safe` samples. Nonzero publication cannot precede those
   gates.

   Prepare zero, nonzero motion, and every finalize/abort/failure cleanup zero
   all use `d455_twist_publisher.py`; the harness contains no `ros2 topic pub`
   publishing path and no inline `rclpy` publisher program. Each child runs in
   a uniquely tagged, process-group-owned wrapper behind a start gate. Its
   container ID, PID, PGID, SID, `/proc` start time, and exact command line are
   persisted in a child-written receipt before the gate releases, and its
   launch parent persists the gate phase and performs a bounded `wait` so the
   wrapper is reaped rather than orphaned.
   Prepare and motion wait up to 5 seconds for endpoint-specific discovery of
   both `/:command_arbiter` and `/:rosbag2_recorder`; cleanup requires both
   while the recorder is expected, but may require only the arbiter after an
   invalid state has made the recorder unavailable. Every required endpoint
   must report exact `reliable` reliability and `volatile` durability.
   `keep_last` history and depth `1` are recorded as verified when available;
   Humble/Fast DDS reports of history `unknown` and depth `0` are recorded as
   tolerated unreported metadata rather than treated as mismatches. Other
   history/depth values remain real mismatches. The recorder endpoint is not
   accepted unless the rosbag override path and SHA-256 were verified before
   publisher startup and reproduced in publisher evidence. The raw Fast DDS
   matched-subscription count and all discovered endpoint QoS details are
   retained as evidence. The raw aggregate count is telemetry only because it
   may undercount; it never authorizes or blocks publication. Only one
   unambiguous record for each required endpoint can make the publisher ready.
   A missing, duplicated, conflicting, reliability-mismatched, or
   durability-mismatched required endpoint fails before any publish.
   It then schedules exactly `duration * rate` identical nonzero `Twist`
   messages against monotonic deadlines. The publisher atomically refreshes its
   evidence after every publish, including command type, requested/actual
   count, raw monotonic/system timestamps, graph count, endpoint QoS detail,
   UTC start/first/last/end times, interval min/mean/max, and maximum schedule
   lateness. The harness adds the atomic child exit status and
   stdout/stderr/JSON artifact paths before accepting that evidence. Completion
   is accepted only when the launch-parent wait status matches that exit
   artifact, the complete evidence matches the requested command type, values,
   count, and timing, and a strict post-check proves the leader PID and every
   process-group member absent. A zombie-only publisher group is a cleanup
   failure; unlike recorder shutdown, it is never treated as sufficiently
   quiescent. Prepare failure, motion cleanup, finalize, and abort recheck every
   durably registered launch attempt before proceeding. Interruption stops an
   exact live identity when its receipt exists. A missing identity is accepted
   only when the durable phase remains `PRELAUNCH`, the gate is absent, at
   least one safe child/parent PID-and-expected-PGID breadcrumb exists, all
   available breadcrumbs agree, and the exact breadcrumb PID, expected process
   group, and separate token-bearing processes are absent. A breadcrumb PID
   that is still a zombie, a conflicting breadcrumb, or a missing breadcrumb
   fails cleanup; a token scan with no match is never sufficient by itself.
   The strict verifier excludes its own shell PID from the token scan but
   rejects every separate token-bearing process.
   Partial count/timing evidence is preserved when the gate was authorized.
   The host treats `SIGHUP` and `SIGTERM` as cleanup-aware interruptions
   (returning `128 + signal`) as well as handling `SIGINT`.
   `motion-publisher-evidence.json` is accepted only
   when its raw timestamps reproduce the summary, its count is exact, its
   requested window and message span are within 0.10 seconds, every interval
   is within 0.025 seconds of 20 Hz, and maximum deadline lateness is no more
   than 0.05 seconds.

   After every motion whose publisher is proven absent, including exceptions
   and interruption, the harness publishes bounded exact zero and verifies at
   least 10 exact-zero `/cmd_vel/safe` samples before inspecting delivery.
   If an attempted nonzero publisher cannot be proven absent, starting another
   publisher is blocked; the stage fails closed and the recorders are stopped
   without launching cleanup zero. Failure to send or prove an allowed zero
   also makes the stage fail. A standard-library SQLite/CDR check then persists
   `motion-delivery-evidence.json` and requires exactly the intended nonzero
   count and payload on `/cmd_vel/test`, with its first-to-last span within
   0.10 seconds of `(count - 1) / rate` and no inter-arrival more than
   0.025 seconds beyond the requested period. `/cmd_vel/safe` must contain only
   the matching forwarded nonzero payload, no internal zero sample between its
   first and last nonzero messages, no excessive inter-arrival, start within
   0.10 seconds, and have count, end offset, and duration consistent with the
   verified 20 Hz arbiter and 0.25-second test-source timeout. Incomplete,
   discontinuous, or mistimed publication/delivery invalidates the trial and
   cannot print `motion completed`.

   Every prepare, motion, finalize, and abort cleanup persists the recorder log
   and zero exit result, and refuses a successful status until both robot and
   IMU wrapper leaders have been reaped and both owned process groups are
   strictly empty. A `recorder_stop_failed` event is included in the terminal
   error list; therefore `abort_completed.cleanup_errors` cannot be empty when
   an owned recorder remains live, zombie, identity-drifted, or only partially
   identified.

   Partial prepare cleanup distinguishes a recorder for which no launch was
   attempted, a durably registered but incomplete attempt, and a fully
   identified started recorder. A genuine never-started recorder is neither
   container-validated nor copied as partial evidence. A registered attempt
   must have hash-pinned launch-cleanup evidence: receipt-free cleanup requires
   a bounded quiet poll proving both receipt and token absent; a receipt-bearing
   attempt requires exact receipt identity, strict group/PID absence, and its
   child exit artifact. Receipt-bearing attempts may preserve only available
   invalid partial artifacts. Started recorders retain the full identity checks
   and fail cleanup on container replacement, incomplete identity, any
   remaining live or zombie group member, a missing zero exit result, or an
   unproven reap.

4. Collect a distinct fresh post-motion audit in the same exact format, then
   run `finalize`:

   ```bash
   python3 src/roboteq_ros2_driver/tools/d455_rotation_validation.py \
     --evidence-dir TRIAL_EVIDENCE_DIR finalize \
     --kernel-audit-artifact FRESH_POST_MOTION_AUDIT
   ```

   The post-motion audit must not be the pre-motion artifact and must postdate
   motion completion. Finalization sends and verifies zero once more, stops both
   owned recorder process groups, repeats the exact motion-delivery validation
   against the closed robot bag in `final-motion-delivery-evidence.json`,
   requires nonzero counts for `/cmd_vel/test`, `/cmd_vel/safe`,
   `/wheel_ticks`, `/odom`, and `/camera/imu`, then copies both finalized bags
   to the host. Only state `complete` is valid trial evidence.

5. `status` is read-only and reports the durable host state. `abort` is the
   checked cleanup path for any non-complete harness trial:

   ```bash
   python3 src/roboteq_ros2_driver/tools/d455_rotation_validation.py \
     --evidence-dir TRIAL_EVIDENCE_DIR status

   python3 src/roboteq_ros2_driver/tools/d455_rotation_validation.py \
     --evidence-dir TRIAL_EVIDENCE_DIR abort
   ```

   For terminal `aborted` or `complete` state, read-only `status` first
   validates both started recorders' pinned cleanup JSON, final log hashes,
   zero exit status, and detached-exec reap ownership. For a recovered
   incomplete launch it instead validates the hash-pinned
   `*-recorder-launch-cleanup.json`. It fails instead of printing an unproved
   terminal state.

   `abort` is also operator-gated because it accesses Docker, ROS, Roboteq
   command topics, and recorder processes. It sends and proves zero before
   stopping owned recorders. A completed trial cannot be aborted.

Any stage failure exits nonzero and cannot print `<stage> completed`. Motion or
finalization failure marks the trial `invalid`, stops the owned recorders, and
copies recoverable bags as `partial-robot-bag/` and `partial-imu-bag/` with
`validity=invalid_partial` events. Partial bags are diagnostic evidence only;
they must never be used for sign acceptance or rotation calibration. If cleanup
or preservation also fails, those errors remain in state/events and require a
separately approved operator cleanup. Never reuse an invalid trial directory,
container identity, bag path, or recorder identity for a new trial.

Run the hardware-free focused tests with:

```bash
PYTHONDONTWRITEBYTECODE=1 \
  python3 -m unittest -v \
    src/roboteq_ros2_driver/tools/test_d455_twist_publisher.py \
    src/roboteq_ros2_driver/tools/test_d455_rotation_validation.py
```

## Roboteq `?CR` hardware-validation tool

`roboteq_cr_validation.py` is an isolated commissioning diagnostic. It does not
start ROS, import the production driver, or write controller settings. Run it
only from `pharma_container`, with the drive wheels lifted, the emergency stop
accessible, and every production Roboteq process stopped.

The tool checks `/proc` for another process holding the same character device,
takes advisory and kernel serial-exclusive locks, uses bounded reads/writes,
and creates a new JSONL evidence file. It refuses to overwrite evidence. Each
request and received byte chunk records UTC and monotonic timestamps, a
transaction ID, hexadecimal bytes, and escaped visible bytes. Use one shared
`--validation-id` for all files in a validation run and describe the intended
observation with `--annotation`.

An encoder poll counts as completed only when the transaction contains exactly
one complete `CR=` line with exactly two colon-separated signed 32-bit decimal
integers. Missing, duplicated, truncated, malformed, or out-of-range replies
invalidate the action; a prefix match alone is never accepted as encoder
evidence.

Every action starts and ends with an all-mode safety stop containing all four
zero commands (`!G` and `!S` for both channels). A safety stop records separate
write-attempt and write-complete events. A stale standalone `-` is reported but
never prevents the zero write attempt. A standalone `-` received after the zero
write is also reported only after transmission was attempted. Either rejection
then raises after the zero attempt, so it cannot permit later motion. An
incomplete or rejected zero write raises a hard failure containing `USE
OPERATOR EMERGENCY STOP`. Completed zero bytes are explicitly recorded with
`physical_stop_confirmed=false`; only operator observation can establish that
the wheel physically stopped.

Before each non-stop command write, pending serial bytes are captured as
separately labeled preexisting input. A complete standalone `-` in that drain
is treated as a delayed explicit rejection and aborts before the next non-zero
write; stale `+` is retained only as evidence and has no effect. Writes are
complete and bounded. After a write, optional response capture remains open for
the full configured `--timeout` (default 0.5 seconds), recorded as
`response_capture_timeout_s`. A complete standalone `-` in that window fails
the action immediately rather than waiting out the rest of the window. The
installed firmware is not assumed to emit `+`; presence or
absence of `+` does not prove command acceptance, and evidence uses the precise
outcome `write_complete_no_rejection_observed`.

Immediately before any non-zero `!S` write, the tool issues the same read-only
configuration syntax used by the production worker: `~MMOD 1\r` and
`~MMOD 2\r`, expecting strict `MMOD=<int>` replies. Both must equal `1`, the
closed-loop speed mode required by the current `open_loop: false` production
configuration. Missing, malformed, unknown, or mismatched mode evidence aborts
before the non-zero write. Motion always stops and captures post-stop `?CR`
polls in the same evidence file. `--duration` is measured from completion of the
non-zero serial write. Before polling, the tool reserves the complete worst-case
safety-stop drain plus full bounded write budget. The motion stop receives the
absolute motion deadline; its write cannot create a fresh timeout extending
that deadline. By the deadline, all zero bytes either completed or a hard
E-stop-required failure was recorded. A later final-cleanup retry uses its own
bounded deadline and does not satisfy or retroactively repair the motion bound.
The motion loop also refuses to start a final encoder query when only its
pre-write drain and timing margin remain; such a query could not begin reliably
and previously caused completed motion evidence to be rejected intermittently.
The tool may stop earlier on rejection, error, signal, or poll-count completion.
A motion action cannot succeed unless at least one in-motion `?CR` transaction
completed before the safety stop.

`SIGINT`, `SIGTERM`, or any internal stop request after the non-zero write is a
terminal validation abort, even if one or more encoder polls already completed.
The tool first executes the bounded safety stop, then records
`motion_action_outcome=aborted_by_stop_request`, `evidence_valid=false`, and
exits nonzero. Post-stop polling is skipped (or marked aborted if the request
arrives during it); an aborted file must not be used as successful semantic
evidence. No further motion command is issued.

To make at least one in-motion `?CR` transaction feasible, duration must be at
least `3 * timeout + poll_interval + min(0.05, timeout) + 0.10` seconds. This
conservatively reserves full command-response observation, one full encoder
transaction plus its interval, complete stop drain/write budgets, and margin.
With the default 0.5-second timeout and 0.05-second poll interval, the minimum is
1.70 seconds and the default/documented duration is 1.8 seconds. Motion is
hard-limited to 20 RPM magnitude and a requested duration of 2 seconds.

## Preconditions and abort conditions

1. Confirm the wheels remain lifted and the emergency stop is reachable.
2. Stop the production control stack and verify no ROS, host, or other-container
   process owns the physical serial device. The tool can inspect only processes
   visible inside its container. Failure to prove exclusive ownership is an
   abort condition.
3. Create a new evidence directory and run identifier. Do not reuse or overwrite
   files. Keep an observation log beside the JSONL files; for each file record
   the filename, validation ID, observed wheel, physical direction, and whether
   the emergency stop or another abort was used.
4. Abort on unexpected wheel movement, an explicit controller rejection, MMOD
   validation failure, serial timeout, inability to observe stopping, or loss
   of exclusive ownership. Use the emergency stop if software zero does not
   stop a wheel.

Example setup from `/ros_ws`:

```bash
RUN_ID="roboteq-cr-$(date -u +%Y%m%dT%H%M%SZ)"
EVIDENCE_DIR="/ros_ws/src/roboteq_ros2_driver/validation_evidence/$RUN_ID"
mkdir -p "$EVIDENCE_DIR"
TOOL="src/roboteq_ros2_driver/tools/roboteq_cr_validation.py"
COMMON="--validation-id $RUN_ID"
```

## Ordered procedure

Run only one command at a time. Do not infer channel-to-wheel or sign mapping
from configuration; record physical observations first.

1. Record firmware identification:

   ```bash
   python3 "$TOOL" $COMMON --annotation "firmware identification" \
     --output "$EVIDENCE_DIR/00-fid.jsonl" identify
   ```

2. Establish stationary behavior with repeated polling:

   ```bash
   python3 "$TOOL" $COMMON --annotation "stationary baseline; no wheel motion" \
     --output "$EVIDENCE_DIR/01-stationary.jsonl" \
     poll --count 20 --interval 0.1
   ```

3. Identify channel 1 physically. Command only channel 1 positive, stop, inspect
   its automatic post-stop polls, and record which wheel moved and its physical
   direction. Then use a separate invocation for channel 1 negative:

   ```bash
   python3 "$TOOL" $COMMON --annotation "channel 1 only, positive; mapping observation" \
     --output "$EVIDENCE_DIR/02-ch1-positive.jsonl" motion \
     --channel-1-rpm 5 --channel-2-rpm 0 --duration 1.8 \
     --count 8 --interval 0.05 --post-stop-count 10 \
     --confirm-wheels-lifted-and-estop-accessible

   python3 "$TOOL" $COMMON --annotation "channel 1 only, negative; sign observation" \
     --output "$EVIDENCE_DIR/03-ch1-negative.jsonl" motion \
     --channel-1-rpm -5 --channel-2-rpm 0 --duration 1.8 \
     --count 8 --interval 0.05 --post-stop-count 10 \
     --confirm-wheels-lifted-and-estop-accessible
   ```

4. Repeat the two separate observations for channel 2, keeping channel 1 zero:

   ```bash
   python3 "$TOOL" $COMMON --annotation "channel 2 only, positive; mapping observation" \
     --output "$EVIDENCE_DIR/04-ch2-positive.jsonl" motion \
     --channel-1-rpm 0 --channel-2-rpm 5 --duration 1.8 \
     --count 8 --interval 0.05 --post-stop-count 10 \
     --confirm-wheels-lifted-and-estop-accessible

   python3 "$TOOL" $COMMON --annotation "channel 2 only, negative; sign observation" \
     --output "$EVIDENCE_DIR/05-ch2-negative.jsonl" motion \
     --channel-1-rpm 0 --channel-2-rpm -5 --duration 1.8 \
     --count 8 --interval 0.05 --post-stop-count 10 \
     --confirm-wheels-lifted-and-estop-accessible
   ```

5. Only after the observation log establishes both channel-to-wheel mapping and
   the RPM sign that physically drives each wheel forward, derive the paired
   commands. Let `C1_FWD` and `C2_FWD` be either `5` or `-5` from those observed
   signs. Run forward, reverse, left rotation, and right rotation separately:

   - forward: `C1_FWD`, `C2_FWD`;
   - reverse: `-C1_FWD`, `-C2_FWD`;
   - left rotation: physical right wheel forward and physical left wheel reverse;
   - right rotation: physical left wheel forward and physical right wheel reverse.

   For each case use the motion form below, substitute only values derived from
   the observation log, give it a distinct filename/annotation, and inspect the
   automatic post-stop polls before continuing:

   ```bash
   python3 "$TOOL" $COMMON --annotation "DERIVED CASE; signs recorded in observations" \
     --output "$EVIDENCE_DIR/06-derived-case.jsonl" motion \
     --channel-1-rpm C1_VALUE --channel-2-rpm C2_VALUE --duration 1.8 \
     --count 8 --interval 0.05 --post-stop-count 10 \
     --confirm-wheels-lifted-and-estop-accessible
   ```

6. After all motion cases, capture repeated no-motion polls in a separate file:

   ```bash
   python3 "$TOOL" $COMMON --annotation "final stopped repeated polling" \
     --output "$EVIDENCE_DIR/10-final-stationary.jsonl" \
     poll --count 30 --interval 0.1
   ```

7. Test only a diagnostic-process restart: allow the previous invocation to
   close normally, then start another stationary poll with the same validation
   ID. Compare the last pre-restart and first post-restart raw counters. Do not
   power-cycle the controller or reconnect USB without separate approval.

   ```bash
   python3 "$TOOL" $COMMON --annotation "safe diagnostic process restart; stationary" \
     --output "$EVIDENCE_DIR/11-process-restart.jsonl" \
     poll --count 20 --interval 0.1
   ```

This procedure intentionally does not attempt encoder wraparound: reaching a
counter boundary cannot be assumed safe. Record wraparound as untested unless a
separately reviewed method can reach it without extended or high-speed motion.

Run the offline tests without a serial device:

```bash
PYTHONDONTWRITEBYTECODE=1 \
  python3 src/roboteq_ros2_driver/tools/test_roboteq_cr_validation.py
```

## Read-only diagnostic capture

`roboteq_diagnostic_capture.py` is a separate, isolated evidence tool for the
SBL2360 diagnostic investigation. It has an immutable allowlist containing
only `?FID\r`, `?FF\r`, `?FM 1\r`, `?FM 2\r`, and `?FS\r`. There is no CLI
argument for supplying a query or command. The lowest real serial-write
boundary checks the complete request bytes against the allowlist immediately
before writing. The tool never imports or changes the production driver.

Each invocation sends FID once, followed by FF, FM channel 1, FM channel 2,
and FS for every requested cycle. Responses may contain the exact request echo
and standalone `+` acknowledgement lines. A result is valid only when it has
one complete response line with the exact expected prefix. Numeric values must
be unsigned ASCII decimal in the range 0 through 255. FID must be nonempty
printable ASCII. Wrong prefixes, signs, overflow, trailing data, duplicate or
partial replies, explicit rejection, oversized input, and timeouts are invalid.

The evidence file is locked and opened in append mode. Every attempted
transaction produces one flushed and `fsync`ed JSONL record. Existing records
are retained and sequence numbers continue from the highest existing value;
an incomplete or malformed existing file is refused. Raw request, response,
pre-write drain, and post-transaction drain bytes are stored as both exact
hexadecimal and escaped visible text. Transaction times and durations are
integer nanoseconds from a monotonic clock. A successful serial open increments
the connection generation.

Before any query is written after each successful open, schema version 2 adds a
separate `startup_synchronization` record. The tool passively captures all
startup bytes, including NUL and non-ASCII values, then requires two seconds of
continuous silence after the latest byte. Each received chunk restarts that
quiet interval. Synchronization has a five-second overall deadline and a
4096-byte accepted-input cap. Deadline, cap, read, or non-monotonic-clock
failure produces an invalid record, closes the endpoint, and prevents every
query. A late read or clock overshoot cannot authorize synchronization after
the five-second bound; quiet completion exactly at the bound remains valid.
The startup record is flushed and `fsync`ed before the first query can be
written. Startup bytes are never considered a query reply.

Every tool session creates an immutable UUID in `session_id`. Every
schema-version-2 startup and transaction record includes that UUID, and every
transaction also names both its connection generation and the successful
synchronization generation that authorized it. This keeps generations
unambiguous when multiple process sessions append to one file. Closing or
reopening invalidates the previous synchronization. The recorder continues to
accept and append after existing schema-version-1 records without changing
them. Synchronization never flushes controller input and never changes DTR or
RTS. The endpoint retains its pre-existing `TIOCEXCL` exclusive-access lock;
that ioctl does not discard bytes or manipulate modem lines.

Serial responses have no transaction identifier. Preexisting bytes prevent a
write. Any post-write timeout, malformed response, or transport failure causes
a bounded evidence drain, records `close_required`, and makes the CLI exit
without sending another query. A finite drain is evidence collection, not proof
of resynchronisation.

Offline tests use an injected fake endpoint and clock and never open a serial
device:

```bash
PYTHONDONTWRITEBYTECODE=1 \
  python3 src/roboteq_ros2_driver/tools/test_roboteq_diagnostic_capture.py
```

Hardware execution requires separate approval, exclusive serial ownership,
and confirmation that the production driver is stopped. The proposed next
Phase 2 attempt is limited to one capture cycle and is documented here for
review only; do not run it during offline testing:

```bash
python3 src/roboteq_ros2_driver/tools/roboteq_diagnostic_capture.py \
  --port /dev/roboteq --baud 115200 \
  --output "$EVIDENCE_DIR/00-diagnostic-baseline.jsonl" \
  --cycles 1 --interval 1.0
```

Review the one-cycle evidence before proceeding. A ten-cycle collection is a
separate subsequent hardware action and requires separate explicit approval.

## Phase 3 timeout and resynchronisation validation

`roboteq_timeout_resync_validation.py` is a diagnostic-only Phase 3 harness.
It reuses the Phase 2 serial endpoint, strict parser, and immutable allowlist;
there is no arbitrary-query argument. Its only modes are `baseline`,
`boundary`, `reconnect`, and `bounded-resync`. The output is a newly created
schema-version-3 JSONL file. Existing output is refused, and every record is
flushed and `fsync`ed before another serial write can occur. Transaction
evidence distinguishes the intended allowlisted request from the exact prefix
actually transmitted if a low-level write times out after partial progress.

The complete response deadline starts immediately before the write and covers
both the bounded write and receipt of one complete framed response. A timeout
marks telemetry `UNKNOWN` and framing unresolved. Normal transactions are then
blocked. Boundary mode permits only a distinguishable FF-to-FS or FS-to-FF
diagnostic probe and closes afterward. Reconnect mode closes the descriptor,
increments the connection generation on open, passively captures startup
bytes, requires bounded continuous quiet, and accepts only a strictly framed
fresh synchronisation reply. Bytes isolated during startup are retained and a
reply matching the timed-out query is classified as old-generation input.

Bounded resynchronisation drains until 100 ms from the timed-out write and
requires 20 ms of continuous quiet, with an absolute 120 ms bound and a
4096-byte cap. Only complete echo, standalone `+`, and at most one strictly
valid reply to the timed-out query are classifiable. Partial, wrong, duplicate,
oversized, nonquiet, clock, or read-error input is ambiguous and forces the
reconnect fallback. A different FF/FS synchronisation query is sent only after
a clean drain. After every recovery synchronisation response, a separate
byte-capped observation requires 20 ms of continuous quiet within a hard 50 ms
bound. Any delayed, duplicate, partial, or unclassifiable byte makes framing
unresolved. Bounded recovery then reconnects; reconnect recovery fails closed.
A mismatch or failure also forces reconnect. Unresolved framing never permits
normal polling.

Offline tests never open a serial device:

```bash
PYTHONDONTWRITEBYTECODE=1 \
  python3 src/roboteq_ros2_driver/tools/test_roboteq_timeout_resync_validation.py
```

Hardware runs require the per-stage approval and ownership procedure in
`AGENTS.md`. The examples below document syntax only; do not combine them or
run a later threshold/recovery policy under an earlier approval:

```bash
python3 src/roboteq_ros2_driver/tools/roboteq_timeout_resync_validation.py \
  --port /dev/roboteq --baud 115200 --output "$NEW_EVIDENCE" \
  baseline --deadline 0.100

python3 src/roboteq_ros2_driver/tools/roboteq_timeout_resync_validation.py \
  --port /dev/roboteq --baud 115200 --output "$NEW_EVIDENCE" \
  boundary --deadline 0.030 --attempts 1

python3 src/roboteq_ros2_driver/tools/roboteq_timeout_resync_validation.py \
  --port /dev/roboteq --baud 115200 --output "$NEW_EVIDENCE" \
  reconnect --deadline APPROVED_SECONDS --attempts APPROVED_COUNT

python3 src/roboteq_ros2_driver/tools/roboteq_timeout_resync_validation.py \
  --port /dev/roboteq --baud 115200 --output "$NEW_EVIDENCE" \
  bounded-resync --deadline APPROVED_SECONDS --attempts APPROVED_COUNT
```

## Phase 5B stop-latency validation status

Phase 5B is complete for the production stop/write-acceptance path. The
validated procedure is:

1. startup drain;
2. exact four-command zero stop batch;
3. ownership of exactly four `+\r` acknowledgements;
4. post-ACK quiet verification;
5. startup `?FID\r` validation;
6. transition to `waiting_for_fresh_command`;
7. measured runtime stop with the same exact four-command zero batch;
8. ownership of exactly four `+\r` acknowledgements for that stop;
9. post-stop `?FF\r` verification;
10. fail-closed bounded recovery and reconnect on ambiguity.

The motivating failures were two production-path integration issues, not a
proven Roboteq protocol defect:

- unowned command acknowledgements could contaminate later diagnostics if the
  stop batch did not own its four `+\r` lines before a query;
- startup validation could still fail after a valid classified `FID=` reply if
  the old post-reply quiet check reached its absolute deadline without any new
  bytes.

The final production evidence batch is:

- evidence:
  `src/roboteq_ros2_driver/validation_evidence/roboteq-final-phase5b-stop-ff-20260715T134630Z/00-final-phase5b-stop-ff.jsonl`
- SHA-256:
  `d3c6750ca92b37bc540a16fff05ebf5f8fa9d54e09d924c099481b1a7a19223a`
- result: 30/30 attempts passed
- startup: clean drain, startup stop owned four ACKs, startup `?FID\r`
  succeeded, worker entered `waiting_for_fresh_command`
- measured stop: every stop owned four ACKs, every post-stop diagnostic
  returned `FF=0\r`, final framing remained synchronized
- stop-write latency min/median/p95/max: 6.477/7.321/8.137/8.166 ms

This latency means `requestStop()` to serial-library/OS write acceptance only.
It is not physical motor stop time and does not prove STO actuation. Phase 4
remains blocked until the real LiDAR/OSSD/STO safety chain is implemented and
validated separately.

## Phase 5B tools

`tools/phase5b/` contains the fixed-mode hardware-validation executables used
for the completed Phase 5B stop-latency work. They remain excluded from normal
builds and do not authorize hardware execution by themselves.

The completed Phase 5B record documents the Option E production procedure:

- bounded startup drain before any ownership-sensitive transaction;
- exact four-command startup stop batch `!G 1 0\r`, `!G 2 0\r`,
  `!S 1 0\r`, `!S 2 0\r`;
- ownership of exactly four `+\r` ACKs for that startup stop;
- post-ACK quiet verification before startup `?FID`;
- startup `?FID` validation;
- exact four-command production stop batch `!G 1 0\r`, `!G 2 0\r`,
  `!S 1 0\r`, `!S 2 0\r`;
- ownership of exactly four `+\r` ACKs for each runtime stop;
- post-ACK quiet verification before the follow-up query;
- measured runtime `requestStop()` sample;
- post-stop `?FF` verification;
- fail-closed unresolved framing and reconnect behavior that returns to
  `waiting_for_fresh_command`.

Validation progressed through H1, H2, H3, staged diagnosis, one production
regression attempt after the startup fix, and a final 30-attempt production
batch. The final evidence file is
`../validation_evidence/roboteq-final-phase5b-stop-ff-20260715T134630Z/00-final-phase5b-stop-ff.jsonl`
with SHA-256
`d3c6750ca92b37bc540a16fff05ebf5f8fa9d54e09d924c099481b1a7a19223a`. The
documented result is 30/30 passed with clean startup drain, exactly four owned
`+\r` ACKs at startup and runtime, successful startup `?FID`, synchronized
final framing, `FF=0\r` for every post-stop diagnostic, and stop latency
min/median/p95/max of 6.477/7.321/8.137/8.166 ms.

Those latency numbers are limited to `requestStop()` through full serial-write
acceptance by the OS/library path. They do not establish physical stop timing,
STO timing, or Phase 4 safety-chain behavior.
