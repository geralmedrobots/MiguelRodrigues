# Supervised square odometry baseline

`odometry_square_validation` runs four commanded straight segments alternating
with four commanded quarter turns. It publishes only on `/cmd_vel/test`; the
deployed `/command_arbiter` must own the subscription on that topic and the
publisher on `/cmd_vel/safe`, and `/roboteq_ros2_driver` must subscribe to the
safe topic. Bounded DDS discovery verifies that ownership before preflight.

The default square is four 2.0 m CCW legs at 0.2 m/s with 0.3 rad/s turns.
`--square-control-mode open-loop` remains the default and preserves the original
fixed-command/fixed-duration turns for raw calibration benchmarking.
Immediately before the first nonzero command, the operator must type
`EXECUTE-SQUARE`. Every segment uses the existing preflight, diagnostic,
freshness, command-forwarding, controlled-zero and stationarity guards. Any
exception aborts the unstarted segments and repeatedly publishes zero while
the safe graph is discovered, then requires a fresh downstream safe-zero
sample and the existing stationarity proof.

## Closed-loop corner control

`--square-control-mode closed-loop` changes only the four rotations. Straight
segments remain open loop, so encoder distance is still an independent
measurement rather than a control input. Each turn starts from fresh encoder
and processed `/imu/data` baselines and uses continuous relative heading:

```text
fused_delta_yaw = 0.50 * encoder_delta_yaw + 0.50 * imu_delta_yaw
```

These are explicit operator-selected control weights, not weights inferred from
the validation evidence. The target is `+pi/2` for CCW and `-pi/2` for CW. A
bounded proportional controller uses the configured square angular velocity as
its maximum, slows as error decreases, commands zero inside tolerance, and
requires an uninterrupted tolerance hold before accepting the corner. It can
reverse at the bounded correction speed after an overshoot.

The conservative defaults are 0.02 rad (1.15 degrees) yaw tolerance, 1.0/s
proportional gain, 0.08 rad/s minimum correction speed, 0.25 s tolerance hold,
10 s turn timeout, 0.20 rad encoder/IMU disagreement threshold with a 0.25 s
sustained-disagreement hold, and 0.20 rad maximum single-sample heading step.
The hold periods reject transient noise, the proportional region begins below
0.30 rad with the default maximum speed, and the timeout remains within the
existing rotation-duration safety limit. The disagreement and heading-step
limits may be tightened but cannot be configured above 0.20 rad, so the CLI
cannot disable those default fail-closed sensor guards.

Every feedback cycle retains the existing diagnostic, sensor-freshness and
command-forwarding guards. Non-finite/reset/stale headings, a timeout, or
sustained sensor disagreement aborts the remaining square and enters the
existing verified zero-command cleanup. Per-turn and aggregate
`closed_loop_yaw_feedback.csv` files record the target, encoder/IMU/fused yaw,
error, command, disagreement, timestamp, and settle state.

Stop verification latches the first fresh downstream `/cmd_vel/safe = 0`
confirmation for that stop attempt; a later repeated zero publication cannot
erase it. Final stationarity acceptance also requires the latest downstream
safe-command sample to remain within the existing stale-data limit, so the
latch cannot hide a lost command stream. Stationarity remains independently
fail-closed. Because each
emergency cleanup resets its fresh-sample and stationarity windows, square
cleanup uses `max(zero_publish_timeout_s, post_stop_settle_s)`. With the default
configuration this is 3.0 s; the tick, odometry and safe-zero thresholds are not
changed. Stop records include the first test zero, first fresh safe zero and
latency, first encoder/odometry stationary times, last detected motion, full
stationarity time, and the exact timeout condition.

Encoder stationarity uses at least five recent post-zero samples as a rolling
window spanning at least 0.35 s; additional recent samples are included when
the callback rate requires them to cover that duration. The evidence-derived
chatter envelope is the same for
both wheels: net displacement at most 1 tick, absolute cumulative excursion at
most 4 ticks, maximum individual sample at most 1 tick, and no same-direction
accumulation longer than 1 tick. With the configured 4096 ticks/revolution and
0.0881 m wheel radius, one tick is 0.135144 mm; the envelope therefore permits
at most 0.135144 mm net wheel displacement and 0.540575 mm cumulative activity
per wheel in the accepted window. These limits accept alternating isolated
`+1/-1` chatter, including one-sided chatter, but reject directional creep,
coherent coast, and excessive zero-net oscillation. The window requires fresh,
finite, monotonic encoder samples; safe zero and the existing odometry velocity
thresholds remain mandatory. Evidence records net/absolute ticks and mm,
maximum sample delta, directional accumulation, activity classification, window
timestamps, safe-command age, odometry state, and the exact acceptance reason.

A failed stop campaign can be analyzed without changing or relabeling its
source evidence:

```bash
PYTHONPATH=src/odometry_validation python3 -m \
  odometry_validation.square_stop_reprocess \
  --source SOURCE_CAMPAIGN \
  --output NEW_SIBLING_ANALYSIS_DIRECTORY \
  --comparison-campaign PREVIOUS_COMPLETED_SQUARE
```

The result is diagnostics-only and cannot be used as a complete square
baseline when motion segments are missing or invalid.

The campaign directory contains per-segment evidence plus `square_report.json`,
`segment_manifest.json`, `square_trajectory.json`, encoder and IMU trajectory
CSVs, and the complete continuous wheel, processed/raw IMU, odometry,
diagnostic and command samples. The report uses the continuous encoder
trajectory for the final x/y/yaw and path length, and processed `/imu/data` for
the separate IMU heading. Segment aggregates are retained under explicit
comparison fields.

The continuous subscriptions start before preflight. After the final
stationarity timestamp, the runner boundedly services callbacks for at most the
existing stale-data timeout until a processed `/imu/data` message timestamp
brackets that exact boundary. Reconstruction still fails closed if the first
and last IMU stamps do not bracket the square, if stamps are non-finite or
non-monotonic, or if their time range does not overlap the ROS-time trajectory.
The report and failure evidence include bounds, first/last timestamps, coverage
margins, sample counts, maximum gaps, and gaps exceeding the stale-data limit.

A completed failed campaign can be reconstructed without motion and without
overwriting its source evidence:

```bash
PYTHONPATH=src/odometry_validation python3 -m \
  odometry_validation.square_reprocess \
  --source SOURCE_CAMPAIGN \
  --output NEW_SIBLING_RECOVERY_DIRECTORY \
  --max-imu-gap-s ACTUAL_LEGACY_STALE_TIMEOUT
```

The gap argument is required only for legacy campaigns that did not record
their runtime stale-data threshold; it must be the threshold actually used by
that run. New square metadata records the value directly.

## Heading fusion decision

The evidence audit excludes TeleDex, dry runs, rejected/skipped reports and
non-rotation trials. The current valid non-TeleDex cohort has seven rotations.
Relative to commanded rotation, encoder residual mean/std are
0.0420948529/0.0711542022 rad and processed-IMU residual mean/std are
0.0121178584/0.0493193916 rad. Command is not an independent physical heading
reference, so those residuals combine actuation and estimator error. None of
the seven reports has a reviewed homogeneous independent heading reference.
Inverse-variance weights would therefore be unjustified; the runner records
encoder pose and IMU heading separately and leaves fused pose unavailable.
This does not block square execution.

## Commands

Run from the repository root in an environment with ROS 2 and this workspace
sourced. Dry-run (graph and sensor preflight, no nonzero command):

```bash
PYTHONPATH=src/odometry_validation python3 -m odometry_validation.square_trial \
  --wheel-radius-m 0.0881 \
  --track-width-m 0.453 \
  --encoder-ticks-per-revolution 4096 \
  --evidence-root src/odometry_validation/validation_evidence \
  --heading-evidence-root src/odometry_validation/validation_evidence \
  --square-side-length-m 2.0 \
  --square-linear-velocity-m-s 0.2 \
  --square-angular-velocity-rad-s 0.3 \
  --square-direction ccw \
  --square-control-mode closed-loop \
  --closed-loop-yaw-tolerance-rad 0.02 \
  --closed-loop-turn-timeout-s 10.0 \
  --closed-loop-min-angular-velocity-rad-s 0.08 \
  --closed-loop-yaw-gain 1.0 \
  --closed-loop-settle-hold-s 0.25 \
  --closed-loop-max-yaw-disagreement-rad 0.20 \
  --closed-loop-disagreement-hold-s 0.25 \
  --closed-loop-max-heading-step-rad 0.20 \
  --post-stop-settle-s 3.0 \
  --square-between-segment-pause-s 1.0 \
  --graph-discovery-timeout-s 5.0
```

Supervised physical command (do not run without a cleared 2 m square plus
stopping margin and an operator at the controls):

```bash
PYTHONPATH=src/odometry_validation python3 -m odometry_validation.square_trial \
  --execute-motion \
  --wheel-radius-m 0.0881 \
  --track-width-m 0.453 \
  --encoder-ticks-per-revolution 4096 \
  --evidence-root src/odometry_validation/validation_evidence \
  --heading-evidence-root src/odometry_validation/validation_evidence \
  --square-side-length-m 2.0 \
  --square-linear-velocity-m-s 0.2 \
  --square-angular-velocity-rad-s 0.3 \
  --square-direction ccw \
  --square-control-mode closed-loop \
  --closed-loop-yaw-tolerance-rad 0.02 \
  --closed-loop-turn-timeout-s 10.0 \
  --closed-loop-min-angular-velocity-rad-s 0.08 \
  --closed-loop-yaw-gain 1.0 \
  --closed-loop-settle-hold-s 0.25 \
  --closed-loop-max-yaw-disagreement-rad 0.20 \
  --closed-loop-disagreement-hold-s 0.25 \
  --closed-loop-max-heading-step-rad 0.20 \
  --post-stop-settle-s 3.0 \
  --square-between-segment-pause-s 1.0 \
  --graph-discovery-timeout-s 5.0
```

Do not use diagnostic ignore flags or relax thresholds to obtain a pass.
