# Roboteq `?CR` hardware validation — 2026-06-29

Validation ID: `roboteq-cr-20260629T143156Z`

Controller identification: `Roboteq v1.8d SBL2XXX 1/8/2018`

## Setup and scope

The drive wheels were lifted and the operator reported that the emergency stop
was accessible. The production control service was stopped and no process in
the host-visible process table or container held `/dev/roboteq` before the
diagnostic opened it. Tests used closed-loop `!S` commands at 5 or 10 RPM for at
most 1.8 seconds. Both controller channels reported `MMOD=1` before every
non-zero command. Every invocation issued all-mode zero commands before and
after its action.

No controller setting was written. Production source and configuration were not
changed during this validation.

## Operator observations

The operator reported these physical results during the run:

| Command | Physical observation |
| --- | --- |
| channel 1 `+5`, channel 2 `0` | no visible wheel movement |
| channel 1 `+10`, channel 2 `0` | left wheel forward |
| channel 1 `-10`, channel 2 `0` | left wheel backward |
| channel 1 `0`, channel 2 `+10` | right wheel forward |
| channel 1 `0`, channel 2 `-10` | right wheel backward |
| `+10`, `+10` | both wheels forward |
| `-10`, `-10` | both wheels backward |
| `-10`, `+10` | left backward, right forward |
| `+10`, `-10` | left forward, right backward |

No failure to stop was reported. The JSONL records intentionally retain
`physical_stop_confirmed=false`, because the tool cannot itself prove physical
stopping.

## Confirmed `?CR` behavior

- Exact response format observed: `CR=<signed channel 1 integer>:<signed channel
  2 integer>\r`.
- Results behave as signed counts accumulated since the previous `?CR` read,
  with the read resetting the reported interval. They are not cumulative
  lifetime positions: constant motion produced a new bounded interval value on
  each poll, and stopped polling returned to zero or small settling values.
- Positive physical-forward motion produced positive counts on the corresponding
  channel. Reverse produced negative counts. The configured encoder signs of
  `+1`, `+1` are consistent with these observations.
- The first poll after a new command can contain counts accumulated before that
  command. Direction changes occasionally produced an initial value with the
  previous sign, followed by values matching the new command. Consumers must
  treat every response as the complete interval since the preceding controller
  read, not as an instantaneous speed or cumulative position.
- The initial stationary baseline returned `CR=0:0` for all 20 polls. The final
  30 stationary polls contained four channel-1 values of magnitude one and had
  a net sum of zero; channel 2 remained zero. A fresh diagnostic process then
  returned `CR=0:0` for all 20 polls.
- The current odometry integration consumes each response directly as a wheel
  tick delta. That matches the observed reset-on-read semantics; it must not add
  another cumulative-counter differencing layer.

## Confirmed channel mapping

Observed hardware mapping:

| Controller channel | Physical wheel | Positive command/count |
| --- | --- | --- |
| 1 | left | robot-forward wheel rotation |
| 2 | right | robot-forward wheel rotation |

The production YAML currently declares the inverse mapping:

```yaml
channel_1: "right"
channel_2: "left"
```

This causes left/right encoder deltas to be exchanged and reverses angular
odometry. Straight-line distance is largely unaffected because it uses the
average of both wheels.

The current `command_angular_sign: -1` compensates for the swapped mapping in
motor command generation. A production correction should therefore be reviewed
as one coherent sign-convention change: set channel 1 to `left`, channel 2 to
`right`, and reassess `command_angular_sign` (expected `+1` to preserve the
observed physical turning behavior). Do not change only the channel names and
assume turning behavior is preserved. Validate the correction through the
normal ROS command and odometry paths with lifted wheels.

## Evidence integrity

The following motion files are valid successful evidence:

- `02-ch1-positive.jsonl`
- `02b-ch1-positive-10rpm.jsonl`
- `02d-ch1-positive-10rpm.jsonl`
- `03-ch1-negative.jsonl`
- `03b-ch1-negative.jsonl`
- `03c-ch1-negative.jsonl`
- `04b-ch2-positive.jsonl`
- `05-ch2-negative.jsonl`
- `06b-forward.jsonl`
- `07-reverse.jsonl`
- `08-left-rotation.jsonl`
- `09-right-rotation.jsonl`

Do not use these files as successful semantic evidence:

- `02c-ch1-positive-10rpm.jsonl`
- `02e-ch1-positive-10rpm.jsonl`
- `04-ch2-positive.jsonl`
- `05b-ch2-negative.jsonl`
- `06-forward.jsonl`

Their non-zero writes and bounded safety stops completed, but the diagnostic
started one final in-motion query too close to the reserved stop window. Its
pre-write drain exhausted the query deadline, so each file correctly recorded
`evidence_valid=false` and exited nonzero. The tool now guards that query-start
boundary. Offline regression tests cover both the drain-plus-margin threshold
and an integrated motion sequence that skips the final unsafe query while
completing the safety-stop write within the motion deadline.

## Not established

- Encoder counter wraparound was not tested because reaching a boundary was not
  considered safe.
- Controller power-cycle, USB reconnect, and production-worker reconnect
  behavior were not tested. Only a safe diagnostic-process restart while
  stationary was tested.
- This run did not read back every production controller setting and does not
  establish `EPPR`, maximum RPM/current, gains, acceleration, or deceleration.
- Lifted-wheel observations do not validate loaded traction, stopping distance,
  ground-path accuracy, wheel radius, wheelbase, or odometry calibration.
