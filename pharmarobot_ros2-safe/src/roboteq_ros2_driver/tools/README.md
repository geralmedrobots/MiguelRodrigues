# Roboteq `?CR` hardware-validation tool

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
