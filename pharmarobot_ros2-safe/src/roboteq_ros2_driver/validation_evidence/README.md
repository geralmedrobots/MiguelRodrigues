# Roboteq validation evidence

Evidence in this directory is append-only. Do not edit, truncate, or replace
captured JSONL files. New validation runs must use a new timestamped directory.

## Phase 3 timeout and resynchronisation evidence

The following results were verified from the JSONL records themselves. The
files are owned by `nobody:nogroup` with mode `0600` on the host; they were read
through their existing read-only container-visible paths without changing
ownership or permissions.

### Reconnect after a 3 ms test-only timeout

- Evidence: `roboteq-phase3-reconnect-3ms-20260702T145345Z/12-reconnect-3ms.jsonl`
- SHA-256: `bcc7a3d64ecda7e1b81c1aa6e9dd09adffe35783e8f894483bf0e5ac91009471`
- `?FID` completed normally in connection generation 1.
- `?FF` timed out with a 3 ms overall deadline and telemetry was marked
  `UNKNOWN`; framing became unresolved.
- Reconnect advanced connection and synchronisation generations from 1 to 2.
- The new-generation `?FS` response was exactly `FS=129\r` and was parsed as
  valid. No delayed bytes or old-generation reply were accepted.
- The recorded reconnect recovery duration was 2,053,972,850 ns.

### Bounded resynchronisation after a 3 ms test-only timeout

- Evidence: `roboteq-phase3-bounded-resync-3ms-20260702T150219Z/13-bounded-resync-3ms.jsonl`
- SHA-256: `8b8293923294e461e7875fada7f0d88e4d97c8edd2175821b56278814d957d68`
- `?FID` completed normally in connection generation 1.
- `?FF` timed out with a 3 ms overall deadline and telemetry was marked
  `UNKNOWN`; framing became unresolved.
- The bounded delimiter drain retained the delayed reply exactly as `FF=0\r`
  (`46463D300D`) and completed in 91,809,269 ns.
- The same-generation synchronisation query returned exactly `FS=129\r` and
  was unambiguous. Post-synchronisation quiet verification found no remaining
  bytes; reconnect fallback was not required.
- The recorded bounded-resynchronisation recovery duration was 131,525,749 ns.

These are single attempts. They establish that the captured delayed `FF` reply
did not contaminate the following `FS` transaction, but they do not establish
recovery determinism. Repetitions under separately approved hardware testing
would still be required. The normal diagnostic deadline candidate remains
100 ms; 3 ms was test-only.

## Phase 5A scope

Phase 5A is entirely offline. Its serial-worker tests use a fake transport and
do not open a device, start ROS production nodes, or send controller commands.
The tests inject a bounded fake write delay and verify the exact four-command
stop batch at startup, command timeout, transport failure, and shutdown.

## Final Phase 5B stop and `?FF` evidence

The final Phase 5B batch is recorded in
`roboteq-final-phase5b-stop-ff-20260715T134630Z/00-final-phase5b-stop-ff.jsonl`
with SHA-256
`d3c6750ca92b37bc540a16fff05ebf5f8fa9d54e09d924c099481b1a7a19223a`.

This run records the completed production-observability scope for Option E
after the startup drain and query-helper fixes:

- 30/30 attempts passed.
- Startup drain clean.
- Startup stop owned exactly four `+\r` ACKs.
- Startup `?FID` validation succeeded.
- The worker reached `waiting_for_fresh_command`.
- All measured stops owned exactly four `+\r` ACKs.
- All post-stop `?FF` diagnostics returned `FF=0\r`.
- Final framing synchronized.
- Request-stop to write-accepted latency min/median/p95/max:
  6.477/7.321/8.137/8.166 ms.

Interpret those latency numbers narrowly. They measure only
`requestStop()` through full serial-library or operating-system write
acceptance of the 28-byte stop batch. They do not prove physical UART
completion, controller execution, physical stopping, LiDAR/OSSD/STO chain
behavior, or physical STO behavior. Phase 4 remains blocked pending the real
external safety chain.
