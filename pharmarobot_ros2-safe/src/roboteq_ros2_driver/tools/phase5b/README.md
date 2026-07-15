# Phase 5B stop-latency validation harness

This harness is excluded from normal builds. Build it only with
`-DROBOTEQ_BUILD_HARDWARE_VALIDATION=ON`, and execute it only after separate
hardware approval and serial-ownership checks.

The executable has no non-zero motor-command path and accepts no arbitrary
serial command. Its transport wrapper permits only the exact production stop
batch (`!G 1 0`, `!G 2 0`, `!S 1 0`, `!S 2 0`), startup `?FID`, and fixed
read-only `?FF`/`?FS` diagnostic transactions. Controller settings are empty,
periodic encoder polling is deferred for 24 hours, and any unexpected write or
query aborts the run.

Output is a new mode-0600 JSONL file created with `O_EXCL`. Every record is
appended and `fsync`ed. Monotonic timestamps, correlation IDs, connection
generations, diagnostic phases, stop phases, and the serial-write acceptance
byte count are recorded. A successful stop write means the serial library/OS
write path accepted all 28 bytes; it does not prove physical UART transmission.
Every diagnostic query also emits a result record before recovery mutates the
snapshot, so incomplete or unresolved replies preserve their exact raw bytes,
hex encoding, delimiter state, and timestamps in the JSONL evidence.
Preselection and normal scenarios treat unresolved or recovered-after-timeout
diagnostic paths as explicit partial outcomes so a valid stop-latency sample is
preserved without waiting indefinitely for a normal `transaction_complete`.

Observer-triggered barriers or delays are synthetic fault injection and must
not be included in normal hardware-latency populations. The `fallback-injected`
mode overrides a successfully completed recovery result and labels its evidence
exactly `synthetic reconnect-fallback path validation`. It validates the
production reconnect-fallback path; it is not evidence that an SBL2360
naturally produced ambiguous framing.

The command line is intentionally fixed to `/dev/roboteq`, baud 115200, query
`FF` or `FS`, deadline 3 or 100 ms, one of the compiled scenario names, and one
of the compiled phase plans. Each fully expanded command and evidence path
still requires separate approval before execution.

`roboteq_phase5b_ack_characterization` is a separate fixed-mode hardware
evidence tool for command-acknowledgement characterization. It does not import
or modify the production worker or transport reply policy. The tool accepts no
arbitrary serial command. Its only write payloads are `!G 1 0\r`, `!G 2 0\r`,
`!S 1 0\r`, `!S 2 0\r`, and the exact four-command stop batch; its only
read-only follow-up queries are `?FID\r` and `?FF\r` in the compiled H3 modes.
It uses `std::chrono::steady_clock`, records exact transmitted and received raw
bytes, line completion timestamps, inter-line gaps, delimiter state, trailing
partial bytes, and whether any line completed after the H3 query write begins.
Like the stop-latency harness, it creates a new mode-0600 JSONL file with
`O_EXCL`, appends every record, and `fsync`s before continuing.
