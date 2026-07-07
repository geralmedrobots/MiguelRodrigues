#!/usr/bin/env python3
"""Deterministic Phase 3 tests; no test opens a serial device."""

import argparse
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


TOOLS = Path(__file__).parent
sys.path.insert(0, str(TOOLS))
MODULE_PATH = TOOLS / "roboteq_timeout_resync_validation.py"
SPEC = importlib.util.spec_from_file_location("roboteq_timeout_resync_validation", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class FakeClock:
    def __init__(self):
        self.now = 1_000_000_000

    def monotonic_ns(self):
        return self.now

    def sleep(self, seconds):
        self.now += int(seconds * 1e9)


class MemoryRecorder:
    def __init__(self, events=None):
        self.items = []
        self.events = events

    def record(self, item):
        if self.events is not None:
            self.events.append("record:" + item["record_type"])
        copied = json.loads(json.dumps(item))
        copied["sequence"] = len(self.items) + 1
        self.items.append(copied)
        return copied


class FakeEndpoint:
    """Responses are (delay-after-write-ns, bytes), one script per write."""

    def __init__(self, clock, scripts=(), startup_by_open=None, events=None,
                 read_error_at=None, write_error=None, open_error=None,
                 backward_at=None, partial_write_prefix=None,
                 overshoot_reads=None):
        self.clock = clock
        self.scripts = list(scripts)
        self.startup_by_open = dict(startup_by_open or {})
        self.events = events
        self.read_error_at = read_error_at
        self.write_error = write_error
        self.open_error = open_error
        self.backward_at = backward_at
        self.partial_write_prefix = partial_write_prefix
        self.overshoot_reads = dict(overshoot_reads or {})
        self.last_write_prefix = b""
        self.pending = []
        self.writes = []
        self.open_count = 0
        self.read_count = 0
        self.closed = True

    def open(self):
        if self.open_error is not None:
            raise self.open_error
        self.open_count += 1
        self.closed = False
        for delay, data in self.startup_by_open.get(self.open_count, []):
            self.pending.append((self.clock.now + delay, data))
        self.pending.sort()
        if self.events is not None:
            self.events.append("open")

    def close(self):
        self.closed = True
        self.pending.clear()  # Kernel input from the old descriptor is discarded.
        if self.events is not None:
            self.events.append("close")

    def write(self, data, deadline_ns):
        if data not in MODULE.phase2.ALLOWED_REQUESTS:
            raise ValueError("forbidden")
        if self.closed:
            raise OSError("closed")
        if self.write_error is not None:
            raise self.write_error
        if self.events is not None:
            self.events.append("write:" + data.decode("ascii").strip())
        self.writes.append(data)
        if self.partial_write_prefix is not None:
            self.last_write_prefix = self.partial_write_prefix
            self.clock.now = deadline_ns
            raise TimeoutError("write_timeout")
        self.last_write_prefix = data
        self.clock.now += 1_000_000
        if self.clock.now > deadline_ns:
            raise TimeoutError("write_timeout")
        script = self.scripts.pop(0) if self.scripts else []
        for delay, chunk in script:
            self.pending.append((self.clock.now + delay, chunk))
        self.pending.sort()

    def read(self, maximum, deadline_ns):
        self.read_count += 1
        if self.read_count in self.overshoot_reads:
            delta, data = self.overshoot_reads[self.read_count]
            self.clock.now = deadline_ns + delta
            return data
        if self.backward_at == self.read_count:
            self.clock.now -= 1
            return b""
        if self.read_error_at == self.read_count:
            raise OSError("injected")
        if self.pending and self.pending[0][0] <= deadline_ns:
            when, data = self.pending.pop(0)
            self.clock.now = max(self.clock.now, when)
            if len(data) > maximum:
                self.pending.insert(0, (self.clock.now, data[maximum:]))
                data = data[:maximum]
            return data
        self.clock.now = max(self.clock.now, deadline_ns)
        return b""


def ok(name, value=b"0", delay=1_000_000):
    spec = MODULE.phase2.QUERY_SPECS[name]
    payload = (b"Roboteq v1.8d SBL2360" if name == "FID" else value)
    return [(delay, spec.prefix + payload + b"\r")]


class Fixture:
    def __init__(self, scripts=(), endpoint_kwargs=None, harness_kwargs=None, events=None):
        self.clock = FakeClock()
        self.recorder = MemoryRecorder(events)
        self.endpoint = FakeEndpoint(self.clock, scripts, events=events,
                                     **(endpoint_kwargs or {}))
        options = dict(startup_quiet_ns=2_000_000, startup_deadline_ns=5_000_000,
                       drain_horizon_ns=10_000_000, drain_absolute_ns=20_000_000,
                       drain_quiet_ns=2_000_000, max_drain_bytes=32,
                       post_sync_quiet_ns=2_000_000,
                       post_sync_absolute_ns=5_000_000,
                       max_post_sync_bytes=32)
        options.update(harness_kwargs or {})
        self.harness = MODULE.TimeoutResyncValidation(
            self.endpoint, self.recorder, self.clock, **options)

    def open(self):
        item = self.harness.open_and_synchronize()
        if not item["valid"]:
            raise AssertionError(item)


class AllowlistAndEvidenceTest(unittest.TestCase):
    def test_arbitrary_query_is_rejected_before_write(self):
        fixture = Fixture()
        fixture.open()
        with self.assertRaises(ValueError):
            fixture.harness.transaction("G", 10_000_000)
        self.assertEqual(fixture.endpoint.writes, [])

    def test_parser_has_only_fixed_modes_and_no_query_argument(self):
        parser = MODULE.build_parser()
        with self.assertRaises(SystemExit):
            parser.parse_args(["--port", "x", "--output", "y", "boundary",
                               "--deadline", "0.03", "--query", "!G 1 1"])
        for mode in ("baseline", "boundary", "reconnect", "bounded-resync"):
            argv = ["--port", "x", "--output", "y", mode]
            if mode != "baseline":
                argv += ["--deadline", "0.03"]
            self.assertEqual(parser.parse_args(argv).mode, mode)

    def test_new_evidence_refuses_existing_and_fsyncs_each_record(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "evidence.jsonl"
            recorder = MODULE.NewEvidenceRecorder(path)
            with mock.patch.object(MODULE.os, "fsync") as fsync:
                recorder.record({"schema_version": 3, "record_type": "x"})
                self.assertEqual(fsync.call_count, 1)
            recorder.close()
            with self.assertRaises(FileExistsError):
                MODULE.NewEvidenceRecorder(path)

    def test_startup_record_is_durable_before_first_write(self):
        events = []
        fixture = Fixture(scripts=[ok("FID")], events=events)
        fixture.open()
        fixture.harness.transaction("FID", 100_000_000)
        self.assertLess(events.index("record:recovery_action"), events.index("write:?FID"))

    def test_open_failure_is_recorded_without_a_generation(self):
        fixture = Fixture(endpoint_kwargs={"open_error": OSError("fail")})
        item = fixture.harness.open_and_synchronize()
        self.assertFalse(item["valid"])
        self.assertEqual(item["abort_reason"], "serial_open_failed")
        self.assertEqual(item["connection_generation"], 0)

    def test_real_endpoint_wrapper_retains_partial_os_write_prefix(self):
        clock = FakeClock()
        endpoint = MODULE.ObservedBoundedSerialEndpoint("unused", 115200, clock)
        endpoint.fd = 9
        with mock.patch.object(MODULE.select, "select",
                               side_effect=[([], [9], []), ([], [], [])]), \
                mock.patch.object(MODULE.os, "write", return_value=2):
            with self.assertRaises(TimeoutError):
                endpoint.write(b"?FF\r", clock.now + 10_000_000)
        self.assertEqual(endpoint.last_write_prefix, b"?F")


class TransactionBoundaryTest(unittest.TestCase):
    def _transaction(self, response_delay):
        fixture = Fixture(scripts=[[ (response_delay, b"FF=0\r") ]])
        fixture.open()
        return fixture.harness.transaction("FF", 10_000_000), fixture

    def test_complete_reply_at_d_minus_one_d_and_d_plus_one(self):
        # One millisecond is consumed by the bounded write.
        for delay, valid in ((8_000_000, True), (9_000_000, True), (10_000_000, False)):
            with self.subTest(delay=delay):
                item, _ = self._transaction(delay)
                self.assertEqual(item["valid"], valid)
                self.assertEqual(item["telemetry_state"], "VALID" if valid else "UNKNOWN")

    def test_split_complete_and_partial_timeout(self):
        fixture = Fixture(scripts=[[(2_000_000, b"FF="), (4_000_000, b"1\r")]])
        fixture.open()
        self.assertTrue(fixture.harness.transaction("FF", 10_000_000)["valid"])
        fixture = Fixture(scripts=[[(2_000_000, b"FF=")]])
        fixture.open()
        item = fixture.harness.transaction("FF", 10_000_000)
        self.assertEqual(item["error"], "partial_reply_timeout")

    def test_wrong_prefix_duplicate_and_oversized_are_invalid(self):
        cases = (
            ([(1, b"FS=0\r")], "wrong_prefix"),
            ([(1, b"FF=0\rFF=1\r")], "ambiguous_reply"),
            ([(1, b"x" * 257)], "oversized_reply"),
        )
        for script, expected in cases:
            with self.subTest(expected=expected):
                fixture = Fixture(scripts=[script])
                fixture.open()
                item = fixture.harness.transaction("FF", 10_000_000)
                self.assertFalse(item["valid"])
                self.assertEqual(item["error"], expected)

    def test_echo_and_ack_are_accepted_only_with_one_reply(self):
        fixture = Fixture(scripts=[[(1, b"?FF\r+\rFF=3\r")]])
        fixture.open()
        self.assertTrue(fixture.harness.transaction("FF", 10_000_000)["valid"])

    def test_transport_failure_and_unresolved_blocks_next_normal_write(self):
        fixture = Fixture(endpoint_kwargs={"write_error": OSError("fail")})
        fixture.open()
        item = fixture.harness.transaction("FF", 10_000_000)
        self.assertIn("transport_error", item["error"])
        with self.assertRaises(RuntimeError):
            fixture.harness.transaction("FS", 10_000_000)
        self.assertEqual(fixture.endpoint.writes, [])

    def test_partial_write_timeout_records_intended_and_transmitted_bytes(self):
        fixture = Fixture(endpoint_kwargs={"partial_write_prefix": b"?F"})
        fixture.open()
        item = fixture.harness.transaction("FF", 10_000_000)
        self.assertEqual(item["error"], "write_timeout")
        self.assertEqual(bytes.fromhex(item["request_bytes"]["hex"]), b"?FF\r")
        self.assertEqual(bytes.fromhex(item["transmitted_request_bytes"]["hex"]), b"?F")
        self.assertEqual(item["telemetry_state"], "UNKNOWN")

    def test_nonmonotonic_read_is_invalid_and_blocks_next_write(self):
        fixture = Fixture(scripts=[ok("FF")], endpoint_kwargs={"backward_at": 2})
        fixture.open()  # startup consumes read #1
        item = fixture.harness.transaction("FF", 10_000_000)
        self.assertEqual(item["error"], "nonmonotonic_clock")
        with self.assertRaises(RuntimeError):
            fixture.harness.transaction("FS", 10_000_000)


class LateReplyAndReconnectTest(unittest.TestCase):
    def test_late_ff_before_fs_is_never_accepted_as_fs(self):
        fixture = Fixture(scripts=[[], ok("FS")])
        fixture.open()
        timed = fixture.harness.transaction("FF", 5_000_000)
        self.assertTrue(timed["timeout"])
        fixture.endpoint.pending.append((fixture.clock.now + 1, b"FF=7\r"))
        probe = fixture.harness.timeout_probe("FS")
        self.assertFalse(probe["valid"])
        self.assertEqual(probe["parsed_value"], None)
        self.assertEqual(fixture.recorder.items[-1]["error"], "delayed_or_cross_query_reply")

    def test_late_ff_and_fs_same_read_is_ambiguous(self):
        fixture = Fixture(scripts=[[], [(1, b"FF=1\rFS=2\r")]])
        fixture.open()
        fixture.harness.transaction("FF", 5_000_000)
        probe = fixture.harness.timeout_probe("FS")
        self.assertFalse(probe["valid"])

    def test_reply_after_probe_deadline_is_not_consumed_or_accepted(self):
        fixture = Fixture(scripts=[[], [(110_000_000, b"FS=1\r")]])
        fixture.open()
        fixture.harness.transaction("FF", 5_000_000)
        probe = fixture.harness.timeout_probe("FS")
        self.assertTrue(probe["timeout"])
        self.assertFalse(probe["valid"])

    def test_reconnect_increments_generations_and_isolates_startup(self):
        scripts = [[], ok("FS")]
        fixture = Fixture(
            scripts=scripts,
            endpoint_kwargs={"startup_by_open": {2: [(1, b"Starting ...\r")]}})
        fixture.open()
        fixture.harness.transaction("FF", 5_000_000)
        result = fixture.harness.reconnect_recovery("FS")
        self.assertTrue(result["valid"])
        self.assertEqual(fixture.harness.connection_generation, 2)
        self.assertEqual(fixture.harness.synchronization_generation, 2)
        startup = [x for x in fixture.recorder.items if x.get("action") == "startup_synchronization"][-1]
        self.assertEqual(bytes.fromhex(startup["startup_bytes"]["hex"]), b"Starting ...\r")
        self.assertFalse(result["reply_belonged_to_old_generation"])

    def test_old_reply_in_new_generation_startup_is_recorded_not_accepted(self):
        fixture = Fixture(
            scripts=[[], ok("FS")],
            endpoint_kwargs={"startup_by_open": {2: [(1, b"FF=9\r")]}})
        fixture.open()
        fixture.harness.transaction("FF", 5_000_000)
        result = fixture.harness.reconnect_recovery("FS")
        self.assertTrue(result["valid"])
        self.assertTrue(result["reply_belonged_to_old_generation"])
        startup = [x for x in fixture.recorder.items
                   if x.get("action") == "startup_synchronization"][-1]
        self.assertTrue(startup["reply_belonged_to_old_generation"])
        sync = [x for x in fixture.recorder.items
                if x.get("purpose") == "reconnect_sync_query"][-1]
        self.assertEqual(sync["query_name"], "FS")
        self.assertEqual(sync["parsed_value"], 0)

    def test_reconnect_rejects_old_reply_arriving_after_fresh_sync(self):
        fresh_then_old = [(1_000_000, b"FS=0\r"), (2_000_000, b"FF=9\r")]
        fixture = Fixture(scripts=[[], fresh_then_old])
        fixture.open()
        fixture.harness.transaction("FF", 5_000_000)
        result = fixture.harness.reconnect_recovery("FS")
        self.assertFalse(result["valid"])
        self.assertEqual(fixture.harness.framing_state, "unresolved")
        self.assertEqual(fixture.endpoint.pending, [])
        verification = [x for x in fixture.recorder.items
                        if x.get("action") == "post_sync_quiet_verification"][-1]
        self.assertEqual(verification["post_sync_classification"],
                         "delayed_timed_out_reply")
        self.assertTrue(verification["reply_belonged_to_old_generation"])

    def test_reconnect_rejects_duplicate_partial_and_unclassifiable_post_sync_bytes(self):
        for extra, classification in (
                (b"FS=0\r", "duplicate_sync_reply"),
                (b"FF=", "partial_line"),
                (b"junk\r", "unclassifiable_line")):
            with self.subTest(classification=classification):
                script = [(1_000_000, b"FS=0\r"), (2_000_000, extra)]
                fixture = Fixture(scripts=[[], script])
                fixture.open()
                fixture.harness.transaction("FF", 5_000_000)
                result = fixture.harness.reconnect_recovery("FS")
                self.assertFalse(result["valid"])
                self.assertEqual(fixture.harness.framing_state, "unresolved")
                verification = [x for x in fixture.recorder.items
                                if x.get("action") == "post_sync_quiet_verification"][-1]
                self.assertIn(classification, verification["post_sync_classification"])

    def test_post_sync_empty_read_overshoot_fails_closed(self):
        fixture = Fixture(
            scripts=[[], ok("FS")],
            endpoint_kwargs={"overshoot_reads": {5: (10_000_000, b"")}})
        fixture.open()
        fixture.harness.transaction("FF", 5_000_000)
        result = fixture.harness.reconnect_recovery("FS")
        self.assertFalse(result["valid"])
        self.assertEqual(fixture.harness.framing_state, "unresolved")
        verification = [x for x in fixture.recorder.items
                        if x.get("action") == "post_sync_quiet_verification"][-1]
        self.assertEqual(verification["error"], "post_sync_absolute_deadline_overshoot")

    def test_post_sync_late_chunk_overshoot_is_retained_and_fails_closed(self):
        fixture = Fixture(
            scripts=[[], ok("FS")],
            endpoint_kwargs={"overshoot_reads": {5: (10_000_000, b"FF=6\r")}})
        fixture.open()
        fixture.harness.transaction("FF", 5_000_000)
        result = fixture.harness.reconnect_recovery("FS")
        self.assertFalse(result["valid"])
        self.assertEqual(fixture.harness.framing_state, "unresolved")
        verification = [x for x in fixture.recorder.items
                        if x.get("action") == "post_sync_quiet_verification"][-1]
        self.assertEqual(bytes.fromhex(verification["post_drain_bytes"]["hex"]), b"FF=6\r")
        self.assertEqual(verification["error"], "post_sync_absolute_deadline_overshoot")


class BoundedResynchronisationTest(unittest.TestCase):
    def _timed_fixture(self, drain_chunks=(), sync_script=None, **kwargs):
        fixture = Fixture(scripts=[[], sync_script if sync_script is not None else ok("FS")],
                          harness_kwargs=kwargs)
        fixture.open()
        fixture.harness.transaction("FF", 5_000_000)
        for delay, data in drain_chunks:
            fixture.endpoint.pending.append((fixture.clock.now + delay, data))
        fixture.endpoint.pending.sort()
        return fixture

    def test_clean_delayed_reply_then_distinguishable_sync_succeeds(self):
        fixture = self._timed_fixture([(1, b"FF=4\r")])
        result = fixture.harness.bounded_resync_recovery("FS")
        self.assertTrue(result["valid"])
        self.assertEqual(result["final_framing_state"], "synchronized")
        self.assertEqual(bytes.fromhex(result["bytes_received_after_timeout"]["hex"]), b"FF=4\r")
        sync = [x for x in fixture.recorder.items
                if x.get("purpose") == "bounded_resync_query"][-1]
        self.assertEqual(sync["final_framing_state"], "drained")

    def test_echo_ack_and_one_reply_are_clean(self):
        fixture = self._timed_fixture([(1, b"?FF\r+\rFF=4\r")])
        self.assertTrue(fixture.harness.bounded_resync_recovery("FS")["valid"])

    def test_partial_wrong_duplicate_and_cap_force_reconnect(self):
        cases = (
            b"FF=4", b"FS=4\r", b"FF=4\rFF=5\r", b"-\r", b"x" * 33,
        )
        for data in cases:
            with self.subTest(data=data[:8]):
                # The fallback reconnect consumes the third scripted response.
                fixture = Fixture(scripts=[[], ok("FS"), ok("FS")],
                                  harness_kwargs={"max_drain_bytes": 32})
                fixture.open()
                fixture.harness.transaction("FF", 5_000_000)
                fixture.endpoint.pending.append((fixture.clock.now + 1, data))
                result = fixture.harness.bounded_resync_recovery("FS")
                self.assertTrue(result["valid"])
                self.assertEqual(fixture.harness.connection_generation, 2)
                self.assertTrue(any(x.get("action") == "fallback_reconnect_started"
                                    for x in fixture.recorder.items))
                completed = [x for x in fixture.recorder.items
                             if x.get("action") == "bounded_resynchronisation_fallback_completed"]
                self.assertEqual(len(completed), 1)
                self.assertGreaterEqual(completed[0]["recovery_duration"], 0)

    def test_sync_mismatch_forces_reconnect_before_success(self):
        fixture = Fixture(scripts=[[], [(1, b"FF=8\r")], ok("FS")])
        fixture.open()
        fixture.harness.transaction("FF", 5_000_000)
        result = fixture.harness.bounded_resync_recovery("FS")
        self.assertTrue(result["valid"])
        self.assertEqual(fixture.harness.connection_generation, 2)
        failed_sync = [x for x in fixture.recorder.items
                       if x.get("purpose") == "bounded_resync_query"][-1]
        self.assertEqual(failed_sync["error"], "wrong_prefix")

    def test_old_reply_after_fresh_bounded_sync_forces_reconnect(self):
        fresh_then_old = [(1_000_000, b"FS=0\r"), (2_000_000, b"FF=8\r")]
        fixture = Fixture(scripts=[[], fresh_then_old, ok("FS")])
        fixture.open()
        fixture.harness.transaction("FF", 5_000_000)
        result = fixture.harness.bounded_resync_recovery("FS")
        self.assertTrue(result["valid"])
        self.assertEqual(fixture.harness.connection_generation, 2)
        self.assertEqual(fixture.endpoint.pending, [])
        verification = [x for x in fixture.recorder.items
                        if x.get("action") == "post_sync_quiet_verification"]
        self.assertFalse(verification[0]["valid"])
        self.assertEqual(verification[0]["post_sync_classification"],
                         "delayed_timed_out_reply")
        self.assertTrue(verification[-1]["valid"])

    def test_read_error_forces_reconnect(self):
        fixture = Fixture(scripts=[[], ok("FS")], endpoint_kwargs={"read_error_at": 3})
        fixture.open()  # startup read is #1; timed transaction read is #2; drain is #3.
        fixture.harness.transaction("FF", 5_000_000)
        result = fixture.harness.bounded_resync_recovery("FS")
        self.assertTrue(result["valid"])
        self.assertEqual(fixture.harness.connection_generation, 2)

    def test_nonquiet_absolute_deadline_forces_reconnect(self):
        fixture = Fixture(scripts=[[], ok("FS")],
                          harness_kwargs={"drain_horizon_ns": 5_000_000,
                                          "drain_absolute_ns": 7_000_000,
                                          "drain_quiet_ns": 4_000_000})
        fixture.open()
        fixture.harness.transaction("FF", 2_000_000)
        fixture.endpoint.pending.append((fixture.clock.now + 3_000_000, b"FF=1\r"))
        result = fixture.harness.bounded_resync_recovery("FS")
        self.assertTrue(result["valid"])
        self.assertEqual(fixture.harness.connection_generation, 2)

    def test_drain_evidence_is_recorded_before_sync_write(self):
        events = []
        fixture = Fixture(scripts=[[], ok("FS")], events=events)
        fixture.open()
        fixture.harness.transaction("FF", 5_000_000)
        fixture.endpoint.pending.append((fixture.clock.now + 1, b"FF=1\r"))
        fixture.harness.bounded_resync_recovery("FS")
        drain_record = next(i for i, event in enumerate(events)
                            if event == "record:recovery_action" and i > events.index("write:?FF"))
        self.assertLess(drain_record, events.index("write:?FS"))

    def test_bounded_drain_empty_read_overshoot_forces_reconnect(self):
        fixture = Fixture(
            scripts=[[], ok("FS")],
            endpoint_kwargs={"overshoot_reads": {3: (20_000_000, b"")}})
        fixture.open()
        fixture.harness.transaction("FF", 5_000_000)
        result = fixture.harness.bounded_resync_recovery("FS")
        self.assertTrue(result["valid"])
        self.assertEqual(fixture.harness.connection_generation, 2)
        drain = [x for x in fixture.recorder.items
                 if x.get("action") == "bounded_delimiter_drain"][-1]
        self.assertEqual(drain["error"], "drain_absolute_deadline_overshoot")

    def test_bounded_drain_late_chunk_overshoot_is_retained_then_reconnects(self):
        fixture = Fixture(
            scripts=[[], ok("FS")],
            endpoint_kwargs={"overshoot_reads": {3: (20_000_000, b"FF=5\r")}})
        fixture.open()
        fixture.harness.transaction("FF", 5_000_000)
        result = fixture.harness.bounded_resync_recovery("FS")
        self.assertTrue(result["valid"])
        self.assertEqual(fixture.harness.connection_generation, 2)
        drain = [x for x in fixture.recorder.items
                 if x.get("action") == "bounded_delimiter_drain"][-1]
        self.assertEqual(bytes.fromhex(drain["post_drain_bytes"]["hex"]), b"FF=5\r")
        self.assertEqual(drain["error"], "drain_absolute_deadline_overshoot")


class RunModeTest(unittest.TestCase):
    def args(self, mode, deadline=.1, attempts=1):
        return argparse.Namespace(mode=mode, deadline=deadline, attempts=attempts,
                                  port="unused", baud=115200, output="unused")

    def test_baseline_fixed_sequence(self):
        fixture = Fixture(scripts=[ok("FID"), ok("FF"), ok("FS")])
        self.assertEqual(MODULE.run(self.args("baseline"), fixture.endpoint,
                                    fixture.recorder, fixture.clock), 0)
        self.assertEqual(fixture.endpoint.writes,
                         [b"?FID\r", b"?FF\r", b"?FS\r"])

    def test_attempts_alternate_only_ff_fs_pairs(self):
        scripts = [ok("FID"), [], ok("FS"), ok("FID"), [], ok("FF")]
        fixture = Fixture(scripts=scripts)
        self.assertEqual(MODULE.run(self.args("boundary", .005, 2), fixture.endpoint,
                                    fixture.recorder, fixture.clock), 0)
        self.assertEqual(fixture.endpoint.writes,
                         [b"?FID\r", b"?FF\r", b"?FS\r",
                          b"?FID\r", b"?FS\r", b"?FF\r"])

    def test_recovery_mode_fails_if_approved_threshold_does_not_timeout(self):
        fixture = Fixture(scripts=[ok("FID"), ok("FF")])
        with self.assertRaisesRegex(RuntimeError, "expected_timeout_not_observed"):
            MODULE.run(self.args("reconnect", .030), fixture.endpoint,
                       fixture.recorder, fixture.clock)
        self.assertTrue(any(x.get("action") == "expected_timeout_not_observed"
                            for x in fixture.recorder.items))


if __name__ == "__main__":
    unittest.main(verbosity=2)
