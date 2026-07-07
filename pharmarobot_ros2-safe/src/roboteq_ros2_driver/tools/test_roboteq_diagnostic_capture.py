#!/usr/bin/env python3
# Copyright 2026 Medrobots
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Deterministic offline tests; this file never opens a serial device."""

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


MODULE_PATH = Path(__file__).with_name("roboteq_diagnostic_capture.py")
SPEC = importlib.util.spec_from_file_location("roboteq_diagnostic_capture", MODULE_PATH)
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


class FakeEndpoint:
    def __init__(self, clock, scripts=None, write_ns=1_000_000, write_error=None,
                 read_errors=None, events=None):
        self.clock = clock
        self.scripts = list(scripts or [])
        self.write_ns = write_ns
        self.write_error = write_error
        self.read_errors = dict(read_errors or {})
        self.read_count = 0
        self.pending = []
        self.writes = []
        self.open_count = 0
        self.closed = True
        self.events = events

    def open(self):
        self.open_count += 1
        self.closed = False
        if self.events is not None:
            self.events.append("open")

    def close(self):
        self.closed = True
        if self.events is not None:
            self.events.append("close")

    def write(self, data, deadline_ns):
        if data not in MODULE.ALLOWED_REQUESTS:
            raise ValueError("forbidden")
        if self.write_error:
            self.clock.now = deadline_ns
            raise self.write_error
        self.clock.now += self.write_ns
        if self.clock.now > deadline_ns:
            raise TimeoutError("write_timeout")
        self.writes.append(data)
        if self.events is not None:
            self.events.append("write")
        script = self.scripts.pop(0) if self.scripts else []
        base = self.clock.now
        self.pending.extend((base + delay, data) for delay, data in script)
        self.pending.sort(key=lambda item: item[0])

    def read(self, maximum, deadline_ns):
        self.read_count += 1
        if self.read_count in self.read_errors:
            raise self.read_errors[self.read_count]
        if not self.pending or self.pending[0][0] > deadline_ns:
            self.clock.now = max(self.clock.now, deadline_ns)
            return b""
        when, data = self.pending.pop(0)
        self.clock.now = max(self.clock.now, when)
        chunk, remainder = data[:maximum], data[maximum:]
        if remainder:
            self.pending.insert(0, (when, remainder))
        return chunk

    def inject(self, data, delay=0):
        self.pending.append((self.clock.now + delay, data))
        self.pending.sort(key=lambda item: item[0])


class CaptureFixture:
    def __init__(self, testcase, scripts=None, startup=None, endpoint_kwargs=None,
                 auto_open=True, **capture_kwargs):
        self.testcase = testcase
        self.temporary = tempfile.TemporaryDirectory()
        self.path = Path(self.temporary.name) / "evidence.jsonl"
        self.clock = FakeClock()
        self.endpoint = FakeEndpoint(
            self.clock, scripts, **(endpoint_kwargs or {}))
        for delay, data in startup or []:
            self.endpoint.inject(data, delay)
        self.recorder = MODULE.EvidenceRecorder(self.path)
        defaults = dict(
            write_ns=10_000_000,
            query_ns=100_000_000,
            pre_drain_ns=5_000_000,
            quiet_drain_ns=5_000_000,
            failure_drain_ns=20_000_000,
            startup_quiet_ns=20_000_000,
            startup_deadline_ns=50_000_000,
        )
        defaults.update(capture_kwargs)
        self.capture = MODULE.DiagnosticCapture(
            self.endpoint, self.recorder, self.clock, **defaults)
        self.startup_record = None
        if auto_open:
            self.startup_record = self.capture.open()
            testcase.assertTrue(self.startup_record["valid"])

    def close(self):
        self.capture.close()
        self.recorder.close()
        self.temporary.cleanup()

    def records(self):
        return [json.loads(line) for line in self.path.read_text().splitlines()]


class ParsingTest(unittest.TestCase):
    def test_every_uint8_value_for_all_numeric_queries(self):
        for name in ("FF", "FM1", "FM2", "FS"):
            spec = MODULE.QUERY_SPECS[name]
            for value in range(256):
                with self.subTest(name=name, value=value):
                    raw = spec.prefix + str(value).encode() + b"\r"
                    self.assertEqual(
                        MODULE.parse_response(spec, raw),
                        (spec.prefix[:-1].decode(), value, None),
                    )

    def test_fid_and_supported_framing(self):
        spec = MODULE.QUERY_SPECS["FID"]
        for ending in (b"\r", b"\n", b"\r\n"):
            with self.subTest(ending=ending):
                raw = b"?FID" + ending + b"+" + ending + b"FID=Roboteq v1.8d SBL2360" + ending
                self.assertEqual(
                    MODULE.parse_response(spec, raw),
                    ("FID", "Roboteq v1.8d SBL2360", None),
                )

    def test_malformed_values_are_rejected(self):
        spec = MODULE.QUERY_SPECS["FF"]
        cases = {
            b"FF=+1\r": "invalid_numeric_value",
            b"FF=-1\r": "invalid_numeric_value",
            b"FF=256\r": "numeric_overflow",
            b"FF=1x\r": "invalid_numeric_value",
            b"FF= 1\r": "invalid_numeric_value",
            b"FF=1": "partial_reply",
            b"FS=1\r": "wrong_prefix",
            b"FF=1\rjunk\r": "ambiguous_reply",
            b"FF=1\rFF=2\r": "ambiguous_reply",
            b"-\r": "explicit_rejection",
            b"+\r": "missing_reply",
        }
        for raw, expected in cases.items():
            with self.subTest(raw=raw):
                self.assertEqual(MODULE.parse_response(spec, raw)[2], expected)

    def test_fid_rejects_empty_control_and_non_ascii_payloads(self):
        spec = MODULE.QUERY_SPECS["FID"]
        for raw in (b"FID=\r", b"FID=a\x00b\r", b"FID=\xff\r", b"FID=a\tb\r"):
            with self.subTest(raw=raw):
                self.assertIsNotNone(MODULE.parse_response(spec, raw)[2])

    def test_raw_bytes_are_round_trippable_and_visible(self):
        raw = b"FID=a\r\n\\\x00\xff"
        field = MODULE.bytes_field(raw)
        self.assertEqual(bytes.fromhex(field["hex"]), raw)
        self.assertEqual(field["visible"], r"FID=a\r\n\\\x00\xFF")


class StartupSynchronizationTest(unittest.TestCase):
    def use_fixture(self, *args, **kwargs):
        fixture = CaptureFixture(self, *args, **kwargs)
        self.addCleanup(fixture.close)
        return fixture

    def test_nul_banner_is_preserved_then_continuous_quiet_authorizes_queries(self):
        banner = b"\x00Starting ...\rStarting ...\r\x00Start"
        fixture = self.use_fixture(
            startup=[(1_000_000, banner[:9]), (8_000_000, banner[9:])],
            scripts=[[(1, b"FF=0\r")]],
        )
        item = fixture.startup_record
        self.assertEqual(item["schema_version"], 2)
        self.assertEqual(item["record_type"], "startup_synchronization")
        self.assertEqual(bytes.fromhex(item["startup_bytes"]["hex"]), banner)
        self.assertEqual(item["monotonic_last_byte"], 1_008_000_000)
        self.assertEqual(item["monotonic_synchronization_complete"], 1_028_000_000)
        transaction = fixture.capture.transact("FF")
        self.assertEqual(transaction["session_id"], item["session_id"])
        self.assertEqual(
            transaction["synchronization_generation"],
            item["synchronization_generation"],
        )

    def test_quiet_period_restarts_after_every_chunk(self):
        fixture = self.use_fixture(startup=[
            (19_000_000, b"a"),
            (38_000_000, b"b"),
        ], startup_deadline_ns=100_000_000)
        self.assertEqual(
            fixture.startup_record["monotonic_synchronization_complete"],
            1_058_000_000,
        )

    def test_quiet_and_overall_deadline_boundaries(self):
        # Quiet completion exactly at the overall deadline is valid.
        valid = self.use_fixture(
            startup=[(19_000_000, b"x"), (30_000_000, b"y")],
            startup_quiet_ns=20_000_000,
            startup_deadline_ns=50_000_000,
        )
        self.assertTrue(valid.startup_record["valid"])
        self.assertEqual(
            valid.startup_record["monotonic_synchronization_complete"],
            1_050_000_000,
        )

        invalid = self.use_fixture(
            startup=[(19_000_000, b"x"), (38_000_000, b"y")],
            auto_open=False,
            startup_quiet_ns=20_000_000,
            startup_deadline_ns=50_000_000,
        )
        item = invalid.capture.open()
        self.assertFalse(item["valid"])
        self.assertTrue(item["timeout"])
        self.assertEqual(item["error"], "startup_synchronization_timeout")
        self.assertTrue(invalid.endpoint.closed)
        self.assertEqual(invalid.endpoint.writes, [])

    def test_read_overshoot_after_both_deadlines_cannot_authorize_sync(self):
        fixture = self.use_fixture(auto_open=False)

        def overshooting_read(unused_maximum, unused_deadline_ns):
            fixture.clock.now = 1_060_000_000
            return b"late startup bytes"

        fixture.endpoint.read = overshooting_read
        item = fixture.capture.open()
        self.assertFalse(item["valid"])
        self.assertTrue(item["timeout"])
        self.assertEqual(item["error"], "startup_synchronization_timeout")
        self.assertEqual(
            bytes.fromhex(item["startup_bytes"]["hex"]),
            b"late startup bytes",
        )
        self.assertIsNone(item["synchronization_generation"])
        self.assertTrue(fixture.endpoint.closed)
        self.assertEqual(fixture.endpoint.writes, [])

    def test_startup_byte_cap_accepts_4096_and_rejects_4097(self):
        at_cap = self.use_fixture(
            startup=[(1, b"A" * 4096)], max_startup_bytes=4096)
        self.assertTrue(at_cap.startup_record["valid"])
        self.assertEqual(at_cap.startup_record["startup_bytes"]["length"], 4096)

        over_cap = self.use_fixture(
            startup=[(1, b"A" * 4097)], auto_open=False,
            max_startup_bytes=4096)
        item = over_cap.capture.open()
        self.assertFalse(item["valid"])
        self.assertEqual(item["error"], "startup_bytes_exceeded")
        self.assertEqual(item["startup_bytes"]["length"], 4097)
        self.assertTrue(item["startup_bytes"]["truncated"])
        self.assertTrue(over_cap.endpoint.closed)

    def test_startup_read_error_records_invalid_and_closes(self):
        fixture = self.use_fixture(
            auto_open=False,
            endpoint_kwargs={"read_errors": {1: OSError("injected")}},
        )
        item = fixture.capture.open()
        self.assertFalse(item["valid"])
        self.assertEqual(item["error"], "startup_read_error:OSError")
        self.assertTrue(fixture.endpoint.closed)
        self.assertEqual(fixture.endpoint.writes, [])
        self.assertEqual(fixture.records(), [item])

    def test_nonmonotonic_clock_records_invalid_and_closes(self):
        fixture = self.use_fixture(auto_open=False)
        original_read = fixture.endpoint.read

        def regressing_read(maximum, deadline_ns):
            result = original_read(maximum, deadline_ns)
            fixture.clock.now = 900_000_000
            return result

        fixture.endpoint.read = regressing_read
        item = fixture.capture.open()
        self.assertFalse(item["valid"])
        self.assertEqual(item["error"], "nonmonotonic_clock")
        self.assertTrue(fixture.endpoint.closed)
        self.assertEqual(fixture.endpoint.writes, [])

    def test_startup_fid_line_cannot_satisfy_later_fid_query(self):
        fixture = self.use_fixture(
            startup=[(1, b"FID=startup banner\r")], scripts=[[]])
        item = fixture.capture.transact("FID")
        self.assertFalse(item["valid"])
        self.assertEqual(item["error"], "response_timeout")
        self.assertIsNone(item["parsed_value"])
        self.assertEqual(
            bytes.fromhex(fixture.startup_record["startup_bytes"]["hex"]),
            b"FID=startup banner\r",
        )

    def test_sync_record_is_fsynced_before_first_write(self):
        events = []
        fixture = self.use_fixture(
            auto_open=False,
            scripts=[[(1, b"FF=0\r")]],
            endpoint_kwargs={"events": events},
        )
        real_fsync = MODULE.os.fsync

        def observed_fsync(fd):
            events.append("fsync")
            return real_fsync(fd)

        with mock.patch.object(MODULE.os, "fsync", side_effect=observed_fsync):
            self.assertTrue(fixture.capture.open()["valid"])
            self.assertTrue(fixture.capture.transact("FF")["valid"])
        self.assertLess(events.index("fsync"), events.index("write"))

    def test_reopen_invalidates_previous_synchronization_reference(self):
        fixture = self.use_fixture(scripts=[[(1, b"FF=0\r")], [(1, b"FF=1\r")]])
        first = fixture.capture.transact("FF")
        fixture.capture.close()
        second_sync = fixture.capture.open()
        second = fixture.capture.transact("FF")
        self.assertNotEqual(
            first["synchronization_generation"],
            second["synchronization_generation"],
        )
        self.assertEqual(
            second["synchronization_generation"],
            second_sync["synchronization_generation"],
        )


class TransactionTest(unittest.TestCase):
    def use_fixture(self, *args, **kwargs):
        fixture = CaptureFixture(self, *args, **kwargs)
        self.addCleanup(fixture.close)
        return fixture

    def test_valid_chunked_echo_ack_reply_has_monotonic_timings(self):
        fixture = self.use_fixture(scripts=[[
            (2_000_000, b"?FF\r"),
            (3_000_000, b"+\r"),
            (4_000_000, b"FF=7\r\n"),
        ]])
        item = fixture.capture.transact("FF")

        self.assertTrue(item["valid"])
        self.assertEqual(item["parsed_prefix"], "FF")
        self.assertEqual(item["parsed_value"], 7)
        self.assertEqual(item["write_duration"], 1_000_000)
        self.assertEqual(item["first_byte_latency"], 2_000_000)
        self.assertGreaterEqual(item["monotonic_response_complete"], item["monotonic_first_byte"])
        self.assertEqual(bytes.fromhex(item["response_bytes"]["hex"]), b"?FF\r+\rFF=7\r\n")

    def test_wrong_prefix_is_invalid_and_requires_close(self):
        fixture = self.use_fixture(scripts=[[(1, b"FS=0\r")], [(1, b"FF=0\r")]])
        item = fixture.capture.transact("FF")
        self.assertFalse(item["valid"])
        self.assertEqual(item["error"], "wrong_prefix")
        self.assertIn("close_required", item["drain_or_resynchronisation_action"])
        with self.assertRaises(RuntimeError):
            fixture.capture.transact("FF")
        self.assertEqual(fixture.endpoint.writes, [b"?FF\r"])
        fixture.capture.close()
        fixture.capture.open()
        self.assertTrue(fixture.capture.transact("FF")["valid"])

    def test_partial_reply_times_out_and_is_preserved(self):
        fixture = self.use_fixture(scripts=[[(1, b"FF=2")]])
        item = fixture.capture.transact("FF")
        self.assertFalse(item["valid"])
        self.assertTrue(item["timeout"])
        self.assertEqual(item["error"], "partial_reply_timeout")
        self.assertEqual(bytes.fromhex(item["response_bytes"]["hex"]), b"FF=2")

    def test_missing_reply_times_out(self):
        fixture = self.use_fixture(scripts=[[]])
        item = fixture.capture.transact("FS")
        self.assertFalse(item["valid"])
        self.assertTrue(item["timeout"])
        self.assertEqual(item["error"], "response_timeout")

    def test_delayed_reply_is_captured_only_as_failure_drain(self):
        fixture = self.use_fixture(scripts=[[(110_000_000, b"FF=4\r")]])
        item = fixture.capture.transact("FF")
        self.assertFalse(item["valid"])
        self.assertTrue(item["timeout"])
        self.assertEqual(bytes.fromhex(item["drain_bytes"]["after"]["hex"]), b"FF=4\r")

    def test_trailing_junk_in_same_or_later_chunk_is_invalid(self):
        for script in (
                [(1, b"FF=1\rjunk\r")],
                [(1, b"FF=1\r"), (2_000_000, b"junk\r")]):
            with self.subTest(script=script):
                fixture = CaptureFixture(self, scripts=[script])
                try:
                    item = fixture.capture.transact("FF")
                    self.assertFalse(item["valid"])
                    self.assertIsNotNone(item["error"])
                finally:
                    fixture.close()

    def test_oversized_reply_is_invalid_and_bounded(self):
        fixture = self.use_fixture(
            scripts=[[(1, b"X" * 40)]], max_response=32)
        item = fixture.capture.transact("FF")
        self.assertFalse(item["valid"])
        self.assertEqual(item["error"], "oversized_reply")
        self.assertEqual(item["response_bytes"]["length"], 33)
        self.assertTrue(item["response_bytes"]["truncated"])

    def test_preexisting_data_prevents_write(self):
        fixture = self.use_fixture(scripts=[])
        fixture.endpoint.inject(b"old\r")
        item = fixture.capture.transact("FF")
        self.assertFalse(item["valid"])
        self.assertEqual(item["error"], "preexisting_data")
        self.assertEqual(fixture.endpoint.writes, [])
        self.assertIsNone(item["monotonic_before_write"])

    def test_write_timeout_is_recorded(self):
        fixture = self.use_fixture(scripts=[])
        fixture.endpoint.write_error = TimeoutError("write_timeout")
        item = fixture.capture.transact("FF")
        self.assertFalse(item["valid"])
        self.assertTrue(item["timeout"])
        self.assertEqual(item["error"], "write_timeout")

    def test_initial_drain_read_error_records_and_prevents_any_write(self):
        fixture = self.use_fixture(scripts=[])
        # Read 1 is startup synchronization; read 2 is the transaction drain.
        fixture.endpoint.read_errors[2] = OSError("injected pre-drain failure")
        item = fixture.capture.transact("FF")
        self.assertFalse(item["valid"])
        self.assertEqual(item["error"], "pre_write_drain_error")
        self.assertIn("pre_write_drain_failed:OSError", item["drain_or_resynchronisation_action"])
        self.assertIn("close_required", item["drain_or_resynchronisation_action"])
        self.assertEqual(fixture.endpoint.writes, [])
        self.assertEqual(len(fixture.records()), 2)
        with self.assertRaises(RuntimeError):
            fixture.capture.transact("FF")

    def test_failure_drain_read_error_still_records_and_poison_connection(self):
        fixture = self.use_fixture(
            scripts=[[(1, b"X" * 40)]], max_response=32)
        # Read 1 is startup synchronization, read 2 is the empty pre-drain,
        # read 3 receives the oversized reply, and read 4 is failure drain.
        fixture.endpoint.read_errors[4] = OSError("injected failure-drain failure")
        item = fixture.capture.transact("FF")
        self.assertFalse(item["valid"])
        self.assertEqual(item["error"], "oversized_reply")
        self.assertIn(
            "bounded_failure_drain_failed:OSError",
            item["drain_or_resynchronisation_action"],
        )
        self.assertIn("close_required", item["drain_or_resynchronisation_action"])
        self.assertEqual(len(fixture.records()), 2)
        with self.assertRaises(RuntimeError):
            fixture.capture.transact("FF")
        self.assertEqual(fixture.endpoint.writes, [b"?FF\r"])

    def test_connection_generation_increments_after_each_open(self):
        fixture = self.use_fixture(scripts=[[(1, b"FF=0\r")], [(1, b"FF=1\r")]])
        first = fixture.capture.transact("FF")
        fixture.capture.close()
        fixture.capture.open()
        second = fixture.capture.transact("FF")
        self.assertEqual(first["connection_generation"], 1)
        self.assertEqual(second["connection_generation"], 2)

    def test_only_five_fixed_requests_can_be_emitted(self):
        scripts = [[(1, spec.prefix + (b"ok" if spec.value_type == "text" else b"0") + b"\r")]
                   for spec in MODULE.QUERY_SPECS.values()]
        fixture = self.use_fixture(scripts=scripts)
        for name in MODULE.QUERY_SPECS:
            self.assertTrue(fixture.capture.transact(name)["valid"])
        self.assertEqual(set(fixture.endpoint.writes), MODULE.ALLOWED_REQUESTS)
        for forbidden in ("DI", "CR", "!G 1 0", "^RWD 500"):
            with self.assertRaises(ValueError):
                fixture.capture.transact(forbidden)


class RealWriteBoundaryTest(unittest.TestCase):
    def test_actual_endpoint_allows_only_five_exact_payloads(self):
        clock = FakeClock()
        endpoint = MODULE.BoundedSerialEndpoint("/never-opened", 115200, clock)
        endpoint.fd = 123
        deadline = clock.monotonic_ns() + 10_000_000

        with mock.patch.object(MODULE.select, "select") as select_mock:
            with mock.patch.object(MODULE.os, "write") as write_mock:
                select_mock.return_value = ([], [endpoint.fd], [])
                write_mock.side_effect = lambda unused_fd, data: len(data)
                for payload in MODULE.ALLOWED_REQUESTS:
                    with self.subTest(allowed=payload):
                        select_mock.reset_mock()
                        write_mock.reset_mock()
                        endpoint.write(payload, deadline)
                        select_mock.assert_called_once()
                        write_mock.assert_called_once_with(endpoint.fd, payload)

                for payload in (
                        b"?DI 1\r", b"?CR\r", b"!G 1 0\r", b"^RWD 500\r",
                        b"?FF\n", b"?FF\r\x00"):
                    with self.subTest(forbidden=payload):
                        select_mock.reset_mock()
                        write_mock.reset_mock()
                        with self.assertRaises(ValueError):
                            endpoint.write(payload, deadline)
                        select_mock.assert_not_called()
                        write_mock.assert_not_called()

    def test_no_serial_input_flush_or_modem_line_manipulation_exists(self):
        source = MODULE_PATH.read_text()
        self.assertNotIn("tcflush", source)
        for token in (
                "TIOCMGET", "TIOCMSET", "TIOCMBIS", "TIOCMBIC",
                "TIOCM_DTR", "TIOCM_RTS"):
            with self.subTest(token=token):
                self.assertNotIn(token, source)
        # The sole ioctl is the pre-existing TIOCEXCL exclusive-access lock;
        # synchronization itself uses only bounded endpoint reads.
        self.assertEqual(source.count("fcntl.ioctl"), 1)
        self.assertIn("fcntl.ioctl(self.fd, termios.TIOCEXCL)", source)


class EvidenceAndCliTest(unittest.TestCase):
    def test_append_preserves_records_and_continues_sequence(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.jsonl"
            first = MODULE.EvidenceRecorder(path)
            first.record({"sequence": first.next_sequence(), "marker": "first"})
            first.close()
            original = path.read_bytes()
            second = MODULE.EvidenceRecorder(path)
            second.record({"sequence": second.next_sequence(), "marker": "second"})
            second.close()
            self.assertTrue(path.read_bytes().startswith(original))
            records = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual([item["sequence"] for item in records], [1, 2])

    def test_schema_v1_record_remains_append_compatible_with_schema_v2(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.jsonl"
            legacy = {"schema_version": 1, "sequence": 7, "query_name": "FF"}
            path.write_text(json.dumps(legacy) + "\n")
            recorder = MODULE.EvidenceRecorder(path)
            recorder.record({
                "schema_version": 2,
                "sequence": recorder.next_sequence(),
                "record_type": "startup_synchronization",
            })
            recorder.close()
            records = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(records[0], legacy)
            self.assertEqual(records[1]["sequence"], 8)
            self.assertEqual(records[1]["schema_version"], 2)

    def test_appended_capture_sessions_have_distinct_linked_session_ids(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.jsonl"
            session_ids = []
            for value in (1, 2):
                clock = FakeClock()
                endpoint = FakeEndpoint(
                    clock, scripts=[[(1, f"FF={value}\r".encode())]])
                recorder = MODULE.EvidenceRecorder(path)
                capture = MODULE.DiagnosticCapture(
                    endpoint,
                    recorder,
                    clock,
                    startup_quiet_ns=20_000_000,
                    startup_deadline_ns=50_000_000,
                    pre_drain_ns=5_000_000,
                    quiet_drain_ns=5_000_000,
                )
                try:
                    startup = capture.open()
                    transaction = capture.transact("FF")
                    self.assertEqual(startup["session_id"], transaction["session_id"])
                    self.assertEqual(startup["connection_generation"], 1)
                    self.assertEqual(transaction["connection_generation"], 1)
                    session_ids.append(startup["session_id"])
                finally:
                    capture.close()
                    recorder.close()

            self.assertNotEqual(session_ids[0], session_ids[1])
            records = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual([item["sequence"] for item in records], [1, 2, 3, 4])
            self.assertEqual(
                [item["session_id"] for item in records],
                [session_ids[0], session_ids[0], session_ids[1], session_ids[1]],
            )

    def test_incomplete_or_malformed_existing_jsonl_is_refused(self):
        for content in ('{"sequence":1}', 'not-json\n'):
            with self.subTest(content=content), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "capture.jsonl"
                path.write_text(content)
                with self.assertRaises(RuntimeError):
                    MODULE.EvidenceRecorder(path)

    def test_cli_has_no_arbitrary_query_or_command_argument(self):
        parser = MODULE.build_parser()
        args = parser.parse_args(["--port", "/not-opened", "--output", "/tmp/not-created"])
        self.assertFalse(hasattr(args, "query"))
        self.assertFalse(hasattr(args, "command"))
        with self.assertRaises(SystemExit):
            parser.parse_args([
                "--port", "/not-opened", "--output", "/tmp/not-created",
                "--query", "1",
            ])


if __name__ == "__main__":
    unittest.main(verbosity=2)
