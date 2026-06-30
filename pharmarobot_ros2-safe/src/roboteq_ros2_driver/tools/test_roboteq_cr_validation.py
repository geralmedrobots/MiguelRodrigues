#!/usr/bin/env python3
# Copyright 2026 Medrobots
#
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

"""Offline tests for roboteq_cr_validation.py; no hardware access."""

import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import pty
import select
import tempfile
import threading
import time
import unittest


MODULE_PATH = Path(__file__).with_name("roboteq_cr_validation.py")
SPEC = importlib.util.spec_from_file_location("roboteq_cr_validation", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class RawFormattingTest(unittest.TestCase):
    def test_visible_bytes_preserves_control_and_non_ascii_bytes(self):
        self.assertEqual(
            MODULE.visible_bytes(b"CR=1:-2\r\n\\\x00\xff"),
            r"CR=1:-2\r\n\\\x00\xFF",
        )

    def test_recorder_keeps_hex_visible_and_timestamps(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "evidence.jsonl"
            recorder = MODULE.EvidenceRecorder(output)
            with contextlib.redirect_stdout(io.StringIO()):
                recorder.bytes("rx", b"CR=0:0\r", transaction_id="test")
            recorder.close()
            line = output.read_text(encoding="utf-8")
            self.assertIn('"raw_hex":"43 52 3D 30 3A 30 0D"', line)
            self.assertIn('"raw_visible":"CR=0:0\\\\r"', line)
            self.assertIn('"monotonic_ns":', line)
            self.assertIn('"wall_time_utc":', line)

    def test_explicit_rejection_requires_complete_standalone_line(self):
        self.assertTrue(MODULE.contains_explicit_rejection(b"-\r"))
        self.assertTrue(MODULE.contains_explicit_rejection(b"echo\r-\n"))
        self.assertFalse(MODULE.contains_explicit_rejection(b"-"))
        self.assertFalse(MODULE.contains_explicit_rejection(b"value=-1\r"))

    def test_encoder_counts_require_exactly_two_signed_int32_fields(self):
        self.assertEqual(MODULE.parse_encoder_counts(b"CR=1:-2\r"), (1, -2))
        self.assertEqual(
            MODULE.parse_encoder_counts(b"?CR\r+\rCR=-2147483648:2147483647\r\n"),
            (-2147483648, 2147483647),
        )

    def test_encoder_counts_reject_malformed_or_ambiguous_replies(self):
        malformed = [
            b"CR=garbage\r",
            b"CR=1:2:3\r",
            b"CR=1\r",
            b"CR=+1:2\r",
            b"CR=1:2\rCR=3:4\r",
            b"CR=2147483648:0\r",
            b"CR=0:-2147483649\r",
            b"CR=1:2",
        ]
        for raw in malformed:
            with self.subTest(raw=raw):
                self.assertIsNone(MODULE.parse_encoder_counts(raw))


class BoundedSerialTest(unittest.TestCase):
    def run_command_transaction(
            self, reply, command=b"!S 1 5\r", preexisting=b"",
            timeout=0.08, reply_delay=0.0):
        master, slave = pty.openpty()
        slave_path = os.ttyname(slave)
        os.close(slave)

        def controller():
            readable, _, _ = select.select([master], [], [], 1.0)
            if readable:
                os.read(master, 256)
                if reply:
                    if reply_delay:
                        threading.Event().wait(reply_delay)
                    os.write(master, reply)

        temporary = tempfile.TemporaryDirectory()
        output = Path(temporary.name) / "evidence.jsonl"
        recorder = MODULE.EvidenceRecorder(output)
        serial_port = MODULE.BoundedSerial(slave_path, 115200, recorder, timeout)
        with contextlib.redirect_stdout(io.StringIO()):
            serial_port.open()
            if preexisting:
                os.write(master, preexisting)
            thread = threading.Thread(target=controller)
            thread.start()
            started_at = time.monotonic()
            try:
                serial_port.command(command, "test-command")
                error = None
            except Exception as caught:
                error = caught
            elapsed = time.monotonic() - started_at
            thread.join(timeout=1.0)
            serial_port.close()
        recorder.close()
        os.close(master)
        records = [
            json.loads(line) for line in output.read_text(encoding="utf-8").splitlines()
        ]
        temporary.cleanup()
        completed = [item for item in records if item["event"] == "transaction_complete"]
        return error, completed[-1] if completed else None, records, elapsed

    def test_query_retains_echo_ack_and_line_endings_in_one_transaction(self):
        master, slave = pty.openpty()
        slave_path = os.ttyname(slave)
        os.close(slave)

        def controller():
            readable, _, _ = select.select([master], [], [], 1.0)
            self.assertTrue(readable)
            self.assertEqual(os.read(master, 64), b"?CR\r")
            os.write(master, b"?CR\r+\rCR=1:-2\r\n")

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "evidence.jsonl"
            recorder = MODULE.EvidenceRecorder(output)
            serial_port = MODULE.BoundedSerial(slave_path, 115200, recorder, 0.5)
            with contextlib.redirect_stdout(io.StringIO()):
                serial_port.open()
                thread = threading.Thread(target=controller)
                thread.start()
                raw = serial_port.query(b"?CR\r", b"CR=", "test-cr")
                thread.join(timeout=1.0)
                serial_port.close()
            recorder.close()
            os.close(master)

            self.assertEqual(raw, b"?CR\r+\rCR=1:-2\r\n")
            records = [
                json.loads(line) for line in output.read_text(encoding="utf-8").splitlines()
            ]
            transmitted = [item for item in records if item["event"] == "tx"]
            completed = [
                item for item in records if item["event"] == "transaction_complete"
            ]
            self.assertEqual(len(transmitted), 1)
            self.assertEqual(len(completed), 1)
            self.assertEqual(
                transmitted[0]["transaction_id"], completed[0]["transaction_id"])
            self.assertEqual(completed[0]["outcome"], "matched")
            self.assertEqual(
                completed[0]["raw_visible"], r"?CR\r+\rCR=1:-2\r\n")

    def test_no_ack_is_successful_write_without_claiming_acceptance(self):
        error, completed, _, _ = self.run_command_transaction(b"")

        self.assertIsNone(error)
        self.assertEqual(completed["outcome"], "write_complete_no_rejection_observed")
        self.assertEqual(completed["raw_length"], 0)

    def test_command_rejection_is_visible_and_fails(self):
        error, completed, _, _ = self.run_command_transaction(b"-\r")

        self.assertIsInstance(error, RuntimeError)
        self.assertEqual(completed["outcome"], "explicit_rejection_observed")

    def test_rejection_later_than_old_short_window_is_observed(self):
        error, completed, _, elapsed = self.run_command_transaction(
            b"-\r", timeout=0.25, reply_delay=0.10)

        self.assertIsInstance(error, RuntimeError)
        self.assertEqual(completed["outcome"], "explicit_rejection_observed")
        self.assertEqual(completed["response_capture_timeout_s"], 0.25)
        self.assertLess(elapsed, 0.22)

    def test_full_stop_explicit_rejection_is_visible_and_fails(self):
        stop_bytes = b"!G 1 0\r!G 2 0\r!S 1 0\r!S 2 0\r"
        error, completed, records, _ = self.run_command_transaction(
            b"-\r", command=stop_bytes)

        self.assertIsInstance(error, RuntimeError)
        self.assertEqual(completed["outcome"], "explicit_rejection_observed")
        transmitted = [item for item in records if item["event"] == "tx"]
        visible_stop = MODULE.visible_bytes(stop_bytes)
        self.assertEqual(transmitted[0]["raw_visible"], visible_stop)
        self.assertEqual(visible_stop, r"!G 1 0\r!G 2 0\r!S 1 0\r!S 2 0\r")

    def test_optional_plus_is_captured_without_claiming_acceptance(self):
        error, completed, _, _ = self.run_command_transaction(b"+\r")

        self.assertIsNone(error)
        self.assertEqual(completed["outcome"], "write_complete_no_rejection_observed")
        self.assertEqual(completed["raw_visible"], r"+\r")

    def test_stale_preexisting_plus_is_separate_from_command_transaction(self):
        error, completed, records, _ = self.run_command_transaction(
            b"", preexisting=b"+\r")

        self.assertIsNone(error)
        drained = [item for item in records if item["event"] == "rx_preexisting"]
        self.assertEqual(len(drained), 1)
        self.assertEqual(drained[0]["raw_visible"], r"+\r")
        self.assertTrue(drained[0]["transaction_id"].startswith("pre-command-test-command-"))
        self.assertEqual(completed["raw_length"], 0)
        self.assertEqual(completed["outcome"], "write_complete_no_rejection_observed")

    def test_preexisting_delayed_rejection_aborts_before_nonzero_write(self):
        error, completed, records, _ = self.run_command_transaction(
            b"", command=b"!S 1 5\r!S 2 0\r", preexisting=b"-\r")

        self.assertIsInstance(error, RuntimeError)
        self.assertIsNone(completed)
        self.assertEqual([item for item in records if item["event"] == "tx"], [])
        aborted = [
            item for item in records if item["event"] == "command_aborted_before_write"
        ]
        self.assertEqual(len(aborted), 1)
        self.assertEqual(
            aborted[0]["outcome"], "preexisting_explicit_rejection_observed")
        self.assertEqual(aborted[0]["raw_visible"], r"-\r")


class SafetyValidationTest(unittest.TestCase):
    def parse_and_validate(self, arguments):
        parser = MODULE.build_parser()
        args = parser.parse_args(arguments)
        with contextlib.redirect_stderr(io.StringIO()):
            MODULE.validate_args(args, parser)

    def test_motion_requires_explicit_safety_confirmation(self):
        with self.assertRaises(SystemExit):
            self.parse_and_validate([
                "--output", "/tmp/not-created", "motion",
                "--channel-1-rpm", "5", "--channel-2-rpm", "5",
            ])

    def test_motion_rejects_excessive_speed_and_duration(self):
        common = [
            "--output", "/tmp/not-created", "motion",
            "--confirm-wheels-lifted-and-estop-accessible",
            "--channel-2-rpm", "1",
        ]
        with self.assertRaises(SystemExit):
            self.parse_and_validate(common + ["--channel-1-rpm", "21"])
        with self.assertRaises(SystemExit):
            self.parse_and_validate(common + [
                "--channel-1-rpm", "1", "--duration", "2.1",
            ])

    def test_bounded_low_speed_motion_arguments_are_valid(self):
        self.parse_and_validate([
            "--output", "/tmp/not-created", "motion",
            "--confirm-wheels-lifted-and-estop-accessible",
            "--channel-1-rpm", "-10", "--channel-2-rpm", "10",
            "--duration", "1.8", "--count", "8", "--interval", "0.05",
        ])

    def test_duration_shorter_than_required_overhead_is_rejected(self):
        with self.assertRaises(SystemExit):
            self.parse_and_validate([
                "--output", "/tmp/not-created", "motion",
                "--confirm-wheels-lifted-and-estop-accessible",
                "--channel-1-rpm", "5", "--channel-2-rpm", "5",
                "--duration", "0.6",
            ])

    def test_minimum_duration_boundary_is_accepted_but_just_below_is_rejected(self):
        parser = MODULE.build_parser()
        args = parser.parse_args([
            "--output", "/tmp/not-created", "motion",
            "--confirm-wheels-lifted-and-estop-accessible",
            "--channel-1-rpm", "5", "--channel-2-rpm", "5",
        ])
        boundary = MODULE.minimum_motion_duration(args.timeout, args.interval)
        args.duration = boundary
        MODULE.validate_args(args, parser)
        args.duration = boundary - 0.001
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                MODULE.validate_args(args, parser)

    def test_query_start_budget_reserves_drain_and_timing_margin(self):
        required = (
            MODULE.COMMAND_PREWRITE_DRAIN_SECONDS + MODULE.TIMING_MARGIN_SECONDS)

        self.assertFalse(MODULE.has_query_start_budget(required, 0.5))
        self.assertFalse(MODULE.has_query_start_budget(required - 0.001, 0.5))
        self.assertTrue(MODULE.has_query_start_budget(required + 0.001, 0.5))

    def test_query_start_budget_uses_shorter_timeout_as_drain_cap(self):
        required = 0.01 + MODULE.TIMING_MARGIN_SECONDS

        self.assertFalse(MODULE.has_query_start_budget(required, 0.01))
        self.assertTrue(MODULE.has_query_start_budget(required + 0.001, 0.01))


class FakeActionSerial:
    class Recorder:
        def __init__(self):
            self.events = []

        def record(self, event, **fields):
            self.events.append({"event": event, **fields})

        def bytes(self, event, data, **fields):
            self.events.append({
                "event": event, "raw": data, "raw_visible": MODULE.visible_bytes(data),
                **fields,
            })

    def __init__(self, fail_labels=None, mmod_responses=None):
        self.timeout = 0.01
        self.commands = []
        self.queries = []
        self.fail_labels = set(fail_labels or [])
        self.mmod_responses = mmod_responses or {1: b"MMOD=1\r", 2: b"MMOD=1\r"}
        self.recorder = self.Recorder()
        self.last_command_write_completed_at = None

    def command(self, data, label, abort_requested=None):
        if abort_requested is not None and abort_requested():
            raise InterruptedError("abort requested before command write")
        self.commands.append((data, label))
        if label in self.fail_labels:
            raise TimeoutError(f"injected failure for {label}")
        write_completed_at = time.monotonic()
        self.last_command_write_completed_at = write_completed_at
        return write_completed_at

    def safety_stop(self, label, absolute_deadline=None):
        self.commands.append((MODULE.ALL_MODE_ZERO_PAYLOAD, label))
        if label in self.fail_labels:
            raise TimeoutError(f"injected failure for {label}")
        return time.monotonic()

    def query(self, data, expected_prefix, label, timeout=None):
        self.queries.append((data, expected_prefix, label, timeout))
        if label.startswith("mmod-channel-"):
            channel = int(label.rsplit("-", 1)[1])
            response = self.mmod_responses[channel]
            if isinstance(response, Exception):
                raise response
            return response
        return b"CR=0:0\r"


class SafetyStopHarness:
    def __init__(self, preexisting=b"", fail_write=False, current_response=b""):
        self.timeout = 0.05
        self.recorder = FakeActionSerial.Recorder()
        self.preexisting = preexisting
        self.fail_write = fail_write
        self.current_response = current_response
        self.write_calls = []

    def drain_preexisting(self, timeout, label):
        return self.preexisting

    def _write_all(self, data, transaction_id, deadline):
        self.write_calls.append((data, transaction_id, deadline))
        if self.fail_write:
            time.sleep(max(0.0, deadline - time.monotonic()))
            raise TimeoutError("injected non-writable serial timeout")
        return time.monotonic()

    def _read_command_response(self, deadline, transaction_id):
        return self.current_response, MODULE.contains_explicit_rejection(
            self.current_response)

    def safety_stop(self, label, absolute_deadline=None):
        return MODULE.BoundedSerial.safety_stop(self, label, absolute_deadline)


class SafetyStopTest(unittest.TestCase):
    def test_stale_rejection_is_reported_but_exact_zero_write_is_attempted(self):
        serial_port = SafetyStopHarness(preexisting=b"-\r")

        with self.assertRaisesRegex(RuntimeError, "EMERGENCY STOP"):
            MODULE.stop(
                serial_port, "test-stop", absolute_deadline=time.monotonic() + 0.1)

        self.assertEqual(serial_port.write_calls[0][0], MODULE.ALL_MODE_ZERO_PAYLOAD)
        events = [event["event"] for event in serial_port.recorder.events]
        self.assertIn("safety_stop_preexisting_rejection", events)
        self.assertIn("safety_stop_write_attempt", events)
        self.assertIn("safety_stop_write_complete", events)

    def test_nonwritable_stop_respects_absolute_deadline_and_requires_estop(self):
        serial_port = SafetyStopHarness(fail_write=True)
        started_at = time.monotonic()
        deadline = started_at + 0.06

        with self.assertRaisesRegex(RuntimeError, "EMERGENCY STOP"):
            MODULE.stop(serial_port, "test-stop", absolute_deadline=deadline)

        elapsed = time.monotonic() - started_at
        self.assertLessEqual(elapsed, 0.08)
        self.assertEqual(serial_port.write_calls[0][2], deadline)
        failure = [
            event for event in serial_port.recorder.events
            if event["event"] == "safety_stop_failure"
        ][0]
        self.assertEqual(failure["outcome"], "zero_write_incomplete")
        self.assertEqual(failure["operator_action"], MODULE.ESTOP_GUIDANCE)

    def test_current_rejection_is_reported_only_after_zero_write_attempt(self):
        serial_port = SafetyStopHarness(current_response=b"-\r")

        with self.assertRaisesRegex(RuntimeError, "EMERGENCY STOP"):
            MODULE.stop(
                serial_port, "test-stop", absolute_deadline=time.monotonic() + 0.1)

        self.assertEqual(serial_port.write_calls[0][0], MODULE.ALL_MODE_ZERO_PAYLOAD)
        failure = [
            event for event in serial_port.recorder.events
            if event["event"] == "safety_stop_failure"
        ][0]
        self.assertEqual(failure["outcome"], "explicit_rejection_after_zero_write")


class ActionSequenceTest(unittest.TestCase):
    def motion_args(self):
        parser = MODULE.build_parser()
        args = parser.parse_args([
            "--output", "/tmp/not-created", "--timeout", "0.01", "motion",
            "--confirm-wheels-lifted-and-estop-accessible",
            "--channel-1-rpm", "5", "--channel-2-rpm", "0",
            "--duration", "0.17", "--count", "1", "--interval", "0.02",
            "--post-stop-count", "2",
        ])
        MODULE.validate_args(args, parser)
        return args

    def test_stop_uses_both_modes_and_failure_propagates(self):
        serial_port = FakeActionSerial(fail_labels={"test-stop"})

        with self.assertRaises(TimeoutError):
            MODULE.stop(serial_port, "test-stop")

        self.assertEqual(
            serial_port.commands[0],
            (b"!G 1 0\r!G 2 0\r!S 1 0\r!S 2 0\r", "test-stop"),
        )

    def test_abort_before_motion_never_writes_nonzero_command(self):
        serial_port = FakeActionSerial()

        with self.assertRaises(InterruptedError):
            MODULE.execute_action(self.motion_args(), serial_port, [True])

        self.assertEqual([item[1] for item in serial_port.commands], ["initial-stop"])
        self.assertNotIn(b"!S 1 5\r", b"".join(item[0] for item in serial_port.commands))

    def test_representative_motion_sequence_stops_then_polls(self):
        serial_port = FakeActionSerial()
        stop_requested = [False]

        MODULE.execute_action(self.motion_args(), serial_port, stop_requested)

        self.assertEqual(
            [item[1] for item in serial_port.commands],
            ["initial-stop", "motion-start", "motion-stop"],
        )
        self.assertEqual(
            [item[2] for item in serial_port.queries],
            [
                "mmod-channel-1", "mmod-channel-2", "motion-cr-0000",
                "cr-0000", "cr-0001",
            ],
        )

    def test_malformed_motion_encoder_reply_fails_evidence_after_stop(self):
        class MalformedEncoderSerial(FakeActionSerial):
            def query(self, data, expected_prefix, label, timeout=None):
                response = super().query(data, expected_prefix, label, timeout)
                if label.startswith("motion-cr-"):
                    return b"CR=garbage\r"
                return response

        serial_port = MalformedEncoderSerial()

        with self.assertRaisesRegex(RuntimeError, r"not exactly CR=<int32>:<int32>"):
            MODULE.execute_action(self.motion_args(), serial_port, [False])

        self.assertIn("motion-stop", [item[1] for item in serial_port.commands])
        outcome = [
            event for event in serial_port.recorder.events
            if event["event"] == "motion_action_outcome"
        ][-1]
        self.assertEqual(outcome["outcome"], "motion_failed")
        self.assertFalse(outcome["evidence_valid"])
        self.assertEqual(outcome["completed_motion_polls"], 0)

    def test_mmod_match_uses_exact_queries_and_records_validation(self):
        serial_port = FakeActionSerial()

        MODULE.validate_closed_loop_modes(serial_port)

        self.assertEqual(
            [(item[0], item[1]) for item in serial_port.queries],
            [(b"~MMOD 1\r", b"MMOD="), (b"~MMOD 2\r", b"MMOD=")],
        )
        validations = [
            event for event in serial_port.recorder.events
            if event["event"] == "mmod_validation"
        ]
        self.assertEqual([event["outcome"] for event in validations], ["match", "match"])
        self.assertEqual([event["actual"] for event in validations], [1, 1])

    def test_mmod_failures_prevent_nonzero_write(self):
        cases = {
            "mismatch": {1: b"MMOD=0\r", 2: b"MMOD=1\r"},
            "malformed": {1: b"MMOD=not-an-int\r", 2: b"MMOD=1\r"},
            "missing": {1: TimeoutError("missing MMOD response"), 2: b"MMOD=1\r"},
        }
        for name, responses in cases.items():
            with self.subTest(name=name):
                serial_port = FakeActionSerial(mmod_responses=responses)

                with self.assertRaises(RuntimeError):
                    MODULE.execute_action(self.motion_args(), serial_port, [False])

                self.assertEqual(
                    [item[1] for item in serial_port.commands], ["initial-stop"])
                self.assertNotIn(
                    b"!S 1 5\r", b"".join(item[0] for item in serial_port.commands))
                validation = serial_port.recorder.events[-1]
                self.assertEqual(validation["event"], "mmod_validation")
                self.assertEqual(
                    validation["outcome"],
                    {"mismatch": "mismatch", "malformed": "malformed",
                     "missing": "query_failed"}[name],
                )


class TimedActionSerial(FakeActionSerial):
    def __init__(self, timeout, delayed_rejection=None):
        super().__init__()
        self.timeout = timeout
        self.delayed_rejection = delayed_rejection
        self.motion_write_time = None
        self.motion_stop_write_time = None
        self.in_motion_queries = 0

    def command(self, data, label, abort_requested=None):
        if abort_requested is not None and abort_requested():
            raise InterruptedError("abort requested before command write")
        time.sleep(min(MODULE.COMMAND_PREWRITE_DRAIN_SECONDS, self.timeout))
        write_completed_at = time.monotonic()
        self.last_command_write_completed_at = write_completed_at
        self.commands.append((data, label))
        if label == "motion-start":
            self.motion_write_time = write_completed_at
            if self.delayed_rejection is not None:
                time.sleep(self.delayed_rejection)
                raise RuntimeError("injected delayed explicit rejection")
            time.sleep(self.timeout)
        return write_completed_at

    def safety_stop(self, label, absolute_deadline=None):
        time.sleep(min(MODULE.COMMAND_PREWRITE_DRAIN_SECONDS, self.timeout))
        write_completed_at = time.monotonic()
        if absolute_deadline is not None and write_completed_at > absolute_deadline:
            raise TimeoutError("injected safety stop deadline exceeded")
        self.commands.append((MODULE.ALL_MODE_ZERO_PAYLOAD, label))
        if label == "motion-stop":
            self.motion_stop_write_time = write_completed_at
        return write_completed_at

    def query(self, data, expected_prefix, label, timeout=None):
        self.queries.append((data, expected_prefix, label, timeout))
        if label.startswith("mmod-channel-"):
            return b"MMOD=1\r"
        if label.startswith("motion-cr-"):
            self.in_motion_queries += 1
            time.sleep(0.01)
        return b"CR=0:0\r"


class MotionTimingTest(unittest.TestCase):
    def default_motion_args(self):
        parser = MODULE.build_parser()
        args = parser.parse_args([
            "--output", "/tmp/not-created", "motion",
            "--confirm-wheels-lifted-and-estop-accessible",
            "--channel-1-rpm", "5", "--channel-2-rpm", "5",
            "--post-stop-count", "1",
        ])
        MODULE.validate_args(args, parser)
        return args

    def test_default_motion_polls_and_starts_stop_within_duration(self):
        args = self.default_motion_args()
        serial_port = TimedActionSerial(args.timeout)

        MODULE.execute_action(args, serial_port, [False])

        self.assertGreaterEqual(serial_port.in_motion_queries, 1)
        elapsed = serial_port.motion_stop_write_time - serial_port.motion_write_time
        self.assertLessEqual(elapsed, args.duration + 0.025)

    def test_final_query_is_skipped_and_stop_write_meets_motion_deadline(self):
        parser = MODULE.build_parser()
        args = parser.parse_args([
            "--output", "/tmp/not-created", "--timeout", "0.01", "motion",
            "--confirm-wheels-lifted-and-estop-accessible",
            "--channel-1-rpm", "5", "--channel-2-rpm", "5",
            "--duration", "0.19", "--count", "8", "--interval", "0.05",
            "--post-stop-count", "1",
        ])
        MODULE.validate_args(args, parser)

        class BoundaryTimedSerial(TimedActionSerial):
            def query(self, data, expected_prefix, label, timeout=None):
                response = super().query(data, expected_prefix, label, timeout)
                if label.startswith("motion-cr-"):
                    time.sleep(min(MODULE.COMMAND_PREWRITE_DRAIN_SECONDS, self.timeout))
                return response

        serial_port = BoundaryTimedSerial(args.timeout)

        MODULE.execute_action(args, serial_port, [False])

        motion_queries = [
            item[2] for item in serial_port.queries
            if item[2].startswith("motion-cr-")
        ]
        self.assertEqual(motion_queries, ["motion-cr-0000"])
        elapsed = serial_port.motion_stop_write_time - serial_port.motion_write_time
        self.assertLessEqual(elapsed, args.duration + 0.025)

    def test_delayed_rejection_triggers_early_stop_and_failure(self):
        args = self.default_motion_args()
        serial_port = TimedActionSerial(args.timeout, delayed_rejection=0.10)

        with self.assertRaises(RuntimeError):
            MODULE.execute_action(args, serial_port, [False])

        elapsed = serial_port.motion_stop_write_time - serial_port.motion_write_time
        self.assertLessEqual(elapsed, 0.18)
        self.assertLess(elapsed, args.duration)
        self.assertEqual(serial_port.in_motion_queries, 0)

    def test_zero_completed_motion_polls_cannot_succeed(self):
        args = self.default_motion_args()

        class ExhaustedDeadlineSerial(FakeActionSerial):
            def command(self, data, label, abort_requested=None):
                completed_at = super().command(data, label, abort_requested)
                if label == "motion-start":
                    return completed_at - args.duration
                return completed_at

        serial_port = ExhaustedDeadlineSerial()

        with self.assertRaisesRegex(RuntimeError, r"no in-motion \?CR"):
            MODULE.execute_action(args, serial_port, [False])

        self.assertNotIn(
            "motion-cr-0000", [item[2] for item in serial_port.queries])
        self.assertIn("motion-stop", [item[1] for item in serial_port.commands])

    def test_stop_request_after_one_poll_aborts_evidence_and_returns_failure(self):
        args = self.default_motion_args()
        stop_requested = [False]

        class StopAfterFirstPollSerial(FakeActionSerial):
            def query(self, data, expected_prefix, label, timeout=None):
                response = super().query(data, expected_prefix, label, timeout)
                if label == "motion-cr-0000":
                    stop_requested[0] = True
                return response

        serial_port = StopAfterFirstPollSerial()

        with self.assertRaisesRegex(InterruptedError, "evidence is invalid"):
            MODULE.execute_action(args, serial_port, stop_requested)

        self.assertEqual(
            [item[2] for item in serial_port.queries if item[2].startswith("motion-cr-")],
            ["motion-cr-0000"],
        )
        self.assertNotIn("cr-0000", [item[2] for item in serial_port.queries])
        motion_stop = [item for item in serial_port.commands if item[1] == "motion-stop"]
        self.assertEqual(motion_stop, [(MODULE.ALL_MODE_ZERO_PAYLOAD, "motion-stop")])
        outcome = [
            event for event in serial_port.recorder.events
            if event["event"] == "motion_action_outcome"
        ][-1]
        self.assertEqual(outcome["outcome"], "aborted_by_stop_request")
        self.assertFalse(outcome["evidence_valid"])
        self.assertEqual(outcome["completed_motion_polls"], 1)
        self.assertEqual(outcome["post_stop_polling"], "skipped")


if __name__ == "__main__":
    unittest.main()
