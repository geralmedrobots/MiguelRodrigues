#!/usr/bin/env python3
"""Bounded, isolated Roboteq ?CR hardware-validation logger.

This tool intentionally does not import ROS or the production driver. It never
writes controller settings. Non-zero output is limited to short, low-RPM !S
commands and requires an explicit mechanical-safety acknowledgement.
"""

import argparse
import datetime
import fcntl
import json
import os
import select
import signal
import stat
import sys
import termios
import time
import tty
import uuid


MAX_ABS_RPM = 20
MAX_MOTION_SECONDS = 2.0
MAX_QUERY_SECONDS = 1.0
MAX_POLL_COUNT = 200
MIN_POLL_INTERVAL_SECONDS = 0.02
MAX_POLL_INTERVAL_SECONDS = 5.0
MAX_RAW_BYTES = 4096
EXPECTED_CLOSED_LOOP_MMOD = 1
COMMAND_PREWRITE_DRAIN_SECONDS = 0.05
TIMING_MARGIN_SECONDS = 0.10
ALL_MODE_ZERO_PAYLOAD = b"!G 1 0\r!G 2 0\r!S 1 0\r!S 2 0\r"
ESTOP_GUIDANCE = "ALL-MODE ZERO STOP FAILED; USE OPERATOR EMERGENCY STOP"


def visible_bytes(data):
    result = []
    for value in data:
        if value == 0x0D:
            result.append(r"\r")
        elif value == 0x0A:
            result.append(r"\n")
        elif value == 0x09:
            result.append(r"\t")
        elif value == 0x5C:
            result.append(r"\\")
        elif 0x20 <= value <= 0x7E:
            result.append(chr(value))
        else:
            result.append(f"\\x{value:02X}")
    return "".join(result)


def contains_explicit_rejection(data):
    complete_lines = data.replace(b"\r", b"\n").split(b"\n")[:-1]
    return any(line == b"-" for line in complete_lines)


class EvidenceRecorder:
    def __init__(self, path):
        self._stream = open(path, "x", encoding="utf-8", buffering=1)

    def close(self):
        self._stream.flush()
        os.fsync(self._stream.fileno())
        self._stream.close()

    def record(self, event, **fields):
        item = {
            "wall_time_utc": datetime.datetime.now(
                datetime.timezone.utc).isoformat(timespec="microseconds"),
            "monotonic_ns": time.monotonic_ns(),
            "event": event,
            **fields,
        }
        line = json.dumps(item, sort_keys=True, separators=(",", ":"))
        print(line, flush=True)
        self._stream.write(line + "\n")

    def bytes(self, event, data, **fields):
        self.record(
            event,
            raw_hex=data.hex(" ").upper(),
            raw_visible=visible_bytes(data),
            raw_length=len(data),
            **fields,
        )


def processes_holding_device(path):
    device = os.stat(path)
    if not stat.S_ISCHR(device.st_mode):
        raise RuntimeError(f"serial path is not a character device: {path}")
    users = []
    own_pid = os.getpid()
    for pid_text in os.listdir("/proc"):
        if not pid_text.isdigit() or int(pid_text) == own_pid:
            continue
        fd_dir = f"/proc/{pid_text}/fd"
        try:
            fd_names = os.listdir(fd_dir)
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        for fd_name in fd_names:
            try:
                opened = os.stat(f"{fd_dir}/{fd_name}")
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
            if stat.S_ISCHR(opened.st_mode) and opened.st_rdev == device.st_rdev:
                users.append((int(pid_text), int(fd_name)))
    return sorted(set(users))


class BoundedSerial:
    def __init__(self, path, baud, recorder, timeout):
        self.path = path
        self.baud = baud
        self.recorder = recorder
        self.timeout = timeout
        self.fd = None
        self.last_command_write_completed_at = None

    def open(self):
        users = processes_holding_device(self.path)
        if users:
            detail = ", ".join(f"pid={pid}/fd={fd}" for pid, fd in users)
            raise RuntimeError(f"serial device is already open ({detail})")
        self.fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if hasattr(termios, "TIOCEXCL"):
                fcntl.ioctl(self.fd, termios.TIOCEXCL)
            users = processes_holding_device(self.path)
            if users:
                detail = ", ".join(f"pid={pid}/fd={fd}" for pid, fd in users)
                raise RuntimeError(f"serial device raced with another opener ({detail})")
            tty.setraw(self.fd, when=termios.TCSANOW)
            attrs = termios.tcgetattr(self.fd)
            speed = getattr(termios, f"B{self.baud}", None)
            if speed is None:
                raise RuntimeError(f"unsupported baud rate: {self.baud}")
            attrs[4] = speed
            attrs[5] = speed
            attrs[2] |= termios.CLOCAL | termios.CREAD
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        except Exception:
            self.close()
            raise
        self.recorder.record("serial_open", port=self.path, baud=self.baud)

    def close(self):
        if self.fd is not None:
            fd = self.fd
            self.fd = None
            try:
                os.close(fd)
            finally:
                self.recorder.record("serial_close", port=self.path)

    def _write_all(self, data, transaction_id, deadline=None):
        if deadline is None:
            deadline = time.monotonic() + self.timeout
        offset = 0
        while offset < len(data):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("bounded serial write timed out")
            _, writable, _ = select.select([], [self.fd], [], remaining)
            if not writable:
                raise TimeoutError("bounded serial write timed out")
            offset += os.write(self.fd, data[offset:])
        write_completed_at = time.monotonic()
        self.recorder.bytes("tx", data, transaction_id=transaction_id)
        return write_completed_at

    def _read_available(self, deadline, transaction_id, event):
        chunks = []
        total = 0
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            readable, _, _ = select.select([self.fd], [], [], max(0.0, remaining))
            if not readable:
                break
            chunk = os.read(self.fd, min(1024, MAX_RAW_BYTES - total))
            if not chunk:
                continue
            chunks.append(chunk)
            total += len(chunk)
            self.recorder.bytes(event, chunk, transaction_id=transaction_id)
            if total >= MAX_RAW_BYTES:
                raise RuntimeError("serial response exceeded maximum evidence size")
        return b"".join(chunks)

    def _read_command_response(self, deadline, transaction_id):
        response = bytearray()
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            readable, _, _ = select.select([self.fd], [], [], max(0.0, remaining))
            if not readable:
                break
            chunk = os.read(self.fd, min(1024, MAX_RAW_BYTES - len(response)))
            if not chunk:
                continue
            response.extend(chunk)
            self.recorder.bytes("rx_chunk", chunk, transaction_id=transaction_id)
            if len(response) >= MAX_RAW_BYTES:
                raise RuntimeError("serial command response exceeded maximum evidence size")
            if contains_explicit_rejection(response):
                return bytes(response), True
        return bytes(response), False

    def drain_preexisting(self, timeout=0.05, label="query"):
        transaction_id = f"pre-{label}-{uuid.uuid4()}"
        return self._read_available(
            time.monotonic() + max(0.0, timeout), transaction_id, "rx_preexisting")

    def command(self, data, label, abort_requested=None):
        preexisting = self.drain_preexisting(
            min(COMMAND_PREWRITE_DRAIN_SECONDS, self.timeout),
            label=f"command-{label}")
        if contains_explicit_rejection(preexisting):
            self.recorder.bytes(
                "command_aborted_before_write", preexisting, command_label=label,
                outcome="preexisting_explicit_rejection_observed")
            raise RuntimeError(
                f"preexisting explicit rejection observed before command {label}")
        transaction_id = f"{label}-{uuid.uuid4()}"
        if abort_requested is not None and abort_requested():
            self.recorder.bytes(
                "transaction_complete", b"", transaction_id=transaction_id,
                outcome="aborted_before_write")
            raise InterruptedError("abort requested before command write")
        write_completed_at = self._write_all(data, transaction_id)
        self.last_command_write_completed_at = write_completed_at
        response, rejected = self._read_command_response(
            write_completed_at + self.timeout, transaction_id)
        outcome = (
            "explicit_rejection_observed" if rejected else
            "write_complete_no_rejection_observed")
        self.recorder.bytes(
            "transaction_complete", response, transaction_id=transaction_id,
            outcome=outcome, response_capture_timeout_s=self.timeout)
        if rejected:
            raise RuntimeError(f"controller rejected command transaction {label}")
        return write_completed_at

    def safety_stop(self, label, absolute_deadline=None):
        drain_budget = min(COMMAND_PREWRITE_DRAIN_SECONDS, self.timeout)
        if absolute_deadline is None:
            absolute_deadline = time.monotonic() + drain_budget + self.timeout
        transaction_id = f"{label}-{uuid.uuid4()}"
        preexisting = self.drain_preexisting(
            min(drain_budget, max(0.0, absolute_deadline - time.monotonic())),
            label=f"safety-stop-{label}")
        preexisting_rejected = contains_explicit_rejection(preexisting)
        if preexisting_rejected:
            self.recorder.bytes(
                "safety_stop_preexisting_rejection", preexisting,
                transaction_id=transaction_id,
                outcome="preexisting_explicit_rejection_observed_continuing_stop")
        self.recorder.bytes(
            "safety_stop_write_attempt", ALL_MODE_ZERO_PAYLOAD,
            transaction_id=transaction_id, absolute_deadline=absolute_deadline)
        try:
            write_completed_at = self._write_all(
                ALL_MODE_ZERO_PAYLOAD, transaction_id, absolute_deadline)
        except Exception as error:
            self.recorder.record(
                "safety_stop_failure", transaction_id=transaction_id,
                outcome="zero_write_incomplete", message=str(error),
                operator_action=ESTOP_GUIDANCE, absolute_deadline=absolute_deadline)
            raise RuntimeError(f"{ESTOP_GUIDANCE}: {error}") from error
        self.recorder.record(
            "safety_stop_write_complete", transaction_id=transaction_id,
            write_completed_monotonic=write_completed_at,
            physical_stop_confirmed=False,
            note="zero bytes completed; physical stop requires operator observation")
        response, rejected = self._read_command_response(absolute_deadline, transaction_id)
        if rejected:
            outcome = "explicit_rejection_observed_after_zero_write"
        elif preexisting_rejected:
            outcome = "preexisting_rejection_then_zero_write_complete"
        else:
            outcome = "zero_write_complete_no_rejection_observed"
        self.recorder.bytes(
            "safety_stop_complete", response, transaction_id=transaction_id,
            outcome=outcome, physical_stop_confirmed=False)
        if rejected:
            self.recorder.record(
                "safety_stop_failure", transaction_id=transaction_id,
                outcome="explicit_rejection_after_zero_write",
                operator_action=ESTOP_GUIDANCE)
            raise RuntimeError(f"{ESTOP_GUIDANCE}: controller rejected zero stop")
        if preexisting_rejected:
            self.recorder.record(
                "safety_stop_failure", transaction_id=transaction_id,
                outcome="preexisting_explicit_rejection_before_zero_write",
                operator_action=ESTOP_GUIDANCE)
            raise RuntimeError(
                f"{ESTOP_GUIDANCE}: delayed rejection preceded zero stop")
        return write_completed_at

    def query(self, data, expected_prefix, label, timeout=None):
        query_timeout = self.timeout if timeout is None else min(self.timeout, timeout)
        deadline = time.monotonic() + max(0.0, query_timeout)
        preexisting = self.drain_preexisting(
            min(0.05, max(0.0, deadline - time.monotonic())), label=label)
        if contains_explicit_rejection(preexisting):
            self.recorder.bytes(
                "query_aborted_before_write", preexisting, query_label=label,
                outcome="preexisting_explicit_rejection_observed")
            raise RuntimeError(f"preexisting explicit rejection observed before query {label}")
        if time.monotonic() >= deadline:
            raise TimeoutError(f"query deadline elapsed before writing {data!r}")
        transaction_id = f"{label}-{uuid.uuid4()}"
        self._write_all(data, transaction_id, deadline)
        raw = bytearray()
        matched = False
        while time.monotonic() < deadline and not matched:
            readable, _, _ = select.select([self.fd], [], [], deadline - time.monotonic())
            if not readable:
                break
            chunk = os.read(self.fd, min(1024, MAX_RAW_BYTES - len(raw)))
            if not chunk:
                continue
            raw.extend(chunk)
            self.recorder.bytes("rx_chunk", chunk, transaction_id=transaction_id)
            if len(raw) >= MAX_RAW_BYTES:
                raise RuntimeError("serial query exceeded maximum evidence size")
            if contains_explicit_rejection(raw):
                self.recorder.bytes(
                    "transaction_complete", bytes(raw), transaction_id=transaction_id,
                    outcome="explicit_rejection_observed",
                    expected_prefix=expected_prefix.decode("ascii"))
                raise RuntimeError(f"controller rejected query transaction {label}")
            lines = bytes(raw).replace(b"\r", b"\n").split(b"\n")[:-1]
            matched = any(line.startswith(expected_prefix) for line in lines)
        if matched:
            raw.extend(self._read_available(
                min(deadline, time.monotonic() + 0.02), transaction_id, "rx_chunk"))
            self.recorder.bytes(
                "transaction_complete", bytes(raw), transaction_id=transaction_id,
                outcome="matched", expected_prefix=expected_prefix.decode("ascii"))
            return bytes(raw)
        self.recorder.bytes(
            "transaction_complete", bytes(raw), transaction_id=transaction_id,
            outcome="timeout", expected_prefix=expected_prefix.decode("ascii"))
        raise TimeoutError(f"query timed out waiting for {expected_prefix!r}")


def stop(serial_port, label="stop", absolute_deadline=None):
    return serial_port.safety_stop(label, absolute_deadline)


def parse_signed_int32(text):
    if not text:
        return None
    if text.startswith(b"-"):
        digits = text[1:]
    else:
        digits = text
    if not digits or any(value < ord("0") or value > ord("9") for value in digits):
        return None
    parsed = int(text)
    if parsed < -2147483648 or parsed > 2147483647:
        return None
    return parsed


def parse_single_config_integer(raw, prefix):
    lines = raw.replace(b"\r", b"\n").split(b"\n")[:-1]
    values = [line[len(prefix):] for line in lines if line.startswith(prefix)]
    if len(values) != 1:
        return None
    return parse_signed_int32(values[0])


def parse_encoder_counts(raw):
    lines = raw.replace(b"\r", b"\n").split(b"\n")[:-1]
    values = [line[len(b"CR="):] for line in lines if line.startswith(b"CR=")]
    if len(values) != 1:
        return None
    fields = values[0].split(b":")
    if len(fields) != 2:
        return None
    channel_1 = parse_signed_int32(fields[0])
    channel_2 = parse_signed_int32(fields[1])
    if channel_1 is None or channel_2 is None:
        return None
    return channel_1, channel_2


def query_encoder_counts(serial_port, label, timeout=None):
    raw = serial_port.query(b"?CR\r", b"CR=", label, timeout=timeout)
    counts = parse_encoder_counts(raw)
    if counts is None:
        serial_port.recorder.record(
            "encoder_response_validation", query_label=label, outcome="malformed",
            raw_visible=visible_bytes(raw))
        raise RuntimeError(f"{label} response is not exactly CR=<int32>:<int32>")
    serial_port.recorder.record(
        "encoder_response_validation", query_label=label, outcome="valid",
        channel_1=counts[0], channel_2=counts[1])
    return counts


def validate_closed_loop_modes(serial_port):
    for channel in (1, 2):
        command = f"~MMOD {channel}\r".encode("ascii")
        try:
            raw = serial_port.query(command, b"MMOD=", f"mmod-channel-{channel}")
        except Exception as error:
            serial_port.recorder.record(
                "mmod_validation", channel=channel, expected=EXPECTED_CLOSED_LOOP_MMOD,
                outcome="query_failed", message=str(error))
            raise RuntimeError(f"MMOD channel {channel} query failed") from error
        actual = parse_single_config_integer(raw, b"MMOD=")
        if actual is None:
            serial_port.recorder.record(
                "mmod_validation", channel=channel, expected=EXPECTED_CLOSED_LOOP_MMOD,
                outcome="malformed")
            raise RuntimeError(f"MMOD channel {channel} response is malformed")
        if actual != EXPECTED_CLOSED_LOOP_MMOD:
            serial_port.recorder.record(
                "mmod_validation", channel=channel, expected=EXPECTED_CLOSED_LOOP_MMOD,
                actual=actual, outcome="mismatch")
            raise RuntimeError(
                f"MMOD channel {channel} expected {EXPECTED_CLOSED_LOOP_MMOD}, got {actual}")
        serial_port.recorder.record(
            "mmod_validation", channel=channel, expected=EXPECTED_CLOSED_LOOP_MMOD,
            actual=actual, outcome="match")


def poll(serial_port, count, interval, stop_requested):
    for index in range(count):
        if stop_requested[0]:
            break
        query_encoder_counts(serial_port, f"cr-{index:04d}")
        if index + 1 < count:
            time.sleep(interval)


def minimum_motion_duration(timeout, interval):
    stop_drain = min(COMMAND_PREWRITE_DRAIN_SECONDS, timeout)
    return (
        timeout + timeout + interval + stop_drain + timeout +
        TIMING_MARGIN_SECONDS)


def has_query_start_budget(remaining, timeout):
    prewrite_drain = min(COMMAND_PREWRITE_DRAIN_SECONDS, timeout)
    return remaining > prewrite_drain + TIMING_MARGIN_SECONDS


def validate_args(args, parser):
    if not 0.01 <= args.timeout <= MAX_QUERY_SECONDS:
        parser.error(f"--timeout must be between 0.01 and {MAX_QUERY_SECONDS} seconds")
    if args.action in ("poll", "motion"):
        if not 1 <= args.count <= MAX_POLL_COUNT:
            parser.error(f"--count must be between 1 and {MAX_POLL_COUNT}")
        if not MIN_POLL_INTERVAL_SECONDS <= args.interval <= MAX_POLL_INTERVAL_SECONDS:
            parser.error(
                f"--interval must be between {MIN_POLL_INTERVAL_SECONDS} and "
                f"{MAX_POLL_INTERVAL_SECONDS} seconds")
    if args.action == "motion":
        if not args.confirm_wheels_lifted_and_estop_accessible:
            parser.error("motion requires --confirm-wheels-lifted-and-estop-accessible")
        if args.channel_1_rpm == 0 and args.channel_2_rpm == 0:
            parser.error("motion requires at least one non-zero channel RPM")
        if max(abs(args.channel_1_rpm), abs(args.channel_2_rpm)) > MAX_ABS_RPM:
            parser.error(f"motion RPM magnitude cannot exceed {MAX_ABS_RPM}")
        if not 0.05 <= args.duration <= MAX_MOTION_SECONDS:
            parser.error(f"--duration must be between 0.05 and {MAX_MOTION_SECONDS} seconds")
        minimum_duration = minimum_motion_duration(args.timeout, args.interval)
        if args.duration + 1e-12 < minimum_duration:
            parser.error(
                f"--duration must be at least {minimum_duration:.3f} seconds for "
                "command observation, one full encoder poll, stop drain/write, and margin")
        if not 1 <= args.post_stop_count <= MAX_POLL_COUNT:
            parser.error(f"--post-stop-count must be between 1 and {MAX_POLL_COUNT}")


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/roboteq")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--output", required=True, help="new JSONL evidence file (must not exist)")
    parser.add_argument("--validation-id", help="identifier shared by related evidence files")
    parser.add_argument("--annotation", default="", help="operator/scenario annotation")
    actions = parser.add_subparsers(dest="action", required=True)
    actions.add_parser("identify", help="stop first, then issue bounded ?FID")
    poll_parser = actions.add_parser("poll", help="stop first, then issue repeated ?CR")
    poll_parser.add_argument("--count", type=int, default=10)
    poll_parser.add_argument("--interval", type=float, default=0.1)
    motion = actions.add_parser("motion", help="short bounded !S motion while polling ?CR")
    motion.add_argument("--channel-1-rpm", type=int, required=True)
    motion.add_argument("--channel-2-rpm", type=int, required=True)
    motion.add_argument("--duration", type=float, default=1.8)
    motion.add_argument("--count", type=int, default=10)
    motion.add_argument("--interval", type=float, default=0.05)
    motion.add_argument("--post-stop-count", type=int, default=10)
    motion.add_argument(
        "--confirm-wheels-lifted-and-estop-accessible", action="store_true")
    return parser


def execute_action(args, serial_port, stop_requested):
    stop(serial_port, "initial-stop")
    if args.action == "identify":
        serial_port.query(b"?FID\r", b"FID=", "firmware-id")
    elif args.action == "poll":
        poll(serial_port, args.count, args.interval, stop_requested)
    else:
        if stop_requested[0]:
            raise InterruptedError("abort requested before non-zero motion")
        validate_closed_loop_modes(serial_port)
        if stop_requested[0]:
            raise InterruptedError("abort requested after MMOD validation")
        command = (
            f"!S 1 {args.channel_1_rpm}\r"
            f"!S 2 {args.channel_2_rpm}\r"
        ).encode("ascii")
        serial_port.last_command_write_completed_at = None
        completed_motion_polls = 0
        motion_deadline = None
        motion_error = None
        stop_error = None
        try:
            write_completed_at = serial_port.command(
                command, "motion-start", abort_requested=lambda: stop_requested[0])
            motion_deadline = write_completed_at + args.duration
            complete_stop_budget = (
                min(COMMAND_PREWRITE_DRAIN_SECONDS, serial_port.timeout) +
                serial_port.timeout)
            stop_start_deadline = motion_deadline - complete_stop_budget
            index = 0
            while (time.monotonic() < stop_start_deadline and index < args.count
                   and not stop_requested[0]):
                remaining = stop_start_deadline - time.monotonic()
                if not has_query_start_budget(remaining, serial_port.timeout):
                    break
                query_encoder_counts(
                    serial_port, f"motion-cr-{index:04d}", timeout=remaining)
                index += 1
                completed_motion_polls += 1
                time.sleep(min(
                    args.interval, max(0.0, stop_start_deadline - time.monotonic())))
        except Exception as error:
            motion_error = error
        finally:
            if motion_deadline is None:
                if serial_port.last_command_write_completed_at is not None:
                    motion_deadline = (
                        serial_port.last_command_write_completed_at + args.duration)
                else:
                    stop_budget = (
                        min(COMMAND_PREWRITE_DRAIN_SECONDS, serial_port.timeout) +
                        serial_port.timeout)
                    motion_deadline = time.monotonic() + stop_budget
            try:
                stop(serial_port, "motion-stop", absolute_deadline=motion_deadline)
            except Exception as error:
                stop_error = error
        if stop_requested[0]:
            serial_port.recorder.record(
                "motion_action_outcome", outcome="aborted_by_stop_request",
                evidence_valid=False, completed_motion_polls=completed_motion_polls,
                post_stop_polling="skipped")
            aborted = InterruptedError(
                "motion validation aborted by signal/stop request; evidence is invalid")
            if stop_error is not None:
                raise stop_error from aborted
            raise aborted from motion_error
        if stop_error is not None:
            serial_port.recorder.record(
                "motion_action_outcome", outcome="safety_stop_failed",
                evidence_valid=False, completed_motion_polls=completed_motion_polls,
                post_stop_polling="skipped")
            raise stop_error
        if motion_error is not None:
            serial_port.recorder.record(
                "motion_action_outcome", outcome="motion_failed",
                evidence_valid=False, completed_motion_polls=completed_motion_polls,
                post_stop_polling="skipped", message=str(motion_error))
            raise motion_error
        if completed_motion_polls == 0:
            serial_port.recorder.record(
                "motion_action_outcome", outcome="no_in_motion_encoder_sample",
                evidence_valid=False, completed_motion_polls=0,
                post_stop_polling="skipped")
            raise RuntimeError(
                "motion validation failed: no in-motion ?CR transaction completed")
        try:
            poll(serial_port, args.post_stop_count, args.interval, stop_requested)
        except Exception as error:
            serial_port.recorder.record(
                "motion_action_outcome", outcome="post_stop_polling_failed",
                evidence_valid=False, completed_motion_polls=completed_motion_polls,
                post_stop_polling="failed", message=str(error))
            raise
        if stop_requested[0]:
            serial_port.recorder.record(
                "motion_action_outcome", outcome="aborted_by_stop_request",
                evidence_valid=False, completed_motion_polls=completed_motion_polls,
                post_stop_polling="aborted")
            raise InterruptedError(
                "motion validation aborted during post-stop polling; evidence is invalid")
        serial_port.recorder.record(
            "motion_action_outcome", outcome="completed",
            evidence_valid=True, completed_motion_polls=completed_motion_polls,
            post_stop_polling="completed")


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(args, parser)
    recorder = EvidenceRecorder(args.output)
    serial_port = BoundedSerial(args.port, args.baud, recorder, args.timeout)
    stop_requested = [False]

    def request_stop(signum, _frame):
        stop_requested[0] = True
        recorder.record("signal", signal=signum)

    old_handlers = {
        sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)
    }
    exit_code = 0
    try:
        if args.validation_id is None:
            args.validation_id = str(uuid.uuid4())
        recorder.record(
            "session_start", action=args.action, validation_id=args.validation_id,
            annotation=args.annotation, arguments=vars(args))
        serial_port.open()
        execute_action(args, serial_port, stop_requested)
    except Exception as error:
        exit_code = 1
        recorder.record("error", error_type=type(error).__name__, message=str(error))
    finally:
        if serial_port.fd is not None:
            try:
                stop(serial_port, "final-stop")
            except Exception as error:
                exit_code = 1
                recorder.record(
                    "stop_error", error_type=type(error).__name__, message=str(error))
            serial_port.close()
        if stop_requested[0]:
            exit_code = 1
            recorder.record(
                "session_outcome", outcome="aborted_by_signal_or_stop_request",
                evidence_valid=False)
        recorder.record("session_end", exit_code=exit_code)
        recorder.close()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
