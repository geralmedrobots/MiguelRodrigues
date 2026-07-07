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

"""
Isolated, read-only Roboteq diagnostic transaction capture.

Only five compile-time query byte strings can reach the real serial write.
This module does not import ROS or production-driver code.
"""

import argparse
import dataclasses
import fcntl
import json
import os
import re
import select
import stat
import termios
import time
import tty
import uuid
from types import MappingProxyType


SCHEMA_VERSION = 2
MAX_RESPONSE_BYTES = 256
MAX_DRAIN_BYTES = 256
MAX_STARTUP_BYTES = 4096
STARTUP_QUIET_NS = 2_000_000_000
STARTUP_DEADLINE_NS = 5_000_000_000
MAX_CYCLES = 10
MAX_INTERVAL_SECONDS = 60.0
MAX_WRITE_SECONDS = 1.0
MAX_QUERY_SECONDS = 2.0


@dataclasses.dataclass(frozen=True)
class QuerySpec:
    name: str
    request: bytes
    prefix: bytes
    value_type: str


_QUERY_SPECS = (
    QuerySpec("FID", b"?FID\r", b"FID=", "text"),
    QuerySpec("FF", b"?FF\r", b"FF=", "uint8"),
    QuerySpec("FM1", b"?FM 1\r", b"FM=", "uint8"),
    QuerySpec("FM2", b"?FM 2\r", b"FM=", "uint8"),
    QuerySpec("FS", b"?FS\r", b"FS=", "uint8"),
)
QUERY_SPECS = MappingProxyType({spec.name: spec for spec in _QUERY_SPECS})
ALLOWED_REQUESTS = frozenset(spec.request for spec in _QUERY_SPECS)
NUMERIC_VALUE = re.compile(rb"[0-9]{1,3}\Z")


class RealClock:
    @staticmethod
    def monotonic_ns():
        return time.monotonic_ns()

    @staticmethod
    def sleep(seconds):
        time.sleep(seconds)


def bytes_field(data, truncated=False):
    return {
        "length": len(data),
        "hex": data.hex().upper(),
        "visible": visible_bytes(data),
        "truncated": bool(truncated),
    }


def visible_bytes(data):
    rendered = []
    for value in data:
        if value == 0x0D:
            rendered.append(r"\r")
        elif value == 0x0A:
            rendered.append(r"\n")
        elif value == 0x09:
            rendered.append(r"\t")
        elif value == 0x5C:
            rendered.append(r"\\")
        elif 0x20 <= value <= 0x7E:
            rendered.append(chr(value))
        else:
            rendered.append(f"\\x{value:02X}")
    return "".join(rendered)


def _complete_lines(data):
    """Return (lines, remainder), treating CRLF as one terminator."""
    lines = []
    start = 0
    index = 0
    while index < len(data):
        if data[index] not in (0x0D, 0x0A):
            index += 1
            continue
        lines.append(data[start:index])
        if data[index] == 0x0D and index + 1 < len(data) and data[index + 1] == 0x0A:
            index += 1
        index += 1
        start = index
    return lines, data[start:]


def parse_response(spec, raw):
    """Strictly parse a complete framed response and optional echo/ack lines."""
    lines, remainder = _complete_lines(raw)
    if remainder:
        return None, None, "partial_reply"
    response_lines = []
    echo = spec.request[:-1]
    for line in lines:
        if line == echo or line == b"+":
            if response_lines:
                return None, None, "trailing_junk"
            continue
        if line == b"-":
            return None, None, "explicit_rejection"
        response_lines.append(line)
    if len(response_lines) != 1:
        return None, None, "missing_reply" if not response_lines else "ambiguous_reply"
    line = response_lines[0]
    if not line.startswith(spec.prefix):
        return None, None, "wrong_prefix"
    payload = line[len(spec.prefix):]
    if spec.value_type == "uint8":
        if NUMERIC_VALUE.fullmatch(payload) is None:
            return None, None, "invalid_numeric_value"
        value = int(payload)
        if value > 255:
            return None, None, "numeric_overflow"
    else:
        if not payload:
            return None, None, "empty_fid"
        if any(value < 0x20 or value > 0x7E for value in payload):
            return None, None, "invalid_fid_text"
        value = payload.decode("ascii")
    return spec.prefix[:-1].decode("ascii"), value, None


class EvidenceRecorder:
    """Locked append-only JSONL writer with file-global transaction sequence."""

    def __init__(self, path):
        self._stream = open(path, "a+", encoding="utf-8", buffering=1)
        try:
            fcntl.flock(self._stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._stream.seek(0)
            lines = self._stream.readlines()
            if lines and not lines[-1].endswith("\n"):
                raise RuntimeError("evidence file has an incomplete final record")
            sequence = 0
            for line in lines:
                try:
                    item = json.loads(line)
                except json.JSONDecodeError as error:
                    raise RuntimeError("evidence file contains malformed JSONL") from error
                if not isinstance(item.get("sequence"), int):
                    raise RuntimeError("evidence file record has no integer sequence")
                sequence = max(sequence, item["sequence"])
            self._next_sequence = sequence + 1
            self._stream.seek(0, os.SEEK_END)
        except Exception:
            self._stream.close()
            raise

    def next_sequence(self):
        value = self._next_sequence
        self._next_sequence += 1
        return value

    def record(self, item):
        self._stream.write(json.dumps(item, sort_keys=True, separators=(",", ":")) + "\n")
        self._stream.flush()
        os.fsync(self._stream.fileno())

    def close(self):
        if not self._stream.closed:
            self._stream.flush()
            os.fsync(self._stream.fileno())
            self._stream.close()


class BoundedSerialEndpoint:
    """POSIX serial endpoint. The final OS-write boundary enforces the allowlist."""

    def __init__(self, path, baud, clock=None):
        self.path = path
        self.baud = baud
        self.clock = clock or RealClock()
        self.fd = None

    def open(self):
        device = os.stat(self.path)
        if not stat.S_ISCHR(device.st_mode):
            raise RuntimeError(f"serial path is not a character device: {self.path}")
        self.fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if hasattr(termios, "TIOCEXCL"):
                fcntl.ioctl(self.fd, termios.TIOCEXCL)
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

    def close(self):
        if self.fd is not None:
            fd = self.fd
            self.fd = None
            os.close(fd)

    def write(self, data, deadline_ns):
        if data not in ALLOWED_REQUESTS:
            raise ValueError("serial write rejected by immutable diagnostic-query allowlist")
        offset = 0
        while offset < len(data):
            remaining = (deadline_ns - self.clock.monotonic_ns()) / 1e9
            if remaining <= 0:
                raise TimeoutError("write_timeout")
            _, writable, _ = select.select([], [self.fd], [], remaining)
            if not writable:
                raise TimeoutError("write_timeout")
            # No other method in this tool calls os.write.
            offset += os.write(self.fd, data[offset:])

    def read(self, maximum, deadline_ns):
        remaining = (deadline_ns - self.clock.monotonic_ns()) / 1e9
        if remaining <= 0:
            return b""
        readable, _, _ = select.select([self.fd], [], [], remaining)
        if not readable:
            return b""
        return os.read(self.fd, maximum)


class DiagnosticCapture:
    def __init__(self, endpoint, recorder, clock=None, write_ns=50_000_000,
                 query_ns=500_000_000, pre_drain_ns=50_000_000,
                 quiet_drain_ns=20_000_000, failure_drain_ns=50_000_000,
                 max_response=MAX_RESPONSE_BYTES,
                 startup_quiet_ns=STARTUP_QUIET_NS,
                 startup_deadline_ns=STARTUP_DEADLINE_NS,
                 max_startup_bytes=MAX_STARTUP_BYTES, session_id=None):
        self.endpoint = endpoint
        self.recorder = recorder
        self.clock = clock or RealClock()
        self.write_ns = write_ns
        self.query_ns = query_ns
        self.pre_drain_ns = pre_drain_ns
        self.quiet_drain_ns = quiet_drain_ns
        self.failure_drain_ns = failure_drain_ns
        self.max_response = max_response
        self.startup_quiet_ns = startup_quiet_ns
        self.startup_deadline_ns = startup_deadline_ns
        self.max_startup_bytes = max_startup_bytes
        self._session_id = str(
            uuid.uuid4() if session_id is None else uuid.UUID(str(session_id)))
        self.connection_generation = 0
        self.synchronization_generation = 0
        self._active_synchronization_generation = None
        self._must_close = True

    @property
    def session_id(self):
        return self._session_id

    def open(self):
        if not self._must_close:
            raise RuntimeError("connection is already open")
        self._active_synchronization_generation = None
        self.endpoint.open()
        self.connection_generation += 1
        self._must_close = False
        try:
            item = self._synchronize_startup()
        except Exception:
            self.endpoint.close()
            self._must_close = True
            raise
        if not item["valid"]:
            self.endpoint.close()
            self._must_close = True
        return item

    def close(self):
        self.endpoint.close()
        self._must_close = True
        self._active_synchronization_generation = None

    def _synchronize_startup(self):
        """Capture unsolicited startup bytes and require continuous quiet."""
        sequence = self.recorder.next_sequence()
        startup = bytearray()
        started = self.clock.monotonic_ns()
        last_observed = started
        first_byte = None
        last_byte = None
        quiet_deadline = started + self.startup_quiet_ns
        overall_deadline = started + self.startup_deadline_ns
        completed = None
        error = None
        timed_out = False

        try:
            while True:
                now = self.clock.monotonic_ns()
                if now < last_observed:
                    error = "nonmonotonic_clock"
                    break
                last_observed = now
                # A late read must not authorize synchronization after the
                # overall bound, even if quiet would otherwise have elapsed.
                if now > overall_deadline:
                    error = "startup_synchronization_timeout"
                    timed_out = True
                    break
                # Quiet completion exactly at the overall deadline is valid.
                if quiet_deadline <= overall_deadline and now >= quiet_deadline:
                    completed = now
                    break
                if now >= overall_deadline:
                    error = "startup_synchronization_timeout"
                    timed_out = True
                    break
                read_deadline = min(quiet_deadline, overall_deadline)
                remaining = self.max_startup_bytes + 1 - len(startup)
                chunk = self.endpoint.read(max(1, remaining), read_deadline)
                observed = self.clock.monotonic_ns()
                if observed < last_observed:
                    error = "nonmonotonic_clock"
                    break
                last_observed = observed
                if not chunk:
                    continue
                if first_byte is None:
                    first_byte = observed
                last_byte = observed
                startup.extend(chunk)
                if len(startup) > self.max_startup_bytes:
                    error = "startup_bytes_exceeded"
                    break
                quiet_deadline = observed + self.startup_quiet_ns
        except Exception as caught:
            error = f"startup_read_error:{type(caught).__name__}"

        ended = self.clock.monotonic_ns()
        if ended < last_observed and error is None:
            error = "nonmonotonic_clock"
        valid = error is None
        if valid:
            self.synchronization_generation += 1
            self._active_synchronization_generation = self.synchronization_generation
        item = {
            "schema_version": SCHEMA_VERSION,
            "sequence": sequence,
            "record_type": "startup_synchronization",
            "session_id": self.session_id,
            "connection_generation": self.connection_generation,
            "synchronization_generation": (
                self._active_synchronization_generation if valid else None),
            "startup_bytes": bytes_field(
                bytes(startup), len(startup) > self.max_startup_bytes),
            "monotonic_synchronization_started": started,
            "monotonic_first_byte": first_byte,
            "monotonic_last_byte": last_byte,
            "monotonic_synchronization_complete": completed,
            "total_synchronization_duration": (
                ended - started if ended >= started else None),
            "required_continuous_quiet": self.startup_quiet_ns,
            "overall_deadline": self.startup_deadline_ns,
            "maximum_startup_bytes": self.max_startup_bytes,
            "valid": valid,
            "timeout": timed_out,
            "error": error,
            "drain_or_resynchronisation_action": (
                "startup_bytes_captured;continuous_quiet_observed"
                if valid else "startup_synchronization_failed;close_required"),
        }
        # Durability is part of synchronization: no transaction may start
        # until the complete record has been flushed and fsync'ed.
        self.recorder.record(item)
        return item

    def _drain(self, duration_ns):
        deadline = self.clock.monotonic_ns() + duration_ns
        result = bytearray()
        truncated = False
        drain_error = None
        try:
            while self.clock.monotonic_ns() < deadline:
                remaining = MAX_DRAIN_BYTES - len(result)
                if remaining <= 0:
                    truncated = True
                    break
                chunk = self.endpoint.read(remaining, deadline)
                if not chunk:
                    break
                result.extend(chunk)
        except Exception as caught:
            drain_error = type(caught).__name__
        return bytes(result), truncated, drain_error

    def transact(self, query_name):
        if query_name not in QUERY_SPECS:
            raise ValueError("query rejected by immutable diagnostic-query allowlist")
        if self._must_close:
            raise RuntimeError("connection must be opened or reopened before a transaction")
        if self._active_synchronization_generation is None:
            raise RuntimeError("valid startup synchronization is required")
        spec = QUERY_SPECS[query_name]
        sequence = self.recorder.next_sequence()
        before, before_truncated, before_drain_error = self._drain(self.pre_drain_ns)
        response = bytearray()
        after = b""
        after_truncated = False
        before_write = after_write = first_byte = response_complete = None
        parsed_prefix = parsed_value = None
        error = None
        timed_out = False
        action = "none"

        if before_drain_error is not None:
            error = "pre_write_drain_error"
            action = (
                f"pre_write_drain_failed:{before_drain_error};"
                "write_not_attempted;close_required")
        elif before or before_truncated:
            error = "preexisting_data"
            action = "pre_write_drain;write_not_attempted;close_required"
        else:
            before_write = self.clock.monotonic_ns()
            try:
                # The real endpoint repeats this exact-byte allowlist check at os.write.
                if spec.request not in ALLOWED_REQUESTS:
                    raise ValueError("request_not_allowlisted")
                self.endpoint.write(spec.request, before_write + self.write_ns)
                after_write = self.clock.monotonic_ns()
                query_deadline = before_write + self.query_ns
                while self.clock.monotonic_ns() <= query_deadline:
                    remaining = self.max_response + 1 - len(response)
                    chunk = self.endpoint.read(max(1, remaining), query_deadline)
                    now = self.clock.monotonic_ns()
                    if not chunk:
                        break
                    if first_byte is None:
                        first_byte = now
                    response.extend(chunk)
                    if len(response) > self.max_response:
                        error = "oversized_reply"
                        break
                    lines, remainder = _complete_lines(response)
                    if not remainder:
                        echo = spec.request[:-1]
                        if any(line not in (echo, b"+") for line in lines):
                            response_complete = now
                            break
                if error is None and response_complete is None:
                    timed_out = True
                    error = "response_timeout" if not response else "partial_reply_timeout"
                if error is None:
                    after, after_truncated, drain_error = self._drain(self.quiet_drain_ns)
                    combined = bytes(response) + after
                    parsed_prefix, parsed_value, error = parse_response(spec, combined)
                    if drain_error is not None:
                        error = "post_response_drain_error"
                        parsed_prefix = parsed_value = None
                        action = f"post_response_drain_failed:{drain_error};close_required"
                    elif after or after_truncated:
                        action = "post_response_quiet_drain"
                    if after_truncated and error is None:
                        error = "oversized_drain"
                if error is not None:
                    if action == "none":
                        after, after_truncated, drain_error = self._drain(
                            self.failure_drain_ns)
                        action = "bounded_failure_drain"
                        if drain_error is not None:
                            action += f"_failed:{drain_error}"
                        action += ";close_required"
                    elif "close_required" not in action:
                        action += ";close_required"
            except TimeoutError:
                timed_out = True
                error = "write_timeout" if after_write is None else "response_timeout"
                after, after_truncated, drain_error = self._drain(self.failure_drain_ns)
                action = "bounded_failure_drain"
                if drain_error is not None:
                    action += f"_failed:{drain_error}"
                action += ";close_required"
            except Exception as caught:  # Preserve a bounded transaction record.
                error = "transport_error"
                after, after_truncated, drain_error = self._drain(self.failure_drain_ns)
                action = f"transaction_failed:{type(caught).__name__};bounded_failure_drain"
                if drain_error is not None:
                    action += f"_failed:{drain_error}"
                action += ";close_required"

        end = self.clock.monotonic_ns()
        valid = error is None
        item = {
            "schema_version": SCHEMA_VERSION,
            "sequence": sequence,
            "record_type": "transaction",
            "session_id": self.session_id,
            "connection_generation": self.connection_generation,
            "synchronization_generation": self._active_synchronization_generation,
            "query_name": query_name,
            "request_bytes": bytes_field(spec.request),
            "response_bytes": bytes_field(bytes(response), len(response) > self.max_response),
            "drain_bytes": {
                "before": bytes_field(before, before_truncated),
                "after": bytes_field(after, after_truncated),
            },
            "monotonic_before_write": before_write,
            "monotonic_after_write": after_write,
            "monotonic_first_byte": first_byte,
            "monotonic_response_complete": response_complete,
            "write_duration": (
                after_write - before_write
                if before_write is not None and after_write is not None else None),
            "first_byte_latency": (
                first_byte - after_write
                if first_byte is not None and after_write is not None else None),
            "total_transaction_duration": end - (
                before_write if before_write is not None else end),
            "parsed_prefix": parsed_prefix,
            "parsed_value": parsed_value,
            "valid": valid,
            "timeout": timed_out,
            "error": error,
            "drain_or_resynchronisation_action": action,
        }
        self.recorder.record(item)
        if not valid:
            self._must_close = True
        return item


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--write-timeout", type=float, default=0.05)
    parser.add_argument("--query-timeout", type=float, default=0.5)
    return parser


def validate_args(args, parser):
    if not 1 <= args.cycles <= MAX_CYCLES:
        parser.error(f"--cycles must be between 1 and {MAX_CYCLES}")
    if not 0 <= args.interval <= MAX_INTERVAL_SECONDS:
        parser.error(f"--interval must be between 0 and {MAX_INTERVAL_SECONDS}")
    if not 0 < args.write_timeout <= MAX_WRITE_SECONDS:
        parser.error(f"--write-timeout must be > 0 and <= {MAX_WRITE_SECONDS}")
    if not 0 < args.query_timeout <= MAX_QUERY_SECONDS:
        parser.error(f"--query-timeout must be > 0 and <= {MAX_QUERY_SECONDS}")


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(args, parser)
    recorder = EvidenceRecorder(args.output)
    endpoint = BoundedSerialEndpoint(args.port, args.baud)
    capture = DiagnosticCapture(
        endpoint, recorder,
        write_ns=int(args.write_timeout * 1e9),
        query_ns=int(args.query_timeout * 1e9),
    )
    try:
        startup = capture.open()
        if not startup["valid"]:
            return 1
        names = ["FID"] + [name for _ in range(args.cycles) for name in ("FF", "FM1", "FM2", "FS")]
        for index, name in enumerate(names):
            result = capture.transact(name)
            if not result["valid"]:
                return 1
            if index + 1 < len(names):
                capture.clock.sleep(args.interval)
        return 0
    finally:
        capture.close()
        recorder.close()


if __name__ == "__main__":
    raise SystemExit(main())
