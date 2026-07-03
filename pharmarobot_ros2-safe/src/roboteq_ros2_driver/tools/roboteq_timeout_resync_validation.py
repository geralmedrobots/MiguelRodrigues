#!/usr/bin/env python3
"""Read-only Roboteq Phase 3 timeout and resynchronisation validation.

This diagnostic-only program has four fixed modes.  The only bytes that can
reach the serial endpoint are the five queries allowlisted by the Phase 2
capture tool.  It does not import ROS or production-driver code.
"""

import argparse
import fcntl
import json
import os
import select
import uuid

import roboteq_diagnostic_capture as phase2


SCHEMA_VERSION = 3
NORMAL_DEADLINE_NS = 100_000_000
WRITE_DEADLINE_NS = 50_000_000
MAX_RESPONSE_BYTES = 256
MAX_ATTEMPTS = 3
DRAIN_HORIZON_NS = 100_000_000
DRAIN_ABSOLUTE_NS = 120_000_000
DRAIN_QUIET_NS = 20_000_000
MAX_DRAIN_BYTES = 4096
POST_SYNC_QUIET_NS = 20_000_000
POST_SYNC_ABSOLUTE_NS = 50_000_000
MAX_POST_SYNC_BYTES = 4096


class ObservedBoundedSerialEndpoint(phase2.BoundedSerialEndpoint):
    """Phase 2 endpoint plus exact evidence of a partially transmitted write."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.last_write_prefix = b""

    def write(self, data, deadline_ns):
        if data not in phase2.ALLOWED_REQUESTS:
            raise ValueError("serial write rejected by immutable diagnostic-query allowlist")
        self.last_write_prefix = b""
        while len(self.last_write_prefix) < len(data):
            remaining = (deadline_ns - self.clock.monotonic_ns()) / 1e9
            if remaining <= 0:
                raise TimeoutError("write_timeout")
            _, writable, _ = select.select([], [self.fd], [], remaining)
            if not writable:
                raise TimeoutError("write_timeout")
            offset = len(self.last_write_prefix)
            written = os.write(self.fd, data[offset:])
            if written <= 0:
                raise OSError("serial write made no progress")
            self.last_write_prefix += data[offset:offset + written]


class NewEvidenceRecorder:
    """Create one new locked JSONL file and fsync every complete record."""

    def __init__(self, path):
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        fd = os.open(path, flags, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._stream = os.fdopen(fd, "w", encoding="utf-8", buffering=1)
        except Exception:
            os.close(fd)
            raise
        self._next_sequence = 1

    def record(self, item):
        item = dict(item)
        item["sequence"] = self._next_sequence
        self._next_sequence += 1
        self._stream.write(json.dumps(item, sort_keys=True, separators=(",", ":")) + "\n")
        self._stream.flush()
        os.fsync(self._stream.fileno())
        return item

    def close(self):
        if not self._stream.closed:
            self._stream.flush()
            os.fsync(self._stream.fileno())
            self._stream.close()


def _empty_bytes():
    return phase2.bytes_field(b"")


def _lines(raw):
    return phase2._complete_lines(raw)  # Reuse the reviewed Phase 2 framing.


class TimeoutResyncValidation:
    """Single-session state machine with explicit framing authority."""

    def __init__(self, endpoint, recorder, clock=None, session_id=None,
                 startup_quiet_ns=phase2.STARTUP_QUIET_NS,
                 startup_deadline_ns=phase2.STARTUP_DEADLINE_NS,
                 drain_horizon_ns=DRAIN_HORIZON_NS,
                 drain_absolute_ns=DRAIN_ABSOLUTE_NS,
                 drain_quiet_ns=DRAIN_QUIET_NS,
                 max_drain_bytes=MAX_DRAIN_BYTES,
                 post_sync_quiet_ns=POST_SYNC_QUIET_NS,
                 post_sync_absolute_ns=POST_SYNC_ABSOLUTE_NS,
                 max_post_sync_bytes=MAX_POST_SYNC_BYTES):
        self.endpoint = endpoint
        self.recorder = recorder
        self.clock = clock or phase2.RealClock()
        self.session_id = str(uuid.uuid4() if session_id is None else uuid.UUID(str(session_id)))
        self.startup_quiet_ns = startup_quiet_ns
        self.startup_deadline_ns = startup_deadline_ns
        self.drain_horizon_ns = drain_horizon_ns
        self.drain_absolute_ns = drain_absolute_ns
        self.drain_quiet_ns = drain_quiet_ns
        self.max_drain_bytes = max_drain_bytes
        self.post_sync_quiet_ns = post_sync_quiet_ns
        self.post_sync_absolute_ns = post_sync_absolute_ns
        self.max_post_sync_bytes = max_post_sync_bytes
        self.connection_generation = 0
        self.synchronization_generation = 0
        self.active_synchronization_generation = None
        self.framing_state = "closed"
        self.last_timeout = None

    def _base(self, record_type):
        return {
            "schema_version": SCHEMA_VERSION,
            "record_type": record_type,
            "session_id": self.session_id,
            "connection_generation": self.connection_generation,
            "synchronization_generation": self.active_synchronization_generation,
            "query_name": None,
            "request_bytes": _empty_bytes(),
            "transmitted_request_bytes": _empty_bytes(),
            "response_bytes": _empty_bytes(),
            "overall_deadline": None,
            "monotonic_before_write": None,
            "monotonic_after_write": None,
            "monotonic_first_byte": None,
            "monotonic_response_complete": None,
            "timeout": False,
            "timeout_phase": None,
            "parsed_prefix": None,
            "parsed_value": None,
            "valid": False,
            "error": None,
            "abort_reason": None,
            "pre_drain_bytes": _empty_bytes(),
            "post_drain_bytes": _empty_bytes(),
            "bytes_received_after_timeout": _empty_bytes(),
            "synchronisation_query": None,
            "synchronisation_response": _empty_bytes(),
            "recovery_policy": None,
            "reply_belonged_to_old_generation": False,
            "recovery_duration": None,
            "final_framing_state": self.framing_state,
        }

    def close(self):
        self.endpoint.close()
        self.active_synchronization_generation = None
        self.framing_state = "closed"

    def open_and_synchronize(self, recovery_policy="initial"):
        if self.framing_state != "closed":
            raise RuntimeError("connection_not_closed")
        open_started = self.clock.monotonic_ns()
        try:
            self.endpoint.open()
        except Exception as caught:
            item = self._base("recovery_action")
            item.update({
                "action": "serial_open", "recovery_policy": recovery_policy,
                "recovery_duration": self.clock.monotonic_ns() - open_started,
                "final_framing_state": "closed", "valid": False,
                "error": "serial_open_error:" + type(caught).__name__,
                "abort_reason": "serial_open_failed",
            })
            self.recorder.record(item)
            return item
        self.connection_generation += 1
        self.framing_state = "unresolved"
        started = self.clock.monotonic_ns()
        previous = started
        last_byte = None
        first_byte = None
        data = bytearray()
        quiet_deadline = started + self.startup_quiet_ns
        overall = started + self.startup_deadline_ns
        error = None
        complete = None
        try:
            while True:
                now = self.clock.monotonic_ns()
                if now < previous:
                    error = "nonmonotonic_clock"
                    break
                previous = now
                if now > overall:
                    error = "startup_synchronization_timeout"
                    break
                if now >= quiet_deadline and quiet_deadline <= overall:
                    complete = now
                    break
                if now >= overall:
                    error = "startup_synchronization_timeout"
                    break
                chunk = self.endpoint.read(
                    max(1, phase2.MAX_STARTUP_BYTES + 1 - len(data)),
                    min(quiet_deadline, overall))
                observed = self.clock.monotonic_ns()
                if observed < previous:
                    error = "nonmonotonic_clock"
                    break
                previous = observed
                if not chunk:
                    continue
                if first_byte is None:
                    first_byte = observed
                last_byte = observed
                data.extend(chunk)
                if len(data) > phase2.MAX_STARTUP_BYTES:
                    error = "startup_bytes_exceeded"
                    break
                quiet_deadline = observed + self.startup_quiet_ns
        except Exception as caught:
            error = "startup_read_error:" + type(caught).__name__
        ended = self.clock.monotonic_ns()
        if ended < previous and error is None:
            error = "nonmonotonic_clock"
        valid = error is None
        old_generation_reply = False
        if self.last_timeout is not None and self.connection_generation > self.last_timeout[
                "connection_generation"]:
            old_spec = phase2.QUERY_SPECS[self.last_timeout["query_name"]]
            complete_lines, _ = _lines(bytes(data))
            old_generation_reply = any(
                phase2.parse_response(old_spec, line + b"\r")[2] is None
                for line in complete_lines)
        if valid:
            self.synchronization_generation += 1
            self.active_synchronization_generation = self.synchronization_generation
            self.framing_state = "synchronized"
        item = self._base("recovery_action")
        item.update({
            "action": "startup_synchronization",
            "recovery_policy": recovery_policy,
            "startup_bytes": phase2.bytes_field(bytes(data), len(data) > phase2.MAX_STARTUP_BYTES),
            "pre_drain_bytes": _empty_bytes(), "post_drain_bytes": _empty_bytes(),
            "bytes_received_after_timeout": (
                phase2.bytes_field(bytes(data)) if old_generation_reply else _empty_bytes()),
            "monotonic_started": started, "monotonic_first_byte": first_byte,
            "monotonic_last_byte": last_byte, "monotonic_completed": complete,
            "recovery_duration": ended - started if ended >= started else None,
            "overall_deadline": self.startup_deadline_ns,
            "timeout_phase": "startup_synchronization" if error and "timeout" in error else None,
            "synchronisation_query": None, "synchronisation_response": _empty_bytes(),
            "reply_belonged_to_old_generation": old_generation_reply,
            "final_framing_state": self.framing_state,
            "valid": valid, "error": error, "abort_reason": None if valid else error,
        })
        # This fsync is the authority for the first subsequent write.
        self.recorder.record(item)
        if not valid:
            self.close()
        return item

    def transaction(self, query_name, deadline_ns, purpose="normal",
                    permit_unresolved_probe=False):
        if query_name not in phase2.QUERY_SPECS:
            raise ValueError("query rejected by immutable diagnostic-query allowlist")
        if deadline_ns <= 0:
            raise ValueError("deadline must be positive")
        if self.framing_state != "synchronized" and not (
                permit_unresolved_probe and self.framing_state in (
                    "unresolved", "drained", "recovery_sync")):
            raise RuntimeError("framing_not_synchronized")
        spec = phase2.QUERY_SPECS[query_name]
        response = bytearray()
        after_timeout = bytearray()
        before_write = self.clock.monotonic_ns()
        after_write = first_byte = complete = None
        deadline = before_write + deadline_ns
        timeout_phase = None
        error = None
        parsed_prefix = parsed_value = None
        previous = before_write
        if hasattr(self.endpoint, "last_write_prefix"):
            self.endpoint.last_write_prefix = b""
        try:
            if spec.request not in phase2.ALLOWED_REQUESTS:
                raise ValueError("request_not_allowlisted")
            self.endpoint.write(spec.request, min(deadline, before_write + WRITE_DEADLINE_NS))
            after_write = self.clock.monotonic_ns()
            if after_write < previous:
                raise RuntimeError("nonmonotonic_clock")
            previous = after_write
            while True:
                now = self.clock.monotonic_ns()
                if now < previous:
                    error = "nonmonotonic_clock"
                    break
                previous = now
                if now >= deadline:
                    timeout_phase = "response"
                    error = "response_timeout" if not response else "partial_reply_timeout"
                    break
                chunk = self.endpoint.read(MAX_RESPONSE_BYTES + 1 - len(response), deadline)
                observed = self.clock.monotonic_ns()
                if observed < previous:
                    error = "nonmonotonic_clock"
                    break
                previous = observed
                if not chunk:
                    timeout_phase = "response"
                    error = "response_timeout" if not response else "partial_reply_timeout"
                    break
                if observed > deadline:
                    after_timeout.extend(chunk)
                    timeout_phase = "response"
                    error = "response_timeout"
                    break
                if first_byte is None:
                    first_byte = observed
                response.extend(chunk)
                if len(response) > MAX_RESPONSE_BYTES:
                    error = "oversized_reply"
                    break
                lines, remainder = _lines(response)
                if not remainder:
                    echo = spec.request[:-1]
                    if any(line not in (echo, b"+") for line in lines):
                        complete = observed
                        break
            if error is None:
                parsed_prefix, parsed_value, error = phase2.parse_response(spec, bytes(response))
        except TimeoutError:
            timeout_phase = "write" if after_write is None else "response"
            error = timeout_phase + "_timeout"
        except Exception as caught:
            error = "transport_error:" + type(caught).__name__
        ended = self.clock.monotonic_ns()
        transmitted = getattr(
            self.endpoint, "last_write_prefix",
            spec.request if after_write is not None else b"")
        valid = error is None
        if not valid:
            self.framing_state = "unresolved"
        item = self._base("transaction")
        item.update({
            "query_name": query_name, "purpose": purpose,
            "request_bytes": phase2.bytes_field(spec.request),
            "transmitted_request_bytes": phase2.bytes_field(transmitted),
            "response_bytes": phase2.bytes_field(bytes(response), len(response) > MAX_RESPONSE_BYTES),
            "bytes_received_after_timeout": phase2.bytes_field(bytes(after_timeout)),
            "pre_drain_bytes": _empty_bytes(), "post_drain_bytes": _empty_bytes(),
            "overall_deadline": deadline_ns,
            "monotonic_before_write": before_write, "monotonic_after_write": after_write,
            "monotonic_first_byte": first_byte, "monotonic_response_complete": complete,
            "monotonic_transaction_end": ended,
            "first_byte_latency": (
                first_byte - after_write
                if first_byte is not None and after_write is not None else None),
            "total_transaction_duration": ended - before_write if ended >= before_write else None,
            "timeout": timeout_phase is not None, "timeout_phase": timeout_phase,
            "parsed_prefix": parsed_prefix, "parsed_value": parsed_value,
            "telemetry_state": "VALID" if valid else "UNKNOWN",
            "valid": valid, "error": error,
            "recovery_policy": None, "synchronisation_query": None,
            "synchronisation_response": _empty_bytes(),
            "reply_belonged_to_old_generation": False,
            "recovery_duration": None, "final_framing_state": self.framing_state,
            "abort_reason": None,
        })
        self.recorder.record(item)
        if timeout_phase is not None:
            self.last_timeout = {
                "query_name": query_name, "write_time": before_write,
                "connection_generation": self.connection_generation,
            }
        return item

    def timeout_probe(self, query_name):
        if self.last_timeout is None or self.framing_state != "unresolved":
            raise RuntimeError("probe_requires_timeout")
        item = self.transaction(query_name, NORMAL_DEADLINE_NS,
                                purpose="distinguishable_post_timeout_probe",
                                permit_unresolved_probe=True)
        expected = phase2.QUERY_SPECS[query_name].prefix[:-1].decode("ascii")
        raw = bytes.fromhex(item["response_bytes"]["hex"])
        complete, remainder = _lines(raw)
        foreign = [line for line in complete if line and line not in (
            phase2.QUERY_SPECS[query_name].request[:-1], b"+") and
            not line.startswith((expected + "=").encode("ascii"))]
        if foreign or remainder:
            item["reply_belonged_to_old_generation"] = False
            item["error"] = "delayed_or_cross_query_reply"
            item["valid"] = False
            item["telemetry_state"] = "UNKNOWN"
            item["final_framing_state"] = "unresolved"
            self.framing_state = "unresolved"
            # Persist classification as a separate immutable correction record.
            action = self._base("recovery_action")
            action.update({
                "action": "post_timeout_probe_classification", "recovery_policy": "observe_then_close",
                "pre_drain_bytes": _empty_bytes(), "post_drain_bytes": _empty_bytes(),
                "bytes_received_after_timeout": item["response_bytes"],
                "synchronisation_query": query_name,
                "synchronisation_response": item["response_bytes"],
                "reply_belonged_to_old_generation": False,
                "recovery_duration": item["total_transaction_duration"],
                "final_framing_state": "unresolved", "valid": False,
                "error": "delayed_or_cross_query_reply", "abort_reason": "boundary_observation_complete",
            })
            self.recorder.record(action)
        return item

    def _recovery_record(self, action, policy, started, **values):
        item = self._base("recovery_action")
        item.update({
            "action": action, "recovery_policy": policy,
            "pre_drain_bytes": _empty_bytes(), "post_drain_bytes": _empty_bytes(),
            "bytes_received_after_timeout": _empty_bytes(),
            "synchronisation_query": None, "synchronisation_response": _empty_bytes(),
            "reply_belonged_to_old_generation": False,
            "recovery_duration": self.clock.monotonic_ns() - started,
            "final_framing_state": self.framing_state,
            "valid": False, "error": None, "abort_reason": None,
        })
        item.update(values)
        self.recorder.record(item)
        return item

    def reconnect_recovery(self, sync_query):
        if self.last_timeout is None or self.framing_state != "unresolved":
            raise RuntimeError("reconnect_requires_timeout")
        started = self.clock.monotonic_ns()
        old_generation = self.connection_generation
        self._recovery_record("reconnect_started", "reconnect", started,
                              final_framing_state="unresolved")
        self.close()
        startup = self.open_and_synchronize("reconnect")
        if not startup["valid"]:
            return startup
        startup_old = startup["reply_belonged_to_old_generation"]
        self.framing_state = "recovery_sync"
        result = self.transaction(sync_query, NORMAL_DEADLINE_NS,
                                  purpose="reconnect_sync_query",
                                  permit_unresolved_probe=True)
        verification = None
        if result["valid"]:
            verification = self._post_sync_verification(sync_query, result["response_bytes"])
        valid = (result["valid"] and verification is not None and verification["valid"] and
                 self.connection_generation > old_generation)
        if not valid:
            self.framing_state = "unresolved"
        return self._recovery_record(
            "reconnect_completed", "reconnect", started,
            synchronisation_query=sync_query,
            synchronisation_response=result["response_bytes"],
            post_drain_bytes=(
                verification["post_drain_bytes"] if verification is not None else _empty_bytes()),
            bytes_received_after_timeout=(
                verification["bytes_received_after_timeout"]
                if verification is not None else _empty_bytes()),
            reply_belonged_to_old_generation=(
                startup_old or (verification is not None and
                                verification["reply_belonged_to_old_generation"])),
            recovery_duration=self.clock.monotonic_ns() - started,
            final_framing_state=self.framing_state,
            valid=valid, error=None if valid else "reconnect_sync_failed",
            abort_reason=None if valid else "unresolved_framing")

    def _post_sync_verification(self, sync_query, sync_response):
        """Accept recovery only after a hard-bounded, byte-capped quiet period."""
        started = self.clock.monotonic_ns()
        quiet_deadline = started + self.post_sync_quiet_ns
        absolute = started + self.post_sync_absolute_ns
        previous = started
        data = bytearray()
        error = None
        try:
            while True:
                now = self.clock.monotonic_ns()
                if now < previous:
                    error = "nonmonotonic_clock"
                    break
                previous = now
                if now > absolute:
                    error = "post_sync_absolute_deadline_overshoot"
                    break
                if now >= quiet_deadline and quiet_deadline <= absolute:
                    break
                if now >= absolute:
                    error = "post_sync_not_quiet_before_absolute_deadline"
                    break
                chunk = self.endpoint.read(
                    max(1, self.max_post_sync_bytes + 1 - len(data)),
                    min(quiet_deadline, absolute))
                observed = self.clock.monotonic_ns()
                if observed < previous:
                    error = "nonmonotonic_clock"
                    break
                previous = observed
                if not chunk:
                    continue
                data.extend(chunk)
                if len(data) > self.max_post_sync_bytes:
                    error = "post_sync_byte_cap_exceeded"
                    break
                quiet_deadline = observed + self.post_sync_quiet_ns
        except Exception as caught:
            error = "post_sync_read_error:" + type(caught).__name__

        classification = "quiet"
        old_candidate = False
        if data:
            complete, remainder = _lines(bytes(data))
            old_spec = phase2.QUERY_SPECS[self.last_timeout["query_name"]]
            sync_spec = phase2.QUERY_SPECS[sync_query]
            classifications = []
            for line in complete:
                if phase2.parse_response(old_spec, line + b"\r")[2] is None:
                    classifications.append("delayed_timed_out_reply")
                    old_candidate = (
                        self.connection_generation > self.last_timeout["connection_generation"])
                elif phase2.parse_response(sync_spec, line + b"\r")[2] is None:
                    classifications.append("duplicate_sync_reply")
                else:
                    classifications.append("unclassifiable_line")
            if remainder:
                classifications.append("partial_line")
            classification = ",".join(classifications) or "unclassifiable_bytes"
            error = error or "post_sync_unexpected_bytes"
        valid = error is None
        self.framing_state = "synchronized" if valid else "unresolved"
        item = self._recovery_record(
            "post_sync_quiet_verification", "post_sync_verification", started,
            post_drain_bytes=phase2.bytes_field(
                bytes(data), len(data) > self.max_post_sync_bytes),
            bytes_received_after_timeout=phase2.bytes_field(bytes(data)),
            synchronisation_query=sync_query,
            synchronisation_response=sync_response,
            reply_belonged_to_old_generation=old_candidate,
            recovery_duration=self.clock.monotonic_ns() - started,
            final_framing_state=self.framing_state,
            valid=valid, error=error,
            abort_reason=None if valid else classification,
            post_sync_classification=classification)
        return item

    def _bounded_drain(self):
        timeout = self.last_timeout
        if timeout is None:
            raise RuntimeError("drain_requires_timeout")
        started = self.clock.monotonic_ns()
        horizon = timeout["write_time"] + self.drain_horizon_ns
        absolute = timeout["write_time"] + self.drain_absolute_ns
        last = None
        data = bytearray()
        error = None
        previous = started
        try:
            while True:
                now = self.clock.monotonic_ns()
                if now < previous:
                    error = "nonmonotonic_clock"
                    break
                previous = now
                quiet_deadline = last + self.drain_quiet_ns if last is not None else horizon
                completion_deadline = max(horizon, quiet_deadline)
                if now > absolute:
                    error = "drain_absolute_deadline_overshoot"
                    break
                if now >= completion_deadline and completion_deadline <= absolute:
                    break
                if now >= absolute:
                    error = "drain_not_quiet_before_absolute_deadline"
                    break
                remaining = self.max_drain_bytes + 1 - len(data)
                chunk = self.endpoint.read(
                    max(1, remaining), min(absolute, completion_deadline))
                observed = self.clock.monotonic_ns()
                if observed < previous:
                    error = "nonmonotonic_clock"
                    break
                previous = observed
                if not chunk:
                    continue
                data.extend(chunk)
                last = observed
                if len(data) > self.max_drain_bytes:
                    error = "drain_byte_cap_exceeded"
                    break
        except Exception as caught:
            error = "drain_read_error:" + type(caught).__name__
        complete, remainder = _lines(bytes(data))
        expected_spec = phase2.QUERY_SPECS[timeout["query_name"]]
        expected_prefix = expected_spec.prefix
        expected_count = 0
        for line in complete:
            if line in (expected_spec.request[:-1], b"+"):
                continue
            if line.startswith(expected_prefix) and phase2.parse_response(
                    expected_spec, line + b"\r")[2] is None:
                expected_count += 1
            else:
                error = error or "unclassifiable_drain_line"
        if remainder:
            error = error or "partial_drain_line"
        if expected_count > 1:
            error = error or "duplicate_delayed_reply"
        return bytes(data), error, started

    def bounded_resync_recovery(self, sync_query):
        if self.last_timeout is None or self.framing_state != "unresolved":
            raise RuntimeError("resync_requires_timeout")
        recovery_started = self.clock.monotonic_ns()
        drained, error, drain_started = self._bounded_drain()
        clean = error is None
        drain_item = self._recovery_record(
            "bounded_delimiter_drain", "bounded_resynchronisation", drain_started,
            post_drain_bytes=phase2.bytes_field(drained, len(drained) > self.max_drain_bytes),
            bytes_received_after_timeout=phase2.bytes_field(drained),
            final_framing_state="drained" if clean else "unresolved",
            valid=clean, error=error,
            abort_reason=None if clean else "fallback_reconnect")
        if not clean:
            return self._fallback_reconnect(sync_query, recovery_started, drain_item)
        self.framing_state = "drained"
        sync = self.transaction(sync_query, NORMAL_DEADLINE_NS,
                                purpose="bounded_resync_query",
                                permit_unresolved_probe=True)
        if not sync["valid"]:
            self.framing_state = "unresolved"
            return self._fallback_reconnect(sync_query, recovery_started, drain_item)
        verification = self._post_sync_verification(sync_query, sync["response_bytes"])
        if not verification["valid"]:
            self.framing_state = "unresolved"
            return self._fallback_reconnect(sync_query, recovery_started, verification)
        self.framing_state = "synchronized"
        return self._recovery_record(
            "bounded_resynchronisation_completed", "bounded_resynchronisation",
            recovery_started, post_drain_bytes=verification["post_drain_bytes"],
            bytes_received_after_timeout=phase2.bytes_field(
                drained + bytes.fromhex(verification["bytes_received_after_timeout"]["hex"])),
            synchronisation_query=sync_query,
            synchronisation_response=sync["response_bytes"],
            final_framing_state="synchronized", valid=True,
            recovery_duration=self.clock.monotonic_ns() - recovery_started)

    def _fallback_reconnect(self, sync_query, started, cause):
        self._recovery_record(
            "fallback_reconnect_started", "bounded_resynchronisation_then_reconnect",
            started, error=cause.get("error"), final_framing_state="unresolved")
        # Preserve timeout authority across close solely to enter the reviewed reconnect path.
        reconnect = self.reconnect_recovery(sync_query)
        return self._recovery_record(
            "bounded_resynchronisation_fallback_completed",
            "bounded_resynchronisation_then_reconnect", started,
            synchronisation_query=sync_query,
            synchronisation_response=reconnect.get("synchronisation_response", _empty_bytes()),
            reply_belonged_to_old_generation=reconnect.get(
                "reply_belonged_to_old_generation", False),
            recovery_duration=self.clock.monotonic_ns() - started,
            final_framing_state=self.framing_state,
            valid=reconnect["valid"],
            error=None if reconnect["valid"] else "fallback_reconnect_failed",
            abort_reason=None if reconnect["valid"] else "unresolved_framing")


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output", required=True)
    sub = parser.add_subparsers(dest="mode", required=True)
    baseline = sub.add_parser("baseline", allow_abbrev=False)
    baseline.add_argument("--deadline", type=float, default=0.100)
    for name in ("boundary", "reconnect", "bounded-resync"):
        command = sub.add_parser(name, allow_abbrev=False)
        command.add_argument("--deadline", type=float, required=True)
        command.add_argument("--attempts", type=int, default=1)
    return parser


def validate_args(args, parser):
    if not 1 <= args.baud <= 4_000_000:
        parser.error("--baud is out of range")
    if not 0.001 <= args.deadline <= 0.100:
        parser.error("--deadline must be between 0.001 and 0.100 seconds")
    if args.mode == "baseline" and args.deadline != 0.100:
        parser.error("baseline deadline is fixed at 0.100 seconds")
    if hasattr(args, "attempts") and not 1 <= args.attempts <= MAX_ATTEMPTS:
        parser.error("--attempts must be between 1 and 3")


def _require_valid(item, label):
    if not item["valid"]:
        raise RuntimeError(label + ":" + str(item.get("error")))


def run(args, endpoint, recorder, clock=None):
    harness = TimeoutResyncValidation(endpoint, recorder, clock)
    deadline_ns = int(args.deadline * 1e9)
    attempts = getattr(args, "attempts", 1)
    try:
        for attempt in range(attempts):
            _require_valid(harness.open_and_synchronize("initial"), "startup")
            _require_valid(harness.transaction("FID", NORMAL_DEADLINE_NS, "identity"), "fid")
            if args.mode == "baseline":
                _require_valid(harness.transaction("FF", deadline_ns), "baseline_ff")
                _require_valid(harness.transaction("FS", deadline_ns), "baseline_fs")
            else:
                target, sync = (("FF", "FS") if attempt % 2 == 0 else ("FS", "FF"))
                timed = harness.transaction(target, deadline_ns, "timeout_target")
                if timed["valid"]:
                    if args.mode != "boundary":
                        started = harness.clock.monotonic_ns()
                        harness._recovery_record(
                            "expected_timeout_not_observed", args.mode, started,
                            final_framing_state="synchronized", valid=False,
                            error="expected_timeout_not_observed",
                            abort_reason="recovery_policy_not_exercised")
                        raise RuntimeError("expected_timeout_not_observed")
                    harness.close()
                    continue
                if not timed["timeout"]:
                    raise RuntimeError("target_failed_without_timeout")
                if args.mode == "boundary":
                    harness.timeout_probe(sync)
                elif args.mode == "reconnect":
                    _require_valid(harness.reconnect_recovery(sync), "reconnect")
                else:
                    _require_valid(harness.bounded_resync_recovery(sync), "bounded_resync")
            harness.close()
        return 0
    finally:
        harness.close()


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(args, parser)
    recorder = NewEvidenceRecorder(args.output)
    endpoint = ObservedBoundedSerialEndpoint(args.port, args.baud)
    try:
        return run(args, endpoint, recorder)
    finally:
        recorder.close()


if __name__ == "__main__":
    raise SystemExit(main())
