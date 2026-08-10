#!/usr/bin/env python3
# Copyright 2026 Medrobots

"""Frozen host orchestrator for historical Roboteq/D455 rotation evidence.

The tool deliberately performs one checked stage per invocation.  It does not
open a device or import ROS; Docker and ROS commands are executed through the
operator's host environment.  State is kept on the host so recorder cleanup
does not depend on ad-hoc PID files inside either container.

The nonzero ``motion`` CLI stage is deprecated and blocked.  Historical
evidence inspection and the checked zero/cleanup stages remain available.
"""

import argparse
import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import uuid


STATE_NAME = "rotation-harness-state.json"
EVENTS_NAME = "rotation-harness-events.jsonl"
MOTION_PUBLISHER_EVIDENCE_NAME = "motion-publisher-evidence.json"
MOTION_DELIVERY_EVIDENCE_NAME = "motion-delivery-evidence.json"
FINAL_MOTION_DELIVERY_EVIDENCE_NAME = "final-motion-delivery-evidence.json"
MOTION_ACK = "robot-clear-estop-ready"
FROZEN_MOTION_MESSAGE = (
    "the D455 rotation harness is frozen for new nonzero motion testing; "
    "use status/abort for existing evidence and require a separately reviewed "
    "implementation change before motion can be re-enabled"
)
PUBLISHER_CONTAINER_PATH = (
    "/ros_ws/src/roboteq_ros2_driver/tools/d455_twist_publisher.py")
PUBLISHER_HOST_PATH = (
    Path(__file__).resolve().with_name("d455_twist_publisher.py"))
ROSBAG_QOS_OVERRIDE_CONTAINER_PATH = (
    "/ros_ws/src/roboteq_ros2_driver/config/"
    "d455_rotation_rosbag_qos.yaml")
ROSBAG_QOS_OVERRIDE_HOST_PATH = (
    Path(__file__).resolve().parent.parent /
    "config" / "d455_rotation_rosbag_qos.yaml")
CMD_VEL_TEST_QOS = {
    "history": "keep_last",
    "depth": 1,
    "reliability": "reliable",
    "durability": "volatile",
}
UNREPORTED_QOS_VALUES = {
    "history": {"unknown"},
    "depth": {0},
}
ROSBAG_QOS_OVERRIDE_SHA256 = hashlib.sha256(
    ROSBAG_QOS_OVERRIDE_HOST_PATH.read_bytes()).hexdigest()
PUBLISHER_SHA256 = hashlib.sha256(
    PUBLISHER_HOST_PATH.read_bytes()).hexdigest()
REQUIRED_PUBLISHER_ENDPOINTS = (
    "/:command_arbiter",
    "/:rosbag2_recorder",
)
ALLOWED_ANGULAR_Z = {
    -0.675, -0.45, -0.30, -0.15,
    0.15, 0.30, 0.45, 0.675,
}
ALLOWED_DURATIONS = {2.0, 5.0}
ALLOWED_RATE_HZ = {20}
ZERO_MESSAGE_COUNT = 20
ZERO_RATE_HZ = 20
SAFE_ZERO_SAMPLES = 10
MAX_AUDIT_AGE_SECONDS = 120.0
DIAGNOSTIC_DISCOVERY_TIMEOUT_SECONDS = 10.0
DIAGNOSTIC_MESSAGE_TIMEOUT_SECONDS = 8.0
DIAGNOSTIC_COHERENCE_WINDOW_SECONDS = 2.0
DIAGNOSTIC_SHELL_TIMEOUT_SECONDS = 22
DIAGNOSTIC_COMMAND_TIMEOUT_SECONDS = 24
PREPARE_TOPIC_EVIDENCE_TIMEOUT_SECONDS = 10.0
PREPARE_TOPIC_EVIDENCE_SHELL_TIMEOUT_SECONDS = 12
PREPARE_TOPIC_EVIDENCE_COMMAND_TIMEOUT_SECONDS = 14
MOTION_PUBLISHER_DISCOVERY_TIMEOUT_SECONDS = 5.0
MOTION_PUBLISHER_DURATION_TOLERANCE_SECONDS = 0.10
MOTION_TOPIC_SPAN_TOLERANCE_SECONDS = 0.10
MOTION_INTERVAL_TOLERANCE_SECONDS = 0.025
MOTION_MAX_SCHEDULE_LATENESS_SECONDS = 0.05
MOTION_DELIVERY_EVIDENCE_TIMEOUT_SECONDS = 5.0
MOTION_DELIVERY_SHELL_TIMEOUT_SECONDS = 7
MOTION_DELIVERY_COMMAND_TIMEOUT_SECONDS = 9
ARBITER_PUBLISH_RATE_HZ = 20.0
ARBITER_TEST_TIMEOUT_SECONDS = 0.25
SAFE_FORWARD_COUNT_TOLERANCE = 1
SAFE_FORWARD_START_TOLERANCE_SECONDS = 0.10
SAFE_FORWARD_END_TOLERANCE_SECONDS = 0.10
ROBOT_TOPICS = (
    "/cmd_vel/joy", "/cmd_vel/test", "/cmd_vel/nav", "/cmd_vel/safe",
    "/wheel_ticks", "/odom", "/diagnostics", "/tf", "/tf_static",
)
IMU_TOPICS = ("/camera/imu",)
RECORDER_IDENTITY_FIELDS = (
    "pid", "pgid", "sid", "starttime", "cmdline_hex",
)
PREMOTION_REQUIRED_TOPICS = {
    "robot": ("/cmd_vel/safe", "/wheel_ticks", "/odom"),
    "imu": ("/camera/imu",),
}
FINAL_REQUIRED_TOPICS = {
    "robot": ("/cmd_vel/test", "/cmd_vel/safe", "/wheel_ticks", "/odom"),
    "imu": ("/camera/imu",),
}
GROUP_EMPTY_AWK = """
$1 == wanted {present=1}
END {exit present ? 1 : 0}
""".strip()
ZERO_COMMAND_TYPES = {"prepare_zero", "cleanup_zero"}
RECORDER_PENDING_STATUSES = {
    "launch_registered",
    "launch_ambiguous",
    "launch_cancelled",
    "identity_pending",
    "launch_cleanup_unproven",
}


class HarnessError(RuntimeError):
    """A fail-closed validation error."""


class HarnessTermination(BaseException):
    """A host termination signal that must pass through safety cleanup."""

    def __init__(self, signum):
        super().__init__(f"received signal {signum}")
        self.signum = signum


def is_interruption(error):
    return isinstance(error, (KeyboardInterrupt, HarnessTermination))


def recorder_identity_status(spec):
    present = [
        field for field in RECORDER_IDENTITY_FIELDS if field in spec]
    if not present:
        if spec.get("status") == "launch_attempt_reaped":
            return "launch_reaped"
        if (
                spec.get("status") in RECORDER_PENDING_STATUSES or
                any(
                    field in spec for field in (
                        "token", "receipt_path", "exit_path"))):
            return "launch_pending"
        return "never_started"
    if len(present) != len(RECORDER_IDENTITY_FIELDS):
        return "incomplete"
    return "started"


def install_termination_handlers():
    termination_signals = (signal.SIGHUP, signal.SIGTERM)
    previous = {
        signum: signal.getsignal(signum)
        for signum in termination_signals
    }

    def terminate(signum, _frame):
        for termination_signal in termination_signals:
            signal.signal(termination_signal, signal.SIG_IGN)
        raise HarnessTermination(signum)

    for signum in termination_signals:
        signal.signal(signum, terminate)
    return previous


def restore_signal_handlers(previous):
    for signum, handler in previous.items():
        signal.signal(signum, handler)


class CommandResult:
    def __init__(self, returncode, stdout="", stderr=""):
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


class SubprocessRunner:
    """Injectable, bounded external-command boundary."""

    def run(self, argv, timeout):
        try:
            result = subprocess.run(
                argv, text=True, capture_output=True, timeout=timeout, check=False)
        except subprocess.TimeoutExpired as error:
            raise HarnessError(
                f"command timed out after {timeout}s: {shlex.join(argv)}") from error
        return CommandResult(result.returncode, result.stdout, result.stderr)


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(
        timespec="microseconds")


def atomic_write_json(path, value, exclusive=False):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if exclusive:
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        fd = os.open(path, flags, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, sort_keys=True, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        return
    with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent,
            prefix=f".{path.name}.", delete=False) as stream:
        temporary = Path(stream.name)
        json.dump(value, stream, sort_keys=True, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


class StateStore:
    def __init__(self, evidence_dir, read_only=False):
        self.directory = Path(evidence_dir).resolve()
        self.read_only = read_only
        self.path = self.directory / STATE_NAME
        self.events_path = self.directory / EVENTS_NAME
        self.lock_path = self.directory / ".rotation-harness.lock"
        self.lock_stream = None

    def __enter__(self):
        if self.read_only:
            if not self.directory.is_dir():
                raise HarnessError(f"evidence directory does not exist: {self.directory}")
            mode = "r"
            lock_mode = fcntl.LOCK_SH | fcntl.LOCK_NB
        else:
            self.directory.mkdir(parents=True, exist_ok=True)
            mode = "a+"
            lock_mode = fcntl.LOCK_EX | fcntl.LOCK_NB
        try:
            self.lock_stream = open(self.lock_path, mode, encoding="utf-8")
        except FileNotFoundError as error:
            raise HarnessError(f"state lock does not exist: {self.lock_path}") from error
        try:
            fcntl.flock(self.lock_stream.fileno(), lock_mode)
        except BlockingIOError as error:
            self.lock_stream.close()
            raise HarnessError("another harness stage holds the evidence lock") from error
        return self

    def __exit__(self, *_):
        fcntl.flock(self.lock_stream.fileno(), fcntl.LOCK_UN)
        self.lock_stream.close()

    def create(self, state):
        try:
            atomic_write_json(self.path, state, exclusive=True)
        except FileExistsError as error:
            raise HarnessError(
                f"refusing to overwrite existing state: {self.path}") from error

    def load(self):
        try:
            value = json.loads(self.path.read_text(encoding="utf-8"))
        except FileNotFoundError as error:
            raise HarnessError(f"state does not exist: {self.path}") from error
        except (json.JSONDecodeError, OSError) as error:
            raise HarnessError(f"state is unreadable: {self.path}: {error}") from error
        if value.get("schema_version") != 1:
            raise HarnessError("unsupported or missing state schema_version")
        return value

    def save(self, state):
        state["updated_at"] = utc_now()
        atomic_write_json(self.path, state)

    def event(self, name, **fields):
        item = {"time_utc": utc_now(), "event": name, **fields}
        try:
            with open(self.events_path, "a", encoding="utf-8", buffering=1) as stream:
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
                stream.write(
                    json.dumps(item, sort_keys=True, separators=(",", ":")) + "\n")
                stream.flush()
                os.fsync(stream.fileno())
            return True
        except OSError:
            return False


def require_safe_token(value, label):
    if not value or not re.fullmatch(r"[A-Za-z0-9_.:/-]+", value):
        raise HarnessError(f"unsafe or empty {label}: {value!r}")
    return value


def ros_shell(setup_paths, body):
    """Source ROS while nounset is disabled, then run a strict shell body."""
    lines = ["set -eo pipefail", "set +u"]
    for path in setup_paths:
        lines.append(f"source {shlex.quote(require_safe_token(path, 'setup path'))}")
    lines.extend(["set -u", body])
    return "\n".join(lines)


def docker_exec(container, script):
    require_safe_token(container, "container name")
    return ["docker", "exec", container, "bash", "-c", script]


def docker_exec_detached(container, script):
    require_safe_token(container, "container name")
    return ["docker", "exec", "--detach", container, "bash", "-c", script]


def run_checked(runner, argv, timeout, label):
    result = runner.run(argv, timeout)
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        raise HarnessError(f"{label} failed ({result.returncode}): {detail}")
    return result


def validate_motion(angular_z, duration, rate_hz, linear_x, acknowledgement):
    if linear_x != 0.0:
        raise HarnessError("linear velocity must be exactly zero")
    if angular_z not in ALLOWED_ANGULAR_Z:
        raise HarnessError(
            f"angular.z must be one of {sorted(ALLOWED_ANGULAR_Z)}")
    if duration not in ALLOWED_DURATIONS:
        raise HarnessError(f"duration must be one of {sorted(ALLOWED_DURATIONS)}")
    if rate_hz not in ALLOWED_RATE_HZ:
        raise HarnessError(f"rate must be one of {sorted(ALLOWED_RATE_HZ)}")
    if acknowledgement != MOTION_ACK:
        raise HarnessError(
            f"motion requires --acknowledge-motion {MOTION_ACK}")


def twist_yaml(angular_z):
    if (not isinstance(angular_z, (int, float)) or
            isinstance(angular_z, bool) or not math.isfinite(angular_z)):
        raise HarnessError("Twist angular.z must be a finite number")
    message = {
        "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": float(angular_z)},
    }
    return json.dumps(
        message, allow_nan=False, separators=(",", ":"), sort_keys=True)


def bag_topic_count(bag_info, topic):
    match = re.search(
        rf"(?m)^\s*(?:Topic information:\s*)?Topic:\s+{re.escape(topic)}\s+\|"
        rf".*?\|\s+Count:\s+(\d+)\s+\|",
        bag_info)
    return int(match.group(1)) if match else None


def topic_info_has_subscription(topic_info, node_name):
    current_node = None
    matching_subscription = False
    for raw_line in topic_info.splitlines():
        line = raw_line.strip()
        if line.startswith("Node name:"):
            current_node = line.partition(":")[2].strip()
        elif line == "Endpoint type: SUBSCRIPTION" and (
                current_node == node_name):
            matching_subscription = True
    return matching_subscription


def topic_info_has_recorder_subscription(topic_info):
    return topic_info_has_subscription(topic_info, "rosbag2_recorder")


def parse_json_marker(text, marker):
    prefix = marker + " "
    payloads = [
        line[len(prefix):] for line in text.splitlines()
        if line.startswith(prefix)
    ]
    if len(payloads) != 1:
        raise HarnessError(
            f"expected exactly one {marker} marker, got {len(payloads)}")
    try:
        value = json.loads(payloads[0])
    except json.JSONDecodeError as error:
        raise HarnessError(f"invalid {marker} JSON: {error}") from error
    if not isinstance(value, dict):
        raise HarnessError(f"{marker} payload must be a JSON object")
    return value


def require_evidence_integer(value, label, minimum=0):
    if not isinstance(value, int) or isinstance(value, bool) or value < minimum:
        raise HarnessError(f"{label} must be an integer >= {minimum}")
    return value


def require_evidence_number(value, label, minimum=None):
    if (not isinstance(value, (int, float)) or isinstance(value, bool) or
            not math.isfinite(value)):
        raise HarnessError(f"{label} must be a finite number")
    number = float(value)
    if minimum is not None and number < minimum:
        raise HarnessError(f"{label} must be >= {minimum}")
    return number


def assess_subscription_qos(reported_qos):
    reported_qos = reported_qos if isinstance(reported_qos, dict) else {}
    fields = {}
    for field in ("reliability", "durability", "history", "depth"):
        expected = CMD_VEL_TEST_QOS[field]
        reported = reported_qos.get(field)
        if reported == expected:
            status = "verified"
        elif reported in UNREPORTED_QOS_VALUES.get(field, set()):
            status = "tolerated_unreported"
        else:
            status = "mismatch"
        fields[field] = {
            "expected": expected,
            "reported": reported,
            "status": status,
        }
    accepted = all(
        result["status"] != "mismatch" for result in fields.values())
    tolerated = sorted(
        field for field, result in fields.items()
        if result["status"] == "tolerated_unreported")
    return {
        "accepted": accepted,
        "status": (
            "accepted_unreported" if accepted and tolerated
            else "accepted_verified" if accepted
            else "rejected_mismatch"),
        "tolerated_unreported_fields": tolerated,
        "fields": fields,
    }


def expected_endpoint_assessment(endpoint, qos):
    assessment = assess_subscription_qos(qos)
    record = {
        "qos": qos,
        "assessment": assessment,
    }
    return {
        **assessment,
        "endpoint": endpoint,
        "record_count": 1,
        "records": [record],
    }


def validate_publisher_evidence(
        evidence, requested_count, requested_duration, requested_rate_hz,
        requested_angular_z, command_type="motion",
        required_endpoints=REQUIRED_PUBLISHER_ENDPOINTS):
    if evidence.get("schema_version") != 1:
        raise HarnessError("publisher evidence schema_version must be 1")
    if evidence.get("status") != "complete":
        raise HarnessError(
            "publisher did not complete: "
            f"status={evidence.get('status')!r} "
            f"error={evidence.get('error')!r}")
    if evidence.get("command_type") != command_type:
        raise HarnessError("publisher command type does not match intent")
    if evidence.get("publisher_qos") != CMD_VEL_TEST_QOS:
        raise HarnessError("publisher QoS does not match the harness contract")
    exit_status = require_evidence_integer(
        evidence.get("child_exit_status"), "publisher child_exit_status")
    if exit_status != 0:
        raise HarnessError(
            f"publisher child exited unsuccessfully: {exit_status}")
    if not isinstance(evidence.get("stdout_stderr_path"), str):
        raise HarnessError("publisher stdout/stderr artifact is missing")
    actual_count = require_evidence_integer(
        evidence.get("actual_publish_count"),
        "publisher actual_publish_count")
    recorded_requested_count = require_evidence_integer(
        evidence.get("requested_publish_count"),
        "publisher requested_publish_count")
    recorded_rate = require_evidence_number(
        evidence.get("requested_rate_hz"), "publisher requested_rate_hz", 0.0)
    recorded_duration = require_evidence_number(
        evidence.get("requested_duration_s"),
        "publisher requested_duration_s", 0.0)
    require_evidence_integer(
        evidence.get("matched_subscriptions"),
        "publisher matched_subscriptions")
    if recorded_requested_count != requested_count:
        raise HarnessError(
            "publisher evidence requested count does not match motion intent")
    if actual_count != requested_count:
        raise HarnessError(
            f"publisher count mismatch: expected {requested_count}, "
            f"got {actual_count}")
    if not math.isclose(recorded_rate, requested_rate_hz, abs_tol=1e-12):
        raise HarnessError("publisher evidence rate does not match motion intent")
    if not math.isclose(recorded_duration, requested_duration, abs_tol=1e-12):
        raise HarnessError(
            "publisher evidence duration does not match motion intent")
    if evidence.get("published_twist") != json.loads(
            twist_yaml(requested_angular_z)):
        raise HarnessError(
            "publisher evidence Twist does not match motion intent")
    trusted_required_endpoints = list(required_endpoints)
    required_endpoints = set(trusted_required_endpoints)
    recorded_required_endpoints = evidence.get(
        "required_subscription_endpoints")
    if (
            not isinstance(recorded_required_endpoints, list) or
            len(recorded_required_endpoints) !=
            len(set(recorded_required_endpoints)) or
            set(recorded_required_endpoints) != required_endpoints):
        raise HarnessError(
            "publisher required subscription endpoints do not match intent")
    endpoints = evidence.get("matched_subscription_endpoints")
    if not isinstance(endpoints, list) or set(endpoints) != required_endpoints:
        raise HarnessError(
            "publisher did not prove arbiter and recorder endpoints ready")
    subscription_details = evidence.get("subscription_details")
    if not isinstance(subscription_details, list):
        raise HarnessError(
            "publisher subscription QoS details are missing")
    qos_by_endpoint = {}
    for index, detail in enumerate(subscription_details):
        if not isinstance(detail, dict):
            raise HarnessError(
                f"publisher subscription_details[{index}] is invalid")
        endpoint = detail.get("endpoint")
        qos = detail.get("qos")
        if not isinstance(endpoint, str) or not isinstance(qos, dict):
            raise HarnessError(
                f"publisher subscription_details[{index}] is incomplete")
        qos_by_endpoint.setdefault(endpoint, []).append(qos)
    for endpoint in required_endpoints:
        endpoint_qos = qos_by_endpoint.get(endpoint, [])
        if len(endpoint_qos) != 1:
            raise HarnessError(
                "publisher subscription identity is missing or ambiguous "
                f"for {endpoint}")
        assessment = assess_subscription_qos(endpoint_qos[0])
        if not assessment["accepted"]:
            mismatches = [
                field for field, result in assessment["fields"].items()
                if result["status"] == "mismatch"
            ]
            raise HarnessError(
                "publisher subscription QoS mismatch for "
                f"{endpoint}: fields={mismatches!r}")
    persisted_assessments = evidence.get(
        "subscription_qos_assessments")
    if not isinstance(persisted_assessments, list):
        raise HarnessError(
            "publisher subscription QoS assessments are missing")
    expected_assessments = [
        expected_endpoint_assessment(endpoint, qos_by_endpoint[endpoint][0])
        for endpoint in recorded_required_endpoints
    ]
    if persisted_assessments != expected_assessments:
        raise HarnessError(
            "publisher subscription QoS assessment evidence is inconsistent")
    if "/:rosbag2_recorder" in required_endpoints:
        override = evidence.get("recorder_qos_override")
        if not isinstance(override, dict):
            raise HarnessError(
                "publisher recorder QoS override proof is missing")
        expected_override = {
            "required": True,
            "path": ROSBAG_QOS_OVERRIDE_CONTAINER_PATH,
            "expected_sha256": ROSBAG_QOS_OVERRIDE_SHA256,
            "actual_sha256": ROSBAG_QOS_OVERRIDE_SHA256,
            "verified": True,
        }
        if override != expected_override:
            raise HarnessError(
                "publisher recorder QoS override proof is not pinned")

    publish_monotonic_ns = evidence.get("publish_monotonic_ns")
    publish_system_ns = evidence.get("publish_system_ns")
    schedule_lateness_ns = evidence.get("schedule_lateness_ns")
    for values, label in (
            (publish_monotonic_ns, "publish_monotonic_ns"),
            (publish_system_ns, "publish_system_ns"),
            (schedule_lateness_ns, "schedule_lateness_ns")):
        if not isinstance(values, list):
            raise HarnessError(f"publisher {label} must be a list")
        for index, value in enumerate(values):
            require_evidence_integer(
                value, f"publisher {label}[{index}]", minimum=0)
    if not (
            len(publish_monotonic_ns) == len(publish_system_ns) ==
            len(schedule_lateness_ns) == actual_count):
        raise HarnessError(
            "publisher raw timestamp counts do not match actual count")
    if any(current >= following for current, following in zip(
            publish_monotonic_ns, publish_monotonic_ns[1:])):
        raise HarnessError(
            "publisher raw monotonic timestamps are not strictly increasing")
    if any(current > following for current, following in zip(
            publish_system_ns, publish_system_ns[1:])):
        raise HarnessError(
            "publisher raw system timestamps are out of order")

    timestamp_fields = (
        "subscriber_ready_monotonic_ns",
        "window_start_monotonic_ns",
        "first_publish_monotonic_ns",
        "last_publish_monotonic_ns",
        "window_end_monotonic_ns",
        "window_start_system_ns",
        "first_publish_system_ns",
        "last_publish_system_ns",
        "window_end_system_ns",
    )
    timestamps = {
        field: require_evidence_integer(
            evidence.get(field), f"publisher {field}", minimum=1)
        for field in timestamp_fields
    }
    monotonic_order = (
        timestamps["subscriber_ready_monotonic_ns"],
        timestamps["window_start_monotonic_ns"],
        timestamps["first_publish_monotonic_ns"],
        timestamps["last_publish_monotonic_ns"],
        timestamps["window_end_monotonic_ns"],
    )
    system_order = (
        timestamps["window_start_system_ns"],
        timestamps["first_publish_system_ns"],
        timestamps["last_publish_system_ns"],
        timestamps["window_end_system_ns"],
    )
    if any(current > following for current, following in zip(
            monotonic_order, monotonic_order[1:])):
        raise HarnessError("publisher monotonic timestamps are out of order")
    if any(current > following for current, following in zip(
            system_order, system_order[1:])):
        raise HarnessError("publisher system timestamps are out of order")
    if (
            timestamps["first_publish_monotonic_ns"] !=
            publish_monotonic_ns[0] or
            timestamps["last_publish_monotonic_ns"] !=
            publish_monotonic_ns[-1] or
            timestamps["first_publish_system_ns"] != publish_system_ns[0] or
            timestamps["last_publish_system_ns"] != publish_system_ns[-1]):
        raise HarnessError(
            "publisher first/last timestamps disagree with raw evidence")
    for field in (
            "window_start_utc", "first_publish_utc",
            "last_publish_utc", "window_end_utc"):
        value = evidence.get(field)
        if not isinstance(value, str) or not value:
            raise HarnessError(f"publisher {field} must be a nonempty string")
        try:
            datetime.datetime.fromisoformat(value)
        except ValueError as error:
            raise HarnessError(f"publisher {field} is not ISO-8601") from error

    timing = evidence.get("timing")
    if not isinstance(timing, dict):
        raise HarnessError("publisher timing statistics are missing")
    timing_values = {
        field: require_evidence_number(
            timing.get(field), f"publisher timing.{field}", 0.0)
        for field in (
            "period_s", "window_duration_s", "publish_span_s",
            "mean_interval_s", "min_interval_s", "max_interval_s",
            "max_schedule_lateness_s",
        )
    }
    expected_period = 1.0 / requested_rate_hz
    expected_span = (requested_count - 1) / requested_rate_hz
    intervals = [
        (following - current) / 1000000000.0
        for current, following in zip(
            publish_monotonic_ns, publish_monotonic_ns[1:])
    ]
    recomputed = {
        "window_duration_s": (
            timestamps["window_end_monotonic_ns"] -
            timestamps["window_start_monotonic_ns"]) / 1000000000.0,
        "publish_span_s": (
            publish_monotonic_ns[-1] -
            publish_monotonic_ns[0]) / 1000000000.0,
        "mean_interval_s": sum(intervals) / len(intervals),
        "min_interval_s": min(intervals),
        "max_interval_s": max(intervals),
        "max_schedule_lateness_s": (
            max(schedule_lateness_ns) / 1000000000.0),
    }
    for field, value in recomputed.items():
        if not math.isclose(
                timing_values[field], value, abs_tol=1e-9):
            raise HarnessError(
                f"publisher timing.{field} disagrees with raw timestamps")
    if not math.isclose(
            timing_values["period_s"], expected_period, abs_tol=1e-12):
        raise HarnessError("publisher period does not match requested rate")
    if abs(
            timing_values["window_duration_s"] -
            requested_duration) > MOTION_PUBLISHER_DURATION_TOLERANCE_SECONDS:
        raise HarnessError(
            "publisher window duration outside tolerance: "
            f"expected {requested_duration:.6f}s, got "
            f"{timing_values['window_duration_s']:.6f}s")
    if abs(
            timing_values["publish_span_s"] -
            expected_span) > MOTION_TOPIC_SPAN_TOLERANCE_SECONDS:
        raise HarnessError(
            "publisher nonzero span outside tolerance: "
            f"expected {expected_span:.6f}s, got "
            f"{timing_values['publish_span_s']:.6f}s")
    minimum_interval = expected_period - MOTION_INTERVAL_TOLERANCE_SECONDS
    maximum_interval = expected_period + MOTION_INTERVAL_TOLERANCE_SECONDS
    if not (
            timing_values["min_interval_s"] >= minimum_interval and
            timing_values["max_interval_s"] <= maximum_interval):
        raise HarnessError(
            "publisher interval outside tolerance: "
            f"expected {minimum_interval:.6f}..{maximum_interval:.6f}s, "
            f"got {timing_values['min_interval_s']:.6f}.."
            f"{timing_values['max_interval_s']:.6f}s")
    if (
            timing_values["max_schedule_lateness_s"] >
            MOTION_MAX_SCHEDULE_LATENESS_SECONDS):
        raise HarnessError(
            "publisher schedule lateness outside tolerance: "
            f"{timing_values['max_schedule_lateness_s']:.6f}s")
    return evidence


def validate_delivery_evidence(
        evidence, requested_count, requested_duration, requested_rate_hz,
        requested_angular_z):
    if evidence.get("schema_version") != 1:
        raise HarnessError("delivery evidence schema_version must be 1")
    if evidence.get("database_errors") != []:
        raise HarnessError(
            f"delivery evidence database errors: "
            f"{evidence.get('database_errors')!r}")
    expected_count = require_evidence_integer(
        evidence.get("expected_publish_count"),
        "delivery expected_publish_count")
    expected_duration = require_evidence_number(
        evidence.get("expected_duration_s"),
        "delivery expected_duration_s", 0.0)
    expected_rate = require_evidence_number(
        evidence.get("expected_rate_hz"),
        "delivery expected_rate_hz", 0.0)
    expected_angular_z = require_evidence_number(
        evidence.get("expected_angular_z"),
        "delivery expected_angular_z")
    if expected_count != requested_count:
        raise HarnessError("delivery expected count does not match motion intent")
    if not math.isclose(expected_duration, requested_duration, abs_tol=1e-12):
        raise HarnessError(
            "delivery expected duration does not match motion intent")
    if not math.isclose(expected_rate, requested_rate_hz, abs_tol=1e-12):
        raise HarnessError("delivery expected rate does not match motion intent")
    if not math.isclose(
            expected_angular_z, requested_angular_z, abs_tol=1e-12):
        raise HarnessError(
            "delivery expected angular velocity does not match motion intent")

    topics = evidence.get("topics")
    if not isinstance(topics, dict):
        raise HarnessError("delivery topic evidence is missing")
    topic_values = {}
    for topic in ("/cmd_vel/test", "/cmd_vel/safe"):
        item = topics.get(topic)
        if not isinstance(item, dict):
            raise HarnessError(f"delivery evidence lacks {topic}")
        topic_values[topic] = {
            field: require_evidence_integer(
                item.get(field), f"{topic} {field}")
            for field in (
                "message_count", "nonzero_count",
                "matching_nonzero_count", "mismatched_nonzero_count",
                "internal_zero_count",
            )
        }
        topic_values[topic]["nonzero_span_s"] = require_evidence_number(
            item.get("nonzero_span_s"), f"{topic} nonzero_span_s", 0.0)
        nonzero_timestamps = item.get("nonzero_timestamps_ns")
        if not isinstance(nonzero_timestamps, list):
            raise HarnessError(
                f"{topic} nonzero_timestamps_ns must be a list")
        for index, timestamp in enumerate(nonzero_timestamps):
            require_evidence_integer(
                timestamp, f"{topic} nonzero_timestamps_ns[{index}]",
                minimum=1)
        if len(nonzero_timestamps) != topic_values[topic]["nonzero_count"]:
            raise HarnessError(
                f"{topic} timestamp count does not match nonzero count")
        if (
                topic_values[topic]["message_count"] <
                topic_values[topic]["nonzero_count"]):
            raise HarnessError(
                f"{topic} message count is below nonzero count")
        if len(nonzero_timestamps) < 2:
            raise HarnessError(
                f"{topic} has insufficient nonzero timestamp evidence")
        if any(current >= following for current, following in zip(
                nonzero_timestamps, nonzero_timestamps[1:])):
            raise HarnessError(
                f"{topic} nonzero timestamps are not strictly increasing")
        intervals = [
            (following - current) / 1000000000.0
            for current, following in zip(
                nonzero_timestamps, nonzero_timestamps[1:])
        ]
        span = (
            (nonzero_timestamps[-1] - nonzero_timestamps[0]) /
            1000000000.0)
        if not math.isclose(
                topic_values[topic]["nonzero_span_s"], span, abs_tol=1e-9):
            raise HarnessError(
                f"{topic} span disagrees with raw timestamps")
        topic_values[topic]["max_nonzero_interval_s"] = max(intervals)
        topic_values[topic]["first_nonzero_timestamp_ns"] = (
            nonzero_timestamps[0])
        topic_values[topic]["last_nonzero_timestamp_ns"] = (
            nonzero_timestamps[-1])
        if (
                item.get("first_nonzero_timestamp_ns") !=
                nonzero_timestamps[0] or
                item.get("last_nonzero_timestamp_ns") !=
                nonzero_timestamps[-1]):
            raise HarnessError(
                f"{topic} first/last timestamps disagree with raw evidence")

    test = topic_values["/cmd_vel/test"]
    if test["nonzero_count"] != requested_count:
        raise HarnessError(
            f"/cmd_vel/test nonzero count mismatch: expected "
            f"{requested_count}, got {test['nonzero_count']}")
    if test["matching_nonzero_count"] != requested_count or (
            test["mismatched_nonzero_count"] != 0):
        raise HarnessError("/cmd_vel/test recorded an unexpected nonzero Twist")
    if test["internal_zero_count"] != 0:
        raise HarnessError(
            "/cmd_vel/test contains an internal zero-command gap")
    expected_span = (requested_count - 1) / requested_rate_hz
    if abs(
            test["nonzero_span_s"] -
            expected_span) > MOTION_TOPIC_SPAN_TOLERANCE_SECONDS:
        raise HarnessError(
            "/cmd_vel/test nonzero duration mismatch: "
            f"expected {expected_span:.6f}s, got "
            f"{test['nonzero_span_s']:.6f}s")
    maximum_interval = (
        1.0 / requested_rate_hz + MOTION_INTERVAL_TOLERANCE_SECONDS)
    if test["max_nonzero_interval_s"] > maximum_interval:
        raise HarnessError(
            "/cmd_vel/test contains a nonzero inter-arrival gap: "
            f"{test['max_nonzero_interval_s']:.6f}s")

    safe = topic_values["/cmd_vel/safe"]
    timeout_messages = math.ceil(
        ARBITER_TEST_TIMEOUT_SECONDS * ARBITER_PUBLISH_RATE_HZ)
    safe_min_count = max(1, requested_count - SAFE_FORWARD_COUNT_TOLERANCE)
    safe_max_count = (
        requested_count + timeout_messages + SAFE_FORWARD_COUNT_TOLERANCE)
    if not safe_min_count <= safe["nonzero_count"] <= safe_max_count:
        raise HarnessError(
            "/cmd_vel/safe nonzero count outside arbiter tolerance: "
            f"expected {safe_min_count}..{safe_max_count}, "
            f"got {safe['nonzero_count']}")
    if safe["matching_nonzero_count"] != safe["nonzero_count"] or (
            safe["mismatched_nonzero_count"] != 0):
        raise HarnessError("/cmd_vel/safe recorded an unexpected nonzero Twist")
    if safe["internal_zero_count"] != 0:
        raise HarnessError(
            "/cmd_vel/safe contains an internal zero-command gap")
    minimum_safe_span = (
        expected_span - MOTION_TOPIC_SPAN_TOLERANCE_SECONDS)
    maximum_safe_span = (
        expected_span + ARBITER_TEST_TIMEOUT_SECONDS +
        MOTION_TOPIC_SPAN_TOLERANCE_SECONDS)
    if not minimum_safe_span <= safe["nonzero_span_s"] <= maximum_safe_span:
        raise HarnessError(
            "/cmd_vel/safe forwarded duration outside arbiter tolerance: "
            f"expected {minimum_safe_span:.6f}..{maximum_safe_span:.6f}s, "
            f"got {safe['nonzero_span_s']:.6f}s")
    if safe["max_nonzero_interval_s"] > (
            1.0 / ARBITER_PUBLISH_RATE_HZ +
            MOTION_INTERVAL_TOLERANCE_SECONDS):
        raise HarnessError(
            "/cmd_vel/safe contains a forwarded nonzero inter-arrival gap: "
            f"{safe['max_nonzero_interval_s']:.6f}s")

    start_offset = require_evidence_number(
        evidence.get("safe_start_offset_s"),
        "delivery safe_start_offset_s")
    end_offset = require_evidence_number(
        evidence.get("safe_end_offset_s"),
        "delivery safe_end_offset_s")
    recomputed_start_offset = (
        safe["first_nonzero_timestamp_ns"] -
        test["first_nonzero_timestamp_ns"]) / 1000000000.0
    recomputed_end_offset = (
        safe["last_nonzero_timestamp_ns"] -
        test["last_nonzero_timestamp_ns"]) / 1000000000.0
    if not math.isclose(
            start_offset, recomputed_start_offset, abs_tol=1e-9):
        raise HarnessError(
            "/cmd_vel/safe start offset disagrees with raw timestamps")
    if not math.isclose(
            end_offset, recomputed_end_offset, abs_tol=1e-9):
        raise HarnessError(
            "/cmd_vel/safe end offset disagrees with raw timestamps")
    if abs(start_offset) > SAFE_FORWARD_START_TOLERANCE_SECONDS:
        raise HarnessError(
            "/cmd_vel/safe start offset outside arbiter tolerance: "
            f"{start_offset:.6f}s")
    if not (
            -SAFE_FORWARD_END_TOLERANCE_SECONDS <= end_offset <=
            ARBITER_TEST_TIMEOUT_SECONDS +
            SAFE_FORWARD_END_TOLERANCE_SECONDS):
        raise HarnessError(
            "/cmd_vel/safe end offset outside arbiter timeout tolerance: "
            f"{end_offset:.6f}s")
    return evidence


def validate_kernel_audit(path, state_created_at, now=None):
    audit_path = Path(path)
    if audit_path.is_symlink() or not audit_path.is_file():
        raise HarnessError("kernel audit artifact must be a regular non-symlink file")
    data = audit_path.read_bytes()
    if len(data) > 4096:
        raise HarnessError("kernel audit artifact exceeds 4096 bytes")
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as error:
        raise HarnessError("kernel audit artifact is not UTF-8") from error
    lines = text.splitlines()
    required = ["apparmor_denials=0", "d455_usb_reset_or_disconnect=0"]
    if lines != required:
        raise HarnessError("kernel audit artifact lacks exact clean markers")
    current = datetime.datetime.now(datetime.timezone.utc).timestamp() if now is None else now
    modified = audit_path.stat().st_mtime
    created = datetime.datetime.fromisoformat(state_created_at).timestamp()
    if modified < created or current - modified < 0 or current - modified > MAX_AUDIT_AGE_SECONDS:
        raise HarnessError("kernel audit artifact is stale or predates this trial")
    return hashlib.sha256(data).hexdigest()


SAFE_ZERO_CHECK = r'''
import rclpy
import time
from geometry_msgs.msg import Twist
rclpy.init()
node = rclpy.create_node("rotation_harness_zero_check")
remaining = 10
bad = []
def callback(msg):
    global remaining
    values = (msg.linear.x, msg.linear.y, msg.linear.z,
              msg.angular.x, msg.angular.y, msg.angular.z)
    if any(abs(value) > 1e-9 for value in values):
        bad.append(values)
    remaining -= 1
node.create_subscription(Twist, "/cmd_vel/safe", callback, 10)
deadline = time.monotonic() + 3.0
while remaining > 0 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
if remaining > 0:
    raise SystemExit("insufficient /cmd_vel/safe samples")
if bad:
    raise SystemExit("nonzero /cmd_vel/safe observed")
'''.strip()


DIAGNOSTIC_WINDOW_HELPER = r'''
DIAGNOSTIC_COHERENCE_WINDOW_SECONDS = __COHERENCE_WINDOW__

def normalize_diagnostic_level(level):
    if isinstance(level, int) and not isinstance(level, bool):
        return level
    if isinstance(level, (bytes, bytearray)) and len(level) == 1:
        return level[0]
    return None

def diagnostic_stamp_seconds(stamp):
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if (not isinstance(sec, int) or isinstance(sec, bool) or
            not isinstance(nanosec, int) or isinstance(nanosec, bool)):
        return None
    if sec < 0 or nanosec < 0 or nanosec >= 1000000000:
        return None
    if sec == 0 and nanosec == 0:
        return None
    return sec + nanosec / 1000000000.0

def new_diagnostic_window():
    return {
        "serial": None,
        "encoder": None,
        "last_serial": [],
        "last_encoder": [],
        "sticky_errors": [],
        "coherence_source": None,
        "coherence_delta": None,
    }

def observe_diagnostic_array(window, statuses, stamp, receive_time):
    source_time = diagnostic_stamp_seconds(stamp)
    observations = {"serial": [], "encoder": []}
    for status in statuses:
        if status.name.endswith("roboteq/serial_connection"):
            kind = "serial"
            expected_message = "ready"
        elif status.name.endswith("roboteq/encoder_freshness"):
            kind = "encoder"
            expected_message = "fresh"
        else:
            continue
        level = normalize_diagnostic_level(status.level)
        observation = (level, status.message)
        observations[kind].append(observation)
        if level != 0 or status.message != expected_message:
            window["sticky_errors"].append(
                f"{kind}:level={level!r},message={status.message!r}")
            continue
        window[kind] = {
            "source_time": source_time,
            "receive_time": float(receive_time),
        }
    if observations["serial"]:
        window["last_serial"] = observations["serial"]
    if observations["encoder"]:
        window["last_encoder"] = observations["encoder"]
    if window["sticky_errors"]:
        return False
    if window["serial"] is None or window["encoder"] is None:
        return False
    serial = window["serial"]
    encoder = window["encoder"]
    if serial["source_time"] is not None and encoder["source_time"] is not None:
        source = "header"
        delta = abs(serial["source_time"] - encoder["source_time"])
    else:
        source = "receive"
        delta = abs(serial["receive_time"] - encoder["receive_time"])
    window["coherence_source"] = source
    window["coherence_delta"] = delta
    return delta <= DIAGNOSTIC_COHERENCE_WINDOW_SECONDS
'''.strip().replace(
    "__COHERENCE_WINDOW__", str(DIAGNOSTIC_COHERENCE_WINDOW_SECONDS)
)


DIAGNOSTIC_CHECK = DIAGNOSTIC_WINDOW_HELPER + "\n\n" + r'''
import rclpy
import time
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
rclpy.init()
node = rclpy.create_node("rotation_harness_diagnostic_check")
message_count = 0
accepted = False
window = new_diagnostic_window()
def callback(msg):
    global accepted, message_count
    message_count += 1
    if observe_diagnostic_array(
            window, msg.status, msg.header.stamp, time.monotonic()):
        accepted = True
qos = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)
subscription = node.create_subscription(
    DiagnosticArray, "/diagnostics", callback, qos)
discovery_deadline = time.monotonic() + __DISCOVERY_TIMEOUT__
publisher_seen = False
while not publisher_seen and time.monotonic() < discovery_deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    publisher_seen = message_count > 0 or node.count_publishers("/diagnostics") > 0
if not publisher_seen:
    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(
        "Roboteq diagnostics publisher not discovered: "
        f"messages={message_count} last_serial={window['last_serial']!r} "
        f"last_encoder={window['last_encoder']!r} "
        f"sticky_errors={window['sticky_errors']!r}")
message_deadline = time.monotonic() + __MESSAGE_TIMEOUT__
while (not accepted and not window["sticky_errors"] and
       time.monotonic() < message_deadline):
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_subscription(subscription)
node.destroy_node()
rclpy.shutdown()
if not accepted:
    raise SystemExit(
        "Roboteq ready/fresh diagnostics not observed coherently: "
        f"messages={message_count} last_serial={window['last_serial']!r} "
        f"last_encoder={window['last_encoder']!r} "
        f"coherence_source={window['coherence_source']!r} "
        f"coherence_delta={window['coherence_delta']!r} "
        f"sticky_errors={window['sticky_errors']!r}")
'''.strip().replace(
    "__DISCOVERY_TIMEOUT__", str(DIAGNOSTIC_DISCOVERY_TIMEOUT_SECONDS)
).replace(
    "__MESSAGE_TIMEOUT__", str(DIAGNOSTIC_MESSAGE_TIMEOUT_SECONDS)
)



MOTION_DELIVERY_MARKER = "ROTATION_DELIVERY_EVIDENCE"
MOTION_DELIVERY_EVIDENCE_CHECK = r'''
import datetime
import glob
import json
import math
from pathlib import Path
import sqlite3
import struct
import sys
import time

bag_path = sys.argv[1]
expected_angular_z = float(sys.argv[2])
expected_publish_count = int(sys.argv[3])
expected_duration_s = float(sys.argv[4])
expected_rate_hz = float(sys.argv[5])
evidence_timeout_s = float(sys.argv[6])
expected_values = (0.0, 0.0, 0.0, 0.0, 0.0, expected_angular_z)
topics = ("/cmd_vel/test", "/cmd_vel/safe")

def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(
        timespec="microseconds")

def decode_twist(data):
    if not isinstance(data, bytes) or len(data) != 52:
        raise ValueError(f"unexpected Twist CDR size {len(data)}")
    if data[:2] == b"\x00\x01":
        byte_order = "<"
    elif data[:2] == b"\x00\x00":
        byte_order = ">"
    else:
        raise ValueError(f"unsupported Twist CDR header {data[:4].hex()}")
    values = struct.unpack(byte_order + "6d", data[4:])
    if any(not math.isfinite(value) for value in values):
        raise ValueError("Twist CDR contains a non-finite value")
    return values

def topic_summary(rows):
    matching_timestamps = []
    mismatched_timestamps = []
    zero_timestamps = []
    for timestamp, data in rows:
        values = decode_twist(data)
        if not any(abs(value) > 1e-12 for value in values):
            zero_timestamps.append(timestamp)
            continue
        if all(
                abs(actual - expected) <= 1e-12
                for actual, expected in zip(values, expected_values)):
            matching_timestamps.append(timestamp)
        else:
            mismatched_timestamps.append(timestamp)
    nonzero_timestamps = sorted(
        matching_timestamps + mismatched_timestamps)
    first = nonzero_timestamps[0] if nonzero_timestamps else None
    last = nonzero_timestamps[-1] if nonzero_timestamps else None
    internal_zero_count = sum(
        first < timestamp < last for timestamp in zero_timestamps
    ) if first is not None and last is not None else 0
    return {
        "message_count": len(rows),
        "nonzero_count": len(nonzero_timestamps),
        "matching_nonzero_count": len(matching_timestamps),
        "mismatched_nonzero_count": len(mismatched_timestamps),
        "internal_zero_count": internal_zero_count,
        "nonzero_timestamps_ns": nonzero_timestamps,
        "first_nonzero_timestamp_ns": first,
        "last_nonzero_timestamp_ns": last,
        "nonzero_span_s": (
            (last - first) / 1000000000.0
            if first is not None and last is not None else 0.0
        ),
    }

def snapshot():
    rows = {topic: [] for topic in topics}
    database_errors = []
    databases = sorted(glob.glob(str(Path(bag_path) / "*.db3")))
    if not databases:
        database_errors.append("no sqlite3 bag file")
    for database in databases:
        try:
            uri = Path(database).resolve().as_uri() + "?mode=ro"
            connection = sqlite3.connect(uri, uri=True, timeout=0.2)
            try:
                for topic in topics:
                    rows[topic].extend(connection.execute(
                        "SELECT m.timestamp, m.data FROM messages AS m "
                        "JOIN topics AS t ON t.id = m.topic_id "
                        "WHERE t.name = ? ORDER BY m.timestamp",
                        (topic,)).fetchall())
            finally:
                connection.close()
        except (OSError, sqlite3.Error, TypeError, ValueError) as error:
            database_errors.append(
                f"{Path(database).name}: {type(error).__name__}: {error}")
    try:
        summaries = {
            topic: topic_summary(sorted(rows[topic]))
            for topic in topics
        }
    except (TypeError, ValueError, struct.error) as error:
        database_errors.append(f"Twist decode: {type(error).__name__}: {error}")
        summaries = {
            topic: {
                "message_count": len(rows[topic]),
                "nonzero_count": 0,
                "matching_nonzero_count": 0,
                "mismatched_nonzero_count": 0,
                "internal_zero_count": 0,
                "nonzero_timestamps_ns": [],
                "first_nonzero_timestamp_ns": None,
                "last_nonzero_timestamp_ns": None,
                "nonzero_span_s": 0.0,
            }
            for topic in topics
        }
    test = summaries["/cmd_vel/test"]
    safe = summaries["/cmd_vel/safe"]
    if (
            test["first_nonzero_timestamp_ns"] is not None and
            safe["first_nonzero_timestamp_ns"] is not None):
        safe_start_offset_s = (
            safe["first_nonzero_timestamp_ns"] -
            test["first_nonzero_timestamp_ns"]
        ) / 1000000000.0
        safe_end_offset_s = (
            safe["last_nonzero_timestamp_ns"] -
            test["last_nonzero_timestamp_ns"]
        ) / 1000000000.0
    else:
        safe_start_offset_s = 0.0
        safe_end_offset_s = 0.0
    return {
        "schema_version": 1,
        "captured_at_utc": utc_now(),
        "expected_angular_z": expected_angular_z,
        "expected_publish_count": expected_publish_count,
        "expected_duration_s": expected_duration_s,
        "expected_rate_hz": expected_rate_hz,
        "database_files": [Path(path).name for path in databases],
        "database_errors": database_errors,
        "topics": summaries,
        "safe_start_offset_s": safe_start_offset_s,
        "safe_end_offset_s": safe_end_offset_s,
    }

deadline = time.monotonic() + evidence_timeout_s
while True:
    evidence = snapshot()
    test = evidence["topics"]["/cmd_vel/test"]
    safe = evidence["topics"]["/cmd_vel/safe"]
    ready = (
        not evidence["database_errors"] and
        test["nonzero_count"] >= expected_publish_count and
        safe["nonzero_count"] >= max(1, expected_publish_count - 1)
    )
    if ready or time.monotonic() >= deadline:
        break
    time.sleep(0.1)
print(
    "ROTATION_DELIVERY_EVIDENCE " +
    json.dumps(evidence, allow_nan=False, sort_keys=True, separators=(",", ":")),
    flush=True)
'''.strip()


PREPARE_TOPIC_EVIDENCE_CHECK = r'''
import glob
from pathlib import Path
import sqlite3
import sys
import time

bag_path = sys.argv[1]
topic_name = sys.argv[2]
deadline = time.monotonic() + float(sys.argv[3])
expected_count = int(sys.argv[4])
last_count = 0
last_error = "no sqlite3 bag file"
while time.monotonic() < deadline:
    last_count = 0
    for database in sorted(glob.glob(str(Path(bag_path) / "*.db3"))):
        try:
            uri = Path(database).resolve().as_uri() + "?mode=ro"
            connection = sqlite3.connect(uri, uri=True, timeout=0.2)
            try:
                row = connection.execute(
                    "SELECT COUNT(*) FROM messages AS m "
                    "JOIN topics AS t ON t.id = m.topic_id "
                    "WHERE t.name = ?",
                    (topic_name,)).fetchone()
                last_count += int(row[0]) if row is not None else 0
                last_error = ""
            finally:
                connection.close()
        except (OSError, sqlite3.Error, TypeError, ValueError) as error:
            last_error = f"{type(error).__name__}: {error}"
            continue
    if last_count >= expected_count:
        print(f"ROTATION_PREPARE_TOPIC_COUNT {last_count}")
        raise SystemExit(0)
    time.sleep(0.1)
raise SystemExit(
    f"active bag has incomplete {topic_name} messages: "
    f"expected={expected_count} count={last_count} "
    f"last_error={last_error!r}")
'''.strip()


class RotationHarness:
    def __init__(self, runner, store):
        self.runner = runner
        self.store = store
        self.pending_interrupt = None

    def _container_identity(self, container):
        result = run_checked(
            self.runner,
            ["docker", "inspect", "-f", "{{.Id}} {{.State.Running}}", container],
            5, f"inspect {container}")
        match = re.fullmatch(r"([0-9a-f]{12,64}) true\s*", result.stdout)
        if not match:
            raise HarnessError(f"container is not running: {container}")
        return match.group(1)

    def _verify_container(self, spec):
        actual = self._container_identity(spec["container"])
        if actual != spec.get("container_id"):
            raise HarnessError(f"container identity changed: {spec['kind']}")

    @staticmethod
    def _identity_body(spec):
        return "\n".join([
            "set -eo pipefail",
            f"pid={spec['pid']}",
            "test -r \"/proc/$pid/stat\"",
            "actual_pgid=$(ps -o pgid= -p \"$pid\" | tr -d ' ')",
            "actual_sid=$(ps -o sid= -p \"$pid\" | tr -d ' ')",
            "stat_rest=$(sed 's/^[^)]*) //' \"/proc/$pid/stat\")",
            "set -- $stat_rest",
            "actual_state=$1",
            "actual_start=${20}",
            "actual_cmd=$(od -An -tx1 -v \"/proc/$pid/cmdline\" | tr -d ' \\n')",
            f"test \"$actual_pgid\" = {spec['pgid']}",
            f"test \"$actual_sid\" = {spec['sid']}",
            f"test \"$actual_start\" = {spec['starttime']}",
            f"test \"$actual_cmd\" = {shlex.quote(spec['cmdline_hex'])}",
            "case \"$actual_state\" in R|S|D|I|T|t|W) ;; *) exit 43;; esac",
        ])

    def _cleanup_start_token(
            self, spec, token, token_prefix="d455-recorder-",
            description=None, allow_missing=False):
        self._verify_container(spec)
        cleanup = "\n".join([
            "set -eo pipefail",
            f"token={shlex.quote(token_prefix + token)}",
            "pgids=''",
            "for path in /proc/[0-9]*/cmdline; do",
            "  [ -r \"$path\" ] || continue",
            "  cmd=$(tr '\\0' ' ' < \"$path\")",
            "  case \"$cmd\" in *\"$token\"*)",
            "    pid=${path#/proc/}; pid=${pid%/cmdline}",
            "    [ \"$pid\" = \"$$\" ] && continue",
            "    pgid=$(ps -o pgid= -p \"$pid\" | tr -d ' ')",
            "    case \" $pgids \" in *\" $pgid \"*) ;; *) pgids=\"$pgids $pgid\";; esac;;",
            "  esac",
            "done",
            "set -- $pgids",
            "if [ $# = 0 ]; then " + (
                "printf 'RECORDER_PENDING_QUIESCENT\\n'; exit 0"
                if allow_missing else "exit 73") + "; fi",
            "test $# = 1",
            "pgid=$1",
            "group_empty() {",
            "  ps -eo pgid=,stat= | awk -v wanted=\"$pgid\" "
            f"{shlex.quote(GROUP_EMPTY_AWK)}",
            "}",
            "group_empty && { printf 'RECORDER_PENDING_QUIESCENT\\n'; exit 0; }",
            "kill -INT -- \"-$pgid\" 2>/dev/null || true",
            "for unused in $(seq 1 50); do group_empty && exit 0; sleep 0.1; done",
            "kill -TERM -- \"-$pgid\" 2>/dev/null || true",
            "for unused in $(seq 1 20); do group_empty && exit 0; sleep 0.1; done",
            "kill -KILL -- \"-$pgid\" 2>/dev/null || true",
            "for unused in $(seq 1 10); do group_empty && exit 0; sleep 0.1; done",
            "exit 72",
        ])
        return run_checked(
            self.runner, docker_exec(spec["container"], cleanup), 10,
            description or f"cleanup launching {spec['kind']} recorder")

    def _persist_recorder_spec(self, spec):
        state = self.store.load()
        try:
            stored = state["recorders"][spec["kind"]]
        except KeyError as error:
            raise HarnessError(
                f"state lacks {spec['kind']} recorder slot") from error
        for field in (
                "kind", "container", "setup", "bag_path", "log_path",
                "topics"):
            if stored.get(field) != spec.get(field):
                raise HarnessError(
                    f"{spec['kind']} recorder static state changed: {field}")
        state["recorders"][spec["kind"]] = json.loads(json.dumps(spec))
        self.store.save(state)

    def _cancel_recorder_attempt(self, spec):
        token = spec.get("token")
        cancel_path = spec.get("cancel_path")
        release_path = spec.get("release_path")
        if not isinstance(token, str) or not token.startswith(
                "d455-recorder-"):
            raise HarnessError(
                f"{spec['kind']} recorder cancellation token is missing")
        # Older durable states predate the explicit marker paths.  Derive the
        # same package-local paths from the pinned token so recovery remains
        # deterministic without weakening identity checks.
        token_suffix = token[len("d455-recorder-"):]
        if not isinstance(cancel_path, str) or not cancel_path:
            cancel_path = f"/tmp/d455-rotation-recorder-{token_suffix}.cancel"
            spec["cancel_path"] = cancel_path
        if not isinstance(release_path, str) or not release_path:
            release_path = f"/tmp/d455-rotation-recorder-{token_suffix}.release"
            spec["release_path"] = release_path
        cancel_content = f"RECORDER_START_CANCELLED {token}\n"
        release_content = f"RECORDER_START_AUTHORIZED {token}\n"
        expected_cancel_sha256 = hashlib.sha256(
            cancel_content.encode("utf-8")).hexdigest()
        expected_release_sha256 = hashlib.sha256(
            release_content.encode("utf-8")).hexdigest()
        self._verify_container(spec)
        body = "\n".join([
            "set -eo pipefail",
            f"cancel_path={shlex.quote(cancel_path)}",
            f"release_path={shlex.quote(release_path)}",
            f"cancel_content={shlex.quote(cancel_content)}",
            f"expected_cancel_sha={expected_cancel_sha256}",
            f"expected_release_sha={expected_release_sha256}",
            "if [ -e \"$cancel_path\" ]; then",
            "  test -f \"$cancel_path\"",
            "  test ! -L \"$cancel_path\"",
            "else",
            "  temporary=\"$cancel_path.$$\"",
            "  printf '%s' \"$cancel_content\" > \"$temporary\"",
            "  mv \"$temporary\" \"$cancel_path\"",
            "fi",
            "cancel_sha=$(sha256sum \"$cancel_path\" | cut -d' ' -f1)",
            "test \"$cancel_sha\" = \"$expected_cancel_sha\"",
            "release_sha=absent",
            "if [ -e \"$release_path\" ]; then",
            "  test -f \"$release_path\"",
            "  test ! -L \"$release_path\"",
            "  release_sha=$(sha256sum \"$release_path\" | cut -d' ' -f1)",
            "  test \"$release_sha\" = \"$expected_release_sha\"",
            "fi",
            "printf 'RECORDER_START_CANCEL_PERSISTED %s %s\\n' "
            "\"$cancel_sha\" \"$release_sha\"",
        ])
        result = run_checked(
            self.runner, docker_exec(spec["container"], body), 5,
            f"persist {spec['kind']} recorder cancellation")
        match = re.fullmatch(
            r"RECORDER_START_CANCEL_PERSISTED ([0-9a-f]{64}) "
            r"(absent|[0-9a-f]{64})\n?",
            result.stdout)
        if not match:
            raise HarnessError(
                f"{spec['kind']} recorder cancellation is unproven")
        spec["status"] = "launch_cancelled"
        spec["cancel_sha256"] = match.group(1)
        spec["release_observed_sha256"] = (
            None if match.group(2) == "absent" else match.group(2))
        spec["cancel_persisted_at"] = utc_now()
        self._persist_recorder_spec(spec)

    def _recover_pending_recorder(self, spec):
        token = spec.get("token")
        receipt_path = spec.get("receipt_path")
        if not isinstance(token, str) or not token.startswith(
                "d455-recorder-"):
            raise HarnessError(
                f"{spec['kind']} pending recorder token is missing")
        if not isinstance(receipt_path, str) or not receipt_path:
            raise HarnessError(
                f"{spec['kind']} pending recorder receipt path is missing")
        self._cancel_recorder_attempt(spec)
        self._verify_container(spec)
        receipt_probe = run_checked(
            self.runner,
            docker_exec(
                spec["container"],
                "\n".join([
                    "set -eo pipefail",
                    f"receipt={shlex.quote(receipt_path)}",
                    "for unused in $(seq 1 20); do",
                    "  [ -e \"$receipt\" ] && break",
                    "  sleep 0.1",
                    "done",
                    "if [ ! -e \"$receipt\" ]; then",
                    "  printf 'RECORDER_RECEIPT_ABSENT\\n'",
                    "  exit 0",
                    "fi",
                    "test -f \"$receipt\"",
                    "test ! -L \"$receipt\"",
                    "grep -Eq '^ROTATION_RECORDER_RECEIPT "
                    "[0-9]+ [0-9]+ [0-9]+ [0-9]+ [0-9a-f]+$' "
                    "\"$receipt\"",
                    "cat \"$receipt\"",
                ])),
            4, f"inspect pending {spec['kind']} recorder receipt")
        if receipt_probe.stdout == "RECORDER_RECEIPT_ABSENT\n":
            cleanup_result = self._cleanup_start_token(
                spec, token[len("d455-recorder-"):],
                allow_missing=True,
                description=(
                    f"recover receipt-free {spec['kind']} recorder launch"))
            if cleanup_result is not None:
                # No receipt means the wrapper never crossed its durable
                # launch boundary.  Persist the quiescence proof so abort,
                # finalize, and status can distinguish this safe
                # never-started case from an unproven cleanup.
                self._persist_pending_recorder_cleanup(spec, {
                    "schema_version": 1,
                    "kind": spec["kind"],
                    "container": spec["container"],
                    "container_id": spec["container_id"],
                    "token": token,
                    "receipt_path": receipt_path,
                    "receipt_status": "absent_before_side_effects",
                    "quiescence_poll_count": 10,
                    "cancel_path": spec["cancel_path"],
                    "cancel_sha256": spec["cancel_sha256"],
                    "release_path": spec["release_path"],
                    "release_observed_sha256": spec.get(
                        "release_observed_sha256"),
                    "launch_mode": spec["launch_mode"],
                    "wrapper_reap_owner": spec["wrapper_reap_owner"],
                    "reap_verified_at": utc_now(),
                })
                return
            raise HarnessError(
                f"{spec['kind']} recorder launch remains ambiguous after "
                "durable cancellation: no wrapper receipt")
        receipt_match = re.fullmatch(
            r"ROTATION_RECORDER_RECEIPT "
            r"(\d+) (\d+) (\d+) (\d+) ([0-9a-f]+)\n?",
            receipt_probe.stdout)
        if not receipt_match:
            raise HarnessError(
                f"{spec['kind']} pending recorder receipt is ambiguous")
        pid, pgid, sid, starttime = (
            int(receipt_match.group(index)) for index in range(1, 5))
        if min(pid, pgid, sid, starttime) <= 1:
            raise HarnessError(
                f"{spec['kind']} pending recorder receipt is unsafe")
        cmdline_hex = receipt_match.group(5)
        cleanup_body = "\n".join([
            "set -eo pipefail",
            f"pid={pid}", f"pgid={pgid}", f"sid={sid}",
            f"starttime={starttime}",
            f"cmdline_hex={shlex.quote(cmdline_hex)}",
            f"receipt={shlex.quote(receipt_path)}",
            f"exit_path={shlex.quote(spec['exit_path'])}",
            f"log_path={shlex.quote(spec['log_path'])}",
            "group_empty() {",
            "  ps -eo pgid=,stat= | awk -v wanted=\"$pgid\" "
            f"{shlex.quote(GROUP_EMPTY_AWK)}",
            "}",
            "fully_absent() { group_empty && [ ! -e \"/proc/$pid\" ]; }",
            "if [ -e \"/proc/$pid\" ]; then",
            "  actual_pgid=$(ps -o pgid= -p \"$pid\" | tr -d ' ')",
            "  actual_sid=$(ps -o sid= -p \"$pid\" | tr -d ' ')",
            "  stat_rest=$(sed 's/^[^)]*) //' \"/proc/$pid/stat\")",
            "  set -- $stat_rest",
            "  actual_state=$1",
            "  actual_ppid=$2",
            "  actual_start=${20}",
            "  actual_cmd=$(od -An -tx1 -v \"/proc/$pid/cmdline\" | "
            "tr -d ' \\n')",
            "  test \"$actual_pgid\" = \"$pgid\"",
            "  test \"$actual_sid\" = \"$sid\"",
            "  test \"$actual_start\" = \"$starttime\"",
            "  if [ \"$actual_state\" = Z ]; then",
            "    test -z \"$actual_cmd\"",
            "    for unused in $(seq 1 20); do",
            "      fully_absent && break",
            "      sleep 0.1",
            "    done",
            "    if ! fully_absent; then",
            "      printf 'owned pending recorder zombie pid=%s ppid=%s\\n' "
            "\"$pid\" \"$actual_ppid\" >&2",
            "      exit 44",
            "    fi",
            "  else",
            "    case \"$actual_state\" in R|S|D|I|T|t|W) ;; *) exit 43;; esac",
            "    test -n \"$actual_cmd\"",
            "    test \"$actual_cmd\" = \"$cmdline_hex\"",
            "    kill -INT -- \"-$pgid\" 2>/dev/null || true",
            "    for unused in $(seq 1 50); do",
            "      fully_absent && break",
            "      sleep 0.1",
            "    done",
            "    if ! fully_absent; then",
            "      kill -TERM -- \"-$pgid\" 2>/dev/null || true",
            "      for unused in $(seq 1 20); do",
            "        fully_absent && break",
            "        sleep 0.1",
            "      done",
            "    fi",
            "    if ! fully_absent; then",
            "      kill -KILL -- \"-$pgid\" 2>/dev/null || true",
            "      for unused in $(seq 1 10); do",
            "        fully_absent && break",
            "        sleep 0.1",
            "      done",
            "    fi",
            "    fully_absent",
            "  fi",
            "else",
            "  group_empty",
            "fi",
            "test -f \"$receipt\"",
            "test ! -L \"$receipt\"",
            "receipt_sha=$(sha256sum \"$receipt\" | cut -d' ' -f1)",
            "test -f \"$exit_path\"",
            "test ! -L \"$exit_path\"",
            "recorder_exit=$(cat \"$exit_path\")",
            "case \"$recorder_exit\" in ''|*[!0-9]*) exit 45;; esac",
            "if [ -e \"$log_path\" ]; then",
            "  test -f \"$log_path\"",
            "  test ! -L \"$log_path\"",
            "  log_sha=$(sha256sum \"$log_path\" | cut -d' ' -f1)",
            "else",
            "  log_sha=none",
            "fi",
            "printf 'RECORDER_PENDING_REAPED %s %s %s\\n' "
            "\"$receipt_sha\" \"$recorder_exit\" \"$log_sha\"",
        ])
        result = run_checked(
            self.runner,
            docker_exec(spec["container"], cleanup_body),
            12, f"recover pending {spec['kind']} recorder launch")
        match = re.fullmatch(
            r"RECORDER_PENDING_REAPED ([0-9a-f]{64}) "
            r"(\d+) (none|[0-9a-f]{64})\n?",
            result.stdout)
        if not match:
            raise HarnessError(
                f"{spec['kind']} pending recorder recovery is unproven")
        recovery = {
            "schema_version": 1,
            "kind": spec["kind"],
            "container": spec["container"],
            "container_id": spec["container_id"],
            "token": token,
            "receipt_path": receipt_path,
            "receipt_status": "pinned_and_reaped",
            "cancel_path": spec["cancel_path"],
            "cancel_sha256": spec["cancel_sha256"],
            "release_path": spec["release_path"],
            "release_observed_sha256":
                spec.get("release_observed_sha256"),
            "receipt_sha256": match.group(1),
            "receipt_identity": {
                "pid": pid,
                "pgid": pgid,
                "sid": sid,
                "starttime": starttime,
                "cmdline_hex": cmdline_hex,
            },
            "child_exit_status": int(match.group(2)),
            "log_sha256": (
                None if match.group(3) == "none" else match.group(3)),
            "launch_mode": spec["launch_mode"],
            "wrapper_reap_owner": spec["wrapper_reap_owner"],
            "reap_verified_at": utc_now(),
        }
        self._persist_pending_recorder_cleanup(spec, recovery)

    def _persist_pending_recorder_cleanup(self, spec, evidence):
        expected_log_sha256 = evidence.get("log_sha256")
        if expected_log_sha256 is not None:
            marker = "D455_RECORDER_LOG_READ"
            result = run_checked(
                self.runner,
                docker_exec(
                    spec["container"],
                    "\n".join([
                        "set -eo pipefail",
                        f"log_path={shlex.quote(spec['log_path'])}",
                        "test -f \"$log_path\"",
                        "test ! -L \"$log_path\"",
                        f"printf '{marker}\\n'",
                        "cat \"$log_path\"",
                    ])),
                5, f"read pending {spec['kind']} recorder log")
            prefix = marker + "\n"
            if not result.stdout.startswith(prefix):
                raise HarnessError(
                    f"{spec['kind']} pending recorder log is ambiguous")
            log_text = result.stdout[len(prefix):]
            if hashlib.sha256(
                    log_text.encode("utf-8")).hexdigest() != (
                        expected_log_sha256):
                raise HarnessError(
                    f"{spec['kind']} pending recorder log hash changed")
            log_path = (
                self.store.directory /
                f"{spec['kind']}-recorder-launch.log")
            if log_path.exists():
                if log_path.read_text(encoding="utf-8") != log_text:
                    raise HarnessError(
                        f"{spec['kind']} pending recorder log evidence "
                        "changed")
            else:
                log_path.write_text(log_text, encoding="utf-8")
            evidence["log_path"] = str(log_path)
        else:
            evidence["log_path"] = None
        path = (
            self.store.directory /
            f"{spec['kind']}-recorder-launch-cleanup.json")
        if path.exists():
            existing = json.loads(path.read_text(encoding="utf-8"))
            existing_stable = {
                key: value for key, value in existing.items()
                if key != "reap_verified_at"
            }
            evidence_stable = {
                key: value for key, value in evidence.items()
                if key != "reap_verified_at"
            }
            if existing_stable != evidence_stable:
                raise HarnessError(
                    f"{spec['kind']} pending recorder cleanup evidence changed")
            evidence = existing
        else:
            atomic_write_json(path, evidence, exclusive=True)
        spec["status"] = "launch_attempt_reaped"
        spec["launch_cleanup_evidence_path"] = str(path)
        spec["launch_cleanup_evidence_sha256"] = hashlib.sha256(
            path.read_bytes()).hexdigest()
        spec["launch_reap_verified_at"] = evidence["reap_verified_at"]
        self._persist_recorder_spec(spec)

    def _validate_pending_recorder_cleanup(self, spec):
        evidence_value = spec.get("launch_cleanup_evidence_path")
        evidence_sha256 = spec.get("launch_cleanup_evidence_sha256")
        if not isinstance(evidence_value, str) or not re.fullmatch(
                r"[0-9a-f]{64}", evidence_sha256 or ""):
            raise HarnessError(
                f"{spec['kind']} pending recorder cleanup evidence is missing")
        evidence_path = Path(evidence_value)
        expected_path = (
            self.store.directory /
            f"{spec['kind']}-recorder-launch-cleanup.json")
        if (
                evidence_path != expected_path or
                not evidence_path.is_file() or evidence_path.is_symlink()):
            raise HarnessError(
                f"{spec['kind']} pending recorder cleanup evidence path "
                "is invalid")
        if hashlib.sha256(
                evidence_path.read_bytes()).hexdigest() != evidence_sha256:
            raise HarnessError(
                f"{spec['kind']} pending recorder cleanup evidence hash "
                "changed")
        try:
            evidence = json.loads(
                evidence_path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError) as error:
            raise HarnessError(
                f"{spec['kind']} pending recorder cleanup evidence is "
                f"unreadable: {error}") from error
        expected_fields = {
            "kind": spec["kind"],
            "container": spec["container"],
            "container_id": spec["container_id"],
            "token": spec.get("token"),
            "receipt_path": spec.get("receipt_path"),
            "launch_mode": "detached_foreground_docker_exec",
            "wrapper_reap_owner": "docker_exec_parent",
        }
        if any(
                evidence.get(key) != value
                for key, value in expected_fields.items()):
            raise HarnessError(
                f"{spec['kind']} pending recorder cleanup evidence identity "
                "changed")
        receipt_status = evidence.get("receipt_status")
        if receipt_status == "absent_before_side_effects":
            if evidence.get("quiescence_poll_count") != 10:
                raise HarnessError(
                    f"{spec['kind']} receipt-free cleanup lacks quiescence "
                    "proof")
        elif receipt_status == "pinned_and_reaped":
            identity = evidence.get("receipt_identity")
            if not isinstance(identity, dict) or any(
                    not isinstance(identity.get(field), expected_type)
                    for field, expected_type in {
                        "pid": int,
                        "pgid": int,
                        "sid": int,
                        "starttime": int,
                        "cmdline_hex": str,
                    }.items()):
                raise HarnessError(
                    f"{spec['kind']} pending recorder receipt identity is "
                    "invalid")
            if min(
                    identity["pid"], identity["pgid"], identity["sid"],
                    identity["starttime"]) <= 1 or not re.fullmatch(
                        r"[0-9a-f]+", identity["cmdline_hex"]):
                raise HarnessError(
                    f"{spec['kind']} pending recorder receipt identity is "
                    "unsafe")
            if not re.fullmatch(
                    r"[0-9a-f]{64}", evidence.get("receipt_sha256", "")):
                raise HarnessError(
                    f"{spec['kind']} pending recorder receipt hash is invalid")
            if not isinstance(evidence.get("child_exit_status"), int):
                raise HarnessError(
                    f"{spec['kind']} pending recorder child exit is invalid")
            log_sha256 = evidence.get("log_sha256")
            if log_sha256 is not None and not re.fullmatch(
                    r"[0-9a-f]{64}", log_sha256):
                raise HarnessError(
                    f"{spec['kind']} pending recorder log hash is invalid")
            if log_sha256 is not None:
                log_path = (
                    self.store.directory /
                    f"{spec['kind']}-recorder-launch.log")
                if (
                        evidence.get("log_path") != str(log_path) or
                        not log_path.is_file() or log_path.is_symlink() or
                        hashlib.sha256(log_path.read_bytes()).hexdigest() !=
                        log_sha256):
                    raise HarnessError(
                        f"{spec['kind']} pending recorder log evidence is "
                        "invalid")
            elif evidence.get("log_path") is not None:
                raise HarnessError(
                    f"{spec['kind']} receipt cleanup has unexpected log "
                    "evidence")
        else:
            raise HarnessError(
                f"{spec['kind']} pending recorder cleanup outcome is invalid")
        return evidence

    def _start_recorder(self, spec):
        token = uuid.uuid4().hex
        ack_path = f"/tmp/d455-rotation-recorder-{token}.ack"
        exit_path = f"/tmp/d455-rotation-recorder-{token}.exit"
        receipt_path = f"/tmp/d455-rotation-recorder-{token}.receipt"
        cancel_path = f"/tmp/d455-rotation-recorder-{token}.cancel"
        release_path = f"/tmp/d455-rotation-recorder-{token}.release"
        spec["startup_ack"] = ack_path
        spec["exit_path"] = exit_path
        spec["receipt_path"] = receipt_path
        spec["cancel_path"] = cancel_path
        spec["release_path"] = release_path
        spec["token"] = f"d455-recorder-{token}"
        spec["launch_mode"] = "detached_foreground_docker_exec"
        spec["wrapper_reap_owner"] = "docker_exec_parent"
        spec["status"] = "launch_registered"
        spec["launch_registered_at"] = utc_now()
        self._persist_recorder_spec(spec)
        topics = " ".join(shlex.quote(topic) for topic in spec["topics"])
        qos_setup = []
        qos_argument = ""
        if spec["kind"] == "robot":
            qos_path = ROSBAG_QOS_OVERRIDE_CONTAINER_PATH
            qos_setup = [
                f"test -f {shlex.quote(qos_path)}",
                f"test ! -L {shlex.quote(qos_path)}",
                f"test \"$(sha256sum {shlex.quote(qos_path)} | "
                f"cut -d' ' -f1)\" = {ROSBAG_QOS_OVERRIDE_SHA256}",
            ]
            qos_argument = (
                "--qos-profile-overrides-path " +
                shlex.quote(qos_path) + " ")
            spec["cmd_vel_test_qos"] = dict(CMD_VEL_TEST_QOS)
            spec["qos_override_path"] = qos_path
            spec["qos_override_sha256"] = ROSBAG_QOS_OVERRIDE_SHA256
        wrapper = "\n".join([
            "set -eo pipefail",
            f"ack={shlex.quote(ack_path)}",
            f"exit_path={shlex.quote(exit_path)}",
            f"receipt_path={shlex.quote(receipt_path)}",
            f"cancel_path={shlex.quote(cancel_path)}",
            f"release_path={shlex.quote(release_path)}",
            f"test ! -e {shlex.quote(spec['bag_path'])}",
            f"test ! -e {shlex.quote(spec['log_path'])}",
            f"test ! -e {shlex.quote(ack_path)}",
            f"test ! -e {shlex.quote(exit_path)}",
            f"test ! -e {shlex.quote(receipt_path)}",
            f"test ! -e {shlex.quote(cancel_path)}",
            f"test ! -e {shlex.quote(release_path)}",
            "recorder=''",
            "write_exit() {",
            "  status=$1",
            "  temporary=\"$exit_path.$$\"",
            "  printf '%s\\n' \"$status\" > \"$temporary\"",
            "  mv \"$temporary\" \"$exit_path\"",
            "}",
            "wait_recorder() {",
            "  set +e",
            "  while :; do",
            "    wait \"$recorder\"",
            "    status=$?",
            "    if kill -0 \"$recorder\" 2>/dev/null; then continue; fi",
            "    return \"$status\"",
            "  done",
            "}",
            "cleanup() {",
            "  wrapper_status=$?",
            "  trap - EXIT",
            "  set +e",
            "  if [ -n \"$recorder\" ] && kill -0 \"$recorder\" 2>/dev/null; then",
            "    kill -INT -- -$$ 2>/dev/null || true",
            "    wait_recorder",
            "    child_status=$?",
            "    if [ \"$wrapper_status\" = 0 ]; then",
            "      wrapper_status=$child_status",
            "    fi",
            "  fi",
            "  write_exit \"$wrapper_status\"",
            "  exit \"$wrapper_status\"",
            "}",
            "trap cleanup EXIT",
            "trap ':' INT TERM",
            "pgid=$(ps -o pgid= -p $$ | tr -d ' ')",
            "sid=$(ps -o sid= -p $$ | tr -d ' ')",
            "stat_rest=$(sed 's/^[^)]*) //' /proc/$$/stat)",
            "set -- $stat_rest",
            "starttime=${20}",
            "cmdline_hex=$(od -An -tx1 -v /proc/$$/cmdline | tr -d ' \\n')",
            "temporary=\"$receipt_path.$$\"",
            "printf 'ROTATION_RECORDER_RECEIPT %s %s %s %s %s\\n' "
            "\"$$\" \"$pgid\" \"$sid\" \"$starttime\" \"$cmdline_hex\" "
            "> \"$temporary\"",
            "mv \"$temporary\" \"$receipt_path\"",
            *qos_setup,
            "ros2 bag record --storage sqlite3 "
            f"{qos_argument}"
            f"--output {shlex.quote(spec['bag_path'])} {topics} "
            f">{shlex.quote(spec['log_path'])} 2>&1 </dev/null &",
            "recorder=$!",
            "for unused in $(seq 1 100); do",
            "  kill -0 \"$recorder\"",
            "  if [ -e \"$ack\" ]; then",
            "    rm -f \"$ack\"",
            "    set +e",
            "    wait_recorder",
            "    status=$?",
            "    set -e",
            "    trap - EXIT",
            "    write_exit \"$status\"",
            "    exit \"$status\"",
            "  fi",
            "  sleep 0.1",
            "done",
            "exit 70",
        ])
        # Keep an in-container parent alive to wait(2) the short-lived
        # recorder wrapper.  `docker exec --detach` otherwise reparents the
        # wrapper to PID 1, leaving a PID-1-owned zombie after shutdown.
        launch = (
            "setsid bash -c " + shlex.quote(wrapper) +
            f" d455-recorder-{token} & wrapper_pid=$!; "
            "wait \"$wrapper_pid\"")
        identity_body = "\n".join([
            "set -eo pipefail",
            f"receipt={shlex.quote(receipt_path)}",
            "for unused in $(seq 1 50); do",
            "  if [ -f \"$receipt\" ] && [ ! -L \"$receipt\" ] && "
            f"find {shlex.quote(spec['bag_path'])} -maxdepth 1 -type f "
            "-name '*.db3' -print -quit 2>/dev/null | grep -q .; then break; fi",
            "  sleep 0.1",
            "done",
            "test -f \"$receipt\"",
            "test ! -L \"$receipt\"",
            f"find {shlex.quote(spec['bag_path'])} -maxdepth 1 -type f "
            "-name '*.db3' -print -quit | grep -q .",
            "grep -Eq '^ROTATION_RECORDER_RECEIPT "
            "[0-9]+ [0-9]+ [0-9]+ [0-9]+ [0-9a-f]+$' \"$receipt\"",
            "cat \"$receipt\"",
        ])
        try:
            run_checked(
                self.runner,
                docker_exec_detached(
                    spec["container"], ros_shell(spec["setup"], launch)),
                5, f"launch detached {spec['kind']} recorder wrapper")
            spec["status"] = "identity_pending"
            spec["detached_launch_returned_at"] = utc_now()
            self._persist_recorder_spec(spec)
            result = run_checked(
                self.runner,
                docker_exec(spec["container"], identity_body),
                8, f"start {spec['kind']} recorder")
            match = re.fullmatch(
                r"ROTATION_RECORDER_RECEIPT "
                r"(\d+) (\d+) (\d+) (\d+) ([0-9a-f]+)\n?",
                result.stdout)
            if not match:
                raise HarnessError(
                    f"invalid {spec['kind']} recorder identity receipt: "
                    f"{result.stdout!r}")
            pid, pgid, sid, starttime = (
                int(match.group(index)) for index in range(1, 5))
            if min(pid, pgid, sid, starttime) <= 1:
                raise HarnessError("refusing unsafe recorder PID/PGID")
            spec["pid"] = pid
            spec["pgid"] = pgid
            spec["sid"] = sid
            spec["starttime"] = starttime
            spec["cmdline_hex"] = match.group(5)
            spec["status"] = "running"
            self._verify_recorder(spec)
            self._persist_recorder_spec(spec)
        except BaseException as start_error:
            if is_interruption(start_error):
                self.pending_interrupt = start_error
            spec["status"] = "launch_ambiguous"
            try:
                self._persist_recorder_spec(spec)
            except BaseException:
                pass
            try:
                self._recover_pending_recorder(spec)
            except BaseException as cleanup_error:
                spec["status"] = "launch_cleanup_unproven"
                try:
                    self._persist_recorder_spec(spec)
                except BaseException:
                    pass
                raise HarnessError(
                    f"{spec['kind']} recorder launch/identity failed: "
                    f"{start_error}; pending launch cleanup failed: "
                    f"{cleanup_error}") from start_error
            raise

    def _acknowledge_recorder(self, spec):
        self._verify_recorder(spec)
        run_checked(
            self.runner,
            docker_exec(
                spec["container"], f"touch {shlex.quote(spec['startup_ack'])}"),
            3, f"acknowledge {spec['kind']} recorder startup")
        self._verify_recorder(spec)

    def _verify_recorder(self, spec):
        self._verify_container(spec)
        run_checked(
            self.runner,
            docker_exec(spec["container"], self._identity_body(spec)), 4,
            f"verify {spec['kind']} recorder")

    def _stop_recorder(self, spec, allow_missing=False):
        identity_status = recorder_identity_status(spec)
        if identity_status == "never_started":
            return "never_started"
        if identity_status == "launch_pending":
            try:
                self._recover_pending_recorder(spec)
            except BaseException:
                spec["status"] = "launch_cleanup_unproven"
                try:
                    self._persist_recorder_spec(spec)
                except BaseException:
                    pass
                raise
            return "launch_attempt_reaped"
        if identity_status == "launch_reaped":
            self._validate_pending_recorder_cleanup(spec)
            return "launch_attempt_reaped"
        if identity_status == "incomplete":
            raise HarnessError(
                f"incomplete {spec['kind']} recorder identity")
        self._verify_container(spec)
        missing_status = (
            self._recorder_completion_body(
                spec, "RECORDER_ALREADY_MISSING")
            if allow_missing else "exit 41")
        body = "\n".join([
            "set -eo pipefail",
            f"pid={spec['pid']}", f"pgid={spec['pgid']}",
            f"exit_path={shlex.quote(spec.get('exit_path', ''))}",
            f"log_path={shlex.quote(spec['log_path'])}",
            "actual=$(ps -o pgid= -p \"$pid\" 2>/dev/null | tr -d ' ' || true)",
            f"if [ -z \"$actual\" ]; then {missing_status}; fi",
            "test -r \"/proc/$pid/stat\"",
            "actual_pgid=$(ps -o pgid= -p \"$pid\" | tr -d ' ')",
            "actual_sid=$(ps -o sid= -p \"$pid\" | tr -d ' ')",
            "stat_rest=$(sed 's/^[^)]*) //' \"/proc/$pid/stat\")",
            "set -- $stat_rest",
            "actual_state=$1",
            "actual_ppid=$2",
            "actual_start=${20}",
            "actual_cmd=$(od -An -tx1 -v \"/proc/$pid/cmdline\" | tr -d ' \\n')",
            self._stop_decision_body(spec),
        ])
        result = run_checked(
            self.runner, docker_exec(spec["container"], body), 15,
            f"stop {spec['kind']} recorder cleanly")
        if "RECORDER_ALREADY_MISSING" in result.stdout:
            outcome = "already_missing"
            marker = "RECORDER_ALREADY_MISSING"
        else:
            outcome = "reaped"
            marker = "RECORDER_REAPED"
        match = re.search(
            rf"(?m)^{marker} (\d+) ([0-9a-f]{{64}})$",
            result.stdout)
        if not match:
            raise HarnessError(f"{spec['kind']} recorder stop outcome is unproven")
        exit_status = int(match.group(1))
        log_sha256 = match.group(2)
        self._persist_recorder_cleanup_evidence(
            spec, exit_status, log_sha256, outcome)
        return outcome

    @staticmethod
    def _recorder_completion_body(spec, marker):
        return "\n".join([
            "set -eo pipefail",
            f"pid={shlex.quote(str(spec.get('pid', '')))}",
            f"pgid={shlex.quote(str(spec.get('pgid', '')))}",
            f"token={shlex.quote(spec.get('token', ''))}",
            f"exit_path={shlex.quote(spec.get('exit_path', ''))}",
            f"log_path={shlex.quote(spec.get('log_path', ''))}",
            "case \"$pid\" in ''|*[!0-9]*) exit 45;; esac",
            "case \"$pgid\" in ''|*[!0-9]*) exit 45;; esac",
            "test \"$pid\" -gt 1",
            "test \"$pgid\" -gt 1",
            "case \"$token\" in d455-recorder-*) ;; *) exit 45;; esac",
            "test -n \"$exit_path\"",
            "test -n \"$log_path\"",
            "group_empty() {",
            "  ps -eo pgid=,stat= | awk -v wanted=\"$pgid\" "
            f"{shlex.quote(GROUP_EMPTY_AWK)}",
            "}",
            "owned_token_process() {",
            "  for path in /proc/[0-9]*/cmdline; do",
            "    [ -r \"$path\" ] || continue",
            "    candidate=${path#/proc/}; candidate=${candidate%/cmdline}",
            "    [ \"$candidate\" = \"$$\" ] && continue",
            "    cmd=$(tr '\\0' ' ' < \"$path\")",
            "    case \"$cmd\" in *\"$token\"*) return 0;; esac",
            "  done",
            "  return 1",
            "}",
            "absence_streak=0",
            "for unused in $(seq 1 100); do",
            "  if group_empty && test ! -e \"/proc/$pid\" && "
            "! owned_token_process; then",
            "    absence_streak=$((absence_streak + 1))",
            "    [ \"$absence_streak\" -ge 10 ] && break",
            "  else",
            "    absence_streak=0",
            "  fi",
            "  sleep 0.1",
            "done",
            "test \"$absence_streak\" -ge 10",
            "test -f \"$exit_path\"",
            "test ! -L \"$exit_path\"",
            "recorder_exit=$(cat \"$exit_path\")",
            "case \"$recorder_exit\" in ''|*[!0-9]*) exit 45;; esac",
            "test \"$recorder_exit\" = 0",
            "test -f \"$log_path\"",
            "test ! -L \"$log_path\"",
            "log_sha256=$(sha256sum \"$log_path\" | cut -d' ' -f1)",
            f"printf '{marker} %s %s\\n' \"$recorder_exit\" \"$log_sha256\"",
            "exit 0",
        ])

    @staticmethod
    def _stop_decision_body(spec):
        return "\n".join([
            "set -eo pipefail",
            f"test \"$actual_pgid\" = {spec['pgid']}",
            f"test \"$actual_sid\" = {spec['sid']}",
            f"test \"$actual_start\" = {spec['starttime']}",
            "if [ \"$actual_state\" = Z ]; then",
            "  test -z \"$actual_cmd\"",
            "  printf 'owned recorder wrapper zombie pid=%s ppid=%s\\n' "
            "\"$pid\" \"$actual_ppid\" >&2",
            "  exit 44",
            "fi",
            "case \"$actual_state\" in R|S|D|I|T|t|W) ;; *) exit 43;; esac",
            "test -n \"$actual_cmd\"",
            f"test \"$actual_cmd\" = {shlex.quote(spec['cmdline_hex'])}",
            "kill -INT -- \"-$pgid\"",
            "for unused in $(seq 1 100); do",
            "  if ps -eo pgid=,stat= | awk -v wanted=\"$pgid\" "
            f"{shlex.quote(GROUP_EMPTY_AWK)} && "
            "[ ! -e \"/proc/$pid\" ]; then",
            RotationHarness._recorder_completion_body(
                spec, "RECORDER_REAPED"),
            "  fi",
            "  sleep 0.1",
            "done",
            "kill -TERM -- \"-$pgid\" 2>/dev/null || true",
            "for unused in $(seq 1 20); do",
            "  if ps -eo pgid=,stat= | awk -v wanted=\"$pgid\" "
            f"{shlex.quote(GROUP_EMPTY_AWK)} && "
            "[ ! -e \"/proc/$pid\" ]; then",
            RotationHarness._recorder_completion_body(
                spec, "RECORDER_REAPED"),
            "  fi",
            "  sleep 0.1",
            "done",
            "kill -KILL -- \"-$pgid\" 2>/dev/null || true",
            "for unused in $(seq 1 10); do",
            "  if ps -eo pgid=,stat= | awk -v wanted=\"$pgid\" "
            f"{shlex.quote(GROUP_EMPTY_AWK)} && "
            "[ ! -e \"/proc/$pid\" ]; then",
            RotationHarness._recorder_completion_body(
                spec, "RECORDER_REAPED"),
            "  fi",
            "  sleep 0.1",
            "done",
            "exit 42",
        ])

    def _persist_recorder_cleanup_evidence(
            self, spec, exit_status, expected_log_sha256, outcome):
        marker = "D455_RECORDER_LOG_READ"
        result = run_checked(
            self.runner,
            docker_exec(
                spec["container"],
                "\n".join([
                    "set -eo pipefail",
                    f"log_path={shlex.quote(spec['log_path'])}",
                    "test -f \"$log_path\"",
                    "test ! -L \"$log_path\"",
                    f"printf '{marker}\\n'",
                    "cat \"$log_path\"",
                ])),
            5, f"read {spec['kind']} recorder log")
        prefix = marker + "\n"
        if not result.stdout.startswith(prefix):
            raise HarnessError(
                f"{spec['kind']} recorder log evidence is ambiguous")
        log_text = result.stdout[len(prefix):]
        actual_log_sha256 = hashlib.sha256(
            log_text.encode("utf-8")).hexdigest()
        if actual_log_sha256 != expected_log_sha256:
            raise HarnessError(
                f"{spec['kind']} recorder log hash changed during cleanup")
        log_path = self.store.directory / f"{spec['kind']}-recorder.log"
        if log_path.exists():
            if log_path.read_text(encoding="utf-8") != log_text:
                raise HarnessError(
                    f"{spec['kind']} recorder log evidence changed")
        else:
            log_path.write_text(log_text, encoding="utf-8")
        result_path = (
            self.store.directory / f"{spec['kind']}-recorder-cleanup.json")
        evidence = {
            "schema_version": 1,
            "kind": spec["kind"],
            "container": spec["container"],
            "container_id": spec["container_id"],
            "pid": spec["pid"],
            "pgid": spec["pgid"],
            "sid": spec["sid"],
            "starttime": spec["starttime"],
            "token": spec.get("token"),
            "launch_mode": spec.get("launch_mode"),
            "wrapper_reap_owner": spec.get("wrapper_reap_owner"),
            "absence_proof": "10_consecutive_samples_100ms",
            "outcome": outcome,
            "child_exit_status": exit_status,
            "log_path": str(log_path),
            "log_sha256": actual_log_sha256,
            "reap_verified_at": utc_now(),
        }
        if result_path.exists():
            existing = json.loads(result_path.read_text(encoding="utf-8"))
            stable_fields = {
                key: value for key, value in evidence.items()
                if key != "reap_verified_at"
            }
            existing_stable = {
                key: value for key, value in existing.items()
                if key != "reap_verified_at"
            }
            if existing_stable != stable_fields:
                raise HarnessError(
                    f"{spec['kind']} recorder cleanup evidence changed")
            evidence = existing
        else:
            atomic_write_json(result_path, evidence, exclusive=True)
        spec["status"] = "reaped"
        spec["child_exit_status"] = exit_status
        spec["reap_verified_at"] = evidence["reap_verified_at"]
        spec["cleanup_evidence_path"] = str(result_path)
        spec["cleanup_evidence_sha256"] = hashlib.sha256(
            result_path.read_bytes()).hexdigest()

    def _validate_terminal_recorder_cleanup(self, state):
        errors = []
        for spec in state["recorders"].values():
            identity_status = recorder_identity_status(spec)
            if identity_status == "never_started":
                continue
            if identity_status == "launch_pending":
                errors.append(
                    f"{spec['kind']} recorder launch cleanup is unproven")
                continue
            if identity_status == "launch_reaped":
                try:
                    self._validate_pending_recorder_cleanup(spec)
                except HarnessError as error:
                    errors.append(str(error))
                continue
            if identity_status == "incomplete":
                errors.append(
                    f"incomplete {spec['kind']} recorder identity")
                continue
            if spec.get("status") != "reaped":
                errors.append(
                    f"{spec['kind']} recorder lacks reaped status")
                continue
            if spec.get("child_exit_status") != 0:
                errors.append(
                    f"{spec['kind']} recorder lacks zero child exit")
                continue
            if (
                    spec.get("launch_mode") !=
                    "detached_foreground_docker_exec" or
                    spec.get("wrapper_reap_owner") !=
                    "docker_exec_parent"):
                errors.append(
                    f"{spec['kind']} recorder lacks pinned reap ownership")
                continue
            evidence_value = spec.get("cleanup_evidence_path")
            evidence_sha256 = spec.get("cleanup_evidence_sha256")
            if not isinstance(evidence_value, str) or not re.fullmatch(
                    r"[0-9a-f]{64}", evidence_sha256 or ""):
                errors.append(
                    f"{spec['kind']} recorder cleanup evidence is missing")
                continue
            evidence_path = Path(evidence_value)
            expected_path = (
                self.store.directory /
                f"{spec['kind']}-recorder-cleanup.json")
            if (
                    evidence_path != expected_path or
                    not evidence_path.is_file() or
                    evidence_path.is_symlink()):
                errors.append(
                    f"{spec['kind']} recorder cleanup evidence path is invalid")
                continue
            if hashlib.sha256(
                    evidence_path.read_bytes()).hexdigest() != evidence_sha256:
                errors.append(
                    f"{spec['kind']} recorder cleanup evidence hash changed")
                continue
            try:
                evidence = json.loads(
                    evidence_path.read_text(encoding="utf-8"))
            except (json.JSONDecodeError, OSError) as error:
                errors.append(
                    f"{spec['kind']} recorder cleanup evidence unreadable: "
                    f"{error}")
                continue
            expected_fields = {
                "kind": spec["kind"],
                "container": spec["container"],
                "container_id": spec["container_id"],
                "pid": spec["pid"],
                "pgid": spec["pgid"],
                "sid": spec["sid"],
                "starttime": spec["starttime"],
                "token": spec.get("token"),
                "launch_mode": spec["launch_mode"],
                "wrapper_reap_owner": spec["wrapper_reap_owner"],
                "absence_proof": "10_consecutive_samples_100ms",
                "child_exit_status": 0,
            }
            if any(
                    evidence.get(key) != value
                    for key, value in expected_fields.items()):
                errors.append(
                    f"{spec['kind']} recorder cleanup evidence identity changed")
                continue
            try:
                run_checked(
                    self.runner,
                    docker_exec(
                        spec["container"],
                        self._recorder_completion_body(
                            spec, "RECORDER_ABSENCE_VERIFIED")),
                    12,
                    f"verify {spec['kind']} recorder post-reap absence")
            except HarnessError as error:
                errors.append(str(error))
                continue
            if evidence.get("outcome") not in {
                    "reaped", "already_missing"}:
                errors.append(
                    f"{spec['kind']} recorder cleanup outcome is invalid")
                continue
            log_path = self.store.directory / f"{spec['kind']}-recorder.log"
            if (
                    evidence.get("log_path") != str(log_path) or
                    not log_path.is_file() or log_path.is_symlink() or
                    hashlib.sha256(log_path.read_bytes()).hexdigest() !=
                    evidence.get("log_sha256")):
                errors.append(
                    f"{spec['kind']} recorder log evidence is invalid")
        if errors:
            raise HarnessError(
                "terminal recorder cleanup proof invalid: " +
                "; ".join(errors))

    def _verify_safe_zero(self, state):
        robot = state["recorders"]["robot"]
        self._verify_container(robot)
        body = f"timeout 5s python3 -c {shlex.quote(SAFE_ZERO_CHECK)}"
        run_checked(
            self.runner,
            docker_exec(robot["container"], ros_shell(robot["setup"], body)),
            7, "verify /cmd_vel/safe zero")

    def _verify_topic_message(self, spec, topic, expected_type):
        self._verify_container(spec)
        body = "\n".join([
            f"test \"$(ros2 topic type {shlex.quote(topic)})\" = "
            f"{shlex.quote(expected_type)}",
            f"timeout 5s ros2 topic echo --once {shlex.quote(topic)} "
            f"{shlex.quote(expected_type)} >/dev/null",
        ])
        run_checked(
            self.runner,
            docker_exec(spec["container"], ros_shell(spec["setup"], body)),
            7, f"verify live {topic} {expected_type}")

    def _verify_named_subscription(self, spec, topic, node_name):
        self._verify_container(spec)
        result = run_checked(
            self.runner,
            docker_exec(
                spec["container"],
                ros_shell(
                    spec["setup"],
                    f"ros2 topic info --verbose {shlex.quote(topic)}")),
            6, f"inspect {node_name} subscription for {topic}")
        if not topic_info_has_subscription(result.stdout, node_name):
            raise HarnessError(
                f"{node_name} subscription is absent for {topic}")

    def _verify_recorder_subscription(self, spec, topic):
        self._verify_named_subscription(
            spec, topic, "rosbag2_recorder")

    def _verify_arbiter_configuration(self, robot):
        self._verify_container(robot)
        body = "\n".join([
            "test \"$(ros2 param get /command_arbiter publish_rate_hz)\" = "
            f"\"Double value is: {ARBITER_PUBLISH_RATE_HZ}\"",
            "test \"$(ros2 param get /command_arbiter test_timeout_s)\" = "
            f"\"Double value is: {ARBITER_TEST_TIMEOUT_SECONDS}\"",
        ])
        run_checked(
            self.runner,
            docker_exec(
                robot["container"], ros_shell(robot["setup"], body)),
            6, "verify command-arbiter timing configuration")

    def _verify_pre_motion_data(self, state, audit_path):
        robot = state["recorders"]["robot"]
        imu = state["recorders"]["imu"]
        self._verify_topic_message(imu, "/camera/imu", "sensor_msgs/msg/Imu")
        self._verify_topic_message(
            robot, "/wheel_ticks", "roboteq_ros2_driver/msg/WheelTicks")
        self._verify_topic_message(robot, "/odom", "nav_msgs/msg/Odometry")
        for topic in PREMOTION_REQUIRED_TOPICS["imu"]:
            self._verify_recorder_subscription(imu, topic)
        for topic in PREMOTION_REQUIRED_TOPICS["robot"]:
            self._verify_recorder_subscription(robot, topic)
        self._verify_arbiter_configuration(robot)
        self._verify_container(robot)
        body = (
            f"timeout {DIAGNOSTIC_SHELL_TIMEOUT_SECONDS}s "
            f"python3 -c {shlex.quote(DIAGNOSTIC_CHECK)}")
        run_checked(
            self.runner,
            docker_exec(robot["container"], ros_shell(robot["setup"], body)),
            DIAGNOSTIC_COMMAND_TIMEOUT_SECONDS,
            "verify Roboteq ready/fresh diagnostics")
        audit_sha = validate_kernel_audit(audit_path, state["created_at"])
        state["kernel_audit"] = {
            "path": str(Path(audit_path).resolve()), "sha256": audit_sha,
            "validated_at": utc_now(),
        }
        self.store.save(state)
        if not self.store.event("pre_motion_gates_passed", audit_sha256=audit_sha):
            raise HarnessError("cannot record pre-motion gate evidence")

    def _persist_motion_evidence(self, state, key, artifact_name, evidence):
        path = self.store.directory / artifact_name
        atomic_write_json(path, evidence, exclusive=True)
        state[key] = {
            "path": str(path),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "evidence": evidence,
        }
        self.store.save(state)
        return path

    @staticmethod
    def _twist_publisher_child_script(
            gate_path, receipt_path, child_pid_path, exit_path, log_path,
            command, gate_wait_attempts=6000):
        if gate_wait_attempts <= 0:
            raise HarnessError("publisher gate wait must be positive")
        return "\n".join([
            "set -uo pipefail",
            f"printf 'D455_PUBLISHER_CHILD_BREADCRUMB %s %s\\n' "
            f"\"$$\" \"$$\" "
            f">{shlex.quote(child_pid_path)}",
            f"gate={shlex.quote(gate_path)}",
            f"receipt={shlex.quote(receipt_path)}",
            "pid=$$",
            "pgid=$(ps -o pgid= -p \"$pid\" | tr -d ' ')",
            "sid=$(ps -o sid= -p \"$pid\" | tr -d ' ')",
            "stat_rest=$(sed 's/^[^)]*) //' \"/proc/$pid/stat\")",
            "set -- $stat_rest",
            "starttime=${20}",
            "cmdline_hex=$(od -An -tx1 -v \"/proc/$pid/cmdline\" | "
            "tr -d ' \\n')",
            "test \"$pgid\" = \"$pid\" || exit 81",
            "test \"$sid\" = \"$pid\" || exit 82",
            "test -n \"$starttime\" || exit 83",
            "test -n \"$cmdline_hex\" || exit 84",
            "receipt_tmp=\"$receipt.tmp.$pid\"",
            "printf 'D455_PUBLISHER_RECEIPT %s %s %s %s %s\\n' "
            "\"$pid\" \"$pgid\" \"$sid\" \"$starttime\" \"$cmdline_hex\" "
            ">\"$receipt_tmp\"",
            "mv \"$receipt_tmp\" \"$receipt\"",
            f"for unused in $(seq 1 {gate_wait_attempts}); do",
            "  [ -e \"$gate\" ] && break",
            "  sleep 0.01",
            "done",
            "if [ ! -e \"$gate\" ]; then",
            "  child_status=78",
            "else",
            "  rm -f \"$gate\"",
            "  set +e",
            f"  {command} >{shlex.quote(log_path)} 2>&1 </dev/null",
            "  child_status=$?",
            "  set -e",
            "fi",
            f"status_tmp={shlex.quote(exit_path)}.tmp.$$",
            "printf '%s\\n' \"$child_status\" >\"$status_tmp\"",
            f"mv \"$status_tmp\" {shlex.quote(exit_path)}",
            "exit \"$child_status\"",
        ])

    def _start_twist_publisher(
            self, state, command_type, angular_z, duration, rate_hz, count,
            required_endpoints):
        robot = state["recorders"]["robot"]
        self._verify_container(robot)
        recorder_required = "/:rosbag2_recorder" in required_endpoints
        if recorder_required and (
                robot.get("qos_override_path") !=
                ROSBAG_QOS_OVERRIDE_CONTAINER_PATH or
                robot.get("qos_override_sha256") !=
                ROSBAG_QOS_OVERRIDE_SHA256 or
                robot.get("cmd_vel_test_qos") != CMD_VEL_TEST_QOS):
            raise HarnessError(
                "robot recorder QoS override is not pinned")
        token_id = uuid.uuid4().hex
        token_prefix = (
            f"d455-{command_type.replace('_', '-')}-publisher-")
        token = f"{token_prefix}{token_id}"
        result_path = f"/tmp/{token}.json"
        log_path = f"/tmp/{token}.log"
        exit_path = f"/tmp/{token}.exit"
        gate_path = f"/tmp/{token}.start"
        receipt_path = f"/tmp/{token}.receipt"
        child_pid_path = f"/tmp/{token}.child-pid"
        parent_pid_path = f"/tmp/{token}.parent-child-pid"
        phase_path = f"/tmp/{token}.phase"
        wait_path = f"/tmp/{token}.wait"
        hard_timeout = (
            MOTION_PUBLISHER_DISCOVERY_TIMEOUT_SECONDS + duration + 2.0)
        attempt = {
            "kind": f"{command_type} publisher",
            "command_type": command_type,
            "container": robot["container"],
            "container_id": robot["container_id"],
            "setup": robot["setup"],
            "token": token,
            "token_id": token_id,
            "token_prefix": token_prefix,
            "result_path": result_path,
            "log_path": log_path,
            "exit_path": exit_path,
            "gate_path": gate_path,
            "receipt_path": receipt_path,
            "child_pid_path": child_pid_path,
            "parent_pid_path": parent_pid_path,
            "phase_path": phase_path,
            "wait_path": wait_path,
            "hard_timeout": hard_timeout,
            "publisher_path": PUBLISHER_CONTAINER_PATH,
            "publisher_sha256": PUBLISHER_SHA256,
            "publisher_qos": dict(CMD_VEL_TEST_QOS),
            "required_subscription_endpoints": list(required_endpoints),
            "recorder_qos_override": (
                {
                    "path": ROSBAG_QOS_OVERRIDE_CONTAINER_PATH,
                    "sha256": ROSBAG_QOS_OVERRIDE_SHA256,
                    "verified_before_start": True,
                }
                if recorder_required else {
                    "verified_before_start": False,
                }),
            "status": "registered_before_launch",
            "gate_release": "unproven",
        }
        state.setdefault("publisher_launch_attempts", []).append(attempt)
        self.store.save(state)
        publisher_argv = [
            "python3", PUBLISHER_CONTAINER_PATH,
            "--command-type", command_type,
            "--payload-json", twist_yaml(angular_z),
            "--duration", f"{duration:.17g}",
            "--rate-hz", str(rate_hz),
            "--count", str(count),
        ]
        for endpoint in required_endpoints:
            publisher_argv.extend(["--required-endpoint", endpoint])
        if recorder_required:
            publisher_argv.extend([
                "--recorder-qos-override-path",
                ROSBAG_QOS_OVERRIDE_CONTAINER_PATH,
                "--recorder-qos-override-sha256",
                ROSBAG_QOS_OVERRIDE_SHA256,
            ])
        publisher_argv.extend([
            "--discovery-timeout",
            f"{MOTION_PUBLISHER_DISCOVERY_TIMEOUT_SECONDS:.17g}",
            "--result", result_path,
        ])
        command = (
            f"timeout --signal=TERM --kill-after=1s {hard_timeout:.1f}s " +
            shlex.join(publisher_argv))
        child_script = self._twist_publisher_child_script(
            gate_path, receipt_path, child_pid_path, exit_path, log_path,
            command)
        qos_override_checks = []
        if recorder_required:
            qos_override_checks = [
                f"test -f {shlex.quote(ROSBAG_QOS_OVERRIDE_CONTAINER_PATH)}",
                f"test ! -L "
                f"{shlex.quote(ROSBAG_QOS_OVERRIDE_CONTAINER_PATH)}",
                f"test \"$(sha256sum "
                f"{shlex.quote(ROSBAG_QOS_OVERRIDE_CONTAINER_PATH)} | "
                f"cut -d' ' -f1)\" = {ROSBAG_QOS_OVERRIDE_SHA256}",
            ]
        body = "\n".join([
            f"test -f {shlex.quote(PUBLISHER_CONTAINER_PATH)}",
            f"test ! -L {shlex.quote(PUBLISHER_CONTAINER_PATH)}",
            f"test \"$(sha256sum {shlex.quote(PUBLISHER_CONTAINER_PATH)} | "
            f"cut -d' ' -f1)\" = {PUBLISHER_SHA256}",
            *qos_override_checks,
            f"test ! -e {shlex.quote(result_path)}",
            f"test ! -e {shlex.quote(log_path)}",
            f"test ! -e {shlex.quote(exit_path)}",
            f"test ! -e {shlex.quote(gate_path)}",
            f"test ! -e {shlex.quote(receipt_path)}",
            f"test ! -e {shlex.quote(child_pid_path)}",
            f"test ! -e {shlex.quote(parent_pid_path)}",
            f"test ! -e {shlex.quote(phase_path)}",
            f"test ! -e {shlex.quote(wait_path)}",
            f"phase={shlex.quote(phase_path)}",
            "write_phase() {",
            "  phase_tmp=\"$phase.tmp.$$\"",
            "  printf '%s\\n' \"$1\" >\"$phase_tmp\"",
            "  mv \"$phase_tmp\" \"$phase\"",
            "}",
            "write_phase PRELAUNCH",
            "child=''",
            "child_pgid=''",
            "cleanup_wrapper() {",
            f"  rm -f {shlex.quote(gate_path)}",
            "  [ -n \"$child\" ] || return 0",
            "  if kill -0 \"$child\" 2>/dev/null; then",
            "    if [ -n \"$child_pgid\" ]; then",
            "      kill -TERM -- \"-$child_pgid\" 2>/dev/null || true",
            "    else",
            "      kill -TERM \"$child\" 2>/dev/null || true",
            "    fi",
            "  fi",
            "  set +e",
            "  wait \"$child\" 2>/dev/null",
            "  set -e",
            "  child=''",
            "  child_pgid=''",
            "}",
            "terminate_wrapper() {",
            "  termination_status=$1",
            "  trap - EXIT HUP INT TERM",
            "  cleanup_wrapper",
            "  exit \"$termination_status\"",
            "}",
            "trap cleanup_wrapper EXIT",
            "trap 'terminate_wrapper 129' HUP",
            "trap 'terminate_wrapper 130' INT",
            "trap 'terminate_wrapper 143' TERM",
            "setsid bash -c " + shlex.quote(child_script) +
            f" {shlex.quote(token)} &",
            "child=$!",
            "printf 'D455_PUBLISHER_PARENT_BREADCRUMB %s %s\\n' "
            "\"$child\" \"$child\" "
            f">{shlex.quote(parent_pid_path)}",
            "for unused in $(seq 1 50); do",
            "  kill -0 \"$child\"",
            f"  test -f {shlex.quote(receipt_path)} && break",
            "  sleep 0.1",
            "done",
            "kill -0 \"$child\"",
            f"test -f {shlex.quote(receipt_path)}",
            f"test ! -L {shlex.quote(receipt_path)}",
            f"receipt=$(cat {shlex.quote(receipt_path)})",
            "set -- $receipt",
            "test \"$#\" = 6",
            "test \"$1\" = D455_PUBLISHER_RECEIPT",
            "shift",
            "pid=$1; pgid=$2; sid=$3; starttime=$4; cmdline_hex=$5",
            "test \"$pid\" = \"$child\"",
            "test \"$pgid\" = \"$child\"",
            "test \"$sid\" = \"$child\"",
            "test -r \"/proc/$child/stat\"",
            "actual_start=$(sed 's/^[^)]*) //' \"/proc/$child/stat\" | "
            "awk '{print $20}')",
            "actual_cmd=$(od -An -tx1 -v \"/proc/$child/cmdline\" | "
            "tr -d ' \\n')",
            "test \"$actual_start\" = \"$starttime\"",
            "test \"$actual_cmd\" = \"$cmdline_hex\"",
            "child_pgid=$pgid",
            "printf 'ROTATION_TWIST_PUBLISHER_ID %s %s %s %s %s\\n' "
            "\"$child\" \"$pgid\" \"$sid\" \"$starttime\" \"$cmdline_hex\"",
            "write_phase GATE_RELEASE_AUTHORIZED",
            f"touch {shlex.quote(gate_path)}",
            "set +e",
            "wait \"$child\"",
            "wait_status=$?",
            "set -e",
            f"wait_tmp={shlex.quote(wait_path)}.tmp.$$",
            "printf '%s\\n' \"$wait_status\" >\"$wait_tmp\"",
            f"mv \"$wait_tmp\" {shlex.quote(wait_path)}",
            "reaped_pid=$child",
            "child=''",
            "test ! -e \"/proc/$reaped_pid\"",
            "ps -eo pgid=,stat= | awk -v wanted=\"$pgid\" "
            f"{shlex.quote(GROUP_EMPTY_AWK)}",
            f"rm -f {shlex.quote(gate_path)}",
            "write_phase REAPED",
            "trap - EXIT HUP INT TERM",
            "printf 'ROTATION_TWIST_PUBLISHER_REAPED %s %s\\n' "
            "\"$reaped_pid\" \"$wait_status\"",
        ])
        try:
            result = run_checked(
                self.runner,
                docker_exec(
                    robot["container"], ros_shell(robot["setup"], body)),
                hard_timeout + 5.0,
                f"run and reap owned {command_type} publisher")
        except BaseException as start_error:
            try:
                self._recover_failed_twist_publisher_start(attempt)
                self.store.save(state)
            except BaseException as cleanup_error:
                combined = HarnessError(
                    f"{command_type} publisher run failed: {start_error}; "
                    f"launch recovery failed: {cleanup_error}")
                combined._rotation_twist_publisher_spec = attempt
                raise combined from start_error
            start_error._rotation_twist_publisher_spec = attempt
            raise
        identity_matches = re.findall(
            r"(?m)^ROTATION_TWIST_PUBLISHER_ID "
            r"(\d+) (\d+) (\d+) (\d+) ([0-9a-f]+)$",
            result.stdout)
        reaped_matches = re.findall(
            r"(?m)^ROTATION_TWIST_PUBLISHER_REAPED (\d+) (\d{1,3})$",
            result.stdout)
        if len(identity_matches) != 1 or len(reaped_matches) != 1:
            error = HarnessError(
                f"invalid {command_type} publisher completion identity: "
                f"{result.stdout!r}")
            try:
                self._recover_failed_twist_publisher_start(attempt)
                self.store.save(state)
            except BaseException as cleanup_error:
                combined = HarnessError(
                    f"{error}; launch recovery failed: {cleanup_error}")
                combined._rotation_twist_publisher_spec = attempt
                raise combined from error
            error._rotation_twist_publisher_spec = attempt
            raise error
        identity_match = identity_matches[0]
        reaped_match = reaped_matches[0]
        pid, pgid, sid, starttime = (
            int(identity_match[index]) for index in range(4))
        reaped_pid = int(reaped_match[0])
        wrapper_wait_status = int(reaped_match[1])
        if (
                min(pid, pgid, sid, starttime) <= 1 or
                reaped_pid != pid or wrapper_wait_status > 255):
            error = HarnessError(
                f"refusing unsafe or inconsistent {command_type} "
                "publisher completion identity")
            try:
                self._recover_failed_twist_publisher_start(attempt)
                self.store.save(state)
            except BaseException as cleanup_error:
                combined = HarnessError(
                    f"{error}; launch recovery failed: {cleanup_error}")
                combined._rotation_twist_publisher_spec = attempt
                raise combined from error
            error._rotation_twist_publisher_spec = attempt
            raise error
        attempt.update({
            "pid": pid,
            "pgid": pgid,
            "sid": sid,
            "starttime": starttime,
            "cmdline_hex": identity_match[4],
            "identity_pinned_before_launch": True,
            "wrapper_wait_status": wrapper_wait_status,
            "reaped_by_launch_parent": True,
            "gate_release": "authorized",
            "status": "reaped_pending_verification",
        })
        self.store.save(state)
        return attempt

    def _publisher_launch_snapshot(self, attempt):
        phase = shlex.quote(attempt["phase_path"])
        receipt = shlex.quote(attempt["receipt_path"])
        child_pid_path = shlex.quote(attempt["child_pid_path"])
        parent_pid_path = shlex.quote(attempt["parent_pid_path"])
        wait_path = shlex.quote(attempt["wait_path"])
        gate = shlex.quote(attempt["gate_path"])
        body = "\n".join([
            "set -eo pipefail",
            f"test -f {phase}",
            f"test ! -L {phase}",
            f"test \"$(stat -c %s {phase})\" -gt 0",
            f"test \"$(stat -c %s {phase})\" -le 64",
            "for unused in $(seq 1 100); do",
            f"  phase_probe=$(cat {phase})",
            f"  [ -e {receipt} ] && break",
            "  [ \"$phase_probe\" != PRELAUNCH ] && break",
            "  sleep 0.01",
            "done",
            f"phase_value=$(cat {phase})",
            "case \"$phase_value\" in",
            "  PRELAUNCH|GATE_RELEASE_AUTHORIZED|REAPED) ;;",
            "  *) exit 85;;",
            "esac",
            f"if [ -e {gate} ]; then gate_value=PRESENT; "
            "else gate_value=ABSENT; fi",
            "printf 'D455_PUBLISHER_ATTEMPT_PHASE %s %s\\n' "
            "\"$phase_value\" \"$gate_value\"",
            f"if [ -e {child_pid_path} ]; then",
            f"  test -f {child_pid_path}",
            f"  test ! -L {child_pid_path}",
            f"  test \"$(stat -c %s {child_pid_path})\" -gt 0",
            f"  test \"$(stat -c %s {child_pid_path})\" -le 64",
            f"  cat {child_pid_path}",
            "else",
            "  printf 'D455_PUBLISHER_NO_CHILD_BREADCRUMB\\n'",
            "fi",
            f"if [ -e {parent_pid_path} ]; then",
            f"  test -f {parent_pid_path}",
            f"  test ! -L {parent_pid_path}",
            f"  test \"$(stat -c %s {parent_pid_path})\" -gt 0",
            f"  test \"$(stat -c %s {parent_pid_path})\" -le 64",
            f"  cat {parent_pid_path}",
            "else",
            "  printf 'D455_PUBLISHER_NO_PARENT_BREADCRUMB\\n'",
            "fi",
            f"if [ -e {receipt} ]; then",
            f"  test -f {receipt}",
            f"  test ! -L {receipt}",
            f"  test \"$(stat -c %s {receipt})\" -gt 0",
            f"  test \"$(stat -c %s {receipt})\" -le 4096",
            f"  cat {receipt}",
            "else",
            "  printf 'D455_PUBLISHER_NO_RECEIPT\\n'",
            "fi",
            f"if [ -e {wait_path} ]; then",
            f"  test -f {wait_path}",
            f"  test ! -L {wait_path}",
            f"  test \"$(stat -c %s {wait_path})\" -gt 0",
            f"  test \"$(stat -c %s {wait_path})\" -le 16",
            f"  wait_value=$(cat {wait_path})",
            "  case \"$wait_value\" in ''|*[!0-9]*) exit 86;; esac",
            "  test \"$wait_value\" -le 255",
            "  printf 'D455_PUBLISHER_WAIT_STATUS %s\\n' \"$wait_value\"",
            "else",
            "  printf 'D455_PUBLISHER_NO_WAIT_STATUS\\n'",
            "fi",
        ])
        result = run_checked(
            self.runner,
            docker_exec(attempt["container"], body),
            4.0,
            f"inspect failed {attempt['command_type']} publisher launch")
        phase_matches = re.findall(
            r"(?m)^D455_PUBLISHER_ATTEMPT_PHASE "
            r"(PRELAUNCH|GATE_RELEASE_AUTHORIZED|REAPED) "
            r"(PRESENT|ABSENT)$",
            result.stdout)
        receipt_matches = re.findall(
            r"(?m)^D455_PUBLISHER_RECEIPT "
            r"(\d+) (\d+) (\d+) (\d+) ([0-9a-f]+)$",
            result.stdout)
        child_pid_matches = re.findall(
            r"(?m)^D455_PUBLISHER_CHILD_BREADCRUMB (\d+) (\d+)$",
            result.stdout)
        no_child_pid = len(re.findall(
            r"(?m)^D455_PUBLISHER_NO_CHILD_BREADCRUMB$",
            result.stdout))
        parent_pid_matches = re.findall(
            r"(?m)^D455_PUBLISHER_PARENT_BREADCRUMB (\d+) (\d+)$",
            result.stdout)
        no_parent_pid = len(re.findall(
            r"(?m)^D455_PUBLISHER_NO_PARENT_BREADCRUMB$",
            result.stdout))
        no_receipt = len(re.findall(
            r"(?m)^D455_PUBLISHER_NO_RECEIPT$", result.stdout))
        wait_matches = re.findall(
            r"(?m)^D455_PUBLISHER_WAIT_STATUS (\d{1,3})$",
            result.stdout)
        no_wait = len(re.findall(
            r"(?m)^D455_PUBLISHER_NO_WAIT_STATUS$", result.stdout))
        if (
                len(phase_matches) != 1 or
                len(child_pid_matches) + no_child_pid != 1 or
                len(parent_pid_matches) + no_parent_pid != 1 or
                len(receipt_matches) + no_receipt != 1 or
                len(wait_matches) + no_wait != 1):
            raise HarnessError(
                "publisher launch snapshot is incomplete or ambiguous")
        return {
            "phase": phase_matches[0][0],
            "gate": phase_matches[0][1],
            "child_breadcrumb": (
                tuple(int(value) for value in child_pid_matches[0])
                if child_pid_matches else None),
            "parent_breadcrumb": (
                tuple(int(value) for value in parent_pid_matches[0])
                if parent_pid_matches else None),
            "identity": receipt_matches[0] if receipt_matches else None,
            "wait_status": (
                int(wait_matches[0]) if wait_matches else None),
        }

    def _reap_never_released_publisher(self, attempt, pid, pgid):
        token = attempt["token"]
        body = "\n".join([
            "set -eo pipefail",
            f"pid={pid}",
            f"pgid={pgid}",
            f"token={shlex.quote(token)}",
            "test \"$pid\" = \"$pgid\"",
            "verifier_pid=$$",
            "group_empty() {",
            "  ps -eo pgid=,stat= | awk -v wanted=\"$pgid\" "
            f"{shlex.quote(GROUP_EMPTY_AWK)}",
            "}",
            "token_present() {",
            "  for path in /proc/[0-9]*/cmdline; do",
            "    [ -r \"$path\" ] || continue",
            "    candidate=${path#/proc/}; "
            "candidate=${candidate%/cmdline}",
            "    [ \"$candidate\" = \"$verifier_pid\" ] && continue",
            "    cmd=$(tr '\\0' ' ' < \"$path\")",
            "    case \"$cmd\" in *\"$token\"*) return 0;; esac",
            "  done",
            "  return 1",
            "}",
            "all_absent() {",
            "  test ! -e \"/proc/$pid\" && group_empty && "
            "! token_present",
            "}",
            "if [ -e \"/proc/$pid\" ]; then",
            "  test -r \"/proc/$pid/stat\"",
            "  actual_pgid=$(ps -o pgid= -p \"$pid\" | tr -d ' ')",
            "  actual_sid=$(ps -o sid= -p \"$pid\" | tr -d ' ')",
            "  stat_rest=$(sed 's/^[^)]*) //' \"/proc/$pid/stat\")",
            "  set -- $stat_rest",
            "  actual_state=$1",
            "  actual_cmd=$(od -An -tx1 -v \"/proc/$pid/cmdline\" | "
            "tr -d ' \\n')",
            "  test \"$actual_pgid\" = \"$pgid\"",
            "  test \"$actual_sid\" = \"$pid\"",
            "  case \"$actual_state\" in",
            "    Z) exit 44;;",
            "    R|S|D|I|T|t|W) ;;",
            "    *) exit 43;;",
            "  esac",
            "  test -n \"$actual_cmd\"",
            "  expected_token_hex=$(printf '%s' \"$token\" | "
            "od -An -tx1 -v | tr -d ' \\n')",
            "  case \"$actual_cmd\" in *\"$expected_token_hex\"*) ;; "
            "*) exit 45;; esac",
            "  kill -TERM -- \"-$pgid\" 2>/dev/null || true",
            "fi",
            "for unused in $(seq 1 100); do",
            "  all_absent && {",
            "    printf 'TWIST_NEVER_RELEASED_REAP_VERIFIED\\n'",
            "    exit 0",
            "  }",
            "  sleep 0.01",
            "done",
            "kill -KILL -- \"-$pgid\" 2>/dev/null || true",
            "for unused in $(seq 1 100); do",
            "  all_absent && {",
            "    printf 'TWIST_NEVER_RELEASED_REAP_VERIFIED\\n'",
            "    exit 0",
            "  }",
            "  sleep 0.01",
            "done",
            "exit 46",
        ])
        result = run_checked(
            self.runner,
            docker_exec(attempt["container"], body),
            5.0,
            f"reap never-released {attempt['command_type']} publisher")
        if "TWIST_NEVER_RELEASED_REAP_VERIFIED" not in result.stdout:
            raise HarnessError(
                "never-released publisher reap outcome is unproven")

    def _recover_failed_twist_publisher_start(self, attempt):
        self._verify_container(attempt)
        snapshot = self._publisher_launch_snapshot(attempt)
        attempt["gate_release_phase_observed"] = snapshot["phase"]
        attempt["gate_path_observed"] = snapshot["gate"].lower()
        breadcrumbs = [
            breadcrumb for breadcrumb in (
                snapshot["child_breadcrumb"],
                snapshot["parent_breadcrumb"])
            if breadcrumb is not None
        ]
        if not breadcrumbs:
            raise HarnessError(
                "publisher launch has no durable PID/PGID breadcrumb")
        if any(
                pid <= 1 or pgid <= 1 or pid != pgid
                for pid, pgid in breadcrumbs):
            raise HarnessError(
                "publisher launch breadcrumb is unsafe")
        if len(set(breadcrumbs)) != 1:
            raise HarnessError(
                "publisher launch breadcrumbs conflict")
        breadcrumb_pid, breadcrumb_pgid = breadcrumbs[0]
        attempt["breadcrumb_pid"] = breadcrumb_pid
        attempt["breadcrumb_pgid"] = breadcrumb_pgid
        identity = snapshot["identity"]
        if identity is None:
            if (
                    snapshot["phase"] != "PRELAUNCH" or
                    snapshot["gate"] != "ABSENT"):
                raise HarnessError(
                    "publisher identity is missing after gate release "
                    "could have been authorized")
            self._reap_never_released_publisher(
                attempt, breadcrumb_pid, breadcrumb_pgid)
            attempt["gate_release"] = "never_authorized"
            attempt["identity_pinned_before_launch"] = False
            attempt["status"] = "never_released_reaped"
            return

        pid, pgid, sid, starttime = (
            int(identity[index]) for index in range(4))
        if min(pid, pgid, sid, starttime) <= 1:
            raise HarnessError(
                "publisher launch receipt contains unsafe identity")
        if (pid, pgid) != (breadcrumb_pid, breadcrumb_pgid):
            raise HarnessError(
                "publisher launch receipt conflicts with breadcrumb")
        attempt.update({
            "pid": pid,
            "pgid": pgid,
            "sid": sid,
            "starttime": starttime,
            "cmdline_hex": identity[4],
            "identity_pinned_before_launch": True,
            "gate_release": (
                "authorized"
                if snapshot["phase"] in {
                    "GATE_RELEASE_AUTHORIZED", "REAPED"}
                else "never_authorized"),
            "status": (
                "reaped_pending_verification"
                if snapshot["wait_status"] is not None
                else "launch_recovery_required"),
        })
        if snapshot["wait_status"] is not None:
            attempt["wrapper_wait_status"] = snapshot["wait_status"]
            attempt["reaped_by_launch_parent"] = True
        self._stop_twist_publisher(attempt)

    @staticmethod
    def _publisher_token_absence_body(token):
        return "\n".join([
            f"token={shlex.quote(token)}",
            "verifier_pid=$$",
            "if [ -r \"/proc/$PPID/cmdline\" ]; then",
            "  parent_cmd=$(tr '\\0' ' ' < \"/proc/$PPID/cmdline\")",
            "  case \"$parent_cmd\" in *\"$token\"*) exit 77;; esac",
            "fi",
            "for path in /proc/[0-9]*/cmdline; do",
            "  [ -r \"$path\" ] || continue",
            "  candidate=${path#/proc/}; candidate=${candidate%/cmdline}",
            "  [ \"$candidate\" = \"$verifier_pid\" ] && continue",
            "  cmd=$(tr '\\0' ' ' < \"$path\")",
            "  case \"$cmd\" in *\"$token\"*) exit 76;; esac",
            "done",
        ])

    @classmethod
    def _publisher_reap_verification_body(cls, spec):
        return "\n".join([
            "set -eo pipefail",
            f"pid={spec['pid']}",
            f"pgid={spec['pgid']}",
            "test ! -e \"/proc/$pid\"",
            "ps -eo pgid=,stat= | awk -v wanted=\"$pgid\" "
            f"{shlex.quote(GROUP_EMPTY_AWK)}",
            cls._publisher_token_absence_body(spec["token"]),
            "printf 'TWIST_PUBLISHER_REAP_VERIFIED\\n'",
        ])

    def _wait_twist_publisher(self, spec):
        if spec.get("status") not in {
                "reaped_pending_verification", "reaped",
                "never_released_reaped"}:
            raise HarnessError(
                f"{spec['command_type']} publisher is not awaiting "
                "strict reap verification")
        self._verify_container(spec)
        if spec.get("status") == "never_released_reaped":
            return
        body = self._publisher_reap_verification_body(spec)
        result = run_checked(
            self.runner,
            docker_exec(spec["container"], body),
            4.0, f"verify reaped {spec['command_type']} publisher")
        if "TWIST_PUBLISHER_REAP_VERIFIED" not in result.stdout:
            raise HarnessError(
                f"{spec['command_type']} publisher reaping is unproven")
        spec["status"] = "reaped"
        spec["reap_verified_at"] = utc_now()

    def _stop_twist_publisher(self, spec):
        self._verify_container(spec)
        if spec.get("status") in {
                "reaped_pending_verification", "reaped",
                "never_released_reaped"}:
            self._wait_twist_publisher(spec)
            return
        body = "\n".join([
            "set -eo pipefail",
            f"pid={spec['pid']}", f"pgid={spec['pgid']}",
            "if [ ! -e \"/proc/$pid\" ]; then",
            "  ps -eo pgid=,stat= | awk -v wanted=\"$pgid\" "
            f"{shlex.quote(GROUP_EMPTY_AWK)}",
            "  " + self._publisher_token_absence_body(
                spec["token"]).replace("\n", "\n  "),
            "  printf 'TWIST_PUBLISHER_STOPPED\\n'",
            "  exit 0",
            "fi",
            "test -r \"/proc/$pid/stat\"",
            "actual_pgid=$(ps -o pgid= -p \"$pid\" | tr -d ' ')",
            "actual_sid=$(ps -o sid= -p \"$pid\" | tr -d ' ')",
            "stat_rest=$(sed 's/^[^)]*) //' \"/proc/$pid/stat\")",
            "set -- $stat_rest",
            "actual_state=$1",
            "actual_start=${20}",
            "actual_cmd=$(od -An -tx1 -v \"/proc/$pid/cmdline\" | tr -d ' \\n')",
            self._publisher_stop_decision_body(spec),
        ])
        result = run_checked(
            self.runner,
            docker_exec(spec["container"], body),
            15, f"stop owned {spec['command_type']} publisher")
        if "TWIST_PUBLISHER_STOPPED" not in result.stdout:
            raise HarnessError(
                f"{spec['command_type']} publisher stop outcome is unproven")
        spec["status"] = "reaped"
        spec["reap_verified_at"] = utc_now()

    @staticmethod
    def _publisher_stop_decision_body(spec):
        return "\n".join([
            "set -eo pipefail",
            f"test \"$actual_pgid\" = {spec['pgid']}",
            f"test \"$actual_sid\" = {spec['sid']}",
            f"test \"$actual_start\" = {spec['starttime']}",
            "case \"$actual_state\" in R|S|D|I|T|t|W) ;; Z) exit 44;; "
            "*) exit 43;; esac",
            "test -n \"$actual_cmd\"",
            f"test \"$actual_cmd\" = {shlex.quote(spec['cmdline_hex'])}",
            "group_empty() {",
            "  ps -eo pgid=,stat= | awk -v wanted=\"$pgid\" "
            f"{shlex.quote(GROUP_EMPTY_AWK)}",
            "}",
            "token_absent() {",
            "  " + RotationHarness._publisher_token_absence_body(
                spec["token"]).replace("\n", "\n  "),
            "}",
            "kill -INT -- \"-$pgid\"",
            "for unused in $(seq 1 100); do",
            "  group_empty && { test ! -e \"/proc/$pid\"; token_absent; "
            "printf 'TWIST_PUBLISHER_STOPPED\\n'; exit 0; }",
            "  sleep 0.1",
            "done",
            "kill -TERM -- \"-$pgid\" 2>/dev/null || true",
            "for unused in $(seq 1 20); do",
            "  group_empty && { test ! -e \"/proc/$pid\"; token_absent; "
            "printf 'TWIST_PUBLISHER_STOPPED\\n'; exit 0; }",
            "  sleep 0.1",
            "done",
            "kill -KILL -- \"-$pgid\" 2>/dev/null || true",
            "for unused in $(seq 1 10); do",
            "  group_empty && { test ! -e \"/proc/$pid\"; token_absent; "
            "printf 'TWIST_PUBLISHER_STOPPED\\n'; exit 0; }",
            "  sleep 0.1",
            "done",
            "exit 42",
        ])

    def _read_publisher_file(
            self, spec, remote_path, description, maximum_size,
            allow_empty=False):
        self._verify_container(spec)
        path = shlex.quote(remote_path)
        minimum = "-ge 0" if allow_empty else "-gt 0"
        body = "\n".join([
            "set -eo pipefail",
            "# D455_PUBLISHER_FILE_READ",
            f"test -f {path}",
            f"test ! -L {path}",
            f"size=$(stat -c %s {path})",
            f"test \"$size\" {minimum}",
            f"test \"$size\" -le {maximum_size}",
            f"cat {path}",
        ])
        return run_checked(
            self.runner,
            docker_exec(spec["container"], body),
            4, description).stdout

    def _read_twist_publisher_evidence(self, spec):
        raw = self._read_publisher_file(
            spec, spec["result_path"],
            f"read {spec['command_type']} publisher evidence", 1048576)
        try:
            evidence = json.loads(raw)
        except json.JSONDecodeError as error:
            raise HarnessError(
                f"invalid {spec['command_type']} publisher evidence JSON: "
                f"{error}") from error
        if not isinstance(evidence, dict):
            raise HarnessError("publisher evidence must be a JSON object")
        return evidence

    def _read_twist_publisher_exit_status(self, spec):
        raw = self._read_publisher_file(
            spec, spec["exit_path"],
            f"read {spec['command_type']} publisher exit status", 16).strip()
        if not re.fullmatch(r"\d{1,3}", raw):
            raise HarnessError(
                f"invalid {spec['command_type']} publisher exit status: "
                f"{raw!r}")
        status = int(raw)
        if status > 255:
            raise HarnessError(
                f"invalid {spec['command_type']} publisher exit status: "
                f"{raw!r}")
        return status

    def _persist_publisher_run(
            self, state, spec, evidence, exit_status, log_text, state_key=None):
        sequence = state.get("publisher_run_sequence", 0) + 1
        state["publisher_run_sequence"] = sequence
        slug = f"{sequence:02d}-{spec['command_type'].replace('_', '-')}"
        log_path = self.store.directory / f"{slug}-publisher.log"
        result_name = (
            MOTION_PUBLISHER_EVIDENCE_NAME
            if spec["command_type"] == "motion"
            else f"{slug}-publisher-evidence.json")
        result_path = self.store.directory / result_name
        with open(log_path, "x", encoding="utf-8") as stream:
            stream.write(log_text)
            stream.flush()
            os.fsync(stream.fileno())
        persisted_evidence = dict(evidence)
        persisted_evidence["child_exit_status"] = exit_status
        persisted_evidence["stdout_stderr_path"] = str(log_path)
        persisted_evidence["json_result_path"] = str(result_path)
        atomic_write_json(result_path, persisted_evidence, exclusive=True)
        record = {
            "command_type": spec["command_type"],
            "process": spec,
            "child_exit_status": exit_status,
            "stdout_stderr_path": str(log_path),
            "stdout_stderr_sha256": hashlib.sha256(
                log_path.read_bytes()).hexdigest(),
            "json_result_path": str(result_path),
            "json_result_sha256": hashlib.sha256(
                result_path.read_bytes()).hexdigest(),
            "path": str(result_path),
            "sha256": hashlib.sha256(result_path.read_bytes()).hexdigest(),
            "evidence": persisted_evidence,
        }
        state.setdefault("publisher_runs", []).append(record)
        if state_key is not None:
            state[state_key] = record
        self.store.save(state)
        return record

    def _run_twist_publisher(
            self, state, command_type, angular_z, duration, rate_hz, count,
            required_endpoints=REQUIRED_PUBLISHER_ENDPOINTS,
            state_key=None):
        if command_type in ZERO_COMMAND_TYPES and angular_z != 0.0:
            raise HarnessError(
                f"{command_type} publisher requires exact-zero Twist")
        if command_type not in {*ZERO_COMMAND_TYPES, "motion"}:
            raise HarnessError(
                f"unsupported publisher command type: {command_type}")
        spec = None
        persisted = False
        try:
            spec = self._start_twist_publisher(
                state, command_type, angular_z, duration, rate_hz, count,
                required_endpoints)
            if command_type == "motion":
                state["motion_publisher_process"] = spec
            self.store.save(state)
            if not self.store.event(
                    "publisher_wrapper_reaped", command_type=command_type,
                    pid=spec["pid"], pgid=spec["pgid"], token=spec["token"],
                    identity_pinned_before_launch=True,
                    wrapper_wait_status=spec["wrapper_wait_status"]):
                raise HarnessError(
                    "cannot record publisher wrapper-reap evidence")
            self._wait_twist_publisher(spec)
            self.store.save(state)
            if not self.store.event(
                    "publisher_reap_verified", command_type=command_type):
                raise HarnessError("cannot record publisher reaping")
            evidence = self._read_twist_publisher_evidence(spec)
            exit_status = self._read_twist_publisher_exit_status(spec)
            log_text = self._read_publisher_file(
                spec, spec["log_path"],
                f"read {command_type} publisher stdout/stderr",
                1048576, allow_empty=True)
            record = self._persist_publisher_run(
                state, spec, evidence, exit_status, log_text,
                state_key=state_key)
            persisted = True
            if exit_status != spec["wrapper_wait_status"]:
                raise HarnessError(
                    f"{command_type} publisher exit evidence is ambiguous: "
                    f"wait={spec['wrapper_wait_status']} "
                    f"artifact={exit_status}")
            evidence = record["evidence"]
            qos_acceptance = {
                assessment.get("endpoint"): {
                    "status": assessment.get("status"),
                    "tolerated_unreported_fields": assessment.get(
                        "tolerated_unreported_fields", []),
                }
                for assessment in evidence.get(
                    "subscription_qos_assessments", [])
                if isinstance(assessment, dict) and
                isinstance(assessment.get("endpoint"), str)
            }
            if not self.store.event(
                    "publisher_evidence_persisted",
                    command_type=command_type,
                    path=record["json_result_path"],
                    status=evidence.get("status"),
                    child_exit_status=exit_status,
                    actual_publish_count=evidence.get(
                        "actual_publish_count"),
                    qos_acceptance=qos_acceptance,
                    recorder_qos_override=evidence.get(
                        "recorder_qos_override")):
                raise HarnessError("cannot record publisher evidence")
            validate_publisher_evidence(
                evidence, count, duration, rate_hz, angular_z,
                command_type=command_type,
                required_endpoints=required_endpoints)
            if not self.store.event(
                    "publisher_completed", command_type=command_type,
                    actual_publish_count=evidence["actual_publish_count"],
                    window_duration_s=evidence["timing"]["window_duration_s"],
                    publish_span_s=evidence["timing"]["publish_span_s"],
                    qos_acceptance=qos_acceptance):
                raise HarnessError("cannot record publisher completion")
            return evidence
        except BaseException as error:
            if spec is None:
                spec = getattr(
                    error, "_rotation_twist_publisher_spec", None)
            cleanup_errors = []
            if spec is not None and spec.get("status") not in {
                    "reaped", "never_released_reaped"}:
                try:
                    if all(
                            field in spec
                            for field in RECORDER_IDENTITY_FIELDS):
                        self._stop_twist_publisher(spec)
                    else:
                        self._recover_failed_twist_publisher_start(spec)
                    self.store.event(
                        "publisher_cleanup_reaped",
                        command_type=command_type,
                        method="pinned_identity_or_completion")
                except BaseException as identity_error:
                    cleanup_errors.append(
                        f"{command_type} publisher reaping failed: "
                        f"{identity_error}")
            partial_expected = (
                spec is not None and
                spec.get("gate_release") == "authorized")
            if partial_expected and not persisted:
                try:
                    evidence = self._read_twist_publisher_evidence(spec)
                    exit_status = self._read_twist_publisher_exit_status(spec)
                    log_text = self._read_publisher_file(
                        spec, spec["log_path"],
                        f"read partial {command_type} publisher log",
                        1048576, allow_empty=True)
                    record = self._persist_publisher_run(
                        state, spec, evidence, exit_status, log_text,
                        state_key=state_key)
                    persisted = True
                    self.store.event(
                        "publisher_partial_evidence_persisted",
                        command_type=command_type,
                        path=record["json_result_path"],
                        status=evidence.get("status"),
                        actual_publish_count=evidence.get(
                            "actual_publish_count"))
                except BaseException as evidence_error:
                    cleanup_errors.append(
                        f"{command_type} publisher partial evidence failed: "
                        f"{evidence_error}")
            if cleanup_errors:
                raise HarnessError(
                    f"{str(error) or error.__class__.__name__}; " +
                    "; ".join(cleanup_errors)) from error
            raise

    def _zero_required_endpoints(self, state):
        robot = state["recorders"]["robot"]
        recorder_expected = state.get("status") in {
            "preparing", "prepared", "motion_in_progress",
            "motion_completing", "motion_complete", "finalizing",
        }
        if (
                recorder_expected and
                recorder_identity_status(robot) == "started"):
            return REQUIRED_PUBLISHER_ENDPOINTS
        return ("/:command_arbiter",)

    def _publish_zero(
            self, state, command_type="cleanup_zero",
            required_endpoints=None):
        if required_endpoints is None:
            required_endpoints = self._zero_required_endpoints(state)
        return self._run_twist_publisher(
            state, command_type, 0.0,
            ZERO_MESSAGE_COUNT / ZERO_RATE_HZ,
            ZERO_RATE_HZ, ZERO_MESSAGE_COUNT,
            required_endpoints=required_endpoints)

    def _verify_prepare_topic_evidence(self, state, expected_count):
        robot = state["recorders"]["robot"]
        self._verify_recorder(robot)
        body = (
            f"timeout {PREPARE_TOPIC_EVIDENCE_SHELL_TIMEOUT_SECONDS}s "
            f"python3 -c {shlex.quote(PREPARE_TOPIC_EVIDENCE_CHECK)} "
            f"{shlex.quote(robot['bag_path'])} /cmd_vel/test "
            f"{PREPARE_TOPIC_EVIDENCE_TIMEOUT_SECONDS} {expected_count}")
        result = run_checked(
            self.runner,
            docker_exec(robot["container"], ros_shell(robot["setup"], body)),
            PREPARE_TOPIC_EVIDENCE_COMMAND_TIMEOUT_SECONDS,
            "prove active robot bag recorded prepare zero")
        match = re.search(
            r"(?m)^ROTATION_PREPARE_TOPIC_COUNT (\d+)$", result.stdout)
        if not match or int(match.group(1)) != expected_count:
            raise HarnessError(
                "prepare zero message count mismatch: "
                f"expected {expected_count}, output={result.stdout!r}")
        self._verify_recorder(robot)
        return int(match.group(1))

    def _prime_test_topic_and_verify_zero(self, state):
        self._publish_zero(state)
        if not self.store.event("test_topic_zero_prime_completed"):
            raise HarnessError("cannot record /cmd_vel/test zero-prime evidence")
        robot = state["recorders"]["robot"]
        self._verify_recorder_subscription(robot, "/cmd_vel/test")
        self._verify_named_subscription(
            robot, "/cmd_vel/test", "command_arbiter")
        self._verify_safe_zero(state)
        if not self.store.event("post_prime_safe_zero_verified"):
            raise HarnessError("cannot record post-prime safe-zero evidence")

    def _run_motion_publisher(
            self, state, angular_z, duration, rate_hz, count):
        return self._run_twist_publisher(
            state, "motion", angular_z, duration, rate_hz, count,
            required_endpoints=REQUIRED_PUBLISHER_ENDPOINTS,
            state_key="motion_publisher")

    def _collect_motion_delivery_evidence(
            self, state, angular_z, duration, rate_hz, count,
            artifact_name=MOTION_DELIVERY_EVIDENCE_NAME,
            state_key="motion_delivery", require_live_recorder=True):
        robot = state["recorders"]["robot"]
        self._verify_container(robot)
        if require_live_recorder:
            self._verify_recorder(robot)
        body = (
            f"timeout {MOTION_DELIVERY_SHELL_TIMEOUT_SECONDS}s "
            f"python3 -c {shlex.quote(MOTION_DELIVERY_EVIDENCE_CHECK)} "
            f"{shlex.quote(robot['bag_path'])} {angular_z:.17g} {count} "
            f"{duration:.17g} {rate_hz} "
            f"{MOTION_DELIVERY_EVIDENCE_TIMEOUT_SECONDS:.17g}")
        result = run_checked(
            self.runner,
            docker_exec(
                robot["container"], ros_shell(robot["setup"], body)),
            MOTION_DELIVERY_COMMAND_TIMEOUT_SECONDS,
            "collect recorded motion-delivery evidence")
        evidence = parse_json_marker(
            result.stdout + "\n" + result.stderr,
            MOTION_DELIVERY_MARKER)
        self._verify_container(robot)
        if require_live_recorder:
            self._verify_recorder(robot)
        path = self._persist_motion_evidence(
            state, state_key, artifact_name, evidence)
        if not self.store.event(
                "motion_delivery_evidence_persisted",
                path=str(path),
                state_key=state_key,
                test_nonzero_count=evidence.get(
                    "topics", {}).get(
                        "/cmd_vel/test", {}).get("nonzero_count"),
                safe_nonzero_count=evidence.get(
                    "topics", {}).get(
                        "/cmd_vel/safe", {}).get("nonzero_count")):
            raise HarnessError("cannot record motion delivery evidence")
        validate_delivery_evidence(
            evidence, count, duration, rate_hz, angular_z)
        if not self.store.event(
                "motion_delivery_verified", state_key=state_key):
            raise HarnessError("cannot record verified motion delivery")
        return evidence

    def _zero_and_verify(self, state, required_endpoints=None):
        errors = []
        try:
            self._publish_zero(
                state, required_endpoints=required_endpoints)
            if not self.store.event("zero_publisher_completed"):
                errors.append("cannot record zero publisher completion")
        except BaseException as error:
            if is_interruption(error):
                self.pending_interrupt = error
            errors.append(str(error) or error.__class__.__name__)
            self.store.event("zero_publisher_failed", error=str(error))
        try:
            self._verify_safe_zero(state)
            if not self.store.event("safe_zero_verified", samples=SAFE_ZERO_SAMPLES):
                errors.append("cannot record safe-zero verification")
        except BaseException as error:
            if is_interruption(error):
                self.pending_interrupt = error
            errors.append(str(error) or error.__class__.__name__)
            self.store.event("safe_zero_verification_failed", error=str(error))
        if errors:
            raise HarnessError("; ".join(errors))

    def _reap_tracked_publishers(self, state, phase):
        specs = []
        seen = set()
        for spec in state.get("publisher_launch_attempts", []):
            if isinstance(spec, dict):
                token = spec.get("token")
                identity = (
                    ("token", token)
                    if isinstance(token, str) and token
                    else ("object", id(spec)))
                if identity not in seen:
                    seen.add(identity)
                    specs.append(spec)
        for record in state.get("publisher_runs", []):
            if isinstance(record, dict):
                spec = record.get("process")
                if isinstance(spec, dict):
                    token = spec.get("token")
                    identity = (
                        ("token", token)
                        if isinstance(token, str) and token
                        else ("object", id(spec)))
                    if identity not in seen:
                        seen.add(identity)
                        specs.append(spec)
        motion_spec = state.get("motion_publisher_process")
        if isinstance(motion_spec, dict):
            token = motion_spec.get("token")
            identity = (
                ("token", token)
                if isinstance(token, str) and token
                else ("object", id(motion_spec)))
            if identity not in seen:
                seen.add(identity)
                specs.append(motion_spec)

        errors = []
        for spec in specs:
            command_type = spec.get("command_type", "unknown")
            try:
                if spec.get("status") == "never_released_reaped":
                    pass
                elif not all(
                        field in spec
                        for field in RECORDER_IDENTITY_FIELDS):
                    self._recover_failed_twist_publisher_start(spec)
                elif spec.get("status") in {
                        "reaped_pending_verification", "reaped"}:
                    self._wait_twist_publisher(spec)
                else:
                    self._stop_twist_publisher(spec)
                if not self.store.event(
                        "tracked_publisher_reap_verified",
                        phase=phase, command_type=command_type,
                        token=spec.get("token")):
                    raise HarnessError(
                        "cannot record tracked publisher reaping")
            except BaseException as error:
                if is_interruption(error):
                    self.pending_interrupt = error
                message = (
                    f"{command_type} tracked publisher reaping failed: "
                    f"{str(error) or error.__class__.__name__}")
                errors.append(message)
                self.store.event(
                    "tracked_publisher_reap_failed",
                    phase=phase, command_type=command_type,
                    token=spec.get("token"), error=str(error))
        if specs:
            try:
                self.store.save(state)
            except BaseException as error:
                if is_interruption(error):
                    self.pending_interrupt = error
                errors.append(
                    "tracked publisher reap state save failed: "
                    f"{str(error) or error.__class__.__name__}")
        return errors

    @staticmethod
    def _motion_publisher_absence_unproven(errors):
        return any(
            message.startswith(
                "motion tracked publisher reaping failed:")
            for message in errors)

    def prepare(self, args):
        created = utc_now()
        state = {
            "schema_version": 1, "trial_id": args.trial_id,
            "status": "preparing", "created_at": created, "updated_at": created,
            "recorders": {
                "robot": {
                    "kind": "robot", "container": args.robot_container,
                    "setup": args.robot_setup, "bag_path": args.robot_bag,
                    "log_path": args.robot_log, "topics": list(ROBOT_TOPICS),
                },
                "imu": {
                    "kind": "imu", "container": args.imu_container,
                    "setup": args.imu_setup, "bag_path": args.imu_bag,
                    "log_path": args.imu_log, "topics": list(IMU_TOPICS),
                },
            },
        }
        self.store.create(state)
        try:
            if not self.store.event("prepare_started", trial_id=args.trial_id):
                raise HarnessError("cannot record prepare start")
            robot = state["recorders"]["robot"]
            imu = state["recorders"]["imu"]
            robot["container_id"] = self._container_identity(robot["container"])
            self._start_recorder(robot)
            self.store.save(state)
            self._acknowledge_recorder(robot)
            if not self.store.event(
                    "recorder_started", kind=robot["kind"], pid=robot["pid"],
                    pgid=robot["pgid"], bag_path=robot["bag_path"]):
                raise HarnessError("cannot record recorder-start evidence")
            self._run_twist_publisher(
                state, "prepare_zero", 0.0,
                ZERO_MESSAGE_COUNT / ZERO_RATE_HZ,
                ZERO_RATE_HZ, ZERO_MESSAGE_COUNT,
                required_endpoints=REQUIRED_PUBLISHER_ENDPOINTS,
                state_key="prepare_zero_publisher")
            topic_count = self._verify_prepare_topic_evidence(
                state, ZERO_MESSAGE_COUNT)
            if not self.store.event(
                    "prepare_zero_recorded", topic="/cmd_vel/test",
                    count=topic_count):
                raise HarnessError("cannot record prepare zero-message evidence")

            imu["container_id"] = self._container_identity(imu["container"])
            self._start_recorder(imu)
            self.store.save(state)
            self._acknowledge_recorder(imu)
            if not self.store.event(
                    "recorder_started", kind=imu["kind"], pid=imu["pid"],
                    pgid=imu["pgid"], bag_path=imu["bag_path"]):
                raise HarnessError("cannot record recorder-start evidence")
            self._verify_safe_zero(state)
            if not self.store.event(
                    "prepare_safe_zero_verified", samples=SAFE_ZERO_SAMPLES):
                raise HarnessError("cannot record prepare safe-zero evidence")
            state["status"] = "prepared"
            self.store.save(state)
            if not self.store.event("prepare_completed"):
                raise HarnessError("cannot record prepare completion")
        except BaseException as error:
            if is_interruption(error):
                self.pending_interrupt = error
            cleanup_errors = []
            cleanup_errors.extend(self._reap_tracked_publishers(
                state, "prepare_failure_before_zero"))
            if "container_id" in state["recorders"]["robot"]:
                try:
                    self._zero_and_verify(
                        state,
                        required_endpoints=("/:command_arbiter",))
                except BaseException as cleanup_error:
                    cleanup_errors.append(
                        str(cleanup_error) or cleanup_error.__class__.__name__)
            cleanup_errors.extend(self._reap_tracked_publishers(
                state, "prepare_failure_after_zero"))
            cleanup_errors.extend(self._stop_all(state, allow_missing=True))
            cleanup_errors.extend(self._preserve_partial_bags(state))
            state["status"] = "invalid"
            failures = [str(error) or error.__class__.__name__, *cleanup_errors]
            state["failure"] = "; ".join(failures)
            try:
                self.store.save(state)
            except OSError as save_error:
                failures.append(f"state save failed: {save_error}")
            self.store.event("prepare_failed", errors=failures)
            if self.pending_interrupt is not None:
                raise self.pending_interrupt
            raise HarnessError(state["failure"]) from error

    def motion(self, args):
        validate_motion(
            args.angular_z, args.duration, args.rate_hz, args.linear_x,
            args.acknowledge_motion)
        state = self.store.load()
        if state["status"] != "prepared":
            raise HarnessError(f"motion requires prepared state, got {state['status']}")
        state["status"] = "motion_in_progress"
        self.store.save(state)
        motion_error = None
        zero_error = None
        interrupted = None
        try:
            for spec in state["recorders"].values():
                self._verify_recorder(spec)
            self._verify_pre_motion_data(state, args.kernel_audit_artifact)
            self._prime_test_topic_and_verify_zero(state)
            if not self.store.event(
                    "motion_intent", angular_z=args.angular_z,
                    duration=args.duration, rate_hz=args.rate_hz,
                    linear_x=args.linear_x):
                raise HarnessError("cannot record motion intent")
            count = int(args.duration * args.rate_hz)
            self._run_motion_publisher(
                state, args.angular_z, args.duration, args.rate_hz, count)
        except BaseException as error:
            motion_error = error
            if is_interruption(error):
                interrupted = error
            self.store.event("motion_failed", error=str(error))
        finally:
            reap_errors = self._reap_tracked_publishers(
                state, "motion_before_zero")
            if reap_errors:
                combined = "; ".join(reap_errors)
                motion_error = HarnessError(
                    f"{str(motion_error) + '; ' if motion_error else ''}"
                    f"{combined}")
            if self._motion_publisher_absence_unproven(reap_errors):
                zero_error = HarnessError(
                    "cleanup-zero publisher blocked because motion "
                    "publisher absence is unproven")
                self.store.event(
                    "cleanup_zero_blocked_unproven_motion_absence",
                    errors=reap_errors)
            else:
                try:
                    self._zero_and_verify(state)
                except BaseException as error:
                    zero_error = error
            reap_errors = self._reap_tracked_publishers(
                state, "motion_after_zero")
            if reap_errors:
                combined = "; ".join(reap_errors)
                zero_error = HarnessError(
                    f"{str(zero_error) + '; ' if zero_error else ''}"
                    f"{combined}")
        if motion_error is None and zero_error is None:
            try:
                self._collect_motion_delivery_evidence(
                    state, args.angular_z, args.duration, args.rate_hz,
                    int(args.duration * args.rate_hz))
            except BaseException as error:
                motion_error = error
                if is_interruption(error):
                    interrupted = error
                self.store.event(
                    "motion_delivery_failed", error=str(error))
        if motion_error or zero_error:
            cleanup_errors = self._stop_all(state, allow_missing=False)
            cleanup_errors.extend(self._preserve_partial_bags(state))
            state["status"] = "invalid"
            failures = [
                str(item) or item.__class__.__name__
                for item in (motion_error, zero_error) if item]
            failures.extend(cleanup_errors)
            state["failure"] = "; ".join(failures)
            try:
                self.store.save(state)
            except OSError as error:
                failures.append(f"state save failed: {error}")
            self.store.event("motion_stage_failed", errors=failures)
            if interrupted is not None or self.pending_interrupt is not None:
                raise interrupted or self.pending_interrupt
            raise HarnessError(state["failure"])
        try:
            state["status"] = "motion_completing"
            state["motion"] = {
                "angular_z": args.angular_z, "linear_x": args.linear_x,
                "duration": args.duration, "rate_hz": args.rate_hz,
                "requested_publish_count": int(
                    args.duration * args.rate_hz),
            }
            state["motion_completed_at"] = utc_now()
            self.store.save(state)
            if not self.store.event("motion_stage_completed"):
                raise HarnessError("cannot record motion-stage completion")
            state["status"] = "motion_complete"
            self.store.save(state)
        except BaseException as error:
            terminal_errors = [str(error) or error.__class__.__name__]
            if is_interruption(error):
                self.pending_interrupt = error
            try:
                self._zero_and_verify(state)
            except BaseException as zero_error:
                terminal_errors.append(
                    str(zero_error) or zero_error.__class__.__name__)
            terminal_errors.extend(self._stop_all(state, allow_missing=False))
            terminal_errors.extend(self._preserve_partial_bags(state))
            state["status"] = "invalid"
            state["failure"] = "; ".join(terminal_errors)
            try:
                self.store.save(state)
            except BaseException as save_error:
                terminal_errors.append(
                    f"state save failed: {save_error or save_error.__class__.__name__}")
            self.store.event(
                "motion_terminal_transition_failed", errors=terminal_errors)
            if self.pending_interrupt is not None:
                raise self.pending_interrupt
            raise HarnessError(state["failure"]) from error

    def _stop_all(self, state, allow_missing):
        errors = []
        for spec in reversed(list(state["recorders"].values())):
            try:
                outcome = self._stop_recorder(spec, allow_missing=allow_missing)
                if outcome == "never_started" and not allow_missing:
                    raise HarnessError(
                        f"missing {spec['kind']} recorder identity")
                event = {
                    "reaped": "recorder_stopped",
                    "already_missing": "recorder_already_missing",
                    "never_started": "recorder_never_started",
                    "launch_attempt_reaped":
                        "recorder_launch_attempt_reaped",
                }[outcome]
                recorded = self.store.event(
                    event, kind=spec["kind"], outcome=outcome,
                    pid=spec.get("pid"), pgid=spec.get("pgid"),
                    status=spec.get("status"),
                    child_exit_status=spec.get("child_exit_status"),
                    cleanup_evidence_path=spec.get(
                        "cleanup_evidence_path"),
                    cleanup_evidence_sha256=spec.get(
                        "cleanup_evidence_sha256"),
                    launch_cleanup_evidence_path=spec.get(
                        "launch_cleanup_evidence_path"),
                    launch_cleanup_evidence_sha256=spec.get(
                        "launch_cleanup_evidence_sha256"))
                if not recorded:
                    errors.append(
                        f"cannot record {spec['kind']} recorder stop outcome")
                self.store.save(state)
            except BaseException as error:
                if is_interruption(error):
                    self.pending_interrupt = error
                errors.append(str(error) or error.__class__.__name__)
                self.store.event(
                    "recorder_stop_failed", kind=spec["kind"], error=str(error))
        return errors

    def _verify_and_copy_bag(self, spec):
        self._verify_container(spec)
        body = f"ros2 bag info {shlex.quote(spec['bag_path'])}"
        result = run_checked(
            self.runner,
            docker_exec(spec["container"], ros_shell(spec["setup"], body)),
            10, f"verify {spec['kind']} bag")
        for topic in FINAL_REQUIRED_TOPICS[spec["kind"]]:
            count = bag_topic_count(result.stdout, topic)
            if count is None:
                raise HarnessError(
                    f"{spec['kind']} bag info lacks required topic {topic}")
            if count == 0:
                raise HarnessError(
                    f"{spec['kind']} bag has zero messages for {topic}")
        info_path = self.store.directory / f"{spec['kind']}-bag-info.txt"
        info_path.write_text(result.stdout, encoding="utf-8")
        destination = self.store.directory / f"{spec['kind']}-bag"
        if destination.exists():
            raise HarnessError(f"refusing to overwrite copied bag: {destination}")
        self._verify_container(spec)
        run_checked(
            self.runner,
            ["docker", "cp", f"{spec['container']}:{spec['bag_path']}",
             str(destination)],
            30, f"copy {spec['kind']} bag")
        if not destination.is_dir() or not (destination / "metadata.yaml").is_file():
            raise HarnessError(f"copied {spec['kind']} bag is incomplete")

    def _preserve_partial_bags(self, state):
        errors = []
        for spec in state["recorders"].values():
            identity_status = recorder_identity_status(spec)
            if identity_status == "never_started":
                self.store.event(
                    "partial_bag_not_started_skipped", kind=spec["kind"],
                    validity="not_started")
                continue
            if identity_status == "launch_pending":
                message = (
                    f"{spec['kind']} recorder launch cleanup is unproven")
                errors.append(message)
                self.store.event(
                    "partial_bag_launch_cleanup_unproven",
                    kind=spec["kind"], error=message,
                    validity="invalid_partial")
                continue
            if identity_status == "launch_reaped":
                try:
                    launch_evidence = (
                        self._validate_pending_recorder_cleanup(spec))
                except HarnessError as error:
                    errors.append(str(error))
                    self.store.event(
                        "partial_bag_launch_cleanup_invalid",
                        kind=spec["kind"], error=str(error),
                        validity="invalid_partial")
                    continue
                if (
                        launch_evidence["receipt_status"] ==
                        "absent_before_side_effects"):
                    self.store.event(
                        "partial_bag_launch_not_started_skipped",
                        kind=spec["kind"], validity="not_started")
                    continue
            if identity_status == "incomplete":
                message = f"incomplete {spec['kind']} recorder identity"
                errors.append(message)
                self.store.event(
                    "partial_bag_identity_incomplete", kind=spec["kind"],
                    error=message, validity="invalid_partial")
                continue
            destination = self.store.directory / f"partial-{spec['kind']}-bag"
            if destination.is_dir() and (
                    destination / "metadata.yaml").is_file():
                self.store.event(
                    "partial_bag_already_preserved", kind=spec["kind"],
                    path=str(destination), validity="invalid_partial")
                continue
            verified = False
            try:
                self._verify_container(spec)
                result = run_checked(
                    self.runner,
                    docker_exec(
                        spec["container"],
                        ros_shell(
                            spec["setup"],
                            f"ros2 bag info {shlex.quote(spec['bag_path'])}")),
                    10, f"inspect partial {spec['kind']} bag")
                info_path = self.store.directory / f"partial-{spec['kind']}-bag-info.txt"
                if not info_path.exists():
                    info_path.write_text(result.stdout, encoding="utf-8")
                verified = True
            except BaseException as error:
                message = str(error) or error.__class__.__name__
                errors.append(f"partial {spec['kind']} verification failed: {message}")
                self.store.event(
                    "partial_bag_verification_failed", kind=spec["kind"],
                    error=message, validity="invalid_partial")
            try:
                self._verify_container(spec)
                if destination.exists():
                    raise HarnessError(f"refusing to overwrite {destination}")
                run_checked(
                    self.runner,
                    ["docker", "cp", f"{spec['container']}:{spec['bag_path']}",
                     str(destination)],
                    30, f"copy partial {spec['kind']} bag")
                if not destination.is_dir() or not (
                        destination / "metadata.yaml").is_file():
                    raise HarnessError(
                        f"copied partial {spec['kind']} bag is incomplete")
                self.store.event(
                    "partial_bag_preserved", kind=spec["kind"],
                    path=str(destination), validity="invalid_partial",
                    relaxed_verification_passed=verified)
            except BaseException as error:
                message = str(error) or error.__class__.__name__
                errors.append(f"partial {spec['kind']} preservation failed: {message}")
                self.store.event(
                    "partial_bag_preservation_failed", kind=spec["kind"],
                    error=message, validity="invalid_partial")
        return errors

    def finalize(self, args):
        state = self.store.load()
        if state["status"] != "motion_complete":
            raise HarnessError(
                f"finalize requires motion_complete state, got {state['status']}")
        state["status"] = "finalizing"
        errors = []
        interrupted = None
        try:
            self.store.save(state)
            if not self.store.event("finalize_started"):
                raise HarnessError("cannot record finalize start")
            pre_audit_path = state.get("kernel_audit", {}).get("path")
            post_audit_path = str(Path(args.kernel_audit_artifact).resolve())
            if post_audit_path == pre_audit_path:
                raise HarnessError("post-motion audit must be a distinct fresh artifact")
            audit_sha = validate_kernel_audit(
                post_audit_path, state["motion_completed_at"])
            state["post_motion_kernel_audit"] = {
                "path": post_audit_path, "sha256": audit_sha,
                "validated_at": utc_now(),
            }
            self.store.save(state)
            if not self.store.event(
                    "post_motion_kernel_audit_validated", sha256=audit_sha):
                raise HarnessError("cannot record post-motion audit evidence")
        except BaseException as error:
            errors.append(str(error) or error.__class__.__name__)
            if is_interruption(error):
                interrupted = error
        pre_zero_reap_errors = self._reap_tracked_publishers(
            state, "finalize_before_zero")
        errors.extend(pre_zero_reap_errors)
        if self._motion_publisher_absence_unproven(
                pre_zero_reap_errors):
            message = (
                "finalize zero publisher blocked because motion "
                "publisher absence is unproven")
            errors.append(message)
            self.store.event(
                "finalize_zero_blocked_unproven_motion_absence",
                errors=pre_zero_reap_errors)
        else:
            try:
                self._zero_and_verify(state)
            except BaseException as error:
                errors.append(str(error) or error.__class__.__name__)
                if is_interruption(error):
                    interrupted = error
        errors.extend(self._reap_tracked_publishers(
            state, "finalize_after_zero"))
        errors.extend(self._stop_all(state, allow_missing=False))
        if not errors:
            try:
                self._validate_terminal_recorder_cleanup(state)
            except BaseException as error:
                errors.append(str(error) or error.__class__.__name__)
        if not errors:
            try:
                motion = state["motion"]
                self._collect_motion_delivery_evidence(
                    state,
                    motion["angular_z"],
                    motion["duration"],
                    motion["rate_hz"],
                    motion["requested_publish_count"],
                    artifact_name=FINAL_MOTION_DELIVERY_EVIDENCE_NAME,
                    state_key="final_motion_delivery",
                    require_live_recorder=False)
            except BaseException as error:
                errors.append(str(error) or error.__class__.__name__)
                if is_interruption(error):
                    interrupted = error
        if not errors:
            for spec in state["recorders"].values():
                try:
                    self._verify_and_copy_bag(spec)
                except BaseException as error:
                    errors.append(str(error) or error.__class__.__name__)
                    if is_interruption(error):
                        interrupted = error
        if errors:
            errors.extend(self._preserve_partial_bags(state))
            state["status"] = "invalid"
            state["failure"] = "; ".join(errors)
            try:
                self.store.save(state)
            except OSError as save_error:
                errors.append(f"state save failed: {save_error}")
            self.store.event("finalize_failed", errors=errors)
            if interrupted is not None or self.pending_interrupt is not None:
                raise interrupted or self.pending_interrupt
            raise HarnessError(state["failure"])
        state["status"] = "complete"
        try:
            self.store.save(state)
            if not self.store.event("finalize_completed"):
                raise HarnessError("cannot record finalize completion")
        except BaseException as error:
            errors = [str(error) or error.__class__.__name__]
            errors.extend(self._preserve_partial_bags(state))
            state["status"] = "invalid"
            state["failure"] = "; ".join(errors)
            try:
                self.store.save(state)
            except BaseException:
                pass
            self.store.event("finalize_failed", errors=errors)
            if is_interruption(error):
                raise
            raise HarnessError(state["failure"]) from error

    def abort(self):
        state = self.store.load()
        if state["status"] == "complete":
            raise HarnessError("refusing to invalidate a completed trial")
        if state["status"] == "aborted":
            self._validate_terminal_recorder_cleanup(state)
            return
        self.store.event("abort_started", previous_status=state["status"])
        errors = []
        pre_zero_reap_errors = self._reap_tracked_publishers(
            state, "abort_before_zero")
        errors.extend(pre_zero_reap_errors)
        if self._motion_publisher_absence_unproven(
                pre_zero_reap_errors):
            errors.append(
                "abort zero publisher blocked because motion publisher "
                "absence is unproven")
            self.store.event(
                "abort_zero_blocked_unproven_motion_absence",
                errors=pre_zero_reap_errors)
        else:
            try:
                self._zero_and_verify(state)
            except BaseException as error:
                errors.append(str(error) or error.__class__.__name__)
        errors.extend(self._reap_tracked_publishers(
            state, "abort_after_zero"))
        recorder_expected = state["status"] in {
            "prepared", "motion_in_progress", "motion_complete", "finalizing"}
        errors.extend(self._stop_all(state, allow_missing=not recorder_expected))
        errors.extend(self._preserve_partial_bags(state))
        if not errors:
            try:
                self._validate_terminal_recorder_cleanup(state)
            except BaseException as error:
                errors.append(str(error) or error.__class__.__name__)
        state["status"] = "aborted" if not errors else "invalid"
        if errors:
            state["failure"] = "; ".join(errors)
        try:
            self.store.save(state)
        except BaseException as error:
            if is_interruption(error):
                self.pending_interrupt = error
            errors.append(f"state save failed: {error or error.__class__.__name__}")
            state["failure"] = "; ".join(errors)
        self.store.event("abort_completed", cleanup_errors=errors)
        if self.pending_interrupt is not None:
            raise self.pending_interrupt
        if errors:
            raise HarnessError(state["failure"])


def add_common_prepare_args(parser):
    parser.add_argument("--trial-id", required=True)
    parser.add_argument("--robot-container", required=True)
    parser.add_argument("--imu-container", required=True)
    parser.add_argument("--robot-bag", required=True)
    parser.add_argument("--imu-bag", required=True)
    parser.add_argument("--robot-log", required=True)
    parser.add_argument("--imu-log", required=True)
    parser.add_argument(
        "--robot-setup", action="append", default=["/opt/ros/humble/setup.bash"])
    parser.add_argument(
        "--imu-setup", action="append", default=["/opt/ros/humble/setup.bash"])


def make_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-dir", required=True)
    subparsers = parser.add_subparsers(dest="stage", required=True)
    add_common_prepare_args(subparsers.add_parser("prepare"))
    motion = subparsers.add_parser(
        "motion",
        help="deprecated and blocked: nonzero rotation trials are frozen",
    )
    motion.add_argument("--linear-x", type=float, default=0.0)
    motion.add_argument("--angular-z", type=float, required=True)
    motion.add_argument("--duration", type=float, required=True)
    motion.add_argument("--rate-hz", type=int, default=20)
    motion.add_argument("--acknowledge-motion", required=True)
    motion.add_argument("--kernel-audit-artifact", required=True)
    finalize = subparsers.add_parser("finalize")
    finalize.add_argument("--kernel-audit-artifact", required=True)
    subparsers.add_parser("abort")
    subparsers.add_parser("status")
    return parser


def main(argv=None, runner=None, *, allow_frozen_motion_for_tests=False):
    args = make_parser().parse_args(argv)
    previous_handlers = {}
    try:
        if args.stage == "motion" and not allow_frozen_motion_for_tests:
            raise HarnessError(FROZEN_MOTION_MESSAGE)
        if args.stage != "status":
            previous_handlers = install_termination_handlers()
        with StateStore(args.evidence_dir, read_only=args.stage == "status") as store:
            if args.stage == "status":
                state = store.load()
                if state["status"] in {"aborted", "complete"}:
                    RotationHarness(runner or SubprocessRunner(), store)._validate_terminal_recorder_cleanup(
                        state)
                print(json.dumps(state, sort_keys=True, indent=2))
                return 0
            harness = RotationHarness(runner or SubprocessRunner(), store)
            if args.stage == "prepare":
                harness.prepare(args)
            elif args.stage == "motion":
                harness.motion(args)
            elif args.stage == "finalize":
                harness.finalize(args)
            elif args.stage == "abort":
                harness.abort()
        print(f"{args.stage} completed", flush=True)
        return 0
    except KeyboardInterrupt:
        print(
            f"{args.stage} interrupted after best-effort safety cleanup",
            file=sys.stderr, flush=True)
        return 130
    except HarnessTermination as error:
        print(
            f"{args.stage} terminated by signal {error.signum} after "
            "best-effort safety cleanup",
            file=sys.stderr, flush=True)
        return 128 + error.signum
    except (HarnessError, OSError) as error:
        print(f"{args.stage} failed: {error}", file=sys.stderr, flush=True)
        return 1
    finally:
        restore_signal_handlers(previous_handlers)


if __name__ == "__main__":
    sys.exit(main())
