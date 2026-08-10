#!/usr/bin/env python3
"""Hardware-free tests for d455_rotation_validation.py."""

import argparse
import contextlib
import datetime
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import re
import shlex
import signal
import sqlite3
import struct
import subprocess
import tempfile
from types import SimpleNamespace
import unittest


MODULE_PATH = Path(__file__).with_name("d455_rotation_validation.py")
SPEC = importlib.util.spec_from_file_location("d455_rotation_validation", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
ROBOT_ID = "a" * 64
IMU_ID = "b" * 64


def publishes_twist(command, angular_z):
    return (
        "d455_twist_publisher.py" in command and
        MODULE.twist_yaml(angular_z) in command)


def is_recorder_identity_probe(command):
    return (
        "ROTATION_RECORDER_RECEIPT" in command and
        "-name '*.db3'" in command)


class FakeRunner:
    def __init__(self, callback):
        self.callback = callback
        self.calls = []
        self.publisher_required_endpoints = {}

    def run(self, argv, timeout):
        self.calls.append((argv, timeout))
        text = " ".join(argv)
        if "ROTATION_TWIST_PUBLISHER_ID" in text:
            path_match = re.search(
                r"--result (/tmp/[A-Za-z0-9_.-]+\.json)", text)
            if path_match:
                self.publisher_required_endpoints[path_match.group(1)] = (
                    re.findall(
                        r"--required-endpoint (/[A-Za-z0-9_:/.-]+)",
                        text))
        result = self.callback(argv, timeout)
        for path, endpoints in self.publisher_required_endpoints.items():
            if "cat " not in text or path not in text or result.returncode != 0:
                continue
            try:
                evidence = json.loads(result.stdout)
            except json.JSONDecodeError:
                continue
            evidence["required_subscription_endpoints"] = list(endpoints)
            evidence["matched_subscription_endpoints"] = sorted(endpoints)
            evidence["subscription_qos_assessments"] = [
                MODULE.expected_endpoint_assessment(
                    endpoint,
                    next(
                        detail["qos"]
                        for detail in evidence["subscription_details"]
                        if detail["endpoint"] == endpoint))
                for endpoint in endpoints
            ]
            if "/:rosbag2_recorder" not in endpoints:
                evidence["recorder_qos_override"] = {
                    "required": False,
                    "path": None,
                    "expected_sha256": None,
                    "actual_sha256": None,
                    "verified": False,
                }
            return MODULE.CommandResult(
                result.returncode,
                json.dumps(evidence, sort_keys=True) + "\n",
                result.stderr)
        return result


def base_state(status="prepared"):
    created = (
        datetime.datetime.now(datetime.timezone.utc) - datetime.timedelta(seconds=1)
    ).isoformat()
    return {
        "schema_version": 1,
        "trial_id": "01-cw-short",
        "status": status,
        "created_at": created,
        "updated_at": created,
        "recorders": {
            "robot": {
                "kind": "robot", "container": "robot", "container_id": ROBOT_ID,
                "pid": 101, "pgid": 101, "sid": 101, "starttime": 1001,
                "cmdline_hex": "726f626f74",
                "token": "d455-recorder-robot",
                "exit_path": "/tmp/d455-recorder-robot.exit",
                "launch_mode": "detached_foreground_docker_exec",
                "wrapper_reap_owner": "docker_exec_parent",
                "setup": ["/opt/ros/humble/setup.bash"],
                "bag_path": "/tmp/robot-bag", "log_path": "/tmp/robot.log",
                "topics": list(MODULE.ROBOT_TOPICS),
                "cmd_vel_test_qos": dict(MODULE.CMD_VEL_TEST_QOS),
                "qos_override_path":
                    MODULE.ROSBAG_QOS_OVERRIDE_CONTAINER_PATH,
                "qos_override_sha256":
                    MODULE.ROSBAG_QOS_OVERRIDE_SHA256,
            },
            "imu": {
                "kind": "imu", "container": "imu", "container_id": IMU_ID,
                "pid": 202, "pgid": 202, "sid": 202, "starttime": 2002,
                "cmdline_hex": "696d75",
                "token": "d455-recorder-imu",
                "exit_path": "/tmp/d455-recorder-imu.exit",
                "launch_mode": "detached_foreground_docker_exec",
                "wrapper_reap_owner": "docker_exec_parent",
                "setup": ["/opt/ros/humble/setup.bash", "/tmp/imu/setup.bash"],
                "bag_path": "/tmp/imu-bag", "log_path": "/tmp/imu.log",
                "topics": list(MODULE.IMU_TOPICS),
            },
        },
    }


def clean_audit(directory, name="kernel-audit.txt"):
    path = Path(directory) / name
    path.write_text(
        "apparmor_denials=0\nd455_usb_reset_or_disconnect=0\n",
        encoding="utf-8")
    return str(path)


def motion_args(audit_path):
    return argparse.Namespace(
        angular_z=-0.15, duration=2.0, rate_hz=20, linear_x=0.0,
        acknowledge_motion=MODULE.MOTION_ACK, kernel_audit_artifact=audit_path)


def publisher_evidence(
        count=40, duration=2.0, rate_hz=20, actual_count=None,
        window_duration=None, publish_span=None, angular_z=-0.15,
        status="complete", command_type="motion"):
    actual_count = count if actual_count is None else actual_count
    period = 1.0 / rate_hz
    window_duration = duration if window_duration is None else window_duration
    expected_span = (count - 1) / rate_hz
    publish_span = expected_span if publish_span is None else publish_span
    origin = 1000000000
    first = origin + int(period * 1000000000)
    if actual_count >= 2:
        timestamps = [
            first + round(
                index * publish_span * 1000000000 / (actual_count - 1))
            for index in range(actual_count)
        ]
    else:
        timestamps = [first] if actual_count == 1 else []
    last = timestamps[-1] if timestamps else origin
    end = origin + int(window_duration * 1000000000)
    intervals = [
        (following - current) / 1000000000.0
        for current, following in zip(timestamps, timestamps[1:])
    ]
    utc = "2026-07-23T10:00:00.000000+00:00"
    return {
        "schema_version": 1,
        "status": status,
        "error": None,
        "command_type": command_type,
        "publisher_qos": dict(MODULE.CMD_VEL_TEST_QOS),
        "child_exit_status": 0,
        "stdout_stderr_path": "/tmp/publisher.log",
        "json_result_path": "/tmp/publisher.json",
        "requested_publish_count": count,
        "actual_publish_count": actual_count,
        "requested_duration_s": duration,
        "requested_rate_hz": rate_hz,
        "published_twist": json.loads(MODULE.twist_yaml(angular_z)),
        "required_subscription_endpoints": [
            "/:command_arbiter", "/:rosbag2_recorder",
        ],
        "matched_subscriptions": 2,
        "matched_subscription_endpoints": [
            "/:command_arbiter", "/:rosbag2_recorder",
        ],
        "subscription_details": [
            {
                "endpoint": endpoint,
                "qos": dict(MODULE.CMD_VEL_TEST_QOS),
            }
            for endpoint in (
                "/:command_arbiter", "/:rosbag2_recorder")
        ],
        "subscription_qos_assessments": [
            MODULE.expected_endpoint_assessment(
                endpoint, dict(MODULE.CMD_VEL_TEST_QOS))
            for endpoint in (
                "/:command_arbiter", "/:rosbag2_recorder")
        ],
        "recorder_qos_override": {
            "required": True,
            "path": MODULE.ROSBAG_QOS_OVERRIDE_CONTAINER_PATH,
            "expected_sha256": MODULE.ROSBAG_QOS_OVERRIDE_SHA256,
            "actual_sha256": MODULE.ROSBAG_QOS_OVERRIDE_SHA256,
            "verified": True,
        },
        "subscriber_ready_monotonic_ns": origin - 1,
        "window_start_monotonic_ns": origin,
        "first_publish_monotonic_ns": first,
        "last_publish_monotonic_ns": last,
        "window_end_monotonic_ns": max(end, last),
        "window_start_system_ns": origin,
        "first_publish_system_ns": first,
        "last_publish_system_ns": last,
        "window_end_system_ns": max(end, last),
        "window_start_utc": utc,
        "first_publish_utc": utc,
        "last_publish_utc": utc,
        "window_end_utc": utc,
        "publish_monotonic_ns": timestamps,
        "publish_system_ns": timestamps,
        "schedule_lateness_ns": [1000000] * actual_count,
        "timing": {
            "period_s": period,
            "window_duration_s": window_duration,
            "publish_span_s": publish_span,
            "mean_interval_s": (
                sum(intervals) / len(intervals) if intervals else 0.0),
            "min_interval_s": min(intervals) if intervals else 0.0,
            "max_interval_s": max(intervals) if intervals else 0.0,
            "max_schedule_lateness_s": 0.001,
        },
    }


def delivery_evidence(
        count=40, duration=2.0, rate_hz=20,
        test_count=None, test_span=None, safe_count=None, safe_span=None,
        angular_z=-0.15):
    expected_span = (count - 1) / rate_hz
    test_count = count if test_count is None else test_count
    test_span = expected_span if test_span is None else test_span
    safe_count = count + 2 if safe_count is None else safe_count
    safe_span = expected_span + 0.10 if safe_span is None else safe_span

    def topic(message_count, nonzero_count, span, start_offset=0.0):
        start = 1000000000 + int(start_offset * 1000000000)
        timestamps = [
            start + round(
                index * span * 1000000000 / (nonzero_count - 1))
            for index in range(nonzero_count)
        ]
        return {
            "message_count": message_count,
            "nonzero_count": nonzero_count,
            "matching_nonzero_count": nonzero_count,
            "mismatched_nonzero_count": 0,
            "internal_zero_count": 0,
            "nonzero_timestamps_ns": timestamps,
            "first_nonzero_timestamp_ns": start,
            "last_nonzero_timestamp_ns": timestamps[-1],
            "nonzero_span_s": span,
        }

    return {
        "schema_version": 1,
        "captured_at_utc": "2026-07-23T10:00:10.000000+00:00",
        "expected_angular_z": angular_z,
        "expected_publish_count": count,
        "expected_duration_s": duration,
        "expected_rate_hz": rate_hz,
        "database_files": ["robot_0.db3"],
        "database_errors": [],
        "topics": {
            "/cmd_vel/test": topic(100, test_count, test_span),
            "/cmd_vel/safe": topic(
                100, safe_count, safe_span, start_offset=0.02),
        },
        "safe_start_offset_s": 0.02,
        "safe_end_offset_s": 0.12,
    }


def marker_result(marker, evidence):
    return MODULE.CommandResult(
        0, marker + " " + json.dumps(evidence, sort_keys=True) + "\n")


def prepare_args():
    return argparse.Namespace(
        trial_id="trial", robot_container="robot", imu_container="imu",
        robot_bag="/tmp/rbag", imu_bag="/tmp/ibag",
        robot_log="/tmp/r.log", imu_log="/tmp/i.log",
        robot_setup=["/opt/ros/humble/setup.bash"],
        imu_setup=["/opt/ros/humble/setup.bash"])


def finalize_state(directory):
    state = base_state(status="motion_complete")
    completed = (
        datetime.datetime.now(datetime.timezone.utc) - datetime.timedelta(seconds=1)
    ).isoformat()
    pre_path = clean_audit(directory, "pre-motion-audit.txt")
    state["motion_completed_at"] = completed
    state["motion"] = {
        "angular_z": -0.15, "linear_x": 0.0, "duration": 2.0,
        "rate_hz": 20, "requested_publish_count": 40,
    }
    state["kernel_audit"] = {"path": str(Path(pre_path).resolve()), "sha256": "pre"}
    return state


def successful_callback(argv, _timeout):
    text = " ".join(argv)
    if argv[:3] == ["docker", "inspect", "-f"]:
        identity = ROBOT_ID if argv[-1] == "robot" else IMU_ID
        return MODULE.CommandResult(0, f"{identity} true\n")
    if "D455_PUBLISHER_ATTEMPT_PHASE" in text:
        return MODULE.CommandResult(
            0,
            "D455_PUBLISHER_ATTEMPT_PHASE REAPED ABSENT\n"
            "D455_PUBLISHER_CHILD_BREADCRUMB 404 404\n"
            "D455_PUBLISHER_PARENT_BREADCRUMB 404 404\n"
            "D455_PUBLISHER_RECEIPT "
            "404 404 404 4004 7075626c6973686572\n"
            "D455_PUBLISHER_WAIT_STATUS 0\n")
    if "ROTATION_PREPARE_TOPIC_COUNT" in text:
        return MODULE.CommandResult(0, "ROTATION_PREPARE_TOPIC_COUNT 20\n")
    if "ROTATION_TWIST_PUBLISHER_ID" in text:
        return MODULE.CommandResult(
            0, "ROTATION_TWIST_PUBLISHER_ID "
            "404 404 404 4004 7075626c6973686572\n"
            "ROTATION_TWIST_PUBLISHER_REAPED 404 0\n")
    if "TWIST_PUBLISHER_REAP_VERIFIED" in text:
        return MODULE.CommandResult(
            0, "TWIST_PUBLISHER_REAP_VERIFIED\n")
    if "TWIST_PUBLISHER_STOPPED" in text:
        return MODULE.CommandResult(0, "TWIST_PUBLISHER_STOPPED\n")
    if "TWIST_NEVER_RELEASED_REAP_VERIFIED" in text:
        return MODULE.CommandResult(
            0, "TWIST_NEVER_RELEASED_REAP_VERIFIED\n")
    if (
            "D455_PUBLISHER_FILE_READ" in text and
            "/tmp/d455-" in text and "-publisher-" in text):
        if ".exit" in text:
            return MODULE.CommandResult(0, "0\n")
        if ".log" in text:
            return MODULE.CommandResult(0, "publisher log\n")
        if "prepare-zero-publisher-" in text:
            evidence = publisher_evidence(
                count=20, duration=1.0, angular_z=0.0,
                command_type="prepare_zero")
        elif "cleanup-zero-publisher-" in text:
            evidence = publisher_evidence(
                count=20, duration=1.0, angular_z=0.0,
                command_type="cleanup_zero")
        else:
            evidence = publisher_evidence()
        for field in (
                "child_exit_status", "stdout_stderr_path",
                "json_result_path"):
            evidence.pop(field)
        return MODULE.CommandResult(
            0, json.dumps(evidence, sort_keys=True) + "\n")
    if MODULE.MOTION_DELIVERY_MARKER in text:
        return marker_result(
            MODULE.MOTION_DELIVERY_MARKER, delivery_evidence())
    if "D455_RECORDER_LOG_READ" in text:
        return MODULE.CommandResult(
            0, "D455_RECORDER_LOG_READ\nrecorder log\n")
    if "RECORDER_START_CANCEL_PERSISTED" in text:
        return MODULE.CommandResult(
            0, "RECORDER_START_CANCEL_PERSISTED " + "c" * 64 +
            " absent\n")
    if "RECORDER_RECEIPT_ABSENT" in text:
        identity = (
            "101 101 101 1001 726f626f74"
            if argv[2] == "robot"
            else "202 202 202 2002 696d75")
        return MODULE.CommandResult(
            0, f"ROTATION_RECORDER_RECEIPT {identity}\n")
    if "RECORDER_PENDING_REAPED" in text:
        receipt_sha256 = "c" * 64
        log_sha256 = hashlib.sha256(b"recorder log\n").hexdigest()
        return MODULE.CommandResult(
            0, "RECORDER_PENDING_REAPED "
            f"{receipt_sha256} 0 {log_sha256}\n")
    if "RECORDER_PENDING_QUIESCENT" in text:
        return MODULE.CommandResult(
            0, "RECORDER_PENDING_QUIESCENT\n")
    if argv[:2] == ["docker", "cp"]:
        destination = Path(argv[-1])
        destination.mkdir()
        (destination / "metadata.yaml").write_text("partial\n", encoding="utf-8")
        return MODULE.CommandResult(0, "")
    if "kill -INT" in text and "RECORDER_REAPED" in text:
        log_sha256 = hashlib.sha256(b"recorder log\n").hexdigest()
        return MODULE.CommandResult(
            0, f"RECORDER_REAPED 0 {log_sha256}\n")
    if "ros2 topic info --verbose" in text:
        return MODULE.CommandResult(
            0,
            "Node name: rosbag2_recorder\n"
            "Node namespace: /\n"
            "Endpoint type: SUBSCRIPTION\n"
            "Node name: command_arbiter\n"
            "Node namespace: /\n"
            "Endpoint type: SUBSCRIPTION\n")
    if "ros2 bag info" in text:
        if argv[2] == "robot":
            return MODULE.CommandResult(
                0,
                "Topic information: Topic: /cmd_vel/test | Type: "
                "geometry_msgs/msg/Twist | Count: 40 | Serialization Format: cdr\n"
                "                   Topic: /cmd_vel/safe | Type: "
                "geometry_msgs/msg/Twist | Count: 80 | Serialization Format: cdr\n"
                "                   Topic: /wheel_ticks | Type: "
                "roboteq_ros2_driver/msg/WheelTicks | Count: 50 | "
                "Serialization Format: cdr\n"
                "                   Topic: /odom | Type: nav_msgs/msg/Odometry | "
                "Count: 50 | Serialization Format: cdr\n")
        return MODULE.CommandResult(
            0,
            "Topic information: Topic: /camera/imu | Type: sensor_msgs/msg/Imu | "
            "Count: 1000 | Serialization Format: cdr\n")
    return MODULE.CommandResult(0, "")


class ShellSafetyTest(unittest.TestCase):
    def test_ros_setup_is_sourced_before_nounset_is_enabled(self):
        shell = MODULE.ros_shell(
            ["/opt/ros/humble/setup.bash", "/tmp/install/setup.bash"], "true")
        self.assertLess(shell.index("set +u"), shell.index("source /opt/ros"))
        self.assertLess(shell.index("source /tmp/install"), shell.index("set -u"))
        self.assertTrue(shell.startswith("set -eo pipefail\n"))

    def test_host_termination_handlers_raise_cleanup_aware_exception(self):
        previous = MODULE.install_termination_handlers()
        try:
            handler = signal.getsignal(signal.SIGTERM)
            with self.assertRaises(MODULE.HarnessTermination) as raised:
                handler(signal.SIGTERM, None)
            self.assertEqual(raised.exception.signum, signal.SIGTERM)
        finally:
            MODULE.restore_signal_handlers(previous)

    def test_motion_allowlist_accepts_only_exact_approved_values(self):
        expected_angular_z = {
            -0.675, -0.45, -0.30, -0.15,
            0.15, 0.30, 0.45, 0.675,
        }
        self.assertEqual(MODULE.ALLOWED_ANGULAR_Z, expected_angular_z)
        self.assertEqual(MODULE.ALLOWED_DURATIONS, {2.0, 5.0})
        self.assertEqual(MODULE.ALLOWED_RATE_HZ, {20})
        for angular_z in sorted(expected_angular_z):
            for duration in sorted(MODULE.ALLOWED_DURATIONS):
                with self.subTest(angular_z=angular_z, duration=duration):
                    MODULE.validate_motion(
                        angular_z, duration, 20, 0.0, MODULE.MOTION_ACK)

        adjacent_angular_z = {
            value + delta
            for value in expected_angular_z
            for delta in (-1e-12, 1e-12)
        }
        for angular_z in (
                *sorted(adjacent_angular_z), 0.0, 0.2,
                float("nan"), float("inf"), float("-inf")):
            with self.subTest(angular_z=angular_z), self.assertRaises(
                    MODULE.HarnessError):
                MODULE.validate_motion(
                    angular_z, 2.0, 20, 0.0, MODULE.MOTION_ACK)

        invalid = [
            (0.15, 3.0, 20, 0.0, MODULE.MOTION_ACK),
            (0.15, 2.0, 10, 0.0, MODULE.MOTION_ACK),
            (0.15, 2.0, 20, 0.01, MODULE.MOTION_ACK),
            (0.15, 2.0, 20, 0.0, "yes"),
        ]
        for values in invalid:
            with self.subTest(values=values), self.assertRaises(MODULE.HarnessError):
                MODULE.validate_motion(*values)

    def test_twist_payload_is_complete_deterministic_json_for_allowed_values(self):
        expected_keys = {"linear", "angular"}
        expected_zero_vector = {"x": 0.0, "y": 0.0, "z": 0.0}
        for angular_z in (0.0, *sorted(MODULE.ALLOWED_ANGULAR_Z)):
            with self.subTest(angular_z=angular_z):
                payload = MODULE.twist_yaml(angular_z)
                decoded = json.loads(payload)
                self.assertEqual(set(decoded), expected_keys)
                self.assertEqual(decoded["linear"], expected_zero_vector)
                self.assertEqual(
                    decoded["angular"],
                    {"x": 0.0, "y": 0.0, "z": angular_z})
                self.assertEqual(
                    payload,
                    json.dumps(
                        decoded, allow_nan=False, separators=(",", ":"),
                        sort_keys=True))
                self.assertEqual(shlex.split(shlex.quote(payload)), [payload])

    def test_twist_payload_rejects_nonfinite_or_nonnumeric_angular_z(self):
        for angular_z in (float("nan"), float("inf"), float("-inf"), True, "0"):
            with self.subTest(angular_z=angular_z), self.assertRaisesRegex(
                    MODULE.HarnessError, "finite number"):
                MODULE.twist_yaml(angular_z)

    def test_motion_publisher_is_subscriber_ready_and_monotonic_not_cli_based(self):
        source = MODULE.PUBLISHER_HOST_PATH.read_text()
        compile(source, str(MODULE.PUBLISHER_HOST_PATH), "exec")
        self.assertIn("publisher.get_subscription_count()", source)
        self.assertIn("get_subscriptions_info_by_topic", source)
        self.assertIn("matched_subscriptions", source)
        self.assertIn("time.monotonic_ns()", source)
        self.assertIn("actual_publish_count", source)
        self.assertIn("os.fsync", source)
        self.assertNotIn("ros2 topic pub", source)
        self.assertNotIn(
            "ros2 topic pub", MODULE_PATH.read_text())

    def test_bag_topic_parser_is_exact_and_returns_count(self):
        evidence = MODULE_PATH.parent.parent / "validation_evidence"
        smoke = evidence / "d455-roboteq-rotation-20260721T111538Z" / (
            "preflight/rosbag-smoke")
        robot_info = (smoke / "robot-bag-info.txt").read_text()
        imu_info = (smoke / "imu-bag-info.txt").read_text()
        self.assertEqual(MODULE.bag_topic_count(robot_info, "/wheel_ticks"), 51)
        self.assertEqual(MODULE.bag_topic_count(robot_info, "/cmd_vel/safe"), 83)
        self.assertEqual(MODULE.bag_topic_count(robot_info, "/cmd_vel/joy"), 0)
        self.assertIsNone(MODULE.bag_topic_count(robot_info, "/tf_missing"))
        self.assertEqual(MODULE.bag_topic_count(imu_info, "/camera/imu"), 1147)

    def test_recorder_subscription_parser_rejects_live_publisher_only(self):
        publisher_only = (
            "Node name: d455\nNode namespace: /realsense\n"
            "Endpoint type: PUBLISHER\n")
        recorder = (
            publisher_only + "\nNode name: rosbag2_recorder\n"
            "Node namespace: /\nEndpoint type: SUBSCRIPTION\n")
        self.assertFalse(
            MODULE.topic_info_has_recorder_subscription(publisher_only))
        self.assertTrue(MODULE.topic_info_has_recorder_subscription(recorder))
        self.assertFalse(
            MODULE.topic_info_has_subscription(recorder, "command_arbiter"))
        self.assertTrue(MODULE.topic_info_has_subscription(
            recorder + "\nNode name: command_arbiter\n"
            "Endpoint type: SUBSCRIPTION\n",
            "command_arbiter"))


class MotionEvidenceValidationTest(unittest.TestCase):
    def test_full_requested_nonzero_count_and_duration_are_accepted(self):
        MODULE.validate_publisher_evidence(
            publisher_evidence(), 40, 2.0, 20, -0.15)
        MODULE.validate_delivery_evidence(
            delivery_evidence(), 40, 2.0, 20, -0.15)

    def test_shortened_publication_is_rejected(self):
        shortened = publisher_evidence(
            actual_count=30, window_duration=1.5, publish_span=1.45)
        with self.assertRaisesRegex(MODULE.HarnessError, "count mismatch"):
            MODULE.validate_publisher_evidence(
                shortened, 40, 2.0, 20, -0.15)

    def test_cmd_vel_test_count_and_duration_mismatches_are_rejected(self):
        mismatches = (
            (delivery_evidence(test_count=39), "nonzero count mismatch"),
            (delivery_evidence(test_span=1.50), "nonzero duration mismatch"),
        )
        for evidence, message in mismatches:
            with self.subTest(message=message), self.assertRaisesRegex(
                    MODULE.HarnessError, message):
                MODULE.validate_delivery_evidence(
                    evidence, 40, 2.0, 20, -0.15)

    def test_cmd_vel_safe_forwarded_duration_mismatch_is_rejected(self):
        evidence = delivery_evidence(safe_count=20, safe_span=1.0)
        with self.assertRaisesRegex(
                MODULE.HarnessError, "count outside arbiter tolerance"):
            MODULE.validate_delivery_evidence(
                evidence, 40, 2.0, 20, -0.15)
        evidence = delivery_evidence(safe_span=2.50)
        with self.assertRaisesRegex(
                MODULE.HarnessError, "duration outside arbiter tolerance"):
            MODULE.validate_delivery_evidence(
                evidence, 40, 2.0, 20, -0.15)

    def test_publisher_requires_exact_twist_and_subscriber_readiness(self):
        wrong_twist = publisher_evidence()
        wrong_twist["published_twist"]["angular"]["z"] = 0.15
        with self.assertRaisesRegex(MODULE.HarnessError, "Twist"):
            MODULE.validate_publisher_evidence(
                wrong_twist, 40, 2.0, 20, -0.15)
        wrong_zero = publisher_evidence(
            count=20, duration=1.0, angular_z=0.0,
            command_type="prepare_zero")
        wrong_zero["published_twist"]["angular"]["z"] = 0.15
        with self.assertRaisesRegex(MODULE.HarnessError, "Twist"):
            MODULE.validate_publisher_evidence(
                wrong_zero, 20, 1.0, 20, 0.0,
                command_type="prepare_zero")
        not_ready = publisher_evidence()
        not_ready["matched_subscription_endpoints"].remove(
            "/:rosbag2_recorder")
        not_ready["subscription_details"] = [
            detail for detail in not_ready["subscription_details"]
            if detail["endpoint"] != "/:rosbag2_recorder"
        ]
        with self.assertRaisesRegex(
                MODULE.HarnessError, "endpoints ready"):
            MODULE.validate_publisher_evidence(
                not_ready, 40, 2.0, 20, -0.15)

    def test_publisher_requires_unambiguous_zero_exit_status(self):
        missing = publisher_evidence()
        del missing["child_exit_status"]
        with self.assertRaisesRegex(
                MODULE.HarnessError, "child_exit_status"):
            MODULE.validate_publisher_evidence(
                missing, 40, 2.0, 20, -0.15)

        failed = publisher_evidence()
        failed["child_exit_status"] = 124
        with self.assertRaisesRegex(
                MODULE.HarnessError, "exited unsuccessfully"):
            MODULE.validate_publisher_evidence(
                failed, 40, 2.0, 20, -0.15)

    def test_publisher_accepts_graph_count_undercount_with_exact_endpoints(self):
        evidence = publisher_evidence()
        evidence["matched_subscriptions"] = 1
        accepted = MODULE.validate_publisher_evidence(
            evidence, 40, 2.0, 20, -0.15)
        self.assertEqual(accepted["matched_subscriptions"], 1)

    def test_publisher_accepts_unreported_history_depth_with_pinned_override(self):
        evidence = publisher_evidence()
        for detail in evidence["subscription_details"]:
            detail["qos"]["history"] = "unknown"
            detail["qos"]["depth"] = 0
        evidence["subscription_qos_assessments"] = [
            MODULE.expected_endpoint_assessment(
                detail["endpoint"], detail["qos"])
            for detail in evidence["subscription_details"]
        ]
        accepted = MODULE.validate_publisher_evidence(
            evidence, 40, 2.0, 20, -0.15)
        self.assertEqual(
            {
                item["endpoint"]: item["tolerated_unreported_fields"]
                for item in accepted["subscription_qos_assessments"]
            },
            {
                "/:command_arbiter": ["depth", "history"],
                "/:rosbag2_recorder": ["depth", "history"],
            })

    def test_publisher_rejects_endpoint_present_but_qos_incompatible(self):
        for field, value in (
                ("reliability", "best_effort"),
                ("durability", "transient_local")):
            with self.subTest(field=field):
                evidence = publisher_evidence()
                evidence["subscription_details"][1]["qos"][field] = value
                with self.assertRaisesRegex(MODULE.HarnessError, "QoS mismatch"):
                    MODULE.validate_publisher_evidence(
                        evidence, 40, 2.0, 20, -0.15)

    def test_publisher_rejects_missing_arbiter_or_recorder_endpoint(self):
        for endpoint in ("/:command_arbiter", "/:rosbag2_recorder"):
            with self.subTest(endpoint=endpoint):
                evidence = publisher_evidence()
                evidence["matched_subscription_endpoints"].remove(endpoint)
                evidence["subscription_details"] = [
                    detail for detail in evidence["subscription_details"]
                    if detail["endpoint"] != endpoint
                ]
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "endpoints ready"):
                    MODULE.validate_publisher_evidence(
                        evidence, 40, 2.0, 20, -0.15)

    def test_publisher_rejects_unpinned_recorder_override(self):
        evidence = publisher_evidence()
        evidence["recorder_qos_override"]["verified"] = False
        with self.assertRaisesRegex(MODULE.HarnessError, "not pinned"):
            MODULE.validate_publisher_evidence(
                evidence, 40, 2.0, 20, -0.15)

    def test_publisher_rejects_omitted_or_truncated_qos_assessments(self):
        omitted = publisher_evidence()
        omitted["required_subscription_endpoints"] = []
        omitted["subscription_qos_assessments"] = []
        with self.assertRaisesRegex(
                MODULE.HarnessError, "endpoints do not match intent"):
            MODULE.validate_publisher_evidence(
                omitted, 40, 2.0, 20, -0.15)

        truncated = publisher_evidence()
        truncated["subscription_qos_assessments"].pop()
        with self.assertRaisesRegex(
                MODULE.HarnessError, "assessment evidence is inconsistent"):
            MODULE.validate_publisher_evidence(
                truncated, 40, 2.0, 20, -0.15)

        extra = publisher_evidence()
        extra["subscription_qos_assessments"].append(
            dict(extra["subscription_qos_assessments"][0]))
        with self.assertRaisesRegex(
                MODULE.HarnessError, "assessment evidence is inconsistent"):
            MODULE.validate_publisher_evidence(
                extra, 40, 2.0, 20, -0.15)

        duplicate_endpoint = publisher_evidence()
        duplicate_endpoint["required_subscription_endpoints"].append(
            "/:rosbag2_recorder")
        with self.assertRaisesRegex(
                MODULE.HarnessError, "endpoints do not match intent"):
            MODULE.validate_publisher_evidence(
                duplicate_endpoint, 40, 2.0, 20, -0.15)

    def test_publisher_rejects_ambiguous_duplicate_endpoint_identity(self):
        evidence = publisher_evidence()
        duplicate = {
            "endpoint": "/:rosbag2_recorder",
            "qos": dict(MODULE.CMD_VEL_TEST_QOS),
        }
        evidence["subscription_details"].append(duplicate)
        with self.assertRaisesRegex(MODULE.HarnessError, "ambiguous"):
            MODULE.validate_publisher_evidence(
                evidence, 40, 2.0, 20, -0.15)

    def test_publisher_rejects_internal_timing_gap_and_excess_lateness(self):
        gap = publisher_evidence()
        gap["publish_monotonic_ns"][20] += 30000000
        gap["timing"]["min_interval_s"] = 0.02
        gap["timing"]["max_interval_s"] = 0.08
        with self.assertRaisesRegex(MODULE.HarnessError, "interval outside"):
            MODULE.validate_publisher_evidence(
                gap, 40, 2.0, 20, -0.15)
        late = publisher_evidence()
        late["schedule_lateness_ns"][20] = 60000000
        late["timing"]["max_schedule_lateness_s"] = 0.06
        with self.assertRaisesRegex(MODULE.HarnessError, "lateness"):
            MODULE.validate_publisher_evidence(
                late, 40, 2.0, 20, -0.15)

    def test_delivery_rejects_internal_zero_and_interarrival_gap(self):
        internal_zero = delivery_evidence()
        internal_zero["topics"]["/cmd_vel/safe"]["internal_zero_count"] = 1
        with self.assertRaisesRegex(MODULE.HarnessError, "internal zero"):
            MODULE.validate_delivery_evidence(
                internal_zero, 40, 2.0, 20, -0.15)
        gap = delivery_evidence()
        gap["topics"]["/cmd_vel/test"]["nonzero_timestamps_ns"][20] += 30000000
        with self.assertRaisesRegex(MODULE.HarnessError, "inter-arrival gap"):
            MODULE.validate_delivery_evidence(
                gap, 40, 2.0, 20, -0.15)

    def test_stdlib_sqlite_delivery_proof_decodes_recorded_twists(self):
        cdr = b"\x00\x01\x00\x00" + struct.pack(
            "<6d", 0.0, 0.0, 0.0, 0.0, 0.0, -0.15)
        with tempfile.TemporaryDirectory() as directory:
            database = Path(directory) / "robot_0.db3"
            connection = sqlite3.connect(database)
            connection.executescript(
                "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT);"
                "CREATE TABLE messages("
                "id INTEGER PRIMARY KEY, topic_id INTEGER, "
                "timestamp INTEGER, data BLOB);"
                "INSERT INTO topics(id, name) VALUES(1, '/cmd_vel/test');"
                "INSERT INTO topics(id, name) VALUES(2, '/cmd_vel/safe');")
            rows = []
            for index in range(40):
                rows.append((
                    index + 1, 1,
                    1000000000 + index * 50000000, cdr))
            for index in range(42):
                rows.append((
                    index + 41, 2,
                    1020000000 + index * 50000000, cdr))
            connection.executemany(
                "INSERT INTO messages(id, topic_id, timestamp, data) "
                "VALUES(?, ?, ?, ?)", rows)
            connection.commit()
            connection.close()
            result = subprocess.run(
                [
                    "python3", "-c", MODULE.MOTION_DELIVERY_EVIDENCE_CHECK,
                    directory, "-0.15", "40", "2.0", "20", "0.0",
                ],
                text=True, capture_output=True, check=False, timeout=1)
        self.assertEqual(result.returncode, 0, result.stderr)
        evidence = MODULE.parse_json_marker(
            result.stdout, MODULE.MOTION_DELIVERY_MARKER)
        MODULE.validate_delivery_evidence(
            evidence, 40, 2.0, 20, -0.15)
        self.assertEqual(
            evidence["topics"]["/cmd_vel/test"]["nonzero_count"], 40)


class PublisherLifecycleTest(unittest.TestCase):
    @staticmethod
    def run_prepare_zero(callback=successful_callback):
        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                state = base_state()
                store.create(state)
                MODULE.RotationHarness(runner, store)._run_twist_publisher(
                    state, "prepare_zero", 0.0, 1.0, 20, 20,
                    required_endpoints=("/:command_arbiter",),
                    state_key="prepare_zero_publisher")
                completed = store.load()
                events = store.events_path.read_text()
        return runner, completed, events

    def test_short_lived_completed_publisher_uses_pinned_evidence_not_live_cmdline(self):
        runner, state, events = self.run_prepare_zero()
        record = state["prepare_zero_publisher"]
        process = record["process"]
        self.assertEqual(record["child_exit_status"], 0)
        self.assertEqual(record["evidence"]["actual_publish_count"], 20)
        self.assertTrue(process["identity_pinned_before_launch"])
        self.assertTrue(process["reaped_by_launch_parent"])
        self.assertEqual(process["wrapper_wait_status"], 0)
        self.assertEqual(process["status"], "reaped")
        self.assertIn("reap_verified_at", process)
        self.assertIn('"event":"publisher_wrapper_reaped"', events)
        self.assertIn('"event":"publisher_reap_verified"', events)
        self.assertIn('"event":"publisher_completed"', events)

        launch_argv = next(
            argv for argv, _ in runner.calls
            if "ROTATION_TWIST_PUBLISHER_ID" in " ".join(argv) and
            "--command-type prepare_zero" in " ".join(argv))
        calls = [" ".join(argv) for argv, _ in runner.calls]
        launch = " ".join(launch_argv)
        reap_check = next(
            call for call in calls
            if "TWIST_PUBLISHER_REAP_VERIFIED" in call)
        self.assertIn('wait "$child"', launch)
        self.assertIn("ROTATION_TWIST_PUBLISHER_REAPED", launch)
        self.assertIn(MODULE.GROUP_EMPTY_AWK, launch)
        self.assertIn("write_phase PRELAUNCH", launch)
        self.assertIn("write_phase GATE_RELEASE_AUTHORIZED", launch)
        self.assertIn('if [ ! -e "$gate" ]; then', launch)
        self.assertIn("D455_PUBLISHER_RECEIPT", launch)
        self.assertIn('test ! -e "/proc/$pid"', reap_check)
        self.assertIn(MODULE.GROUP_EMPTY_AWK, reap_check)
        self.assertIn('[ "$candidate" = "$verifier_pid" ] && continue', reap_check)
        self.assertNotIn(
            'test "$actual_cmd" = 7075626c6973686572', reap_check)
        syntax = subprocess.run(
            ["bash", "-n", "-c", launch_argv[-1]],
            text=True, capture_output=True, check=False, timeout=1)
        self.assertEqual(syntax.returncode, 0, syntax.stderr)

    def test_parent_wait_actually_reaps_short_lived_wrapper(self):
        shell = "\n".join([
            "set -eo pipefail",
            "setsid bash -c 'sleep 0.05' d455-offline-reap-test &",
            "child=$!",
            "pgid=$(ps -o pgid= -p \"$child\" | tr -d ' ')",
            "wait \"$child\"",
            "test ! -e \"/proc/$child\"",
            "ps -eo pgid=,stat= | awk -v wanted=\"$pgid\" "
            + shlex.quote(MODULE.GROUP_EMPTY_AWK),
            "printf 'OFFLINE_WRAPPER_REAPED\\n'",
        ])
        result = subprocess.run(
            ["bash", "-c", shell], text=True, capture_output=True,
            check=False, timeout=2)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "OFFLINE_WRAPPER_REAPED\n")

    def test_missing_gate_cannot_reach_protected_command(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            gate = root / "missing-gate"
            receipt = root / "receipt"
            child_pid = root / "child-pid"
            exit_path = root / "exit"
            log = root / "log"
            reached = root / "protected-command-reached"
            command = (
                "printf reached >" + shlex.quote(str(reached)))
            child = MODULE.RotationHarness._twist_publisher_child_script(
                str(gate), str(receipt), str(child_pid),
                str(exit_path), str(log), command,
                gate_wait_attempts=1)
            result = subprocess.run(
                ["setsid", "bash", "-c", child, "offline-gate-test"],
                text=True, capture_output=True, check=False, timeout=2)
            exit_evidence = exit_path.read_text().strip()
            receipt_evidence = receipt.read_text()
            child_pid_evidence = child_pid.read_text()
            protected_command_reached = reached.exists()
        self.assertEqual(result.returncode, 78, result.stderr)
        self.assertEqual(exit_evidence, "78")
        self.assertIn("D455_PUBLISHER_RECEIPT", receipt_evidence)
        self.assertIn(
            "D455_PUBLISHER_CHILD_BREADCRUMB", child_pid_evidence)
        self.assertFalse(protected_command_reached)

    def test_exact_reap_verifier_ignores_self_but_detects_other_token_process(self):
        token = f"d455-offline-reap-verifier-{os.getpid()}"
        spec = {
            "pid": 99999991,
            "pgid": 99999992,
            "token": token,
        }
        body = MODULE.RotationHarness._publisher_reap_verification_body(spec)
        clean = subprocess.run(
            ["bash", "-c", body], text=True, capture_output=True,
            check=False, timeout=2)
        self.assertEqual(clean.returncode, 0, clean.stderr)
        self.assertEqual(
            clean.stdout, "TWIST_PUBLISHER_REAP_VERIFIED\n")

        other = subprocess.Popen(
            [
                "bash", "-c",
                "while :; do sleep 0.1; done",
                token,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True)
        try:
            detected = subprocess.run(
                ["bash", "-c", body], text=True, capture_output=True,
                check=False, timeout=2)
        finally:
            other.terminate()
            other.wait(timeout=2)
        self.assertEqual(detected.returncode, 76, detected.stderr)
        self.assertNotIn(
            "TWIST_PUBLISHER_REAP_VERIFIED", detected.stdout)

    def test_failed_launch_without_receipt_is_safe_only_when_gate_never_authorized(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if "ROTATION_TWIST_PUBLISHER_ID" in text:
                return MODULE.CommandResult(124, "", "launch timed out")
            if "D455_PUBLISHER_ATTEMPT_PHASE" in text:
                return MODULE.CommandResult(
                    0,
                    "D455_PUBLISHER_ATTEMPT_PHASE PRELAUNCH ABSENT\n"
                    "D455_PUBLISHER_CHILD_BREADCRUMB 404 404\n"
                    "D455_PUBLISHER_PARENT_BREADCRUMB 404 404\n"
                    "D455_PUBLISHER_NO_RECEIPT\n"
                    "D455_PUBLISHER_NO_WAIT_STATUS\n")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                state = base_state()
                store.create(state)
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "launch timed out"):
                    MODULE.RotationHarness(
                        runner, store)._run_twist_publisher(
                            state, "prepare_zero", 0.0, 1.0, 20, 20,
                            required_endpoints=("/:command_arbiter",))
                failed = store.load()
        attempt = failed["publisher_launch_attempts"][0]
        self.assertEqual(attempt["gate_release"], "never_authorized")
        self.assertEqual(attempt["status"], "never_released_reaped")
        self.assertFalse(attempt["identity_pinned_before_launch"])

    def test_launch_timeout_nonzero_and_signal_all_resolve_never_released_gate(self):
        outcomes = {
            "timeout": MODULE.CommandResult(124, "", "timeout"),
            "nonzero": MODULE.CommandResult(9, "", "launch error"),
            "signal": MODULE.HarnessTermination(signal.SIGTERM),
        }
        for label, outcome in outcomes.items():
            with self.subTest(label=label):
                def callback(argv, timeout):
                    text = " ".join(argv)
                    if "ROTATION_TWIST_PUBLISHER_ID" in text:
                        if isinstance(outcome, BaseException):
                            raise outcome
                        return outcome
                    if "D455_PUBLISHER_ATTEMPT_PHASE" in text:
                        return MODULE.CommandResult(
                            0,
                            "D455_PUBLISHER_ATTEMPT_PHASE "
                            "PRELAUNCH ABSENT\n"
                            "D455_PUBLISHER_CHILD_BREADCRUMB 404 404\n"
                            "D455_PUBLISHER_PARENT_BREADCRUMB 404 404\n"
                            "D455_PUBLISHER_NO_RECEIPT\n"
                            "D455_PUBLISHER_NO_WAIT_STATUS\n")
                    return successful_callback(argv, timeout)

                runner = FakeRunner(callback)
                with tempfile.TemporaryDirectory() as directory:
                    with MODULE.StateStore(directory) as store:
                        state = base_state()
                        store.create(state)
                        harness = MODULE.RotationHarness(runner, store)
                        expected = (
                            MODULE.HarnessTermination
                            if label == "signal" else MODULE.HarnessError)
                        with self.assertRaises(expected):
                            harness._start_twist_publisher(
                                state, "prepare_zero", 0.0,
                                1.0, 20, 20,
                                ("/:command_arbiter",))
                        failed = store.load()
                attempt = failed["publisher_launch_attempts"][0]
                self.assertEqual(
                    attempt["status"], "never_released_reaped")

    def test_missing_never_released_phase_evidence_is_rejected(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if "ROTATION_TWIST_PUBLISHER_ID" in text:
                return MODULE.CommandResult(1, "", "launch failed")
            if "D455_PUBLISHER_ATTEMPT_PHASE" in text:
                return MODULE.CommandResult(1, "", "phase missing")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                state = base_state()
                store.create(state)
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "launch recovery failed"):
                    MODULE.RotationHarness(
                        runner, store)._run_twist_publisher(
                            state, "prepare_zero", 0.0, 1.0, 20, 20,
                            required_endpoints=("/:command_arbiter",))
                failed = store.load()
        self.assertEqual(
            failed["publisher_launch_attempts"][0]["status"],
            "registered_before_launch")

    def test_ambiguous_never_released_token_groups_fail_cleanup(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if "ROTATION_TWIST_PUBLISHER_ID" in text:
                return MODULE.CommandResult(1, "", "launch failed")
            if "D455_PUBLISHER_ATTEMPT_PHASE" in text:
                return MODULE.CommandResult(
                    0,
                    "D455_PUBLISHER_ATTEMPT_PHASE PRELAUNCH ABSENT\n"
                    "D455_PUBLISHER_CHILD_BREADCRUMB 404 404\n"
                    "D455_PUBLISHER_PARENT_BREADCRUMB 404 404\n"
                    "D455_PUBLISHER_NO_RECEIPT\n"
                    "D455_PUBLISHER_NO_WAIT_STATUS\n")
            if (
                    "token=d455-prepare-zero-publisher-" in text and
                    "TWIST_NEVER_RELEASED_REAP_VERIFIED" in text):
                return MODULE.CommandResult(
                    1, "", "multiple token groups")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                state = base_state()
                store.create(state)
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "launch recovery failed"):
                    MODULE.RotationHarness(
                        runner, store)._run_twist_publisher(
                            state, "prepare_zero", 0.0, 1.0, 20, 20,
                            required_endpoints=("/:command_arbiter",))

    def test_never_released_zombie_breadcrumb_fails_cleanup(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if "ROTATION_TWIST_PUBLISHER_ID" in text:
                return MODULE.CommandResult(1, "", "launch failed")
            if "D455_PUBLISHER_ATTEMPT_PHASE" in text:
                return MODULE.CommandResult(
                    0,
                    "D455_PUBLISHER_ATTEMPT_PHASE PRELAUNCH ABSENT\n"
                    "D455_PUBLISHER_CHILD_BREADCRUMB 404 404\n"
                    "D455_PUBLISHER_PARENT_BREADCRUMB 404 404\n"
                    "D455_PUBLISHER_NO_RECEIPT\n"
                    "D455_PUBLISHER_NO_WAIT_STATUS\n")
            if "TWIST_NEVER_RELEASED_REAP_VERIFIED" in text:
                return MODULE.CommandResult(
                    44, "", "breadcrumb PID is a zombie")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                state = base_state()
                store.create(state)
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "launch recovery failed"):
                    MODULE.RotationHarness(
                        runner, store)._run_twist_publisher(
                            state, "prepare_zero", 0.0, 1.0, 20, 20,
                            required_endpoints=("/:command_arbiter",))

    def test_missing_pid_breadcrumb_never_counts_as_absence(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if "ROTATION_TWIST_PUBLISHER_ID" in text:
                return MODULE.CommandResult(1, "", "launch failed")
            if "D455_PUBLISHER_ATTEMPT_PHASE" in text:
                return MODULE.CommandResult(
                    0,
                    "D455_PUBLISHER_ATTEMPT_PHASE PRELAUNCH ABSENT\n"
                    "D455_PUBLISHER_NO_CHILD_BREADCRUMB\n"
                    "D455_PUBLISHER_NO_PARENT_BREADCRUMB\n"
                    "D455_PUBLISHER_NO_RECEIPT\n"
                    "D455_PUBLISHER_NO_WAIT_STATUS\n")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                state = base_state()
                store.create(state)
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "no durable PID/PGID"):
                    MODULE.RotationHarness(
                        runner, store)._run_twist_publisher(
                            state, "prepare_zero", 0.0, 1.0, 20, 20,
                            required_endpoints=("/:command_arbiter",))

    def test_unproven_motion_launch_blocks_later_cleanup_zero_publisher(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if (
                    "ROTATION_TWIST_PUBLISHER_ID" in text and
                    "--command-type motion" in text):
                return MODULE.CommandResult(1, "", "motion launch failed")
            if (
                    "D455_PUBLISHER_ATTEMPT_PHASE" in text and
                    "d455-motion-publisher-" in text):
                return MODULE.CommandResult(
                    0,
                    "D455_PUBLISHER_ATTEMPT_PHASE "
                    "GATE_RELEASE_AUTHORIZED ABSENT\n"
                    "D455_PUBLISHER_CHILD_BREADCRUMB 505 505\n"
                    "D455_PUBLISHER_PARENT_BREADCRUMB 505 505\n"
                    "D455_PUBLISHER_RECEIPT "
                    "505 505 505 5005 6d6f74696f6e\n"
                    "D455_PUBLISHER_NO_WAIT_STATUS\n")
            if (
                    "TWIST_PUBLISHER_STOPPED" in text and
                    "token=d455-motion-publisher-" in text):
                return MODULE.CommandResult(44, "", "owned zombie")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                with self.assertRaises(MODULE.HarnessError):
                    MODULE.RotationHarness(runner, store).motion(
                        motion_args(clean_audit(directory)))
                failed = store.load()
                events = store.events_path.read_text()
        calls = [" ".join(argv) for argv, _ in runner.calls]
        motion_index = next(
            index for index, call in enumerate(calls)
            if "--command-type motion" in call and
            "ROTATION_TWIST_PUBLISHER_ID" in call)
        later_cleanup_zero = [
            call for call in calls[motion_index + 1:]
            if "--command-type cleanup_zero" in call and
            "ROTATION_TWIST_PUBLISHER_ID" in call]
        self.assertEqual(failed["status"], "invalid")
        self.assertEqual(later_cleanup_zero, [])
        self.assertIn(
            "cleanup_zero_blocked_unproven_motion_absence", events)

    def test_abort_and_finalize_block_zero_when_motion_absence_is_unproven(self):
        def unresolved_motion_attempt():
            return {
                "kind": "motion publisher",
                "command_type": "motion",
                "container": "robot",
                "container_id": ROBOT_ID,
                "setup": ["/opt/ros/humble/setup.bash"],
                "pid": 505,
                "pgid": 505,
                "sid": 505,
                "starttime": 5005,
                "cmdline_hex": "6d6f74696f6e",
                "token": "d455-motion-publisher-unresolved",
                "status": "launch_recovery_required",
            }

        def callback(argv, timeout):
            text = " ".join(argv)
            if (
                    "TWIST_PUBLISHER_STOPPED" in text and
                    "d455-motion-publisher-unresolved" in text):
                return MODULE.CommandResult(44, "", "owned zombie")
            return successful_callback(argv, timeout)

        for stage in ("abort", "finalize"):
            with self.subTest(stage=stage):
                runner = FakeRunner(callback)
                with tempfile.TemporaryDirectory() as directory:
                    if stage == "abort":
                        initial = base_state(status="motion_in_progress")
                        stage_args = None
                    else:
                        initial = finalize_state(directory)
                        stage_args = argparse.Namespace(
                            kernel_audit_artifact=clean_audit(
                                directory, "post-audit.txt"))
                    initial["publisher_launch_attempts"] = [
                        unresolved_motion_attempt()]
                    with MODULE.StateStore(directory) as store:
                        store.create(initial)
                        harness = MODULE.RotationHarness(runner, store)
                        with self.assertRaises(MODULE.HarnessError):
                            if stage == "abort":
                                harness.abort()
                            else:
                                harness.finalize(stage_args)
                        failed = store.load()
                        events = store.events_path.read_text()
                calls = [" ".join(argv) for argv, _ in runner.calls]
                self.assertEqual(failed["status"], "invalid")
                self.assertFalse(any(
                    "--command-type cleanup_zero" in call and
                    "ROTATION_TWIST_PUBLISHER_ID" in call
                    for call in calls))
                self.assertIn(
                    f"{stage}_zero_blocked_unproven_motion_absence",
                    events)

    def test_wrapper_wait_status_must_match_pinned_exit_artifact(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if "ROTATION_TWIST_PUBLISHER_ID" in text:
                return MODULE.CommandResult(
                    0,
                    "ROTATION_TWIST_PUBLISHER_ID "
                    "404 404 404 4004 7075626c6973686572\n"
                    "ROTATION_TWIST_PUBLISHER_REAPED 404 7\n")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                state = base_state()
                store.create(state)
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "exit evidence is ambiguous"):
                    MODULE.RotationHarness(
                        runner, store)._run_twist_publisher(
                            state, "prepare_zero", 0.0, 1.0, 20, 20,
                            required_endpoints=("/:command_arbiter",))
                failed = store.load()
        self.assertEqual(len(failed["publisher_runs"]), 1)
        self.assertEqual(
            failed["publisher_runs"][0]["process"]["wrapper_wait_status"], 7)
        self.assertEqual(
            failed["publisher_runs"][0]["child_exit_status"], 0)

    def test_missing_result_json_is_rejected_after_reap(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if (
                    "D455_PUBLISHER_FILE_READ" in text and
                    "/tmp/d455-prepare-zero-publisher-" in text and
                    ".json" in text):
                return MODULE.CommandResult(
                    1, "", "publisher result is missing")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                state = base_state()
                store.create(state)
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "publisher evidence"):
                    MODULE.RotationHarness(
                        runner, store)._run_twist_publisher(
                            state, "prepare_zero", 0.0, 1.0, 20, 20,
                            required_endpoints=("/:command_arbiter",))
                failed = store.load()
                events = store.events_path.read_text()
        self.assertEqual(failed.get("publisher_runs", []), [])
        self.assertNotIn('"event":"publisher_completed"', events)

    def test_unreaped_zombie_fails_even_with_complete_evidence(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if "TWIST_PUBLISHER_REAP_VERIFIED" in text:
                return MODULE.CommandResult(1, "", "owned zombie remains")
            if (
                    "token=d455-prepare-zero-publisher-" in text and
                    "kill -INT" in text):
                return MODULE.CommandResult(1, "", "owned zombie remains")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                state = base_state()
                store.create(state)
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "reaping failed"):
                    MODULE.RotationHarness(
                        runner, store)._run_twist_publisher(
                            state, "prepare_zero", 0.0, 1.0, 20, 20,
                            required_endpoints=("/:command_arbiter",))
                failed = store.load()
                events = store.events_path.read_text()
        self.assertEqual(
            failed["publisher_runs"][0]["evidence"]["status"], "complete")
        self.assertEqual(
            failed["publisher_runs"][0]["evidence"]["actual_publish_count"], 20)
        self.assertIn("publisher_partial_evidence_persisted", events)
        self.assertNotIn('"event":"publisher_completed"', events)

    def test_zero_phase_rejects_nonzero_before_external_command(self):
        runner = FakeRunner(successful_callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                state = base_state()
                store.create(state)
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "requires exact-zero"):
                    MODULE.RotationHarness(
                        runner, store)._run_twist_publisher(
                            state, "cleanup_zero", 0.15, 1.0, 20, 20,
                            required_endpoints=("/:command_arbiter",))
        self.assertEqual(runner.calls, [])

    def test_abort_cleanup_zero_is_reaped_before_success(self):
        runner = FakeRunner(successful_callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                initial = base_state(status="invalid")
                initial["publisher_runs"] = [{
                    "process": {
                        "kind": "prepare_zero publisher",
                        "command_type": "prepare_zero",
                        "container": "robot",
                        "container_id": ROBOT_ID,
                        "pid": 303,
                        "pgid": 303,
                        "sid": 303,
                        "starttime": 3003,
                        "cmdline_hex": "7075626c6973686572",
                        "token": "d455-prepare-zero-publisher-prior",
                        "status": "reaped",
                    },
                }]
                store.create(initial)
                MODULE.RotationHarness(runner, store).abort()
                state = store.load()
                events = store.events_path.read_text()
        self.assertEqual(state["status"], "aborted")
        self.assertEqual(len(state["publisher_runs"]), 2)
        for record in state["publisher_runs"]:
            process = record["process"]
            self.assertEqual(process["status"], "reaped")
            self.assertIn("reap_verified_at", process)
        self.assertEqual(
            state["publisher_runs"][1]["process"]["command_type"],
            "cleanup_zero")
        self.assertIn('"event":"publisher_reap_verified"', events)
        self.assertIn(
            '"event":"tracked_publisher_reap_verified"', events)
        self.assertIn('"phase":"abort_before_zero"', events)
        self.assertIn('"phase":"abort_after_zero"', events)
        self.assertIn('"cleanup_errors":[]', events)

    def test_strict_publisher_stop_rejects_zombie_leader(self):
        spec = {
            "pid": 404, "pgid": 404, "sid": 404,
            "starttime": 4004, "cmdline_hex": "7075626c6973686572",
            "token": "d455-offline-zombie-test",
        }
        setup = "\n".join([
            "set -eo pipefail",
            "pid=404",
            "pgid=404",
            "actual_pgid=404",
            "actual_sid=404",
            "actual_start=4004",
            "actual_state=Z",
            "actual_cmd=''",
        ])
        result = subprocess.run(
            ["bash", "-c", setup + "\n" +
             MODULE.RotationHarness._publisher_stop_decision_body(spec)],
            text=True, capture_output=True, check=False, timeout=1)
        self.assertEqual(result.returncode, 44)
        self.assertNotIn("TWIST_PUBLISHER_STOPPED", result.stdout)


class DiagnosticGateTest(unittest.TestCase):
    @staticmethod
    def status(name, level, message):
        return SimpleNamespace(name=name, level=level, message=message)

    @staticmethod
    def stamp(seconds):
        whole = int(seconds)
        return SimpleNamespace(
            sec=whole, nanosec=int(round((seconds - whole) * 1000000000)))

    def helper(self):
        namespace = {}
        exec(MODULE.DIAGNOSTIC_WINDOW_HELPER, namespace)
        return namespace

    def test_same_array_ready_and_fresh_is_accepted(self):
        helper = self.helper()
        window = helper["new_diagnostic_window"]()
        accepted = helper["observe_diagnostic_array"](
            window,
            [
                self.status("roboteq/serial_connection", 0, "ready"),
                self.status("roboteq/encoder_freshness", 0, "fresh"),
            ],
            self.stamp(10.0),
            20.0)
        self.assertTrue(accepted)
        self.assertEqual(window["coherence_source"], "header")
        self.assertEqual(window["coherence_delta"], 0.0)

    def test_separate_arrays_within_window_are_accepted(self):
        helper = self.helper()
        window = helper["new_diagnostic_window"]()
        observe = helper["observe_diagnostic_array"]
        self.assertFalse(observe(
            window,
            [self.status("roboteq/serial_connection", 0, "ready")],
            self.stamp(10.0),
            20.0))
        self.assertTrue(observe(
            window,
            [self.status("roboteq/encoder_freshness", 0, "fresh")],
            self.stamp(11.5),
            21.5))
        self.assertEqual(window["coherence_source"], "header")
        self.assertAlmostEqual(window["coherence_delta"], 1.5)

    def test_receive_time_fallback_accepts_when_one_stamp_is_invalid(self):
        helper = self.helper()
        window = helper["new_diagnostic_window"]()
        observe = helper["observe_diagnostic_array"]
        missing_stamp = self.stamp(0.0)
        observe(
            window,
            [self.status("roboteq/serial_connection", 0, "ready")],
            missing_stamp,
            20.0)
        self.assertTrue(observe(
            window,
            [self.status("roboteq/encoder_freshness", 0, "fresh")],
            self.stamp(100.0),
            21.0))
        self.assertEqual(window["coherence_source"], "receive")
        self.assertEqual(window["coherence_delta"], 1.0)

    def test_observations_too_far_apart_are_rejected(self):
        helper = self.helper()
        window = helper["new_diagnostic_window"]()
        observe = helper["observe_diagnostic_array"]
        observe(
            window,
            [self.status("roboteq/serial_connection", 0, "ready")],
            self.stamp(10.0),
            20.0)
        self.assertFalse(observe(
            window,
            [self.status("roboteq/encoder_freshness", 0, "fresh")],
            self.stamp(12.1),
            22.1))
        self.assertAlmostEqual(window["coherence_delta"], 2.1)

        boundary = helper["new_diagnostic_window"]()
        observe(
            boundary,
            [self.status("roboteq/serial_connection", 0, "ready")],
            self.stamp(20.0),
            30.0)
        self.assertTrue(observe(
            boundary,
            [self.status("roboteq/encoder_freshness", 0, "fresh")],
            self.stamp(22.0),
            32.0))

    def test_zero_byte_levels_are_normalized(self):
        helper = self.helper()
        normalize = helper["normalize_diagnostic_level"]
        self.assertEqual(normalize(0), 0)
        self.assertEqual(normalize(b"\x00"), 0)
        self.assertEqual(normalize(bytearray(b"\x00")), 0)
        self.assertIsNone(normalize(b""))
        self.assertIsNone(normalize(b"\x00\x00"))
        self.assertIsNone(normalize(False))
        window = helper["new_diagnostic_window"]()
        self.assertTrue(helper["observe_diagnostic_array"](
            window,
            [
                self.status("roboteq/serial_connection", b"\x00", "ready"),
                self.status(
                    "roboteq/encoder_freshness", bytearray(b"\x00"), "fresh"),
            ],
            self.stamp(10.0),
            20.0))

    def test_stale_disconnected_resync_and_error_are_sticky(self):
        cases = [
            self.status("roboteq/serial_connection", 2, "disconnected"),
            self.status("roboteq/encoder_freshness", 1, "stale"),
            self.status("roboteq/serial_connection", 1, "resync"),
            self.status("roboteq/encoder_freshness", b"\x02", "fresh"),
            self.status("roboteq/serial_connection", b"", "ready"),
        ]
        helper = self.helper()
        for bad in cases:
            with self.subTest(status=bad):
                window = helper["new_diagnostic_window"]()
                observe = helper["observe_diagnostic_array"]
                self.assertFalse(observe(
                    window, [bad], self.stamp(10.0), 20.0))
                self.assertTrue(window["sticky_errors"])
                self.assertFalse(observe(
                    window,
                    [
                        self.status("roboteq/serial_connection", 0, "ready"),
                        self.status("roboteq/encoder_freshness", 0, "fresh"),
                    ],
                    self.stamp(10.5),
                    20.5))

    def test_missing_serial_is_rejected(self):
        helper = self.helper()
        window = helper["new_diagnostic_window"]()
        accepted = helper["observe_diagnostic_array"](
            window,
            [self.status("roboteq/encoder_freshness", 0, "fresh")],
            self.stamp(10.0),
            20.0)
        self.assertFalse(accepted)
        self.assertIsNone(window["serial"])

    def test_missing_encoder_is_rejected(self):
        helper = self.helper()
        window = helper["new_diagnostic_window"]()
        accepted = helper["observe_diagnostic_array"](
            window,
            [self.status("roboteq/serial_connection", 0, "ready")],
            self.stamp(10.0),
            20.0)
        self.assertFalse(accepted)
        self.assertIsNone(window["encoder"])

    def test_qos_bounds_and_failure_detail_are_explicit(self):
        self.assertEqual(MODULE.DIAGNOSTIC_DISCOVERY_TIMEOUT_SECONDS, 10.0)
        self.assertEqual(MODULE.DIAGNOSTIC_MESSAGE_TIMEOUT_SECONDS, 8.0)
        self.assertEqual(MODULE.DIAGNOSTIC_COHERENCE_WINDOW_SECONDS, 2.0)
        self.assertIn("QoSReliabilityPolicy.BEST_EFFORT", MODULE.DIAGNOSTIC_CHECK)
        self.assertIn("QoSDurabilityPolicy.VOLATILE", MODULE.DIAGNOSTIC_CHECK)
        self.assertIn("messages={message_count}", MODULE.DIAGNOSTIC_CHECK)
        self.assertIn("last_serial={window['last_serial']!r}", MODULE.DIAGNOSTIC_CHECK)
        self.assertIn("last_encoder={window['last_encoder']!r}", MODULE.DIAGNOSTIC_CHECK)
        self.assertIn("sticky_errors={window['sticky_errors']!r}",
                      MODULE.DIAGNOSTIC_CHECK)
        compile(MODULE.DIAGNOSTIC_CHECK, "<diagnostic-check>", "exec")

    def test_diagnostic_gate_is_bounded_and_remains_before_nonzero(self):
        def successful_motion(argv, timeout):
            return successful_callback(argv, timeout)

        runner = FakeRunner(successful_motion)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                MODULE.RotationHarness(runner, store).motion(
                    motion_args(clean_audit(directory)))
        calls = [(" ".join(argv), timeout) for argv, timeout in runner.calls]
        diagnostic = next(
            index for index, (call, _) in enumerate(calls)
            if "rotation_harness_diagnostic_check" in call)
        arbiter_config = next(
            index for index, (call, _) in enumerate(calls)
            if "ros2 param get /command_arbiter publish_rate_hz" in call)
        zero_prime = next(
            index for index, (call, _) in enumerate(calls)
            if publishes_twist(call, 0.0))
        nonzero = next(
            index for index, (call, _) in enumerate(calls)
            if publishes_twist(call, -0.15))
        self.assertEqual(
            calls[diagnostic][1], MODULE.DIAGNOSTIC_COMMAND_TIMEOUT_SECONDS)
        self.assertIn(
            f"timeout {MODULE.DIAGNOSTIC_SHELL_TIMEOUT_SECONDS}s",
            calls[diagnostic][0])
        self.assertLess(arbiter_config, diagnostic)
        self.assertLess(diagnostic, zero_prime)
        self.assertLess(zero_prime, nonzero)

    def test_arbiter_timing_gate_failure_blocks_all_nonzero_commands(self):
        def wrong_arbiter(argv, timeout):
            text = " ".join(argv)
            if "ros2 param get /command_arbiter publish_rate_hz" in text:
                return MODULE.CommandResult(
                    1, "", "unexpected command-arbiter timing")
            return successful_callback(argv, timeout)

        runner = FakeRunner(wrong_arbiter)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "arbiter timing"):
                    MODULE.RotationHarness(runner, store).motion(
                        motion_args(clean_audit(directory)))
        commands = [" ".join(argv) for argv, _ in runner.calls]
        self.assertFalse(any(
            publishes_twist(command, angular_z)
            for command in commands
            for angular_z in MODULE.ALLOWED_ANGULAR_Z))


class StateStoreTest(unittest.TestCase):
    def test_state_creation_is_exclusive_and_atomic_updates_remain_readable(self):
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                state = base_state()
                store.create(state)
                with self.assertRaises(MODULE.HarnessError):
                    store.create(state)
                state["status"] = "motion_complete"
                store.save(state)
                self.assertEqual(store.load()["status"], "motion_complete")


class AuditGateTest(unittest.TestCase):
    def test_audit_requires_fresh_exact_clean_markers(self):
        with tempfile.TemporaryDirectory() as directory:
            created = (
                datetime.datetime.now(datetime.timezone.utc) -
                datetime.timedelta(seconds=1)).isoformat()
            path = Path(clean_audit(directory))
            self.assertEqual(len(MODULE.validate_kernel_audit(path, created)), 64)
            path.write_text(
                "apparmor_denials=1\nd455_usb_reset_or_disconnect=0\n",
                encoding="utf-8")
            with self.assertRaises(MODULE.HarnessError):
                MODULE.validate_kernel_audit(path, created)
            path.write_text(
                "apparmor_denials=0\nd455_usb_reset_or_disconnect=0\n",
                encoding="utf-8")
            stale = datetime.datetime.now().timestamp() - 300
            os.utime(path, (stale, stale))
            with self.assertRaises(MODULE.HarnessError):
                MODULE.validate_kernel_audit(path, created)


class MotionFailureTest(unittest.TestCase):
    def test_motion_failure_attempts_zero_verifies_zero_and_stops_recorders(self):
        def callback(argv, _timeout):
            text = " ".join(argv)
            if publishes_twist(text, -0.15):
                return MODULE.CommandResult(
                    1, "", "AMENT_TRACE_SETUP_FILES: unbound variable")
            return successful_callback(argv, _timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                harness = MODULE.RotationHarness(runner, store)
                args = motion_args(clean_audit(directory))
                with self.assertRaises(MODULE.HarnessError):
                    harness.motion(args)
                state = store.load()
                events = [
                    json.loads(line)["event"]
                    for line in store.events_path.read_text().splitlines()
                ]
                partial_robot = (Path(directory) / "partial-robot-bag").is_dir()
                partial_imu = (Path(directory) / "partial-imu-bag").is_dir()
                event_text = store.events_path.read_text()

        joined = [" ".join(call[0]) for call in runner.calls]
        self.assertEqual(state["status"], "invalid")
        self.assertTrue(any(publishes_twist(call, 0.0) for call in joined))
        self.assertTrue(any("rotation_harness_zero_check" in call for call in joined))
        self.assertEqual(sum("RECORDER_REAPED" in call for call in joined), 2)
        self.assertIn("motion_failed", events)
        self.assertIn("safe_zero_verified", events)
        self.assertNotIn("motion_stage_completed", events)
        self.assertTrue(any("/camera/imu sensor_msgs/msg/Imu" in call for call in joined))
        self.assertTrue(any("/wheel_ticks" in call for call in joined))
        self.assertTrue(any("/odom nav_msgs/msg/Odometry" in call for call in joined))
        self.assertTrue(any("rotation_harness_diagnostic_check" in call for call in joined))
        self.assertTrue(partial_robot)
        self.assertTrue(partial_imu)
        self.assertIn("invalid_partial", event_text)

    def test_recorded_delivery_failure_keeps_zero_cleanup_and_never_succeeds(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if MODULE.MOTION_DELIVERY_MARKER in text:
                return marker_result(
                    MODULE.MOTION_DELIVERY_MARKER,
                    delivery_evidence(test_count=30, test_span=1.45))
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                harness = MODULE.RotationHarness(runner, store)
                with self.assertRaisesRegex(
                        MODULE.HarnessError,
                        "/cmd_vel/test nonzero count mismatch"):
                    harness.motion(motion_args(clean_audit(directory)))
                state = store.load()
                events = store.events_path.read_text()
                artifact = (
                    Path(directory) / MODULE.MOTION_DELIVERY_EVIDENCE_NAME)
                persisted = json.loads(artifact.read_text())
        calls = [" ".join(argv) for argv, _ in runner.calls]
        nonzero = next(
            index for index, call in enumerate(calls)
            if publishes_twist(call, -0.15))
        post_zero = next(
            index for index, call in enumerate(calls)
            if index > nonzero and publishes_twist(call, 0.0))
        zero_check = next(
            index for index, call in enumerate(calls)
            if index > post_zero and "rotation_harness_zero_check" in call)
        self.assertEqual(state["status"], "invalid")
        self.assertEqual(persisted["topics"]["/cmd_vel/test"]["nonzero_count"], 30)
        self.assertLess(nonzero, post_zero)
        self.assertLess(post_zero, zero_check)
        self.assertIn("motion_delivery_failed", events)
        self.assertNotIn("motion_stage_completed", events)

    def test_missing_owned_publisher_identity_is_invalid(self):
        def callback(argv, _timeout):
            text = " ".join(argv)
            if publishes_twist(text, -0.15):
                return MODULE.CommandResult(0, "publisher exited without output\n")
            return successful_callback(argv, _timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                harness = MODULE.RotationHarness(runner, store)
                args = motion_args(clean_audit(directory))
                with self.assertRaisesRegex(
                        MODULE.HarnessError,
                        "invalid motion publisher completion identity"):
                    harness.motion(args)
                state = store.load()
                events = store.events_path.read_text()
        self.assertEqual(state["status"], "invalid")
        self.assertNotIn("motion_stage_completed", events)

    def test_motion_rejects_stale_or_wrong_stage_before_external_commands(self):
        runner = FakeRunner(lambda *_: MODULE.CommandResult(0, ""))
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state(status="invalid"))
                harness = MODULE.RotationHarness(runner, store)
                args = motion_args(str(Path(directory) / "not-used"))
                with self.assertRaises(MODULE.HarnessError):
                    harness.motion(args)
        self.assertEqual(runner.calls, [])

    def test_container_replacement_aborts_before_nonzero_publication(self):
        def callback(argv, timeout):
            if argv[:3] == ["docker", "inspect", "-f"] and argv[-1] == "imu":
                return MODULE.CommandResult(0, f"{'c' * 64} true\n")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                harness = MODULE.RotationHarness(runner, store)
                with self.assertRaisesRegex(MODULE.HarnessError, "identity changed"):
                    harness.motion(motion_args(clean_audit(directory)))
        joined = "\n".join(" ".join(call[0]) for call in runner.calls)
        self.assertNotIn(MODULE.twist_yaml(-0.15), joined)

    def test_live_but_unsubscribed_recorder_fails_before_motion(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if "ros2 topic info --verbose /camera/imu" in text:
                return MODULE.CommandResult(
                    0,
                    "Node name: realsense_imu_relay\n"
                    "Endpoint type: PUBLISHER\n")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                harness = MODULE.RotationHarness(runner, store)
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "subscription is absent"):
                    harness.motion(motion_args(clean_audit(directory)))
        joined = "\n".join(" ".join(call[0]) for call in runner.calls)
        self.assertNotIn(MODULE.twist_yaml(-0.15), joined)
        self.assertIn(MODULE.twist_yaml(0.0), joined)

    def test_missing_arbiter_endpoint_blocks_nonzero_after_zero_prime(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if "ros2 topic info --verbose /cmd_vel/test" in text:
                return MODULE.CommandResult(
                    0,
                    "Node name: rosbag2_recorder\n"
                    "Endpoint type: SUBSCRIPTION\n")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                with self.assertRaisesRegex(
                        MODULE.HarnessError,
                        "command_arbiter subscription is absent"):
                    MODULE.RotationHarness(runner, store).motion(
                        motion_args(clean_audit(directory)))
        calls = [" ".join(argv) for argv, _ in runner.calls]
        self.assertFalse(any(
            publishes_twist(call, angular_z)
            for call in calls for angular_z in MODULE.ALLOWED_ANGULAR_Z))
        self.assertTrue(any(publishes_twist(call, 0.0) for call in calls))

    def test_reap_verification_interrupt_rechecks_before_zero_and_persists_partial(self):
        wait_interrupted = False
        motion_started = False

        def callback(argv, timeout):
            nonlocal wait_interrupted, motion_started
            text = " ".join(argv)
            if (
                    "ROTATION_TWIST_PUBLISHER_ID" in text and
                    "--command-type motion" in text):
                motion_started = True
            if (
                    motion_started and
                    "TWIST_PUBLISHER_REAP_VERIFIED" in text and
                    not wait_interrupted):
                wait_interrupted = True
                raise KeyboardInterrupt()
            if (
                    "D455_PUBLISHER_FILE_READ" in text and
                    "/tmp/d455-motion-publisher-" in text and
                    ".json" in text):
                partial = publisher_evidence(
                    actual_count=10, window_duration=0.5,
                    publish_span=0.45, status="interrupted")
                partial["error"] = "PublisherInterrupted: signal 2"
                return MODULE.CommandResult(
                    0, json.dumps(partial, sort_keys=True) + "\n")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                with self.assertRaises(KeyboardInterrupt):
                    MODULE.RotationHarness(runner, store).motion(
                        motion_args(clean_audit(directory)))
                state = store.load()
                partial_path = (
                    Path(directory) /
                    MODULE.MOTION_PUBLISHER_EVIDENCE_NAME)
                self.assertTrue(
                    partial_path.is_file(), state.get("failure"))
                partial = json.loads(partial_path.read_text())
                events = store.events_path.read_text()
        calls = [" ".join(argv) for argv, _ in runner.calls]
        first_reap_check = next(
            index for index, call in enumerate(calls)
            if "TWIST_PUBLISHER_REAP_VERIFIED" in call)
        cleanup_reap_check = next(
            index for index, call in enumerate(calls)
            if index > first_reap_check and
            "TWIST_PUBLISHER_REAP_VERIFIED" in call)
        post_zero = next(
            index for index, call in enumerate(calls)
            if index > cleanup_reap_check and publishes_twist(call, 0.0))
        self.assertEqual(state["status"], "invalid")
        self.assertEqual(partial["status"], "interrupted")
        self.assertEqual(partial["actual_publish_count"], 10)
        self.assertLess(first_reap_check, cleanup_reap_check)
        self.assertLess(cleanup_reap_check, post_zero)
        self.assertIn("publisher_partial_evidence_persisted", events)
        self.assertIn("publisher_cleanup_reaped", events)

    def test_post_completion_reap_verification_interrupt_precedes_zero(self):
        verification_interrupted = False
        motion_started = False

        def callback(argv, timeout):
            nonlocal verification_interrupted, motion_started
            text = " ".join(argv)
            if (
                    "ROTATION_TWIST_PUBLISHER_ID" in text and
                    "--command-type motion" in text):
                motion_started = True
                return successful_callback(argv, timeout)
            if (
                    motion_started and
                    "TWIST_PUBLISHER_REAP_VERIFIED" in text and
                    not verification_interrupted):
                verification_interrupted = True
                raise KeyboardInterrupt()
            if (
                    "D455_PUBLISHER_FILE_READ" in text and
                    "/tmp/d455-motion-publisher-" in text and
                    ".json" in text):
                partial = publisher_evidence(
                    actual_count=1, window_duration=0.05,
                    publish_span=0.0, status="interrupted")
                return MODULE.CommandResult(
                    0, json.dumps(partial, sort_keys=True) + "\n")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                with self.assertRaises(KeyboardInterrupt):
                    MODULE.RotationHarness(runner, store).motion(
                        motion_args(clean_audit(directory)))
                partial_path = (
                    Path(directory) /
                    MODULE.MOTION_PUBLISHER_EVIDENCE_NAME)
                partial_persisted = partial_path.is_file()
                state = store.load()
        calls = [" ".join(argv) for argv, _ in runner.calls]
        motion_start = next(
            index for index, call in enumerate(calls)
            if "ROTATION_TWIST_PUBLISHER_ID" in call and
            "--command-type motion" in call)
        verification = next(
            index for index, call in enumerate(calls)
            if index > motion_start and
            "TWIST_PUBLISHER_REAP_VERIFIED" in call)
        cleanup_reap = next(
            index for index, call in enumerate(calls)
            if index > verification and
            "TWIST_PUBLISHER_REAP_VERIFIED" in call)
        zero = next(
            index for index, call in enumerate(calls)
            if index > cleanup_reap and publishes_twist(call, 0.0))
        self.assertTrue(partial_persisted, state.get("failure"))
        self.assertLess(verification, cleanup_reap)
        self.assertLess(cleanup_reap, zero)

    def test_main_sigterm_path_rechecks_publisher_before_zero_and_returns_143(self):
        terminated = False

        def callback(argv, timeout):
            nonlocal terminated
            text = " ".join(argv)
            if "TWIST_PUBLISHER_REAP_VERIFIED" in text and not terminated:
                terminated = True
                raise MODULE.HarnessTermination(signal.SIGTERM)
            if (
                    "D455_PUBLISHER_FILE_READ" in text and
                    "/tmp/d455-motion-publisher-" in text and
                    ".json" in text):
                partial = publisher_evidence(
                    actual_count=10, window_duration=0.5,
                    publish_span=0.45, status="interrupted")
                return MODULE.CommandResult(
                    0, json.dumps(partial, sort_keys=True) + "\n")
            return successful_callback(argv, timeout)

        with tempfile.TemporaryDirectory() as directory:
            audit = clean_audit(directory)
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
            stdout = io.StringIO()
            stderr = io.StringIO()
            runner = FakeRunner(callback)
            argv = [
                "--evidence-dir", directory, "motion",
                "--linear-x", "0.0", "--angular-z", "-0.15",
                "--duration", "2.0", "--rate-hz", "20",
                "--acknowledge-motion", MODULE.MOTION_ACK,
                "--kernel-audit-artifact", audit,
            ]
            with contextlib.redirect_stdout(
                    stdout), contextlib.redirect_stderr(stderr):
                status = MODULE.main(
                    argv,
                    runner=runner,
                    allow_frozen_motion_for_tests=True,
                )
        calls = [" ".join(argv) for argv, _ in runner.calls]
        reap_check = next(
            index for index, call in enumerate(calls)
            if "TWIST_PUBLISHER_REAP_VERIFIED" in call)
        cleanup_reap = next(
            index for index, call in enumerate(calls)
            if index > reap_check and
            "TWIST_PUBLISHER_REAP_VERIFIED" in call)
        zero = next(
            index for index, call in enumerate(calls)
            if index > cleanup_reap and publishes_twist(call, 0.0))
        self.assertEqual(status, 143)
        self.assertNotIn("motion completed", stdout.getvalue())
        self.assertIn("terminated by signal 15", stderr.getvalue())
        self.assertLess(reap_check, cleanup_reap)
        self.assertLess(cleanup_reap, zero)

    def test_keyboard_interrupt_zeroes_and_cleans_both_before_reraise(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if publishes_twist(text, -0.15):
                raise KeyboardInterrupt()
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                harness = MODULE.RotationHarness(runner, store)
                with self.assertRaises(KeyboardInterrupt):
                    harness.motion(motion_args(clean_audit(directory)))
                state = store.load()
        joined = [" ".join(call[0]) for call in runner.calls]
        self.assertEqual(state["status"], "invalid")
        self.assertTrue(any(publishes_twist(call, 0.0) for call in joined))
        self.assertEqual(sum("RECORDER_REAPED" in call for call in joined), 2)

    def test_main_failure_does_not_print_motion_completed(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if MODULE.MOTION_DELIVERY_MARKER in text:
                return marker_result(
                    MODULE.MOTION_DELIVERY_MARKER,
                    delivery_evidence(test_count=30, test_span=1.45))
            return successful_callback(argv, timeout)

        with tempfile.TemporaryDirectory() as directory:
            audit = clean_audit(directory)
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
            stdout = io.StringIO()
            stderr = io.StringIO()
            argv = [
                "--evidence-dir", directory, "motion",
                "--linear-x", "0.0", "--angular-z", "-0.15",
                "--duration", "2.0", "--rate-hz", "20",
                "--acknowledge-motion", MODULE.MOTION_ACK,
                "--kernel-audit-artifact", audit,
            ]
            with contextlib.redirect_stdout(
                    stdout), contextlib.redirect_stderr(stderr):
                status = MODULE.main(
                    argv,
                    runner=FakeRunner(callback),
                    allow_frozen_motion_for_tests=True,
                )
        self.assertEqual(status, 1)
        self.assertNotIn("motion completed", stdout.getvalue())
        self.assertIn("motion failed:", stderr.getvalue())


class MotionTerminalTransitionTest(unittest.TestCase):
    @staticmethod
    def successful_motion_callback(argv, timeout):
        return successful_callback(argv, timeout)

    def test_discovery_prime_subscription_safe_zero_then_nonzero_order(self):
        runner = FakeRunner(self.successful_motion_callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                MODULE.RotationHarness(runner, store).motion(
                    motion_args(clean_audit(directory)))
                state = store.load()
        calls = [" ".join(call[0]) for call in runner.calls]
        prime = next(
            index for index, call in enumerate(calls)
            if publishes_twist(call, 0.0))
        subscription = next(
            index for index, call in enumerate(calls)
            if "ros2 topic info --verbose /cmd_vel/test" in call)
        final_safe_zero = next(
            index for index, call in enumerate(calls)
            if index > subscription and "rotation_harness_zero_check" in call)
        nonzero = next(
            index for index, call in enumerate(calls)
            if publishes_twist(call, -0.15))
        self.assertEqual(state["status"], "motion_complete")
        self.assertLess(prime, subscription)
        self.assertLess(subscription, final_safe_zero)
        self.assertLess(final_safe_zero, nonzero)

    def test_publisher_timing_and_delivery_evidence_are_persisted(self):
        runner = FakeRunner(self.successful_motion_callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                MODULE.RotationHarness(runner, store).motion(
                    motion_args(clean_audit(directory)))
                state = store.load()
                publisher_path = (
                    Path(directory) / MODULE.MOTION_PUBLISHER_EVIDENCE_NAME)
                delivery_path = (
                    Path(directory) / MODULE.MOTION_DELIVERY_EVIDENCE_NAME)
                publisher = json.loads(publisher_path.read_text())
                delivery = json.loads(delivery_path.read_text())
        motion_command = next(
            " ".join(argv) for argv, _ in runner.calls
            if "ROTATION_TWIST_PUBLISHER_ID" in " ".join(argv) and
            "--command-type motion" in " ".join(argv))
        self.assertEqual(publisher["actual_publish_count"], 40)
        self.assertEqual(publisher["timing"]["window_duration_s"], 2.0)
        self.assertEqual(publisher["timing"]["publish_span_s"], 1.95)
        self.assertIn(
            "--recorder-qos-override-path "
            + MODULE.ROSBAG_QOS_OVERRIDE_CONTAINER_PATH,
            motion_command)
        self.assertIn(
            "--recorder-qos-override-sha256 "
            + MODULE.ROSBAG_QOS_OVERRIDE_SHA256,
            motion_command)
        self.assertEqual(
            delivery["topics"]["/cmd_vel/test"]["nonzero_count"], 40)
        self.assertEqual(
            state["motion_publisher"]["evidence"], publisher)
        self.assertEqual(state["motion_delivery"]["evidence"], delivery)
        self.assertEqual(
            len(state["motion_publisher"]["sha256"]), 64)

    def test_every_allowed_motion_payload_is_published_only_after_gates(self):
        def successful_allowed_motion(argv, timeout):
            text = " ".join(argv)
            if (
                    "D455_PUBLISHER_FILE_READ" in text and
                    "/tmp/d455-motion-publisher-" in text and
                    ".json" in text):
                return MODULE.CommandResult(
                    0, json.dumps(
                        publisher_evidence(angular_z=angular_z),
                        sort_keys=True) + "\n")
            if MODULE.MOTION_DELIVERY_MARKER in text:
                return marker_result(
                    MODULE.MOTION_DELIVERY_MARKER,
                    delivery_evidence(angular_z=angular_z))
            return successful_callback(argv, timeout)

        for angular_z in sorted(MODULE.ALLOWED_ANGULAR_Z):
            with self.subTest(angular_z=angular_z):
                runner = FakeRunner(successful_allowed_motion)
                with tempfile.TemporaryDirectory() as directory:
                    with MODULE.StateStore(directory) as store:
                        store.create(base_state())
                        args = motion_args(clean_audit(directory))
                        args.angular_z = angular_z
                        MODULE.RotationHarness(runner, store).motion(args)
                calls = [" ".join(argv) for argv, _ in runner.calls]
                diagnostic = next(
                    index for index, call in enumerate(calls)
                    if "rotation_harness_diagnostic_check" in call)
                safe_zero = next(
                    index for index, call in enumerate(calls)
                    if "rotation_harness_zero_check" in call)
                motion_calls = [
                    (index, call) for index, call in enumerate(calls)
                    if any(publishes_twist(call, value)
                           for value in MODULE.ALLOWED_ANGULAR_Z)]
                self.assertEqual(len(motion_calls), 1)
                motion_index, motion_call = motion_calls[0]
                self.assertTrue(publishes_twist(motion_call, angular_z))
                self.assertLess(diagnostic, safe_zero)
                self.assertLess(safe_zero, motion_index)

    def test_terminal_state_save_failure_invalidates_and_cleans(self):
        runner = FakeRunner(self.successful_motion_callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                original_save = store.save
                failed_once = False

                def fail_motion_complete(state):
                    nonlocal failed_once
                    if state["status"] == "motion_complete" and not failed_once:
                        failed_once = True
                        raise OSError("injected terminal state-save failure")
                    return original_save(state)

                store.save = fail_motion_complete
                harness = MODULE.RotationHarness(runner, store)
                stdout = io.StringIO()
                with contextlib.redirect_stdout(stdout), self.assertRaisesRegex(
                        MODULE.HarnessError, "terminal state-save failure"):
                    harness.motion(motion_args(clean_audit(directory)))
                state = store.load()
                events = store.events_path.read_text()
                partial = (Path(directory) / "partial-robot-bag").is_dir()
        calls = [" ".join(call[0]) for call in runner.calls]
        self.assertEqual(state["status"], "invalid")
        self.assertTrue(partial)
        self.assertEqual(sum("RECORDER_REAPED" in call for call in calls), 2)
        self.assertGreaterEqual(
            sum(publishes_twist(call, 0.0) for call in calls),
            3)
        self.assertIn("motion_terminal_transition_failed", events)
        self.assertNotIn("motion completed", stdout.getvalue())

    def test_terminal_event_failure_invalidates_and_never_completes(self):
        runner = FakeRunner(self.successful_motion_callback)
        status_at_completion_event = []
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                original_event = store.event

                def fail_completion(name, **fields):
                    if name == "motion_stage_completed":
                        status_at_completion_event.append(store.load()["status"])
                        return False
                    return original_event(name, **fields)

                store.event = fail_completion
                harness = MODULE.RotationHarness(runner, store)
                stdout = io.StringIO()
                with contextlib.redirect_stdout(stdout), self.assertRaisesRegex(
                        MODULE.HarnessError, "motion-stage completion"):
                    harness.motion(motion_args(clean_audit(directory)))
                state = store.load()
                events = store.events_path.read_text()
        calls = [" ".join(call[0]) for call in runner.calls]
        self.assertEqual(state["status"], "invalid")
        self.assertEqual(status_at_completion_event, ["motion_completing"])
        self.assertEqual(sum("RECORDER_REAPED" in call for call in calls), 2)
        self.assertNotIn('"event":"motion_stage_completed"', events)
        self.assertIn("motion_terminal_transition_failed", events)
        self.assertNotIn("motion completed", stdout.getvalue())

    def test_event_log_failure_prevents_motion_but_not_zero_or_cleanup(self):
        runner = FakeRunner(successful_callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                original_event = store.event

                def fail_intent(name, **fields):
                    if name == "motion_intent":
                        return False
                    return original_event(name, **fields)

                store.event = fail_intent
                harness = MODULE.RotationHarness(runner, store)
                with self.assertRaisesRegex(MODULE.HarnessError, "motion intent"):
                    harness.motion(motion_args(clean_audit(directory)))
        joined = [" ".join(call[0]) for call in runner.calls]
        self.assertFalse(any(publishes_twist(call, -0.15) for call in joined))
        self.assertTrue(any(publishes_twist(call, 0.0) for call in joined))
        self.assertEqual(sum("RECORDER_REAPED" in call for call in joined), 2)


class PrepareDiscoveryZeroPublisherTest(unittest.TestCase):
    @staticmethod
    def callback_with_recorders(overrides=None):
        identities = iter([
            "ROTATION_RECORDER_RECEIPT 101 101 101 1001 726f626f74\n",
            "ROTATION_RECORDER_RECEIPT 202 202 202 2002 696d75\n",
        ])

        def callback(argv, timeout):
            text = " ".join(argv)
            if overrides is not None:
                overridden = overrides(argv, timeout)
                if overridden is not None:
                    return overridden
            if is_recorder_identity_probe(text):
                return MODULE.CommandResult(0, next(identities))
            return successful_callback(argv, timeout)

        return callback

    def test_prepare_starts_qos_pinned_recorder_before_zero_publisher_and_proves_exact_count(self):
        runner = FakeRunner(self.callback_with_recorders())
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                MODULE.RotationHarness(runner, store).prepare(prepare_args())
                state = store.load()
                events = store.events_path.read_text()
        calls = [" ".join(argv) for argv, _ in runner.calls]
        robot_recorder = next(
            index for index, call in enumerate(calls)
            if "ros2 bag record" in call and "--output /tmp/rbag" in call)
        publisher_start = next(
            index for index, call in enumerate(calls)
            if "ROTATION_TWIST_PUBLISHER_ID" in call and
            "--command-type prepare_zero" in call)
        topic_proof = next(
            index for index, call in enumerate(calls)
            if "ROTATION_PREPARE_TOPIC_COUNT" in call)
        imu_recorder = next(
            index for index, call in enumerate(calls)
            if "ros2 bag record" in call and "--output /tmp/ibag" in call)
        prepare_zero_check = next(
            index for index, call in enumerate(calls)
            if "rotation_harness_zero_check" in call)
        self.assertEqual(state["status"], "prepared")
        prepare_record = state["prepare_zero_publisher"]
        self.assertEqual(prepare_record["command_type"], "prepare_zero")
        self.assertEqual(prepare_record["child_exit_status"], 0)
        self.assertEqual(
            prepare_record["evidence"]["publisher_qos"],
            MODULE.CMD_VEL_TEST_QOS)
        self.assertEqual(
            prepare_record["evidence"]["actual_publish_count"],
            MODULE.ZERO_MESSAGE_COUNT)
        robot_spec = state["recorders"]["robot"]
        self.assertEqual(
            robot_spec["cmd_vel_test_qos"], MODULE.CMD_VEL_TEST_QOS)
        self.assertEqual(
            robot_spec["qos_override_path"],
            MODULE.ROSBAG_QOS_OVERRIDE_CONTAINER_PATH)
        self.assertEqual(
            robot_spec["qos_override_sha256"],
            MODULE.ROSBAG_QOS_OVERRIDE_SHA256)
        self.assertLess(robot_recorder, publisher_start)
        self.assertLess(publisher_start, topic_proof)
        self.assertLess(topic_proof, imu_recorder)
        self.assertLess(imu_recorder, prepare_zero_check)
        recorder_command = calls[robot_recorder]
        self.assertIn(
            "--qos-profile-overrides-path "
            + MODULE.ROSBAG_QOS_OVERRIDE_CONTAINER_PATH,
            recorder_command)
        self.assertIn('"event":"prepare_zero_recorded"', events)
        self.assertIn('"event":"prepare_safe_zero_verified"', events)
        topic_call, topic_timeout = next(
            (" ".join(argv), timeout) for argv, timeout in runner.calls
            if "ROTATION_PREPARE_TOPIC_COUNT" in " ".join(argv))
        self.assertEqual(
            topic_timeout, MODULE.PREPARE_TOPIC_EVIDENCE_COMMAND_TIMEOUT_SECONDS)
        self.assertIn(
            f"timeout {MODULE.PREPARE_TOPIC_EVIDENCE_SHELL_TIMEOUT_SECONDS}s",
            topic_call)
        self.assertIn(
            str(MODULE.PREPARE_TOPIC_EVIDENCE_TIMEOUT_SECONDS), topic_call)
        self.assertIn(
            str(MODULE.ZERO_MESSAGE_COUNT), topic_call)

    def test_recorders_use_detached_foreground_exec_with_external_reap_owner(self):
        runner = FakeRunner(self.callback_with_recorders())
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                MODULE.RotationHarness(runner, store).prepare(prepare_args())
                state = store.load()
        detached = [
            " ".join(argv) for argv, _ in runner.calls
            if argv[:3] == ["docker", "exec", "--detach"]]
        self.assertEqual(len(detached), 2)
        for command in detached:
            self.assertIn("setsid bash -c", command)
            self.assertIn("wrapper_pid=$!", command)
            self.assertIn('wait "$wrapper_pid"', command)
            self.assertIn('wait "$recorder"', command)
            self.assertIn("write_exit", command)
            self.assertIn("wrapper_pid=$!", command)
        for spec in state["recorders"].values():
            self.assertEqual(
                spec["launch_mode"], "detached_foreground_docker_exec")
            self.assertEqual(
                spec["wrapper_reap_owner"], "docker_exec_parent")
            self.assertTrue(spec["token"].startswith("d455-recorder-"))
            self.assertTrue(spec["exit_path"].endswith(".exit"))

    def test_prepare_publisher_constructs_exact_zero_with_package_helper(self):
        runner = FakeRunner(self.callback_with_recorders())
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                MODULE.RotationHarness(runner, store).prepare(prepare_args())
        start_calls = [
            " ".join(argv) for argv, _ in runner.calls
            if "ROTATION_TWIST_PUBLISHER_ID" in " ".join(argv) and
            "--command-type prepare_zero" in " ".join(argv)]
        self.assertEqual(len(start_calls), 1)
        command = start_calls[0]
        self.assertIn(MODULE.PUBLISHER_CONTAINER_PATH, command)
        self.assertIn("--command-type prepare_zero", command)
        payload = MODULE.twist_yaml(0.0)
        self.assertIn(payload, command)
        self.assertEqual(
            json.loads(payload),
            {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            })
        self.assertIn(f"--count {MODULE.ZERO_MESSAGE_COUNT}", command)
        self.assertIn(f"--rate-hz {MODULE.ZERO_RATE_HZ}", command)
        self.assertIn(
            "--recorder-qos-override-path "
            + MODULE.ROSBAG_QOS_OVERRIDE_CONTAINER_PATH,
            command)
        self.assertIn(
            "--recorder-qos-override-sha256 "
            + MODULE.ROSBAG_QOS_OVERRIDE_SHA256,
            command)
        self.assertNotIn("ros2 topic pub", command)

    def test_prepare_and_recorder_share_one_explicit_qos_contract(self):
        policy = {
            "history": "keep_last",
            "depth": 1,
            "reliability": "reliable",
            "durability": "volatile",
        }
        self.assertEqual(MODULE.CMD_VEL_TEST_QOS, policy)
        self.assertEqual(
            MODULE.ROSBAG_QOS_OVERRIDE_HOST_PATH.read_text(
                encoding="utf-8"),
            "/cmd_vel/test:\n"
            "  history: keep_last\n"
            "  depth: 1\n"
            "  reliability: reliable\n"
            "  durability: volatile\n")
        self.assertEqual(
            hashlib.sha256(
                MODULE.ROSBAG_QOS_OVERRIDE_HOST_PATH.read_bytes()).hexdigest(),
            MODULE.ROSBAG_QOS_OVERRIDE_SHA256)
        arbiter_source = (
            MODULE_PATH.parents[2] /
            "command_arbiter" / "src" / "command_arbiter.cpp").read_text(
                encoding="utf-8")
        self.assertIn(
            "this->create_subscription<geometry_msgs::msg::Twist>",
            arbiter_source)
        self.assertIn("rclcpp::QoS(1).reliable()", arbiter_source)

    def test_missing_prepare_delivery_fails_closed_and_runs_same_zero_path(self):
        def absent_topic(argv, _timeout):
            if "ROTATION_PREPARE_TOPIC_COUNT" in " ".join(argv):
                return MODULE.CommandResult(
                    1, "",
                    "active bag has incomplete /cmd_vel/test messages")
            return None

        runner = FakeRunner(self.callback_with_recorders(absent_topic))
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                with self.assertRaisesRegex(
                        MODULE.HarnessError,
                        "active bag has incomplete /cmd_vel/test"):
                    MODULE.RotationHarness(runner, store).prepare(prepare_args())
                state = store.load()
                events = store.events_path.read_text()
        calls = [" ".join(argv) for argv, _ in runner.calls]
        self.assertEqual(state["status"], "invalid")
        self.assertTrue(any(
            "--command-type cleanup_zero" in call and
            MODULE.PUBLISHER_CONTAINER_PATH in call
            for call in calls))
        self.assertFalse(any("--output /tmp/ibag" in call for call in calls))
        self.assertIn('"event":"zero_publisher_completed"', events)
        self.assertNotIn("prepare_completed", events)

    def test_stdlib_sqlite_proof_requires_the_exact_recorded_count(self):
        with tempfile.TemporaryDirectory() as directory:
            database = Path(directory) / "bag_0.db3"
            connection = sqlite3.connect(database)
            connection.executescript(
                "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT);"
                "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER);"
                "INSERT INTO topics(id, name) VALUES(1, '/cmd_vel/test');")
            connection.executemany(
                "INSERT INTO messages(id, topic_id) VALUES(?, 1)",
                [(index,) for index in range(1, 21)])
            connection.commit()
            connection.close()
            command = [
                "python3", "-c", MODULE.PREPARE_TOPIC_EVIDENCE_CHECK,
                directory, "/cmd_vel/test", "0.2", "20"]
            present = subprocess.run(
                command, text=True, capture_output=True, check=False, timeout=1)
            self.assertEqual(present.returncode, 0)
            self.assertIn("ROTATION_PREPARE_TOPIC_COUNT 20", present.stdout)

            connection = sqlite3.connect(database)
            connection.execute("DELETE FROM messages WHERE id = 20")
            connection.commit()
            connection.close()
            absent = subprocess.run(
                command, text=True, capture_output=True, check=False, timeout=1)
            self.assertNotEqual(absent.returncode, 0)
            self.assertIn("expected=20 count=19", absent.stderr)


class PrepareFailureTest(unittest.TestCase):
    def test_recorder_identity_is_durable_before_watchdog_ack(self):
        identities = iter([
            "ROTATION_RECORDER_RECEIPT 101 101 101 1001 726f626f74\n",
            "ROTATION_RECORDER_RECEIPT 202 202 202 2002 696d75\n",
        ])
        observed = []
        with tempfile.TemporaryDirectory() as directory:
            state_path = Path(directory) / MODULE.STATE_NAME

            def callback(argv, timeout):
                text = " ".join(argv)
                if is_recorder_identity_probe(text):
                    return MODULE.CommandResult(0, next(identities))
                if (
                        argv[:2] == ["docker", "exec"] and
                        "touch /tmp/d455-rotation-recorder-" in text):
                    state = json.loads(state_path.read_text())
                    kind = "robot" if argv[2] == "robot" else "imu"
                    observed.append((kind, "pid" in state["recorders"][kind]))
                return successful_callback(argv, timeout)

            runner = FakeRunner(callback)
            args = argparse.Namespace(
                trial_id="trial", robot_container="robot", imu_container="imu",
                robot_bag="/tmp/rbag", imu_bag="/tmp/ibag",
                robot_log="/tmp/r.log", imu_log="/tmp/i.log",
                robot_setup=["/opt/ros/humble/setup.bash"],
                imu_setup=["/opt/ros/humble/setup.bash"])
            with MODULE.StateStore(directory) as store:
                MODULE.RotationHarness(runner, store).prepare(args)
        self.assertEqual(observed, [("robot", True), ("imu", True)])

    def test_prepare_keyboard_interrupt_zeroes_and_cleans_started_recorder(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if is_recorder_identity_probe(text) and argv[2] == "robot":
                return MODULE.CommandResult(
                    0, "ROTATION_RECORDER_RECEIPT "
                    "101 101 101 1001 726f626f74\n")
            if is_recorder_identity_probe(text) and argv[2] == "imu":
                raise KeyboardInterrupt()
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        args = argparse.Namespace(
            trial_id="trial", robot_container="robot", imu_container="imu",
            robot_bag="/tmp/rbag", imu_bag="/tmp/ibag",
            robot_log="/tmp/r.log", imu_log="/tmp/i.log",
            robot_setup=["/opt/ros/humble/setup.bash"],
            imu_setup=["/opt/ros/humble/setup.bash"])
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                harness = MODULE.RotationHarness(runner, store)
                with self.assertRaises(KeyboardInterrupt):
                    harness.prepare(args)
                state = store.load()
        joined = [" ".join(call[0]) for call in runner.calls]
        self.assertEqual(state["status"], "invalid")
        self.assertTrue(any(publishes_twist(call, 0.0) for call in joined))
        self.assertEqual(sum("RECORDER_REAPED" in call for call in joined), 1)
        self.assertEqual(
            sum("RECORDER_PENDING_REAPED" in call for call in joined), 1)
        self.assertTrue(any(
            "d455-cleanup-zero-publisher-" in call for call in joined))

    def test_current_and_earlier_recorders_are_cleaned_on_second_verify_failure(self):
        identities = iter([
            "ROTATION_RECORDER_RECEIPT 101 101 101 1001 726f626f74\n",
            "ROTATION_RECORDER_RECEIPT 202 202 202 2002 696d75\n",
        ])

        def callback(argv, _timeout):
            text = " ".join(argv)
            if is_recorder_identity_probe(text):
                return MODULE.CommandResult(0, next(identities))
            if argv[:2] == ["docker", "exec"] and argv[2] == "imu" and (
                    "actual_start" in text and "d455-recorder-" not in text):
                return MODULE.CommandResult(1, "", "identity mismatch")
            return successful_callback(argv, _timeout)

        runner = FakeRunner(callback)
        args = argparse.Namespace(
            trial_id="trial", robot_container="robot", imu_container="imu",
            robot_bag="/tmp/rbag", imu_bag="/tmp/ibag",
            robot_log="/tmp/r.log", imu_log="/tmp/i.log",
            robot_setup=["/opt/ros/humble/setup.bash"],
            imu_setup=["/opt/ros/humble/setup.bash"])
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                harness = MODULE.RotationHarness(runner, store)
                with self.assertRaises(MODULE.HarnessError):
                    harness.prepare(args)
                state = store.load()
                events = store.events_path.read_text()
        self.assertEqual(state["status"], "invalid")
        self.assertIn("prepare_failed", events)
        self.assertNotIn("prepare_completed", events)
        joined = [" ".join(call[0]) for call in runner.calls]
        self.assertTrue(any(
            "RECORDER_PENDING_REAPED" in call and "kill -INT" in call
            for call in joined))
        self.assertTrue(any(
            "RECORDER_REAPED" in call and "kill -INT" in call for call in joined))


class RecorderLaunchRecoveryTest(unittest.TestCase):
    @staticmethod
    def launch_state():
        state = base_state(status="preparing")
        for spec in state["recorders"].values():
            for field in (
                    *MODULE.RECORDER_IDENTITY_FIELDS,
                    "token", "exit_path", "launch_mode",
                    "wrapper_reap_owner"):
                spec.pop(field, None)
        return state

    @staticmethod
    def absent_receipt_callback(launch_result):
        def callback(argv, timeout):
            text = " ".join(argv)
            if argv[:3] == ["docker", "exec", "--detach"]:
                if isinstance(launch_result, BaseException):
                    raise launch_result
                return launch_result
            if "RECORDER_RECEIPT_ABSENT" in text:
                return MODULE.CommandResult(
                    0, "RECORDER_RECEIPT_ABSENT\n")
            if "RECORDER_PENDING_QUIESCENT" in text:
                return MODULE.CommandResult(
                    0, "RECORDER_PENDING_QUIESCENT\n")
            return successful_callback(argv, timeout)

        return callback

    @staticmethod
    def identity_failure_callback(identity_result):
        def callback(argv, timeout):
            text = " ".join(argv)
            if is_recorder_identity_probe(text):
                if isinstance(identity_result, BaseException):
                    raise identity_result
                return identity_result
            return successful_callback(argv, timeout)

        return callback

    def run_failed_start(self, callback, expected_exception):
        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                state = self.launch_state()
                store.create(state)
                spec = state["recorders"]["robot"]
                with self.assertRaises(expected_exception):
                    MODULE.RotationHarness(runner, store)._start_recorder(spec)
                persisted = store.load()["recorders"]["robot"]
                evidence_path = Path(
                    persisted["launch_cleanup_evidence_path"])
                evidence = json.loads(evidence_path.read_text())
                actual_evidence_sha256 = hashlib.sha256(
                    evidence_path.read_bytes()).hexdigest()
        calls = [" ".join(argv) for argv, _ in runner.calls]
        self.assertEqual(persisted["status"], "launch_attempt_reaped")
        self.assertEqual(
            actual_evidence_sha256,
            persisted["launch_cleanup_evidence_sha256"])
        self.assertFalse(any(
            publishes_twist(call, value)
            for call in calls for value in MODULE.ALLOWED_ANGULAR_Z))
        return persisted, evidence, calls

    def test_detached_launch_nonzero_is_durably_recovered(self):
        persisted, evidence, _ = self.run_failed_start(
            self.absent_receipt_callback(
                MODULE.CommandResult(70, "", "detached launch failed")),
            MODULE.HarnessError)
        self.assertEqual(
            evidence["receipt_status"], "absent_before_side_effects")
        self.assertEqual(evidence["quiescence_poll_count"], 10)
        self.assertIn("launch_registered_at", persisted)

    def test_detached_launch_timeout_is_durably_recovered(self):
        _, evidence, _ = self.run_failed_start(
            self.absent_receipt_callback(
                MODULE.HarnessError("command timed out after 5s")),
            MODULE.HarnessError)
        self.assertEqual(
            evidence["receipt_status"], "absent_before_side_effects")

    def test_detached_launch_interrupt_is_durably_recovered(self):
        _, evidence, _ = self.run_failed_start(
            self.absent_receipt_callback(KeyboardInterrupt()),
            KeyboardInterrupt)
        self.assertEqual(
            evidence["receipt_status"], "absent_before_side_effects")

    def test_identity_scan_nonzero_reaps_receipted_wrapper(self):
        persisted, evidence, calls = self.run_failed_start(
            self.identity_failure_callback(
                MODULE.CommandResult(71, "", "identity scan failed")),
            MODULE.HarnessError)
        self.assertEqual(evidence["receipt_status"], "pinned_and_reaped")
        self.assertEqual(evidence["receipt_identity"]["pid"], 101)
        self.assertTrue(evidence["log_path"].endswith(
            "robot-recorder-launch.log"))
        self.assertTrue(any("RECORDER_PENDING_REAPED" in call for call in calls))
        self.assertEqual(
            persisted["wrapper_reap_owner"], "docker_exec_parent")

    def test_identity_scan_timeout_reaps_receipted_wrapper(self):
        _, evidence, _ = self.run_failed_start(
            self.identity_failure_callback(
                MODULE.HarnessError("command timed out after 8s")),
            MODULE.HarnessError)
        self.assertEqual(evidence["receipt_status"], "pinned_and_reaped")

    def test_identity_scan_interrupt_reaps_receipted_wrapper(self):
        _, evidence, _ = self.run_failed_start(
            self.identity_failure_callback(KeyboardInterrupt()),
            KeyboardInterrupt)
        self.assertEqual(evidence["receipt_status"], "pinned_and_reaped")

    def test_unproven_launch_cleanup_blocks_abort(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if argv[:3] == ["docker", "exec", "--detach"]:
                return MODULE.CommandResult(70, "", "detached launch failed")
            if "RECORDER_RECEIPT_ABSENT" in text:
                return MODULE.CommandResult(
                    0, "RECORDER_RECEIPT_ABSENT\n")
            if (
                    "token=d455-recorder-" in text and
                    "pgids=''" in text):
                return MODULE.CommandResult(
                    44, "", "owned recorder wrapper zombie pid=101 ppid=1")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                state = self.launch_state()
                store.create(state)
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "pending launch cleanup failed"):
                    MODULE.RotationHarness(runner, store)._start_recorder(
                        state["recorders"]["robot"])
                self.assertEqual(
                    store.load()["recorders"]["robot"]["status"],
                    "launch_cleanup_unproven")
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "owned recorder wrapper zombie"):
                    MODULE.RotationHarness(runner, store).abort()
                failed = store.load()
        self.assertEqual(failed["status"], "invalid")
        self.assertFalse(any(
            publishes_twist(" ".join(argv), value)
            for argv, _ in runner.calls
            for value in MODULE.ALLOWED_ANGULAR_Z))

    def test_abort_recovers_durably_registered_pending_attempt(self):
        state = self.launch_state()
        robot = state["recorders"]["robot"]
        robot.update({
            "token": "d455-recorder-persisted",
            "startup_ack": "/tmp/persisted.ack",
            "exit_path": "/tmp/persisted.exit",
            "receipt_path": "/tmp/persisted.receipt",
            "launch_mode": "detached_foreground_docker_exec",
            "wrapper_reap_owner": "docker_exec_parent",
            "status": "launch_registered",
            "launch_registered_at": MODULE.utc_now(),
        })
        runner = FakeRunner(self.absent_receipt_callback(
            MODULE.CommandResult(0, "")))
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(state)
                MODULE.RotationHarness(runner, store).abort()
                completed = store.load()
                events = store.events_path.read_text()
        self.assertEqual(completed["status"], "aborted")
        self.assertEqual(
            completed["recorders"]["robot"]["status"],
            "launch_attempt_reaped")
        self.assertIn("recorder_launch_attempt_reaped", events)
        self.assertFalse(any(
            publishes_twist(" ".join(argv), value)
            for argv, _ in runner.calls
            for value in MODULE.ALLOWED_ANGULAR_Z))


class RecorderIdentityTest(unittest.TestCase):
    @staticmethod
    def run_completion_census(exit_exists=True, log_exists=True):
        spec = base_state()["recorders"]["robot"]
        spec.update({
            "pid": 999991,
            "pgid": 999991,
            "token": "d455-recorder-terminal-census",
        })
        with tempfile.TemporaryDirectory() as directory:
            exit_path = Path(directory) / "recorder.exit"
            log_path = Path(directory) / "recorder.log"
            if exit_exists:
                exit_path.write_text("0\n", encoding="utf-8")
            if log_exists:
                log_path.write_text("recorder log\n", encoding="utf-8")
            spec["exit_path"] = str(exit_path)
            spec["log_path"] = str(log_path)
            setup = "\n".join([
                "ps() { :; }",
                "sleep() { :; }",
            ])
            result = subprocess.run(
                ["bash", "-c", setup + "\n" +
                 MODULE.RotationHarness._recorder_completion_body(
                     spec, "RECORDER_ABSENCE_VERIFIED")],
                text=True, capture_output=True, check=False, timeout=2)
        return result

    @staticmethod
    def run_stop_decision(
            member_rows, state="Z", command_hex="", starttime="1001",
            reap_after_signal=True, ppid=1):
        spec = base_state()["recorders"]["robot"]
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "kill-called"
            exit_path = Path(directory) / "recorder.exit"
            log_path = Path(directory) / "recorder.log"
            exit_path.write_text("0\n", encoding="utf-8")
            log_path.write_text("recorder log\n", encoding="utf-8")
            spec["exit_path"] = str(exit_path)
            spec["log_path"] = str(log_path)
            environment = os.environ.copy()
            environment.update({
                "FIXTURE_ROWS": member_rows,
                "KILL_MARKER": str(marker),
                "REAP_AFTER_SIGNAL": "1" if reap_after_signal else "0",
            })
            setup = "\n".join([
                "set -eo pipefail",
                "pid=101",
                "pgid=101",
                f"exit_path={shlex.quote(str(exit_path))}",
                f"log_path={shlex.quote(str(log_path))}",
                "actual_pgid=101",
                "actual_sid=101",
                f"actual_start={starttime}",
                f"actual_state={state}",
                f"actual_ppid={ppid}",
                f"actual_cmd={command_hex}",
                "ps() {",
                "  if [ \"$REAP_AFTER_SIGNAL\" = 1 ] && "
                "[ -e \"$KILL_MARKER\" ]; then return; fi",
                "  printf '%s' \"$FIXTURE_ROWS\"",
                "}",
                "kill() { printf called > \"$KILL_MARKER\"; }",
                "sleep() { :; }",
            ])
            result = subprocess.run(
                ["bash", "-c", setup + "\n" +
                 MODULE.RotationHarness._stop_decision_body(spec)],
                text=True, capture_output=True, check=False, env=environment,
                timeout=2)
            kill_called = marker.exists()
        return result, kill_called

    def test_terminal_recorder_census_defines_all_inputs_before_use(self):
        spec = base_state()["recorders"]["robot"]
        body = MODULE.RotationHarness._recorder_completion_body(
            spec, "RECORDER_ABSENCE_VERIFIED")
        definitions = {
            "pid=": '"/proc/$pid"',
            "pgid=": 'wanted="$pgid"',
            "token=": '*"$token"*',
            "exit_path=": 'test -n "$exit_path"',
            "log_path=": 'test -n "$log_path"',
        }
        for definition, first_use in definitions.items():
            with self.subTest(definition=definition):
                self.assertIn(definition, body)
                self.assertIn(first_use, body)
                self.assertLess(
                    body.index(definition), body.index(first_use))
        self.assertTrue(body.startswith("set -eo pipefail\n"))

    def test_completed_recorder_census_accepts_pinned_exit_and_log(self):
        result = self.run_completion_census()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertRegex(
            result.stdout,
            r"^RECORDER_ABSENCE_VERIFIED 0 [0-9a-f]{64}\n$")

    def test_completed_recorder_census_rejects_missing_exit_or_log(self):
        for exit_exists, log_exists in ((False, True), (True, False)):
            with self.subTest(
                    exit_exists=exit_exists, log_exists=log_exists):
                result = self.run_completion_census(
                    exit_exists=exit_exists, log_exists=log_exists)
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn(
                    "RECORDER_ABSENCE_VERIFIED", result.stdout)

    def test_exact_group_classifier_accepts_only_empty_owned_group(self):
        for rows in ("", "202 S\n", "202 Z\n"):
            with self.subTest(rows=rows):
                result = subprocess.run(
                    ["awk", "-v", "wanted=101", MODULE.GROUP_EMPTY_AWK],
                    input=rows, text=True, capture_output=True, check=False)
                self.assertEqual(result.returncode, 0)

    def test_exact_group_classifier_rejects_live_zombie_mixed_or_unknown_member(self):
        for rows in (
                "101 S\n", "101 Z\n", "101 Z\n101 S\n", "101 ?\n"):
            with self.subTest(rows=rows):
                result = subprocess.run(
                    ["awk", "-v", "wanted=101", MODULE.GROUP_EMPTY_AWK],
                    input=rows, text=True, capture_output=True, check=False)
                self.assertNotEqual(result.returncode, 0)

    def test_pid_reuse_identity_mismatch_is_rejected(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if "actual_start" in text and "test \"$actual_start\"" in text:
                return MODULE.CommandResult(1, "", "starttime mismatch")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                harness = MODULE.RotationHarness(runner, store)
                with self.assertRaisesRegex(MODULE.HarnessError, "starttime mismatch"):
                    harness._verify_recorder(base_state()["recorders"]["robot"])

    def test_stop_command_requires_owned_identity_and_empty_process_group(self):
        runner = FakeRunner(successful_callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                harness = MODULE.RotationHarness(runner, store)
                spec = base_state()["recorders"]["robot"]
                outcome = harness._stop_recorder(
                    spec, allow_missing=False)
                cleanup = json.loads(
                    (Path(directory) / "robot-recorder-cleanup.json").read_text())
                log = (
                    Path(directory) / "robot-recorder.log").read_text()
        command = "\n".join(" ".join(call[0]) for call in runner.calls)
        self.assertEqual(outcome, "reaped")
        self.assertEqual(spec["status"], "reaped")
        self.assertEqual(spec["child_exit_status"], 0)
        self.assertEqual(cleanup["outcome"], "reaped")
        self.assertEqual(cleanup["child_exit_status"], 0)
        self.assertEqual(log, "recorder log\n")
        self.assertIn("actual_start", command)
        self.assertIn("actual_cmd", command)
        self.assertIn(MODULE.GROUP_EMPTY_AWK, command)
        self.assertIn('test ! -e "/proc/$pid"', command)
        self.assertIn("kill -KILL", command)

    def test_recorder_wrapper_zombie_is_rejected_without_signalling(self):
        result, kill_called = self.run_stop_decision(
            "101 Zs\n101 Z\n", ppid=55)
        self.assertEqual(result.returncode, 44)
        self.assertIn("ppid=55", result.stderr)
        self.assertFalse(kill_called)

    def test_pid1_owned_recorder_zombie_is_rejected_without_signalling(self):
        result, kill_called = self.run_stop_decision(
            "101 Zs\n101 Z\n", ppid=1)
        self.assertEqual(result.returncode, 44)
        self.assertIn("ppid=1", result.stderr)
        self.assertFalse(kill_called)

    def test_zombie_leader_with_live_recorder_member_is_rejected(self):
        result, kill_called = self.run_stop_decision("101 S\n")
        self.assertEqual(result.returncode, 44)
        self.assertFalse(kill_called)

    def test_live_recorder_remaining_after_bounded_signals_is_rejected(self):
        result, kill_called = self.run_stop_decision(
            "101 S\n", state="S", command_hex="726f626f74",
            reap_after_signal=False)
        self.assertEqual(result.returncode, 42)
        self.assertTrue(kill_called)

    def test_mixed_live_and_zombie_group_is_rejected(self):
        result, kill_called = self.run_stop_decision("101 Z\n101 S\n")
        self.assertEqual(result.returncode, 44)
        self.assertFalse(kill_called)

    def test_stop_rejects_pid_reuse_before_signalling(self):
        result, kill_called = self.run_stop_decision(
            "101 S\n", state="S", command_hex="726f626f74", starttime="9999")
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(kill_called)

    def test_empty_or_unreadable_live_cmdline_is_rejected(self):
        result, kill_called = self.run_stop_decision(
            "101 S\n", state="S", command_hex="")
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(kill_called)

    def test_live_cmdline_mismatch_cannot_reach_signal(self):
        result, kill_called = self.run_stop_decision(
            "101 S\n", state="S", command_hex="77726f6e67")
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(kill_called)

    def test_identity_shell_and_stop_shell_are_strict(self):
        spec = base_state()["recorders"]["robot"]
        self.assertTrue(
            MODULE.RotationHarness._identity_body(spec).startswith(
                "set -eo pipefail\n"))
        runner = FakeRunner(successful_callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                MODULE.RotationHarness(runner, store)._stop_recorder(
                    spec, allow_missing=False)
        stop_shell = next(
            " ".join(argv) for argv, _ in runner.calls
            if "RECORDER_REAPED" in argv[-1])
        self.assertIn("set -eo pipefail", stop_shell)

    def test_start_token_cleanup_requires_strictly_empty_group(self):
        runner = FakeRunner(successful_callback)
        spec = base_state()["recorders"]["robot"]
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                MODULE.RotationHarness(runner, store)._cleanup_start_token(
                    spec, "abc123")
        cleanup_shell = next(
            argv[-1] for argv, _ in runner.calls if "token=d455-recorder-abc123" in argv[-1])
        self.assertTrue(cleanup_shell.startswith("set -eo pipefail\n"))
        self.assertIn(MODULE.GROUP_EMPTY_AWK, cleanup_shell)
        self.assertLess(
            cleanup_shell.index("group_empty && {"),
            cleanup_shell.index('kill -INT -- "-$pgid"'))

    def test_missing_expected_recorder_is_failure_not_stopped_event(self):
        state = base_state()
        del state["recorders"]["imu"]["pid"]
        runner = FakeRunner(successful_callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(state)
                harness = MODULE.RotationHarness(runner, store)
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "incomplete imu"):
                    harness.abort()
                events = [
                    json.loads(line) for line in store.events_path.read_text().splitlines()
                ]
        imu_stopped = [
            event for event in events
            if event["event"] == "recorder_stopped" and event.get("kind") == "imu"]
        imu_failed = [
            event for event in events
            if event["event"] == "recorder_stop_failed" and event.get("kind") == "imu"]
        self.assertEqual(imu_stopped, [])
        self.assertEqual(len(imu_failed), 1)

    def test_never_started_recorder_is_not_tolerated_in_prepared_state(self):
        state = base_state()
        for field in MODULE.RECORDER_IDENTITY_FIELDS:
            del state["recorders"]["imu"][field]
        for field in (
                "token", "exit_path", "launch_mode",
                "wrapper_reap_owner"):
            del state["recorders"]["imu"][field]
        runner = FakeRunner(successful_callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(state)
                with self.assertRaisesRegex(
                        MODULE.HarnessError,
                        "missing imu recorder identity"):
                    MODULE.RotationHarness(runner, store).abort()
                failed = store.load()
        self.assertEqual(failed["status"], "invalid")


class RecorderCleanupPropagationTest(unittest.TestCase):
    @staticmethod
    def zombie_cleanup_callback(argv, timeout):
        text = " ".join(argv)
        if "RECORDER_REAPED" in text and "kill -INT" in text:
            return MODULE.CommandResult(
                44, "", "owned recorder wrapper zombie pid=202 ppid=1")
        return successful_callback(argv, timeout)

    def test_abort_reports_cleanup_error_when_recorder_zombie_remains(self):
        runner = FakeRunner(self.zombie_cleanup_callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "owned recorder wrapper zombie"):
                    MODULE.RotationHarness(runner, store).abort()
                state = store.load()
                events = [
                    json.loads(line)
                    for line in store.events_path.read_text().splitlines()
                ]
        completed = next(
            event for event in events if event["event"] == "abort_completed")
        self.assertEqual(state["status"], "invalid")
        self.assertNotEqual(completed["cleanup_errors"], [])
        self.assertTrue(any(
            "owned recorder wrapper zombie" in error
            for error in completed["cleanup_errors"]))
        self.assertFalse(any(
            publishes_twist(" ".join(argv), value)
            for argv, _ in runner.calls
            for value in MODULE.ALLOWED_ANGULAR_Z))

    def test_finalize_reports_cleanup_error_when_recorder_zombie_remains(self):
        runner = FakeRunner(self.zombie_cleanup_callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(finalize_state(directory))
                post = clean_audit(directory, "post-motion-audit.txt")
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "owned recorder wrapper zombie"):
                    MODULE.RotationHarness(runner, store).finalize(
                        argparse.Namespace(kernel_audit_artifact=post))
                state = store.load()
                events = store.events_path.read_text()
        self.assertEqual(state["status"], "invalid")
        self.assertIn('"event":"recorder_stop_failed"', events)
        self.assertIn('"event":"finalize_failed"', events)
        self.assertNotIn('"event":"finalize_completed"', events)

    def test_prepare_failure_cleanup_detects_recorder_zombie(self):
        def override(argv, timeout):
            text = " ".join(argv)
            if "ROTATION_PREPARE_TOPIC_COUNT" in text:
                return MODULE.CommandResult(1, "", "prepare proof failed")
            if "RECORDER_REAPED" in text and "kill -INT" in text:
                return MODULE.CommandResult(
                    44, "", "owned recorder wrapper zombie pid=101 ppid=1")
            return None

        runner = FakeRunner(
            PrepareDiscoveryZeroPublisherTest.callback_with_recorders(
                override))
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "owned recorder wrapper zombie"):
                    MODULE.RotationHarness(runner, store).prepare(
                        prepare_args())
                state = store.load()
                events = store.events_path.read_text()
        self.assertEqual(state["status"], "invalid")
        self.assertIn('"event":"recorder_stop_failed"', events)
        self.assertIn('"event":"prepare_failed"', events)
        self.assertFalse(any(
            publishes_twist(" ".join(argv), value)
            for argv, _ in runner.calls
            for value in MODULE.ALLOWED_ANGULAR_Z))

    def test_motion_failure_cleanup_detects_recorder_zombie(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if "rotation_harness_diagnostic_check" in text:
                return MODULE.CommandResult(1, "", "pre-motion gate failed")
            return self.zombie_cleanup_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                with self.assertRaisesRegex(
                        MODULE.HarnessError, "owned recorder wrapper zombie"):
                    MODULE.RotationHarness(runner, store).motion(
                        motion_args(clean_audit(directory)))
                state = store.load()
                events = store.events_path.read_text()
        self.assertEqual(state["status"], "invalid")
        self.assertIn('"event":"recorder_stop_failed"', events)
        self.assertIn('"event":"motion_stage_failed"', events)
        self.assertFalse(any(
            publishes_twist(" ".join(argv), value)
            for argv, _ in runner.calls
            for value in MODULE.ALLOWED_ANGULAR_Z))


class PartialEvidenceTest(unittest.TestCase):
    def test_relaxed_verification_failure_still_copies_both_invalid_partial_bags(self):
        def callback(argv, timeout):
            if "ros2 bag info" in " ".join(argv):
                return MODULE.CommandResult(1, "", "metadata incomplete")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                harness = MODULE.RotationHarness(runner, store)
                errors = harness._preserve_partial_bags(base_state())
                events = store.events_path.read_text()
                robot_copied = (Path(directory) / "partial-robot-bag").is_dir()
                imu_copied = (Path(directory) / "partial-imu-bag").is_dir()
        self.assertEqual(len(errors), 2)
        self.assertTrue(robot_copied)
        self.assertTrue(imu_copied)
        self.assertIn("partial_bag_verification_failed", events)
        self.assertIn('"validity":"invalid_partial"', events)
        self.assertIn('"relaxed_verification_passed":false', events)

    def test_partial_abort_skips_never_started_imu_and_preserves_robot_only(self):
        state = base_state(status="invalid")
        state["recorders"]["imu"] = {
            "kind": "imu",
            "container": "imu",
            "setup": ["/opt/ros/humble/setup.bash"],
            "bag_path": "/tmp/imu-bag",
            "log_path": "/tmp/imu.log",
            "topics": list(MODULE.IMU_TOPICS),
        }
        runner = FakeRunner(successful_callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(state)
                MODULE.RotationHarness(runner, store).abort()
                completed = store.load()
                events = store.events_path.read_text()
                robot_partial = (
                    Path(directory) / "partial-robot-bag").is_dir()
                imu_partial = (
                    Path(directory) / "partial-imu-bag").exists()
        imu_calls = [
            argv for argv, _ in runner.calls
            if argv and argv[0] == "docker" and (
                (len(argv) > 2 and argv[1] == "inspect" and argv[-1] == "imu") or
                (len(argv) > 2 and argv[1] == "exec" and argv[2] == "imu") or
                (len(argv) > 2 and argv[1] == "cp" and
                 argv[2].startswith("imu:")))]
        self.assertEqual(completed["status"], "aborted")
        self.assertTrue(robot_partial)
        self.assertFalse(imu_partial)
        self.assertEqual(imu_calls, [])
        self.assertIn('"event":"recorder_never_started"', events)
        self.assertIn('"event":"partial_bag_not_started_skipped"', events)

    def test_started_recorder_container_identity_drift_fails_abort(self):
        def callback(argv, timeout):
            if (
                    argv[:3] == ["docker", "inspect", "-f"] and
                    argv[-1] == "imu"):
                return MODULE.CommandResult(0, f"{'c' * 64} true\n")
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state(status="invalid"))
                with self.assertRaisesRegex(
                        MODULE.HarnessError,
                        "container identity changed: imu"):
                    MODULE.RotationHarness(runner, store).abort()
                failed = store.load()
                events = store.events_path.read_text()
        self.assertEqual(failed["status"], "invalid")
        self.assertIn('"event":"recorder_stop_failed"', events)
        self.assertIn("partial_bag_preservation_failed", events)


class AbortTest(unittest.TestCase):
    def test_abort_is_idempotent_after_successful_cleanup(self):
        runner = FakeRunner(successful_callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                harness = MODULE.RotationHarness(runner, store)
                harness.abort()
                first_call_count = len(runner.calls)
                harness.abort()
                state = store.load()
        self.assertEqual(state["status"], "aborted")
        self.assertEqual(len(runner.calls), first_call_count + 2)
        terminal_calls = [
            argv[-1] for argv, _ in runner.calls
            if "RECORDER_ABSENCE_VERIFIED" in " ".join(argv)]
        self.assertGreaterEqual(len(terminal_calls), 4)
        for call in terminal_calls:
            self.assertIn("exit_path=", call)
            self.assertIn("log_path=", call)

    def test_status_rejects_aborted_state_without_durable_recorder_reap_proof(self):
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state(status="aborted"))
            stdout = io.StringIO()
            stderr = io.StringIO()
            with contextlib.redirect_stdout(
                    stdout), contextlib.redirect_stderr(stderr):
                result = MODULE.main(
                    ["--evidence-dir", directory, "status"],
                    runner=FakeRunner(successful_callback))
        self.assertEqual(result, 1)
        self.assertEqual(stdout.getvalue(), "")
        self.assertIn(
            "terminal recorder cleanup proof invalid", stderr.getvalue())

    def test_status_accepts_aborted_state_with_hash_verified_reap_proof(self):
        runner = FakeRunner(successful_callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                MODULE.RotationHarness(runner, store).abort()
            stdout = io.StringIO()
            stderr = io.StringIO()
            with contextlib.redirect_stdout(
                    stdout), contextlib.redirect_stderr(stderr):
                result = MODULE.main(
                    ["--evidence-dir", directory, "status"],
                    runner=FakeRunner(successful_callback))
        self.assertEqual(result, 0)
        self.assertEqual(stderr.getvalue(), "")
        self.assertEqual(json.loads(stdout.getvalue())["status"], "aborted")
        terminal_calls = [
            argv[-1] for argv, _ in runner.calls
            if "RECORDER_ABSENCE_VERIFIED" in " ".join(argv)]
        self.assertGreaterEqual(len(terminal_calls), 2)
        for call in terminal_calls:
            self.assertIn("exit_path=", call)
            self.assertIn("log_path=", call)

    def test_status_accepts_old_reaped_launch_without_optional_markers(self):
        state = RecorderLaunchRecoveryTest.launch_state()
        robot = state["recorders"]["robot"]
        robot.update({
            "token": "d455-recorder-persisted",
            "startup_ack": "/tmp/persisted.ack",
            "exit_path": "/tmp/persisted.exit",
            "receipt_path": "/tmp/persisted.receipt",
            "launch_mode": "detached_foreground_docker_exec",
            "wrapper_reap_owner": "docker_exec_parent",
            "status": "launch_registered",
            "launch_registered_at": MODULE.utc_now(),
        })
        runner = FakeRunner(
            RecorderLaunchRecoveryTest.absent_receipt_callback(
                MODULE.CommandResult(0, "")))
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(state)
                MODULE.RotationHarness(runner, store).abort()
                completed = store.load()
                completed["recorders"]["robot"].pop("cancel_path", None)
                completed["recorders"]["robot"].pop("release_path", None)
                store.save(completed)
            stdout = io.StringIO()
            stderr = io.StringIO()
            with contextlib.redirect_stdout(
                    stdout), contextlib.redirect_stderr(stderr):
                result = MODULE.main(
                    ["--evidence-dir", directory, "status"],
                    runner=FakeRunner(successful_callback))
        self.assertEqual(result, 0)
        self.assertEqual(stderr.getvalue(), "")
        self.assertEqual(json.loads(stdout.getvalue())["status"], "aborted")
        self.assertFalse(any(
            publishes_twist(" ".join(argv), value)
            for argv, _ in runner.calls
            for value in MODULE.ALLOWED_ANGULAR_Z))

    def test_abort_keyboard_interrupt_still_stops_both_and_marks_invalid(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if publishes_twist(text, 0.0):
                raise KeyboardInterrupt()
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
                harness = MODULE.RotationHarness(runner, store)
                with self.assertRaises(KeyboardInterrupt):
                    harness.abort()
                state = store.load()
        joined = [" ".join(call[0]) for call in runner.calls]
        self.assertEqual(state["status"], "invalid")
        self.assertTrue(any("rotation_harness_zero_check" in call for call in joined))
        self.assertEqual(sum("RECORDER_REAPED" in call for call in joined), 2)
        self.assertTrue(any(
            "D455_PUBLISHER_ATTEMPT_PHASE" in call for call in joined))

    def test_main_reports_interrupt_with_exit_130_after_abort_cleanup(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if publishes_twist(text, 0.0):
                raise KeyboardInterrupt()
            return successful_callback(argv, timeout)

        with tempfile.TemporaryDirectory() as directory:
            with MODULE.StateStore(directory) as store:
                store.create(base_state())
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                status = MODULE.main(
                    ["--evidence-dir", directory, "abort"],
                    runner=FakeRunner(callback))
        self.assertEqual(status, 130)
        self.assertIn("interrupted after best-effort safety cleanup", stderr.getvalue())


class FinalizeSafetyTest(unittest.TestCase):
    def test_finalize_requires_distinct_fresh_post_audit_and_persists_sha(self):
        runner = FakeRunner(successful_callback)
        with tempfile.TemporaryDirectory() as directory:
            state = finalize_state(directory)
            post = clean_audit(directory, "post-motion-audit.txt")
            with MODULE.StateStore(directory) as store:
                store.create(state)
                harness = MODULE.RotationHarness(runner, store)
                harness.finalize(argparse.Namespace(kernel_audit_artifact=post))
                completed = store.load()
        self.assertEqual(completed["status"], "complete")
        self.assertEqual(len(completed["post_motion_kernel_audit"]["sha256"]), 64)
        self.assertNotEqual(
            completed["post_motion_kernel_audit"]["path"],
            completed["kernel_audit"]["path"])
        terminal_calls = [
            argv[-1] for argv, _ in runner.calls
            if "RECORDER_ABSENCE_VERIFIED" in " ".join(argv)]
        self.assertEqual(len(terminal_calls), 2)
        for call in terminal_calls:
            self.assertIn("exit_path=", call)
            self.assertIn("log_path=", call)

    def test_dirty_post_audit_zeroes_stops_and_preserves_partial_bags(self):
        runner = FakeRunner(successful_callback)
        with tempfile.TemporaryDirectory() as directory:
            state = finalize_state(directory)
            post = Path(directory) / "post-motion-audit.txt"
            post.write_text(
                "apparmor_denials=1\nd455_usb_reset_or_disconnect=0\n",
                encoding="utf-8")
            with MODULE.StateStore(directory) as store:
                store.create(state)
                harness = MODULE.RotationHarness(runner, store)
                with self.assertRaises(MODULE.HarnessError):
                    harness.finalize(
                        argparse.Namespace(kernel_audit_artifact=str(post)))
                failed = store.load()
                partial_robot = (Path(directory) / "partial-robot-bag").is_dir()
        joined = [" ".join(call[0]) for call in runner.calls]
        self.assertEqual(failed["status"], "invalid")
        self.assertTrue(any(publishes_twist(call, 0.0) for call in joined))
        self.assertEqual(sum("RECORDER_REAPED" in call for call in joined), 2)
        self.assertTrue(partial_robot)

    def test_finalize_keyboard_interrupt_cleans_then_main_style_reraises(self):
        def callback(argv, timeout):
            text = " ".join(argv)
            if publishes_twist(text, 0.0):
                raise KeyboardInterrupt()
            return successful_callback(argv, timeout)

        runner = FakeRunner(callback)
        with tempfile.TemporaryDirectory() as directory:
            state = finalize_state(directory)
            post = clean_audit(directory, "post-motion-audit.txt")
            with MODULE.StateStore(directory) as store:
                store.create(state)
                harness = MODULE.RotationHarness(runner, store)
                with self.assertRaises(KeyboardInterrupt):
                    harness.finalize(
                        argparse.Namespace(kernel_audit_artifact=post))
                failed = store.load()
        joined = [" ".join(call[0]) for call in runner.calls]
        self.assertEqual(failed["status"], "invalid")
        self.assertTrue(any("rotation_harness_zero_check" in call for call in joined))
        self.assertEqual(sum("RECORDER_REAPED" in call for call in joined), 2)
        self.assertTrue(any(
            "D455_PUBLISHER_ATTEMPT_PHASE" in call for call in joined))


class FrozenMotionCliTest(unittest.TestCase):
    def test_motion_cli_fails_before_state_or_runner_access(self):
        with tempfile.TemporaryDirectory() as directory:
            evidence_dir = Path(directory) / "must-not-be-created"
            stdout = io.StringIO()
            stderr = io.StringIO()
            argv = [
                "--evidence-dir", str(evidence_dir), "motion",
                "--linear-x", "0.0", "--angular-z", "0.30",
                "--duration", "2.0", "--rate-hz", "20",
                "--acknowledge-motion", MODULE.MOTION_ACK,
                "--kernel-audit-artifact", str(Path(directory) / "unused"),
            ]
            with contextlib.redirect_stdout(
                    stdout), contextlib.redirect_stderr(stderr):
                status = MODULE.main(argv)

        self.assertEqual(status, 1)
        self.assertEqual(stdout.getvalue(), "")
        self.assertIn(MODULE.FROZEN_MOTION_MESSAGE, stderr.getvalue())
        self.assertFalse(evidence_dir.exists())


if __name__ == "__main__":
    unittest.main()
