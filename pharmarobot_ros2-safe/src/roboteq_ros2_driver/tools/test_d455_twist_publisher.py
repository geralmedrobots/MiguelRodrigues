#!/usr/bin/env python3
"""Hardware-free tests for d455_twist_publisher.py."""

import importlib.util
import hashlib
import json
from pathlib import Path
import tempfile
import unittest


MODULE_PATH = Path(__file__).with_name("d455_twist_publisher.py")
SPEC = importlib.util.spec_from_file_location(
    "d455_twist_publisher", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
QOS_OVERRIDE_PATH = (
    MODULE_PATH.parent.parent / "config" /
    "d455_rotation_rosbag_qos.yaml").resolve()
QOS_OVERRIDE_SHA256 = hashlib.sha256(
    QOS_OVERRIDE_PATH.read_bytes()).hexdigest()


def request(command_type="motion", angular_z=0.30, count=40, duration=2.0):
    return {
        "command_type": command_type,
        "published_twist": {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
        },
        "requested_duration_s": duration,
        "requested_rate_hz": 20,
        "requested_publish_count": count,
        "required_subscription_endpoints": [
            "/:command_arbiter", "/:rosbag2_recorder",
        ],
        "recorder_qos_override_path": str(QOS_OVERRIDE_PATH),
        "recorder_qos_override_sha256": QOS_OVERRIDE_SHA256,
        "discovery_timeout_s": 5.0,
    }


class FakeRuntime:
    def __init__(self, subscription_count=2, details=None, fail_after=None):
        self.now = 1_000_000_000
        self.subscription_count_value = subscription_count
        self.details = details or [
            {
                "endpoint": endpoint,
                "qos": dict(MODULE.QOS_POLICY),
            }
            for endpoint in (
                "/:command_arbiter", "/:rosbag2_recorder")
        ]
        self.fail_after = fail_after
        self.published = 0

    def subscription_count(self):
        return self.subscription_count_value

    def subscription_details(self):
        return self.details

    def spin_once(self, timeout_sec):
        self.now += round(timeout_sec * 1_000_000_000)

    def publish(self):
        if self.fail_after is not None and self.published >= self.fail_after:
            raise RuntimeError("injected publication failure")
        self.published += 1

    def monotonic_ns(self):
        return self.now

    def system_ns(self):
        return self.now + 10_000_000_000

    def utc_now(self):
        return "2026-07-23T10:00:00.000000+00:00"

    def sleep_until_monotonic_ns(self, target_ns):
        self.now = max(self.now, target_ns)


class RequestValidationTest(unittest.TestCase):
    def test_zero_command_types_reject_nonzero_and_motion_rejects_zero(self):
        for command_type in ("prepare_zero", "cleanup_zero"):
            with self.subTest(command_type=command_type):
                with self.assertRaisesRegex(ValueError, "cannot publish nonzero"):
                    MODULE.validate_request(request(command_type=command_type))
        with self.assertRaisesRegex(ValueError, "must be nonzero"):
            MODULE.validate_request(request(angular_z=0.0))

    def test_qos_contract_is_explicit(self):
        self.assertEqual(MODULE.QOS_POLICY, {
            "history": "keep_last",
            "depth": 1,
            "reliability": "reliable",
            "durability": "volatile",
        })

    def test_discovery_timeout_must_be_finite_and_positive(self):
        for value in (0.0, -1.0, float("nan"), float("inf")):
            with self.subTest(value=value):
                invalid = request()
                invalid["discovery_timeout_s"] = value
                with self.assertRaisesRegex(
                        ValueError, "discovery timeout"):
                    MODULE.validate_request(invalid)

    def test_motion_and_prepare_require_arbiter_and_recorder(self):
        for command_type, angular_z in (
                ("motion", 0.30), ("prepare_zero", 0.0)):
            with self.subTest(command_type=command_type):
                invalid = request(
                    command_type=command_type, angular_z=angular_z)
                invalid["required_subscription_endpoints"] = [
                    "/:command_arbiter"]
                with self.assertRaisesRegex(
                        ValueError, "requires both"):
                    MODULE.validate_request(invalid)
        cleanup = request(
            command_type="cleanup_zero", angular_z=0.0,
            count=20, duration=1.0)
        cleanup["required_subscription_endpoints"] = [
            "/:command_arbiter"]
        cleanup["recorder_qos_override_path"] = None
        cleanup["recorder_qos_override_sha256"] = None
        MODULE.validate_request(cleanup)


class PublisherExecutionTest(unittest.TestCase):
    def test_motion_publishes_exact_count_with_monotonic_evidence(self):
        runtime = FakeRuntime()
        with tempfile.TemporaryDirectory() as directory:
            result = Path(directory) / "result.json"
            evidence = MODULE.execute(request(), runtime, result)
            persisted = json.loads(result.read_text())
        self.assertEqual(runtime.published, 40)
        self.assertEqual(evidence["status"], "complete")
        self.assertEqual(persisted["actual_publish_count"], 40)
        self.assertEqual(persisted["command_type"], "motion")
        self.assertEqual(persisted["publisher_qos"], MODULE.QOS_POLICY)
        self.assertEqual(
            persisted["timing"]["window_duration_s"], 2.0)
        self.assertEqual(persisted["timing"]["publish_span_s"], 1.95)
        self.assertEqual(
            len(persisted["publish_monotonic_ns"]), 40)
        self.assertTrue(
            persisted["recorder_qos_override"]["verified"])
        self.assertEqual(
            persisted["recorder_qos_override"]["actual_sha256"],
            QOS_OVERRIDE_SHA256)

    def test_prepare_and_cleanup_zero_use_the_same_execution_path(self):
        for command_type in ("prepare_zero", "cleanup_zero"):
            with self.subTest(command_type=command_type):
                runtime = FakeRuntime()
                zero = request(
                    command_type=command_type, angular_z=0.0,
                    count=20, duration=1.0)
                with tempfile.TemporaryDirectory() as directory:
                    result = Path(directory) / "result.json"
                    evidence = MODULE.execute(zero, runtime, result)
                self.assertEqual(runtime.published, 20)
                self.assertEqual(evidence["command_type"], command_type)
                self.assertEqual(evidence["publisher_qos"], MODULE.QOS_POLICY)

    def test_qos_mismatch_fails_before_any_publish(self):
        mismatched = dict(MODULE.QOS_POLICY)
        mismatched["reliability"] = "best_effort"
        runtime = FakeRuntime(
            subscription_count=2,
            details=[
                {
                    "endpoint": "/:command_arbiter",
                    "qos": dict(MODULE.QOS_POLICY),
                },
                {
                    "endpoint": "/:rosbag2_recorder",
                    "qos": mismatched,
                },
            ])
        with tempfile.TemporaryDirectory() as directory:
            result = Path(directory) / "result.json"
            with self.assertRaisesRegex(
                    RuntimeError, "QoS not accepted"):
                MODULE.execute(request(), runtime, result)
            evidence = json.loads(result.read_text())
        self.assertEqual(runtime.published, 0)
        self.assertEqual(evidence["status"], "failed")
        self.assertEqual(evidence["actual_publish_count"], 0)
        self.assertEqual(evidence["matched_subscriptions"], 2)
        self.assertEqual(
            evidence["matched_subscription_endpoints"],
            ["/:command_arbiter"])
        self.assertEqual(
            evidence["subscription_details"][1]["qos"]["reliability"],
            "best_effort")
        recorder = next(
            assessment for assessment in
            evidence["subscription_qos_assessments"]
            if assessment["endpoint"] == "/:rosbag2_recorder")
        self.assertEqual(recorder["status"], "rejected_mismatch")
        self.assertEqual(
            recorder["fields"]["reliability"]["status"], "mismatch")

    def test_unknown_history_and_zero_depth_are_accepted_as_unreported(self):
        unreported = {
            "history": "unknown",
            "depth": 0,
            "reliability": "reliable",
            "durability": "volatile",
        }
        runtime = FakeRuntime(details=[
            {"endpoint": endpoint, "qos": dict(unreported)}
            for endpoint in ("/:command_arbiter", "/:rosbag2_recorder")
        ])
        with tempfile.TemporaryDirectory() as directory:
            result = Path(directory) / "result.json"
            evidence = MODULE.execute(request(), runtime, result)
        self.assertEqual(runtime.published, 40)
        self.assertEqual(evidence["status"], "complete")
        self.assertTrue(
            evidence["recorder_qos_override"]["verified"])
        for assessment in evidence["subscription_qos_assessments"]:
            self.assertEqual(
                assessment["status"], "accepted_unreported")
            self.assertEqual(
                assessment["tolerated_unreported_fields"],
                ["depth", "history"])
            self.assertEqual(
                assessment["fields"]["reliability"]["status"], "verified")
            self.assertEqual(
                assessment["fields"]["durability"]["status"], "verified")

    def test_durability_mismatch_fails_before_any_publish(self):
        mismatched = dict(MODULE.QOS_POLICY)
        mismatched["durability"] = "transient_local"
        runtime = FakeRuntime(details=[
            {
                "endpoint": "/:command_arbiter",
                "qos": mismatched,
            },
            {
                "endpoint": "/:rosbag2_recorder",
                "qos": dict(MODULE.QOS_POLICY),
            },
        ])
        with tempfile.TemporaryDirectory() as directory:
            result = Path(directory) / "result.json"
            with self.assertRaisesRegex(
                    RuntimeError, "QoS not accepted"):
                MODULE.execute(request(), runtime, result)
            evidence = json.loads(result.read_text())
        self.assertEqual(runtime.published, 0)
        arbiter = next(
            assessment for assessment in
            evidence["subscription_qos_assessments"]
            if assessment["endpoint"] == "/:command_arbiter")
        self.assertEqual(
            arbiter["fields"]["durability"]["status"], "mismatch")

    def test_unpinned_override_fails_before_discovery_or_publish(self):
        invalid = request()
        invalid["recorder_qos_override_sha256"] = "0" * 64
        runtime = FakeRuntime()
        with tempfile.TemporaryDirectory() as directory:
            result = Path(directory) / "result.json"
            with self.assertRaisesRegex(RuntimeError, "SHA-256 mismatch"):
                MODULE.execute(invalid, runtime, result)
            evidence = json.loads(result.read_text())
        self.assertEqual(runtime.published, 0)
        self.assertEqual(evidence["matched_subscriptions"], 0)
        self.assertFalse(evidence["recorder_qos_override"]["verified"])
        self.assertEqual(
            evidence["recorder_qos_override"]["actual_sha256"],
            QOS_OVERRIDE_SHA256)

    def test_missing_required_endpoints_fail_before_any_publish(self):
        cases = (
            (
                "arbiter",
                [{
                    "endpoint": "/:rosbag2_recorder",
                    "qos": dict(MODULE.QOS_POLICY),
                }],
                "/:command_arbiter",
            ),
            (
                "recorder",
                [{
                    "endpoint": "/:command_arbiter",
                    "qos": dict(MODULE.QOS_POLICY),
                }],
                "/:rosbag2_recorder",
            ),
        )
        for label, details, missing in cases:
            with self.subTest(label=label):
                runtime = FakeRuntime(subscription_count=1, details=details)
                with tempfile.TemporaryDirectory() as directory:
                    result = Path(directory) / "result.json"
                    with self.assertRaisesRegex(
                            RuntimeError, "QoS not accepted"):
                        MODULE.execute(request(), runtime, result)
                    evidence = json.loads(result.read_text())
                self.assertEqual(runtime.published, 0)
                assessment = next(
                    item for item in
                    evidence["subscription_qos_assessments"]
                    if item["endpoint"] == missing)
                self.assertEqual(assessment["status"], "rejected_missing")

    def test_raw_graph_count_undercount_is_telemetry_only(self):
        runtime = FakeRuntime(subscription_count=1)
        with tempfile.TemporaryDirectory() as directory:
            result = Path(directory) / "result.json"
            evidence = MODULE.execute(request(), runtime, result)
        self.assertEqual(runtime.published, 40)
        self.assertEqual(evidence["status"], "complete")
        self.assertEqual(evidence["matched_subscriptions"], 1)
        self.assertEqual(
            evidence["matched_subscription_endpoints"],
            ["/:command_arbiter", "/:rosbag2_recorder"])

    def test_conflicting_duplicate_required_identity_fails_before_publish(self):
        conflicting = dict(MODULE.QOS_POLICY)
        conflicting["durability"] = "transient_local"
        runtime = FakeRuntime(
            subscription_count=3,
            details=[
                {
                    "endpoint": "/:command_arbiter",
                    "qos": dict(MODULE.QOS_POLICY),
                },
                {
                    "endpoint": "/:rosbag2_recorder",
                    "qos": dict(MODULE.QOS_POLICY),
                },
                {
                    "endpoint": "/:rosbag2_recorder",
                    "qos": conflicting,
                },
            ])
        with tempfile.TemporaryDirectory() as directory:
            result = Path(directory) / "result.json"
            with self.assertRaisesRegex(
                    RuntimeError, "QoS not accepted"):
                MODULE.execute(request(), runtime, result)
            evidence = json.loads(result.read_text())
        self.assertEqual(runtime.published, 0)
        self.assertEqual(evidence["status"], "failed")
        self.assertNotIn(
            "/:rosbag2_recorder",
            evidence["matched_subscription_endpoints"])

    def test_partial_publication_persists_failure_count(self):
        runtime = FakeRuntime(fail_after=7)
        with tempfile.TemporaryDirectory() as directory:
            result = Path(directory) / "result.json"
            with self.assertRaisesRegex(
                    RuntimeError, "injected publication failure"):
                MODULE.execute(request(), runtime, result)
            evidence = json.loads(result.read_text())
        self.assertEqual(evidence["status"], "failed")
        self.assertEqual(evidence["actual_publish_count"], 7)
        self.assertEqual(len(evidence["publish_monotonic_ns"]), 7)

    def test_runtime_initialization_failure_persists_zero_count(self):
        original_runtime = MODULE.RclpyRuntime

        class FailedRuntime:
            def __init__(self, _payload):
                raise RuntimeError("injected rclpy initialization failure")

        MODULE.RclpyRuntime = FailedRuntime
        try:
            with tempfile.TemporaryDirectory() as directory:
                result = Path(directory) / "result.json"
                argv = [
                    "--command-type", "motion",
                    "--payload-json",
                    json.dumps(request()["published_twist"]),
                    "--duration", "2.0",
                    "--rate-hz", "20",
                    "--count", "40",
                    "--required-endpoint", "/:command_arbiter",
                    "--required-endpoint", "/:rosbag2_recorder",
                    "--recorder-qos-override-path",
                    str(QOS_OVERRIDE_PATH),
                    "--recorder-qos-override-sha256",
                    QOS_OVERRIDE_SHA256,
                    "--discovery-timeout", "5.0",
                    "--result", str(result),
                ]
                with self.assertRaisesRegex(
                        RuntimeError, "initialization failure"):
                    MODULE.main(argv)
                evidence = json.loads(result.read_text())
        finally:
            MODULE.RclpyRuntime = original_runtime
        self.assertEqual(evidence["status"], "failed")
        self.assertEqual(evidence["command_type"], "motion")
        self.assertEqual(evidence["requested_publish_count"], 40)
        self.assertEqual(evidence["actual_publish_count"], 0)
        self.assertIn("initialization failure", evidence["error"])


if __name__ == "__main__":
    unittest.main()
