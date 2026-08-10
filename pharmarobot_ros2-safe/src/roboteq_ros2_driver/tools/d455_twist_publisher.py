#!/usr/bin/env python3
# Copyright 2026 Medrobots

"""Deterministic, evidence-producing publisher for D455 rotation validation."""

import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import sys
import time


QOS_POLICY = {
    "history": "keep_last",
    "depth": 1,
    "reliability": "reliable",
    "durability": "volatile",
}
ALLOWED_COMMAND_TYPES = {"prepare_zero", "motion", "cleanup_zero"}
ALLOWED_ENDPOINTS = {"/:command_arbiter", "/:rosbag2_recorder"}
UNREPORTED_QOS_VALUES = {
    "history": {"unknown"},
    "depth": {0},
}


class PublisherInterrupted(RuntimeError):
    """Raised when the owned publisher receives a termination signal."""


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(
        timespec="microseconds")


def atomic_write_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC
    fd = os.open(temporary, flags, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        json.dump(
            value, stream, allow_nan=False, sort_keys=True,
            separators=(",", ":"))
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def assess_subscription_qos(reported_qos):
    """Classify endpoint QoS without treating DDS omissions as mismatches."""
    reported_qos = reported_qos if isinstance(reported_qos, dict) else {}
    fields = {}
    for field in ("reliability", "durability", "history", "depth"):
        expected = QOS_POLICY[field]
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
        field["status"] != "mismatch" for field in fields.values())
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


def assess_required_endpoints(subscription_details, required_endpoints):
    assessments = []
    for endpoint in required_endpoints:
        endpoint_details = [
            detail for detail in subscription_details
            if isinstance(detail, dict) and detail.get("endpoint") == endpoint
        ]
        if not endpoint_details:
            assessments.append({
                "endpoint": endpoint,
                "record_count": 0,
                "accepted": False,
                "status": "rejected_missing",
                "records": [],
            })
            continue
        records = [
            {
                "qos": detail.get("qos"),
                "assessment": assess_subscription_qos(detail.get("qos")),
            }
            for detail in endpoint_details
        ]
        if len(endpoint_details) != 1:
            assessments.append({
                "endpoint": endpoint,
                "record_count": len(endpoint_details),
                "accepted": False,
                "status": "rejected_ambiguous",
                "records": records,
            })
            continue
        assessment = dict(records[0]["assessment"])
        assessment.update({
            "endpoint": endpoint,
            "record_count": 1,
            "records": records,
        })
        assessments.append(assessment)
    return assessments


def override_evidence(request):
    required = (
        "/:rosbag2_recorder" in
        request.get("required_subscription_endpoints", []))
    return {
        "required": required,
        "path": request.get("recorder_qos_override_path"),
        "expected_sha256": request.get("recorder_qos_override_sha256"),
        "actual_sha256": None,
        "verified": False,
    }


def verify_recorder_qos_override(run):
    proof = run["recorder_qos_override"]
    if not proof["required"]:
        return
    path = Path(proof["path"])
    if path.is_symlink() or not path.is_file():
        raise RuntimeError(
            f"recorder QoS override is not a regular non-symlink file: {path}")
    proof["actual_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    if proof["actual_sha256"] != proof["expected_sha256"]:
        raise RuntimeError(
            "recorder QoS override SHA-256 mismatch: "
            f"expected={proof['expected_sha256']} "
            f"actual={proof['actual_sha256']}")
    proof["verified"] = True


def validate_request(request):
    command_type = request["command_type"]
    if command_type not in ALLOWED_COMMAND_TYPES:
        raise ValueError(f"unsupported command type: {command_type!r}")
    duration = request["requested_duration_s"]
    rate = request["requested_rate_hz"]
    count = request["requested_publish_count"]
    if (
            not isinstance(duration, (int, float)) or
            not math.isfinite(duration) or duration <= 0.0):
        raise ValueError("duration must be finite and positive")
    if (
            not isinstance(rate, (int, float)) or
            not math.isfinite(rate) or rate <= 0.0):
        raise ValueError("rate must be finite and positive")
    if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
        raise ValueError("count must be a positive integer")
    if count != round(duration * rate):
        raise ValueError("count does not match duration and rate")
    discovery_timeout = request["discovery_timeout_s"]
    if (
            not isinstance(discovery_timeout, (int, float)) or
            isinstance(discovery_timeout, bool) or
            not math.isfinite(discovery_timeout) or
            discovery_timeout <= 0.0):
        raise ValueError("discovery timeout must be finite and positive")

    payload = request["published_twist"]
    values = (
        payload["linear"]["x"], payload["linear"]["y"], payload["linear"]["z"],
        payload["angular"]["x"], payload["angular"]["y"], payload["angular"]["z"],
    )
    if any(
            not isinstance(value, (int, float)) or isinstance(value, bool) or
            not math.isfinite(value) for value in values):
        raise ValueError("Twist payload contains a non-finite value")
    if any(abs(value) > 1e-12 for value in values[:5]):
        raise ValueError("publisher payload is not rotation-only")
    if command_type == "motion" and abs(values[5]) <= 1e-12:
        raise ValueError("motion command must be nonzero")
    if command_type != "motion" and abs(values[5]) > 1e-12:
        raise ValueError("zero command type cannot publish nonzero motion")

    endpoints = request["required_subscription_endpoints"]
    if (
            not isinstance(endpoints, list) or
            not endpoints or
            len(endpoints) != len(set(endpoints)) or
            not set(endpoints).issubset(ALLOWED_ENDPOINTS) or
            "/:command_arbiter" not in endpoints):
        raise ValueError("required subscription endpoints are invalid")
    if (
            command_type in {"prepare_zero", "motion"} and
            set(endpoints) != ALLOWED_ENDPOINTS):
        raise ValueError(
            f"{command_type} requires both arbiter and recorder endpoints")
    override_path = request.get("recorder_qos_override_path")
    override_sha256 = request.get("recorder_qos_override_sha256")
    recorder_required = "/:rosbag2_recorder" in endpoints
    if recorder_required:
        if (
                not isinstance(override_path, str) or
                not override_path or
                not Path(override_path).is_absolute()):
            raise ValueError(
                "recorder endpoint requires an absolute QoS override path")
        if (
                not isinstance(override_sha256, str) or
                re.fullmatch(r"[0-9a-f]{64}", override_sha256) is None):
            raise ValueError(
                "recorder endpoint requires a pinned QoS override SHA-256")
    elif override_path is not None or override_sha256 is not None:
        raise ValueError(
            "recorder QoS override proof is invalid without recorder endpoint")


def build_evidence(run, status, error=None):
    publish_monotonic_ns = run["publish_monotonic_ns"]
    intervals_ns = [
        following - current
        for current, following in zip(
            publish_monotonic_ns, publish_monotonic_ns[1:])
    ]
    window_start = run["window_start_monotonic_ns"]
    window_end = run["window_end_monotonic_ns"]
    return {
        "schema_version": 1,
        "status": status,
        "error": error,
        "command_type": run["request"]["command_type"],
        "publisher_qos": dict(QOS_POLICY),
        "requested_publish_count": run["request"]["requested_publish_count"],
        "actual_publish_count": len(publish_monotonic_ns),
        "requested_duration_s": run["request"]["requested_duration_s"],
        "requested_rate_hz": run["request"]["requested_rate_hz"],
        "published_twist": run["request"]["published_twist"],
        "required_subscription_endpoints": (
            run["request"]["required_subscription_endpoints"]),
        "matched_subscriptions": run["matched_subscriptions"],
        "matched_subscription_endpoints": (
            run["matched_subscription_endpoints"]),
        "subscription_details": run["subscription_details"],
        "subscription_qos_assessments": (
            run["subscription_qos_assessments"]),
        "recorder_qos_override": run["recorder_qos_override"],
        "subscriber_ready_monotonic_ns": (
            run["subscriber_ready_monotonic_ns"]),
        "window_start_monotonic_ns": window_start,
        "first_publish_monotonic_ns": (
            publish_monotonic_ns[0] if publish_monotonic_ns else None),
        "last_publish_monotonic_ns": (
            publish_monotonic_ns[-1] if publish_monotonic_ns else None),
        "window_end_monotonic_ns": window_end,
        "window_start_system_ns": run["window_start_system_ns"],
        "first_publish_system_ns": (
            run["publish_system_ns"][0] if run["publish_system_ns"] else None),
        "last_publish_system_ns": (
            run["publish_system_ns"][-1] if run["publish_system_ns"] else None),
        "window_end_system_ns": run["window_end_system_ns"],
        "window_start_utc": run["window_start_utc"],
        "first_publish_utc": (
            run["publish_utc"][0] if run["publish_utc"] else None),
        "last_publish_utc": (
            run["publish_utc"][-1] if run["publish_utc"] else None),
        "window_end_utc": run["window_end_utc"],
        "publish_monotonic_ns": publish_monotonic_ns,
        "publish_system_ns": run["publish_system_ns"],
        "schedule_lateness_ns": run["schedule_lateness_ns"],
        "timing": {
            "period_s": run["period_ns"] / 1000000000.0,
            "window_duration_s": (
                (window_end - window_start) / 1000000000.0
                if window_start is not None else 0.0),
            "publish_span_s": (
                (publish_monotonic_ns[-1] - publish_monotonic_ns[0]) /
                1000000000.0
                if len(publish_monotonic_ns) >= 2 else 0.0),
            "mean_interval_s": (
                sum(intervals_ns) / len(intervals_ns) / 1000000000.0
                if intervals_ns else 0.0),
            "min_interval_s": (
                min(intervals_ns) / 1000000000.0
                if intervals_ns else 0.0),
            "max_interval_s": (
                max(intervals_ns) / 1000000000.0
                if intervals_ns else 0.0),
            "max_schedule_lateness_s": (
                max(run["schedule_lateness_ns"]) / 1000000000.0
                if run["schedule_lateness_ns"] else 0.0),
        },
        "evidence_updated_at_utc": utc_now(),
    }


def execute(request, runtime, evidence_path):
    """Execute one validated request using an injected ROS/runtime adapter."""
    validate_request(request)
    evidence_path = Path(evidence_path)
    if evidence_path.exists() or evidence_path.is_symlink():
        raise ValueError(
            f"refusing to overwrite publisher evidence: {evidence_path}")

    run = {
        "request": request,
        "period_ns": round(
            1000000000.0 / request["requested_rate_hz"]),
        "matched_subscriptions": 0,
        "matched_subscription_endpoints": [],
        "subscription_details": [],
        "subscription_qos_assessments": [],
        "recorder_qos_override": override_evidence(request),
        "subscriber_ready_monotonic_ns": None,
        "window_start_monotonic_ns": None,
        "window_start_system_ns": None,
        "window_start_utc": None,
        "window_end_monotonic_ns": runtime.monotonic_ns(),
        "window_end_system_ns": runtime.system_ns(),
        "window_end_utc": runtime.utc_now(),
        "publish_monotonic_ns": [],
        "publish_system_ns": [],
        "publish_utc": [],
        "schedule_lateness_ns": [],
    }

    def refresh_end():
        run["window_end_monotonic_ns"] = runtime.monotonic_ns()
        run["window_end_system_ns"] = runtime.system_ns()
        run["window_end_utc"] = runtime.utc_now()

    def refresh_subscriptions():
        run["matched_subscriptions"] = runtime.subscription_count()
        run["subscription_details"] = runtime.subscription_details()
        run["subscription_qos_assessments"] = assess_required_endpoints(
            run["subscription_details"],
            request["required_subscription_endpoints"])
        run["matched_subscription_endpoints"] = sorted(
            assessment["endpoint"]
            for assessment in run["subscription_qos_assessments"]
            if assessment["accepted"])

    def write(status, error=None):
        refresh_end()
        atomic_write_json(
            evidence_path, build_evidence(run, status, error))

    try:
        write("verifying_qos_override")
        verify_recorder_qos_override(run)
        write("discovering")
        discovery_deadline_ns = (
            runtime.monotonic_ns() +
            round(request["discovery_timeout_s"] * 1000000000.0))
        required_endpoints = set(
            request["required_subscription_endpoints"])
        while True:
            runtime.spin_once(0.05)
            refresh_subscriptions()
            write("discovering")
            ready = (
                required_endpoints.issubset(
                    set(run["matched_subscription_endpoints"])))
            if ready:
                break
            if runtime.monotonic_ns() >= discovery_deadline_ns:
                raise RuntimeError(
                    "required /cmd_vel/test subscription QoS not accepted: "
                    f"expected>={len(required_endpoints)} "
                    f"matched={run['matched_subscriptions']} "
                    f"accepted_endpoints="
                    f"{run['matched_subscription_endpoints']!r} "
                    f"assessments="
                    f"{run['subscription_qos_assessments']!r}")

        run["subscriber_ready_monotonic_ns"] = runtime.monotonic_ns()
        run["window_start_monotonic_ns"] = runtime.monotonic_ns()
        run["window_start_system_ns"] = runtime.system_ns()
        run["window_start_utc"] = runtime.utc_now()
        write("ready")
        for index in range(request["requested_publish_count"]):
            target_ns = (
                run["window_start_monotonic_ns"] +
                (index + 1) * run["period_ns"])
            runtime.sleep_until_monotonic_ns(target_ns)
            runtime.publish()
            actual_monotonic_ns = runtime.monotonic_ns()
            run["publish_monotonic_ns"].append(actual_monotonic_ns)
            run["publish_system_ns"].append(runtime.system_ns())
            run["publish_utc"].append(runtime.utc_now())
            run["schedule_lateness_ns"].append(
                max(0, actual_monotonic_ns - target_ns))
            write("publishing")
        write("complete")
        return build_evidence(run, "complete")
    except BaseException as error:
        try:
            write(
                "interrupted" if isinstance(error, PublisherInterrupted)
                else "failed",
                f"{type(error).__name__}: {error}")
        except BaseException:
            pass
        raise


class RclpyRuntime:
    """ROS adapter kept small so deterministic scheduling can be unit-tested."""

    def __init__(self, payload):
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.qos import (
            QoSDurabilityPolicy,
            QoSHistoryPolicy,
            QoSProfile,
            QoSReliabilityPolicy,
        )

        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("d455_rotation_twist_publisher")
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=QOS_POLICY["depth"],
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.publisher = self.node.create_publisher(
            Twist, "/cmd_vel/test", qos)
        self.message = Twist()
        self.message.linear.x = float(payload["linear"]["x"])
        self.message.linear.y = float(payload["linear"]["y"])
        self.message.linear.z = float(payload["linear"]["z"])
        self.message.angular.x = float(payload["angular"]["x"])
        self.message.angular.y = float(payload["angular"]["y"])
        self.message.angular.z = float(payload["angular"]["z"])

    def subscription_count(self):
        return self.publisher.get_subscription_count()

    @staticmethod
    def _policy_name(value):
        name = getattr(value, "name", None)
        if not isinstance(name, str):
            name = str(value).rsplit(".", 1)[-1]
        return name.lower()

    def subscription_details(self):
        details = []
        for info in self.node.get_subscriptions_info_by_topic(
                "/cmd_vel/test"):
            profile = info.qos_profile
            details.append({
                "endpoint": f"{info.node_namespace}:{info.node_name}",
                "qos": {
                    "history": self._policy_name(profile.history),
                    "depth": int(profile.depth),
                    "reliability": self._policy_name(
                        profile.reliability),
                    "durability": self._policy_name(
                        profile.durability),
                },
            })
        return sorted(
            details,
            key=lambda detail: (
                detail["endpoint"],
                json.dumps(detail["qos"], sort_keys=True)))

    def spin_once(self, timeout_sec):
        self.rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def publish(self):
        self.publisher.publish(self.message)

    @staticmethod
    def monotonic_ns():
        return time.monotonic_ns()

    @staticmethod
    def system_ns():
        return time.time_ns()

    @staticmethod
    def utc_now():
        return utc_now()

    @staticmethod
    def sleep_until_monotonic_ns(target_ns):
        while True:
            remaining_ns = target_ns - time.monotonic_ns()
            if remaining_ns <= 0:
                return
            time.sleep(remaining_ns / 1000000000.0)

    def close(self):
        self.node.destroy_publisher(self.publisher)
        self.node.destroy_node()
        self.rclpy.shutdown()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--command-type", choices=sorted(ALLOWED_COMMAND_TYPES), required=True)
    parser.add_argument("--payload-json", required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--rate-hz", type=float, required=True)
    parser.add_argument("--count", type=int, required=True)
    parser.add_argument(
        "--required-endpoint", action="append", required=True)
    parser.add_argument("--recorder-qos-override-path")
    parser.add_argument("--recorder-qos-override-sha256")
    parser.add_argument("--discovery-timeout", type=float, required=True)
    parser.add_argument("--result", required=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    result_path = Path(args.result)
    request = {
        "command_type": args.command_type,
        "published_twist": json.loads(args.payload_json),
        "requested_duration_s": args.duration,
        "requested_rate_hz": args.rate_hz,
        "requested_publish_count": args.count,
        "required_subscription_endpoints": args.required_endpoint,
        "recorder_qos_override_path": args.recorder_qos_override_path,
        "recorder_qos_override_sha256": (
            args.recorder_qos_override_sha256),
        "discovery_timeout_s": args.discovery_timeout,
    }

    def interrupt(signum, _frame):
        raise PublisherInterrupted(f"signal {signum}")

    previous_handlers = {
        signum: signal.getsignal(signum)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    runtime = None
    try:
        validate_request(request)
        runtime = RclpyRuntime(request["published_twist"])
        execute(request, runtime, result_path)
        print(
            "D455_TWIST_PUBLISHER_COMPLETE "
            f"type={request['command_type']} "
            f"count={request['requested_publish_count']}",
            flush=True)
        return 0
    except BaseException as error:
        if not result_path.exists() and not result_path.is_symlink():
            try:
                now_monotonic = time.monotonic_ns()
                now_system = time.time_ns()
                failed_run = {
                    "request": request,
                    "period_ns": round(
                        1000000000.0 / request["requested_rate_hz"]),
                    "matched_subscriptions": 0,
                    "matched_subscription_endpoints": [],
                    "subscription_details": [],
                    "subscription_qos_assessments": [],
                    "recorder_qos_override": override_evidence(request),
                    "subscriber_ready_monotonic_ns": None,
                    "window_start_monotonic_ns": None,
                    "window_start_system_ns": None,
                    "window_start_utc": None,
                    "window_end_monotonic_ns": now_monotonic,
                    "window_end_system_ns": now_system,
                    "window_end_utc": utc_now(),
                    "publish_monotonic_ns": [],
                    "publish_system_ns": [],
                    "publish_utc": [],
                    "schedule_lateness_ns": [],
                }
                atomic_write_json(
                    result_path,
                    build_evidence(
                        failed_run, "failed",
                        f"{type(error).__name__}: {error}"))
            except BaseException:
                pass
        raise
    finally:
        try:
            if runtime is not None:
                runtime.close()
        finally:
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)


if __name__ == "__main__":
    sys.exit(main())
