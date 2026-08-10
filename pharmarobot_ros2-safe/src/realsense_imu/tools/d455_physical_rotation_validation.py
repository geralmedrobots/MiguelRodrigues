#!/usr/bin/env python3
"""Fail-closed, dry-run-first D455 physical rotation validator.

The validator publishes to the existing arbiter input, ``/cmd_vel/test``,
and observes the sole motor-facing publisher on ``/cmd_vel/safe``. Runtime
access is behind an injected adapter so the campaign can be tested offline.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Mapping, Optional, Protocol, Sequence

INPUT_TOPIC = "/cmd_vel/test"
SAFE_TOPIC = "/cmd_vel/safe"
RAW_IMU_TOPIC = "/imu/d455/data_raw"
PROCESSED_IMU_TOPIC = "/imu/data"
DIAGNOSTICS_TOPIC = "/diagnostics"
DEFAULT_SPEEDS = (0.10, 0.20, 0.30)
DEFAULT_DURATIONS = (2.0, 4.0)
DEFAULT_RATE_HZ = 20.0
DEFAULT_REPETITIONS = 2
DEFAULT_SETTLE_TIMEOUT_S = 8.0
DEFAULT_ZERO_DURATION_S = 0.50
DEFAULT_GRAPH_CONVERGENCE_TIMEOUT_S = 8.0
DEFAULT_GRAPH_CONVERGENCE_POLL_S = 0.25
MAX_GRAPH_CONVERGENCE_TIMEOUT_S = 15.0
MAX_GRAPH_CONVERGENCE_POLL_S = 2.0
DEFAULT_LINEAR_SPEEDS = (0.05,)
# The command arbiter's configured angular limit is 0.90 rad/s.  Keep this
# validator at or below that limit so a requested value can never be silently
# clamped downstream (which would invalidate command/evidence correspondence).
MAX_SPEED = 0.90
MAX_LINEAR_SPEED = 1.0
MAX_DURATION = 4.0
MAX_REPETITIONS = 2
MAX_TRIALS = 24
WHEEL_RADIUS_M = 0.0881
APPROVAL_TEXT = "ROTATE"
EXPECTED_SENSOR_CONTAINER = "pharmarobot_d455_sensor"
EXPECTED_APPARMOR = "pharmarobot-d455-imu"
EXPECTED_RAW_FRAME = "d455_imu_optical_frame"
EXPECTED_PROCESSED_FRAME = "d455_imu_link"
BEST_EFFORT_QOS = "best_effort/volatile"
RELIABLE_QOS = "reliable/volatile"
MAX_PUBLISH_LATENESS_PERIODS = 0.5
MIN_YAW_RATIO = 0.25
MAX_YAW_RATIO = 2.0
HANDOFF_SCHEMA = 1
WORKER_CONTAINER = "pharma_container"
WORKER_TEMP_ROOT = Path("/tmp")
WORKER_SOURCE_PREFIX = "d455-rotation-worker-"
WORKER_HANDOFF_PREFIX = "d455-rotation-handoff-"
WORKER_EVIDENCE_PREFIX = "d455-rotation-evidence-"
HOST_VALIDATION_ROOT = Path(
    "src/realsense_imu/validation_evidence"
)
WORKER_RESULT_PREFIX = "D455_ROTATION_WORKER_RESULT="
HEARTBEAT_INTERVAL_S = 0.10
HEARTBEAT_MAX_AGE_S = 0.50
WORKER_COMPLETION_FILE = "worker-complete.json"
WORKER_ABORT_GRACE_S = DEFAULT_SETTLE_TIMEOUT_S * 2.0 + 2.0


class ValidationError(RuntimeError):
    """A fail-closed preflight, trial, stop, or evidence failure."""


@dataclass(frozen=True)
class Trial:
    index: int
    direction: str
    angular_z: float
    duration_s: float
    repetition: int
    linear_x: float = 0.0
    command_type: str = "rotation"


@dataclass(frozen=True)
class GraphEndpoint:
    node: str
    qos: str
    gid: str = ""


@dataclass(frozen=True)
class PreflightSnapshot:
    production_containers: tuple[str, ...]
    validation_containers: tuple[str, ...]
    foreign_d455_owners: tuple[str, ...]
    apparmor_profile: str
    apparmor_enforcing: bool
    sensor_container_id: str
    sensor_image_id: str
    immutable_config_sha256: str
    raw_publishers: tuple[GraphEndpoint, ...]
    processed_publishers: tuple[GraphEndpoint, ...]
    diagnostics_publishers: tuple[GraphEndpoint, ...]
    safe_publishers: tuple[GraphEndpoint, ...]
    input_publishers: tuple[GraphEndpoint, ...]
    input_subscribers: tuple[GraphEndpoint, ...]
    safe_subscribers: tuple[GraphEndpoint, ...]
    relay_nodes: tuple[str, ...]
    processor_nodes: tuple[str, ...]
    main_container_id: str
    main_immutable_config_sha256: str


@dataclass(frozen=True)
class HostPreflightSnapshot:
    production_containers: tuple[str, ...]
    validation_containers: tuple[str, ...]
    foreign_d455_owners: tuple[str, ...]
    apparmor_profile: str
    apparmor_enforcing: bool
    sensor_container_id: str
    sensor_image_id: str
    immutable_config_sha256: str
    main_container_id: str
    main_immutable_config_sha256: str


@dataclass(frozen=True)
class RosPreflightSnapshot:
    raw_publishers: tuple[GraphEndpoint, ...]
    processed_publishers: tuple[GraphEndpoint, ...]
    diagnostics_publishers: tuple[GraphEndpoint, ...]
    safe_publishers: tuple[GraphEndpoint, ...]
    input_publishers: tuple[GraphEndpoint, ...]
    input_subscribers: tuple[GraphEndpoint, ...]
    safe_subscribers: tuple[GraphEndpoint, ...]
    relay_nodes: tuple[str, ...]
    processor_nodes: tuple[str, ...]


@dataclass(frozen=True)
class GraphConvergence:
    attempts: tuple[Mapping[str, object], ...]
    started_monotonic_s: float
    started_wall_time_ns: int
    finished_monotonic_s: float
    finished_wall_time_ns: int
    result: str
    duration_s: float


@dataclass(frozen=True)
class TimedTwist:
    monotonic_s: float
    wall_time_ns: int
    payload: Mapping[str, Mapping[str, float]]
    phase: str
    trial_index: Optional[int]


@dataclass(frozen=True)
class ImuSample:
    monotonic_s: float
    stamp_ns: int
    frame_id: str
    angular_velocity: tuple[float, float, float]
    linear_acceleration: tuple[float, float, float]
    orientation_covariance: tuple[float, ...] = ()
    angular_velocity_covariance: tuple[float, ...] = ()
    linear_acceleration_covariance: tuple[float, ...] = ()


@dataclass
class TrialEvidence:
    trial: Trial
    expected_yaw_rad: float
    command_samples: list[TimedTwist] = field(default_factory=list)
    safe_samples: list[TimedTwist] = field(default_factory=list)
    raw_imu: list[ImuSample] = field(default_factory=list)
    processed_imu: list[ImuSample] = field(default_factory=list)
    diagnostics: list[Mapping[str, object]] = field(default_factory=list)
    encoder: list[Mapping[str, object]] = field(default_factory=list)
    odometry: list[Mapping[str, object]] = field(default_factory=list)
    analysis: Mapping[str, object] = field(default_factory=dict)
    stop_latency_s: Optional[float] = None
    settling_time_s: Optional[float] = None
    zero_verified: bool = False
    result: str = "pending"
    error: str = ""
    cleanup_error: str = ""


class Runtime(Protocol):
    """Read-only observation plus bounded command publication adapter."""

    def monotonic(self) -> float: ...
    def wall_time_ns(self) -> int: ...
    def sleep(self, duration_s: float) -> None: ...
    def preflight_snapshot(self) -> PreflightSnapshot: ...
    def create_publisher(self) -> None: ...
    def assert_motion_authorized(self) -> None: ...
    def publish(self, payload: Mapping[str, Mapping[str, float]]) -> None: ...

    def verify_runtime_identity(
        self,
        expected: PreflightSnapshot,
    ) -> None: ...

    def observe_safe(
        self,
        newer_than_s: float,
        timeout_s: float,
    ) -> TimedTwist: ...

    def stationary(self, newer_than_s: float, timeout_s: float) -> bool: ...

    def capture_trial(
        self,
        trial: Trial,
        start_s: float,
        end_s: float,
    ) -> Mapping[str, object]: ...
    def close(self) -> None: ...


def validate_values(
    speeds: Sequence[float],
    durations: Sequence[float],
    repetitions: int,
) -> None:
    if not speeds or not durations:
        raise ValueError("speeds and durations must be non-empty")
    if repetitions < 1 or repetitions > MAX_REPETITIONS:
        raise ValueError(f"repetitions must be in [1,{MAX_REPETITIONS}]")
    if (
        len(set(speeds)) != len(speeds)
        or len(set(durations)) != len(durations)
    ):
        raise ValueError("duplicate speeds or durations are not allowed")
    if any(
        not math.isfinite(value) or value <= 0 or value > MAX_SPEED
        for value in speeds
    ):
        raise ValueError(
            f"angular speeds must be finite and in (0,{MAX_SPEED}]"
        )
    if any(
        not math.isfinite(value) or value <= 0 or value > MAX_DURATION
        for value in durations
    ):
        raise ValueError(f"durations must be finite and in (0,{MAX_DURATION}]")
    if len(speeds) * len(durations) * 2 * repetitions > MAX_TRIALS:
        raise ValueError(f"campaign cannot exceed {MAX_TRIALS} trials")


def build_matrix(
    speeds: Sequence[float] = DEFAULT_SPEEDS,
    durations: Sequence[float] = DEFAULT_DURATIONS,
    repetitions: int = DEFAULT_REPETITIONS,
) -> list[Trial]:
    validate_values(speeds, durations, repetitions)
    rows: list[Trial] = []
    for speed in speeds:
        for duration in durations:
            for direction, sign in (("cw", -1.0), ("ccw", 1.0)):
                for repetition in range(1, repetitions + 1):
                    rows.append(
                        Trial(
                            len(rows),
                            direction,
                            sign * speed,
                            duration,
                            repetition,
                        )
                    )
    return rows


def validate_linear_values(
    speeds: Sequence[float],
    durations: Sequence[float],
    repetitions: int,
) -> None:
    if not speeds or not durations:
        raise ValueError("linear speeds and durations must be non-empty")
    if repetitions < 1 or repetitions > MAX_REPETITIONS:
        raise ValueError(f"repetitions must be in [1,{MAX_REPETITIONS}]")
    if (
        len(set(speeds)) != len(speeds)
        or len(set(durations)) != len(durations)
    ):
        raise ValueError(
            "duplicate linear speeds or durations are not allowed"
        )
    if any(
        not math.isfinite(value) or value <= 0 or value > MAX_LINEAR_SPEED
        for value in speeds
    ):
        raise ValueError(
            f"linear speeds must be finite and in (0,{MAX_LINEAR_SPEED}]"
        )
    if any(
        not math.isfinite(value) or value <= 0 or value > MAX_DURATION
        for value in durations
    ):
        raise ValueError(f"durations must be finite and in (0,{MAX_DURATION}]")
    if len(speeds) * len(durations) * 2 * repetitions > MAX_TRIALS:
        raise ValueError(f"campaign cannot exceed {MAX_TRIALS} trials")


def build_linear_matrix(
    speeds: Sequence[float] = DEFAULT_LINEAR_SPEEDS,
    durations: Sequence[float] = DEFAULT_DURATIONS,
    repetitions: int = 1,
) -> list[Trial]:
    validate_linear_values(speeds, durations, repetitions)
    rows: list[Trial] = []
    for speed in speeds:
        for duration in durations:
            for direction, sign in (("forward", 1.0), ("backward", -1.0)):
                for repetition in range(1, repetitions + 1):
                    rows.append(
                        Trial(
                            len(rows),
                            direction,
                            0.0,
                            duration,
                            repetition,
                            linear_x=sign * speed,
                            command_type="straight_line",
                        )
                    )
    return rows


def twist_components(linear_x: float, angular_z: float) -> dict:
    values = (linear_x, angular_z)
    if any(not math.isfinite(value) for value in values):
        raise ValueError("Twist value is non-finite")
    if abs(linear_x) > MAX_LINEAR_SPEED:
        raise ValueError("linear_x is outside the approved bound")
    if abs(angular_z) > MAX_SPEED:
        raise ValueError("angular_z is outside the approved bound")
    return {
        "linear": {"x": float(linear_x), "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": float(angular_z)},
    }


def twist(angular_z: float) -> dict:
    return twist_components(0.0, angular_z)


def trial_twist(trial: Trial) -> dict:
    return twist_components(trial.linear_x, trial.angular_z)


def is_exact_zero(payload: Mapping[str, Mapping[str, float]]) -> bool:
    return payload == twist(0.0)


def validate_preflight(snapshot: PreflightSnapshot) -> None:
    failures = []
    if snapshot.production_containers != (EXPECTED_SENSOR_CONTAINER,):
        failures.append(
            "exactly one expected production sensor container is required"
        )
    if snapshot.validation_containers:
        failures.append("validation container is running")
    if snapshot.foreign_d455_owners:
        failures.append("foreign D455 owner detected")
    if (
        not snapshot.apparmor_enforcing
        or snapshot.apparmor_profile != EXPECTED_APPARMOR
    ):
        failures.append("expected AppArmor profile is not enforcing")
    if not snapshot.sensor_container_id or not snapshot.sensor_image_id:
        failures.append("production image/container identity is unavailable")
    if (
        not snapshot.main_container_id
        or len(snapshot.main_immutable_config_sha256) != 64
    ):
        failures.append("main-container identity is unavailable")
    if len(snapshot.immutable_config_sha256) != 64:
        failures.append("immutable configuration fingerprint is invalid")
    expected = (
        (
            snapshot.raw_publishers,
            "raw publisher",
            "realsense_imu_relay",
            BEST_EFFORT_QOS,
        ),
        (
            snapshot.processed_publishers,
            "processed publisher",
            "d455_imu_processor",
            BEST_EFFORT_QOS,
        ),
        (
            snapshot.safe_publishers,
            "safe publisher",
            "command_arbiter",
            RELIABLE_QOS,
        ),
    )
    for endpoints, description, node, qos in expected:
        if (
            len(endpoints) != 1
            or endpoints[0].node != node
            or endpoints[0].qos != qos
        ):
            failures.append(f"expected exactly one {description} from {node}")
    if not any(
        endpoint.node == "d455_imu_processor"
        and endpoint.qos == RELIABLE_QOS
        for endpoint in snapshot.diagnostics_publishers
    ):
        failures.append("expected diagnostics publisher is missing")
    if (
        len(snapshot.input_subscribers) != 1
        or snapshot.input_subscribers[0].node != "command_arbiter"
        or snapshot.input_subscribers[0].qos != RELIABLE_QOS
    ):
        failures.append("expected command-arbiter input subscriber is missing")
    if snapshot.input_publishers:
        failures.append("pre-existing command-input publisher detected")
    expected_safe_nodes = {
        "d455_imu_processor",
        "roboteq_ros2_driver",
        "d455_rotation_validator",
    }
    safe_nodes = [endpoint.node for endpoint in snapshot.safe_subscribers]
    if (
        len(safe_nodes) != len(expected_safe_nodes)
        or set(safe_nodes) != expected_safe_nodes
        or any(safe_nodes.count(node) != 1 for node in expected_safe_nodes)
    ):
        failures.append("expected safe-command subscribers are missing")
    for endpoint in snapshot.safe_subscribers:
        expected_qos = (
            BEST_EFFORT_QOS
            if endpoint.node == "d455_imu_processor"
            else RELIABLE_QOS
        )
        if endpoint.qos != expected_qos:
            failures.append("safe-command subscriber QoS is incompatible")
    if snapshot.relay_nodes != ("realsense_imu_relay",):
        failures.append("expected relay node is missing or duplicated")
    if snapshot.processor_nodes != ("d455_imu_processor",):
        failures.append("expected processor node is missing or duplicated")
    if failures:
        raise ValidationError("; ".join(failures))


def plan_payload(trials: Sequence[Trial], rate_hz: float) -> dict:
    command_types = sorted({trial.command_type for trial in trials})
    payload = {
        "input_topic": INPUT_TOPIC,
        "safe_topic": SAFE_TOPIC,
        "raw_imu_topic": RAW_IMU_TOPIC,
        "processed_imu_topic": PROCESSED_IMU_TOPIC,
        "diagnostics_topic": DIAGNOSTICS_TOPIC,
        "rate_hz": rate_hz,
        "command_types": command_types,
        "trial_count": len(trials),
        "requested_angular_velocities_rad_s": sorted(
            {abs(float(trial.angular_z)) for trial in trials}
            if any(abs(float(trial.angular_z)) > 0.0 for trial in trials)
            else set()
        ),
        "requested_linear_velocities_m_s": sorted(
            {abs(float(trial.linear_x)) for trial in trials}
            if any(abs(float(trial.linear_x)) > 0.0 for trial in trials)
            else set()
        ),
        "matrix": [asdict(trial) for trial in trials],
    }
    encoded = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    payload["plan_sha256"] = hashlib.sha256(encoded).hexdigest()
    return payload


def canonical_json(payload: object) -> str:
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def payload_sha256(payload: object) -> str:
    return hashlib.sha256(canonical_json(payload).encode()).hexdigest()


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def endpoint_payload(endpoint: GraphEndpoint) -> dict:
    return asdict(endpoint)


def endpoint_from_payload(payload: Mapping[str, object]) -> GraphEndpoint:
    return GraphEndpoint(
        node=str(payload["node"]),
        qos=str(payload["qos"]),
        gid=str(payload.get("gid", "")),
    )


def ros_snapshot_payload(snapshot: RosPreflightSnapshot) -> dict:
    payload = asdict(snapshot)
    return payload


def ros_snapshot_from_payload(
    payload: Mapping[str, object],
) -> RosPreflightSnapshot:
    endpoint_fields = (
        "raw_publishers",
        "processed_publishers",
        "diagnostics_publishers",
        "safe_publishers",
        "input_publishers",
        "input_subscribers",
        "safe_subscribers",
    )
    values = {
        field_name: tuple(
            endpoint_from_payload(item)
            for item in payload[field_name]
        )
        for field_name in endpoint_fields
    }
    values["relay_nodes"] = tuple(payload["relay_nodes"])
    values["processor_nodes"] = tuple(payload["processor_nodes"])
    return RosPreflightSnapshot(**values)


def host_snapshot_payload(snapshot: HostPreflightSnapshot) -> dict:
    return asdict(snapshot)


def host_snapshot_from_payload(
    payload: Mapping[str, object],
) -> HostPreflightSnapshot:
    values = dict(payload)
    for field_name in (
        "production_containers",
        "validation_containers",
        "foreign_d455_owners",
    ):
        values[field_name] = tuple(values[field_name])
    return HostPreflightSnapshot(**values)


def compose_preflight(
    host: HostPreflightSnapshot,
    ros: RosPreflightSnapshot,
) -> PreflightSnapshot:
    return PreflightSnapshot(
        **{
            name: getattr(host, name)
            for name in HostPreflightSnapshot.__dataclass_fields__
        },
        **{
            name: getattr(ros, name)
            for name in RosPreflightSnapshot.__dataclass_fields__
        },
    )


def approval_graph_identity(snapshot: RosPreflightSnapshot) -> dict:
    """Normalize only the short-lived validator subscription identity."""
    payload = ros_snapshot_payload(snapshot)
    payload["safe_subscribers"] = [
        {
            **item,
            "gid": (
                ""
                if item["node"] == "d455_rotation_validator"
                else item["gid"]
            ),
        }
        for item in payload["safe_subscribers"]
    ]
    return payload


def approval_binding_payload(handoff: Mapping[str, object]) -> dict:
    return {
        "schema": handoff["schema"],
        "plan_sha256": handoff["plan_sha256"],
        "worker_sha256": handoff["worker_sha256"],
        "worker_source_path": handoff["worker_source_path"],
        "host_snapshot_sha256": handoff["host_snapshot_sha256"],
        "ros_snapshot_sha256": handoff["ros_snapshot_sha256"],
        "evidence_relative": handoff["evidence_relative"],
        "worker_evidence_path": handoff["worker_evidence_path"],
        "heartbeat_token": handoff["heartbeat_token"],
        "heartbeat_max_age_s": handoff["heartbeat_max_age_s"],
        "worker_container": handoff["worker_container"],
        "approval_text": APPROVAL_TEXT,
    }


def validate_plan_payload(plan: Mapping[str, object]) -> list[Trial]:
    plan_without_hash = dict(plan)
    claimed = str(plan_without_hash.pop("plan_sha256", ""))
    expected = payload_sha256(plan_without_hash)
    if claimed != expected:
        raise ValidationError("plan hash mismatch")
    expected_topics = {
        "input_topic": INPUT_TOPIC,
        "safe_topic": SAFE_TOPIC,
        "raw_imu_topic": RAW_IMU_TOPIC,
        "processed_imu_topic": PROCESSED_IMU_TOPIC,
        "diagnostics_topic": DIAGNOSTICS_TOPIC,
    }
    if any(plan.get(key) != value for key, value in expected_topics.items()):
        raise ValidationError("plan topic contract mismatch")
    rate_hz = float(plan["rate_hz"])
    if (
        not math.isfinite(rate_hz)
        or rate_hz < 10.0
        or rate_hz > 50.0
    ):
        raise ValidationError("plan publication rate is invalid")
    trials = [Trial(**row) for row in plan["matrix"]]
    if int(plan["trial_count"]) != len(trials):
        raise ValidationError("plan trial count mismatch")
    for expected_index, trial in enumerate(trials):
        if trial.index != expected_index:
            raise ValidationError("plan trial indexes are not contiguous")
        if trial.command_type not in {"rotation", "straight_line"}:
            raise ValidationError("plan command type is invalid")
        if trial.command_type == "rotation" and trial.linear_x != 0.0:
            raise ValidationError("rotation plan includes linear velocity")
        if trial.command_type == "straight_line" and trial.angular_z != 0.0:
            raise ValidationError(
                "straight-line plan includes angular velocity"
            )
        if trial_twist(trial)["angular"]["z"] != trial.angular_z:
            raise ValidationError("plan angular velocity is invalid")
        if trial_twist(trial)["linear"]["x"] != trial.linear_x:
            raise ValidationError("plan linear velocity is invalid")
        if (
            not math.isfinite(trial.duration_s)
            or trial.duration_s <= 0.0
            or trial.duration_s > MAX_DURATION
        ):
            raise ValidationError("plan duration is invalid")
    return trials


def atomic_json(path: Path, payload: object) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


class EvidenceWriter:
    def __init__(
        self,
        directory: Path,
        plan: Mapping[str, object],
        dry_run: bool,
    ):
        directory.mkdir(parents=True, exist_ok=False)
        self.directory = directory
        self.plan = dict(plan)
        self.dry_run = dry_run
        atomic_json(
            directory / "rotation-plan.json",
            {"dry_run": dry_run, **self.plan},
        )
        self.write_event("evidence_created", {"dry_run": dry_run})

    def write_event(
        self,
        event: str,
        values: Optional[Mapping[str, object]] = None,
    ) -> None:
        row = {
            "event": event,
            "monotonic_s": time.monotonic(),
            "wall_time_ns": time.time_ns(),
            "plan_sha256": self.plan["plan_sha256"],
            "values": dict(values or {}),
        }
        with (self.directory / "events.jsonl").open("a") as stream:
            stream.write(json.dumps(row, sort_keys=True) + "\n")

    def write_trial(self, evidence: TrialEvidence) -> None:
        self.write_event(
            "trial_result",
            {
                "trial_index": evidence.trial.index,
                "result": evidence.result,
                "zero_verified": evidence.zero_verified,
                "cleanup_error": evidence.cleanup_error,
            },
        )
        payload = asdict(evidence)
        atomic_json(
            self.directory / f"trial-{evidence.trial.index:02d}.json",
            payload,
        )
        command_path = (
            self.directory
            / f"trial-{evidence.trial.index:02d}-commands.csv"
        )
        with command_path.open("x", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                (
                    "monotonic_s",
                    "wall_time_ns",
                    "phase",
                    "angular_z",
                    "linear_x",
                    "linear_y",
                    "linear_z",
                )
            )
            for sample in evidence.command_samples:
                writer.writerow(
                    (
                        sample.monotonic_s,
                        sample.wall_time_ns,
                        sample.phase,
                        sample.payload["angular"]["z"],
                        sample.payload["linear"]["x"],
                        sample.payload["linear"]["y"],
                        sample.payload["linear"]["z"],
                    )
                )
        self._write_twist_csv(
            self.directory
            / f"trial-{evidence.trial.index:02d}-safe.csv",
            evidence.safe_samples,
        )
        self._write_imu_csv(
            self.directory
            / f"trial-{evidence.trial.index:02d}-raw-imu.csv",
            evidence.raw_imu,
        )
        self._write_imu_csv(
            self.directory
            / f"trial-{evidence.trial.index:02d}-processed-imu.csv",
            evidence.processed_imu,
        )

    @staticmethod
    def _write_twist_csv(path: Path, samples: Sequence[TimedTwist]) -> None:
        with path.open("x", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                (
                    "monotonic_s",
                    "wall_time_ns",
                    "angular_z",
                    "linear_x",
                    "linear_y",
                    "linear_z",
                )
            )
            for sample in samples:
                writer.writerow(
                    (
                        sample.monotonic_s,
                        sample.wall_time_ns,
                        sample.payload["angular"]["z"],
                        sample.payload["linear"]["x"],
                        sample.payload["linear"]["y"],
                        sample.payload["linear"]["z"],
                    )
                )

    @staticmethod
    def _write_imu_csv(path: Path, samples: Sequence[ImuSample]) -> None:
        with path.open("x", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                (
                    "monotonic_s",
                    "stamp_ns",
                    "frame_id",
                    "gyro_x",
                    "gyro_y",
                    "gyro_z",
                    "accel_x",
                    "accel_y",
                    "accel_z",
                )
            )
            for sample in samples:
                writer.writerow(
                    (
                        sample.monotonic_s,
                        sample.stamp_ns,
                        sample.frame_id,
                        *sample.angular_velocity,
                        *sample.linear_acceleration,
                    )
                )

    def finish(
        self,
        result: str,
        preflight: Optional[PreflightSnapshot],
        trials: Sequence[TrialEvidence],
        error: str = "",
        cleanup: Optional[Mapping[str, object]] = None,
    ) -> None:
        payload = {
            "result": result,
            "error": error,
            "dry_run": self.dry_run,
            "plan_sha256": self.plan["plan_sha256"],
            "preflight": asdict(preflight) if preflight else None,
            "trials": [asdict(item) for item in trials],
            "cleanup": dict(cleanup or {}),
        }
        atomic_json(self.directory / "summary.json", payload)
        lines = [
            f"result: {result}",
            f"dry_run: {self.dry_run}",
            f"plan_sha256: {self.plan['plan_sha256']}",
            f"completed_trials: {len(trials)}",
        ]
        if error:
            lines.append(f"error: {error}")
        temporary = self.directory / "summary.txt.tmp"
        temporary.write_text("\n".join(lines) + "\n")
        temporary.replace(self.directory / "summary.txt")


def integrate_gyro(samples: Iterable[tuple[float, float]]) -> dict:
    rows = list(samples)
    if len(rows) < 2:
        raise ValueError("at least two gyro samples are required")
    times = [float(stamp) for stamp, _ in rows]
    values = [float(value) for _, value in rows]
    if any(not math.isfinite(value) for value in times + values):
        raise ValueError("non-finite IMU sample")
    if any(after <= before for before, after in zip(times, times[1:])):
        raise ValueError("IMU timestamps must be strictly monotonic")
    yaw = sum(
        (after_t - before_t) * (before_v + after_v) * 0.5
        for (before_t, before_v), (after_t, after_v) in zip(rows, rows[1:])
    )
    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / len(values)
    return {
        "integrated_yaw_rad": yaw,
        "mean_gyro_z": mean,
        "stddev_gyro_z": math.sqrt(variance),
        "samples": len(rows),
        "rate_hz": (len(rows) - 1) / (times[-1] - times[0]),
    }


def validate_imu_sample(sample: ImuSample, expected_frame: str) -> None:
    values = (
        sample.angular_velocity
        + sample.linear_acceleration
        + sample.orientation_covariance
        + sample.angular_velocity_covariance
        + sample.linear_acceleration_covariance
    )
    if sample.frame_id != expected_frame:
        raise ValidationError(
            f"unexpected IMU frame {sample.frame_id!r}; "
            f"expected {expected_frame!r}"
        )
    if any(not math.isfinite(value) for value in values):
        raise ValidationError("non-finite IMU value or covariance")
    if (
        len(sample.orientation_covariance) != 9
        or sample.orientation_covariance[0] != -1.0
    ):
        raise ValidationError("IMU orientation is not marked unavailable")
    if (
        len(sample.angular_velocity_covariance) != 9
        or len(sample.linear_acceleration_covariance) != 9
    ):
        raise ValidationError("IMU covariance shape is invalid")


def diagnostics_are_acceptable(
    diagnostics: Sequence[Mapping[str, object]],
) -> bool:
    latest = {}
    for sample in diagnostics:
        name = str(sample.get("name", ""))
        if name.startswith("D455 IMU/"):
            latest[name] = int(sample.get("level", 3))
    expected_maximum = {
        "D455 IMU/Raw Input": 0,
        "D455 IMU/Processed Output": 0,
        "D455 IMU/Bias": 0,
        "D455 IMU/Transform": 1,
        "D455 IMU/Covariance": 1,
    }
    return all(
        name in latest and latest[name] <= maximum
        for name, maximum in expected_maximum.items()
    )


def wheel_symmetry(encoder: Sequence[Mapping[str, object]]) -> dict:
    """Summarize numeric left/right encoder feedback when available."""
    rows = []
    for sample in encoder:
        message = str(sample.get("message", ""))
        left = re.search(
            r"(?:left|channel_1|motor_1)[_ ]?(?:ticks|count)"
            r"\s*[:=]\s*(-?\d+(?:\.\d+)?)",
            message,
            re.I,
        )
        right = re.search(
            r"(?:right|channel_2|motor_2)[_ ]?(?:ticks|count)"
            r"\s*[:=]\s*(-?\d+(?:\.\d+)?)",
            message,
            re.I,
        )
        if left and right:
            rows.append((float(left.group(1)), float(right.group(1))))
    if len(rows) < 2:
        return {
            "available": False,
            "reason": "numeric left/right encoder fields unavailable",
        }
    left_delta = rows[-1][0] - rows[0][0]
    right_delta = rows[-1][1] - rows[0][1]
    magnitude = max(abs(left_delta), abs(right_delta))
    opposite_direction = (
        (left_delta == 0.0 and right_delta == 0.0)
        or math.copysign(1.0, left_delta)
        != math.copysign(1.0, right_delta)
    )
    return {
        "available": True,
        "left_delta": left_delta,
        "right_delta": right_delta,
        "opposite_direction": opposite_direction,
        "same_direction": not opposite_direction,
        "magnitude_ratio": (
            min(abs(left_delta), abs(right_delta)) / magnitude
            if magnitude else 1.0
        ),
        "equal_magnitude": math.isclose(
            abs(left_delta), abs(right_delta), rel_tol=0.2, abs_tol=1.0
        ),
    }


def expected_wheel_rpm(linear_x: float) -> float:
    return linear_x / (2.0 * math.pi * WHEEL_RADIUS_M) * 60.0


def analyze_straight_line_trial(
    evidence: TrialEvidence,
) -> Mapping[str, object]:
    expected = trial_twist(evidence.trial)
    if any(sample.payload != expected for sample in evidence.safe_samples):
        raise ValidationError("/cmd_vel/safe differed during motion")
    if not diagnostics_are_acceptable(evidence.diagnostics):
        raise ValidationError("D455 diagnostic state is unacceptable")
    wheels = wheel_symmetry(evidence.encoder)
    if not wheels["available"]:
        raise ValidationError("left/right wheel feedback is unavailable")
    left_delta = float(wheels["left_delta"])
    right_delta = float(wheels["right_delta"])
    linear_x = float(evidence.trial.linear_x)
    expected_sign = math.copysign(1.0, linear_x)
    if left_delta == 0.0 or right_delta == 0.0:
        raise ValidationError("one or both wheels remained stationary")
    if (
        math.copysign(1.0, left_delta) != expected_sign
        or math.copysign(1.0, right_delta) != expected_sign
    ):
        raise ValidationError("wheel encoder sign is incorrect")
    if float(wheels["magnitude_ratio"]) < 0.70:
        raise ValidationError("left/right wheel feedback is asymmetric")
    if len(evidence.odometry) < 2:
        raise ValidationError("odometry evidence is required")
    first = evidence.odometry[0]
    last = evidence.odometry[-1]
    delta_x = float(last.get("x_m", 0.0)) - float(first.get("x_m", 0.0))
    delta_y = float(last.get("y_m", 0.0)) - float(first.get("y_m", 0.0))
    delta_yaw = float(last.get("yaw_rad", 0.0)) - float(
        first.get("yaw_rad", 0.0)
    )
    if delta_x == 0.0 or math.copysign(1.0, delta_x) != expected_sign:
        raise ValidationError("odometry linear sign is incorrect")
    odometry_analysis = {
        "delta_x_m": delta_x,
        "delta_y_m": delta_y,
        "delta_yaw_rad": math.atan2(
            math.sin(delta_yaw),
            math.cos(delta_yaw),
        ),
        "expected_delta_x_m": linear_x * evidence.trial.duration_s,
        "x_ratio_to_command": abs(
            delta_x / (linear_x * evidence.trial.duration_s)
        ),
    }
    wheel_rpm = expected_wheel_rpm(linear_x)
    return {
        "command_interval": {
            "expected_delta_x_m": linear_x * evidence.trial.duration_s,
            "commanded_linear_velocity_m_s": linear_x,
            "commanded_angular_velocity_rad_s": 0.0,
            "expected_left_wheel_rpm": wheel_rpm,
            "expected_right_wheel_rpm": wheel_rpm,
        },
        "wheel_symmetry": {
            **wheels,
            "same_direction": True,
            "left_right_movement_ratio": wheels["magnitude_ratio"],
            "expected_sign": "forward" if expected_sign > 0 else "reverse",
        },
        "odometry": odometry_analysis,
    }


def analyze_trial(evidence: TrialEvidence) -> Mapping[str, object]:
    if evidence.trial.command_type == "straight_line":
        return analyze_straight_line_trial(evidence)
    for sample in evidence.raw_imu:
        validate_imu_sample(sample, EXPECTED_RAW_FRAME)
    for sample in evidence.processed_imu:
        validate_imu_sample(sample, EXPECTED_PROCESSED_FRAME)
    for samples in (evidence.raw_imu, evidence.processed_imu):
        if any(
            after.stamp_ns <= before.stamp_ns
            for before, after in zip(samples, samples[1:])
        ):
            raise ValidationError("IMU header timestamps are not monotonic")
    if not evidence.safe_samples:
        raise ValidationError("motor-facing safe-command evidence is required")
    expected = twist(evidence.trial.angular_z)
    if any(sample.payload != expected for sample in evidence.safe_samples):
        raise ValidationError("/cmd_vel/safe differed during motion")
    if not diagnostics_are_acceptable(evidence.diagnostics):
        raise ValidationError("D455 diagnostic state is unacceptable")

    # The production transform maps raw optical Y to robot-frame yaw Z.
    raw_analysis = integrate_gyro(
        (sample.monotonic_s, sample.angular_velocity[1])
        for sample in evidence.raw_imu
    )
    processed_analysis = integrate_gyro(
        (sample.monotonic_s, sample.angular_velocity[2])
        for sample in evidence.processed_imu
    )
    measured = float(processed_analysis["integrated_yaw_rad"])
    expected_yaw = evidence.expected_yaw_rad
    if measured == 0.0 or math.copysign(1.0, measured) != math.copysign(
        1.0,
        expected_yaw,
    ):
        raise ValidationError("processed IMU yaw sign is incorrect")
    ratio = abs(measured / expected_yaw)
    if ratio < MIN_YAW_RATIO or ratio > MAX_YAW_RATIO:
        raise ValidationError("processed IMU yaw magnitude is implausible")
    raw_yaw = float(raw_analysis["integrated_yaw_rad"])
    raw_ratio = abs(raw_yaw / measured)
    if (
        raw_yaw == 0.0
        or math.copysign(1.0, raw_yaw) != math.copysign(1.0, measured)
        or raw_ratio < MIN_YAW_RATIO
        or raw_ratio > MAX_YAW_RATIO
    ):
        raise ValidationError("raw and processed IMU yaw disagree")

    odometry_analysis = None
    if len(evidence.odometry) >= 2:
        first = float(evidence.odometry[0]["yaw_rad"])
        last = float(evidence.odometry[-1]["yaw_rad"])
        delta = last - first
        odometry_yaw = math.atan2(math.sin(delta), math.cos(delta))
        if odometry_yaw == 0.0 or math.copysign(
            1.0,
            odometry_yaw,
        ) != math.copysign(1.0, measured):
            raise ValidationError("odometry and IMU yaw signs disagree")
        odometry_analysis = {
            "integrated_yaw_rad": odometry_yaw,
            "imu_to_odometry_ratio": abs(measured / odometry_yaw),
        }
        ratio = odometry_analysis["imu_to_odometry_ratio"]
        if ratio < MIN_YAW_RATIO or ratio > MAX_YAW_RATIO:
            raise ValidationError("odometry and IMU yaw magnitudes disagree")

    wheel_analysis = wheel_symmetry(evidence.encoder)
    if wheel_analysis["available"] and (
        not wheel_analysis["equal_magnitude"]
        or not wheel_analysis["opposite_direction"]
    ):
        raise ValidationError("left/right wheel feedback is asymmetric")

    return {
        "command_interval": {
            "expected_yaw_rad": expected_yaw,
            "commanded_angular_velocity_rad_s": abs(
                float(evidence.trial.angular_z)
            ),
        },
        "raw_imu": raw_analysis,
        "processed_imu": processed_analysis,
        "sign_correct": True,
        "yaw_ratio_to_command": abs(measured / expected_yaw),
        "commanded_angular_velocity_rad_s": abs(
            float(evidence.trial.angular_z)
        ),
        "raw_to_processed_yaw_ratio": raw_ratio,
        "odometry": odometry_analysis,
        "wheel_symmetry": wheel_analysis,
    }


class Campaign:
    def __init__(
        self,
        runtime: Runtime,
        writer: EvidenceWriter,
        rate_hz: float = DEFAULT_RATE_HZ,
        settle_timeout_s: float = DEFAULT_SETTLE_TIMEOUT_S,
        zero_duration_s: float = DEFAULT_ZERO_DURATION_S,
    ):
        if not math.isfinite(rate_hz) or rate_hz < 10.0 or rate_hz > 50.0:
            raise ValueError("rate_hz must be finite and in [10,50]")
        self.runtime = runtime
        self.writer = writer
        self.rate_hz = rate_hz
        self.period_s = 1.0 / rate_hz
        self.settle_timeout_s = settle_timeout_s
        self.zero_duration_s = zero_duration_s
        self.publisher_created = False
        self.cleanup_records: list[TimedTwist] = []
        self.final_zero_verified = False

    def _publish_for(
        self,
        payload: Mapping[str, Mapping[str, float]],
        duration_s: float,
        phase: str,
        trial_index: Optional[int],
        records: list[TimedTwist],
        safe_records: Optional[list[TimedTwist]] = None,
    ) -> None:
        count = max(1, int(math.ceil(duration_s * self.rate_hz)))
        started = self.runtime.monotonic()
        for index in range(count):
            target = started + index * self.period_s
            current = self.runtime.monotonic()
            lateness = current - target
            if lateness > self.period_s * MAX_PUBLISH_LATENESS_PERIODS:
                raise ValidationError("command publication deadline missed")
            delay = target - current
            if delay > 0:
                self.runtime.sleep(delay)
            if not is_exact_zero(payload):
                self.runtime.assert_motion_authorized()
            sample = TimedTwist(
                self.runtime.monotonic(),
                self.runtime.wall_time_ns(),
                payload,
                phase,
                trial_index,
            )
            self.runtime.publish(payload)
            records.append(sample)
            if safe_records is not None:
                observed = self.runtime.observe_safe(
                    sample.monotonic_s,
                    self.period_s,
                )
                if (
                    observed.monotonic_s < sample.monotonic_s
                    or observed.payload != payload
                ):
                    raise ValidationError(
                        "/cmd_vel/safe did not match the commanded Twist"
                    )
                safe_records.append(observed)
        remaining = started + duration_s - self.runtime.monotonic()
        if remaining > 0:
            self.runtime.sleep(remaining)

    def _verified_zero(
        self,
        trial_index: Optional[int],
        records: list[TimedTwist],
        motion_end_s: float,
    ) -> tuple[float, float]:
        started = self.runtime.monotonic()
        deadline = started + self.settle_timeout_s
        first_zero = None
        consecutive = 0
        while self.runtime.monotonic() < deadline:
            cycle = self.runtime.monotonic()
            self.runtime.publish(twist(0.0))
            records.append(
                TimedTwist(
                    cycle,
                    self.runtime.wall_time_ns(),
                    twist(0.0),
                    "stop",
                    trial_index,
                )
            )
            safe = self.runtime.observe_safe(cycle, self.period_s)
            if safe.monotonic_s < cycle:
                raise ValidationError("stale /cmd_vel/safe sample during stop")
            if is_exact_zero(safe.payload):
                first_zero = (
                    first_zero
                    if first_zero is not None
                    else safe.monotonic_s
                )
                consecutive += 1
            else:
                consecutive = 0
            required = max(
                3,
                int(math.ceil(self.zero_duration_s * self.rate_hz)),
            )
            if consecutive >= required:
                if self.runtime.stationary(cycle, self.period_s):
                    return (
                        first_zero - motion_end_s,
                        self.runtime.monotonic() - started,
                    )
            delay = cycle + self.period_s - self.runtime.monotonic()
            if delay > 0:
                self.runtime.sleep(delay)
        raise ValidationError(
            "exact-zero /cmd_vel/safe or stationary verification timed out"
        )

    def run(self, trials: Sequence[Trial]) -> list[TrialEvidence]:
        self.completed: list[TrialEvidence] = []
        self.snapshot: Optional[PreflightSnapshot] = None
        self.cleanup_error = ""
        original_error: Optional[BaseException] = None
        try:
            self.snapshot = self.runtime.preflight_snapshot()
            validate_preflight(self.snapshot)
            self.publisher_created = True
            self.runtime.create_publisher()
            self.runtime.verify_runtime_identity(self.snapshot)
            self._publish_for(
                twist(0.0),
                self.zero_duration_s,
                "preflight-zero",
                None,
                self.cleanup_records,
            )
            self._verified_zero(
                None,
                self.cleanup_records,
                self.runtime.monotonic(),
            )
            for trial in trials:
                evidence = TrialEvidence(
                    trial=trial,
                    expected_yaw_rad=trial.angular_z * trial.duration_s,
                )
                try:
                    self._publish_for(
                        twist(0.0),
                        self.zero_duration_s,
                        "pre-trial-zero",
                        trial.index,
                        evidence.command_samples,
                    )
                    zero_start = self.runtime.monotonic()
                    self._verified_zero(
                        trial.index,
                        evidence.command_samples,
                        zero_start,
                    )
                    motion_start = self.runtime.monotonic()
                    self._publish_for(
                        trial_twist(trial),
                        trial.duration_s,
                        "motion",
                        trial.index,
                        evidence.command_samples,
                        evidence.safe_samples,
                    )
                    motion_end = self.runtime.monotonic()
                    stop_latency, settling = self._verified_zero(
                        trial.index,
                        evidence.command_samples,
                        motion_end,
                    )
                    evidence.stop_latency_s = stop_latency
                    evidence.settling_time_s = settling
                    evidence.zero_verified = True
                    capture = self.runtime.capture_trial(
                        trial,
                        motion_start,
                        motion_end,
                    )
                    captured_safe = list(capture.get("safe", ()))
                    evidence.safe_samples.extend(captured_safe)
                    evidence.raw_imu = list(capture.get("raw_imu", ()))
                    evidence.processed_imu = list(
                        capture.get("processed_imu", ())
                    )
                    evidence.diagnostics = list(capture.get("diagnostics", ()))
                    evidence.encoder = list(capture.get("encoder", ()))
                    evidence.odometry = list(capture.get("odometry", ()))
                    if not evidence.raw_imu or not evidence.processed_imu:
                        raise ValidationError(
                            "raw and processed IMU evidence is required"
                        )
                    if not evidence.diagnostics:
                        raise ValidationError(
                            "diagnostic evidence is required"
                        )
                    analysis = dict(analyze_trial(evidence))
                    analysis["command_interval"].update(
                        {
                            "start_monotonic_s": motion_start,
                            "end_monotonic_s": motion_end,
                            "duration_s": motion_end - motion_start,
                        }
                    )
                    evidence.analysis = analysis
                    self.runtime.verify_runtime_identity(self.snapshot)
                    evidence.result = "passed"
                except BaseException as exc:
                    evidence.result = "failed"
                    evidence.error = str(exc)
                    try:
                        self._verified_zero(
                            trial.index,
                            evidence.command_samples,
                            self.runtime.monotonic(),
                        )
                        evidence.zero_verified = True
                    except BaseException as cleanup_exc:
                        evidence.cleanup_error = str(cleanup_exc)
                        evidence.error += f"; cleanup failed: {cleanup_exc}"
                    self.writer.write_trial(evidence)
                    self.completed.append(evidence)
                    raise
                self.writer.write_trial(evidence)
                self.completed.append(evidence)
        except BaseException as exc:
            original_error = exc
        finally:
            cleanup_errors = []
            if self.publisher_created:
                try:
                    self._verified_zero(
                        None,
                        self.cleanup_records,
                        self.runtime.monotonic(),
                    )
                    self.runtime.verify_runtime_identity(self.snapshot)
                    self.final_zero_verified = True
                except BaseException as exc:
                    cleanup_errors.append(f"final zero failed: {exc}")
            try:
                self.runtime.close()
            except BaseException as exc:
                cleanup_errors.append(f"runtime close failed: {exc}")
            self.cleanup_error = "; ".join(cleanup_errors)
        if original_error is not None:
            if self.cleanup_error:
                raise ValidationError(
                    f"{original_error}; {self.cleanup_error}"
                ) from original_error
            raise original_error
        if self.cleanup_error:
            raise ValidationError(self.cleanup_error)
        return self.completed


def require_operator_approval(
    enabled: bool,
    input_fn: Callable[[str], str] = input,
    binding_sha256: str = "",
) -> None:
    if not enabled:
        raise ValidationError("--enable-motion is required")
    binding_text = (
        f" Binding SHA256: {binding_sha256}."
        if binding_sha256
        else ""
    )
    response = input_fn(
        f"{binding_text} Type exactly {APPROVAL_TEXT} "
        "to authorize the printed plan: "
    )
    print(
        json.dumps(
            {
                "approval_input_debug": {
                    "repr": repr(response),
                    "stripped_repr": repr(response.strip()),
                    "codepoints": [ord(character) for character in response],
                }
            },
            sort_keys=True,
        )
    )
    if response.strip() != APPROVAL_TEXT:
        raise ValidationError("operator confirmation did not match exactly")


class HeartbeatGuard:
    """Reject nonzero publication when the host heartbeat stream is absent."""

    def __init__(
        self,
        token: str,
        max_age_s: float,
        *,
        wall_time_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        if (
            not token
            or not math.isfinite(max_age_s)
            or max_age_s <= 0.0
            or max_age_s > 1.0
        ):
            raise ValidationError("heartbeat contract is invalid")
        self.token = token
        self.max_age_ns = int(max_age_s * 1e9)
        self.wall_time_ns = wall_time_ns
        self._lock = threading.Lock()
        self._updated_ns = 0
        self._closed = False

    def accept(self, line: str) -> None:
        try:
            payload = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValidationError("host heartbeat is invalid") from exc
        if payload.get("token") != self.token:
            raise ValidationError("host heartbeat token mismatch")
        updated_ns = int(payload.get("wall_time_ns", 0))
        with self._lock:
            self._updated_ns = updated_ns

    def start(self, stream) -> None:
        def read_stream() -> None:
            try:
                for line in stream:
                    self.accept(line)
            except (OSError, ValidationError, ValueError):
                pass
            finally:
                with self._lock:
                    self._closed = True

        threading.Thread(
            target=read_stream,
            name="d455-host-heartbeat",
            daemon=True,
        ).start()

    def check(self) -> None:
        with self._lock:
            updated_ns = self._updated_ns
            closed = self._closed
        if closed or updated_ns <= 0:
            raise ValidationError("host heartbeat is unavailable")
        age_ns = self.wall_time_ns() - updated_ns
        if age_ns < 0 or age_ns > self.max_age_ns:
            raise ValidationError("host heartbeat is stale")


class RosWorkerRuntime:
    """ROS-only adapter instantiated inside the approved main container."""

    def __init__(
        self,
        host_snapshot: HostPreflightSnapshot,
        approved_graph: Optional[Mapping[str, object]] = None,
        heartbeat_guard: Optional[HeartbeatGuard] = None,
    ) -> None:
        try:
            import rclpy
            from diagnostic_msgs.msg import DiagnosticArray
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Imu
        except ImportError as exc:
            raise ValidationError(
                "ROS runtime dependencies are unavailable"
            ) from exc
        self.rclpy = rclpy
        self.Twist = Twist
        self.host_snapshot = host_snapshot
        self.approved_graph = (
            dict(approved_graph) if approved_graph is not None else None
        )
        self.heartbeat_guard = heartbeat_guard
        self.rclpy.init()
        self.node = self.rclpy.create_node("d455_rotation_validator")
        self.publisher = None
        self.graph_convergence: Optional[GraphConvergence] = None
        self.safe: list[TimedTwist] = []
        self.raw: list[ImuSample] = []
        self.processed: list[ImuSample] = []
        self.diagnostics: list[Mapping[str, object]] = []
        self.odometry: list[Mapping[str, object]] = []
        self.encoder: list[Mapping[str, object]] = []
        self.node.create_subscription(
            Twist,
            SAFE_TOPIC,
            self._safe_callback,
            10,
        )
        self.node.create_subscription(
            Imu,
            RAW_IMU_TOPIC,
            lambda message: self.raw.append(self._imu_sample(message)),
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            Imu,
            PROCESSED_IMU_TOPIC,
            lambda message: self.processed.append(
                self._imu_sample(message)
            ),
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            DiagnosticArray,
            DIAGNOSTICS_TOPIC,
            self._diagnostic_callback,
            10,
        )
        self.node.create_subscription(
            Odometry,
            "/odom",
            self._odometry_callback,
            10,
        )
        try:
            from roboteq_ros2_driver.msg import WheelTicks

            self.node.create_subscription(
                WheelTicks,
                "/wheel_ticks",
                self._encoder_callback,
                10,
            )
        except ImportError:
            pass

    def monotonic(self) -> float:
        return time.monotonic()

    def wall_time_ns(self) -> int:
        return time.time_ns()

    def _spin(self, timeout_s: float = 0.0) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=timeout_s)

    def sleep(self, duration_s: float) -> None:
        deadline = self.monotonic() + duration_s
        while self.monotonic() < deadline:
            self._spin(min(0.01, deadline - self.monotonic()))

    @staticmethod
    def _qos_label(profile) -> str:
        reliability = str(profile.reliability).lower()
        durability = str(profile.durability).lower()
        reliability = (
            "best_effort"
            if "best_effort" in reliability
            else "reliable"
        )
        durability = (
            "transient_local"
            if "transient_local" in durability
            else "volatile"
        )
        return f"{reliability}/{durability}"

    def _endpoints(self, topic: str, publishers: bool) -> tuple:
        getter = (
            self.node.get_publishers_info_by_topic
            if publishers
            else self.node.get_subscriptions_info_by_topic
        )
        rows = []
        for info in getter(topic):
            gid = getattr(info, "endpoint_gid", b"")
            if isinstance(gid, bytes):
                gid = gid.hex()
            rows.append(
                GraphEndpoint(
                    str(info.node_name).lstrip("/"),
                    self._qos_label(info.qos_profile),
                    str(gid),
                )
            )
        return tuple(
            sorted(rows, key=lambda item: (item.node, item.qos, item.gid))
        )

    def ros_preflight_snapshot(self) -> RosPreflightSnapshot:
        node_names = {
            str(name).lstrip("/")
            for name, _namespace in self.node.get_node_names_and_namespaces()
        }
        return RosPreflightSnapshot(
            raw_publishers=self._endpoints(RAW_IMU_TOPIC, True),
            processed_publishers=self._endpoints(
                PROCESSED_IMU_TOPIC,
                True,
            ),
            diagnostics_publishers=self._endpoints(
                DIAGNOSTICS_TOPIC,
                True,
            ),
            safe_publishers=self._endpoints(SAFE_TOPIC, True),
            input_publishers=self._endpoints(INPUT_TOPIC, True),
            input_subscribers=self._endpoints(INPUT_TOPIC, False),
            safe_subscribers=self._endpoints(SAFE_TOPIC, False),
            relay_nodes=tuple(
                name
                for name in ("realsense_imu_relay",)
                if name in node_names
            ),
            processor_nodes=tuple(
                name
                for name in ("d455_imu_processor",)
                if name in node_names
            ),
        )

    def converge_graph(
        self,
        host_snapshot: HostPreflightSnapshot,
        *,
        timeout_s: float = DEFAULT_GRAPH_CONVERGENCE_TIMEOUT_S,
        poll_s: float = DEFAULT_GRAPH_CONVERGENCE_POLL_S,
    ) -> RosPreflightSnapshot:
        """Wait for the existing graph without creating a publisher."""
        if (
            not math.isfinite(timeout_s)
            or timeout_s <= 0
            or timeout_s > MAX_GRAPH_CONVERGENCE_TIMEOUT_S
        ):
            raise ValueError(
                "graph convergence timeout is outside the bounded range"
            )
        if (
            not math.isfinite(poll_s)
            or poll_s <= 0
            or poll_s > MAX_GRAPH_CONVERGENCE_POLL_S
        ):
            raise ValueError(
                "graph convergence poll interval is outside the bounded range"
            )
        started = self.monotonic()
        started_wall = self.wall_time_ns()
        attempts: list[Mapping[str, object]] = []
        attempt_no = 0
        while True:
            attempt_no += 1
            attempt_started = self.monotonic()
            attempt_started_wall = self.wall_time_ns()
            snapshot = self.ros_preflight_snapshot()
            failures: list[str] = []
            try:
                validate_preflight(compose_preflight(host_snapshot, snapshot))
            except ValidationError as exc:
                failures = [item.strip() for item in str(exc).split(";")]
            attempt_finished = self.monotonic()
            attempts.append(
                {
                    "attempt": attempt_no,
                    "monotonic_start_s": attempt_started,
                    "monotonic_end_s": attempt_finished,
                    "wall_time_start_ns": attempt_started_wall,
                    "wall_time_end_ns": self.wall_time_ns(),
                    "observed": ros_snapshot_payload(snapshot),
                    "missing_or_conflicting": failures,
                    "result": "passed" if not failures else "partial",
                }
            )
            if not failures:
                self.graph_convergence = GraphConvergence(
                    tuple(attempts),
                    started,
                    started_wall,
                    attempt_finished,
                    self.wall_time_ns(),
                    "converged",
                    attempt_finished - started,
                )
                return snapshot
            elapsed = attempt_finished - started
            if elapsed >= timeout_s:
                self.graph_convergence = GraphConvergence(
                    tuple(attempts),
                    started,
                    started_wall,
                    attempt_finished,
                    self.wall_time_ns(),
                    "timeout",
                    attempt_finished - started,
                )
                raise ValidationError(
                    "ROS graph convergence timed out: "
                    + "; ".join(failures)
                )
            self.sleep(min(poll_s, timeout_s - elapsed))

    def preflight_snapshot(self) -> PreflightSnapshot:
        if hasattr(self, "host_snapshot"):
            ros_snapshot = self.converge_graph(self.host_snapshot)
        else:
            # Keep lightweight injected adapters usable in offline tests.
            ros_snapshot = self.ros_preflight_snapshot()
        if (
            self.approved_graph is not None
            and approval_graph_identity(ros_snapshot)
            != self.approved_graph
        ):
            raise ValidationError(
                "ROS graph differs from the approved preflight"
            )
        return compose_preflight(self.host_snapshot, ros_snapshot)

    def create_publisher(self) -> None:
        self.publisher = self.node.create_publisher(
            self.Twist,
            INPUT_TOPIC,
            10,
        )

    def assert_motion_authorized(self) -> None:
        if self.heartbeat_guard is None:
            raise ValidationError("host heartbeat guard is missing")
        self.heartbeat_guard.check()

    def verify_runtime_identity(
        self,
        expected: PreflightSnapshot,
    ) -> None:
        input_publishers = self._endpoints(INPUT_TOPIC, True)
        if (
            len(input_publishers) != 1
            or input_publishers[0].node != "d455_rotation_validator"
            or input_publishers[0].qos != RELIABLE_QOS
        ):
            raise ValidationError(
                "validator is not the sole command-input publisher"
            )
        for topic, wanted in (
            (RAW_IMU_TOPIC, expected.raw_publishers),
            (PROCESSED_IMU_TOPIC, expected.processed_publishers),
            (DIAGNOSTICS_TOPIC, expected.diagnostics_publishers),
            (SAFE_TOPIC, expected.safe_publishers),
        ):
            if self._endpoints(topic, True) != wanted:
                raise ValidationError(
                    f"publisher identity drift detected on {topic}"
                )
        if (
            self._endpoints(INPUT_TOPIC, False)
            != expected.input_subscribers
            or self._endpoints(SAFE_TOPIC, False)
            != expected.safe_subscribers
        ):
            raise ValidationError("subscriber identity drift detected")

    def publish(
        self,
        payload: Mapping[str, Mapping[str, float]],
    ) -> None:
        if self.publisher is None:
            raise ValidationError("command publisher was not created")
        message = self.Twist()
        message.linear.x = payload["linear"]["x"]
        message.linear.y = payload["linear"]["y"]
        message.linear.z = payload["linear"]["z"]
        message.angular.x = payload["angular"]["x"]
        message.angular.y = payload["angular"]["y"]
        message.angular.z = payload["angular"]["z"]
        self.publisher.publish(message)
        self._spin(0.0)

    def _safe_callback(self, message) -> None:
        payload = {
            "linear": {
                "x": float(message.linear.x),
                "y": float(message.linear.y),
                "z": float(message.linear.z),
            },
            "angular": {
                "x": float(message.angular.x),
                "y": float(message.angular.y),
                "z": float(message.angular.z),
            },
        }
        self.safe.append(
            TimedTwist(
                self.monotonic(),
                self.wall_time_ns(),
                payload,
                "safe",
                None,
            )
        )

    def observe_safe(
        self,
        newer_than_s: float,
        timeout_s: float,
    ) -> TimedTwist:
        deadline = self.monotonic() + timeout_s
        while self.monotonic() < deadline:
            self._spin(min(0.01, deadline - self.monotonic()))
            if self.safe and self.safe[-1].monotonic_s >= newer_than_s:
                return self.safe[-1]
        raise ValidationError("fresh /cmd_vel/safe observation timed out")

    def stationary(
        self,
        newer_than_s: float,
        timeout_s: float,
    ) -> bool:
        deadline = self.monotonic() + timeout_s
        while self.monotonic() < deadline:
            self._spin(min(0.01, deadline - self.monotonic()))
            samples = [
                sample
                for sample in self.processed
                if sample.monotonic_s >= newer_than_s
            ]
            if len(samples) >= 10 and all(
                math.sqrt(
                    sum(
                        value * value
                        for value in sample.angular_velocity
                    )
                )
                < 0.05
                for sample in samples[-10:]
            ):
                return True
        return False

    def _imu_sample(self, message) -> ImuSample:
        return ImuSample(
            self.monotonic(),
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec),
            str(message.header.frame_id),
            (
                float(message.angular_velocity.x),
                float(message.angular_velocity.y),
                float(message.angular_velocity.z),
            ),
            (
                float(message.linear_acceleration.x),
                float(message.linear_acceleration.y),
                float(message.linear_acceleration.z),
            ),
            tuple(float(value) for value in message.orientation_covariance),
            tuple(
                float(value)
                for value in message.angular_velocity_covariance
            ),
            tuple(
                float(value)
                for value in message.linear_acceleration_covariance
            ),
        )

    def _diagnostic_callback(self, message) -> None:
        observed = self.monotonic()
        for status in message.status:
            level = status.level
            if isinstance(level, bytes):
                if len(level) != 1:
                    raise ValidationError(
                        "diagnostic level byte width is invalid"
                    )
                level = level[0]
            self.diagnostics.append(
                {
                    "monotonic_s": observed,
                    "name": str(status.name),
                    "level": int(level),
                    "message": str(status.message),
                    "values": {
                        str(value.key): str(value.value)
                        for value in status.values
                    },
                }
            )

    def _odometry_callback(self, message) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        self.odometry.append(
            {
                "monotonic_s": self.monotonic(),
                "x_m": float(position.x),
                "y_m": float(position.y),
                "yaw_rad": math.atan2(sin_yaw, cos_yaw),
            }
        )

    def _encoder_callback(self, message) -> None:
        self.encoder.append(
            {
                "monotonic_s": self.monotonic(),
                "message": str(message),
            }
        )

    def capture_trial(
        self,
        trial: Trial,
        start_s: float,
        end_s: float,
    ) -> Mapping[str, object]:
        del trial

        def bounded(rows):
            return [
                row
                for row in rows
                if start_s <= (
                    row.monotonic_s
                    if hasattr(row, "monotonic_s")
                    else float(row["monotonic_s"])
                ) <= end_s
            ]

        return {
            "safe": bounded(self.safe),
            "raw_imu": bounded(self.raw),
            "processed_imu": bounded(self.processed),
            "diagnostics": bounded(self.diagnostics),
            "encoder": bounded(self.encoder),
            "odometry": bounded(self.odometry),
        }

    def close(self) -> None:
        try:
            self.node.destroy_node()
        finally:
            if self.rclpy.ok():
                self.rclpy.shutdown()


@dataclass(frozen=True)
class CommandResult:
    returncode: int
    stdout: str = ""
    stderr: str = ""


class CommandRunner(Protocol):
    def run(self, args: Sequence[str]) -> CommandResult: ...

    def run_supervised(
        self,
        args: Sequence[str],
        heartbeat_token: str,
    ) -> CommandResult: ...


class SubprocessRunner:
    def run(self, args: Sequence[str]) -> CommandResult:
        completed = subprocess.run(
            list(args),
            check=False,
            capture_output=True,
            text=True,
        )
        return CommandResult(
            completed.returncode,
            completed.stdout,
            completed.stderr,
        )

    def run_supervised(
        self,
        args: Sequence[str],
        heartbeat_token: str,
    ) -> CommandResult:
        process = subprocess.Popen(
            list(args),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        try:
            while process.poll() is None:
                heartbeat = canonical_json(
                    {
                        "token": heartbeat_token,
                        "wall_time_ns": time.time_ns(),
                    }
                )
                if process.stdin is None:
                    raise ValidationError(
                        "worker heartbeat stream is unavailable"
                    )
                try:
                    process.stdin.write(heartbeat + "\n")
                    process.stdin.flush()
                except BrokenPipeError:
                    process.stdin.close()
                    process.stdin = None
                    if process.poll() is None:
                        raise
                    stdout, stderr = process.communicate()
                    return CommandResult(
                        process.returncode,
                        stdout,
                        stderr,
                    )
                time.sleep(HEARTBEAT_INTERVAL_S)
            stdout, stderr = process.communicate()
            return CommandResult(process.returncode, stdout, stderr)
        except BaseException:
            if process.stdin is not None:
                process.stdin.close()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=3.0)
            raise


def collect_host_preflight() -> HostPreflightSnapshot:
    """Run Docker, ownership, immutable-contract, and AppArmor gates."""
    try:
        import d455_production_container as production
    except ImportError as exc:
        raise ValidationError(
            "production lifecycle verification module is unavailable"
        ) from exc
    lifecycle = production.ProductionLifecycle(
        production.config_from_environment()
    )
    production.assert_no_active_validation_container(lifecycle.runner)
    production.assert_unique_production_container(
        lifecycle.runner,
        lifecycle.config.container_name,
    )
    production.assert_main_container_has_no_d455(lifecycle.runner)
    inspect = lifecycle.require_recorded_owned()
    if (
        inspect.get("State", {}).get("Running") is not True
        or inspect.get("State", {}).get("Health", {}).get("Status")
        != "healthy"
    ):
        raise ValidationError(
            "production D455 sensor container is not healthy"
        )
    resources = lifecycle.select_resources(
        lifecycle.config.usb_serial_number
    )
    production.validate_resource_set(resources)
    production.assert_no_foreign_running_d455_containers(
        lifecycle.runner,
        resources,
        expected_container_id=str(inspect["Id"]),
    )
    image_id = lifecycle.image_id()
    labels = inspect.get("Config", {}).get("Labels", {}) or {}
    config_hash = str(labels.get(production.CONFIG_LABEL_KEY, ""))
    lifecycle.verify_container_contract(
        inspect,
        resources=resources,
        image_id=image_id,
        config_sha256=config_hash,
    )
    profiles = production.KERNEL_PROFILES_PATH.read_text()
    enforcing = (
        f"{EXPECTED_APPARMOR} (enforce)" in profiles
        and inspect.get("AppArmorProfile") == EXPECTED_APPARMOR
    )
    main = production._parse_single_inspect(
        lifecycle.runner.run(
            ["docker", "inspect", production.MAIN_CONTAINER_NAME]
        )
    )
    if main is None or main.get("State", {}).get("Running") is not True:
        raise ValidationError("main container is not running")
    main_payload = {
        "Config": main.get("Config", {}),
        "HostConfig": main.get("HostConfig", {}),
        "Image": main.get("Image"),
    }
    main_hash = hashlib.sha256(
        production.canonical_json(main_payload).encode()
    ).hexdigest()
    return HostPreflightSnapshot(
        production_containers=(lifecycle.config.container_name,),
        validation_containers=(),
        foreign_d455_owners=(),
        apparmor_profile=str(inspect.get("AppArmorProfile", "")),
        apparmor_enforcing=enforcing,
        sensor_container_id=str(inspect["Id"]),
        sensor_image_id=image_id,
        immutable_config_sha256=config_hash,
        main_container_id=str(main["Id"]),
        main_immutable_config_sha256=main_hash,
    )


def resolve_evidence_paths(
    evidence_dir: Path,
    workspace_root: Path,
) -> tuple[Path, Path, str]:
    workspace = workspace_root.resolve()
    candidate = (
        evidence_dir
        if evidence_dir.is_absolute()
        else workspace / evidence_dir
    ).resolve()
    approved_root = (workspace / HOST_VALIDATION_ROOT).resolve()
    try:
        relative_under_root = candidate.relative_to(approved_root)
        relative_workspace = candidate.relative_to(workspace)
    except ValueError as exc:
        raise ValidationError(
            "evidence directory must remain under validation_evidence"
        ) from exc
    if not relative_under_root.parts:
        raise ValidationError("evidence directory must name a campaign")
    if candidate.exists():
        raise ValidationError("evidence directory already exists")
    container_root = Path("/ros_ws") / relative_workspace
    worker_dir = container_root / "worker-evidence"
    return candidate, worker_dir, relative_workspace.as_posix()


def parse_worker_result(output: str) -> Mapping[str, object]:
    rows = [
        line[len(WORKER_RESULT_PREFIX):]
        for line in output.splitlines()
        if line.startswith(WORKER_RESULT_PREFIX)
    ]
    if len(rows) != 1:
        raise ValidationError("worker result marker is missing or duplicated")
    try:
        payload = json.loads(rows[0])
    except json.JSONDecodeError as exc:
        raise ValidationError("worker result is not valid JSON") from exc
    if not isinstance(payload, dict):
        raise ValidationError("worker result must be an object")
    return payload


def validate_handoff(
    payload: Mapping[str, object],
    *,
    script_path: Path,
    require_approval: bool,
) -> tuple[
    Mapping[str, object],
    HostPreflightSnapshot,
    Optional[RosPreflightSnapshot],
]:
    if payload.get("schema") != HANDOFF_SCHEMA:
        raise ValidationError("handoff schema mismatch")
    if payload.get("worker_container") != WORKER_CONTAINER:
        raise ValidationError("handoff worker-container mismatch")
    worker_source = str(payload.get("worker_source_path", ""))
    worker_hash = str(payload.get("worker_sha256", ""))
    source_path = Path(worker_source)
    expected_name = f"{WORKER_SOURCE_PREFIX}{worker_hash}.py"
    try:
        source_path.resolve().relative_to(WORKER_TEMP_ROOT.resolve())
        source_is_temporary = True
    except ValueError:
        source_is_temporary = False
    if (
        not source_is_temporary
        or script_path.resolve() != source_path.resolve()
        or re.fullmatch(r"[0-9a-f]{64}", worker_hash) is None
    ):
        raise ValidationError("handoff worker-source mismatch")
    if file_sha256(script_path) != worker_hash:
        raise ValidationError("worker code hash mismatch")
    if source_path.name != expected_name:
        raise ValidationError("handoff worker-source mismatch")
    plan = payload.get("plan")
    if not isinstance(plan, dict):
        raise ValidationError("handoff plan is missing")
    validate_plan_payload(plan)
    if payload.get("plan_sha256") != plan["plan_sha256"]:
        raise ValidationError("handoff plan binding mismatch")
    host_payload = payload.get("host_snapshot")
    if not isinstance(host_payload, dict):
        raise ValidationError("handoff host snapshot is missing")
    if (
        payload.get("host_snapshot_sha256")
        != payload_sha256(host_payload)
    ):
        raise ValidationError("host snapshot hash mismatch")
    host_snapshot = host_snapshot_from_payload(host_payload)
    evidence_relative = Path(str(payload.get("evidence_relative", "")))
    if (
        evidence_relative.is_absolute()
        or ".." in evidence_relative.parts
        or not evidence_relative.parts
        or evidence_relative.parts[:3]
        != ("src", "realsense_imu", "validation_evidence")
    ):
        raise ValidationError("handoff evidence path is unsafe")
    worker_evidence = str(payload.get("worker_evidence_path", ""))
    worker_evidence_path = Path(worker_evidence)
    try:
        worker_evidence_path.resolve().relative_to(
            WORKER_TEMP_ROOT.resolve()
        )
        evidence_is_temporary = True
    except ValueError:
        evidence_is_temporary = False
    if (
        not evidence_is_temporary
        or re.fullmatch(
            rf"{re.escape(WORKER_EVIDENCE_PREFIX)}[0-9a-f]{{64}}",
            worker_evidence_path.name,
        )
        is None
    ):
        raise ValidationError("handoff worker-evidence path is unsafe")
    heartbeat_token = str(payload.get("heartbeat_token", ""))
    if (
        re.fullmatch(r"[0-9a-f]{64}", heartbeat_token) is None
        or float(payload.get("heartbeat_max_age_s", 0.0))
        != HEARTBEAT_MAX_AGE_S
    ):
        raise ValidationError("handoff heartbeat contract is invalid")
    ros_snapshot = None
    ros_payload = payload.get("ros_snapshot")
    if ros_payload is not None:
        if not isinstance(ros_payload, dict):
            raise ValidationError("handoff ROS snapshot is invalid")
        if (
            payload.get("ros_snapshot_sha256")
            != payload_sha256(ros_payload)
        ):
            raise ValidationError("ROS snapshot hash mismatch")
        ros_snapshot = ros_snapshot_from_payload(ros_payload)
    if require_approval:
        if ros_snapshot is None:
            raise ValidationError("approved ROS snapshot is missing")
        approval = payload.get("approval")
        if not isinstance(approval, dict):
            raise ValidationError("motion approval is missing")
        if approval.get("text") != APPROVAL_TEXT:
            raise ValidationError("motion approval text is invalid")
        expected_binding = payload_sha256(
            approval_binding_payload(payload)
        )
        if approval.get("binding_sha256") != expected_binding:
            raise ValidationError("motion approval binding mismatch")
    return plan, host_snapshot, ros_snapshot


def worker_result(payload: Mapping[str, object]) -> None:
    print(WORKER_RESULT_PREFIX + canonical_json(payload), flush=True)


def graph_convergence_config() -> tuple[float, float]:
    """Read bounded worker-side convergence tuning."""
    timeout = float(
        os.environ.get(
            "D455_GRAPH_CONVERGENCE_TIMEOUT_S",
            DEFAULT_GRAPH_CONVERGENCE_TIMEOUT_S,
        )
    )
    poll = float(
        os.environ.get(
            "D455_GRAPH_CONVERGENCE_POLL_S",
            DEFAULT_GRAPH_CONVERGENCE_POLL_S,
        )
    )
    return timeout, poll


def run_campaign(
    *,
    plan: Mapping[str, object],
    evidence_dir: Path,
    runtime_factory: Callable[[], Runtime],
    approval_binding: str = "",
) -> int:
    trials = validate_plan_payload(plan)
    writer = EvidenceWriter(evidence_dir, plan, dry_run=False)
    approval = {
        "text": APPROVAL_TEXT,
        "binding_sha256": approval_binding,
    }
    writer.write_event("operator_approved", approval)
    runtime = None
    campaign = None
    handled_signals = [signal.SIGINT, signal.SIGTERM]
    if hasattr(signal, "SIGHUP"):
        handled_signals.append(signal.SIGHUP)
    old_handlers = {
        signum: signal.getsignal(signum)
        for signum in handled_signals
    }
    for signum in old_handlers:
        signal.signal(
            signum,
            lambda *_unused: (_ for _ in ()).throw(KeyboardInterrupt()),
        )
    try:
        runtime = runtime_factory()
        campaign = Campaign(
            runtime,
            writer,
            rate_hz=float(plan["rate_hz"]),
        )
        completed = campaign.run(trials)
        writer.finish(
            "passed",
            campaign.snapshot,
            completed,
            cleanup={
                "final_zero_verified": campaign.final_zero_verified,
                "graph_convergence": asdict(
                    campaign.runtime.graph_convergence
                )
                if getattr(campaign.runtime, "graph_convergence", None)
                else None,
                "command_samples": [
                    asdict(sample)
                    for sample in campaign.cleanup_records
                ],
                "error": campaign.cleanup_error,
            },
        )
        return 0
    except BaseException as exc:
        snapshot = campaign.snapshot if campaign else None
        completed = campaign.completed if campaign else []
        cleanup = (
            {
                "final_zero_verified": campaign.final_zero_verified,
                "graph_convergence": asdict(
                    campaign.runtime.graph_convergence
                )
                if getattr(campaign.runtime, "graph_convergence", None)
                else None,
                "command_samples": [
                    asdict(sample)
                    for sample in campaign.cleanup_records
                ],
                "error": campaign.cleanup_error,
            }
            if campaign
            else {}
        )
        writer.finish(
            "failed",
            snapshot,
            completed,
            str(exc),
            cleanup=cleanup,
        )
        raise
    finally:
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)


def worker_preflight(
    handoff_path: Path,
    *,
    runtime_factory: Callable[
        [
            HostPreflightSnapshot,
            Optional[Mapping[str, object]],
            Optional[HeartbeatGuard],
        ],
        RosWorkerRuntime,
    ] = RosWorkerRuntime,
    script_path: Optional[Path] = None,
) -> int:
    active_script = script_path or Path(__file__)
    handoff = json.loads(handoff_path.read_text())
    _plan, host_snapshot, _approved = validate_handoff(
        handoff,
        script_path=active_script,
        require_approval=False,
    )
    runtime = runtime_factory(host_snapshot, None, None)
    try:
        try:
            if hasattr(runtime, "converge_graph"):
                timeout_s, poll_s = graph_convergence_config()
                ros_snapshot = runtime.converge_graph(
                    host_snapshot,
                    timeout_s=timeout_s,
                    poll_s=poll_s,
                )
            else:
                # Compatibility for injected hardware-free adapters.
                ros_snapshot = runtime.ros_preflight_snapshot()
                validate_preflight(
                    compose_preflight(host_snapshot, ros_snapshot)
                )
        except ValidationError as exc:
            worker_result(
                {
                    "result": "preflight_failed",
                    "worker_sha256": file_sha256(active_script),
                    "error": str(exc),
                    "graph_convergence": asdict(
                        getattr(runtime, "graph_convergence", None)
                    )
                    if getattr(runtime, "graph_convergence", None)
                    else None,
                }
            )
            raise
        worker_result(
            {
                "result": "preflight_passed",
                "worker_sha256": file_sha256(active_script),
                "ros_snapshot": ros_snapshot_payload(ros_snapshot),
                "graph_convergence": asdict(
                    getattr(runtime, "graph_convergence", None)
                )
                if getattr(runtime, "graph_convergence", None)
                else None,
            }
        )
        return 0
    finally:
        runtime.close()


def worker_execute(
    handoff_path: Path,
    *,
    enable_motion: bool,
    runtime_factory: Callable[
        [
            HostPreflightSnapshot,
            Optional[Mapping[str, object]],
            Optional[HeartbeatGuard],
        ],
        Runtime,
    ] = RosWorkerRuntime,
    heartbeat_stream=None,
    script_path: Optional[Path] = None,
) -> int:
    if not enable_motion:
        raise ValidationError("--enable-motion is required in worker mode")
    handoff = json.loads(handoff_path.read_text())
    active_script = script_path or Path(__file__)
    plan, host_snapshot, ros_snapshot = validate_handoff(
        handoff,
        script_path=active_script,
        require_approval=True,
    )
    assert ros_snapshot is not None
    evidence_dir = Path(str(handoff["worker_evidence_path"]))
    heartbeat_guard = HeartbeatGuard(
        str(handoff["heartbeat_token"]),
        float(handoff["heartbeat_max_age_s"]),
    )
    heartbeat_guard.start(
        sys.stdin if heartbeat_stream is None else heartbeat_stream
    )
    binding = str(handoff["approval"]["binding_sha256"])
    try:
        result = run_campaign(
            plan=plan,
            evidence_dir=evidence_dir,
            runtime_factory=lambda: runtime_factory(
                host_snapshot,
                approval_graph_identity(ros_snapshot),
                heartbeat_guard,
            ),
            approval_binding=binding,
        )
    except BaseException as exc:
        atomic_json(
            evidence_dir / WORKER_COMPLETION_FILE,
            {
                "result": "failed",
                "error": str(exc),
                "plan_sha256": plan["plan_sha256"],
                "approval_binding_sha256": binding,
            },
        )
        raise
    atomic_json(
        evidence_dir / WORKER_COMPLETION_FILE,
        {
            "result": "passed",
            "error": "",
            "plan_sha256": plan["plan_sha256"],
            "approval_binding_sha256": binding,
        },
    )
    worker_result(
        {
            "result": "campaign_passed",
            "plan_sha256": plan["plan_sha256"],
            "approval_binding_sha256": binding,
        }
    )
    return result


class HostOrchestrator:
    def __init__(
        self,
        *,
        runner: Optional[CommandRunner] = None,
        host_gate: Callable[[], HostPreflightSnapshot] = (
            collect_host_preflight
        ),
        workspace_root: Optional[Path] = None,
        script_path: Optional[Path] = None,
        token_factory: Callable[[], str] = (
            lambda: os.urandom(32).hex()
        ),
        sleep_fn: Callable[[float], None] = time.sleep,
        completion_timeout_s: float = WORKER_ABORT_GRACE_S,
    ) -> None:
        self.runner = runner or SubprocessRunner()
        self.host_gate = host_gate
        self.workspace_root = (
            workspace_root or Path(__file__).resolve().parents[3]
        )
        self.script_path = script_path or Path(__file__).resolve()
        self.token_factory = token_factory
        self.sleep_fn = sleep_fn
        self.completion_timeout_s = completion_timeout_s

    def _worker_command(
        self,
        mode: str,
        worker_source: Path,
        container_handoff: Path,
        *,
        enable_motion: bool = False,
    ) -> list[str]:
        worker_command = (
            f"exec python3 {shlex.quote(str(worker_source))} "
            f"{mode} --handoff {shlex.quote(str(container_handoff))}"
        )
        if enable_motion:
            worker_command += " --enable-motion"
        command = [
            "source /opt/ros/humble/setup.bash",
            "source /ros_ws/install/setup.bash",
            "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4",
            "export SKIP_DEFAULT_XML=1",
            "unset FASTDDS_DEFAULT_PROFILES_FILE",
            "unset FASTRTPS_DEFAULT_PROFILES_FILE",
            "unset RMW_FASTRTPS_USE_QOS_FROM_XML",
            "export D455_ROTATION_WORKER=1",
            worker_command,
        ]
        result = [
            "docker",
            "exec",
        ]
        if enable_motion:
            result.append("-i")
        result.extend(
            [
                WORKER_CONTAINER,
                "bash",
                "-lc",
                " && ".join(command),
            ]
        )
        return result

    def _checked(self, args: Sequence[str], description: str) -> CommandResult:
        result = self.runner.run(args)
        if result.returncode != 0:
            raise ValidationError(f"{description} failed")
        return result

    def _stage_read_only(
        self,
        host_path: Path,
        container_path: Path,
    ) -> None:
        self._checked(
            [
                "docker",
                "cp",
                str(host_path),
                f"{WORKER_CONTAINER}:{container_path}",
            ],
            "worker artifact transfer",
        )
        self._checked(
            [
                "docker",
                "exec",
                WORKER_CONTAINER,
                "chmod",
                "0444",
                str(container_path),
            ],
            "worker artifact read-only protection",
        )

    def _cleanup_worker_artifacts(
        self,
        paths: Sequence[Path],
        worker_evidence: Path,
        *,
        remove_worker_evidence: bool,
    ) -> list[Mapping[str, object]]:
        results = []
        for path in paths:
            result = self.runner.run(
                [
                    "docker",
                    "exec",
                    WORKER_CONTAINER,
                    "rm",
                    "-f",
                    str(path),
                ]
            )
            results.append(
                {
                    "path": str(path),
                    "returncode": result.returncode,
                    "stderr": result.stderr,
                }
            )
        if remove_worker_evidence:
            result = self.runner.run(
                [
                    "docker",
                    "exec",
                    WORKER_CONTAINER,
                    "rm",
                    "-rf",
                    str(worker_evidence),
                ]
            )
            results.append(
                {
                    "path": str(worker_evidence),
                    "returncode": result.returncode,
                    "stderr": result.stderr,
                }
            )
        else:
            results.append(
                {
                    "path": str(worker_evidence),
                    "returncode": None,
                    "stderr": "",
                    "retained_for_recovery": True,
                }
            )
        return results

    def _wait_worker_completion(
        self,
        worker_evidence: Path,
        *,
        plan_sha256: str,
        binding_sha256: str,
    ) -> Optional[Mapping[str, object]]:
        completion = worker_evidence / WORKER_COMPLETION_FILE
        deadline = time.monotonic() + self.completion_timeout_s
        while True:
            result = self.runner.run(
                [
                    "docker",
                    "exec",
                    WORKER_CONTAINER,
                    "cat",
                    str(completion),
                ]
            )
            if result.returncode == 0:
                try:
                    payload = json.loads(result.stdout)
                except json.JSONDecodeError:
                    payload = None
                if (
                    isinstance(payload, dict)
                    and payload.get("result") in {"passed", "failed"}
                    and payload.get("plan_sha256") == plan_sha256
                    and payload.get("approval_binding_sha256")
                    == binding_sha256
                ):
                    return payload
            if time.monotonic() >= deadline:
                return None
            self.sleep_fn(HEARTBEAT_INTERVAL_S)

    @staticmethod
    def _validate_copied_worker_evidence(
        directory: Path,
        *,
        plan_sha256: str,
        binding_sha256: str,
    ) -> bool:
        try:
            completion = json.loads(
                (directory / WORKER_COMPLETION_FILE).read_text()
            )
            summary = json.loads(
                (directory / "summary.json").read_text()
            )
        except (OSError, json.JSONDecodeError):
            return False
        return (
            completion.get("result") in {"passed", "failed"}
            and completion.get("plan_sha256") == plan_sha256
            and completion.get("approval_binding_sha256")
            == binding_sha256
            and summary.get("plan_sha256") == plan_sha256
            and summary.get("result") in {"passed", "failed"}
            and completion.get("result") == summary.get("result")
        )

    def run(
        self,
        *,
        plan: Mapping[str, object],
        evidence_dir: Path,
        input_fn: Callable[[str], str],
    ) -> int:
        host_evidence, _unused_worker_evidence, relative = (
            resolve_evidence_paths(
                evidence_dir,
                self.workspace_root,
            )
        )
        host_snapshot = self.host_gate()
        host_payload = host_snapshot_payload(host_snapshot)
        worker_bytes = self.script_path.read_bytes()
        worker_hash = hashlib.sha256(worker_bytes).hexdigest()
        worker_source = (
            WORKER_TEMP_ROOT
            / f"{WORKER_SOURCE_PREFIX}{worker_hash}.py"
        )
        heartbeat_token = self.token_factory()
        if re.fullmatch(r"[0-9a-f]{64}", heartbeat_token) is None:
            raise ValidationError("heartbeat token factory is invalid")
        preflight_handoff = (
            WORKER_TEMP_ROOT
            / f"{WORKER_HANDOFF_PREFIX}preflight-"
            f"{heartbeat_token}.json"
        )
        execution_id = payload_sha256(
            {
                "plan_sha256": plan["plan_sha256"],
                "worker_sha256": worker_hash,
                "host_snapshot_sha256": payload_sha256(host_payload),
                "heartbeat_token": heartbeat_token,
            }
        )
        worker_evidence = (
            WORKER_TEMP_ROOT
            / f"{WORKER_EVIDENCE_PREFIX}{execution_id}"
        )
        host_evidence.mkdir(parents=True, exist_ok=False)
        atomic_json(host_evidence / "host-preflight.json", host_payload)
        local_worker = host_evidence / worker_source.name
        local_worker.write_bytes(worker_bytes)
        local_worker.chmod(0o444)
        handoff = {
            "schema": HANDOFF_SCHEMA,
            "worker_container": WORKER_CONTAINER,
            "worker_source_path": str(worker_source),
            "worker_sha256": worker_hash,
            "plan": dict(plan),
            "plan_sha256": plan["plan_sha256"],
            "host_snapshot": host_payload,
            "host_snapshot_sha256": payload_sha256(host_payload),
            "evidence_relative": relative,
            "worker_evidence_path": str(worker_evidence),
            "heartbeat_token": heartbeat_token,
            "heartbeat_max_age_s": HEARTBEAT_MAX_AGE_S,
        }
        handoff_path = host_evidence / "host-worker-handoff.json"
        atomic_json(handoff_path, handoff)
        staged_paths = [worker_source, preflight_handoff]
        approved_handoff = None
        remove_worker_evidence = True
        try:
            self._stage_read_only(local_worker, worker_source)
            container_hash = self._checked(
                [
                    "docker",
                    "exec",
                    WORKER_CONTAINER,
                    "sha256sum",
                    str(worker_source),
                ],
                "container worker hash verification",
            )
            if container_hash.stdout.split()[:1] != [worker_hash]:
                raise ValidationError(
                    "container worker code hash mismatch"
                )
            self._stage_read_only(handoff_path, preflight_handoff)
            preflight = self.runner.run(
                self._worker_command(
                    "--worker-preflight",
                    worker_source,
                    preflight_handoff,
                )
            )
            atomic_json(
                host_evidence / "worker-preflight-command.json",
                {
                    "returncode": preflight.returncode,
                    "stdout": preflight.stdout,
                    "stderr": preflight.stderr,
                },
            )
            if preflight.returncode != 0:
                raise ValidationError(
                    "ROS worker preflight failed before publisher creation"
                )
            result = parse_worker_result(preflight.stdout)
            if (
                result.get("result") != "preflight_passed"
                or result.get("worker_sha256") != worker_hash
                or not isinstance(result.get("ros_snapshot"), dict)
            ):
                raise ValidationError(
                    "ROS worker preflight result is invalid"
                )
            ros_payload = result["ros_snapshot"]
            ros_snapshot = ros_snapshot_from_payload(ros_payload)
            validate_preflight(
                compose_preflight(host_snapshot, ros_snapshot)
            )
            handoff["ros_snapshot"] = ros_payload
            handoff["ros_snapshot_sha256"] = payload_sha256(ros_payload)
            binding = payload_sha256(approval_binding_payload(handoff))
            atomic_json(handoff_path, handoff)
            print(
                json.dumps(
                    {
                        "plan_sha256": plan["plan_sha256"],
                        "worker_sha256": worker_hash,
                        "host_snapshot_sha256": handoff[
                            "host_snapshot_sha256"
                        ],
                        "ros_snapshot_sha256": handoff[
                            "ros_snapshot_sha256"
                        ],
                        "approval_binding_sha256": binding,
                    },
                    indent=2,
                )
            )
            require_operator_approval(True, input_fn, binding)
            approved_host = self.host_gate()
            atomic_json(
                host_evidence / "host-pre-execution.json",
                host_snapshot_payload(approved_host),
            )
            if approved_host != host_snapshot:
                raise ValidationError(
                    "host production identity changed after approval"
                )
            handoff["approval"] = {
                "text": APPROVAL_TEXT,
                "binding_sha256": binding,
            }
            atomic_json(handoff_path, handoff)
            approved_handoff = (
                WORKER_TEMP_ROOT
                / f"{WORKER_HANDOFF_PREFIX}approved-{binding}.json"
            )
            staged_paths.append(approved_handoff)
            self._stage_read_only(handoff_path, approved_handoff)
            worker = None
            worker_error = None
            remove_worker_evidence = False
            try:
                worker = self.runner.run_supervised(
                    self._worker_command(
                        "--worker-execute",
                        worker_source,
                        approved_handoff,
                        enable_motion=True,
                    ),
                    heartbeat_token,
                )
            except BaseException as exc:
                worker_error = exc
            atomic_json(
                host_evidence / "worker-execution-command.json",
                {
                    "returncode": (
                        worker.returncode if worker is not None else None
                    ),
                    "stdout": worker.stdout if worker is not None else "",
                    "stderr": worker.stderr if worker is not None else "",
                    "exception": (
                        f"{type(worker_error).__name__}: {worker_error}"
                        if worker_error is not None
                        else ""
                    ),
                },
            )
            completion = self._wait_worker_completion(
                worker_evidence,
                plan_sha256=str(plan["plan_sha256"]),
                binding_sha256=binding,
            )
            atomic_json(
                host_evidence / "worker-completion-wait.json",
                {
                    "completion": completion,
                    "container_evidence_path": str(worker_evidence),
                },
            )
            copy_result = None
            copied_valid = False
            copied_evidence = host_evidence / "worker-evidence"
            if completion is not None:
                copy_result = self.runner.run(
                    [
                        "docker",
                        "cp",
                        f"{WORKER_CONTAINER}:{worker_evidence}",
                        str(copied_evidence),
                    ]
                )
                if copy_result.returncode == 0:
                    copied_valid = self._validate_copied_worker_evidence(
                        copied_evidence,
                        plan_sha256=str(plan["plan_sha256"]),
                        binding_sha256=binding,
                    )
            remove_worker_evidence = copied_valid
            atomic_json(
                host_evidence / "worker-evidence-copy.json",
                {
                    "returncode": (
                        copy_result.returncode
                        if copy_result is not None
                        else None
                    ),
                    "stdout": (
                        copy_result.stdout
                        if copy_result is not None
                        else ""
                    ),
                    "stderr": (
                        copy_result.stderr
                        if copy_result is not None
                        else ""
                    ),
                    "validated": copied_valid,
                    "container_evidence_retained": not copied_valid,
                    "recovery_path": (
                        str(worker_evidence)
                        if not copied_valid
                        else ""
                    ),
                },
            )
            postflight = self.host_gate()
            atomic_json(
                host_evidence / "host-postflight.json",
                host_snapshot_payload(postflight),
            )
            if postflight != host_snapshot:
                raise ValidationError(
                    "host production identity changed during "
                    "worker execution"
                )
            if worker_error is not None:
                raise worker_error
            assert worker is not None
            if worker.returncode != 0:
                raise ValidationError(
                    "ROS worker failed; worker evidence preserves "
                    "cleanup state"
                )
            if completion is None:
                raise ValidationError(
                    "worker cleanup completion was not confirmed; "
                    f"evidence retained at {worker_evidence}"
                )
            if not copied_valid:
                raise ValidationError(
                    "worker evidence copy was not validated; "
                    f"container evidence retained at {worker_evidence}"
                )
            worker_payload = parse_worker_result(worker.stdout)
            if (
                worker_payload.get("result") != "campaign_passed"
                or worker_payload.get("plan_sha256")
                != plan["plan_sha256"]
                or worker_payload.get("approval_binding_sha256")
                != binding
            ):
                raise ValidationError(
                    "ROS worker completion binding is invalid"
                )
            atomic_json(
                host_evidence / "host-final.json",
                {
                    "result": "passed",
                    "plan_sha256": plan["plan_sha256"],
                    "worker_sha256": worker_hash,
                    "approval_binding_sha256": binding,
                },
            )
            return 0
        finally:
            cleanup = self._cleanup_worker_artifacts(
                staged_paths,
                worker_evidence,
                remove_worker_evidence=remove_worker_evidence,
            )
            atomic_json(
                host_evidence / "worker-artifact-cleanup.json",
                cleanup,
            )


def direct_campaign_main(
    *,
    plan: Mapping[str, object],
    evidence_dir: Path,
    input_fn: Callable[[str], str],
    runtime_factory: Callable[[], Runtime],
) -> int:
    require_operator_approval(True, input_fn)
    return run_campaign(
        plan=plan,
        evidence_dir=evidence_dir,
        runtime_factory=runtime_factory,
    )


def main(
    argv: Sequence[str] | None = None,
    *,
    input_fn: Callable[[str], str] = input,
    runtime_factory: Optional[Callable[[], Runtime]] = None,
    orchestrator_factory: Callable[[], HostOrchestrator] = HostOrchestrator,
) -> int:
    argument_rows = list(argv) if argv is not None else sys.argv[1:]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--worker-preflight",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--worker-execute",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--handoff", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--evidence-dir", type=Path)
    parser.add_argument(
        "--speeds",
        type=float,
        nargs="+",
        default=DEFAULT_SPEEDS,
    )
    parser.add_argument(
        "--angular-velocity",
        type=float,
        help=(
            "single positive angular-speed magnitude in rad/s; used for "
            "both CW (-V) and CCW (+V) trials"
        ),
    )
    parser.add_argument(
        "--linear-velocity",
        type=float,
        help=(
            "single positive linear-speed magnitude in m/s; used for "
            "forward (+V) and backward (-V) trials with angular.z zero"
        ),
    )
    parser.add_argument(
        "--durations",
        type=float,
        nargs="+",
        default=DEFAULT_DURATIONS,
    )
    parser.add_argument("--repetitions", type=int, default=DEFAULT_REPETITIONS)
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    args = parser.parse_args(argument_rows)
    if args.worker_preflight and args.worker_execute:
        raise ValidationError("worker modes are mutually exclusive")
    if args.worker_preflight or args.worker_execute:
        forbidden = {
            "--evidence-dir",
            "--speeds",
            "--angular-velocity",
            "--linear-velocity",
            "--durations",
            "--repetitions",
            "--rate-hz",
        }
        if any(
            value in forbidden
            or value.startswith("--speeds=")
            or value.startswith("--angular-velocity=")
            or value.startswith("--linear-velocity=")
            for value in argument_rows
        ):
            raise ValidationError(
                "worker mode accepts plan values only from the handoff"
            )
        if args.worker_preflight and args.enable_motion:
            raise ValidationError(
                "worker preflight cannot enable motion"
            )
        if os.environ.get("D455_ROTATION_WORKER") != "1":
            raise ValidationError(
                "worker mode requires the container execution marker"
            )
        if args.handoff is None:
            raise ValidationError("--handoff is required in worker mode")
        if args.worker_preflight:
            return worker_preflight(args.handoff)
        return worker_execute(
            args.handoff,
            enable_motion=args.enable_motion,
        )
    if args.angular_velocity is not None and any(
        value == "--speeds" or value.startswith("--speeds=")
        for value in argument_rows
    ):
        raise ValidationError(
            "--angular-velocity cannot be combined with --speeds"
        )
    if args.linear_velocity is not None and (
        args.angular_velocity is not None
        or any(
            value == "--speeds" or value.startswith("--speeds=")
            for value in argument_rows
        )
    ):
        raise ValidationError(
            "--linear-velocity cannot be combined with rotation speed options"
        )
    speeds = (
        (args.angular_velocity,)
        if args.angular_velocity is not None
        else args.speeds
    )
    trials = (
        build_linear_matrix(
            (args.linear_velocity,),
            args.durations,
            args.repetitions,
        )
        if args.linear_velocity is not None
        else build_matrix(speeds, args.durations, args.repetitions)
    )
    if (
        not math.isfinite(args.rate_hz)
        or args.rate_hz < 10.0
        or args.rate_hz > 50.0
    ):
        raise ValidationError("rate_hz must be finite and in [10,50]")
    plan = plan_payload(trials, args.rate_hz)
    print(json.dumps({"dry_run": not args.enable_motion, **plan}, indent=2))
    if not args.enable_motion:
        if args.evidence_dir:
            writer = EvidenceWriter(args.evidence_dir, plan, dry_run=True)
            writer.finish("dry_run", None, ())
        return 0
    if args.evidence_dir is None:
        raise ValidationError("--evidence-dir is required for motion")
    if runtime_factory is not None:
        return direct_campaign_main(
            plan=plan,
            evidence_dir=args.evidence_dir,
            input_fn=input_fn,
            runtime_factory=runtime_factory,
        )
    return orchestrator_factory().run(
        plan=plan,
        evidence_dir=args.evidence_dir,
        input_fn=input_fn,
    )


if __name__ == "__main__":
    sys.exit(main())
