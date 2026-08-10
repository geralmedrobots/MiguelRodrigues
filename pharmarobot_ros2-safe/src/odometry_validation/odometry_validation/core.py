# Copyright 2026 Medrobots Engineering
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Hardware-independent odometry validation primitives."""

from dataclasses import asdict
from dataclasses import dataclass
import csv
import datetime
import json
import math
import os
from pathlib import Path
from queue import Empty
from queue import Queue
import select
import statistics
from threading import Event
from threading import Lock
from threading import Thread
import time
import traceback
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple


DEFAULT_ROTATION_VELOCITIES_RAD_S = (0.30, 0.40, 0.50)
DEFAULT_ROTATION_DURATIONS_S = (1.0, 2.0, 4.0, 5.0, 7.0, 8.0, 10.0)
DEFAULT_TRANSLATION_VELOCITIES_M_S = (
    0.10, 0.20, 0.30, 0.50, 0.75, 0.80, 1.00)
DEFAULT_TRANSLATION_DURATIONS_S = (
    2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0)
PRIMARY_IMU_SOURCE_TOPIC = "/imu/data"
ROTATION_DIRECTIONS = ("cw", "ccw")
TRANSLATION_DIRECTIONS = ("forward", "backward")
SUMMARY_COMPARISONS = (
    ("encoder", "odometry"),
    ("encoder", "imu"),
    ("encoder", "physical"),
    ("odometry", "physical"),
    ("imu", "physical"),
)


class ValidationError(RuntimeError):
    """Fail-closed validation error."""


class EmergencyStopError(RuntimeError):
    """Emergency stop was attempted but could not be verified."""

    def __init__(self, message: str, record: Optional[Dict[str, object]] = None):
        self.record = dict(record or {})
        super().__init__(message)


class ControlledStopError(EmergencyStopError):
    """Normal post-command stop did not verify before its deadline."""


class EmergencyStopCleanupError(EmergencyStopError):
    """Preserve both a primary failure and a zero-cleanup failure."""

    def __init__(self, primary_error: BaseException, cleanup_error: BaseException):
        self.primary_error = primary_error
        self.cleanup_error = cleanup_error
        primary_text = str(primary_error) or type(primary_error).__name__
        cleanup_text = str(cleanup_error) or type(cleanup_error).__name__
        super().__init__(
            f"{primary_text}; zero-command cleanup failed: {cleanup_text}")


def diagnostic_level_to_int(value: object) -> int:
    """Normalize ROS diagnostic levels from integer or byte representations."""
    if isinstance(value, int):
        return int(value)
    if isinstance(value, (bytes, bytearray, memoryview)):
        raw_value = bytes(value)
        if len(raw_value) != 1:
            raise ValueError("diagnostic level bytes must contain one byte")
        return raw_value[0]
    try:
        return int(value)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f"unsupported diagnostic level representation: {value!r}") from error


@dataclass(frozen=True)
class GeometryConfig:
    """Wheel geometry used only for validator-side comparisons."""

    wheel_radius_m: float
    track_width_m: float
    encoder_ticks_per_revolution: int

    def __post_init__(self):
        if self.wheel_radius_m <= 0.0 or not math.isfinite(
                self.wheel_radius_m):
            raise ValueError("wheel_radius_m must be finite and positive")
        if self.track_width_m <= 0.0 or not math.isfinite(
                self.track_width_m):
            raise ValueError("track_width_m must be finite and positive")
        if self.encoder_ticks_per_revolution <= 0:
            raise ValueError("encoder_ticks_per_revolution must be positive")


@dataclass(frozen=True)
class TrialSpec:
    """One commanded validation trial."""

    trial_id: str
    movement_type: str
    velocity: float
    duration_s: float
    direction: str

    @property
    def linear_x(self) -> float:
        if self.movement_type != "translation":
            return 0.0
        return self.velocity if self.direction == "forward" else -self.velocity

    @property
    def angular_z(self) -> float:
        if self.movement_type != "rotation":
            return 0.0
        return self.velocity if self.direction == "ccw" else -self.velocity

    @property
    def commanded_angle_rad(self) -> float:
        return self.angular_z * self.duration_s

    @property
    def commanded_distance_m(self) -> float:
        return self.linear_x * self.duration_s


@dataclass(frozen=True)
class WheelTickSample:
    timestamp_s: float
    left_ticks: int
    right_ticks: int
    phase: str = "unknown"


@dataclass(frozen=True)
class ImuSample:
    timestamp_s: float
    angular_velocity_z_rad_s: float
    phase: str = "unknown"
    source_topic: str = PRIMARY_IMU_SOURCE_TOPIC


@dataclass(frozen=True)
class ProcessedImuSample:
    timestamp_s: float
    raw_angular_velocity_z_rad_s: float
    bias_rad_s: float
    corrected_angular_velocity_z_rad_s: float
    integrated_angle_rad: float


@dataclass(frozen=True)
class OdomSample:
    timestamp_s: float
    x_m: float
    y_m: float
    yaw_rad: float
    linear_x_m_s: float = 0.0
    angular_z_rad_s: float = 0.0
    phase: str = "unknown"


@dataclass(frozen=True)
class CommandSample:
    timestamp_s: float
    topic: str
    linear_x_m_s: float
    angular_z_rad_s: float
    phase: str = "unknown"


@dataclass(frozen=True)
class DiagnosticSample:
    timestamp_s: float
    level: int
    name: str
    message: str


@dataclass(frozen=True)
class StationaritySample:
    """One consecutive encoder delta used by stop verification."""

    previous_timestamp_s: float
    timestamp_s: float
    previous_left_ticks: int
    previous_right_ticks: int
    left_ticks: int
    right_ticks: int
    left_delta_ticks: int
    right_delta_ticks: int


@dataclass(frozen=True)
class StationarityAssessment:
    """Auditable result of the post-stop stationarity check."""

    stationary: bool
    reason: str
    required_delta_samples: int
    observed_delta_samples: int
    tick_delta_tolerance: int
    linear_velocity_tolerance_m_s: float
    angular_velocity_tolerance_rad_s: float
    first_zero_timestamp_s: Optional[float]
    wheel_tick_semantics: str
    safe_zero: bool
    tick_deltas_stationary: bool
    odom_twist_stationary: bool
    stationarity_samples: Tuple[StationaritySample, ...] = ()
    odom_samples: Tuple[OdomSample, ...] = ()
    assessment_timestamp_s: Optional[float] = None
    elapsed_since_first_zero_s: Optional[float] = None


@dataclass(frozen=True)
class TrialSamples:
    wheel_ticks: Tuple[WheelTickSample, ...] = ()
    imu: Tuple[ImuSample, ...] = ()
    processed_imu: Tuple[ProcessedImuSample, ...] = ()
    odom: Tuple[OdomSample, ...] = ()
    diagnostics: Tuple[DiagnosticSample, ...] = ()
    ignored_diagnostics: Tuple[DiagnosticSample, ...] = ()
    commands: Tuple[CommandSample, ...] = ()
    stationarity: Tuple[StationarityAssessment, ...] = ()


def _freeze_identity(value: object) -> object:
    """Convert nested dataclass values into deterministic hashable values."""
    if isinstance(value, dict):
        return tuple(sorted(
            (str(key), _freeze_identity(nested))
            for key, nested in value.items()))
    if isinstance(value, (tuple, list)):
        return tuple(_freeze_identity(nested) for nested in value)
    return value


def _sample_identity(field_name: str, value: object) -> Tuple[object, ...]:
    """Return an explicit identity key for one sample family."""
    if field_name == "wheel_ticks":
        return (value.timestamp_s, value.left_ticks, value.right_ticks, value.phase)
    if field_name == "imu":
        return (
            value.source_topic, value.timestamp_s,
            value.angular_velocity_z_rad_s, value.phase)
    if field_name == "processed_imu":
        return (
            value.timestamp_s, value.raw_angular_velocity_z_rad_s,
            value.bias_rad_s, value.corrected_angular_velocity_z_rad_s,
            value.integrated_angle_rad)
    if field_name == "odom":
        return (
            value.timestamp_s, value.x_m, value.y_m, value.yaw_rad,
            value.linear_x_m_s, value.angular_z_rad_s, value.phase)
    if field_name in ("diagnostics", "ignored_diagnostics"):
        return (value.timestamp_s, value.level, value.name, value.message)
    if field_name == "commands":
        return (
            value.topic, value.timestamp_s, value.linear_x_m_s,
            value.angular_z_rad_s, value.phase)
    if field_name == "stationarity":
        return (_freeze_identity(asdict(value)),)
    raise ValueError(f"unsupported trial sample field: {field_name}")


def _sample_timestamp(field_name: str, value: object) -> float:
    """Return the ordering timestamp for a sample family."""
    if field_name == "stationarity":
        return float(value.assessment_timestamp_s or 0.0)
    return float(value.timestamp_s)


def merge_trial_samples(*snapshots: TrialSamples) -> TrialSamples:
    """
    Merge immutable snapshots with indexed, stable sample identities.

    Keys preserve distinct same-timestamp samples, while a dictionary avoids
    dataclass equality scans.  Sorting is stable, so equal timestamps retain
    snapshot/insertion order.  The result is idempotent and uses O(n) extra
    index space for n input samples.
    """
    field_names = (
        "wheel_ticks", "imu", "processed_imu", "odom", "diagnostics",
        "ignored_diagnostics", "commands", "stationarity")
    merged = {}
    for field_name in field_names:
        values = []
        seen = set()
        for snapshot in snapshots:
            for value in getattr(snapshot, field_name):
                identity = _sample_identity(field_name, value)
                if identity in seen:
                    continue
                seen.add(identity)
                values.append(value)
        values.sort(key=lambda value: _sample_timestamp(field_name, value))
        merged[field_name] = tuple(values)
    return TrialSamples(**merged)


@dataclass(frozen=True)
class TrialMeasurements:
    encoder_distance_m: Optional[float]
    encoder_angle_rad: Optional[float]
    left_wheel_distance_m: Optional[float]
    right_wheel_distance_m: Optional[float]
    odometry_distance_m: Optional[float]
    odometry_angle_rad: Optional[float]
    imu_angle_rad: Optional[float]
    physical_measurement: Optional[float]
    commanded_distance_m: float
    commanded_angle_rad: float
    command_start_timestamp_s: Optional[float] = None
    command_end_timestamp_s: Optional[float] = None
    stationary_confirmation_timestamp_s: Optional[float] = None
    commanded_window_imu_angle_rad: Optional[float] = None
    settling_imu_angle_rad: Optional[float] = None
    commanded_window_imu_sample_count: int = 0
    settling_imu_sample_count: int = 0
    total_imu_sample_count: int = 0
    imu_source_topic: str = PRIMARY_IMU_SOURCE_TOPIC
    imu_start_boundary_gap_s: Optional[float] = None
    imu_command_end_boundary_gap_s: Optional[float] = None
    imu_stationary_boundary_gap_s: Optional[float] = None


@dataclass(frozen=True)
class TrialResult:
    spec: TrialSpec
    timestamp: str
    measurements: TrialMeasurements
    errors: Dict[str, Optional[float]]
    valid: Optional[bool]
    skipped: bool
    rejection_reason: Optional[str]
    operator_notes: str
    evidence_dir: Optional[str] = None
    initial_compass_heading_deg: Optional[float] = None
    final_compass_heading_deg: Optional[float] = None


def utc_timestamp() -> str:
    """Return an evidence-safe UTC timestamp."""
    return datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y%m%dT%H%M%SZ")


def _finite_positive_values(values: Iterable[float], name: str) -> Tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not result:
        raise ValueError(f"{name} cannot be empty")
    for value in result:
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} values must be finite and positive")
    return result


def generate_rotation_trials(
        velocities: Iterable[float] = DEFAULT_ROTATION_VELOCITIES_RAD_S,
        durations: Iterable[float] = DEFAULT_ROTATION_DURATIONS_S,
        include_cw: bool = True,
        include_ccw: bool = True) -> List[TrialSpec]:
    """Generate the rotation trial matrix."""
    velocities = _finite_positive_values(velocities, "rotation velocity")
    durations = _finite_positive_values(durations, "rotation duration")
    directions = []
    if include_cw:
        directions.append("cw")
    if include_ccw:
        directions.append("ccw")
    if not directions:
        raise ValueError("at least one rotation direction must be enabled")

    trials = []
    index = 1
    for velocity in velocities:
        for duration in durations:
            for direction in directions:
                trials.append(TrialSpec(
                    trial_id=f"rot-{index:03d}-{direction}",
                    movement_type="rotation",
                    velocity=velocity,
                    duration_s=duration,
                    direction=direction))
                index += 1
    return trials


def generate_translation_trials(
        velocities: Iterable[float] = DEFAULT_TRANSLATION_VELOCITIES_M_S,
        durations: Iterable[float] = DEFAULT_TRANSLATION_DURATIONS_S,
        include_forward: bool = True,
        include_backward: bool = True) -> List[TrialSpec]:
    """Generate the translation trial matrix."""
    velocities = _finite_positive_values(velocities, "translation velocity")
    durations = _finite_positive_values(durations, "translation duration")
    directions = []
    if include_forward:
        directions.append("forward")
    if include_backward:
        directions.append("backward")
    if not directions:
        raise ValueError("at least one translation direction must be enabled")

    trials = []
    index = 1
    for velocity in velocities:
        for duration in durations:
            for direction in directions:
                trials.append(TrialSpec(
                    trial_id=f"trans-{index:03d}-{direction}",
                    movement_type="translation",
                    velocity=velocity,
                    duration_s=duration,
                    direction=direction))
                index += 1
    return trials


def wheel_tick_measurements(
        samples: Sequence[WheelTickSample],
        geometry: GeometryConfig,
        semantics: str = "delta") -> Tuple[float, float, float, float]:
    """Return displacement using the explicitly selected tick contract."""
    meters_per_tick = (
        2.0 * math.pi * geometry.wheel_radius_m /
        geometry.encoder_ticks_per_revolution)
    if semantics == "delta":
        left_ticks = sum(sample.left_ticks for sample in samples)
        right_ticks = sum(sample.right_ticks for sample in samples)
    elif semantics == "cumulative":
        if len(samples) < 2:
            left_ticks = 0
            right_ticks = 0
        else:
            left_ticks = samples[-1].left_ticks - samples[0].left_ticks
            right_ticks = samples[-1].right_ticks - samples[0].right_ticks
    else:
        raise ValueError("wheel tick semantics must be 'delta' or 'cumulative'")
    left_distance = left_ticks * meters_per_tick
    right_distance = right_ticks * meters_per_tick
    encoder_distance = (left_distance + right_distance) / 2.0
    encoder_angle = (right_distance - left_distance) / geometry.track_width_m
    return encoder_distance, encoder_angle, left_distance, right_distance


def wheel_tick_totals(
        samples: Sequence[WheelTickSample],
        semantics: str = "delta") -> Tuple[int, int]:
    """Return the exact left/right tick totals used for measurements."""
    if semantics == "delta":
        return (
            sum(sample.left_ticks for sample in samples),
            sum(sample.right_ticks for sample in samples))
    if semantics == "cumulative":
        if len(samples) < 2:
            return 0, 0
        return (
            samples[-1].left_ticks - samples[0].left_ticks,
            samples[-1].right_ticks - samples[0].right_ticks)
    raise ValueError("wheel tick semantics must be 'delta' or 'cumulative'")


def wrap_radians(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def odometry_measurements(
        samples: Sequence[OdomSample]) -> Tuple[Optional[float], Optional[float]]:
    """Return odometry displacement and yaw delta from first to last sample."""
    if len(samples) < 2:
        return None, None
    first = samples[0]
    last = samples[-1]
    distance = math.hypot(last.x_m - first.x_m, last.y_m - first.y_m)
    angle = odometry_yaw_change(samples)
    return distance, angle


def odometry_signed_displacement(
        samples: Sequence[OdomSample]) -> Optional[float]:
    """Project endpoint displacement onto the initial odometry heading."""
    if len(samples) < 2:
        return None
    first = samples[0]
    last = samples[-1]
    return (
        (last.x_m - first.x_m) * math.cos(first.yaw_rad) +
        (last.y_m - first.y_m) * math.sin(first.yaw_rad))


def odometry_yaw_change(samples: Sequence[OdomSample]) -> Optional[float]:
    """Unwrap every yaw increment across the complete odometry interval."""
    if len(samples) < 2:
        return None
    return sum(
        wrap_radians(current.yaw_rad - previous.yaw_rad)
        for previous, current in zip(samples, samples[1:]))


def integrate_imu_angle(
        samples: Sequence[ImuSample],
        bias_rad_s: Optional[float] = None) -> Tuple[float, Tuple[ProcessedImuSample, ...]]:
    """Integrate timestamp-aware bias-corrected yaw rate samples."""
    if not samples:
        return 0.0, ()
    ordered = sorted(samples, key=lambda sample: sample.timestamp_s)
    if bias_rad_s is None:
        bias_rad_s = statistics.mean(
            sample.angular_velocity_z_rad_s for sample in ordered)
    if not math.isfinite(bias_rad_s):
        raise ValueError("bias_rad_s must be finite")

    processed = [
        ProcessedImuSample(
            timestamp_s=ordered[0].timestamp_s,
            raw_angular_velocity_z_rad_s=ordered[0].angular_velocity_z_rad_s,
            bias_rad_s=bias_rad_s,
            corrected_angular_velocity_z_rad_s=(
                ordered[0].angular_velocity_z_rad_s - bias_rad_s),
            integrated_angle_rad=0.0)
    ]
    angle = 0.0
    for previous, current in zip(ordered, ordered[1:]):
        dt = current.timestamp_s - previous.timestamp_s
        if dt < 0.0 or not math.isfinite(dt):
            raise ValueError("IMU timestamps must be monotonic and finite")
        previous_rate = previous.angular_velocity_z_rad_s - bias_rad_s
        current_rate = current.angular_velocity_z_rad_s - bias_rad_s
        angle += 0.5 * (previous_rate + current_rate) * dt
        processed.append(ProcessedImuSample(
            timestamp_s=current.timestamp_s,
            raw_angular_velocity_z_rad_s=current.angular_velocity_z_rad_s,
            bias_rad_s=bias_rad_s,
            corrected_angular_velocity_z_rad_s=current_rate,
            integrated_angle_rad=angle))
    return angle, tuple(processed)


def imu_samples_in_motion_window(
        samples: Sequence[ImuSample],
        motion_start_timestamp_s: Optional[float],
        motion_end_timestamp_s: Optional[float],
        source_topic: str = PRIMARY_IMU_SOURCE_TOPIC,
        max_boundary_gap_s: Optional[float] = None) -> Tuple[ImuSample, ...]:
    """Return only IMU samples inside the recorded commanded-motion window."""
    if (
            motion_start_timestamp_s is None or
            motion_end_timestamp_s is None or
            not math.isfinite(motion_start_timestamp_s) or
            not math.isfinite(motion_end_timestamp_s) or
            motion_end_timestamp_s < motion_start_timestamp_s):
        raise ValidationError("commanded motion timestamps are unavailable")
    selected = tuple(
        sample for sample in samples
        if (sample.source_topic == source_topic and
            motion_start_timestamp_s <= sample.timestamp_s <=
            motion_end_timestamp_s))
    if len(selected) < 2:
        raise ValidationError(
            "insufficient primary IMU samples inside commanded motion interval")
    first_timestamp = min(sample.timestamp_s for sample in selected)
    last_timestamp = max(sample.timestamp_s for sample in selected)
    if last_timestamp <= first_timestamp:
        raise ValidationError(
            "primary IMU motion-window timestamps do not advance")
    if max_boundary_gap_s is not None:
        if not math.isfinite(max_boundary_gap_s) or max_boundary_gap_s < 0.0:
            raise ValueError("max_boundary_gap_s must be finite and nonnegative")
        if (first_timestamp - motion_start_timestamp_s > max_boundary_gap_s or
                motion_end_timestamp_s - last_timestamp > max_boundary_gap_s):
            raise ValidationError(
                "primary IMU samples do not cover commanded motion interval")
    return selected


def imu_samples_with_interpolated_boundary(
        samples: Sequence[ImuSample], boundary_timestamp_s: float
        ) -> Tuple[ImuSample, ...]:
    """Insert a primary-IMU linear-interpolation sample at a split boundary."""
    ordered = tuple(sorted(samples, key=lambda sample: sample.timestamp_s))
    if any(sample.timestamp_s == boundary_timestamp_s for sample in ordered):
        return ordered
    previous = tuple(
        sample for sample in ordered if sample.timestamp_s < boundary_timestamp_s)
    following = tuple(
        sample for sample in ordered if sample.timestamp_s > boundary_timestamp_s)
    if not previous or not following:
        raise ValidationError("IMU samples do not bracket command-end boundary")
    before = previous[-1]
    after = following[0]
    interval_s = after.timestamp_s - before.timestamp_s
    if interval_s <= 0.0 or not math.isfinite(interval_s):
        raise ValidationError("IMU timestamps do not bracket command-end boundary")
    fraction = (boundary_timestamp_s - before.timestamp_s) / interval_s
    rate = before.angular_velocity_z_rad_s + fraction * (
        after.angular_velocity_z_rad_s - before.angular_velocity_z_rad_s)
    interpolated = ImuSample(
        timestamp_s=boundary_timestamp_s,
        angular_velocity_z_rad_s=rate,
        phase="interpolated_command_boundary",
        source_topic=PRIMARY_IMU_SOURCE_TOPIC)
    return tuple(sorted(ordered + (interpolated,), key=lambda sample: sample.timestamp_s))


def compass_rotation_radians(initial_heading_deg: float, final_heading_deg: float) -> float:
    """
    Convert compass heading change to ROS-positive yaw rotation.

    Compass headings increase clockwise.  ROS yaw is positive counter-clockwise,
    so the shortest compass delta is negated.
    """
    delta_clockwise = (
        (float(final_heading_deg) - float(initial_heading_deg) + 180.0) %
        360.0) - 180.0
    return math.radians(-delta_clockwise)


def expected_final_compass_heading_deg(
        initial_heading_deg: float,
        expected_angle_rad: float) -> float:
    """Apply ROS-positive yaw to a clockwise-positive compass heading."""
    return (float(initial_heading_deg) - math.degrees(expected_angle_rad)) % 360.0


def percentage_error(value: Optional[float], reference: Optional[float]) -> Optional[float]:
    """Return signed percent error, or None when a zero reference is undefined."""
    if value is None or reference is None or abs(reference) <= 1e-12:
        return None
    return 100.0 * (value - reference) / abs(reference)


def _comparison_entry(
        value: Optional[float], reference: Optional[float]) -> Dict[str, Optional[float]]:
    if value is None or reference is None:
        return {
            "value": value,
            "signed_error": None,
            "absolute_error": None,
            "percentage_error": None,
        }
    signed_error = value - reference
    return {
        "value": value,
        "signed_error": signed_error,
        "absolute_error": abs(signed_error),
        "percentage_error": percentage_error(value, reference),
    }


def _interval_s(start_s: Optional[float], end_s: Optional[float]) -> Optional[float]:
    """Return a finite ordered interval or None when its boundaries are absent."""
    if start_s is None or end_s is None:
        return None
    return end_s - start_s


def _degrees_or_none(value: Optional[float]) -> Optional[float]:
    """Convert an optional radians value to degrees."""
    return None if value is None else math.degrees(value)


def _validate_report_values(value: object, path: str = "report") -> None:
    """Reject nonfinite values before a human-readable report is accepted."""
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValidationError(f"{path} contains a nonfinite value")
        return
    if isinstance(value, int):
        return
    if isinstance(value, dict):
        for key, nested in value.items():
            _validate_report_values(nested, f"{path}.{key}")
        return
    if isinstance(value, (tuple, list)):
        for index, nested in enumerate(value):
            _validate_report_values(nested, f"{path}[{index}]")
        return
    raise ValidationError(f"{path} contains unsupported value type {type(value).__name__}")


def build_trial_report(
        result: TrialResult,
        samples: TrialSamples,
        geometry: GeometryConfig,
        wheel_tick_semantics: str = "delta",
        imu_bias_rad_s: float = 0.0) -> Dict[str, object]:
    """Build one complete, JSON-safe report from a trial's full sample interval."""
    measurements = result.measurements
    spec = result.spec
    left_ticks, right_ticks = wheel_tick_totals(
        samples.wheel_ticks, wheel_tick_semantics)
    odom_start = samples.odom[0] if samples.odom else None
    odom_end = samples.odom[-1] if samples.odom else None
    imu_start = samples.processed_imu[0] if samples.processed_imu else None
    imu_end = samples.processed_imu[-1] if samples.processed_imu else None
    report = {
        "schema_version": 1,
        "trial": asdict(spec),
        "operator": {
            "valid": result.valid,
            "skipped": result.skipped,
            "rejection_reason": result.rejection_reason,
            "notes": result.operator_notes,
        },
        "sample_interval": {
            "wheel_tick_count": len(samples.wheel_ticks),
            "odometry_count": len(samples.odom),
            "imu_count": len(samples.imu),
        },
    }
    if spec.movement_type == "rotation":
        expected_angle = measurements.commanded_angle_rad
        physical_angle = measurements.physical_measurement
        report.update({
            "theoretical": {
                "expected_angle_rad": expected_angle,
                "expected_angle_deg": math.degrees(expected_angle),
                "expected_final_compass_heading_deg": (
                    None if result.initial_compass_heading_deg is None else
                    expected_final_compass_heading_deg(
                        result.initial_compass_heading_deg, expected_angle)),
            },
            "encoder": {
                "left_tick_total": left_ticks,
                "right_tick_total": right_ticks,
                "wheel_tick_semantics": wheel_tick_semantics,
                "left_distance_m": measurements.left_wheel_distance_m,
                "right_distance_m": measurements.right_wheel_distance_m,
                "track_width_m": geometry.track_width_m,
                "angle_rad": measurements.encoder_angle_rad,
                "angle_deg": math.degrees(measurements.encoder_angle_rad),
                "sign_convention": (
                    "positive is ROS counter-clockwise; right wheel distance "
                    "minus left wheel distance divided by track width"),
                "error_vs_theoretical_rad": (
                    measurements.encoder_angle_rad - expected_angle),
                "error_vs_compass_rad": (
                    None if physical_angle is None else
                    measurements.encoder_angle_rad - physical_angle),
            },
            "odometry": {
                "initial_yaw_rad": None if odom_start is None else odom_start.yaw_rad,
                "final_yaw_rad": None if odom_end is None else odom_end.yaw_rad,
                "unwrapped_angle_rad": measurements.odometry_angle_rad,
                "unwrapped_angle_deg": (
                    None if measurements.odometry_angle_rad is None else
                    math.degrees(measurements.odometry_angle_rad)),
                "error_vs_theoretical_rad": (
                    None if measurements.odometry_angle_rad is None else
                    measurements.odometry_angle_rad - expected_angle),
                "error_vs_compass_rad": (
                    None if measurements.odometry_angle_rad is None or
                    physical_angle is None else
                    measurements.odometry_angle_rad - physical_angle),
            },
            "imu": {
                "gyro_bias_rad_s": imu_bias_rad_s,
                "command_start_timestamp_s": measurements.command_start_timestamp_s,
                "command_end_timestamp_s": measurements.command_end_timestamp_s,
                "stationary_confirmation_timestamp_s": (
                    measurements.stationary_confirmation_timestamp_s),
                "source_topic": measurements.imu_source_topic,
                "start_boundary_gap_s": measurements.imu_start_boundary_gap_s,
                "command_end_boundary_gap_s": (
                    measurements.imu_command_end_boundary_gap_s),
                "stationary_boundary_gap_s": (
                    measurements.imu_stationary_boundary_gap_s),
                "integration_start_timestamp_s": (
                    None if imu_start is None else imu_start.timestamp_s),
                "integration_end_timestamp_s": (
                    None if imu_end is None else imu_end.timestamp_s),
                "commanded_window_interval_s": _interval_s(
                    measurements.command_start_timestamp_s,
                    measurements.command_end_timestamp_s),
                "settling_interval_s": _interval_s(
                    measurements.command_end_timestamp_s,
                    measurements.stationary_confirmation_timestamp_s),
                "total_integration_interval_s": _interval_s(
                    measurements.command_start_timestamp_s,
                    measurements.stationary_confirmation_timestamp_s),
                "commanded_window_sample_count": (
                    measurements.commanded_window_imu_sample_count),
                "settling_sample_count": measurements.settling_imu_sample_count,
                "total_sample_count": measurements.total_imu_sample_count,
                "commanded_window_angle_rad": (
                    measurements.commanded_window_imu_angle_rad),
                "commanded_window_angle_deg": _degrees_or_none(
                    measurements.commanded_window_imu_angle_rad),
                "settling_angle_rad": measurements.settling_imu_angle_rad,
                "settling_angle_deg": _degrees_or_none(
                    measurements.settling_imu_angle_rad),
                "total_physical_motion_angle_rad": measurements.imu_angle_rad,
                "total_physical_motion_angle_deg": _degrees_or_none(
                    measurements.imu_angle_rad),
                "initial_relative_heading_rad": (
                    None if imu_start is None else imu_start.integrated_angle_rad),
                "final_relative_heading_rad": (
                    None if imu_end is None else imu_end.integrated_angle_rad),
                "error_vs_theoretical_rad": measurements.imu_angle_rad - expected_angle,
                "error_vs_compass_rad": (
                    None if physical_angle is None else
                    measurements.imu_angle_rad - physical_angle),
            },
            "physical_reference": {
                "initial_compass_heading_deg": result.initial_compass_heading_deg,
                "final_compass_heading_deg": result.final_compass_heading_deg,
                "wrapped_physical_angle_deg": (
                    None if physical_angle is None else math.degrees(physical_angle)),
                "angle_rad": physical_angle,
            },
            "summary_comparison": {
                "theoretical": _comparison_entry(expected_angle, expected_angle),
                "encoder": _comparison_entry(measurements.encoder_angle_rad, expected_angle),
                "odometry": _comparison_entry(measurements.odometry_angle_rad, expected_angle),
                "imu": _comparison_entry(measurements.imu_angle_rad, expected_angle),
                "physical_compass": _comparison_entry(physical_angle, expected_angle),
            },
        })
        _validate_report_values(report)
        return report

    expected_distance = measurements.commanded_distance_m
    report.update({
        "theoretical": {
            "expected_displacement_m": expected_distance,
        },
        "encoder": {
            "left_tick_total": left_ticks,
            "right_tick_total": right_ticks,
            "wheel_tick_semantics": wheel_tick_semantics,
            "left_distance_m": measurements.left_wheel_distance_m,
            "right_distance_m": measurements.right_wheel_distance_m,
            "mean_distance_m": measurements.encoder_distance_m,
        },
        "odometry": {
            "displacement_m": measurements.odometry_distance_m,
        },
        "physical_reference": {
            "displacement_m": measurements.physical_measurement,
        },
        "summary_comparison": {
            "theoretical": _comparison_entry(expected_distance, expected_distance),
            "encoder": _comparison_entry(
                measurements.encoder_distance_m, expected_distance),
            "odometry": _comparison_entry(
                measurements.odometry_distance_m, expected_distance),
            "physical": _comparison_entry(
                measurements.physical_measurement, expected_distance),
        },
    })
    _validate_report_values(report)
    return report


def render_trial_report(report: Dict[str, object]) -> str:
    """Render the JSON report as concise terminal/evidence Markdown."""
    trial = report["trial"]
    lines = ["# Trial Report", "", "## Trial Configuration", ""]
    lines.extend([
        f"- direction: {str(trial['direction']).upper()}",
        f"- movement type: {trial['movement_type']}",
        f"- velocity: {trial['velocity']}",
        f"- duration_s: {trial['duration_s']}",
    ])
    if trial["movement_type"] == "rotation":
        theoretical = report["theoretical"]
        encoder = report["encoder"]
        odometry = report["odometry"]
        imu = report["imu"]
        physical = report["physical_reference"]
        lines.extend([
            "", "## Theoretical Command", "",
            f"- expected_angle_rad: {theoretical['expected_angle_rad']}",
            f"- expected_angle_deg: {theoretical['expected_angle_deg']}",
            "- expected_final_compass_heading_deg: "
            f"{theoretical['expected_final_compass_heading_deg']}",
            "", "## Encoder-based Rotation", "",
            f"- left_tick_total: {encoder['left_tick_total']}",
            f"- right_tick_total: {encoder['right_tick_total']}",
            f"- left_distance_m: {encoder['left_distance_m']}",
            f"- right_distance_m: {encoder['right_distance_m']}",
            f"- track_width_m: {encoder['track_width_m']}",
            f"- angle_rad: {encoder['angle_rad']}",
            f"- angle_deg: {encoder['angle_deg']}",
            f"- sign_convention: {encoder['sign_convention']}",
            f"- error_vs_theoretical_rad: {encoder['error_vs_theoretical_rad']}",
            f"- error_vs_compass_rad: {encoder['error_vs_compass_rad']}",
            "", "## Odometry Result", "",
            f"- initial_yaw_rad: {odometry['initial_yaw_rad']}",
            f"- final_yaw_rad: {odometry['final_yaw_rad']}",
            f"- unwrapped_angle_rad: {odometry['unwrapped_angle_rad']}",
            f"- unwrapped_angle_deg: {odometry['unwrapped_angle_deg']}",
            f"- error_vs_theoretical_rad: {odometry['error_vs_theoretical_rad']}",
            f"- error_vs_compass_rad: {odometry['error_vs_compass_rad']}",
            "", "## IMU-based Heading", "",
            f"- gyro_bias_rad_s: {imu['gyro_bias_rad_s']}",
            f"- source_topic: {imu['source_topic']}",
            f"- command_start_timestamp_s: {imu['command_start_timestamp_s']}",
            f"- command_end_timestamp_s: {imu['command_end_timestamp_s']}",
            "- stationary_confirmation_timestamp_s: "
            f"{imu['stationary_confirmation_timestamp_s']}",
            f"- commanded_window_interval_s: {imu['commanded_window_interval_s']}",
            f"- settling_interval_s: {imu['settling_interval_s']}",
            f"- total_integration_interval_s: {imu['total_integration_interval_s']}",
            f"- start_boundary_gap_s: {imu['start_boundary_gap_s']}",
            f"- command_end_boundary_gap_s: {imu['command_end_boundary_gap_s']}",
            f"- stationary_boundary_gap_s: {imu['stationary_boundary_gap_s']}",
            f"- commanded_window_sample_count: {imu['commanded_window_sample_count']}",
            f"- settling_sample_count: {imu['settling_sample_count']}",
            f"- total_sample_count: {imu['total_sample_count']}",
            f"- commanded_window_angle_rad: {imu['commanded_window_angle_rad']}",
            f"- commanded_window_angle_deg: {imu['commanded_window_angle_deg']}",
            f"- settling_angle_rad: {imu['settling_angle_rad']}",
            f"- settling_angle_deg: {imu['settling_angle_deg']}",
            "- total_physical_motion_angle_rad: "
            f"{imu['total_physical_motion_angle_rad']}",
            "- total_physical_motion_angle_deg: "
            f"{imu['total_physical_motion_angle_deg']}",
            f"- initial_relative_heading_rad: {imu['initial_relative_heading_rad']}",
            f"- final_relative_heading_rad: {imu['final_relative_heading_rad']}",
            f"- error_vs_theoretical_rad: {imu['error_vs_theoretical_rad']}",
            f"- error_vs_compass_rad: {imu['error_vs_compass_rad']}",
            "", "## Physical Reference", "",
            f"- initial_compass_heading_deg: {physical['initial_compass_heading_deg']}",
            f"- final_compass_heading_deg: {physical['final_compass_heading_deg']}",
            f"- wrapped_physical_angle_deg: {physical['wrapped_physical_angle_deg']}",
        ])
    else:
        theoretical = report["theoretical"]
        encoder = report["encoder"]
        odometry = report["odometry"]
        physical = report["physical_reference"]
        lines.extend([
            "", "## Theoretical Command", "",
            f"- expected_displacement_m: {theoretical['expected_displacement_m']}",
            "", "## Encoder-based Translation", "",
            f"- left_tick_total: {encoder['left_tick_total']}",
            f"- right_tick_total: {encoder['right_tick_total']}",
            f"- left_distance_m: {encoder['left_distance_m']}",
            f"- right_distance_m: {encoder['right_distance_m']}",
            f"- mean_distance_m: {encoder['mean_distance_m']}",
            "", "## Odometry Result", "",
            f"- displacement_m: {odometry['displacement_m']}",
            "", "## Physical Reference", "",
            f"- displacement_m: {physical['displacement_m']}",
        ])
    lines.extend([
        "", "## Summary Comparison", "",
        "| estimator | value | signed error | absolute error | percent error |",
        "| --- | ---: | ---: | ---: | ---: |",
    ])
    comparison_names = (
        "theoretical", "encoder", "odometry", "imu", "physical_compass",
        "physical")
    comparisons = report["summary_comparison"]
    for name in (
            [name for name in comparison_names if name in comparisons] +
            sorted(name for name in comparisons if name not in comparison_names)):
        comparison = comparisons[name]
        lines.append(
            "| {name} | {value} | {signed_error} | {absolute_error} | {percentage_error} |".format(
                name=name, **comparison))
    lines.extend(["", "## Operator", ""])
    for key in sorted(report["operator"]):
        lines.append(f"- {key}: {report['operator'][key]}")
    return "\n".join(lines) + "\n"


def compute_pairwise_errors(values: Dict[str, Optional[float]]) -> Dict[str, Optional[float]]:
    """Compute signed pairwise differences for available measurements."""
    errors = {}
    names = sorted(values)
    for left_index, left in enumerate(names):
        for right in names[left_index + 1:]:
            left_value = values[left]
            right_value = values[right]
            key = f"{left}_minus_{right}"
            errors[key] = (
                None if left_value is None or right_value is None
                else left_value - right_value)
    return errors


def build_measurements(
        spec: TrialSpec,
        samples: TrialSamples,
        geometry: GeometryConfig,
        physical_measurement: Optional[float],
        imu_bias_rad_s: float = 0.0,
        wheel_tick_semantics: str = "delta",
        command_start_timestamp_s: Optional[float] = None,
        command_end_timestamp_s: Optional[float] = None,
        stationary_confirmation_timestamp_s: Optional[float] = None,
        imu_source_topic: str = PRIMARY_IMU_SOURCE_TOPIC,
        imu_boundary_tolerance_s: Optional[float] = None
        ) -> Tuple[TrialMeasurements, TrialSamples]:
    """Compute all validator-side measurements for one trial."""
    encoder = wheel_tick_measurements(
        samples.wheel_ticks, geometry, wheel_tick_semantics)
    odom_distance, odom_angle = odometry_measurements(samples.odom)
    if spec.movement_type == "translation":
        odom_distance = odometry_signed_displacement(samples.odom)
    if (
            command_start_timestamp_s is None and
            command_end_timestamp_s is None and
            stationary_confirmation_timestamp_s is None):
        command_imu = settling_imu = total_imu = ()
        command_angle = settling_angle = imu_angle = 0.0
        processed_imu = ()
    else:
        if (
                command_start_timestamp_s is None or
                command_end_timestamp_s is None or
                stationary_confirmation_timestamp_s is None):
            raise ValidationError("complete physical-motion IMU window is unavailable")
        command_imu = imu_samples_in_motion_window(
            samples.imu,
            command_start_timestamp_s,
            command_end_timestamp_s,
            imu_source_topic,
            imu_boundary_tolerance_s)
        if stationary_confirmation_timestamp_s == command_end_timestamp_s:
            settling_imu = ()
            settling_angle = 0.0
        else:
            settling_imu = imu_samples_in_motion_window(
                samples.imu,
                command_end_timestamp_s,
                stationary_confirmation_timestamp_s,
                imu_source_topic,
                imu_boundary_tolerance_s)
            settling_angle, _ = integrate_imu_angle(
                settling_imu, imu_bias_rad_s)
        total_imu = imu_samples_in_motion_window(
            samples.imu,
            command_start_timestamp_s,
            stationary_confirmation_timestamp_s,
            imu_source_topic,
            imu_boundary_tolerance_s)
        split_imu = imu_samples_with_interpolated_boundary(
            total_imu, command_end_timestamp_s)
        command_imu = tuple(
            sample for sample in split_imu
            if command_start_timestamp_s <= sample.timestamp_s <=
            command_end_timestamp_s)
        if settling_imu:
            settling_imu = tuple(
                sample for sample in split_imu
                if command_end_timestamp_s <= sample.timestamp_s <=
                stationary_confirmation_timestamp_s)
            settling_angle, _ = integrate_imu_angle(
                settling_imu, imu_bias_rad_s)
        command_angle, _ = integrate_imu_angle(command_imu, imu_bias_rad_s)
        imu_angle, processed_imu = integrate_imu_angle(total_imu, imu_bias_rad_s)
    enriched_samples = TrialSamples(
        wheel_ticks=tuple(samples.wheel_ticks),
        imu=tuple(samples.imu),
        processed_imu=processed_imu,
        odom=tuple(samples.odom),
        diagnostics=tuple(samples.diagnostics),
        ignored_diagnostics=tuple(samples.ignored_diagnostics),
        commands=tuple(samples.commands),
        stationarity=tuple(samples.stationarity))
    measurements = TrialMeasurements(
        encoder_distance_m=encoder[0],
        encoder_angle_rad=encoder[1],
        left_wheel_distance_m=encoder[2],
        right_wheel_distance_m=encoder[3],
        odometry_distance_m=odom_distance,
        odometry_angle_rad=odom_angle,
        imu_angle_rad=imu_angle,
        physical_measurement=physical_measurement,
        commanded_distance_m=spec.commanded_distance_m,
        commanded_angle_rad=spec.commanded_angle_rad,
        command_start_timestamp_s=command_start_timestamp_s,
        command_end_timestamp_s=command_end_timestamp_s,
        stationary_confirmation_timestamp_s=stationary_confirmation_timestamp_s,
        commanded_window_imu_angle_rad=command_angle,
        settling_imu_angle_rad=settling_angle,
        commanded_window_imu_sample_count=len(command_imu),
        settling_imu_sample_count=len(settling_imu),
        total_imu_sample_count=len(total_imu),
        imu_source_topic=imu_source_topic,
        imu_start_boundary_gap_s=(
            None if not total_imu else
            min(sample.timestamp_s for sample in total_imu) -
            command_start_timestamp_s),
        imu_command_end_boundary_gap_s=(
            None if not command_imu else max(
                command_end_timestamp_s -
                max(sample.timestamp_s for sample in command_imu),
                0.0 if not settling_imu else
                min(sample.timestamp_s for sample in settling_imu) -
                command_end_timestamp_s)),
        imu_stationary_boundary_gap_s=(
            None if not total_imu else stationary_confirmation_timestamp_s -
            max(sample.timestamp_s for sample in total_imu)))
    return measurements, enriched_samples


def comparison_values(
        spec: TrialSpec,
        measurements: TrialMeasurements) -> Dict[str, Optional[float]]:
    """Select distance or angle values for pairwise comparison."""
    if spec.movement_type == "rotation":
        return {
            "commanded": measurements.commanded_angle_rad,
            "encoder": measurements.encoder_angle_rad,
            "odometry": measurements.odometry_angle_rad,
            "imu": measurements.imu_angle_rad,
            "physical": measurements.physical_measurement,
        }
    return {
        "commanded": measurements.commanded_distance_m,
        "encoder": measurements.encoder_distance_m,
        "odometry": measurements.odometry_distance_m,
        "physical": measurements.physical_measurement,
    }


def summarize_results(results: Sequence[TrialResult]) -> List[Dict[str, object]]:
    """Return summary statistics grouped by comparison name."""
    grouped: Dict[str, List[float]] = {}
    for result in results:
        if not result.valid or result.skipped:
            continue
        for key, value in result.errors.items():
            if value is None:
                continue
            grouped.setdefault(key, []).append(value)

    summary = []
    for key in sorted(grouped):
        values = grouped[key]
        mean = statistics.mean(values)
        stddev = statistics.pstdev(values) if len(values) > 1 else 0.0
        rmse = math.sqrt(statistics.mean(value * value for value in values))
        summary.append({
            "comparison": key,
            "count": len(values),
            "mean_error": mean,
            "stddev": stddev,
            "rmse": rmse,
            "maximum_error": max(values),
            "minimum_error": min(values),
        })
    return summary


class EvidenceWriter:
    """Append-only evidence writer for one validation campaign."""

    def __init__(self, root: Path, prefix: str = "odometry-validation"):
        self.root = Path(root)
        self.prefix = prefix
        self.directory: Optional[Path] = None
        self._campaign_metadata: Dict[str, object] = {}
        self._ignored_diagnostics: List[DiagnosticSample] = []
        self._ignored_diagnostic_keys = set()

    def create(self, metadata: Dict[str, object]) -> Path:
        timestamp = utc_timestamp()
        for attempt in range(1000):
            suffix = f"-{attempt:03d}" if attempt else ""
            candidate = self.root / f"{self.prefix}-{timestamp}{suffix}"
            try:
                os.makedirs(candidate, mode=0o755, exist_ok=False)
            except FileExistsError:
                continue
            self.directory = candidate
            self._campaign_metadata = dict(metadata)
            self._write_json(candidate / "metadata.json", metadata)
            return candidate
        raise RuntimeError("could not create unique evidence directory")

    def write_trial(
            self,
            result: TrialResult,
            samples: TrialSamples) -> Path:
        if self.directory is None:
            raise RuntimeError("evidence directory has not been created")
        report = None
        if (
                result.valid and not result.skipped and
                "geometry" in self._campaign_metadata and
                "stationarity_thresholds" in self._campaign_metadata):
            geometry_values = self._campaign_metadata["geometry"]
            thresholds = self._campaign_metadata["stationarity_thresholds"]
            report = build_trial_report(
                result,
                samples,
                GeometryConfig(**geometry_values),
                wheel_tick_semantics=thresholds["wheel_tick_semantics"],
                imu_bias_rad_s=self._campaign_metadata.get(
                    "imu_bias_rad_s", 0.0))
        trial_dir = self._unique_trial_dir(result.spec.trial_id)
        self._write_json(
            trial_dir / "metadata.json", self._trial_payload(result, samples))
        self._write_csv(trial_dir / "wheel_ticks.csv", samples.wheel_ticks)
        self._write_csv(trial_dir / "raw_imu.csv", samples.imu)
        self._write_csv(trial_dir / "processed_imu.csv", samples.processed_imu)
        self._write_csv(trial_dir / "odometry.csv", samples.odom)
        self._write_csv(trial_dir / "diagnostics.csv", samples.diagnostics)
        self._write_csv(
            trial_dir / "ignored_diagnostics.csv",
            samples.ignored_diagnostics)
        self._write_csv(trial_dir / "commanded_velocity.csv", samples.commands)
        stationarity_rows = tuple(
            row
            for assessment in samples.stationarity
            for row in assessment.stationarity_samples)
        self._write_csv(
            trial_dir / "stationarity_window.csv", stationarity_rows)
        self._write_json(trial_dir / "stationarity.json", {
            "assessments": [
                asdict(assessment) for assessment in samples.stationarity],
            "final_accepted_window": next((
                asdict(assessment)
                for assessment in reversed(samples.stationarity)
                if assessment.stationary), None),
        })
        if report is not None:
            self._write_json(trial_dir / "report.json", report)
            with (trial_dir / "report.md").open("x", encoding="utf-8") as stream:
                stream.write(render_trial_report(report))
        self._record_ignored_diagnostics(samples.ignored_diagnostics)
        return trial_dir

    def _unique_trial_dir(self, trial_id: str) -> Path:
        if self.directory is None:
            raise RuntimeError("evidence directory has not been created")
        for attempt in range(1000):
            suffix = f"-attempt-{attempt + 1:03d}"
            candidate = self.directory / f"{trial_id}{suffix}"
            try:
                os.makedirs(candidate, mode=0o755, exist_ok=False)
            except FileExistsError:
                continue
            return candidate
        raise RuntimeError(f"could not create unique trial directory for {trial_id}")

    def write_summary(self, results: Sequence[TrialResult]) -> None:
        if self.directory is None:
            raise RuntimeError("evidence directory has not been created")
        summary = summarize_results(results)
        self._write_table(self.directory / "summary.csv", summary)
        self._write_csv(
            self.directory / "ignored_diagnostics.csv",
            self._ignored_diagnostics)
        self._write_json(self.directory / "campaign-summary.json", {
            "recorded_trial_count": len(results),
            "valid_trial_count": sum(1 for result in results if result.valid),
            "skipped_trial_count": sum(1 for result in results if result.skipped),
            "summary": summary,
        })
        report = self._report_text(results, summary)
        (self.directory / "report.md").write_text(report, encoding="utf-8")

    def write_failure(
            self,
            results: Sequence[TrialResult],
            error: BaseException,
            samples: TrialSamples,
            failure_context: Optional[Dict[str, object]] = None) -> None:
        """Persist fail-closed outcome and ignored samples before re-raising."""
        if self.directory is None:
            raise RuntimeError("evidence directory has not been created")
        self._record_ignored_diagnostics(samples.ignored_diagnostics)
        traceback_text = self._format_failure_traceback(error)
        payload = {
            "error_type": type(error).__name__,
            "error": str(error),
            "traceback": traceback_text,
            "ignored_diagnostic_names": sorted({
                sample.name for sample in self._ignored_diagnostics}),
            "ignored_diagnostic_samples": [
                asdict(sample) for sample in self._ignored_diagnostics],
            "samples": self._samples_payload(samples),
        }
        payload.update(self._emergency_stop_payload(error))
        if failure_context:
            payload["failure_context"] = failure_context
        if isinstance(error, EmergencyStopCleanupError):
            payload["primary_traceback"] = self._format_failure_traceback(
                error.primary_error)
            payload["cleanup_traceback"] = self._format_failure_traceback(
                error.cleanup_error)
        self._write_json(self.directory / "failure.json", payload)
        self._write_json(
            self.directory / "failure_samples.json",
            self._samples_payload(samples))
        self._write_failure_csv_files(samples)
        with (self.directory / "traceback.txt").open(
                "x", encoding="utf-8") as stream:
            stream.write(traceback_text)
        summary = summarize_results(results)
        self._write_table(self.directory / "summary.csv", summary)
        self._write_csv(
            self.directory / "ignored_diagnostics.csv",
            self._ignored_diagnostics)
        report = self._report_text(results, summary, failure=str(error))
        (self.directory / "report.md").write_text(report, encoding="utf-8")

    def _write_failure_csv_files(self, samples: TrialSamples) -> None:
        if self.directory is None:
            raise RuntimeError("evidence directory has not been created")
        self._write_csv(self.directory / "wheel_ticks.csv", samples.wheel_ticks)
        self._write_csv(self.directory / "raw_imu.csv", samples.imu)
        self._write_csv(self.directory / "processed_imu.csv", samples.processed_imu)
        self._write_csv(self.directory / "odometry.csv", samples.odom)
        self._write_csv(self.directory / "diagnostics.csv", samples.diagnostics)
        self._write_csv(
            self.directory / "commanded_velocity.csv", samples.commands)
        stationarity_rows = tuple(
            row
            for assessment in samples.stationarity
            for row in assessment.stationarity_samples)
        self._write_csv(
            self.directory / "stationarity_window.csv", stationarity_rows)

    @staticmethod
    def _samples_payload(samples: TrialSamples) -> Dict[str, object]:
        return {
            "wheel_ticks": [asdict(sample) for sample in samples.wheel_ticks],
            "imu": [asdict(sample) for sample in samples.imu],
            "processed_imu": [
                asdict(sample) for sample in samples.processed_imu],
            "odom": [asdict(sample) for sample in samples.odom],
            "diagnostics": [
                asdict(sample) for sample in samples.diagnostics],
            "ignored_diagnostics": [
                asdict(sample) for sample in samples.ignored_diagnostics],
            "commands": [asdict(sample) for sample in samples.commands],
            "stationarity": [
                asdict(assessment) for assessment in samples.stationarity],
        }

    @staticmethod
    def _emergency_stop_payload(error: BaseException) -> Dict[str, object]:
        records = []

        def collect(current: BaseException) -> None:
            if isinstance(current, EmergencyStopCleanupError):
                collect(current.primary_error)
                collect(current.cleanup_error)
            elif isinstance(current, EmergencyStopError) and current.record:
                records.append(current.record)

        collect(error)
        if not records:
            return {}
        return {
            "emergency_stop_records": records,
            "final_emergency_stop_record": records[-1],
        }

    @staticmethod
    def _format_failure_traceback(error: BaseException) -> str:
        if isinstance(error, EmergencyStopCleanupError):
            primary = EvidenceWriter._format_failure_traceback(
                error.primary_error)
            cleanup = EvidenceWriter._format_failure_traceback(
                error.cleanup_error)
            return (
                "Primary failure traceback:\n" + primary +
                "\nZero-command cleanup failure traceback:\n" + cleanup)
        return "".join(traceback.format_exception(
            type(error), error, error.__traceback__))

    def _record_ignored_diagnostics(
            self,
            samples: Sequence[DiagnosticSample]) -> None:
        for sample in samples:
            if sample in self._ignored_diagnostic_keys:
                continue
            self._ignored_diagnostic_keys.add(sample)
            self._ignored_diagnostics.append(sample)

    @staticmethod
    def _write_json(path: Path, payload: Dict[str, object]) -> None:
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        temporary = path.with_name(
            f".{path.name}.tmp-{os.getpid()}-{id(payload)}")
        try:
            with os.fdopen(os.open(temporary, flags, 0o644), "w", encoding="utf-8") as stream:
                json.dump(
                    EvidenceWriter._json_safe(payload),
                    stream,
                    indent=2,
                    sort_keys=True,
                    allow_nan=False)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            # Hard-link is atomic and refuses to replace an earlier failure.
            os.link(temporary, path)
        finally:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass

    @staticmethod
    def _json_safe(value: object) -> object:
        """Encode malformed floating values explicitly without invalid JSON."""
        if isinstance(value, float) and not math.isfinite(value):
            return {"nonfinite_float": repr(value)}
        if isinstance(value, dict):
            return {
                str(key): EvidenceWriter._json_safe(nested)
                for key, nested in value.items()}
        if isinstance(value, (tuple, list)):
            return [EvidenceWriter._json_safe(nested) for nested in value]
        return value

    @staticmethod
    def _write_csv(path: Path, rows: Sequence[object]) -> None:
        with path.open("x", encoding="utf-8", newline="") as stream:
            if not rows:
                stream.write("")
                return
            fieldnames = list(asdict(rows[0]).keys())
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(asdict(row))

    @staticmethod
    def _write_table(path: Path, rows: Sequence[Dict[str, object]]) -> None:
        with path.open("x", encoding="utf-8", newline="") as stream:
            if not rows:
                stream.write("")
                return
            fieldnames = list(rows[0].keys())
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

    @staticmethod
    def _trial_payload(
            result: TrialResult,
            samples: TrialSamples) -> Dict[str, object]:
        return {
            "trial": asdict(result.spec),
            "timestamp": result.timestamp,
            "measurements": asdict(result.measurements),
            "errors": result.errors,
            "valid": result.valid,
            "skipped": result.skipped,
            "rejection_reason": result.rejection_reason,
            "operator_notes": result.operator_notes,
            "evidence_dir": result.evidence_dir,
            "initial_compass_heading_deg": result.initial_compass_heading_deg,
            "final_compass_heading_deg": result.final_compass_heading_deg,
            "ignored_diagnostic_names": sorted({
                sample.name for sample in samples.ignored_diagnostics}),
            "ignored_diagnostic_samples": [
                asdict(sample) for sample in samples.ignored_diagnostics],
            "stationarity": [
                asdict(assessment) for assessment in samples.stationarity],
        }

    def _report_text(
            self,
            results: Sequence[TrialResult],
            summary: Sequence[Dict[str, object]],
            failure: Optional[str] = None) -> str:
        valid_count = sum(1 for result in results if result.valid)
        skipped_count = sum(1 for result in results if result.skipped)
        lines = [
            "# Odometry Validation Report",
            "",
            f"Trials recorded: {len(results)}",
            f"Valid trials: {valid_count}",
            f"Skipped trials: {skipped_count}",
            "",
        ]
        if failure is not None:
            lines.extend(["Outcome: failed closed", f"Failure: {failure}", ""])
        lines.extend(["## Ignored Diagnostics", ""])
        configured_names = self._campaign_metadata.get(
            "ignored_diagnostic_names", [])
        if configured_names:
            lines.append(
                "Configured exact names: " + ", ".join(configured_names))
        else:
            lines.append("Configured exact names: none")
        lines.append(
            f"Ignored samples recorded: {len(self._ignored_diagnostics)}")
        for sample in self._ignored_diagnostics:
            message = sample.message.replace("\n", " ")
            lines.append(
                f"- timestamp_s={sample.timestamp_s:.9g}, "
                f"level={sample.level}, name={sample.name}, message={message}")
        lines.extend([
            "",
            "## Summary Statistics",
            "",
        ])
        if not summary:
            lines.append("No valid comparison data recorded.")
        else:
            lines.append(
                "| comparison | count | mean | stddev | rmse | min | max |")
            lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: |")
            for row in summary:
                lines.append(
                    "| {comparison} | {count} | {mean_error:.9g} | "
                    "{stddev:.9g} | {rmse:.9g} | {minimum_error:.9g} | "
                    "{maximum_error:.9g} |".format(**row))
        lines.extend(["", "## Trials", ""])
        for result in results:
            lines.append(
                f"- {result.spec.trial_id}: valid={result.valid}, "
                f"skipped={result.skipped}, notes={result.operator_notes}")
        return "\n".join(lines) + "\n"


class EmergencyStopController:
    """Bounded zero-command stop routine with injectable ROS-facing operations."""

    def __init__(
            self,
            publish_zero: Callable[[], None],
            verify_safe_zero: Callable[[], bool],
            verify_stationary: Callable[[], object],
            sleep: Callable[[float], None] = time.sleep,
            record_result: Optional[Callable[[Dict[str, object]], None]] = None,
            stationarity_required: Optional[Callable[[], bool]] = None,
            begin_stop: Optional[Callable[[], None]] = None,
            prepare_verification: Optional[Callable[[], None]] = None,
            verify_stop_guards: Optional[Callable[[], None]] = None,
            cleanup_context_valid: Optional[Callable[[], bool]] = None,
            cleanup_publisher_valid: Optional[Callable[[], bool]] = None,
            confirmed_safe_state: Optional[Callable[[], Dict[str, object]]] = None,
            monotonic: Callable[[], float] = time.monotonic):
        self.publish_zero = publish_zero
        self.verify_safe_zero = verify_safe_zero
        self.verify_stationary = verify_stationary
        self.sleep = sleep
        self.record_result = record_result
        self.stationarity_required = stationarity_required or (lambda: True)
        self.begin_stop = begin_stop
        self.prepare_verification = prepare_verification
        self.verify_stop_guards = verify_stop_guards
        self.cleanup_context_valid = cleanup_context_valid or (lambda: True)
        self.cleanup_publisher_valid = cleanup_publisher_valid or (lambda: True)
        self.confirmed_safe_state = confirmed_safe_state or (lambda: {})
        self.monotonic = monotonic
        self.attempts = 0

    def stop(
            self,
            timeout_s: float,
            rate_hz: float,
            mode: str = "emergency_cleanup") -> Dict[str, object]:
        if (
                not math.isfinite(timeout_s) or timeout_s <= 0.0 or
                not math.isfinite(rate_hz) or rate_hz <= 0.0):
            raise ValueError("timeout_s and rate_hz must be finite and positive")
        if mode not in ("controlled", "emergency_cleanup"):
            raise ValueError("mode must be 'controlled' or 'emergency_cleanup'")
        context_valid = False
        publisher_valid = False
        invalid_reason = None
        try:
            context_valid = bool(self.cleanup_context_valid())
            if context_valid:
                publisher_valid = bool(self.cleanup_publisher_valid())
            if not context_valid:
                invalid_reason = "ROS context is invalid"
            elif not publisher_valid:
                invalid_reason = "zero-command publisher is invalid"
        except BaseException as error:
            invalid_reason = (
                f"cleanup lifecycle check failed: {type(error).__name__}: {error}")
        if not context_valid or not publisher_valid:
            try:
                confirmed = dict(self.confirmed_safe_state() or {})
            except BaseException as error:
                confirmed = {}
                invalid_reason = (
                    f"{invalid_reason}; confirmed-state read failed: "
                    f"{type(error).__name__}: {error}")
            record = {
                "stop_mode": mode,
                "timeout_s": timeout_s,
                "zero_publish_count": 0,
                "safe_zero": confirmed.get("safe_zero"),
                "stationary": confirmed.get("stationary"),
                "stationarity_required": confirmed.get("stationarity_required"),
                "active_zero_verification_possible": False,
                "cleanup_skipped_reason": invalid_reason,
                "ros_context_valid": context_valid,
                "zero_publisher_valid": publisher_valid,
                "timeout_reason": invalid_reason,
            }
            if self.record_result is not None:
                self.record_result(record)
            return record
        period_s = 1.0 / rate_hz
        if self.begin_stop is not None:
            self.begin_stop()
        if self.prepare_verification is not None:
            self.prepare_verification()
        require_stationarity = bool(self.stationarity_required())
        assessments = []
        first_zero_monotonic = None
        safe_zero = False
        stationary = None if not require_stationarity else False
        stationarity_result = None
        zero_publish_count = 0
        deadline = None
        stationarity = {
            "required": require_stationarity,
            "reason": (
                "motion was not armed or published"
                if not require_stationarity else
                "stationarity was not assessed"),
        }

        while True:
            self.publish_zero()
            self.attempts += 1
            zero_publish_count += 1
            if first_zero_monotonic is None:
                first_zero_monotonic = self.monotonic()
                deadline = first_zero_monotonic + timeout_s
            remaining_s = max(0.0, deadline - self.monotonic())
            if remaining_s > 0.0:
                self.sleep(min(period_s, remaining_s))
            if (
                    mode == "controlled" and require_stationarity and
                    self.verify_stop_guards is not None):
                self.verify_stop_guards()
            safe_zero = bool(self.verify_safe_zero())
            if require_stationarity:
                stationarity_result = self.verify_stationary()
                if isinstance(stationarity_result, StationarityAssessment):
                    stationary = stationarity_result.stationary
                    stationarity = asdict(stationarity_result)
                else:
                    stationary = bool(stationarity_result)
                    stationarity = {
                        "stationary": stationary,
                        "reason": "legacy boolean stationarity verifier",
                    }
                assessments.append(stationarity)
            if safe_zero and (not require_stationarity or stationary):
                elapsed_s = max(
                    0.0, self.monotonic() - first_zero_monotonic)
                if isinstance(stationarity_result, StationarityAssessment):
                    elapsed_s = (
                        stationarity_result.elapsed_since_first_zero_s
                        if stationarity_result.elapsed_since_first_zero_s is not None
                        else elapsed_s)
                record = {
                    "stop_mode": mode,
                    "timeout_s": timeout_s,
                    "zero_publish_count": zero_publish_count,
                    "safe_zero": safe_zero,
                    "stationarity_required": require_stationarity,
                    "stationary": stationary,
                    "stationarity_assessments": assessments,
                    "final_accepted_window": (
                        stationarity if require_stationarity else None),
                    "time_from_first_zero_to_stationary_s": (
                        elapsed_s if require_stationarity else None),
                    "timeout_reason": None,
                }
                if self.record_result is not None:
                    self.record_result(record)
                return record
            if self.monotonic() >= deadline:
                break

        if not safe_zero:
            timeout_detail = "safe zero was not confirmed"
        elif require_stationarity:
            timeout_detail = stationarity.get("reason", "stationarity not confirmed")
        else:
            timeout_detail = "stop verification did not complete"
        timeout_reason = (
            f"stop verification timeout expired after {timeout_s:.3f}s: "
            f"{timeout_detail}")
        record = {
            "stop_mode": mode,
            "timeout_s": timeout_s,
            "zero_publish_count": zero_publish_count,
            "safe_zero": safe_zero,
            "stationarity_required": require_stationarity,
            "stationary": stationary,
            "stationarity_assessments": assessments,
            "final_accepted_window": None,
            "time_from_first_zero_to_stationary_s": None,
            "timeout_reason": timeout_reason,
        }
        if self.record_result is not None:
            self.record_result(record)
        summary = {
            "zero_publish_count": zero_publish_count,
            "safe_zero": safe_zero,
            "stationary": stationary,
            "reason": timeout_reason,
        }
        error_type = (
            ControlledStopError if mode == "controlled" else
            EmergencyStopError)
        description = (
            "controlled stop verification failed"
            if mode == "controlled" else
            "emergency stop verification failed")
        raise error_type(f"{description}: {summary}", record)


class EmergencyCleanupOnce:
    """Ensure exception-triggered zero-command cleanup has one owner."""

    def __init__(self, controller: EmergencyStopController):
        self.controller = controller
        self.attempted = False
        self.running = False
        self.completed = False
        self.second_interrupt = False
        self.result: Optional[Dict[str, object]] = None
        self.error: Optional[BaseException] = None
        self._lock = Lock()

    def stop(self, timeout_s: float, rate_hz: float) -> Dict[str, object]:
        """Run cleanup once; later exception handlers must not run it again."""
        with self._lock:
            if self.attempted:
                self.second_interrupt = True
                if self.completed:
                    return {"already_attempted": True}
                return {
                    "already_attempted": True,
                    "cleanup_state": "completed" if self.completed else "running",
                    "second_interrupt": True,
                }
            self.attempted = True
            self.running = True
        try:
            result = self.controller.stop(timeout_s, rate_hz)
        except BaseException as error:
            with self._lock:
                self.running = False
                self.error = error
            raise
        with self._lock:
            self.running = False
            self.completed = True
            self.result = dict(result)
        return result


def run_with_emergency_stop(
        action: Callable[[], object],
        emergency_stop: EmergencyCleanupOnce,
        timeout_s: float,
        rate_hz: float) -> object:
    """Run an action and stop on interruption or exception."""
    try:
        return action()
    except BaseException as primary_error:
        try:
            emergency_stop.stop(timeout_s, rate_hz)
        except BaseException as cleanup_error:
            raise EmergencyStopCleanupError(
                primary_error, cleanup_error) from primary_error
        raise


class OperatorInterface:
    """Small protocol wrapper for tests and the interactive node."""

    def __init__(self, prompt: Callable[[str], str]):
        self.prompt = prompt

    def ask_validity(self) -> Tuple[str, Optional[str], str]:
        while True:
            answer = self.prompt("Was this trial valid? [yes/no/skip]: ").strip().lower()
            if answer in ("yes", "y"):
                notes = self.prompt("Operator notes: ").strip()
                return "valid", None, notes
            if answer in ("no", "n"):
                reason = self.prompt("Rejection reason: ").strip()
                return "invalid", reason, ""
            if answer in ("skip", "s"):
                reason = self.prompt("Skip reason: ").strip()
                return "skipped", reason, ""


@dataclass(frozen=True)
class InteractiveLimits:
    """Explicit reject-only limits for operator-derived interactive trials."""

    max_angular_velocity_rad_s: float
    max_rotation_duration_s: float
    min_linear_velocity_m_s: float
    max_linear_velocity_m_s: float
    min_translation_duration_s: float
    max_translation_duration_s: float

    def __post_init__(self):
        values = (
            self.max_angular_velocity_rad_s,
            self.max_rotation_duration_s,
            self.min_linear_velocity_m_s,
            self.max_linear_velocity_m_s,
            self.min_translation_duration_s,
            self.max_translation_duration_s,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("interactive limits must be finite and positive")
        if self.min_linear_velocity_m_s > self.max_linear_velocity_m_s:
            raise ValueError("minimum linear velocity exceeds maximum")
        if self.min_translation_duration_s > self.max_translation_duration_s:
            raise ValueError("minimum duration exceeds maximum")


def describe_trial(spec: TrialSpec) -> str:
    """Return the exact movement proposal shown before confirmation."""
    if spec.movement_type == "rotation":
        return (
            f"rotation direction={spec.direction.upper()} "
            f"angular_velocity_rad_s={spec.velocity:.9g} "
            f"duration_s={spec.duration_s:.9g} "
            f"expected_angle_rad={spec.commanded_angle_rad:.9g}")
    return (
        f"translation direction={spec.direction} "
        f"linear_velocity_m_s={spec.velocity:.9g} "
        f"duration_s={spec.duration_s:.9g} "
        f"expected_displacement_m={spec.commanded_distance_m:.9g}")


class InteractiveTrialMenu:
    """Confirmation-gated post-trial menu with no direct ROS dependencies."""

    def __init__(
            self,
            operator_input: "ResponsiveOperatorInput",
            limits: InteractiveLimits,
            display: Callable[[str], None]):
        self.operator_input = operator_input
        self.limits = limits
        self.display = display

    def choose_next(self, previous: TrialSpec, sequence: int) -> Optional[TrialSpec]:
        """Return one confirmed next trial, or None when the campaign finishes."""
        while True:
            option = self._read_menu_option()
            if option == 5:
                return None
            if option == 1:
                candidate = previous
            elif option == 2:
                if previous.movement_type != "rotation":
                    self.operator_input.notify("Option 2 is available for rotation only.")
                    continue
                duration_s = previous.duration_s * 2.0
                if (
                        not math.isfinite(duration_s) or
                        duration_s > self.limits.max_rotation_duration_s):
                    self.operator_input.notify(
                        "The doubled rotation duration exceeds the configured "
                        f"limit ({self.limits.max_rotation_duration_s:.9g} s).")
                    continue
                candidate = self._rotation_trial(
                    previous, previous.velocity, duration_s, sequence)
            elif option == 3:
                if previous.movement_type != "rotation":
                    self.operator_input.notify("Option 3 is available for rotation only.")
                    continue
                velocity = previous.velocity * 1.5
                if velocity > self.limits.max_angular_velocity_rad_s:
                    velocity = self._read_lower_rotation_velocity()
                    if velocity is None:
                        continue
                candidate = self._rotation_trial(
                    previous, velocity, previous.duration_s, sequence)
            else:
                candidate = self._translation_trial(previous, sequence)
            if (
                    candidate.movement_type == "rotation" and
                    not self._rotation_candidate_is_permitted(candidate)):
                self.operator_input.notify(
                    "Rotation velocity and duration must be finite and positive "
                    f"and not exceed {self.limits.max_angular_velocity_rad_s:.9g} "
                    "rad/s and "
                    f"{self.limits.max_rotation_duration_s:.9g} s.")
                continue
            if (
                    candidate.movement_type == "translation" and
                    not self._translation_candidate_is_permitted(candidate)):
                self.operator_input.notify(
                    "Translation direction must be forward or backward; velocity "
                    f"must be within [{self.limits.min_linear_velocity_m_s:.9g}, "
                    f"{self.limits.max_linear_velocity_m_s:.9g}] m/s; duration "
                    f"must be within [{self.limits.min_translation_duration_s:.9g}, "
                    f"{self.limits.max_translation_duration_s:.9g}] s.")
                continue
            if self._confirm(candidate):
                return candidate

    def _rotation_candidate_is_permitted(self, candidate: TrialSpec) -> bool:
        return (
            math.isfinite(candidate.velocity) and candidate.velocity > 0.0 and
            candidate.velocity <= self.limits.max_angular_velocity_rad_s and
            math.isfinite(candidate.duration_s) and candidate.duration_s > 0.0 and
            candidate.duration_s <= self.limits.max_rotation_duration_s)

    def _translation_candidate_is_permitted(self, candidate: TrialSpec) -> bool:
        return (
            candidate.direction in ("forward", "backward") and
            math.isfinite(candidate.velocity) and
            self.limits.min_linear_velocity_m_s <= candidate.velocity <=
            self.limits.max_linear_velocity_m_s and
            math.isfinite(candidate.duration_s) and
            self.limits.min_translation_duration_s <= candidate.duration_s <=
            self.limits.max_translation_duration_s)

    def _read_menu_option(self) -> int:
        while True:
            answer = self.operator_input.read_text(
                "Select next action:\n"
                "1 - Repeat the same test\n"
                "2 - Repeat rotation with 2x duration\n"
                "3 - Repeat rotation with 1.5x velocity\n"
                "4 - Switch to translation test\n"
                "5 - Finish and write campaign summary\n"
                "Selection: ").strip()
            if answer.isdigit() and int(answer) in (1, 2, 3, 4, 5):
                return int(answer)
            self.operator_input.notify("Invalid menu selection. Enter a number from 1 to 5.")

    def _read_lower_rotation_velocity(self) -> Optional[float]:
        self.operator_input.notify(
            "The proposed angular velocity exceeds the configured limit "
            f"({self.limits.max_angular_velocity_rad_s:.9g} rad/s).")
        while True:
            answer = self.operator_input.read_text(
                "Enter a lower angular velocity in rad/s, or 'menu': ").strip()
            if answer.lower() == "menu":
                return None
            try:
                value = float(answer)
            except ValueError:
                self.operator_input.notify("Invalid numeric input. Please enter a number.")
                continue
            if not math.isfinite(value) or value <= 0.0:
                self.operator_input.notify("Angular velocity must be finite and positive.")
                continue
            if value > self.limits.max_angular_velocity_rad_s:
                self.operator_input.notify(
                    f"Angular velocity must not exceed "
                    f"{self.limits.max_angular_velocity_rad_s:.9g} rad/s.")
                continue
            return value

    def _translation_trial(self, previous: TrialSpec, sequence: int) -> TrialSpec:
        while True:
            direction = self.operator_input.read_text(
                "Translation direction [forward/backward]: ").strip().lower()
            if direction in ("forward", "backward"):
                break
            self.operator_input.notify("Direction must be 'forward' or 'backward'.")
        velocity = self._read_bounded_float(
            "Enter linear velocity (m/s): ",
            self.limits.min_linear_velocity_m_s,
            self.limits.max_linear_velocity_m_s,
            "linear velocity")
        duration = self._read_bounded_float(
            "Enter duration (s): ",
            self.limits.min_translation_duration_s,
            self.limits.max_translation_duration_s,
            "translation duration")
        return TrialSpec(
            trial_id=f"{previous.trial_id}-menu-{sequence:03d}-{direction}",
            movement_type="translation",
            velocity=velocity,
            duration_s=duration,
            direction=direction)

    def _read_bounded_float(
            self,
            prompt: str,
            minimum: float,
            maximum: float,
            label: str) -> float:
        while True:
            value = self.operator_input.read_float(prompt)
            if minimum <= value <= maximum:
                return value
            self.operator_input.notify(
                f"{label.capitalize()} must be within "
                f"[{minimum:.9g}, {maximum:.9g}].")

    @staticmethod
    def _rotation_trial(
            previous: TrialSpec,
            velocity: float,
            duration_s: float,
            sequence: int) -> TrialSpec:
        return TrialSpec(
            trial_id=f"{previous.trial_id}-menu-{sequence:03d}-{previous.direction}",
            movement_type="rotation",
            velocity=velocity,
            duration_s=duration_s,
            direction=previous.direction)

    def _confirm(self, candidate: TrialSpec) -> bool:
        self.display("Proposed command: " + describe_trial(candidate))
        while True:
            answer = self.operator_input.read_text(
                "Confirm this test before motion? [yes/no]: ").strip().lower()
            if answer in ("yes", "y"):
                return True
            if answer in ("no", "n"):
                self.operator_input.notify("Proposed test not confirmed; returning to menu.")
                return False
            self.operator_input.notify("Please answer yes or no.")


class TerminalLineReader:
    """Read and decode exactly one raw terminal line per prompt."""

    def __init__(self, stream, output, encoding: str = "utf-8"):
        self.stream = stream
        self.output = output
        self.encoding = encoding
        self._cancel_event = Event()

    def cancel(self) -> None:
        """Cancel a pending real-terminal read without leaving a worker behind."""
        self._cancel_event.set()

    def __call__(self, prompt_text: str) -> str:
        self._cancel_event.clear()
        self.output.write(prompt_text)
        self.output.flush()
        try:
            descriptor = self.stream.fileno()
        except (AttributeError, OSError, ValueError):
            descriptor = None
        if descriptor is None:
            raw_line = self.stream.readline()
        else:
            while not self._cancel_event.is_set():
                readable, _unused, _exceptional = select.select(
                    [descriptor], [], [], 0.05)
                if readable:
                    raw_line = self.stream.readline()
                    break
            else:
                raise KeyboardInterrupt("operator input cancelled")
        if raw_line == b"":
            raise EOFError("EOF while reading operator input")
        return raw_line.decode(self.encoding, errors="strict").rstrip("\r\n")


class ResponsiveOperatorInput:
    """
    Read operator input while allowing the caller to service callbacks.

    The terminal worker is deliberately daemonized because a blocking raw
    terminal read cannot be cancelled portably.  On main-thread interruption,
    the console entry point performs zero cleanup and exits the process.
    """

    def __init__(
            self,
            prompt: Callable[[str], str],
            poll: Callable[[], None],
            notify: Callable[[str], None],
            poll_interval_s: float = 0.05):
        if poll_interval_s <= 0.0:
            raise ValueError("poll_interval_s must be positive")
        self.prompt = prompt
        self.poll = poll
        self.notify = notify
        self.poll_interval_s = poll_interval_s
        self._active_thread: Optional[Thread] = None

    def cancel(self) -> None:
        """Cancel the active reader when its prompt supports cancellation."""
        cancel = getattr(self.prompt, "cancel", None)
        if cancel is not None:
            cancel()
        thread = self._active_thread
        if thread is not None:
            thread.join(timeout=max(0.1, self.poll_interval_s * 4.0))

    def read_text(self, prompt_text: str) -> str:
        """Read one line, retrying recoverable terminal decoding failures."""
        while True:
            try:
                return self._read_once(prompt_text)
            except UnicodeError as error:
                self.notify(
                    f"Input encoding error: {error}. Please enter the value again.")

    def read_float(self, prompt_text: str) -> float:
        """Read a finite floating-point value, retrying invalid text."""
        while True:
            text = self.read_text(prompt_text)
            try:
                value = float(text.strip())
            except (TypeError, ValueError):
                self.notify("Invalid numeric input. Please enter a number.")
                continue
            if not math.isfinite(value):
                self.notify("Invalid numeric input. Please enter a finite number.")
                continue
            return value

    def _read_once(self, prompt_text: str) -> str:
        result_queue = Queue(maxsize=1)

        def worker() -> None:
            try:
                result_queue.put((True, self.prompt(prompt_text)))
            except BaseException as error:
                result_queue.put((False, error))

        thread = Thread(target=worker, daemon=True)
        self._active_thread = thread
        thread.start()
        try:
            while True:
                try:
                    succeeded, value = result_queue.get(
                        timeout=self.poll_interval_s)
                except Empty:
                    self.poll()
                    continue
                if succeeded:
                    return value
                raise value
        except BaseException:
            self.cancel()
            raise
        finally:
            if not thread.is_alive():
                self._active_thread = None


def run_trial_until_accepted(
        spec: TrialSpec,
        execute_once: Callable[[TrialSpec], TrialResult],
        operator: OperatorInterface) -> TrialResult:
    """Repeat exactly the same trial until valid or skipped."""
    while True:
        result = execute_once(spec)
        verdict, reason, notes = operator.ask_validity()
        if verdict == "valid":
            return TrialResult(
                spec=result.spec,
                timestamp=result.timestamp,
                measurements=result.measurements,
                errors=result.errors,
                valid=True,
                skipped=False,
                rejection_reason=None,
                operator_notes=notes,
                evidence_dir=result.evidence_dir,
                initial_compass_heading_deg=result.initial_compass_heading_deg,
                final_compass_heading_deg=result.final_compass_heading_deg)
        if verdict == "skipped":
            return TrialResult(
                spec=result.spec,
                timestamp=result.timestamp,
                measurements=result.measurements,
                errors=result.errors,
                valid=False,
                skipped=True,
                rejection_reason=reason,
                operator_notes=notes,
                evidence_dir=result.evidence_dir,
                initial_compass_heading_deg=result.initial_compass_heading_deg,
                final_compass_heading_deg=result.final_compass_heading_deg)


def make_trial_result(
        spec: TrialSpec,
        measurements: TrialMeasurements,
        valid: Optional[bool] = None,
        skipped: bool = False,
        rejection_reason: Optional[str] = None,
        operator_notes: str = "",
        evidence_dir: Optional[str] = None,
        initial_compass_heading_deg: Optional[float] = None,
        final_compass_heading_deg: Optional[float] = None) -> TrialResult:
    """Construct a result and compute movement-appropriate errors."""
    return TrialResult(
        spec=spec,
        timestamp=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        measurements=measurements,
        errors=compute_pairwise_errors(comparison_values(spec, measurements)),
        valid=valid,
        skipped=skipped,
        rejection_reason=rejection_reason,
        operator_notes=operator_notes,
        evidence_dir=evidence_dir,
        initial_compass_heading_deg=initial_compass_heading_deg,
        final_compass_heading_deg=final_compass_heading_deg)
