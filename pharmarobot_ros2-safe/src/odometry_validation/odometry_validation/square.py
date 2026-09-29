# Copyright 2026 Medrobots Engineering
#
# Licensed under the Apache License, Version 2.0 (the "License");
"""Pure, fail-closed calculations for a supervised square trial."""

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import statistics
import time
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from odometry_validation.core import GeometryConfig
from odometry_validation.core import ImuSample
from odometry_validation.core import PRIMARY_IMU_SOURCE_TOPIC
from odometry_validation.core import TrialSpec
from odometry_validation.core import ValidationError
from odometry_validation.core import WheelTickSample


CLOSED_LOOP_ENCODER_WEIGHT = 0.50
CLOSED_LOOP_IMU_WEIGHT = 0.50
MAX_CLOSED_LOOP_DISAGREEMENT_RAD = 0.20
MAX_CLOSED_LOOP_HEADING_STEP_RAD = 0.20


@dataclass(frozen=True)
class SquareConfig:
    side_length_m: float = 2.0
    linear_velocity_m_s: float = 0.2
    angular_velocity_rad_s: float = 0.3
    direction: str = "ccw"
    settle_s: float = 3.0
    between_segment_pause_s: float = 1.0

    def __post_init__(self):
        for name, value in asdict(self).items():
            if name == "direction":
                continue
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.direction not in ("cw", "ccw"):
            raise ValueError("direction must be cw or ccw")

    def segments(self) -> Tuple[TrialSpec, ...]:
        translation_duration = self.side_length_m / self.linear_velocity_m_s
        turn_duration = (math.pi / 2.0) / self.angular_velocity_rad_s
        result: List[TrialSpec] = []
        for leg in range(1, 5):
            result.append(TrialSpec(
                f"square-leg-{leg}", "translation", self.linear_velocity_m_s,
                translation_duration, "forward"))
            result.append(TrialSpec(
                f"square-turn-{leg}", "rotation", self.angular_velocity_rad_s,
                turn_duration, self.direction))
        return tuple(result)


@dataclass(frozen=True)
class ClosedLoopYawConfig:
    """Conservative quarter-turn controller limits, independent of ROS."""

    max_angular_velocity_rad_s: float = 0.3
    min_correction_angular_velocity_rad_s: float = 0.08
    proportional_gain_s_inv: float = 1.0
    yaw_tolerance_rad: float = 0.02
    settle_hold_s: float = 0.25
    turn_timeout_s: float = 10.0
    max_disagreement_rad: float = 0.20
    disagreement_hold_s: float = 0.25
    max_heading_step_rad: float = 0.20

    def __post_init__(self):
        for name, value in asdict(self).items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if (self.min_correction_angular_velocity_rad_s >
                self.max_angular_velocity_rad_s):
            raise ValueError("minimum correction velocity exceeds maximum")
        if self.yaw_tolerance_rad >= math.pi / 2.0:
            raise ValueError("closed-loop yaw tolerance must be below pi/2")
        if self.max_disagreement_rad > MAX_CLOSED_LOOP_DISAGREEMENT_RAD:
            raise ValueError(
                "closed-loop disagreement threshold exceeds safety maximum")
        if self.max_heading_step_rad > MAX_CLOSED_LOOP_HEADING_STEP_RAD:
            raise ValueError(
                "closed-loop heading step threshold exceeds safety maximum")


@dataclass(frozen=True)
class ClosedLoopYawOutput:
    timestamp_s: float
    elapsed_s: float
    target_yaw_rad: float
    encoder_relative_yaw_rad: float
    imu_relative_yaw_rad: float
    fused_relative_yaw_rad: float
    heading_error_rad: float
    commanded_angular_velocity_rad_s: float
    encoder_imu_disagreement_rad: float
    within_tolerance: bool
    tolerance_hold_elapsed_s: float
    complete: bool


def fuse_closed_loop_yaw(
        encoder_delta_yaw_rad: float, imu_delta_yaw_rad: float) -> float:
    """Operator-specified control fusion; not a calibration-derived estimate."""
    if not all(math.isfinite(value) for value in (
            encoder_delta_yaw_rad, imu_delta_yaw_rad)):
        raise ValidationError("closed-loop heading estimates must be finite")
    return (
        CLOSED_LOOP_ENCODER_WEIGHT * encoder_delta_yaw_rad +
        CLOSED_LOOP_IMU_WEIGHT * imu_delta_yaw_rad)


def unwrap_angle(previous_unwrapped_rad: float, current_wrapped_rad: float) -> float:
    """Continue a wrapped angle through either side of +/-pi."""
    if not all(math.isfinite(value) for value in (
            previous_unwrapped_rad, current_wrapped_rad)):
        raise ValidationError("yaw values must be finite")
    previous_wrapped = math.atan2(
        math.sin(previous_unwrapped_rad), math.cos(previous_unwrapped_rad))
    delta = math.atan2(
        math.sin(current_wrapped_rad - previous_wrapped),
        math.cos(current_wrapped_rad - previous_wrapped))
    return previous_unwrapped_rad + delta


def encoder_relative_yaw(
        samples: Sequence[WheelTickSample], geometry: GeometryConfig,
        max_heading_step_rad: float, max_sample_gap_s: float,
        baseline_timestamp_s: Optional[float] = None) -> float:
    """Integrate delta wheel ticks into continuous relative encoder yaw."""
    if (not math.isfinite(max_heading_step_rad) or
            not math.isfinite(max_sample_gap_s) or
            max_heading_step_rad <= 0.0 or max_sample_gap_s <= 0.0):
        raise ValueError("heading step and sample gap must be finite and positive")
    meters_per_tick = (
        2.0 * math.pi * geometry.wheel_radius_m /
        geometry.encoder_ticks_per_revolution)
    yaw = 0.0
    previous_timestamp = baseline_timestamp_s
    for sample in samples:
        if not math.isfinite(sample.timestamp_s):
            raise ValidationError("encoder timestamp is not finite")
        if (previous_timestamp is not None and
                sample.timestamp_s <= previous_timestamp):
            raise ValidationError("encoder timestamp reset during closed-loop turn")
        if (previous_timestamp is not None and
                sample.timestamp_s - previous_timestamp > max_sample_gap_s):
            raise ValidationError("stale encoder sample gap during closed-loop turn")
        step = (
            (sample.right_ticks - sample.left_ticks) * meters_per_tick /
            geometry.track_width_m)
        if not math.isfinite(step) or abs(step) > max_heading_step_rad:
            raise ValidationError("unexpected encoder yaw reset/step")
        yaw += step
        previous_timestamp = sample.timestamp_s
    return yaw


def imu_relative_yaw(
        samples: Sequence[ImuSample], imu_bias_rad_s: float,
        max_heading_step_rad: float, max_sample_gap_s: float) -> float:
    """Trapezoid-integrate processed /imu/data into unwrapped relative yaw."""
    if (not math.isfinite(imu_bias_rad_s) or
            not math.isfinite(max_heading_step_rad) or
            not math.isfinite(max_sample_gap_s) or
            max_heading_step_rad <= 0.0 or max_sample_gap_s <= 0.0):
        raise ValueError("IMU bias, heading step, and gap must be finite")
    primary = tuple(
        sample for sample in samples
        if sample.source_topic == PRIMARY_IMU_SOURCE_TOPIC)
    if not primary:
        raise ValidationError("processed /imu/data unavailable for closed-loop turn")
    yaw = 0.0
    for previous, current in zip(primary, primary[1:]):
        if not all(math.isfinite(value) for value in (
                previous.timestamp_s, current.timestamp_s,
                previous.angular_velocity_z_rad_s,
                current.angular_velocity_z_rad_s)):
            raise ValidationError("processed /imu/data contains nonfinite values")
        dt = current.timestamp_s - previous.timestamp_s
        if dt <= 0.0:
            raise ValidationError("processed IMU timestamp reset during closed-loop turn")
        if dt > max_sample_gap_s:
            raise ValidationError("stale IMU sample gap during closed-loop turn")
        step = 0.5 * (
            previous.angular_velocity_z_rad_s +
            current.angular_velocity_z_rad_s - 2.0 * imu_bias_rad_s) * dt
        if not math.isfinite(step) or abs(step) > max_heading_step_rad:
            raise ValidationError("unexpected IMU yaw reset/step")
        yaw += step
    return yaw


class ClosedLoopYawController:
    """Bounded proportional yaw controller with settle and disagreement holds."""

    def __init__(
            self, target_yaw_rad: float, config: ClosedLoopYawConfig,
            start_timestamp_s: float):
        if (not math.isfinite(target_yaw_rad) or
                not math.isfinite(start_timestamp_s) or
                abs(target_yaw_rad) != math.pi / 2.0):
            raise ValueError("closed-loop target must be a signed quarter-turn")
        self.target_yaw_rad = target_yaw_rad
        self.config = config
        self.start_timestamp_s = start_timestamp_s
        self.last_timestamp_s = start_timestamp_s
        self.within_since_s: Optional[float] = None
        self.disagreement_since_s: Optional[float] = None

    def update(
            self, timestamp_s: float, encoder_yaw_rad: float,
            imu_yaw_rad: float,
            evidence_timestamp_s: Optional[float] = None) -> ClosedLoopYawOutput:
        if not all(math.isfinite(value) for value in (
                timestamp_s, encoder_yaw_rad, imu_yaw_rad)):
            raise ValidationError("closed-loop controller input is nonfinite")
        if (evidence_timestamp_s is not None and
                not math.isfinite(evidence_timestamp_s)):
            raise ValidationError("closed-loop evidence timestamp is nonfinite")
        if timestamp_s < self.last_timestamp_s:
            raise ValidationError("closed-loop controller clock moved backwards")
        elapsed_s = timestamp_s - self.start_timestamp_s
        if elapsed_s >= self.config.turn_timeout_s:
            raise ValidationError("closed-loop turn timed out")
        fused_yaw = fuse_closed_loop_yaw(encoder_yaw_rad, imu_yaw_rad)
        disagreement = abs(encoder_yaw_rad - imu_yaw_rad)
        if disagreement > self.config.max_disagreement_rad:
            if self.disagreement_since_s is None:
                self.disagreement_since_s = timestamp_s
            elif (timestamp_s - self.disagreement_since_s >=
                    self.config.disagreement_hold_s):
                raise ValidationError(
                    "sustained encoder/IMU yaw disagreement during closed-loop turn")
        else:
            self.disagreement_since_s = None
        error = self.target_yaw_rad - fused_yaw
        within = abs(error) <= self.config.yaw_tolerance_rad
        if within:
            if self.within_since_s is None:
                self.within_since_s = timestamp_s
            hold_elapsed = timestamp_s - self.within_since_s
            command = 0.0
            complete = (
                hold_elapsed + 1e-12 >= self.config.settle_hold_s)
        else:
            self.within_since_s = None
            hold_elapsed = 0.0
            magnitude = min(
                self.config.max_angular_velocity_rad_s,
                max(self.config.min_correction_angular_velocity_rad_s,
                    self.config.proportional_gain_s_inv * abs(error)))
            command = math.copysign(magnitude, error)
            complete = False
        self.last_timestamp_s = timestamp_s
        return ClosedLoopYawOutput(
            (timestamp_s if evidence_timestamp_s is None else
             evidence_timestamp_s), elapsed_s, self.target_yaw_rad,
            encoder_yaw_rad, imu_yaw_rad, fused_yaw, error, command,
            disagreement, within, hold_elapsed, complete)


def summarize_closed_loop_turn(
        rows: Sequence[Dict[str, object]]) -> Dict[str, object]:
    if not rows:
        raise ValidationError("closed-loop turn has no feedback evidence")
    final = rows[-1]
    target = float(final["target_yaw_rad"])
    encoder = float(final["encoder_relative_yaw_rad"])
    imu = float(final["imu_relative_yaw_rad"])
    fused = float(final["fused_relative_yaw_rad"])
    error = float(final["heading_error_rad"])
    direction_sign = math.copysign(1.0, target)
    signed_progress = direction_sign * fused
    overshoot = max(0.0, signed_progress - abs(target))
    undershoot = max(0.0, abs(target) - signed_progress)
    maximum_disagreement = max(
        float(row["encoder_imu_disagreement_rad"]) for row in rows)
    return {
        "target_yaw_rad": target,
        "target_yaw_deg": math.degrees(target),
        "final_encoder_yaw_rad": encoder,
        "final_encoder_yaw_deg": math.degrees(encoder),
        "final_imu_yaw_rad": imu,
        "final_imu_yaw_deg": math.degrees(imu),
        "final_fused_yaw_rad": fused,
        "final_fused_yaw_deg": math.degrees(fused),
        "final_fused_error_rad": error,
        "final_fused_error_deg": math.degrees(error),
        "overshoot_rad": overshoot,
        "overshoot_deg": math.degrees(overshoot),
        "undershoot_rad": undershoot,
        "undershoot_deg": math.degrees(undershoot),
        "turn_duration_s": final["elapsed_s"],
        "maximum_encoder_imu_disagreement_rad": maximum_disagreement,
        "maximum_encoder_imu_disagreement_deg": math.degrees(
            maximum_disagreement),
        "sample_count": len(rows),
    }


def service_closed_loop_wait(
        spin: Callable[[float], None], verify_guards: Callable[[], None],
        next_publication_s: float, deadline_s: float,
        monotonic: Callable[[], float] = time.monotonic) -> None:
    """Service callbacks and guards without waiting beyond the turn deadline."""
    wait_end_s = min(next_publication_s, deadline_s)
    while True:
        now_s = monotonic()
        if now_s >= wait_end_s:
            return
        spin(min(0.01, wait_end_s - now_s))
        verify_guards()


def square_emergency_stop_timeout_s(
        zero_confirmation_timeout_s: float,
        controlled_stationarity_timeout_s: float) -> float:
    """Give a reset cleanup window the same bounded settling budget as normal."""
    values = (zero_confirmation_timeout_s, controlled_stationarity_timeout_s)
    if any(not math.isfinite(value) or value <= 0.0 for value in values):
        raise ValueError("square stop timeouts must be finite and positive")
    return max(values)


@dataclass(frozen=True)
class HeadingFusion:
    available: bool
    encoder_weight: Optional[float]
    imu_weight: Optional[float]
    encoder_error_std_rad: Optional[float]
    imu_error_std_rad: Optional[float]
    trials: Tuple[str, ...]
    limitation: Optional[str]
    valid_non_teledex_rotation_trials: int = 0
    trusted_physical_reference_trials: int = 0
    encoder_command_error_mean_rad: Optional[float] = None
    encoder_command_error_std_rad: Optional[float] = None
    imu_command_error_mean_rad: Optional[float] = None
    imu_command_error_std_rad: Optional[float] = None
    error_reference_note: str = (
        "command residual includes actuation and estimator error; commanded angle "
        "is not independent physical truth and cannot justify fusion weights")
    command_residual_trials: Tuple[str, ...] = ()

    def fuse(self, encoder_yaw_rad: float, imu_yaw_rad: float) -> Optional[float]:
        if not self.available:
            return None
        return self.encoder_weight * encoder_yaw_rad + self.imu_weight * imu_yaw_rad


def _sample_stats(values: Sequence[float]) -> Tuple[Optional[float], Optional[float]]:
    if not values:
        return None, None
    return statistics.mean(values), statistics.stdev(values) if len(values) >= 2 else None


def _campaign_is_dry_run(report_path: Path, evidence_root: Path) -> bool:
    for parent in report_path.parents:
        if parent == evidence_root.parent:
            break
        metadata = parent / "metadata.json"
        if not metadata.exists():
            continue
        try:
            payload = json.loads(metadata.read_text(encoding="utf-8"))
        except (OSError, ValueError, TypeError):
            continue
        if "dry_run" in payload:
            return bool(payload["dry_run"])
    return False


def derive_heading_fusion(evidence_root: Path) -> HeadingFusion:
    """Audit valid non-TeleDex rotation evidence without inventing weights."""
    root = Path(evidence_root)
    encoder_residuals: List[float] = []
    imu_residuals: List[float] = []
    trusted: List[str] = []
    audited: List[str] = []
    valid_count = 0
    for path in sorted(root.glob("**/report.json")):
        try:
            report = json.loads(path.read_text(encoding="utf-8"))
            operator = report.get("operator", {})
            if (report.get("trial", {}).get("movement_type") != "rotation" or
                    not operator.get("valid") or operator.get("skipped") or
                    report.get("teledex_reference") is not None or
                    report.get("teledex") is not None or
                    _campaign_is_dry_run(path, root)):
                continue
            commanded = float(report["theoretical"]["expected_angle_rad"])
            encoder = float(report["encoder"]["angle_rad"])
            imu = float(report["imu"]["total_physical_motion_angle_rad"])
            if not all(math.isfinite(value) for value in (commanded, encoder, imu)):
                continue
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            continue
        valid_count += 1
        audited.append(str(path))
        encoder_residuals.append(encoder - commanded)
        imu_residuals.append(imu - commanded)
        laser = report.get("laser_reference") or {}
        physical = report.get("physical_reference", {}).get("angle_rad")
        if laser.get("quality_passed") and physical is not None:
            trusted.append(str(path))
    encoder_mean, encoder_std = _sample_stats(encoder_residuals)
    imu_mean, imu_std = _sample_stats(imu_residuals)
    return HeadingFusion(
        available=False, encoder_weight=None, imu_weight=None,
        encoder_error_std_rad=None, imu_error_std_rad=None,
        trials=tuple(trusted),
        limitation=(
            "no reviewed homogeneous non-TeleDex independent heading-reference "
            "cohort; encoder pose and processed /imu/data heading are reported "
            "separately and fused pose is unavailable"),
        valid_non_teledex_rotation_trials=valid_count,
        trusted_physical_reference_trials=len(trusted),
        encoder_command_error_mean_rad=encoder_mean,
        encoder_command_error_std_rad=encoder_std,
        imu_command_error_mean_rad=imu_mean,
        imu_command_error_std_rad=imu_std,
        command_residual_trials=tuple(audited))


def normalize_node_name(name: str) -> str:
    return "/" + name.strip("/")


def graph_contract_missing(
        nodes: Sequence[str], test_subscriber_nodes: Sequence[str],
        safe_publisher_nodes: Sequence[str], safe_subscriber_nodes: Sequence[str]
        ) -> Tuple[str, ...]:
    """Return missing owned endpoints for the deployed safe command path."""
    node_names = {normalize_node_name(name) for name in nodes}
    test_nodes = {normalize_node_name(name) for name in test_subscriber_nodes}
    safe_publishers = {normalize_node_name(name) for name in safe_publisher_nodes}
    safe_subscribers = {normalize_node_name(name) for name in safe_subscriber_nodes}
    missing = []
    if "/command_arbiter" not in node_names:
        missing.append("node:/command_arbiter")
    if "/command_arbiter" not in test_nodes:
        missing.append("arbiter-subscriber:/cmd_vel/test")
    if "/command_arbiter" not in safe_publishers:
        missing.append("arbiter-publisher:/cmd_vel/safe")
    if "/roboteq_ros2_driver" not in safe_subscribers:
        missing.append("roboteq-subscriber:/cmd_vel/safe")
    return tuple(missing)


def wait_for_graph_contract(
        observe_missing: Callable[[], Sequence[str]], spin: Callable[[float], None],
        timeout_s: float, monotonic: Callable[[], float] = time.monotonic) -> None:
    """Retry DDS discovery to one explicit deadline, then fail closed."""
    if not math.isfinite(timeout_s) or timeout_s <= 0.0:
        raise ValueError("graph discovery timeout must be finite and positive")
    deadline = monotonic() + timeout_s
    while True:
        missing = tuple(observe_missing())
        if not missing:
            return
        remaining = deadline - monotonic()
        if remaining <= 0.0:
            raise ValidationError(
                "square graph discovery timed out: " + ", ".join(missing))
        spin(min(0.05, remaining))


def safe_zero_confirmed(
        graph_missing: Sequence[str], base_safe_zero: bool,
        zero_publication_timestamp_s: Optional[float],
        safe_callback_timestamp_s: Optional[float]) -> bool:
    """Accept safe zero only from a callback after this cleanup publication."""
    return bool(
        not graph_missing and base_safe_zero and
        zero_publication_timestamp_s is not None and
        safe_callback_timestamp_s is not None and
        safe_callback_timestamp_s >= zero_publication_timestamp_s)


def stop_with_graph_discovery(
        graph_missing: Callable[[], Sequence[str]],
        publish_zero: Callable[[], None], spin: Callable[[float], None],
        verified_stop: Callable[[], object], discovery_timeout_s: float,
        rate_hz: float, monotonic: Callable[[], float] = time.monotonic):
    """Continuously command zero while DDS endpoints become discoverable."""
    if (not math.isfinite(discovery_timeout_s) or
            discovery_timeout_s <= 0.0 or
            not math.isfinite(rate_hz) or rate_hz <= 0.0):
        raise ValueError("cleanup discovery timeout and rate must be positive")
    deadline = monotonic() + discovery_timeout_s
    period_s = 1.0 / rate_hz
    last_graph_error: Optional[Exception] = None
    while True:
        publish_zero()
        try:
            missing = tuple(graph_missing())
            last_graph_error = None
        except Exception as error:
            last_graph_error = error
            missing = (f"graph-query-error:{type(error).__name__}",)
        if not missing:
            return verified_stop()
        remaining = deadline - monotonic()
        if remaining <= 0.0:
            failure = ValidationError(
                "cleanup graph discovery timed out: " + ", ".join(missing))
            if last_graph_error is not None:
                raise failure from last_graph_error
            raise failure
        spin(min(period_s, remaining))


def run_square_sequence(
        segments: Sequence[TrialSpec],
        execute_segment: Callable[[int, TrialSpec], object],
        pause: Callable[[], None]) -> Tuple[object, ...]:
    """Execute in order; an exception aborts all remaining segments."""
    completed = []
    for index, segment in enumerate(segments, start=1):
        completed.append(execute_segment(index, segment))
        if index < len(segments):
            pause()
    return tuple(completed)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def square_metrics(
        config: SquareConfig, segment_measurements: Sequence[Dict[str, float]],
        fusion: HeadingFusion) -> Dict[str, object]:
    """Accumulate eight completed segments into closure metrics."""
    if len(segment_measurements) != 8:
        raise ValidationError("square trial requires exactly eight completed segments")
    encoder_x = encoder_y = encoder_yaw = imu_yaw = 0.0
    fused_x = fused_y = fused_yaw = 0.0
    per_leg, rotations = [], []
    for index, measurement in enumerate(segment_measurements):
        if index % 2 == 0:
            distance = float(measurement["encoder_distance_m"])
            encoder_x += distance * math.cos(encoder_yaw)
            encoder_y += distance * math.sin(encoder_yaw)
            if fusion.available:
                fused_x += distance * math.cos(fused_yaw)
                fused_y += distance * math.sin(fused_yaw)
            per_leg.append(distance)
        else:
            encoder_turn = float(measurement["encoder_angle_rad"])
            imu_turn = float(measurement["imu_angle_rad"])
            encoder_yaw += encoder_turn
            imu_yaw += imu_turn
            if fusion.available:
                fused_yaw = fusion.fuse(encoder_yaw, imu_yaw)
            rotations.append({
                "turn_index": len(rotations) + 1,
                "expected_rad": (
                    math.pi / 2.0 if config.direction == "ccw" else
                    -math.pi / 2.0),
                "encoder_rad": encoder_turn, "imu_rad": imu_turn})
    closure = math.hypot(encoder_x, encoder_y)
    return {
        "configuration": asdict(config), "heading_fusion": asdict(fusion),
        "total_commanded_path_m": 4.0 * config.side_length_m,
        "total_encoder_path_m": sum(abs(value) for value in per_leg),
        "per_leg_encoder_distance_m": per_leg, "turn_estimates": rotations,
        "encoder_only_final_pose": {
            "x_m": encoder_x, "y_m": encoder_y,
            "yaw_rad": encoder_yaw, "yaw_error_rad": _wrap(encoder_yaw)},
        "imu_heading_final_yaw_rad": imu_yaw,
        "imu_final_yaw_error_rad": _wrap(imu_yaw),
        "fused_final_pose": None if not fusion.available else {
            "x_m": fused_x, "y_m": fused_y, "yaw_rad": fused_yaw,
            "yaw_error_rad": _wrap(fused_yaw)},
        "final_delta_x_m": encoder_x, "final_delta_y_m": encoder_y,
        "position_closure_error_m": closure,
        "final_encoder_yaw_error_rad": _wrap(encoder_yaw),
        "closure_percent_of_commanded_path": (
            100.0 * closure / (4.0 * config.side_length_m)),
    }


def reconstruct_trajectories(
        wheel_samples: Sequence[WheelTickSample], imu_samples: Sequence[ImuSample],
        geometry: GeometryConfig, imu_bias_rad_s: float,
        wheel_tick_semantics: str = "delta",
        start_timestamp_s: Optional[float] = None,
        end_timestamp_s: Optional[float] = None) -> Dict[str, object]:
    """Reconstruct continuous encoder pose and processed-IMU yaw trajectories."""
    if wheel_tick_semantics != "delta":
        raise ValidationError("square trajectory currently requires delta wheel ticks")
    meters_per_tick = (
        2.0 * math.pi * geometry.wheel_radius_m /
        geometry.encoder_ticks_per_revolution)
    x_m = y_m = yaw_rad = path_m = 0.0
    encoder_rows = []
    for sample in sorted(wheel_samples, key=lambda item: item.timestamp_s):
        left_m = sample.left_ticks * meters_per_tick
        right_m = sample.right_ticks * meters_per_tick
        distance_m = 0.5 * (left_m + right_m)
        delta_yaw = (right_m - left_m) / geometry.track_width_m
        x_m += distance_m * math.cos(yaw_rad + 0.5 * delta_yaw)
        y_m += distance_m * math.sin(yaw_rad + 0.5 * delta_yaw)
        yaw_rad += delta_yaw
        path_m += abs(distance_m)
        encoder_rows.append({
            "timestamp_s": sample.timestamp_s, "x_m": x_m, "y_m": y_m,
            "yaw_rad": yaw_rad, "path_length_m": path_m,
            "left_ticks": sample.left_ticks, "right_ticks": sample.right_ticks,
            "phase": sample.phase})
    primary = sorted(
        (sample for sample in imu_samples
         if sample.source_topic == PRIMARY_IMU_SOURCE_TOPIC),
        key=lambda item: item.timestamp_s)
    if start_timestamp_s is not None or end_timestamp_s is not None:
        if (start_timestamp_s is None or end_timestamp_s is None or
                not all(math.isfinite(value) for value in (
                    start_timestamp_s, end_timestamp_s)) or
                start_timestamp_s >= end_timestamp_s):
            raise ValidationError("trajectory bounds must be finite and ordered")
        primary = _clip_imu_to_bounds(
            primary, start_timestamp_s, end_timestamp_s)
    imu_rows = []
    imu_yaw = 0.0
    if primary:
        imu_rows.append({
            "timestamp_s": primary[0].timestamp_s, "yaw_rad": 0.0,
            "corrected_angular_velocity_z_rad_s": (
                primary[0].angular_velocity_z_rad_s - imu_bias_rad_s),
            "phase": primary[0].phase})
    for previous, current in zip(primary, primary[1:]):
        dt = current.timestamp_s - previous.timestamp_s
        if dt < 0.0 or not math.isfinite(dt):
            raise ValidationError("primary IMU timestamps are not monotonic")
        previous_rate = previous.angular_velocity_z_rad_s - imu_bias_rad_s
        current_rate = current.angular_velocity_z_rad_s - imu_bias_rad_s
        imu_yaw += 0.5 * (previous_rate + current_rate) * dt
        imu_rows.append({
            "timestamp_s": current.timestamp_s, "yaw_rad": imu_yaw,
            "corrected_angular_velocity_z_rad_s": current_rate,
            "phase": current.phase})
    return {
        "encoder": encoder_rows, "imu": imu_rows,
        "encoder_final_pose": None if not encoder_rows else encoder_rows[-1],
        "imu_final_yaw_rad": None if not imu_rows else imu_rows[-1]["yaw_rad"]}


def timestamp_coverage(
        timestamps: Sequence[float], start_s: float, end_s: float,
        gap_threshold_s: Optional[float] = None) -> Dict[str, object]:
    """Describe one callback stream against square bounds in ROS time."""
    if (not math.isfinite(start_s) or not math.isfinite(end_s) or
            start_s >= end_s):
        raise ValidationError("trajectory bounds must be finite and ordered")
    values = tuple(float(value) for value in timestamps)
    invalid_count = sum(not math.isfinite(value) for value in values)
    finite = tuple(value for value in values if math.isfinite(value))
    out_of_order_count = sum(
        current < previous for previous, current in zip(finite, finite[1:]))
    ordered = tuple(sorted(finite))
    gaps = tuple(
        current - previous for previous, current in zip(ordered, ordered[1:]))
    first = None if not ordered else ordered[0]
    last = None if not ordered else ordered[-1]
    overlaps = bool(
        first is not None and last is not None and
        first <= end_s and last >= start_s)
    return {
        "time_basis": "ROS time seconds from message header or node clock fallback",
        "trajectory_start_timestamp_s": start_s,
        "trajectory_end_timestamp_s": end_s,
        "sample_count": len(values),
        "finite_sample_count": len(finite),
        "invalid_timestamp_count": invalid_count,
        "out_of_order_timestamp_count": out_of_order_count,
        "first_timestamp_s": first,
        "last_timestamp_s": last,
        "coverage_before_start_s": (
            None if first is None else start_s - first),
        "coverage_after_end_s": (
            None if last is None else last - end_s),
        "overlaps_trajectory": overlaps,
        "max_sample_gap_s": None if not gaps else max(gaps),
        "gap_threshold_s": gap_threshold_s,
        "gaps_over_threshold_count": (
            None if gap_threshold_s is None else
            sum(gap > gap_threshold_s for gap in gaps)),
    }


def imu_coverage(
        samples: Sequence[ImuSample], start_s: float, end_s: float,
        gap_threshold_s: Optional[float] = None) -> Dict[str, object]:
    primary = tuple(
        sample for sample in samples
        if sample.source_topic == PRIMARY_IMU_SOURCE_TOPIC)
    coverage = timestamp_coverage(
        tuple(sample.timestamp_s for sample in primary), start_s, end_s,
        gap_threshold_s)
    coverage["source_topic"] = PRIMARY_IMU_SOURCE_TOPIC
    coverage["complete"] = bool(
        coverage["invalid_timestamp_count"] == 0 and
        coverage["out_of_order_timestamp_count"] == 0 and
        (coverage["gaps_over_threshold_count"] in (None, 0)) and
        coverage["coverage_before_start_s"] is not None and
        coverage["coverage_before_start_s"] >= 0.0 and
        coverage["coverage_after_end_s"] is not None and
        coverage["coverage_after_end_s"] >= 0.0)
    return coverage


def require_complete_imu_coverage(coverage: Dict[str, object]) -> None:
    """Keep exact bracketing fail-closed and explain the failed boundary."""
    if coverage.get("invalid_timestamp_count"):
        raise ValidationError("processed /imu/data has invalid timestamps")
    if coverage.get("out_of_order_timestamp_count"):
        raise ValidationError("processed /imu/data timestamp basis is not monotonic")
    if coverage.get("gaps_over_threshold_count"):
        raise ValidationError(
            "processed /imu/data has callback/sample gaps exceeding "
            f"{coverage.get('gap_threshold_s')} s; max gap is "
            f"{coverage.get('max_sample_gap_s')} s")
    if not coverage.get("overlaps_trajectory"):
        raise ValidationError(
            "processed /imu/data timestamp basis does not overlap square bounds")
    before = coverage.get("coverage_before_start_s")
    after = coverage.get("coverage_after_end_s")
    if before is None or before < 0.0:
        missing = None if before is None else -before
        raise ValidationError(
            f"processed /imu/data is missing square start coverage by {missing} s")
    if after is None or after < 0.0:
        missing = None if after is None else -after
        raise ValidationError(
            f"processed /imu/data is missing square end coverage by {missing} s")


def wait_for_imu_end_coverage(
        observe_samples: Callable[[], Sequence[ImuSample]],
        spin: Callable[[float], None], end_s: float, timeout_s: float,
        poll_interval_s: float,
        monotonic: Callable[[], float] = time.monotonic) -> bool:
    """Boundedly drain callbacks until a primary IMU stamp brackets the end."""
    if (not math.isfinite(end_s) or not math.isfinite(timeout_s) or
            timeout_s <= 0.0 or not math.isfinite(poll_interval_s) or
            poll_interval_s <= 0.0):
        raise ValueError("IMU end coverage wait parameters must be finite and positive")
    deadline = monotonic() + timeout_s
    while True:
        primary_timestamps = tuple(
            sample.timestamp_s for sample in observe_samples()
            if (sample.source_topic == PRIMARY_IMU_SOURCE_TOPIC and
                math.isfinite(sample.timestamp_s)))
        if primary_timestamps and max(primary_timestamps) >= end_s:
            return True
        remaining = deadline - monotonic()
        if remaining <= 0.0:
            return False
        spin(min(poll_interval_s, remaining))


def _clip_imu_to_bounds(
        samples: Sequence[ImuSample], start_s: float,
        end_s: float) -> List[ImuSample]:
    """Interpolate bracketing IMU samples so integration covers exact bounds."""
    if len(samples) < 2:
        return []

    def at(timestamp_s: float) -> Optional[ImuSample]:
        exact = next(
            (sample for sample in samples
             if sample.timestamp_s == timestamp_s), None)
        if exact is not None:
            return exact
        for left, right in zip(samples, samples[1:]):
            if left.timestamp_s < timestamp_s < right.timestamp_s:
                span = right.timestamp_s - left.timestamp_s
                if span <= 0.0:
                    raise ValidationError("primary IMU timestamps are not monotonic")
                fraction = (timestamp_s - left.timestamp_s) / span
                rate = (
                    left.angular_velocity_z_rad_s + fraction *
                    (right.angular_velocity_z_rad_s -
                     left.angular_velocity_z_rad_s))
                return ImuSample(
                    timestamp_s, rate, "interpolated_trial_boundary",
                    PRIMARY_IMU_SOURCE_TOPIC)
        return None

    start = at(start_s)
    end = at(end_s)
    if start is None or end is None:
        raise ValidationError(
            "processed /imu/data does not bracket square trajectory bounds")
    interior = [
        sample for sample in samples
        if start_s < sample.timestamp_s < end_s]
    return [start, *interior, end]


def apply_continuous_final_metrics(
        report: Dict[str, object], trajectories: Dict[str, object]
        ) -> Dict[str, object]:
    """Make continuous samples the one canonical source for final metrics."""
    pose = trajectories.get("encoder_final_pose")
    imu_yaw = trajectories.get("imu_final_yaw_rad")
    if pose is None:
        raise ValidationError("continuous encoder trajectory is unavailable")
    if imu_yaw is None:
        raise ValidationError("continuous processed IMU trajectory is unavailable")
    updated = dict(report)
    updated["segment_aggregate_encoder_final_pose"] = report.get(
        "encoder_only_final_pose")
    updated["segment_aggregate_imu_final_yaw_rad"] = report.get(
        "imu_heading_final_yaw_rad")
    closure = math.hypot(pose["x_m"], pose["y_m"])
    encoder_error = _wrap(pose["yaw_rad"])
    imu_error = _wrap(imu_yaw)
    updated.update({
        "encoder_only_final_pose": {
            "x_m": pose["x_m"], "y_m": pose["y_m"],
            "yaw_rad": pose["yaw_rad"], "yaw_error_rad": encoder_error},
        "imu_heading_final_yaw_rad": imu_yaw,
        "final_delta_x_m": pose["x_m"],
        "final_delta_y_m": pose["y_m"],
        "position_closure_error_m": closure,
        "final_encoder_yaw_error_rad": encoder_error,
        "imu_final_yaw_error_rad": imu_error,
        "total_encoder_path_m": pose["path_length_m"],
        "closure_percent_of_commanded_path": (
            100.0 * closure / updated["total_commanded_path_m"]),
    })
    return updated
