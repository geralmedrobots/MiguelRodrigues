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

"""Fixed odometry calibration campaign bookkeeping and analysis helpers."""

from dataclasses import asdict
from dataclasses import dataclass
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from odometry_validation.core import TrialResult
from odometry_validation.core import TrialSpec
from odometry_validation.core import percentage_error
from odometry_validation.core import translation_heading_comparison


CAMPAIGN_REPETITIONS_PER_CONDITION = 10
CALIBRATION_REPETITIONS_PER_CONDITION = 5
CAMPAIGN_ROTATION_VELOCITIES_RAD_S = (0.20, 0.30, 0.40, 0.50)
CAMPAIGN_TRANSLATION_VELOCITIES_M_S = (0.20, 0.30, 0.40, 0.50)
CAMPAIGN_DURATIONS_S = (2.0, 3.0, 4.0)
CALIBRATION_DURATIONS_S = (3.0, 4.0)
TRANSLATION_HEADING_LIMIT_DEG = 5.0

CAMPAIGN_MODES = {
    "1": "baseline",
    "2": "calibration",
    "3": "validation",
    "4": "covariance",
}

CAMPAIGN_MODE_LABELS = {
    "baseline": "CHARACTERIZATION / BASELINE",
    "calibration": "CALIBRATION",
    "validation": "VALIDATION",
    "covariance": "COVARIANCE CHARACTERIZATION",
}


@dataclass(frozen=True)
class CampaignCondition:
    """One fixed campaign condition with a valid-repetition target."""

    condition_id: str
    condition_number: int
    section: str
    repetition_target: int
    spec: TrialSpec


def fixed_campaign_conditions(
        repetitions_per_condition: int = CAMPAIGN_REPETITIONS_PER_CONDITION,
        mode: str = "baseline"
        ) -> Tuple[CampaignCondition, ...]:
    """Return the fixed matrix, selecting the reduced matrix for calibration."""
    if mode not in CAMPAIGN_MODE_LABELS:
        raise ValueError(f"unsupported campaign mode: {mode}")
    if mode == "calibration":
        repetitions_per_condition = CALIBRATION_REPETITIONS_PER_CONDITION
        durations = CALIBRATION_DURATIONS_S
    else:
        durations = CAMPAIGN_DURATIONS_S
    conditions: List[CampaignCondition] = []

    def add(section: str, movement_type: str, direction: str,
            velocities: Iterable[float]) -> None:
        for duration in durations:
            for velocity in velocities:
                number = len(conditions) + 1
                prefix = "rot" if movement_type == "rotation" else "trans"
                condition_id = f"{prefix}-{number:02d}-{direction}-{velocity:.2f}-{duration:.0f}s"
                conditions.append(CampaignCondition(
                    condition_id=condition_id,
                    condition_number=number,
                    section=section,
                    repetition_target=repetitions_per_condition,
                    spec=TrialSpec(
                        trial_id=condition_id,
                        movement_type=movement_type,
                        velocity=float(velocity),
                        duration_s=float(duration),
                        direction=direction)))

    add("CW", "rotation", "cw", CAMPAIGN_ROTATION_VELOCITIES_RAD_S)
    add("CCW", "rotation", "ccw", CAMPAIGN_ROTATION_VELOCITIES_RAD_S)
    add("Forward", "translation", "forward", CAMPAIGN_TRANSLATION_VELOCITIES_M_S)
    add("Backward", "translation", "backward", CAMPAIGN_TRANSLATION_VELOCITIES_M_S)
    return tuple(conditions)


def campaign_matrix_identity(
        conditions: Sequence[CampaignCondition]) -> str:
    """Return a stable identity for the exact checkpoint matrix."""
    payload = [asdict(condition) for condition in conditions]
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def planned_valid_repetition_count(
        conditions: Sequence[CampaignCondition]) -> int:
    return sum(condition.repetition_target for condition in conditions)


def signed_compass_delta_deg(initial_heading_deg: float, final_heading_deg: float) -> float:
    """Return ROS-positive yaw delta in degrees from clockwise-positive compass headings."""
    return math.degrees(math.radians(-(
        (float(final_heading_deg) - float(initial_heading_deg) + 180.0) %
        360.0 - 180.0)))


def translation_reference_distance_m(
        direction: str, initial_wall_distance_m: float,
        final_wall_distance_m: float) -> float:
    """Return signed wall-distance travel in robot-forward coordinates."""
    if direction == "forward":
        return float(initial_wall_distance_m) - float(final_wall_distance_m)
    if direction == "backward":
        return float(final_wall_distance_m) - float(initial_wall_distance_m)
    raise ValueError("translation direction must be forward or backward")


def heading_deviation_auto_invalid(
        final_heading_deviation_deg: float,
        limit_deg: float = TRANSLATION_HEADING_LIMIT_DEG) -> bool:
    return abs(float(final_heading_deviation_deg)) > limit_deg


def finite_stats(values: Sequence[float]) -> Dict[str, Optional[float]]:
    """Return required scalar statistics for one residual vector."""
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {
            "count": 0, "mean_signed_error": None, "mean_absolute_error": None,
            "median_error": None, "stddev": None, "rmse": None,
            "minimum": None, "maximum": None, "ci95_low": None, "ci95_high": None,
        }
    mean = statistics.mean(finite)
    stddev = statistics.stdev(finite) if len(finite) > 1 else 0.0
    ci_half_width = (
        None if len(finite) < 2 else 1.96 * stddev / math.sqrt(len(finite)))
    return {
        "count": len(finite),
        "mean_signed_error": mean,
        "mean_absolute_error": statistics.mean(abs(value) for value in finite),
        "median_error": statistics.median(finite),
        "stddev": stddev,
        "rmse": math.sqrt(statistics.mean(value * value for value in finite)),
        "minimum": min(finite),
        "maximum": max(finite),
        "ci95_low": None if ci_half_width is None else mean - ci_half_width,
        "ci95_high": None if ci_half_width is None else mean + ci_half_width,
    }


def valid_counts_by_condition(
        attempts: Sequence[Dict[str, object]]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for attempt in attempts:
        if attempt.get("valid") is True and not attempt.get("skipped"):
            condition_id = str(attempt["condition_id"])
            counts[condition_id] = counts.get(condition_id, 0) + 1
    return counts


def invalid_counts_by_condition(
        attempts: Sequence[Dict[str, object]]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for attempt in attempts:
        if attempt.get("valid") is True and not attempt.get("skipped"):
            continue
        condition_id = str(attempt["condition_id"])
        counts[condition_id] = counts.get(condition_id, 0) + 1
    return counts


def next_pending(
        conditions: Sequence[CampaignCondition],
        attempts: Sequence[Dict[str, object]]
        ) -> Optional[Tuple[CampaignCondition, int]]:
    """Return the next condition and 1-based valid repetition number."""
    counts = valid_counts_by_condition(attempts)
    for condition in conditions:
        completed = counts.get(condition.condition_id, 0)
        if completed < condition.repetition_target:
            return condition, completed + 1
    return None


def campaign_progress(
        conditions: Sequence[CampaignCondition],
        attempts: Sequence[Dict[str, object]]) -> Dict[str, object]:
    pending = next_pending(conditions, attempts)
    valid_total = sum(valid_counts_by_condition(attempts).values())
    invalid_total = sum(invalid_counts_by_condition(attempts).values())
    return {
        "planned_valid_repetitions": planned_valid_repetition_count(conditions),
        "completed_valid_repetitions": valid_total,
        "invalid_or_retried_attempts": invalid_total,
        "complete": pending is None,
        "next_pending_condition_id": None if pending is None else pending[0].condition_id,
        "next_pending_condition_number": None if pending is None else pending[0].condition_number,
        "next_pending_valid_repetition": None if pending is None else pending[1],
    }


def attempt_record(
        attempt_id: str,
        condition: CampaignCondition,
        repetition_number: int,
        result: TrialResult,
        samples: Optional[object] = None,
        wheel_radius_m: Optional[float] = None) -> Dict[str, object]:
    """Flatten one TrialResult into a campaign-level machine-readable record."""
    m = result.measurements
    spec = result.spec
    reference = m.physical_measurement
    rotation = spec.movement_type == "rotation"
    record = {
        "attempt_id": attempt_id,
        "condition_id": condition.condition_id,
        "condition_number": condition.condition_number,
        "section": condition.section,
        "valid_repetition_number": repetition_number,
        "valid": result.valid,
        "wheel_radius_m": wheel_radius_m,
        "skipped": result.skipped,
        "invalid_reason": result.rejection_reason,
        "evidence_dir": result.evidence_dir,
        "direction": spec.direction,
        "movement_type": spec.movement_type,
        "velocity": spec.velocity,
        "duration_s": spec.duration_s,
        "commanded_distance_m": m.commanded_distance_m,
        "commanded_angle_rad": m.commanded_angle_rad,
        "commanded_angle_deg": math.degrees(m.commanded_angle_rad),
        "encoder_distance_m": m.encoder_distance_m,
        "encoder_angle_rad": m.encoder_angle_rad,
        "encoder_angle_deg": (
            None if m.encoder_angle_rad is None else math.degrees(m.encoder_angle_rad)),
        "imu_angle_rad": m.imu_angle_rad,
        "imu_angle_deg": (
            None if m.imu_angle_rad is None else math.degrees(m.imu_angle_rad)),
        "manual_reference": result.manual_reference,
        "reference_value": reference,
        "command_start_timestamp_s": m.command_start_timestamp_s,
        "command_end_timestamp_s": m.command_end_timestamp_s,
        "stationary_confirmation_timestamp_s": (
            m.stationary_confirmation_timestamp_s),
        "source_topics": {
            "imu": m.imu_source_topic,
            "commands": ["/cmd_vel/test", "/cmd_vel/safe"],
            "wheel_ticks": "/wheel_ticks",
            "odometry": "/odom",
            "diagnostics": "/diagnostics",
        },
        "time_bases": {
            "manual_reference": "operator-entered after prompts",
            "command_timestamps": "node receive/command monotonic seconds",
            "sensor_samples": "ROS message stamps where available",
        },
        "stationarity_record_count": (
            0 if samples is None else len(getattr(samples, "stationarity", ()))),
        "diagnostic_sample_count": (
            0 if samples is None else len(getattr(samples, "diagnostics", ()))),
        "ignored_diagnostic_sample_count": (
            0 if samples is None else len(
                getattr(samples, "ignored_diagnostics", ()))),
        "encoder_imu_disagreement_rad": (
            None if m.encoder_angle_rad is None or m.imu_angle_rad is None else
            m.encoder_angle_rad - m.imu_angle_rad),
    }
    if rotation:
        record["manual_physical_reference_rotation_rad"] = reference
        record["manual_physical_reference_rotation_deg"] = (
            None if reference is None else math.degrees(reference))
        record["command_expected_rotation_rad"] = m.commanded_angle_rad
        record["compass_measured_rotation_rad"] = reference
        record["encoder_rotation_rad"] = m.encoder_angle_rad
        record["imu_rotation_rad"] = m.imu_angle_rad
        for name, value in (
                ("command", m.commanded_angle_rad),
                ("encoder", m.encoder_angle_rad),
                ("imu", m.imu_angle_rad)):
            record[f"{name}_error_rad"] = (
                None if value is None or reference is None else value - reference)
            record[f"{name}_error_deg"] = (
                None if record[f"{name}_error_rad"] is None else
                math.degrees(record[f"{name}_error_rad"]))
            record[f"{name}_absolute_error_rad"] = (
                None if record[f"{name}_error_rad"] is None else
                abs(record[f"{name}_error_rad"]))
            record[f"{name}_percentage_error"] = percentage_error(value, reference)
            real_error = (
                None if value is None or reference is None else value - reference)
            record[f"{name}_error_vs_manual_reference_rad"] = real_error
            record[f"{name}_error_vs_manual_reference_deg"] = (
                None if real_error is None else math.degrees(real_error))
            record[f"{name}_absolute_error_vs_manual_reference_rad"] = (
                None if real_error is None else abs(real_error))
            record[f"{name}_absolute_error_vs_manual_reference_deg"] = (
                None if real_error is None else abs(math.degrees(real_error)))
            record[f"{name}_percent_error_vs_manual_reference"] = (
                None if real_error is None or abs(reference) <= 1e-12 else
                abs(real_error) / abs(reference) * 100.0)
    else:
        record["manual_physical_reference_distance_m"] = reference
        reference_magnitude = None if reference is None else abs(reference)
        record["raw_manual_physical_reference_m"] = reference
        record["reference_distance_magnitude_m"] = reference_magnitude
        for name, value in (
                ("command", m.commanded_distance_m),
                ("encoder", m.encoder_distance_m),
                ("odometry", m.odometry_distance_m)):
            estimator_magnitude = None if value is None else abs(value)
            record[f"{name}_distance_error_m"] = (
                None if estimator_magnitude is None or reference_magnitude is None else
                estimator_magnitude - reference_magnitude)
            record[f"{name}_absolute_distance_error_m"] = (
                None if record[f"{name}_distance_error_m"] is None else
                abs(record[f"{name}_distance_error_m"]))
            record[f"{name}_percentage_error"] = (
                None if record[f"{name}_distance_error_m"] is None or
                reference_magnitude <= 1e-12 else
                abs(record[f"{name}_distance_error_m"]) /
                reference_magnitude * 100.0)
            real_error = (
                None if estimator_magnitude is None or reference_magnitude is None else
                estimator_magnitude - reference_magnitude)
            record[f"{name}_error_vs_manual_reference_m"] = real_error
            record[f"{name}_absolute_error_vs_manual_reference_m"] = (
                None if real_error is None else abs(real_error))
            record[f"{name}_percent_error_vs_manual_reference"] = (
                None if real_error is None or reference_magnitude <= 1e-12 else
                abs(real_error) / reference_magnitude * 100.0)
        final_heading_deg = (
            None if result.manual_reference is None else
            result.manual_reference.get("final_heading_deviation_deg"))
        final_heading_rad = (
            None if final_heading_deg is None else math.radians(float(final_heading_deg)))
        heading_comparison = translation_heading_comparison(
            m.encoder_angle_rad, m.imu_angle_rad, final_heading_deg,
            m.odometry_yaw_drift_rad)
        record.update({
            "compass_heading_deg": heading_comparison["compass_heading_deg"],
            "compass_heading_rad": heading_comparison["compass_heading_rad"],
            "encoder_heading_rad": heading_comparison["encoder"]["rad"],
            "encoder_heading_deg": heading_comparison["encoder"]["deg"],
            "imu_heading_rad": heading_comparison["imu"]["rad"],
            "imu_heading_deg": heading_comparison["imu"]["deg"],
            "odometry_heading_rad": heading_comparison["odometry"]["rad"],
            "odometry_heading_deg": heading_comparison["odometry"]["deg"],
            "encoder_heading_absolute_error_rad": heading_comparison[
                "encoder"]["absolute_error_vs_compass_rad"],
            "imu_heading_absolute_error_rad": heading_comparison[
                "imu"]["absolute_error_vs_compass_rad"],
            "odometry_heading_absolute_error_rad": heading_comparison[
                "odometry"]["absolute_error_vs_compass_rad"],
        })
        for name, value in (("encoder", m.encoder_angle_rad),
                            ("imu", m.imu_angle_rad),
                            ("odometry", m.odometry_yaw_drift_rad)):
            record[f"{name}_heading_error_rad"] = (
                None if value is None or final_heading_rad is None else
                value - final_heading_rad)
            record[f"{name}_heading_error_deg"] = (
                None if record[f"{name}_heading_error_rad"] is None else
                math.degrees(record[f"{name}_heading_error_rad"]))
            record[f"{name}_absolute_heading_error_deg"] = (
                None if record[f"{name}_heading_error_deg"] is None else
                abs(record[f"{name}_heading_error_deg"]))
            record[f"{name}_absolute_heading_error_rad"] = (
                None if record[f"{name}_heading_error_rad"] is None else
                abs(record[f"{name}_heading_error_rad"]))
            record[f"{name}_heading_error_vs_manual_reference_rad"] = (
                record[f"{name}_heading_error_rad"])
            record[f"{name}_heading_error_vs_manual_reference_deg"] = (
                record[f"{name}_heading_error_deg"])
            record[f"{name}_absolute_heading_error_vs_manual_reference_rad"] = (
                record[f"{name}_absolute_heading_error_rad"])
            record[f"{name}_absolute_heading_error_vs_manual_reference_deg"] = (
                record[f"{name}_absolute_heading_error_deg"])
            record[f"{name}_percent_heading_error_vs_manual_reference"] = (
                None if record[f"{name}_heading_error_rad"] is None or
                abs(final_heading_rad) <= 1e-12 else
                abs(record[f"{name}_heading_error_rad"]) /
                abs(final_heading_rad) * 100.0)
    return record


def condition_summary(
        condition: CampaignCondition,
        attempts: Sequence[Dict[str, object]]) -> Dict[str, object]:
    selected = [
        attempt for attempt in attempts
        if attempt.get("condition_id") == condition.condition_id]
    valid = [
        attempt for attempt in selected
        if attempt.get("valid") is True and not attempt.get("skipped")]
    keys = (
        ("command_error_rad", "encoder_error_rad", "imu_error_rad",
         "encoder_imu_disagreement_rad", "encoder_error_vs_manual_reference_rad",
         "odometry_error_vs_manual_reference_rad",
         "imu_error_vs_manual_reference_rad",
         "encoder_absolute_error_vs_manual_reference_rad",
         "odometry_absolute_error_vs_manual_reference_rad",
         "imu_absolute_error_vs_manual_reference_rad",
         "encoder_percent_error_vs_manual_reference",
         "odometry_percent_error_vs_manual_reference",
         "imu_percent_error_vs_manual_reference")
        if condition.spec.movement_type == "rotation" else
        ("command_distance_error_m", "encoder_distance_error_m",
         "theoretical_error_vs_manual_reference_m",
         "encoder_error_vs_manual_reference_m",
         "odometry_error_vs_manual_reference_m",
         "theoretical_absolute_error_vs_manual_reference_m",
         "encoder_absolute_error_vs_manual_reference_m",
         "odometry_absolute_error_vs_manual_reference_m",
         "theoretical_percent_error_vs_manual_reference",
         "encoder_percent_error_vs_manual_reference",
         "odometry_percent_error_vs_manual_reference",
         "encoder_heading_error_rad", "encoder_heading_error_deg",
         "encoder_absolute_heading_error_rad",
         "encoder_absolute_heading_error_deg", "imu_heading_error_rad",
         "imu_heading_error_deg", "imu_absolute_heading_error_rad",
         "imu_absolute_heading_error_deg"))
    stats = {
        key: finite_stats([
            attempt.get(key) for attempt in valid
            if attempt.get(key) is not None])
        for key in keys}
    if condition.spec.movement_type == "rotation":
        stats["compass_measured_rotation_rad"] = finite_stats([
            attempt["reference_value"] for attempt in valid
            if attempt.get("reference_value") is not None])
    return {
        "condition": asdict(condition),
        "valid_count": len(valid),
        "invalid_or_retried_count": len(selected) - len(valid),
        "statistics": stats,
    }


def radius_calibration_analysis(
        attempts: Sequence[Dict[str, object]],
        candidate_radius_m: float,
        track_width_m: float = 0.453) -> Dict[str, object]:
    """Estimate radius from translation and retain rotation as a cross-check."""
    estimates = []
    rotation_estimates = []
    recorded_radii = {
        float(attempt["wheel_radius_m"])
        for attempt in attempts
        if attempt.get("valid") is True and attempt.get("wheel_radius_m") is not None}
    if len(recorded_radii) > 1 or (
            recorded_radii and abs(next(iter(recorded_radii)) - candidate_radius_m) > 1e-12):
        raise ValueError("calibration attempts contain mixed wheel-radius candidates")
    grouped: Dict[str, List[float]] = {}
    for attempt in attempts:
        if attempt.get("valid") is not True or attempt.get("movement_type") != "translation":
            if (attempt.get("valid") is True and
                    attempt.get("movement_type") == "rotation"):
                compass = attempt.get("reference_value")
                encoder_angle = attempt.get("encoder_angle_rad")
                if (compass is not None and encoder_angle is not None and
                        abs(float(encoder_angle)) > 1e-12):
                    rotation_estimates.append({
                        "attempt_id": attempt.get("attempt_id"),
                        "condition_id": attempt.get("condition_id"),
                        "track_width_m": track_width_m,
                        "compass_measured_rotation_rad": compass,
                        "encoder_rotation_rad": encoder_angle,
                        "implied_radius_m": candidate_radius_m * (
                            float(compass) / float(encoder_angle)),
                    })
            continue
        reference = attempt.get("reference_value")
        encoder = attempt.get("encoder_distance_m")
        if reference is None or encoder is None or abs(float(encoder)) <= 1e-12:
            continue
        reference_magnitude = abs(float(reference))
        encoder_magnitude = abs(float(encoder))
        implied = candidate_radius_m * (reference_magnitude / encoder_magnitude)
        estimates.append({
            "attempt_id": attempt.get("attempt_id"),
            "condition_id": attempt.get("condition_id"),
            "direction": attempt.get("direction"),
            "velocity": attempt.get("velocity"),
            "duration_s": attempt.get("duration_s"),
            "raw_reference_distance_m": reference,
            "reference_distance_magnitude_m": reference_magnitude,
            "encoder_distance_m": encoder,
            "encoder_distance_magnitude_m": encoder_magnitude,
            "implied_radius_m": implied,
        })
        grouped.setdefault(str(attempt.get("direction")), []).append(implied)
        grouped.setdefault(f"speed_{attempt.get('velocity')}", []).append(implied)
        grouped.setdefault(f"duration_{attempt.get('duration_s')}", []).append(implied)
    values = [item["implied_radius_m"] for item in estimates]
    stats = finite_stats(values)
    outliers = outlier_candidates(
        [{"attempt_id": item["attempt_id"], "condition_id": item["condition_id"],
          "implied_radius_m": item["implied_radius_m"]} for item in estimates],
        keys=("implied_radius_m",))
    return {
        "current_candidate_radius_m": candidate_radius_m,
        "individual_implied_radius_estimates": estimates,
        "aggregate_recommended_radius_m": (
            None if not values else statistics.median(values)),
        "median_implied_radius_m": stats["median_error"],
        "mean_implied_radius_m": stats["mean_signed_error"],
        "standard_deviation_m": stats["stddev"],
        "confidence_or_spread": stats,
        "spread": stats,
        "forward_only": finite_stats(grouped.get("forward", [])),
        "backward_only": finite_stats(grouped.get("backward", [])),
        "estimate_by_velocity": {
            key: finite_stats(value) for key, value in sorted(grouped.items())
            if key.startswith("speed_")},
        "estimate_by_duration": {
            key: finite_stats(value) for key, value in sorted(grouped.items())
            if key.startswith("duration_")},
        "grouped_estimates": {key: finite_stats(value)
                              for key, value in sorted(grouped.items())},
        "outlier_diagnostics": outliers,
        "rotation_cross_check": {
            "track_width_m": track_width_m,
            "individual_implied_radius_estimates": rotation_estimates,
            "statistics": finite_stats([
                item["implied_radius_m"] for item in rotation_estimates]),
            "automatic_radius_update": False,
            "interpretation": (
                "rotation radius is diagnostic only because it also depends on "
                "track width")},
        "translation_is_primary_estimator": True,
        "policy": (
            "translation-derived radius recommendation only; production parameters "
            "are not modified; rotation residuals are a track-width cross-check"),
    }


def covariance_analysis(attempts: Sequence[Dict[str, object]]) -> Dict[str, object]:
    """Estimate only planar residual components observed by this campaign."""
    encoder_translation = []
    encoder_yaw = []
    imu_yaw = []
    paired_yaw = []
    for attempt in attempts:
        if attempt.get("valid") is not True:
            continue
        if attempt.get("movement_type") == "translation":
            if attempt.get("encoder_distance_error_m") is not None:
                encoder_translation.append(float(attempt["encoder_distance_error_m"]))
            if attempt.get("encoder_heading_error_rad") is not None:
                encoder_yaw.append(float(attempt["encoder_heading_error_rad"]))
            if attempt.get("imu_heading_error_rad") is not None:
                imu_yaw.append(float(attempt["imu_heading_error_rad"]))
        else:
            if attempt.get("encoder_error_rad") is not None:
                encoder_yaw.append(float(attempt["encoder_error_rad"]))
            if attempt.get("imu_error_rad") is not None:
                imu_yaw.append(float(attempt["imu_error_rad"]))
            if (attempt.get("encoder_error_rad") is not None and
                    attempt.get("imu_error_rad") is not None):
                paired_yaw.append((
                    float(attempt["encoder_error_rad"]),
                    float(attempt["imu_error_rad"])))

    def variance(values: Sequence[float]) -> Optional[float]:
        return None if len(values) < 2 else statistics.variance(values)

    cross = None
    if len(paired_yaw) >= 2:
        mean_encoder = statistics.mean(left for left, _right in paired_yaw)
        mean_imu = statistics.mean(right for _left, right in paired_yaw)
        cross = statistics.mean(
            (left - mean_encoder) * (right - mean_imu)
            for left, right in paired_yaw)
    return {
        "encoder_planar_translation_residual_m": {
            "samples": encoder_translation,
            "statistics": finite_stats(encoder_translation),
            "variance": variance(encoder_translation),
        },
        "encoder_planar_yaw_residual_rad": {
            "samples": encoder_yaw,
            "statistics": finite_stats(encoder_yaw),
            "variance": variance(encoder_yaw),
        },
        "imu_planar_yaw_residual_rad": {
            "samples": imu_yaw,
            "statistics": finite_stats(imu_yaw),
            "variance": variance(imu_yaw),
        },
        "encoder_imu_yaw_cross_covariance": cross,
        "grouped": grouped_residual_statistics(attempts),
        "candidate_ros_covariance_representation": {
            "encoder_odom_planar": {
                "linear_x_variance_from_translation_residual_m2": variance(
                    encoder_translation),
                "yaw_variance_from_planar_residual_rad2": variance(encoder_yaw),
                "unsupported_entries": [
                    "absolute y variance",
                    "z variance", "roll variance", "pitch variance"],
            },
            "imu_planar_yaw": {
                "yaw_variance_rad2": variance(imu_yaw),
                "unsupported_entries": [
                    "x", "y", "z", "roll", "pitch"],
            },
        },
        "unsupported_dimensions": [
            "x/y absolute pose covariance from wall-only scalar ranges",
            "z translation", "roll", "pitch",
            "3D IMU covariance axes not observed by planar yaw experiments",
        ],
        "production_write_policy": "candidate values only; production covariance is not modified",
    }


def grouped_residual_statistics(
        attempts: Sequence[Dict[str, object]]) -> Dict[str, object]:
    """Compute required grouped campaign residual statistics."""
    group_values: Dict[str, Dict[str, List[float]]] = {}

    def add(group: str, key: str, value: object) -> None:
        if value is None:
            return
        numeric = float(value)
        if not math.isfinite(numeric):
            return
        group_values.setdefault(group, {}).setdefault(key, []).append(numeric)

    for attempt in attempts:
        if attempt.get("valid") is not True or attempt.get("skipped"):
            continue
        direction = str(attempt.get("direction"))
        speed = f"speed={attempt.get('velocity')}"
        duration = f"duration={attempt.get('duration_s')}"
        speed_duration = f"{speed},duration={attempt.get('duration_s')}"
        movement = str(attempt.get("movement_type"))
        groups = (
            f"direction:{direction}",
            f"speed:{speed}",
            f"duration:{duration}",
            f"speed_duration:{speed_duration}",
            "cw_vs_ccw:rotation" if movement == "rotation" else
            "forward_vs_backward:translation",
            f"movement:{movement}",
        )
        keys = (
            "command_error_rad", "encoder_error_rad", "imu_error_rad",
            "encoder_imu_disagreement_rad",
            "command_distance_error_m", "encoder_distance_error_m",
            "command_percentage_error", "encoder_percentage_error",
            "theoretical_error_vs_manual_reference_m",
            "encoder_error_vs_manual_reference_m",
            "odometry_error_vs_manual_reference_m",
            "theoretical_absolute_error_vs_manual_reference_m",
            "encoder_absolute_error_vs_manual_reference_m",
            "odometry_absolute_error_vs_manual_reference_m",
            "theoretical_percent_error_vs_manual_reference",
            "encoder_percent_error_vs_manual_reference",
            "odometry_percent_error_vs_manual_reference",
            "encoder_error_vs_manual_reference_rad",
            "odometry_error_vs_manual_reference_rad",
            "imu_error_vs_manual_reference_rad",
            "encoder_absolute_error_vs_manual_reference_rad",
            "odometry_absolute_error_vs_manual_reference_rad",
            "imu_absolute_error_vs_manual_reference_rad",
            "encoder_percent_error_vs_manual_reference",
            "odometry_percent_error_vs_manual_reference",
            "imu_percent_error_vs_manual_reference",
            "encoder_heading_error_rad", "encoder_heading_error_deg",
            "encoder_absolute_heading_error_rad",
            "encoder_absolute_heading_error_deg", "imu_heading_error_rad",
            "imu_heading_error_deg", "imu_absolute_heading_error_rad",
            "imu_absolute_heading_error_deg",
            "encoder_heading_error_vs_manual_reference_rad",
            "encoder_heading_error_vs_manual_reference_deg",
            "imu_heading_error_vs_manual_reference_rad",
            "imu_heading_error_vs_manual_reference_deg",
            "odometry_heading_error_vs_manual_reference_rad",
            "odometry_heading_error_vs_manual_reference_deg",
            "encoder_percent_heading_error_vs_manual_reference",
            "imu_percent_heading_error_vs_manual_reference",
            "odometry_percent_heading_error_vs_manual_reference",
        )
        for group in groups:
            for key in keys:
                add(group, key, attempt.get(key))
    return {
        group: {
            key: finite_stats(values)
            for key, values in sorted(values_by_key.items())}
        for group, values_by_key in sorted(group_values.items())}


def manual_reference_accuracy_statistics(
        attempts: Sequence[Dict[str, object]]) -> Dict[str, object]:
    """Report calibration accuracy primarily against measured references."""
    grouped: Dict[str, Dict[str, Dict[str, List[float]]]] = {}

    def add(group: str, estimator: str, error: object, percentage: object) -> None:
        if error is None:
            return
        error_value = float(error)
        if not math.isfinite(error_value):
            return
        target = grouped.setdefault(group, {}).setdefault(
            estimator, {"errors": [], "percentages": []})
        target["errors"].append(error_value)
        if percentage is not None and math.isfinite(float(percentage)):
            target["percentages"].append(float(percentage))

    for attempt in attempts:
        if attempt.get("valid") is not True or attempt.get("skipped"):
            continue
        movement = str(attempt.get("movement_type"))
        groups = (
            f"direction:{attempt.get('direction')}",
            f"speed:{attempt.get('velocity')}",
            f"duration:{attempt.get('duration_s')}",
            f"speed_duration:speed={attempt.get('velocity')},"
            f"duration={attempt.get('duration_s')}",
            f"movement:{movement}",
        )
        if movement == "rotation":
            prefix = "_error_vs_manual_reference_rad"
            percent_suffix = "_percent_error_vs_manual_reference"
            estimators = ("encoder", "odometry", "imu")
        else:
            prefix = "_error_vs_manual_reference_m"
            percent_suffix = "_percent_error_vs_manual_reference"
            estimators = ("theoretical", "encoder", "odometry")
        for group in groups:
            for estimator in estimators:
                add(group, estimator, attempt.get(estimator + prefix),
                    attempt.get(estimator + percent_suffix))
            if movement == "translation":
                for estimator in ("encoder", "imu", "odometry"):
                    add(group, "heading_" + estimator,
                        attempt.get(estimator + "_heading_error_vs_manual_reference_rad"),
                        attempt.get(estimator + "_percent_heading_error_vs_manual_reference"))

    output = {}
    for group, estimators in sorted(grouped.items()):
        output[group] = {}
        for estimator, values in sorted(estimators.items()):
            errors = values["errors"]
            percentages = values["percentages"]
            absolute = [abs(error) for error in errors]
            output[group][estimator] = {
                "count": len(errors),
                "mae_vs_manual_reference": statistics.mean(absolute),
                "rmse_vs_manual_reference": math.sqrt(
                    statistics.mean(error * error for error in errors)),
                "median_absolute_error_vs_manual_reference": statistics.median(absolute),
                "mean_percentage_error_vs_manual_reference": (
                    None if not percentages else statistics.mean(percentages)),
            }
    return output


def whole_campaign_analysis(
        attempts: Sequence[Dict[str, object]]) -> Dict[str, object]:
    """Identify campaign-level bias/dependence/asymmetry without discarding data."""
    valid = [
        attempt for attempt in attempts
        if attempt.get("valid") is True and not attempt.get("skipped")]
    translation_errors = [
        float(attempt["encoder_distance_error_m"])
        for attempt in valid
        if attempt.get("encoder_distance_error_m") is not None]
    rotation_errors = [
        float(attempt["encoder_error_rad"])
        for attempt in valid
        if attempt.get("encoder_error_rad") is not None]
    radius_estimates = []
    for attempt in valid:
        reference = attempt.get("reference_value")
        encoder = attempt.get("encoder_distance_m")
        if (attempt.get("movement_type") == "translation" and
                reference is not None and encoder is not None and
                abs(float(encoder)) > 1e-12):
            radius_estimates.append(abs(float(reference)) / abs(float(encoder)))
    return {
        "grouped_statistics": grouped_residual_statistics(attempts),
        "manual_reference_accuracy": manual_reference_accuracy_statistics(attempts),
        "systematic_scale_bias": {
            "encoder_translation_error_m": finite_stats(translation_errors),
            "encoder_rotation_error_rad": finite_stats(rotation_errors),
            "translation_reference_over_encoder_scale": finite_stats(
                radius_estimates),
        },
        "speed_dependence": grouped_residual_statistics([
            attempt for attempt in valid]),
        "duration_dependence": grouped_residual_statistics([
            attempt for attempt in valid]),
        "direction_asymmetry": {
            "cw": grouped_residual_statistics([
                attempt for attempt in valid
                if attempt.get("direction") == "cw"]),
            "ccw": grouped_residual_statistics([
                attempt for attempt in valid
                if attempt.get("direction") == "ccw"]),
            "forward": grouped_residual_statistics([
                attempt for attempt in valid
                if attempt.get("direction") == "forward"]),
            "backward": grouped_residual_statistics([
                attempt for attempt in valid
                if attempt.get("direction") == "backward"]),
        },
        "repeatability": grouped_residual_statistics(attempts),
        "outliers_policy": (
            "outliers are retained; no automatic discard is performed"),
        "outlier_candidates": outlier_candidates(valid),
        "encoder_imu_disagreement": finite_stats([
            float(attempt["encoder_imu_disagreement_rad"])
            for attempt in valid
            if attempt.get("encoder_imu_disagreement_rad") is not None]),
        "wheel_radius_error_condition_dependence": (
            "inspect grouped translation_reference_over_encoder_scale; "
            "constant bias suggests radius error, condition-dependent bias "
            "suggests traction/speed/duration effects"),
    }


def outlier_candidates(
        attempts: Sequence[Dict[str, object]],
        keys: Sequence[str] = (
            "encoder_distance_error_m", "encoder_error_rad", "imu_error_rad",
            "encoder_heading_error_deg", "imu_heading_error_deg")) \
        -> Tuple[Dict[str, object], ...]:
    """Flag residuals outside three sample standard deviations; retain them."""
    candidates = []
    for key in keys:
        values = [
            float(attempt[key]) for attempt in attempts
            if attempt.get(key) is not None]
        if len(values) < 3:
            continue
        mean = statistics.mean(values)
        stddev = statistics.stdev(values)
        if stddev <= 0.0:
            continue
        for attempt in attempts:
            value = attempt.get(key)
            if value is None:
                continue
            z_score = (float(value) - mean) / stddev
            if abs(z_score) > 3.0:
                candidates.append({
                    "attempt_id": attempt.get("attempt_id"),
                    "condition_id": attempt.get("condition_id"),
                    "residual": key,
                    "value": value,
                    "z_score": z_score,
                    "discarded": False,
                })
    return tuple(candidates)


def validation_comparison(
        baseline_attempts: Sequence[Dict[str, object]],
        validation_attempts: Sequence[Dict[str, object]]) -> Dict[str, object]:
    """Compare validation metrics against a selected baseline/calibration session."""
    keys = (
        "encoder_distance_error_m", "encoder_error_rad", "imu_error_rad",
        "encoder_heading_error_deg", "imu_heading_error_deg")

    def metrics(attempts, key):
        values = [
            float(attempt[key]) for attempt in attempts
            if attempt.get("valid") is True and attempt.get(key) is not None]
        stats = finite_stats(values)
        return {
            "count": stats["count"],
            "bias": stats["mean_signed_error"],
            "mae": stats["mean_absolute_error"],
            "rmse": stats["rmse"],
        }

    comparison = {}
    for key in keys:
        before = metrics(baseline_attempts, key)
        after = metrics(validation_attempts, key)
        comparison[key] = {
            "baseline": before,
            "validation": after,
            "mae_improved": (
                None if before["mae"] is None or after["mae"] is None else
                after["mae"] < before["mae"]),
            "rmse_improved": (
                None if before["rmse"] is None or after["rmse"] is None else
                after["rmse"] < before["rmse"]),
            "bias_magnitude_improved": (
                None if before["bias"] is None or after["bias"] is None else
                abs(after["bias"]) < abs(before["bias"])),
        }
    comparison["direction_asymmetry"] = {
        "baseline": grouped_residual_statistics(baseline_attempts),
        "validation": grouped_residual_statistics(validation_attempts),
        "improvement": direction_asymmetry_improvement(
            baseline_attempts, validation_attempts),
    }
    return comparison


def direction_asymmetry_improvement(
        baseline_attempts: Sequence[Dict[str, object]],
        validation_attempts: Sequence[Dict[str, object]]) -> Dict[str, object]:
    """Report whether opposite-direction MAE gaps improved."""
    pairs = (
        ("cw", "ccw", "encoder_error_rad"),
        ("forward", "backward", "encoder_distance_error_m"),
        ("forward", "backward", "encoder_heading_error_deg"),
        ("forward", "backward", "imu_heading_error_deg"),
    )

    def mae_for(attempts, direction, key):
        values = [
            abs(float(attempt[key])) for attempt in attempts
            if (attempt.get("valid") is True and
                attempt.get("direction") == direction and
                attempt.get(key) is not None)]
        return None if not values else statistics.mean(values)

    output = {}
    for left, right, key in pairs:
        baseline_left = mae_for(baseline_attempts, left, key)
        baseline_right = mae_for(baseline_attempts, right, key)
        validation_left = mae_for(validation_attempts, left, key)
        validation_right = mae_for(validation_attempts, right, key)
        baseline_gap = (
            None if baseline_left is None or baseline_right is None else
            abs(baseline_left - baseline_right))
        validation_gap = (
            None if validation_left is None or validation_right is None else
            abs(validation_left - validation_right))
        output[f"{left}_vs_{right}:{key}"] = {
            "baseline_mae_gap": baseline_gap,
            "validation_mae_gap": validation_gap,
            "direction_asymmetry_improved": (
                None if baseline_gap is None or validation_gap is None else
                validation_gap < baseline_gap),
        }
    return output


def atomic_write_json(path: Path, payload: Dict[str, object]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def write_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    path = Path(path)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        if rows:
            fieldnames = sorted({key for row in rows for key in row})
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
