# Copyright 2026 Medrobots Engineering
"""Offline covariance characterization from one completed calibration campaign.

This module deliberately reads per-attempt ``report.json`` files rather than
campaign summary fields.  The latter are historical presentation artifacts and
must not be used to derive translation covariance.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
from typing import Any, Iterable


EXPECTED_GEOMETRY = {
    "wheel_radius_m": 0.087,
    "track_width_m": 0.453,
    "encoder_ticks_per_revolution": 4096,
}
EXPECTED_VALID_TRIALS = 160
ROBUST_Z_THRESHOLD = 3.5
HIGH_COVARIANCE = 1_000_000.0


@dataclass(frozen=True)
class Residual:
    attempt_id: str
    movement_type: str
    direction: str
    velocity: float
    duration_s: float
    reference: float
    estimate: float
    residual: float


def _finite(values: Iterable[float]) -> list[float]:
    return [float(value) for value in values if math.isfinite(float(value))]


def _stats(values: Iterable[float]) -> dict[str, float | int | None]:
    data = _finite(values)
    if not data:
        return {"n": 0, "mean": None, "bias": None, "sample_stddev": None,
                "variance": None, "rmse": None, "median_absolute_error": None,
                "mad": None, "robust_spread": None, "min": None, "max": None}
    mean = statistics.mean(data)
    median = statistics.median(data)
    mad = statistics.median(abs(value - median) for value in data)
    stddev = statistics.stdev(data) if len(data) > 1 else 0.0
    return {
        "n": len(data),
        "mean": mean,
        "bias": mean,
        "sample_stddev": stddev,
        "variance": stddev * stddev,
        "rmse": math.sqrt(statistics.mean(value * value for value in data)),
        "median_absolute_error": statistics.median(abs(value) for value in data),
        "mad": mad,
        "robust_spread": 1.4826 * mad,
        "min": min(data),
        "max": max(data),
    }


def _wrapped_delta(estimate: float, reference: float) -> float:
    """Return the shortest ROS-signed angular residual in radians."""
    delta = float(estimate) - float(reference)
    return math.atan2(math.sin(delta), math.cos(delta))


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def validate_completed_campaign(campaign: Path) -> dict[str, Any]:
    """Fail closed unless *campaign* is the requested complete fixed matrix."""
    metadata = _load_json(campaign / "metadata.json")
    manifest = _load_json(campaign / "campaign_manifest.json")
    geometry = metadata.get("geometry")
    if geometry != EXPECTED_GEOMETRY:
        raise ValueError(f"campaign geometry mismatch: {geometry!r}")
    if manifest.get("mode") != "calibration":
        raise ValueError("only calibration campaigns are accepted")
    progress = manifest.get("progress", {})
    if (progress.get("complete") is not True or
            progress.get("completed_valid_repetitions") != EXPECTED_VALID_TRIALS or
            progress.get("planned_valid_repetitions") != EXPECTED_VALID_TRIALS):
        raise ValueError(f"campaign is incomplete: {progress!r}")
    attempts = _load_json(campaign / "campaign_attempts.json")
    radii = {attempt.get("wheel_radius_m") for attempt in attempts}
    if radii != {EXPECTED_GEOMETRY["wheel_radius_m"]}:
        raise ValueError(f"mixed or unexpected attempt radii: {radii!r}")
    reports = sorted(campaign.glob("*-attempt-*/report.json"))
    valid_reports = []
    for report_path in reports:
        report = _load_json(report_path)
        if report.get("operator", {}).get("valid") is True and not report.get(
                "operator", {}).get("skipped"):
            valid_reports.append(report_path)
    if len(valid_reports) != EXPECTED_VALID_TRIALS:
        raise ValueError(f"expected {EXPECTED_VALID_TRIALS} valid reports, got {len(valid_reports)}")
    types = [_load_json(path)["trial"]["movement_type"] for path in valid_reports]
    if types.count("translation") != 80 or types.count("rotation") != 80:
        raise ValueError(f"unexpected movement counts: {types.count('translation')}/{types.count('rotation')}")
    return {"metadata": metadata, "manifest": manifest, "attempts": attempts,
            "valid_reports": valid_reports}


def load_residuals(campaign: Path) -> tuple[list[Residual], list[Residual]]:
    validated = validate_completed_campaign(campaign)
    translation: list[Residual] = []
    rotation: list[Residual] = []
    for report_path in validated["valid_reports"]:
        report = _load_json(report_path)
        trial = report["trial"]
        reference = report["physical_reference"]["manual_reference"]
        if trial["movement_type"] == "translation":
            ref = abs(float(reference["distance_reference_m"]))
            estimate = abs(float(report["odometry"]["displacement_m"]))
            residual = estimate - ref
            target = translation
        else:
            ref = float(reference["signed_measured_rotation_rad"])
            estimate = float(report["odometry"]["unwrapped_angle_rad"])
            residual = _wrapped_delta(estimate, ref)
            target = rotation
        target.append(Residual(
            attempt_id=report_path.parent.name,
            movement_type=trial["movement_type"],
            direction=trial["direction"], velocity=float(trial["velocity"]),
            duration_s=float(trial["duration_s"]), reference=ref,
            estimate=estimate, residual=residual))
    return translation, rotation


def _outliers(values: list[Residual]) -> list[Residual]:
    errors = [row.residual for row in values]
    median = statistics.median(errors)
    mad = statistics.median(abs(error - median) for error in errors)
    if mad == 0.0:
        return []
    return [row for row in values if abs(0.67448975 *
            (row.residual - median) / mad) > ROBUST_Z_THRESHOLD]


def _grouped(values: list[Residual], rate: bool = False) -> dict[str, dict[str, Any]]:
    def key(row: Residual) -> dict[str, str]:
        return {
            "speed": f"{row.velocity:.2f}", "duration": f"{row.duration_s:.1f}",
            "direction": row.direction,
        }
    groups: dict[str, list[float]] = {}
    for row in values:
        error = row.residual / row.duration_s if rate else row.residual
        for name, value in key(row).items():
            groups.setdefault(f"{name}:{value}", []).append(error)
        groups.setdefault(f"speed_duration:{row.velocity:.2f}/{row.duration_s:.1f}", []).append(error)
    return {name: _stats(errors) for name, errors in sorted(groups.items())}


def characterize_campaign(campaign: Path) -> dict[str, Any]:
    """Return deterministic statistics and selected diagonal covariance values."""
    translation, rotation = load_residuals(campaign)
    translation_outliers = _outliers(translation)
    rotation_outliers = _outliers(rotation)
    translation_rate = [row.residual / row.duration_s for row in translation]
    rotation_rate = [row.residual / row.duration_s for row in rotation]
    translation_robust_variance = _stats(row.residual for row in translation)["robust_spread"] ** 2
    translation_rate_robust_variance = _stats(translation_rate)["robust_spread"] ** 2
    rotation_variance = _stats(row.residual for row in rotation)["variance"]
    rotation_rate_variance = _stats(rotation_rate)["variance"]
    pose = [translation_robust_variance, HIGH_COVARIANCE, HIGH_COVARIANCE,
            HIGH_COVARIANCE, HIGH_COVARIANCE, rotation_variance]
    twist = [translation_rate_robust_variance, HIGH_COVARIANCE, HIGH_COVARIANCE,
             HIGH_COVARIANCE, HIGH_COVARIANCE, rotation_rate_variance]
    return {
        "source_campaign": str(campaign), "geometry": EXPECTED_GEOMETRY,
        "trial_counts": {"valid": len(translation) + len(rotation),
                          "translation": len(translation), "rotation": len(rotation),
                          "retained_invalid_or_retried": 9},
        "method": {
            "translation": "abs(odometry displacement) - abs(wall distance reference); compass heading ignored",
            "rotation": "shortest wrapped odometry yaw - manual compass rotation in ROS sign convention",
            "twist": "same residual divided by fixed trial duration_s",
            "outlier_rule": "two-sided robust z > 3.5 using median and MAD; valid rows retained",
            "selected_translation_estimator": "(1.4826 * MAD)^2 over all valid rows",
            "selected_rotation_estimator": "sample variance over all valid rows",
        },
        "residual_statistics": {
            "pose_x_translation_m": _stats(row.residual for row in translation),
            "twist_linear_x_translation_m_per_s": _stats(translation_rate),
            "pose_yaw_rotation_rad": _stats(row.residual for row in rotation),
            "twist_angular_z_rotation_rad_per_s": _stats(rotation_rate),
        },
        "grouped": {"translation": _grouped(translation),
                    "translation_rate": _grouped(translation, True),
                    "rotation": _grouped(rotation),
                    "rotation_rate": _grouped(rotation, True)},
        "outliers": {
            "translation": [{"attempt_id": row.attempt_id, "residual_m": row.residual}
                            for row in translation_outliers],
            "rotation": [{"attempt_id": row.attempt_id, "residual_rad": row.residual}
                          for row in rotation_outliers],
            "sensitivity_full": {
                "translation": _stats(row.residual for row in translation),
                "translation_rate": _stats(translation_rate),
                "rotation": _stats(row.residual for row in rotation),
                "rotation_rate": _stats(rotation_rate)},
            "sensitivity_flagged_excluded": {
                "translation": _stats(row.residual for row in translation if row not in translation_outliers),
                "translation_rate": _stats(row.residual / row.duration_s for row in translation if row not in translation_outliers),
                "rotation": _stats(row.residual for row in rotation if row not in rotation_outliers),
                "rotation_rate": _stats(row.residual / row.duration_s for row in rotation if row not in rotation_outliers)},
        },
        "selected_covariance": {
            "pose_x": pose[0], "pose_yaw": pose[5],
            "twist_linear_x": twist[0], "twist_angular_z": twist[5],
        },
        "pose_covariance": pose, "twist_covariance": twist,
        "unsupported_components": {
            "pose": ["y", "z", "roll", "pitch"],
            "twist": ["linear_y", "linear_z", "angular_x", "angular_y"],
            "rationale": "No physical measurement of these axes exists in the campaign; retain high covariance.",
        },
    }
