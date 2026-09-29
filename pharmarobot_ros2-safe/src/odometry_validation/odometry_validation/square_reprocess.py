# Copyright 2026 Medrobots Engineering
#
# Licensed under the Apache License, Version 2.0 (the "License");
"""Immutable offline reprocessing for a completed square failure campaign."""

import argparse
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import statistics
from typing import Dict, Iterable, List, Sequence

from odometry_validation.core import GeometryConfig
from odometry_validation.core import ImuSample
from odometry_validation.core import WheelTickSample
from odometry_validation.square import HeadingFusion
from odometry_validation.square import SquareConfig
from odometry_validation.square import apply_continuous_final_metrics
from odometry_validation.square import imu_coverage
from odometry_validation.square import reconstruct_trajectories
from odometry_validation.square import require_complete_imu_coverage
from odometry_validation.square import square_metrics
from odometry_validation.square import timestamp_coverage


KNOWN_RECOVERABLE_ERROR = (
    "processed /imu/data does not bracket square trajectory bounds")


def _read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _read_imu(path: Path) -> List[ImuSample]:
    with path.open(encoding="utf-8", newline="") as stream:
        return [
            ImuSample(
                float(row["timestamp_s"]),
                float(row["angular_velocity_z_rad_s"]), row["phase"],
                row["source_topic"])
            for row in csv.DictReader(stream)]


def _read_wheels(path: Path) -> List[WheelTickSample]:
    with path.open(encoding="utf-8", newline="") as stream:
        return [
            WheelTickSample(
                float(row["timestamp_s"]), int(row["left_ticks"]),
                int(row["right_ticks"]), row["phase"])
            for row in csv.DictReader(stream)]


def _read_timestamps(path: Path) -> List[float]:
    with path.open(encoding="utf-8", newline="") as stream:
        return [float(row["timestamp_s"]) for row in csv.DictReader(stream)]


def _finite_float(value, label: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"source {label} must be finite")
    return result


def _require_stream_coverage(name: str, stream: Dict[str, object]) -> None:
    if (stream["invalid_timestamp_count"] or
            stream["out_of_order_timestamp_count"] or
            not stream["overlaps_trajectory"] or
            stream["coverage_before_start_s"] is None or
            stream["coverage_before_start_s"] < 0.0 or
            stream["coverage_after_end_s"] is None or
            stream["coverage_after_end_s"] < 0.0 or
            stream["gaps_over_threshold_count"]):
        raise ValueError(
            f"{name} does not continuously bracket square trajectory bounds")


def _write_json(path: Path, payload) -> None:
    with path.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def _write_rows(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    fieldnames = tuple(rows[0].keys()) if rows else ("timestamp_s",)
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _source_hashes(source: Path) -> Dict[str, str]:
    result = {}
    for path in sorted(item for item in source.rglob("*") if item.is_file()):
        result[str(path.relative_to(source))] = hashlib.sha256(
            path.read_bytes()).hexdigest()
    return result


def _turn_baseline(source: Path, manifest) -> Dict[str, object]:
    turns = []
    for segment in manifest:
        if segment["movement_type"] != "rotation":
            continue
        report = _read_json(
            source / Path(segment["evidence_dir"]).name / "report.json")
        commanded = _finite_float(
            report["theoretical"]["expected_angle_rad"], "commanded angle")
        encoder = _finite_float(report["encoder"]["angle_rad"], "encoder angle")
        imu = _finite_float(
            report["imu"]["total_physical_motion_angle_rad"], "IMU angle")
        turns.append({
            "turn_index": len(turns) + 1,
            "commanded_angle_rad": commanded,
            "encoder_angle_rad": encoder,
            "imu_angle_rad": imu,
            "encoder_under_rotation_rad": abs(commanded) - abs(encoder),
            "imu_under_rotation_rad": abs(commanded) - abs(imu),
        })
    encoder_deficits = [row["encoder_under_rotation_rad"] for row in turns]
    imu_deficits = [row["imu_under_rotation_rad"] for row in turns]

    def stats(values: Iterable[float]) -> Dict[str, float]:
        values = list(values)
        return {
            "mean_rad": statistics.mean(values),
            "sample_std_rad": statistics.stdev(values),
            "mean_deg": statistics.mean(values) * 180.0 / 3.141592653589793,
            "sample_std_deg": (
                statistics.stdev(values) * 180.0 / 3.141592653589793),
        }

    return {
        "turns": turns,
        "encoder_under_rotation": stats(encoder_deficits),
        "imu_under_rotation": stats(imu_deficits),
        "calibration_changed": False,
    }


def qualify_source_campaign(metadata, failure) -> None:
    """Accept only a safely completed square with the known reporting failure."""
    if (failure.get("error_type") != "ValidationError" or
            failure.get("error") != KNOWN_RECOVERABLE_ERROR):
        raise ValueError("source failure is not the known recoverable IMU report error")
    context = failure.get("failure_context") or {}
    if (not context.get("cleanup_attempted") or
            not context.get("cleanup_completed")):
        raise ValueError("source campaign does not prove successful safe cleanup")
    manifest = context.get("completed_segments") or []
    config = SquareConfig(**metadata["square"])
    expected = config.segments()
    if len(manifest) != len(expected):
        raise ValueError("source campaign does not contain eight completed segments")
    previous_stationary_s = None
    for index, (segment, spec) in enumerate(zip(manifest, expected), start=1):
        stationarity = segment.get("stationarity") or {}
        if (segment.get("segment_index") != index or
                segment.get("trial_id") != spec.trial_id or
                segment.get("movement_type") != spec.movement_type or
                segment.get("direction") != spec.direction):
            raise ValueError("source segment order does not match configured square")
        if not segment.get("valid") or not stationarity.get("stationary"):
            raise ValueError("source contains an invalid or nonstationary segment")
        start_s = segment.get("command_start_timestamp_s")
        end_s = segment.get("command_end_timestamp_s")
        stationary_s = segment.get("stationary_confirmation_timestamp_s")
        if (not all(isinstance(value, (int, float)) for value in (
                start_s, end_s, stationary_s)) or
                not all(math.isfinite(value) for value in (
                    start_s, end_s, stationary_s)) or
                not start_s < end_s <= stationary_s or
                (previous_stationary_s is not None and
                 start_s <= previous_stationary_s)):
            raise ValueError("source segment timestamps are invalid or unordered")
        previous_stationary_s = stationary_s
    if (context.get("next_segment_index") != 9 or
            not context.get("remaining_square_aborted")):
        raise ValueError("source failure did not occur after all square segments")


def reprocess_campaign(
        source: Path, output: Path,
        legacy_max_imu_gap_s: float = None) -> Dict[str, object]:
    source = source.resolve()
    output = output.resolve()
    if output == source or source in output.parents:
        raise ValueError("reprocessed evidence must be a separate sibling directory")
    if output.exists():
        raise FileExistsError(f"reprocessed evidence already exists: {output}")
    metadata = _read_json(source / "metadata.json")
    failure = _read_json(source / "failure.json")
    qualify_source_campaign(metadata, failure)
    manifest = failure["failure_context"]["completed_segments"]
    max_imu_gap_s = metadata.get("stale_timeout_s")
    if max_imu_gap_s is None:
        max_imu_gap_s = legacy_max_imu_gap_s
    if (max_imu_gap_s is None or not isinstance(max_imu_gap_s, (int, float)) or
            max_imu_gap_s <= 0.0 or not math.isfinite(max_imu_gap_s)):
        raise ValueError(
            "legacy campaign requires an explicit positive maximum IMU gap")
    start_s = float(manifest[0]["command_start_timestamp_s"])
    end_s = float(manifest[-1]["stationary_confirmation_timestamp_s"])
    geometry = GeometryConfig(**metadata["geometry"])
    config = SquareConfig(**metadata["square"])
    fusion_payload = dict(metadata["heading_fusion"])
    for key in ("trials", "command_residual_trials"):
        fusion_payload[key] = tuple(fusion_payload.get(key, ()))
    fusion = HeadingFusion(**fusion_payload)
    imu = _read_imu(source / "raw_imu.csv")
    if any(not math.isfinite(sample.angular_velocity_z_rad_s) for sample in imu):
        raise ValueError("source IMU angular velocities must be finite")
    imu_bias_rad_s = _finite_float(metadata["imu_bias_rad_s"], "IMU bias")
    wheels = _read_wheels(source / "wheel_ticks.csv")
    bounded_wheels = tuple(
        sample for sample in wheels if start_s <= sample.timestamp_s <= end_s)
    trajectory = reconstruct_trajectories(
        bounded_wheels, imu, geometry, imu_bias_rad_s,
        metadata["stationarity_thresholds"]["wheel_tick_semantics"],
        start_s, end_s)
    coverage = {
        "bounds": {
            "start_timestamp_s": start_s, "end_timestamp_s": end_s,
            "duration_s": end_s - start_s,
            "time_basis": (
                "ROS time seconds; sensor header stamps with node-clock fallback"),
        },
        "imu": imu_coverage(imu, start_s, end_s, max_imu_gap_s),
        "imu_callback_delivery": {
            "available": False,
            "reason": (
                "legacy campaign predates per-callback receipt timestamps; "
                "source-sample gaps remain available"),
        },
        "wheel_ticks": timestamp_coverage(
            tuple(sample.timestamp_s for sample in wheels), start_s, end_s,
            max_imu_gap_s),
        "odometry": timestamp_coverage(
            _read_timestamps(source / "odometry.csv"), start_s, end_s,
            max_imu_gap_s),
        "commands": timestamp_coverage(
            _read_timestamps(source / "commanded_velocity.csv"), start_s, end_s,
            max_imu_gap_s),
    }
    require_complete_imu_coverage(coverage["imu"])
    for stream_name in ("wheel_ticks", "odometry", "commands"):
        _require_stream_coverage(stream_name, coverage[stream_name])
    measurements = []
    for segment in manifest:
        report = _read_json(
            source / Path(segment["evidence_dir"]).name / "report.json")
        operator = report.get("operator") or {}
        if (not operator.get("valid") or operator.get("skipped") or
                report.get("trial", {}).get("trial_id") != segment["trial_id"]):
            raise ValueError("source segment report is not valid and accepted")
        if segment["movement_type"] == "translation":
            measurements.append({
                "encoder_distance_m": _finite_float(
                    report["encoder"]["mean_distance_m"], "encoder distance"),
                "encoder_angle_rad": _finite_float(
                    report["translation_drift"]["encoder_yaw_estimate_rad"],
                    "encoder translation yaw"),
                "imu_angle_rad": _finite_float(
                    report["translation_drift"]["imu_yaw_drift_rad"],
                    "IMU translation yaw"),
            })
        else:
            measurements.append({
                "encoder_distance_m": 0.0,
                "encoder_angle_rad": _finite_float(
                    report["encoder"]["angle_rad"], "encoder angle"),
                "imu_angle_rad": _finite_float(
                    report["imu"]["total_physical_motion_angle_rad"],
                    "IMU angle"),
            })
    report = apply_continuous_final_metrics(
        square_metrics(config, measurements, fusion), trajectory)
    report.update({
        "measurement_status": "recovered_from_completed_failed_campaign",
        "source_evidence_dir": str(source),
        "source_error": failure["error"],
        "source_evidence_unchanged": True,
        "max_imu_gap_s": max_imu_gap_s,
        "legacy_gap_threshold_override_s": (
            legacy_max_imu_gap_s if "stale_timeout_s" not in metadata else None),
        "trajectory_diagnostics": coverage,
        "under_rotation_baseline": _turn_baseline(source, manifest),
        "segment_timing_and_stationarity": manifest,
    })
    output.mkdir(parents=True, exist_ok=False)
    _write_json(output / "recovery_metadata.json", {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "source_evidence_dir": str(source),
        "source_sha256": _source_hashes(source),
        "method": "offline reconstruction from immutable raw campaign evidence",
        "geometry": asdict(geometry),
    })
    _write_json(output / "square_report.json", report)
    _write_json(output / "square_trajectory.json", trajectory)
    _write_json(output / "trajectory_diagnostics.json", coverage)
    _write_json(output / "segment_manifest.json", {"segments": manifest})
    _write_rows(output / "encoder_trajectory.csv", trajectory["encoder"])
    _write_rows(output / "imu_trajectory.csv", trajectory["imu"])
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Reprocess a completed square failure without motion.")
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--max-imu-gap-s", type=float,
        help=("Required for legacy evidence that did not record stale_timeout_s; "
              "use the actual runtime stale-data threshold."))
    args = parser.parse_args(argv)
    report = reprocess_campaign(
        args.source, args.output, args.max_imu_gap_s)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
