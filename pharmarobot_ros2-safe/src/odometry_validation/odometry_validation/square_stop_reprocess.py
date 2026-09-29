# Copyright 2026 Medrobots Engineering
#
# Licensed under the Apache License, Version 2.0 (the "License");
"""Immutable offline stop-timeline recovery for a failed square campaign."""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import statistics
from typing import Dict, Sequence


def _read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _read_rows(path: Path):
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def _write_json(path: Path, payload) -> None:
    with path.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def _write_rows(path: Path, rows) -> None:
    fieldnames = tuple(rows[0]) if rows else ("timestamp_s",)
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _source_hashes(source: Path) -> Dict[str, str]:
    return {
        str(path.relative_to(source)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(item for item in source.rglob("*") if item.is_file())}


def _finite(value, label: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _first_assessment(assessments, field):
    return next((item for item in assessments if item.get(field) is True), None)


def _record_timeline(record) -> Dict[str, object]:
    assessments = record.get("stationarity_assessments") or []
    first_safe = _first_assessment(assessments, "safe_zero")
    first_odom = _first_assessment(assessments, "odom_twist_stationary")
    first_encoder = _first_assessment(assessments, "tick_deltas_stationary")
    first_full = _first_assessment(assessments, "stationary")
    first = assessments[0] if assessments else {}
    last = assessments[-1] if assessments else {}
    return {
        "stop_mode": record.get("stop_mode"),
        "timeout_s": record.get("timeout_s"),
        "reported_safe_zero": record.get("safe_zero"),
        "reported_stationary": record.get("stationary"),
        "first_zero_command_timestamp_s": first.get("first_zero_timestamp_s"),
        "first_fresh_safe_zero_timestamp_s": (
            None if first_safe is None else
            first_safe.get("first_safe_zero_timestamp_s")),
        "safe_zero_latency_s": (
            None if first_safe is None else
            first_safe.get("time_from_first_zero_to_safe_zero_s")),
        "first_odom_stationary_timestamp_s": (
            None if first_odom is None else
            first_odom.get("assessment_timestamp_s")),
        "first_encoder_stationary_timestamp_s": (
            None if first_encoder is None else
            first_encoder.get("assessment_timestamp_s")),
        "full_stationarity_timestamp_s": (
            None if first_full is None else
            first_full.get("assessment_timestamp_s")),
        "time_to_full_stationarity_s": (
            None if first_full is None else
            first_full.get("elapsed_since_first_zero_s")),
        "last_assessment_timestamp_s": last.get("assessment_timestamp_s"),
        "last_assessment_reason": last.get("reason"),
        "assessment_count": len(assessments),
        "safe_zero_was_observed_then_lost_in_summary": bool(
            first_safe is not None and not record.get("safe_zero")),
    }


def _distribution(values: Sequence[float]) -> Dict[str, object]:
    ordered = sorted(values)
    if not ordered:
        return {"count": 0}
    p95_index = max(0, math.ceil(0.95 * len(ordered)) - 1)
    return {
        "count": len(ordered),
        "minimum_s": ordered[0],
        "mean_s": statistics.mean(ordered),
        "median_s": statistics.median(ordered),
        "p95_nearest_rank_s": ordered[p95_index],
        "maximum_s": ordered[-1],
        "count_over_1s": sum(value > 1.0 for value in ordered),
        "count_over_2s": sum(value > 2.0 for value in ordered),
    }


def _settling_rows(campaigns: Sequence[Path]):
    rows = []
    for campaign in campaigns:
        for directory in sorted(campaign.iterdir()):
            path = directory / "stationarity.json"
            if not path.exists():
                continue
            payload = _read_json(path)
            if not payload.get("final_accepted_window"):
                continue
            assessments = payload.get("assessments") or []

            def elapsed(field):
                item = _first_assessment(assessments, field)
                return None if item is None else item.get(
                    "elapsed_since_first_zero_s")

            rows.append({
                "campaign": campaign.name,
                "segment": directory.name,
                "movement_type": (
                    "rotation" if "turn" in directory.name else "translation"),
                "zero_to_odom_stationary_s": elapsed("odom_twist_stationary"),
                "zero_to_encoder_stationary_s": elapsed(
                    "tick_deltas_stationary"),
                "zero_to_full_stationary_s": elapsed("stationary"),
            })
    return rows


def analyze_stop_campaign(
        source: Path, comparison_campaigns: Sequence[Path]) -> Dict[str, object]:
    source = source.resolve()
    metadata = _read_json(source / "metadata.json")
    failure = _read_json(source / "failure.json")
    records = failure.get("emergency_stop_records") or []
    if not records:
        raise ValueError("source campaign has no stop verification records")
    context = failure.get("failure_context") or {}
    start_s = _finite(context["command_start_timestamp_s"], "command start")
    zero_tolerance = _finite(
        metadata["stationarity_thresholds"]["safe_command_zero_tolerance"],
        "safe command zero tolerance")
    tick_tolerance = int(
        metadata["stationarity_thresholds"]["tick_delta_tolerance"])
    linear_tolerance = _finite(
        metadata["stationarity_thresholds"][
            "linear_velocity_tolerance_m_s"], "linear tolerance")
    angular_tolerance = _finite(
        metadata["stationarity_thresholds"][
            "angular_velocity_tolerance_rad_s"], "angular tolerance")

    commands = _read_rows(source / "commanded_velocity.csv")
    command_rows = [{
        **row,
        "timestamp_s": _finite(row["timestamp_s"], "command timestamp"),
        "linear_x_m_s": _finite(row["linear_x_m_s"], "linear command"),
        "angular_z_rad_s": _finite(row["angular_z_rad_s"], "angular command"),
    } for row in commands if _finite(row["timestamp_s"], "command timestamp") >= start_s]

    def command_is_zero(row):
        return (
            abs(row["linear_x_m_s"]) <= zero_tolerance and
            abs(row["angular_z_rad_s"]) <= zero_tolerance)

    last_nonzero_test = max(
        (row for row in command_rows
         if row["topic"] == "/cmd_vel/test" and not command_is_zero(row)),
        key=lambda row: row["timestamp_s"])
    first_zero_test = min(
        (row for row in command_rows
         if row["topic"] == "/cmd_vel/test" and command_is_zero(row) and
         row["timestamp_s"] > last_nonzero_test["timestamp_s"]),
        key=lambda row: row["timestamp_s"])
    first_zero_safe = min(
        (row for row in command_rows
         if row["topic"] == "/cmd_vel/safe" and command_is_zero(row) and
         row["timestamp_s"] >= first_zero_test["timestamp_s"]),
        key=lambda row: row["timestamp_s"])
    first_zero_s = first_zero_test["timestamp_s"]

    encoder_decay = []
    for row in _read_rows(source / "wheel_ticks.csv"):
        timestamp_s = _finite(row["timestamp_s"], "wheel timestamp")
        if timestamp_s < first_zero_s:
            continue
        left = int(row["left_ticks"])
        right = int(row["right_ticks"])
        encoder_decay.append({
            "timestamp_s": timestamp_s,
            "elapsed_since_first_zero_s": timestamp_s - first_zero_s,
            "left_delta_ticks": left,
            "right_delta_ticks": right,
            "within_stationarity_tolerance": (
                abs(left) <= tick_tolerance and abs(right) <= tick_tolerance),
        })
    odom_decay = []
    for row in _read_rows(source / "odometry.csv"):
        timestamp_s = _finite(row["timestamp_s"], "odometry timestamp")
        if timestamp_s < first_zero_s:
            continue
        linear = _finite(row["linear_x_m_s"], "odometry linear twist")
        angular = _finite(row["angular_z_rad_s"], "odometry angular twist")
        odom_decay.append({
            "timestamp_s": timestamp_s,
            "elapsed_since_first_zero_s": timestamp_s - first_zero_s,
            "linear_x_m_s": linear,
            "angular_z_rad_s": angular,
            "within_stationarity_tolerance": (
                abs(linear) <= linear_tolerance and
                abs(angular) <= angular_tolerance),
        })
    moving_encoder = [
        row for row in encoder_decay
        if not row["within_stationarity_tolerance"]]
    nonzero_odom = [
        row for row in odom_decay
        if row["linear_x_m_s"] != 0.0 or row["angular_z_rad_s"] != 0.0]
    moving_odom = [
        row for row in odom_decay
        if not row["within_stationarity_tolerance"]]

    stop_records = [_record_timeline(record) for record in records]
    settling_rows = _settling_rows(tuple(comparison_campaigns) + (source,))
    analysis = {
        "recovery_status": "partial_stop_failure_diagnostics_only",
        "usable_as_complete_square_baseline": False,
        "source_evidence_dir": str(source),
        "source_evidence_unchanged": True,
        "failure_type": failure.get("error_type"),
        "failure": failure.get("error"),
        "completed_segment_count": len(context.get("completed_segments") or []),
        "failing_segment_index": context.get("next_segment_index"),
        "command_timeline": {
            "last_nonzero_test_timestamp_s": last_nonzero_test["timestamp_s"],
            "first_zero_test_timestamp_s": first_zero_s,
            "first_zero_safe_timestamp_s": first_zero_safe["timestamp_s"],
            "last_nonzero_to_first_zero_test_s": (
                first_zero_s - last_nonzero_test["timestamp_s"]),
            "test_zero_to_safe_zero_s": (
                first_zero_safe["timestamp_s"] - first_zero_s),
        },
        "stop_records": stop_records,
        "physical_decay": {
            "last_encoder_motion_timestamp_s": (
                None if not moving_encoder else
                moving_encoder[-1]["timestamp_s"]),
            "last_encoder_motion_elapsed_s": (
                None if not moving_encoder else
                moving_encoder[-1]["elapsed_since_first_zero_s"]),
            "last_nonzero_odom_twist_timestamp_s": (
                None if not nonzero_odom else nonzero_odom[-1]["timestamp_s"]),
            "last_nonzero_odom_twist_elapsed_s": (
                None if not nonzero_odom else
                nonzero_odom[-1]["elapsed_since_first_zero_s"]),
            "last_above_tolerance_odom_timestamp_s": (
                None if not moving_odom else moving_odom[-1]["timestamp_s"]),
            "last_above_tolerance_odom_elapsed_s": (
                None if not moving_odom else
                moving_odom[-1]["elapsed_since_first_zero_s"]),
            "encoder_stationarity_reached": any(
                item.get("first_encoder_stationary_timestamp_s") is not None
                for item in stop_records),
            "full_stationarity_reached": any(
                item.get("full_stationarity_timestamp_s") is not None
                for item in stop_records),
        },
        "safe_zero_confirmation_bug_observed": any(
            item["safe_zero_was_observed_then_lost_in_summary"]
            for item in stop_records),
        "settling_measurements": {
            "segments": settling_rows,
            "zero_to_odom_stationary": _distribution([
                row["zero_to_odom_stationary_s"] for row in settling_rows]),
            "zero_to_encoder_stationary": _distribution([
                row["zero_to_encoder_stationary_s"] for row in settling_rows]),
            "zero_to_full_stationary": _distribution([
                row["zero_to_full_stationary_s"] for row in settling_rows]),
        },
    }
    return {
        "analysis": analysis,
        "encoder_decay": encoder_decay,
        "odom_decay": odom_decay,
        "source_sha256": _source_hashes(source),
    }


def reprocess_stop_campaign(
        source: Path, output: Path,
        comparison_campaigns: Sequence[Path]) -> Dict[str, object]:
    source = source.resolve()
    output = output.resolve()
    if output == source or source in output.parents:
        raise ValueError("stop analysis output must not be inside source evidence")
    if output.exists():
        raise FileExistsError(f"stop analysis output already exists: {output}")
    result = analyze_stop_campaign(
        source, tuple(path.resolve() for path in comparison_campaigns))
    output.mkdir(parents=True, exist_ok=False)
    _write_json(output / "recovery_metadata.json", {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "source_evidence_dir": str(source),
        "source_sha256": result["source_sha256"],
        "method": "immutable offline square stop-timeline reconstruction",
    })
    _write_json(output / "stop_analysis.json", result["analysis"])
    _write_rows(output / "encoder_stop_decay.csv", result["encoder_decay"])
    _write_rows(output / "odometry_stop_decay.csv", result["odom_decay"])
    return result["analysis"]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Reprocess a failed square stop without motion.")
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--comparison-campaign", type=Path, action="append", default=[])
    args = parser.parse_args(argv)
    report = reprocess_stop_campaign(
        args.source, args.output, args.comparison_campaign)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
