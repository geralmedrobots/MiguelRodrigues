# Copyright 2026 Medrobots Engineering
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import math
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from odometry_validation.campaign import CAMPAIGN_MODE_LABELS
from odometry_validation.campaign import CALIBRATION_DURATIONS_S
from odometry_validation.campaign import CALIBRATION_REPETITIONS_PER_CONDITION
from odometry_validation.campaign import campaign_matrix_identity
from odometry_validation.campaign import covariance_analysis
from odometry_validation.campaign import fixed_campaign_conditions
from odometry_validation.campaign import heading_deviation_auto_invalid
from odometry_validation.campaign import next_pending
from odometry_validation.campaign import planned_valid_repetition_count
from odometry_validation.campaign import radius_calibration_analysis
from odometry_validation.campaign import translation_reference_distance_m
from odometry_validation.campaign import validation_comparison
from odometry_validation.core import TrialMeasurements
from odometry_validation.core import make_trial_result
from odometry_validation.campaign import attempt_record
from odometry_validation.covariance_characterization import EXPECTED_GEOMETRY
from odometry_validation.covariance_characterization import _wrapped_delta
from odometry_validation.covariance_characterization import characterize_campaign
from odometry_validation.covariance_characterization import validate_completed_campaign


def measurements(reference, encoder_distance=1.0, encoder_angle=0.1, imu_angle=0.11):
    return TrialMeasurements(
        encoder_distance_m=encoder_distance,
        encoder_angle_rad=encoder_angle,
        left_wheel_distance_m=0.0,
        right_wheel_distance_m=0.0,
        odometry_distance_m=encoder_distance,
        odometry_angle_rad=encoder_angle,
        imu_angle_rad=imu_angle,
        physical_measurement=reference,
        commanded_distance_m=1.0,
        commanded_angle_rad=0.1)


def test_fixed_campaign_matrix_has_48_conditions_and_480_valid_target():
    conditions = fixed_campaign_conditions()

    assert len(conditions) == 48
    assert planned_valid_repetition_count(conditions) == 480
    assert [condition.section for condition in conditions[:12]] == ["CW"] * 12
    assert [condition.section for condition in conditions[12:24]] == ["CCW"] * 12
    assert [condition.section for condition in conditions[24:36]] == ["Forward"] * 12
    assert [condition.section for condition in conditions[36:]] == ["Backward"] * 12
    assert conditions[0].spec.velocity == pytest.approx(0.20)
    assert conditions[3].spec.velocity == pytest.approx(0.50)
    assert conditions[4].spec.duration_s == pytest.approx(3.0)


def test_calibration_matrix_is_reduced_to_160_valid_trials():
    conditions = fixed_campaign_conditions(mode="calibration")

    assert len(conditions) == 32
    assert planned_valid_repetition_count(conditions) == 160
    assert {condition.repetition_target for condition in conditions} == {5}
    assert {condition.spec.duration_s for condition in conditions} == {3.0, 4.0}
    assert 2.0 not in {condition.spec.duration_s for condition in conditions}
    assert {condition.spec.velocity for condition in conditions[:8]} == {
        0.20, 0.30, 0.40, 0.50}
    assert [condition.section for condition in conditions[0:8]] == ["CW"] * 8
    assert [condition.section for condition in conditions[8:16]] == ["CCW"] * 8
    assert [condition.section for condition in conditions[16:24]] == ["Forward"] * 8
    assert [condition.section for condition in conditions[24:32]] == ["Backward"] * 8
    assert campaign_matrix_identity(conditions) == campaign_matrix_identity(
        fixed_campaign_conditions(mode="calibration"))


def test_non_calibration_fixed_matrices_remain_original():
    assert planned_valid_repetition_count(fixed_campaign_conditions(mode="baseline")) == 480
    assert planned_valid_repetition_count(fixed_campaign_conditions(mode="validation")) == 480
    assert planned_valid_repetition_count(fixed_campaign_conditions(mode="covariance")) == 480
    assert CALIBRATION_DURATIONS_S == (3.0, 4.0)
    assert CALIBRATION_REPETITIONS_PER_CONDITION == 5


def test_translation_reference_forward_and_backward_wall_distances():
    assert translation_reference_distance_m("forward", 2.0, 1.25) == pytest.approx(0.75)
    assert translation_reference_distance_m("backward", 1.25, 2.0) == pytest.approx(0.75)
    with pytest.raises(ValueError):
        translation_reference_distance_m("sideways", 1.0, 2.0)


def test_translation_heading_gate_accepts_5_deg_and_retries_above_5_deg():
    assert not heading_deviation_auto_invalid(5.0)
    assert not heading_deviation_auto_invalid(-5.0)
    assert heading_deviation_auto_invalid(5.0001)
    assert heading_deviation_auto_invalid(-5.0001)


def test_invalid_attempts_do_not_increment_next_valid_repetition():
    condition = fixed_campaign_conditions(mode="calibration")[0]
    attempts = [
        {"condition_id": condition.condition_id, "valid": False, "skipped": False},
        {"condition_id": condition.condition_id, "valid": True, "skipped": False},
    ]

    pending_condition, repetition = next_pending((condition,), attempts)

    assert pending_condition == condition
    assert repetition == 2


def test_calibration_invalid_attempts_are_retried_until_five_valid():
    condition = fixed_campaign_conditions(mode="calibration")[0]
    attempts = [
        {"condition_id": condition.condition_id, "valid": False, "skipped": False}
        for _ in range(3)]
    attempts.extend(
        {"condition_id": condition.condition_id, "valid": True, "skipped": False}
        for _ in range(5))

    assert next_pending((condition,), attempts) is None


def test_campaign_mode_labels_cover_four_startup_modes():
    assert set(CAMPAIGN_MODE_LABELS) == {
        "baseline", "calibration", "validation", "covariance"}


def test_radius_calibration_uses_translation_reference_over_encoder_distance():
    condition = fixed_campaign_conditions()[24]
    result = make_trial_result(
        condition.spec,
        measurements(reference=1.10, encoder_distance=1.0),
        valid=True,
        manual_reference={"type": "manual_wall_distance_translation"})
    record = attempt_record("attempt-1", condition, 1, result)

    analysis = radius_calibration_analysis((record,), candidate_radius_m=0.088)

    assert analysis["aggregate_recommended_radius_m"] == pytest.approx(0.0968)
    assert analysis["forward_only"]["count"] == 1
    assert analysis["translation_is_primary_estimator"] is True
    assert analysis["median_implied_radius_m"] == pytest.approx(0.0968)


def test_radius_calibration_uses_translation_magnitudes_for_negative_reference():
    condition = fixed_campaign_conditions(mode="calibration")[16]
    result = make_trial_result(
        condition.spec,
        measurements(reference=-1.46, encoder_distance=1.493176),
        valid=True,
        manual_reference={"type": "manual_wall_distance_translation",
                          "final_heading_deviation_deg": 0.0})
    record = attempt_record("negative-reference", condition, 1, result)

    analysis = radius_calibration_analysis((record,), candidate_radius_m=0.087)

    assert analysis["individual_implied_radius_estimates"][0][
        "reference_distance_magnitude_m"] == pytest.approx(1.46)
    assert analysis["aggregate_recommended_radius_m"] == pytest.approx(
        0.087 * 1.46 / 1.493176)


def test_rotation_radius_is_cross_check_only():
    condition = fixed_campaign_conditions(mode="calibration")[0]
    result = make_trial_result(
        condition.spec,
        measurements(reference=0.20, encoder_angle=0.18),
        valid=True,
        manual_reference={"type": "iphone_compass_rotation"})
    record = attempt_record("rotation", condition, 1, result)

    analysis = radius_calibration_analysis((record,), 0.087, 0.453)

    cross_check = analysis["rotation_cross_check"]
    assert cross_check["statistics"]["count"] == 1
    assert cross_check["individual_implied_radius_estimates"][0][
        "implied_radius_m"] == pytest.approx(0.087 * 0.20 / 0.18)
    assert cross_check["automatic_radius_update"] is False


def test_radius_analysis_rejects_mixed_iteration_candidates():
    condition = fixed_campaign_conditions(mode="calibration")[16]
    result = make_trial_result(
        condition.spec, measurements(reference=1.0), valid=True,
        manual_reference={"type": "manual_wall_distance_translation"})
    record = attempt_record("attempt", condition, 1, result, wheel_radius_m=0.088)

    with pytest.raises(ValueError, match="mixed wheel-radius"):
        radius_calibration_analysis((record,), candidate_radius_m=0.087)


def test_covariance_reports_only_supported_planar_components():
    rotation_condition = fixed_campaign_conditions()[0]
    translation_condition = fixed_campaign_conditions()[24]
    rotation = make_trial_result(
        rotation_condition.spec,
        measurements(reference=0.12, encoder_angle=0.10, imu_angle=0.11),
        valid=True,
        manual_reference={"type": "iphone_compass_rotation"})
    translation = make_trial_result(
        translation_condition.spec,
        measurements(reference=1.1, encoder_distance=1.0, encoder_angle=0.01, imu_angle=0.02),
        valid=True,
        manual_reference={"type": "manual_wall_distance_translation",
                          "final_heading_deviation_deg": 0.0})
    attempts = (
        attempt_record("rot", rotation_condition, 1, rotation),
        attempt_record("trans", translation_condition, 1, translation),
    )

    analysis = covariance_analysis(attempts)

    assert analysis["encoder_planar_translation_residual_m"]["samples"] == pytest.approx([-0.1])
    assert "roll" in analysis["unsupported_dimensions"]
    assert "pitch" in analysis["unsupported_dimensions"]
    assert "3D IMU covariance axes not observed by planar yaw experiments" in (
        analysis["unsupported_dimensions"])


def test_attempt_record_keeps_rotation_and_translation_residuals_separate():
    rotation_condition = fixed_campaign_conditions()[0]
    result = make_trial_result(
        rotation_condition.spec,
        measurements(reference=math.radians(-24.0), encoder_angle=math.radians(-23.0),
                     imu_angle=math.radians(-25.0)),
        valid=True,
        manual_reference={"initial_heading_raw": "359", "final_heading_raw": "23"})

    record = attempt_record("attempt", rotation_condition, 1, result)

    assert record["command_error_rad"] is not None
    assert record["encoder_error_deg"] == pytest.approx(1.0)
    assert record["imu_error_deg"] == pytest.approx(-1.0)
    assert record["manual_reference"]["initial_heading_raw"] == "359"


def test_attempt_record_counts_stationarity_and_diagnostics_from_samples():
    condition = fixed_campaign_conditions()[24]
    result = make_trial_result(
        condition.spec,
        measurements(reference=1.0),
        valid=True,
        manual_reference={"type": "manual_wall_distance_translation"})
    samples = SimpleNamespace(
        stationarity=("stationary", "confirmed"),
        diagnostics=("diagnostic",),
        ignored_diagnostics=("ignored",))

    record = attempt_record("attempt", condition, 1, result, samples)

    assert record["stationarity_record_count"] == 2
    assert record["diagnostic_sample_count"] == 1
    assert record["ignored_diagnostic_sample_count"] == 1
    assert record["command_start_timestamp_s"] is None
    assert record["source_topics"]["imu"] == "/imu/data"


def test_attempt_record_preserves_translation_heading_comparison_values():
    condition = fixed_campaign_conditions(mode="calibration")[16]
    result = make_trial_result(
        condition.spec,
        measurements(reference=1.0, encoder_angle=math.radians(1.2),
                     imu_angle=math.radians(2.0)),
        valid=True,
        manual_reference={"type": "manual_wall_distance_translation",
                          "final_heading_deviation_deg": 1.8})

    record = attempt_record("translation", condition, 1, result)

    assert record["compass_heading_rad"] == pytest.approx(math.radians(1.8))
    assert record["encoder_heading_deg"] == pytest.approx(1.2)
    assert record["encoder_heading_error_deg"] == pytest.approx(-0.6)
    assert record["encoder_absolute_heading_error_rad"] == pytest.approx(
        math.radians(0.6))
    assert record["imu_heading_error_deg"] == pytest.approx(0.2)
    assert record["encoder_absolute_error_vs_manual_reference_m"] == pytest.approx(0.0)
    assert record["encoder_percent_error_vs_manual_reference"] == pytest.approx(0.0)


def test_attempt_record_translation_distance_uses_magnitude_and_keeps_raw_sign():
    condition = fixed_campaign_conditions(mode="calibration")[24]
    result = make_trial_result(
        condition.spec,
        measurements(reference=-1.46, encoder_distance=1.493176),
        valid=True,
        manual_reference={"type": "manual_wall_distance_translation"})

    record = attempt_record("negative-distance", condition, 1, result)

    assert record["raw_manual_physical_reference_m"] == pytest.approx(-1.46)
    assert record["reference_distance_magnitude_m"] == pytest.approx(1.46)
    assert record["encoder_error_vs_manual_reference_m"] == pytest.approx(0.033176)
    assert record["encoder_percent_error_vs_manual_reference"] == pytest.approx(
        0.033176 / 1.46 * 100.0)


def test_validation_comparison_reports_direction_asymmetry_improvement():
    forward = fixed_campaign_conditions()[24]
    backward = fixed_campaign_conditions()[36]

    def record(condition, attempt_id, reference, encoder):
        result = make_trial_result(
            condition.spec,
            measurements(reference=reference, encoder_distance=encoder),
            valid=True,
            manual_reference={"type": "manual_wall_distance_translation"})
        return attempt_record(attempt_id, condition, 1, result)

    baseline = (
        record(forward, "bf", 1.0, 0.8),
        record(backward, "bb", 1.0, 1.0),
    )
    validation = (
        record(forward, "vf", 1.0, 0.95),
        record(backward, "vb", 1.0, 1.0),
    )

    comparison = validation_comparison(baseline, validation)

    asymmetry = comparison["direction_asymmetry"]["improvement"]
    assert asymmetry[
        "forward_vs_backward:encoder_distance_error_m"][
            "direction_asymmetry_improved"] is True


def test_covariance_characterization_uses_magnitude_and_ignores_translation_heading():
    assert abs(-1.25) - abs(-1.2) == pytest.approx(0.05)
    assert _wrapped_delta(math.radians(-179.0), math.radians(179.0)) == pytest.approx(
        math.radians(2.0))


def test_covariance_rejects_mixed_radius_campaign(tmp_path):
    campaign = tmp_path / "campaign"
    campaign.mkdir()
    (campaign / "metadata.json").write_text(json.dumps({"geometry": EXPECTED_GEOMETRY}))
    (campaign / "campaign_manifest.json").write_text(json.dumps({
        "mode": "calibration",
        "progress": {"complete": True, "completed_valid_repetitions": 160,
                      "planned_valid_repetitions": 160},
    }))
    (campaign / "campaign_attempts.json").write_text(json.dumps([
        {"wheel_radius_m": 0.087}, {"wheel_radius_m": 0.085}]))
    with pytest.raises(ValueError, match="mixed or unexpected"):
        validate_completed_campaign(campaign)


def test_completed_campaign_covariance_is_deterministic_when_evidence_is_present():
    campaign = Path(__file__).parents[1] / "validation_evidence" / \
        "odometry-validation-20260820T134021Z"
    if not (campaign / "campaign_attempts.json").exists():
        pytest.skip("local campaign evidence is not part of source distributions")
    first = characterize_campaign(campaign)
    second = characterize_campaign(campaign)
    assert first == second
    assert first["trial_counts"] == {
        "valid": 160, "translation": 80, "rotation": 80,
        "retained_invalid_or_retried": 9}
    assert len(first["pose_covariance"]) == 6
    assert len(first["twist_covariance"]) == 6
    assert first["outliers"]["translation"]
    assert first["selected_covariance"]["pose_x"] > 0.0
