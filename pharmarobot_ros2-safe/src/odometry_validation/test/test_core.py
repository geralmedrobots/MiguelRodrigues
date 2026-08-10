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

from dataclasses import asdict
import json
from io import BytesIO
from io import StringIO
import math
from threading import Event
import time

import pytest

from odometry_validation.core import DiagnosticSample
from odometry_validation.core import CommandSample
from odometry_validation.core import ControlledStopError
from odometry_validation.core import EmergencyStopController
from odometry_validation.core import EmergencyCleanupOnce
from odometry_validation.core import EmergencyStopCleanupError
from odometry_validation.core import EmergencyStopError
from odometry_validation.core import EvidenceWriter
from odometry_validation.core import GeometryConfig
from odometry_validation.core import ImuSample
from odometry_validation.core import InteractiveLimits
from odometry_validation.core import InteractiveTrialMenu
from odometry_validation.core import OperatorInterface
from odometry_validation.core import OdomSample
from odometry_validation.core import ProcessedImuSample
from odometry_validation.core import ResponsiveOperatorInput
from odometry_validation.core import StationarityAssessment
from odometry_validation.core import StationaritySample
from odometry_validation.core import TerminalLineReader
from odometry_validation.core import TrialMeasurements
from odometry_validation.core import TrialSamples
from odometry_validation.core import TrialSpec
from odometry_validation.core import ValidationError
from odometry_validation.core import WheelTickSample
from odometry_validation.core import compass_rotation_radians
from odometry_validation.core import build_measurements
from odometry_validation.core import build_trial_report
from odometry_validation.core import diagnostic_level_to_int
from odometry_validation.core import expected_final_compass_heading_deg
from odometry_validation.core import generate_rotation_trials
from odometry_validation.core import generate_translation_trials
from odometry_validation.core import integrate_imu_angle
from odometry_validation.core import imu_samples_in_motion_window
from odometry_validation.core import make_trial_result
from odometry_validation.core import merge_trial_samples
from odometry_validation.core import run_trial_until_accepted
from odometry_validation.core import run_with_emergency_stop
from odometry_validation.core import wheel_tick_measurements
from odometry_validation.core import odometry_yaw_change
from odometry_validation.core import percentage_error
from odometry_validation.core import render_trial_report


class ScriptedInput:
    """Small prompt double that exercises menu retries without a terminal."""

    def __init__(self, answers):
        self.answers = iter(answers)
        self.notifications = []

    def read_text(self, _prompt):
        return next(self.answers)

    def read_float(self, prompt):
        return float(self.read_text(prompt))

    def notify(self, message):
        self.notifications.append(message)


def empty_measurements():
    return TrialMeasurements(
        encoder_distance_m=0.0,
        encoder_angle_rad=0.0,
        left_wheel_distance_m=0.0,
        right_wheel_distance_m=0.0,
        odometry_distance_m=0.0,
        odometry_angle_rad=0.0,
        imu_angle_rad=0.0,
        physical_measurement=0.0,
        commanded_distance_m=0.0,
        commanded_angle_rad=0.0)


class FakeClock:
    def __init__(self):
        self.now_s = 0.0
        self.sleep_calls = []

    def monotonic(self):
        return self.now_s

    def sleep(self, duration_s):
        self.sleep_calls.append(duration_s)
        self.now_s += duration_s


@pytest.mark.parametrize(
    ("value", "expected"),
    ((b"\x00", 0), (b"\x01", 1), (b"\x02", 2), (0, 0), (1, 1), (2, 2)))
def test_diagnostic_level_accepts_ros_byte_and_integer_values(value, expected):
    assert diagnostic_level_to_int(value) == expected


def test_default_rotation_trial_matrix_includes_all_velocities_durations_and_directions():
    trials = generate_rotation_trials()

    assert len(trials) == 3 * 7 * 2
    assert trials[0].movement_type == "rotation"
    assert trials[0].direction == "cw"
    assert trials[0].angular_z == pytest.approx(-0.30)
    assert trials[1].direction == "ccw"
    assert trials[1].angular_z == pytest.approx(0.30)
    assert trials[-1].velocity == pytest.approx(0.50)
    assert trials[-1].duration_s == pytest.approx(10.0)


def test_translation_trial_matrix_supports_direction_filtering():
    trials = generate_translation_trials(
        velocities=(0.1, 0.2),
        durations=(2.0,),
        include_forward=False,
        include_backward=True)

    assert [trial.direction for trial in trials] == ["backward", "backward"]
    assert [trial.linear_x for trial in trials] == pytest.approx([-0.1, -0.2])


def test_encoder_distance_and_angle_use_production_per_sample_deltas():
    geometry = GeometryConfig(
        wheel_radius_m=1.0 / (2.0 * math.pi),
        track_width_m=0.5,
        encoder_ticks_per_revolution=100)
    samples = (
        WheelTickSample(1.0, left_ticks=40, right_ticks=90),
        WheelTickSample(1.1, left_ticks=60, right_ticks=110),
    )

    distance, angle, left, right = wheel_tick_measurements(samples, geometry)

    assert left == pytest.approx(1.0)
    assert right == pytest.approx(2.0)
    assert distance == pytest.approx(1.5)
    assert angle == pytest.approx(2.0)


def test_encoder_distance_supports_explicit_cumulative_counter_contract():
    geometry = GeometryConfig(
        wheel_radius_m=1.0 / (2.0 * math.pi),
        track_width_m=0.5,
        encoder_ticks_per_revolution=100)
    samples = (
        WheelTickSample(1.0, left_ticks=40, right_ticks=90),
        WheelTickSample(1.1, left_ticks=60, right_ticks=110),
    )

    distance, angle, left, right = wheel_tick_measurements(
        samples, geometry, semantics="cumulative")

    assert left == pytest.approx(0.2)
    assert right == pytest.approx(0.2)
    assert distance == pytest.approx(0.2)
    assert angle == pytest.approx(0.0)


def test_imu_integration_is_timestamp_aware_and_bias_corrected():
    samples = (
        ImuSample(10.0, 0.12),
        ImuSample(10.5, 0.12),
        ImuSample(12.0, 0.12),
    )

    angle, processed = integrate_imu_angle(samples, bias_rad_s=0.02)

    assert angle == pytest.approx(0.2)
    assert processed[-1].integrated_angle_rad == pytest.approx(0.2)
    assert processed[-1].corrected_angular_velocity_z_rad_s == pytest.approx(0.1)


def test_imu_motion_window_excludes_operator_wait_and_preserves_raw_samples():
    geometry = GeometryConfig(0.1, 0.5, 100)
    spec = TrialSpec("rot-001-ccw", "rotation", 0.3, 1.0, "ccw")
    raw = TrialSamples(imu=(
        ImuSample(0.0, 0.4, "before_motion"),
        ImuSample(1.0, 0.3, "during_motion"),
        ImuSample(2.0, 0.3, "during_motion"),
        ImuSample(3.0, 0.3, "after_motion"),
        ImuSample(4.0, 0.4, "after_motion"),
    ))

    measurements, enriched = build_measurements(
        spec, raw, geometry, None, command_start_timestamp_s=1.0,
        command_end_timestamp_s=2.0,
        stationary_confirmation_timestamp_s=3.0)

    assert measurements.commanded_window_imu_angle_rad == pytest.approx(0.3)
    assert measurements.settling_imu_angle_rad == pytest.approx(0.3)
    assert measurements.imu_angle_rad == pytest.approx(0.6)
    assert measurements.total_imu_sample_count == 3
    assert len(enriched.imu) == 5
    assert len(enriched.processed_imu) == 3


def test_imu_physical_motion_window_captures_large_coasting_rotation():
    geometry = GeometryConfig(0.1, 0.5, 100)
    spec = TrialSpec("rot-001-ccw", "rotation", 0.3, 1.0, "ccw")
    raw = TrialSamples(imu=(
        ImuSample(1.0, 0.3), ImuSample(2.0, 0.3),
        ImuSample(3.0, 0.6), ImuSample(4.0, 0.0),
    ))
    measurements, _ = build_measurements(
        spec, raw, geometry, None, command_start_timestamp_s=1.0,
        command_end_timestamp_s=2.0,
        stationary_confirmation_timestamp_s=4.0)

    assert measurements.commanded_window_imu_angle_rad == pytest.approx(0.3)
    assert measurements.settling_imu_angle_rad == pytest.approx(0.75)
    assert measurements.imu_angle_rad == pytest.approx(1.05)
    assert measurements.imu_angle_rad == pytest.approx(
        measurements.commanded_window_imu_angle_rad +
        measurements.settling_imu_angle_rad)


def test_imu_components_sum_when_command_end_brackets_samples():
    raw = TrialSamples(imu=(
        ImuSample(1.0, 0.3), ImuSample(1.95, 0.3),
        ImuSample(2.05, 0.5), ImuSample(3.0, 0.0),
    ))
    measurements, _ = build_measurements(
        TrialSpec("rot", "rotation", 0.3, 1.0, "ccw"), raw,
        GeometryConfig(0.1, 0.5, 100), None,
        command_start_timestamp_s=1.0, command_end_timestamp_s=2.0,
        stationary_confirmation_timestamp_s=3.0,
        imu_boundary_tolerance_s=0.1)

    assert measurements.imu_angle_rad == pytest.approx(
        measurements.commanded_window_imu_angle_rad +
        measurements.settling_imu_angle_rad)


def test_imu_physical_motion_window_allows_immediate_stationary_stop():
    geometry = GeometryConfig(0.1, 0.5, 100)
    raw = TrialSamples(imu=(ImuSample(1.0, 0.3), ImuSample(2.0, 0.3)))
    measurements, _ = build_measurements(
        TrialSpec("rot", "rotation", 0.3, 1.0, "ccw"), raw, geometry,
        None, command_start_timestamp_s=1.0, command_end_timestamp_s=2.0,
        stationary_confirmation_timestamp_s=2.0)

    assert measurements.commanded_window_imu_angle_rad == pytest.approx(0.3)
    assert measurements.settling_imu_angle_rad == pytest.approx(0.0)
    assert measurements.imu_angle_rad == pytest.approx(0.3)
    assert measurements.settling_imu_sample_count == 0


def test_imu_physical_motion_window_fails_without_stationary_confirmation():
    with pytest.raises(ValidationError, match="complete physical-motion"):
        build_measurements(
            TrialSpec("rot", "rotation", 0.3, 1.0, "ccw"), TrialSamples(),
            GeometryConfig(0.1, 0.5, 100), None,
            command_start_timestamp_s=1.0, command_end_timestamp_s=2.0)


def test_imu_physical_motion_window_fails_with_insufficient_settling_samples():
    with pytest.raises(ValidationError, match="insufficient primary"):
        build_measurements(
            TrialSpec("rot", "rotation", 0.3, 1.0, "ccw"),
            TrialSamples(imu=(ImuSample(1.0, 0.3), ImuSample(2.0, 0.3))),
            GeometryConfig(0.1, 0.5, 100), None,
            command_start_timestamp_s=1.0, command_end_timestamp_s=2.0,
            stationary_confirmation_timestamp_s=3.0)


def test_imu_physical_motion_window_fails_when_boundary_gap_exceeds_limit():
    with pytest.raises(ValidationError, match="do not cover"):
        build_measurements(
            TrialSpec("rot", "rotation", 0.3, 1.0, "ccw"),
            TrialSamples(imu=(
                ImuSample(1.2, 0.3), ImuSample(1.8, 0.3),
                ImuSample(2.2, 0.3), ImuSample(2.8, 0.3))),
            GeometryConfig(0.1, 0.5, 100), None,
            command_start_timestamp_s=1.0, command_end_timestamp_s=2.0,
            stationary_confirmation_timestamp_s=3.0,
            imu_boundary_tolerance_s=0.1)


def test_imu_motion_window_fails_closed_with_insufficient_samples():
    with pytest.raises(ValidationError, match="insufficient primary IMU samples"):
        imu_samples_in_motion_window((ImuSample(1.0, 0.3),), 1.0, 2.0)


def test_imu_motion_window_uses_primary_source_and_requires_coverage():
    samples = (
        ImuSample(1.0, 0.3, source_topic="/imu/data"),
        ImuSample(1.5, 9.0, source_topic="/imu/d455/data_raw"),
        ImuSample(2.0, 0.3, source_topic="/imu/data"),
    )

    selected = imu_samples_in_motion_window(
        samples, 1.0, 2.0, max_boundary_gap_s=0.01)

    assert selected == (samples[0], samples[2])
    with pytest.raises(ValidationError, match="do not cover"):
        imu_samples_in_motion_window(
            samples, 0.5, 2.0, max_boundary_gap_s=0.1)


def test_total_imu_window_does_not_mix_raw_source_at_matching_timestamps():
    raw = TrialSamples(imu=(
        ImuSample(1.0, 0.3, source_topic="/imu/data"),
        ImuSample(1.0, 9.0, source_topic="/imu/d455/data_raw"),
        ImuSample(2.0, 0.3, source_topic="/imu/data"),
        ImuSample(2.0, 9.0, source_topic="/imu/d455/data_raw"),
    ))
    measurements, enriched = build_measurements(
        TrialSpec("rot", "rotation", 0.3, 1.0, "ccw"), raw,
        GeometryConfig(0.1, 0.5, 100), None,
        command_start_timestamp_s=1.0, command_end_timestamp_s=2.0,
        stationary_confirmation_timestamp_s=2.0)

    assert measurements.imu_angle_rad == pytest.approx(0.3)
    assert all(sample.timestamp_s in (1.0, 2.0) for sample in enriched.imu)
    assert len(enriched.imu) == 4
    assert len(enriched.processed_imu) == 2


def test_compass_wraparound_converts_to_ros_yaw_sign():
    clockwise = compass_rotation_radians(350.0, 10.0)
    counter_clockwise = compass_rotation_radians(10.0, 350.0)

    assert clockwise == pytest.approx(math.radians(-20.0))
    assert counter_clockwise == pytest.approx(math.radians(20.0))


def test_theoretical_compass_heading_wraps_in_compass_direction():
    assert expected_final_compass_heading_deg(350.0, math.radians(20.0)) == pytest.approx(330.0)
    assert expected_final_compass_heading_deg(10.0, math.radians(-20.0)) == pytest.approx(30.0)


def test_odometry_yaw_change_unwraps_the_complete_interval():
    samples = (
        OdomSample(1.0, 0.0, 0.0, math.radians(170.0), 0.0, 0.0),
        OdomSample(2.0, 0.0, 0.0, math.radians(-170.0), 0.0, 0.0),
        OdomSample(3.0, 0.0, 0.0, math.radians(-150.0), 0.0, 0.0),
    )
    assert odometry_yaw_change(samples) == pytest.approx(math.radians(40.0))


def test_percentage_error_is_unavailable_for_zero_reference():
    assert percentage_error(1.0, 0.0) is None
    assert percentage_error(1.5, 1.0) == pytest.approx(50.0)


def test_rotation_report_captures_complete_interval_and_schema():
    geometry = GeometryConfig(1.0 / (2.0 * math.pi), 0.5, 100)
    spec = TrialSpec("rot-001-ccw", "rotation", 0.5, 2.0, "ccw")
    raw = TrialSamples(
        wheel_ticks=(WheelTickSample(1.0, 25, 75),),
        imu=(ImuSample(1.0, 0.6), ImuSample(3.0, 0.6),
             ImuSample(4.0, 0.6)),
        odom=(
            OdomSample(1.0, 0.0, 0.0, math.radians(170), 0.0, 0.0),
            OdomSample(2.0, 0.0, 0.0, math.radians(-170), 0.0, 0.0),
            OdomSample(3.0, 0.0, 0.0, math.radians(-150), 0.0, 0.0),
        ))
    measurements, samples = build_measurements(
        spec, raw, geometry, math.radians(40), 0.1,
        command_start_timestamp_s=1.0,
        command_end_timestamp_s=3.0,
        stationary_confirmation_timestamp_s=4.0)
    result = make_trial_result(
        spec, measurements, initial_compass_heading_deg=350.0,
        final_compass_heading_deg=310.0)

    report = build_trial_report(result, samples, geometry, imu_bias_rad_s=0.1)

    assert report["schema_version"] == 1
    assert report["theoretical"]["expected_angle_rad"] == pytest.approx(1.0)
    assert report["theoretical"]["expected_final_compass_heading_deg"] == (
        pytest.approx(292.7042204869))
    assert report["encoder"]["left_tick_total"] == 25
    assert report["encoder"]["right_tick_total"] == 75
    assert report["odometry"]["unwrapped_angle_deg"] == pytest.approx(40.0)
    assert report["imu"]["commanded_window_interval_s"] == pytest.approx(2.0)
    assert report["imu"]["settling_interval_s"] == pytest.approx(1.0)
    assert report["imu"]["total_integration_interval_s"] == pytest.approx(3.0)
    assert report["imu"]["total_physical_motion_angle_rad"] == pytest.approx(1.5)
    assert report["summary_comparison"]["imu"]["value"] == pytest.approx(1.5)
    assert report["imu"]["source_topic"] == "/imu/data"
    assert report["imu"]["total_sample_count"] == 3
    assert "Encoder-based Rotation" in render_trial_report(report)
    assert "Summary Comparison" in render_trial_report(report)


def test_valid_trial_writes_matching_markdown_and_json_reports(tmp_path):
    geometry = GeometryConfig(0.1, 0.5, 100)
    spec = TrialSpec("rot-001-ccw", "rotation", 0.2, 1.0, "ccw")
    measurements = TrialMeasurements(
        encoder_distance_m=0.0,
        encoder_angle_rad=0.2,
        left_wheel_distance_m=-0.05,
        right_wheel_distance_m=0.05,
        odometry_distance_m=0.0,
        odometry_angle_rad=0.2,
        imu_angle_rad=0.2,
        physical_measurement=0.2,
        commanded_distance_m=0.0,
        commanded_angle_rad=0.2)
    result = make_trial_result(
        spec, measurements, valid=True,
        initial_compass_heading_deg=350.0, final_compass_heading_deg=338.5)
    samples = TrialSamples(
        wheel_ticks=(WheelTickSample(1.0, -25, 25),),
        imu=(ImuSample(1.0, 0.2), ImuSample(2.0, 0.2)),
        processed_imu=(
            ProcessedImuSample(1.0, 0.2, 0.0, 0.2, 0.0),
            ProcessedImuSample(2.0, 0.2, 0.0, 0.2, 0.2)),
        odom=(
            OdomSample(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            OdomSample(2.0, 0.0, 0.0, 0.2, 0.0, 0.0)))
    writer = EvidenceWriter(tmp_path)
    writer.create({
        "geometry": asdict(geometry),
        "stationarity_thresholds": {"wheel_tick_semantics": "delta"},
        "imu_bias_rad_s": 0.0,
    })

    trial_dir = writer.write_trial(result, samples)

    report = json.loads((trial_dir / "report.json").read_text(encoding="utf-8"))
    markdown = (trial_dir / "report.md").read_text(encoding="utf-8")
    metadata = json.loads((trial_dir / "metadata.json").read_text(encoding="utf-8"))
    assert report["schema_version"] == 1
    assert markdown == render_trial_report(report)
    assert metadata["initial_compass_heading_deg"] == pytest.approx(350.0)
    assert metadata["final_compass_heading_deg"] == pytest.approx(338.5)
    writer.write_summary([result])
    campaign_summary = json.loads(
        (writer.directory / "campaign-summary.json").read_text(encoding="utf-8"))
    assert campaign_summary["recorded_trial_count"] == 1


def test_report_rejects_nonfinite_measurements_and_never_serializes_nan():
    geometry = GeometryConfig(0.1, 0.5, 100)
    spec = TrialSpec("rot-001-ccw", "rotation", 0.2, 1.0, "ccw")
    measurements = TrialMeasurements(
        encoder_distance_m=0.0,
        encoder_angle_rad=0.2,
        left_wheel_distance_m=0.0,
        right_wheel_distance_m=0.0,
        odometry_distance_m=0.0,
        odometry_angle_rad=float("nan"),
        imu_angle_rad=0.2,
        physical_measurement=None,
        commanded_distance_m=0.0,
        commanded_angle_rad=0.2)

    with pytest.raises(ValidationError, match="nonfinite"):
        build_trial_report(make_trial_result(spec, measurements), TrialSamples(), geometry)


def test_invalid_report_creates_no_partial_trial_and_failure_evidence_is_json_safe(
        tmp_path):
    geometry = GeometryConfig(0.1, 0.5, 100)
    spec = TrialSpec("rot-001-ccw", "rotation", 0.2, 1.0, "ccw")
    measurements = TrialMeasurements(
        encoder_distance_m=0.0,
        encoder_angle_rad=0.2,
        left_wheel_distance_m=0.0,
        right_wheel_distance_m=0.0,
        odometry_distance_m=0.0,
        odometry_angle_rad=float("nan"),
        imu_angle_rad=0.2,
        physical_measurement=None,
        commanded_distance_m=0.0,
        commanded_angle_rad=0.2)
    samples = TrialSamples(odom=(
        OdomSample(1.0, float("nan"), 0.0, 0.0, 0.0, 0.0),))
    writer = EvidenceWriter(tmp_path)
    campaign = writer.create({
        "geometry": asdict(geometry),
        "stationarity_thresholds": {"wheel_tick_semantics": "delta"},
    })

    with pytest.raises(ValidationError, match="nonfinite"):
        writer.write_trial(make_trial_result(spec, measurements, valid=True), samples)
    assert not list(campaign.glob("rot-001-ccw-attempt-*"))

    writer.write_failure([], ValidationError("malformed odometry"), samples)
    payload = json.loads((campaign / "failure.json").read_text(encoding="utf-8"))
    assert payload["samples"]["odom"][0]["x_m"] == {"nonfinite_float": "nan"}


@pytest.mark.parametrize(
    ("answers", "movement_type", "velocity", "duration"),
    (
        (("1", "yes"), "rotation", 0.3, 2.0),
        (("2", "yes"), "rotation", 0.3, 4.0),
        (("3", "yes"), "rotation", 0.45, 2.0),
        (("4", "backward", "0.2", "3", "yes"), "translation", 0.2, 3.0),
    ))
def test_interactive_menu_options_create_only_confirmed_trial(
        answers, movement_type, velocity, duration):
    source = ScriptedInput(answers)
    menu = InteractiveTrialMenu(
        source, InteractiveLimits(0.5, 10.0, 0.1, 1.0, 2.0, 10.0), lambda _text: None)
    prior = TrialSpec("rot-001-ccw", "rotation", 0.3, 2.0, "ccw")

    next_spec = menu.choose_next(prior, 2)

    assert next_spec is not None
    assert next_spec.movement_type == movement_type
    assert next_spec.velocity == pytest.approx(velocity)
    assert next_spec.duration_s == pytest.approx(duration)


def test_interactive_menu_option_five_finishes_without_a_trial():
    source = ScriptedInput(("5",))
    menu = InteractiveTrialMenu(
        source, InteractiveLimits(0.5, 10.0, 0.1, 1.0, 2.0, 10.0), lambda _text: None)
    prior = TrialSpec("rot-001-cw", "rotation", 0.3, 2.0, "cw")
    assert menu.choose_next(prior, 2) is None


def test_menu_rejects_invalid_input_and_over_limit_angular_velocity():
    source = ScriptedInput(("bad", "3", "0.8", "0.4", "yes"))
    menu = InteractiveTrialMenu(
        source, InteractiveLimits(0.5, 10.0, 0.1, 1.0, 2.0, 10.0), lambda _text: None)
    prior = TrialSpec("rot-001-cw", "rotation", 0.4, 2.0, "cw")

    next_spec = menu.choose_next(prior, 2)

    assert next_spec.velocity == pytest.approx(0.4)
    assert any("Invalid menu" in item for item in source.notifications)
    assert any("exceeds" in item for item in source.notifications)


def test_translation_menu_retries_outside_configured_ranges():
    source = ScriptedInput((
        "4", "sideways", "forward", "0.05", "0.2", "1", "2", "yes"))
    menu = InteractiveTrialMenu(
        source, InteractiveLimits(0.5, 10.0, 0.1, 1.0, 2.0, 10.0), lambda _text: None)
    prior = TrialSpec("rot-001-cw", "rotation", 0.3, 2.0, "cw")

    next_spec = menu.choose_next(prior, 2)

    assert next_spec.direction == "forward"
    assert next_spec.velocity == pytest.approx(0.2)
    assert next_spec.duration_s == pytest.approx(2.0)
    assert any("within" in item for item in source.notifications)


def test_menu_rejects_rotation_duration_doubling_past_safe_limit():
    source = ScriptedInput(("2", "5"))
    menu = InteractiveTrialMenu(
        source, InteractiveLimits(0.5, 10.0, 0.1, 1.0, 2.0, 10.0), lambda _text: None)
    prior = TrialSpec("rot-001-cw", "rotation", 0.3, 10.0, "cw")

    assert menu.choose_next(prior, 2) is None
    assert any("doubled rotation duration" in item for item in source.notifications)


@pytest.mark.parametrize(
    ("option", "prior"),
    (
        ("1", TrialSpec("rot-001-cw", "rotation", 0.6, 2.0, "cw")),
        ("2", TrialSpec("rot-001-cw", "rotation", 0.6, 2.0, "cw")),
        ("3", TrialSpec("rot-001-cw", "rotation", 0.3, 20.0, "cw")),
    ))
def test_menu_rejects_any_rotation_candidate_with_out_of_limit_inherited_value(
        option, prior):
    answers = (option, "0.4", "5") if option == "3" else (option, "5")
    source = ScriptedInput(answers)
    menu = InteractiveTrialMenu(
        source, InteractiveLimits(0.5, 10.0, 0.1, 1.0, 2.0, 10.0), lambda _text: None)

    assert menu.choose_next(prior, 2) is None
    assert any("Rotation velocity and duration" in item for item in source.notifications)


def test_menu_rejects_out_of_limit_translation_repeat_before_confirmation():
    source = ScriptedInput(("1", "5"))
    menu = InteractiveTrialMenu(
        source, InteractiveLimits(0.5, 10.0, 0.1, 1.0, 2.0, 10.0), lambda _text: None)
    prior = TrialSpec("line-001-forward", "translation", 1.1, 2.0, "forward")

    assert menu.choose_next(prior, 2) is None
    assert any("Translation direction" in item for item in source.notifications)


def test_validation_reruns_exact_same_trial_until_valid():
    spec = TrialSpec("rot-001-cw", "rotation", 0.3, 2.0, "cw")
    calls = []

    def execute_once(trial):
        calls.append(trial)
        return make_trial_result(trial, empty_measurements())

    answers = iter(("no", "slipped", "yes", "clean"))
    operator = OperatorInterface(lambda _prompt: next(answers))

    result = run_trial_until_accepted(spec, execute_once, operator)

    assert calls == [spec, spec]
    assert result.valid
    assert not result.skipped
    assert result.operator_notes == "clean"


def test_immediate_stationary_stop_returns_after_first_zero():
    publishes = []
    controller = EmergencyStopController(
        publish_zero=lambda: publishes.append("zero"),
        verify_safe_zero=lambda: True,
        verify_stationary=lambda: True,
        sleep=lambda _seconds: None)

    record = controller.stop(timeout_s=1.0, rate_hz=4.0)

    assert publishes == ["zero"]
    assert record["zero_publish_count"] == 1
    assert record["safe_zero"]
    assert record["stationary"]
    assert record["timeout_reason"] is None


@pytest.mark.parametrize(
    ("timeout_s", "rate_hz"),
    (
        (float("nan"), 20.0),
        (float("inf"), 20.0),
        (1.0, float("nan")),
        (1.0, float("inf")),
    ))
def test_stop_rejects_nonfinite_timing(timeout_s, rate_hz):
    controller = EmergencyStopController(
        publish_zero=lambda: pytest.fail("invalid timing must not publish"),
        verify_safe_zero=lambda: True,
        verify_stationary=lambda: True)

    with pytest.raises(ValueError, match="finite and positive"):
        controller.stop(timeout_s=timeout_s, rate_hz=rate_hz)


def test_residual_movement_settles_before_controlled_stop_timeout():
    publishes = []
    assessments = iter((False, False, True))
    records = []
    clock = FakeClock()
    controller = EmergencyStopController(
        publish_zero=lambda: publishes.append("zero"),
        verify_safe_zero=lambda: True,
        verify_stationary=lambda: next(assessments),
        sleep=clock.sleep,
        record_result=records.append,
        monotonic=clock.monotonic)

    record = controller.stop(
        timeout_s=0.5, rate_hz=10.0, mode="controlled")

    assert publishes == ["zero", "zero", "zero"]
    assert len(record["stationarity_assessments"]) == 3
    assert record["final_accepted_window"]["stationary"]
    assert record["time_from_first_zero_to_stationary_s"] >= 0.0
    assert record["timeout_reason"] is None
    assert records == [record]


def test_movement_that_never_settles_fails_only_at_timeout():
    publishes = []
    clock = FakeClock()
    controller = EmergencyStopController(
        publish_zero=lambda: publishes.append("zero"),
        verify_safe_zero=lambda: True,
        verify_stationary=lambda: False,
        sleep=clock.sleep,
        monotonic=clock.monotonic)

    with pytest.raises(ControlledStopError) as captured:
        controller.stop(
            timeout_s=0.5, rate_hz=10.0, mode="controlled")

    assert publishes == ["zero"] * 5
    assert len(captured.value.record["stationarity_assessments"]) == 5
    assert captured.value.record["final_accepted_window"] is None
    assert captured.value.record["time_from_first_zero_to_stationary_s"] is None
    assert "timeout expired after 0.500s" in (
        captured.value.record["timeout_reason"])
    assert "controlled stop verification failed" in str(captured.value)


def test_controlled_stop_clips_callback_service_to_actual_deadline():
    clock = FakeClock()
    controller = EmergencyStopController(
        publish_zero=lambda: None,
        verify_safe_zero=lambda: True,
        verify_stationary=lambda: False,
        sleep=clock.sleep,
        monotonic=clock.monotonic)

    with pytest.raises(ControlledStopError):
        controller.stop(
            timeout_s=0.25, rate_hz=2.0, mode="controlled")

    assert clock.sleep_calls == pytest.approx([0.25])
    assert clock.now_s == pytest.approx(0.25)


def test_controlled_stop_guard_fault_aborts_after_first_zero():
    publishes = []
    clock = FakeClock()

    def fail_guard():
        raise RuntimeError("unexpected controlled-stop fault")

    controller = EmergencyStopController(
        publish_zero=lambda: publishes.append("zero"),
        verify_safe_zero=lambda: pytest.fail("guard failure must abort first"),
        verify_stationary=lambda: pytest.fail("guard failure must abort first"),
        verify_stop_guards=fail_guard,
        sleep=clock.sleep,
        monotonic=clock.monotonic)

    with pytest.raises(RuntimeError, match="unexpected controlled-stop fault"):
        controller.stop(
            timeout_s=1.0, rate_hz=10.0, mode="controlled")

    assert publishes == ["zero"]


def test_emergency_stop_before_motion_skips_stationarity_but_verifies_safe_zero():
    publishes = []
    stationarity_calls = []
    controller = EmergencyStopController(
        publish_zero=lambda: publishes.append("zero"),
        verify_safe_zero=lambda: True,
        verify_stationary=lambda: stationarity_calls.append("called"),
        sleep=lambda _seconds: None,
        stationarity_required=lambda: False)

    record = controller.stop(timeout_s=0.4, rate_hz=5.0)

    assert publishes == ["zero"]
    assert stationarity_calls == []
    assert record["safe_zero"]
    assert not record["stationarity_required"]
    assert record["stationary"] is None


def test_emergency_stop_before_motion_still_fails_closed_without_safe_zero():
    clock = FakeClock()
    controller = EmergencyStopController(
        publish_zero=lambda: None,
        verify_safe_zero=lambda: False,
        verify_stationary=lambda: pytest.fail("stationarity must be skipped"),
        sleep=clock.sleep,
        stationarity_required=lambda: False,
        monotonic=clock.monotonic)

    with pytest.raises(EmergencyStopError, match="safe_zero.*False"):
        controller.stop(timeout_s=0.4, rate_hz=5.0)


def test_interruption_invokes_emergency_stop_and_reraises():
    publishes = []
    controller = EmergencyStopController(
        publish_zero=lambda: publishes.append("zero"),
        verify_safe_zero=lambda: True,
        verify_stationary=lambda: True,
        sleep=lambda _seconds: None)

    def interrupted():
        raise KeyboardInterrupt()

    with pytest.raises(KeyboardInterrupt):
        run_with_emergency_stop(
            interrupted,
            controller,
            timeout_s=0.5,
            rate_hz=4.0)

    assert len(publishes) == 1


@pytest.mark.parametrize(
    ("text", "expected"),
    (("0", 0.0), ("90", 90.0), ("180.5", 180.5)))
def test_responsive_operator_input_accepts_ascii_numbers(text, expected):
    notifications = []
    operator_input = ResponsiveOperatorInput(
        prompt=lambda _prompt: text,
        poll=lambda: None,
        notify=notifications.append,
        poll_interval_s=0.001)

    assert operator_input.read_float("heading: ") == pytest.approx(expected)
    assert notifications == []


def test_responsive_operator_input_retries_invalid_numeric_text():
    answers = iter(("not-a-number", "90"))
    notifications = []
    operator_input = ResponsiveOperatorInput(
        prompt=lambda _prompt: next(answers),
        poll=lambda: None,
        notify=notifications.append,
        poll_interval_s=0.001)

    assert operator_input.read_float("heading: ") == pytest.approx(90.0)
    assert notifications == ["Invalid numeric input. Please enter a number."]


def test_responsive_operator_input_retries_utf8_decode_error():
    notifications = []
    output = StringIO()
    terminal_reader = TerminalLineReader(
        BytesIO(b"0\xc2\n180.5\n"), output, encoding="utf-8")
    operator_input = ResponsiveOperatorInput(
        prompt=terminal_reader,
        poll=lambda: None,
        notify=notifications.append,
        poll_interval_s=0.001)

    assert operator_input.read_float("heading: ") == pytest.approx(180.5)
    assert output.getvalue() == "heading: heading: "
    assert notifications[0].startswith("Input encoding error:")


def test_incomplete_utf8_at_eof_retries_then_propagates_eof():
    notifications = []
    output = StringIO()
    terminal_reader = TerminalLineReader(
        BytesIO(b"0\xc2"), output, encoding="utf-8")
    operator_input = ResponsiveOperatorInput(
        prompt=terminal_reader,
        poll=lambda: None,
        notify=notifications.append,
        poll_interval_s=0.001)

    with pytest.raises(EOFError, match="EOF while reading operator input"):
        operator_input.read_float("heading: ")

    assert output.getvalue() == "heading: heading: "
    assert "unexpected end of data" in notifications[0]


def test_responsive_operator_input_polls_while_prompt_is_pending():
    release_prompt = Event()
    poll_count = []

    def prompt(_prompt):
        assert release_prompt.wait(timeout=1.0)
        return "90"

    def poll():
        poll_count.append("poll")
        if len(poll_count) == 3:
            release_prompt.set()

    operator_input = ResponsiveOperatorInput(
        prompt=prompt,
        poll=poll,
        notify=lambda _message: None,
        poll_interval_s=0.001)

    assert operator_input.read_float("heading: ") == pytest.approx(90.0)
    assert len(poll_count) >= 3


@pytest.mark.parametrize("error", (EOFError(), KeyboardInterrupt()))
def test_operator_eof_or_interrupt_invokes_zero_cleanup(error):
    publishes = []

    def prompt(_prompt):
        raise error

    operator_input = ResponsiveOperatorInput(
        prompt=prompt,
        poll=lambda: None,
        notify=lambda _message: None,
        poll_interval_s=0.001)
    controller = EmergencyStopController(
        publish_zero=lambda: publishes.append("zero"),
        verify_safe_zero=lambda: True,
        verify_stationary=lambda: True,
        sleep=lambda _seconds: None)

    with pytest.raises(type(error)):
        run_with_emergency_stop(
            lambda: operator_input.read_float("heading: "),
            controller,
            timeout_s=0.5,
            rate_hz=4.0)

    assert publishes == ["zero"]


def test_emergency_cleanup_once_prevents_duplicate_zero_cleanup():
    calls = []

    class FakeController:
        def stop(self, _timeout_s, _rate_hz):
            calls.append("stop")
            return {"safe_zero": True}

    cleanup = EmergencyCleanupOnce(FakeController())

    with pytest.raises(KeyboardInterrupt):
        run_with_emergency_stop(
            lambda: (_ for _ in ()).throw(KeyboardInterrupt()),
            cleanup, timeout_s=0.5, rate_hz=4.0)

    assert cleanup.stop(0.5, 4.0) == {"already_attempted": True}
    assert calls == ["stop"]


def test_main_thread_interrupt_during_pending_input_invokes_zero_cleanup():
    publishes = []
    release_prompt = Event()

    def prompt(_prompt):
        release_prompt.wait(timeout=1.0)
        return "0"

    def interrupted_poll():
        raise KeyboardInterrupt()

    operator_input = ResponsiveOperatorInput(
        prompt=prompt,
        poll=interrupted_poll,
        notify=lambda _message: None,
        poll_interval_s=0.001)
    controller = EmergencyStopController(
        publish_zero=lambda: publishes.append("zero"),
        verify_safe_zero=lambda: True,
        verify_stationary=lambda: True,
        sleep=lambda _seconds: None)

    try:
        with pytest.raises(KeyboardInterrupt):
            run_with_emergency_stop(
                lambda: operator_input.read_float("heading: "),
                controller,
                timeout_s=0.5,
                rate_hz=4.0)
    finally:
        release_prompt.set()

    assert publishes == ["zero"]


def test_cleanup_failure_evidence_preserves_both_tracebacks(tmp_path):
    clock = FakeClock()
    controller = EmergencyStopController(
        publish_zero=lambda: None,
        verify_safe_zero=lambda: False,
        verify_stationary=lambda: False,
        sleep=clock.sleep,
        monotonic=clock.monotonic)

    def primary_failure():
        raise UnicodeDecodeError(
            "utf-8", b"\xc2", 0, 1, "unexpected end of data")

    with pytest.raises(EmergencyStopCleanupError) as captured:
        run_with_emergency_stop(
            primary_failure, controller, timeout_s=0.5, rate_hz=4.0)

    writer = EvidenceWriter(tmp_path)
    evidence_dir = writer.create({"ignored_diagnostic_names": []})
    writer.write_failure([], captured.value, TrialSamples())
    payload = json.loads(
        (evidence_dir / "failure.json").read_text(encoding="utf-8"))

    assert "UnicodeDecodeError" in payload["primary_traceback"]
    assert "unexpected end of data" in payload["primary_traceback"]
    assert "EmergencyStopError" in payload["cleanup_traceback"]
    assert "Primary failure traceback" in payload["traceback"]
    assert "Zero-command cleanup failure traceback" in payload["traceback"]


def test_nested_cleanup_failures_preserve_every_traceback(tmp_path):
    class FailingCleanup:
        def __init__(self, message):
            self.message = message

        def stop(self, _timeout_s, _rate_hz):
            raise RuntimeError(self.message)

    def primary_failure():
        raise UnicodeDecodeError(
            "utf-8", b"\xc2", 0, 1, "unexpected end of data")

    def inner_action():
        return run_with_emergency_stop(
            primary_failure,
            FailingCleanup("first cleanup failed"),
            timeout_s=0.5,
            rate_hz=4.0)

    with pytest.raises(EmergencyStopCleanupError) as captured:
        run_with_emergency_stop(
            inner_action,
            FailingCleanup("second cleanup failed"),
            timeout_s=0.5,
            rate_hz=4.0)

    writer = EvidenceWriter(tmp_path)
    evidence_dir = writer.create({"ignored_diagnostic_names": []})
    writer.write_failure([], captured.value, TrialSamples())
    payload = json.loads(
        (evidence_dir / "failure.json").read_text(encoding="utf-8"))

    assert "UnicodeDecodeError" in payload["primary_traceback"]
    assert "first cleanup failed" in payload["primary_traceback"]
    assert "second cleanup failed" in payload["cleanup_traceback"]
    assert "first cleanup failed" in payload["traceback"]
    assert "second cleanup failed" in payload["traceback"]


def test_evidence_records_ignored_diagnostic_names_and_samples(tmp_path):
    ignored_name = "roboteq/channel_1_telemetry"
    ignored_sample = DiagnosticSample(
        timestamp_s=12.5,
        level=2,
        name=ignored_name,
        message="telemetry is stale")
    accepted = StationarityAssessment(
        stationary=True,
        reason="stationary",
        required_delta_samples=1,
        observed_delta_samples=1,
        tick_delta_tolerance=0,
        linear_velocity_tolerance_m_s=0.01,
        angular_velocity_tolerance_rad_s=0.02,
        first_zero_timestamp_s=12.0,
        wheel_tick_semantics="delta",
        safe_zero=True,
        tick_deltas_stationary=True,
        odom_twist_stationary=True,
        assessment_timestamp_s=12.5,
        elapsed_since_first_zero_s=0.5)
    samples = TrialSamples(
        diagnostics=(ignored_sample,),
        ignored_diagnostics=(ignored_sample,),
        stationarity=(accepted,))
    spec = TrialSpec("rot-001-ccw", "rotation", 0.3, 1.0, "ccw")
    result = make_trial_result(spec, empty_measurements())
    writer = EvidenceWriter(tmp_path)

    campaign_dir = writer.create({
        "ignored_diagnostic_names": [ignored_name],
    })
    trial_dir = writer.write_trial(result, samples)
    writer.write_summary([result])

    campaign_metadata = json.loads(
        (campaign_dir / "metadata.json").read_text(encoding="utf-8"))
    trial_metadata = json.loads(
        (trial_dir / "metadata.json").read_text(encoding="utf-8"))
    report = (campaign_dir / "report.md").read_text(encoding="utf-8")
    stationarity = json.loads(
        (trial_dir / "stationarity.json").read_text(encoding="utf-8"))

    assert campaign_metadata["ignored_diagnostic_names"] == [ignored_name]
    assert trial_metadata["ignored_diagnostic_names"] == [ignored_name]
    assert trial_metadata["ignored_diagnostic_samples"] == [{
        "timestamp_s": 12.5,
        "level": 2,
        "name": ignored_name,
        "message": "telemetry is stale",
    }]
    assert ignored_name in report
    assert "telemetry is stale" in report
    assert len(stationarity["assessments"]) == 1
    assert stationarity["final_accepted_window"]["stationary"]
    assert stationarity["final_accepted_window"][
        "elapsed_since_first_zero_s"] == pytest.approx(0.5)
    assert (trial_dir / "ignored_diagnostics.csv").read_text(
        encoding="utf-8").count(ignored_name) == 1


def test_failure_evidence_records_raw_samples_stationarity_and_thresholds(
        tmp_path):
    stationarity_sample = StationaritySample(
        previous_timestamp_s=2.0,
        timestamp_s=2.1,
        previous_left_ticks=100,
        previous_right_ticks=-50,
        left_ticks=101,
        right_ticks=-50,
        left_delta_ticks=1,
        right_delta_ticks=0)
    odom = OdomSample(
        timestamp_s=2.1,
        x_m=0.2,
        y_m=0.0,
        yaw_rad=0.3,
        linear_x_m_s=0.02,
        angular_z_rad_s=0.03,
        phase="after_motion")
    assessment = StationarityAssessment(
        stationary=False,
        reason="encoder ticks changed within stationarity window",
        required_delta_samples=1,
        observed_delta_samples=1,
        tick_delta_tolerance=0,
        linear_velocity_tolerance_m_s=0.01,
        angular_velocity_tolerance_rad_s=0.02,
        first_zero_timestamp_s=2.0,
        wheel_tick_semantics="cumulative",
        safe_zero=True,
        tick_deltas_stationary=False,
        odom_twist_stationary=False,
        stationarity_samples=(stationarity_sample,),
        odom_samples=(odom,))
    samples = TrialSamples(
        wheel_ticks=(
            WheelTickSample(2.0, 100, -50, "during_motion"),
            WheelTickSample(2.1, 101, -50, "after_motion"),
        ),
        imu=(ImuSample(2.05, 0.12, "after_motion"),),
        odom=(odom,),
        commands=(
            CommandSample(2.0, "/cmd_vel/test", 0.0, 0.3, "during_motion"),
            CommandSample(2.1, "/cmd_vel/safe", 0.0, 0.0, "after_motion"),
        ),
        stationarity=(assessment,))
    record = {
        "zero_publish_count": 20,
        "safe_zero": True,
        "stationary": False,
        "stationarity": asdict(assessment),
    }
    error = EmergencyStopError(
        f"emergency stop verification failed: {record}", record)
    writer = EvidenceWriter(tmp_path)
    evidence_dir = writer.create({"ignored_diagnostic_names": []})

    writer.write_failure(
        [],
        error,
        samples,
        failure_context={
            "zero_publish_count": 20,
            "stationarity_thresholds": {
                "required_delta_samples": 1,
                "tick_delta_tolerance": 0,
            },
            "final_stationarity_reason": assessment.reason,
        })

    failure = json.loads(
        (evidence_dir / "failure.json").read_text(encoding="utf-8"))
    raw = json.loads(
        (evidence_dir / "failure_samples.json").read_text(encoding="utf-8"))
    assert failure["final_emergency_stop_record"]["zero_publish_count"] == 20
    assert failure["failure_context"]["final_stationarity_reason"] == (
        "encoder ticks changed within stationarity window")
    assert raw["wheel_ticks"][0]["phase"] == "during_motion"
    assert raw["odom"][0]["linear_x_m_s"] == pytest.approx(0.02)
    assert {sample["topic"] for sample in raw["commands"]} == {
        "/cmd_vel/test", "/cmd_vel/safe"}
    assert raw["imu"][0]["angular_velocity_z_rad_s"] == pytest.approx(0.12)
    assert raw["stationarity"][0]["tick_delta_tolerance"] == 0
    assert (evidence_dir / "wheel_ticks.csv").is_file()
    assert (evidence_dir / "odometry.csv").is_file()
    assert "left_delta_ticks" in (
        evidence_dir / "stationarity_window.csv").read_text(encoding="utf-8")


def test_trial_sample_merge_keeps_enriched_and_later_callback_samples():
    earlier = TrialSamples(
        wheel_ticks=(WheelTickSample(1.0, 10, 20, "during_motion"),),
        processed_imu=(ProcessedImuSample(1.0, 0.2, 0.0, 0.2, 0.0),))
    later = TrialSamples(
        wheel_ticks=(
            WheelTickSample(1.0, 10, 20, "during_motion"),
            WheelTickSample(1.1, 10, 20, "after_motion"),
        ),
        commands=(
            CommandSample(1.1, "/cmd_vel/test", 0.0, 0.0, "after_motion"),))

    merged = merge_trial_samples(earlier, later)

    assert len(merged.wheel_ticks) == 2
    assert len(merged.processed_imu) == 1
    assert merged.commands[0].phase == "after_motion"


def test_trial_sample_merge_preserves_same_timestamp_sources_and_is_idempotent():
    samples = TrialSamples(imu=(
        ImuSample(2.0, 0.2, "during_motion", "/imu/data"),
        ImuSample(1.0, 0.1, "during_motion", "/imu/d455/data_raw"),
        ImuSample(1.0, 0.3, "during_motion", "/imu/data"),
    ))

    merged = merge_trial_samples(samples, samples)

    assert len(merged.imu) == 3
    assert [sample.timestamp_s for sample in merged.imu] == [1.0, 1.0, 2.0]
    assert {sample.source_topic for sample in merged.imu} == {
        "/imu/data", "/imu/d455/data_raw"}
    assert merge_trial_samples(merged) == merged


def test_large_trial_sample_merge_is_linear_and_bounded_in_time():
    count = 20_000
    first = TrialSamples(wheel_ticks=tuple(
        WheelTickSample(float(index), index, -index, "during_motion")
        for index in range(count)))
    second = TrialSamples(wheel_ticks=tuple(
        WheelTickSample(float(index), index, -index, "during_motion")
        for index in range(count // 2, count + count // 2)))

    started = time.monotonic()
    merged = merge_trial_samples(first, second)
    elapsed = time.monotonic() - started

    assert len(merged.wheel_ticks) == count + count // 2
    assert elapsed < 2.0


def test_cleanup_skips_publication_when_ros_context_is_invalid():
    publishes = []
    records = []
    controller = EmergencyStopController(
        publish_zero=lambda: publishes.append("zero"),
        verify_safe_zero=lambda: pytest.fail("invalid context must not verify"),
        verify_stationary=lambda: pytest.fail("invalid context must not verify"),
        cleanup_context_valid=lambda: False,
        confirmed_safe_state=lambda: {
            "safe_zero": True, "stationary": True},
        record_result=records.append)

    record = controller.stop(1.0, 20.0)

    assert publishes == []
    assert record["active_zero_verification_possible"] is False
    assert record["ros_context_valid"] is False
    assert record["safe_zero"] is True
    assert records == [record]


def test_second_cleanup_request_is_recorded_without_repeating_stop():
    calls = []

    class Controller:
        def stop(self, _timeout_s, _rate_hz):
            calls.append("stop")
            return {"safe_zero": True}

    cleanup = EmergencyCleanupOnce(Controller())
    assert cleanup.stop(1.0, 20.0) == {"safe_zero": True}
    second = cleanup.stop(1.0, 20.0)

    assert calls == ["stop"]
    assert second == {"already_attempted": True}
    assert cleanup.second_interrupt is True
