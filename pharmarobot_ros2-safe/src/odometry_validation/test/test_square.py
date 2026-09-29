# Copyright 2026 Medrobots Engineering

import csv
from dataclasses import asdict
import json
import math

import pytest

from odometry_validation.core import EmergencyCleanupOnce
from odometry_validation.core import EmergencyStopController
from odometry_validation.core import GeometryConfig
from odometry_validation.core import ImuSample
from odometry_validation.core import ValidationError
from odometry_validation.core import WheelTickSample
from odometry_validation.core import run_with_emergency_stop
from odometry_validation.square import HeadingFusion
from odometry_validation.square import SquareConfig
from odometry_validation.square import ClosedLoopYawConfig
from odometry_validation.square import ClosedLoopYawController
from odometry_validation.square import apply_continuous_final_metrics
from odometry_validation.square import derive_heading_fusion
from odometry_validation.square import encoder_relative_yaw
from odometry_validation.square import fuse_closed_loop_yaw
from odometry_validation.square import graph_contract_missing
from odometry_validation.square import imu_coverage
from odometry_validation.square import imu_relative_yaw
from odometry_validation.square import reconstruct_trajectories
from odometry_validation.square import require_complete_imu_coverage
from odometry_validation.square import run_square_sequence
from odometry_validation.square import safe_zero_confirmed
from odometry_validation.square import service_closed_loop_wait
from odometry_validation.square import square_metrics
from odometry_validation.square import square_emergency_stop_timeout_s
from odometry_validation.square import stop_with_graph_discovery
from odometry_validation.square import summarize_closed_loop_turn
from odometry_validation.square import unwrap_angle
from odometry_validation.square import wait_for_graph_contract
from odometry_validation.square import wait_for_imu_end_coverage
from odometry_validation.square_reprocess import KNOWN_RECOVERABLE_ERROR
from odometry_validation.square_reprocess import qualify_source_campaign
from odometry_validation.square_reprocess import reprocess_campaign
from odometry_validation.square_stop_reprocess import analyze_stop_campaign
from odometry_validation.square_stop_reprocess import reprocess_stop_campaign


def unavailable_fusion():
    return HeadingFusion(False, None, None, None, None, (), "test")


def synthetic_reprocess_source(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    config = SquareConfig()
    metadata = {
        "geometry": {
            "wheel_radius_m": 0.1, "track_width_m": 0.5,
            "encoder_ticks_per_revolution": 100},
        "square": asdict(config),
        "heading_fusion": asdict(unavailable_fusion()),
        "imu_bias_rad_s": 0.0,
        "stale_timeout_s": 0.5,
        "stationarity_thresholds": {"wheel_tick_semantics": "delta"},
    }
    manifest = []
    timestamp = 10.0
    for index, spec in enumerate(config.segments(), start=1):
        directory_name = f"{spec.trial_id}-attempt-001"
        directory = source / directory_name
        directory.mkdir()
        report = {
            "trial": asdict(spec),
            "operator": {"valid": True, "skipped": False},
            "encoder": ({
                "mean_distance_m": config.side_length_m}
                if spec.movement_type == "translation" else
                {"angle_rad": spec.commanded_angle_rad}),
        }
        if spec.movement_type == "translation":
            report["translation_drift"] = {
                "encoder_yaw_estimate_rad": 0.0,
                "imu_yaw_drift_rad": 0.0}
        else:
            report["theoretical"] = {
                "expected_angle_rad": spec.commanded_angle_rad}
            report["imu"] = {
                "total_physical_motion_angle_rad": spec.commanded_angle_rad}
        (directory / "report.json").write_text(
            json.dumps(report), encoding="utf-8")
        end = timestamp + 1.0
        stationary = end + 0.5
        manifest.append({
            "segment_index": index, "trial_id": spec.trial_id,
            "movement_type": spec.movement_type, "direction": spec.direction,
            "command_start_timestamp_s": timestamp,
            "command_end_timestamp_s": end,
            "stationary_confirmation_timestamp_s": stationary,
            "valid": True, "stationarity": {"stationary": True},
            "evidence_dir": f"/immutable/{directory_name}",
        })
        timestamp = stationary + 0.5
    failure = {
        "error": KNOWN_RECOVERABLE_ERROR, "error_type": "ValidationError",
        "failure_context": {
            "cleanup_attempted": True, "cleanup_completed": True,
            "completed_segments": manifest, "next_segment_index": 9,
            "remaining_square_aborted": True,
        },
    }
    (source / "metadata.json").write_text(
        json.dumps(metadata), encoding="utf-8")
    (source / "failure.json").write_text(
        json.dumps(failure), encoding="utf-8")
    start = manifest[0]["command_start_timestamp_s"]
    finish = manifest[-1]["stationary_confirmation_timestamp_s"]

    def write_csv(name, fieldnames, rows):
        with (source / name).open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

    imu_rows = []
    value = start - 0.1
    while value <= finish + 0.11:
        imu_rows.append({
            "timestamp_s": value, "angular_velocity_z_rad_s": 0.0,
            "phase": "after_motion", "source_topic": "/imu/data"})
        value += 0.1
    write_csv("raw_imu.csv", tuple(imu_rows[0]), imu_rows)
    stream_timestamps = []
    value = start - 0.1
    while value <= finish + 0.11:
        stream_timestamps.append(value)
        value += 0.1
    wheel_rows = [
        {"timestamp_s": value, "left_ticks": 0,
         "right_ticks": 0, "phase": "after_motion"}
        for value in stream_timestamps]
    write_csv("wheel_ticks.csv", tuple(wheel_rows[0]), wheel_rows)
    timing_rows = [{"timestamp_s": value} for value in stream_timestamps]
    write_csv("odometry.csv", ("timestamp_s",), timing_rows)
    write_csv("commanded_velocity.csv", ("timestamp_s",), timing_rows)
    return source, metadata, failure


def synthetic_stop_source(tmp_path):
    source = tmp_path / "failed-stop"
    source.mkdir()
    metadata = {
        "stationarity_thresholds": {
            "safe_command_zero_tolerance": 0.0001,
            "tick_delta_tolerance": 0,
            "linear_velocity_tolerance_m_s": 0.01,
            "angular_velocity_tolerance_rad_s": 0.02,
        }}
    assessment = {
        "assessment_timestamp_s": 11.1,
        "elapsed_since_first_zero_s": 1.1,
        "first_zero_timestamp_s": 10.0,
        "first_safe_zero_timestamp_s": 10.05,
        "time_from_first_zero_to_safe_zero_s": 0.05,
        "safe_zero": True,
        "odom_twist_stationary": True,
        "tick_deltas_stationary": False,
        "stationary": False,
        "reason": "encoder motion exceeded stationarity tolerance",
    }
    failure = {
        "error_type": "EmergencyStopCleanupError",
        "error": "controlled stop verification failed",
        "emergency_stop_records": [{
            "stop_mode": "controlled", "timeout_s": 3.0,
            "safe_zero": False, "stationary": False,
            "stationarity_assessments": [assessment]}],
        "failure_context": {
            "command_start_timestamp_s": 9.0,
            "completed_segments": [{}] * 6,
            "next_segment_index": 7,
        }}
    (source / "metadata.json").write_text(
        json.dumps(metadata), encoding="utf-8")
    (source / "failure.json").write_text(
        json.dumps(failure), encoding="utf-8")

    def write(name, fields, rows):
        with (source / name).open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

    write("commanded_velocity.csv", (
        "timestamp_s", "topic", "linear_x_m_s", "angular_z_rad_s", "phase"), (
            {"timestamp_s": 9.9, "topic": "/cmd_vel/test",
             "linear_x_m_s": 0.2, "angular_z_rad_s": 0.0, "phase": "motion"},
            {"timestamp_s": 10.0, "topic": "/cmd_vel/test",
             "linear_x_m_s": 0.0, "angular_z_rad_s": 0.0, "phase": "stop"},
            {"timestamp_s": 10.05, "topic": "/cmd_vel/safe",
             "linear_x_m_s": 0.0, "angular_z_rad_s": 0.0, "phase": "stop"},
        ))
    write("wheel_ticks.csv", (
        "timestamp_s", "left_ticks", "right_ticks", "phase"), (
            {"timestamp_s": 10.1, "left_ticks": 2,
             "right_ticks": 2, "phase": "stop"},
            {"timestamp_s": 10.2, "left_ticks": 0,
             "right_ticks": 0, "phase": "stop"},
        ))
    write("odometry.csv", (
        "timestamp_s", "linear_x_m_s", "angular_z_rad_s"), (
            {"timestamp_s": 10.1, "linear_x_m_s": 0.02,
             "angular_z_rad_s": 0.0},
            {"timestamp_s": 10.2, "linear_x_m_s": 0.0,
             "angular_z_rad_s": 0.0},
        ))
    segment = source / "square-leg-1-attempt-001"
    segment.mkdir()
    (segment / "stationarity.json").write_text(json.dumps({
        "assessments": [{
            **assessment, "tick_deltas_stationary": True,
            "stationary": True, "elapsed_since_first_zero_s": 1.1}],
        "final_accepted_window": {"stationary": True},
    }), encoding="utf-8")
    return source


@pytest.mark.parametrize("direction,expected_sign", [("ccw", 1.0), ("cw", -1.0)])
def test_square_sequence_has_four_sides_and_signed_quarter_turns(
        direction, expected_sign):
    segments = SquareConfig(
        side_length_m=2.0, linear_velocity_m_s=0.5,
        angular_velocity_rad_s=0.5, direction=direction).segments()
    assert [segment.movement_type for segment in segments] == [
        "translation", "rotation"] * 4
    assert all(math.isclose(segment.commanded_distance_m, 2.0)
               for segment in segments[::2])
    assert all(math.isclose(
        segment.commanded_angle_rad, expected_sign * math.pi / 2.0)
        for segment in segments[1::2])


def test_square_runner_executes_eight_segments_and_seven_settling_pauses():
    events = []
    segments = SquareConfig().segments()
    completed = run_square_sequence(
        segments,
        lambda index, segment: events.append(("segment", index, segment.trial_id)),
        lambda: events.append(("pause",)))
    assert len(completed) == 8
    assert [event[0] for event in events].count("segment") == 8
    assert [event[0] for event in events].count("pause") == 7
    assert events[:3] == [
        ("segment", 1, "square-leg-1"), ("pause",),
        ("segment", 2, "square-turn-1")]


def test_open_loop_square_specs_remain_fixed_duration_and_commanded_angle():
    config = SquareConfig(
        side_length_m=2.0, linear_velocity_m_s=0.2,
        angular_velocity_rad_s=0.3, direction="ccw")
    segments = config.segments()
    assert [segment.duration_s for segment in segments[::2]] == [10.0] * 4
    assert all(segment.duration_s == pytest.approx((math.pi / 2.0) / 0.3)
               for segment in segments[1::2])
    assert all(segment.commanded_angle_rad == pytest.approx(math.pi / 2.0)
               for segment in segments[1::2])


def test_closed_loop_fusion_is_exact_operator_specified_50_50():
    assert fuse_closed_loop_yaw(1.0, 2.0) == pytest.approx(1.5)
    assert fuse_closed_loop_yaw(-1.0, -2.0) == pytest.approx(-1.5)


def test_yaw_unwrap_continues_across_positive_pi_boundary():
    previous = math.pi - 0.01
    current_wrapped = -math.pi + 0.02
    assert unwrap_angle(previous, current_wrapped) == pytest.approx(
        math.pi + 0.02)


@pytest.mark.parametrize("direction", (1.0, -1.0))
def test_closed_loop_ccw_and_cw_turn_reach_target_after_settle(direction):
    config = ClosedLoopYawConfig(settle_hold_s=0.2)
    controller = ClosedLoopYawController(
        direction * math.pi / 2.0, config, 0.0)
    initial = controller.update(0.0, 0.0, 0.0)
    assert initial.commanded_angular_velocity_rad_s == pytest.approx(
        direction * config.max_angular_velocity_rad_s)
    near = controller.update(1.0, direction * 1.56, direction * 1.56)
    assert near.within_tolerance
    assert near.commanded_angular_velocity_rad_s == 0.0
    complete = controller.update(1.2, direction * 1.56, direction * 1.56)
    assert complete.complete


def test_closed_loop_controller_gradually_slows_near_target():
    controller = ClosedLoopYawController(
        math.pi / 2.0, ClosedLoopYawConfig(), 0.0)
    far = controller.update(0.0, 0.0, 0.0)
    near = controller.update(0.1, 1.30, 1.30)
    assert far.commanded_angular_velocity_rad_s == pytest.approx(0.3)
    assert 0.08 < near.commanded_angular_velocity_rad_s < 0.3


def test_closed_loop_tolerance_hold_resets_when_heading_leaves_tolerance():
    controller = ClosedLoopYawController(
        math.pi / 2.0, ClosedLoopYawConfig(settle_hold_s=0.25), 0.0)
    first = controller.update(1.0, 1.56, 1.56)
    assert first.tolerance_hold_elapsed_s == 0.0
    outside = controller.update(1.1, 1.53, 1.53)
    assert not outside.within_tolerance
    second = controller.update(1.2, 1.56, 1.56)
    assert second.tolerance_hold_elapsed_s == 0.0
    assert not second.complete


def test_closed_loop_overshoot_commands_bounded_reverse_correction():
    config = ClosedLoopYawConfig()
    controller = ClosedLoopYawController(math.pi / 2.0, config, 0.0)
    output = controller.update(0.1, 1.65, 1.65)
    assert output.commanded_angular_velocity_rad_s == pytest.approx(
        -config.min_correction_angular_velocity_rad_s)
    summary = summarize_closed_loop_turn([asdict(output)])
    assert summary["overshoot_rad"] == pytest.approx(1.65 - math.pi / 2.0)
    assert summary["undershoot_rad"] == 0.0


def test_closed_loop_stale_encoder_gap_fails_closed():
    geometry = GeometryConfig(0.1, 0.5, 100)
    samples = (
        WheelTickSample(0.1, -1, 1), WheelTickSample(1.0, -1, 1))
    with pytest.raises(ValidationError, match="stale encoder sample gap"):
        encoder_relative_yaw(samples, geometry, 0.2, 0.5, 0.0)


def test_closed_loop_stale_imu_gap_fails_closed():
    samples = (ImuSample(0.0, 0.3), ImuSample(0.6, 0.3))
    with pytest.raises(ValidationError, match="stale IMU sample gap"):
        imu_relative_yaw(samples, 0.0, 0.2, 0.5)


def test_closed_loop_rejects_nonfinite_sensor_values():
    with pytest.raises(ValidationError, match="nonfinite"):
        imu_relative_yaw(
            (ImuSample(0.0, 0.3), ImuSample(0.1, math.nan)),
            0.0, 0.2, 0.5)


def test_closed_loop_rejects_sensor_timestamp_reset():
    geometry = GeometryConfig(0.1, 0.5, 100)
    with pytest.raises(ValidationError, match="timestamp reset"):
        encoder_relative_yaw(
            (WheelTickSample(1.0, -1, 1),
             WheelTickSample(0.9, -1, 1)),
            geometry, 0.2, 0.5, 0.8)


def test_closed_loop_sustained_encoder_imu_disagreement_aborts():
    config = ClosedLoopYawConfig(
        max_disagreement_rad=0.2, disagreement_hold_s=0.25)
    controller = ClosedLoopYawController(math.pi / 2.0, config, 0.0)
    controller.update(0.0, 0.3, 0.0)
    with pytest.raises(ValidationError, match="sustained encoder/IMU"):
        controller.update(0.25, 0.4, 0.0)


def test_closed_loop_turn_timeout_aborts():
    controller = ClosedLoopYawController(
        math.pi / 2.0, ClosedLoopYawConfig(turn_timeout_s=1.0), 0.0)
    controller.update(0.0, 0.0, 0.0)
    with pytest.raises(ValidationError, match="turn timed out"):
        controller.update(1.0, 0.1, 0.1)


def test_closed_loop_settle_hold_cannot_complete_after_turn_timeout():
    controller = ClosedLoopYawController(
        math.pi / 2.0,
        ClosedLoopYawConfig(settle_hold_s=0.3, turn_timeout_s=1.0), 0.0)
    controller.update(0.7, 1.56, 1.56)
    with pytest.raises(ValidationError, match="turn timed out"):
        controller.update(1.0, 1.56, 1.56)


@pytest.mark.parametrize(
    "option",
    ({"max_disagreement_rad": 0.200001},
     {"max_heading_step_rad": 0.200001}),
)
def test_closed_loop_sensor_guard_thresholds_cannot_be_weakened(option):
    with pytest.raises(ValueError, match="exceeds safety maximum"):
        ClosedLoopYawConfig(**option)


def test_closed_loop_wait_services_guards_and_stops_at_deadline():
    clock = [0.0]
    guards = []

    def spin(duration):
        clock[0] += duration

    service_closed_loop_wait(
        spin, lambda: guards.append(clock[0]),
        next_publication_s=100.0, deadline_s=1.0,
        monotonic=lambda: clock[0])
    assert clock[0] == pytest.approx(1.0)
    assert guards
    assert max(guards) <= 1.0


def test_square_cleanup_uses_full_configured_stationarity_window():
    assert square_emergency_stop_timeout_s(1.0, 3.0) == 3.0
    assert square_emergency_stop_timeout_s(4.0, 3.0) == 4.0


def test_closed_loop_failure_runs_zero_cleanup_and_aborts_square():
    zeros = []
    cleanup = EmergencyCleanupOnce(EmergencyStopController(
        publish_zero=lambda: zeros.append(0), verify_safe_zero=lambda: True,
        verify_stationary=lambda: True, sleep=lambda _duration: None,
        stationarity_required=lambda: False))

    def fail_controller():
        controller = ClosedLoopYawController(
            math.pi / 2.0, ClosedLoopYawConfig(turn_timeout_s=0.1), 0.0)
        controller.update(0.1, 0.0, 0.0)

    with pytest.raises(ValidationError, match="turn timed out"):
        run_with_emergency_stop(fail_controller, cleanup, 0.1, 20.0)
    assert cleanup.completed and zeros


def test_closed_loop_square_sequence_completes_all_four_feedback_turns():
    completed_turns = []

    def execute(_index, spec):
        if spec.movement_type == "rotation":
            controller = ClosedLoopYawController(
                spec.commanded_angle_rad,
                ClosedLoopYawConfig(settle_hold_s=0.1), 0.0)
            controller.update(0.0, 0.0, 0.0)
            controller.update(
                1.0, spec.commanded_angle_rad, spec.commanded_angle_rad)
            final = controller.update(
                1.1, spec.commanded_angle_rad, spec.commanded_angle_rad)
            assert final.complete
            completed_turns.append(spec.trial_id)

    completed = run_square_sequence(
        SquareConfig(direction="cw").segments(), execute, lambda: None)
    assert len(completed) == 8
    assert completed_turns == [f"square-turn-{index}" for index in range(1, 5)]


def test_bounded_graph_discovery_accepts_delayed_arbiter():
    clock = [0.0]
    observations = iter([
        ("node:/command_arbiter",),
        ("arbiter-subscriber:/cmd_vel/test",),
        (),
    ])
    spins = []

    def spin(duration):
        spins.append(duration)
        clock[0] += duration

    wait_for_graph_contract(
        lambda: next(observations), spin, 1.0,
        monotonic=lambda: clock[0])
    assert len(spins) == 2


def test_bounded_graph_discovery_fails_closed_when_arbiter_missing():
    clock = [0.0]

    def spin(duration):
        clock[0] += duration

    with pytest.raises(ValidationError, match="graph discovery timed out"):
        wait_for_graph_contract(
            lambda: ("node:/command_arbiter",), spin, 0.1,
            monotonic=lambda: clock[0])


def test_graph_contract_normalizes_names_and_verifies_endpoint_owners():
    assert graph_contract_missing(
        ("command_arbiter",), ("/command_arbiter",),
        ("command_arbiter",), ("/roboteq_ros2_driver",)) == ()
    missing = graph_contract_missing(
        ("/command_arbiter",), ("/other",), ("/other",), ("/other",))
    assert "arbiter-subscriber:/cmd_vel/test" in missing
    assert "arbiter-publisher:/cmd_vel/safe" in missing
    assert "roboteq-subscriber:/cmd_vel/safe" in missing


def test_safe_zero_waits_for_graph_and_post_publication_callback():
    assert not safe_zero_confirmed(("publisher:/cmd_vel/safe",), True, 10.0, 11.0)
    assert not safe_zero_confirmed((), True, 10.0, 9.9)
    assert not safe_zero_confirmed((), False, 10.0, 10.1)
    assert safe_zero_confirmed((), True, 10.0, 10.1)


def test_cleanup_publishes_zero_during_delayed_discovery_then_confirms_fresh_zero():
    clock = [0.0]
    graph_observations = iter([
        ("node:/command_arbiter",),
        ("arbiter-publisher:/cmd_vel/safe",),
        (),
    ])
    zeros = []
    confirmations = iter([False, True])

    def spin(duration):
        clock[0] += duration

    def verified_stop():
        zeros.append("verified-stop-zero")
        publication_s = clock[0]
        while True:
            confirmed = next(confirmations)
            callback_s = publication_s + 0.01 if confirmed else publication_s - 0.01
            if safe_zero_confirmed((), confirmed, publication_s, callback_s):
                return "stopped"
            spin(0.01)

    result = stop_with_graph_discovery(
        lambda: next(graph_observations),
        lambda: zeros.append("discovery-zero"), spin, verified_stop,
        1.0, 20.0, monotonic=lambda: clock[0])
    assert result == "stopped"
    assert zeros == [
        "discovery-zero", "discovery-zero", "discovery-zero",
        "verified-stop-zero"]


def test_cleanup_fails_closed_after_bounded_missing_graph_while_publishing_zero():
    clock = [0.0]
    zeros = []

    def spin(duration):
        clock[0] += duration

    with pytest.raises(ValidationError, match="cleanup graph discovery timed out"):
        stop_with_graph_discovery(
            lambda: ("node:/command_arbiter",),
            lambda: zeros.append(clock[0]), spin,
            lambda: pytest.fail("verified stop must not run"),
            0.11, 20.0, monotonic=lambda: clock[0])
    assert len(zeros) >= 3


def test_cleanup_publishes_zero_before_graph_query_exception_and_recovers():
    clock = [0.0]
    zeros = []
    observations = iter([RuntimeError("DDS unavailable"), ()])

    def graph_missing():
        observation = next(observations)
        if isinstance(observation, BaseException):
            raise observation
        return observation

    def spin(duration):
        clock[0] += duration

    result = stop_with_graph_discovery(
        graph_missing, lambda: zeros.append(clock[0]), spin,
        lambda: "verified", 1.0, 20.0, monotonic=lambda: clock[0])
    assert result == "verified"
    assert len(zeros) == 2
    assert zeros[0] == 0.0


def test_segment_failure_aborts_remaining_square_and_runs_zero_cleanup():
    called = []
    zeros = []
    cleanup = EmergencyCleanupOnce(EmergencyStopController(
        publish_zero=lambda: zeros.append(0), verify_safe_zero=lambda: True,
        verify_stationary=lambda: True, sleep=lambda _duration: None,
        stationarity_required=lambda: False))
    segments = SquareConfig().segments()

    def action():
        def execute(index, _segment):
            called.append(index)
            if index == 3:
                raise ValidationError("segment guard failed")
            return index
        return run_square_sequence(segments, execute, lambda: None)

    with pytest.raises(ValidationError, match="segment guard failed"):
        run_with_emergency_stop(action, cleanup, 0.1, 10.0)
    assert called == [1, 2, 3]
    assert zeros and cleanup.completed


@pytest.mark.parametrize("direction", ["ccw", "cw"])
def test_final_drift_metrics_close_ideal_square(direction):
    sign = 1.0 if direction == "ccw" else -1.0
    observations = []
    for _index in range(4):
        observations.append({
            "encoder_distance_m": 2.0, "encoder_angle_rad": 0.0,
            "imu_angle_rad": 0.0})
        observations.append({
            "encoder_distance_m": 0.0,
            "encoder_angle_rad": sign * math.pi / 2.0,
            "imu_angle_rad": sign * math.pi / 2.0})
    report = square_metrics(
        SquareConfig(direction=direction), observations, unavailable_fusion())
    assert report["position_closure_error_m"] == pytest.approx(0.0, abs=1e-12)
    assert report["final_encoder_yaw_error_rad"] == pytest.approx(0.0, abs=1e-12)
    assert report["imu_final_yaw_error_rad"] == pytest.approx(0.0, abs=1e-12)
    assert report["closure_percent_of_commanded_path"] == pytest.approx(0.0)
    assert report["fused_final_pose"] is None


def test_drift_metrics_report_nonzero_closure_and_each_leg():
    observations = []
    distances = (2.0, 2.1, 1.9, 2.0)
    for distance in distances:
        observations.append({
            "encoder_distance_m": distance, "encoder_angle_rad": 0.0,
            "imu_angle_rad": 0.0})
        observations.append({
            "encoder_distance_m": 0.0,
            "encoder_angle_rad": math.pi / 2.0 + 0.01,
            "imu_angle_rad": math.pi / 2.0 + 0.005})
    report = square_metrics(SquareConfig(), observations, unavailable_fusion())
    assert report["per_leg_encoder_distance_m"] == list(distances)
    assert len(report["turn_estimates"]) == 4
    assert report["position_closure_error_m"] > 0.0
    assert report["total_encoder_path_m"] == pytest.approx(8.0)


def test_continuous_encoder_and_imu_trajectory_reconstruction():
    geometry = GeometryConfig(1.0 / (2.0 * math.pi), 1.0, 1)
    wheels = (
        WheelTickSample(1.0, 1, 1, "during_motion"),
        WheelTickSample(2.0, -1, 1, "during_motion"),
        WheelTickSample(3.0, 1, 1, "during_motion"),
    )
    imu = (
        ImuSample(1.0, 1.0), ImuSample(2.0, 1.0), ImuSample(3.0, 1.0))
    result = reconstruct_trajectories(wheels, imu, geometry, 0.0)
    assert len(result["encoder"]) == 3
    assert result["encoder"][-1]["path_length_m"] == pytest.approx(2.0)
    assert result["imu_final_yaw_rad"] == pytest.approx(2.0)


def test_imu_trajectory_interpolates_non_aligned_trial_boundaries():
    geometry = GeometryConfig(1.0 / (2.0 * math.pi), 1.0, 1)
    imu = (
        ImuSample(0.0, 0.0), ImuSample(1.0, 2.0),
        ImuSample(2.0, 2.0), ImuSample(3.0, 0.0))
    result = reconstruct_trajectories(
        (), imu, geometry, 0.0, start_timestamp_s=0.5,
        end_timestamp_s=2.5)
    assert result["imu"][0]["timestamp_s"] == pytest.approx(0.5)
    assert result["imu"][-1]["timestamp_s"] == pytest.approx(2.5)
    assert result["imu_final_yaw_rad"] == pytest.approx(3.5)


def test_imu_capture_brackets_complete_square_trajectory():
    samples = (
        ImuSample(9.9, 0.0), ImuSample(10.0, 0.1),
        ImuSample(10.4, 0.1), ImuSample(10.8, 0.1),
        ImuSample(11.0, 0.1), ImuSample(11.1, 0.0))
    coverage = imu_coverage(samples, 10.0, 11.0, 0.5)
    require_complete_imu_coverage(coverage)
    assert coverage["complete"]
    assert coverage["coverage_before_start_s"] == pytest.approx(0.1)
    assert coverage["coverage_after_end_s"] == pytest.approx(0.1)
    assert coverage["sample_count"] == 6
    assert coverage["max_sample_gap_s"] == pytest.approx(0.4)
    assert coverage["gaps_over_threshold_count"] == 0


def test_imu_coverage_rejects_mid_trajectory_gap_over_stale_limit():
    coverage = imu_coverage(
        (ImuSample(9.9, 0.0), ImuSample(10.0, 0.0),
         ImuSample(19.9, 0.0), ImuSample(20.1, 0.0)),
        10.0, 20.0, 0.5)
    with pytest.raises(ValidationError, match="gaps exceeding"):
        require_complete_imu_coverage(coverage)


def test_delayed_callback_delivery_is_bounded_and_uses_valid_sample_timestamp():
    clock = [0.0]
    delivered = [
        ImuSample(9.9 + 0.1 * index, 0.0)
        for index in range(101)]
    delivered.append(ImuSample(19.99, 0.0))

    def spin(duration):
        clock[0] += duration
        delivered.append(ImuSample(20.01, 0.0, "after_motion"))

    assert wait_for_imu_end_coverage(
        lambda: tuple(delivered), spin, 20.0, 0.5, 0.01,
        monotonic=lambda: clock[0])
    coverage = imu_coverage(delivered, 10.0, 20.0, 0.5)
    require_complete_imu_coverage(coverage)
    assert coverage["coverage_after_end_s"] == pytest.approx(0.01)


def test_imu_coverage_fails_closed_when_start_is_missing():
    coverage = imu_coverage(
        (ImuSample(10.01, 0.0), ImuSample(20.01, 0.0)), 10.0, 20.0)
    with pytest.raises(ValidationError, match="missing square start coverage"):
        require_complete_imu_coverage(coverage)


def test_imu_coverage_fails_closed_when_end_is_missing():
    coverage = imu_coverage(
        (ImuSample(9.99, 0.0), ImuSample(19.99, 0.0)), 10.0, 20.0)
    with pytest.raises(ValidationError, match="missing square end coverage"):
        require_complete_imu_coverage(coverage)


def test_imu_coverage_rejects_nonoverlapping_timestamp_basis():
    coverage = imu_coverage(
        (ImuSample(1.0, 0.0), ImuSample(2.0, 0.0)), 1000.0, 1010.0)
    with pytest.raises(ValidationError, match="timestamp basis does not overlap"):
        require_complete_imu_coverage(coverage)


def test_final_stationarity_period_is_included_in_imu_trajectory():
    geometry = GeometryConfig(1.0 / (2.0 * math.pi), 1.0, 1)
    samples = (
        ImuSample(0.9, 0.0, "before_motion"),
        ImuSample(1.0, 1.0, "during_motion"),
        ImuSample(2.0, 1.0, "after_motion"),
        ImuSample(3.0, 0.0, "after_motion"),
        ImuSample(3.1, 0.0, "after_motion"))
    result = reconstruct_trajectories(
        (), samples, geometry, 0.0, start_timestamp_s=1.0,
        end_timestamp_s=3.0)
    assert result["imu"][-1]["timestamp_s"] == pytest.approx(3.0)
    assert result["imu"][-1]["phase"] == "after_motion"
    assert result["imu_final_yaw_rad"] == pytest.approx(1.5)


def test_continuous_final_metrics_are_canonical_and_keep_segment_comparison():
    base = {
        "total_commanded_path_m": 8.0,
        "encoder_only_final_pose": {"x_m": 99.0},
        "imu_heading_final_yaw_rad": 99.0,
    }
    trajectories = {
        "encoder_final_pose": {
            "x_m": 0.3, "y_m": -0.4, "yaw_rad": 2.0 * math.pi + 0.1,
            "path_length_m": 8.2},
        "imu_final_yaw_rad": 2.0 * math.pi - 0.2,
    }
    report = apply_continuous_final_metrics(base, trajectories)
    assert report["encoder_only_final_pose"]["x_m"] == pytest.approx(0.3)
    assert report["final_delta_x_m"] == pytest.approx(0.3)
    assert report["position_closure_error_m"] == pytest.approx(0.5)
    assert report["final_encoder_yaw_error_rad"] == pytest.approx(0.1)
    assert report["imu_final_yaw_error_rad"] == pytest.approx(-0.2)
    assert report["segment_aggregate_encoder_final_pose"] == {"x_m": 99.0}


def test_fusion_audit_excludes_teledex_and_dry_run(tmp_path):
    report = {
        "trial": {"movement_type": "rotation"},
        "operator": {"valid": True, "skipped": False},
        "theoretical": {"expected_angle_rad": 1.0},
        "encoder": {"angle_rad": 1.1},
        "imu": {"total_physical_motion_angle_rad": 1.05},
        "teledex_reference": {"present": True},
    }
    trial = tmp_path / "campaign" / "rot-1"
    trial.mkdir(parents=True)
    (tmp_path / "campaign" / "metadata.json").write_text(
        json.dumps({"dry_run": False}), encoding="utf-8")
    (trial / "report.json").write_text(json.dumps(report), encoding="utf-8")
    fusion = derive_heading_fusion(tmp_path)
    assert not fusion.available
    assert fusion.valid_non_teledex_rotation_trials == 0
    assert fusion.trusted_physical_reference_trials == 0


def test_reprocessor_accepts_only_completed_safe_square_and_writes_new_evidence(
        tmp_path):
    source, metadata, failure = synthetic_reprocess_source(tmp_path)
    qualify_source_campaign(metadata, failure)
    source_failure_before = (source / "failure.json").read_bytes()
    output = tmp_path / "recovered"
    report = reprocess_campaign(source, output)
    assert report["measurement_status"] == (
        "recovered_from_completed_failed_campaign")
    assert report["source_evidence_unchanged"]
    assert report["max_imu_gap_s"] == pytest.approx(0.5)
    assert (source / "failure.json").read_bytes() == source_failure_before
    assert (output / "square_report.json").exists()
    assert (output / "encoder_trajectory.csv").exists()
    assert (output / "imu_trajectory.csv").exists()


@pytest.mark.parametrize("mutation,match", [
    (lambda failure: failure["failure_context"].update(
        cleanup_completed=False), "successful safe cleanup"),
    (lambda failure: failure["failure_context"]["completed_segments"][2].update(
        valid=False), "invalid or nonstationary"),
    (lambda failure: failure["failure_context"]["completed_segments"][1].update(
        trial_id="wrong"), "segment order"),
])
def test_reprocessor_rejects_unqualified_source(tmp_path, mutation, match):
    _source, metadata, failure = synthetic_reprocess_source(tmp_path)
    mutation(failure)
    with pytest.raises(ValueError, match=match):
        qualify_source_campaign(metadata, failure)


def test_reprocessor_rejects_source_overwrite_and_existing_output(tmp_path):
    source, _metadata, _failure = synthetic_reprocess_source(tmp_path)
    with pytest.raises(ValueError, match="separate sibling"):
        reprocess_campaign(source, source)
    output = tmp_path / "already-exists"
    output.mkdir()
    with pytest.raises(FileExistsError, match="already exists"):
        reprocess_campaign(source, output)


def test_legacy_reprocessor_requires_explicit_actual_imu_gap_limit(tmp_path):
    source, metadata, _failure = synthetic_reprocess_source(tmp_path)
    metadata.pop("stale_timeout_s")
    (source / "metadata.json").write_text(
        json.dumps(metadata), encoding="utf-8")
    with pytest.raises(ValueError, match="explicit positive maximum IMU gap"):
        reprocess_campaign(source, tmp_path / "missing-limit")
    report = reprocess_campaign(
        source, tmp_path / "with-limit", legacy_max_imu_gap_s=0.5)
    assert report["legacy_gap_threshold_override_s"] == pytest.approx(0.5)


def test_reprocessor_rejects_sparse_critical_stream(tmp_path):
    source, _metadata, failure = synthetic_reprocess_source(tmp_path)
    manifest = failure["failure_context"]["completed_segments"]
    start = manifest[0]["command_start_timestamp_s"]
    end = manifest[-1]["stationary_confirmation_timestamp_s"]
    with (source / "wheel_ticks.csv").open(
            "w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=(
            "timestamp_s", "left_ticks", "right_ticks", "phase"))
        writer.writeheader()
        writer.writerows([
            {"timestamp_s": start - 0.1, "left_ticks": 0,
             "right_ticks": 0, "phase": "before_motion"},
            {"timestamp_s": end + 0.1, "left_ticks": 0,
             "right_ticks": 0, "phase": "after_motion"},
        ])
    with pytest.raises(ValueError, match="continuously bracket"):
        reprocess_campaign(source, tmp_path / "sparse-output")


def test_reprocessor_rejects_nonfinite_primary_imu_rate(tmp_path):
    source, _metadata, _failure = synthetic_reprocess_source(tmp_path)
    rows = list(csv.DictReader((source / "raw_imu.csv").open(encoding="utf-8")))
    rows[len(rows) // 2]["angular_velocity_z_rad_s"] = "nan"
    with (source / "raw_imu.csv").open(
            "w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with pytest.raises(ValueError, match="angular velocities must be finite"):
        reprocess_campaign(source, tmp_path / "nan-imu-output")


def test_reprocessor_rejects_nonfinite_segment_measurement(tmp_path):
    source, _metadata, _failure = synthetic_reprocess_source(tmp_path)
    report_path = source / "square-turn-1-attempt-001" / "report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["encoder"]["angle_rad"] = float("inf")
    report_path.write_text(json.dumps(report), encoding="utf-8")
    with pytest.raises(ValueError, match="encoder angle must be finite"):
        reprocess_campaign(source, tmp_path / "inf-segment-output")


def test_stop_reprocessor_recovers_timeline_and_safe_zero_latch_bug(tmp_path):
    source = synthetic_stop_source(tmp_path)

    result = analyze_stop_campaign(source, ())
    analysis = result["analysis"]

    assert analysis["safe_zero_confirmation_bug_observed"]
    assert analysis["command_timeline"]["test_zero_to_safe_zero_s"] == pytest.approx(
        0.05)
    assert analysis["physical_decay"][
        "last_encoder_motion_elapsed_s"] == pytest.approx(0.1)
    assert analysis["physical_decay"][
        "last_above_tolerance_odom_elapsed_s"] == pytest.approx(0.1)
    assert not analysis["usable_as_complete_square_baseline"]


def test_stop_reprocessor_preserves_source_and_refuses_overwrite(tmp_path):
    source = synthetic_stop_source(tmp_path)
    before = {
        path.relative_to(source): path.read_bytes()
        for path in source.rglob("*") if path.is_file()}
    output = tmp_path / "stop-recovery"

    report = reprocess_stop_campaign(source, output, ())

    assert report["recovery_status"] == "partial_stop_failure_diagnostics_only"
    assert (output / "stop_analysis.json").exists()
    assert before == {
        path.relative_to(source): path.read_bytes()
        for path in source.rglob("*") if path.is_file()}
    with pytest.raises(FileExistsError):
        reprocess_stop_campaign(source, output, ())
    with pytest.raises(ValueError, match="must not be inside source"):
        reprocess_stop_campaign(source, source / "nested", ())
