# Copyright 2026 Medrobots Engineering
#
# Licensed under the Apache License, Version 2.0 (the "License");
"""Supervised square runner using the existing test-to-safe command path."""

import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import sys
import time
import traceback
from typing import Callable, Dict, List, Optional, Sequence

import rclpy

from odometry_validation.core import EmergencyCleanupOnce
from odometry_validation.core import EmergencyStopCleanupError
from odometry_validation.core import EmergencyStopController
from odometry_validation.core import EvidenceWriter
from odometry_validation.core import GeometryConfig
from odometry_validation.core import TrialResult
from odometry_validation.core import TrialSamples
from odometry_validation.core import TrialSpec
from odometry_validation.core import ValidationError
from odometry_validation.node import OPERATOR_CALLBACK_SERVICE_S
from odometry_validation.node import OPERATOR_INPUT_POLL_INTERVAL_S
from odometry_validation.node import ENCODER_CHATTER_MAX_ABSOLUTE_TICKS
from odometry_validation.node import ENCODER_CHATTER_MAX_DIRECTIONAL_TICKS
from odometry_validation.node import ENCODER_CHATTER_MAX_NET_TICKS
from odometry_validation.node import ENCODER_CHATTER_MAX_SAMPLE_DELTA_TICKS
from odometry_validation.node import ENCODER_STATIONARITY_WINDOW_S
from odometry_validation.node import OdometryValidationNode
from odometry_validation.node import ResponsiveOperatorInput
from odometry_validation.node import TerminalLineReader
from odometry_validation.node import build_arg_parser
from odometry_validation.node import execute_trial
from odometry_validation.node import validate_args
from odometry_validation.square import SquareConfig
from odometry_validation.square import CLOSED_LOOP_ENCODER_WEIGHT
from odometry_validation.square import CLOSED_LOOP_IMU_WEIGHT
from odometry_validation.square import ClosedLoopYawConfig
from odometry_validation.square import ClosedLoopYawController
from odometry_validation.square import apply_continuous_final_metrics
from odometry_validation.square import derive_heading_fusion
from odometry_validation.square import graph_contract_missing
from odometry_validation.square import imu_coverage
from odometry_validation.square import encoder_relative_yaw
from odometry_validation.square import fuse_closed_loop_yaw
from odometry_validation.square import imu_relative_yaw
from odometry_validation.square import reconstruct_trajectories
from odometry_validation.square import require_complete_imu_coverage
from odometry_validation.square import safe_zero_confirmed
from odometry_validation.square import service_closed_loop_wait
from odometry_validation.square import run_square_sequence
from odometry_validation.square import square_metrics
from odometry_validation.square import square_emergency_stop_timeout_s
from odometry_validation.square import stop_with_graph_discovery
from odometry_validation.square import summarize_closed_loop_turn
from odometry_validation.square import timestamp_coverage
from odometry_validation.square import wait_for_graph_contract
from odometry_validation.square import wait_for_imu_end_coverage


def _endpoint_name(info) -> str:
    namespace = str(info.node_namespace).rstrip("/")
    return f"{namespace}/{info.node_name}"


class SquareValidationNode(OdometryValidationNode):
    """Existing validator plus square-wide samples and bounded graph discovery."""

    def __init__(self, args, geometry: GeometryConfig):
        super().__init__(args)
        self.square_geometry = geometry
        self.confirm_motion: Optional[Callable[[], None]] = None
        self.motion_confirmed = False
        self.continuous_wheel_ticks = []
        self.continuous_imu = []
        self.continuous_imu_callback_timing = []
        self.continuous_odom = []
        self.continuous_diagnostics = []
        self.continuous_ignored_diagnostics = []
        self.continuous_commands = []
        self.continuous_stationarity = []
        self.closed_loop_feedback = []
        self.closed_loop_feedback_by_trial = {}

    def _wheel_ticks_callback(self, message) -> None:
        super()._wheel_ticks_callback(message)
        with self._samples_lock:
            self.continuous_wheel_ticks.append(self.wheel_ticks[-1])

    def _imu_callback(self, message, source_topic="/imu/data") -> None:
        super()._imu_callback(message, source_topic)
        received_timestamp_s = self._now_seconds()
        with self._samples_lock:
            sample = self.imu[-1]
            self.continuous_imu.append(sample)
            self.continuous_imu_callback_timing.append({
                "source_topic": sample.source_topic,
                "sample_timestamp_s": sample.timestamp_s,
                "callback_received_timestamp_s": received_timestamp_s,
                "source_to_callback_offset_s": (
                    received_timestamp_s - sample.timestamp_s),
                "phase": sample.phase,
            })

    def _odom_callback(self, message) -> None:
        super()._odom_callback(message)
        with self._samples_lock:
            self.continuous_odom.append(self.odom[-1])

    def _diagnostics_callback(self, message) -> None:
        before = len(self.diagnostics)
        ignored_before = len(self.ignored_diagnostics)
        super()._diagnostics_callback(message)
        with self._samples_lock:
            self.continuous_diagnostics.extend(self.diagnostics[before:])
            self.continuous_ignored_diagnostics.extend(
                self.ignored_diagnostics[ignored_before:])

    def _safe_command_callback(self, message) -> None:
        super()._safe_command_callback(message)
        with self._samples_lock:
            self.continuous_commands.append(self.safe_commands[-1])

    def publish_twist(self, linear_x: float, angular_z: float) -> None:
        before = len(self.test_commands)
        super().publish_twist(linear_x, angular_z)
        with self._samples_lock:
            self.continuous_commands.extend(self.test_commands[before:])

    def verify_stationary(self):
        assessment = super().verify_stationary()
        self.continuous_stationarity.append(assessment)
        return assessment

    def continuous_samples(self) -> TrialSamples:
        with self._samples_lock:
            return TrialSamples(
                wheel_ticks=tuple(self.continuous_wheel_ticks),
                imu=tuple(self.continuous_imu), odom=tuple(self.continuous_odom),
                diagnostics=tuple(self.continuous_diagnostics),
                ignored_diagnostics=tuple(self.continuous_ignored_diagnostics),
                commands=tuple(sorted(
                    self.continuous_commands, key=lambda item: item.timestamp_s)),
                stationarity=tuple(self.continuous_stationarity))

    def imu_callback_timing(self):
        with self._samples_lock:
            return tuple(dict(row) for row in self.continuous_imu_callback_timing)

    def _endpoint_nodes(self, topic: str, publishers: bool):
        infos = (self.get_publishers_info_by_topic(topic) if publishers else
                 self.get_subscriptions_info_by_topic(topic))
        return tuple(_endpoint_name(info) for info in infos)

    def graph_missing(self):
        nodes, _topics = self.snapshot_graph()
        return graph_contract_missing(
            tuple(nodes),
            self._endpoint_nodes("/cmd_vel/test", publishers=False),
            self._endpoint_nodes("/cmd_vel/safe", publishers=True),
            self._endpoint_nodes("/cmd_vel/safe", publishers=False))

    def wait_for_square_graph(self) -> None:
        wait_for_graph_contract(
            self.graph_missing, self.spin_for,
            self.args.graph_discovery_timeout_s)

    def verify_preflight(self) -> None:
        self.wait_for_square_graph()
        super().verify_preflight()

    def verify_safe_zero(self) -> bool:
        """Require a post-publication safe-zero callback from the owned path."""
        try:
            missing = self.graph_missing()
        except Exception:
            return False
        return safe_zero_confirmed(
            missing, super().verify_safe_zero(),
            self.last_test_command_timestamp_s, self.last_safe_time)

    def run_command_phase(self, spec, observation_hook=None) -> None:
        if not self.motion_confirmed:
            if self.confirm_motion is None:
                raise ValidationError("square operator confirmation is unavailable")
            self.confirm_motion()
            with self._samples_lock:
                primary = tuple(
                    sample for sample in self.continuous_imu
                    if sample.source_topic == "/imu/data")
            if not primary:
                raise ValidationError(
                    "processed /imu/data capture did not start before square motion")
            primary_timestamps = tuple(
                sample.timestamp_s for sample in primary)
            if (any(not math.isfinite(value) for value in primary_timestamps) or
                    any(current < previous for previous, current in zip(
                        primary_timestamps, primary_timestamps[1:]))):
                raise ValidationError(
                    "processed /imu/data timestamp basis is invalid before motion")
            latest_timestamp_s = primary_timestamps[-1]
            timestamp_offset_s = self._now_seconds() - latest_timestamp_s
            if (not math.isfinite(latest_timestamp_s) or
                    abs(timestamp_offset_s) > self.args.stale_timeout_s):
                raise ValidationError(
                    "processed /imu/data timestamp basis is inconsistent with "
                    f"the square ROS clock: offset_s={timestamp_offset_s}")
            self.motion_confirmed = True
        if (self.args.square_control_mode == "closed-loop" and
                spec.movement_type == "rotation"):
            self._run_closed_loop_turn(spec, observation_hook)
        else:
            super().run_command_phase(spec, observation_hook)

    def _run_closed_loop_turn(self, spec, observation_hook=None) -> None:
        """Control one signed quarter-turn; existing stop logic remains external."""
        config = ClosedLoopYawConfig(
            max_angular_velocity_rad_s=(
                self.args.square_angular_velocity_rad_s),
            min_correction_angular_velocity_rad_s=(
                self.args.closed_loop_min_angular_velocity_rad_s),
            proportional_gain_s_inv=self.args.closed_loop_yaw_gain,
            yaw_tolerance_rad=self.args.closed_loop_yaw_tolerance_rad,
            settle_hold_s=self.args.closed_loop_settle_hold_s,
            turn_timeout_s=self.args.closed_loop_turn_timeout_s,
            max_disagreement_rad=(
                self.args.closed_loop_max_yaw_disagreement_rad),
            disagreement_hold_s=(
                self.args.closed_loop_disagreement_hold_s),
            max_heading_step_rad=self.args.closed_loop_max_heading_step_rad)
        self.motion_armed = True
        self.sample_phase = "during_motion"
        self.verify_ready_after_operator_input()
        with self._samples_lock:
            wheel_start_index = len(self.wheel_ticks)
            wheel_baseline_timestamp_s = (
                None if not self.wheel_ticks else self.wheel_ticks[-1].timestamp_s)
            primary_indices = [
                index for index, sample in enumerate(self.imu)
                if sample.source_topic == "/imu/data"]
            if not primary_indices:
                raise ValidationError(
                    "processed /imu/data baseline unavailable for closed-loop turn")
            imu_start_index = primary_indices[-1]
        self.command_start_timestamp_s = self._now_seconds()
        target = math.copysign(math.pi / 2.0, spec.angular_z)
        control_start_s = time.monotonic()
        control_deadline_s = control_start_s + config.turn_timeout_s
        controller = ClosedLoopYawController(target, config, control_start_s)
        rows = []
        self.closed_loop_feedback_by_trial[spec.trial_id] = rows
        previous_command_spec = None
        period_s = 1.0 / self.args.publish_rate_hz
        try:
            while True:
                iteration_start = time.monotonic()
                if iteration_start >= control_deadline_s:
                    raise ValidationError("closed-loop turn timed out")
                self.spin_for(min(
                    0.01, period_s * 0.5,
                    control_deadline_s - iteration_start))
                if time.monotonic() >= control_deadline_s:
                    raise ValidationError("closed-loop turn timed out")
                if previous_command_spec is not None:
                    self.check_runtime_guards(previous_command_spec)
                with self._samples_lock:
                    wheel_samples = tuple(self.wheel_ticks[wheel_start_index:])
                    imu_samples = tuple(self.imu[imu_start_index:])
                encoder_yaw = encoder_relative_yaw(
                    wheel_samples, self.square_geometry,
                    config.max_heading_step_rad, self.args.stale_timeout_s,
                    wheel_baseline_timestamp_s)
                imu_yaw = imu_relative_yaw(
                    imu_samples, self.args.imu_bias_rad_s,
                    config.max_heading_step_rad, self.args.stale_timeout_s)
                now_s = self._now_seconds()
                try:
                    output = controller.update(
                        time.monotonic(), encoder_yaw, imu_yaw, now_s)
                except BaseException as error:
                    fused_yaw = fuse_closed_loop_yaw(encoder_yaw, imu_yaw)
                    failure_row = {
                        "timestamp_s": now_s,
                        "elapsed_s": time.monotonic() - control_start_s,
                        "target_yaw_rad": target,
                        "encoder_relative_yaw_rad": encoder_yaw,
                        "imu_relative_yaw_rad": imu_yaw,
                        "fused_relative_yaw_rad": fused_yaw,
                        "heading_error_rad": target - fused_yaw,
                        "commanded_angular_velocity_rad_s": 0.0,
                        "encoder_imu_disagreement_rad": abs(
                            encoder_yaw - imu_yaw),
                        "within_tolerance": False,
                        "tolerance_hold_elapsed_s": 0.0,
                        "complete": False,
                        "trial_id": spec.trial_id,
                        "controller_failure": (
                            f"{type(error).__name__}: {error}")}
                    rows.append(failure_row)
                    self.closed_loop_feedback.append(dict(failure_row))
                    raise
                row = asdict(output)
                row["trial_id"] = spec.trial_id
                row["controller_failure"] = None
                rows.append(row)
                self.closed_loop_feedback.append(dict(row))
                command_spec = TrialSpec(
                    spec.trial_id, "rotation",
                    abs(output.commanded_angular_velocity_rad_s), 0.0,
                    ("ccw" if output.commanded_angular_velocity_rad_s >= 0.0
                     else "cw"))
                self.publish_twist(
                    0.0, output.commanded_angular_velocity_rad_s)
                self.wait_for_expected_safe_command(
                    command_spec, control_deadline_s)
                previous_command_spec = command_spec
                if observation_hook is not None:
                    observation_hook()
                if output.complete:
                    return
                next_publication = iteration_start + period_s
                service_closed_loop_wait(
                    self.spin_for,
                    lambda: self.check_runtime_guards(command_spec),
                    next_publication, control_deadline_s,
                    monotonic=time.monotonic)
        finally:
            self.publish_zero()
            self.command_end_timestamp_s = self._now_seconds()


class SquareEmergencyStopController:
    """Add bounded zero-publication DDS discovery before normal verification."""

    def __init__(self, node: SquareValidationNode, controller):
        self.node = node
        self.controller = controller

    def stop(self, timeout_s: float, rate_hz: float, mode="emergency_cleanup"):
        return stop_with_graph_discovery(
            self.node.graph_missing, self.node.publish_zero,
            self.node.spin_for,
            lambda: self.controller.stop(timeout_s, rate_hz, mode=mode),
            self.node.args.graph_discovery_timeout_s, rate_hz)


def build_square_parser() -> argparse.ArgumentParser:
    parser = build_arg_parser()
    parser.description = "Supervised 2 m square odometry closure validation."
    parser.add_argument("--square-side-length-m", type=float, default=2.0)
    parser.add_argument("--square-linear-velocity-m-s", type=float, default=0.2)
    parser.add_argument("--square-angular-velocity-rad-s", type=float, default=0.3)
    parser.add_argument("--square-direction", choices=("cw", "ccw"), default="ccw")
    parser.add_argument(
        "--square-control-mode", choices=("open-loop", "closed-loop"),
        default="open-loop")
    parser.add_argument("--square-between-segment-pause-s", type=float, default=1.0)
    parser.add_argument(
        "--closed-loop-yaw-tolerance-rad", type=float, default=0.02)
    parser.add_argument(
        "--closed-loop-turn-timeout-s", type=float, default=10.0)
    parser.add_argument(
        "--closed-loop-min-angular-velocity-rad-s", type=float, default=0.08)
    parser.add_argument("--closed-loop-yaw-gain", type=float, default=1.0)
    parser.add_argument("--closed-loop-settle-hold-s", type=float, default=0.25)
    parser.add_argument(
        "--closed-loop-max-yaw-disagreement-rad", type=float, default=0.20)
    parser.add_argument(
        "--closed-loop-disagreement-hold-s", type=float, default=0.25)
    parser.add_argument(
        "--closed-loop-max-heading-step-rad", type=float, default=0.20)
    parser.add_argument("--graph-discovery-timeout-s", type=float, default=5.0)
    parser.add_argument(
        "--heading-evidence-root", type=Path,
        default=Path(__file__).resolve().parents[1] / "validation_evidence")
    return parser


def _validate_execution_limits(args, config: SquareConfig) -> None:
    if args.wheel_tick_semantics != "delta":
        raise ValidationError(
            "square execution requires delta /wheel_ticks semantics")
    values = (args.graph_discovery_timeout_s,)
    if any(not math.isfinite(value) or value <= 0.0 for value in values):
        raise ValueError("graph discovery timeout must be finite and positive")
    translation_duration = config.side_length_m / config.linear_velocity_m_s
    rotation_duration = (math.pi / 2.0) / config.angular_velocity_rad_s
    if not (
            args.min_linear_velocity_m_s <= config.linear_velocity_m_s <=
            args.max_linear_velocity_m_s):
        raise ValidationError(
            "square linear velocity is outside existing validation limits")
    if config.angular_velocity_rad_s > args.max_angular_velocity_rad_s:
        raise ValidationError("square angular velocity exceeds existing validation limit")
    if not (
            args.min_translation_duration_s <= translation_duration <=
            args.max_translation_duration_s):
        raise ValidationError(
            "square leg duration is outside existing validation limits")
    if rotation_duration > args.max_rotation_duration_s:
        raise ValidationError("square turn duration exceeds existing validation limit")
    if args.square_control_mode == "closed-loop":
        closed_loop = ClosedLoopYawConfig(
            args.square_angular_velocity_rad_s,
            args.closed_loop_min_angular_velocity_rad_s,
            args.closed_loop_yaw_gain, args.closed_loop_yaw_tolerance_rad,
            args.closed_loop_settle_hold_s, args.closed_loop_turn_timeout_s,
            args.closed_loop_max_yaw_disagreement_rad,
            args.closed_loop_disagreement_hold_s,
            args.closed_loop_max_heading_step_rad)
        if closed_loop.turn_timeout_s > args.max_rotation_duration_s:
            raise ValidationError(
                "closed-loop turn timeout exceeds existing rotation-duration limit")
        if closed_loop.max_disagreement_rad <= closed_loop.yaw_tolerance_rad:
            raise ValidationError(
                "closed-loop disagreement threshold must exceed yaw tolerance")


def _write_square_outputs(
        evidence: EvidenceWriter, directory: Path, report: Dict[str, object],
        trajectories: Dict[str, object],
        segment_manifest: Sequence[Dict[str, object]],
        samples: TrialSamples,
        imu_callback_timing: Sequence[Dict[str, object]],
        closed_loop_feedback: Sequence[Dict[str, object]]) -> None:
    evidence._write_json(directory / "square_report.json", report)
    evidence._write_json(directory / "square_trajectory.json", trajectories)
    evidence._write_json(directory / "segment_manifest.json", {
        "segments": list(segment_manifest)})
    evidence._write_table(directory / "encoder_trajectory.csv", trajectories["encoder"])
    evidence._write_table(directory / "imu_trajectory.csv", trajectories["imu"])
    evidence._write_csv(directory / "continuous_wheel_ticks.csv", samples.wheel_ticks)
    evidence._write_csv(directory / "continuous_raw_imu.csv", samples.imu)
    evidence._write_csv(directory / "continuous_odometry.csv", samples.odom)
    evidence._write_csv(directory / "continuous_diagnostics.csv", samples.diagnostics)
    evidence._write_csv(directory / "continuous_commands.csv", samples.commands)
    evidence._write_table(
        directory / "continuous_imu_callback_timing.csv", imu_callback_timing)
    if closed_loop_feedback:
        evidence._write_table(
            directory / "closed_loop_yaw_feedback.csv", closed_loop_feedback)


def run(args) -> int:
    config = SquareConfig(
        args.square_side_length_m, args.square_linear_velocity_m_s,
        args.square_angular_velocity_rad_s, args.square_direction,
        args.post_stop_settle_s, args.square_between_segment_pause_s)
    _validate_execution_limits(args, config)
    fusion = derive_heading_fusion(args.heading_evidence_root)
    geometry = GeometryConfig(
        args.wheel_radius_m, args.track_width_m,
        args.encoder_ticks_per_revolution)
    closed_loop_config = (
        ClosedLoopYawConfig(
            args.square_angular_velocity_rad_s,
            args.closed_loop_min_angular_velocity_rad_s,
            args.closed_loop_yaw_gain, args.closed_loop_yaw_tolerance_rad,
            args.closed_loop_settle_hold_s, args.closed_loop_turn_timeout_s,
            args.closed_loop_max_yaw_disagreement_rad,
            args.closed_loop_disagreement_hold_s,
            args.closed_loop_max_heading_step_rad)
        if args.square_control_mode == "closed-loop" else None)
    emergency_stop_timeout_s = square_emergency_stop_timeout_s(
        args.zero_publish_timeout_s, args.post_stop_settle_s)
    evidence = EvidenceWriter(Path(args.evidence_root), prefix="odometry-square")
    directory = evidence.create({
        "dry_run": args.dry_run, "geometry": asdict(geometry),
        "square": asdict(config), "command_topic": "/cmd_vel/test",
        "safe_command_topic": "/cmd_vel/safe",
        "square_control_mode": args.square_control_mode,
        "closed_loop_yaw_controller": {
            "active": closed_loop_config is not None,
            "configuration": (
                None if closed_loop_config is None else
                asdict(closed_loop_config)),
            "encoder_weight": CLOSED_LOOP_ENCODER_WEIGHT,
            "imu_weight": CLOSED_LOOP_IMU_WEIGHT,
            "weight_policy": "explicit operator-specified control weights",
        },
        "heading_fusion": asdict(fusion),
        "stationarity_thresholds": {
            "wheel_tick_semantics": args.wheel_tick_semantics,
            "required_delta_samples": args.stationary_samples,
            "tick_delta_tolerance": args.stationary_tick_tolerance,
            "encoder_window_duration_s": ENCODER_STATIONARITY_WINDOW_S,
            "encoder_chatter_max_net_ticks": ENCODER_CHATTER_MAX_NET_TICKS,
            "encoder_chatter_max_absolute_ticks": ENCODER_CHATTER_MAX_ABSOLUTE_TICKS,
            "encoder_chatter_max_sample_delta_ticks": (
                ENCODER_CHATTER_MAX_SAMPLE_DELTA_TICKS),
            "encoder_chatter_max_directional_ticks": (
                ENCODER_CHATTER_MAX_DIRECTIONAL_TICKS),
            "encoder_tick_displacement_mm": (
                2.0 * math.pi * args.wheel_radius_m * 1000.0 /
                args.encoder_ticks_per_revolution),
            "linear_velocity_tolerance_m_s": args.stationary_linear_velocity_tolerance,
            "angular_velocity_tolerance_rad_s": args.stationary_angular_velocity_tolerance,
            "safe_command_zero_tolerance": args.zero_tolerance,
        },
        "imu_bias_rad_s": args.imu_bias_rad_s,
        "stale_timeout_s": args.stale_timeout_s,
        "graph_discovery_timeout_s": args.graph_discovery_timeout_s,
        "emergency_stop_timeout_s": emergency_stop_timeout_s,
        "emergency_stop_timeout_policy": (
            "max(zero_publish_timeout_s, post_stop_settle_s); cleanup resets "
            "its fresh-safe-zero and stationarity windows"),
        "trajectory_time_basis": (
            "ROS time seconds; sensor header stamps with node-clock fallback"),
        "continuous_capture_policy": (
            "subscriptions start before preflight and remain active through final "
            "stationarity and bounded callback drain"),
        "reference_policy": "encoder and processed /imu/data only; TeleDex excluded",
    })
    print(f"evidence_dir={directory}")
    if args.dry_run:
        print("dry_run=true; no nonzero /cmd_vel/test commands will be published")

    rclpy.init(args=None)
    node = SquareValidationNode(args, geometry)
    base_cleanup_controller = EmergencyStopController(
        publish_zero=node.publish_zero, verify_safe_zero=node.verify_safe_zero,
        verify_stationary=node.verify_stationary, sleep=node.spin_for,
        record_result=node.record_emergency_stop,
        stationarity_required=node.stationarity_required,
        begin_stop=node.begin_emergency_stop,
        prepare_verification=node.prepare_emergency_stop_verification,
        verify_stop_guards=node.verify_controlled_stop_guards,
        cleanup_context_valid=node.cleanup_context_valid,
        cleanup_publisher_valid=node.cleanup_publisher_valid,
        confirmed_safe_state=node.confirmed_safe_state)
    cleanup = EmergencyCleanupOnce(SquareEmergencyStopController(
        node, base_cleanup_controller))
    results: List[TrialResult] = []
    segment_measurements: List[Dict[str, float]] = []
    manifest: List[Dict[str, object]] = []
    closed_loop_summaries: List[Dict[str, object]] = []
    trajectory_diagnostics: Optional[Dict[str, object]] = None
    try:
        if args.dry_run:
            node.verify_preflight()
            report = {
                "measurement_status": "not_a_motion_trial",
                "reason": "dry-run performs graph/sensor preflight only",
                "heading_fusion": asdict(fusion),
                "square_control_mode": args.square_control_mode,
                "graph_contract": "verified"}
            evidence._write_json(directory / "square_report.json", report)
            print(json.dumps(report, indent=2, sort_keys=True))
            return 0

        terminal = TerminalLineReader(
            sys.stdin.buffer, sys.stdout, sys.stdin.encoding or "utf-8")
        operator = ResponsiveOperatorInput(
            terminal, lambda: node.spin_for(OPERATOR_CALLBACK_SERVICE_S),
            lambda message: print(message, file=sys.stderr),
            OPERATOR_INPUT_POLL_INTERVAL_S)

        def confirm_motion() -> None:
            prompt = (
                f"Type EXECUTE-SQUARE to execute {config.side_length_m:.3f} m sides, "
                f"linear={config.linear_velocity_m_s:.3f} m/s, "
                f"angular={config.angular_velocity_rad_s:.3f} rad/s, "
                f"control={args.square_control_mode}, "
                f"direction={config.direction.upper()}: ")
            if operator.read_text(prompt).strip() != "EXECUTE-SQUARE":
                raise ValidationError("operator did not confirm square motion")

        node.confirm_motion = confirm_motion

        def execute_segment(index, spec):
            started_s = node._now_seconds()
            result, samples = execute_trial(
                node, args, geometry, spec, operator,
                emergency_cleanup=cleanup,
                collect_physical_measurement=False,
                emergency_stop_timeout_s=emergency_stop_timeout_s)
            if result.measurements.imu_angle_rad is None:
                raise ValidationError(
                    f"processed /imu/data heading unavailable for {spec.trial_id}")
            result = replace(
                result, valid=True,
                operator_notes="square segment accepted by all runtime guards")
            trial_dir = evidence.write_trial(result, samples)
            controller_summary = None
            if (args.square_control_mode == "closed-loop" and
                    spec.movement_type == "rotation"):
                feedback = node.closed_loop_feedback_by_trial.get(spec.trial_id, ())
                controller_summary = summarize_closed_loop_turn(feedback)
                controller_summary.update({
                    "turn_index": len(closed_loop_summaries) + 1,
                    "trial_id": spec.trial_id})
                closed_loop_summaries.append(controller_summary)
                evidence._write_table(
                    trial_dir / "closed_loop_yaw_feedback.csv", feedback)
            results.append(result)
            segment_measurements.append({
                "encoder_distance_m": result.measurements.encoder_distance_m,
                "encoder_angle_rad": result.measurements.encoder_angle_rad,
                "imu_angle_rad": result.measurements.imu_angle_rad})
            stationarity = (
                None if not samples.stationarity else asdict(samples.stationarity[-1]))
            manifest.append({
                "segment_index": index, "trial_id": spec.trial_id,
                "movement_type": spec.movement_type, "direction": spec.direction,
                "command_start_timestamp_s": node.command_start_timestamp_s,
                "command_end_timestamp_s": node.command_end_timestamp_s,
                "stationary_confirmation_timestamp_s": node.stationary_confirmation_timestamp_s,
                "segment_elapsed_s": node._now_seconds() - started_s,
                "stationarity": stationarity, "valid": True,
                "closed_loop_yaw_summary": controller_summary,
                "evidence_dir": str(trial_dir)})
            return result

        def pause_between_segments() -> None:
            started_s = node._now_seconds()
            node.spin_for(config.between_segment_pause_s)
            ended_s = node._now_seconds()
            manifest[-1]["between_segment_pause"] = {
                "requested_s": config.between_segment_pause_s,
                "start_timestamp_s": started_s,
                "end_timestamp_s": ended_s,
                "elapsed_s": ended_s - started_s,
            }

        run_square_sequence(
            config.segments(), execute_segment, pause_between_segments)
        manifest[-1]["between_segment_pause"] = None

        trajectory_start_s = manifest[0]["command_start_timestamp_s"]
        trajectory_end_s = manifest[-1]["stationary_confirmation_timestamp_s"]
        imu_end_covered_after_drain = wait_for_imu_end_coverage(
            lambda: node.continuous_samples().imu, node.spin_for,
            trajectory_end_s, args.stale_timeout_s,
            min(0.01, args.imu_motion_boundary_tolerance_s))
        samples = node.continuous_samples()
        imu_callback_timing = node.imu_callback_timing()
        primary_callback_timing = tuple(
            row for row in imu_callback_timing
            if row["source_topic"] == "/imu/data")
        callback_offsets = tuple(
            row["source_to_callback_offset_s"]
            for row in primary_callback_timing
            if math.isfinite(row["source_to_callback_offset_s"]))
        trajectory_diagnostics = {
            "bounds": {
                "start_timestamp_s": trajectory_start_s,
                "end_timestamp_s": trajectory_end_s,
                "duration_s": trajectory_end_s - trajectory_start_s,
                "time_basis": (
                    "ROS time seconds; sensor header stamps with node-clock fallback"),
            },
            "imu_end_covered_after_bounded_callback_drain": (
                imu_end_covered_after_drain),
            "imu": imu_coverage(
                samples.imu, trajectory_start_s, trajectory_end_s,
                args.stale_timeout_s),
            "imu_callback_delivery": {
                "sample_count": len(primary_callback_timing),
                "max_source_to_callback_offset_s": (
                    None if not callback_offsets else max(callback_offsets)),
                "min_source_to_callback_offset_s": (
                    None if not callback_offsets else min(callback_offsets)),
                "mean_source_to_callback_offset_s": (
                    None if not callback_offsets else
                    sum(callback_offsets) / len(callback_offsets)),
                "invalid_offset_count": (
                    len(primary_callback_timing) - len(callback_offsets)),
                "callback_receipt_coverage": timestamp_coverage(
                    tuple(row["callback_received_timestamp_s"]
                          for row in primary_callback_timing),
                    trajectory_start_s, trajectory_end_s,
                    args.stale_timeout_s),
            },
            "wheel_ticks": timestamp_coverage(
                tuple(sample.timestamp_s for sample in samples.wheel_ticks),
                trajectory_start_s, trajectory_end_s, args.stale_timeout_s),
            "odometry": timestamp_coverage(
                tuple(sample.timestamp_s for sample in samples.odom),
                trajectory_start_s, trajectory_end_s, args.stale_timeout_s),
            "commands": timestamp_coverage(
                tuple(sample.timestamp_s for sample in samples.commands),
                trajectory_start_s, trajectory_end_s, args.stale_timeout_s),
        }
        require_complete_imu_coverage(trajectory_diagnostics["imu"])
        for stream_name in ("wheel_ticks", "odometry", "commands"):
            stream = trajectory_diagnostics[stream_name]
            if (stream["invalid_timestamp_count"] or
                    stream["out_of_order_timestamp_count"] or
                    not stream["overlaps_trajectory"] or
                    stream["coverage_before_start_s"] is None or
                    stream["coverage_before_start_s"] < 0.0 or
                    stream["coverage_after_end_s"] is None or
                    stream["coverage_after_end_s"] < 0.0 or
                    stream["gaps_over_threshold_count"]):
                raise ValidationError(
                    f"{stream_name} does not continuously bracket square "
                    "trajectory bounds")
        trajectory_wheels = tuple(
            sample for sample in samples.wheel_ticks
            if trajectory_start_s <= sample.timestamp_s <= trajectory_end_s)
        trajectories = reconstruct_trajectories(
            trajectory_wheels, samples.imu, geometry, args.imu_bias_rad_s,
            args.wheel_tick_semantics, trajectory_start_s, trajectory_end_s)
        report = apply_continuous_final_metrics(
            square_metrics(config, segment_measurements, fusion), trajectories)
        continuous_imu_yaw = report["imu_heading_final_yaw_rad"]
        report.update({
            "measurement_status": "complete_pre_calibration_baseline",
            "completed_segment_count": len(results),
            "aborted_or_invalid_segments": [],
            "continuous_encoder_final_pose": trajectories["encoder_final_pose"],
            "continuous_imu_final_yaw_rad": continuous_imu_yaw,
            "square_control_mode": args.square_control_mode,
            "closed_loop_control_fusion": {
                "encoder_weight": CLOSED_LOOP_ENCODER_WEIGHT,
                "imu_weight": CLOSED_LOOP_IMU_WEIGHT,
                "statistically_derived": False},
            "closed_loop_turns": closed_loop_summaries,
            "trajectory_diagnostics": trajectory_diagnostics,
            "segment_timing_and_stationarity": manifest})
        _write_square_outputs(
            evidence, directory, report, trajectories, manifest, samples,
            imu_callback_timing, node.closed_loop_feedback)
        evidence.write_summary(results)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    except BaseException as primary_error:
        failure: BaseException = primary_error
        if not cleanup.attempted:
            try:
                cleanup.stop(
                    emergency_stop_timeout_s, args.zero_publish_rate_hz)
            except BaseException as cleanup_error:
                failure = EmergencyStopCleanupError(primary_error, cleanup_error)
        samples = node.continuous_samples()
        imu_callback_timing = node.imu_callback_timing()
        failure_context = node.failure_context()
        failure_context["stationarity_thresholds"][
            "effective_square_emergency_cleanup_timeout_s"] = (
                emergency_stop_timeout_s)
        failure_context.update({
            "completed_segments": manifest,
            "next_segment_index": len(results) + 1,
            "remaining_square_aborted": True,
            "cleanup_attempted": cleanup.attempted,
            "cleanup_completed": cleanup.completed,
            "emergency_stop_timeout_s": emergency_stop_timeout_s,
            "heading_fusion": asdict(fusion),
            "square_control_mode": args.square_control_mode,
            "closed_loop_controller": (
                None if closed_loop_config is None else
                asdict(closed_loop_config)),
            "closed_loop_turns_completed": closed_loop_summaries,
            "trajectory_diagnostics": trajectory_diagnostics})
        try:
            callback_path = directory / "continuous_imu_callback_timing.csv"
            if not callback_path.exists():
                evidence._write_table(callback_path, imu_callback_timing)
            feedback_path = directory / "closed_loop_yaw_feedback.csv"
            if node.closed_loop_feedback and not feedback_path.exists():
                evidence._write_table(
                    feedback_path, node.closed_loop_feedback)
            evidence.write_failure(
                results, failure, samples, failure_context=failure_context)
        except BaseException as evidence_error:
            print(f"failed to write square failure evidence: {evidence_error}",
                  file=sys.stderr)
        if failure is not primary_error:
            raise failure from primary_error
        raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_square_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)
    try:
        return run(args)
    except KeyboardInterrupt:
        print("interrupted; square zero-command cleanup was attempted",
              file=sys.stderr)
        return 130
    except Exception as error:
        print(f"square validation failed: {error}", file=sys.stderr)
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
