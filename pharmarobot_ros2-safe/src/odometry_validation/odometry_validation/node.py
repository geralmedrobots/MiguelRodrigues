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

"""ROS 2 node for odometry validation orchestration and data collection."""

import argparse
from dataclasses import asdict
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from threading import RLock
import time
import traceback
from typing import Callable, Dict, List, Optional, Sequence, Set, Tuple

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from roboteq_ros2_driver.msg import WheelTicks
from sensor_msgs.msg import Imu

from odometry_validation.campaign import CAMPAIGN_MODE_LABELS
from odometry_validation.campaign import CAMPAIGN_MODES
from odometry_validation.campaign import TRANSLATION_HEADING_LIMIT_DEG
from odometry_validation.campaign import attempt_record
from odometry_validation.campaign import atomic_write_json
from odometry_validation.campaign import campaign_progress
from odometry_validation.campaign import campaign_matrix_identity
from odometry_validation.campaign import condition_summary
from odometry_validation.campaign import covariance_analysis
from odometry_validation.campaign import fixed_campaign_conditions
from odometry_validation.campaign import heading_deviation_auto_invalid
from odometry_validation.campaign import next_pending
from odometry_validation.campaign import planned_valid_repetition_count
from odometry_validation.campaign import radius_calibration_analysis
from odometry_validation.campaign import translation_reference_distance_m
from odometry_validation.campaign import validation_comparison
from odometry_validation.campaign import whole_campaign_analysis
from odometry_validation.campaign import write_csv
from odometry_validation.core import CommandSample
from odometry_validation.core import DEFAULT_ROTATION_DURATIONS_S
from odometry_validation.core import DEFAULT_ROTATION_VELOCITIES_RAD_S
from odometry_validation.core import DEFAULT_TRANSLATION_DURATIONS_S
from odometry_validation.core import DEFAULT_TRANSLATION_VELOCITIES_M_S
from odometry_validation.core import diagnostic_level_to_int
from odometry_validation.core import DiagnosticSample
from odometry_validation.core import EmergencyStopController
from odometry_validation.core import EmergencyCleanupOnce
from odometry_validation.core import EmergencyStopCleanupError
from odometry_validation.core import EvidenceWriter
from odometry_validation.core import GeometryConfig
from odometry_validation.core import ImuSample
from odometry_validation.core import InteractiveLimits
from odometry_validation.core import InteractiveCampaignMenu
from odometry_validation.core import InteractiveTrialMenu
from odometry_validation.core import OdomSample
from odometry_validation.core import OperatorInterface
from odometry_validation.core import ResponsiveOperatorInput
from odometry_validation.core import StationarityAssessment
from odometry_validation.core import StationaritySample
from odometry_validation.core import TerminalLineReader
from odometry_validation.core import TrialSamples
from odometry_validation.core import TrialSpec
from odometry_validation.core import TrialResult
from odometry_validation.core import ValidationError
from odometry_validation.core import WheelTickSample
from odometry_validation.core import build_measurements
from odometry_validation.core import build_trial_report
from odometry_validation.core import compass_rotation_radians
from odometry_validation.core import generate_rotation_trials
from odometry_validation.core import generate_translation_trials
from odometry_validation.core import make_trial_result
from odometry_validation.core import laser_rotation_reference
from odometry_validation.core import laser_translation_reference
from odometry_validation.core import apply_laser_translation_quality
from odometry_validation.core import merge_trial_samples
from odometry_validation.core import run_with_emergency_stop
from odometry_validation.core import render_trial_report
from odometry_validation.core import utc_timestamp
from odometry_validation.teledex_adapter import TeleDexMountMappingMismatch
from odometry_validation.teledex_adapter import TeleDexReferenceAdapter


CMD_VEL_TEST_TOPIC = "/cmd_vel/test"
CMD_VEL_SAFE_TOPIC = "/cmd_vel/safe"
WHEEL_TICKS_TOPIC = "/wheel_ticks"
ODOM_TOPIC = "/odom"
PRIMARY_IMU_TOPIC = "/imu/data"
D455_IMU_TOPIC = "/imu/d455/data_raw"
DIAGNOSTICS_TOPIC = "/diagnostics"
DEFAULT_REQUIRED_TOPICS = (
    CMD_VEL_SAFE_TOPIC,
    WHEEL_TICKS_TOPIC,
    ODOM_TOPIC,
    PRIMARY_IMU_TOPIC,
    D455_IMU_TOPIC,
    DIAGNOSTICS_TOPIC,
)
DEFAULT_REQUIRED_NODES = ("command_arbiter",)
OPERATOR_INPUT_POLL_INTERVAL_S = 0.05
OPERATOR_CALLBACK_SERVICE_S = 0.02
# Evidence-derived post-stop encoder chatter envelope.  The 0.35 s minimum
# spans the five-sample stationary windows accepted by prior campaigns while
# allowing normal ~10 Hz callback jitter.  The remaining limits are one tick
# net, four ticks total excursion, and no directional run longer than one tick;
# this accepts alternating +/-1 chatter but rejects sustained accumulation.
ENCODER_STATIONARITY_WINDOW_S = 0.35
ENCODER_CHATTER_MAX_NET_TICKS = 1
ENCODER_CHATTER_MAX_ABSOLUTE_TICKS = 4
ENCODER_CHATTER_MAX_SAMPLE_DELTA_TICKS = 1
ENCODER_CHATTER_MAX_DIRECTIONAL_TICKS = 1


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quaternion_to_yaw(orientation) -> float:
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def twist_is_zero(message: Twist, tolerance: float) -> bool:
    return (
        abs(message.linear.x) <= tolerance and
        abs(message.linear.y) <= tolerance and
        abs(message.linear.z) <= tolerance and
        abs(message.angular.x) <= tolerance and
        abs(message.angular.y) <= tolerance and
        abs(message.angular.z) <= tolerance)


def imu_qos_profile(depth: int) -> QoSProfile:
    """Return a depth-configurable sensor-data-compatible IMU QoS profile."""
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE)


class OdometryValidationNode(Node):
    """Test orchestrator that uses the existing robot ROS graph."""

    def __init__(self, args, context=None):
        super().__init__("odometry_validation", context=context)
        self.args = args
        qos_depth = int(args.qos_depth)
        self.command_publisher = self.create_publisher(
            Twist, CMD_VEL_TEST_TOPIC, qos_depth)
        self.safe_commands: List[CommandSample] = []
        self.test_commands: List[CommandSample] = []
        self.wheel_ticks: List[WheelTickSample] = []
        self.imu: List[ImuSample] = []
        self.odom: List[OdomSample] = []
        self.diagnostics: List[DiagnosticSample] = []
        self.ignored_diagnostics: List[DiagnosticSample] = []
        self.stationarity_assessments: List[StationarityAssessment] = []
        self.emergency_stop_records: List[Dict[str, object]] = []
        self.zero_publish_count = 0
        self.motion_armed = False
        self.nonzero_command_published = False
        self.sample_phase = "before_motion"
        self.first_zero_timestamp_s: Optional[float] = None
        self.first_safe_zero_timestamp_s: Optional[float] = None
        self.first_encoder_stationary_timestamp_s: Optional[float] = None
        self.first_odom_stationary_timestamp_s: Optional[float] = None
        self.last_encoder_motion_timestamp_s: Optional[float] = None
        self.last_nonzero_odom_twist_timestamp_s: Optional[float] = None
        self.command_start_timestamp_s: Optional[float] = None
        self.command_end_timestamp_s: Optional[float] = None
        self.last_test_command_timestamp_s: Optional[float] = None
        self.last_test_command_safe_index: Optional[int] = None
        self.stationary_confirmation_timestamp_s: Optional[float] = None
        self.post_zero_wheel_start: Optional[int] = None
        self.post_zero_imu_start: Optional[int] = None
        self.post_zero_odom_start: Optional[int] = None
        self.post_zero_safe_start: Optional[int] = None
        self.ignored_diagnostic_names = frozenset(
            getattr(args, "ignore_diagnostic", ()))
        self.last_safe: Optional[Twist] = None
        self.last_safe_time: Optional[float] = None
        self.last_wheel_time: Optional[float] = None
        self.last_imu_time: Optional[float] = None
        self.last_primary_imu_time: Optional[float] = None
        self.last_odom_time: Optional[float] = None
        self.last_diagnostic_level = 0
        self.graph_nodes: Optional[Set[str]] = None
        self.graph_topics: Optional[Set[str]] = None
        self._samples_lock = RLock()

        self.create_subscription(
            Twist, CMD_VEL_SAFE_TOPIC, self._safe_command_callback, qos_depth)
        self.create_subscription(
            WheelTicks, WHEEL_TICKS_TOPIC, self._wheel_ticks_callback, qos_depth)
        self.create_subscription(
            Odometry, ODOM_TOPIC, self._odom_callback, qos_depth)
        imu_qos = imu_qos_profile(qos_depth)
        self.create_subscription(
            Imu, PRIMARY_IMU_TOPIC,
            lambda message: self._imu_callback(message, PRIMARY_IMU_TOPIC),
            imu_qos)
        self.create_subscription(
            Imu, D455_IMU_TOPIC,
            lambda message: self._imu_callback(message, D455_IMU_TOPIC),
            imu_qos)
        self.create_subscription(
            DiagnosticArray,
            DIAGNOSTICS_TOPIC,
            self._diagnostics_callback,
            qos_depth)

    def _safe_command_callback(self, message: Twist) -> None:
        timestamp = self._now_seconds()
        self.last_safe = message
        self.last_safe_time = timestamp
        with self._samples_lock:
            self.safe_commands.append(CommandSample(
                timestamp_s=timestamp,
                topic=CMD_VEL_SAFE_TOPIC,
                linear_x_m_s=message.linear.x,
                angular_z_rad_s=message.angular.z,
                phase=self.sample_phase))

    def _wheel_ticks_callback(self, message: WheelTicks) -> None:
        timestamp = stamp_to_seconds(message.header.stamp) or self._now_seconds()
        self.last_wheel_time = self._now_seconds()
        with self._samples_lock:
            self.wheel_ticks.append(WheelTickSample(
                timestamp_s=timestamp,
                left_ticks=message.left_ticks,
                right_ticks=message.right_ticks,
                phase=self.sample_phase))

    def _imu_callback(
            self, message: Imu,
            source_topic: str = PRIMARY_IMU_TOPIC) -> None:
        timestamp = stamp_to_seconds(message.header.stamp) or self._now_seconds()
        received_at = self._now_seconds()
        self.last_imu_time = received_at
        if source_topic == PRIMARY_IMU_TOPIC:
            self.last_primary_imu_time = received_at
        with self._samples_lock:
            self.imu.append(ImuSample(
                timestamp_s=timestamp,
                angular_velocity_z_rad_s=message.angular_velocity.z,
                phase=self.sample_phase,
                source_topic=source_topic))

    def _odom_callback(self, message: Odometry) -> None:
        timestamp = stamp_to_seconds(message.header.stamp) or self._now_seconds()
        self.last_odom_time = self._now_seconds()
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        with self._samples_lock:
            self.odom.append(OdomSample(
                timestamp_s=timestamp,
                x_m=position.x,
                y_m=position.y,
                yaw_rad=quaternion_to_yaw(orientation),
                linear_x_m_s=message.twist.twist.linear.x,
                angular_z_rad_s=message.twist.twist.angular.z,
                phase=self.sample_phase))

    def _diagnostics_callback(self, message: DiagnosticArray) -> None:
        timestamp = stamp_to_seconds(message.header.stamp) or self._now_seconds()
        for status in message.status:
            level_is_valid = True
            try:
                level = diagnostic_level_to_int(status.level)
            except (TypeError, ValueError):
                level = 255
                level_is_valid = False
                self.get_logger().error(
                    "invalid diagnostic status level; marking diagnostics unhealthy")
            sample = DiagnosticSample(
                timestamp_s=timestamp,
                level=level,
                name=status.name,
                message=status.message)
            with self._samples_lock:
                self.diagnostics.append(sample)
                if level_is_valid and status.name in self.ignored_diagnostic_names:
                    self.ignored_diagnostics.append(sample)
                    continue
                self.last_diagnostic_level = max(self.last_diagnostic_level, level)

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def reset_trial_buffers(self) -> None:
        with self._samples_lock:
            self.safe_commands.clear()
            self.test_commands.clear()
            self.wheel_ticks.clear()
            self.imu.clear()
            self.odom.clear()
            self.diagnostics.clear()
            self.ignored_diagnostics.clear()
            self.stationarity_assessments.clear()
        self.emergency_stop_records.clear()
        self.zero_publish_count = 0
        self.motion_armed = False
        self.nonzero_command_published = False
        self.sample_phase = "before_motion"
        self.first_zero_timestamp_s = None
        self.first_safe_zero_timestamp_s = None
        self.first_encoder_stationary_timestamp_s = None
        self.first_odom_stationary_timestamp_s = None
        self.last_encoder_motion_timestamp_s = None
        self.last_nonzero_odom_twist_timestamp_s = None
        self.command_start_timestamp_s = None
        self.command_end_timestamp_s = None
        self.last_test_command_timestamp_s = None
        self.last_test_command_safe_index = None
        self.stationary_confirmation_timestamp_s = None
        self.post_zero_wheel_start = None
        self.post_zero_imu_start = None
        self.post_zero_odom_start = None
        self.post_zero_safe_start = None
        self.last_wheel_time = None
        self.last_imu_time = None
        self.last_primary_imu_time = None
        self.last_odom_time = None
        self.last_safe = None
        self.last_safe_time = None
        self.last_diagnostic_level = 0

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, duration_s))

    def snapshot_graph(self) -> Tuple[Set[str], Set[str]]:
        nodes = set(self.get_node_names())
        topics = {name for name, _types in self.get_topic_names_and_types()}
        return nodes, topics

    def latch_expected_graph(self) -> None:
        self.graph_nodes, self.graph_topics = self.snapshot_graph()

    def verify_graph_unchanged(self) -> None:
        if self.graph_nodes is None or self.graph_topics is None:
            return
        nodes, topics = self.snapshot_graph()
        if nodes != self.graph_nodes or topics != self.graph_topics:
            raise ValidationError("ROS graph changed unexpectedly")

    def verify_preflight(self) -> None:
        self.spin_for(self.args.preflight_spin_s)
        nodes, topics = self.snapshot_graph()
        missing_nodes = [
            node for node in self.args.required_node
            if node not in nodes and f"/{node}" not in nodes]
        missing_topics = [
            topic for topic in self.args.required_topic if topic not in topics]
        if missing_nodes:
            raise ValidationError(f"missing required nodes: {missing_nodes}")
        if missing_topics:
            raise ValidationError(f"missing required topics: {missing_topics}")
        now = self._now_seconds()
        if self.last_wheel_time is None or not self.wheel_ticks:
            raise ValidationError("encoder messages are not available")
        if now - self.last_wheel_time > self.args.stale_timeout_s:
            raise ValidationError("encoder messages are stale")
        if self.last_primary_imu_time is None or not any(
                sample.source_topic == PRIMARY_IMU_TOPIC for sample in self.imu):
            raise ValidationError("primary IMU messages are not available")
        if now - self.last_primary_imu_time > self.args.stale_timeout_s:
            raise ValidationError("primary IMU messages are stale")
        if self.last_odom_time is None or not self.odom:
            raise ValidationError("odometry messages are not available")
        if now - self.last_odom_time > self.args.stale_timeout_s:
            raise ValidationError("odometry messages are stale")
        if self.last_safe is None or not self.safe_commands:
            raise ValidationError("/cmd_vel/safe is not available")
        if not twist_is_zero(self.last_safe, self.args.zero_tolerance):
            raise ValidationError("/cmd_vel/safe is not zero before trial")
        if self.last_diagnostic_level > self.args.max_diagnostic_level:
            raise ValidationError("diagnostics are not healthy")
        self.latch_expected_graph()

    def verify_ready_after_operator_input(self) -> None:
        """Recheck motion-critical observations after a blocking operator wait."""
        self.verify_graph_unchanged()
        now = self._now_seconds()
        if (
                self.last_wheel_time is None or
                now - self.last_wheel_time > self.args.stale_timeout_s):
            raise ValidationError("stale encoder before command")
        if (
                self.last_primary_imu_time is None or
                now - self.last_primary_imu_time > self.args.stale_timeout_s):
            raise ValidationError("stale primary IMU before command")
        if self.last_diagnostic_level > self.args.max_diagnostic_level:
            raise ValidationError("diagnostics failure before command")
        if self.last_safe is None:
            raise ValidationError("missing /cmd_vel/safe before command")
        if not twist_is_zero(self.last_safe, self.args.zero_tolerance):
            raise ValidationError("/cmd_vel/safe is not zero before command")

    def publish_twist(self, linear_x: float, angular_z: float) -> None:
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = angular_z
        self.last_test_command_safe_index = len(self.safe_commands)
        self.last_test_command_timestamp_s = self._now_seconds()
        self.command_publisher.publish(message)
        if not twist_is_zero(message, self.args.zero_tolerance):
            self.nonzero_command_published = True
        with self._samples_lock:
            self.test_commands.append(CommandSample(
                timestamp_s=self._now_seconds(),
                topic=CMD_VEL_TEST_TOPIC,
                linear_x_m_s=linear_x,
                angular_z_rad_s=angular_z,
                phase=self.sample_phase))

    def publish_zero(self) -> None:
        if self.first_zero_timestamp_s is None:
            self.first_zero_timestamp_s = self._now_seconds()
        self.sample_phase = "after_motion"
        self.zero_publish_count += 1
        self.publish_twist(0.0, 0.0)

    def begin_emergency_stop(self) -> None:
        self.first_zero_timestamp_s = None
        self.first_safe_zero_timestamp_s = None
        self.first_encoder_stationary_timestamp_s = None
        self.first_odom_stationary_timestamp_s = None
        self.last_encoder_motion_timestamp_s = None
        self.last_nonzero_odom_twist_timestamp_s = None
        self.post_zero_wheel_start = len(self.wheel_ticks)
        self.post_zero_imu_start = len(self.imu)
        self.post_zero_odom_start = len(self.odom)
        self.post_zero_safe_start = len(self.safe_commands)

    def prepare_emergency_stop_verification(self) -> None:
        if self.post_zero_safe_start is None:
            self.post_zero_safe_start = len(self.safe_commands)

    def verify_safe_zero(self) -> bool:
        if self.post_zero_safe_start is None:
            return False
        fresh_safe_commands = self.safe_commands[self.post_zero_safe_start:]
        return (
            bool(fresh_safe_commands) and
            self.last_safe is not None and
            twist_is_zero(self.last_safe, self.args.zero_tolerance))

    def verify_controlled_stop_guards(self) -> None:
        """Fail immediately on faults that are not normal settling motion."""
        self.verify_graph_unchanged()
        now = self._now_seconds()
        if self.first_zero_timestamp_s is None:
            raise ValidationError(
                "controlled stop verification was not initialized")
        post_zero_sources = (
            ("encoder", self.wheel_ticks, self.post_zero_wheel_start, lambda _s: True),
            ("primary IMU", self.imu, self.post_zero_imu_start,
             lambda sample: sample.source_topic == PRIMARY_IMU_TOPIC),
            ("odometry", self.odom, self.post_zero_odom_start, lambda _s: True),
            ("/cmd_vel/safe", self.safe_commands, self.post_zero_safe_start,
             lambda _s: True),
        )
        for label, samples, start, source_matches in post_zero_sources:
            qualifying = (
                () if start is None else tuple(
                    sample for sample in samples[start:]
                    if (source_matches(sample) and
                        sample.timestamp_s >= self.first_zero_timestamp_s)))
            freshness_timestamp_s = (
                qualifying[-1].timestamp_s
                if qualifying else self.first_zero_timestamp_s)
            if now - freshness_timestamp_s > self.args.stale_timeout_s:
                raise ValidationError(f"stale {label} during controlled stop")
        if self.last_diagnostic_level > self.args.max_diagnostic_level:
            raise ValidationError("diagnostics failure during controlled stop")
        if self.post_zero_safe_start is None:
            raise ValidationError(
                "controlled stop verification was not initialized")
        fresh_safe_commands = self.safe_commands[self.post_zero_safe_start:]
        if (
                fresh_safe_commands and self.last_safe is not None and
                not twist_is_zero(self.last_safe, self.args.zero_tolerance)):
            sample_age = (
                None if self.last_safe_time is None else
                max(0.0, self._now_seconds() - self.last_safe_time))
            raise ValidationError(
                "command mismatch on /cmd_vel/safe during controlled stop: "
                f"expected linear=0.000000, angular=0.000000; observed "
                f"linear={self.last_safe.linear.x:.6f}, "
                f"angular={self.last_safe.angular.z:.6f}; "
                f"sample_age_s={sample_age}; "
                f"phase={self.sample_phase}")

    def verify_stationary(self) -> StationarityAssessment:
        assessment_timestamp_s = self._now_seconds()
        required = self.args.stationary_samples
        if self.post_zero_wheel_start is None:
            post_zero_ticks = ()
        else:
            post_zero_ticks = tuple(
                sample
                for sample in self.wheel_ticks[self.post_zero_wheel_start:]
                if (
                    self.first_zero_timestamp_s is not None and
                    sample.timestamp_s >= self.first_zero_timestamp_s))
        semantics = self.args.wheel_tick_semantics
        if semantics == "cumulative":
            all_delta_samples = tuple(
                StationaritySample(
                    previous_timestamp_s=previous.timestamp_s,
                    timestamp_s=current.timestamp_s,
                    previous_left_ticks=previous.left_ticks,
                    previous_right_ticks=previous.right_ticks,
                    left_ticks=current.left_ticks,
                    right_ticks=current.right_ticks,
                    left_delta_ticks=current.left_ticks - previous.left_ticks,
                    right_delta_ticks=current.right_ticks - previous.right_ticks)
                for previous, current in zip(
                    post_zero_ticks, post_zero_ticks[1:]))
        else:
            all_delta_samples = tuple(
                StationaritySample(
                    previous_timestamp_s=sample.timestamp_s,
                    timestamp_s=sample.timestamp_s,
                    previous_left_ticks=0,
                    previous_right_ticks=0,
                    left_ticks=sample.left_ticks,
                    right_ticks=sample.right_ticks,
                    left_delta_ticks=sample.left_ticks,
                    right_delta_ticks=sample.right_ticks)
                for sample in post_zero_ticks)
        window_limit_s = getattr(
            self.args, "stationarity_window_s", ENCODER_STATIONARITY_WINDOW_S)

        def sample_window_duration(samples):
            if not samples:
                return None
            start = (samples[0].timestamp_s if semantics == "delta" else
                     samples[0].previous_timestamp_s)
            return max(0.0, samples[-1].timestamp_s - start)

        delta_samples = all_delta_samples[-required:]
        while (len(delta_samples) < len(all_delta_samples) and
               (sample_window_duration(delta_samples) or 0.0) + 1e-9 <
               window_limit_s):
            delta_samples = all_delta_samples[-(len(delta_samples) + 1):]
        window_start_timestamp_s = (
            None if not delta_samples else
            (delta_samples[0].timestamp_s if semantics == "delta" else
             delta_samples[0].previous_timestamp_s))
        window_end_timestamp_s = (
            None if not delta_samples else
            delta_samples[-1].timestamp_s)
        window_duration_s = (
            None if window_start_timestamp_s is None else
            max(0.0, window_end_timestamp_s - window_start_timestamp_s))
        enough_ticks = (
            len(delta_samples) >= required and
            window_duration_s is not None and
            window_duration_s + 1e-9 >= window_limit_s)
        left_values = tuple(sample.left_delta_ticks for sample in delta_samples)
        right_values = tuple(sample.right_delta_ticks for sample in delta_samples)
        encoder_data_valid = all(
            math.isfinite(float(sample.timestamp_s)) and
            math.isfinite(float(sample.left_ticks)) and
            math.isfinite(float(sample.right_ticks))
            for sample in post_zero_ticks)
        encoder_data_valid = encoder_data_valid and all(
            current.timestamp_s >= previous.timestamp_s
            for previous, current in zip(post_zero_ticks, post_zero_ticks[1:]))
        encoder_data_age_s = (
            None if self.last_wheel_time is None else
            max(0.0, assessment_timestamp_s - self.last_wheel_time))
        encoder_data_fresh = (
            encoder_data_age_s is not None and
            encoder_data_age_s <= self.args.stale_timeout_s)

        def directional_accumulation(values):
            running = 0
            previous_sign = 0
            maximum = 0
            for value in values:
                sign = 1 if value > 0 else -1 if value < 0 else 0
                if sign == 0 or sign != previous_sign:
                    running = abs(value)
                else:
                    running += abs(value)
                maximum = max(maximum, running)
                previous_sign = sign
            return maximum

        left_net_ticks = sum(left_values) if delta_samples else None
        right_net_ticks = sum(right_values) if delta_samples else None
        left_absolute_ticks = sum(abs(value) for value in left_values) if delta_samples else None
        right_absolute_ticks = sum(abs(value) for value in right_values) if delta_samples else None
        maximum_individual_delta_ticks = (
            max((max(abs(left), abs(right)) for left, right in
                 zip(left_values, right_values)), default=None))
        left_directional_ticks = directional_accumulation(left_values)
        right_directional_ticks = directional_accumulation(right_values)
        encoder_window_failures = []
        if not enough_ticks:
            encoder_window_failures.append(
                "insufficient encoder deltas/observation window "
                f"({len(delta_samples)}/{required} samples, "
                f"span {window_duration_s or 0.0:.3f}/"
                f"{window_limit_s:.3f}s)")
        if not encoder_data_valid:
            encoder_window_failures.append("encoder data was invalid or non-monotonic")
        if not encoder_data_fresh:
            encoder_window_failures.append(
                "encoder data became stale during stop verification")
        if (maximum_individual_delta_ticks is not None and
                maximum_individual_delta_ticks > ENCODER_CHATTER_MAX_SAMPLE_DELTA_TICKS):
            encoder_window_failures.append(
                "maximum encoder sample delta exceeded chatter bound")
        if (left_net_ticks is not None and
                abs(left_net_ticks) > ENCODER_CHATTER_MAX_NET_TICKS):
            encoder_window_failures.append("left encoder net displacement exceeded chatter bound")
        if (right_net_ticks is not None and
                abs(right_net_ticks) > ENCODER_CHATTER_MAX_NET_TICKS):
            encoder_window_failures.append("right encoder net displacement exceeded chatter bound")
        if (left_absolute_ticks is not None and
                left_absolute_ticks > ENCODER_CHATTER_MAX_ABSOLUTE_TICKS):
            encoder_window_failures.append(
                "left encoder cumulative excursion exceeded chatter bound")
        if (right_absolute_ticks is not None and
                right_absolute_ticks > ENCODER_CHATTER_MAX_ABSOLUTE_TICKS):
            encoder_window_failures.append(
                "right encoder cumulative excursion exceeded chatter bound")
        if left_directional_ticks > ENCODER_CHATTER_MAX_DIRECTIONAL_TICKS:
            encoder_window_failures.append(
                "left encoder directional accumulation exceeded chatter bound")
        if right_directional_ticks > ENCODER_CHATTER_MAX_DIRECTIONAL_TICKS:
            encoder_window_failures.append(
                "right encoder directional accumulation exceeded chatter bound")
        ticks_stationary = not encoder_window_failures
        if left_absolute_ticks is None or right_absolute_ticks is None:
            encoder_activity_class = "insufficient_window"
        elif encoder_window_failures:
            encoder_activity_class = (
                "directional_motion" if (
                    (left_net_ticks is not None and
                     abs(left_net_ticks) > ENCODER_CHATTER_MAX_NET_TICKS) or
                    (right_net_ticks is not None and
                     abs(right_net_ticks) > ENCODER_CHATTER_MAX_NET_TICKS) or
                    left_directional_ticks > ENCODER_CHATTER_MAX_DIRECTIONAL_TICKS or
                    right_directional_ticks > ENCODER_CHATTER_MAX_DIRECTIONAL_TICKS)
                else "excessive_chatter")
        elif left_absolute_ticks == 0 and right_absolute_ticks == 0:
            encoder_activity_class = "zero_activity"
        else:
            encoder_activity_class = "alternating_chatter"
        moving_encoder_samples = tuple(
            sample for sample in all_delta_samples
            if (abs(sample.left_delta_ticks) >
                self.args.stationary_tick_tolerance or
                abs(sample.right_delta_ticks) >
                self.args.stationary_tick_tolerance))
        if moving_encoder_samples:
            self.last_encoder_motion_timestamp_s = (
                moving_encoder_samples[-1].timestamp_s)
        if (ticks_stationary and
                self.first_encoder_stationary_timestamp_s is None):
            self.first_encoder_stationary_timestamp_s = assessment_timestamp_s

        if self.post_zero_odom_start is None:
            post_zero_odom = ()
        else:
            post_zero_odom = tuple(
                sample
                for sample in self.odom[self.post_zero_odom_start:]
                if (
                    self.first_zero_timestamp_s is not None and
                    sample.timestamp_s >= self.first_zero_timestamp_s))
        odom_window = post_zero_odom[-required:]
        enough_odom = len(odom_window) >= required
        odom_stationary = enough_odom and all(
            abs(sample.linear_x_m_s) <=
            self.args.stationary_linear_velocity_tolerance and
            abs(sample.angular_z_rad_s) <=
            self.args.stationary_angular_velocity_tolerance
            for sample in odom_window)
        nonzero_odom_samples = tuple(
            sample for sample in post_zero_odom
            if (sample.linear_x_m_s != 0.0 or
                sample.angular_z_rad_s != 0.0))
        if nonzero_odom_samples:
            self.last_nonzero_odom_twist_timestamp_s = (
                nonzero_odom_samples[-1].timestamp_s)
        if (odom_stationary and
                self.first_odom_stationary_timestamp_s is None):
            self.first_odom_stationary_timestamp_s = assessment_timestamp_s
        if self.post_zero_safe_start is None:
            post_zero_safe = ()
        else:
            post_zero_safe = tuple(
                self.safe_commands[self.post_zero_safe_start:])
        safe_command_age_s = (
            None if self.last_safe_time is None else
            max(0.0, assessment_timestamp_s - self.last_safe_time))
        safe_command_fresh = (
            safe_command_age_s is not None and
            safe_command_age_s <= self.args.stale_timeout_s)
        safe_is_zero = (
            bool(post_zero_safe) and
            self.last_safe is not None and
            twist_is_zero(self.last_safe, self.args.zero_tolerance))
        safe_zero = safe_is_zero and safe_command_fresh
        tick_mm = None
        if (hasattr(self.args, "wheel_radius_m") and
                hasattr(self.args, "encoder_ticks_per_revolution")):
            tick_mm = (2.0 * math.pi * self.args.wheel_radius_m * 1000.0 /
                       self.args.encoder_ticks_per_revolution)
        if safe_zero and self.first_safe_zero_timestamp_s is None:
            self.first_safe_zero_timestamp_s = self.last_safe_time
        failures = []
        if not ticks_stationary:
            failures.extend(encoder_window_failures)
        if not enough_odom:
            failures.append(
                f"insufficient odometry twist samples ({len(odom_window)}/{required})")
        elif not odom_stationary:
            failures.append("odometry twist exceeded stationarity tolerance")
        if not post_zero_safe:
            failures.append("no fresh /cmd_vel/safe sample after zero publication")
        elif not safe_is_zero:
            failures.append("/cmd_vel/safe was not zero")
        elif not safe_command_fresh:
            failures.append(
                "/cmd_vel/safe sample became stale during stop verification "
                f"(age {safe_command_age_s:.6f}s > "
                f"{self.args.stale_timeout_s:.6f}s)")
        assessment = StationarityAssessment(
            stationary=ticks_stationary and odom_stationary and safe_zero,
            reason="stationary" if not failures else "; ".join(failures),
            required_delta_samples=required,
            observed_delta_samples=len(delta_samples),
            tick_delta_tolerance=self.args.stationary_tick_tolerance,
            linear_velocity_tolerance_m_s=(
                self.args.stationary_linear_velocity_tolerance),
            angular_velocity_tolerance_rad_s=(
                self.args.stationary_angular_velocity_tolerance),
            first_zero_timestamp_s=self.first_zero_timestamp_s,
            wheel_tick_semantics=semantics,
            safe_zero=safe_zero,
            tick_deltas_stationary=ticks_stationary,
            odom_twist_stationary=odom_stationary,
            stationarity_samples=delta_samples,
            odom_samples=odom_window,
            assessment_timestamp_s=assessment_timestamp_s,
            elapsed_since_first_zero_s=(
                None if self.first_zero_timestamp_s is None else
                max(0.0, assessment_timestamp_s - self.first_zero_timestamp_s)),
            first_safe_zero_timestamp_s=self.first_safe_zero_timestamp_s,
            time_from_first_zero_to_safe_zero_s=(
                None if (self.first_zero_timestamp_s is None or
                         self.first_safe_zero_timestamp_s is None) else
                max(0.0, self.first_safe_zero_timestamp_s -
                    self.first_zero_timestamp_s)),
            max_abs_left_delta_ticks=(
                None if not delta_samples else max(
                    abs(sample.left_delta_ticks) for sample in delta_samples)),
            max_abs_right_delta_ticks=(
                None if not delta_samples else max(
                    abs(sample.right_delta_ticks) for sample in delta_samples)),
            max_abs_odom_linear_x_m_s=(
                None if not odom_window else max(
                    abs(sample.linear_x_m_s) for sample in odom_window)),
            max_abs_odom_angular_z_rad_s=(
                None if not odom_window else max(
                    abs(sample.angular_z_rad_s) for sample in odom_window)),
            first_encoder_stationary_timestamp_s=(
                self.first_encoder_stationary_timestamp_s),
            first_odom_stationary_timestamp_s=(
                self.first_odom_stationary_timestamp_s),
            last_encoder_motion_timestamp_s=(
                self.last_encoder_motion_timestamp_s),
            last_nonzero_odom_twist_timestamp_s=(
                self.last_nonzero_odom_twist_timestamp_s),
            safe_command_fresh=safe_command_fresh,
            safe_command_age_s=safe_command_age_s,
            encoder_window_duration_s=window_duration_s,
            encoder_window_start_timestamp_s=window_start_timestamp_s,
            encoder_window_end_timestamp_s=window_end_timestamp_s,
            left_net_ticks=left_net_ticks,
            right_net_ticks=right_net_ticks,
            left_absolute_ticks=left_absolute_ticks,
            right_absolute_ticks=right_absolute_ticks,
            left_net_displacement_mm=(
                None if left_net_ticks is None or
                tick_mm is None else left_net_ticks * tick_mm),
            right_net_displacement_mm=(
                None if right_net_ticks is None or
                tick_mm is None else right_net_ticks * tick_mm),
            left_absolute_displacement_mm=(
                None if left_absolute_ticks is None or
                tick_mm is None else left_absolute_ticks * tick_mm),
            right_absolute_displacement_mm=(
                None if right_absolute_ticks is None or
                tick_mm is None else right_absolute_ticks * tick_mm),
            maximum_individual_delta_ticks=maximum_individual_delta_ticks,
            left_directional_accumulation_ticks=left_directional_ticks,
            right_directional_accumulation_ticks=right_directional_ticks,
            encoder_activity_class=encoder_activity_class,
            encoder_window_acceptance_reason=(
                "accepted bounded encoder chatter"
                if not encoder_window_failures else
                "; ".join(encoder_window_failures)),
            encoder_data_fresh=encoder_data_fresh,
            encoder_data_valid=encoder_data_valid)
        with self._samples_lock:
            self.stationarity_assessments.append(assessment)
        if assessment.stationary:
            self.stationary_confirmation_timestamp_s = assessment_timestamp_s
        return assessment

    def record_emergency_stop(self, record: Dict[str, object]) -> None:
        self.emergency_stop_records.append(record)

    def stationarity_required(self) -> bool:
        return self.motion_armed or self.nonzero_command_published

    def cleanup_context_valid(self) -> bool:
        """Return whether ROS publication is still legal."""
        context = getattr(self, "context", None)
        return context is None or bool(context.ok)

    def cleanup_publisher_valid(self) -> bool:
        """Return whether the test publisher still has a usable ROS context."""
        if not self.cleanup_context_valid():
            return False
        publisher = getattr(self, "command_publisher", None)
        return (
            publisher is not None and
            (not hasattr(publisher, "handle") or publisher.handle is not None))

    def confirmed_safe_state(self) -> Dict[str, object]:
        """Return only safety state already confirmed by received samples."""
        with self._samples_lock:
            safe_zero = (
                self.last_safe is not None and
                twist_is_zero(self.last_safe, self.args.zero_tolerance))
            stationary = (
                self.stationarity_assessments[-1].stationary
                if self.stationarity_assessments else None)
        return {
            "safe_zero": safe_zero,
            "stationary": stationary,
            "stationarity_required": self.stationarity_required(),
        }

    def check_runtime_guards(self, spec: TrialSpec) -> None:
        self.verify_graph_unchanged()
        now = self._now_seconds()
        if self.last_wheel_time is None or now - self.last_wheel_time > self.args.stale_timeout_s:
            raise ValidationError("stale encoder during trial")
        if (self.last_primary_imu_time is None or
                now - self.last_primary_imu_time > self.args.stale_timeout_s):
            raise ValidationError("stale primary IMU during trial")
        if self.last_diagnostic_level > self.args.max_diagnostic_level:
            raise ValidationError("diagnostics failure during trial")
        if self.last_safe is None:
            raise ValidationError(
                "missing /cmd_vel/safe during trial: " +
                self.command_mismatch_details(spec))
        safe_age = (None if self.last_safe_time is None else
                    max(0.0, now - self.last_safe_time))
        if safe_age is None or safe_age > self.args.stale_timeout_s:
            raise ValidationError(
                "stale /cmd_vel/safe during trial: " +
                self.command_mismatch_details(spec))
        if (self.last_test_command_timestamp_s is not None and
                (self.last_safe_time is None or
                 self.last_safe_time < self.last_test_command_timestamp_s)):
            raise ValidationError(
                "pre-publication /cmd_vel/safe sample during trial: " +
                self.command_mismatch_details(spec))
        if (self.last_test_command_safe_index is not None and
                len(self.safe_commands) <= self.last_test_command_safe_index):
            raise ValidationError(
                "missing post-publication /cmd_vel/safe sample during trial: " +
                self.command_mismatch_details(spec))
        linear_error = abs(self.last_safe.linear.x - spec.linear_x)
        angular_error = abs(self.last_safe.angular.z - spec.angular_z)
        if (
                linear_error > self.args.command_tolerance or
                angular_error > self.args.command_tolerance):
            raise ValidationError(self.command_mismatch_details(spec))

    def command_mismatch_details(self, spec: TrialSpec) -> str:
        """Describe a safe-command mismatch without weakening the guard."""
        now = self._now_seconds()
        observed_linear = None if self.last_safe is None else self.last_safe.linear.x
        observed_angular = None if self.last_safe is None else self.last_safe.angular.z
        sample_age = (None if self.last_safe_time is None else
                      max(0.0, now - self.last_safe_time))
        publication_elapsed = (
            None if self.last_test_command_timestamp_s is None else
            max(0.0, now - self.last_test_command_timestamp_s))
        try:
            publisher_count = self.count_publishers(CMD_VEL_SAFE_TOPIC)
        except Exception:
            publisher_count = None
        return (
            "command mismatch on /cmd_vel/safe: "
            f"expected linear={spec.linear_x:.6f}, angular={spec.angular_z:.6f}; "
            f"observed linear={observed_linear}, angular={observed_angular}; "
            f"sample_age_s={sample_age}; publisher_count={publisher_count}; "
            f"phase={self.sample_phase}; elapsed_since_publication_s={publication_elapsed}")

    def wait_for_expected_safe_command(
            self, spec: TrialSpec,
            absolute_deadline_s: Optional[float] = None) -> None:
        """Allow bounded ROS propagation before applying the mismatch guard."""
        deadline = time.monotonic() + max(0.05, self.args.stale_timeout_s)
        if absolute_deadline_s is not None:
            if not math.isfinite(absolute_deadline_s):
                raise ValidationError(
                    "safe-command forwarding deadline is nonfinite")
            deadline = min(deadline, absolute_deadline_s)
        while True:
            if self.last_safe is not None:
                linear_error = abs(self.last_safe.linear.x - spec.linear_x)
                angular_error = abs(self.last_safe.angular.z - spec.angular_z)
                sample_is_new = (
                    self.last_test_command_timestamp_s is None or
                    (self.last_safe_time is not None and
                     self.last_safe_time >= self.last_test_command_timestamp_s))
                if self.last_test_command_safe_index is not None:
                    sample_is_new = (
                        len(self.safe_commands) >
                        self.last_test_command_safe_index)
                sample_is_fresh = (
                    self.last_safe_time is not None and
                    self._now_seconds() - self.last_safe_time <=
                    self.args.stale_timeout_s)
                if (sample_is_new and sample_is_fresh and
                        linear_error <= self.args.command_tolerance and
                        angular_error <= self.args.command_tolerance):
                    return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise ValidationError(self.command_mismatch_details(spec))
            self.spin_for(min(0.01, remaining))

    def run_command_phase(
            self,
            spec: TrialSpec,
            observation_hook: Optional[Callable[[], None]] = None) -> None:
        self.motion_armed = True
        self.sample_phase = "during_motion"
        period_s = 1.0 / self.args.publish_rate_hz
        deadline = time.monotonic() + spec.duration_s
        first_publication = True
        try:
            while time.monotonic() < deadline:
                iteration_start = time.monotonic()
                self.spin_for(min(0.01, period_s * 0.5))
                if first_publication:
                    self.verify_ready_after_operator_input()
                    self.command_start_timestamp_s = self._now_seconds()
                else:
                    self.check_runtime_guards(spec)
                if observation_hook is not None:
                    observation_hook()
                self.publish_twist(spec.linear_x, spec.angular_z)
                if (not getattr(self.args, "dry_run", False) and
                        hasattr(self, "wait_for_expected_safe_command")):
                    self.wait_for_expected_safe_command(spec)
                first_publication = False
                next_publication = iteration_start + period_s
                sleep_s = min(
                    max(0.0, next_publication - time.monotonic()),
                    max(0.0, deadline - time.monotonic()))
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
        finally:
            if getattr(self, "command_start_timestamp_s", None) is not None:
                self.command_end_timestamp_s = self._now_seconds()

    def samples(self) -> TrialSamples:
        """Capture one immutable callback-buffer snapshot at one lock boundary."""
        with self._samples_lock:
            return TrialSamples(
                wheel_ticks=tuple(self.wheel_ticks),
                imu=tuple(self.imu),
                odom=tuple(self.odom),
                diagnostics=tuple(self.diagnostics),
                ignored_diagnostics=tuple(self.ignored_diagnostics),
                commands=tuple(sorted(
                    (*self.test_commands, *self.safe_commands),
                    key=lambda sample: sample.timestamp_s)),
                stationarity=tuple(self.stationarity_assessments))

    def failure_context(self) -> Dict[str, object]:
        latest = (
            self.stationarity_assessments[-1]
            if self.stationarity_assessments else None)
        return {
            "zero_publish_count": self.zero_publish_count,
            "motion_armed": self.motion_armed,
            "nonzero_command_published": self.nonzero_command_published,
            "stationarity_required": self.stationarity_required(),
            "first_zero_timestamp_s": self.first_zero_timestamp_s,
            "first_safe_zero_timestamp_s": self.first_safe_zero_timestamp_s,
            "safe_zero_latency_s": (
                None if (self.first_zero_timestamp_s is None or
                         self.first_safe_zero_timestamp_s is None) else
                max(0.0, self.first_safe_zero_timestamp_s -
                    self.first_zero_timestamp_s)),
            "first_encoder_stationary_timestamp_s": (
                self.first_encoder_stationary_timestamp_s),
            "first_odom_stationary_timestamp_s": (
                self.first_odom_stationary_timestamp_s),
            "last_encoder_motion_timestamp_s": (
                self.last_encoder_motion_timestamp_s),
            "last_nonzero_odom_twist_timestamp_s": (
                self.last_nonzero_odom_twist_timestamp_s),
            "command_start_timestamp_s": self.command_start_timestamp_s,
            "command_end_timestamp_s": self.command_end_timestamp_s,
            "last_test_command_timestamp_s": self.last_test_command_timestamp_s,
            "last_test_command_safe_index": self.last_test_command_safe_index,
            "last_safe_command": (
                None if self.last_safe is None else {
                    "linear_x_m_s": self.last_safe.linear.x,
                    "angular_z_rad_s": self.last_safe.angular.z,
                    "received_timestamp_s": self.last_safe_time,
                    "sample_age_s": (
                        None if self.last_safe_time is None else
                        max(0.0, self._now_seconds() - self.last_safe_time)),
                    "phase": self.sample_phase,
                }),
            "stationary_confirmation_timestamp_s": (
                self.stationary_confirmation_timestamp_s),
            "imu_validation_source_topic": PRIMARY_IMU_TOPIC,
            "imu_motion_boundary_tolerance_s": (
                getattr(self.args, "imu_motion_boundary_tolerance_s", 0.1)),
            "imu_validation_source_sample_count": sum(
                sample.source_topic == PRIMARY_IMU_TOPIC for sample in self.imu),
            "emergency_stop_records": list(self.emergency_stop_records),
            "stationarity_thresholds": {
                "wheel_tick_semantics": self.args.wheel_tick_semantics,
                "required_delta_samples": self.args.stationary_samples,
                "tick_delta_tolerance": self.args.stationary_tick_tolerance,
                "encoder_window_duration_s": ENCODER_STATIONARITY_WINDOW_S,
                "encoder_chatter_max_net_ticks": ENCODER_CHATTER_MAX_NET_TICKS,
                "encoder_chatter_max_absolute_ticks": (
                    ENCODER_CHATTER_MAX_ABSOLUTE_TICKS),
                "encoder_chatter_max_sample_delta_ticks": (
                    ENCODER_CHATTER_MAX_SAMPLE_DELTA_TICKS),
                "encoder_chatter_max_directional_ticks": (
                    ENCODER_CHATTER_MAX_DIRECTIONAL_TICKS),
                "encoder_tick_displacement_mm": (
                    2.0 * math.pi * self.args.wheel_radius_m * 1000.0 /
                    self.args.encoder_ticks_per_revolution),
                "linear_velocity_tolerance_m_s": (
                    self.args.stationary_linear_velocity_tolerance),
                "angular_velocity_tolerance_rad_s": (
                    self.args.stationary_angular_velocity_tolerance),
                "safe_command_zero_tolerance": self.args.zero_tolerance,
                "controlled_stop_timeout_s": self.args.post_stop_settle_s,
                "emergency_cleanup_timeout_s": (
                    self.args.zero_publish_timeout_s),
                "zero_publish_rate_hz": self.args.zero_publish_rate_hz,
            },
            "final_stationarity_reason": (
                latest.reason if latest is not None else
                "stationarity was not assessed" if self.stationarity_required()
                else "motion was not armed or published"),
            "final_stationarity_assessment": (
                None if latest is None else asdict(latest)),
        }


def parse_csv_floats(values: Optional[Sequence[str]], defaults: Sequence[float]):
    if not values:
        return tuple(defaults)
    parsed = []
    for value in values:
        parsed.extend(float(part) for part in value.split(",") if part)
    return tuple(parsed)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="ROS 2 odometry validation orchestrator.")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--rotation", action="store_true")
    mode.add_argument("--translation", action="store_true")
    mode.add_argument("--all", action="store_true")
    parser.add_argument(
        "--velocity", action="append",
        help="Velocity list, comma-separated or repeated.")
    parser.add_argument(
        "--duration", action="append",
        help="Duration list, comma-separated or repeated.")
    parser.add_argument("--cw", action="store_true")
    parser.add_argument("--ccw", action="store_true")
    parser.add_argument("--forward", action="store_true")
    parser.add_argument("--backward", action="store_true")
    parser.add_argument(
        "--dry-run", action="store_true", default=True,
        help="Keep motion disabled. This is the default.")
    parser.add_argument(
        "--execute-motion", action="store_false", dest="dry_run",
        help="Explicitly enable publication of nonzero /cmd_vel/test commands.")
    parser.add_argument("--evidence-root", default="validation_evidence")
    parser.add_argument(
        "--resume-campaign",
        help="Resume an existing fixed campaign evidence/session directory.")
    parser.add_argument(
        "--legacy-interactive-menu", action="store_true",
        help="Use the old user-selected interactive trial menu instead of fixed campaign mode.")
    parser.add_argument(
        "--compare-session",
        help="Optional baseline/calibration session directory for validation comparison.")
    parser.add_argument("--wheel-radius-m", type=float, required=True)
    parser.add_argument("--track-width-m", type=float, required=True)
    parser.add_argument(
        "--encoder-ticks-per-revolution", type=int, required=True)
    parser.add_argument("--publish-rate-hz", type=float, default=20.0)
    parser.add_argument("--zero-publish-timeout-s", type=float, default=1.0)
    parser.add_argument("--zero-publish-rate-hz", type=float, default=20.0)
    parser.add_argument("--stale-timeout-s", type=float, default=0.5)
    parser.add_argument(
        "--imu-motion-boundary-tolerance-s", type=float, default=0.1,
        help=("maximum permitted gap from each commanded-motion boundary to "
              "a primary /imu/data sample"))
    parser.add_argument("--zero-tolerance", type=float, default=1e-4)
    parser.add_argument("--command-tolerance", type=float, default=0.05)
    parser.add_argument("--stationary-samples", type=int, default=5)
    parser.add_argument(
        "--wheel-tick-semantics",
        choices=("delta", "cumulative"),
        default="delta",
        help="/wheel_ticks contract; production driver publishes per-sample deltas.")
    parser.add_argument(
        "--stationary-tick-tolerance", type=int, default=0,
        help=("legacy exact-delta compatibility field; must remain 0 because "
              "stationarity uses the evidence-derived rolling chatter window"))
    parser.add_argument(
        "--stationary-linear-velocity-tolerance",
        type=float,
        default=0.01)
    parser.add_argument(
        "--stationary-angular-velocity-tolerance",
        type=float,
        default=0.02)
    parser.add_argument(
        "--post-stop-settle-s",
        type=float,
        default=3.0,
        help=(
            "Bounded controlled-stop timeout while zero commands continue; "
            "success returns as soon as a stationary window is confirmed."))
    parser.add_argument("--between-trial-stop-s", type=float, default=1.0)
    parser.add_argument("--preflight-spin-s", type=float, default=1.0)
    parser.add_argument("--qos-depth", type=int, default=100)
    parser.add_argument("--imu-bias-rad-s", type=float, default=0.0)
    parser.add_argument(
        "--max-angular-velocity-rad-s",
        type=float,
        default=max(DEFAULT_ROTATION_VELOCITIES_RAD_S),
        help=(
            "Maximum confirmation-gated interactive rotation velocity; "
            "defaults to the highest approved rotation-test value."))
    parser.add_argument(
        "--max-rotation-duration-s",
        type=float,
        default=max(DEFAULT_ROTATION_DURATIONS_S),
        help=(
            "Maximum confirmation-gated interactive rotation duration; "
            "defaults to the highest approved rotation-test duration."))
    parser.add_argument(
        "--min-linear-velocity-m-s", type=float, default=0.10)
    parser.add_argument(
        "--max-linear-velocity-m-s", type=float, default=1.00)
    parser.add_argument(
        "--min-translation-duration-s", type=float, default=2.0)
    parser.add_argument(
        "--max-translation-duration-s", type=float, default=10.0)
    parser.add_argument(
        "--max-diagnostic-level", type=int, choices=(0, 1), default=1,
        help="Maximum accepted non-ignored level: 0 blocks warnings, 1 allows them.")
    parser.add_argument(
        "--ignore-diagnostic",
        action="append",
        default=[],
        help="Exact diagnostic status name to ignore; repeat as needed.")
    parser.add_argument(
        "--rotation-physical-mode",
        choices=("compass", "angle"),
        default="compass")
    parser.add_argument(
        "--rotation-angle-unit", choices=("deg", "rad"), default="deg")
    parser.add_argument(
        "--reference-mode", choices=("teledex", "laser", "manual"),
        default="teledex",
        help=(
            "TeleDex default; laser uses operator wall measurements; manual is "
            "compatibility-only."))
    parser.add_argument(
        "--laser-side-wall", choices=("left", "right"), default="left")
    parser.add_argument("--laser-origin-base-m", type=float, nargs=3,
                        default=(0.41, 0.0, 0.29), metavar=("X", "Y", "Z"),
                        help=(
                            "selected laser optical origin in base_link; active "
                            "front laser default"))
    parser.add_argument("--laser-beam-yaw-deg", type=float, default=180.0,
                        help=(
                            "selected laser optical-axis yaw in base_link; active "
                            "front laser default"))
    parser.add_argument("--laser-max-lateral-drift-m", type=float, default=0.05)
    parser.add_argument("--laser-max-yaw-drift-deg", type=float, default=1.0)
    parser.add_argument(
        "--teledex-reference-root",
        default=os.environ.get(
            "TELEDEX_REFERENCE_ROOT", "/opt/teledex_reference"),
        help="Directory containing the standalone teledex_logger.py.")
    parser.add_argument(
        "--teledex-phone-to-base-rpy-deg", type=float, nargs=3,
        default=(-90.0, 0.0, 0.0), metavar=("ROLL", "PITCH", "YAW"),
        help=(
            "fixed T_phone_base rotation as ZYX roll/pitch/yaw degrees; the "
            "default maps phone +X to robot +X, phone -Z to robot +Y, and "
            "phone +Y to robot +Z"))
    parser.add_argument(
        "--teledex-phone-to-base-translation-m", type=float, nargs=3,
        default=(0.1425, -0.1000, -0.0075), metavar=("X", "Y", "Z"),
        help="base_link origin expressed in phone coordinates")
    parser.add_argument("--teledex-stale-timeout-s", type=float, default=0.5)
    parser.add_argument("--teledex-readiness-timeout-s", type=float, default=5.0)
    parser.add_argument(
        "--teledex-axis-validation", action="store_true",
        help="Guide forward and CCW observations to identify the discrete phone mount.")
    parser.add_argument(
        "--teledex-axis-validation-min-displacement-m", type=float, default=0.05)
    parser.add_argument(
        "--teledex-axis-validation-min-yaw-deg", type=float, default=10.0)
    parser.add_argument(
        "--cli-trial-mode", action="store_true",
        help="Use the existing one-trial CLI selection instead of startup selection.")
    parser.add_argument(
        "--required-node", action="append",
        default=list(DEFAULT_REQUIRED_NODES))
    parser.add_argument(
        "--required-topic", action="append",
        default=list(DEFAULT_REQUIRED_TOPICS))
    return parser


def build_trials(args) -> List[TrialSpec]:
    run_rotation = args.rotation or args.all or not args.translation
    run_translation = args.translation or args.all
    trials: List[TrialSpec] = []
    directions_rotation_selected = args.cw or args.ccw
    directions_translation_selected = args.forward or args.backward
    if run_rotation:
        velocities = parse_csv_floats(
            args.velocity, DEFAULT_ROTATION_VELOCITIES_RAD_S)
        durations = parse_csv_floats(
            args.duration, DEFAULT_ROTATION_DURATIONS_S)
        trials.extend(generate_rotation_trials(
            velocities=velocities,
            durations=durations,
            include_cw=args.cw or not directions_rotation_selected,
            include_ccw=args.ccw or not directions_rotation_selected))
    if run_translation:
        velocities = parse_csv_floats(
            args.velocity, DEFAULT_TRANSLATION_VELOCITIES_M_S)
        durations = parse_csv_floats(
            args.duration, DEFAULT_TRANSLATION_DURATIONS_S)
        trials.extend(generate_translation_trials(
            velocities=velocities,
            durations=durations,
            include_forward=args.forward or not directions_translation_selected,
            include_backward=args.backward or not directions_translation_selected))
    return trials


def ask_physical_measurement(
        args,
        spec: TrialSpec,
        operator_input: ResponsiveOperatorInput) -> Optional[float]:
    if args.dry_run:
        return None
    if spec.movement_type == "rotation":
        if args.rotation_physical_mode == "compass":
            initial = operator_input.read_float(
                "Enter initial compass heading: ")
            final = operator_input.read_float("Enter final compass heading: ")
            return compass_rotation_radians(initial, final)
        measured = operator_input.read_float(
            "Enter measured physical rotation angle: ")
        return measured if args.rotation_angle_unit == "rad" else math.radians(measured)
    return operator_input.read_float(
        "Enter measured physical displacement (meters): ")


def choose_reference_mode(operator_input: ResponsiveOperatorInput) -> str:
    """Ask once, before interactive trial selection, without changing CLI mode."""
    while True:
        selected = operator_input.read_text(
            "Select physical reference:\n1 - TeleDex / ARKit\n2 - Laser\n> ").strip()
        if selected == "1":
            return "teledex"
        if selected == "2":
            return "laser"
        print("Enter 1 or 2.", file=sys.stderr)


def ask_laser_initial(args, spec: TrialSpec, operator_input: ResponsiveOperatorInput):
    """Collect immutable raw laser readings before preflight/motion."""
    if spec.movement_type == "translation":
        return (
            operator_input.read_float("Initial longitudinal-wall distance [m]: "),
            operator_input.read_float("Initial side-wall distance [m]: "))
    return (operator_input.read_float("Initial distance to reference wall [m]: "),)


def ask_laser_final(spec: TrialSpec, operator_input: ResponsiveOperatorInput):
    """Collect final readings only after controlled-stop stationarity succeeds."""
    if spec.movement_type == "translation":
        return (
            operator_input.read_float("Final longitudinal-wall distance [m]: "),
            operator_input.read_float("Final side-wall distance [m]: "))
    return (operator_input.read_float("Final distance to reference wall [m]: "),)


def read_finite_float_with_raw(
        operator_input: ResponsiveOperatorInput,
        prompt: str) -> Tuple[str, float]:
    """Read a finite float while preserving the operator's raw text entry."""
    while True:
        raw = operator_input.read_text(prompt)
        if raw.strip().upper() == "ABORT":
            raise KeyboardInterrupt("operator aborted numeric input")
        try:
            value = float(raw.strip())
        except (TypeError, ValueError):
            operator_input.notify("Invalid numeric input. Please enter a number or ABORT.")
            continue
        if math.isfinite(value):
            return raw, value
        operator_input.notify("Invalid numeric input. Please enter a finite number or ABORT.")


def ask_campaign_mode(operator_input: ResponsiveOperatorInput) -> str:
    """Select the fixed campaign mode once at startup."""
    while True:
        selected = operator_input.read_text(
            "Select campaign mode:\n"
            "1 - CHARACTERIZATION / BASELINE\n"
            "2 - CALIBRATION\n"
            "3 - VALIDATION\n"
            "4 - COVARIANCE CHARACTERIZATION\n"
            "ABORT - Exit without starting\n"
            "Selection: ").strip()
        if selected.upper() == "ABORT":
            raise KeyboardInterrupt("operator aborted before campaign start")
        if selected in CAMPAIGN_MODES:
            return CAMPAIGN_MODES[selected]
        operator_input.notify("Invalid campaign mode. Enter 1, 2, 3, 4, or ABORT.")


def ask_campaign_validity(
        operator_input: ResponsiveOperatorInput,
        auto_invalid_reason: Optional[str] = None) -> Tuple[str, Optional[str], str]:
    """Campaign validity prompt: invalid attempts are preserved and retried."""
    if auto_invalid_reason:
        print(auto_invalid_reason, flush=True)
        note = operator_input.read_text(
            "Optional invalidation note, or ABORT: ").strip()
        if note.upper() == "ABORT":
            raise KeyboardInterrupt("operator aborted at invalidation note")
        return "invalid", auto_invalid_reason, note
    while True:
        answer = operator_input.read_text("Was this test valid? [Y/N/ABORT]: ").strip()
        if answer.upper() == "ABORT":
            raise KeyboardInterrupt("operator aborted at validity prompt")
        if answer.lower() in ("y", "yes"):
            notes = operator_input.read_text("Operator notes, or ABORT: ").strip()
            if notes.upper() == "ABORT":
                raise KeyboardInterrupt("operator aborted at operator notes")
            return "valid", None, notes
        if answer.lower() in ("n", "no"):
            reason = operator_input.read_text(
                "Invalidation reason, or ABORT: ").strip()
            if reason.upper() == "ABORT":
                raise KeyboardInterrupt("operator aborted at invalidation reason")
            return "invalid", reason, ""
        operator_input.notify("Enter Y, N, or ABORT.")


def confirm_campaign_repetition(
        operator_input: ResponsiveOperatorInput,
        mode: str,
        condition,
        repetition_number: int,
        geometry: GeometryConfig,
        evidence_dir: Path) -> None:
    """Print the required pre-motion block and require explicit approval."""
    spec = condition.spec
    if spec.movement_type == "rotation":
        expected = (
            f"expected_commanded_angle_rad={spec.commanded_angle_rad:.9g}, "
            f"expected_commanded_angle_deg={math.degrees(spec.commanded_angle_rad):.9g}")
        velocity = f"angular_velocity_rad_s={spec.velocity:.9g}"
    else:
        expected = f"expected_commanded_distance_m={abs(spec.commanded_distance_m):.9g}"
        velocity = f"linear_velocity_m_s={spec.velocity:.9g}"
    print(
        "\nCAMPAIGN REPETITION CONFIRMATION\n"
        f"- campaign_mode: {CAMPAIGN_MODE_LABELS[mode]}\n"
        f"- section: {condition.section}\n"
        f"- condition_number: {condition.condition_number}/48\n"
        f"- repetition: {repetition_number}/10 valid\n"
        f"- direction: {spec.direction}\n"
        f"- {velocity}\n"
        f"- duration_s: {spec.duration_s:.9g}\n"
        f"- {expected}\n"
        f"- wheel_radius_m: {geometry.wheel_radius_m:.12g}\n"
        f"- track_width_m: {geometry.track_width_m:.12g}\n"
        f"- evidence_dir: {evidence_dir}\n",
        flush=True)
    while True:
        answer = operator_input.read_text(
            "Publish this nonzero motion command? [YES/ABORT]: ").strip()
        if answer == "YES":
            return
        if answer.upper() == "ABORT":
            raise KeyboardInterrupt("operator aborted before nonzero motion")
        operator_input.notify("Type YES to execute this repetition, or ABORT.")


def load_campaign_attempts(evidence_dir: Path) -> List[Dict[str, object]]:
    path = evidence_dir / "campaign_attempts.json"
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    if not isinstance(payload, list):
        raise ValidationError("campaign_attempts.json must contain a list")
    return payload


def git_identity() -> Dict[str, object]:
    """Best-effort read-only software identity for campaign manifests."""
    def run_git(*args: str) -> Optional[str]:
        try:
            completed = subprocess.run(
                ("git",) + args,
                check=True,
                cwd=Path(__file__).resolve().parents[3],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE)
        except Exception:
            return None
        return completed.stdout.strip()

    return {
        "commit": run_git("rev-parse", "HEAD"),
        "branch": run_git("branch", "--show-current"),
        "status_short": run_git("status", "--short"),
    }


def write_campaign_artifacts(
        evidence_dir: Path,
        mode: str,
        geometry: GeometryConfig,
        attempts: Sequence[Dict[str, object]],
        compare_session: Optional[str] = None) -> None:
    """Atomically write campaign checkpoint, analysis, and CSV exports."""
    conditions = fixed_campaign_conditions(mode=mode)
    manifest = {
        "schema_version": 1,
        "campaign_session_id": evidence_dir.name,
        "calibration_iteration_id": evidence_dir.name if mode == "calibration" else None,
        "mode": mode,
        "mode_label": CAMPAIGN_MODE_LABELS[mode],
        "start_timestamp": utc_timestamp(),
        "software_git_identity": git_identity(),
        "wheel_radius_m": geometry.wheel_radius_m,
        "fixed_candidate_wheel_radius_m": (
            geometry.wheel_radius_m if mode == "calibration" else None),
        "track_width_m": geometry.track_width_m,
        "encoder_ticks_per_revolution": geometry.encoder_ticks_per_revolution,
        "matrix_identity": campaign_matrix_identity(conditions),
        "complete_test_matrix": [asdict(condition) for condition in conditions],
        "progress": campaign_progress(conditions, attempts),
        "compare_session": compare_session,
        "manual_reference_limitations": (
            "iPhone compass and wall-distance readings are practical campaign "
            "references, not laboratory-grade ground truth."),
    }
    atomic_write_json(evidence_dir / "campaign_manifest.json", manifest)
    atomic_write_json(evidence_dir / "campaign_attempts.json", list(attempts))
    write_csv(evidence_dir / "campaign_attempts.csv", list(attempts))
    summaries = [
        condition_summary(condition, attempts)
        for condition in conditions
        if condition.condition_id in {
            str(attempt.get("condition_id")) for attempt in attempts}]
    atomic_write_json(evidence_dir / "condition_summaries.json", {
        "condition_summaries": summaries,
    })
    whole_campaign = whole_campaign_analysis(attempts)
    analysis = {
        "mode": mode,
        "progress": campaign_progress(conditions, attempts),
        "whole_campaign": whole_campaign,
        "calibration_primary_accuracy": (
            whole_campaign["manual_reference_accuracy"]
            if mode == "calibration" else None),
        "radius_calibration": (
            radius_calibration_analysis(
                attempts, geometry.wheel_radius_m, geometry.track_width_m)
            if mode == "calibration" else None),
        "covariance": covariance_analysis(attempts) if mode == "covariance" else None,
        "validation_compare_session": compare_session if mode == "validation" else None,
        "validation_comparison": (
            None if mode != "validation" or compare_session is None else
            validation_comparison(
                load_campaign_attempts(Path(compare_session)), attempts)),
        "scientific_separation": {
            "command_expectation": "reported separately; not ground truth",
            "encoder_estimate": "reported separately",
            "imu_estimate": "reported separately",
            "manual_reference": "iPhone compass and wall distance",
            "calibration_recommendation": "candidate only; not applied",
            "validation_evidence": "separate frozen-parameter dataset",
            "covariance_estimation": "candidate planar residual statistics only",
        },
    }
    atomic_write_json(evidence_dir / "campaign_analysis.json", analysis)


def execute_trial(
        node: OdometryValidationNode,
        args,
        geometry: GeometryConfig,
        spec: TrialSpec,
        operator_input: ResponsiveOperatorInput,
        sample_sink: Optional[Callable[[TrialSamples], None]] = None,
        emergency_cleanup: Optional[EmergencyCleanupOnce] = None,
        teledex: Optional[TeleDexReferenceAdapter] = None,
        teledex_sink: Optional[Callable[[Sequence[object]], None]] = None,
        collect_physical_measurement: bool = True,
        emergency_stop_timeout_s: Optional[float] = None,
        pre_motion_confirmation: Optional[Callable[[], None]] = None,
        ) -> Tuple[TrialResult, TrialSamples]:
    controlled_stop = EmergencyStopController(
        publish_zero=node.publish_zero,
        verify_safe_zero=node.verify_safe_zero,
        verify_stationary=node.verify_stationary,
        sleep=getattr(node, "spin_for", time.sleep),
        record_result=getattr(node, "record_emergency_stop", None),
        stationarity_required=getattr(
            node, "stationarity_required", None),
        begin_stop=getattr(node, "begin_emergency_stop", None),
        prepare_verification=getattr(
            node, "prepare_emergency_stop_verification", None),
        verify_stop_guards=getattr(
            node, "verify_controlled_stop_guards", None),
        cleanup_context_valid=getattr(
            node, "cleanup_context_valid", None),
        cleanup_publisher_valid=getattr(
            node, "cleanup_publisher_valid", None),
        confirmed_safe_state=getattr(
            node, "confirmed_safe_state", None))
    emergency = emergency_cleanup or EmergencyCleanupOnce(
        EmergencyStopController(
            publish_zero=node.publish_zero,
            verify_safe_zero=node.verify_safe_zero,
            verify_stationary=node.verify_stationary,
            sleep=getattr(node, "spin_for", time.sleep),
            record_result=getattr(node, "record_emergency_stop", None),
            stationarity_required=getattr(
                node, "stationarity_required", None),
            begin_stop=getattr(node, "begin_emergency_stop", None),
            prepare_verification=getattr(
                node, "prepare_emergency_stop_verification", None),
            verify_stop_guards=getattr(
                node, "verify_controlled_stop_guards", None),
            cleanup_context_valid=getattr(
                node, "cleanup_context_valid", None),
            cleanup_publisher_valid=getattr(
                node, "cleanup_publisher_valid", None),
            confirmed_safe_state=getattr(
                node, "confirmed_safe_state", None)))

    def action():
        using_laser = (
            collect_physical_measurement and
            getattr(args, "reference_mode", "manual") == "laser")
        laser_initial = None
        manual_reference = None
        translation_initial_raw = None
        translation_initial_m = None
        physical_initial = None
        using_teledex = teledex is not None and not args.dry_run
        if using_laser and not args.dry_run:
            node.interrupted_operation = "laser_initial_measurement"
            laser_initial = ask_laser_initial(args, spec, operator_input)
        if (
                collect_physical_measurement and not args.dry_run and
                not using_laser and getattr(args, "reference_mode", "manual") == "manual" and
                spec.movement_type == "translation"):
            node.interrupted_operation = "manual_translation_initial_reference"
            print(
                "Robot must start perpendicular to the wall. Translation "
                "heading reference is 0 deg relative to this starting orientation.",
                flush=True)
            translation_initial_raw, translation_initial_m = read_finite_float_with_raw(
                operator_input,
                "Initial wall distance from robot/reference point [m]: ")
        if (
                spec.movement_type == "rotation" and
                collect_physical_measurement and
                args.rotation_physical_mode == "compass" and
                not using_teledex and not using_laser):
            node.interrupted_operation = "manual_rotation_initial_reference"
            physical_initial_raw = None
            if args.dry_run:
                physical_initial = None
            else:
                physical_initial_raw, physical_initial = read_finite_float_with_raw(
                    operator_input,
                    "Enter initial iPhone compass heading [deg]: ")
            manual_reference = {
                "type": "iphone_compass_rotation",
                "initial_heading_raw": physical_initial_raw,
                "initial_heading_deg": physical_initial,
                "compass_wrap_policy": (
                    "shortest signed delta; compass clockwise positive converted "
                    "to ROS yaw counter-clockwise positive"),
            }
        if pre_motion_confirmation is not None and not args.dry_run:
            node.interrupted_operation = "campaign_pre_motion_confirmation"
            pre_motion_confirmation()
        node.interrupted_operation = "preflight"
        node.reset_trial_buffers()
        node.verify_preflight()
        physical_measurement = None
        physical_final = None
        laser_reference = None
        teledex_result = None
        if using_teledex:
            node.interrupted_operation = "teledex_initial_reference"
            teledex.begin_trial(node._now_seconds())

        if args.dry_run:
            node.get_logger().warn(
                "dry-run active: nonzero /cmd_vel/test publication skipped")
        else:
            node.run_command_phase(
                spec,
                observation_hook=(
                    None if not using_teledex else
                    lambda: teledex.poll_required("trajectory during motion")))
        stop_sleep = getattr(node, "spin_for", time.sleep)
        active_controlled_stop = controlled_stop
        if using_teledex:
            def stop_sleep(duration_s):
                node.spin_for(duration_s)
                teledex.poll_required("trajectory during controlled stop")
            active_controlled_stop = EmergencyStopController(
                publish_zero=node.publish_zero,
                verify_safe_zero=node.verify_safe_zero,
                verify_stationary=node.verify_stationary,
                sleep=stop_sleep,
                record_result=getattr(node, "record_emergency_stop", None),
                stationarity_required=getattr(node, "stationarity_required", None),
                begin_stop=getattr(node, "begin_emergency_stop", None),
                prepare_verification=getattr(node, "prepare_emergency_stop_verification", None),
                verify_stop_guards=getattr(node, "verify_controlled_stop_guards", None),
                cleanup_context_valid=getattr(node, "cleanup_context_valid", None),
                cleanup_publisher_valid=getattr(node, "cleanup_publisher_valid", None),
                confirmed_safe_state=getattr(node, "confirmed_safe_state", None))
        active_controlled_stop.stop(
            args.post_stop_settle_s, args.zero_publish_rate_hz, mode="controlled")

        if not args.dry_run and collect_physical_measurement:
            if using_teledex:
                node.interrupted_operation = "teledex_final_reference"
                teledex_result = teledex.finalize_after_stationarity(
                    getattr(node, "stationary_confirmation_timestamp_s", None))
                physical_measurement = (
                    teledex_result.final_yaw_unwrapped_rad
                    if spec.movement_type == "rotation" else
                    teledex_result.net_forward_displacement_m)
            elif spec.movement_type == "rotation":
                if using_laser:
                    node.interrupted_operation = "laser_final_measurement"
                    laser_final = ask_laser_final(spec, operator_input)
                    laser_reference = laser_rotation_reference(
                        spec.direction, laser_initial[0], laser_final[0],
                        tuple(args.laser_origin_base_m),
                        math.radians(args.laser_beam_yaw_deg))
                    physical_measurement = laser_reference.angle_rad
                elif args.rotation_physical_mode == "compass":
                    node.interrupted_operation = "input"
                    physical_final_raw, physical_final = read_finite_float_with_raw(
                        operator_input, "Enter final iPhone compass heading [deg]: ")
                    physical_measurement = compass_rotation_radians(
                        physical_initial, physical_final)
                    if manual_reference is not None:
                        manual_reference.update({
                            "final_heading_raw": physical_final_raw,
                            "final_heading_deg": physical_final,
                            "signed_measured_rotation_deg": math.degrees(
                                physical_measurement),
                            "signed_measured_rotation_rad": physical_measurement,
                            "absolute_rotation_magnitude_deg": abs(math.degrees(
                                physical_measurement)),
                            "absolute_rotation_magnitude_rad": abs(physical_measurement),
                        })
                else:
                    node.interrupted_operation = "input"
                    measured = operator_input.read_float(
                        "Enter measured physical rotation angle: ")
                    physical_measurement = (
                        measured if args.rotation_angle_unit == "rad"
                        else math.radians(measured))
            else:
                if using_laser:
                    node.interrupted_operation = "laser_final_measurement"
                    laser_final = ask_laser_final(spec, operator_input)
                    laser_reference = laser_translation_reference(
                        spec.direction, laser_initial[0], laser_final[0],
                        laser_initial[1], laser_final[1], args.laser_side_wall)
                    physical_measurement = laser_reference.signed_displacement_m
                else:
                    node.interrupted_operation = "manual_translation_final_reference"
                    final_raw, final_m = read_finite_float_with_raw(
                        operator_input,
                        "Final wall distance from robot/reference point [m]: ")
                    heading_raw, heading_deg = read_finite_float_with_raw(
                        operator_input,
                        "Final compass heading deviation from initial 0 deg [deg]: ")
                    physical_measurement = translation_reference_distance_m(
                        spec.direction, translation_initial_m, final_m)
                    manual_reference = {
                        "type": "manual_wall_distance_translation",
                        "unit": "meters",
                        "initial_wall_distance_raw": translation_initial_raw,
                        "initial_wall_distance_m": translation_initial_m,
                        "final_wall_distance_raw": final_raw,
                        "final_wall_distance_m": final_m,
                        "distance_reference_m": physical_measurement,
                        "travel_magnitude_m": abs(physical_measurement),
                        "initial_heading_reference_deg": 0.0,
                        "final_heading_deviation_raw": heading_raw,
                        "final_heading_deviation_deg": heading_deg,
                        "heading_limit_deg": TRANSLATION_HEADING_LIMIT_DEG,
                    }
        node.interrupted_operation = "snapshot/report"
        measurements, samples = build_measurements(
            spec,
            node.samples(),
            geometry,
            physical_measurement,
            imu_bias_rad_s=args.imu_bias_rad_s,
            wheel_tick_semantics=args.wheel_tick_semantics,
            command_start_timestamp_s=getattr(
                node, "command_start_timestamp_s", None),
            command_end_timestamp_s=getattr(
                node, "command_end_timestamp_s", None),
            stationary_confirmation_timestamp_s=getattr(
                node, "stationary_confirmation_timestamp_s", None),
            imu_source_topic=PRIMARY_IMU_TOPIC,
            imu_boundary_tolerance_s=getattr(
                args, "imu_motion_boundary_tolerance_s", 0.1),
            teledex_result=teledex_result)
        if laser_reference is not None and spec.movement_type == "translation":
            yaw_aid = (measurements.imu_angle_rad if measurements.imu_angle_rad is not None
                       else measurements.odometry_yaw_drift_rad)
            aid_name = ("IMU yaw (quality/rejection aid; not laser ground truth)"
                        if measurements.imu_angle_rad is not None else
                        "odometry yaw (quality/rejection aid; not laser ground truth)")
            laser_reference = apply_laser_translation_quality(
                laser_reference, args.laser_max_lateral_drift_m,
                math.radians(args.laser_max_yaw_drift_deg), yaw_aid, aid_name)
        result = make_trial_result(
            spec,
            measurements,
            initial_compass_heading_deg=physical_initial,
            final_compass_heading_deg=physical_final,
            manual_reference=manual_reference,
            teledex=(
                None if teledex_result is None else teledex_result.as_dict()),
            laser=(None if laser_reference is None else asdict(laser_reference)))
        if (
                spec.movement_type == "translation" and
                manual_reference is not None and
                heading_deviation_auto_invalid(
                    manual_reference["final_heading_deviation_deg"])):
            result = replace(
                result,
                valid=False,
                rejection_reason=(
                    "Heading deviation exceeded 5°. This repetition must be repeated."))
        if laser_reference is not None and not laser_reference.quality_passed:
            result = replace(
                result, valid=False,
                rejection_reason=laser_reference.quality_rejection_reason)
        if teledex_sink is not None and teledex is not None:
            teledex_sink(teledex.trajectory)
        return result, samples

    result = run_with_emergency_stop(
        action,
        emergency,
        (args.zero_publish_timeout_s if emergency_stop_timeout_s is None else
         emergency_stop_timeout_s),
        args.zero_publish_rate_hz)
    if sample_sink is not None:
        node.interrupted_operation = "merge"
        sample_sink(result[1])
    return result


def apply_operator_verdict(
        result: TrialResult,
        verdict: str,
        reason: Optional[str],
        notes: str) -> TrialResult:
    if (
            result.valid is False and result.rejection_reason and
            "Heading deviation exceeded 5" in result.rejection_reason):
        return replace(result, skipped=False, operator_notes=notes)
    if result.laser is not None and not result.laser.get("quality_passed", False):
        return replace(
            result, valid=False, skipped=False,
            rejection_reason=result.laser.get("quality_rejection_reason"),
            operator_notes=notes)
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
            final_compass_heading_deg=result.final_compass_heading_deg,
            manual_reference=result.manual_reference,
            teledex=result.teledex, laser=result.laser)
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
            final_compass_heading_deg=result.final_compass_heading_deg,
            manual_reference=result.manual_reference,
            teledex=result.teledex, laser=result.laser)
    return TrialResult(
        spec=result.spec,
        timestamp=result.timestamp,
        measurements=result.measurements,
        errors=result.errors,
        valid=False,
        skipped=False,
        rejection_reason=reason,
        operator_notes=notes,
        evidence_dir=result.evidence_dir,
        initial_compass_heading_deg=result.initial_compass_heading_deg,
        final_compass_heading_deg=result.final_compass_heading_deg,
        manual_reference=result.manual_reference,
        teledex=result.teledex, laser=result.laser)


def run(args) -> int:
    if (not args.dry_run and getattr(args, "reference_mode", "manual") == "teledex" and
            args.teledex_phone_to_base_translation_m is None):
        raise ValidationError(
            "TeleDex phone-to-base translation is not configured; measure the "
            "phone height relative to base_link and pass all three components")
    geometry = GeometryConfig(
        wheel_radius_m=args.wheel_radius_m,
        track_width_m=args.track_width_m,
        encoder_ticks_per_revolution=args.encoder_ticks_per_revolution)
    if getattr(args, "cli_trial_mode", False):
        trials = build_trials(args)
        if len(trials) != 1:
            raise ValidationError(
                "CLI trial mode requires exactly one selected direction, velocity, and duration")
    else:
        trials = []
    fixed_campaign_enabled = (
        not getattr(args, "cli_trial_mode", False) and
        not getattr(args, "legacy_interactive_menu", False))
    evidence = EvidenceWriter(Path(args.evidence_root))
    metadata = {
        "dry_run": args.dry_run,
        "initial_trial_count": len(trials),
        "interactive_menu": (
            not fixed_campaign_enabled and
            not getattr(args, "cli_trial_mode", False)),
        "fixed_campaign_enabled": fixed_campaign_enabled,
        "geometry": {
            "wheel_radius_m": args.wheel_radius_m,
            "track_width_m": args.track_width_m,
            "encoder_ticks_per_revolution": (
                args.encoder_ticks_per_revolution),
        },
        "command_topic": CMD_VEL_TEST_TOPIC,
        "safe_command_topic": CMD_VEL_SAFE_TOPIC,
        "ignored_diagnostic_names": sorted(set(args.ignore_diagnostic)),
        "stationarity_thresholds": {
            "wheel_tick_semantics": args.wheel_tick_semantics,
            "required_delta_samples": args.stationary_samples,
            "tick_delta_tolerance": args.stationary_tick_tolerance,
            "encoder_window_duration_s": ENCODER_STATIONARITY_WINDOW_S,
            "encoder_chatter_max_net_ticks": ENCODER_CHATTER_MAX_NET_TICKS,
            "encoder_chatter_max_absolute_ticks": (
                ENCODER_CHATTER_MAX_ABSOLUTE_TICKS),
            "encoder_chatter_max_sample_delta_ticks": (
                ENCODER_CHATTER_MAX_SAMPLE_DELTA_TICKS),
            "encoder_chatter_max_directional_ticks": (
                ENCODER_CHATTER_MAX_DIRECTIONAL_TICKS),
            "encoder_tick_displacement_mm": (
                2.0 * math.pi * args.wheel_radius_m * 1000.0 /
                args.encoder_ticks_per_revolution),
            "linear_velocity_tolerance_m_s": (
                args.stationary_linear_velocity_tolerance),
            "angular_velocity_tolerance_rad_s": (
                args.stationary_angular_velocity_tolerance),
            "safe_command_zero_tolerance": args.zero_tolerance,
            "controlled_stop_timeout_s": args.post_stop_settle_s,
            "imu_motion_boundary_tolerance_s": (
                args.imu_motion_boundary_tolerance_s),
            "emergency_cleanup_timeout_s": args.zero_publish_timeout_s,
            "zero_publish_rate_hz": args.zero_publish_rate_hz,
        },
        "imu_bias_rad_s": args.imu_bias_rad_s,
        "reference_mode": getattr(args, "reference_mode", "manual"),
        "teledex": {
            "reference_root": getattr(args, "teledex_reference_root", None),
            "frame_chain": (
                "T_world_base=T_world_phone*T_phone_base; "
                "T_base_rel=inverse(T_world_base_start)*T_world_base_current"),
            "phone_to_base_rpy_deg": list(getattr(
                args, "teledex_phone_to_base_rpy_deg", ())),
            "selected_mapping_source": "explicit CLI/configuration",
            "axis_validation_policy": (
                "read-only consistency check; ambiguous or mismatched "
                "observations fail closed and never replace the selected mount"),
            "phone_to_base_translation_m": list(getattr(
                args, "teledex_phone_to_base_translation_m", None) or []),
            "base_axis_convention": (
                "+X forward, +Y left, +Z up, positive yaw ROS counter-clockwise"),
            "stale_timeout_s": getattr(args, "teledex_stale_timeout_s", None),
            "readiness_timeout_s": getattr(
                args, "teledex_readiness_timeout_s", None),
            "synchronization": (
                "TeleDex installed API has no ARKit source timestamp; local monotonic "
                "receive timestamps and ROS callback timestamps are preserved separately."),
        },
        "laser": {
            "active_front_laser_origin_base_m": [0.41, 0.0, 0.29],
            "selected_laser_origin_base_m": list(args.laser_origin_base_m),
            "selected_laser_beam_yaw_deg": args.laser_beam_yaw_deg,
            "side_wall": args.laser_side_wall,
            "max_lateral_drift_m": args.laser_max_lateral_drift_m,
            "max_yaw_drift_deg": args.laser_max_yaw_drift_deg,
            "rotation_model": (
                "(d_final+x_l)*cos(theta)-y_l*sin(theta)=d_initial+x_l; "
                "commanded direction selects the near-zero signed root"),
        },
        "role": "test orchestrator and data collection framework",
    }
    if args.resume_campaign:
        evidence_dir = evidence.open_existing(
            Path(args.resume_campaign), metadata_override=metadata)
    else:
        evidence_dir = evidence.create(metadata)
    print(f"evidence_dir={evidence_dir}")
    if args.dry_run:
        print("dry_run=true; no nonzero /cmd_vel/test commands will be published")

    try:
        rclpy.init(args=None)
        node = OdometryValidationNode(args)
    except BaseException:
        rclpy.try_shutdown()
        raise
    shutdown_done = False

    def shutdown_once() -> None:
        nonlocal shutdown_done
        if shutdown_done:
            return
        shutdown_done = True
        rclpy.try_shutdown()
    emergency_cleanup = EmergencyCleanupOnce(EmergencyStopController(
        publish_zero=node.publish_zero,
        verify_safe_zero=node.verify_safe_zero,
        verify_stationary=node.verify_stationary,
        sleep=node.spin_for,
        record_result=node.record_emergency_stop,
        stationarity_required=node.stationarity_required,
        begin_stop=node.begin_emergency_stop,
        prepare_verification=node.prepare_emergency_stop_verification,
        verify_stop_guards=node.verify_controlled_stop_guards,
        cleanup_context_valid=node.cleanup_context_valid,
        cleanup_publisher_valid=node.cleanup_publisher_valid,
        confirmed_safe_state=node.confirmed_safe_state))
    try:
        terminal_reader = TerminalLineReader(
            stream=sys.stdin.buffer,
            output=sys.stdout,
            encoding=sys.stdin.encoding or "utf-8")
        operator_input = ResponsiveOperatorInput(
            prompt=terminal_reader,
            poll=lambda: node.spin_for(OPERATOR_CALLBACK_SERVICE_S),
            notify=lambda message: print(message, file=sys.stderr),
            poll_interval_s=OPERATOR_INPUT_POLL_INTERVAL_S)
        operator = OperatorInterface(operator_input.read_text)
        if (
                not getattr(args, "cli_trial_mode", False) and
                not fixed_campaign_enabled):
            args.reference_mode = choose_reference_mode(operator_input)
            evidence.write_reference_selection(args.reference_mode)
        menu = InteractiveTrialMenu(
            operator_input,
            InteractiveLimits(
                max_angular_velocity_rad_s=args.max_angular_velocity_rad_s,
                max_rotation_duration_s=args.max_rotation_duration_s,
                min_linear_velocity_m_s=args.min_linear_velocity_m_s,
                max_linear_velocity_m_s=args.max_linear_velocity_m_s,
                min_translation_duration_s=args.min_translation_duration_s,
                max_translation_duration_s=args.max_translation_duration_s),
            display=print)
        campaign_menu = InteractiveCampaignMenu(
            operator_input,
            InteractiveLimits(
                max_angular_velocity_rad_s=args.max_angular_velocity_rad_s,
                max_rotation_duration_s=args.max_rotation_duration_s,
                min_linear_velocity_m_s=args.min_linear_velocity_m_s,
                max_linear_velocity_m_s=args.max_linear_velocity_m_s,
                min_translation_duration_s=args.min_translation_duration_s,
                max_translation_duration_s=args.max_translation_duration_s),
            display=print)
        if fixed_campaign_enabled:
            args.reference_mode = "manual"
    except BaseException as setup_error:
        failure: BaseException = setup_error
        try:
            emergency_cleanup.stop(
                args.zero_publish_timeout_s, args.zero_publish_rate_hz)
        except BaseException as cleanup_error:
            failure = EmergencyStopCleanupError(setup_error, cleanup_error)
        finally:
            try:
                node.destroy_node()
            finally:
                shutdown_once()
        try:
            evidence.write_failure(
                [], failure, TrialSamples(), failure_context={
                    "setup_interrupted": True,
                    "cleanup_attempted": emergency_cleanup.attempted,
                })
        except BaseException as evidence_error:
            print(
                f"failed to write validation failure evidence: {evidence_error}",
                file=sys.stderr)
        if failure is not setup_error:
            raise failure from setup_error
        raise
    results: List[TrialResult] = []
    current_trial_samples = TrialSamples()
    latest_complete_snapshot = TrialSamples()
    interrupted_operation = "menu"
    teledex = None

    def retain_trial_samples(samples: TrialSamples) -> None:
        nonlocal current_trial_samples, latest_complete_snapshot
        current_trial_samples = samples
        latest_complete_snapshot = samples

    try:
        if (not args.dry_run and
                getattr(args, "reference_mode", "manual") == "teledex"):
            interrupted_operation = "teledex_connect"
            teledex = TeleDexReferenceAdapter(
                Path(args.teledex_reference_root),
                tuple(args.teledex_phone_to_base_rpy_deg),
                tuple(args.teledex_phone_to_base_translation_m),
                args.teledex_stale_timeout_s,
                readiness_timeout_s=args.teledex_readiness_timeout_s)
            teledex.start()
            if args.teledex_axis_validation:
                interrupted_operation = "teledex_axis_validation"
                try:
                    axis_observation = teledex.validate_mount_mapping(
                        operator_input.read_text,
                        args.teledex_axis_validation_min_displacement_m,
                        math.radians(args.teledex_axis_validation_min_yaw_deg))
                except TeleDexMountMappingMismatch as mismatch:
                    evidence.write_axis_validation(mismatch.observation)
                    raise
                evidence.write_axis_validation(axis_observation)
        if fixed_campaign_enabled:
            args.reference_mode = "manual"
            attempts = load_campaign_attempts(evidence_dir)
            manifest_path = evidence_dir / "campaign_manifest.json"
            if args.resume_campaign and manifest_path.exists():
                with manifest_path.open("r", encoding="utf-8") as stream:
                    existing_manifest = json.load(stream)
                mode = existing_manifest["mode"]
                conditions = fixed_campaign_conditions(mode=mode)
                if abs(float(existing_manifest["wheel_radius_m"]) -
                       geometry.wheel_radius_m) > 1e-12:
                    raise ValidationError(
                        "resume wheel radius does not match campaign manifest")
                if abs(float(existing_manifest["track_width_m"]) -
                       geometry.track_width_m) > 1e-12:
                    raise ValidationError(
                        "resume track width does not match campaign manifest")
                if int(existing_manifest["encoder_ticks_per_revolution"]) != \
                        geometry.encoder_ticks_per_revolution:
                    raise ValidationError(
                        "resume encoder ticks per revolution do not match campaign manifest")
                if (existing_manifest.get("matrix_identity") !=
                        campaign_matrix_identity(conditions)):
                    raise ValidationError(
                        "resume calibration matrix identity does not match campaign manifest")
            else:
                mode = ask_campaign_mode(operator_input)
                conditions = fixed_campaign_conditions(mode=mode)
            print(
                f"fixed_campaign_mode={CAMPAIGN_MODE_LABELS[mode]}; "
                f"planned_valid_repetitions="
                f"{planned_valid_repetition_count(conditions)}",
                flush=True)
            write_campaign_artifacts(
                evidence_dir, mode, geometry, attempts, args.compare_session)
            if args.dry_run:
                print(
                    "dry-run fixed campaign planning complete; no campaign "
                    "repetitions were executed.",
                    flush=True)
            while not args.dry_run:
                pending = next_pending(conditions, attempts)
                if pending is None:
                    break
                condition, repetition_number = pending
                interrupted_operation = "trial"
                current_trial_samples = TrialSamples()
                spec = condition.spec
                result, samples = execute_trial(
                    node, args, geometry, spec, operator_input,
                    sample_sink=retain_trial_samples,
                    emergency_cleanup=emergency_cleanup,
                    teledex=None,
                    pre_motion_confirmation=lambda: confirm_campaign_repetition(
                        operator_input, mode, condition, repetition_number,
                        geometry, evidence_dir))
                preliminary_report = build_trial_report(
                    result, samples, geometry,
                    wheel_tick_semantics=args.wheel_tick_semantics,
                    imu_bias_rad_s=args.imu_bias_rad_s)
                print(render_trial_report(preliminary_report), end="")
                auto_invalid = (
                    result.rejection_reason if (
                        result.valid is False and result.rejection_reason and
                        "Heading deviation exceeded 5" in result.rejection_reason)
                    else None)
                verdict, reason, notes = ask_campaign_validity(
                    operator_input, auto_invalid)
                interrupted_operation = "evidence"
                recorded = apply_operator_verdict(result, verdict, reason, notes)
                trial_dir = evidence.write_trial(recorded, samples, ())
                recorded = TrialResult(
                    spec=recorded.spec,
                    timestamp=recorded.timestamp,
                    measurements=recorded.measurements,
                    errors=recorded.errors,
                    valid=recorded.valid,
                    skipped=recorded.skipped,
                    rejection_reason=recorded.rejection_reason,
                    operator_notes=recorded.operator_notes,
                    evidence_dir=str(trial_dir),
                    initial_compass_heading_deg=(
                        recorded.initial_compass_heading_deg),
                    final_compass_heading_deg=recorded.final_compass_heading_deg,
                    manual_reference=recorded.manual_reference,
                    teledex=recorded.teledex,
                    laser=recorded.laser)
                results.append(recorded)
                attempts.append(attempt_record(
                    f"{condition.condition_id}-attempt-{len(attempts) + 1:04d}",
                    condition, repetition_number, recorded, samples,
                    wheel_radius_m=geometry.wheel_radius_m))
                write_campaign_artifacts(
                    evidence_dir, mode, geometry, attempts,
                    args.compare_session)
                if (
                        campaign_progress(conditions, attempts)[
                            "next_pending_condition_id"] !=
                        condition.condition_id):
                    atomic_write_json(
                        evidence_dir / f"{condition.condition_id}-summary.json",
                        condition_summary(condition, attempts))
                interrupted_operation = "between_trials"
                node.spin_for(max(args.between_trial_stop_s, 1.0))
            write_campaign_artifacts(
                evidence_dir, mode, geometry, attempts, args.compare_session)
            if mode == "calibration" and not args.dry_run:
                recommendation = radius_calibration_analysis(
                    attempts, geometry.wheel_radius_m, geometry.track_width_m)
                print(
                    "Recommended next wheel radius: "
                    f"{recommendation['aggregate_recommended_radius_m']}",
                    flush=True)
                answer = operator_input.read_text(
                    "Start another calibration iteration with a new evidence "
                    "identity using the recommended radius? [Y/N/ABORT]: ").strip()
                if answer.upper() == "ABORT":
                    raise KeyboardInterrupt("operator aborted after calibration")
                if answer.lower() in ("y", "yes"):
                    print(
                        "Start a new script invocation with --wheel-radius-m set "
                        "to the recommended value; this run will not mix radii.",
                        flush=True)
        else:
            spec = trials[0] if trials else campaign_menu.choose_initial(1)
            while spec is not None:
                interrupted_operation = "trial"
                current_trial_samples = TrialSamples()
                result, samples = execute_trial(
                    node, args, geometry, spec, operator_input,
                    sample_sink=retain_trial_samples,
                    emergency_cleanup=emergency_cleanup,
                    teledex=teledex)
                preliminary_report = build_trial_report(
                    result, samples, geometry,
                    wheel_tick_semantics=args.wheel_tick_semantics,
                    imu_bias_rad_s=args.imu_bias_rad_s)
                print(render_trial_report(preliminary_report), end="")
                verdict, reason, notes = operator.ask_validity()
                interrupted_operation = "evidence"
                recorded = apply_operator_verdict(result, verdict, reason, notes)
                trial_dir = evidence.write_trial(
                    recorded, samples,
                    () if teledex is None else teledex.trajectory)
                recorded = TrialResult(
                    spec=recorded.spec,
                    timestamp=recorded.timestamp,
                    measurements=recorded.measurements,
                    errors=recorded.errors,
                    valid=recorded.valid,
                    skipped=recorded.skipped,
                    rejection_reason=recorded.rejection_reason,
                    operator_notes=recorded.operator_notes,
                    evidence_dir=str(trial_dir),
                    initial_compass_heading_deg=(
                        recorded.initial_compass_heading_deg),
                    final_compass_heading_deg=recorded.final_compass_heading_deg,
                    manual_reference=recorded.manual_reference,
                    teledex=recorded.teledex, laser=recorded.laser)
                results.append(recorded)
                interrupted_operation = "menu"
                node.spin_for(max(args.between_trial_stop_s, 1.0))
                spec = (
                    menu.choose_next(recorded.spec, len(results) + 1)
                    if getattr(args, "cli_trial_mode", False) else
                    campaign_menu.choose_next(recorded.spec, len(results) + 1))
            evidence.write_summary(results)
    except BaseException as error:
        failure = error
        interrupted_operation = getattr(
            node, "interrupted_operation", interrupted_operation)
        if isinstance(error, KeyboardInterrupt):
            interrupted_operation = interrupted_operation or "unknown"
        if not emergency_cleanup.attempted:
            try:
                emergency_cleanup.stop(
                    args.zero_publish_timeout_s,
                    args.zero_publish_rate_hz)
            except BaseException as cleanup_error:
                failure = EmergencyStopCleanupError(error, cleanup_error)
                print(str(failure), file=sys.stderr)
        try:
            try:
                # Exactly one live-buffer snapshot is taken for this failure.
                complete_samples = merge_trial_samples(
                    latest_complete_snapshot, current_trial_samples, node.samples())
            except BaseException as snapshot_error:
                complete_samples = (
                    latest_complete_snapshot
                    if latest_complete_snapshot != TrialSamples() else
                    current_trial_samples)
                failure_context = {
                    "sample_snapshot_error": (
                        f"{type(snapshot_error).__name__}: {snapshot_error}"),
                }
            else:
                try:
                    failure_context = node.failure_context()
                except BaseException as context_error:
                    failure_context = {
                        "failure_context_error": (
                            f"{type(context_error).__name__}: {context_error}"),
                    }
            failure_context.update({
                "interrupted_operation": interrupted_operation,
                "cleanup_attempted": emergency_cleanup.attempted,
                "cleanup_completed": emergency_cleanup.completed,
                "cleanup_second_interrupt": emergency_cleanup.second_interrupt,
                "cleanup_result": emergency_cleanup.result,
                "ros_context_valid": (
                    emergency_cleanup.result.get("ros_context_valid")
                    if emergency_cleanup.result is not None else
                    node.cleanup_context_valid()),
                "zero_publisher_valid": (
                    emergency_cleanup.result.get("zero_publisher_valid")
                    if emergency_cleanup.result is not None else
                    node.cleanup_publisher_valid()),
                "last_confirmed_safe_state": node.confirmed_safe_state(),
                "cleanup_error": (
                    None if emergency_cleanup.error is None else
                    f"{type(emergency_cleanup.error).__name__}: "
                    f"{emergency_cleanup.error}"),
            })
            evidence.write_failure(
                results,
                failure,
                complete_samples,
                failure_context=failure_context)
            if teledex is not None:
                evidence.write_partial_teledex_trajectory(
                    teledex.trajectory, asdict(teledex.stream_diagnostics()))
        except BaseException as evidence_error:
            print(
                f"failed to write validation failure evidence: {evidence_error}",
                file=sys.stderr)
        if failure is not error:
            raise failure from error
        raise
    finally:
        try:
            try:
                if teledex is not None:
                    teledex.stop()
            finally:
                node.destroy_node()
        finally:
            shutdown_once()
    return 0


def validate_args(parser: argparse.ArgumentParser, args) -> None:
    """Apply the shared fail-closed CLI validation to parsed arguments."""
    finite_positive_options = (
        ("--publish-rate-hz", args.publish_rate_hz),
        ("--zero-publish-timeout-s", args.zero_publish_timeout_s),
        ("--zero-publish-rate-hz", args.zero_publish_rate_hz),
        ("--post-stop-settle-s", args.post_stop_settle_s),
        ("--stale-timeout-s", args.stale_timeout_s),
        ("--imu-motion-boundary-tolerance-s",
         args.imu_motion_boundary_tolerance_s),
        ("--max-angular-velocity-rad-s", args.max_angular_velocity_rad_s),
        ("--max-rotation-duration-s", args.max_rotation_duration_s),
        ("--min-linear-velocity-m-s", args.min_linear_velocity_m_s),
        ("--max-linear-velocity-m-s", args.max_linear_velocity_m_s),
        ("--min-translation-duration-s", args.min_translation_duration_s),
        ("--max-translation-duration-s", args.max_translation_duration_s),
        ("--teledex-stale-timeout-s", args.teledex_stale_timeout_s),
        ("--teledex-readiness-timeout-s", args.teledex_readiness_timeout_s),
        ("--teledex-axis-validation-min-displacement-m",
         args.teledex_axis_validation_min_displacement_m),
        ("--teledex-axis-validation-min-yaw-deg",
         args.teledex_axis_validation_min_yaw_deg),
        ("--laser-max-lateral-drift-m", args.laser_max_lateral_drift_m),
        ("--laser-max-yaw-drift-deg", args.laser_max_yaw_drift_deg),
    )
    for option, value in finite_positive_options:
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"{option} must be finite and positive")
    finite_nonnegative_options = (
        ("--between-trial-stop-s", args.between_trial_stop_s),
        ("--preflight-spin-s", args.preflight_spin_s),
        ("--zero-tolerance", args.zero_tolerance),
        ("--command-tolerance", args.command_tolerance),
        (
            "--stationary-linear-velocity-tolerance",
            args.stationary_linear_velocity_tolerance),
        (
            "--stationary-angular-velocity-tolerance",
            args.stationary_angular_velocity_tolerance),
    )
    for option, value in finite_nonnegative_options:
        if not math.isfinite(value) or value < 0.0:
            parser.error(f"{option} must be finite and nonnegative")
    if args.stationary_samples < 1:
        parser.error("--stationary-samples must be positive")
    if args.qos_depth < 1:
        parser.error("--qos-depth must be positive")
    if args.stationary_tick_tolerance < 0:
        parser.error("--stationary-tick-tolerance must be nonnegative")
    if args.stationary_tick_tolerance != 0:
        parser.error(
            "--stationary-tick-tolerance must remain 0; use the fixed rolling "
            "encoder chatter envelope")
    if not math.isfinite(args.imu_bias_rad_s):
        parser.error("--imu-bias-rad-s must be finite")
    if args.min_linear_velocity_m_s > args.max_linear_velocity_m_s:
        parser.error("minimum linear velocity exceeds maximum")
    if args.min_translation_duration_s > args.max_translation_duration_s:
        parser.error("minimum translation duration exceeds maximum")
    if len(args.laser_origin_base_m) != 3 or not all(
            math.isfinite(value) for value in args.laser_origin_base_m):
        parser.error("--laser-origin-base-m must contain three finite values")
    if not math.isfinite(args.laser_beam_yaw_deg):
        parser.error("--laser-beam-yaw-deg must be finite")
    if any(not name for name in args.ignore_diagnostic):
        parser.error("--ignore-diagnostic names must not be empty")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)
    try:
        return run(args)
    except KeyboardInterrupt:
        print(
            "interrupted; zero-command cleanup was attempted when available",
            file=sys.stderr)
        return 130
    except Exception as error:
        print(f"odometry validation failed: {error}", file=sys.stderr)
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
