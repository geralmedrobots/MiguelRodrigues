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
import math
from pathlib import Path
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
from odometry_validation.core import merge_trial_samples
from odometry_validation.core import run_with_emergency_stop
from odometry_validation.core import render_trial_report


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
        self.command_start_timestamp_s: Optional[float] = None
        self.command_end_timestamp_s: Optional[float] = None
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
        self.command_start_timestamp_s = None
        self.command_end_timestamp_s = None
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
            raise ValidationError(
                "command mismatch on /cmd_vel/safe during controlled stop")

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
            tick_window = post_zero_ticks[-(required + 1):]
            delta_samples = tuple(
                StationaritySample(
                    previous_timestamp_s=previous.timestamp_s,
                    timestamp_s=current.timestamp_s,
                    previous_left_ticks=previous.left_ticks,
                    previous_right_ticks=previous.right_ticks,
                    left_ticks=current.left_ticks,
                    right_ticks=current.right_ticks,
                    left_delta_ticks=current.left_ticks - previous.left_ticks,
                    right_delta_ticks=current.right_ticks - previous.right_ticks)
                for previous, current in zip(tick_window, tick_window[1:]))
        else:
            tick_window = post_zero_ticks[-required:]
            delta_samples = tuple(
                StationaritySample(
                    previous_timestamp_s=sample.timestamp_s,
                    timestamp_s=sample.timestamp_s,
                    previous_left_ticks=0,
                    previous_right_ticks=0,
                    left_ticks=sample.left_ticks,
                    right_ticks=sample.right_ticks,
                    left_delta_ticks=sample.left_ticks,
                    right_delta_ticks=sample.right_ticks)
                for sample in tick_window)
        enough_ticks = len(delta_samples) >= required
        ticks_stationary = enough_ticks and all(
            abs(sample.left_delta_ticks) <= self.args.stationary_tick_tolerance and
            abs(sample.right_delta_ticks) <= self.args.stationary_tick_tolerance
            for sample in delta_samples)

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
        if self.post_zero_safe_start is None:
            post_zero_safe = ()
        else:
            post_zero_safe = tuple(
                self.safe_commands[self.post_zero_safe_start:])
        safe_zero = (
            bool(post_zero_safe) and
            self.last_safe is not None and
            twist_is_zero(self.last_safe, self.args.zero_tolerance))
        failures = []
        if not enough_ticks:
            failures.append(
                f"insufficient encoder deltas ({len(delta_samples)}/{required})")
        elif not ticks_stationary:
            failures.append("encoder motion exceeded stationarity tolerance")
        if not enough_odom:
            failures.append(
                f"insufficient odometry twist samples ({len(odom_window)}/{required})")
        elif not odom_stationary:
            failures.append("odometry twist exceeded stationarity tolerance")
        if not post_zero_safe:
            failures.append("no fresh /cmd_vel/safe sample after zero publication")
        elif not safe_zero:
            failures.append("/cmd_vel/safe was not zero")
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
                max(0.0, assessment_timestamp_s - self.first_zero_timestamp_s)))
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
            raise ValidationError("missing /cmd_vel/safe during trial")
        linear_error = abs(self.last_safe.linear.x - spec.linear_x)
        angular_error = abs(self.last_safe.angular.z - spec.angular_z)
        if (
                linear_error > self.args.command_tolerance or
                angular_error > self.args.command_tolerance):
            raise ValidationError("command mismatch on /cmd_vel/safe")

    def run_command_phase(self, spec: TrialSpec) -> None:
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
                self.publish_twist(spec.linear_x, spec.angular_z)
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
            "command_start_timestamp_s": self.command_start_timestamp_s,
            "command_end_timestamp_s": self.command_end_timestamp_s,
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
    parser.add_argument("--stationary-tick-tolerance", type=int, default=0)
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


def execute_trial(
        node: OdometryValidationNode,
        args,
        geometry: GeometryConfig,
        spec: TrialSpec,
        operator_input: ResponsiveOperatorInput,
        sample_sink: Optional[Callable[[TrialSamples], None]] = None,
        emergency_cleanup: Optional[EmergencyCleanupOnce] = None
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
        node.interrupted_operation = "preflight"
        node.reset_trial_buffers()
        node.verify_preflight()
        physical_measurement = None
        physical_final = None
        if (
                spec.movement_type == "rotation" and
                args.rotation_physical_mode == "compass"):
            node.interrupted_operation = "input"
            physical_initial = None if args.dry_run else operator_input.read_float(
                "Enter initial compass heading: ")
        else:
            physical_initial = None

        if args.dry_run:
            node.get_logger().warn(
                "dry-run active: nonzero /cmd_vel/test publication skipped")
        else:
            node.run_command_phase(spec)
        controlled_stop.stop(
            args.post_stop_settle_s,
            args.zero_publish_rate_hz,
            mode="controlled")

        if not args.dry_run:
            if spec.movement_type == "rotation":
                if args.rotation_physical_mode == "compass":
                    node.interrupted_operation = "input"
                    physical_final = operator_input.read_float(
                        "Enter final compass heading: ")
                    physical_measurement = compass_rotation_radians(
                        physical_initial, physical_final)
                else:
                    node.interrupted_operation = "input"
                    measured = operator_input.read_float(
                        "Enter measured physical rotation angle: ")
                    physical_measurement = (
                        measured if args.rotation_angle_unit == "rad"
                        else math.radians(measured))
            else:
                node.interrupted_operation = "input"
                physical_measurement = operator_input.read_float(
                    "Enter measured physical displacement (meters): ")
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
                args, "imu_motion_boundary_tolerance_s", 0.1))
        result = make_trial_result(
            spec,
            measurements,
            initial_compass_heading_deg=physical_initial,
            final_compass_heading_deg=physical_final)
        return result, samples

    result = run_with_emergency_stop(
        action,
        emergency,
        args.zero_publish_timeout_s,
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
            final_compass_heading_deg=result.final_compass_heading_deg)
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
            final_compass_heading_deg=result.final_compass_heading_deg)
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
        final_compass_heading_deg=result.final_compass_heading_deg)


def run(args) -> int:
    geometry = GeometryConfig(
        wheel_radius_m=args.wheel_radius_m,
        track_width_m=args.track_width_m,
        encoder_ticks_per_revolution=args.encoder_ticks_per_revolution)
    trials = build_trials(args)
    if len(trials) != 1:
        raise ValidationError(
            "interactive workflow requires exactly one initial trial; "
            "select one direction, velocity, and duration")
    evidence = EvidenceWriter(Path(args.evidence_root))
    evidence_dir = evidence.create({
        "dry_run": args.dry_run,
        "initial_trial_count": len(trials),
        "interactive_menu": True,
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
        "role": "test orchestrator and data collection framework",
    })
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

    def retain_trial_samples(samples: TrialSamples) -> None:
        nonlocal current_trial_samples, latest_complete_snapshot
        current_trial_samples = samples
        latest_complete_snapshot = samples

    try:
        spec = trials[0]
        while spec is not None:
            interrupted_operation = "trial"
            current_trial_samples = TrialSamples()
            result, samples = execute_trial(
                node, args, geometry, spec, operator_input,
                sample_sink=retain_trial_samples,
                emergency_cleanup=emergency_cleanup)
            verdict, reason, notes = operator.ask_validity()
            interrupted_operation = "evidence"
            recorded = apply_operator_verdict(result, verdict, reason, notes)
            trial_dir = evidence.write_trial(recorded, samples)
            if recorded.valid and not recorded.skipped:
                report = build_trial_report(
                    recorded,
                    samples,
                    geometry,
                    wheel_tick_semantics=args.wheel_tick_semantics,
                    imu_bias_rad_s=args.imu_bias_rad_s)
                print(render_trial_report(report), end="")
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
                final_compass_heading_deg=recorded.final_compass_heading_deg)
            results.append(recorded)
            interrupted_operation = "menu"
            node.spin_for(max(args.between_trial_stop_s, 1.0))
            spec = menu.choose_next(recorded.spec, len(results) + 1)
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
        except BaseException as evidence_error:
            print(
                f"failed to write validation failure evidence: {evidence_error}",
                file=sys.stderr)
        if failure is not error:
            raise failure from error
        raise
    finally:
        try:
            node.destroy_node()
        finally:
            shutdown_once()
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
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
    if args.stationary_tick_tolerance < 0:
        parser.error("--stationary-tick-tolerance must be nonnegative")
    if args.min_linear_velocity_m_s > args.max_linear_velocity_m_s:
        parser.error("minimum linear velocity exceeds maximum")
    if args.min_translation_duration_s > args.max_translation_duration_s:
        parser.error("minimum translation duration exceeds maximum")
    if any(not name for name in args.ignore_diagnostic):
        parser.error("--ignore-diagnostic names must not be empty")
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
