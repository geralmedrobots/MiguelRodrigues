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

"""Validate, bias-correct, and rotate raw D455 IMU measurements."""

import math
import time
from typing import Optional, Sequence

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from realsense_imu.bias_estimator import BiasConfig
from realsense_imu.bias_estimator import GyroBiasEstimator
from realsense_imu.imu_transform import determinant
from realsense_imu.imu_transform import quaternion_to_rotation_matrix
from realsense_imu.processor_core import D455ImuProcessorCore
from realsense_imu.processor_core import ProcessorConfig
from realsense_imu.processor_core import RawImuSample
from realsense_imu.processor_core import SampleValidationError
from realsense_imu.processor_health import HealthConfig
from realsense_imu.processor_health import ProcessorHealthTracker


def diagnostic_level_byte(level: int) -> bytes:
    """Encode a diagnostic level for ROS Humble's uint8 Python binding."""
    if isinstance(level, bool) or not isinstance(level, int):
        raise ValueError("diagnostic level must be an integer")
    if not 0 <= level <= 255:
        raise ValueError("diagnostic level must be an unsigned byte")
    return bytes((level,))


class D455ImuProcessor(Node):
    """Publish robot-aligned IMU data without creating an orientation."""

    def __init__(self) -> None:
        """Build the configured processor, subscriptions, and diagnostics."""
        super().__init__("d455_imu_processor")
        self._declare_parameters()

        input_topic = self._string_parameter("input_topic")
        output_topic = self._string_parameter("output_topic")
        diagnostics_topic = self._string_parameter("diagnostics_topic")
        raw_frame = self._string_parameter("expected_raw_frame")
        processed_frame = self._string_parameter("processed_frame")
        if self.resolve_topic_name(input_topic) == self.resolve_topic_name(
            output_topic
        ):
            raise ValueError("input_topic and output_topic must be different")

        quaternion = tuple(
            self.get_parameter("rotation_quaternion_xyzw").value
        )
        rotation = quaternion_to_rotation_matrix(quaternion)
        self._transform_values = {
            "raw_frame": raw_frame,
            "processed_frame": processed_frame,
            "source": "configured_quaternion_xyzw",
            "tf_lookup": "not_used",
            "tf_publication": "not_owned_by_processor",
            "quaternion_xyzw": ",".join(
                f"{value:.12g}" for value in quaternion
            ),
            "rotation_row_0": ",".join(
                f"{value:.12g}" for value in rotation[0]
            ),
            "rotation_row_1": ",".join(
                f"{value:.12g}" for value in rotation[1]
            ),
            "rotation_row_2": ",".join(
                f"{value:.12g}" for value in rotation[2]
            ),
            "determinant": f"{determinant(rotation):.12g}",
            "validation": (
                "finite,normalized,orthonormal,determinant_plus_one"
            ),
        }
        self._covariance_values = {
            "source": "raw_message",
            "operation": "R_times_covariance_times_R_transpose",
            "validation": "finite,symmetric,positive_semidefinite",
        }
        bias_config = BiasConfig(
            warmup_duration_s=self._float_parameter(
                "bias.warmup_duration_s"
            ),
            warmup_min_samples=self._int_parameter(
                "bias.warmup_min_samples"
            ),
            stationary_window_duration_s=self._float_parameter(
                "bias.stationary_window_duration_s"
            ),
            stationary_min_samples=self._int_parameter(
                "bias.stationary_min_samples"
            ),
            gyro_stationary_threshold_rad_s=self._float_parameter(
                "bias.gyro_stationary_threshold_rad_s"
            ),
            gravity_m_s2=self._float_parameter("bias.gravity_m_s2"),
            acceleration_tolerance_m_s2=self._float_parameter(
                "bias.acceleration_tolerance_m_s2"
            ),
            max_sample_gap_s=self._float_parameter(
                "bias.max_sample_gap_s"
            ),
            max_residual_stddev_rad_s=self._float_parameter(
                "bias.max_residual_stddev_rad_s"
            ),
            online_update_enabled=self._bool_parameter(
                "bias.online_update_enabled"
            ),
            online_update_alpha=self._float_parameter(
                "bias.online_update_alpha"
            ),
            require_command_zero=self._bool_parameter(
                "bias.require_cmd_vel_zero"
            ),
        )
        self._bias_estimator = GyroBiasEstimator(bias_config)
        self._core = D455ImuProcessorCore(
            ProcessorConfig(
                expected_raw_frame=raw_frame,
                processed_frame=processed_frame,
                rotation=rotation,
                publish_before_bias_ready=self._bool_parameter(
                    "publish_before_bias_ready"
                ),
            ),
            self._bias_estimator,
        )
        self._health = ProcessorHealthTracker(
            HealthConfig(
                stale_timeout_s=self._float_parameter(
                    "diagnostics.stale_timeout_s"
                ),
                minimum_output_rate_hz=self._float_parameter(
                    "diagnostics.minimum_output_rate_hz"
                ),
                rate_window_samples=self._int_parameter(
                    "diagnostics.rate_window_samples"
                ),
            )
        )
        self._transform_calibrated = self._bool_parameter(
            "transform_calibrated"
        )
        self._covariance_calibrated = self._bool_parameter(
            "covariance_calibrated"
        )
        self._use_cmd_vel_zero = self._bool_parameter(
            "bias.use_cmd_vel_zero_if_available"
        )
        if (
            bias_config.require_command_zero
            and not self._use_cmd_vel_zero
        ):
            raise ValueError(
                "bias.require_cmd_vel_zero requires "
                "bias.use_cmd_vel_zero_if_available"
            )
        self._cmd_vel_timeout = self._float_parameter(
            "bias.cmd_vel_zero_timeout_s"
        )
        if self._cmd_vel_timeout <= 0.0:
            raise ValueError("bias.cmd_vel_zero_timeout_s must be positive")
        self._last_cmd_vel_receive = None
        self._last_cmd_vel_exact_zero = None

        self._publisher = self.create_publisher(
            Imu, output_topic, qos_profile_sensor_data
        )
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray, diagnostics_topic, 10
        )
        self._subscription = self.create_subscription(
            Imu,
            input_topic,
            self._process,
            qos_profile_sensor_data,
        )
        self._cmd_vel_subscription = None
        if self._use_cmd_vel_zero:
            self._cmd_vel_subscription = self.create_subscription(
                Twist,
                self._string_parameter("bias.cmd_vel_topic"),
                self._record_cmd_vel,
                qos_profile_sensor_data,
            )
        diagnostic_period = self._float_parameter(
            "diagnostics.publish_period_s"
        )
        if diagnostic_period <= 0.0:
            raise ValueError(
                "diagnostics.publish_period_s must be positive"
            )
        self._diagnostic_timer = self.create_timer(
            diagnostic_period, self._publish_diagnostics
        )

    def _declare_parameters(self):
        parameters = {
            "input_topic": "/imu/d455/data_raw",
            "output_topic": "/imu/data",
            "diagnostics_topic": "/diagnostics",
            "expected_raw_frame": "d455_imu_optical_frame",
            "processed_frame": "d455_imu_link",
            "rotation_quaternion_xyzw": [0.5, 0.5, 0.5, 0.5],
            "transform_calibrated": False,
            "covariance_calibrated": False,
            "publish_before_bias_ready": False,
            "bias.warmup_duration_s": 2.0,
            "bias.warmup_min_samples": 200,
            "bias.stationary_window_duration_s": 3.0,
            "bias.stationary_min_samples": 400,
            "bias.gyro_stationary_threshold_rad_s": 0.15,
            "bias.gravity_m_s2": 9.80665,
            "bias.acceleration_tolerance_m_s2": 1.0,
            "bias.max_sample_gap_s": 0.1,
            "bias.max_residual_stddev_rad_s": 0.02,
            "bias.online_update_enabled": True,
            "bias.online_update_alpha": 0.1,
            "bias.use_cmd_vel_zero_if_available": True,
            "bias.require_cmd_vel_zero": True,
            "bias.cmd_vel_topic": "/cmd_vel/safe",
            "bias.cmd_vel_zero_timeout_s": 0.5,
            "diagnostics.publish_period_s": 1.0,
            "diagnostics.stale_timeout_s": 0.5,
            "diagnostics.minimum_output_rate_hz": 50.0,
            "diagnostics.rate_window_samples": 200,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)

    def _string_parameter(self, name: str) -> str:
        value = self.get_parameter(name).value
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{name} must be a nonempty string")
        return value.strip()

    def _float_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    def _int_parameter(self, name: str) -> int:
        value = self.get_parameter(name).value
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{name} must be an integer")
        return value

    def _bool_parameter(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if not isinstance(value, bool):
            raise ValueError(f"{name} must be a boolean")
        return value

    @staticmethod
    def _exact_zero_twist(message: Twist) -> bool:
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        return all(math.isfinite(value) and value == 0.0 for value in values)

    def _record_cmd_vel(self, message: Twist):
        self._last_cmd_vel_receive = time.monotonic()
        self._last_cmd_vel_exact_zero = self._exact_zero_twist(message)

    def _command_zero_state(self, now_monotonic_s: float):
        if not self._use_cmd_vel_zero:
            return None
        if self._last_cmd_vel_receive is None:
            return None
        if now_monotonic_s - self._last_cmd_vel_receive > (
            self._cmd_vel_timeout
        ):
            return None
        return self._last_cmd_vel_exact_zero

    @staticmethod
    def _timestamp_seconds(message: Imu) -> float:
        return (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )

    def _process(self, message: Imu):
        now = time.monotonic()
        self._health.record_raw_received(now)
        sample = RawImuSample(
            timestamp_s=self._timestamp_seconds(message),
            frame_id=message.header.frame_id,
            angular_velocity=(
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ),
            linear_acceleration=(
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ),
            angular_velocity_covariance=tuple(
                message.angular_velocity_covariance
            ),
            linear_acceleration_covariance=tuple(
                message.linear_acceleration_covariance
            ),
        )
        try:
            result = self._core.process(
                sample,
                command_zero=self._command_zero_state(now),
            )
        except SampleValidationError as error:
            self._health.record_rejection(error.reason)
            if error.reason == "dropout":
                self._health.record_dropout()
            return

        self._health.record_raw_accepted()
        if result.output is None:
            return

        output = Imu()
        output.header.stamp = message.header.stamp
        output.header.frame_id = result.output.frame_id
        output.orientation_covariance = list(
            result.output.orientation_covariance
        )
        output.angular_velocity.x = result.output.angular_velocity[0]
        output.angular_velocity.y = result.output.angular_velocity[1]
        output.angular_velocity.z = result.output.angular_velocity[2]
        output.angular_velocity_covariance = list(
            result.output.angular_velocity_covariance
        )
        output.linear_acceleration.x = result.output.linear_acceleration[0]
        output.linear_acceleration.y = result.output.linear_acceleration[1]
        output.linear_acceleration.z = result.output.linear_acceleration[2]
        output.linear_acceleration_covariance = list(
            result.output.linear_acceleration_covariance
        )
        self._publisher.publish(output)
        self._health.record_output(now, covariance_valid=True)

    def _publish_diagnostics(self):
        statuses = self._health.snapshot(
            time.monotonic(),
            bias=self._bias_estimator.snapshot(),
            transform_calibrated=self._transform_calibrated,
            covariance_calibrated=self._covariance_calibrated,
            transform_values=self._transform_values,
            covariance_values=self._covariance_values,
        )
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        for status in statuses:
            diagnostic = DiagnosticStatus()
            diagnostic.name = status.name
            diagnostic.hardware_id = "intel-realsense-d455"
            diagnostic.level = diagnostic_level_byte(status.level)
            diagnostic.message = status.message
            diagnostic.values = [
                KeyValue(key=key, value=value)
                for key, value in sorted(status.values.items())
            ]
            message.status.append(diagnostic)
        self._diagnostics_publisher.publish(message)


def main(args: Optional[Sequence[str]] = None) -> None:
    """Run the D455 IMU processor."""
    rclpy.init(args=args)
    node = None
    try:
        node = D455ImuProcessor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
