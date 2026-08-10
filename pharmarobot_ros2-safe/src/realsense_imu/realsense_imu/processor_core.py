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

"""Hardware-free D455 IMU processing pipeline."""

from dataclasses import dataclass
import math
from typing import Optional

from realsense_imu.bias_estimator import BiasObservation
from realsense_imu.bias_estimator import GyroBiasEstimator
from realsense_imu.imu_transform import Covariance3
from realsense_imu.imu_transform import determinant
from realsense_imu.imu_transform import Matrix3
from realsense_imu.imu_transform import rotate_covariance
from realsense_imu.imu_transform import rotate_vector
from realsense_imu.imu_transform import validate_rotation_matrix
from realsense_imu.imu_transform import Vector3


ORIENTATION_UNAVAILABLE: Covariance3 = (
    -1.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
)


class SampleValidationError(ValueError):
    """Report one rejected raw IMU sample."""

    def __init__(self, reason: str, detail: str):
        """Retain one stable diagnostic reason with the exception detail."""
        super().__init__(detail)
        self.reason = reason


@dataclass(frozen=True)
class ProcessorConfig:
    """Validated frame and publication behavior."""

    expected_raw_frame: str
    processed_frame: str
    rotation: Matrix3
    publish_before_bias_ready: bool = False

    def __post_init__(self):
        """Validate frames and the complete configured 3D rotation."""
        if not self.expected_raw_frame.strip():
            raise ValueError("expected_raw_frame must not be empty")
        if not self.processed_frame.strip():
            raise ValueError("processed_frame must not be empty")
        if self.expected_raw_frame == self.processed_frame:
            raise ValueError("raw and processed frames must be different")
        validate_rotation_matrix(self.rotation)


@dataclass(frozen=True)
class RawImuSample:
    """ROS-independent raw IMU sample."""

    timestamp_s: float
    frame_id: str
    angular_velocity: Vector3
    linear_acceleration: Vector3
    angular_velocity_covariance: Covariance3
    linear_acceleration_covariance: Covariance3


@dataclass(frozen=True)
class ProcessedImuSample:
    """ROS-independent processed IMU result."""

    timestamp_s: float
    frame_id: str
    angular_velocity: Vector3
    linear_acceleration: Vector3
    angular_velocity_covariance: Covariance3
    linear_acceleration_covariance: Covariance3
    orientation_covariance: Covariance3 = ORIENTATION_UNAVAILABLE


@dataclass(frozen=True)
class ProcessingResult:
    """Bias and optional publication result for one raw sample."""

    output: Optional[ProcessedImuSample]
    bias_observation: BiasObservation


def _validate_vector(values, name):
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise SampleValidationError(
            "nonfinite_sample",
            f"{name} must contain 3 finite values",
        )


def _validate_covariance(values, name):
    if len(values) != 9 or not all(math.isfinite(value) for value in values):
        raise SampleValidationError(
            "invalid_covariance",
            f"{name} must contain 9 finite values",
        )
    for row in range(3):
        for column in range(3):
            if abs(values[row * 3 + column] - values[column * 3 + row]) > (
                1.0e-9
            ):
                raise SampleValidationError(
                    "invalid_covariance",
                    f"{name} must be symmetric",
                )
    matrix = tuple(
        tuple(values[row * 3 + column] for column in range(3))
        for row in range(3)
    )
    tolerance = 1.0e-12
    principal_minors = (
        matrix[0][0],
        matrix[1][1],
        matrix[2][2],
        matrix[0][0] * matrix[1][1] - matrix[0][1] ** 2,
        matrix[0][0] * matrix[2][2] - matrix[0][2] ** 2,
        matrix[1][1] * matrix[2][2] - matrix[1][2] ** 2,
        determinant(matrix),
    )
    if any(value < -tolerance for value in principal_minors):
        raise SampleValidationError(
            "invalid_covariance",
            f"{name} must be positive semidefinite",
        )


class D455ImuProcessorCore:
    """Validate, bias-correct, and rotate raw D455 IMU samples."""

    def __init__(
        self,
        config: ProcessorConfig,
        bias_estimator: GyroBiasEstimator,
    ):
        """Create one ROS-independent processing pipeline."""
        self.config = config
        self.bias_estimator = bias_estimator

    def _validate(self, sample: RawImuSample):
        if sample.frame_id != self.config.expected_raw_frame:
            raise SampleValidationError(
                "unexpected_frame",
                "raw IMU frame mismatch: "
                f"expected {self.config.expected_raw_frame!r}, "
                f"received {sample.frame_id!r}",
            )
        if not math.isfinite(sample.timestamp_s) or sample.timestamp_s <= 0.0:
            raise SampleValidationError(
                "invalid_timestamp",
                "raw IMU timestamp must be finite and positive",
            )
        _validate_vector(sample.angular_velocity, "angular_velocity")
        _validate_vector(sample.linear_acceleration, "linear_acceleration")
        _validate_covariance(
            sample.angular_velocity_covariance,
            "angular_velocity_covariance",
        )
        _validate_covariance(
            sample.linear_acceleration_covariance,
            "linear_acceleration_covariance",
        )

    def process(
        self,
        sample: RawImuSample,
        *,
        command_zero: Optional[bool] = None,
    ) -> ProcessingResult:
        """Process one sample without any ROS or hardware dependency."""
        try:
            self._validate(sample)
        except SampleValidationError as error:
            self.bias_estimator.reject_sample(error.reason)
            raise

        observation = self.bias_estimator.observe(
            sample.timestamp_s,
            sample.angular_velocity,
            sample.linear_acceleration,
            command_zero,
        )
        if not observation.sample_valid:
            raise SampleValidationError(
                observation.reason,
                f"bias estimator rejected sample: {observation.reason}",
            )
        if (
            not observation.snapshot.calibrated
            and not self.config.publish_before_bias_ready
        ):
            return ProcessingResult(None, observation)

        bias = observation.snapshot.bias
        corrected_raw_gyro = tuple(
            sample.angular_velocity[axis] - bias[axis]
            for axis in range(3)
        )
        output = ProcessedImuSample(
            timestamp_s=sample.timestamp_s,
            frame_id=self.config.processed_frame,
            angular_velocity=rotate_vector(
                self.config.rotation, corrected_raw_gyro
            ),
            linear_acceleration=rotate_vector(
                self.config.rotation, sample.linear_acceleration
            ),
            angular_velocity_covariance=rotate_covariance(
                self.config.rotation,
                sample.angular_velocity_covariance,
            ),
            linear_acceleration_covariance=rotate_covariance(
                self.config.rotation,
                sample.linear_acceleration_covariance,
            ),
        )
        return ProcessingResult(output, observation)
