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

import pytest

from realsense_imu.bias_estimator import BiasConfig
from realsense_imu.bias_estimator import GyroBiasEstimator
from realsense_imu.imu_transform import quaternion_to_rotation_matrix
from realsense_imu.processor_core import D455ImuProcessorCore
from realsense_imu.processor_core import ProcessorConfig
from realsense_imu.processor_core import RawImuSample
from realsense_imu.processor_core import SampleValidationError


RAW_FRAME = "d455_imu_optical_frame"
PROCESSED_FRAME = "d455_imu_link"
COVARIANCE = (
    1.0, 0.1, 0.2,
    0.1, 2.0, 0.3,
    0.2, 0.3, 3.0,
)


def bias_config():
    return BiasConfig(
        warmup_duration_s=0.0,
        warmup_min_samples=0,
        stationary_window_duration_s=0.0,
        stationary_min_samples=1,
        gyro_stationary_threshold_rad_s=0.5,
        gravity_m_s2=9.80665,
        acceleration_tolerance_m_s2=0.2,
        max_sample_gap_s=0.2,
        max_residual_stddev_rad_s=0.1,
        online_update_enabled=False,
        online_update_alpha=0.1,
    )


def core():
    return D455ImuProcessorCore(
        ProcessorConfig(
            expected_raw_frame=RAW_FRAME,
            processed_frame=PROCESSED_FRAME,
            rotation=quaternion_to_rotation_matrix(
                (0.5, 0.5, 0.5, 0.5)
            ),
        ),
        GyroBiasEstimator(bias_config()),
    )


def sample(timestamp=1.0, frame=RAW_FRAME, gyro=(0.1, 0.2, 0.3)):
    return RawImuSample(
        timestamp_s=timestamp,
        frame_id=frame,
        angular_velocity=gyro,
        linear_acceleration=(0.0, 0.0, 9.80665),
        angular_velocity_covariance=COVARIANCE,
        linear_acceleration_covariance=COVARIANCE,
    )


def test_expected_frame_is_accepted_and_unexpected_frame_rejected():
    processor = core()
    accepted = processor.process(sample())

    assert accepted.output is not None
    assert accepted.output.frame_id == PROCESSED_FRAME

    with pytest.raises(SampleValidationError, match="frame mismatch"):
        processor.process(sample(timestamp=1.1, frame="base_link"))


def test_orientation_remains_explicitly_unavailable():
    result = core().process(sample())

    assert result.output.orientation_covariance[0] == -1.0
    assert result.output.orientation_covariance[1:] == (0.0,) * 8


def test_output_is_withheld_until_initial_bias_is_calibrated():
    estimator = GyroBiasEstimator(
        BiasConfig(
            warmup_duration_s=0.0,
            warmup_min_samples=0,
            stationary_window_duration_s=0.1,
            stationary_min_samples=2,
            gyro_stationary_threshold_rad_s=0.5,
            gravity_m_s2=9.80665,
            acceleration_tolerance_m_s2=0.2,
            max_sample_gap_s=0.2,
            max_residual_stddev_rad_s=0.1,
            online_update_enabled=False,
            online_update_alpha=0.1,
        )
    )
    processor = D455ImuProcessorCore(
        ProcessorConfig(
            expected_raw_frame=RAW_FRAME,
            processed_frame=PROCESSED_FRAME,
            rotation=quaternion_to_rotation_matrix(
                (0.5, 0.5, 0.5, 0.5)
            ),
        ),
        estimator,
    )

    first = processor.process(sample(timestamp=1.0))
    second = processor.process(sample(timestamp=1.1))

    assert first.output is None
    assert not first.bias_observation.snapshot.calibrated
    assert second.output is not None
    assert second.bias_observation.snapshot.calibrated


def test_bias_is_subtracted_in_raw_frame_before_rotation():
    processor = core()
    first = processor.process(sample(gyro=(0.1, 0.2, 0.3)))
    second = processor.process(
        sample(timestamp=1.1, gyro=(0.1, 1.2, 0.3))
    )

    assert first.bias_observation.snapshot.bias == pytest.approx(
        (0.1, 0.2, 0.3)
    )
    assert second.output.angular_velocity == pytest.approx((0.0, 0.0, 1.0))


def test_vectors_and_covariances_are_rotated():
    result = core().process(sample())

    assert result.output.linear_acceleration == pytest.approx(
        (9.80665, 0.0, 0.0)
    )
    assert result.output.angular_velocity_covariance == pytest.approx(
        (
            3.0, 0.2, 0.3,
            0.2, 1.0, 0.1,
            0.3, 0.1, 2.0,
        )
    )


@pytest.mark.parametrize(
    "field,value,reason",
    [
        ("angular_velocity", (float("nan"), 0.0, 0.0), "nonfinite_sample"),
        (
            "angular_velocity_covariance",
            (float("nan"),) * 9,
            "invalid_covariance",
        ),
        (
            "angular_velocity_covariance",
            (
                -1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
            ),
            "invalid_covariance",
        ),
    ],
)
def test_nonfinite_sample_or_covariance_is_rejected(field, value, reason):
    values = sample().__dict__.copy()
    values[field] = value

    with pytest.raises(SampleValidationError) as error:
        core().process(RawImuSample(**values))

    assert error.value.reason == reason


def test_timestamp_dropout_is_rejected_without_processed_output():
    processor = core()
    processor.process(sample(timestamp=1.0))

    with pytest.raises(SampleValidationError) as error:
        processor.process(sample(timestamp=1.5))

    assert error.value.reason == "dropout"
