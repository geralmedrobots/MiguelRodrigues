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

from dataclasses import replace

import pytest

from realsense_imu.bias_estimator import BiasConfig
from realsense_imu.bias_estimator import GyroBiasEstimator


GRAVITY = (0.0, 0.0, 9.80665)
BIAS = (0.01, -0.02, 0.03)


def config(**overrides):
    base = BiasConfig(
        warmup_duration_s=0.0,
        warmup_min_samples=0,
        stationary_window_duration_s=0.2,
        stationary_min_samples=3,
        gyro_stationary_threshold_rad_s=0.2,
        gravity_m_s2=9.80665,
        acceleration_tolerance_m_s2=0.2,
        max_sample_gap_s=0.2,
        max_residual_stddev_rad_s=0.01,
        online_update_enabled=True,
        online_update_alpha=0.5,
        require_command_zero=False,
    )
    return replace(base, **overrides)


def calibrate(estimator):
    observations = [
        estimator.observe(timestamp, BIAS, GRAVITY, True)
        for timestamp in (1.0, 1.1, 1.21)
    ]
    return observations[-1]


def test_stationary_window_is_accepted_and_exposes_evidence():
    estimator = GyroBiasEstimator(config())

    result = calibrate(estimator)

    assert result.bias_updated
    assert result.snapshot.calibrated
    assert result.snapshot.bias == pytest.approx(BIAS)
    assert result.snapshot.residual_stddev == pytest.approx((0.0, 0.0, 0.0))
    assert result.snapshot.last_update_sample_count == 3
    assert result.snapshot.last_update_timestamp_s == pytest.approx(1.21)


@pytest.mark.parametrize(
    "timestamp,gyro,acceleration,expected_reason",
    [
        (1.1, (1.0, 0.0, 0.0), GRAVITY, "motion"),
        (1.1, BIAS, (0.0, 0.0, 5.0), "motion"),
        (1.1, (float("nan"), 0.0, 0.0), GRAVITY, "nonfinite_sample"),
    ],
)
def test_motion_nonfinite_and_wrong_gravity_are_rejected_for_learning(
    timestamp, gyro, acceleration, expected_reason
):
    estimator = GyroBiasEstimator(config())
    estimator.observe(1.0, BIAS, GRAVITY, True)

    result = estimator.observe(timestamp, gyro, acceleration, True)

    assert result.reason == expected_reason
    assert not result.bias_updated
    assert result.snapshot.candidate_sample_count == 0


def test_timestamp_reset_and_dropout_restart_candidate_window():
    estimator = GyroBiasEstimator(config())
    estimator.observe(1.0, BIAS, GRAVITY, True)
    reset = estimator.observe(0.9, BIAS, GRAVITY, True)
    dropout = estimator.observe(1.5, BIAS, GRAVITY, True)

    assert not reset.sample_valid
    assert reset.reason == "timestamp_reset"
    assert not dropout.sample_valid
    assert dropout.reason == "dropout"
    assert not dropout.bias_updated
    assert dropout.snapshot.candidate_sample_count == 0
    assert dropout.snapshot.warmup_sample_count == 0


def test_required_command_zero_blocks_missing_or_nonzero_command():
    estimator = GyroBiasEstimator(config(require_command_zero=True))

    missing = estimator.observe(1.0, BIAS, GRAVITY, None)
    nonzero = estimator.observe(1.1, BIAS, GRAVITY, False)

    assert missing.reason == "motion"
    assert nonzero.reason == "motion"
    assert not estimator.calibrated


def test_online_bias_never_updates_during_motion():
    estimator = GyroBiasEstimator(config())
    calibrate(estimator)
    original_bias = estimator.bias

    for timestamp in (1.31, 1.41, 1.51, 1.61):
        result = estimator.observe(
            timestamp, (1.0, 0.0, 0.0), GRAVITY, False
        )
        assert result.reason == "motion"
        assert not result.bias_updated

    assert estimator.bias == original_bias
    assert estimator.snapshot().candidate_sample_count == 0


def test_unstable_stationary_window_does_not_update_bias():
    estimator = GyroBiasEstimator(
        config(max_residual_stddev_rad_s=0.001)
    )
    samples = (
        (1.0, (0.0, 0.0, 0.0)),
        (1.1, (0.01, 0.0, 0.0)),
        (1.21, (-0.01, 0.0, 0.0)),
    )

    result = None
    for timestamp, gyro in samples:
        result = estimator.observe(timestamp, gyro, GRAVITY, True)

    assert result.reason == "unstable_stationary_window"
    assert not result.bias_updated
    assert not estimator.calibrated
