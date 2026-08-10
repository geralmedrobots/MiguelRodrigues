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

from realsense_imu.bias_estimator import BiasSnapshot
from realsense_imu.processor_health import ERROR
from realsense_imu.processor_health import HealthConfig
from realsense_imu.processor_health import OK
from realsense_imu.processor_health import ProcessorHealthTracker
from realsense_imu.processor_health import WARN


def bias(calibrated=False):
    return BiasSnapshot(
        state="calibrated" if calibrated else "warming_up",
        calibrated=calibrated,
        bias=(0.01, 0.02, 0.03),
        residual_stddev=(0.001, 0.001, 0.001),
        warmup_sample_count=10,
        candidate_sample_count=5,
        last_update_sample_count=100 if calibrated else 0,
        last_update_timestamp_s=1.0 if calibrated else None,
    )


def by_name(statuses):
    return {status.name: status for status in statuses}


def test_diagnostics_transition_from_missing_to_healthy_to_stale():
    tracker = ProcessorHealthTracker(
        HealthConfig(
            stale_timeout_s=0.5,
            minimum_output_rate_hz=50.0,
            rate_window_samples=10,
        )
    )

    initial = by_name(
        tracker.snapshot(
            0.0,
            bias=bias(False),
            transform_calibrated=False,
            covariance_calibrated=False,
        )
    )
    assert initial["D455 IMU/Raw Input"].level == ERROR
    assert initial["D455 IMU/Bias"].level == WARN

    tracker.record_raw_received(1.0)
    tracker.record_raw_accepted()
    tracker.record_output(1.0, covariance_valid=True)
    tracker.record_raw_received(1.01)
    tracker.record_raw_accepted()
    tracker.record_output(1.01, covariance_valid=True)
    healthy = by_name(
        tracker.snapshot(
            1.01,
            bias=bias(True),
            transform_calibrated=True,
            covariance_calibrated=True,
        )
    )
    assert healthy["D455 IMU/Raw Input"].level == OK
    assert healthy["D455 IMU/Processed Output"].level == OK
    assert healthy["D455 IMU/Transform"].level == OK
    assert healthy["D455 IMU/Bias"].level == OK
    assert healthy["D455 IMU/Covariance"].level == OK

    stale = by_name(
        tracker.snapshot(
            2.0,
            bias=bias(True),
            transform_calibrated=True,
            covariance_calibrated=True,
        )
    )
    assert stale["D455 IMU/Raw Input"].level == ERROR
    assert stale["D455 IMU/Processed Output"].level == ERROR


def test_rejection_dropout_and_uncalibrated_covariance_are_visible():
    tracker = ProcessorHealthTracker(
        HealthConfig(
            stale_timeout_s=1.0,
            minimum_output_rate_hz=0.0,
        )
    )
    tracker.record_raw_received(1.0)
    tracker.record_rejection("nonfinite_sample")
    tracker.record_dropout()
    tracker.record_output(1.0, covariance_valid=True)

    statuses = by_name(
        tracker.snapshot(
            1.0,
            bias=bias(True),
            transform_calibrated=False,
            covariance_calibrated=False,
        )
    )

    assert statuses["D455 IMU/Raw Input"].level == WARN
    assert (
        statuses["D455 IMU/Raw Input"].values["rejected_nonfinite_sample"]
        == "1"
    )
    assert statuses["D455 IMU/Raw Input"].values["dropout_count"] == "1"
    assert statuses["D455 IMU/Transform"].level == WARN
    assert statuses["D455 IMU/Covariance"].level == WARN
