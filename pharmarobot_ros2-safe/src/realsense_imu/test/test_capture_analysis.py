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

from realsense_imu.capture_analysis import analyze_capture
from realsense_imu.capture_analysis import CaptureAnalysisError
from realsense_imu.capture_analysis import ImuSample
from realsense_imu.capture_analysis import parse_imu_yaml


def samples_with_yaw(yaw_norm, *, stationary_norm=0.01):
    """Return samples in all three required windows at one-second cadence."""
    values = []
    for timestamp in range(10):
        norm = yaw_norm if 3 <= timestamp < 7 else stationary_norm
        values.append(ImuSample(float(timestamp), (0.0, 0.0, norm)))
    return tuple(values)


def test_analysis_proves_yaw_only_when_it_exceeds_both_baselines():
    analysis = analyze_capture(samples_with_yaw(0.10))

    assert analysis.yaw_proven
    assert [window.count for window in analysis.windows] == [3, 4, 3]
    assert analysis.windows[1].mean_angular_norm == pytest.approx(0.10)


def test_analysis_rejects_stationary_noise_as_yaw():
    analysis = analyze_capture(samples_with_yaw(0.011))

    assert not analysis.yaw_proven


def test_analysis_rejects_an_all_zero_capture_as_yaw():
    analysis = analyze_capture(samples_with_yaw(0.0, stationary_norm=0.0))

    assert not analysis.yaw_proven


def test_analysis_requires_every_stationary_yaw_stationary_window():
    with pytest.raises(CaptureAnalysisError, match="stationary_post"):
        analyze_capture(samples_with_yaw(0.10)[:7])


def test_yaml_parser_rejects_missing_gyro_and_non_increasing_timestamps():
    with pytest.raises(CaptureAnalysisError, match="lacks stamp or gyro"):
        parse_imu_yaml("header:\n  stamp:\n    sec: 1\n    nanosec: 0\n")

    content = """header:
  stamp:
    sec: 1
    nanosec: 0
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0
---
header:
  stamp:
    sec: 1
    nanosec: 0
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0
"""
    with pytest.raises(CaptureAnalysisError, match="not strictly increasing"):
        parse_imu_yaml(content)
