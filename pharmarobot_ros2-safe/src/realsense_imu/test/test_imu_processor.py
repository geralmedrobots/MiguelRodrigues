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

"""Focused ROS-message compatibility checks for the IMU processor."""

import pytest

from diagnostic_msgs.msg import DiagnosticStatus

from realsense_imu.imu_processor import diagnostic_level_byte


@pytest.mark.parametrize("level", [0, 1, 2, 255])
def test_diagnostic_level_uses_a_single_uint8_byte(level):
    encoded = diagnostic_level_byte(level)

    assert isinstance(encoded, bytes)
    assert encoded == bytes((level,))
    assert len(encoded) == 1


def test_humble_diagnostic_status_accepts_encoded_level():
    diagnostic = DiagnosticStatus()
    diagnostic.level = diagnostic_level_byte(2)

    assert diagnostic.level == b"\x02"


@pytest.mark.parametrize("level", [-1, 256, True, 1.0])
def test_diagnostic_level_rejects_invalid_uint8_values(level):
    with pytest.raises(ValueError, match="diagnostic level"):
        diagnostic_level_byte(level)
