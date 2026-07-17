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

from pathlib import Path

import pytest

from realsense_imu.launch_support import camera_launch_arguments
from realsense_imu.launch_support import normalize_serial_number
from realsense_imu.launch_support import realsense_launch_path


@pytest.mark.parametrize(
    "value", ["151223061922", "_151223061922", " 151223061922 "]
)
def test_normalize_serial_number_applies_wrapper_prefix(value):
    assert normalize_serial_number(value) == "_151223061922"


@pytest.mark.parametrize("value", ["", "_", "not-a-serial"])
def test_normalize_serial_number_rejects_invalid_values(value):
    with pytest.raises(ValueError, match="serial_number"):
        normalize_serial_number(value)


def test_camera_launch_arguments_enable_only_motion_streams():
    arguments = camera_launch_arguments("_151223061922")

    assert arguments["serial_no"] == "_151223061922"
    assert arguments["enable_gyro"] == "true"
    assert arguments["enable_accel"] == "true"
    assert arguments["unite_imu_method"] == "2"
    assert arguments["angular_velocity_cov"] == "0.01"
    assert arguments["linear_accel_cov"] == "0.01"
    assert arguments["publish_tf"] == "false"
    assert arguments["wait_for_device_timeout"] == "5.0"
    for stream in (
        "enable_color",
        "enable_depth",
        "enable_infra",
        "enable_infra1",
        "enable_infra2",
        "enable_rgbd",
        "pointcloud.enable",
        "align_depth.enable",
    ):
        assert arguments[stream] == "false"


def test_realsense_launch_path_reports_missing_package():
    def missing_package(_package_name):
        raise LookupError("not installed")

    with pytest.raises(RuntimeError, match="realsense2_camera is unavailable"):
        realsense_launch_path(missing_package)


def test_realsense_launch_path_reports_missing_launch_file(tmp_path):
    with pytest.raises(RuntimeError, match="launch file is missing"):
        realsense_launch_path(lambda _package_name: str(tmp_path))


def test_realsense_launch_path_returns_existing_launch_file(tmp_path):
    launch_directory = tmp_path / "launch"
    launch_directory.mkdir()
    launch_file = launch_directory / "rs_launch.py"
    launch_file.touch()

    result = realsense_launch_path(lambda _package_name: str(tmp_path))

    assert result == str(Path(launch_file))
