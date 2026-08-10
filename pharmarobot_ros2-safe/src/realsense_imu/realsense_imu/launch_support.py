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

"""Testable launch configuration helpers for the RealSense wrapper."""

from pathlib import Path
from typing import Callable, Dict

from ament_index_python.packages import get_package_share_directory


DEFAULT_SERIAL_NUMBER = "146222250608"
UPSTREAM_IMU_TOPIC = "/realsense/d455/imu"
DEFAULT_RAW_TOPIC = "/imu/d455/data_raw"
# The official wrapper's combined gyro/accelerometer topic uses this frame.
# Keep the raw interface faithful to that live upstream contract; the processor
# applies its explicit rotation only when publishing /imu/data.
DEFAULT_EXPECTED_FRAME_ID = "d455_imu_optical_frame"


def normalize_serial_number(serial_number: str) -> str:
    """Apply the RealSense wrapper's prefix for digit-only serial strings."""
    normalized = serial_number.strip().lstrip("_")
    if not normalized:
        raise ValueError("serial_number must not be empty")
    if not normalized.isdigit():
        raise ValueError("serial_number must contain only digits")
    return f"_{normalized}"


def camera_launch_arguments(serial_number: object) -> Dict[str, object]:
    """Return an IMU-only configuration for the upstream RealSense wrapper."""
    return {
        "camera_namespace": "realsense",
        "camera_name": "d455",
        "serial_no": serial_number,
        "enable_color": "false",
        "enable_depth": "false",
        "enable_infra": "false",
        "enable_infra1": "false",
        "enable_infra2": "false",
        "enable_rgbd": "false",
        "pointcloud.enable": "false",
        "align_depth.enable": "false",
        "enable_gyro": "true",
        "enable_accel": "true",
        "unite_imu_method": "2",
        "angular_velocity_cov": "0.01",
        "linear_accel_cov": "0.01",
        "publish_tf": "false",
        "wait_for_device_timeout": "5.0",
    }


def realsense_launch_path(
    package_resolver: Callable[[str], str] = get_package_share_directory,
) -> str:
    """Resolve the upstream launch file with a useful dependency error."""
    try:
        package_share = Path(package_resolver("realsense2_camera"))
    except Exception as exc:
        raise RuntimeError(
            "realsense2_camera is unavailable; install the ROS 2 Humble "
            "RealSense wrapper and librealsense2 before launching"
        ) from exc

    launch_file = package_share / "launch" / "rs_launch.py"
    if not launch_file.is_file():
        raise RuntimeError(
            f"realsense2_camera launch file is missing: {launch_file}"
        )
    return str(launch_file)
