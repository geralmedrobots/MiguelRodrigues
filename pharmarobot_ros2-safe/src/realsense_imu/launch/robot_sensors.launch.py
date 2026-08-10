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

"""Launch independently supervised sensors for the normal robot runtime."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from realsense_imu.launch_support import DEFAULT_EXPECTED_FRAME_ID
from realsense_imu.launch_support import DEFAULT_RAW_TOPIC
from realsense_imu.launch_support import DEFAULT_SERIAL_NUMBER


def generate_launch_description() -> LaunchDescription:
    """Create the normal-runtime raw D455 IMU launch description."""
    package_share = Path(get_package_share_directory("realsense_imu"))
    d455_launch = package_share / "launch" / "d455_imu.launch.py"
    processor_config = package_share / "config" / "d455_imu_processor.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_number",
                default_value=DEFAULT_SERIAL_NUMBER,
                description="D455 serial number selected by librealsense2",
            ),
            DeclareLaunchArgument(
                "raw_topic",
                default_value=DEFAULT_RAW_TOPIC,
                description="Stable raw D455 sensor_msgs/msg/Imu topic",
            ),
            DeclareLaunchArgument(
                "expected_frame_id",
                default_value=DEFAULT_EXPECTED_FRAME_ID,
                description="Required unmodified upstream D455 frame",
            ),
            DeclareLaunchArgument(
                "processor_config",
                default_value=str(processor_config),
                description="D455 IMU processor parameter file",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(d455_launch)),
                launch_arguments={
                    "serial_number": LaunchConfiguration("serial_number"),
                    "topic_name": LaunchConfiguration("raw_topic"),
                    "expected_frame_id": LaunchConfiguration(
                        "expected_frame_id"
                    ),
                }.items(),
            ),
            Node(
                package="realsense_imu",
                executable="d455_imu_processor",
                name="d455_imu_processor",
                output="screen",
                parameters=[LaunchConfiguration("processor_config")],
            ),
        ]
    )
