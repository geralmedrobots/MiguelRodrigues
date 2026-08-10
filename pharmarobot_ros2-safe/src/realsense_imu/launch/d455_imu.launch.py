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

"""Launch the D455 gyro and accelerometer streams without image streams."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from realsense_imu.launch_support import camera_launch_arguments
from realsense_imu.launch_support import DEFAULT_EXPECTED_FRAME_ID
from realsense_imu.launch_support import DEFAULT_RAW_TOPIC
from realsense_imu.launch_support import DEFAULT_SERIAL_NUMBER
from realsense_imu.launch_support import normalize_serial_number
from realsense_imu.launch_support import realsense_launch_path
from realsense_imu.launch_support import UPSTREAM_IMU_TOPIC


def _launch_setup(context):
    serial_number = normalize_serial_number(
        LaunchConfiguration("serial_number").perform(context)
    )
    expected_frame_id = LaunchConfiguration("expected_frame_id").perform(
        context
    )
    topic_name = LaunchConfiguration("topic_name").perform(context)

    return [
        # rs_launch.py inspects inherited launch configurations as wrapper
        # parameters.  Do not let package-local arguments leak into it.
        GroupAction(
            scoped=True,
            forwarding=False,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(realsense_launch_path()),
                    launch_arguments=camera_launch_arguments(
                        serial_number
                    ).items(),
                ),
                Node(
                    package="realsense_imu",
                    executable="imu_relay",
                    name="realsense_imu_relay",
                    output="screen",
                    parameters=[
                        {
                            "input_topic": UPSTREAM_IMU_TOPIC,
                            "output_topic": topic_name,
                            "expected_frame_id": expected_frame_id,
                        }
                    ],
                ),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Create the IMU-only RealSense and relay launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_number",
                default_value=DEFAULT_SERIAL_NUMBER,
                description="D455 serial number selected by librealsense2",
            ),
            DeclareLaunchArgument(
                "expected_frame_id",
                default_value=DEFAULT_EXPECTED_FRAME_ID,
                description=(
                    "Required upstream D455 frame; samples with another frame "
                    "are rejected rather than relabeled"
                ),
            ),
            DeclareLaunchArgument(
                "topic_name",
                default_value=DEFAULT_RAW_TOPIC,
                description="Published raw sensor_msgs/msg/Imu topic",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
