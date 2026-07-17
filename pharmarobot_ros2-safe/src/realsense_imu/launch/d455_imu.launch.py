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
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from realsense_imu.launch_support import camera_launch_arguments
from realsense_imu.launch_support import normalize_serial_number
from realsense_imu.launch_support import realsense_launch_path


def _launch_setup(context):
    serial_number = normalize_serial_number(
        LaunchConfiguration("serial_number").perform(context)
    )
    frame_id = LaunchConfiguration("frame_id")
    topic_name = LaunchConfiguration("topic_name")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_path()),
            launch_arguments=camera_launch_arguments(serial_number).items(),
        ),
        Node(
            package="realsense_imu",
            executable="imu_relay",
            name="realsense_imu_relay",
            output="screen",
            parameters=[
                {
                    "input_topic": "/realsense/d455/imu",
                    "output_topic": topic_name,
                    "frame_id": frame_id,
                }
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Create the IMU-only RealSense and relay launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_number",
                default_value="151223061922",
                description="D455 serial number selected by librealsense2",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="d455_gyro_optical_frame",
                description="Frame ID assigned to the combined IMU message",
            ),
            DeclareLaunchArgument(
                "topic_name",
                default_value="/camera/imu",
                description="Published sensor_msgs/msg/Imu topic",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
