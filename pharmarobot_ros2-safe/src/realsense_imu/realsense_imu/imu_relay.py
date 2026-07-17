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

"""Relay the official RealSense combined IMU topic with stable local naming."""

from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from realsense_imu.imu_message import prepare_imu_message
from realsense_imu.imu_message import validate_relay_config


class ImuRelay(Node):
    """Apply the configured topic and frame ID to upstream IMU samples."""

    def __init__(self) -> None:
        super().__init__("realsense_imu_relay")
        self.declare_parameter("input_topic", "/realsense/d455/imu")
        self.declare_parameter("output_topic", "/camera/imu")
        self.declare_parameter("frame_id", "d455_gyro_optical_frame")

        config = validate_relay_config(
            self.get_parameter("input_topic").value,
            self.get_parameter("output_topic").value,
            self.get_parameter("frame_id").value,
            topic_resolver=self.resolve_topic_name,
        )
        self._frame_id = config.frame_id
        self._publisher = self.create_publisher(
            Imu, config.output_topic, qos_profile_sensor_data
        )
        self._subscription = self.create_subscription(
            Imu,
            config.input_topic,
            self._relay,
            qos_profile_sensor_data,
        )

    def _relay(self, message: Imu) -> None:
        self._publisher.publish(prepare_imu_message(message, self._frame_id))


def main(args: Optional[Sequence[str]] = None) -> None:
    """Run the IMU relay."""
    rclpy.init(args=args)
    node = None
    try:
        node = ImuRelay()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
