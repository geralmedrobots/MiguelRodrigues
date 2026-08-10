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

"""Relay the official RealSense combined IMU without relabeling its frame."""

from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from realsense_imu.imu_message import prepare_imu_message
from realsense_imu.imu_message import validate_relay_config
from realsense_imu.launch_support import DEFAULT_EXPECTED_FRAME_ID
from realsense_imu.launch_support import DEFAULT_RAW_TOPIC
from realsense_imu.launch_support import UPSTREAM_IMU_TOPIC


class ImuRelay(Node):
    """Expose a stable raw topic while preserving the upstream sensor frame."""

    def __init__(self) -> None:
        super().__init__("realsense_imu_relay")
        self.declare_parameter("input_topic", UPSTREAM_IMU_TOPIC)
        self.declare_parameter("output_topic", DEFAULT_RAW_TOPIC)
        self.declare_parameter(
            "expected_frame_id", DEFAULT_EXPECTED_FRAME_ID
        )

        config = validate_relay_config(
            self.get_parameter("input_topic").value,
            self.get_parameter("output_topic").value,
            self.get_parameter("expected_frame_id").value,
            topic_resolver=self.resolve_topic_name,
        )
        self._expected_frame_id = config.expected_frame_id
        self._frame_mismatch_reported = False
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
        try:
            prepared = prepare_imu_message(
                message, self._expected_frame_id
            )
        except ValueError as error:
            if not self._frame_mismatch_reported:
                self.get_logger().error(str(error))
                self._frame_mismatch_reported = True
            return
        self._frame_mismatch_reported = False
        self._publisher.publish(prepared)


def main(args: Optional[Sequence[str]] = None) -> None:
    """Run the IMU relay."""
    rclpy.init(args=args)
    node = None
    try:
        node = ImuRelay()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
