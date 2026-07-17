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

"""Validation and message preparation for the RealSense IMU relay."""

from dataclasses import dataclass
from typing import Callable

from sensor_msgs.msg import Imu


@dataclass(frozen=True)
class RelayConfig:
    """Validated relay configuration."""

    input_topic: str
    output_topic: str
    frame_id: str


def validate_relay_config(
    input_topic: str,
    output_topic: str,
    frame_id: str,
    *,
    topic_resolver: Callable[[str], str],
) -> RelayConfig:
    """Reject empty names and configurations that resolve to a loop."""
    values = {
        "input_topic": input_topic.strip(),
        "output_topic": output_topic.strip(),
        "frame_id": frame_id.strip(),
    }
    for name, value in values.items():
        if not value:
            raise ValueError(f"{name} must not be empty")

    resolved_input = topic_resolver(values["input_topic"])
    resolved_output = topic_resolver(values["output_topic"])
    if resolved_input == resolved_output:
        raise ValueError("input_topic and output_topic must be different")

    return RelayConfig(**values)


def prepare_imu_message(source: Imu, frame_id: str) -> Imu:
    """Copy measured fields and explicitly mark orientation unavailable."""
    target = Imu()
    target.header.stamp = source.header.stamp
    target.header.frame_id = frame_id

    target.orientation_covariance[0] = -1.0

    target.angular_velocity.x = source.angular_velocity.x
    target.angular_velocity.y = source.angular_velocity.y
    target.angular_velocity.z = source.angular_velocity.z
    target.angular_velocity_covariance = list(
        source.angular_velocity_covariance
    )

    target.linear_acceleration.x = source.linear_acceleration.x
    target.linear_acceleration.y = source.linear_acceleration.y
    target.linear_acceleration.z = source.linear_acceleration.z
    target.linear_acceleration_covariance = list(
        source.linear_acceleration_covariance
    )
    return target
