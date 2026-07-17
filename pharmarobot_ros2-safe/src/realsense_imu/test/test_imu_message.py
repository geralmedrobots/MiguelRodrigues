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
from sensor_msgs.msg import Imu

from realsense_imu.imu_message import prepare_imu_message
from realsense_imu.imu_message import validate_relay_config


def resolve_root_topic(topic):
    """Resolve a topic as ROS does for a node in the root namespace."""
    return topic if topic.startswith("/") else f"/{topic}"


def test_prepare_imu_message_preserves_timestamp_measurements_and_covariance():
    source = Imu()
    source.header.stamp.sec = 123
    source.header.stamp.nanosec = 456
    source.angular_velocity.x = 1.25
    source.angular_velocity.y = -2.5
    source.angular_velocity.z = 3.75
    source.linear_acceleration.x = 4.5
    source.linear_acceleration.y = -5.5
    source.linear_acceleration.z = 6.5
    source.angular_velocity_covariance = [0.01] * 9
    source.linear_acceleration_covariance = [0.02] * 9

    result = prepare_imu_message(source, "imu_link")

    assert result.header.stamp == source.header.stamp
    assert result.header.frame_id == "imu_link"
    assert result.angular_velocity == source.angular_velocity
    assert result.linear_acceleration == source.linear_acceleration
    assert list(result.angular_velocity_covariance) == [0.01] * 9
    assert list(result.linear_acceleration_covariance) == [0.02] * 9


def test_prepare_imu_message_does_not_claim_orientation():
    result = prepare_imu_message(Imu(), "imu_link")

    assert result.orientation_covariance[0] == -1.0


@pytest.mark.parametrize("field", ["input_topic", "output_topic", "frame_id"])
def test_validate_relay_config_rejects_empty_values(field):
    values = {
        "input_topic": "/raw/imu",
        "output_topic": "/camera/imu",
        "frame_id": "camera_imu_frame",
    }
    values[field] = "  "

    with pytest.raises(ValueError, match=field):
        validate_relay_config(**values, topic_resolver=resolve_root_topic)


def test_validate_relay_config_rejects_absolute_same_topic():
    with pytest.raises(ValueError, match="must be different"):
        validate_relay_config(
            "/camera/imu",
            "/camera/imu",
            "imu_link",
            topic_resolver=resolve_root_topic,
        )


def test_validate_relay_config_rejects_relative_absolute_equivalent_topics():
    with pytest.raises(ValueError, match="must be different"):
        validate_relay_config(
            "realsense/d455/imu",
            "/realsense/d455/imu",
            "imu_link",
            topic_resolver=resolve_root_topic,
        )


@pytest.mark.parametrize("output_topic", ["/camera/imu", "/custom/imu"])
def test_validate_relay_config_accepts_safe_output_topics(output_topic):
    config = validate_relay_config(
        "/realsense/d455/imu",
        output_topic,
        "imu_link",
        topic_resolver=resolve_root_topic,
    )

    assert config.output_topic == output_topic
