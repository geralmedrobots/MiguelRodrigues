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

import realsense_imu.imu_relay as imu_relay


def test_main_handles_interrupt_with_idempotent_shutdown(monkeypatch):
    events = []

    class FakeNode:
        def destroy_node(self):
            events.append("destroy_node")

    node = FakeNode()
    monkeypatch.setattr(
        imu_relay.rclpy,
        "init",
        lambda args=None: events.append(("init", args)),
    )
    monkeypatch.setattr(imu_relay, "ImuRelay", lambda: node)

    def interrupt_spin(spun_node):
        assert spun_node is node
        events.append("spin")
        raise KeyboardInterrupt

    monkeypatch.setattr(imu_relay.rclpy, "spin", interrupt_spin)
    monkeypatch.setattr(
        imu_relay.rclpy,
        "try_shutdown",
        lambda: events.append("try_shutdown"),
    )
    monkeypatch.setattr(
        imu_relay.rclpy,
        "shutdown",
        lambda: pytest.fail("non-idempotent shutdown must not be called"),
    )

    imu_relay.main(args=["--test"])

    assert events == [
        ("init", ["--test"]),
        "spin",
        "destroy_node",
        "try_shutdown",
    ]
