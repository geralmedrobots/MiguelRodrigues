#!/usr/bin/env python3
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class ArbiterIntegrationTest(Node):
    def __init__(self) -> None:
        super().__init__("command_arbiter_integration_test")
        self.joy_pub = self.create_publisher(Twist, "/cmd_vel/joy", 10)
        self.test_pub = self.create_publisher(Twist, "/cmd_vel/test", 10)
        self.nav_pub = self.create_publisher(Twist, "/cmd_vel/nav", 10)
        self.last_output = None
        self.create_subscription(Twist, "/cmd_vel/safe", self._output_callback, 10)

    def _output_callback(self, message: Twist) -> None:
        self.last_output = (message.linear.x, message.angular.z)

    def publish_for(self, publishers_and_values, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            for publisher, linear_x, angular_z in publishers_and_values:
                message = Twist()
                message.linear.x = linear_x
                message.angular.z = angular_z
                publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)

    def wait_for_output(self, expected_linear: float, timeout_s: float = 1.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_output is None:
                continue
            linear_x, angular_z = self.last_output
            if math.isclose(linear_x, expected_linear, abs_tol=0.03) and math.isclose(
                angular_z, 0.0, abs_tol=0.03
            ):
                return True
        return False


def assert_stage(test: ArbiterIntegrationTest, stage: str, expected_linear: float) -> None:
    if not test.wait_for_output(expected_linear):
        raise RuntimeError(
            f"{stage}: expected linear.x={expected_linear:.2f}, got {test.last_output!r}"
        )
    print(f"PASS: {stage} -> linear.x={expected_linear:.2f}")


def main() -> int:
    rclpy.init()
    test = ArbiterIntegrationTest()

    try:
        time.sleep(0.5)

        test.publish_for([(test.nav_pub, 0.10, 0.0)], 0.4)
        assert_stage(test, "navigation selected when alone", 0.10)

        test.publish_for(
            [
                (test.nav_pub, 0.10, 0.0),
                (test.test_pub, 0.20, 0.0),
            ],
            0.4,
        )
        assert_stage(test, "test overrides navigation", 0.20)

        test.publish_for(
            [
                (test.nav_pub, 0.10, 0.0),
                (test.test_pub, 0.20, 0.0),
                (test.joy_pub, 0.30, 0.0),
            ],
            0.4,
        )
        assert_stage(test, "joystick overrides test and navigation", 0.30)

        test.publish_for(
            [
                (test.nav_pub, 0.10, 0.0),
                (test.test_pub, 0.20, 0.0),
            ],
            0.5,
        )
        assert_stage(test, "stale joystick releases to test", 0.20)

        test.publish_for([(test.nav_pub, 0.10, 0.0)], 0.5)
        assert_stage(test, "stale test releases to navigation", 0.10)

        deadline = time.monotonic() + 0.7
        while time.monotonic() < deadline:
            rclpy.spin_once(test, timeout_sec=0.05)
        assert_stage(test, "all stale sources produce stop", 0.0)

        print("PASS: command-source arbitration integration test completed")
        return 0
    except Exception as exception:
        print(f"FAIL: {exception}", file=sys.stderr)
        return 1
    finally:
        test.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
