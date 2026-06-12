#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


class DeadmanTest(Node):
    def __init__(self) -> None:
        super().__init__('joystick_deadman_integration_test')
        self.publisher = self.create_publisher(Joy, '/joy', 10)
        self.received = []
        self.subscription = self.create_subscription(
            Twist, '/test/cmd_vel/joy', self.on_command, 10)

    def on_command(self, message: Twist) -> None:
        self.received.append((time.monotonic(), message.linear.x, message.angular.z))

    def publish_joy(self, enabled: bool, linear: float, angular: float = 0.0, repeats: int = 5) -> None:
        message = Joy()
        message.axes = [0.0] * 8
        message.axes[1] = linear
        message.axes[3] = angular
        message.axes[5] = 1.0
        message.buttons = [0] * 13
        message.buttons[4] = 1 if enabled else 0

        for _ in range(repeats):
            self.publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.03)

    def wait_for(self, predicate, timeout: float, description: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if any(predicate(item) for item in self.received):
                print(f'PASS: {description}')
                return
        raise RuntimeError(f'FAIL: {description}; received={self.received[-10:]}')


def approximately_zero(value: float) -> bool:
    return math.isfinite(value) and abs(value) < 1e-6


def main() -> None:
    rclpy.init()
    node = DeadmanTest()
    try:
        time.sleep(0.5)

        start = time.monotonic()
        node.publish_joy(enabled=False, linear=1.0)
        node.wait_for(
            lambda item: item[0] >= start and approximately_zero(item[1]) and approximately_zero(item[2]),
            1.0,
            'stick motion is blocked while L1 is released')

        node.received.clear()
        start = time.monotonic()
        node.publish_joy(enabled=True, linear=1.0, repeats=10)
        node.wait_for(
            lambda item: item[0] >= start and item[1] > 0.01,
            1.5,
            'L1 enables a non-zero joystick command')

        node.received.clear()
        start = time.monotonic()
        node.publish_joy(enabled=False, linear=1.0, repeats=2)
        node.wait_for(
            lambda item: item[0] >= start and approximately_zero(item[1]) and approximately_zero(item[2]),
            0.8,
            'releasing L1 sends an immediate stop')

        node.received.clear()
        node.publish_joy(enabled=True, linear=1.0, repeats=8)
        node.received.clear()
        start = time.monotonic()
        node.wait_for(
            lambda item: item[0] >= start and approximately_zero(item[1]) and approximately_zero(item[2]),
            1.0,
            'joystick message loss sends a timeout stop')

        print('PASS: L1 hold-to-run integration test completed')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
