#!/usr/bin/env python3
"""Read-only live SLAM v1 readiness check; never publishes commands or TF."""

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from tf2_ros import TransformListener
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


EXPECTED_BASE_OFFSET = (0.0, 0.0, 0.042)
TRANSFORM_TOLERANCE_M = 1e-6


class ReadinessNode(Node):
    def __init__(self, scan_topic: str, scan_frame: str, timeout_s: float,
                 freshness_s: float):
        super().__init__("pharmarobot_slam_readiness")
        self.scan_topic = scan_topic
        self.scan_frame = scan_frame
        self.timeout_s = timeout_s
        self.freshness_s = freshness_s
        self.scan_ok = False
        self.scan_error = "no LaserScan received"
        self.odom_ok = False
        self.odom_error = "no Odometry received"
        self.dynamic_tf_edges = set()
        self.static_tf_edges = set()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_subscription(LaserScan, scan_topic, self._scan_callback, 10)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.create_subscription(TFMessage, "/tf", self._tf_callback, 100)
        static_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            TFMessage, "/tf_static", self._tf_static_callback, static_qos)

    def _age_s(self, stamp) -> float:
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        return (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000_000.0

    def _scan_callback(self, message: LaserScan) -> None:
        stamp = message.header.stamp
        values = (message.angle_min, message.angle_max, message.angle_increment,
                  message.range_min, message.range_max)
        if message.header.frame_id != self.scan_frame:
            self.scan_error = f"scan frame is {message.header.frame_id!r}"
        elif stamp.sec == 0 and stamp.nanosec == 0:
            self.scan_error = "scan timestamp is zero"
        elif not -0.1 <= self._age_s(stamp) <= self.freshness_s:
            self.scan_error = f"scan timestamp is stale: age={self._age_s(stamp):.3f}s"
        elif not all(math.isfinite(value) for value in values):
            self.scan_error = "scan angle/range limits contain non-finite values"
        elif message.angle_increment == 0.0 or message.range_min <= 0.0:
            self.scan_error = "scan angle increment or range_min is invalid"
        elif message.range_max <= message.range_min:
            self.scan_error = "scan range_max must exceed range_min"
        else:
            self.scan_ok = True
            self.scan_error = ""

    def _odom_callback(self, message: Odometry) -> None:
        stamp = message.header.stamp
        if message.header.frame_id != "odom":
            self.odom_error = f"odometry frame is {message.header.frame_id!r}"
        elif message.child_frame_id != "base_footprint":
            self.odom_error = (
                f"odometry child frame is {message.child_frame_id!r}; "
                "expected 'base_footprint'")
        elif stamp.sec == 0 and stamp.nanosec == 0:
            self.odom_error = "odometry timestamp is zero"
        elif not -0.1 <= self._age_s(stamp) <= self.freshness_s:
            self.odom_error = f"odometry timestamp is stale: age={self._age_s(stamp):.3f}s"
        else:
            self.odom_ok = True
            self.odom_error = ""

    def _tf_callback(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self.dynamic_tf_edges.add(
                (transform.header.frame_id, transform.child_frame_id))

    def _tf_static_callback(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self.static_tf_edges.add(
                (transform.header.frame_id, transform.child_frame_id))


def _topic_type(node: Node, topic: str):
    return {name: types for name, types in node.get_topic_names_and_types()}.get(topic, [])


def _publisher_nodes(node: Node, topic: str):
    names = []
    for endpoint in node.get_publishers_info_by_topic(topic):
        namespace = endpoint.node_namespace.rstrip("/")
        names.append(f"{namespace}/{endpoint.node_name}" if namespace else
                     f"/{endpoint.node_name}")
    return names


def run(timeout_s: float, freshness_s: float) -> int:
    rclpy.init()
    node = ReadinessNode("/front_scan", "front_laser", timeout_s, freshness_s)
    deadline = time.monotonic() + timeout_s
    errors = []
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            # Observe for the complete bounded window so late-discovered
            # publishers or conflicting frame pairs are included.
        if "sensor_msgs/msg/LaserScan" not in _topic_type(node, "/front_scan"):
            errors.append("/front_scan is not advertised as sensor_msgs/msg/LaserScan")
        if "nav_msgs/msg/Odometry" not in _topic_type(node, "/odom"):
            errors.append("/odom is not advertised as nav_msgs/msg/Odometry")
        if not node.scan_ok:
            errors.append(node.scan_error)
        if not node.odom_ok:
            errors.append(node.odom_error)
        for parent, child in (("odom", "base_footprint"),
                              ("base_footprint", "base_link"),
                              ("base_link", "front_laser")):
            if not node.tf_buffer.can_transform(parent, child, Time()):
                errors.append(f"missing TF {parent} -> {child}")
        if node.tf_buffer.can_transform("base_footprint", "base_link", Time()):
            transform = node.tf_buffer.lookup_transform(
                "base_footprint", "base_link", Time()).transform
            actual = (transform.translation.x, transform.translation.y,
                      transform.translation.z)
            if any(abs(value - expected) > TRANSFORM_TOLERANCE_M
                   for value, expected in zip(actual, EXPECTED_BASE_OFFSET)):
                errors.append(
                    "base_footprint -> base_link translation is "
                    f"{actual!r}; expected {EXPECTED_BASE_OFFSET!r}")
            rotation = transform.rotation
            if (abs(rotation.x) > TRANSFORM_TOLERANCE_M or
                    abs(rotation.y) > TRANSFORM_TOLERANCE_M or
                    abs(rotation.z) > TRANSFORM_TOLERANCE_M or
                    abs(rotation.w - 1.0) > TRANSFORM_TOLERANCE_M):
                errors.append("base_footprint -> base_link rotation is not identity")
        if node.tf_buffer.can_transform("odom", "base_footprint", Time()):
            dynamic_transform = node.tf_buffer.lookup_transform(
                "odom", "base_footprint", Time())
            stamp = dynamic_transform.header.stamp
            if stamp.sec == 0 and stamp.nanosec == 0:
                errors.append("odom -> base_footprint TF timestamp is zero")
            else:
                tf_age_s = node._age_s(stamp)
                if not -0.1 <= tf_age_s <= freshness_s:
                    errors.append(
                        "odom -> base_footprint TF timestamp is stale: "
                        f"age={tf_age_s:.3f}s")
        if node.tf_buffer.can_transform("map", "odom", Time()):
            errors.append("map -> odom already exists; slam_toolbox ownership is not exclusive")
        dynamic_publishers = _publisher_nodes(node, "/tf")
        static_publishers = _publisher_nodes(node, "/tf_static")
        if dynamic_publishers != ["/roboteq_ros2_driver"]:
            errors.append(f"unexpected /tf publishers: {sorted(dynamic_publishers)!r}")
        expected_static = ["/footprint_to_base_link", "/base_to_front_laser",
                           "/base_to_back_laser"]
        if sorted(static_publishers) != sorted(expected_static):
            errors.append(f"unexpected /tf_static publishers: {sorted(static_publishers)!r}")
        expected_dynamic_edges = {("odom", "base_footprint")}
        if node.dynamic_tf_edges != expected_dynamic_edges:
            errors.append(
                "unexpected /tf frame pairs: "
                f"{sorted(node.dynamic_tf_edges)!r}; expected "
                f"{sorted(expected_dynamic_edges)!r}")
        expected_static_edges = {
            ("base_footprint", "base_link"),
            ("base_link", "front_laser"),
            ("base_link", "back_laser"),
        }
        if node.static_tf_edges != expected_static_edges:
            errors.append(
                "unexpected /tf_static frame pairs: "
                f"{sorted(node.static_tf_edges)!r}; expected "
                f"{sorted(expected_static_edges)!r}")
        if errors:
            for error in errors:
                print(f"BLOCKED: {error}", file=sys.stderr)
            return 1
        print("SLAM v1 live readiness: PASS")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-s", type=float, default=5.0)
    parser.add_argument("--freshness-s", type=float, default=1.0)
    args = parser.parse_args()
    return run(args.timeout_s, args.freshness_s)


if __name__ == "__main__":
    raise SystemExit(main())
