#!/usr/bin/env python3
"""Record one manual SLAM v1 trial while supervising the live graph.

This process is read-only with respect to ROS topics.  It starts the existing
mapper and rosbag recorder, but never publishes a command, motor message, or
transform.  A violation stops capture and records the reason.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


TOPICS = ("/front_scan", "/odom", "/tf", "/tf_static", "/map",
          "/cmd_vel", "/cmd_vel/safe", "/diagnostics")
REQUIRED_TOPICS = TOPICS[:7]
FRESHNESS_S = {"/front_scan": 2.0, "/odom": 2.0, "/tf": 2.0,
               "/cmd_vel/safe": 0.5, "/map": 15.0}
STATE_FILE = Path("/tmp/pharmarobot_slam_v1_trial.json")


def _stamp_age(message, now_ns):
    stamp = message.header.stamp
    stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return (now_ns - stamp_ns) / 1_000_000_000.0


class Supervisor(Node):
    def __init__(self, output):
        super().__init__("pharmarobot_slam_v1_trial_supervisor")
        self.output = output
        self.latest = {}
        self.counts = {}
        self.started = time.monotonic()
        self.errors = []
        self.create_subscription(LaserScan, "/front_scan",
                                 self._stamped("/front_scan"), 10)
        self.create_subscription(Odometry, "/odom", self._stamped("/odom"), 10)
        self.create_subscription(TFMessage, "/tf", self._tf, 100)
        self.create_subscription(TFMessage, "/tf_static", self._tf_static, 100)
        self.create_subscription(OccupancyGrid, "/map", self._stamped("/map"), 10)
        self.create_subscription(Twist, "/cmd_vel/safe", self._safe, 10)
        self.timer = self.create_timer(0.25, self._check)

    def _stamped(self, topic):
        def callback(message):
            self.counts[topic] = self.counts.get(topic, 0) + 1
            self.latest[topic] = {"age_s": _stamp_age(message, self.get_clock().now().nanoseconds),
                                  "received_monotonic": time.monotonic()}
        return callback

    def _tf(self, message):
        self.counts["/tf"] = self.counts.get("/tf", 0) + 1
        self.latest["/tf"] = {"age_s": min((_stamp_age(t, self.get_clock().now().nanoseconds)
                                               for t in message.transforms), default=999.0),
                               "received_monotonic": time.monotonic()}

    def _tf_static(self, _message):
        self.counts["/tf_static"] = self.counts.get("/tf_static", 0) + 1
        self.latest["/tf_static"] = {"received_monotonic": time.monotonic()}

    def _safe(self, message):
        self.counts["/cmd_vel/safe"] = self.counts.get("/cmd_vel/safe", 0) + 1
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        if not all(isinstance(value, (int, float)) and abs(value) < float("inf")
                   for value in values):
            self.errors.append("/cmd_vel/safe contains non-finite values")
        self.latest["/cmd_vel/safe"] = {"received_monotonic": time.monotonic(),
                                         "valid": not self.errors}

    def _check(self):
        now = time.monotonic()
        for topic, limit in FRESHNESS_S.items():
            sample = self.latest.get(topic)
            if sample is None:
                if time.monotonic() - self.started > 8.0:
                    self.errors.append(f"required topic has no fresh messages: {topic}")
                continue
            if now - sample["received_monotonic"] > limit:
                self.errors.append(f"{topic} became stale")
        for topic in TOPICS:
            endpoints = self.get_publishers_info_by_topic(topic)
            names = sorted({f"{e.node_namespace.rstrip('/')}/{e.node_name}"
                            for e in endpoints})
            if topic in ("/odom", "/front_scan", "/cmd_vel", "/cmd_vel/safe", "/map", "/tf") and len(endpoints) > 1:
                self.errors.append(f"duplicate publishers on {topic}: {names}")
            if topic == "/tf_static" and len(names) > 3:
                self.errors.append(f"duplicate static publishers on {topic}: {names}")
            if time.monotonic() - self.started > 8.0 and topic in REQUIRED_TOPICS and not names:
                self.errors.append(f"required topic disappeared: {topic}")
            self.latest.setdefault(topic, {})["publishers"] = names
        with self.output.open("a") as stream:
            stream.write(json.dumps({"monotonic": now, "counts": self.counts,
                                     "topics": self.latest,
                                     "errors": self.errors[-5:]}) + "\n")


def command(args, path):
    with path.open("w") as stream:
        return subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT,
                                start_new_session=True)


def snapshot(directory, name, command_line):
    path = directory / name
    with path.open("w") as stream:
        try:
            subprocess.run(command_line, stdout=stream, stderr=subprocess.STDOUT,
                           check=False, timeout=20, cwd=directory)
        except subprocess.TimeoutExpired:
            stream.write("\nCOMMAND TIMEOUT after 20 seconds\n")


def graph_snapshot(directory, name):
    path = directory / name
    with path.open("w") as stream:
        for topic in TOPICS:
            stream.write(f"\n### ros2 topic info --verbose {topic}\n")
            try:
                result = subprocess.run(["ros2", "topic", "info", "--verbose", topic],
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, check=False, timeout=20, cwd=directory)
                stream.write(result.stdout)
            except subprocess.TimeoutExpired:
                stream.write("COMMAND TIMEOUT after 20 seconds\n")


def rate_snapshot(directory):
    with (directory / "topic-rates.txt").open("w") as stream:
        for topic in TOPICS:
            stream.write(f"\n### ros2 topic hz {topic} (5 s bound)\n")
            try:
                result = subprocess.run(["timeout", "5", "ros2", "topic", "hz", topic],
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, check=False, timeout=8, cwd=directory)
                stream.write(result.stdout)
            except subprocess.TimeoutExpired:
                stream.write("COMMAND TIMEOUT after 8 seconds\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root", default="slam-v1-trials")
    parser.add_argument("--poll-s", type=float, default=0.5)
    args = parser.parse_args()
    if STATE_FILE.exists():
        print(f"BLOCKED: active trial state exists: {STATE_FILE}", file=sys.stderr)
        return 2
    directory = Path(args.evidence_root) / time.strftime("slam-v1-%Y%m%dT%H%M%SZ", time.gmtime())
    directory.mkdir(parents=True, exist_ok=False)
    manifest = {"status": "recording", "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "topics": list(TOPICS), "manual_motion_only": True,
                "slam_launch": "pharmarobot_slam slam_v1.launch.py", "abort_reason": None}
    (directory / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (directory / "processes-start.txt").write_text(subprocess.run(["ps", "-eo", "pid,ppid,%cpu,%mem,stat,etime,cmd"],
                                                    capture_output=True, text=True).stdout)
    STATE_FILE.write_text(json.dumps({"pid": os.getpid(), "directory": str(directory)}) + "\n")
    bag = slam = None
    supervisor = None
    rclpy_initialized = False
    reason = None
    try:
        snapshot(directory, "tf-tree-start.txt", ["ros2", "run", "tf2_tools", "view_frames"])
        graph_snapshot(directory, "topic-publishers-start.txt")
        bag = command(["ros2", "bag", "record", "-o", str(directory / "rosbag"), "--topics", *TOPICS, "/map_metadata"],
                      directory / "rosbag.stdout-stderr.log")
        slam = command(["ros2", "launch", "pharmarobot_slam", "slam_v1.launch.py"],
                       directory / "slam_toolbox.stdout-stderr.log")
        rclpy.init()
        rclpy_initialized = True
        supervisor = Supervisor(directory / "topic-freshness.jsonl")
        while rclpy.ok():
            rclpy.spin_once(supervisor, timeout_sec=args.poll_s)
            if slam.poll() is not None:
                reason = f"slam_toolbox exited with status {slam.returncode}"
                break
            if bag.poll() is not None:
                reason = f"rosbag recorder exited with status {bag.returncode}"
                break
            if supervisor.errors:
                reason = supervisor.errors[-1]
                break
    except KeyboardInterrupt:
        reason = "operator stop"
    finally:
        if supervisor is not None:
            supervisor.destroy_node()
            if rclpy_initialized:
                rclpy.shutdown()
        for process in (slam, bag):
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in (slam, bag):
            if process is not None:
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
        snapshot(directory, "tf-tree-end.txt", ["ros2", "run", "tf2_tools", "view_frames"])
        graph_snapshot(directory, "topic-publishers-end.txt")
        snapshot(directory, "map-final.txt", ["ros2", "topic", "echo", "/map", "--once"])
        snapshot(directory, "map-metadata-final.txt", ["ros2", "topic", "echo", "/map_metadata", "--once"])
        snapshot(directory, "diagnostics-final.txt", ["ros2", "topic", "echo", "/diagnostics", "--once"])
        rate_snapshot(directory)
        snapshot(directory, "bag-info.txt", ["ros2", "bag", "info", str(directory / "rosbag")])
        (directory / "processes-end.txt").write_text(subprocess.run(["ps", "-eo", "pid,ppid,%cpu,%mem,stat,etime,cmd"],
                                                      capture_output=True, text=True).stdout)
        manifest.update({"status": "aborted" if reason and reason != "operator stop" else "stopped",
                         "ended_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                         "abort_reason": reason})
        (directory / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        STATE_FILE.unlink(missing_ok=True)
    print(directory)
    return 1 if reason and reason != "operator stop" else 0


if __name__ == "__main__":
    raise SystemExit(main())
