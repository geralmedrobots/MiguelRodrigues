#!/usr/bin/env python3
"""Static SLAM v1 contract checker; it never starts ROS or creates transforms."""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from typing import List

import yaml


EXPECTED = {
    "map_frame": "map",
    "odom_frame": "odom",
    "base_frame": "base_footprint",
    "scan_topic": "/front_scan",
    "scan_frame": "front_laser",
}
FORBIDDEN_LAUNCH_TOKENS = (
    "roboteq", "joy", "teleop", "d455", "realsense", "nav2", "ekf", "amcl",
    "cmd_vel", "twist",
)


def validate(package_dir: Path) -> List[str]:
    errors: list[str] = []
    package_xml = package_dir / "package.xml"
    config_path = package_dir / "config" / "slam_toolbox_v1.yaml"
    launch_path = package_dir / "launch" / "slam_v1.launch.py"
    lidar_launch_path = package_dir.parent / "teleop_pharma" / "launch" / "lidar_only.launch.xml"
    odom_config_path = package_dir.parent / "roboteq_ros2_driver" / "config" / "roboteq.yaml"
    architecture_path = package_dir / "ARCHITECTURE.md"
    try:
        ET.parse(package_xml)
    except (ET.ParseError, OSError) as exc:
        errors.append(f"invalid package.xml: {exc}")
    try:
        document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        parameters = document["slam_toolbox"]["ros__parameters"]
    except (KeyError, OSError, TypeError, yaml.YAMLError) as exc:
        errors.append(f"invalid slam_toolbox config: {exc}")
        parameters = {}
    for key, expected in EXPECTED.items():
        if key == "scan_frame":
            continue
        if parameters.get(key) != expected:
            errors.append(f"{key} must be {expected!r}")
    if not str(parameters.get("scan_topic", "")).startswith("/"):
        errors.append("scan_topic must be absolute")
    if parameters.get("min_laser_range") != 0.1:
        errors.append("min_laser_range must be 0.1")
    try:
        odom_document = yaml.safe_load(odom_config_path.read_text(encoding="utf-8"))
        odom_parameters = odom_document["roboteq_ros2_driver"]["ros__parameters"]
    except (KeyError, OSError, TypeError, yaml.YAMLError) as exc:
        errors.append(f"invalid odometry config: {exc}")
        odom_parameters = {}
    if odom_parameters.get("odom_frame") != "odom":
        errors.append("production odometry frame must be 'odom'")
    if odom_parameters.get("base_frame") != "base_footprint":
        errors.append("production odometry child/base frame must be 'base_footprint'")
    if odom_parameters.get("pub_odom_tf") is not True:
        errors.append("production odometry must publish its dynamic TF")
    try:
        launch_text = launch_path.read_text(encoding="utf-8")
    except OSError as exc:
        errors.append(f"cannot read launch: {exc}")
        launch_text = ""
    if 'package="slam_toolbox"' not in launch_text:
        errors.append("launch must start slam_toolbox")
    if "async_slam_toolbox_node" not in launch_text:
        errors.append("launch must use the asynchronous mapper")
    lower_launch = launch_text.lower()
    for token in FORBIDDEN_LAUNCH_TOKENS:
        if token in lower_launch:
            errors.append(f"forbidden motion/runtime token in launch: {token}")
    try:
        lidar_launch = ET.parse(lidar_launch_path).getroot()
    except (ET.ParseError, OSError) as exc:
        errors.append(f"invalid lidar launch: {exc}")
        lidar_launch = None
    if lidar_launch is not None:
        static_nodes = [node for node in lidar_launch.findall("node")
                        if node.get("pkg") == "tf2_ros"
                        and node.get("exec") == "static_transform_publisher"]
        expected_static = {
            ("footprint_to_base_link", "0 0 0.042 0 0 0 base_footprint base_link"),
            ("base_to_front_laser", "0.41 0 0.29 3.155 0 0 base_link front_laser"),
            ("base_to_back_laser", "-0.40 0 0.29 0 0 0 base_link back_laser"),
        }
        actual_static = [(node.get("name"), node.get("args")) for node in static_nodes]
        if len(actual_static) != len(expected_static) or set(actual_static) != expected_static:
            errors.append(
                "lidar launch must contain exactly the three unique, validated static TF publishers")
    try:
        architecture = architecture_path.read_text(encoding="utf-8").lower()
    except OSError as exc:
        errors.append(f"cannot read architecture: {exc}")
        architecture = ""
    for required in ("map -> odom", "odom -> base_footprint -> base_link",
                     "base_footprint -> base_link", "/front_scan",
                     "front_laser", "no scan merger", "nav2", "ekf"):
        if required not in architecture:
            errors.append(f"architecture is missing: {required}")
    return errors


def main() -> int:
    package_dir = Path(__file__).resolve().parents[1]
    errors = validate(package_dir)
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print("SLAM v1 static contract: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
