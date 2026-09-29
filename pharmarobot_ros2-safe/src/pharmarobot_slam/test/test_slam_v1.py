from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml

sys.path.insert(0, str(Path(__file__).parents[1] / "tools"))
from validate_slam_v1 import EXPECTED, validate  # noqa: E402


PACKAGE = Path(__file__).parents[1]
LIDAR_LAUNCH = PACKAGE.parent / "teleop_pharma" / "launch" / "lidar_only.launch.xml"
ODOM_CONFIG = PACKAGE.parent / "roboteq_ros2_driver" / "config" / "roboteq.yaml"


def test_static_contract_passes():
    assert validate(PACKAGE) == []


def test_config_has_expected_frames_and_absolute_scan_topic():
    document = yaml.safe_load(
        (PACKAGE / "config" / "slam_toolbox_v1.yaml").read_text())
    parameters = document["slam_toolbox"]["ros__parameters"]
    for key in ("map_frame", "odom_frame", "base_frame", "scan_topic"):
        assert parameters[key] == EXPECTED[key]
    assert parameters["scan_topic"].startswith("/")
    assert parameters["min_laser_range"] == 0.1


def test_production_odometry_owns_odom_to_base_footprint():
    document = yaml.safe_load(ODOM_CONFIG.read_text())
    parameters = document["roboteq_ros2_driver"]["ros__parameters"]
    assert parameters["pub_odom_tf"] is True
    assert parameters["odom_frame"] == "odom"
    assert parameters["base_frame"] == "base_footprint"


def test_manifest_declares_slam_toolbox_without_motion_stack():
    root = ET.parse(PACKAGE / "package.xml").getroot()
    dependencies = {node.text for node in root if node.tag.endswith("depend")}
    assert "slam_toolbox" in dependencies
    assert not {"nav2", "robot_localization", "realsense2_camera"} & dependencies


def test_launch_only_owns_map_to_odom():
    launch = (PACKAGE / "launch" / "slam_v1.launch.py").read_text()
    assert 'package="slam_toolbox"' in launch
    assert "async_slam_toolbox_node" in launch
    assert "static_transform" not in launch
    assert "cmd_vel" not in launch


def test_rviz_has_required_displays_and_no_graph_path_type_mismatch():
    rviz = (PACKAGE / "rviz" / "slam_v1.rviz").read_text()
    for display in ("TF", "RobotModel", "LaserScan", "Odometry", "Map"):
        assert display in rviz
    assert "Fixed Frame: map" in rviz
    assert "Topic: /map" in rviz
    assert "Topic: /front_scan" in rviz
    assert "Topic: /odom" in rviz
    assert "/back_scan" not in rviz
    assert "graph_visualization" not in rviz


def test_rviz_launch_uses_installed_v1_config_without_starting_slam():
    launch = (PACKAGE / "launch" / "rviz_v1.launch.py").read_text()
    assert 'package="rviz2"' in launch
    assert 'executable="rviz2"' in launch
    assert '"rviz" / "slam_v1.rviz"' in launch
    assert "slam_toolbox" not in launch


def test_live_readiness_checker_is_read_only_and_has_required_contracts():
    checker = (PACKAGE / "tools" / "slam_v1_readiness.py").read_text()
    assert "LaserScan" in checker
    assert all(frame in checker for frame in (
        "odom", "base_footprint", "base_link", "front_laser"))
    assert 'can_transform("map", "odom"' in checker
    assert "freshness_s" in checker
    assert "get_publishers_info_by_topic" in checker
    assert "TFMessage" in checker
    assert "dynamic_tf_edges" in checker
    assert "static_tf_edges" in checker
    assert "complete bounded window" in checker
    assert '("odom", "base_footprint")' in checker
    assert '("base_footprint", "base_link")' in checker
    assert '("base_link", "front_laser")' in checker
    assert '("base_link", "back_laser")' in checker
    assert "cmd_vel" not in checker
    assert "publish(" not in checker


def test_manual_trial_recorder_is_read_only_and_captures_required_contract():
    recorder = (PACKAGE / "tools" / "slam_v1_trial_recorder.py").read_text()
    stop = (PACKAGE / "tools" / "slam_v1_trial_stop.py").read_text()
    for topic in ("/front_scan", "/odom", "/tf", "/tf_static", "/map",
                  "/cmd_vel", "/cmd_vel/safe"):
        assert topic in recorder
    assert '"ros2", "bag", "record"' in recorder
    assert "slam_v1.launch.py" in recorder
    assert "duplicate publishers" in recorder
    assert "required topic disappeared" in recorder
    assert "os.killpg" in recorder
    assert "publish(" not in recorder
    assert "No active SLAM v1 trial" in stop


def test_lidar_launch_has_one_exact_footprint_transform_and_preserves_lasers():
    root = ET.parse(LIDAR_LAUNCH).getroot()
    static_nodes = [node for node in root.findall("node")
                    if node.get("pkg") == "tf2_ros"
                    and node.get("exec") == "static_transform_publisher"]
    assert len(static_nodes) == 3
    assert {(node.get("name"), node.get("args")) for node in static_nodes} == {
                ("footprint_to_base_link", "0 0 0.042 0 0 0 base_footprint base_link"),
                ("base_to_front_laser", "0.41 0 0.29 3.155 0 0 base_link front_laser"),
                ("base_to_back_laser", "-0.40 0 0.29 0 0 0 base_link back_laser"),
            }


def test_architecture_uses_confirmed_production_sllidar_contract():
    architecture = (PACKAGE / "ARCHITECTURE.md").read_text().lower()
    readme = (PACKAGE / "README.md").read_text().lower()
    assert "production uses the repository's sllidar" in architecture
    assert "/front_scan" in architecture and "front_laser" in architecture
    assert "/back_scan" in architecture and "back_laser" in architecture
    assert "no scan merger" in architecture
    assert "no sick dependency" in readme
    assert "blocked until the production lidar" not in readme
