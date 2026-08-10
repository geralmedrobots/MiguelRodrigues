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

"""Static offline checks for processor launch and configuration wiring."""

from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


def read(relative_path):
    return (REPOSITORY_ROOT / relative_path).read_text(encoding="utf-8")


def test_launch_starts_acquisition_and_processor_and_keeps_raw_topic():
    launch = read("src/realsense_imu/launch/robot_sensors.launch.py")
    d455_launch = read("src/realsense_imu/launch/d455_imu.launch.py")
    setup = read("src/realsense_imu/setup.py")

    assert "d455_imu.launch.py" in launch
    assert 'executable="d455_imu_processor"' in launch
    assert 'name="d455_imu_processor"' in launch
    assert "d455_imu_processor.yaml" in launch
    assert "DEFAULT_RAW_TOPIC" in d455_launch
    assert '"d455_imu_processor = realsense_imu.imu_processor:main"' in setup
    assert '"config/d455_imu_processor.yaml"' in setup


def test_processor_config_has_required_topics_frames_and_visible_mapping():
    config = read("src/realsense_imu/config/d455_imu_processor.yaml")
    processor = read("src/realsense_imu/realsense_imu/imu_processor.py")

    assert "input_topic: /imu/d455/data_raw" in config
    assert "output_topic: /imu/data" in config
    assert "expected_raw_frame: d455_imu_optical_frame" in config
    assert "processed_frame: d455_imu_link" in config
    assert "rotation_quaternion_xyzw: [0.5, 0.5, 0.5, 0.5]" in config
    assert "raw +Y -> processed +Z" in config
    assert '"bias.require_cmd_vel_zero": true' in config
    assert "orientation_covariance = list(" in processor
    assert '"bias.require_cmd_vel_zero": True' in processor
    assert '"determinant":' in processor
    assert '"tf_lookup": "not_used"' in processor
    assert '"tf_publication": "not_owned_by_processor"' in processor


def test_ekf_and_robot_localization_remain_disabled():
    launch = read("src/realsense_imu/launch/robot_sensors.launch.py")
    config = read("src/realsense_imu/config/d455_imu_processor.yaml")
    package = read("src/realsense_imu/package.xml")
    combined = "\n".join((launch, config, package))

    assert "robot_localization" not in combined
    assert "ekf_node" not in combined


def test_processor_does_not_publish_motor_commands_or_own_safety_actions():
    processor = read("src/realsense_imu/realsense_imu/imu_processor.py")

    assert "create_publisher(\n            Imu" in processor
    assert "create_publisher(\n            DiagnosticArray" in processor
    assert "create_publisher(\n            Twist" not in processor
    assert '"/cmd_vel/test"' not in processor
    assert '"/cmd_vel/nav"' not in processor
    assert '"/cmd_vel/joy"' not in processor


def test_sensor_stop_path_targets_processor_independently():
    stop_script = read("deployment/scripts/pharma_stop_sensors.sh")
    entrypoint = read("deployment/scripts/d455_sensor_entrypoint.sh")

    assert "pharma_d455_sensor_container.sh stop" in stop_script
    assert "exec ros2 launch" in entrypoint
    assert "pharma-minimal-nodes.service" not in stop_script


def test_upstream_wrapper_launch_is_scoped_from_package_arguments():
    d455_launch = read("src/realsense_imu/launch/d455_imu.launch.py")

    assert "GroupAction(" in d455_launch
    assert "scoped=True" in d455_launch
    assert "forwarding=False" in d455_launch
