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

"""Offline checks for the production-container D455 integration."""

import ast
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


def read(relative_path):
    """Read one repository file used by the deployment contract."""
    return (REPOSITORY_ROOT / relative_path).read_text(encoding="utf-8")


def constant_assignments(relative_path):
    """Return simple top-level constants without importing ROS dependencies."""
    tree = ast.parse(read(relative_path))
    constants = {}
    for statement in tree.body:
        if (
            isinstance(statement, ast.Assign)
            and len(statement.targets) == 1
            and isinstance(statement.targets[0], ast.Name)
            and isinstance(statement.value, ast.Constant)
        ):
            constants[statement.targets[0].id] = statement.value.value
    return constants


def camera_argument_literals():
    """Extract literal wrapper arguments without importing launch modules."""
    tree = ast.parse(
        read("src/realsense_imu/realsense_imu/launch_support.py")
    )
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "camera_launch_arguments"
    )
    returned = next(
        node.value
        for node in function.body
        if isinstance(node, ast.Return)
    )
    return {
        key.value: value.value
        for key, value in zip(returned.keys, returned.values)
        if isinstance(key, ast.Constant)
        and isinstance(value, ast.Constant)
    }


def test_raw_interface_and_orientation_contract_need_no_ros_imports():
    constants = constant_assignments(
        "src/realsense_imu/realsense_imu/launch_support.py"
    )
    message_source = read(
        "src/realsense_imu/realsense_imu/imu_message.py"
    )

    assert constants["DEFAULT_RAW_TOPIC"] == "/imu/d455/data_raw"
    assert (
        constants["DEFAULT_EXPECTED_FRAME_ID"]
        == "d455_imu_optical_frame"
    )
    assert constants["DEFAULT_EXPECTED_FRAME_ID"] != "base_link"
    assert "target.header.frame_id = source_frame" in message_source
    assert "target.orientation_covariance[0] = -1.0" in message_source


def test_wrapper_configuration_is_strictly_imu_only_without_ros_imports():
    arguments = camera_argument_literals()

    assert arguments["enable_gyro"] == "true"
    assert arguments["enable_accel"] == "true"
    assert arguments["unite_imu_method"] == "2"
    for stream in (
        "enable_color",
        "enable_depth",
        "enable_infra",
        "enable_infra1",
        "enable_infra2",
        "enable_rgbd",
        "pointcloud.enable",
        "align_depth.enable",
    ):
        assert arguments[stream] == "false"


def test_sensor_image_builds_realsense_imu_separately_from_main():
    main_dockerfile = read("Dockerfile")
    sensor_dockerfile = read(
        "deployment/docker/Dockerfile.d455_sensor"
    )
    start_script = read("deployment/scripts/pharma_start_container.sh")
    build_script = read("deployment/scripts/build_core.sh")

    assert "ros-humble-realsense2-camera" in sensor_dockerfile
    assert "ros-humble-librealsense2" in sensor_dockerfile
    assert "COPY src/realsense_imu" in sensor_dockerfile
    for content in (main_dockerfile, build_script):
        assert "realsense_imu" not in content
    assert start_script.count("realsense_imu") == 1
    assert "existing main container has legacy D455" in start_script


def test_robot_sensor_launch_is_installed_and_used_by_sensor_service():
    setup = read("src/realsense_imu/setup.py")
    entrypoint = read("deployment/scripts/d455_sensor_entrypoint.sh")
    service = read("deployment/systemd/pharma-d455-imu.service")

    assert '"launch/robot_sensors.launch.py"' in setup
    assert "realsense_imu robot_sensors.launch.py" in entrypoint
    assert "pharma_d455_sensor_container.sh run" in service
    assert "pharma_d455_sensor_container.sh stop" in service


def test_sensor_service_is_independent_from_control_service():
    sensor_service = read("deployment/systemd/pharma-d455-imu.service")
    control_service = read("deployment/systemd/pharma-minimal-nodes.service")

    assert "Requires=docker.service" in sensor_service
    assert "pharmarobot.service" not in sensor_service
    assert "pharma-minimal-nodes.service" not in sensor_service
    assert "pharma_run_control.sh" not in sensor_service
    assert "pharma-d455-imu.service" not in control_service
    assert "pharma_run_sensors.sh" not in control_service


def test_main_container_has_no_d455_selector_or_hardware_access():
    start_script = read("deployment/scripts/pharma_start_container.sh")

    assert "--docker-device-args" not in start_script
    assert "configure_d455_resources" not in start_script
    assert "D455_IMU_AVAILABLE" in start_script  # migration marker only
    assert start_script.count("D455_IMU_AVAILABLE") == 1
    for forbidden in (
        "--privileged",
        "apparmor=unconfined",
        "src=/dev,dst=/dev",
        "src=/dev/bus/usb,dst=/dev/bus/usb",
        "--device=/dev/bus/usb:/dev/bus/usb",
        "src=/sys,dst=/sys",
    ):
        assert forbidden not in start_script


def test_legacy_main_container_requires_separate_operator_migration():
    start_script = read("deployment/scripts/pharma_start_container.sh")

    assert "PHARMA_MAIN_D455_MIGRATION_APPROVED" not in start_script
    assert "migration-check" in start_script
    assert "exit 78" in start_script
    assert "separate approval" in start_script
    assert start_script.index("existing_inspect=") < start_script.index(
        'docker rm -f "$CONTAINER"'
    )
