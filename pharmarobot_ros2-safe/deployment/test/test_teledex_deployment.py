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

"""Offline tests for the pharma_container TeleDex deployment seam."""

import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
MOUNT_HELPER = ROOT / "deployment/scripts/teledex_reference_mount.sh"
WORKSPACE_HELPER = ROOT / "deployment/scripts/pharma_workspace_mount.sh"


def _resolve_mount(host_dir, container_dir="/opt/teledex_reference"):
    environment = dict(os.environ)
    environment.update(
        TELEDEX_REFERENCE_ENABLED="1",
        TELEDEX_REFERENCE_HOST_DIR=str(host_dir),
        TELEDEX_REFERENCE_CONTAINER_DIR=container_dir,
    )
    command = (
        "source \"$1\"; args=(); "
        "configure_teledex_reference_mount args || exit $?; "
        "printf '<%s>\\n' \"${args[@]}\"")
    return subprocess.run(
        ["bash", "-c", command, "bash", str(MOUNT_HELPER)],
        env=environment, text=True, capture_output=True, check=False)


def _resolve_workspace(workspace_root):
    environment = dict(os.environ)
    if workspace_root is None:
        environment.pop("PHARMA_WS_DIR", None)
    else:
        environment["PHARMA_WS_DIR"] = str(workspace_root)
    command = (
        "source \"$1\"; args=(); root=''; "
        "configure_pharma_workspace_mounts args root || exit $?; "
        "printf 'ROOT=<%s>\\n' \"$root\"; "
        "printf 'ARG=<%s>\\n' \"${args[@]}\"")
    return subprocess.run(
        ["bash", "-c", command, "bash", str(WORKSPACE_HELPER)],
        env=environment, text=True, capture_output=True, check=False)


def _make_workspace(root):
    (root / "deployment").mkdir(parents=True)
    for package in ("odometry_validation", "command_arbiter", "sllidar_ros2"):
        package_dir = root / "src" / package
        package_dir.mkdir(parents=True)
        (package_dir / "package.xml").write_text(
            "<package/>\n", encoding="utf-8")


def test_mount_rejects_missing_host_directory(tmp_path):
    result = _resolve_mount(tmp_path / "missing")
    assert result.returncode == 66
    assert "host directory is missing" in result.stderr


def test_mount_rejects_missing_logger(tmp_path):
    result = _resolve_mount(tmp_path)
    assert result.returncode == 66
    assert "logger is missing" in result.stderr


def test_mount_rejects_unreadable_logger(tmp_path):
    logger = tmp_path / "teledex_logger.py"
    logger.write_text("# test logger\n", encoding="utf-8")
    logger.chmod(0)
    try:
        result = _resolve_mount(tmp_path)
    finally:
        logger.chmod(0o600)
    assert result.returncode == 77
    assert "logger is not readable" in result.stderr


def test_mount_is_minimal_and_read_only(tmp_path):
    logger = tmp_path / "teledex_logger.py"
    logger.write_text("# test logger\n", encoding="utf-8")
    (tmp_path / "teledex_env").mkdir()

    result = _resolve_mount(tmp_path)

    assert result.returncode == 0, result.stderr
    assert "<--mount>" in result.stdout
    assert "src=" + str(logger.resolve()) in result.stdout
    assert "dst=/opt/teledex_reference/teledex_logger.py" in result.stdout
    assert ",readonly>" in result.stdout
    assert "teledex_env" not in result.stdout


def test_mount_rejects_unsafe_container_reference_root(tmp_path):
    (tmp_path / "teledex_logger.py").write_text("# test\n", encoding="utf-8")
    result = _resolve_mount(tmp_path, "relative/path")
    assert result.returncode == 64
    assert "absolute non-root path" in result.stderr


def test_image_uses_exact_validated_teledex_requirements():
    requirements = (ROOT / "deployment/requirements/teledex.txt").read_text(
        encoding="utf-8").splitlines()
    pins = {line for line in requirements if line and not line.startswith("#")}
    assert pins == {
        "teledex==0.0.7",
        "numpy==1.24.4",
        "scipy==1.10.1",
        "websockets==13.1",
        "qrcode==7.4.2",
        "pypng==0.20220715.0",
        "typing_extensions==4.13.2",
    }
    dockerfile = (ROOT / "Dockerfile").read_text(encoding="utf-8")
    assert "-r /tmp/teledex-requirements.txt" in dockerfile
    assert "install -d -m 0555 /opt/teledex_reference" in dockerfile


def test_explicit_workspace_generates_current_source_and_deployment_mounts():
    result = _resolve_workspace(ROOT)

    assert result.returncode == 0, result.stderr
    assert "ROOT=<" + str(ROOT) + ">" in result.stdout
    assert (
        "ARG=<type=bind,src=" + str(ROOT / "src") +
        ",dst=/ros_ws/src>") in result.stdout
    assert (
        "ARG=<type=bind,src=" + str(ROOT / "deployment") +
        ",dst=/ros_ws/deployment,readonly>") in result.stdout
    assert "pharmarobot_ros2-master" not in result.stdout


def test_unset_workspace_defaults_to_repository_containing_helper():
    result = _resolve_workspace(None)

    assert result.returncode == 0, result.stderr
    assert "ROOT=<" + str(ROOT) + ">" in result.stdout


def test_workspace_rejects_empty_or_missing_root(tmp_path):
    empty = _resolve_workspace("")
    assert empty.returncode == 64
    assert "explicitly empty" in empty.stderr

    missing = _resolve_workspace(tmp_path / "missing")
    assert missing.returncode == 66
    assert "PHARMA_WS_DIR does not exist" in missing.stderr

    relative = _resolve_workspace("relative/workspace")
    assert relative.returncode == 64
    assert "must be an absolute path" in relative.stderr


def test_workspace_rejects_missing_required_directories_and_packages(tmp_path):
    missing_src = _resolve_workspace(tmp_path)
    assert missing_src.returncode == 66
    assert "required workspace directory is unavailable" in missing_src.stderr

    workspace = tmp_path / "workspace"
    _make_workspace(workspace)
    (workspace / "src/command_arbiter/package.xml").unlink()
    missing_package = _resolve_workspace(workspace)
    assert missing_package.returncode == 66
    assert "src/command_arbiter/package.xml" in missing_package.stderr


def test_launcher_uses_validated_workspace_mounts_and_keeps_teledex_mount():
    launcher = (
        ROOT / "deployment/scripts/pharma_start_container.sh").read_text(
            encoding="utf-8")
    assert "configure_pharma_workspace_mounts" in launcher
    assert '"${WORKSPACE_MOUNT_ARGS[@]}"' in launcher
    assert '"${TELEDEX_MOUNT_ARGS[@]}"' in launcher
    assert "pharmarobot_ros2-master" not in launcher

    d455_launcher = (
        ROOT / "deployment/scripts/pharma_d455_sensor_container.sh").read_text(
            encoding="utf-8")
    assert "resolve_pharma_workspace_root" in d455_launcher
    assert "pharmarobot_ros2-master" not in d455_launcher


def test_defaults_and_installer_replace_only_the_exact_legacy_workspace():
    defaults = (ROOT / "deployment/systemd/pharmarobot.default").read_text(
        encoding="utf-8")
    assert (
        "PHARMA_WS_DIR=" + str(ROOT)) in defaults
    assert "pharmarobot_ros2-master" not in defaults

    installer = (ROOT / "deployment/install_services.sh").read_text(
        encoding="utf-8")
    assert 'LEGACY_WS_DIR="/home/medrobots/pharmarobot/' in installer
    assert "PHARMA_WS_DIR=${REPO_ROOT}" in installer
    assert 'LEGACY_PHARMA_IMAGE="pharmarobot:2026-06-12-l1"' in installer
    assert 'DEFAULT_PHARMA_IMAGE="pharmarobot:clean"' in installer
    assert "PHARMA_IMAGE=${DEFAULT_PHARMA_IMAGE}" in installer
