#!/usr/bin/env python3
"""Manage the independently supervised production D455 sensor container.

This host-side tool reuses the reviewed serial/resource and AppArmor seams.
It never operates the main robot container and never publishes ROS messages.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Callable, Mapping, Optional, Sequence

from realsense_imu.apparmor_profile import generate_apparmor_profile
from realsense_imu.apparmor_profile import PROFILE_NAME
from realsense_imu.usb_device import DEFAULT_SERIAL_NUMBER
from realsense_imu.usb_device import DEFAULT_USB_SERIAL_NUMBER
from realsense_imu.usb_device import docker_device_arguments
from realsense_imu.usb_device import HostResources
from realsense_imu.usb_device import select_host_resources

try:
    from d455_host_lock import d455_host_lock
    from d455_host_lock import D455HostLockError
    from d455_host_preflight import build_manifest
    from d455_host_preflight import assert_resources_unchanged
    from d455_host_preflight import _access_probe_script
    from d455_host_preflight import atomic_write_json
    from d455_host_preflight import Evidence
    from d455_host_preflight import INSTALLED_MANIFEST_PATH
    from d455_host_preflight import INSTALLED_PROFILE_PATH
    from d455_host_preflight import KERNEL_PROFILES_PATH
    from d455_host_preflight import ProfileManager
    from d455_host_preflight import utc_now
    from d455_host_preflight import validate_current_audit
    from d455_host_preflight import validate_resource_set
    from d455_host_preflight import verify_serial_bounded
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from d455_host_lock import d455_host_lock
    from d455_host_lock import D455HostLockError
    from d455_host_preflight import build_manifest
    from d455_host_preflight import assert_resources_unchanged
    from d455_host_preflight import _access_probe_script
    from d455_host_preflight import atomic_write_json
    from d455_host_preflight import Evidence
    from d455_host_preflight import INSTALLED_MANIFEST_PATH
    from d455_host_preflight import INSTALLED_PROFILE_PATH
    from d455_host_preflight import KERNEL_PROFILES_PATH
    from d455_host_preflight import ProfileManager
    from d455_host_preflight import utc_now
    from d455_host_preflight import validate_current_audit
    from d455_host_preflight import validate_resource_set
    from d455_host_preflight import verify_serial_bounded


PRODUCTION_CONTAINER_NAME = "pharmarobot_d455_sensor"
PRODUCTION_IMAGE = "pharmarobot:d455-sensor"
PRODUCTION_LABEL_KEY = "pharmarobot.d455.production"
PRODUCTION_LABEL = f"{PRODUCTION_LABEL_KEY}=true"
OWNER_LABEL_KEY = "pharmarobot.owner"
OWNER_LABEL_VALUE = "pharmarobot-d455-sensor"
ROLE_LABEL_KEY = "pharmarobot.role"
ROLE_LABEL_VALUE = "d455-sensor"
CONFIG_LABEL_KEY = "pharmarobot.d455.config-sha256"
SENSOR_IMAGE_LABEL_KEY = "pharmarobot.d455.sensor-image"
SOURCE_MANIFEST_LABEL_KEY = "pharmarobot.d455.source-manifest-sha256"
BASE_IMAGE_LABEL_KEY = "pharmarobot.d455.base-image-id"
VALIDATION_LABEL_KEY = "pharmarobot.d455.validation"
DEFAULT_RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"
DEFAULT_FASTDDS_BUILTIN_TRANSPORTS = "UDPv4"
DEFAULT_ROS_DOMAIN_ID = 0
DEFAULT_EVIDENCE_ROOT = Path("/var/log/pharmarobot/d455-sensor")
DEFAULT_IMAGE_MANIFEST = Path(
    "/var/lib/pharmarobot/d455-sensor-image.env"
)
DEFAULT_OWNERSHIP_RECORD = Path(
    "/var/lib/pharmarobot/d455-sensor-container.json"
)
MAIN_CONTAINER_NAME = "pharma_container"
ACCESS_PROBE_NAME = "pharmarobot_d455_sensor_access_probe"
ACCESS_PROBE_LABEL_KEY = "pharmarobot.d455.production-probe"
COMMAND_TIMEOUT_SECONDS = 30.0
STOP_TIMEOUT_SECONDS = 20
CONTAINER_NAME_PATTERN = re.compile(r"pharmarobot_d455_sensor")
IMAGE_ID_PATTERN = re.compile(r"sha256:[0-9a-f]{64}")
CONTAINER_ID_PATTERN = re.compile(r"[0-9a-f]{64}")
FORBIDDEN_CONTAINER_FRAGMENTS = (
    "--privileged",
    "apparmor=unconfined",
    "/dev/roboteq",
    "/dev/ttyUSB",
    "/dev/ttyACM",
    "/dev/input",
    "command_arbiter",
    "joy_node",
    "roboteq",
    "robot_localization",
    "nav2",
    "slam",
)
LEGACY_MAIN_D455_MARKERS = (
    "D455_IMU_",
    "D455_SERIAL_NUMBER=",
    PROFILE_NAME,
    "/dev/bus/usb/",
    "/dev/video",
    "/dev/media",
    "/dev/iio:device",
    "HID-SENSOR",
    "realsense2_camera",
    "realsense_imu",
    "robot_sensors.launch.py",
    "d455_imu.launch.py",
)
LEGACY_MAIN_PROCESS_PATTERN = re.compile(
    r"realsense2_camera|/imu_relay|d455_imu_processor|"
    r"robot_sensors\.launch\.py|d455_imu\.launch\.py"
)
D455_PRODUCTION_PROCESS_PATTERNS = (
    (
        "realsense2_camera_node",
        re.compile(
            r"(?m)^(?=[^\n]*(?:^|[\s/])"
            r"realsense2_camera_node(?:\s|$))"
            r"(?=[^\n]*(?:-r\s+)?__node:=d455(?:\s|$))[^\n]*$"
        ),
    ),
    (
        "imu_relay",
        re.compile(r"(?:^|[\s/])imu_relay(?:\s|$)"),
    ),
    (
        "d455_imu_processor",
        re.compile(
            r"(?:^|[\s/])d455_imu_processor(?:\s|$)"
        ),
    ),
    (
        "realsense_imu_launch",
        re.compile(
            r"ros2\s+launch\s+realsense_imu\s+"
            r"(?:robot_sensors|d455_imu)\.launch\.py(?:\s|$)"
        ),
    ),
)


class ProductionContainerError(RuntimeError):
    """Report one fail-closed production lifecycle error."""


@dataclass(frozen=True)
class CommandResult:
    args: tuple[str, ...]
    returncode: int
    stdout: str = ""
    stderr: str = ""
    timed_out: bool = False


@dataclass(frozen=True)
class ProductionConfig:
    container_name: str = PRODUCTION_CONTAINER_NAME
    image: str = PRODUCTION_IMAGE
    serial_number: str = DEFAULT_SERIAL_NUMBER
    usb_serial_number: str = DEFAULT_USB_SERIAL_NUMBER
    ros_domain_id: int = DEFAULT_ROS_DOMAIN_ID
    rmw_implementation: str = DEFAULT_RMW_IMPLEMENTATION
    fastdds_builtin_transports: str = (
        DEFAULT_FASTDDS_BUILTIN_TRANSPORTS
    )
    evidence_root: Path = DEFAULT_EVIDENCE_ROOT
    image_manifest: Path = DEFAULT_IMAGE_MANIFEST
    ownership_record: Path = DEFAULT_OWNERSHIP_RECORD

    def validate(self) -> None:
        if not CONTAINER_NAME_PATTERN.fullmatch(self.container_name):
            raise ProductionContainerError(
                "production container name must use the fixed reviewed name"
            )
        if not self.serial_number.isdigit():
            raise ProductionContainerError(
                "D455 librealsense serial must contain only digits"
            )
        if not self.usb_serial_number.isdigit():
            raise ProductionContainerError(
                "D455 USB serial must contain only digits"
            )
        if not 0 <= self.ros_domain_id <= 232:
            raise ProductionContainerError(
                "ROS_DOMAIN_ID must be in the DDS-safe range 0..232"
            )
        if self.rmw_implementation != DEFAULT_RMW_IMPLEMENTATION:
            raise ProductionContainerError(
                "production RMW must be rmw_fastrtps_cpp"
            )
        if (
            self.fastdds_builtin_transports
            != DEFAULT_FASTDDS_BUILTIN_TRANSPORTS
        ):
            raise ProductionContainerError(
                "production Fast DDS built-in transports must be UDPv4"
            )


@dataclass(frozen=True)
class PreflightPlan:
    resources: HostResources
    resource_fingerprint: str
    evidence: Evidence
    started_at: str


class Runner:
    """Run Docker commands without a shell."""

    def run(
        self,
        args: Sequence[str],
        *,
        timeout: Optional[float] = COMMAND_TIMEOUT_SECONDS,
    ) -> CommandResult:
        command = tuple(str(value) for value in args)
        try:
            completed = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired as exc:
            return CommandResult(
                command,
                124,
                exc.stdout or "",
                exc.stderr or "",
                True,
            )
        except OSError as exc:
            return CommandResult(command, 127, "", str(exc))
        return CommandResult(
            command,
            completed.returncode,
            completed.stdout,
            completed.stderr,
        )

    def run_attached(self, args: Sequence[str]) -> CommandResult:
        """Stream a supervised container's output without unbounded capture."""
        command = tuple(str(value) for value in args)
        try:
            completed = subprocess.run(command, check=False)
        except OSError as exc:
            return CommandResult(command, 127, "", str(exc))
        return CommandResult(command, completed.returncode)


def canonical_json(value: Mapping[str, Any]) -> str:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    )


def config_fingerprint(
    *,
    config: ProductionConfig,
    image_id: str,
    resource_fingerprint: str,
) -> str:
    value = {
        "container_name": config.container_name,
        "image_id": image_id,
        "resource_fingerprint": resource_fingerprint,
        "serial_number": config.serial_number,
        "usb_serial_number": config.usb_serial_number,
        "network_mode": "host",
        "ros_domain_id": config.ros_domain_id,
        "ros_localhost_only": "0",
        "rmw_implementation": config.rmw_implementation,
        "fastdds_builtin_transports": (
            config.fastdds_builtin_transports
        ),
        "apparmor_profile": PROFILE_NAME,
    }
    return hashlib.sha256(canonical_json(value).encode()).hexdigest()


def production_docker_arguments(
    resources: HostResources,
    *,
    config: ProductionConfig,
    config_sha256: str,
) -> tuple[str, ...]:
    """Return production-only isolation, DDS, and exact-resource arguments."""
    config.validate()
    if not re.fullmatch(r"[0-9a-f]{64}", config_sha256):
        raise ProductionContainerError("invalid production config hash")
    arguments = (
        "--init",
        "--network=host",
        "--cap-drop=ALL",
        "--security-opt=no-new-privileges:true",
        f"--security-opt=apparmor={PROFILE_NAME}",
        "--restart=no",
        "--label",
        PRODUCTION_LABEL,
        "--label",
        f"{OWNER_LABEL_KEY}={OWNER_LABEL_VALUE}",
        "--label",
        f"{ROLE_LABEL_KEY}={ROLE_LABEL_VALUE}",
        "--label",
        f"{CONFIG_LABEL_KEY}={config_sha256}",
        "--env",
        f"ROS_DOMAIN_ID={config.ros_domain_id}",
        "--env",
        "ROS_LOCALHOST_ONLY=0",
        "--env",
        f"RMW_IMPLEMENTATION={config.rmw_implementation}",
        "--env",
        "FASTDDS_BUILTIN_TRANSPORTS="
        f"{config.fastdds_builtin_transports}",
        "--env",
        f"D455_SERIAL_NUMBER={config.serial_number}",
        *docker_device_arguments(resources),
    )
    joined = "\n".join(arguments)
    for forbidden in FORBIDDEN_CONTAINER_FRAGMENTS:
        if forbidden in joined:
            raise ProductionContainerError(
                f"forbidden production sensor access: {forbidden}"
            )
    for broad in (
        "--device=/dev:/dev",
        "--device=/dev/bus/usb:/dev/bus/usb",
        "src=/dev,dst=/dev",
        "src=/sys,dst=/sys",
    ):
        if broad in joined:
            raise ProductionContainerError(
                f"broad production sensor access: {broad}"
            )
    return arguments


def container_create_command(
    resources: HostResources,
    *,
    config: ProductionConfig,
    image_id: str,
    config_sha256: str,
) -> list[str]:
    if not IMAGE_ID_PATTERN.fullmatch(image_id):
        raise ProductionContainerError("sensor image is not pinned by ID")
    return [
        "docker",
        "create",
        "--name",
        config.container_name,
        *production_docker_arguments(
            resources, config=config, config_sha256=config_sha256
        ),
        image_id,
    ]


def _checked(
    runner: Runner,
    args: Sequence[str],
    *,
    accepted: tuple[int, ...] = (0,),
    timeout: Optional[float] = COMMAND_TIMEOUT_SECONDS,
) -> CommandResult:
    result = runner.run(args, timeout=timeout)
    if result.timed_out:
        raise ProductionContainerError(
            f"command timed out: {' '.join(result.args)}"
        )
    if result.returncode not in accepted:
        detail = result.stderr.strip() or result.stdout.strip()
        raise ProductionContainerError(
            f"command failed ({result.returncode}): "
            f"{' '.join(result.args)}: {detail}"
        )
    return result


def _parse_single_inspect(result: CommandResult) -> Optional[dict[str, Any]]:
    if result.returncode == 1:
        return None
    if result.returncode != 0 or result.timed_out:
        raise ProductionContainerError(
            "Docker container identity cannot be determined"
        )
    try:
        values = json.loads(result.stdout)
        if len(values) != 1 or not isinstance(values[0], dict):
            raise ValueError
        return values[0]
    except (json.JSONDecodeError, TypeError, ValueError) as exc:
        raise ProductionContainerError(
            "Docker inspect returned ambiguous data"
        ) from exc


def is_owned_container(inspect: Mapping[str, Any]) -> bool:
    labels = inspect.get("Config", {}).get("Labels", {}) or {}
    return (
        bool(CONTAINER_ID_PATTERN.fullmatch(str(inspect.get("Id", ""))))
        and labels.get(PRODUCTION_LABEL_KEY) == "true"
        and labels.get(OWNER_LABEL_KEY) == OWNER_LABEL_VALUE
        and labels.get(ROLE_LABEL_KEY) == ROLE_LABEL_VALUE
        and bool(
            re.fullmatch(
                r"[0-9a-f]{64}", labels.get(CONFIG_LABEL_KEY, "")
            )
        )
        and inspect.get("Name") == f"/{PRODUCTION_CONTAINER_NAME}"
    )


def assert_no_active_validation_container(runner: Runner) -> None:
    result = _checked(
        runner,
        [
            "docker",
            "ps",
            "--filter",
            f"label={VALIDATION_LABEL_KEY}=true",
            "--format",
            "{{.ID}} {{.Names}}",
        ],
    )
    if result.stdout.strip():
        raise ProductionContainerError(
            "an active D455 validation container blocks production startup"
        )


def assert_unique_production_container(
    runner: Runner, expected_name: str
) -> None:
    result = _checked(
        runner,
        [
            "docker",
            "ps",
            "-a",
            "--filter",
            f"label={PRODUCTION_LABEL}",
            "--format",
            "{{.Names}}",
        ],
    )
    names = [line.strip() for line in result.stdout.splitlines() if line]
    if len(names) > 1 or (names and names != [expected_name]):
        raise ProductionContainerError(
            f"ambiguous production D455 containers: {names}"
        )


def _selected_d455_resource_paths(
    resources: HostResources,
) -> set[str]:
    paths = {str(node.path) for node in resources.device_nodes}
    paths.update(
        str(device.sysfs_path) for device in resources.iio_devices
    )
    return paths


def _covered_selected_paths(
    exposed_path: object,
    selected_paths: set[str],
) -> list[str]:
    if not isinstance(exposed_path, str) or not exposed_path.startswith("/"):
        return []
    exposed = Path(exposed_path)
    return sorted(
        selected
        for selected in selected_paths
        if exposed == Path(selected) or exposed in Path(selected).parents
    )


def assert_no_foreign_running_d455_containers(
    runner: Runner,
    resources: HostResources,
    *,
    expected_container_id: Optional[str],
) -> None:
    """Reject exact selected-D455 exposure outside the expected owner."""
    census = _checked(
        runner,
        [
            "docker",
            "ps",
            "--no-trunc",
            "--format",
            "{{.ID}}",
        ],
    )
    container_ids = [
        line.strip()
        for line in census.stdout.splitlines()
        if line.strip()
    ]
    if len(container_ids) != len(set(container_ids)) or any(
        not CONTAINER_ID_PATTERN.fullmatch(container_id)
        for container_id in container_ids
    ):
        raise ProductionContainerError(
            "running-container D455 census returned ambiguous identities"
        )

    selected_paths = _selected_d455_resource_paths(resources)
    conflicts: list[dict[str, Any]] = []
    for container_id in container_ids:
        inspected = _checked(
            runner,
            ["docker", "inspect", container_id],
        )
        record = _parse_single_inspect(inspected)
        if record is None:
            raise ProductionContainerError(
                "running-container D455 census inspect was ambiguous"
            )
        if (
            container_id == expected_container_id
            and is_owned_container(record)
        ):
            continue

        matches: set[str] = set()
        reasons: set[str] = set()
        host = record.get("HostConfig", {}) or {}
        privileged = host.get("Privileged") is True
        if privileged:
            matches.add("isolation:Privileged=true")
            reasons.add(
                "foreign running container is privileged and has "
                "ambiguous broad D455 access"
            )
        for device in host.get("Devices", ()) or ():
            exposed = device.get("PathOnHost")
            for selected in _covered_selected_paths(
                exposed, selected_paths
            ):
                matches.add(
                    f"device:{exposed} covers selected:{selected}"
                )

        mounts = list(record.get("Mounts", ()) or ())
        mounts.extend(host.get("Mounts", ()) or ())
        for mount in mounts:
            exposed = mount.get("Source")
            for selected in _covered_selected_paths(
                exposed, selected_paths
            ):
                matches.add(
                    f"mount:{exposed} covers selected:{selected}"
                )

        if not privileged:
            processes = _checked(
                runner,
                [
                    "docker",
                    "top",
                    container_id,
                    "-eo",
                    "pid,ppid,stat,args",
                ],
            )
            for name, pattern in D455_PRODUCTION_PROCESS_PATTERNS:
                if pattern.search(processes.stdout):
                    matches.add(f"process:{name}")

        if matches:
            if matches != {"isolation:Privileged=true"}:
                reasons.add(
                    "selected D455 resource or production process is "
                    "owned outside the expected production container"
                )
            labels = record.get("Config", {}).get("Labels", {}) or {}
            conflicts.append(
                {
                    "container_id": container_id,
                    "labels": labels,
                    "matches": sorted(matches),
                    "name": str(record.get("Name", "")).lstrip("/"),
                    "reason": "; ".join(sorted(reasons)),
                }
            )

    if conflicts:
        raise ProductionContainerError(
            "foreign running D455 container conflicts: "
            f"{canonical_json({'conflicts': conflicts})}"
        )


def assert_main_container_has_no_d455(runner: Runner) -> None:
    """Fail closed if the unchanged main container may still own the D455."""
    result = runner.run(
        ["docker", "inspect", MAIN_CONTAINER_NAME],
        timeout=COMMAND_TIMEOUT_SECONDS,
    )
    if result.timed_out:
        raise ProductionContainerError(
            "main-container D455 exclusion inspect timed out"
        )
    if result.returncode == 1:
        if "no such" not in result.stderr.lower():
            raise ProductionContainerError(
                "main-container D455 exclusion inspect is ambiguous"
            )
        return
    if result.returncode != 0:
        raise ProductionContainerError(
            "main-container D455 exclusion inspect failed"
        )
    existing = _parse_single_inspect(result)
    if existing is None:
        raise ProductionContainerError(
            "main-container D455 exclusion identity is ambiguous"
        )
    serialized = canonical_json(existing)
    markers = [
        marker
        for marker in LEGACY_MAIN_D455_MARKERS
        if marker in serialized
    ]
    if markers:
        raise ProductionContainerError(
            "existing main container has legacy D455 access or startup "
            f"markers: {markers}"
        )
    if not existing.get("State", {}).get("Running"):
        return
    container_id = str(existing.get("Id", ""))
    if not CONTAINER_ID_PATTERN.fullmatch(container_id):
        raise ProductionContainerError(
            "running main-container identity is unverifiable"
        )
    processes = runner.run(
        [
            "docker",
            "top",
            container_id,
            "-eo",
            "pid,ppid,stat,args",
        ],
        timeout=COMMAND_TIMEOUT_SECONDS,
    )
    if processes.returncode != 0 or processes.timed_out:
        raise ProductionContainerError(
            "running main-container process exclusion is unverifiable"
        )
    match = LEGACY_MAIN_PROCESS_PATTERN.search(processes.stdout)
    if match:
        raise ProductionContainerError(
            "running main container has a legacy D455 process: "
            f"{match.group(0)}"
        )


class ProductionLifecycle:
    """Own exactly one fixed-name production D455 sensor container."""

    def __init__(
        self,
        config: ProductionConfig,
        *,
        runner: Optional[Runner] = None,
        select_resources: Callable[..., HostResources] = select_host_resources,
        lock_factory: Callable[[str], Any] = d455_host_lock,
    ):
        config.validate()
        self.config = config
        self.runner = runner or Runner()
        self.select_resources = select_resources
        self.lock_factory = lock_factory

    @contextmanager
    def host_lock(self, operation: str):
        try:
            with self.lock_factory(operation):
                yield
        except D455HostLockError as exc:
            raise ProductionContainerError(str(exc)) from exc

    def inspect_optional(self) -> Optional[dict[str, Any]]:
        result = self.runner.run(
            ["docker", "inspect", self.config.container_name],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        return _parse_single_inspect(result)

    def require_owned(self) -> dict[str, Any]:
        existing = self.inspect_optional()
        if existing is None:
            raise ProductionContainerError(
                "production D455 sensor container does not exist"
            )
        if not is_owned_container(existing):
            raise ProductionContainerError(
                "refusing to operate a foreign or partially owned container"
            )
        return existing

    def write_ownership_record(
        self, inspect: Mapping[str, Any]
    ) -> None:
        labels = inspect.get("Config", {}).get("Labels", {}) or {}
        atomic_write_json(
            self.config.ownership_record,
            {
                "schema_version": 1,
                "container_name": self.config.container_name,
                "container_id": inspect["Id"],
                "image_id": inspect["Image"],
                "config_sha256": labels[CONFIG_LABEL_KEY],
                "owner": OWNER_LABEL_VALUE,
            },
        )

    def require_recorded_owned(self) -> dict[str, Any]:
        existing = self.require_owned()
        try:
            record = json.loads(
                self.config.ownership_record.read_text(encoding="utf-8")
            )
        except (OSError, json.JSONDecodeError) as exc:
            raise ProductionContainerError(
                "immutable production ownership record is unavailable"
            ) from exc
        labels = existing.get("Config", {}).get("Labels", {}) or {}
        expected = {
            "schema_version": 1,
            "container_name": self.config.container_name,
            "container_id": existing.get("Id"),
            "image_id": existing.get("Image"),
            "config_sha256": labels.get(CONFIG_LABEL_KEY),
            "owner": OWNER_LABEL_VALUE,
        }
        if record != expected:
            raise ProductionContainerError(
                "production ownership record does not match exact container"
            )
        return existing

    def cleanup_new_container(
        self,
        *,
        container_id: str,
        image_id: str,
        config_sha256: str,
    ) -> None:
        """Remove and prove absence of one exact newly created container."""
        current = self.require_recorded_owned()
        labels = current.get("Config", {}).get("Labels", {}) or {}
        if (
            current.get("Id") != container_id
            or current.get("Image") != image_id
            or labels.get(CONFIG_LABEL_KEY) != config_sha256
        ):
            raise ProductionContainerError(
                "new-container cleanup identity cannot be proven"
            )
        removed = self.runner.run(
            ["docker", "rm", "-f", container_id],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        if removed.returncode != 0 or removed.timed_out:
            raise ProductionContainerError(
                "new-container cleanup command failed"
            )
        absence = self.runner.run(
            ["docker", "inspect", container_id],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        if absence.returncode != 1 or absence.timed_out:
            raise ProductionContainerError(
                "new-container cleanup absence was not proven"
            )

    def image_id(self) -> str:
        manifest = self._read_image_manifest()
        result = _checked(
            self.runner, ["docker", "image", "inspect", self.config.image]
        )
        try:
            values = json.loads(result.stdout)
            image = values[0]
            image_id = image["Id"]
            labels = image.get("Config", {}).get("Labels", {}) or {}
        except (IndexError, KeyError, TypeError, json.JSONDecodeError) as exc:
            raise ProductionContainerError(
                "sensor image identity cannot be parsed"
            ) from exc
        if not IMAGE_ID_PATTERN.fullmatch(image_id):
            raise ProductionContainerError(
                "sensor image identity is not a digest"
            )
        if (
            labels.get(SENSOR_IMAGE_LABEL_KEY) != "true"
            or image_id != manifest["IMAGE_ID"]
            or labels.get(SOURCE_MANIFEST_LABEL_KEY)
            != manifest["SOURCE_MANIFEST_SHA256"]
            or labels.get(BASE_IMAGE_LABEL_KEY)
            != manifest["BASE_IMAGE_ID"]
        ):
            raise ProductionContainerError(
                "image is not a reviewed production D455 sensor image"
            )
        return image_id

    def _read_image_manifest(self) -> dict[str, str]:
        try:
            lines = self.config.image_manifest.read_text(
                encoding="utf-8"
            ).splitlines()
        except OSError as exc:
            raise ProductionContainerError(
                "reviewed sensor-image manifest is unavailable"
            ) from exc
        values: dict[str, str] = {}
        for line in lines:
            if not line or "=" not in line:
                raise ProductionContainerError(
                    "sensor-image manifest is malformed"
                )
            key, value = line.split("=", 1)
            if key in values:
                raise ProductionContainerError(
                    "sensor-image manifest has duplicate keys"
                )
            values[key] = value
        if set(values) != {
            "IMAGE",
            "IMAGE_ID",
            "SOURCE_MANIFEST_SHA256",
            "BASE_IMAGE_ID",
        }:
            raise ProductionContainerError(
                "sensor-image manifest key set is invalid"
            )
        if (
            values["IMAGE"] != self.config.image
            or not IMAGE_ID_PATTERN.fullmatch(values["IMAGE_ID"])
            or not re.fullmatch(
                r"[0-9a-f]{64}", values["SOURCE_MANIFEST_SHA256"]
            )
            or not IMAGE_ID_PATTERN.fullmatch(values["BASE_IMAGE_ID"])
        ):
            raise ProductionContainerError(
                "sensor-image manifest identity is invalid"
            )
        return values

    def verify_container_contract(
        self,
        inspect: Mapping[str, Any],
        *,
        resources: HostResources,
        image_id: str,
        config_sha256: str,
    ) -> None:
        """Prove immutable Docker settings match the selected D455 plan."""
        if not is_owned_container(inspect):
            raise ProductionContainerError(
                "production ownership labels are incomplete"
            )
        host = inspect.get("HostConfig", {}) or {}
        config = inspect.get("Config", {}) or {}
        labels = config.get("Labels", {}) or {}
        environment = set(config.get("Env", ()) or ())
        expected_security = {
            "no-new-privileges:true",
            f"apparmor={PROFILE_NAME}",
        }
        expected_environment = {
            f"ROS_DOMAIN_ID={self.config.ros_domain_id}",
            "ROS_LOCALHOST_ONLY=0",
            f"RMW_IMPLEMENTATION={self.config.rmw_implementation}",
            "FASTDDS_BUILTIN_TRANSPORTS="
            f"{self.config.fastdds_builtin_transports}",
            f"D455_SERIAL_NUMBER={self.config.serial_number}",
        }
        if (
            inspect.get("Image") != image_id
            or host.get("NetworkMode") != "host"
            or host.get("Privileged") is not False
            or set(host.get("CapDrop") or ()) != {"ALL"}
            or set(host.get("SecurityOpt") or ()) != expected_security
            or host.get("Init") is not True
            or (host.get("RestartPolicy") or {}).get("Name") != "no"
            or labels.get(CONFIG_LABEL_KEY) != config_sha256
            or not expected_environment.issubset(environment)
        ):
            raise ProductionContainerError(
                "production container isolation or DDS contract drift"
            )

        expected_devices = {
            (str(node.path), str(node.path), "rwm")
            for node in resources.device_nodes
            if not node.path.name.startswith("iio:device")
        }
        actual_devices = {
            (
                item.get("PathOnHost"),
                item.get("PathInContainer"),
                item.get("CgroupPermissions"),
            )
            for item in host.get("Devices", ()) or ()
        }
        expected_rules = {
            f"c {node.major}:{node.minor} rwm"
            for node in resources.device_nodes
            if node.path.name.startswith("iio:device")
        }
        actual_rules = set(host.get("DeviceCgroupRules", ()) or ())
        expected_mounts = {
            (str(node.path), str(node.path), "", True, "rprivate")
            for node in resources.device_nodes
            if node.path.name.startswith("iio:device")
        } | {
            (
                str(device.sysfs_path),
                str(device.sysfs_path),
                "",
                True,
                "rprivate",
            )
            for device in resources.iio_devices
        }
        actual_mounts = {
            (
                item.get("Source"),
                item.get("Destination"),
                item.get("Mode", ""),
                item.get("RW"),
                item.get("Propagation"),
            )
            for item in inspect.get("Mounts", ()) or ()
            if item.get("Type") == "bind"
        }
        if (
            actual_devices != expected_devices
            or actual_rules != expected_rules
            or actual_mounts != expected_mounts
        ):
            raise ProductionContainerError(
                "production D455 resource scope drift"
            )
        serialized = canonical_json(inspect)
        for forbidden in FORBIDDEN_CONTAINER_FRAGMENTS:
            if forbidden in serialized:
                raise ProductionContainerError(
                    f"forbidden production sensor exposure: {forbidden}"
                )

    def preflight(
        self,
        *,
        authorize_profile_reload: bool,
        expected_container_id: Optional[str] = None,
    ) -> PreflightPlan:
        started_at = utc_now()
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        evidence = Evidence(
            self.config.evidence_root / f"production-preflight-{timestamp}"
        )
        bounded_runner = self.runner
        verify_serial_bounded(
            bounded_runner, evidence, self.config.serial_number
        )
        resources = self.select_resources(self.config.usb_serial_number)
        validate_resource_set(resources)
        assert_no_foreign_running_d455_containers(
            self.runner,
            resources,
            expected_container_id=expected_container_id,
        )
        profile = generate_apparmor_profile(resources)
        manifest = dict(
            build_manifest(
                resources,
                serial_number=self.config.serial_number,
                usb_serial_number=self.config.usb_serial_number,
                profile=profile,
                generated_at=datetime.now(timezone.utc).isoformat(),
            )
        )
        manifest["deployment_mode"] = "production"
        evidence.write_text("candidate-profile.apparmor", profile)
        evidence.write_json("resource-manifest.json", manifest)
        ProfileManager(
            runner=bounded_runner,
            evidence=evidence,
            installed_profile_path=INSTALLED_PROFILE_PATH,
            installed_manifest_path=INSTALLED_MANIFEST_PATH,
            kernel_profiles_path=KERNEL_PROFILES_PATH,
        ).ensure(
            candidate_profile_path=evidence.root
            / "candidate-profile.apparmor",
            profile=profile,
            manifest=manifest,
            authorize_reload=authorize_profile_reload,
        )
        verify_serial_bounded(
            bounded_runner, evidence, self.config.serial_number
        )
        assert_resources_unchanged(
            resources,
            self.select_resources(self.config.usb_serial_number),
            phase="production_resources_after_profile",
        )
        validate_current_audit(
            bounded_runner,
            evidence,
            phase="production_audit_after_profile",
            since_utc=started_at,
            resources=resources,
        )
        return PreflightPlan(
            resources=resources,
            resource_fingerprint=str(manifest["resource_fingerprint"]),
            evidence=evidence,
            started_at=started_at,
        )

    def final_access_probe(
        self, plan: PreflightPlan, *, image_id: str
    ) -> None:
        """Prove final AppArmor/resource access without starting ROS."""
        active = _checked(
            self.runner,
            [
                "docker",
                "ps",
                "--filter",
                f"label={PRODUCTION_LABEL}",
                "--format",
                "{{.ID}}",
            ],
        )
        if active.stdout.strip():
            raise ProductionContainerError(
                "access probe cannot run beside an active production stack"
            )
        existing_probe = self.runner.run(
            ["docker", "inspect", ACCESS_PROBE_NAME],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        plan.evidence.command(
            "production_probe_absent",
            existing_probe,
            COMMAND_TIMEOUT_SECONDS,
        )
        if existing_probe.returncode != 1 or existing_probe.timed_out:
            raise ProductionContainerError(
                "production access-probe name is not provably absent"
            )

        command = [
            "docker",
            "run",
            "--rm",
            "--init",
            "--name",
            ACCESS_PROBE_NAME,
            "--network=host",
            "--cap-drop=ALL",
            "--security-opt=no-new-privileges:true",
            f"--security-opt=apparmor={PROFILE_NAME}",
            "--label",
            f"{OWNER_LABEL_KEY}={OWNER_LABEL_VALUE}",
            "--label",
            f"{ACCESS_PROBE_LABEL_KEY}=true",
            "--env",
            f"ROS_DOMAIN_ID={self.config.ros_domain_id}",
            "--env",
            "ROS_LOCALHOST_ONLY=0",
            "--env",
            f"RMW_IMPLEMENTATION={self.config.rmw_implementation}",
            "--env",
            "FASTDDS_BUILTIN_TRANSPORTS="
            f"{self.config.fastdds_builtin_transports}",
            *docker_device_arguments(plan.resources),
            "--entrypoint",
            "sh",
            image_id,
            "-c",
            _access_probe_script(plan.resources),
        ]
        probe = self.runner.run(command, timeout=COMMAND_TIMEOUT_SECONDS)
        plan.evidence.command(
            "production_final_access_probe",
            probe,
            COMMAND_TIMEOUT_SECONDS,
        )
        cleanup_error = None
        if probe.returncode != 0 or probe.timed_out:
            lingering = self.runner.run(
                ["docker", "inspect", ACCESS_PROBE_NAME],
                timeout=COMMAND_TIMEOUT_SECONDS,
            )
            plan.evidence.command(
                "production_probe_failure_inspect",
                lingering,
                COMMAND_TIMEOUT_SECONDS,
            )
            if lingering.returncode == 0 and not lingering.timed_out:
                try:
                    record = json.loads(lingering.stdout)[0]
                    labels = (
                        record.get("Config", {}).get("Labels", {}) or {}
                    )
                    probe_id = str(record.get("Id", ""))
                except (IndexError, TypeError, json.JSONDecodeError):
                    probe_id = ""
                    labels = {}
                if (
                    CONTAINER_ID_PATTERN.fullmatch(probe_id)
                    and labels.get(OWNER_LABEL_KEY) == OWNER_LABEL_VALUE
                    and labels.get(ACCESS_PROBE_LABEL_KEY) == "true"
                ):
                    removed = self.runner.run(
                        ["docker", "rm", "-f", probe_id],
                        timeout=COMMAND_TIMEOUT_SECONDS,
                    )
                    plan.evidence.command(
                        "production_probe_failure_remove",
                        removed,
                        COMMAND_TIMEOUT_SECONDS,
                    )
                    if removed.returncode != 0 or removed.timed_out:
                        cleanup_error = "probe removal failed"
                else:
                    cleanup_error = "probe ownership is ambiguous"

        absent = self.runner.run(
            ["docker", "inspect", ACCESS_PROBE_NAME],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        plan.evidence.command(
            "production_probe_absence", absent, COMMAND_TIMEOUT_SECONDS
        )
        if absent.returncode != 1 or absent.timed_out:
            cleanup_error = cleanup_error or "probe absence not proven"
        if cleanup_error:
            raise ProductionContainerError(cleanup_error)
        if probe.returncode != 0 or probe.timed_out:
            raise ProductionContainerError(
                "final-policy D455 access probe failed"
            )

        verify_serial_bounded(
            self.runner, plan.evidence, self.config.serial_number
        )
        assert_resources_unchanged(
            plan.resources,
            self.select_resources(self.config.usb_serial_number),
            phase="production_resources_before_start",
        )
        validate_current_audit(
            self.runner,
            plan.evidence,
            phase="production_audit_before_start",
            since_utc=plan.started_at,
            resources=plan.resources,
        )

    def ensure_created(
        self,
        *,
        authorize_profile_reload: bool,
        authorize_recreate: bool,
    ) -> dict[str, Any]:
        with self.host_lock("production_prepare"):
            return self._ensure_created_locked(
                authorize_profile_reload=authorize_profile_reload,
                authorize_recreate=authorize_recreate,
            )

    def _ensure_created_locked(
        self,
        *,
        authorize_profile_reload: bool,
        authorize_recreate: bool,
    ) -> dict[str, Any]:
        assert_no_active_validation_container(self.runner)
        assert_unique_production_container(
            self.runner, self.config.container_name
        )
        assert_main_container_has_no_d455(self.runner)
        existing = self.inspect_optional()
        if (
            existing is None
            and self.config.ownership_record.exists()
        ):
            raise ProductionContainerError(
                "ownership record exists but production container is absent"
            )
        if existing is not None and not is_owned_container(existing):
            raise ProductionContainerError(
                "fixed production container name is occupied by a foreign "
                "container"
            )
        if (
            existing is not None
            and existing.get("State", {}).get("Running")
            and authorize_profile_reload
        ):
            raise ProductionContainerError(
                "stop the production sensor container before authorizing "
                "an AppArmor profile reload"
            )
        image_id = self.image_id()
        plan = self.preflight(
            authorize_profile_reload=authorize_profile_reload,
            expected_container_id=(
                str(existing.get("Id"))
                if existing is not None and is_owned_container(existing)
                else None
            ),
        )
        desired_hash = config_fingerprint(
            config=self.config,
            image_id=image_id,
            resource_fingerprint=plan.resource_fingerprint,
        )
        if existing is not None:
            labels = existing.get("Config", {}).get("Labels", {}) or {}
            existing_hash = labels.get(CONFIG_LABEL_KEY)
            if existing_hash == desired_hash:
                existing = self.require_recorded_owned()
                self.verify_container_contract(
                    existing,
                    resources=plan.resources,
                    image_id=image_id,
                    config_sha256=desired_hash,
                )
                if not existing.get("State", {}).get("Running"):
                    self.final_access_probe(plan, image_id=image_id)
                return existing
            if existing.get("State", {}).get("Running"):
                raise ProductionContainerError(
                    "running production container configuration drift; "
                    "stop it before separately authorized recreation"
                )
            if not authorize_recreate:
                raise ProductionContainerError(
                    "stopped production container configuration drift; "
                    "explicit recreation authorization is required"
                )
            existing = self.require_recorded_owned()
            _checked(
                self.runner,
                ["docker", "rm", existing["Id"]],
            )
            absence = self.runner.run(
                ["docker", "inspect", existing["Id"]],
                timeout=COMMAND_TIMEOUT_SECONDS,
            )
            if absence.returncode != 1 or absence.timed_out:
                raise ProductionContainerError(
                    "authorized drifted-container removal was not proven"
                )
            self.config.ownership_record.unlink()

        self.final_access_probe(plan, image_id=image_id)
        command = container_create_command(
            plan.resources,
            config=self.config,
            image_id=image_id,
            config_sha256=desired_hash,
        )
        result = self.runner.run(command, timeout=COMMAND_TIMEOUT_SECONDS)
        if result.returncode != 0 or result.timed_out:
            partial = self.inspect_optional()
            partial_labels = (
                partial.get("Config", {}).get("Labels", {}) or {}
                if partial is not None
                else {}
            )
            if (
                partial is not None
                and is_owned_container(partial)
                and partial.get("Image") == image_id
                and partial_labels.get(CONFIG_LABEL_KEY) == desired_hash
            ):
                self.write_ownership_record(partial)
                self.cleanup_new_container(
                    container_id=partial["Id"],
                    image_id=image_id,
                    config_sha256=desired_hash,
                )
                self.config.ownership_record.unlink()
                detail = "owned partial state removed and absence proven"
            else:
                detail = "no exact owned partial state could be proven"
            raise ProductionContainerError(
                f"production container creation failed; {detail}"
            )
        created = self.require_owned()
        created_id = created["Id"]
        self.write_ownership_record(created)
        try:
            self.verify_container_contract(
                created,
                resources=plan.resources,
                image_id=image_id,
                config_sha256=desired_hash,
            )
        except BaseException as exc:
            try:
                self.cleanup_new_container(
                    container_id=created_id,
                    image_id=image_id,
                    config_sha256=desired_hash,
                )
                self.config.ownership_record.unlink()
            except BaseException as cleanup_exc:
                raise ProductionContainerError(
                    "post-create verification failed and cleanup was not "
                    f"proven: {cleanup_exc}"
                ) from exc
            raise
        return created

    def start(
        self,
        *,
        authorize_profile_reload: bool,
        authorize_recreate: bool,
        attach: bool,
    ) -> int:
        with self.host_lock("production_start"):
            existing = self._ensure_created_locked(
                authorize_profile_reload=authorize_profile_reload,
                authorize_recreate=authorize_recreate,
            )
            if not existing.get("State", {}).get("Running"):
                result = self.runner.run(
                    ["docker", "start", existing["Id"]],
                    timeout=COMMAND_TIMEOUT_SECONDS,
                )
                if result.timed_out or result.returncode != 0:
                    raise ProductionContainerError(
                        "production sensor container failed to start"
                    )
            elif not attach:
                return 0
        if not attach:
            return 0
        command = ["docker", "attach", existing["Id"]]
        if hasattr(self.runner, "run_attached"):
            result = self.runner.run_attached(command)
        else:
            result = self.runner.run(
                command,
                timeout=None,
            )
        if result.timed_out or result.returncode != 0:
            raise ProductionContainerError(
                "production sensor container failed to start"
            )
        return result.returncode

    def stop(self) -> None:
        with self.host_lock("production_stop"):
            self._stop_locked()

    def _stop_locked(self) -> None:
        existing = self.inspect_optional()
        if existing is None:
            if self.config.ownership_record.exists():
                raise ProductionContainerError(
                    "ownership record exists but container is absent"
                )
            return
        existing = self.require_recorded_owned()
        if not existing.get("State", {}).get("Running"):
            return
        _checked(
            self.runner,
            [
                "docker",
                "stop",
                "--timeout",
                str(STOP_TIMEOUT_SECONDS),
                existing["Id"],
            ],
            accepted=(0,),
        )
        stopped = self.require_recorded_owned()
        if stopped.get("State", {}).get("Running"):
            raise ProductionContainerError(
                "production container stop was not proven"
            )

    def restart(
        self,
        *,
        authorize_profile_reload: bool,
        authorize_recreate: bool,
    ) -> int:
        with self.host_lock("production_restart"):
            self._stop_locked()
            existing = self._ensure_created_locked(
                authorize_profile_reload=authorize_profile_reload,
                authorize_recreate=authorize_recreate,
            )
            result = self.runner.run(
                ["docker", "start", existing["Id"]],
                timeout=COMMAND_TIMEOUT_SECONDS,
            )
            if result.returncode != 0 or result.timed_out:
                raise ProductionContainerError(
                    "production sensor container failed to restart"
                )
        return 0

    def remove(self, *, authorize_remove: bool) -> None:
        with self.host_lock("production_remove"):
            self._remove_locked(authorize_remove=authorize_remove)

    def _remove_locked(self, *, authorize_remove: bool) -> None:
        existing = self.inspect_optional()
        if existing is None:
            if self.config.ownership_record.exists():
                raise ProductionContainerError(
                    "ownership record exists but container is absent"
                )
            return
        existing = self.require_recorded_owned()
        if not authorize_remove:
            raise ProductionContainerError(
                "explicit removal authorization is required"
            )
        if existing.get("State", {}).get("Running"):
            raise ProductionContainerError(
                "stop the production sensor container before removal"
            )
        _checked(
            self.runner, ["docker", "rm", existing["Id"]]
        )
        absent = self.runner.run(
            ["docker", "inspect", existing["Id"]],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        if absent.returncode != 1 or absent.timed_out:
            raise ProductionContainerError(
                "production container removal absence was not proven"
            )
        self.config.ownership_record.unlink()

    def migration_check(self) -> int:
        result = self.runner.run(
            ["docker", "inspect", MAIN_CONTAINER_NAME],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        existing = _parse_single_inspect(result)
        if existing is None:
            print("MAIN_CONTAINER_PRESENT=0")
            return 0
        serialized = canonical_json(existing)
        markers = [
            marker
            for marker in LEGACY_MAIN_D455_MARKERS
            if marker in serialized
        ]
        print("MAIN_CONTAINER_PRESENT=1")
        container_id = str(existing.get("Id", ""))
        if not CONTAINER_ID_PATTERN.fullmatch(container_id):
            raise ProductionContainerError(
                "main-container identity is not a full Docker container ID"
            )
        print(f"MAIN_CONTAINER_ID={container_id}")
        print(f"LEGACY_D455_ACCESS={int(bool(markers))}")
        print(f"LEGACY_MARKERS={','.join(markers)}")
        return 2 if markers else 0


def config_from_environment() -> ProductionConfig:
    try:
        domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    except ValueError as exc:
        raise ProductionContainerError(
            "ROS_DOMAIN_ID must be an integer"
        ) from exc
    return ProductionConfig(
        container_name=os.environ.get(
            "D455_SENSOR_CONTAINER", PRODUCTION_CONTAINER_NAME
        ),
        image=os.environ.get("D455_SENSOR_IMAGE", PRODUCTION_IMAGE),
        serial_number=os.environ.get(
            "D455_SERIAL_NUMBER", DEFAULT_SERIAL_NUMBER
        ),
        usb_serial_number=os.environ.get(
            "D455_USB_SERIAL_NUMBER", DEFAULT_USB_SERIAL_NUMBER
        ),
        ros_domain_id=domain,
        rmw_implementation=os.environ.get(
            "RMW_IMPLEMENTATION", DEFAULT_RMW_IMPLEMENTATION
        ),
        fastdds_builtin_transports=os.environ.get(
            "FASTDDS_BUILTIN_TRANSPORTS",
            DEFAULT_FASTDDS_BUILTIN_TRANSPORTS,
        ),
        evidence_root=Path(
            os.environ.get(
                "D455_SENSOR_EVIDENCE_ROOT",
                str(DEFAULT_EVIDENCE_ROOT),
            )
        ),
        image_manifest=Path(
            os.environ.get(
                "D455_SENSOR_IMAGE_MANIFEST",
                str(DEFAULT_IMAGE_MANIFEST),
            )
        ),
        ownership_record=Path(
            os.environ.get(
                "D455_SENSOR_OWNERSHIP_RECORD",
                str(DEFAULT_OWNERSHIP_RECORD),
            )
        ),
    )


def main(args: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Manage the production D455 sensor container"
    )
    parser.add_argument(
        "action",
        choices=(
            "run",
            "start",
            "stop",
            "restart",
            "status",
            "inspect",
            "logs",
            "remove",
            "migration-check",
        ),
    )
    parser.add_argument("--authorize-profile-reload", action="store_true")
    parser.add_argument("--authorize-recreate", action="store_true")
    parser.add_argument("--authorize-remove", action="store_true")
    parsed = parser.parse_args(args)
    lifecycle = ProductionLifecycle(config_from_environment())
    reload_allowed = parsed.authorize_profile_reload
    recreate_allowed = parsed.authorize_recreate

    try:
        if parsed.action == "run":
            return lifecycle.start(
                authorize_profile_reload=reload_allowed,
                authorize_recreate=recreate_allowed,
                attach=True,
            )
        if parsed.action == "start":
            return lifecycle.start(
                authorize_profile_reload=reload_allowed,
                authorize_recreate=recreate_allowed,
                attach=False,
            )
        if parsed.action == "stop":
            lifecycle.stop()
            return 0
        if parsed.action == "restart":
            return lifecycle.restart(
                authorize_profile_reload=reload_allowed,
                authorize_recreate=recreate_allowed,
            )
        if parsed.action == "remove":
            lifecycle.remove(authorize_remove=parsed.authorize_remove)
            return 0
        if parsed.action == "migration-check":
            return lifecycle.migration_check()
        if parsed.action == "status":
            assert_unique_production_container(
                lifecycle.runner, lifecycle.config.container_name
            )
            existing = lifecycle.inspect_optional()
            if existing is None:
                print("D455_SENSOR_CONTAINER=absent")
                print("D455_SENSOR_READY=0")
                return 3
            existing = lifecycle.require_recorded_owned()
            state = existing.get("State", {}).get("Status", "unknown")
            print(f"D455_SENSOR_CONTAINER={state}")
            health = (
                existing.get("State", {})
                .get("Health", {})
                .get("Status", "unavailable")
            )
            print(f"D455_SENSOR_HEALTH={health}")
            ready = state == "running" and health == "healthy"
            print(f"D455_SENSOR_READY={int(ready)}")
            return 0 if ready else 4
        if parsed.action == "inspect":
            assert_unique_production_container(
                lifecycle.runner, lifecycle.config.container_name
            )
            existing = lifecycle.require_recorded_owned()
            print(json.dumps(existing, indent=2, sort_keys=True))
            return 0
        if parsed.action == "logs":
            assert_unique_production_container(
                lifecycle.runner, lifecycle.config.container_name
            )
            lifecycle.require_recorded_owned()
            result = _checked(
                lifecycle.runner,
                [
                    "docker",
                    "logs",
                    "--tail",
                    "500",
                    lifecycle.config.container_name,
                ],
            )
            print(result.stdout, end="")
            print(result.stderr, end="", file=sys.stderr)
            return 0
    except ProductionContainerError as exc:
        print(f"D455 production container failed: {exc}", file=sys.stderr)
        return 1
    raise AssertionError("unhandled action")


if __name__ == "__main__":
    raise SystemExit(main())
