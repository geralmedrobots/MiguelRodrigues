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

"""Select exact host resources for D455 IMU containers."""

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import re
import stat
import subprocess
from typing import Callable, Optional, Sequence, Tuple

from realsense_imu.apparmor_profile import PROFILE_NAME


DEFAULT_SERIAL_NUMBER = "146222250608"
DEFAULT_USB_SERIAL_NUMBER = "151223061922"
DEFAULT_VENDOR_ID = "8086"
DEFAULT_PRODUCT_ID = "0b5c"
REQUIRED_IIO_NAMES = ("accel_3d", "gyro_3d")
MAX_USB_SYSFS_ENTRIES = 256
MAX_IIO_SYSFS_ENTRIES = 64
MAX_USB_UEVENT_FILES = 256
ENUMERATOR_TIMEOUT_SECONDS = 10.0


class UsbSelectionError(RuntimeError):
    """Report a fail-closed host-resource selection error."""


@dataclass(frozen=True)
class IioDevice:
    """One motion sensor's device node and writable sysfs directory."""

    name: str
    device_node: Path
    sysfs_path: Path
    has_hysteresis_control: bool = False


@dataclass(frozen=True)
class DeviceNode:
    """One validated character device and its exact device-cgroup identity."""

    path: Path
    major: int
    minor: int


@dataclass(frozen=True)
class HostResources:
    """Exact D455 resources required by the native librealsense backend."""

    usb_sysfs_path: Path
    device_nodes: Tuple[DeviceNode, ...]
    iio_devices: Tuple[IioDevice, ...]


def _read_attribute(device_path: Path, name: str) -> Optional[str]:
    try:
        return (device_path / name).read_text(encoding="utf-8").strip()
    except OSError:
        return None


def _is_character_device(path: Path) -> bool:
    try:
        return stat.S_ISCHR(path.stat().st_mode)
    except OSError:
        return False


def _has_read_write_access(path: Path) -> bool:
    return os.access(path, os.R_OK | os.W_OK)


def _device_major_minor(path: Path) -> Tuple[int, int]:
    device_stat = path.stat()
    return os.major(device_stat.st_rdev), os.minor(device_stat.st_rdev)


def _run_realsense_enumerator() -> str:
    try:
        result = subprocess.run(
            ["rs-enumerate-devices", "-s"],
            check=False,
            capture_output=True,
            text=True,
            timeout=ENUMERATOR_TIMEOUT_SECONDS,
        )
    except subprocess.TimeoutExpired as exc:
        raise UsbSelectionError(
            "rs-enumerate-devices exceeded its bounded timeout"
        ) from exc
    except OSError as exc:
        raise UsbSelectionError(
            "rs-enumerate-devices is unavailable; install librealsense2-utils"
        ) from exc
    if result.returncode != 0:
        detail = result.stderr.strip() or "no error detail"
        raise UsbSelectionError(
            f"rs-enumerate-devices failed: {detail}"
        )
    return result.stdout


def verify_librealsense_serial(
    serial_number: str = DEFAULT_SERIAL_NUMBER,
    *,
    enumerate_devices: Callable[[], str] = _run_realsense_enumerator,
) -> None:
    """Require exactly one enumerated D455 with the launch serial."""
    expected_serial = serial_number.strip()
    if not expected_serial or not expected_serial.isdigit():
        raise ValueError("serial_number must contain only digits")

    detected_serials = []
    for line in enumerate_devices().splitlines():
        if "Intel RealSense D455" not in line:
            continue
        remainder = line.split("Intel RealSense D455", 1)[1].strip()
        if remainder:
            detected_serials.append(remainder.split()[0])

    if detected_serials.count(expected_serial) != 1:
        raise UsbSelectionError(
            "expected exactly one librealsense D455 with serial "
            f"{expected_serial}, detected {detected_serials}"
        )


def _is_descendant(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def _require_descendant(path: Path, parent: Path, description: str) -> Path:
    """Resolve one path and reject a symlink/path escape."""
    resolved_path = path.resolve()
    resolved_parent = parent.resolve()
    if resolved_path == resolved_parent or not _is_descendant(
        resolved_path, resolved_parent
    ):
        raise UsbSelectionError(
            f"{description} escapes expected subtree {resolved_parent}: "
            f"{path} -> {resolved_path}"
        )
    return resolved_path


def _device_node_from_uevent(
    uevent_path: Path, dev_root: Path
) -> Optional[Path]:
    try:
        content = uevent_path.read_text(encoding="utf-8")
    except OSError:
        return None
    for line in content.splitlines():
        if line.startswith("DEVNAME="):
            devname = line.split("=", 1)[1]
            if re.fullmatch(
                r"(?:bus/usb/\d{3}/\d{3}|video\d+|media\d+|iio:device\d+)",
                devname,
            ):
                return dev_root / devname
    return None


def _iio_control_paths(
    name: str,
    sysfs_path: Path,
    *,
    include_hysteresis: bool = False,
) -> Tuple[Path, ...]:
    if name == "accel_3d":
        prefix = "in_accel"
    elif name == "gyro_3d":
        prefix = "in_anglvel"
    else:
        raise UsbSelectionError(f"unsupported IIO device name: {name}")

    controls = [
        sysfs_path / "buffer" / "enable",
        sysfs_path / "buffer" / "length",
        sysfs_path / "trigger" / "current_trigger",
        sysfs_path / f"{prefix}_sampling_frequency",
        sysfs_path / "scan_elements" / f"{prefix}_x_en",
        sysfs_path / "scan_elements" / f"{prefix}_y_en",
        sysfs_path / "scan_elements" / f"{prefix}_z_en",
        sysfs_path / "scan_elements" / "in_timestamp_en",
    ]
    if include_hysteresis:
        controls.append(sysfs_path / f"{prefix}_hysteresis")
    return tuple(controls)


def required_iio_control_paths(device: IioDevice) -> Tuple[Path, ...]:
    """Return the exact sysfs controls required for one selected IIO device."""
    return _iio_control_paths(
        device.name,
        device.sysfs_path,
        include_hysteresis=device.has_hysteresis_control,
    )


def select_host_resources(
    usb_serial_number: str = DEFAULT_USB_SERIAL_NUMBER,
    *,
    usb_sysfs_root: Path = Path("/sys/bus/usb/devices"),
    iio_sysfs_root: Path = Path("/sys/bus/iio/devices"),
    dev_root: Path = Path("/dev"),
    vendor_id: str = DEFAULT_VENDOR_ID,
    product_id: str = DEFAULT_PRODUCT_ID,
    physical_sysfs_root: Optional[Path] = None,
    is_character_device: Callable[[Path], bool] = _is_character_device,
    device_major_minor: Callable[
        [Path], Tuple[int, int]
    ] = _device_major_minor,
    has_read_write_access: Callable[[Path], bool] = _has_read_write_access,
) -> HostResources:
    """Return all and only the serial-associated native-backend resources."""
    expected_usb_serial = usb_serial_number.strip()
    if not expected_usb_serial or not expected_usb_serial.isdigit():
        raise ValueError("usb_serial_number must contain only digits")

    try:
        usb_paths = sorted(usb_sysfs_root.iterdir())
    except OSError as exc:
        raise UsbSelectionError(
            f"USB sysfs directory is unavailable: {usb_sysfs_root}"
        ) from exc
    if len(usb_paths) > MAX_USB_SYSFS_ENTRIES:
        raise UsbSelectionError(
            "USB sysfs enumeration exceeded the bounded entry limit"
        )

    product_matches = []
    serial_matches = []
    for usb_path in usb_paths:
        if _read_attribute(usb_path, "idVendor") != vendor_id:
            continue
        if _read_attribute(usb_path, "idProduct") != product_id:
            continue
        product_matches.append(usb_path)
        if _read_attribute(usb_path, "serial") == expected_usb_serial:
            serial_matches.append(usb_path)

    if not product_matches:
        raise UsbSelectionError(
            f"RealSense USB device {vendor_id}:{product_id} was not found"
        )
    if not serial_matches:
        raise UsbSelectionError(
            "D455 USB transport was found, but ASIC/USB serial "
            f"{expected_usb_serial} was not present"
        )
    if len(serial_matches) != 1:
        raise UsbSelectionError(
            "expected exactly one D455 USB transport with ASIC/USB serial "
            f"{expected_usb_serial}, found {len(serial_matches)}"
        )

    usb_path = serial_matches[0]
    if physical_sysfs_root is None:
        if usb_sysfs_root == Path("/sys/bus/usb/devices"):
            physical_sysfs_root = Path("/sys/devices")
        else:
            physical_sysfs_root = usb_sysfs_root
    usb_real_path = _require_descendant(
        usb_path,
        physical_sysfs_root,
        "serial-selected D455 USB path",
    )
    device_nodes = set()
    uevent_paths = sorted(usb_real_path.rglob("uevent"))
    if len(uevent_paths) > MAX_USB_UEVENT_FILES:
        raise UsbSelectionError(
            "selected D455 topology exceeded the bounded uevent limit"
        )
    for uevent_path in uevent_paths:
        device_node = _device_node_from_uevent(uevent_path, dev_root)
        if device_node is not None:
            device_nodes.add(device_node)

    try:
        iio_links = sorted(iio_sysfs_root.glob("iio:device*"))
    except OSError as exc:
        raise UsbSelectionError(
            f"IIO sysfs directory is unavailable: {iio_sysfs_root}"
        ) from exc
    if len(iio_links) > MAX_IIO_SYSFS_ENTRIES:
        raise UsbSelectionError(
            "IIO sysfs enumeration exceeded the bounded entry limit"
        )

    iio_by_name = {}
    for iio_link in iio_links:
        iio_real_path = iio_link.resolve()
        if not _is_descendant(iio_real_path, usb_real_path):
            continue
        name = _read_attribute(iio_link, "name")
        if name not in REQUIRED_IIO_NAMES:
            raise UsbSelectionError(
                "unexpected associated D455 IIO device class: "
                f"{name!r} at {iio_real_path}"
            )
        if name in iio_by_name:
            raise UsbSelectionError(f"multiple associated {name} IIO devices")
        device_node = dev_root / iio_link.name
        hysteresis_path = _iio_control_paths(
            name, iio_real_path, include_hysteresis=True
        )[-1]
        iio_by_name[name] = IioDevice(
            name=name,
            device_node=device_node,
            sysfs_path=iio_real_path,
            has_hysteresis_control=hysteresis_path.exists(),
        )
        device_nodes.add(device_node)

    missing_iio = set(REQUIRED_IIO_NAMES) - set(iio_by_name)
    if missing_iio:
        raise UsbSelectionError(
            f"missing associated D455 IIO devices: {sorted(missing_iio)}"
        )

    node_kinds = {
        "usb": any("/bus/usb/" in str(path) for path in device_nodes),
        "video": any(
            re.fullmatch(r"video\d+", path.name) for path in device_nodes
        ),
        "media": any(
            re.fullmatch(r"media\d+", path.name) for path in device_nodes
        ),
    }
    missing_kinds = sorted(
        name for name, found in node_kinds.items() if not found
    )
    if missing_kinds:
        raise UsbSelectionError(
            f"missing associated D455 device-node classes: {missing_kinds}"
        )

    validated_nodes = []
    for device_path in sorted(device_nodes):
        if device_path.is_symlink():
            _require_descendant(
                device_path,
                dev_root,
                "associated D455 device node",
            )
        if not is_character_device(device_path):
            raise UsbSelectionError(
                "associated device node is missing or is not a character "
                f"device: {device_path}"
            )
        if not has_read_write_access(device_path):
            raise UsbSelectionError(
                f"current user lacks read/write access to {device_path}; "
                "reload the RealSense udev rules and reconnect the camera"
            )
        try:
            major, minor = device_major_minor(device_path)
        except OSError as exc:
            raise UsbSelectionError(
                f"cannot read device identity for {device_path}"
            ) from exc
        if major < 0 or minor < 0:
            raise UsbSelectionError(
                f"invalid device identity for {device_path}: {major}:{minor}"
            )
        validated_nodes.append(
            DeviceNode(path=device_path, major=major, minor=minor)
        )

    for iio_device in iio_by_name.values():
        controls = required_iio_control_paths(iio_device)
        for control in controls:
            _require_descendant(
                control,
                iio_device.sysfs_path,
                f"{iio_device.name} control",
            )
        missing_controls = [path for path in controls if not path.exists()]
        if missing_controls:
            raise UsbSelectionError(
                f"required IIO controls are missing: {missing_controls}"
            )
        inaccessible = [
            path for path in controls if not has_read_write_access(path)
        ]
        if inaccessible:
            raise UsbSelectionError(
                f"IIO controls are not writable: {inaccessible}"
            )

    return HostResources(
        usb_sysfs_path=usb_real_path,
        device_nodes=tuple(validated_nodes),
        iio_devices=tuple(iio_by_name[name] for name in REQUIRED_IIO_NAMES),
    )


def docker_device_arguments(resources: HostResources) -> Tuple[str, ...]:
    """Return only exact device/sysfs arguments for selected D455 resources."""
    arguments = []
    for device_node in resources.device_nodes:
        path = device_node.path
        if path.name.startswith("iio:device"):
            if "," in str(path):
                raise UsbSelectionError(
                    f"unsupported comma in IIO device path: {path}"
                )
            arguments.append(f"--mount=type=bind,src={path},dst={path}")
            arguments.append(
                "--device-cgroup-rule="
                f"c {device_node.major}:{device_node.minor} rwm"
            )
        else:
            arguments.append(f"--device={path}:{path}:rwm")
    for iio_device in resources.iio_devices:
        sysfs_path = iio_device.sysfs_path
        if "," in str(sysfs_path):
            raise UsbSelectionError(
                f"unsupported comma in IIO sysfs path: {sysfs_path}"
            )
        arguments.append(
            f"--mount=type=bind,src={sysfs_path},dst={sysfs_path}"
        )
    return tuple(arguments)


def docker_arguments(resources: HostResources) -> Tuple[str, ...]:
    """Return narrow, isolated Docker arguments for the selected resources."""
    return (
        "--network=none",
        "--env=ROS_DOMAIN_ID=91",
        "--env=ROS_LOCALHOST_ONLY=1",
        "--env=RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
        "--cap-drop=ALL",
        "--security-opt=no-new-privileges:true",
        f"--security-opt=apparmor={PROFILE_NAME}",
        *docker_device_arguments(resources),
    )


def main(args: Optional[Sequence[str]] = None) -> None:
    """Verify identity/resources and print report or safe Docker arguments."""
    parser = argparse.ArgumentParser(
        description="Select exact host resources for one D455 IMU container"
    )
    parser.add_argument(
        "--serial-number",
        default=DEFAULT_SERIAL_NUMBER,
        help="expected librealsense/ROS D455 serial number",
    )
    parser.add_argument(
        "--usb-serial-number",
        default=DEFAULT_USB_SERIAL_NUMBER,
        help="expected ASIC/USB descriptor serial number",
    )
    output_group = parser.add_mutually_exclusive_group()
    output_group.add_argument(
        "--docker-args",
        action="store_true",
        help="print isolated-container Docker arguments",
    )
    output_group.add_argument(
        "--docker-device-args",
        action="store_true",
        help=(
            "print exact D455 device/sysfs arguments without network, ROS, "
            "or security-profile settings"
        ),
    )
    parsed_args = parser.parse_args(args)

    try:
        verify_librealsense_serial(parsed_args.serial_number)
        resources = select_host_resources(parsed_args.usb_serial_number)
        if parsed_args.docker_args:
            print("\n".join(docker_arguments(resources)))
            return
        if parsed_args.docker_device_args:
            print("\n".join(docker_device_arguments(resources)))
            return
    except (UsbSelectionError, ValueError) as exc:
        parser.error(str(exc))

    print(f"serial_number={parsed_args.serial_number}")
    print(f"usb_serial_number={parsed_args.usb_serial_number}")
    print(f"usb_sysfs_path={resources.usb_sysfs_path}")
    for device_node in resources.device_nodes:
        print(
            f"device_node={device_node.path} "
            f"device_id={device_node.major}:{device_node.minor}"
        )
    for iio_device in resources.iio_devices:
        print(f"iio_sysfs_path={iio_device.sysfs_path}")


if __name__ == "__main__":
    main()
