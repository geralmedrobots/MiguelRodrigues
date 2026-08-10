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

"""Generate a narrow AppArmor profile from validated D455 resources."""

import argparse
import hashlib
from pathlib import Path
import re
from typing import Optional, Sequence, Tuple


PROFILE_NAME = "pharmarobot-d455-imu"
PROFILE_TEMPLATE_VERSION = "1"
_DEFAULT_SYSFS_ROOT = Path("/sys/devices")
_SAFE_PATH = re.compile(r"/[A-Za-z0-9_+.,:/=-]+")
_READ_ONLY_DISCOVERY_PATTERNS = (
    "/sys/devices/system/cpu/{,**}",
    "/sys/devices/system/node/{,**}",
    "/sys/devices/pci0000:00/0000:00:0d.0/usb[12]/{,**}",
    "/sys/devices/pci0000:00/0000:00:14.0/usb[34]/{,**}",
    "/sys/bus/platform/drivers/hid_sensor_custom/{,**}",
)


class AppArmorProfileError(RuntimeError):
    """Report a fail-closed profile-generation error."""


def profile_template_sha256() -> str:
    """Hash the reviewed generator source used as the profile template."""
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def _quote_path(path: Path) -> str:
    value = str(path)
    if not path.is_absolute() or not _SAFE_PATH.fullmatch(value):
        raise AppArmorProfileError(
            f"path cannot be represented safely in AppArmor: {path}"
        )
    return f'"{value}"'


def _quote_path_with_suffix(path: Path, suffix: str) -> str:
    quoted = _quote_path(path)
    return f"{quoted[:-1]}{suffix}\""


def _is_descendant(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def _control_paths(
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
        raise AppArmorProfileError(f"unsupported IIO device name: {name}")

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


def _sysfs_ancestor_rules(path: Path) -> Tuple[str, ...]:
    rules = []
    current = path
    while current != Path("/sys"):
        rules.append(f"  {_quote_path_with_suffix(current, '/')} r,")
        current = current.parent
        if (
            not _is_descendant(current, Path("/sys"))
            and current != Path("/sys")
        ):
            raise AppArmorProfileError(f"sysfs path escapes /sys: {path}")
    rules.append('  "/sys/" r,')
    return tuple(reversed(rules))


def _read_only_discovery_rules(resources) -> Tuple[str, ...]:
    """Return bounded read-only rules needed for sysfs discovery."""
    rules = set(_sysfs_ancestor_rules(resources.usb_sysfs_path))
    rules.add(
        "  "
        f"{_quote_path_with_suffix(resources.usb_sysfs_path, '/{,**}')} r,"
    )

    discovery_directories = {
        Path("/sys/bus"),
        Path("/sys/bus/usb"),
        Path("/sys/bus/usb/devices"),
        Path("/sys/bus/iio"),
        Path("/sys/bus/iio/devices"),
        Path("/sys/class"),
    }
    for device in resources.device_nodes:
        name = device.path.name
        if name.startswith("video"):
            discovery_directories.add(Path("/sys/class/video4linux"))
        elif name.startswith("media"):
            discovery_directories.add(Path("/sys/class/media"))

    for directory in discovery_directories:
        rules.update(_sysfs_ancestor_rules(directory))

    # These evidence-derived patterns are bounded to CPU/NUMA introspection,
    # the NUC's USB controller topology, and the HID custom-driver subtree.
    # They are read-only; write access is added only by exact IIO rules below.
    for pattern in _READ_ONLY_DISCOVERY_PATTERNS:
        rules.add(f'  "{pattern}" r,')

    usb_link = Path("/sys/bus/usb/devices") / resources.usb_sysfs_path.name
    rules.add(f"  {_quote_path(usb_link)} r,")
    for device in resources.device_nodes:
        name = device.path.name
        if name.startswith("video"):
            link = Path("/sys/class/video4linux") / name
            rules.add(f"  {_quote_path(link)} r,")
        elif name.startswith("media"):
            link = Path("/sys/class/media") / name
            rules.add(f"  {_quote_path(link)} r,")
        elif name.startswith("iio:device"):
            link = Path("/sys/bus/iio/devices") / name
            rules.add(f"  {_quote_path(link)} r,")
    return tuple(sorted(rules))


def generate_apparmor_profile(
    resources,
    *,
    sysfs_root: Path = _DEFAULT_SYSFS_ROOT,
) -> str:
    """Render the enforcing profile for one validated resource selection."""
    root = sysfs_root.resolve()
    usb_path = resources.usb_sysfs_path.resolve()
    if not _is_descendant(usb_path, root):
        raise AppArmorProfileError(
            f"D455 USB sysfs path is outside {root}: {usb_path}"
        )

    by_name = {device.name: device for device in resources.iio_devices}
    if set(by_name) != {"accel_3d", "gyro_3d"} or len(
        resources.iio_devices
    ) != 2:
        raise AppArmorProfileError(
            "profile requires exactly one accel_3d and one gyro_3d device"
        )

    write_rules = []
    for name in ("accel_3d", "gyro_3d"):
        device = by_name[name]
        path = device.sysfs_path.resolve()
        if not _is_descendant(path, usb_path):
            raise AppArmorProfileError(
                f"{name} sysfs path is outside the selected D455: {path}"
            )
        if not isinstance(device.has_hysteresis_control, bool):
            raise AppArmorProfileError(
                f"{name} hysteresis-control state is invalid"
            )
        for control in _control_paths(
            name,
            path,
            include_hysteresis=device.has_hysteresis_control,
        ):
            write_rules.append(f"  {_quote_path(control)} rw,")

    read_rules = _read_only_discovery_rules(resources)
    profile_lines = [
        "# Generated by realsense_imu.apparmor_profile; do not hand-edit.",
        "# Regenerate after any D455 disconnect/reset or sysfs renumbering.",
        "#include <tunables/global>",
        "",
        f'profile "{PROFILE_NAME}" '
        "flags=(attach_disconnected,mediate_deleted) {",
        "  network,",
        "  deny network alg,",
        "  capability,",
        "  umount,",
        "",
        "  # Match Docker's ordinary file access outside /sys. /sys is",
        "  # deliberately excluded and receives only the rules below.",
        '  "/" r,',
        '  "/[^s]**" rwklmix,',
        '  "/s[^y]**" rwklmix,',
        '  "/sy[^s]**" rwklmix,',
        "",
        "  # Preserve docker-default protections for procfs and mounting.",
        "  deny @{PROC}/* w,",
        "  deny @{PROC}/{[^1-9/],[^1-9/][^0-9/],"
        "[^1-9s/][^0-9y/][^0-9s/],"
        "[^1-9/][^0-9/][^0-9/][^0-9/]*}/** w,",
        "  deny @{PROC}/sys/[^k]** w,",
        "  deny @{PROC}/sys/kernel/{?,??,[^s][^h][^m]**} w,",
        "  deny @{PROC}/sysrq-trigger rwklx,",
        "  deny @{PROC}/kcore rwklx,",
        "  deny mount,",
        "",
        "  # Read-only discovery for this serial-selected D455.",
        *read_rules,
        "",
        "  # The only writable sysfs files: validated accel/gyro controls.",
        *write_rules,
        "",
        "  signal (receive) peer=unconfined,",
        "  signal (receive) peer=runc,",
        "  signal (receive) peer=crun,",
        f'  signal (send,receive) peer="{PROFILE_NAME}",',
        f'  ptrace (trace,tracedby,read,readby) peer="{PROFILE_NAME}",',
        "}",
        "",
    ]
    return "\n".join(profile_lines)


def main(args: Optional[Sequence[str]] = None) -> None:
    """Select one D455 and print its generated AppArmor profile."""
    from realsense_imu.usb_device import DEFAULT_SERIAL_NUMBER
    from realsense_imu.usb_device import DEFAULT_USB_SERIAL_NUMBER
    from realsense_imu.usb_device import select_host_resources
    from realsense_imu.usb_device import UsbSelectionError
    from realsense_imu.usb_device import verify_librealsense_serial

    parser = argparse.ArgumentParser(
        description="Generate one serial-derived D455 IMU AppArmor profile"
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
    parsed_args = parser.parse_args(args)

    try:
        verify_librealsense_serial(parsed_args.serial_number)
        resources = select_host_resources(parsed_args.usb_serial_number)
        profile = generate_apparmor_profile(resources)
    except (AppArmorProfileError, UsbSelectionError, ValueError) as exc:
        parser.error(str(exc))
    print(profile, end="")


if __name__ == "__main__":
    main()
