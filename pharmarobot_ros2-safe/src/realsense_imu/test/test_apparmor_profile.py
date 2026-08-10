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

from dataclasses import replace
from pathlib import Path
import re
import shutil
import subprocess

import pytest

from realsense_imu.apparmor_profile import AppArmorProfileError
from realsense_imu.apparmor_profile import generate_apparmor_profile
from realsense_imu.apparmor_profile import PROFILE_NAME
from realsense_imu.apparmor_profile import profile_template_sha256
from realsense_imu.usb_device import DeviceNode
from realsense_imu.usb_device import HostResources
from realsense_imu.usb_device import IioDevice


USB_SYSFS_PATH = Path(
    "/sys/devices/pci0000:00/0000:00:14.0/usb4/4-3/4-3.1"
)
ACCEL_SYSFS_PATH = USB_SYSFS_PATH / "4-3.1:1.5" / "iio:device0"
GYRO_SYSFS_PATH = USB_SYSFS_PATH / "4-3.1:1.5" / "iio:device1"


def valid_selector_output():
    """Return one valid, serial-selected D455 HostResources result."""
    return HostResources(
        usb_sysfs_path=USB_SYSFS_PATH,
        device_nodes=(
            DeviceNode(Path("/dev/bus/usb/004/004"), 189, 387),
            DeviceNode(Path("/dev/video0"), 81, 0),
            DeviceNode(Path("/dev/media0"), 239, 0),
            DeviceNode(Path("/dev/iio:device0"), 250, 0),
            DeviceNode(Path("/dev/iio:device1"), 250, 1),
        ),
        iio_devices=(
            IioDevice(
                "accel_3d",
                Path("/dev/iio:device0"),
                ACCEL_SYSFS_PATH,
                has_hysteresis_control=True,
            ),
            IioDevice(
                "gyro_3d",
                Path("/dev/iio:device1"),
                GYRO_SYSFS_PATH,
                has_hysteresis_control=True,
            ),
        ),
    )


def sysfs_write_rules(profile):
    """Extract generated sysfs rules that carry write permission."""
    return [
        line.strip()
        for line in profile.splitlines()
        if line.strip().startswith('"/sys/')
        and re.search(r"\s[^,]*w[^,]*,$", line)
    ]


def test_profile_generation_from_valid_selector_output_is_stable():
    resources = valid_selector_output()

    profile = generate_apparmor_profile(resources)

    assert f'profile "{PROFILE_NAME}"' in profile
    assert str(USB_SYSFS_PATH) in profile
    assert str(ACCEL_SYSFS_PATH) in profile
    assert str(GYRO_SYSFS_PATH) in profile
    assert profile == generate_apparmor_profile(resources)


def test_profile_template_source_hash_is_stable_sha256():
    first = profile_template_sha256()

    assert re.fullmatch(r"[0-9a-f]{64}", first)
    assert first == profile_template_sha256()


@pytest.mark.parametrize(
    "iio_devices",
    [
        (),
        (IioDevice("accel_3d", Path("/dev/iio:device0"), ACCEL_SYSFS_PATH),),
        (
            IioDevice("accel_3d", Path("/dev/iio:device0"), ACCEL_SYSFS_PATH),
            IioDevice("accel_3d", Path("/dev/iio:device2"), ACCEL_SYSFS_PATH),
            IioDevice("gyro_3d", Path("/dev/iio:device1"), GYRO_SYSFS_PATH),
        ),
    ],
)
def test_profile_refuses_missing_or_multiple_motion_matches(iio_devices):
    resources = replace(valid_selector_output(), iio_devices=iio_devices)

    with pytest.raises(AppArmorProfileError, match="exactly one"):
        generate_apparmor_profile(resources)


def test_profile_refuses_iio_path_outside_selected_d455():
    resources = valid_selector_output()
    outside_gyro = replace(
        resources.iio_devices[1],
        sysfs_path=Path("/sys/devices/unrelated/iio:device1"),
    )
    resources = replace(
        resources,
        iio_devices=(resources.iio_devices[0], outside_gyro),
    )

    with pytest.raises(
        AppArmorProfileError, match="outside the selected D455"
    ):
        generate_apparmor_profile(resources)


def test_profile_contains_no_broad_or_unconfined_permissions():
    profile = generate_apparmor_profile(valid_selector_output())

    for forbidden in (
        "/sys/**",
        "/sys/devices/**",
        "/sys/devices/**/uevent",
        "/dev/bus/usb/**",
        "c 189:*",
        "apparmor=unconfined",
        "ux,",
        "Ux,",
        "file,",
    ):
        assert forbidden not in profile


def test_bounded_discovery_patterns_are_read_only():
    profile = generate_apparmor_profile(valid_selector_output())
    expected_patterns = (
        "/sys/devices/system/cpu/{,**}",
        "/sys/devices/system/node/{,**}",
        "/sys/devices/pci0000:00/0000:00:0d.0/usb[12]/{,**}",
        "/sys/devices/pci0000:00/0000:00:14.0/usb[34]/{,**}",
        "/sys/bus/platform/drivers/hid_sensor_custom/{,**}",
    )

    for pattern in expected_patterns:
        assert f'"{pattern}" r,' in profile
        assert f'"{pattern}" rw,' not in profile
        matching_rules = [
            line for line in profile.splitlines() if pattern in line
        ]
        assert matching_rules
        assert all(
            "w" not in rule.rsplit(" ", 1)[-1]
            for rule in matching_rules
        )


def test_discovery_patterns_do_not_escape_bounded_subtrees():
    profile = generate_apparmor_profile(valid_selector_output())

    for forbidden in (
        '"/sys/{,**}" r,',
        '"/sys/devices/{,**}" r,',
        '"/sys/bus/{,**}" r,',
        '"/sys/devices/pci0000:00/{,**}" r,',
        '"/sys/devices/pci0000:00/0000:00:14.0/{,**}" r,',
    ):
        assert forbidden not in profile


def test_only_exact_accel_and_gyro_controls_are_sysfs_writable():
    profile = generate_apparmor_profile(valid_selector_output())
    write_rules = sysfs_write_rules(profile)

    assert len(write_rules) == 18
    assert all(
        str(ACCEL_SYSFS_PATH) in rule or str(GYRO_SYSFS_PATH) in rule
        for rule in write_rules
    )
    assert any("in_accel_sampling_frequency" in rule for rule in write_rules)
    assert any("in_anglvel_sampling_frequency" in rule for rule in write_rules)
    assert (
        f'"{ACCEL_SYSFS_PATH}/in_accel_hysteresis" rw,' in write_rules
    )
    assert (
        f'"{GYRO_SYSFS_PATH}/in_anglvel_hysteresis" rw,' in write_rules
    )
    assert all(
        any(
            component in rule
            for component in (
                "/buffer/enable",
                "/buffer/length",
                "/trigger/current_trigger",
                "_sampling_frequency",
                "_hysteresis",
                "/scan_elements/",
            )
        )
        for rule in write_rules
    )


def test_profile_omits_optional_hysteresis_when_not_selected():
    resources = valid_selector_output()
    resources = replace(
        resources,
        iio_devices=tuple(
            replace(device, has_hysteresis_control=False)
            for device in resources.iio_devices
        ),
    )

    write_rules = sysfs_write_rules(generate_apparmor_profile(resources))

    assert len(write_rules) == 16
    assert all("_hysteresis" not in rule for rule in write_rules)


def test_usb_video_media_and_iio_discovery_rules_are_read_only():
    profile = generate_apparmor_profile(valid_selector_output())

    for path in (
        "/sys/bus/usb/devices/4-3.1",
        "/sys/class/video4linux/video0",
        "/sys/class/media/media0",
        "/sys/bus/iio/devices/iio:device0",
        "/sys/bus/iio/devices/iio:device1",
    ):
        assert f'"{path}" r,' in profile
        assert f'"{path}" rw,' not in profile


def test_generated_profile_passes_available_apparmor_parser(tmp_path):
    parser = shutil.which("apparmor_parser")
    if parser is None:
        pytest.skip("apparmor_parser is not installed")
    profile_path = tmp_path / "pharmarobot-d455-imu"
    profile_path.write_text(
        generate_apparmor_profile(valid_selector_output()), encoding="utf-8"
    )

    result = subprocess.run(
        [parser, "-Q", "-T", str(profile_path)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
