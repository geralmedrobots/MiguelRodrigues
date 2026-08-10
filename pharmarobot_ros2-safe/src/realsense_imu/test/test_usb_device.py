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

import pytest

import realsense_imu.apparmor_profile as apparmor_profile
from realsense_imu.apparmor_profile import generate_apparmor_profile
from realsense_imu.usb_device import docker_arguments
from realsense_imu.usb_device import docker_device_arguments
from realsense_imu.usb_device import select_host_resources
from realsense_imu.usb_device import UsbSelectionError
from realsense_imu.usb_device import verify_librealsense_serial


SERIAL_NUMBER = "146222250608"
USB_SERIAL_NUMBER = "151223061922"


def add_usb_device(
    usb_root,
    name="4-3.1",
    *,
    vendor="8086",
    product="0b5c",
    serial=USB_SERIAL_NUMBER,
):
    """Create a fake D455 USB hierarchy with native-backend resources."""
    usb_path = usb_root / name
    usb_path.mkdir()
    attributes = {
        "idVendor": vendor,
        "idProduct": product,
        "serial": serial,
    }
    for attribute, value in attributes.items():
        (usb_path / attribute).write_text(f"{value}\n", encoding="utf-8")

    resources = {
        "usb": ("bus/usb/004/004", usb_path),
        "video": ("video0", usb_path / "video4linux" / "video0"),
        "media": ("media0", usb_path / "media0"),
    }
    for devname, resource_path in resources.values():
        resource_path.mkdir(parents=True, exist_ok=True)
        (resource_path / "uevent").write_text(
            f"DEVNAME={devname}\n", encoding="utf-8"
        )
    return usb_path


def add_iio_device(
    usb_path, iio_root, number, name, *, include_hysteresis=True
):
    """Add a fake accel or gyro IIO device below the USB parent."""
    iio_real_path = usb_path / "motion" / f"iio:device{number}"
    (iio_real_path / "buffer").mkdir(parents=True)
    (iio_real_path / "trigger").mkdir()
    (iio_real_path / "scan_elements").mkdir()
    (iio_real_path / "name").write_text(f"{name}\n", encoding="utf-8")
    (iio_real_path / "uevent").write_text(
        f"DEVNAME=iio:device{number}\n", encoding="utf-8"
    )
    for path in (
        iio_real_path / "buffer" / "enable",
        iio_real_path / "buffer" / "length",
        iio_real_path / "trigger" / "current_trigger",
    ):
        path.write_text("0\n", encoding="utf-8")
    if name == "accel_3d":
        prefix = "in_accel"
    else:
        prefix = "in_anglvel"
    for path in (
        iio_real_path / f"{prefix}_sampling_frequency",
        iio_real_path / "scan_elements" / f"{prefix}_x_en",
        iio_real_path / "scan_elements" / f"{prefix}_y_en",
        iio_real_path / "scan_elements" / f"{prefix}_z_en",
        iio_real_path / "scan_elements" / "in_timestamp_en",
    ):
        path.write_text("0\n", encoding="utf-8")
    if include_hysteresis:
        (iio_real_path / f"{prefix}_hysteresis").write_text(
            "0\n", encoding="utf-8"
        )

    iio_root.mkdir(exist_ok=True)
    (iio_root / f"iio:device{number}").symlink_to(iio_real_path)
    return iio_real_path


def complete_fixture(tmp_path):
    """Return a complete fake D455 native-backend resource hierarchy."""
    usb_root = tmp_path / "usb"
    usb_root.mkdir()
    dev_root = tmp_path / "dev"
    iio_root = tmp_path / "iio"
    usb_path = add_usb_device(usb_root)
    accel_path = add_iio_device(usb_path, iio_root, 0, "accel_3d")
    gyro_path = add_iio_device(usb_path, iio_root, 1, "gyro_3d")
    return usb_root, iio_root, dev_root, accel_path, gyro_path


def select_from_fixture(tmp_path, **kwargs):
    """Select with deterministic character-device and access checks."""
    usb_root, iio_root, dev_root, _, _ = complete_fixture(tmp_path)
    return select_host_resources(
        USB_SERIAL_NUMBER,
        usb_sysfs_root=usb_root,
        iio_sysfs_root=iio_root,
        dev_root=dev_root,
        is_character_device=kwargs.get(
            "is_character_device", lambda _path: True
        ),
        device_major_minor=kwargs.get(
            "device_major_minor", fake_device_major_minor
        ),
        has_read_write_access=kwargs.get(
            "has_read_write_access", lambda _path: True
        ),
    )


def fake_device_major_minor(path):
    """Return deterministic device identities for fake character devices."""
    if path.name.startswith("iio:device"):
        return 250, int(path.name.split("iio:device", 1)[1])
    if path.name.startswith("video"):
        return 81, int(path.name.split("video", 1)[1])
    if path.name.startswith("media"):
        return 239, int(path.name.split("media", 1)[1])
    return 189, 4


def test_verify_librealsense_serial_accepts_one_expected_d455():
    output = "Intel RealSense D455          146222250608        5.13.0.50\n"

    verify_librealsense_serial(
        SERIAL_NUMBER, enumerate_devices=lambda: output
    )


@pytest.mark.parametrize(
    "output",
    [
        "",
        "Intel RealSense D455 000000000000 5.13.0.50\n",
        (
            "Intel RealSense D455 146222250608 5.13.0.50\n"
            "Intel RealSense D455 146222250608 5.13.0.50\n"
        ),
    ],
)
def test_verify_librealsense_serial_rejects_missing_wrong_or_multiple(output):
    with pytest.raises(UsbSelectionError, match="expected exactly one"):
        verify_librealsense_serial(
            SERIAL_NUMBER, enumerate_devices=lambda: output
        )


def test_verify_librealsense_serial_allows_other_uniquely_named_d455():
    output = (
        "Intel RealSense D455 146222250608 5.13.0.50\n"
        "Intel RealSense D455 000000000000 5.13.0.50\n"
    )

    verify_librealsense_serial(
        SERIAL_NUMBER, enumerate_devices=lambda: output
    )


@pytest.mark.parametrize("serial_number", ["", "not-a-serial"])
def test_verify_librealsense_serial_rejects_invalid_serial(serial_number):
    with pytest.raises(ValueError, match="serial_number"):
        verify_librealsense_serial(serial_number, enumerate_devices=lambda: "")


def test_select_host_resources_rejects_missing_product(tmp_path):
    usb_root = tmp_path / "usb"
    usb_root.mkdir()

    with pytest.raises(UsbSelectionError, match="8086:0b5c was not found"):
        select_host_resources(
            USB_SERIAL_NUMBER,
            usb_sysfs_root=usb_root,
            iio_sysfs_root=tmp_path / "iio",
        )


def test_select_host_resources_rejects_wrong_usb_serial(tmp_path):
    usb_root = tmp_path / "usb"
    usb_root.mkdir()
    add_usb_device(usb_root, serial="000000000000")

    with pytest.raises(UsbSelectionError, match="ASIC/USB serial"):
        select_host_resources(
            USB_SERIAL_NUMBER,
            usb_sysfs_root=usb_root,
            iio_sysfs_root=tmp_path / "iio",
        )


def test_select_host_resources_rejects_multiple_usb_matches(tmp_path):
    usb_root = tmp_path / "usb"
    usb_root.mkdir()
    add_usb_device(usb_root, "4-3.1")
    add_usb_device(usb_root, "4-3.2")

    with pytest.raises(UsbSelectionError, match="exactly one.*found 2"):
        select_host_resources(
            USB_SERIAL_NUMBER,
            usb_sysfs_root=usb_root,
            iio_sysfs_root=tmp_path / "iio",
        )


def test_select_host_resources_rejects_serial_selected_usb_symlink_escape(
    tmp_path,
):
    usb_root = tmp_path / "usb"
    usb_root.mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    add_usb_device(outside, "selected")
    (usb_root / "4-3.1").symlink_to(outside / "selected")

    with pytest.raises(UsbSelectionError, match="escapes expected subtree"):
        select_host_resources(
            USB_SERIAL_NUMBER,
            usb_sysfs_root=usb_root,
            iio_sysfs_root=tmp_path / "iio",
        )


def test_select_host_resources_rejects_missing_motion_resource(tmp_path):
    usb_root, iio_root, dev_root, _, _ = complete_fixture(tmp_path)
    (iio_root / "iio:device1").unlink()

    with pytest.raises(UsbSelectionError, match="gyro_3d"):
        select_host_resources(
            USB_SERIAL_NUMBER,
            usb_sysfs_root=usb_root,
            iio_sysfs_root=iio_root,
            dev_root=dev_root,
            device_major_minor=fake_device_major_minor,
        )


def test_select_host_resources_rejects_unexpected_associated_iio_class(
    tmp_path,
):
    usb_root, iio_root, dev_root, _, _ = complete_fixture(tmp_path)
    usb_path = usb_root / "4-3.1"
    add_iio_device(usb_path, iio_root, 2, "unrelated_3d")

    with pytest.raises(UsbSelectionError, match="unexpected associated"):
        select_host_resources(
            USB_SERIAL_NUMBER,
            usb_sysfs_root=usb_root,
            iio_sysfs_root=iio_root,
            dev_root=dev_root,
            is_character_device=lambda _path: True,
            device_major_minor=fake_device_major_minor,
            has_read_write_access=lambda _path: True,
        )


def test_select_host_resources_does_not_choose_first_of_ambiguous_iio_matches(
    tmp_path,
):
    usb_root, iio_root, dev_root, _, _ = complete_fixture(tmp_path)
    usb_path = usb_root / "4-3.1"
    add_iio_device(usb_path, iio_root, 2, "gyro_3d")

    with pytest.raises(UsbSelectionError, match="multiple associated gyro"):
        select_host_resources(
            USB_SERIAL_NUMBER,
            usb_sysfs_root=usb_root,
            iio_sysfs_root=iio_root,
            dev_root=dev_root,
            is_character_device=lambda _path: True,
            device_major_minor=fake_device_major_minor,
            has_read_write_access=lambda _path: True,
        )


def test_select_host_resources_rejects_non_character_node(tmp_path):
    with pytest.raises(UsbSelectionError, match="not a character device"):
        select_from_fixture(
            tmp_path,
            is_character_device=lambda path: path.name != "video0",
        )


def test_select_host_resources_rejects_insufficient_access(tmp_path):
    with pytest.raises(UsbSelectionError, match="lacks read/write access"):
        select_from_fixture(
            tmp_path,
            has_read_write_access=lambda path: path.name != "media0",
        )


def test_select_host_resources_rejects_unwritable_iio_control(tmp_path):
    usb_root, iio_root, dev_root, accel_path, _ = complete_fixture(tmp_path)
    blocked_control = accel_path / "buffer" / "enable"

    with pytest.raises(UsbSelectionError, match="IIO controls"):
        select_host_resources(
            USB_SERIAL_NUMBER,
            usb_sysfs_root=usb_root,
            iio_sysfs_root=iio_root,
            dev_root=dev_root,
            is_character_device=lambda _path: True,
            device_major_minor=fake_device_major_minor,
            has_read_write_access=lambda path: path != blocked_control,
        )


def test_select_host_resources_rejects_iio_control_symlink_escape(tmp_path):
    usb_root, iio_root, dev_root, accel_path, _ = complete_fixture(tmp_path)
    escaped_control = accel_path / "buffer" / "enable"
    outside = tmp_path / "outside-control"
    outside.write_text("0\n", encoding="utf-8")
    escaped_control.unlink()
    escaped_control.symlink_to(outside)

    with pytest.raises(UsbSelectionError, match="control escapes"):
        select_host_resources(
            USB_SERIAL_NUMBER,
            usb_sysfs_root=usb_root,
            iio_sysfs_root=iio_root,
            dev_root=dev_root,
            is_character_device=lambda _path: True,
            device_major_minor=fake_device_major_minor,
            has_read_write_access=lambda _path: True,
        )


def test_select_host_resources_rejects_unwritable_present_hysteresis(tmp_path):
    usb_root, iio_root, dev_root, _, gyro_path = complete_fixture(tmp_path)
    blocked_control = gyro_path / "in_anglvel_hysteresis"

    with pytest.raises(UsbSelectionError, match="IIO controls"):
        select_host_resources(
            USB_SERIAL_NUMBER,
            usb_sysfs_root=usb_root,
            iio_sysfs_root=iio_root,
            dev_root=dev_root,
            is_character_device=lambda _path: True,
            device_major_minor=fake_device_major_minor,
            has_read_write_access=lambda path: path != blocked_control,
        )


def test_select_host_resources_rejects_selected_hysteresis_that_disappears(
    tmp_path,
):
    usb_root, iio_root, dev_root, _, gyro_path = complete_fixture(tmp_path)
    disappearing_control = gyro_path / "in_anglvel_hysteresis"

    def remove_hysteresis_during_node_validation(_path):
        if disappearing_control.exists():
            disappearing_control.unlink()
        return True

    with pytest.raises(UsbSelectionError, match="controls are missing"):
        select_host_resources(
            USB_SERIAL_NUMBER,
            usb_sysfs_root=usb_root,
            iio_sysfs_root=iio_root,
            dev_root=dev_root,
            is_character_device=lambda _path: True,
            device_major_minor=fake_device_major_minor,
            has_read_write_access=remove_hysteresis_during_node_validation,
        )


@pytest.mark.parametrize(
    "relative_path",
    [
        "buffer/enable",
        "buffer/length",
        "trigger/current_trigger",
        "in_accel_sampling_frequency",
        "scan_elements/in_accel_y_en",
        "scan_elements/in_timestamp_en",
    ],
)
def test_select_host_resources_rejects_missing_iio_control(
    tmp_path, relative_path
):
    usb_root, iio_root, dev_root, accel_path, _ = complete_fixture(tmp_path)
    (accel_path / relative_path).unlink()

    with pytest.raises(UsbSelectionError, match="controls are missing"):
        select_host_resources(
            USB_SERIAL_NUMBER,
            usb_sysfs_root=usb_root,
            iio_sysfs_root=iio_root,
            dev_root=dev_root,
            is_character_device=lambda _path: True,
            device_major_minor=fake_device_major_minor,
            has_read_write_access=lambda _path: True,
        )


def test_select_host_resources_accepts_missing_optional_hysteresis(tmp_path):
    usb_root, iio_root, dev_root, accel_path, gyro_path = complete_fixture(
        tmp_path
    )
    (accel_path / "in_accel_hysteresis").unlink()
    (gyro_path / "in_anglvel_hysteresis").unlink()

    resources = select_host_resources(
        USB_SERIAL_NUMBER,
        usb_sysfs_root=usb_root,
        iio_sysfs_root=iio_root,
        dev_root=dev_root,
        is_character_device=lambda _path: True,
        device_major_minor=fake_device_major_minor,
        has_read_write_access=lambda _path: True,
    )

    assert all(
        not device.has_hysteresis_control
        for device in resources.iio_devices
    )


def test_select_host_resources_returns_exact_associated_resources(tmp_path):
    resources = select_from_fixture(tmp_path)

    assert [device.name for device in resources.iio_devices] == [
        "accel_3d",
        "gyro_3d",
    ]
    assert all(
        device.has_hysteresis_control
        for device in resources.iio_devices
    )
    assert {node.path.name for node in resources.device_nodes} == {
        "004",
        "video0",
        "media0",
        "iio:device0",
        "iio:device1",
    }


def test_selected_hysteresis_controls_flow_into_exact_profile_rules(
    tmp_path, monkeypatch
):
    resources = select_from_fixture(tmp_path)
    monkeypatch.setattr(
        apparmor_profile,
        "_read_only_discovery_rules",
        lambda _resources: (),
    )

    profile = generate_apparmor_profile(resources, sysfs_root=tmp_path)

    by_name = {device.name: device for device in resources.iio_devices}
    assert (
        f'"{by_name["accel_3d"].sysfs_path}/in_accel_hysteresis" rw,'
        in profile
    )
    assert (
        f'"{by_name["gyro_3d"].sysfs_path}/in_anglvel_hysteresis" rw,'
        in profile
    )


def test_docker_arguments_are_narrow_and_isolated(tmp_path):
    resources = select_from_fixture(tmp_path)
    arguments = docker_arguments(resources)

    assert "--network=none" in arguments
    assert "--env=ROS_DOMAIN_ID=91" in arguments
    assert "--env=ROS_LOCALHOST_ONLY=1" in arguments
    assert "--cap-drop=ALL" in arguments
    assert "--security-opt=no-new-privileges:true" in arguments
    assert "--security-opt=apparmor=pharmarobot-d455-imu" in arguments
    assert len([arg for arg in arguments if arg.startswith("--device=")]) == 3
    assert len([arg for arg in arguments if arg.startswith("--mount=")]) == 4
    assert "--device-cgroup-rule=c 250:0 rwm" in arguments
    assert "--device-cgroup-rule=c 250:1 rwm" in arguments

    joined = "\n".join(arguments)
    for forbidden in (
        "--privileged",
        "--network=host",
        "c 189:*",
        "c 250:*",
        "/dev/roboteq",
        "/dev/lidar",
        "/dev/input",
        "src=/dev,b",
        "src=/sys,dst=/sys",
        "apparmor=unconfined",
    ):
        assert forbidden not in joined

    assert "--device=/dev/bus/usb:/dev/bus/usb" not in joined
    assert "--device=iio:device" not in joined


def test_docker_device_arguments_are_resource_only_and_exact(tmp_path):
    resources = select_from_fixture(tmp_path)
    arguments = docker_device_arguments(resources)

    assert len([arg for arg in arguments if arg.startswith("--device=")]) == 3
    assert len([arg for arg in arguments if arg.startswith("--mount=")]) == 4
    assert "--device-cgroup-rule=c 250:0 rwm" in arguments
    assert "--device-cgroup-rule=c 250:1 rwm" in arguments

    joined = "\n".join(arguments)
    for forbidden in (
        "--privileged",
        "--network",
        "--env",
        "--security-opt",
        "--cap-",
        "apparmor=unconfined",
        "/dev/bus/usb:/dev/bus/usb",
        "src=/dev,dst=/dev",
        "src=/sys,dst=/sys",
        "c 189:*",
        "c 250:*",
    ):
        assert forbidden not in joined
