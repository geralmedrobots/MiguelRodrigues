# Copyright 2026 Medrobots Engineering
#
# Licensed under the Apache License, Version 2.0 (the "License");

"""Offline tests for the production D455 sensor-container lifecycle."""

import importlib.util
import json
from contextlib import nullcontext
import os
from pathlib import Path
import subprocess
import sys

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = REPOSITORY_ROOT / "src" / "realsense_imu"
TOOLS_ROOT = PACKAGE_ROOT / "tools"
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(TOOLS_ROOT))

SPEC = importlib.util.spec_from_file_location(
    "d455_production_container",
    TOOLS_ROOT / "d455_production_container.py",
)
production = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = production
SPEC.loader.exec_module(production)

from realsense_imu.usb_device import DeviceNode  # noqa: E402
from realsense_imu.usb_device import HostResources  # noqa: E402
from realsense_imu.usb_device import IioDevice  # noqa: E402


def resources():
    usb = Path("/sys/devices/pci/usb4/4-3")
    return HostResources(
        usb_sysfs_path=usb,
        device_nodes=(
            DeviceNode(Path("/dev/bus/usb/004/005"), 189, 388),
            DeviceNode(Path("/dev/video0"), 81, 0),
            DeviceNode(Path("/dev/media0"), 237, 0),
            DeviceNode(Path("/dev/iio:device0"), 511, 0),
            DeviceNode(Path("/dev/iio:device1"), 511, 1),
        ),
        iio_devices=(
            IioDevice(
                "accel_3d",
                Path("/dev/iio:device0"),
                usb / "hid-accel" / "iio:device0",
            ),
            IioDevice(
                "gyro_3d",
                Path("/dev/iio:device1"),
                usb / "hid-gyro" / "iio:device1",
            ),
        ),
    )


def result(args, status=0, stdout="", stderr="", timed_out=False):
    return production.CommandResult(
        tuple(args), status, stdout, stderr, timed_out
    )


def owned_inspect(*, running=False, config_hash="a" * 64):
    return {
        "Id": "b" * 64,
        "Image": "sha256:" + "1" * 64,
        "Name": f"/{production.PRODUCTION_CONTAINER_NAME}",
        "Config": {
            "Labels": {
                production.PRODUCTION_LABEL_KEY: "true",
                production.OWNER_LABEL_KEY: production.OWNER_LABEL_VALUE,
                production.ROLE_LABEL_KEY: production.ROLE_LABEL_VALUE,
                production.CONFIG_LABEL_KEY: config_hash,
            }
        },
        "State": {
            "Running": running,
            "Status": "running" if running else "exited",
        },
    }


def contract_inspect(config_hash="a" * 64, *, running=False):
    selected = resources()
    value = owned_inspect(running=running, config_hash=config_hash)
    value.update(
        {
            "Image": "sha256:" + "1" * 64,
            "HostConfig": {
                "NetworkMode": "host",
                "Privileged": False,
                "CapDrop": ["ALL"],
                "SecurityOpt": [
                    "no-new-privileges:true",
                    "apparmor=pharmarobot-d455-imu",
                ],
                "Init": True,
                "RestartPolicy": {"Name": "no"},
                "Devices": [
                    {
                        "PathOnHost": str(node.path),
                        "PathInContainer": str(node.path),
                        "CgroupPermissions": "rwm",
                    }
                    for node in selected.device_nodes
                    if not node.path.name.startswith("iio:device")
                ],
                "DeviceCgroupRules": [
                    f"c {node.major}:{node.minor} rwm"
                    for node in selected.device_nodes
                    if node.path.name.startswith("iio:device")
                ],
            },
            "Mounts": [
                {
                    "Type": "bind",
                    "Source": str(path),
                    "Destination": str(path),
                    "Mode": "",
                    "RW": True,
                    "Propagation": "rprivate",
                }
                for path in (
                    Path("/dev/iio:device0"),
                    Path("/dev/iio:device1"),
                    selected.iio_devices[0].sysfs_path,
                    selected.iio_devices[1].sysfs_path,
                )
            ],
        }
    )
    value["Config"]["Env"] = [
        "ROS_DOMAIN_ID=0",
        "ROS_LOCALHOST_ONLY=0",
        "RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
        "FASTDDS_BUILTIN_TRANSPORTS=UDPv4",
        "D455_SERIAL_NUMBER=146222250608",
    ]
    return value


def production_config(tmp_path, *, image_id=None):
    selected_image_id = image_id or "sha256:" + "1" * 64
    tmp_path.mkdir(parents=True, exist_ok=True)
    manifest = tmp_path / "d455-sensor-image.env"
    manifest.write_text(
        "\n".join(
            (
                f"IMAGE={production.PRODUCTION_IMAGE}",
                f"IMAGE_ID={selected_image_id}",
                f"SOURCE_MANIFEST_SHA256={'2' * 64}",
                f"BASE_IMAGE_ID=sha256:{'3' * 64}",
                "",
            )
        ),
        encoding="utf-8",
    )
    return production.ProductionConfig(
        evidence_root=tmp_path / "evidence",
        image_manifest=manifest,
        ownership_record=tmp_path / "ownership.json",
    )


def write_ownership_record(config, inspect):
    labels = inspect["Config"]["Labels"]
    config.ownership_record.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "container_name": config.container_name,
                "container_id": inspect["Id"],
                "image_id": inspect["Image"],
                "config_sha256": labels[production.CONFIG_LABEL_KEY],
                "owner": production.OWNER_LABEL_VALUE,
            }
        ),
        encoding="utf-8",
    )


class SequenceRunner:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []

    def run(self, args, *, timeout=production.COMMAND_TIMEOUT_SECONDS):
        self.calls.append((tuple(args), timeout))
        response = self.responses.pop(0)
        if callable(response):
            return response(args)
        return response


def test_production_arguments_are_host_dds_and_exact_d455_only():
    config = production.ProductionConfig()
    arguments = production.production_docker_arguments(
        resources(), config=config, config_sha256="a" * 64
    )
    joined = "\n".join(arguments)

    assert "--init" in arguments
    assert "--network=host" in arguments
    assert "--cap-drop=ALL" in arguments
    assert "--security-opt=no-new-privileges:true" in arguments
    assert (
        "--security-opt=apparmor=pharmarobot-d455-imu" in arguments
    )
    assert "ROS_DOMAIN_ID=0" in arguments
    assert "ROS_LOCALHOST_ONLY=0" in arguments
    assert "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" in arguments
    assert "FASTDDS_BUILTIN_TRANSPORTS=UDPv4" in arguments
    assert "--network=none" not in arguments
    for forbidden in (
        "--privileged",
        "apparmor=unconfined",
        "/dev/roboteq",
        "/dev/ttyUSB",
        "/dev/input",
    ):
        assert forbidden not in joined


def test_validation_policy_remains_separate():
    from realsense_imu.usb_device import docker_arguments

    validation = docker_arguments(resources())
    production_args = production.production_docker_arguments(
        resources(),
        config=production.ProductionConfig(),
        config_sha256="a" * 64,
    )

    assert "--network=none" in validation
    assert "--env=ROS_DOMAIN_ID=91" in validation
    assert "--env=ROS_LOCALHOST_ONLY=1" in validation
    assert "--network=none" not in production_args
    assert production.PRODUCTION_LABEL in production_args
    assert production.VALIDATION_LABEL_KEY not in "\n".join(
        production_args
    )


def test_fixed_name_and_dds_configuration_reject_drift():
    with pytest.raises(production.ProductionContainerError):
        production.ProductionConfig(container_name="other").validate()
    with pytest.raises(production.ProductionContainerError):
        production.ProductionConfig(
            rmw_implementation="rmw_cyclonedds_cpp"
        ).validate()
    with pytest.raises(production.ProductionContainerError):
        production.ProductionConfig(
            fastdds_builtin_transports="DEFAULT"
        ).validate()
    with pytest.raises(production.ProductionContainerError):
        production.ProductionConfig(ros_domain_id=233).validate()


def test_config_fingerprint_includes_fastdds_transport():
    common = {
        "image_id": "sha256:" + "1" * 64,
        "resource_fingerprint": "f" * 64,
    }
    udp_hash = production.config_fingerprint(
        config=production.ProductionConfig(),
        **common,
    )
    other_hash = production.config_fingerprint(
        config=production.ProductionConfig(
            fastdds_builtin_transports="DEFAULT"
        ),
        **common,
    )

    assert udp_hash != other_hash


def test_create_command_is_pinned_and_deterministic():
    command_a = production.container_create_command(
        resources(),
        config=production.ProductionConfig(),
        image_id="sha256:" + "1" * 64,
        config_sha256="2" * 64,
    )
    command_b = production.container_create_command(
        resources(),
        config=production.ProductionConfig(),
        image_id="sha256:" + "1" * 64,
        config_sha256="2" * 64,
    )
    assert command_a == command_b
    assert command_a[:4] == [
        "docker",
        "create",
        "--name",
        production.PRODUCTION_CONTAINER_NAME,
    ]
    assert command_a[-1] == "sha256:" + "1" * 64


def test_sensor_image_identity_requires_reviewed_labels(tmp_path):
    image = {
        "Id": "sha256:" + "1" * 64,
        "Config": {
            "Labels": {
                production.SENSOR_IMAGE_LABEL_KEY: "true",
                production.SOURCE_MANIFEST_LABEL_KEY: "2" * 64,
                production.BASE_IMAGE_LABEL_KEY: "sha256:" + "3" * 64,
            }
        },
    }
    runner = SequenceRunner(
        [result((), stdout=json.dumps([image]))]
    )
    lifecycle = production.ProductionLifecycle(
        production_config(tmp_path),
        runner=runner,
        select_resources=lambda _serial: resources(),
    )
    assert lifecycle.image_id() == "sha256:" + "1" * 64

    image["Config"]["Labels"] = {}
    runner = SequenceRunner(
        [result((), stdout=json.dumps([image]))]
    )
    with pytest.raises(
        production.ProductionContainerError, match="reviewed"
    ):
        production.ProductionLifecycle(
            production_config(tmp_path), runner=runner
        ).image_id()


def test_sensor_image_identity_rejects_manifest_mismatch(tmp_path):
    image = {
        "Id": "sha256:" + "1" * 64,
        "Config": {
            "Labels": {
                production.SENSOR_IMAGE_LABEL_KEY: "true",
                production.SOURCE_MANIFEST_LABEL_KEY: "4" * 64,
                production.BASE_IMAGE_LABEL_KEY: "sha256:" + "3" * 64,
            }
        },
    }
    lifecycle = production.ProductionLifecycle(
        production_config(tmp_path),
        runner=SequenceRunner(
            [result((), stdout=json.dumps([image]))]
        ),
    )
    with pytest.raises(
        production.ProductionContainerError, match="reviewed"
    ):
        lifecycle.image_id()


def test_foreign_container_is_never_owned():
    foreign = owned_inspect()
    foreign["Config"]["Labels"][production.OWNER_LABEL_KEY] = "other"
    assert not production.is_owned_container(foreign)


def test_inspected_container_contract_accepts_only_exact_resources():
    lifecycle = production.ProductionLifecycle(
        production.ProductionConfig(), runner=SequenceRunner([])
    )
    lifecycle.verify_container_contract(
        contract_inspect(),
        resources=resources(),
        image_id="sha256:" + "1" * 64,
        config_sha256="a" * 64,
    )

    broad = contract_inspect()
    broad["HostConfig"]["Privileged"] = True
    with pytest.raises(
        production.ProductionContainerError, match="isolation"
    ):
        lifecycle.verify_container_contract(
            broad,
            resources=resources(),
            image_id="sha256:" + "1" * 64,
            config_sha256="a" * 64,
        )

    redirected = contract_inspect()
    redirected["HostConfig"]["Devices"][0]["PathInContainer"] = "/dev/null"
    with pytest.raises(
        production.ProductionContainerError, match="resource scope"
    ):
        lifecycle.verify_container_contract(
            redirected,
            resources=resources(),
            image_id="sha256:" + "1" * 64,
            config_sha256="a" * 64,
        )

    read_only_mount = contract_inspect()
    read_only_mount["Mounts"][0]["RW"] = False
    with pytest.raises(
        production.ProductionContainerError, match="resource scope"
    ):
        lifecycle.verify_container_contract(
            read_only_mount,
            resources=resources(),
            image_id="sha256:" + "1" * 64,
            config_sha256="a" * 64,
        )


@pytest.mark.parametrize("transport", (None, "", "DEFAULT"))
def test_inspected_container_contract_rejects_fastdds_transport_drift(
    transport,
):
    lifecycle = production.ProductionLifecycle(
        production.ProductionConfig(), runner=SequenceRunner([])
    )
    inspected = contract_inspect()
    inspected["Config"]["Env"] = [
        entry
        for entry in inspected["Config"]["Env"]
        if not entry.startswith("FASTDDS_BUILTIN_TRANSPORTS=")
    ]
    if transport is not None:
        inspected["Config"]["Env"].append(
            f"FASTDDS_BUILTIN_TRANSPORTS={transport}"
        )

    with pytest.raises(
        production.ProductionContainerError,
        match="DDS contract drift",
    ):
        lifecycle.verify_container_contract(
            inspected,
            resources=resources(),
            image_id="sha256:" + "1" * 64,
            config_sha256="a" * 64,
        )


def test_active_validation_container_blocks_startup():
    runner = SequenceRunner(
        [
            result(
                (),
                stdout="abc pharma_realsense_imu_validation\n",
            )
        ]
    )
    with pytest.raises(
        production.ProductionContainerError,
        match="active D455 validation",
    ):
        production.assert_no_active_validation_container(runner)


def test_duplicate_or_foreign_production_container_is_rejected():
    runner = SequenceRunner(
        [result((), stdout="one\ntwo\n")]
    )
    with pytest.raises(
        production.ProductionContainerError, match="ambiguous"
    ):
        production.assert_unique_production_container(
            runner, production.PRODUCTION_CONTAINER_NAME
        )


def test_running_census_excludes_only_expected_valid_owner():
    expected = contract_inspect(running=True)
    runner = SequenceRunner(
        [
            result((), stdout=expected["Id"] + "\n"),
            result((), stdout=json.dumps([expected])),
        ]
    )

    production.assert_no_foreign_running_d455_containers(
        runner,
        resources(),
        expected_container_id=expected["Id"],
    )

    assert all(call[0][:2] != ("docker", "top") for call in runner.calls)


def test_running_census_rejects_foreign_exact_d455_resource():
    foreign_id = "c" * 64
    foreign = {
        "Id": foreign_id,
        "Name": "/foreign_sensor",
        "Config": {"Labels": {"third.party": "camera"}},
        "HostConfig": {
            "Devices": [
                {
                    "PathOnHost": "/dev/iio:device0",
                    "PathInContainer": "/dev/iio:device0",
                }
            ],
        },
        "Mounts": [],
    }
    runner = SequenceRunner(
        [
            result((), stdout=foreign_id + "\n"),
            result((), stdout=json.dumps([foreign])),
            result((), stdout="PID PPID STAT ARGS\n1 0 S sleep infinity\n"),
        ]
    )

    with pytest.raises(production.ProductionContainerError) as exc_info:
        production.assert_no_foreign_running_d455_containers(
            runner,
            resources(),
            expected_container_id=None,
        )
    error = str(exc_info.value)
    assert "foreign_sensor" in error
    assert '"third.party":"camera"' in error
    assert "device:/dev/iio:device0" in error
    assert "selected:/dev/iio:device0" in error

    mutating = {"rm", "restart", "stop", "start"}
    assert all(
        len(call[0]) < 2 or call[0][1] not in mutating
        for call in runner.calls
    )
    assert all(
        production.MAIN_CONTAINER_NAME not in call[0]
        for call in runner.calls
    )


def test_running_census_rejects_foreign_exact_d455_sysfs_mount():
    foreign_id = "7" * 64
    selected_sysfs = resources().iio_devices[0].sysfs_path
    foreign = {
        "Id": foreign_id,
        "Name": "/unlabelled_iio_owner",
        "Config": {"Labels": {}},
        "HostConfig": {"Devices": [], "Privileged": False},
        "Mounts": [
            {
                "Source": str(selected_sysfs),
                "Destination": str(selected_sysfs),
            }
        ],
    }
    runner = SequenceRunner(
        [
            result((), stdout=foreign_id + "\n"),
            result((), stdout=json.dumps([foreign])),
            result((), stdout="PID PPID STAT ARGS\n1 0 S sleep infinity\n"),
        ]
    )

    with pytest.raises(production.ProductionContainerError) as exc_info:
        production.assert_no_foreign_running_d455_containers(
            runner,
            resources(),
            expected_container_id=None,
        )
    error = str(exc_info.value)
    assert "unlabelled_iio_owner" in error
    assert f"mount:{selected_sysfs}" in error
    assert f"selected:{selected_sysfs}" in error


def test_running_census_rejects_narrow_realsense_process():
    foreign_id = "d" * 64
    foreign = {
        "Id": foreign_id,
        "Name": "/foreign_runtime",
        "Config": {"Labels": {}},
        "HostConfig": {"Devices": []},
        "Mounts": [],
    }
    runner = SequenceRunner(
        [
            result((), stdout=foreign_id + "\n"),
            result((), stdout=json.dumps([foreign])),
            result(
                (),
                stdout=(
                    "PID PPID STAT ARGS\n"
                    "7 1 Sl /opt/ros/humble/lib/realsense2_camera/"
                    "realsense2_camera_node --ros-args "
                    "-r __node:=d455 -r __ns:=/realsense\n"
                ),
            ),
        ]
    )

    with pytest.raises(
        production.ProductionContainerError,
        match="process:realsense2_camera_node",
    ):
        production.assert_no_foreign_running_d455_containers(
            runner,
            resources(),
            expected_container_id=None,
        )


def test_running_census_ignores_unrelated_official_realsense_wrapper():
    unrelated_id = "f" * 64
    unrelated = {
        "Id": unrelated_id,
        "Name": "/unrelated_realsense",
        "Config": {"Labels": {}},
        "HostConfig": {"Devices": [], "Privileged": False},
        "Mounts": [],
    }
    runner = SequenceRunner(
        [
            result((), stdout=unrelated_id + "\n"),
            result((), stdout=json.dumps([unrelated])),
            result(
                (),
                stdout=(
                    "PID PPID STAT ARGS\n"
                    "7 1 Sl /opt/ros/humble/lib/realsense2_camera/"
                    "realsense2_camera_node --ros-args "
                    "-r __node:=warehouse_camera "
                    "-r __ns:=/warehouse\n"
                ),
            ),
        ]
    )

    production.assert_no_foreign_running_d455_containers(
        runner,
        resources(),
        expected_container_id=None,
    )


def test_running_census_rejects_foreign_privileged_container():
    foreign_id = "9" * 64
    foreign = {
        "Id": foreign_id,
        "Name": "/privileged_toolbox",
        "Config": {"Labels": {"purpose": "maintenance"}},
        "HostConfig": {"Devices": [], "Privileged": True},
        "Mounts": [],
    }
    runner = SequenceRunner(
        [
            result((), stdout=foreign_id + "\n"),
            result((), stdout=json.dumps([foreign])),
        ]
    )

    with pytest.raises(production.ProductionContainerError) as exc_info:
        production.assert_no_foreign_running_d455_containers(
            runner,
            resources(),
            expected_container_id=None,
        )
    error = str(exc_info.value)
    assert "isolation:Privileged=true" in error
    assert "privileged and has ambiguous broad D455 access" in error
    assert all(call[0][:2] != ("docker", "top") for call in runner.calls)


def test_running_census_avoids_generic_camera_and_ros_false_positives():
    unrelated_id = "e" * 64
    unrelated = {
        "Id": unrelated_id,
        "Name": "/camera_analytics",
        "Config": {
            "Labels": {
                "description": "generic USB camera and ROS analytics"
            }
        },
        "HostConfig": {
            "Devices": [
                {
                    "PathOnHost": "/dev/video99",
                    "PathInContainer": "/dev/video99",
                }
            ],
        },
        "Mounts": [
            {
                "Source": "/sys/devices/unrelated-camera",
                "Destination": "/sys/devices/unrelated-camera",
            }
        ],
    }
    runner = SequenceRunner(
        [
            result((), stdout=unrelated_id + "\n"),
            result((), stdout=json.dumps([unrelated])),
            result(
                (),
                stdout=(
                    "PID PPID STAT ARGS\n"
                    "1 0 S camera_monitor --ros-args\n"
                ),
            ),
        ]
    )

    production.assert_no_foreign_running_d455_containers(
        runner,
        resources(),
        expected_container_id=None,
    )


def test_ensure_conflict_stops_before_profile_probe_create_or_start(
    monkeypatch, tmp_path
):
    foreign_id = "8" * 64
    foreign = {
        "Id": foreign_id,
        "Name": "/foreign_d455_owner",
        "Config": {"Labels": {}},
        "HostConfig": {
            "Devices": [
                {
                    "PathOnHost": "/dev/iio:device0",
                    "PathInContainer": "/dev/iio:device0",
                }
            ],
            "Privileged": False,
        },
        "Mounts": [],
    }
    runner = SequenceRunner(
        [
            result((), stdout=""),  # no validation container
            result((), stdout=""),  # no labelled production container
            result((), status=1, stderr="No such object"),  # main
            result((), status=1, stdout="[]"),  # fixed production name
            result((), stdout=foreign_id + "\n"),  # running census
            result((), stdout=json.dumps([foreign])),
            result((), stdout="PID PPID STAT ARGS\n1 0 S sleep infinity\n"),
        ]
    )
    lifecycle = production.ProductionLifecycle(
        production_config(tmp_path),
        runner=runner,
        select_resources=lambda _serial: resources(),
        lock_factory=lambda _operation: nullcontext(),
    )
    monkeypatch.setattr(lifecycle, "image_id", lambda: "sha256:" + "1" * 64)
    monkeypatch.setattr(
        production, "verify_serial_bounded", lambda *_args, **_kwargs: None
    )
    monkeypatch.setattr(
        production, "validate_resource_set", lambda _resources: None
    )
    monkeypatch.setattr(
        production,
        "generate_apparmor_profile",
        lambda _resources: pytest.fail("profile handling must not run"),
    )
    monkeypatch.setattr(
        lifecycle,
        "final_access_probe",
        lambda *_args, **_kwargs: pytest.fail(
            "access probe must not run"
        ),
    )

    with pytest.raises(
        production.ProductionContainerError,
        match="foreign running D455 container conflicts",
    ):
        lifecycle.ensure_created(
            authorize_profile_reload=False,
            authorize_recreate=False,
        )

    mutating = {"create", "run", "start", "stop", "restart", "rm"}
    assert all(
        len(call[0]) < 2 or call[0][1] not in mutating
        for call in runner.calls
    )


def test_existing_legacy_main_blocks_production_before_preflight(
    monkeypatch, tmp_path
):
    legacy_main = {
        "Id": "c" * 64,
        "Name": "/pharma_container",
        "Config": {"Env": ["D455_IMU_AVAILABLE=1"]},
        "HostConfig": {},
        "State": {"Running": True},
    }
    runner = SequenceRunner(
        [
            result((), stdout=""),
            result((), stdout=""),
            result((), stdout=json.dumps([legacy_main])),
        ]
    )
    lifecycle = production.ProductionLifecycle(
        production_config(tmp_path),
        runner=runner,
        lock_factory=lambda _operation: nullcontext(),
    )
    monkeypatch.setattr(
        lifecycle,
        "preflight",
        lambda **_kwargs: pytest.fail("preflight must not run"),
    )
    monkeypatch.setattr(
        lifecycle,
        "image_id",
        lambda: pytest.fail("image lookup must not run"),
    )
    with pytest.raises(
        production.ProductionContainerError, match="legacy D455"
    ):
        lifecycle.ensure_created(
            authorize_profile_reload=False,
            authorize_recreate=False,
        )
    joined = "\n".join(" ".join(call[0]) for call in runner.calls)
    assert "docker create" not in joined
    assert "docker run" not in joined
    assert "docker rm" not in joined


def test_running_main_d455_process_is_rejected():
    clean_main = {
        "Id": "c" * 64,
        "Name": "/pharma_container",
        "Config": {"Env": []},
        "HostConfig": {},
        "State": {"Running": True},
    }
    runner = SequenceRunner(
        [
            result((), stdout=json.dumps([clean_main])),
            result(
                (),
                stdout=(
                    "1 0 S tail -f /dev/null\n"
                    "42 1 Sl /opt/ros/humble/lib/"
                    "realsense2_camera/realsense2_camera_node\n"
                ),
            ),
        ]
    )
    with pytest.raises(
        production.ProductionContainerError,
        match="legacy D455 process",
    ):
        production.assert_main_container_has_no_d455(runner)


def test_absent_container_with_stale_ownership_blocks_start(
    monkeypatch, tmp_path
):
    config = production_config(tmp_path)
    config.ownership_record.write_text("{}\n", encoding="utf-8")
    runner = SequenceRunner(
        [
            result((), stdout=""),
            result((), stdout=""),
            result(
                (),
                status=1,
                stdout="[]",
                stderr="Error: No such object: pharma_container",
            ),
            result((), status=1, stdout="[]"),
        ]
    )
    lifecycle = production.ProductionLifecycle(
        config,
        runner=runner,
        lock_factory=lambda _operation: nullcontext(),
    )
    monkeypatch.setattr(
        lifecycle,
        "preflight",
        lambda **_kwargs: pytest.fail("preflight must not run"),
    )
    monkeypatch.setattr(
        lifecycle,
        "image_id",
        lambda: pytest.fail("image lookup must not run"),
    )
    with pytest.raises(
        production.ProductionContainerError,
        match="ownership record exists",
    ):
        lifecycle.ensure_created(
            authorize_profile_reload=False,
            authorize_recreate=False,
        )
    joined = "\n".join(" ".join(call[0]) for call in runner.calls)
    assert "docker create" not in joined
    assert "docker rm" not in joined


def test_stop_is_idempotent_for_absent_and_stopped_owned_container(
    tmp_path,
):
    absent_config = production_config(tmp_path / "absent")
    absent = SequenceRunner([result((), status=1, stdout="[]")])
    production.ProductionLifecycle(
        absent_config,
        runner=absent,
        lock_factory=lambda _operation: nullcontext(),
    ).stop()
    assert len(absent.calls) == 1

    stopped_inspect = owned_inspect()
    stopped_config = production_config(tmp_path / "stopped")
    write_ownership_record(stopped_config, stopped_inspect)
    stopped = SequenceRunner(
        [
            result((), stdout=json.dumps([stopped_inspect])),
            result((), stdout=json.dumps([stopped_inspect])),
        ]
    )
    production.ProductionLifecycle(
        stopped_config,
        runner=stopped,
        lock_factory=lambda _operation: nullcontext(),
    ).stop()
    assert len(stopped.calls) == 2


def test_stop_rejects_foreign_container_without_docker_stop(tmp_path):
    foreign = owned_inspect(running=True)
    foreign["Config"]["Labels"] = {}
    runner = SequenceRunner(
        [
            result((), stdout=json.dumps([foreign])),
            result((), stdout=json.dumps([foreign])),
        ]
    )
    with pytest.raises(
        production.ProductionContainerError, match="foreign"
    ):
        production.ProductionLifecycle(
            production_config(tmp_path),
            runner=runner,
            lock_factory=lambda _operation: nullcontext(),
        ).stop()
    assert all(call[0][:2] != ("docker", "stop") for call in runner.calls)


def test_partial_create_cleanup_targets_only_new_owned_container(
    monkeypatch, tmp_path
):
    image_id = "sha256:" + "1" * 64
    desired_hash = production.config_fingerprint(
        config=production_config(tmp_path),
        image_id=image_id,
        resource_fingerprint="f" * 64,
    )
    partial = owned_inspect(config_hash=desired_hash)
    partial["Image"] = image_id
    runner = SequenceRunner(
        [
            result((), stdout=""),  # validation list
            result((), stdout=""),  # production list
            result((), status=1, stdout="[]"),  # fixed-name inspect
            result((), status=1, stderr="create failed"),  # create
            result((), stdout=json.dumps([partial])),
            result((), stdout=json.dumps([partial])),
            result((), stdout="removed"),  # exact cleanup
            result((), status=1, stdout="[]"),  # absence proof
        ]
    )
    config = production_config(tmp_path)
    lifecycle = production.ProductionLifecycle(
        config,
        runner=runner,
        lock_factory=lambda _operation: nullcontext(),
    )
    monkeypatch.setattr(
        production,
        "assert_main_container_has_no_d455",
        lambda _runner: None,
    )
    monkeypatch.setattr(
        lifecycle,
        "preflight",
        lambda **unused: production.PreflightPlan(
            resources(), "f" * 64, None, ""
        ),
    )
    monkeypatch.setattr(
        lifecycle, "final_access_probe", lambda *args, **kwargs: None
    )
    monkeypatch.setattr(
        lifecycle, "image_id", lambda: image_id
    )

    with pytest.raises(
        production.ProductionContainerError, match="partial state"
    ):
        lifecycle.ensure_created(
            authorize_profile_reload=False,
            authorize_recreate=False,
        )
    assert runner.calls[-2][0] == (
        "docker",
        "rm",
        "-f",
        partial["Id"],
    )
    assert runner.calls[-1][0] == (
        "docker",
        "inspect",
        partial["Id"],
    )


def test_new_container_cleanup_fails_when_remove_or_absence_is_unproven(
    tmp_path,
):
    current = owned_inspect()
    config = production_config(tmp_path)
    write_ownership_record(config, current)
    lifecycle = production.ProductionLifecycle(
        config,
        runner=SequenceRunner(
            [
                result((), stdout=json.dumps([current])),
                result((), status=2, stderr="rm failed"),
            ]
        ),
    )
    with pytest.raises(
        production.ProductionContainerError, match="cleanup command failed"
    ):
        lifecycle.cleanup_new_container(
            container_id=current["Id"],
            image_id=current["Image"],
            config_sha256="a" * 64,
        )

    lifecycle = production.ProductionLifecycle(
        config,
        runner=SequenceRunner(
            [
                result((), stdout=json.dumps([current])),
                result((), stdout=current["Id"]),
                result((), stdout=json.dumps([current])),
            ]
        ),
    )
    with pytest.raises(
        production.ProductionContainerError, match="absence was not proven"
    ):
        lifecycle.cleanup_new_container(
            container_id=current["Id"],
            image_id=current["Image"],
            config_sha256="a" * 64,
        )


def test_running_exact_container_start_is_idempotent(monkeypatch, tmp_path):
    state = {"locked": False, "released": False}

    class TrackingLock:
        def __enter__(self):
            state["locked"] = True

        def __exit__(self, *_args):
            state["locked"] = False
            state["released"] = True

    runner = SequenceRunner([])
    lifecycle = production.ProductionLifecycle(
        production_config(tmp_path),
        runner=runner,
        lock_factory=lambda _operation: TrackingLock(),
    )

    def exact_running(**_kwargs):
        assert state["locked"] is True
        return owned_inspect(running=True)

    monkeypatch.setattr(
        lifecycle,
        "_ensure_created_locked",
        exact_running,
    )
    assert (
        lifecycle.start(
            authorize_profile_reload=False,
            authorize_recreate=False,
            attach=False,
        )
        == 0
    )
    assert state["released"] is True
    assert runner.calls == []


def test_restart_is_one_locked_stop_prepare_start_transition(
    monkeypatch, tmp_path
):
    state = {"locked": False, "stopped": False}

    class TrackingLock:
        def __enter__(self):
            state["locked"] = True

        def __exit__(self, *_args):
            state["locked"] = False

    runner = SequenceRunner([result((), stdout="started")])
    lifecycle = production.ProductionLifecycle(
        production_config(tmp_path),
        runner=runner,
        lock_factory=lambda _operation: TrackingLock(),
    )

    def stop_locked():
        assert state["locked"] is True
        state["stopped"] = True

    def ensure_locked(**_kwargs):
        assert state["locked"] is True
        assert state["stopped"] is True
        return owned_inspect(running=False)

    monkeypatch.setattr(lifecycle, "_stop_locked", stop_locked)
    monkeypatch.setattr(
        lifecycle, "_ensure_created_locked", ensure_locked
    )
    assert (
        lifecycle.restart(
            authorize_profile_reload=False,
            authorize_recreate=False,
        )
        == 0
    )
    assert state["locked"] is False
    assert runner.calls == [
        (
            ("docker", "start", "b" * 64),
            production.COMMAND_TIMEOUT_SECONDS,
        )
    ]


def test_existing_exact_container_requires_immutable_ownership_record(
    monkeypatch, tmp_path
):
    image_id = "sha256:" + "1" * 64
    config = production_config(tmp_path)
    desired_hash = production.config_fingerprint(
        config=config,
        image_id=image_id,
        resource_fingerprint="f" * 64,
    )
    existing = contract_inspect(
        desired_hash, running=True
    )
    runner = SequenceRunner(
        [
            result((), stdout=""),
            result(
                (),
                stdout=production.PRODUCTION_CONTAINER_NAME + "\n",
            ),
            result((), stdout=json.dumps([existing])),
            result((), stdout=json.dumps([existing])),
        ]
    )
    lifecycle = production.ProductionLifecycle(
        config,
        runner=runner,
        lock_factory=lambda _operation: nullcontext(),
    )
    monkeypatch.setattr(
        production,
        "assert_main_container_has_no_d455",
        lambda _runner: None,
    )
    monkeypatch.setattr(
        lifecycle,
        "preflight",
        lambda **_kwargs: production.PreflightPlan(
            resources(), "f" * 64, None, ""
        ),
    )
    monkeypatch.setattr(lifecycle, "image_id", lambda: image_id)
    with pytest.raises(
        production.ProductionContainerError,
        match="ownership record is unavailable",
    ):
        lifecycle.ensure_created(
            authorize_profile_reload=False,
            authorize_recreate=False,
        )
    assert all(call[0][:2] != ("docker", "rm") for call in runner.calls)


def test_final_access_probe_is_production_owned_and_not_validation(
    monkeypatch, tmp_path
):
    runner = SequenceRunner(
        [
            result((), stdout=""),
            result((), status=1, stdout="[]"),
            result((), stdout="probe passed"),
            result((), status=1, stdout="[]"),
        ]
    )
    lifecycle = production.ProductionLifecycle(
        production_config(tmp_path),
        runner=runner,
        select_resources=lambda _serial: resources(),
    )
    monkeypatch.setattr(
        production, "verify_serial_bounded", lambda *_args, **_kwargs: None
    )
    monkeypatch.setattr(
        production,
        "assert_resources_unchanged",
        lambda *_args, **_kwargs: None,
    )
    monkeypatch.setattr(
        production, "validate_current_audit", lambda *_args, **_kwargs: None
    )
    plan = production.PreflightPlan(
        resources(),
        "f" * 64,
        production.Evidence(tmp_path / "probe-evidence"),
        "2026-07-27 15:00:00 UTC",
    )
    lifecycle.final_access_probe(
        plan, image_id="sha256:" + "1" * 64
    )
    probe_command = runner.calls[2][0]
    joined = "\n".join(probe_command)
    assert probe_command[:2] == ("docker", "run")
    assert "--rm" in probe_command
    assert "--network=host" in probe_command
    assert production.ACCESS_PROBE_LABEL_KEY in joined
    assert production.VALIDATION_LABEL_KEY not in joined
    assert production.PRODUCTION_LABEL not in probe_command
    assert "--network=none" not in probe_command
    assert "FASTDDS_BUILTIN_TRANSPORTS=UDPv4" in probe_command


def test_stopped_configuration_drift_requires_explicit_recreation(
    monkeypatch, tmp_path
):
    runner = SequenceRunner(
        [
            result((), stdout=""),  # validation list
            result(
                (),
                stdout=production.PRODUCTION_CONTAINER_NAME + "\n",
            ),
            result((), stdout=json.dumps([owned_inspect()])),
        ]
    )
    lifecycle = production.ProductionLifecycle(
        production_config(tmp_path),
        runner=runner,
        lock_factory=lambda _operation: nullcontext(),
    )
    monkeypatch.setattr(
        production,
        "assert_main_container_has_no_d455",
        lambda _runner: None,
    )
    monkeypatch.setattr(
        lifecycle,
        "preflight",
        lambda **unused: production.PreflightPlan(
            resources(), "f" * 64, None, ""
        ),
    )
    monkeypatch.setattr(
        lifecycle, "image_id", lambda: "sha256:" + "1" * 64
    )
    with pytest.raises(
        production.ProductionContainerError,
        match="explicit recreation authorization",
    ):
        lifecycle.ensure_created(
            authorize_profile_reload=False,
            authorize_recreate=False,
        )


def test_profile_reload_authorization_requires_running_sensor_stopped(
    monkeypatch, tmp_path
):
    existing = owned_inspect(running=True)
    runner = SequenceRunner(
        [
            result((), stdout=""),
            result(
                (),
                stdout=production.PRODUCTION_CONTAINER_NAME + "\n",
            ),
            result((), stdout=json.dumps([existing])),
        ]
    )
    lifecycle = production.ProductionLifecycle(
        production_config(tmp_path),
        runner=runner,
        lock_factory=lambda _operation: nullcontext(),
    )
    monkeypatch.setattr(
        production,
        "assert_main_container_has_no_d455",
        lambda _runner: None,
    )
    monkeypatch.setattr(
        lifecycle,
        "preflight",
        lambda **_kwargs: pytest.fail("preflight must not run"),
    )
    monkeypatch.setattr(
        lifecycle,
        "image_id",
        lambda: pytest.fail("image lookup must not run"),
    )
    with pytest.raises(
        production.ProductionContainerError,
        match="before authorizing an AppArmor profile reload",
    ):
        lifecycle.ensure_created(
            authorize_profile_reload=True,
            authorize_recreate=False,
        )
    joined = "\n".join(" ".join(call[0]) for call in runner.calls)
    assert "apparmor_parser" not in joined
    assert "docker start" not in joined


def test_migration_check_is_read_only_and_detects_legacy_main(capsys):
    legacy = {
        "Id": "c" * 64,
        "Name": "/pharma_container",
        "Config": {"Env": ["D455_IMU_AVAILABLE=1"]},
        "HostConfig": {"Devices": [{"PathOnHost": "/dev/iio:device0"}]},
    }
    runner = SequenceRunner(
        [result((), stdout=json.dumps([legacy]))]
    )
    status = production.ProductionLifecycle(
        production.ProductionConfig(), runner=runner
    ).migration_check()
    assert status == 2
    output = capsys.readouterr().out
    assert f"MAIN_CONTAINER_ID={'c' * 64}" in output
    assert "LEGACY_D455_ACCESS=1" in output
    assert runner.calls == [
        (("docker", "inspect", "pharma_container"), 30.0)
    ]


@pytest.mark.parametrize(
    ("health", "expected_status", "ready"),
    (("healthy", 0, "1"), ("unhealthy", 4, "0")),
)
def test_status_succeeds_only_when_container_is_ready(
    monkeypatch, capsys, health, expected_status, ready
):
    state = owned_inspect(running=True)
    state["State"]["Health"] = {"Status": health}

    class FakeLifecycle:
        config = production.ProductionConfig()
        runner = SequenceRunner([])

        @staticmethod
        def inspect_optional():
            return state

        @staticmethod
        def require_recorded_owned():
            return state

    monkeypatch.setattr(
        production, "ProductionLifecycle", lambda _config: FakeLifecycle()
    )
    monkeypatch.setattr(
        production,
        "assert_unique_production_container",
        lambda *_args: None,
    )
    assert production.main(["status"]) == expected_status
    assert f"D455_SENSOR_READY={ready}" in capsys.readouterr().out


def read(relative):
    return (REPOSITORY_ROOT / relative).read_text(encoding="utf-8")


def test_sensor_image_and_entrypoint_exclude_control_stack():
    dockerfile = read("deployment/docker/Dockerfile.d455_sensor")
    entrypoint = read("deployment/scripts/d455_sensor_entrypoint.sh")
    combined = dockerfile + entrypoint

    assert "COPY src/realsense_imu" in dockerfile
    assert "ros-humble-realsense2-camera" in dockerfile
    assert (
        "exec ros2 launch realsense_imu robot_sensors.launch.py"
        in entrypoint
    )
    assert "FASTDDS_BUILTIN_TRANSPORTS=UDPv4" in dockerfile
    assert (
        '${FASTDDS_BUILTIN_TRANSPORTS:-}" != "UDPv4"'
        in entrypoint
    )
    assert entrypoint.index("FASTDDS_BUILTIN_TRANSPORTS") < (
        entrypoint.index("exec ros2 launch")
    )
    assert "pgrep -f '[/]realsense2_camera_node'" in dockerfile
    assert "pgrep -f '[/]imu_relay'" in dockerfile
    assert "pgrep -f '[/]d455_imu_processor'" in dockerfile
    assert "/realsense_imu_relay" not in dockerfile
    for forbidden in (
        "roboteq_ros2_driver",
        "command_arbiter",
        "joy_to_cmdvel",
        "teleop_pharma",
        "robot_localization",
        "nav2",
        "slam_toolbox",
    ):
        assert forbidden not in combined.lower()


def run_sensor_entrypoint(tmp_path, transport):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    fake_ros2 = fake_bin / "ros2"
    fake_ros2.write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' \"$@\" > \"$FAKE_ROS2_LOG\"\n",
        encoding="utf-8",
    )
    fake_ros2.chmod(0o755)
    ros2_log = tmp_path / "ros2.log"
    environment = {
        **os.environ,
        "PATH": f"{fake_bin}:{os.environ['PATH']}",
        "FAKE_ROS2_LOG": str(ros2_log),
        "D455_SERIAL_NUMBER": "146222250608",
        "ROS_DOMAIN_ID": "0",
        "ROS_LOCALHOST_ONLY": "0",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    }
    environment.pop("FASTDDS_BUILTIN_TRANSPORTS", None)
    if transport is not None:
        environment["FASTDDS_BUILTIN_TRANSPORTS"] = transport

    completed = subprocess.run(
        [
            "/bin/bash",
            "-c",
            "source() { return 0; }\n"
            "builtin source \"$1\"",
            "d455-entrypoint-test",
            str(
                REPOSITORY_ROOT
                / "deployment/scripts/d455_sensor_entrypoint.sh"
            ),
        ],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )
    return completed, ros2_log


def test_sensor_entrypoint_exact_udp_transport_launches_fake_ros2(
    tmp_path,
):
    completed, ros2_log = run_sensor_entrypoint(tmp_path, "UDPv4")

    assert completed.returncode == 0
    assert completed.stderr == ""
    assert ros2_log.read_text(encoding="utf-8").splitlines() == [
        "launch",
        "realsense_imu",
        "robot_sensors.launch.py",
        "serial_number:=146222250608",
    ]


@pytest.mark.parametrize("transport", (None, "", "DEFAULT"))
def test_sensor_entrypoint_rejects_invalid_udp_transport(
    tmp_path, transport
):
    completed, ros2_log = run_sensor_entrypoint(tmp_path, transport)

    assert completed.returncode == 64
    assert (
        "FASTDDS_BUILTIN_TRANSPORTS must be UDPv4 for production DDS"
        in " ".join(completed.stderr.split())
    )
    assert not ros2_log.exists()


def test_healthcheck_cannot_match_its_shell_and_requires_all_targets(
    tmp_path,
):
    fake_pgrep = tmp_path / "pgrep"
    fake_pgrep.write_text(
        "#!/usr/bin/env python3\n"
        "import os\n"
        "import re\n"
        "import sys\n"
        "pattern = sys.argv[-1]\n"
        "lines = os.environ.get('FAKE_PROCESS_LINES', '').splitlines()\n"
        "raise SystemExit(0 if any(re.search(pattern, line) "
        "for line in lines) else 1)\n",
        encoding="utf-8",
    )
    fake_pgrep.chmod(0o755)
    healthcheck = (
        "pgrep -f '[/]realsense2_camera_node' >/dev/null "
        "&& pgrep -f '[/]imu_relay' >/dev/null "
        "&& pgrep -f '[/]d455_imu_processor' >/dev/null "
        "|| exit 1"
    )
    environment = {
        **os.environ,
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
    }
    only_healthcheck_shell = subprocess.run(
        ["/bin/sh", "-c", healthcheck],
        check=False,
        env={
            **environment,
            "FAKE_PROCESS_LINES": f"/bin/sh -c {healthcheck}",
        },
    )
    assert only_healthcheck_shell.returncode != 0

    expected_processes = subprocess.run(
        ["/bin/sh", "-c", healthcheck],
        check=False,
        env={
            **environment,
            "FAKE_PROCESS_LINES": "\n".join(
                (
                    "/opt/ros/humble/lib/realsense2_camera/"
                    "realsense2_camera_node",
                    "/sensor_ws/install/realsense_imu/lib/"
                    "realsense_imu/imu_relay",
                    "/sensor_ws/install/realsense_imu/lib/"
                    "realsense_imu/d455_imu_processor",
                )
            ),
        },
    )
    assert expected_processes.returncode == 0


def test_main_container_has_no_d455_hardware_or_realsense_build():
    dockerfile = read("Dockerfile")
    start = read("deployment/scripts/pharma_start_container.sh")
    build = read("deployment/scripts/build_core.sh")

    assert "realsense_imu" not in dockerfile
    assert "realsense2_camera" not in dockerfile
    assert "realsense_imu" not in build
    for forbidden in (
        "--docker-device-args",
        "D455_IMU_AVAILABLE",
        "D455_SERIAL_NUMBER",
        "D455_APPARMOR_PROFILE",
        "pharmarobot-d455-imu",
        "/dev/iio:device",
        "HID-SENSOR",
    ):
        # Migration detection intentionally recognizes legacy markers.
        if forbidden in (
            "D455_IMU_AVAILABLE",
            "D455_SERIAL_NUMBER",
            "pharmarobot-d455-imu",
            "/dev/iio:device",
            "HID-SENSOR",
        ):
            assert start.count(forbidden) == 1
        else:
            assert forbidden not in start
    assert "--device-cgroup-rule 'c 13:* rwm'" in start
    assert "/dev/roboteq" in start


def test_normal_supervision_has_no_docker_exec_or_main_dependency():
    service = read("deployment/systemd/pharma-d455-imu.service")
    run = read("deployment/scripts/pharma_run_sensors.sh")
    stop = read("deployment/scripts/pharma_stop_sensors.sh")

    assert "Requires=docker.service" in service
    assert "pharmarobot.service" not in service
    assert "pharma-minimal-nodes.service" not in service
    assert "pharma_d455_sensor_container.sh run" in service
    assert "docker exec" not in run
    assert "docker exec" not in stop
    assert "pharma_container" not in run + stop


def test_build_provenance_prunes_generated_data_and_uses_local_alias():
    build = read(
        "deployment/scripts/pharma_build_d455_sensor_image.sh"
    )
    dockerfile = read("deployment/docker/Dockerfile.d455_sensor")
    assert "docker image tag \"$base_id\" \"$base_alias\"" in build
    assert "--pull=false" in build
    assert 'ROS_BASE_IMAGE=$base_alias' in build
    assert "BASE_IMAGE_ID=$base_id" in build
    assert "SOURCE_MANIFEST_SHA256=$source_manifest" in build
    for excluded in (
        "validation_evidence",
        ".pytest_cache",
        "__pycache__",
        "build",
        "install",
        "log",
        "*.pyc",
        "*.pyo",
    ):
        assert excluded in build
    assert "FROM ${ROS_BASE_IMAGE}" in dockerfile


@pytest.mark.parametrize("use_override", (False, True))
def test_image_build_wrapper_resolves_default_and_override_manifest(
    tmp_path, use_override
):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    tool_shim = fake_bin / "tool-shim"
    tool_shim.write_text(
        """#!/usr/bin/env python3
import os
from pathlib import Path
import shutil
import sys
import tempfile

tool = Path(sys.argv[0]).name
args = sys.argv[1:]
base_id = "sha256:" + "1" * 64
built_id = "sha256:" + "2" * 64
if tool == "docker":
    with Path(os.environ["FAKE_DOCKER_LOG"]).open(
        "a", encoding="utf-8"
    ) as output:
        output.write(" ".join(args) + "\\n")
    if args[:3] == ["image", "inspect", "--format"]:
        target = args[-1]
        if target == os.environ["D455_SENSOR_IMAGE"]:
            source = Path(
                os.environ["FAKE_SOURCE_HASH"]
            ).read_text(encoding="utf-8")
            print(built_id, "true", source, base_id)
        else:
            print(base_id)
        raise SystemExit(0)
    if args[:2] == ["image", "inspect"]:
        raise SystemExit(1)
    if args[:2] == ["image", "tag"]:
        raise SystemExit(0)
    if args and args[0] == "build":
        prefix = "SOURCE_MANIFEST_SHA256="
        source = next(
            value[len(prefix):]
            for value in args
            if value.startswith(prefix)
        )
        Path(os.environ["FAKE_SOURCE_HASH"]).write_text(
            source, encoding="utf-8"
        )
        raise SystemExit(0)
    if args[:2] == ["image", "rm"]:
        raise SystemExit(0)
    raise SystemExit(91)
if tool == "mkdir" and "/var/lib/pharmarobot" in args:
    raise SystemExit(0)
if tool == "mktemp" and any(
    value.startswith("/var/lib/pharmarobot/") for value in args
):
    descriptor, name = tempfile.mkstemp(
        prefix="manifest.", dir=os.environ["FAKE_MANIFEST_ROOT"]
    )
    os.close(descriptor)
    print(name)
    raise SystemExit(0)
if tool == "mv" and args[-1].startswith("/var/lib/pharmarobot/"):
    shutil.move(args[-2], os.environ["FAKE_DEFAULT_MANIFEST"])
    raise SystemExit(0)
os.execv("/usr/bin/" + tool, [tool, *args])
""",
        encoding="utf-8",
    )
    tool_shim.chmod(0o755)
    for tool in ("docker", "mkdir", "mktemp", "mv"):
        (fake_bin / tool).symlink_to(tool_shim)

    docker_log = tmp_path / "docker.log"
    source_hash = tmp_path / "source-hash"
    default_manifest = tmp_path / "default-manifest.env"
    override_manifest = tmp_path / "override" / "sensor-image.env"
    environment = {
        **os.environ,
        "PATH": f"{fake_bin}:{os.environ['PATH']}",
        "D455_SENSOR_BASE_IMAGE": "reviewed:base",
        "D455_SENSOR_IMAGE": "test:d455-sensor",
        "FAKE_DOCKER_LOG": str(docker_log),
        "FAKE_SOURCE_HASH": str(source_hash),
        "FAKE_MANIFEST_ROOT": str(tmp_path),
        "FAKE_DEFAULT_MANIFEST": str(default_manifest),
    }
    environment.pop("D455_SENSOR_IMAGE_MANIFEST", None)
    if use_override:
        environment["D455_SENSOR_IMAGE_MANIFEST"] = str(
            override_manifest
        )

    completed = subprocess.run(
        [
            "/bin/bash",
            str(
                REPOSITORY_ROOT
                / "deployment/scripts/"
                "pharma_build_d455_sensor_image.sh"
            ),
        ],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )
    assert completed.returncode == 0, completed.stderr
    manifest = override_manifest if use_override else default_manifest
    assert manifest.is_file()
    content = manifest.read_text(encoding="utf-8")
    assert "IMAGE=test:d455-sensor" in content
    assert f"IMAGE_ID=sha256:{'2' * 64}" in content
    assert "bad substitution" not in completed.stderr
    calls = docker_log.read_text(encoding="utf-8")
    assert "image inspect --format {{.Id}} reviewed:base" in calls
    assert "build --pull=false" in calls


def test_persistent_defaults_do_not_grant_destructive_authorization():
    defaults = read("deployment/systemd/pharmarobot.default")
    installer = read("deployment/install_services.sh")
    main_start = read("deployment/scripts/pharma_start_container.sh")
    for forbidden in (
        "D455_SENSOR_AUTHORIZE_PROFILE_RELOAD",
        "D455_SENSOR_AUTHORIZE_RECREATE",
        "PHARMA_MAIN_D455_MIGRATION_APPROVED",
    ):
        assert forbidden not in defaults
        assert forbidden not in installer
        assert forbidden not in main_start
    assert "exit 78" in main_start


def test_dds_environment_and_topic_qos_contract_are_consistent():
    defaults = read("deployment/systemd/pharmarobot.default")
    main_start = read("deployment/scripts/pharma_start_container.sh")
    production_tool = read(
        "src/realsense_imu/tools/d455_production_container.py"
    )
    relay = read("src/realsense_imu/realsense_imu/imu_relay.py")
    processor = read("src/realsense_imu/realsense_imu/imu_processor.py")

    for setting in (
        "ROS_DOMAIN_ID=0",
        "ROS_LOCALHOST_ONLY=0",
        "RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
    ):
        assert setting in defaults
    assert "ROS_LOCALHOST_ONLY_VALUE" in main_start
    assert "DEFAULT_RMW_IMPLEMENTATION = \"rmw_fastrtps_cpp\"" in (
        production_tool
    )
    assert (
        'DEFAULT_FASTDDS_BUILTIN_TRANSPORTS = "UDPv4"'
        in production_tool
    )
    assert "qos_profile_sensor_data" in relay
    assert processor.count("qos_profile_sensor_data") >= 3
    assert "DiagnosticArray, diagnostics_topic, 10" in processor
