# Copyright 2026 Medrobots Engineering
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from dataclasses import replace
from contextlib import nullcontext
import importlib.util
import json
from pathlib import Path
import sys

import pytest

from realsense_imu.apparmor_profile import generate_apparmor_profile
from realsense_imu.usb_device import DeviceNode
from realsense_imu.usb_device import HostResources
from realsense_imu.usb_device import IioDevice


TOOL_PATH = (
    Path(__file__).parents[1] / "tools" / "d455_host_preflight.py"
)
SPEC = importlib.util.spec_from_file_location("d455_host_preflight", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
preflight = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = preflight
SPEC.loader.exec_module(preflight)


SERIAL = "146222250608"
USB_SERIAL = "151223061922"
USB_PATH = Path(
    "/sys/devices/pci0000:00/0000:00:14.0/usb4/4-3/4-3.1"
)


def resources(instance="0001"):
    hid = (
        USB_PATH
        / "4-3.1:1.5"
        / f"0003:8086:0B5C.{instance}"
        / "HID-SENSOR-200073.2.auto"
    )
    accel = hid / "iio:device0"
    gyro = hid.parent / "HID-SENSOR-200076.3.auto" / "iio:device1"
    return HostResources(
        usb_sysfs_path=USB_PATH,
        device_nodes=(
            DeviceNode(Path("/dev/bus/usb/004/004"), 189, 387),
            DeviceNode(Path("/dev/video0"), 81, 0),
            DeviceNode(Path("/dev/media0"), 239, 0),
            DeviceNode(Path("/dev/iio:device0"), 250, 0),
            DeviceNode(Path("/dev/iio:device1"), 250, 1),
        ),
        iio_devices=(
            IioDevice("accel_3d", Path("/dev/iio:device0"), accel),
            IioDevice("gyro_3d", Path("/dev/iio:device1"), gyro),
        ),
    )


def manifest(selected=None, *, generated_at="2026-07-27T12:00:00+00:00"):
    selected = selected or resources()
    profile = generate_apparmor_profile(selected)
    value = preflight.build_manifest(
        selected,
        serial_number=SERIAL,
        usb_serial_number=USB_SERIAL,
        profile=profile,
        generated_at=generated_at,
        tool_sha256="a" * 64,
    )
    return profile, value


class FakeRunner:
    def __init__(self, results=()):
        self.results = list(results)
        self.commands = []

    def run(self, args, *, timeout):
        command = tuple(str(item) for item in args)
        self.commands.append((command, timeout))
        if self.results:
            result = self.results.pop(0)
            return replace(result, args=command)
        return preflight.CommandResult(command, 0, "", "")


def result(status=0, stdout="", stderr="", timed_out=False):
    return preflight.CommandResult((), status, stdout, stderr, timed_out)


def evidence(tmp_path):
    return preflight.Evidence(tmp_path / "evidence")


def test_validation_preflight_rejects_active_production_container(tmp_path):
    runner = FakeRunner(
        [result(stdout="abc pharmarobot_d455_sensor\n")]
    )
    with pytest.raises(
        preflight.PreflightError,
        match="production D455 sensor container",
    ):
        preflight.reject_active_production_container(
            runner, evidence(tmp_path)
        )
    assert (
        f"label={preflight.PRODUCTION_LABEL}"
        in runner.commands[0][0]
    )


def profile_manager(
    tmp_path,
    runner,
    evidence_object,
    states,
    *,
    effective_uid=lambda: 0,
):
    installed = tmp_path / "apparmor.d" / preflight.PROFILE_NAME
    installed_manifest = tmp_path / "apparmor.d" / (
        preflight.PROFILE_NAME + ".manifest.json"
    )
    state_values = list(states)

    def probe(**_kwargs):
        if len(state_values) > 1:
            return state_values.pop(0)
        return state_values[0]

    return preflight.ProfileManager(
        runner=runner,
        evidence=evidence_object,
        installed_profile_path=installed,
        installed_manifest_path=installed_manifest,
        kernel_profiles_path=tmp_path / "profiles",
        state_probe=probe,
        effective_uid=effective_uid,
    )


def stale_state(sha=None):
    return preflight.ProfileState(
        loaded=True,
        enforcing=True,
        installed_sha256=sha or "0" * 64,
        installed_manifest_fingerprint="1" * 64,
    )


def matching_state(value):
    return preflight.ProfileState(
        loaded=True,
        enforcing=True,
        installed_sha256=value["profile_sha256"],
        installed_manifest_fingerprint=value["resource_fingerprint"],
    )


def write_candidate(tmp_path, profile):
    path = tmp_path / "candidate.apparmor"
    path.write_text(profile, encoding="utf-8")
    return path


def test_manifest_records_stable_and_transient_identifiers_separately():
    _, value = manifest(resources("0001"))

    assert value["stable_identity"]["librealsense_serial"] == SERIAL
    assert value["stable_identity"]["usb_serial"] == USB_SERIAL
    devices = value["resolved_resources"]["iio_devices"]
    assert {item["sysfs"]["hid_instance"] for item in devices} == {"0001"}
    assert {item["sysfs"]["hid_identity"] for item in devices} == {
        "0003:8086:0B5C"
    }


def test_manifest_is_deterministic_for_same_timestamp_and_resources():
    _, first = manifest()
    _, second = manifest()

    assert first == second


def test_reenumeration_changes_only_resolved_fingerprint_not_stable_identity():
    _, first = manifest(resources("0001"))
    _, second = manifest(resources("0002"))

    assert first["stable_identity"] == second["stable_identity"]
    assert first["resource_fingerprint"] != second["resource_fingerprint"]
    assert {
        item["sysfs"]["hid_instance"]
        for item in second["resolved_resources"]["iio_devices"]
    } == {"0002"}


def test_resource_validation_rejects_motor_and_broad_resources():
    selected = replace(
        resources(),
        device_nodes=(
            *resources().device_nodes,
            DeviceNode(Path("/dev/roboteq"), 188, 0),
        ),
    )

    with pytest.raises(preflight.PreflightError, match="motor/control"):
        preflight.validate_resource_set(selected)


def test_resource_validation_rejects_duplicate_nodes():
    selected = resources()
    selected = replace(
        selected,
        device_nodes=(*selected.device_nodes, selected.device_nodes[0]),
    )

    with pytest.raises(preflight.PreflightError, match="duplicate"):
        preflight.validate_resource_set(selected)


def test_resource_validation_rejects_iio_path_escape():
    selected = resources()
    escaped = replace(
        selected.iio_devices[1],
        sysfs_path=Path("/sys/devices/unrelated/iio:device1"),
    )

    with pytest.raises(preflight.PreflightError, match="escaped"):
        preflight.validate_resource_set(
            replace(
                selected,
                iio_devices=(selected.iio_devices[0], escaped),
            )
        )


def test_atomic_write_replaces_complete_file(tmp_path):
    target = tmp_path / "manifest.json"
    target.write_bytes(b"old")

    preflight.atomic_write(target, b"new")

    assert target.read_bytes() == b"new"
    assert not list(tmp_path.glob(".manifest.json.*"))


def test_profile_syntax_failure_prevents_reload(tmp_path):
    profile, value = manifest()
    runner = FakeRunner([result(1, stderr="syntax error")])
    manager = profile_manager(
        tmp_path, runner, evidence(tmp_path), [stale_state()]
    )

    with pytest.raises(preflight.PreflightError, match="apparmor_syntax"):
        manager.ensure(
            candidate_profile_path=write_candidate(tmp_path, profile),
            profile=profile,
            manifest=value,
            authorize_reload=True,
        )

    assert len(runner.commands) == 1


def test_unchanged_loaded_enforcing_profile_is_noop(tmp_path):
    profile, value = manifest()
    runner = FakeRunner([result()])
    manager = profile_manager(
        tmp_path, runner, evidence(tmp_path), [matching_state(value)]
    )

    decision = manager.ensure(
        candidate_profile_path=write_candidate(tmp_path, profile),
        profile=profile,
        manifest=value,
        authorize_reload=False,
    )

    assert decision.changed is False
    assert len(runner.commands) == 1


@pytest.mark.parametrize(
    "state",
    [
        preflight.ProfileState(False, False, None, None),
        preflight.ProfileState(True, False, "a" * 64, "b" * 64),
        stale_state(),
    ],
)
def test_missing_non_enforcing_or_stale_profile_requires_approval(
    tmp_path, state
):
    profile, value = manifest()
    runner = FakeRunner([result()])
    manager = profile_manager(tmp_path, runner, evidence(tmp_path), [state])

    with pytest.raises(preflight.ApprovalRequired):
        manager.ensure(
            candidate_profile_path=write_candidate(tmp_path, profile),
            profile=profile,
            manifest=value,
            authorize_reload=False,
        )


def test_conflicting_loaded_profile_fails_without_reload(tmp_path):
    profile, value = manifest()
    state = replace(
        matching_state(value),
        conflicting_profiles=("old-d455-profile (enforce)",),
    )
    runner = FakeRunner([result()])
    manager = profile_manager(tmp_path, runner, evidence(tmp_path), [state])

    with pytest.raises(preflight.PreflightError, match="conflicting"):
        manager.ensure(
            candidate_profile_path=write_candidate(tmp_path, profile),
            profile=profile,
            manifest=value,
            authorize_reload=True,
        )

    assert len(runner.commands) == 1


def test_reload_requires_explicit_root_execution(tmp_path):
    profile, value = manifest()
    runner = FakeRunner([result()])
    manager = profile_manager(
        tmp_path,
        runner,
        evidence(tmp_path),
        [stale_state()],
        effective_uid=lambda: 1000,
    )

    with pytest.raises(preflight.ApprovalRequired, match="root"):
        manager.ensure(
            candidate_profile_path=write_candidate(tmp_path, profile),
            profile=profile,
            manifest=value,
            authorize_reload=True,
        )


def test_successful_reload_writes_and_verifies_candidate(tmp_path):
    profile, value = manifest()
    runner = FakeRunner([result(), result()])
    manager = profile_manager(
        tmp_path,
        runner,
        evidence(tmp_path),
        [
            preflight.ProfileState(False, False, None, None),
            matching_state(value),
        ],
    )

    decision = manager.ensure(
        candidate_profile_path=write_candidate(tmp_path, profile),
        profile=profile,
        manifest=value,
        authorize_reload=True,
    )

    assert decision.changed is True
    assert (
        manager.installed_profile_path.read_text(encoding="utf-8")
        == profile
    )
    installed_manifest = json.loads(
        manager.installed_manifest_path.read_text(encoding="utf-8")
    )
    assert installed_manifest["resource_fingerprint"] == value[
        "resource_fingerprint"
    ]


@pytest.mark.parametrize(
    "unproven_state",
    [
        preflight.ProfileState(False, False, None, None),
        preflight.ProfileState(True, False, None, None),
    ],
)
def test_unproven_installed_profile_is_never_replaced(
    tmp_path, unproven_state
):
    profile, value = manifest()
    runner = FakeRunner([result()])
    manager = profile_manager(
        tmp_path,
        runner,
        evidence(tmp_path),
        [unproven_state],
    )
    manager.installed_profile_path.parent.mkdir()
    manager.installed_profile_path.write_bytes(b"unproven old profile")

    with pytest.raises(preflight.PreflightError, match="not proven"):
        manager.ensure(
            candidate_profile_path=write_candidate(tmp_path, profile),
            profile=profile,
            manifest=value,
            authorize_reload=True,
        )

    assert manager.installed_profile_path.read_bytes() == (
        b"unproven old profile"
    )
    assert len(runner.commands) == 1


def test_failed_first_install_proves_candidate_absent(tmp_path):
    profile, value = manifest()
    absent = preflight.ProfileState(False, False, None, None)
    runner = FakeRunner(
        [
            result(),
            result(1, stderr="reload failed"),
            result(1, stderr="not loaded"),
        ]
    )
    manager = profile_manager(
        tmp_path,
        runner,
        evidence(tmp_path),
        [absent, absent],
    )

    with pytest.raises(preflight.PreflightError, match="apparmor_reload"):
        manager.ensure(
            candidate_profile_path=write_candidate(tmp_path, profile),
            profile=profile,
            manifest=value,
            authorize_reload=True,
        )

    assert not manager.installed_profile_path.exists()
    assert not manager.installed_manifest_path.exists()


def test_reload_failure_restores_previous_profile_and_manifest(tmp_path):
    profile, value = manifest()
    previous = b"previous profile"
    previous_state = stale_state(preflight.sha256_bytes(previous))
    runner = FakeRunner(
        [
            result(),
            result(1, stderr="reload failed"),
            result(),
        ]
    )
    manager = profile_manager(
        tmp_path,
        runner,
        evidence(tmp_path),
        [previous_state, previous_state],
    )
    manager.installed_profile_path.parent.mkdir()
    manager.installed_profile_path.write_bytes(previous)
    manager.installed_manifest_path.write_bytes(b'{"previous":true}\n')

    with pytest.raises(preflight.PreflightError, match="apparmor_reload"):
        manager.ensure(
            candidate_profile_path=write_candidate(tmp_path, profile),
            profile=profile,
            manifest=value,
            authorize_reload=True,
        )

    assert manager.installed_profile_path.read_bytes() == b"previous profile"
    assert (
        manager.installed_manifest_path.read_bytes()
        == b'{"previous":true}\n'
    )
    assert any(
        command[:3] == ("apparmor_parser", "-r", "-W")
        for command, _timeout in runner.commands
    )


def test_failed_post_reload_enforcing_check_rolls_back(tmp_path):
    profile, value = manifest()
    not_enforcing = replace(matching_state(value), enforcing=False)
    previous = b"previous"
    previous_state = stale_state(preflight.sha256_bytes(previous))
    runner = FakeRunner([result(), result(), result()])
    manager = profile_manager(
        tmp_path,
        runner,
        evidence(tmp_path),
        [previous_state, not_enforcing, previous_state],
    )
    manager.installed_profile_path.parent.mkdir()
    manager.installed_profile_path.write_bytes(previous)
    manager.installed_manifest_path.write_bytes(b"{}")

    with pytest.raises(preflight.PreflightError, match="could not be proven"):
        manager.ensure(
            candidate_profile_path=write_candidate(tmp_path, profile),
            profile=profile,
            manifest=value,
            authorize_reload=True,
        )

    assert manager.installed_profile_path.read_bytes() == b"previous"


def test_profile_probe_requires_exact_single_enforcing_name(tmp_path):
    installed = tmp_path / "profile"
    installed.write_text("profile", encoding="utf-8")
    manifest_path = tmp_path / "manifest"
    manifest_path.write_text(
        '{"resource_fingerprint":"abc"}', encoding="utf-8"
    )
    kernel = tmp_path / "profiles"
    kernel.write_text(
        "pharmarobot-d455-imu (enforce)\n"
        "stale-d455-validation (enforce)\n",
        encoding="utf-8",
    )

    state = preflight.probe_profile_state(
        profile_name=preflight.PROFILE_NAME,
        installed_profile_path=installed,
        installed_manifest_path=manifest_path,
        kernel_profiles_path=kernel,
    )

    assert state.loaded
    assert state.enforcing
    assert state.conflicting_profiles == ("stale-d455-validation (enforce)",)


def test_container_command_preserves_narrow_isolation():
    command = preflight.container_run_command(
        container_name="pharma_realsense_imu_validation",
        image="pharmarobot:realsense-imu",
        resources=resources(),
    )
    joined = "\n".join(command)

    for required in (
        "--init",
        "--network=none",
        "--cap-drop=ALL",
        "--security-opt=no-new-privileges:true",
        "--security-opt=apparmor=pharmarobot-d455-imu",
        "pharmarobot.d455.validation=true",
    ):
        assert required in command or required in joined
    for forbidden in (
        "--privileged",
        "apparmor=unconfined",
        "/dev/roboteq",
        "/dev/ttyUSB",
        "/dev/ttyACM",
        "/dev/input",
        "--network=host",
    ):
        assert forbidden not in joined


def test_container_command_changes_transient_mount_after_reenumeration():
    first = "\n".join(
        preflight.container_run_command(
            container_name="pharma_realsense_imu_validation",
            image="image",
            resources=resources("0001"),
        )
    )
    second = "\n".join(
        preflight.container_run_command(
            container_name="pharma_realsense_imu_validation",
            image="image",
            resources=resources("0002"),
        )
    )

    assert ".0001" in first
    assert ".0002" in second
    assert first != second


def test_access_probe_checks_without_writing():
    script = preflight._access_probe_script(resources())

    assert "test -r" in script
    assert "test -w" in script
    assert "printf 'D455_ACCESS_OK=1" in script
    assert "echo 1 >" not in script
    assert "tee " not in script


def test_bounded_command_timeout_is_persisted_and_rejected(tmp_path):
    runner = FakeRunner([result(124, timed_out=True)])
    evidence_object = evidence(tmp_path)

    with pytest.raises(preflight.PreflightError, match="timed out"):
        preflight.run_checked(
            runner,
            evidence_object,
            "bounded",
            ["fake"],
            timeout=0.1,
        )

    record = json.loads(
        (evidence_object.root / "command-001.json").read_text(encoding="utf-8")
    )
    assert record["timed_out"] is True
    assert record["timeout_seconds"] == 0.1


def test_current_audit_ignores_unrelated_history_and_rejects_selected_device():
    unrelated = (
        'apparmor="DENIED" profile="something-else"\n'
        "usb 2-1: reset high-speed USB device\n"
    )
    assert (
        preflight.relevant_audit_failure(
            unrelated,
            profile_name=preflight.PROFILE_NAME,
            usb_topology_name="4-3.1",
        )
        is None
    )
    selected = "usb 4-3.1: USB disconnect, device number 4"
    assert "disconnect" in preflight.relevant_audit_failure(
        selected,
        profile_name=preflight.PROFILE_NAME,
        usb_topology_name="4-3.1",
    )


def test_journalctl_since_timestamp_preserves_utc_microsecond_boundary():
    assert preflight.journalctl_since_timestamp(
        "2026-07-27T12:34:56.123456+00:00"
    ) == "2026-07-27 12:34:56.123456 UTC"


def test_journalctl_since_timestamp_normalizes_offset_and_floors_nanoseconds():
    assert preflight.journalctl_since_timestamp(
        "2026-07-27T13:34:56.123456789+01:00"
    ) == "2026-07-27 12:34:56.123456 UTC"


def test_journalctl_since_timestamp_retains_zero_fraction_boundary():
    assert preflight.journalctl_since_timestamp(
        "2026-07-27T12:34:56Z"
    ) == "2026-07-27 12:34:56.000000 UTC"


@pytest.mark.parametrize(
    "value",
    ["2026-07-27 12:34:56", "not-a-timestamp", ""],
)
def test_journalctl_since_timestamp_rejects_invalid_or_timezone_less(value):
    with pytest.raises(preflight.PreflightError, match="audit_since"):
        preflight.journalctl_since_timestamp(value)


def test_audit_command_uses_journalctl_compatible_since_value():
    command = preflight.audit_command(
        "2026-07-27T12:34:56.123456+00:00"
    )

    assert command[command.index("--since") + 1] == (
        "2026-07-27 12:34:56.123456 UTC"
    )


def docker_inspect_payload(*, security_extras=()):
    selected = resources()
    return json.dumps(
        [
            {
                "HostConfig": {
                    "NetworkMode": "none",
                    "Privileged": False,
                    "CapDrop": ["ALL"],
                    "SecurityOpt": [
                        "no-new-privileges:true",
                        "apparmor=pharmarobot-d455-imu",
                        *security_extras,
                    ],
                    "Devices": [
                        {"PathOnHost": str(node.path)}
                        for node in selected.device_nodes
                        if not node.path.name.startswith("iio:device")
                    ],
                    "DeviceCgroupRules": [
                        f"c {node.major}:{node.minor} rwm"
                        for node in selected.device_nodes
                        if node.path.name.startswith("iio:device")
                    ],
                },
                "State": {"Running": True},
                "Config": {
                    "Labels": {"pharmarobot.d455.validation": "true"},
                    "Entrypoint": [],
                },
                "Mounts": [
                    {"Type": "bind", "Destination": destination}
                    for destination in (
                        *(
                            str(node.path)
                            for node in selected.device_nodes
                            if node.path.name.startswith("iio:device")
                        ),
                        *(
                            str(device.sysfs_path)
                            for device in selected.iio_devices
                        ),
                    )
                ],
            }
        ]
    )


class ContainerRunner:
    def __init__(
        self,
        *,
        access_status=0,
        runtime_status=1,
        security_extras=(),
    ):
        self.commands = []
        self.created = False
        self.removed = False
        self.inspect_count = 0
        self.access_status = access_status
        self.runtime_status = runtime_status
        self.security_extras = security_extras

    def run(self, args, *, timeout):
        command = tuple(str(item) for item in args)
        self.commands.append((command, timeout))
        if command and command[0] == "journalctl":
            return preflight.CommandResult(command, 0, "", "")
        if command[:2] == ("docker", "inspect"):
            self.inspect_count += 1
            if not self.created or self.removed:
                return preflight.CommandResult(command, 1, "", "not found")
            return preflight.CommandResult(
                command,
                0,
                docker_inspect_payload(
                    security_extras=self.security_extras
                ),
                "",
            )
        if command[:3] == ("docker", "image", "inspect"):
            return preflight.CommandResult(
                command, 0, f'[{{"Id":"sha256:{"a" * 64}"}}]', ""
            )
        if command[:3] == ("docker", "run", "-d"):
            self.created = True
            return preflight.CommandResult(command, 0, "container-id\n", "")
        if command[:2] == ("docker", "exec"):
            script = command[-1]
            if script.startswith("test -d /tmp/workspace/runtime-evidence"):
                return preflight.CommandResult(command, 1, "", "")
            if "D455_ACCESS_OK" in script:
                return preflight.CommandResult(
                    command, self.access_status, "", "denied"
                )
            return preflight.CommandResult(command, 0, "", "")
        if command[:2] == ("docker", "stop"):
            return preflight.CommandResult(command, 0, "", "")
        if command[:2] == ("docker", "rm"):
            self.removed = True
            return preflight.CommandResult(command, 0, "", "")
        return preflight.CommandResult(
            command, self.runtime_status, "", "runtime failure"
        )


def test_access_permission_denial_fails_and_cleanup_succeeds(tmp_path):
    runner = ContainerRunner(access_status=1)
    evidence_object = evidence(tmp_path)
    manager = preflight.ContainerManager(
        runner=runner,
        evidence=evidence_object,
        container_name="pharma_realsense_imu_validation",
        image="image",
        resources=resources(),
        workspace="/tmp/workspace",
    )
    manager.create()

    with pytest.raises(
        preflight.PreflightError, match="container_access_probe"
    ):
        manager.verify_and_preflight_access()
    manager.cleanup()

    assert runner.removed is True
    assert any(
        command[:2] == ("docker", "stop")
        for command, _timeout in runner.commands
    )


def test_post_create_rejects_extra_security_option(tmp_path):
    runner = ContainerRunner(security_extras=("seccomp=unconfined",))
    manager = preflight.ContainerManager(
        runner=runner,
        evidence=evidence(tmp_path),
        container_name="pharma_realsense_imu_validation",
        image="image",
        resources=resources(),
        workspace="/tmp/workspace",
    )
    manager.create()

    with pytest.raises(
        preflight.PreflightError, match="container isolation drift"
    ):
        manager.verify_and_preflight_access()
    manager.cleanup()

    assert runner.removed is True


def test_existing_unlabelled_container_is_never_removed(tmp_path):
    class UnlabelledRunner(ContainerRunner):
        def run(self, args, *, timeout):
            command = tuple(str(item) for item in args)
            self.commands.append((command, timeout))
            if command[:2] == ("docker", "inspect"):
                payload = json.loads(docker_inspect_payload())
                payload[0]["Config"]["Labels"] = {}
                return preflight.CommandResult(
                    command, 0, json.dumps(payload), ""
                )
            return super().run(args, timeout=timeout)

    runner = UnlabelledRunner()
    manager = preflight.ContainerManager(
        runner=runner,
        evidence=evidence(tmp_path),
        container_name="pharma_realsense_imu_validation",
        image="image",
        resources=resources(),
        workspace="/tmp/workspace",
    )

    with pytest.raises(preflight.PreflightError, match="refusing to remove"):
        manager.create()

    assert not any(
        command[:2] == ("docker", "rm")
        for command, _timeout in runner.commands
    )


class NoopProfileManager:
    def __init__(self, **_kwargs):
        pass

    def ensure(self, *, manifest, **_kwargs):
        return preflight.ProfileDecision(
            False,
            manifest["profile_sha256"],
            manifest["resource_fingerprint"],
            manifest["profile_sha256"],
        )


def config(tmp_path):
    return preflight.RuntimeConfig(
        serial_number=SERIAL,
        usb_serial_number=USB_SERIAL,
        image="image",
        container_name="pharma_realsense_imu_validation",
        workspace="/tmp/workspace",
        evidence_dir=tmp_path / "run",
        installed_profile_path=tmp_path / "profile",
        installed_manifest_path=tmp_path / "profile.manifest",
        kernel_profiles_path=tmp_path / "profiles",
        validation_wrapper=Path("/repo/d455_no_motion_validation.py"),
    )


def test_workflow_refuses_runtime_without_all_authorizations(tmp_path):
    with pytest.raises(preflight.ApprovalRequired, match="authorization"):
        preflight.execute_workflow(
            config(tmp_path),
            authorize_profile_reload=False,
            authorize_container_recreate=True,
            authorize_stationary_d455=True,
            authorize_ros_no_motion=False,
        )

    result_data = json.loads(
        (tmp_path / "run" / "result.json").read_text(encoding="utf-8")
    )
    assert result_data["result"] == "failed"
    failure_data = json.loads(
        (tmp_path / "run" / "failure.json").read_text(encoding="utf-8")
    )
    assert failure_data["error_type"] == "ApprovalRequired"
    assert failure_data["cleanup_result"] == "not_applicable"
    assert failure_data["context"]["authorizations"]["ros_no_motion"] is False


@pytest.mark.parametrize(
    "flag",
    [
        "--installed-profile-path",
        "--installed-manifest-path",
        "--kernel-profiles-path",
        "--validation-wrapper",
    ],
)
def test_root_sensitive_paths_are_not_cli_overridable(flag, tmp_path):
    with pytest.raises(SystemExit):
        preflight.parse_args(
            [
                "--image",
                "image",
                "--workspace",
                "/tmp/workspace",
                "--evidence-dir",
                str(tmp_path / "evidence"),
                flag,
                str(tmp_path / "override"),
            ]
        )


def test_operational_paths_are_fixed_and_package_local():
    assert preflight.INSTALLED_PROFILE_PATH == (
        Path("/etc/apparmor.d") / preflight.PROFILE_NAME
    )
    assert preflight.INSTALLED_MANIFEST_PATH == (
        Path("/etc/apparmor.d")
        / f"{preflight.PROFILE_NAME}.manifest.json"
    )
    assert preflight.KERNEL_PROFILES_PATH == Path(
        "/sys/kernel/security/apparmor/profiles"
    )
    assert preflight.VALIDATION_WRAPPER_PATH == (
        TOOL_PATH.resolve().with_name("d455_no_motion_validation.py")
    )


def test_early_discovery_failure_retains_failure_evidence_without_docker(
    tmp_path,
):
    runner = ContainerRunner()

    def fail_discovery(_serial):
        raise preflight.PreflightError("discovery", "camera disconnected")

    with pytest.raises(preflight.PreflightError, match="camera disconnected"):
        preflight.execute_workflow(
            config(tmp_path),
            authorize_profile_reload=False,
            authorize_container_recreate=True,
            authorize_stationary_d455=True,
            authorize_ros_no_motion=True,
            runner=runner,
            discover=fail_discovery,
            serial_verifier=lambda _serial: None,
            production_exclusion_checker=lambda _runner, _evidence: None,
            lock_factory=lambda _operation: nullcontext(),
            profile_manager_factory=NoopProfileManager,
        )

    result_data = json.loads(
        (tmp_path / "run" / "result.json").read_text(encoding="utf-8")
    )
    assert result_data["result"] == "failed"
    assert "camera disconnected" in result_data["error"]
    failure_data = json.loads(
        (tmp_path / "run" / "failure.json").read_text(encoding="utf-8")
    )
    assert failure_data["phase"] == "discovery"
    assert failure_data["error_type"] == "PreflightError"
    assert not any(
        command and command[0] == "docker"
        for command, _timeout in runner.commands
    )


def test_serial_failure_is_preserved_before_discovery_or_docker(tmp_path):
    runner = ContainerRunner()

    class TrackingLock:
        entered = False
        released = False

        def __enter__(self):
            self.entered = True
            return self

        def __exit__(self, *_args):
            self.released = True

    workflow_lock = TrackingLock()

    def fail_serial(_serial):
        raise preflight.PreflightError("serial", "D455 serial mismatch")

    with pytest.raises(preflight.PreflightError, match="serial mismatch"):
        preflight.execute_workflow(
            config(tmp_path),
            authorize_profile_reload=False,
            authorize_container_recreate=True,
            authorize_stationary_d455=True,
            authorize_ros_no_motion=True,
            runner=runner,
            discover=lambda _serial: resources(),
            serial_verifier=fail_serial,
            production_exclusion_checker=lambda _runner, _evidence: None,
            lock_factory=lambda _operation: workflow_lock,
            profile_manager_factory=NoopProfileManager,
        )

    failure_data = json.loads(
        (tmp_path / "run" / "failure.json").read_text(encoding="utf-8")
    )
    assert failure_data["phase"] == "serial"
    assert workflow_lock.entered is True
    assert workflow_lock.released is True
    assert not runner.commands


def test_empty_message_failure_retains_repr_and_full_traceback(tmp_path):
    def fail_discovery(_serial):
        raise RuntimeError("")

    with pytest.raises(RuntimeError) as caught:
        preflight.execute_workflow(
            config(tmp_path),
            authorize_profile_reload=False,
            authorize_container_recreate=True,
            authorize_stationary_d455=True,
            authorize_ros_no_motion=True,
            runner=ContainerRunner(),
            discover=fail_discovery,
            serial_verifier=lambda _serial: None,
            production_exclusion_checker=lambda _runner, _evidence: None,
            lock_factory=lambda _operation: nullcontext(),
            profile_manager_factory=NoopProfileManager,
        )

    failure_data = json.loads(
        (tmp_path / "run" / "failure.json").read_text(encoding="utf-8")
    )
    assert failure_data["exception_repr"] == repr(caught.value)
    assert "RuntimeError" in failure_data["traceback"]
    assert "raise RuntimeError(\"\")" in failure_data["traceback"]
    assert failure_data["context"]["container_name"] == (
        "pharma_realsense_imu_validation"
    )


def test_evidence_setup_failure_writes_adjacent_atomic_fallback(tmp_path):
    evidence_dir = tmp_path / "run"
    evidence_dir.mkdir()
    trial_config = replace(config(tmp_path), evidence_dir=evidence_dir)

    with pytest.raises(preflight.PreflightError, match="already exists"):
        preflight.execute_workflow(
            trial_config,
            authorize_profile_reload=False,
            authorize_container_recreate=True,
            authorize_stationary_d455=True,
            authorize_ros_no_motion=True,
        )

    fallback = tmp_path / "run.failure.json"
    assert fallback.is_file()
    saved = json.loads(fallback.read_text(encoding="utf-8"))
    assert saved["phase"] == "evidence"
    assert saved["cleanup_result"] == "not_applicable"
    assert not list(tmp_path.glob(".run.failure.json.*"))


def test_evidence_setup_fallback_preserves_authorization_context(tmp_path):
    evidence_dir = tmp_path / "run"
    evidence_dir.mkdir()
    trial_config = replace(config(tmp_path), evidence_dir=evidence_dir)

    with pytest.raises(preflight.PreflightError):
        preflight.execute_workflow(
            trial_config,
            authorize_profile_reload=False,
            authorize_container_recreate=False,
            authorize_stationary_d455=True,
            authorize_ros_no_motion=False,
        )

    saved = json.loads(
        (tmp_path / "run.failure.json").read_text(encoding="utf-8")
    )
    assert saved["context"]["authorizations"] == {
        "profile_reload": False,
        "container_recreate": False,
        "stationary_d455": True,
        "ros_no_motion": False,
    }


def test_failure_evidence_write_error_does_not_replace_root_exception(
    tmp_path, monkeypatch
):
    original_write_json = preflight.Evidence.write_json

    def fail_failure_write(self, name, value):
        if name == "failure.json":
            raise OSError("evidence storage unavailable")
        return original_write_json(self, name, value)

    monkeypatch.setattr(
        preflight.Evidence, "write_json", fail_failure_write
    )

    def fail_discovery(_serial):
        raise preflight.PreflightError("discovery", "camera disconnected")

    with pytest.raises(preflight.PreflightError, match="camera disconnected"):
        preflight.execute_workflow(
            config(tmp_path),
            authorize_profile_reload=False,
            authorize_container_recreate=True,
            authorize_stationary_d455=True,
            authorize_ros_no_motion=True,
            runner=ContainerRunner(),
            discover=fail_discovery,
            serial_verifier=lambda _serial: None,
            production_exclusion_checker=lambda _runner, _evidence: None,
            lock_factory=lambda _operation: nullcontext(),
            profile_manager_factory=NoopProfileManager,
        )

    result_data = json.loads(
        (tmp_path / "run" / "result.json").read_text(encoding="utf-8")
    )
    assert "camera disconnected" in result_data["error"]


def test_start_event_failure_is_captured_without_losing_original_exception(
    tmp_path, monkeypatch
):
    original_event = preflight.Evidence.event

    def fail_start_event(self, event, **fields):
        if event == "preflight_started":
            raise RuntimeError("event sink failed")
        return original_event(self, event, **fields)

    monkeypatch.setattr(preflight.Evidence, "event", fail_start_event)

    with pytest.raises(RuntimeError, match="event sink failed"):
        preflight.execute_workflow(
            config(tmp_path),
            authorize_profile_reload=False,
            authorize_container_recreate=True,
            authorize_stationary_d455=True,
            authorize_ros_no_motion=True,
            runner=ContainerRunner(),
            discover=lambda _serial: resources(),
            serial_verifier=lambda _serial: None,
            production_exclusion_checker=lambda _runner, _evidence: None,
            lock_factory=lambda _operation: nullcontext(),
            profile_manager_factory=NoopProfileManager,
        )

    result_data = json.loads(
        (tmp_path / "run" / "result.json").read_text(encoding="utf-8")
    )
    assert result_data["result"] == "failed"
    assert "event sink failed" in result_data["error"]


def test_cleanup_failure_does_not_replace_root_runtime_failure(tmp_path):
    class CleanupFailureRunner(ContainerRunner):
        def run(self, args, *, timeout):
            command = tuple(str(item) for item in args)
            if command[:2] == ("docker", "rm"):
                self.commands.append((command, timeout))
                return preflight.CommandResult(command, 2, "", "rm failed")
            return super().run(args, timeout=timeout)

    runner = CleanupFailureRunner(runtime_status=1)

    with pytest.raises(preflight.PreflightError, match="no_motion_runtime"):
        preflight.execute_workflow(
            config(tmp_path),
            authorize_profile_reload=False,
            authorize_container_recreate=True,
            authorize_stationary_d455=True,
            authorize_ros_no_motion=True,
            runner=runner,
            discover=lambda _serial: resources(),
            serial_verifier=lambda _serial: None,
            production_exclusion_checker=lambda _runner, _evidence: None,
            lock_factory=lambda _operation: nullcontext(),
            profile_manager_factory=NoopProfileManager,
        )

    result_data = json.loads(
        (tmp_path / "run" / "result.json").read_text(encoding="utf-8")
    )
    assert "no_motion_runtime" in result_data["error"]
    cleanup_data = json.loads(
        (tmp_path / "run" / "cleanup-failure.json").read_text(
            encoding="utf-8"
        )
    )
    assert cleanup_data["phase"] == "cleanup"


def test_runtime_failure_is_fail_closed_and_removes_only_dedicated_container(
    tmp_path,
):
    runner = ContainerRunner(runtime_status=1)

    with pytest.raises(preflight.PreflightError, match="no_motion_runtime"):
        preflight.execute_workflow(
            config(tmp_path),
            authorize_profile_reload=False,
            authorize_container_recreate=True,
            authorize_stationary_d455=True,
            authorize_ros_no_motion=True,
            runner=runner,
            discover=lambda _serial: resources(),
            serial_verifier=lambda _serial: None,
            production_exclusion_checker=lambda _runner, _evidence: None,
            lock_factory=lambda _operation: nullcontext(),
            profile_manager_factory=NoopProfileManager,
        )

    assert runner.removed is True
    result_data = json.loads(
        (tmp_path / "run" / "result.json").read_text(encoding="utf-8")
    )
    assert result_data == {
        "error": (
            "no_motion_runtime_validation: command failed with exit status 1: "
            f"{sys.executable} /repo/d455_no_motion_validation.py "
            "--container pharma_realsense_imu_validation "
            "--workspace /tmp/workspace "
            f"--evidence-dir {tmp_path}/run/runtime --execute "
            "--acknowledge-exact-zero-only"
        ),
        "no_nonzero_twist": True,
        "result": "failed",
    }
    failure_data = json.loads(
        (tmp_path / "run" / "failure.json").read_text(encoding="utf-8")
    )
    assert failure_data["cleanup_result"] == "succeeded"
    assert all(
        "pharma_container" not in " ".join(command)
        for command, _timeout in runner.commands
    )


def test_no_ros_runtime_or_processor_source_is_modified_by_tool_scope():
    source = TOOL_PATH.read_text(encoding="utf-8")

    assert "d455_imu_processor.py" not in source
    assert "robot_sensors.launch.py" not in source
    assert "/dev/roboteq" in source  # rejection/proof only
    assert "geometry_msgs" not in source
    assert "ros2 topic pub" not in source
