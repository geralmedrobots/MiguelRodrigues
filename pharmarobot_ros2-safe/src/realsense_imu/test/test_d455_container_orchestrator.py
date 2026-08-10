# Copyright 2026 Medrobots Engineering
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from dataclasses import replace
import importlib.util
import json
import os
from pathlib import Path
import sys

import pytest


TOOL_PATH = (
    Path(__file__).parents[1]
    / "tools"
    / "d455_container_orchestrator.py"
)
SPEC = importlib.util.spec_from_file_location(
    "d455_container_orchestrator", TOOL_PATH
)
assert SPEC is not None and SPEC.loader is not None
orchestrator = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = orchestrator
SPEC.loader.exec_module(orchestrator)


BASE_DIGEST = "sha256:" + "1" * 64
DERIVED_DIGEST = "sha256:" + "2" * 64
DERIVED_TAG = (
    "pharmarobot:d455-validation-workspace-20260727t120000z"
)
FULL_ID = "3" * 64
LEGACY_NAME = "pharma_realsense_imu_runtime_20260727T120000Z"


def result(status=0, stdout="", stderr="", timed_out=False):
    return orchestrator.CommandResult(
        (), status, stdout, stderr, timed_out
    )


class FakeRunner:
    def __init__(self, handler=None):
        self.handler = handler or (lambda _command, _timeout: result())
        self.commands = []

    def run(self, args, *, timeout):
        command = tuple(str(value) for value in args)
        self.commands.append((command, timeout))
        value = self.handler(command, timeout)
        return replace(value, args=command)


def make_repository(tmp_path):
    root = tmp_path / "repository"
    package = root / orchestrator.PACKAGE_RELATIVE
    (package / "tools").mkdir(parents=True)
    (package / "test").mkdir()
    (package / "realsense_imu").mkdir()
    (package / "tools" / "Dockerfile.d455_validation_workspace").write_text(
        "ARG BASE_IMAGE\nFROM ${BASE_IMAGE}\n", encoding="utf-8"
    )
    (package / "realsense_imu" / "__init__.py").write_text(
        "# package\n", encoding="utf-8"
    )
    (package / "setup.py").write_text(
        "from setuptools import setup\n", encoding="utf-8"
    )
    for relative in orchestrator.CONTRACT_INPUTS:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"{relative.as_posix()}\n", encoding="utf-8")
    return root


def snapshot(tmp_path):
    root = make_repository(tmp_path)
    destination = tmp_path / "snapshot"
    return root, orchestrator.create_snapshot(root, destination)


def evidence(tmp_path):
    return orchestrator.Evidence(tmp_path / "evidence")


def image(snapshot_result):
    return orchestrator.DerivedImage(
        digest=DERIVED_DIGEST,
        tag=DERIVED_TAG,
        base_digest=BASE_DIGEST,
        manifest_sha256=snapshot_result.manifest_sha256,
    )


def inspect_image(digest, labels=None):
    value = {"Id": digest}
    if labels is not None:
        value["Config"] = {"Labels": labels}
    return json.dumps([value])


def legacy_record(
    *,
    name=LEGACY_NAME,
    full_id=FULL_ID,
    image_digest=BASE_DIGEST,
    running=True,
    init=None,
    devices=None,
    cgroup_rules=None,
    mounts=None,
):
    host = {
        "NetworkMode": "none",
        "Privileged": False,
        "CapDrop": ["ALL"],
        "SecurityOpt": [
            "no-new-privileges:true",
            "apparmor=pharmarobot-d455-imu",
        ],
        "Devices": devices
        if devices is not None
        else [
            {
                "PathOnHost": "/dev/bus/usb/004/004",
                "PathInContainer": "/dev/bus/usb/004/004",
            },
            {
                "PathOnHost": "/dev/video0",
                "PathInContainer": "/dev/video0",
            },
        ],
        "DeviceCgroupRules": cgroup_rules or ["c 250:0 rwm"],
    }
    if init is not None:
        host["Init"] = init
    return {
        "Id": full_id,
        "Name": f"/{name}",
        "Image": image_digest,
        "HostConfig": host,
        "State": {"Running": running},
        "Config": {"Labels": {}},
        "AppArmorProfile": "pharmarobot-d455-imu",
        "Mounts": mounts
        if mounts is not None
        else [
            {
                "Source": "/dev/iio:device0",
                "Destination": "/dev/iio:device0",
            },
            {
                "Source": (
                    "/sys/devices/pci0000:00/usb4/4-3/4-3.1/"
                    "0003:8086:0B5C.0002/iio:device0"
                ),
                "Destination": (
                    "/sys/devices/pci0000:00/usb4/4-3/4-3.1/"
                    "0003:8086:0B5C.0002/iio:device0"
                ),
            },
        ],
    }


def test_snapshot_preserves_repository_paths_and_hashes_every_input(tmp_path):
    root, value = snapshot(tmp_path)
    manifest = json.loads(value.manifest_path.read_text(encoding="utf-8"))
    paths = [record["path"] for record in manifest["files"]]

    assert paths == sorted(paths)
    assert "src/realsense_imu/setup.py" in paths
    assert {item.as_posix() for item in orchestrator.CONTRACT_INPUTS}.issubset(
        paths
    )
    for record in manifest["files"]:
        copied = value.root / record["path"]
        assert copied.read_bytes() == (root / record["path"]).read_bytes()
        assert orchestrator.sha256_bytes(copied.read_bytes()) == record[
            "sha256"
        ]
    assert manifest["file_count"] == len(paths)
    assert manifest["content_manifest_sha256"]


def test_snapshot_is_deterministic_for_unchanged_inputs(tmp_path):
    root = make_repository(tmp_path)
    first = orchestrator.create_snapshot(root, tmp_path / "first")
    second = orchestrator.create_snapshot(root, tmp_path / "second")

    assert (
        first.manifest_path.read_bytes()
        == second.manifest_path.read_bytes()
    )
    assert first.manifest_sha256 == second.manifest_sha256


def test_snapshot_excludes_generated_and_validation_artifacts(tmp_path):
    root = make_repository(tmp_path)
    package = root / orchestrator.PACKAGE_RELATIVE
    for directory in (
        "validation_evidence",
        "__pycache__",
        "build",
        "install",
        "log",
    ):
        path = package / directory
        path.mkdir()
        (path / "ignored.txt").write_text("ignored", encoding="utf-8")
    (package / "ignored.pyc").write_bytes(b"ignored")

    value = orchestrator.create_snapshot(root, tmp_path / "snapshot")
    paths = value.manifest_path.read_text(encoding="utf-8")

    assert "ignored" not in paths


@pytest.mark.parametrize("kind", ["missing", "symlink", "directory"])
def test_snapshot_rejects_invalid_contract_inputs(tmp_path, kind):
    root = make_repository(tmp_path)
    target = root / orchestrator.CONTRACT_INPUTS[0]
    target.unlink()
    if kind == "symlink":
        target.symlink_to(root / orchestrator.CONTRACT_INPUTS[1])
    elif kind == "directory":
        target.mkdir()

    with pytest.raises(
        orchestrator.OrchestratorError, match="contract input"
    ):
        orchestrator.create_snapshot(root, tmp_path / "snapshot")


def test_snapshot_rejects_symlink_even_under_excluded_directory(tmp_path):
    root = make_repository(tmp_path)
    path = (
        root
        / orchestrator.PACKAGE_RELATIVE
        / "validation_evidence"
    )
    path.symlink_to(root / orchestrator.CONTRACT_INPUTS[0])

    with pytest.raises(orchestrator.OrchestratorError, match="symlink"):
        orchestrator.create_snapshot(root, tmp_path / "snapshot")


def test_snapshot_rejects_special_file(tmp_path):
    root = make_repository(tmp_path)
    fifo = root / orchestrator.PACKAGE_RELATIVE / "unexpected.fifo"
    os.mkfifo(fifo)

    with pytest.raises(orchestrator.OrchestratorError, match="special-file"):
        orchestrator.create_snapshot(root, tmp_path / "snapshot")


def test_snapshot_rejects_existing_destination(tmp_path):
    root = make_repository(tmp_path)
    destination = tmp_path / "snapshot"
    destination.mkdir()

    with pytest.raises(orchestrator.OrchestratorError, match="exists"):
        orchestrator.create_snapshot(root, destination)


def test_snapshot_rejects_file_count_bound(tmp_path, monkeypatch):
    root = make_repository(tmp_path)
    monkeypatch.setattr(orchestrator, "MAX_SNAPSHOT_FILES", 1)

    with pytest.raises(orchestrator.OrchestratorError, match="bound|too many"):
        orchestrator.create_snapshot(root, tmp_path / "snapshot")


def test_snapshot_rejects_byte_bound_and_removes_partial_tree(
    tmp_path, monkeypatch
):
    root = make_repository(tmp_path)
    monkeypatch.setattr(orchestrator, "MAX_SNAPSHOT_BYTES", 1)
    destination = tmp_path / "snapshot"

    with pytest.raises(orchestrator.OrchestratorError, match="too large"):
        orchestrator.create_snapshot(root, destination)

    assert not destination.exists()
    assert not list(tmp_path.glob(".snapshot.*"))


def test_snapshot_rejects_concurrent_input_set_mutation(
    tmp_path, monkeypatch
):
    root = make_repository(tmp_path)
    original = orchestrator.enumerate_inputs
    calls = 0

    def mutate(repository):
        nonlocal calls
        calls += 1
        values = original(repository)
        if calls == 2:
            added = (
                repository
                / orchestrator.PACKAGE_RELATIVE
                / "new_input.py"
            )
            added.write_text("new\n", encoding="utf-8")
            values = original(repository)
        return values

    monkeypatch.setattr(orchestrator, "enumerate_inputs", mutate)

    with pytest.raises(orchestrator.OrchestratorError, match="mutated"):
        orchestrator.create_snapshot(root, tmp_path / "snapshot")


def test_read_stable_file_rejects_nonregular_input(tmp_path):
    path = tmp_path / "directory"
    path.mkdir()

    with pytest.raises(orchestrator.OrchestratorError, match="not regular"):
        orchestrator.read_stable_regular_file(path)


def test_reviewed_dockerfile_runs_full_focused_build_and_tests():
    dockerfile = (
        TOOL_PATH.parent / "Dockerfile.d455_validation_workspace"
    ).read_text(encoding="utf-8")

    assert "COPY . /validation_ws" in dockerfile
    assert "--packages-select realsense_imu" in dockerfile
    assert "colcon --log-base /validation_ws/log build" in dockerfile
    assert "colcon --log-base /validation_ws/test-log test" in dockerfile
    assert "colcon test-result" in dockerfile
    assert "/validation_ws/build-evidence/test-result.txt" in dockerfile
    assert dockerfile.count("set +u;") == 2
    assert dockerfile.count("set -u;") == 2


def test_derived_build_pins_digest_network_and_manifest(tmp_path):
    _, source = snapshot(tmp_path)
    calls = 0
    labels = {
        orchestrator.DERIVED_LABEL: "true",
        orchestrator.BASE_LABEL: BASE_DIGEST,
        orchestrator.MANIFEST_LABEL: source.manifest_sha256,
    }

    def handle(command, _timeout):
        nonlocal calls
        if command[:3] == ("docker", "image", "inspect"):
            calls += 1
            if calls == 1:
                return result(stdout=inspect_image(BASE_DIGEST))
            if calls == 2:
                return result(1, stderr="not found")
            if calls == 3:
                return result(stdout=inspect_image(BASE_DIGEST))
            if calls == 4:
                return result(1, stderr="not found")
            if calls == 5:
                return result(stdout=inspect_image(DERIVED_DIGEST))
            return result(
                stdout=inspect_image(DERIVED_DIGEST, labels)
            )
        return result()

    runner = FakeRunner(handle)
    builder = orchestrator.DerivedImageBuilder(
        runner, evidence(tmp_path / "run")
    )

    value = builder.build(
        base_image="pharmarobot:reviewed",
        derived_tag=DERIVED_TAG,
        snapshot=source,
    )

    build = next(
        command
        for command, _timeout in runner.commands
        if command[:2] == ("docker", "build")
    )
    assert value.digest == DERIVED_DIGEST
    assert "--network=none" in build
    base_alias = orchestrator.base_alias_for_derived(DERIVED_TAG)
    assert f"BASE_IMAGE={base_alias}" in build
    assert f"BASE_IMAGE_DIGEST={BASE_DIGEST}" in build
    assert ("docker", "tag", BASE_DIGEST, base_alias) in [
        command for command, _timeout in runner.commands
    ]
    assert (
        f"SOURCE_MANIFEST_SHA256={source.manifest_sha256}" in build
    )
    assert str(source.root) == build[-1]


def test_derived_build_rejects_invalid_digest(tmp_path):
    _, source = snapshot(tmp_path)
    runner = FakeRunner(
        lambda _command, _timeout: result(
            stdout=json.dumps([{"Id": "latest"}])
        )
    )
    builder = orchestrator.DerivedImageBuilder(
        runner, evidence(tmp_path / "e")
    )

    with pytest.raises(orchestrator.OrchestratorError, match="digest"):
        builder.build(
            base_image="base",
            derived_tag=DERIVED_TAG,
            snapshot=source,
        )


def test_derived_build_rejects_label_mismatch(tmp_path):
    _, source = snapshot(tmp_path)
    calls = 0

    def handle(command, _timeout):
        nonlocal calls
        if command[:3] == ("docker", "image", "inspect"):
            calls += 1
            if calls in (2, 4):
                return result(1, stderr="not found")
            if calls == 1:
                return result(stdout=inspect_image(BASE_DIGEST))
            if calls == 3:
                return result(stdout=inspect_image(BASE_DIGEST))
            digest = DERIVED_DIGEST
            labels = {} if calls == 6 else None
            return result(stdout=inspect_image(digest, labels))
        return result()

    builder = orchestrator.DerivedImageBuilder(
        FakeRunner(handle), evidence(tmp_path / "e")
    )

    with pytest.raises(orchestrator.OrchestratorError, match="labels"):
        builder.build(
            base_image="base",
            derived_tag=DERIVED_TAG,
            snapshot=source,
        )


@pytest.mark.parametrize(
    "base_image,derived_tag",
    [
        ("pharmarobot:production", "pharmarobot:production"),
        ("pharmarobot:reviewed", "pharmarobot:latest"),
        ("pharmarobot:reviewed", "pharmarobot:d455-validation"),
        (
            DERIVED_TAG,
            DERIVED_TAG,
        ),
    ],
)
def test_derived_tag_rejects_production_outside_namespace_and_base_alias(
    base_image, derived_tag
):
    with pytest.raises(orchestrator.OrchestratorError):
        orchestrator.validate_derived_tag(base_image, derived_tag)


def test_derived_build_rejects_preexisting_namespace_tag(tmp_path):
    _, source = snapshot(tmp_path)
    calls = 0

    def handle(command, _timeout):
        nonlocal calls
        if command[:3] == ("docker", "image", "inspect"):
            calls += 1
            digest = BASE_DIGEST if calls == 1 else DERIVED_DIGEST
            return result(stdout=inspect_image(digest))
        return result()

    runner = FakeRunner(handle)
    builder = orchestrator.DerivedImageBuilder(
        runner, evidence(tmp_path / "e")
    )

    with pytest.raises(orchestrator.OrchestratorError, match="fresh"):
        builder.build(
            base_image="pharmarobot:reviewed",
            derived_tag=DERIVED_TAG,
            snapshot=source,
        )

    assert not any(
        command[:2] == ("docker", "build")
        for command, _timeout in runner.commands
    )


def test_base_alias_digest_mismatch_fails_before_build_and_is_evidenced(
    tmp_path,
):
    _, source = snapshot(tmp_path)
    calls = 0

    def handle(command, _timeout):
        nonlocal calls
        if command[:3] == ("docker", "image", "inspect"):
            calls += 1
            if calls == 1:
                return result(stdout=inspect_image(BASE_DIGEST))
            if calls == 2:
                return result(1, stderr="not found")
            return result(stdout=inspect_image(DERIVED_DIGEST))
        return result()

    ev = evidence(tmp_path / "e")
    builder = orchestrator.DerivedImageBuilder(FakeRunner(handle), ev)

    with pytest.raises(orchestrator.OrchestratorError, match="alias digest"):
        builder.build(
            base_image="pharmarobot:reviewed",
            derived_tag=DERIVED_TAG,
            snapshot=source,
        )

    saved = json.loads(
        (ev.root / "base-alias.json").read_text(encoding="utf-8")
    )
    assert saved["status"] == "digest_mismatch"
    assert not any(
        command[:2] == ("docker", "build")
        for command, _timeout in builder.runner.commands
    )


def test_base_alias_cleanup_requires_exact_digest_ownership(tmp_path):
    alias = orchestrator.base_alias_for_derived(DERIVED_TAG)

    def handle(command, _timeout):
        if command[:3] == ("docker", "image", "inspect"):
            return result(stdout=inspect_image(BASE_DIGEST))
        return result()

    runner = FakeRunner(handle)
    builder = orchestrator.DerivedImageBuilder(runner, evidence(tmp_path))
    builder.base_alias_tag = alias
    builder.base_alias_digest = BASE_DIGEST

    builder.cleanup_base_alias()

    assert runner.commands[-1][0] == ("docker", "image", "rm", alias)
    assert builder.base_alias_tag is None


def test_base_alias_cleanup_timeout_preserves_alias_and_fails(tmp_path):
    alias = orchestrator.base_alias_for_derived(DERIVED_TAG)
    runner = FakeRunner(
        lambda _command, _timeout: result(124, timed_out=True)
    )
    builder = orchestrator.DerivedImageBuilder(runner, evidence(tmp_path))
    builder.base_alias_tag = alias
    builder.base_alias_digest = BASE_DIGEST

    with pytest.raises(orchestrator.OrchestratorError, match="timed out"):
        builder.cleanup_base_alias()

    assert builder.base_alias_tag == alias
    assert not any(
        command[:3] == ("docker", "image", "rm")
        for command, _timeout in runner.commands
    )


def test_base_alias_cleanup_digest_drift_refuses_removal(tmp_path):
    alias = orchestrator.base_alias_for_derived(DERIVED_TAG)
    runner = FakeRunner(
        lambda _command, _timeout: result(
            stdout=inspect_image(DERIVED_DIGEST)
        )
    )
    builder = orchestrator.DerivedImageBuilder(runner, evidence(tmp_path))
    builder.base_alias_tag = alias
    builder.base_alias_digest = BASE_DIGEST

    with pytest.raises(orchestrator.OrchestratorError, match="changed"):
        builder.cleanup_base_alias()

    assert not any(
        command[:3] == ("docker", "image", "rm")
        for command, _timeout in runner.commands
    )


def test_derived_verification_uses_no_network_and_copies_evidence(tmp_path):
    _, source = snapshot(tmp_path)

    def handle(command, _timeout):
        if command[:2] == ("docker", "cp"):
            destination = Path(command[-1])
            if "snapshot-manifest.json" in command[-2]:
                destination.write_bytes(source.manifest_path.read_bytes())
            else:
                destination.write_text("all tests passed\n", encoding="utf-8")
        return result()

    runner = FakeRunner(handle)
    ev = evidence(tmp_path / "run")
    builder = orchestrator.DerivedImageBuilder(runner, ev)

    builder.verify(image(source), source)

    create = runner.commands[0][0]
    assert create[:2] == ("docker", "create")
    assert "--network=none" in create
    assert "--cap-drop=ALL" in create
    assert "--security-opt=no-new-privileges:true" in create
    assert (ev.root / "derived-test-result.txt").is_file()


def test_derived_verification_rejects_mismatched_manifest(tmp_path):
    _, source = snapshot(tmp_path)

    def handle(command, _timeout):
        if command[:2] == ("docker", "cp"):
            destination = Path(command[-1])
            destination.write_text("mismatch\n", encoding="utf-8")
        return result()

    builder = orchestrator.DerivedImageBuilder(
        FakeRunner(handle), evidence(tmp_path / "e")
    )

    with pytest.raises(orchestrator.OrchestratorError, match="does not match"):
        builder.verify(image(source), source)


def test_bounded_timeout_is_recorded_and_fails_closed(tmp_path):
    runner = FakeRunner(
        lambda _command, _timeout: result(124, timed_out=True)
    )
    ev = evidence(tmp_path)

    with pytest.raises(orchestrator.OrchestratorError, match="timed out"):
        orchestrator.run_checked(
            runner, ev, "bounded", ["command"], timeout=0.5
        )

    saved = json.loads(
        (ev.root / "command-001.json").read_text(encoding="utf-8")
    )
    assert saved["timed_out"] is True
    assert saved["timeout_seconds"] == 0.5


def test_verifier_cleanup_is_deterministic(tmp_path):
    runner = FakeRunner()
    builder = orchestrator.DerivedImageBuilder(runner, evidence(tmp_path))
    builder.verifier_name = "verify-name"

    builder.cleanup_verifier()

    assert runner.commands[-1][0] == (
        "docker",
        "rm",
        "-f",
        "verify-name",
    )
    assert builder.verifier_name is None


def test_failed_build_rollback_removes_only_derived_tag(tmp_path):
    labels = {
        orchestrator.DERIVED_LABEL: "true",
        orchestrator.BASE_LABEL: BASE_DIGEST,
        orchestrator.MANIFEST_LABEL: "a" * 64,
    }

    def handle(command, _timeout):
        if command[:3] == ("docker", "image", "inspect"):
            return result(
                stdout=inspect_image(DERIVED_DIGEST, labels)
            )
        return result()

    runner = FakeRunner(handle)
    builder = orchestrator.DerivedImageBuilder(runner, evidence(tmp_path))
    builder.built_tag = DERIVED_TAG
    builder.built_base_digest = BASE_DIGEST
    builder.built_manifest_sha256 = "a" * 64

    builder.rollback_failed_build()

    assert runner.commands[-1][0] == (
        "docker",
        "image",
        "rm",
        DERIVED_TAG,
    )


def test_failed_build_rollback_does_not_remove_unowned_namespace_tag(
    tmp_path,
):
    runner = FakeRunner(
        lambda command, _timeout: result(
            stdout=inspect_image(DERIVED_DIGEST, {})
        )
        if command[:3] == ("docker", "image", "inspect")
        else result()
    )
    builder = orchestrator.DerivedImageBuilder(runner, evidence(tmp_path))
    builder.built_tag = DERIVED_TAG
    builder.built_base_digest = BASE_DIGEST
    builder.built_manifest_sha256 = "a" * 64

    with pytest.raises(orchestrator.OrchestratorError, match="owned"):
        builder.rollback_failed_build()

    assert not any(
        command[:3] == ("docker", "image", "rm")
        for command, _timeout in runner.commands
    )


def test_failed_build_rollback_rejects_nonvalidation_tag(tmp_path):
    runner = FakeRunner()
    builder = orchestrator.DerivedImageBuilder(runner, evidence(tmp_path))
    builder.built_tag = "pharmarobot:production"

    with pytest.raises(orchestrator.OrchestratorError, match="namespace"):
        builder.rollback_failed_build()

    assert not runner.commands


@pytest.mark.parametrize(
    "change,match",
    [
        ({"Name": "/unrelated"}, "identity/isolation"),
        ({"Id": "4" * 64}, "identity/isolation"),
        ({"Image": DERIVED_DIGEST}, "identity/isolation"),
        ({"AppArmorProfile": "unconfined"}, "identity/isolation"),
    ],
)
def test_legacy_proof_rejects_identity_or_isolation_drift(change, match):
    value = legacy_record()
    value.update(change)

    with pytest.raises(orchestrator.OrchestratorError, match=match):
        orchestrator.LegacyQuarantine._validate_inspect(
            value,
            expected_name=LEGACY_NAME,
            expected_id=FULL_ID,
            expected_image_digest=BASE_DIGEST,
        )


@pytest.mark.parametrize(
    "devices,mounts,match",
    [
        (
            [
                {
                    "PathOnHost": "/dev/roboteq",
                    "PathInContainer": "/dev/roboteq",
                }
            ],
            None,
            "forbidden",
        ),
        (
            [
                {
                    "PathOnHost": "/dev/random",
                    "PathInContainer": "/dev/random",
                }
            ],
            None,
            "unexpected",
        ),
        (
            None,
            [{"Source": "/sys", "Destination": "/sys"}],
            "unexpected",
        ),
    ],
)
def test_legacy_proof_rejects_broad_or_motor_access(
    devices, mounts, match
):
    value = legacy_record(devices=devices, mounts=mounts)

    with pytest.raises(orchestrator.OrchestratorError, match=match):
        orchestrator.LegacyQuarantine._validate_inspect(
            value,
            expected_name=LEGACY_NAME,
            expected_id=FULL_ID,
            expected_image_digest=BASE_DIGEST,
        )


def test_stopped_legacy_requires_inspectable_init_proof(tmp_path):
    record = legacy_record(running=False, init=None)
    runner = FakeRunner(
        lambda _command, _timeout: result(stdout=json.dumps([record]))
    )
    quarantine = orchestrator.LegacyQuarantine(
        runner, evidence(tmp_path)
    )

    with pytest.raises(orchestrator.OrchestratorError, match="PID 1"):
        quarantine.quarantine(
            name=LEGACY_NAME,
            full_id=FULL_ID,
            expected_image_digest=BASE_DIGEST,
            authorize=True,
        )

    assert not any(
        command[:2] == ("docker", "rename")
        for command, _timeout in runner.commands
    )


def test_quarantine_proves_name_before_stopping_running_legacy(tmp_path):
    record = legacy_record(running=True)

    def handle(command, _timeout):
        if command[:2] == ("docker", "inspect") and command[-1] == FULL_ID:
            return result(stdout=json.dumps([record]))
        if command[:2] == ("docker", "inspect"):
            return result(stdout=json.dumps([{"Id": "occupied"}]))
        return result()

    runner = FakeRunner(handle)
    quarantine = orchestrator.LegacyQuarantine(
        runner, evidence(tmp_path)
    )

    with pytest.raises(orchestrator.OrchestratorError, match="not proven"):
        quarantine.quarantine(
            name=LEGACY_NAME,
            full_id=FULL_ID,
            expected_image_digest=BASE_DIGEST,
            authorize=True,
        )

    assert not any(
        command[:2] == ("docker", "stop")
        for command, _timeout in runner.commands
    )


def test_running_legacy_quarantine_is_strict_and_reversible(tmp_path):
    current_name = LEGACY_NAME
    running = True
    quarantine_name = "pharma_realsense_imu_quarantine_" + FULL_ID[:12]

    def handle(command, _timeout):
        nonlocal current_name, running
        if command[:2] == ("docker", "inspect"):
            reference = command[-1]
            if (
                reference == quarantine_name
                and current_name != quarantine_name
            ):
                return result(1, stderr="not found")
            if reference == LEGACY_NAME and current_name != LEGACY_NAME:
                return result(1, stderr="not found")
            return result(
                stdout=json.dumps(
                    [
                        legacy_record(
                            name=current_name,
                            running=running,
                            init=True,
                        )
                    ]
                )
            )
        if command[:2] == ("docker", "stop"):
            running = False
        if command[:2] == ("docker", "rename"):
            current_name = command[-1]
        return result()

    runner = FakeRunner(handle)
    ev = evidence(tmp_path)
    quarantine = orchestrator.LegacyQuarantine(runner, ev)

    value = quarantine.quarantine(
        name=LEGACY_NAME,
        full_id=FULL_ID,
        expected_image_digest=BASE_DIGEST,
        authorize=True,
    )

    assert value.quarantine_name == quarantine_name
    assert current_name == quarantine_name
    commands = [command for command, _timeout in runner.commands]
    assert any(command[:2] == ("docker", "exec") for command in commands)
    assert any(command[:2] == ("docker", "stop") for command in commands)
    assert not any(command[:2] == ("docker", "rm") for command in commands)
    assert not any(
        command[:2] == ("docker", "restart") for command in commands
    )

    quarantine.rollback_identity()

    assert current_name == LEGACY_NAME
    assert running is False
    assert quarantine.migration is None


def test_timed_out_stop_reconciles_side_effect_and_never_renames(tmp_path):
    running = True
    quarantine_name = "pharma_realsense_imu_quarantine_" + FULL_ID[:12]

    def handle(command, _timeout):
        nonlocal running
        if command[:2] == ("docker", "inspect"):
            if command[-1] == quarantine_name:
                return result(1, stderr="not found")
            return result(
                stdout=json.dumps(
                    [legacy_record(running=running, init=True)]
                )
            )
        if command[:2] == ("docker", "stop"):
            running = False
            return result(124, timed_out=True)
        return result()

    runner = FakeRunner(handle)
    ev = evidence(tmp_path)
    quarantine = orchestrator.LegacyQuarantine(runner, ev)

    with pytest.raises(orchestrator.OrchestratorError, match="side effect"):
        quarantine.quarantine(
            name=LEGACY_NAME,
            full_id=FULL_ID,
            expected_image_digest=BASE_DIGEST,
            authorize=True,
        )

    state = json.loads(
        (ev.root / "legacy-migration-state.json").read_text(
            encoding="utf-8"
        )
    )
    assert state["status"] == "stopped_after_ambiguous_stop"
    assert running is False
    assert not any(
        command[:2] == ("docker", "rename")
        for command, _timeout in runner.commands
    )


def test_timed_out_rename_is_reconciled_and_rolled_back(tmp_path):
    current_name = LEGACY_NAME
    quarantine_name = "pharma_realsense_imu_quarantine_" + FULL_ID[:12]
    rename_calls = 0

    def handle(command, _timeout):
        nonlocal current_name, rename_calls
        if command[:2] == ("docker", "inspect"):
            reference = command[-1]
            if (
                reference == quarantine_name
                and current_name != quarantine_name
            ):
                return result(1, stderr="not found")
            if reference == LEGACY_NAME and current_name != LEGACY_NAME:
                return result(1, stderr="not found")
            return result(
                stdout=json.dumps(
                    [
                        legacy_record(
                            name=current_name,
                            running=False,
                            init=True,
                        )
                    ]
                )
            )
        if command[:2] == ("docker", "rename"):
            rename_calls += 1
            current_name = command[-1]
            if rename_calls == 1:
                return result(124, timed_out=True)
        return result()

    runner = FakeRunner(handle)
    ev = evidence(tmp_path)
    quarantine = orchestrator.LegacyQuarantine(runner, ev)

    with pytest.raises(orchestrator.OrchestratorError, match="restored"):
        quarantine.quarantine(
            name=LEGACY_NAME,
            full_id=FULL_ID,
            expected_image_digest=BASE_DIGEST,
            authorize=True,
        )

    state = json.loads(
        (ev.root / "legacy-migration-state.json").read_text(
            encoding="utf-8"
        )
    )
    assert state["status"] == "rolled_back"
    assert current_name == LEGACY_NAME
    assert rename_calls == 2
    assert quarantine.migration is None


def test_uninspectable_timed_out_rename_records_unresolved_state(tmp_path):
    inspect_full_calls = 0
    quarantine_name = "pharma_realsense_imu_quarantine_" + FULL_ID[:12]

    def handle(command, _timeout):
        nonlocal inspect_full_calls
        if command[:2] == ("docker", "inspect"):
            if command[-1] == quarantine_name:
                return result(1, stderr="not found")
            inspect_full_calls += 1
            if inspect_full_calls == 1:
                return result(
                    stdout=json.dumps(
                        [legacy_record(running=False, init=True)]
                    )
                )
            return result(124, timed_out=True)
        if command[:2] == ("docker", "rename"):
            return result(124, timed_out=True)
        return result()

    ev = evidence(tmp_path)
    quarantine = orchestrator.LegacyQuarantine(
        FakeRunner(handle), ev
    )

    with pytest.raises(orchestrator.OrchestratorError, match="reconciled"):
        quarantine.quarantine(
            name=LEGACY_NAME,
            full_id=FULL_ID,
            expected_image_digest=BASE_DIGEST,
            authorize=True,
        )

    state = json.loads(
        (ev.root / "legacy-migration-state.json").read_text(
            encoding="utf-8"
        )
    )
    assert state["status"] == "unresolved"
    assert state["phase"] == "legacy_rename_reconcile"
    assert quarantine.migration is None


def test_process_proof_failure_never_stops_or_renames_legacy(tmp_path):
    record = legacy_record(running=True)

    def handle(command, _timeout):
        if command[:2] == ("docker", "inspect") and command[-1] == FULL_ID:
            return result(stdout=json.dumps([record]))
        if command[:2] == ("docker", "inspect"):
            return result(1, stderr="not found")
        if command[:2] == ("docker", "exec"):
            return result(1, stderr="unexpected ROS process")
        return result()

    runner = FakeRunner(handle)
    quarantine = orchestrator.LegacyQuarantine(
        runner, evidence(tmp_path)
    )

    with pytest.raises(orchestrator.OrchestratorError, match="exit status"):
        quarantine.quarantine(
            name=LEGACY_NAME,
            full_id=FULL_ID,
            expected_image_digest=BASE_DIGEST,
            authorize=True,
        )

    commands = [command for command, _timeout in runner.commands]
    assert not any(command[:2] == ("docker", "stop") for command in commands)
    assert not any(command[:2] == ("docker", "rename") for command in commands)


def test_quarantine_requires_explicit_authorization(tmp_path):
    quarantine = orchestrator.LegacyQuarantine(
        FakeRunner(), evidence(tmp_path)
    )

    with pytest.raises(orchestrator.AuthorizationError):
        quarantine.quarantine(
            name=LEGACY_NAME,
            full_id=FULL_ID,
            expected_image_digest=BASE_DIGEST,
            authorize=False,
        )


def test_host_preflight_receives_only_immutable_image_and_workspace(tmp_path):
    config = orchestrator.OrchestratorConfig(
        base_image="base",
        derived_tag="tag",
        target_container="pharma_realsense_imu_validation_20260727T120000Z",
        evidence_dir=tmp_path / "evidence",
    )
    source = orchestrator.SnapshotResult(
        root=tmp_path,
        manifest_path=tmp_path / "manifest",
        manifest_sha256="a" * 64,
        file_count=1,
        total_bytes=1,
    )

    command = orchestrator.host_preflight_command(
        config, image(source), authorize_profile_reload=True
    )

    assert command[command.index("--image") + 1] == DERIVED_DIGEST
    assert command[command.index("--workspace") + 1] == "/validation_ws"
    assert "--authorize-profile-reload" in command
    assert "--authorize-container-recreate" in command
    assert "--authorize-stationary-d455" in command
    assert "--authorize-ros-no-motion" in command
    assert not any("apparmor_parser" in argument for argument in command)
    assert not any("--device" in argument for argument in command)


def test_host_preflight_rejects_nonvalidation_target(tmp_path):
    config = orchestrator.OrchestratorConfig(
        base_image="base",
        derived_tag="tag",
        target_container="pharma_container",
        evidence_dir=tmp_path,
    )
    source = orchestrator.SnapshotResult(
        root=tmp_path,
        manifest_path=tmp_path / "manifest",
        manifest_sha256="a" * 64,
        file_count=1,
        total_bytes=1,
    )

    with pytest.raises(orchestrator.OrchestratorError, match="namespace"):
        orchestrator.host_preflight_command(
            config, image(source), authorize_profile_reload=False
        )


def test_execute_requires_all_runtime_authorizations_before_evidence(tmp_path):
    config = orchestrator.OrchestratorConfig(
        base_image="base",
        derived_tag="tag",
        target_container="pharma_realsense_imu_validation",
        evidence_dir=tmp_path / "evidence",
    )

    with pytest.raises(orchestrator.AuthorizationError):
        orchestrator.execute(
            config,
            authorize_build=True,
            authorize_profile_reload=False,
            authorize_container_recreate=True,
            authorize_stationary_d455=True,
            authorize_ros_no_motion=False,
            authorize_legacy_quarantine=False,
            runner=FakeRunner(),
            repository_root=tmp_path,
        )

    assert not config.evidence_dir.exists()


def test_cli_requires_execute_flag():
    with pytest.raises(SystemExit, match="without --execute"):
        orchestrator.main(
            [
                "--base-image",
                "base",
                "--derived-tag",
                "tag",
                "--target-container",
                "pharma_realsense_imu_validation",
                "--evidence-dir",
                "/tmp/not-created-by-test",
            ]
        )


def test_orchestrator_contains_no_runtime_discovery_or_security_reload_logic():
    source = TOOL_PATH.read_text(encoding="utf-8")

    assert "usb_device" not in source
    assert "apparmor_parser" not in source
    assert "sudo" not in source
    assert "/dev/roboteq" in source
    assert "docker run" not in source
    assert "ros2 launch" not in source
    assert "Twist" not in source
