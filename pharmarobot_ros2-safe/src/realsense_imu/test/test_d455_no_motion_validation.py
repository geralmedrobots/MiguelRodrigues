"""Offline tests for the D455 no-motion validation wrapper."""

import importlib.util
from pathlib import Path

import pytest


TOOL_PATH = (
    Path(__file__).resolve().parents[1]
    / "tools"
    / "d455_no_motion_validation.py"
)
SPEC = importlib.util.spec_from_file_location("d455_no_motion_validation", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
validation = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(validation)


def test_echo_keeps_supported_sensor_qos_options():
    command = validation.echo_command("/imu/data")

    assert command[-4:] == [
        "--qos-reliability",
        "best_effort",
        "--qos-durability",
        "volatile",
    ]


def test_rate_command_is_bounded_and_has_no_unsupported_qos_options():
    command = validation.rate_command("/imu/data")

    assert command[:5] == [
        "timeout",
        "--foreground",
        "--signal=INT",
        "--kill-after=2s",
        "8s",
    ]
    assert command[-3:] == ["ros2", "topic", "hz"] or command[-4:] == [
        "ros2",
        "topic",
        "hz",
        "/imu/data",
    ]
    assert "--qos-reliability" not in command
    assert "--qos-durability" not in command


def test_runtime_script_preserves_exact_zero_gate_and_rate_evidence():
    script = validation.runtime_script("/tmp/work")

    assert "test ! -e /dev/roboteq" in script
    assert "test ! -e /dev/ttyUSB0" in script
    assert "--rate 20 /cmd_vel/safe" in script
    assert "x: 0.0" in script
    assert "test \"$processed_pre_status\" -eq 124" in script
    assert "collect_rate /imu/d455/data_raw" in script
    assert "collect_rate /imu/data" in script
    assert "average rate:" in script
    assert "abort_signal()" in script
    assert "trap abort_signal INT TERM" in script
    assert "--qos-reliability best_effort" not in script.split(
        "collect_rate()", 1
    )[1].split("timeout 5s ros2 topic echo /diagnostics", 1)[0]


def test_cleanup_checks_owned_process_groups_without_self_matching_pgrep():
    script = validation.runtime_script("/tmp/work")

    assert 'for owned_pgid in "${launch_pid}" "${zero_pid}"' in script
    assert 'pgrep -a -g "${owned_pgid}"' in script
    assert "OWNED_PROCESS_REMAINS=1" in script
    assert "OWNED_PROCESS_REMAINS=0" in script
    assert 'pgrep -af "[r]os2 launch' not in script
    assert 'pgrep -af "[r]os2 topic pub' not in script


def test_execute_requires_fresh_evidence_directory(tmp_path):
    existing = tmp_path / "existing"
    existing.mkdir()

    with pytest.raises(ValueError, match="already exists"):
        validation.execute("container", "/tmp/work", existing)


@pytest.mark.parametrize("workspace", ["", "relative/path", "/tmp/has\x00nul"])
def test_workspace_must_be_safe_and_absolute(workspace):
    with pytest.raises(ValueError, match="workspace"):
        validation.runtime_script(workspace)


def test_workspace_is_shell_quoted_in_runtime_script():
    script = validation.runtime_script("/tmp/work space")

    assert "workspace_path='/tmp/work space'" in script
    assert 'source "$workspace_path/install/setup.bash"' in script


def test_execute_fails_closed_when_artifact_copy_fails(tmp_path, monkeypatch):
    evidence = tmp_path / "evidence"
    monkeypatch.setattr(validation, "run_checked", lambda _command: None)

    class FailedCopy:
        returncode = 1

    monkeypatch.setattr(validation.subprocess, "run", lambda *args, **kwargs: FailedCopy())

    with pytest.raises(RuntimeError, match="copy required runtime evidence"):
        validation.execute("container", "/tmp/work", evidence)
