# Copyright 2026 Medrobots Engineering
#
# Licensed under the Apache License, Version 2.0 (the "License");

"""Offline tests for production/validation D455 host mutual exclusion."""

import importlib.util
import multiprocessing
from pathlib import Path
import sys

import pytest


TOOL_PATH = (
    Path(__file__).parents[1] / "tools" / "d455_host_lock.py"
)
SPEC = importlib.util.spec_from_file_location("d455_host_lock", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
host_lock = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = host_lock
SPEC.loader.exec_module(host_lock)


def _hold_lock(path_string, ready, release):
    """Hold a real OS flock until the parent tells this process to release."""
    try:
        with host_lock.d455_host_lock(
            "production_start", path=Path(path_string)
        ):
            ready.set()
            release.wait(5)
    except BaseException as error:  # pragma: no cover - reported to parent
        ready.set()
        raise error


def test_host_lock_is_process_scoped_and_releases_after_owner_exit(tmp_path):
    """A live owner blocks another process, then the OS releases its flock."""
    context = multiprocessing.get_context("fork")
    lock_path = tmp_path / "d455.lock"
    ready = context.Event()
    release = context.Event()
    owner = context.Process(
        target=_hold_lock,
        args=(str(lock_path), ready, release),
    )
    owner.start()
    assert ready.wait(5)
    try:
        with pytest.raises(
            host_lock.D455HostLockError, match="lock is held"
        ):
            with host_lock.d455_host_lock(
                "validation_workflow", path=lock_path
            ):
                raise AssertionError("concurrent workflow entered lock")
    finally:
        release.set()
        owner.join(5)
    assert owner.exitcode == 0
    with host_lock.d455_host_lock(
        "validation_workflow", path=lock_path
    ):
        pass


def test_production_and_validation_contend_on_one_host_lock(tmp_path):
    lock_path = tmp_path / "d455.lock"
    with host_lock.d455_host_lock(
        "production_start", path=lock_path
    ):
        with pytest.raises(
            host_lock.D455HostLockError, match="lock is held"
        ):
            with host_lock.d455_host_lock(
                "validation_workflow", path=lock_path
            ):
                raise AssertionError("contending workflow entered lock")


def test_host_lock_releases_after_failure_and_can_be_reacquired(tmp_path):
    lock_path = tmp_path / "d455.lock"
    with pytest.raises(RuntimeError, match="injected failure"):
        with host_lock.d455_host_lock(
            "production_prepare", path=lock_path
        ):
            raise RuntimeError("injected failure")
    with host_lock.d455_host_lock(
        "validation_workflow", path=lock_path
    ):
        assert lock_path.is_file()


def test_host_lock_rejects_symlink_and_broad_permissions(tmp_path):
    target = tmp_path / "target"
    target.write_text("do not truncate\n", encoding="utf-8")
    symlink = tmp_path / "d455.lock"
    symlink.symlink_to(target)
    with pytest.raises(
        host_lock.D455HostLockError, match="cannot be opened"
    ):
        with host_lock.d455_host_lock(
            "production_start", path=symlink
        ):
            raise AssertionError("symlink lock was accepted")
    assert target.read_text(encoding="utf-8") == "do not truncate\n"

    symlink.unlink()
    symlink.write_text("", encoding="utf-8")
    symlink.chmod(0o644)
    with pytest.raises(
        host_lock.D455HostLockError, match="permissions are unsafe"
    ):
        with host_lock.d455_host_lock(
            "production_start", path=symlink
        ):
            raise AssertionError("broad lock mode was accepted")


@pytest.mark.parametrize(
    "operation", ("", "../escape", "UPPERCASE", "x" * 65)
)
def test_invalid_lock_operation_is_rejected(tmp_path, operation):
    with pytest.raises(
        host_lock.D455HostLockError, match="invalid"
    ):
        with host_lock.d455_host_lock(
            operation, path=tmp_path / "d455.lock"
        ):
            raise AssertionError("invalid operation acquired lock")
