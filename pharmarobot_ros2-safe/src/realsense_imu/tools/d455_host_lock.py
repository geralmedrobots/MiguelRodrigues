#!/usr/bin/env python3
"""One non-blocking host lock shared by D455 production and validation."""

from __future__ import annotations

from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import re
import stat
from typing import Iterator


DEFAULT_D455_HOST_LOCK = Path("/run/lock/pharmarobot-d455.lock")
OPERATION_PATTERN = re.compile(r"[a-z0-9][a-z0-9_-]{0,63}")


class D455HostLockError(RuntimeError):
    """Report unavailable or invalid D455 host-lock state."""


@contextmanager
def d455_host_lock(
    operation: str,
    *,
    path: Path = DEFAULT_D455_HOST_LOCK,
) -> Iterator[None]:
    """Hold the shared D455 lock for one complete host-side transition."""
    if not OPERATION_PATTERN.fullmatch(operation):
        raise D455HostLockError("invalid D455 host-lock operation")
    try:
        descriptor = os.open(
            path,
            os.O_CLOEXEC
            | os.O_CREAT
            | os.O_NOFOLLOW
            | os.O_RDWR,
            0o600,
        )
    except OSError as exc:
        raise D455HostLockError(
            f"D455 host lock cannot be opened: {path}"
        ) from exc
    try:
        lock_state = os.fstat(descriptor)
        if (
            not stat.S_ISREG(lock_state.st_mode)
            or lock_state.st_nlink != 1
            or lock_state.st_uid != os.geteuid()
            or lock_state.st_mode & 0o077
        ):
            raise D455HostLockError(
                "D455 host-lock file identity or permissions are unsafe"
            )
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            os.lseek(descriptor, 0, os.SEEK_SET)
            owner = os.read(descriptor, 4096).decode(
                "utf-8", errors="replace"
            ).strip()
            detail = owner or "owner metadata unavailable"
            raise D455HostLockError(
                f"D455 host lock is held: {detail}"
            ) from exc
        owner = json.dumps(
            {
                "operation": operation,
                "pid": os.getpid(),
                "time_utc": datetime.now(timezone.utc).isoformat(),
            },
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        os.ftruncate(descriptor, 0)
        os.lseek(descriptor, 0, os.SEEK_SET)
        os.write(descriptor, owner)
        os.fsync(descriptor)
        yield
    finally:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            os.close(descriptor)
