#!/usr/bin/env python3
"""Build an immutable D455 validation workspace and invoke host preflight.

The orchestrator owns source snapshotting, derived-image verification, and an
optional one-time quarantine of a strictly proven legacy validation container.
It intentionally delegates all D455 discovery, AppArmor, exact device scope,
and final container lifecycle to ``d455_host_preflight.py``.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import sys
import tempfile
from typing import Any, Iterable, Mapping, Optional, Sequence


TOOL_VERSION = "1"
PACKAGE_RELATIVE = Path("src/realsense_imu")
CONTRACT_INPUTS = (
    Path("Dockerfile"),
    Path("deployment/scripts/pharma_start_container.sh"),
    Path("deployment/scripts/build_core.sh"),
    Path("deployment/scripts/pharma_run_sensors.sh"),
    Path("deployment/scripts/pharma_stop_sensors.sh"),
    Path("deployment/systemd/pharma-d455-imu.service"),
    Path("deployment/systemd/pharma-minimal-nodes.service"),
)
EXCLUDED_DIRECTORY_NAMES = {
    "__pycache__",
    ".pytest_cache",
    "validation_evidence",
    "build",
    "install",
    "log",
    "test-log",
    ".mypy_cache",
    ".ruff_cache",
}
EXCLUDED_FILE_NAMES = {
    ".coverage",
}
EXCLUDED_SUFFIXES = {
    ".pyc",
    ".pyo",
    ".swp",
    ".tmp",
}
MAX_SNAPSHOT_FILES = 2048
MAX_SNAPSHOT_BYTES = 32 * 1024 * 1024
COMMAND_TIMEOUT_SECONDS = 30.0
BUILD_TIMEOUT_SECONDS = 900.0
HOST_PREFLIGHT_TIMEOUT_SECONDS = 300.0
DERIVED_LABEL = "pharmarobot.d455.validation-workspace"
BASE_LABEL = "pharmarobot.d455.base-image-digest"
MANIFEST_LABEL = "pharmarobot.d455.source-manifest-sha256"
TARGET_NAME_PATTERN = re.compile(
    r"pharma_realsense_imu_validation(?:_[A-Za-z0-9_.-]+)?"
)
LEGACY_NAME_PATTERN = re.compile(
    r"pharma_realsense_imu_(?:runtime|validation)"
    r"(?:_[0-9]{8}T[0-9]{6}Z)?"
)
FULL_CONTAINER_ID = re.compile(r"[0-9a-f]{64}")
IMAGE_DIGEST = re.compile(r"sha256:[0-9a-f]{64}")
DERIVED_TAG_PATTERN = re.compile(
    r"pharmarobot:d455-validation-workspace-"
    r"[a-z0-9][a-z0-9_.-]{0,79}"
)
BASE_ALIAS_TAG_PATTERN = re.compile(
    r"pharmarobot:d455-validation-base-"
    r"[a-z0-9][a-z0-9_.-]{0,79}"
)
HOST_PREFLIGHT_PATH = (
    Path(__file__).resolve().with_name("d455_host_preflight.py")
)
DOCKERFILE_PATH = (
    Path(__file__).resolve().with_name(
        "Dockerfile.d455_validation_workspace"
    )
)
REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


class OrchestratorError(RuntimeError):
    """A deterministic, fail-closed orchestration failure."""

    def __init__(self, phase: str, message: str):
        super().__init__(f"{phase}: {message}")
        self.phase = phase
        self.message = message


class AuthorizationError(OrchestratorError):
    """A separately approved operation was not acknowledged."""


@dataclass(frozen=True)
class CommandResult:
    args: tuple[str, ...]
    returncode: int
    stdout: str = ""
    stderr: str = ""
    timed_out: bool = False


@dataclass(frozen=True)
class FileRecord:
    path: str
    sha256: str
    size: int
    mode: str


@dataclass(frozen=True)
class SnapshotResult:
    root: Path
    manifest_path: Path
    manifest_sha256: str
    file_count: int
    total_bytes: int


@dataclass(frozen=True)
class DerivedImage:
    digest: str
    tag: str
    base_digest: str
    manifest_sha256: str


@dataclass(frozen=True)
class LegacyMigration:
    original_name: str
    full_id: str
    quarantine_name: str
    was_running: bool


@dataclass(frozen=True)
class OrchestratorConfig:
    base_image: str
    derived_tag: str
    target_container: str
    evidence_dir: Path
    legacy_name: Optional[str] = None
    legacy_id: Optional[str] = None


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def canonical_json(value: Mapping[str, Any]) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    )


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def atomic_write(path: Path, value: bytes, *, mode: int = 0o600) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        dir=str(path.parent),
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as output:
            output.write(value)
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary_path, mode)
        os.replace(temporary_path, path)
        directory_descriptor = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
    finally:
        if temporary_path.exists():
            temporary_path.unlink()


def atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    atomic_write(
        path,
        (json.dumps(value, indent=2, sort_keys=True) + "\n").encode(),
    )


class Runner:
    """Run bounded commands without a shell."""

    def run(
        self,
        args: Sequence[str],
        *,
        timeout: float = COMMAND_TIMEOUT_SECONDS,
    ) -> CommandResult:
        command = tuple(str(argument) for argument in args)
        try:
            completed = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired as exc:
            return CommandResult(
                command,
                124,
                exc.stdout or "",
                exc.stderr or "",
                True,
            )
        except OSError as exc:
            return CommandResult(command, 127, "", str(exc))
        return CommandResult(
            command,
            completed.returncode,
            completed.stdout,
            completed.stderr,
        )


class Evidence:
    """Persist fresh orchestration evidence."""

    def __init__(self, root: Path):
        if root.exists():
            raise OrchestratorError(
                "evidence", f"evidence path already exists: {root}"
            )
        root.mkdir(parents=True)
        self.root = root
        self.index = 0

    def event(self, event: str, **values: Any) -> None:
        record = {"event": event, "time_utc": utc_now(), **values}
        path = self.root / "events.jsonl"
        with path.open("a", encoding="utf-8") as output:
            output.write(canonical_json(record) + "\n")
            output.flush()
            os.fsync(output.fileno())

    def command(
        self,
        phase: str,
        result: CommandResult,
        timeout: float,
    ) -> None:
        self.index += 1
        atomic_json(
            self.root / f"command-{self.index:03d}.json",
            {
                "phase": phase,
                "args": list(result.args),
                "timeout_seconds": timeout,
                "returncode": result.returncode,
                "timed_out": result.timed_out,
                "stdout": result.stdout,
                "stderr": result.stderr,
            },
        )
        self.event(
            "command",
            phase=phase,
            command_index=self.index,
            returncode=result.returncode,
            timed_out=result.timed_out,
        )


def run_checked(
    runner: Any,
    evidence: Evidence,
    phase: str,
    args: Sequence[str],
    *,
    timeout: float = COMMAND_TIMEOUT_SECONDS,
    accepted: Iterable[int] = (0,),
) -> CommandResult:
    result = runner.run(args, timeout=timeout)
    evidence.command(phase, result, timeout)
    if result.timed_out:
        raise OrchestratorError(
            phase, f"command timed out after {timeout}s"
        )
    if result.returncode not in tuple(accepted):
        raise OrchestratorError(
            phase,
            f"command failed with exit status {result.returncode}",
        )
    return result


def _excluded(relative: Path) -> bool:
    return (
        any(part in EXCLUDED_DIRECTORY_NAMES for part in relative.parts)
        or relative.name in EXCLUDED_FILE_NAMES
        or relative.suffix in EXCLUDED_SUFFIXES
        or relative.name.endswith("~")
    )


def _walk_regular_files(root: Path, relative_root: Path) -> list[Path]:
    """Enumerate without following symlinks and reject special inputs."""
    start = root / relative_root
    if not start.is_dir() or start.is_symlink():
        raise OrchestratorError(
            "snapshot", f"required directory is invalid: {relative_root}"
        )
    files: list[Path] = []
    pending = [relative_root]
    while pending:
        relative_directory = pending.pop()
        absolute_directory = root / relative_directory
        try:
            entries = sorted(
                os.scandir(absolute_directory),
                key=lambda entry: entry.name,
            )
        except OSError as exc:
            raise OrchestratorError(
                "snapshot",
                f"cannot enumerate {relative_directory}: {exc}",
            ) from exc
        for entry in entries:
            relative = relative_directory / entry.name
            try:
                status = entry.stat(follow_symlinks=False)
            except OSError as exc:
                raise OrchestratorError(
                    "snapshot", f"cannot stat {relative}: {exc}"
                ) from exc
            if stat.S_ISLNK(status.st_mode):
                raise OrchestratorError(
                    "snapshot", f"symlink input is forbidden: {relative}"
                )
            if _excluded(relative) or entry.name.endswith(".egg-info"):
                continue
            if stat.S_ISDIR(status.st_mode):
                pending.append(relative)
            elif stat.S_ISREG(status.st_mode):
                files.append(relative)
            else:
                raise OrchestratorError(
                    "snapshot", f"special-file input is forbidden: {relative}"
                )
            if len(files) + len(pending) > MAX_SNAPSHOT_FILES * 2:
                raise OrchestratorError(
                    "snapshot", "input enumeration exceeded its bound"
                )
    return sorted(files)


def enumerate_inputs(root: Path) -> list[Path]:
    """Return the closed, deterministic input set."""
    package_files = _walk_regular_files(root, PACKAGE_RELATIVE)
    contract_files = []
    for relative in CONTRACT_INPUTS:
        absolute = root / relative
        try:
            status = absolute.lstat()
        except OSError as exc:
            raise OrchestratorError(
                "snapshot", f"required contract input is missing: {relative}"
            ) from exc
        if stat.S_ISLNK(status.st_mode) or not stat.S_ISREG(status.st_mode):
            raise OrchestratorError(
                "snapshot",
                f"contract input is not a regular file: {relative}",
            )
        contract_files.append(relative)
    values = sorted(set(package_files + contract_files))
    if len(values) > MAX_SNAPSHOT_FILES:
        raise OrchestratorError("snapshot", "too many snapshot inputs")
    return values


def _same_file_state(first: os.stat_result, second: os.stat_result) -> bool:
    fields = (
        "st_dev",
        "st_ino",
        "st_mode",
        "st_size",
        "st_mtime_ns",
    )
    return all(
        getattr(first, field) == getattr(second, field)
        for field in fields
    )


def read_stable_regular_file(path: Path) -> tuple[bytes, os.stat_result]:
    """Read one bounded file while detecting swaps and concurrent mutation."""
    before = path.lstat()
    if not stat.S_ISREG(before.st_mode):
        raise OrchestratorError("snapshot", f"input is not regular: {path}")
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        opened = os.fstat(descriptor)
        if not _same_file_state(before, opened):
            raise OrchestratorError(
                "snapshot", f"input changed before open: {path}"
            )
        chunks = []
        total = 0
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            total += len(chunk)
            if total > MAX_SNAPSHOT_BYTES:
                raise OrchestratorError(
                    "snapshot", f"individual input is too large: {path}"
                )
            chunks.append(chunk)
        after_open = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    after_path = path.lstat()
    if not _same_file_state(before, after_open) or not _same_file_state(
        before, after_path
    ):
        raise OrchestratorError(
            "snapshot", f"input mutated during snapshot: {path}"
        )
    return b"".join(chunks), before


def create_snapshot(
    repository_root: Path,
    destination: Path,
) -> SnapshotResult:
    """Create one atomically published repository-relative build context."""
    root = repository_root.resolve()
    if destination.exists():
        raise OrchestratorError(
            "snapshot", f"snapshot destination exists: {destination}"
        )
    initial_inputs = enumerate_inputs(root)
    temporary = Path(
        tempfile.mkdtemp(
            prefix=f".{destination.name}.",
            dir=str(destination.parent),
        )
    )
    records = []
    total_bytes = 0
    try:
        for relative in initial_inputs:
            content, status = read_stable_regular_file(root / relative)
            total_bytes += len(content)
            if total_bytes > MAX_SNAPSHOT_BYTES:
                raise OrchestratorError(
                    "snapshot", "snapshot byte limit exceeded"
                )
            target = temporary / relative
            executable = bool(status.st_mode & stat.S_IXUSR)
            atomic_write(
                target,
                content,
                mode=0o755 if executable else 0o644,
            )
            records.append(
                FileRecord(
                    path=relative.as_posix(),
                    sha256=sha256_bytes(content),
                    size=len(content),
                    mode="0755" if executable else "0644",
                )
            )
        if enumerate_inputs(root) != initial_inputs:
            raise OrchestratorError(
                "snapshot", "input set mutated during snapshot"
            )
        content_manifest = {
            "schema_version": 1,
            "tool": "d455_container_orchestrator",
            "tool_version": TOOL_VERSION,
            "files": [asdict(record) for record in records],
            "file_count": len(records),
            "total_bytes": total_bytes,
        }
        content_hash = sha256_bytes(
            canonical_json(content_manifest).encode()
        )
        manifest = {
            **content_manifest,
            "content_manifest_sha256": content_hash,
        }
        manifest_path = temporary / "snapshot-manifest.json"
        atomic_json(manifest_path, manifest)
        os.replace(temporary, destination)
        directory_descriptor = os.open(destination.parent, os.O_RDONLY)
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
        return SnapshotResult(
            root=destination,
            manifest_path=destination / "snapshot-manifest.json",
            manifest_sha256=sha256_bytes(
                (destination / "snapshot-manifest.json").read_bytes()
            ),
            file_count=len(records),
            total_bytes=total_bytes,
        )
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


def parse_single_inspect(stdout: str, phase: str) -> Mapping[str, Any]:
    try:
        values = json.loads(stdout)
        if len(values) != 1 or not isinstance(values[0], dict):
            raise ValueError
        return values[0]
    except (json.JSONDecodeError, TypeError, ValueError) as exc:
        raise OrchestratorError(
            phase, "Docker inspect output is not exactly one object"
        ) from exc


def validate_derived_tag(base_image: str, derived_tag: str) -> None:
    """Require a dedicated, run-owned validation-workspace tag."""
    if not DERIVED_TAG_PATTERN.fullmatch(derived_tag):
        raise OrchestratorError(
            "derived_tag",
            "derived tag is outside the dedicated D455 validation-workspace "
            "namespace",
        )
    if derived_tag == base_image:
        raise OrchestratorError(
            "derived_tag", "derived tag must not alias the base image"
        )


def base_alias_for_derived(derived_tag: str) -> str:
    prefix = "pharmarobot:d455-validation-workspace-"
    if not DERIVED_TAG_PATTERN.fullmatch(derived_tag):
        raise OrchestratorError(
            "base_alias", "cannot derive a base alias from an invalid tag"
        )
    alias = "pharmarobot:d455-validation-base-" + derived_tag[len(prefix):]
    if not BASE_ALIAS_TAG_PATTERN.fullmatch(alias):
        raise OrchestratorError(
            "base_alias",
            "derived base alias is outside the approved namespace",
        )
    return alias


class DerivedImageBuilder:
    """Build and verify one immutable validation-workspace image."""

    def __init__(self, runner: Any, evidence: Evidence):
        self.runner = runner
        self.evidence = evidence
        self.verifier_name: Optional[str] = None
        self.built_tag: Optional[str] = None
        self.built_base_digest: Optional[str] = None
        self.built_manifest_sha256: Optional[str] = None
        self.base_alias_tag: Optional[str] = None
        self.base_alias_digest: Optional[str] = None

    def _image_digest(self, reference: str, phase: str) -> str:
        result = run_checked(
            self.runner,
            self.evidence,
            phase,
            ["docker", "image", "inspect", reference],
        )
        record = parse_single_inspect(result.stdout, phase)
        digest = record.get("Id")
        if not isinstance(digest, str) or not IMAGE_DIGEST.fullmatch(digest):
            raise OrchestratorError(phase, "image digest is invalid")
        return digest

    def build(
        self,
        *,
        base_image: str,
        derived_tag: str,
        snapshot: SnapshotResult,
    ) -> DerivedImage:
        validate_derived_tag(base_image, derived_tag)
        base_digest = self._image_digest(base_image, "base_image")
        base_alias = base_alias_for_derived(derived_tag)
        alias_absent = self.runner.run(
            ["docker", "image", "inspect", base_alias],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        self.evidence.command(
            "base_alias_absent", alias_absent, COMMAND_TIMEOUT_SECONDS
        )
        if alias_absent.timed_out or alias_absent.returncode != 1:
            raise OrchestratorError(
                "base_alias", "base alias is not proven fresh and run-owned"
            )
        self.base_alias_tag = base_alias
        self.base_alias_digest = base_digest
        tag_result = run_checked(
            self.runner,
            self.evidence,
            "base_alias_tag",
            ["docker", "tag", base_digest, base_alias],
        )
        del tag_result
        alias_digest = self._image_digest(base_alias, "base_alias_verify")
        if alias_digest != base_digest:
            atomic_json(
                self.evidence.root / "base-alias.json",
                {
                    "tag": base_alias,
                    "expected_digest": base_digest,
                    "observed_digest": alias_digest,
                    "tag_was_absent_before_creation": True,
                    "status": "digest_mismatch",
                },
            )
            raise OrchestratorError(
                "base_alias", "base alias digest does not match pinned base"
            )
        atomic_json(
            self.evidence.root / "base-alias.json",
            {
                "tag": base_alias,
                "expected_digest": base_digest,
                "observed_digest": alias_digest,
                "tag_was_absent_before_creation": True,
                "status": "verified",
            },
        )
        preexisting = self.runner.run(
            ["docker", "image", "inspect", derived_tag],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        self.evidence.command(
            "derived_tag_absent", preexisting, COMMAND_TIMEOUT_SECONDS
        )
        if preexisting.timed_out or preexisting.returncode != 1:
            raise OrchestratorError(
                "derived_tag",
                "derived tag is not proven fresh and run-owned",
            )
        dockerfile = snapshot.root / DOCKERFILE_PATH.relative_to(
            REPOSITORY_ROOT
        )
        if not dockerfile.is_file():
            raise OrchestratorError(
                "derived_build", "reviewed Dockerfile is absent from snapshot"
            )
        self.built_tag = derived_tag
        self.built_base_digest = base_digest
        self.built_manifest_sha256 = snapshot.manifest_sha256
        run_checked(
            self.runner,
            self.evidence,
            "derived_build",
            [
                "docker",
                "build",
                "--pull=false",
                "--network=none",
                "--build-arg",
                f"BASE_IMAGE={base_alias}",
                "--build-arg",
                f"BASE_IMAGE_DIGEST={base_digest}",
                "--build-arg",
                f"SOURCE_MANIFEST_SHA256={snapshot.manifest_sha256}",
                "--file",
                str(dockerfile),
                "--tag",
                derived_tag,
                str(snapshot.root),
            ],
            timeout=BUILD_TIMEOUT_SECONDS,
        )
        derived_digest = self._image_digest(
            derived_tag, "derived_image"
        )
        inspect = run_checked(
            self.runner,
            self.evidence,
            "derived_labels",
            ["docker", "image", "inspect", derived_digest],
        )
        record = parse_single_inspect(inspect.stdout, "derived_labels")
        labels = record.get("Config", {}).get("Labels", {})
        expected = {
            DERIVED_LABEL: "true",
            BASE_LABEL: base_digest,
            MANIFEST_LABEL: snapshot.manifest_sha256,
        }
        if any(labels.get(key) != value for key, value in expected.items()):
            raise OrchestratorError(
                "derived_labels", "derived image labels do not match inputs"
            )
        return DerivedImage(
            digest=derived_digest,
            tag=derived_tag,
            base_digest=base_digest,
            manifest_sha256=snapshot.manifest_sha256,
        )

    def cleanup_base_alias(self) -> None:
        """Remove only a freshly-created alias whose digest remains exact."""
        if self.base_alias_tag is None:
            return
        if self.base_alias_digest is None:
            raise OrchestratorError(
                "base_alias_cleanup", "base alias has no ownership digest"
            )
        inspect = self.runner.run(
            ["docker", "image", "inspect", self.base_alias_tag],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        self.evidence.command(
            "base_alias_cleanup_inspect",
            inspect,
            COMMAND_TIMEOUT_SECONDS,
        )
        if inspect.timed_out:
            raise OrchestratorError(
                "base_alias_cleanup", "base alias ownership inspect timed out"
            )
        if inspect.returncode == 1:
            self.base_alias_tag = None
            self.base_alias_digest = None
            return
        if inspect.returncode != 0:
            raise OrchestratorError(
                "base_alias_cleanup", "base alias ownership inspect failed"
            )
        observed = self._image_digest(
            self.base_alias_tag, "base_alias_cleanup_verify"
        )
        if observed != self.base_alias_digest:
            raise OrchestratorError(
                "base_alias_cleanup",
                "base alias digest changed; refusing to remove it",
            )
        removed = self.runner.run(
            ["docker", "image", "rm", self.base_alias_tag],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        self.evidence.command(
            "base_alias_cleanup", removed, COMMAND_TIMEOUT_SECONDS
        )
        if removed.timed_out or removed.returncode != 0:
            raise OrchestratorError(
                "base_alias_cleanup", "base alias cleanup failed"
            )
        self.base_alias_tag = None
        self.base_alias_digest = None

    def verify(
        self,
        image: DerivedImage,
        snapshot: SnapshotResult,
    ) -> None:
        suffix = image.digest.split(":", 1)[1][:12]
        name = f"pharma_realsense_imu_workspace_verify_{suffix}"
        self.verifier_name = name
        run_checked(
            self.runner,
            self.evidence,
            "derived_verify_create",
            [
                "docker",
                "create",
                "--name",
                name,
                "--network=none",
                "--cap-drop=ALL",
                "--security-opt=no-new-privileges:true",
                "--entrypoint",
                "/bin/bash",
                image.digest,
                "-lc",
                "set -eu; "
                "test -f /validation_ws/install/setup.bash; "
                "test -s /validation_ws/build-evidence/test-result.txt; "
                "test -f /validation_ws/build-evidence/result.txt; "
                "grep -qx D455_VALIDATION_WORKSPACE_BUILD=passed "
                "/validation_ws/build-evidence/result.txt; "
                "cmp /validation_ws/snapshot-manifest.json "
                "/validation_ws/build-evidence/snapshot-manifest.json",
            ],
        )
        run_checked(
            self.runner,
            self.evidence,
            "derived_verify_start",
            ["docker", "start", "--attach", name],
        )
        copied_manifest = (
            self.evidence.root / "derived-snapshot-manifest.json"
        )
        copied_results = self.evidence.root / "derived-test-result.txt"
        run_checked(
            self.runner,
            self.evidence,
            "derived_manifest_copy",
            [
                "docker",
                "cp",
                f"{name}:/validation_ws/snapshot-manifest.json",
                str(copied_manifest),
            ],
        )
        run_checked(
            self.runner,
            self.evidence,
            "derived_test_result_copy",
            [
                "docker",
                "cp",
                f"{name}:/validation_ws/build-evidence/test-result.txt",
                str(copied_results),
            ],
        )
        if not copied_manifest.is_file() or not copied_results.is_file():
            raise OrchestratorError(
                "derived_artifacts", "required image evidence was not copied"
            )
        if copied_manifest.read_bytes() != snapshot.manifest_path.read_bytes():
            raise OrchestratorError(
                "derived_artifacts", "image source manifest does not match"
            )
        if not copied_results.read_text(encoding="utf-8").strip():
            raise OrchestratorError(
                "derived_artifacts", "image test result is empty"
            )

    def cleanup_verifier(self) -> None:
        if self.verifier_name is None:
            return
        result = self.runner.run(
            ["docker", "rm", "-f", self.verifier_name],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        self.evidence.command(
            "derived_verify_cleanup", result, COMMAND_TIMEOUT_SECONDS
        )
        if result.timed_out or result.returncode not in (0, 1):
            raise OrchestratorError(
                "derived_verify_cleanup",
                "verifier container cleanup failed",
            )
        self.verifier_name = None

    def rollback_failed_build(self) -> None:
        if self.built_tag is None:
            return
        validate_derived_tag("", self.built_tag)
        if (
            self.built_base_digest is None
            or self.built_manifest_sha256 is None
        ):
            raise OrchestratorError(
                "derived_image_rollback",
                "derived tag has no pinned run ownership evidence",
            )
        inspect = self.runner.run(
            ["docker", "image", "inspect", self.built_tag],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        self.evidence.command(
            "derived_image_rollback_inspect",
            inspect,
            COMMAND_TIMEOUT_SECONDS,
        )
        if inspect.timed_out:
            raise OrchestratorError(
                "derived_image_rollback",
                "derived tag ownership inspection timed out",
            )
        if inspect.returncode == 1:
            self.built_tag = None
            self.built_base_digest = None
            self.built_manifest_sha256 = None
            return
        if inspect.returncode != 0:
            raise OrchestratorError(
                "derived_image_rollback",
                "derived tag ownership could not be inspected",
            )
        record = parse_single_inspect(
            inspect.stdout, "derived_image_rollback"
        )
        image_config = record.get("Config")
        labels = (
            image_config.get("Labels", {})
            if isinstance(image_config, Mapping)
            else {}
        )
        expected = {
            DERIVED_LABEL: "true",
            BASE_LABEL: self.built_base_digest,
            MANIFEST_LABEL: self.built_manifest_sha256,
        }
        if (
            not isinstance(labels, Mapping)
            or any(
                labels.get(key) != value
                for key, value in expected.items()
            )
        ):
            raise OrchestratorError(
                "derived_image_rollback",
                "derived tag is not proven to be owned by this run",
            )
        result = self.runner.run(
            ["docker", "image", "rm", self.built_tag],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        self.evidence.command(
            "derived_image_rollback", result, COMMAND_TIMEOUT_SECONDS
        )
        if result.timed_out or result.returncode not in (0, 1):
            raise OrchestratorError(
                "derived_image_rollback",
                "failed derived-image tag could not be removed",
            )
        self.built_tag = None
        self.built_base_digest = None
        self.built_manifest_sha256 = None


class LegacyQuarantine:
    """Move one strictly proven idle legacy container out of the namespace."""

    def __init__(self, runner: Any, evidence: Evidence):
        self.runner = runner
        self.evidence = evidence
        self.migration: Optional[LegacyMigration] = None

    def _record_state(self, status: str, **values: Any) -> None:
        atomic_json(
            self.evidence.root / "legacy-migration-state.json",
            {
                "status": status,
                "time_utc": utc_now(),
                **values,
            },
        )
        self.evidence.event(
            "legacy_migration_state", status=status, **values
        )

    def _reconcile_inspect(
        self,
        *,
        phase: str,
        full_id: str,
    ) -> Mapping[str, Any]:
        result = self.runner.run(
            ["docker", "inspect", full_id],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        self.evidence.command(phase, result, COMMAND_TIMEOUT_SECONDS)
        if result.timed_out or result.returncode != 0:
            self._record_state(
                "unresolved",
                phase=phase,
                full_id=full_id,
                reason="exact container state could not be inspected",
                inspect_returncode=result.returncode,
                inspect_timed_out=result.timed_out,
            )
            raise OrchestratorError(
                phase, "exact container state could not be reconciled"
            )
        try:
            return parse_single_inspect(result.stdout, phase)
        except OrchestratorError:
            self._record_state(
                "unresolved",
                phase=phase,
                full_id=full_id,
                reason="container inspect evidence was ambiguous",
            )
            raise

    @staticmethod
    def _identity_and_state(
        record: Mapping[str, Any],
    ) -> tuple[str, str, Optional[bool]]:
        return (
            str(record.get("Name", "")).lstrip("/"),
            str(record.get("Id", "")),
            record.get("State", {}).get("Running"),
        )

    @staticmethod
    def _validate_inspect(
        record: Mapping[str, Any],
        *,
        expected_name: str,
        expected_id: str,
        expected_image_digest: str,
    ) -> bool:
        actual_name = str(record.get("Name", "")).lstrip("/")
        host = record.get("HostConfig", {})
        state = record.get("State", {})
        config = record.get("Config", {})
        if not all(
            isinstance(value, Mapping)
            for value in (host, state, config)
        ):
            raise OrchestratorError(
                "legacy_proof", "legacy inspect structure is invalid"
            )
        labels = config.get("Labels") or {}
        if not isinstance(labels, Mapping):
            raise OrchestratorError(
                "legacy_proof", "legacy label structure is invalid"
            )
        if (
            actual_name != expected_name
            or record.get("Id") != expected_id
            or record.get("Image") != expected_image_digest
            or host.get("NetworkMode") != "none"
            or host.get("Privileged") is not False
            or set(host.get("CapDrop") or ()) != {"ALL"}
            or set(host.get("SecurityOpt") or ())
            != {
                "no-new-privileges:true",
                "apparmor=pharmarobot-d455-imu",
            }
            or record.get("AppArmorProfile") != "pharmarobot-d455-imu"
            or labels.get("pharmarobot.d455.validation") == "true"
        ):
            raise OrchestratorError(
                "legacy_proof", "legacy container identity/isolation mismatch"
            )
        devices = json.dumps(
            {
                "devices": host.get("Devices"),
                "mounts": record.get("Mounts"),
            },
            sort_keys=True,
        )
        for forbidden in (
            "/dev/roboteq",
            "/dev/ttyUSB",
            "/dev/ttyACM",
            "/dev/input",
        ):
            if forbidden in devices:
                raise OrchestratorError(
                    "legacy_proof",
                    f"legacy container exposes forbidden device: {forbidden}",
                )
        for device in host.get("Devices") or ():
            path = str(device.get("PathOnHost", ""))
            if not re.fullmatch(
                r"/dev/(?:bus/usb/\d{3}/\d{3}|video\d+|media\d+)",
                path,
            ):
                raise OrchestratorError(
                    "legacy_proof",
                    f"unexpected legacy device mapping: {path}",
                )
        for rule in host.get("DeviceCgroupRules") or ():
            if not re.fullmatch(r"c \d+:\d+ rwm", str(rule)):
                raise OrchestratorError(
                    "legacy_proof",
                    f"broad legacy device-cgroup rule: {rule}",
                )
        for mount in record.get("Mounts") or ():
            source = str(mount.get("Source", ""))
            if not (
                re.fullmatch(r"/dev/iio:device\d+", source)
                or (
                    source.startswith("/sys/devices/")
                    and "0003:8086:0B5C." in source
                    and re.search(r"/iio:device\d+$", source)
                )
            ):
                raise OrchestratorError(
                    "legacy_proof",
                    f"unexpected legacy bind mount: {source}",
                )
        return bool(state.get("Running"))

    def quarantine(
        self,
        *,
        name: str,
        full_id: str,
        expected_image_digest: str,
        authorize: bool,
    ) -> LegacyMigration:
        if not authorize:
            raise AuthorizationError(
                "legacy_quarantine",
                "legacy quarantine requires explicit authorization",
            )
        if not LEGACY_NAME_PATTERN.fullmatch(name):
            raise OrchestratorError(
                "legacy_proof", "legacy name is outside the approved namespace"
            )
        if not FULL_CONTAINER_ID.fullmatch(full_id):
            raise OrchestratorError(
                "legacy_proof", "full 64-character container ID is required"
            )
        result = run_checked(
            self.runner,
            self.evidence,
            "legacy_inspect",
            ["docker", "inspect", full_id],
        )
        record = parse_single_inspect(result.stdout, "legacy_inspect")
        was_running = self._validate_inspect(
            record,
            expected_name=name,
            expected_id=full_id,
            expected_image_digest=expected_image_digest,
        )
        if not was_running and (
            record.get("HostConfig", {}).get("Init") is not True
        ):
            raise OrchestratorError(
                "legacy_proof",
                "stopped legacy container has no provable real PID 1 reaper",
            )
        quarantine_name = (
            "pharma_realsense_imu_quarantine_" + full_id[:12]
        )
        absent = self.runner.run(
            ["docker", "inspect", quarantine_name],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        self.evidence.command(
            "legacy_quarantine_name", absent, COMMAND_TIMEOUT_SECONDS
        )
        if absent.timed_out or absent.returncode != 1:
            raise OrchestratorError(
                "legacy_quarantine",
                "quarantine target name is not proven absent",
            )
        if was_running:
            run_checked(
                self.runner,
                self.evidence,
                "legacy_process_proof",
                [
                    "docker",
                    "exec",
                    full_id,
                    "sh",
                    "-c",
                    "set -eu; "
                    "test \"$(ps -o comm= -p 1)\" = docker-init; "
                    "! ps -eo stat= | grep -q Z; "
                    "! pgrep -af '[r]os2|[r]oboteq|[c]ommand_arbiter|[j]oy'; "
                    "ps -eo pid=,comm= | "
                    "awk '$2 !~ /^(docker-init|tail|sleep|sh|ps)$/ "
                    "{bad=1} END {exit bad}'; "
                    "test ! -e /dev/roboteq; "
                    "test ! -e /dev/ttyUSB0; "
                    "test ! -e /dev/ttyACM0; "
                    "test ! -e /dev/input",
                ],
            )
            stop = self.runner.run(
                ["docker", "stop", "--time", "10", full_id],
                timeout=COMMAND_TIMEOUT_SECONDS,
            )
            self.evidence.command(
                "legacy_stop", stop, COMMAND_TIMEOUT_SECONDS
            )
            stopped_record = self._reconcile_inspect(
                phase="legacy_stop_reconcile", full_id=full_id
            )
            stopped_identity = self._identity_and_state(stopped_record)
            if stopped_identity != (name, full_id, False):
                self._record_state(
                    "unresolved",
                    phase="legacy_stop",
                    full_id=full_id,
                    expected_name=name,
                    observed_name=stopped_identity[0],
                    observed_id=stopped_identity[1],
                    observed_running=stopped_identity[2],
                    stop_returncode=stop.returncode,
                    stop_timed_out=stop.timed_out,
                )
                raise OrchestratorError(
                    "legacy_stop",
                    "stopped legacy identity/state could not be proven",
                )
            self._record_state(
                (
                    "stopped"
                    if stop.returncode == 0 and not stop.timed_out
                    else "stopped_after_ambiguous_stop"
                ),
                phase="legacy_stop",
                full_id=full_id,
                original_name=name,
                stop_returncode=stop.returncode,
                stop_timed_out=stop.timed_out,
            )
            if stop.timed_out or stop.returncode != 0:
                raise OrchestratorError(
                    "legacy_stop",
                    "stop command failed even though its side effect was "
                    "reconciled",
                )
        rename = self.runner.run(
            ["docker", "rename", full_id, quarantine_name],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        self.evidence.command(
            "legacy_rename", rename, COMMAND_TIMEOUT_SECONDS
        )
        verified_record = self._reconcile_inspect(
            phase="legacy_rename_reconcile", full_id=full_id
        )
        identity = self._identity_and_state(verified_record)
        if identity == (quarantine_name, full_id, False):
            self.migration = LegacyMigration(
                original_name=name,
                full_id=full_id,
                quarantine_name=quarantine_name,
                was_running=was_running,
            )
            self._record_state(
                "quarantined",
                **asdict(self.migration),
                rename_returncode=rename.returncode,
                rename_timed_out=rename.timed_out,
            )
            atomic_json(
                self.evidence.root / "legacy-quarantine.json",
                asdict(self.migration),
            )
            if rename.timed_out or rename.returncode != 0:
                self.rollback_identity()
                raise OrchestratorError(
                    "legacy_rename",
                    "rename command failed after a reconciled side effect; "
                    "the original name was restored",
                )
            return self.migration
        status = (
            "rename_not_applied"
            if identity == (name, full_id, False)
            else "unresolved"
        )
        self._record_state(
            status,
            phase="legacy_rename",
            full_id=full_id,
            expected_original_name=name,
            expected_quarantine_name=quarantine_name,
            observed_name=identity[0],
            observed_id=identity[1],
            observed_running=identity[2],
            rename_returncode=rename.returncode,
            rename_timed_out=rename.timed_out,
        )
        raise OrchestratorError(
            "legacy_rename",
            "quarantine rename outcome was not the required stopped state",
        )

    def rollback_identity(self) -> None:
        """Restore the original name without restarting stale hardware."""
        if self.migration is None:
            return
        current = self._reconcile_inspect(
            phase="legacy_rollback_current",
            full_id=self.migration.full_id,
        )
        identity = self._identity_and_state(current)
        expected = (
            self.migration.quarantine_name,
            self.migration.full_id,
            False,
        )
        if identity != expected:
            self._record_state(
                "unresolved",
                phase="legacy_quarantine_rollback",
                full_id=self.migration.full_id,
                observed_name=identity[0],
                observed_id=identity[1],
                observed_running=identity[2],
            )
            raise OrchestratorError(
                "legacy_quarantine_rollback",
                "quarantined container identity/state drifted",
            )
        original = self.runner.run(
            ["docker", "inspect", self.migration.original_name],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        self.evidence.command(
            "legacy_rollback_original_name",
            original,
            COMMAND_TIMEOUT_SECONDS,
        )
        if original.timed_out or original.returncode != 1:
            self._record_state(
                "rollback_blocked",
                phase="legacy_quarantine_rollback",
                full_id=self.migration.full_id,
                reason="original name is not proven available",
            )
            raise OrchestratorError(
                "legacy_quarantine_rollback",
                "original legacy name is not proven available",
            )
        result = self.runner.run(
            [
                "docker",
                "rename",
                self.migration.full_id,
                self.migration.original_name,
            ],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        self.evidence.command(
            "legacy_quarantine_rollback", result, COMMAND_TIMEOUT_SECONDS
        )
        reconciled = self._reconcile_inspect(
            phase="legacy_rollback_reconcile",
            full_id=self.migration.full_id,
        )
        reconciled_identity = self._identity_and_state(reconciled)
        restored = (
            self.migration.original_name,
            self.migration.full_id,
            False,
        )
        if reconciled_identity != restored:
            self._record_state(
                "unresolved",
                phase="legacy_quarantine_rollback",
                full_id=self.migration.full_id,
                observed_name=reconciled_identity[0],
                observed_id=reconciled_identity[1],
                observed_running=reconciled_identity[2],
                rollback_returncode=result.returncode,
                rollback_timed_out=result.timed_out,
            )
            raise OrchestratorError(
                "legacy_quarantine_rollback",
                "legacy quarantine name rollback could not be proven",
            )
        self._record_state(
            "rolled_back",
            phase="legacy_quarantine_rollback",
            full_id=self.migration.full_id,
            restored_name=self.migration.original_name,
            rollback_returncode=result.returncode,
            rollback_timed_out=result.timed_out,
        )
        self.migration = None


def host_preflight_command(
    config: OrchestratorConfig,
    image: DerivedImage,
    *,
    authorize_profile_reload: bool,
) -> list[str]:
    if not TARGET_NAME_PATTERN.fullmatch(config.target_container):
        raise OrchestratorError(
            "host_preflight",
            "target container is outside the dedicated validation namespace",
        )
    command = [
        sys.executable,
        str(HOST_PREFLIGHT_PATH),
        "--image",
        image.digest,
        "--container-name",
        config.target_container,
        "--workspace",
        "/validation_ws",
        "--evidence-dir",
        str(config.evidence_dir / "host-preflight"),
        "--execute",
        "--authorize-container-recreate",
        "--authorize-stationary-d455",
        "--authorize-ros-no-motion",
    ]
    if authorize_profile_reload:
        command.append("--authorize-profile-reload")
    return command


def execute(
    config: OrchestratorConfig,
    *,
    authorize_build: bool,
    authorize_profile_reload: bool,
    authorize_container_recreate: bool,
    authorize_stationary_d455: bool,
    authorize_ros_no_motion: bool,
    authorize_legacy_quarantine: bool,
    runner: Optional[Any] = None,
    repository_root: Path = REPOSITORY_ROOT,
) -> DerivedImage:
    required = (
        authorize_build,
        authorize_container_recreate,
        authorize_stationary_d455,
        authorize_ros_no_motion,
    )
    if not all(required):
        raise AuthorizationError(
            "authorization",
            "build, container, stationary D455, and ROS no-motion "
            "authorizations are required",
        )
    if bool(config.legacy_name) != bool(config.legacy_id):
        raise OrchestratorError(
            "legacy_proof", "legacy name and full ID must be supplied together"
        )
    evidence = Evidence(config.evidence_dir)
    runner = runner or Runner()
    builder = DerivedImageBuilder(runner, evidence)
    quarantine = LegacyQuarantine(runner, evidence)
    image: Optional[DerivedImage] = None
    terminal_error: Optional[BaseException] = None
    evidence.event("orchestration_started")
    try:
        snapshot = create_snapshot(
            repository_root,
            evidence.root / "snapshot",
        )
        atomic_json(
            evidence.root / "snapshot-summary.json",
            {
                "manifest_sha256": snapshot.manifest_sha256,
                "file_count": snapshot.file_count,
                "total_bytes": snapshot.total_bytes,
            },
        )
        image = builder.build(
            base_image=config.base_image,
            derived_tag=config.derived_tag,
            snapshot=snapshot,
        )
        builder.verify(image, snapshot)
        builder.cleanup_verifier()
        if config.legacy_name and config.legacy_id:
            quarantine.quarantine(
                name=config.legacy_name,
                full_id=config.legacy_id,
                expected_image_digest=image.base_digest,
                authorize=authorize_legacy_quarantine,
            )
        run_checked(
            runner,
            evidence,
            "host_preflight",
            host_preflight_command(
                config,
                image,
                authorize_profile_reload=authorize_profile_reload,
            ),
            timeout=HOST_PREFLIGHT_TIMEOUT_SECONDS,
        )
        evidence.event(
            "orchestration_completed",
            derived_image_digest=image.digest,
        )
    except BaseException as exc:
        terminal_error = exc
        evidence.event(
            "orchestration_failed",
            phase=getattr(exc, "phase", "unexpected"),
            error=str(exc),
        )
    finally:
        try:
            builder.cleanup_verifier()
        except BaseException as cleanup_error:
            evidence.event("verifier_cleanup_failed", error=str(cleanup_error))
            if terminal_error is None:
                terminal_error = cleanup_error
        try:
            builder.cleanup_base_alias()
        except BaseException as cleanup_error:
            evidence.event(
                "base_alias_cleanup_failed", error=str(cleanup_error)
            )
            if terminal_error is None:
                terminal_error = cleanup_error
        if terminal_error is not None:
            try:
                quarantine.rollback_identity()
            except BaseException as rollback_error:
                evidence.event(
                    "legacy_rollback_failed", error=str(rollback_error)
                )
                terminal_error = rollback_error
            try:
                builder.rollback_failed_build()
            except BaseException as rollback_error:
                evidence.event(
                    "image_rollback_failed", error=str(rollback_error)
                )
                terminal_error = rollback_error
        atomic_json(
            evidence.root / "result.json",
            {
                "result": "passed" if terminal_error is None else "failed",
                "error": (
                    None if terminal_error is None else str(terminal_error)
                ),
                "derived_image_digest": (
                    None if image is None else image.digest
                ),
                "no_nonzero_twist": True,
            },
        )
    if terminal_error is not None:
        raise terminal_error
    assert image is not None
    return image


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-image", required=True)
    parser.add_argument("--derived-tag", required=True)
    parser.add_argument("--target-container", required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--legacy-name")
    parser.add_argument("--legacy-id")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorize-derived-image-build", action="store_true")
    parser.add_argument("--authorize-profile-reload", action="store_true")
    parser.add_argument("--authorize-container-recreate", action="store_true")
    parser.add_argument("--authorize-stationary-d455", action="store_true")
    parser.add_argument("--authorize-ros-no-motion", action="store_true")
    parser.add_argument("--authorize-legacy-quarantine", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    if not args.execute:
        raise SystemExit("refusing to execute without --execute")
    config = OrchestratorConfig(
        base_image=args.base_image,
        derived_tag=args.derived_tag,
        target_container=args.target_container,
        evidence_dir=args.evidence_dir,
        legacy_name=args.legacy_name,
        legacy_id=args.legacy_id,
    )
    try:
        execute(
            config,
            authorize_build=args.authorize_derived_image_build,
            authorize_profile_reload=args.authorize_profile_reload,
            authorize_container_recreate=args.authorize_container_recreate,
            authorize_stationary_d455=args.authorize_stationary_d455,
            authorize_ros_no_motion=args.authorize_ros_no_motion,
            authorize_legacy_quarantine=args.authorize_legacy_quarantine,
        )
    except (OrchestratorError, OSError, ValueError) as exc:
        print(f"D455 container orchestration failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
