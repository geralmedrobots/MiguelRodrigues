#!/usr/bin/env python3
"""Provision and validate one isolated D455 no-motion validation run.

This is a host-side deployment/validation tool.  It is deliberately outside
the ROS runtime package: no ROS callback, relay, processor, or control package
can discover sysfs resources, modify AppArmor, or invoke Docker through it.

Execution requires four explicit acknowledgements.  AppArmor reload also
requires a fifth acknowledgement and root execution, and occurs only when the
serial-derived candidate differs from the installed/loaded enforcing profile.
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
import shlex
import subprocess
import sys
import tempfile
import traceback
from typing import Any, Callable, Iterable, Mapping, Optional, Sequence

from realsense_imu.apparmor_profile import generate_apparmor_profile
from realsense_imu.apparmor_profile import PROFILE_NAME
from realsense_imu.apparmor_profile import PROFILE_TEMPLATE_VERSION
from realsense_imu.apparmor_profile import profile_template_sha256
from realsense_imu.usb_device import DEFAULT_PRODUCT_ID
from realsense_imu.usb_device import DEFAULT_SERIAL_NUMBER
from realsense_imu.usb_device import DEFAULT_USB_SERIAL_NUMBER
from realsense_imu.usb_device import DEFAULT_VENDOR_ID
from realsense_imu.usb_device import docker_arguments
from realsense_imu.usb_device import HostResources
from realsense_imu.usb_device import required_iio_control_paths
from realsense_imu.usb_device import select_host_resources
from realsense_imu.usb_device import verify_librealsense_serial

try:
    from d455_host_lock import d455_host_lock
    from d455_host_lock import D455HostLockError
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from d455_host_lock import d455_host_lock
    from d455_host_lock import D455HostLockError


TOOL_VERSION = "1"
DEFAULT_CONTAINER_NAME = "pharma_realsense_imu_validation"
VALIDATION_LABEL = "pharmarobot.d455.validation=true"
PRODUCTION_LABEL = "pharmarobot.d455.production=true"
INSTALLED_PROFILE_PATH = Path("/etc/apparmor.d") / PROFILE_NAME
INSTALLED_MANIFEST_PATH = (
    Path("/etc/apparmor.d") / f"{PROFILE_NAME}.manifest.json"
)
KERNEL_PROFILES_PATH = Path("/sys/kernel/security/apparmor/profiles")
VALIDATION_WRAPPER_PATH = (
    Path(__file__).resolve().with_name("d455_no_motion_validation.py")
)
CONTAINER_NAME_PATTERN = re.compile(
    r"pharma_realsense_imu_(?:validation|runtime)"
    r"(?:_[A-Za-z0-9_.-]+)?"
)
COMMAND_TIMEOUT_SECONDS = 15.0
DOCKER_STOP_TIMEOUT_SECONDS = 10
MAX_RESOLVED_DEVICE_NODES = 32
MAX_RESOLVED_CONTROL_PATHS = 32
FORBIDDEN_RESOURCE_FRAGMENTS = (
    "/dev/roboteq",
    "/dev/ttyUSB",
    "/dev/ttyACM",
    "/dev/input",
    "/dev/joystick",
)


class PreflightError(RuntimeError):
    """A deterministic fail-closed preflight failure."""

    def __init__(self, phase: str, message: str):
        super().__init__(f"{phase}: {message}")
        self.phase = phase
        self.message = message


class ApprovalRequired(PreflightError):
    """A required host-side authorization was not supplied."""


@dataclass(frozen=True)
class CommandResult:
    """Bounded command outcome persisted in evidence."""

    args: tuple[str, ...]
    returncode: int
    stdout: str = ""
    stderr: str = ""
    timed_out: bool = False


@dataclass(frozen=True)
class ProfileState:
    """Installed and kernel-loaded state for the dedicated profile."""

    loaded: bool
    enforcing: bool
    installed_sha256: Optional[str]
    installed_manifest_fingerprint: Optional[str]
    conflicting_profiles: tuple[str, ...] = ()


@dataclass(frozen=True)
class ProfileDecision:
    """Result of candidate profile reconciliation."""

    changed: bool
    candidate_sha256: str
    resource_fingerprint: str
    previous_sha256: Optional[str]


@dataclass(frozen=True)
class RuntimeConfig:
    """Explicit identifiers and paths for one stationary validation run."""

    serial_number: str
    usb_serial_number: str
    image: str
    container_name: str
    workspace: str
    evidence_dir: Path
    installed_profile_path: Path
    installed_manifest_path: Path
    kernel_profiles_path: Path
    validation_wrapper: Path


def utc_now() -> str:
    """Return a stable UTC evidence timestamp."""
    return datetime.now(timezone.utc).isoformat()


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def canonical_json(value: Mapping[str, Any]) -> str:
    """Serialize deterministic manifest content."""
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    )


def atomic_write(path: Path, value: bytes, *, mode: int = 0o600) -> None:
    """Atomically replace one regular file in its existing parent."""
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


def atomic_write_json(path: Path, value: Mapping[str, Any]) -> None:
    atomic_write(
        path,
        (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8"),
    )


class SubprocessRunner:
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
                timed_out=True,
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
    """Append command/phase evidence to one fresh run directory."""

    def __init__(self, root: Path):
        if root.exists():
            raise PreflightError(
                "evidence", f"evidence directory already exists: {root}"
            )
        root.mkdir(parents=True)
        self.root = root
        self.command_index = 0

    def event(self, event: str, **fields: Any) -> None:
        entry = {"event": event, "time_utc": utc_now(), **fields}
        path = self.root / "events.jsonl"
        with path.open("a", encoding="utf-8") as output:
            output.write(canonical_json(entry) + "\n")
            output.flush()
            os.fsync(output.fileno())

    def command(
        self, phase: str, result: CommandResult, timeout: float
    ) -> None:
        self.command_index += 1
        record = {
            "phase": phase,
            "args": list(result.args),
            "timeout_seconds": timeout,
            "exit_status": result.returncode,
            "timed_out": result.timed_out,
            "stdout": result.stdout,
            "stderr": result.stderr,
        }
        atomic_write_json(
            self.root / f"command-{self.command_index:03d}.json",
            record,
        )
        self.event(
            "command_completed",
            phase=phase,
            command_index=self.command_index,
            exit_status=result.returncode,
            timed_out=result.timed_out,
        )

    def write_json(self, name: str, value: Mapping[str, Any]) -> None:
        atomic_write_json(self.root / name, value)

    def write_text(self, name: str, value: str) -> None:
        atomic_write(self.root / name, value.encode("utf-8"))


def failure_record(
    error: BaseException,
    *,
    context: Mapping[str, Any],
    cleanup_result: str,
) -> dict[str, Any]:
    """Build complete failure metadata without changing the exception."""
    return {
        "error_type": type(error).__name__,
        "phase": getattr(error, "phase", "unexpected"),
        "error": str(error),
        "exception_repr": repr(error),
        "traceback": "".join(
            traceback.format_exception(
                type(error), error, error.__traceback__
            )
        ),
        "context": dict(context),
        "cleanup_result": cleanup_result,
    }


def failure_context(
    config: RuntimeConfig,
    *,
    authorize_profile_reload: bool,
    authorize_container_recreate: bool,
    authorize_stationary_d455: bool,
    authorize_ros_no_motion: bool,
) -> dict[str, Any]:
    return {
        "serial_number": config.serial_number,
        "usb_serial_number": config.usb_serial_number,
        "image": config.image,
        "container_name": config.container_name,
        "workspace": config.workspace,
        "evidence_dir": str(config.evidence_dir),
        "authorizations": {
            "profile_reload": authorize_profile_reload,
            "container_recreate": authorize_container_recreate,
            "stationary_d455": authorize_stationary_d455,
            "ros_no_motion": authorize_ros_no_motion,
        },
    }


def preserve_fallback_failure(
    evidence_dir: Path,
    error: BaseException,
    *,
    context: Mapping[str, Any],
    cleanup_result: str,
    filename: str = "failure.json",
) -> None:
    """Atomically retain a failure when the run directory cannot initialize."""
    base_name = evidence_dir.name + "." + filename
    fallback = evidence_dir.with_name(base_name)
    suffix = 1
    while fallback.exists():
        fallback = evidence_dir.with_name(
            f"{base_name}.{suffix}"
        )
        suffix += 1
    record = failure_record(
        error, context=context, cleanup_result=cleanup_result
    )
    atomic_write_json(fallback, record)


def preserve_failure_evidence(
    evidence: Evidence,
    error: BaseException,
    *,
    event: str = "preflight_failed",
    filename: str = "failure.json",
    context: Optional[Mapping[str, Any]] = None,
    cleanup_result: str = "not_started",
) -> None:
    """Best-effort failure recording that never replaces the root error."""
    record = failure_record(
        error,
        context=context or {},
        cleanup_result=cleanup_result,
    )
    try:
        evidence.write_json(filename, record)
    except BaseException:
        try:
            preserve_fallback_failure(
                evidence.root,
                error,
                context=context or {},
                cleanup_result=cleanup_result,
                filename=filename,
            )
        except BaseException:
            pass
    try:
        evidence.event(event, **record)
    except BaseException:
        pass


def run_checked(
    runner: Any,
    evidence: Evidence,
    phase: str,
    args: Sequence[str],
    *,
    timeout: float = COMMAND_TIMEOUT_SECONDS,
    accepted_statuses: Iterable[int] = (0,),
) -> CommandResult:
    """Run, persist, and strictly validate one bounded command."""
    result = runner.run(args, timeout=timeout)
    evidence.command(phase, result, timeout)
    accepted = tuple(accepted_statuses)
    if result.timed_out:
        raise PreflightError(phase, f"command timed out after {timeout}s")
    if result.returncode not in accepted:
        raise PreflightError(
            phase,
            "command failed with exit status "
            f"{result.returncode}: {shlex.join(result.args)}",
        )
    return result


def verify_serial_bounded(
    runner: Any,
    evidence: Evidence,
    serial_number: str,
) -> None:
    """Run librealsense enumeration through the bounded evidence runner."""
    result = run_checked(
        runner,
        evidence,
        "librealsense_serial",
        ["rs-enumerate-devices", "-s"],
    )
    verify_librealsense_serial(
        serial_number,
        enumerate_devices=lambda: result.stdout,
    )


def validate_resource_set(resources: HostResources) -> None:
    """Reject broad, unrelated, duplicate, escaped, or unbounded resources."""
    if len(resources.device_nodes) > MAX_RESOLVED_DEVICE_NODES:
        raise PreflightError("discovery", "too many D455 device nodes")
    paths = [str(node.path) for node in resources.device_nodes]
    if len(paths) != len(set(paths)):
        raise PreflightError("discovery", "duplicate D455 device nodes")
    for value in paths:
        if any(
            fragment in value for fragment in FORBIDDEN_RESOURCE_FRAGMENTS
        ):
            raise PreflightError(
                "discovery", f"motor/control-facing resource selected: {value}"
            )
        if value in ("/dev", "/dev/bus/usb"):
            raise PreflightError(
                "discovery", f"broad device path selected: {value}"
            )

    controls = [
        path
        for device in resources.iio_devices
        for path in required_iio_control_paths(device)
    ]
    if len(controls) > MAX_RESOLVED_CONTROL_PATHS:
        raise PreflightError("discovery", "too many D455 sysfs controls")
    if len(controls) != len(set(controls)):
        raise PreflightError("discovery", "duplicate D455 sysfs controls")
    for device in resources.iio_devices:
        try:
            device.sysfs_path.relative_to(resources.usb_sysfs_path)
        except ValueError as exc:
            raise PreflightError(
                "discovery",
                f"{device.name} escaped selected D455: {device.sysfs_path}",
            ) from exc


def reject_active_production_container(
    runner: Any, evidence: Evidence
) -> None:
    """Prevent validation hardware access beside the production publisher."""
    result = run_checked(
        runner,
        evidence,
        "production_container_exclusion",
        [
            "docker",
            "ps",
            "--filter",
            f"label={PRODUCTION_LABEL}",
            "--format",
            "{{.ID}} {{.Names}}",
        ],
    )
    if result.stdout.strip():
        raise PreflightError(
            "production_container_exclusion",
            "an active production D455 sensor container blocks validation",
        )


_HID_INSTANCE = re.compile(r"(?P<stable>0003:8086:0B5C)\.(?P<instance>\d{4})")


def split_transient_sysfs_path(path: Path) -> Mapping[str, Any]:
    """Record a transient HID suffix separately from its stable identity."""
    value = str(path)
    match = _HID_INSTANCE.search(value)
    if match is None:
        return {
            "resolved_path": value,
            "hid_identity": None,
            "hid_instance": None,
        }
    return {
        "resolved_path": value,
        "hid_identity": match.group("stable"),
        "hid_instance": match.group("instance"),
    }


def resource_description(resources: HostResources) -> Mapping[str, Any]:
    """Return deterministic stable and resolved resource descriptions."""
    validate_resource_set(resources)
    return {
        "usb_sysfs_path": str(resources.usb_sysfs_path),
        "usb_topology_name": resources.usb_sysfs_path.name,
        "device_nodes": [
            {
                "path": str(node.path),
                "major": node.major,
                "minor": node.minor,
                "class": (
                    "usb"
                    if "/bus/usb/" in str(node.path)
                    else "iio"
                    if node.path.name.startswith("iio:device")
                    else "video"
                    if node.path.name.startswith("video")
                    else "media"
                ),
            }
            for node in sorted(
                resources.device_nodes,
                key=lambda item: str(item.path),
            )
        ],
        "iio_devices": [
            {
                "name": device.name,
                "device_node": str(device.device_node),
                "sysfs": split_transient_sysfs_path(device.sysfs_path),
                "controls": [
                    str(path) for path in required_iio_control_paths(device)
                ],
                "has_hysteresis_control": device.has_hysteresis_control,
            }
            for device in sorted(
                resources.iio_devices,
                key=lambda item: item.name,
            )
        ],
    }


def assert_resources_unchanged(
    expected: HostResources,
    observed: HostResources,
    *,
    phase: str,
) -> None:
    """Reject reset, disconnect, re-enumeration, or topology drift in-run."""
    expected_description = canonical_json(resource_description(expected))
    observed_description = canonical_json(resource_description(observed))
    if observed_description != expected_description:
        raise PreflightError(
            phase,
            "serial-selected D455 resources changed during preflight",
        )


def build_manifest(
    resources: HostResources,
    *,
    serial_number: str,
    usb_serial_number: str,
    profile: str,
    generated_at: str,
    tool_sha256: Optional[str] = None,
) -> Mapping[str, Any]:
    """Build a serial/resource/profile manifest with a stable fingerprint."""
    resolved = resource_description(resources)
    fingerprint_input = {
        "stable_identity": {
            "librealsense_serial": serial_number,
            "usb_serial": usb_serial_number,
            "vendor_id": DEFAULT_VENDOR_ID,
            "product_id": DEFAULT_PRODUCT_ID,
            "required_iio_names": ["accel_3d", "gyro_3d"],
        },
        "resolved_resources": resolved,
    }
    resource_fingerprint = sha256_bytes(
        canonical_json(fingerprint_input).encode("utf-8")
    )
    if tool_sha256 is None:
        tool_sha256 = sha256_file(Path(__file__))
    return {
        "schema_version": 1,
        **fingerprint_input,
        "resource_fingerprint": resource_fingerprint,
        "profile_name": PROFILE_NAME,
        "profile_sha256": sha256_bytes(profile.encode("utf-8")),
        "profile_template_version": PROFILE_TEMPLATE_VERSION,
        "profile_template_sha256": profile_template_sha256(),
        "generated_at_utc": generated_at,
        "tool": {
            "name": "d455_host_preflight",
            "version": TOOL_VERSION,
            "sha256": tool_sha256,
        },
    }


def probe_profile_state(
    *,
    profile_name: str,
    installed_profile_path: Path,
    installed_manifest_path: Path,
    kernel_profiles_path: Path,
) -> ProfileState:
    """Prove installed hash plus kernel loaded/enforcing identity."""
    lines: list[str] = []
    if kernel_profiles_path.exists():
        lines = kernel_profiles_path.read_text(encoding="utf-8").splitlines()
    exact_prefix = f"{profile_name} ("
    exact_lines = [line for line in lines if line.startswith(exact_prefix)]
    loaded = len(exact_lines) == 1
    enforcing = loaded and exact_lines[0].endswith("(enforce)")
    conflicts = tuple(
        sorted(
            line
            for line in lines
            if "d455" in line.lower()
            and not line.startswith(exact_prefix)
        )
    )
    installed_sha = (
        sha256_file(installed_profile_path)
        if installed_profile_path.is_file()
        else None
    )
    manifest_fingerprint = None
    if installed_manifest_path.is_file():
        try:
            manifest = json.loads(
                installed_manifest_path.read_text(encoding="utf-8")
            )
            manifest_fingerprint = manifest.get("resource_fingerprint")
        except (OSError, json.JSONDecodeError):
            manifest_fingerprint = None
    return ProfileState(
        loaded=loaded,
        enforcing=enforcing,
        installed_sha256=installed_sha,
        installed_manifest_fingerprint=manifest_fingerprint,
        conflicting_profiles=conflicts,
    )


def profile_state_matches(
    state: ProfileState,
    *,
    profile_sha256: str,
    resource_fingerprint: str,
) -> bool:
    return (
        state.loaded
        and state.enforcing
        and not state.conflicting_profiles
        and state.installed_sha256 == profile_sha256
        and state.installed_manifest_fingerprint == resource_fingerprint
    )


class ProfileManager:
    """Validate, compare, reload, verify, and roll back one exact profile."""

    def __init__(
        self,
        *,
        runner: Any,
        evidence: Evidence,
        installed_profile_path: Path,
        installed_manifest_path: Path,
        kernel_profiles_path: Path,
        state_probe: Callable[..., ProfileState] = probe_profile_state,
        atomic_writer: Callable[..., None] = atomic_write,
        effective_uid: Callable[[], int] = os.geteuid,
    ):
        self.runner = runner
        self.evidence = evidence
        self.installed_profile_path = installed_profile_path
        self.installed_manifest_path = installed_manifest_path
        self.kernel_profiles_path = kernel_profiles_path
        self.state_probe = state_probe
        self.atomic_writer = atomic_writer
        self.effective_uid = effective_uid

    def _state(self) -> ProfileState:
        return self.state_probe(
            profile_name=PROFILE_NAME,
            installed_profile_path=self.installed_profile_path,
            installed_manifest_path=self.installed_manifest_path,
            kernel_profiles_path=self.kernel_profiles_path,
        )

    def _restore(
        self,
        previous_profile: Optional[bytes],
        previous_manifest: Optional[bytes],
        *,
        reload_previous: bool,
        previous_state: ProfileState,
    ) -> None:
        if previous_profile is None:
            unload = self.runner.run(
                [
                    "apparmor_parser",
                    "-R",
                    str(self.installed_profile_path),
                ],
                timeout=COMMAND_TIMEOUT_SECONDS,
            )
            self.evidence.command(
                "apparmor_rollback_unload",
                unload,
                COMMAND_TIMEOUT_SECONDS,
            )
            if unload.returncode not in (0, 1) or unload.timed_out:
                raise PreflightError(
                    "apparmor_rollback",
                    "failed to unload the rejected candidate profile",
                )
            self.installed_profile_path.unlink(missing_ok=True)
        else:
            self.atomic_writer(
                self.installed_profile_path,
                previous_profile,
                mode=0o644,
            )
        if previous_manifest is None:
            self.installed_manifest_path.unlink(missing_ok=True)
        else:
            self.atomic_writer(
                self.installed_manifest_path,
                previous_manifest,
                mode=0o600,
            )
        if previous_profile is None:
            absent = self._state()
            if (
                absent.loaded
                or absent.enforcing
                or absent.conflicting_profiles
                or absent.installed_sha256 is not None
                or absent.installed_manifest_fingerprint is not None
            ):
                raise PreflightError(
                    "apparmor_rollback",
                    "rejected candidate profile absence could not be proven",
                )
        if previous_profile is not None and reload_previous:
            rollback = self.runner.run(
                [
                    "apparmor_parser",
                    "-r",
                    "-W",
                    str(self.installed_profile_path),
                ],
                timeout=COMMAND_TIMEOUT_SECONDS,
            )
            self.evidence.command(
                "apparmor_rollback",
                rollback,
                COMMAND_TIMEOUT_SECONDS,
            )
            if rollback.returncode != 0 or rollback.timed_out:
                raise PreflightError(
                    "apparmor_rollback",
                    "failed to restore the previous valid profile",
                )
            restored = self._state()
            if (
                not restored.loaded
                or not restored.enforcing
                or restored.conflicting_profiles
                or restored.installed_sha256
                != sha256_bytes(previous_profile)
                or restored.installed_manifest_fingerprint
                != previous_state.installed_manifest_fingerprint
            ):
                raise PreflightError(
                    "apparmor_rollback",
                    "previous profile restoration could not be proven",
                )

    def ensure(
        self,
        *,
        candidate_profile_path: Path,
        profile: str,
        manifest: Mapping[str, Any],
        authorize_reload: bool,
    ) -> ProfileDecision:
        run_checked(
            self.runner,
            self.evidence,
            "apparmor_syntax",
            ["apparmor_parser", "-Q", "-T", str(candidate_profile_path)],
        )
        expected_sha = str(manifest["profile_sha256"])
        expected_fingerprint = str(manifest["resource_fingerprint"])
        state = self._state()
        self.evidence.write_json("profile-state-before.json", asdict(state))
        if state.conflicting_profiles:
            raise PreflightError(
                "apparmor_state",
                "conflicting D455 profiles are loaded: "
                f"{state.conflicting_profiles}",
            )
        if profile_state_matches(
            state,
            profile_sha256=expected_sha,
            resource_fingerprint=expected_fingerprint,
        ):
            return ProfileDecision(
                False,
                expected_sha,
                expected_fingerprint,
                state.installed_sha256,
            )
        if not authorize_reload:
            raise ApprovalRequired(
                "apparmor_reload",
                "candidate differs from loaded enforcing state; "
                "--authorize-profile-reload is required",
            )
        if self.effective_uid() != 0:
            raise ApprovalRequired(
                "apparmor_reload",
                "profile reload requires this tool to be run through an "
                "explicitly authorized root command",
            )

        previous_profile = (
            self.installed_profile_path.read_bytes()
            if self.installed_profile_path.is_file()
            else None
        )
        previous_manifest = (
            self.installed_manifest_path.read_bytes()
            if self.installed_manifest_path.is_file()
            else None
        )
        reload_previous = (
            previous_profile is not None
            and state.loaded
            and state.enforcing
            and not state.conflicting_profiles
            and state.installed_sha256 == sha256_bytes(previous_profile)
        )
        if previous_profile is not None and not reload_previous:
            raise PreflightError(
                "apparmor_reload",
                "installed profile is not proven loaded/enforcing and "
                "recoverable; refusing replacement",
            )
        if previous_profile is None and (
            state.loaded or previous_manifest is not None
        ):
            raise PreflightError(
                "apparmor_reload",
                "profile state is incomplete and cannot be rolled back",
            )
        if previous_profile is not None:
            self.evidence.write_text(
                "previous-profile.apparmor",
                previous_profile.decode("utf-8", errors="replace"),
            )
        if previous_manifest is not None:
            atomic_write(
                self.evidence.root / "previous-profile-manifest.json",
                previous_manifest,
            )

        try:
            self.atomic_writer(
                self.installed_profile_path,
                profile.encode("utf-8"),
                mode=0o644,
            )
            self.atomic_writer(
                self.installed_manifest_path,
                (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode(
                    "utf-8"
                ),
                mode=0o600,
            )
            run_checked(
                self.runner,
                self.evidence,
                "apparmor_reload",
                [
                    "apparmor_parser",
                    "-r",
                    "-W",
                    str(self.installed_profile_path),
                ],
            )
            verified = self._state()
            self.evidence.write_json(
                "profile-state-after.json", asdict(verified)
            )
            if not profile_state_matches(
                verified,
                profile_sha256=expected_sha,
                resource_fingerprint=expected_fingerprint,
            ):
                raise PreflightError(
                    "apparmor_verify",
                    "reloaded profile identity/enforcement could not be "
                    "proven",
                )
        except Exception:
            self._restore(
                previous_profile,
                previous_manifest,
                reload_previous=reload_previous,
                previous_state=state,
            )
            raise
        return ProfileDecision(
            True,
            expected_sha,
            expected_fingerprint,
            state.installed_sha256,
        )


def container_run_command(
    *,
    container_name: str,
    image: str,
    resources: HostResources,
) -> list[str]:
    """Build the exact dedicated-container creation command."""
    if not CONTAINER_NAME_PATTERN.fullmatch(container_name):
        raise PreflightError(
            "container_plan",
            "container name is not in the dedicated D455 validation namespace",
        )
    command = [
        "docker",
        "run",
        "-d",
        "--init",
        "--name",
        container_name,
        "--label",
        VALIDATION_LABEL,
        *docker_arguments(resources),
        image,
        "sleep",
        "infinity",
    ]
    joined = "\n".join(command)
    for forbidden in (
        "--privileged",
        "apparmor=unconfined",
        "/dev/roboteq",
        "/dev/ttyUSB",
        "/dev/ttyACM",
        "/dev/input",
        "--network=host",
        "src=/dev,dst=/dev",
        "src=/sys,dst=/sys",
    ):
        if forbidden in joined:
            raise PreflightError(
                "container_plan", f"forbidden container access: {forbidden}"
            )
    required = (
        "--init",
        "--network=none",
        "--cap-drop=ALL",
        "--security-opt=no-new-privileges:true",
        f"--security-opt=apparmor={PROFILE_NAME}",
    )
    for value in required:
        if value not in command:
            raise PreflightError(
                "container_plan", f"missing isolation argument: {value}"
            )
    return command


def _access_probe_script(resources: HostResources) -> str:
    checks = []
    for node in resources.device_nodes:
        checks.append(f"test -r {shlex.quote(str(node.path))}")
        checks.append(f"test -w {shlex.quote(str(node.path))}")
    for device in resources.iio_devices:
        checks.append(f"test -d {shlex.quote(str(device.sysfs_path))}")
        for control in required_iio_control_paths(device):
            checks.append(f"test -r {shlex.quote(str(control))}")
            checks.append(f"test -w {shlex.quote(str(control))}")
    return "\n".join(["set -eu", *checks, "printf 'D455_ACCESS_OK=1\\n'"])


ISOLATION_PROBE = """\
set -eu
test ! -e /dev/roboteq
test ! -e /dev/ttyUSB0
test ! -e /dev/ttyACM0
test ! -e /dev/input
! pgrep -af '[r]os2 daemon|[r]os2 launch|[r]os2 run'
! pgrep -af '[r]oboteq|[c]ommand_arbiter|[j]oy_node'
! ps -eo stat= | grep -q Z
printf 'D455_ISOLATION_OK=1\\n'
"""


class ContainerManager:
    """Own only the dedicated validation container and its cleanup."""

    def __init__(
        self,
        *,
        runner: Any,
        evidence: Evidence,
        container_name: str,
        image: str,
        resources: HostResources,
        workspace: str,
    ):
        self.runner = runner
        self.evidence = evidence
        self.container_name = container_name
        self.image = image
        self.resources = resources
        self.workspace = workspace
        self.created = False

    def _inspect_optional(self) -> Optional[Mapping[str, Any]]:
        result = self.runner.run(
            ["docker", "inspect", self.container_name],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        self.evidence.command(
            "container_inspect_existing", result, COMMAND_TIMEOUT_SECONDS
        )
        if result.returncode == 1:
            return None
        if result.returncode != 0 or result.timed_out:
            raise PreflightError(
                "container_inspect_existing",
                "cannot determine existing container identity",
            )
        try:
            values = json.loads(result.stdout)
            if len(values) != 1:
                raise ValueError
            return values[0]
        except (json.JSONDecodeError, ValueError, TypeError) as exc:
            raise PreflightError(
                "container_inspect_existing", "invalid Docker inspect output"
            ) from exc

    def _remove_existing_dedicated(self) -> None:
        existing = self._inspect_optional()
        if existing is None:
            return
        labels = (
            existing.get("Config", {}).get("Labels", {})
            if isinstance(existing, dict)
            else {}
        )
        if labels.get("pharmarobot.d455.validation") != "true":
            raise PreflightError(
                "container_identity",
                "refusing to remove a container without the dedicated "
                "validation label",
            )
        run_checked(
            self.runner,
            self.evidence,
            "container_stop_existing",
            [
                "docker",
                "stop",
                "--time",
                str(DOCKER_STOP_TIMEOUT_SECONDS),
                self.container_name,
            ],
            accepted_statuses=(0, 1),
        )
        run_checked(
            self.runner,
            self.evidence,
            "container_remove_existing",
            ["docker", "rm", self.container_name],
        )

    def create(self) -> None:
        self._remove_existing_dedicated()
        image_result = run_checked(
            self.runner,
            self.evidence,
            "image_identity",
            ["docker", "image", "inspect", self.image],
        )
        self.evidence.write_text("image-inspect.json", image_result.stdout)
        try:
            image_records = json.loads(image_result.stdout)
            if len(image_records) != 1:
                raise ValueError
            image_id = image_records[0]["Id"]
            if not re.fullmatch(r"sha256:[0-9a-f]{64}", image_id):
                raise ValueError
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise PreflightError(
                "image_identity", "image digest could not be pinned"
            ) from exc
        command = container_run_command(
            container_name=self.container_name,
            image=image_id,
            resources=self.resources,
        )
        # A timeout or lost Docker response can still leave a container.
        # From this point cleanup must always attempt the exact dedicated name.
        self.created = True
        run_checked(
            self.runner,
            self.evidence,
            "container_create",
            command,
            timeout=30.0,
        )

    def verify_and_preflight_access(self) -> None:
        inspect_result = run_checked(
            self.runner,
            self.evidence,
            "container_inspect_created",
            ["docker", "inspect", self.container_name],
        )
        self.evidence.write_text(
            "container-inspect.json", inspect_result.stdout
        )
        try:
            data = json.loads(inspect_result.stdout)[0]
            host = data["HostConfig"]
            state = data["State"]
            config = data["Config"]
            mounts = data["Mounts"]
        except (KeyError, IndexError, TypeError, json.JSONDecodeError) as exc:
            raise PreflightError(
                "container_isolation", "invalid Docker inspection result"
            ) from exc
        expected_security = {
            "no-new-privileges:true",
            f"apparmor={PROFILE_NAME}",
        }
        if (
            host.get("NetworkMode") != "none"
            or host.get("Privileged") is not False
            or set(host.get("CapDrop") or ()) != {"ALL"}
            or set(host.get("SecurityOpt") or ()) != expected_security
            or state.get("Running") is not True
            or config.get("Labels", {}).get("pharmarobot.d455.validation")
            != "true"
        ):
            raise PreflightError(
                "container_isolation", "container isolation drift detected"
            )
        expected_device_paths = {
            str(node.path)
            for node in self.resources.device_nodes
            if not node.path.name.startswith("iio:device")
        }
        actual_device_paths = {
            item.get("PathOnHost")
            for item in host.get("Devices") or ()
        }
        expected_cgroup_rules = {
            f"c {node.major}:{node.minor} rwm"
            for node in self.resources.device_nodes
            if node.path.name.startswith("iio:device")
        }
        actual_cgroup_rules = set(host.get("DeviceCgroupRules") or ())
        expected_mounts = {
            str(node.path)
            for node in self.resources.device_nodes
            if node.path.name.startswith("iio:device")
        } | {
            str(device.sysfs_path)
            for device in self.resources.iio_devices
        }
        actual_mounts = {
            item.get("Destination")
            for item in mounts
            if item.get("Type") == "bind"
        }
        if (
            actual_device_paths != expected_device_paths
            or actual_cgroup_rules != expected_cgroup_rules
            or actual_mounts != expected_mounts
        ):
            raise PreflightError(
                "container_isolation",
                "container device/mount scope differs from selected D455",
            )
        run_checked(
            self.runner,
            self.evidence,
            "container_init_census",
            [
                "docker",
                "exec",
                self.container_name,
                "sh",
                "-c",
                "test \"$(ps -o comm= -p 1)\" = docker-init",
            ],
        )
        run_checked(
            self.runner,
            self.evidence,
            "container_isolation_probe",
            [
                "docker",
                "exec",
                self.container_name,
                "sh",
                "-c",
                ISOLATION_PROBE,
            ],
        )
        run_checked(
            self.runner,
            self.evidence,
            "container_access_probe",
            [
                "docker",
                "exec",
                self.container_name,
                "sh",
                "-c",
                _access_probe_script(self.resources),
            ],
        )

    def preserve_container_evidence(self) -> None:
        if not self.created:
            return
        census = self.runner.run(
            [
                "docker",
                "exec",
                self.container_name,
                "sh",
                "-c",
                "ps -eo pid,ppid,pgid,stat,args",
            ],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        self.evidence.command(
            "container_pre_cleanup_census", census, COMMAND_TIMEOUT_SECONDS
        )
        if census.returncode != 0 or census.timed_out:
            raise PreflightError(
                "container_evidence",
                "failed to capture the pre-cleanup process census",
            )
        presence = self.runner.run(
            [
                "docker",
                "exec",
                self.container_name,
                "sh",
                "-c",
                f"test -d {shlex.quote(self.workspace + '/runtime-evidence')}",
            ],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        self.evidence.command(
            "container_runtime_evidence_presence",
            presence,
            COMMAND_TIMEOUT_SECONDS,
        )
        if presence.timed_out or presence.returncode not in (0, 1):
            raise PreflightError(
                "container_evidence",
                "could not determine whether runtime evidence exists",
            )
        if presence.returncode == 1:
            return
        result = self.runner.run(
            [
                "docker",
                "cp",
                f"{self.container_name}:{self.workspace}/runtime-evidence/.",
                str(self.evidence.root / "container-artifacts"),
            ],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        self.evidence.command(
            "container_evidence_copy", result, COMMAND_TIMEOUT_SECONDS
        )
        if result.returncode != 0 or result.timed_out:
            raise PreflightError(
                "container_evidence",
                "failed to preserve existing runtime evidence",
            )

    def cleanup(self) -> None:
        if not self.created:
            return
        errors = []
        try:
            if self._inspect_optional() is None:
                self.created = False
                return
        except PreflightError as exc:
            errors.append(exc.phase)
        try:
            self.preserve_container_evidence()
        except PreflightError as exc:
            errors.append(exc.phase)
        for phase, command, accepted in (
            (
                "container_cleanup_stop",
                [
                    "docker",
                    "stop",
                    "--time",
                    str(DOCKER_STOP_TIMEOUT_SECONDS),
                    self.container_name,
                ],
                (0, 1),
            ),
            (
                "container_cleanup_remove",
                ["docker", "rm", self.container_name],
                (0, 1),
            ),
        ):
            result = self.runner.run(command, timeout=COMMAND_TIMEOUT_SECONDS)
            self.evidence.command(phase, result, COMMAND_TIMEOUT_SECONDS)
            if result.returncode not in accepted or result.timed_out:
                errors.append(phase)
        census = self.runner.run(
            ["docker", "inspect", self.container_name],
            timeout=COMMAND_TIMEOUT_SECONDS,
        )
        self.evidence.command(
            "container_cleanup_census", census, COMMAND_TIMEOUT_SECONDS
        )
        if census.returncode != 1 or census.timed_out:
            errors.append("container_cleanup_census")
        if errors:
            raise PreflightError(
                "cleanup", f"dedicated container cleanup failed: {errors}"
            )
        self.created = False


def relevant_audit_failure(
    text: str,
    *,
    profile_name: str,
    usb_topology_name: str,
) -> Optional[str]:
    """Return the first current-run D455 denial/reset line, if present."""
    topology_pattern = re.escape(usb_topology_name)
    for line in text.splitlines():
        lower = line.lower()
        if 'apparmor="denied"' in lower and profile_name in line:
            return line
        if re.search(
            rf"usb\s+{topology_pattern}:.*"
            r"(reset|disconnect|device descriptor|unable to enumerate|"
            r"not accepting address)",
            line,
            re.IGNORECASE,
        ):
            return line
    return None


def audit_command(since_utc: str) -> list[str]:
    since = journalctl_since_timestamp(since_utc)
    return [
        "journalctl",
        "--utc",
        "--dmesg",
        "--since",
        since,
        "--no-pager",
        "-o",
        "short-iso-precise",
    ]


def journalctl_since_timestamp(value: str) -> str:
    """Format an ISO UTC timestamp for journalctl's bounded --since parser.

    Python's datetime retains microseconds (flooring any finer input), which
    keeps the audit boundary at or just before the recorded workflow start.
    The explicit UTC suffix avoids locale-dependent interpretation.
    """
    try:
        normalized_value = value
        fractional = re.fullmatch(
            r"(.*?[T ]\d{2}:\d{2}:\d{2})"
            r"(?:[.,](\d+))?"
            r"(Z|z|[+-]\d{2}:\d{2})",
            value,
        )
        if fractional:
            digits = (fractional.group(2) or "")[:6].ljust(6, "0")
            normalized_value = (
                fractional.group(1)
                + "."
                + digits
                + fractional.group(3)
            )
        parsed = datetime.fromisoformat(
            normalized_value.replace("Z", "+00:00").replace(
                "z", "+00:00"
            )
        )
    except (TypeError, ValueError) as exc:
        raise PreflightError(
            "audit_since", f"invalid UTC timestamp: {value!r}"
        ) from exc
    if parsed.tzinfo is None:
        raise PreflightError(
            "audit_since", "journalctl timestamp must include a timezone"
        )
    normalized = parsed.astimezone(timezone.utc)
    return normalized.strftime("%Y-%m-%d %H:%M:%S.%f UTC")


def validate_current_audit(
    runner: Any,
    evidence: Evidence,
    *,
    phase: str,
    since_utc: str,
    resources: HostResources,
) -> None:
    result = run_checked(
        runner,
        evidence,
        phase,
        audit_command(since_utc),
    )
    failure = relevant_audit_failure(
        result.stdout,
        profile_name=PROFILE_NAME,
        usb_topology_name=resources.usb_sysfs_path.name,
    )
    if failure:
        raise PreflightError(
            phase, f"current-run kernel audit failure: {failure}"
        )


def runtime_validation_command(config: RuntimeConfig) -> list[str]:
    return [
        sys.executable,
        str(config.validation_wrapper),
        "--container",
        config.container_name,
        "--workspace",
        config.workspace,
        "--evidence-dir",
        str(config.evidence_dir / "runtime"),
        "--execute",
        "--acknowledge-exact-zero-only",
    ]


def execute_workflow(
    config: RuntimeConfig,
    *,
    authorize_profile_reload: bool,
    authorize_container_recreate: bool,
    authorize_stationary_d455: bool,
    authorize_ros_no_motion: bool,
    runner: Optional[Any] = None,
    discover: Callable[[str], HostResources] = select_host_resources,
    serial_verifier: Optional[Callable[[str], None]] = None,
    profile_manager_factory: Optional[Callable[..., ProfileManager]] = None,
    production_exclusion_checker: Callable[
        [Any, Evidence], None
    ] = reject_active_production_container,
    lock_factory: Callable[[str], Any] = d455_host_lock,
) -> None:
    """Execute the serial-derived, fail-closed host/runtime workflow."""
    runner = runner or SubprocessRunner()
    started_at = utc_now()
    container: Optional[ContainerManager] = None
    evidence: Optional[Evidence] = None
    terminal_error: Optional[BaseException] = None
    cleanup_result = "not_applicable"
    workflow_lock: Optional[Any] = None
    lock_acquired = False
    context = failure_context(
        config,
        authorize_profile_reload=authorize_profile_reload,
        authorize_container_recreate=authorize_container_recreate,
        authorize_stationary_d455=authorize_stationary_d455,
        authorize_ros_no_motion=authorize_ros_no_motion,
    )
    try:
        evidence = Evidence(config.evidence_dir)
        evidence.event(
            "preflight_started", serial_number=config.serial_number
        )
        if not (
            authorize_container_recreate
            and authorize_stationary_d455
            and authorize_ros_no_motion
        ):
            raise ApprovalRequired(
                "authorization",
                "container recreation, stationary D455 access, and "
                "ROS no-motion validation each require explicit "
                "authorization",
            )
        try:
            workflow_lock = lock_factory("validation_workflow")
            workflow_lock.__enter__()
            lock_acquired = True
            evidence.event("host_lock_acquired")
        except D455HostLockError as exc:
            raise PreflightError("host_lock", str(exc)) from exc
        production_exclusion_checker(runner, evidence)
        if serial_verifier is None:
            verify_serial_bounded(
                runner, evidence, config.serial_number
            )
        else:
            serial_verifier(config.serial_number)
        resources = discover(config.usb_serial_number)
        validate_resource_set(resources)
        profile = generate_apparmor_profile(resources)
        manifest = build_manifest(
            resources,
            serial_number=config.serial_number,
            usb_serial_number=config.usb_serial_number,
            profile=profile,
            generated_at=started_at,
        )
        candidate_path = evidence.root / "candidate-profile.apparmor"
        evidence.write_text(candidate_path.name, profile)
        evidence.write_json("resource-manifest.json", manifest)

        factory = profile_manager_factory or ProfileManager
        manager = factory(
            runner=runner,
            evidence=evidence,
            installed_profile_path=config.installed_profile_path,
            installed_manifest_path=config.installed_manifest_path,
            kernel_profiles_path=config.kernel_profiles_path,
        )
        decision = manager.ensure(
            candidate_profile_path=candidate_path,
            profile=profile,
            manifest=manifest,
            authorize_reload=authorize_profile_reload,
        )
        evidence.write_json("profile-decision.json", asdict(decision))

        if serial_verifier is None:
            verify_serial_bounded(
                runner, evidence, config.serial_number
            )
        else:
            serial_verifier(config.serial_number)
        assert_resources_unchanged(
            resources,
            discover(config.usb_serial_number),
            phase="resources_before_container",
        )
        validate_current_audit(
            runner,
            evidence,
            phase="audit_before_container",
            since_utc=started_at,
            resources=resources,
        )
        container = ContainerManager(
            runner=runner,
            evidence=evidence,
            container_name=config.container_name,
            image=config.image,
            resources=resources,
            workspace=config.workspace,
        )
        container.create()
        container.verify_and_preflight_access()
        if serial_verifier is None:
            verify_serial_bounded(
                runner, evidence, config.serial_number
            )
        else:
            serial_verifier(config.serial_number)
        assert_resources_unchanged(
            resources,
            discover(config.usb_serial_number),
            phase="resources_before_ros",
        )
        validate_current_audit(
            runner,
            evidence,
            phase="audit_before_ros",
            since_utc=started_at,
            resources=resources,
        )
        run_checked(
            runner,
            evidence,
            "no_motion_runtime_validation",
            runtime_validation_command(config),
            timeout=180.0,
        )
        validate_current_audit(
            runner,
            evidence,
            phase="audit_after_ros",
            since_utc=started_at,
            resources=resources,
        )
        evidence.event("preflight_completed")
    except BaseException as exc:
        terminal_error = exc
    finally:
        if evidence is None:
            if terminal_error is not None:
                try:
                    preserve_fallback_failure(
                        config.evidence_dir,
                        terminal_error,
                        context=context,
                        cleanup_result=cleanup_result,
                    )
                except BaseException:
                    pass
        else:
            if container is not None and container.created:
                try:
                    container.cleanup()
                    cleanup_result = "succeeded"
                except BaseException as cleanup_error:
                    cleanup_result = "failed"
                    preserve_failure_evidence(
                        evidence,
                        cleanup_error,
                        event="cleanup_failed",
                        filename="cleanup-failure.json",
                        context=context,
                        cleanup_result=cleanup_result,
                    )
                    if terminal_error is None:
                        terminal_error = cleanup_error
            if lock_acquired and workflow_lock is not None:
                try:
                    workflow_lock.__exit__(None, None, None)
                    evidence.event("host_lock_released")
                except BaseException as lock_error:
                    if terminal_error is None:
                        terminal_error = lock_error
            if terminal_error is not None:
                preserve_failure_evidence(
                    evidence,
                    terminal_error,
                    event="preflight_failed",
                    filename="failure.json",
                    context=context,
                    cleanup_result=cleanup_result,
                )
            try:
                evidence.write_json(
                    "result.json",
                    {
                        "result": (
                            "passed" if terminal_error is None else "failed"
                        ),
                        "error": (
                            None
                            if terminal_error is None
                            else str(terminal_error)
                        ),
                        "no_nonzero_twist": True,
                    },
                )
            except BaseException as evidence_error:
                preserve_failure_evidence(
                    evidence,
                    evidence_error,
                    event="result_evidence_failed",
                    filename="result-evidence-failure.json",
                    context=context,
                    cleanup_result=cleanup_result,
                )
                if terminal_error is None:
                    terminal_error = evidence_error
    if terminal_error is not None:
        raise terminal_error


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial-number", default=DEFAULT_SERIAL_NUMBER)
    parser.add_argument(
        "--usb-serial-number", default=DEFAULT_USB_SERIAL_NUMBER
    )
    parser.add_argument("--image", required=True)
    parser.add_argument("--container-name", default=DEFAULT_CONTAINER_NAME)
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorize-profile-reload", action="store_true")
    parser.add_argument("--authorize-container-recreate", action="store_true")
    parser.add_argument("--authorize-stationary-d455", action="store_true")
    parser.add_argument("--authorize-ros-no-motion", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    if not args.execute:
        raise SystemExit("refusing to execute without --execute")
    if not Path(args.workspace).is_absolute():
        raise SystemExit("workspace must be an absolute path")
    config = RuntimeConfig(
        serial_number=args.serial_number,
        usb_serial_number=args.usb_serial_number,
        image=args.image,
        container_name=args.container_name,
        workspace=args.workspace,
        evidence_dir=args.evidence_dir,
        installed_profile_path=INSTALLED_PROFILE_PATH,
        installed_manifest_path=INSTALLED_MANIFEST_PATH,
        kernel_profiles_path=KERNEL_PROFILES_PATH,
        validation_wrapper=VALIDATION_WRAPPER_PATH,
    )
    try:
        execute_workflow(
            config,
            authorize_profile_reload=args.authorize_profile_reload,
            authorize_container_recreate=args.authorize_container_recreate,
            authorize_stationary_d455=args.authorize_stationary_d455,
            authorize_ros_no_motion=args.authorize_ros_no_motion,
        )
    except (PreflightError, OSError, ValueError) as exc:
        print(f"D455 preflight failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
