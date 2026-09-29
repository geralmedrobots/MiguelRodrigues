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

"""
Fail-closed adapter for the standalone TeleDex/ARKit logger.

The adapter imports the standalone logger's transform helpers at runtime; it
does not reimplement TeleDex transport, start ROS nodes, or publish commands.
"""

from dataclasses import asdict
from dataclasses import dataclass
import hashlib
import importlib
import importlib.util
import math
import os
from pathlib import Path
import sys
import time
from threading import Lock
from typing import Callable, Dict, List, Optional, Tuple

from odometry_validation.core import ValidationError


REQUIRED_LOGGER_FUNCTIONS = (
    "base_displacement_metrics",
    "relative_pose",
    "relative_base_pose",
    "rotation_from_rpy",
    "rotation_vector_from_matrix",
    "rpy_from_matrix",
    "unwrap_angle",
    "validate_rotation_matrix",
    "validate_pose",
)
REQUIRED_SESSION_METHODS = ("start", "get_latest_data", "stop")
AXIS_DOMINANCE_RATIO = 2.0


def phone_to_base_translation_from_base_phone_position(
        phone_position_base_m: Tuple[float, float, float],
        phone_to_base_rpy_deg: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Convert a phone origin measured in base axes into ``T_phone_base``.

    The measured vector is the phone origin expressed in base coordinates
    (the translation of ``T_base_phone``).  The CLI instead requires the base
    origin expressed in phone coordinates (the translation of
    ``T_phone_base``), so the inverse rigid transform is used:
    ``t_phone_base = -R_phone_base * t_base_phone``.
    """
    if (len(phone_position_base_m) != 3 or
            not all(math.isfinite(value) for value in phone_position_base_m)):
        raise ValueError("phone position in base coordinates must contain three finite values")
    if (len(phone_to_base_rpy_deg) != 3 or
            not all(math.isfinite(value) for value in phone_to_base_rpy_deg)):
        raise ValueError("phone-to-base RPY must contain three finite values")
    roll, pitch, yaw = (math.radians(value) for value in phone_to_base_rpy_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Rz(yaw)*Ry(pitch)*Rx(roll), matching the standalone logger.
    rotation = (
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr)
    rotated = tuple(sum(rotation[row * 3 + column] * phone_position_base_m[column]
                        for column in range(3)) for row in range(3))
    return tuple(-value for value in rotated)


class TeleDexMountMappingMismatch(ValidationError):
    """Carry a valid read-only mount observation that disagrees with config."""

    def __init__(self, message: str, observation: Dict[str, object]):
        super().__init__(message)
        self.observation = dict(observation)


@dataclass(frozen=True)
class TeleDexStreamDiagnostics:
    """Transport-level evidence independent of adapter polling frequency."""

    update_callback_available: bool
    websocket_message_count: int
    first_websocket_message_monotonic_s: Optional[float]
    last_websocket_message_monotonic_s: Optional[float]
    websocket_message_age_s: Optional[float]
    latest_websocket_interarrival_s: Optional[float]
    longest_websocket_interarrival_s: Optional[float]
    pose_change_count: int
    last_pose_change_monotonic_s: Optional[float]
    pose_change_age_s: Optional[float]
    longest_pose_change_interval_s: Optional[float]
    connection_generation: int
    connection_lost: bool
    source_timestamp_available: bool
    source_sequence_available: bool
    session_id: Optional[str] = None
    receive_discontinuities: Tuple[Dict[str, object], ...] = ()


@dataclass(frozen=True)
class TeleDexPoseSample:
    """One raw TeleDex pose preserved with local receive timing."""

    receive_monotonic_s: float
    receive_wall_time_iso: str
    arkit_timestamp_s: Optional[float]
    x_m: Optional[float]
    y_m: Optional[float]
    z_m: Optional[float]
    r00: Optional[float]
    r01: Optional[float]
    r02: Optional[float]
    r10: Optional[float]
    r11: Optional[float]
    r12: Optional[float]
    r20: Optional[float]
    r21: Optional[float]
    r22: Optional[float]
    relative_x_m: Optional[float] = None
    relative_y_m: Optional[float] = None
    relative_z_m: Optional[float] = None
    forward_displacement_m: Optional[float] = None
    lateral_displacement_m: Optional[float] = None
    planar_displacement_m: Optional[float] = None
    total_3d_displacement_m: Optional[float] = None
    yaw_rad: Optional[float] = None
    yaw_unwrapped_rad: Optional[float] = None
    raw_pose_generation: int = 0
    raw_pose_age_s: Optional[float] = None
    raw_pose_changed: bool = False
    websocket_message_count: int = 0
    websocket_message_age_s: Optional[float] = None
    latest_websocket_interarrival_s: Optional[float] = None
    longest_websocket_interarrival_s: Optional[float] = None
    connection_generation: int = 0
    connection_lost: bool = False
    valid_sample: bool = False
    status_reason: str = "uninitialized"


@dataclass(frozen=True)
class TeleDexTrialResult:
    """Physical-reference values finalised only after ROS stationarity."""

    initial_pose: TeleDexPoseSample
    final_pose: TeleDexPoseSample
    total_planar_path_length_m: float
    net_forward_displacement_m: float
    lateral_displacement_m: float
    final_yaw_rad: float
    final_yaw_unwrapped_rad: float
    source_timestamp_available: bool
    phone_to_base_rotation: Tuple[float, ...]
    phone_to_base_translation_m: Tuple[float, float, float]
    ros_initial_timestamp_s: Optional[float]
    ros_final_timestamp_s: Optional[float]
    discontinuities_observed: int
    stream_diagnostics: Dict[str, object]

    def as_dict(self) -> Dict[str, object]:
        return asdict(self)


def _iso_utc_now() -> str:
    from datetime import datetime
    from datetime import timezone
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _source_timestamp_from_update(data: object) -> Optional[float]:
    """Extract optional upstream timing without accepting malformed values."""
    if not isinstance(data, dict):
        return None
    for key in ("timestamp", "arkit_timestamp", "timestamp_s"):
        value = data.get(key)
        try:
            value = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            return value
    return None


def load_standalone_logger(reference_root: Path):
    """Load the existing logger module directly from its explicit location."""
    root = Path(reference_root).expanduser()
    if not root.exists():
        raise ValidationError(
            "standalone TeleDex reference directory does not exist: " + str(root))
    if not root.is_dir():
        raise ValidationError(
            "standalone TeleDex reference root is not a directory: " + str(root))
    if not os.access(root, os.R_OK | os.X_OK):
        raise ValidationError(
            "standalone TeleDex reference directory is not readable: " + str(root))
    module_path = root / "teledex_logger.py"
    if not module_path.is_file():
        raise ValidationError(
            "standalone TeleDex logger is unavailable: " + str(module_path))
    if not os.access(module_path, os.R_OK):
        raise ValidationError(
            "standalone TeleDex logger is not readable: " + str(module_path))
    path_digest = hashlib.sha256(
        str(module_path.resolve()).encode("utf-8")).hexdigest()[:16]
    module_name = "odometry_validation_standalone_teledex_logger_" + path_digest
    existing = sys.modules.get(module_name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise ValidationError("could not load standalone TeleDex logger")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(module_name, None)
        raise
    missing = tuple(
        name for name in REQUIRED_LOGGER_FUNCTIONS
        if not callable(getattr(module, name, None)))
    if missing:
        sys.modules.pop(module_name, None)
        raise ValidationError(
            "standalone TeleDex logger API is incomplete; missing: " +
            ", ".join(missing))
    return module


def load_teledex_session_factory(
        package_importer: Callable[[str], object] = importlib.import_module
        ) -> Callable[[], object]:
    """Import and validate the installed TeleDex transport API."""
    try:
        package = package_importer("teledex")
    except (ImportError, ModuleNotFoundError) as error:
        raise ValidationError(
            "TeleDex Python package is unavailable in this interpreter") from error
    session_factory = getattr(package, "Session", None)
    if not callable(session_factory):
        raise ValidationError("TeleDex Python package has no callable Session")
    missing = tuple(
        name for name in REQUIRED_SESSION_METHODS
        if not callable(getattr(session_factory, name, None)))
    if missing:
        raise ValidationError(
            "TeleDex Session API is incomplete; missing: " + ", ".join(missing))
    return session_factory


class TeleDexReferenceAdapter:
    """Collect a per-trial TeleDex trajectory using the standalone math API."""

    def __init__(
            self,
            reference_root: Path,
            phone_to_base_rpy_deg: Tuple[float, float, float],
            phone_to_base_translation_m: Tuple[float, float, float],
            stale_timeout_s: float,
            session_factory: Optional[Callable[[], object]] = None,
            monotonic: Callable[[], float] = time.monotonic,
            readiness_timeout_s: float = 5.0,
            sleep: Callable[[float], None] = time.sleep):
        if not math.isfinite(stale_timeout_s) or stale_timeout_s <= 0.0:
            raise ValueError("TeleDex stale timeout must be finite and positive")
        if (len(phone_to_base_rpy_deg) != 3 or
                not all(math.isfinite(value) for value in phone_to_base_rpy_deg)):
            raise ValueError("TeleDex phone-to-base RPY must contain three finite values")
        if (len(phone_to_base_translation_m) != 3 or
                not all(math.isfinite(value) for value in phone_to_base_translation_m)):
            raise ValueError(
                "TeleDex phone-to-base translation must contain three finite values")
        self.reference_root = Path(reference_root).expanduser()
        self.phone_to_base_rpy_deg = tuple(phone_to_base_rpy_deg)
        self.phone_to_base_translation_m = tuple(phone_to_base_translation_m)
        self.phone_to_base_rotation: Optional[Tuple[float, ...]] = None
        self.stale_timeout_s = stale_timeout_s
        if not math.isfinite(readiness_timeout_s) or readiness_timeout_s <= 0.0:
            raise ValueError("TeleDex readiness timeout must be finite and positive")
        self.readiness_timeout_s = readiness_timeout_s
        self.session_factory = session_factory
        self.monotonic = monotonic
        self.sleep = sleep
        self._logger_module = None
        self._session = None
        self._stream_lock = Lock()
        self._trajectory: List[TeleDexPoseSample] = []
        self._initial_pose: Optional[TeleDexPoseSample] = None
        self._last_raw_pose_change_monotonic_s: Optional[float] = None
        self._last_raw_pose_source_timestamp_s: Optional[float] = None
        self._raw_pose_fingerprint: Optional[Tuple[float, ...]] = None
        self._raw_pose_generation = 0
        self._last_yaw_unwrapped: Optional[float] = None
        self._path_length_m = 0.0
        self._source_timestamp_available = False
        self._ros_initial_timestamp_s: Optional[float] = None
        self.axis_validation: Optional[Dict[str, object]] = None
        self._trial_connection_generation: Optional[int] = None
        self._connection_generation = 0
        self._connection_lost = False
        self._update_callback_available = False
        self._websocket_message_count = 0
        self._first_websocket_message_monotonic_s: Optional[float] = None
        self._last_websocket_message_monotonic_s: Optional[float] = None
        self._latest_websocket_interarrival_s: Optional[float] = None
        self._longest_websocket_interarrival_s: Optional[float] = None
        self._longest_pose_change_interval_s: Optional[float] = None
        self._source_sequence_available = False
        self._last_websocket_source_timestamp_s: Optional[float] = None
        self._session_id: Optional[str] = None
        self._trial_receiving = False
        self._receive_discontinuities: List[Dict[str, object]] = []

    @property
    def trajectory(self) -> Tuple[TeleDexPoseSample, ...]:
        return tuple(self._trajectory)

    def start(self) -> None:
        """Start the installed TeleDex session and require the first valid pose."""
        self._logger_module = load_standalone_logger(self.reference_root)
        self.phone_to_base_rotation = self._logger_module.validate_rotation_matrix(
            self._logger_module.rotation_from_rpy(
                *(math.radians(value) for value in self.phone_to_base_rpy_deg)))
        if self.session_factory is None:
            self.session_factory = load_teledex_session_factory()
        self._session = self.session_factory()
        configured_session_id = getattr(self._session, "session_id", None)
        self._session_id = (
            str(configured_session_id) if configured_session_id is not None else
            type(self._session).__name__ + ":" + format(id(self._session), "x"))
        on_disconnect = getattr(self._session, "on_disconnect", None)
        if callable(on_disconnect):
            on_disconnect(self._on_disconnect)
        on_connect = getattr(self._session, "on_connect", None)
        if callable(on_connect):
            on_connect(self._on_connect)
        on_update = getattr(self._session, "on_update", None)
        if callable(on_update):
            on_update(self._on_update)
            self._update_callback_available = True
        try:
            self._session.start()
        except BaseException as error:
            self.stop()
            raise ValidationError("TeleDex session failed to start: " + str(error)) from error

    def _on_connect(self, _session: object) -> None:
        """Record a transport reconnect; the ARKit world frame may have changed."""
        with self._stream_lock:
            self._connection_generation += 1
            self._connection_lost = False

    def _on_disconnect(self, _session: object) -> None:
        """Invalidate the active trial after a transport disconnect."""
        with self._stream_lock:
            self._connection_lost = True

    def _on_update(self, _session: object, data: object) -> None:
        """Record WebSocket callback timing without treating it as a valid pose."""
        received_mono = self.monotonic()
        source_timestamp = _source_timestamp_from_update(data)
        with self._stream_lock:
            previous = self._last_websocket_message_monotonic_s
            previous_source_timestamp = self._last_websocket_source_timestamp_s
            previous_message_index = self._websocket_message_count
            if previous is not None:
                interval_s = max(0.0, received_mono - previous)
                self._latest_websocket_interarrival_s = interval_s
                self._longest_websocket_interarrival_s = max(
                    interval_s, self._longest_websocket_interarrival_s or 0.0)
                if self._trial_receiving and interval_s >= self.stale_timeout_s:
                    self._receive_discontinuities.append({
                        "basis": "websocket_callback",
                        "gap_duration_s": interval_s,
                        "previous_receive_monotonic_s": previous,
                        "current_receive_monotonic_s": received_mono,
                        "previous_source_timestamp_s": previous_source_timestamp,
                        "current_source_timestamp_s": source_timestamp,
                        "previous_websocket_message_index": previous_message_index,
                        "current_websocket_message_index": previous_message_index + 1,
                        "connection_generation": self._connection_generation,
                        "session_id": self._session_id,
                        "reconnect_observed": (
                            self._trial_connection_generation is not None and
                            self._connection_generation !=
                            self._trial_connection_generation),
                        "connection_lost": self._connection_lost,
                    })
            self._websocket_message_count += 1
            if self._first_websocket_message_monotonic_s is None:
                self._first_websocket_message_monotonic_s = received_mono
            self._last_websocket_message_monotonic_s = received_mono
            self._last_websocket_source_timestamp_s = source_timestamp
            if isinstance(data, dict):
                self._source_sequence_available |= any(
                    key in data for key in ("sequence", "seq", "frame_id"))

    def stream_diagnostics(self) -> TeleDexStreamDiagnostics:
        """Snapshot observable WebSocket/message and pose-change stream health."""
        now = self.monotonic()
        with self._stream_lock:
            last_message = self._last_websocket_message_monotonic_s
            last_pose = self._last_raw_pose_change_monotonic_s
            return TeleDexStreamDiagnostics(
                update_callback_available=self._update_callback_available,
                websocket_message_count=self._websocket_message_count,
                first_websocket_message_monotonic_s=(
                    self._first_websocket_message_monotonic_s),
                last_websocket_message_monotonic_s=last_message,
                websocket_message_age_s=(
                    None if last_message is None else max(0.0, now - last_message)),
                latest_websocket_interarrival_s=self._latest_websocket_interarrival_s,
                longest_websocket_interarrival_s=self._longest_websocket_interarrival_s,
                pose_change_count=self._raw_pose_generation,
                last_pose_change_monotonic_s=last_pose,
                pose_change_age_s=(
                    None if last_pose is None else max(0.0, now - last_pose)),
                longest_pose_change_interval_s=self._longest_pose_change_interval_s,
                connection_generation=self._connection_generation,
                connection_lost=self._connection_lost,
                source_timestamp_available=self._source_timestamp_available,
                source_sequence_available=self._source_sequence_available,
                session_id=self._session_id,
                receive_discontinuities=tuple(
                    dict(event) for event in self._receive_discontinuities))

    def stop(self) -> None:
        """Close the reference-only TeleDex session without affecting ROS."""
        if self._session is None:
            return
        try:
            self._session.stop()
        except BaseException as error:
            raise ValidationError(
                "TeleDex session failed to stop cleanly: " + str(error)) from error
        finally:
            self._session = None

    def begin_trial(self, ros_timestamp_s: Optional[float]) -> TeleDexPoseSample:
        """Capture an explicit fresh reference pose at the trial start boundary."""
        if self.stream_diagnostics().connection_lost:
            raise ValidationError(
                "TeleDex initial reference is unavailable while transport is disconnected")
        self._trajectory = []
        self._initial_pose = None
        self._last_yaw_unwrapped = None
        self._path_length_m = 0.0
        self._ros_initial_timestamp_s = ros_timestamp_s
        with self._stream_lock:
            self._trial_receiving = False
            self._receive_discontinuities = []
        sample = self._wait_for_next_valid_pose("initial TeleDex reference")
        self._initial_pose = sample
        with self._stream_lock:
            self._trial_connection_generation = self._connection_generation
            # Gaps before this fresh trial reference do not affect this trial.
            self._trial_receiving = True
        return sample

    def _wait_for_next_valid_pose(self, purpose: str) -> TeleDexPoseSample:
        """Require a post-boundary transport update when the API exposes it."""
        diagnostics = self.stream_diagnostics()
        if diagnostics.update_callback_available:
            return self.wait_for_valid_pose(
                purpose,
                minimum_websocket_message_count=diagnostics.websocket_message_count)
        return self.wait_for_valid_pose(purpose, self._raw_pose_generation)

    def wait_for_valid_pose(
            self, purpose: str, minimum_raw_pose_generation: Optional[int] = None,
            minimum_websocket_message_count: Optional[int] = None
            ) -> TeleDexPoseSample:
        """Wait for a valid source update, never treating a cached pose as new."""
        deadline = self.monotonic() + self.readiness_timeout_s
        last_reason = "no sample received"
        while True:
            sample = self.poll()
            sample_freshness_age_s = self._sample_freshness_age(sample)
            if (sample.valid_sample and
                    (minimum_raw_pose_generation is None or
                     sample.raw_pose_generation > minimum_raw_pose_generation) and
                    (minimum_websocket_message_count is None or
                     sample.websocket_message_count >
                     minimum_websocket_message_count) and
                    sample_freshness_age_s is not None and
                    sample_freshness_age_s < self.stale_timeout_s):
                return sample
            if sample.valid_sample:
                if (minimum_websocket_message_count is not None and
                        sample.websocket_message_count <=
                        minimum_websocket_message_count):
                    last_reason = "no new WebSocket source update since the trial boundary"
                elif (minimum_raw_pose_generation is not None and
                        sample.raw_pose_generation <= minimum_raw_pose_generation):
                    last_reason = "no new ARKit pose since the trial boundary"
                else:
                    last_reason = "source update is stale by %.3fs" % (
                        sample_freshness_age_s or 0.0)
            else:
                last_reason = sample.status_reason
            remaining = deadline - self.monotonic()
            if remaining <= 0.0:
                raise ValidationError(
                    "TeleDex " + purpose + " unavailable after %.3fs: %s" %
                    (self.readiness_timeout_s, last_reason))
            self.sleep(min(0.01, remaining))

    def _sample_freshness_age(self, sample: TeleDexPoseSample) -> Optional[float]:
        """Use callback timing when available; otherwise retain cache-age fallback."""
        if self._update_callback_available and sample.websocket_message_count > 0:
            return sample.websocket_message_age_s
        return sample.raw_pose_age_s

    def validate_mount_mapping(
            self, prompt: Callable[[str], str], minimum_displacement_m: float,
            minimum_yaw_rad: float
            ) -> Dict[str, object]:
        """Infer a discrete right-handed mount from forward and CCW observations."""
        if (not math.isfinite(minimum_displacement_m) or
                minimum_displacement_m <= 0.0):
            raise ValueError("axis-validation displacement must be finite and positive")
        if not math.isfinite(minimum_yaw_rad) or minimum_yaw_rad <= 0.0:
            raise ValueError("axis-validation yaw must be finite and positive")
        initial = self._wait_for_next_valid_pose("axis-validation initial pose")
        prompt(
            "After separately approved supervised motion, move the robot/phone "
            "straight forward, stop, then press ENTER to capture the forward axis: ")
        forward_pose = self._wait_for_next_valid_pose("axis-validation forward pose")
        prompt(
            "Return to the initial pose. After separately approved supervised "
            "motion, rotate the robot a small amount counter-clockwise, stop, "
            "then press ENTER to capture the yaw axis: ")
        ccw_pose = self._wait_for_next_valid_pose("axis-validation CCW pose")
        initial_position = (initial.x_m, initial.y_m, initial.z_m)
        initial_rotation = (
            initial.r00, initial.r01, initial.r02,
            initial.r10, initial.r11, initial.r12,
            initial.r20, initial.r21, initial.r22)
        forward_position = (
            forward_pose.x_m, forward_pose.y_m, forward_pose.z_m)
        forward_rotation = (
            forward_pose.r00, forward_pose.r01, forward_pose.r02,
            forward_pose.r10, forward_pose.r11, forward_pose.r12,
            forward_pose.r20, forward_pose.r21, forward_pose.r22)
        ccw_position = (ccw_pose.x_m, ccw_pose.y_m, ccw_pose.z_m)
        ccw_rotation = (
            ccw_pose.r00, ccw_pose.r01, ccw_pose.r02,
            ccw_pose.r10, ccw_pose.r11, ccw_pose.r12,
            ccw_pose.r20, ccw_pose.r21, ccw_pose.r22)
        translation, _unused_rotation = self._logger_module.relative_pose(
            initial_position, initial_rotation,
            forward_position, forward_rotation)
        _unused_translation, phone_relative_ccw = self._logger_module.relative_pose(
            initial_position, initial_rotation, ccw_position, ccw_rotation)
        rotation_vector = self._logger_module.rotation_vector_from_matrix(
            phone_relative_ccw)
        forward_index = max(range(3), key=lambda value: abs(translation[value]))
        magnitude = abs(translation[forward_index])
        if magnitude < minimum_displacement_m:
            raise ValidationError(
                "TeleDex axis validation displacement is too small: %.3fm" % magnitude)
        forward_secondary = max(
            abs(value) for index, value in enumerate(translation)
            if index != forward_index)
        if (forward_secondary > 0.0 and
                magnitude / forward_secondary < AXIS_DOMINANCE_RATIO):
            raise ValidationError(
                "TeleDex axis validation forward observation is ambiguous; "
                "dominant/secondary axis ratio %.3f is below %.3f" % (
                    magnitude / forward_secondary, AXIS_DOMINANCE_RATIO))
        yaw_index = max(range(3), key=lambda value: abs(rotation_vector[value]))
        yaw_magnitude = abs(rotation_vector[yaw_index])
        if yaw_magnitude < minimum_yaw_rad:
            raise ValidationError(
                "TeleDex axis validation CCW rotation is too small: %.3fdeg" %
                math.degrees(yaw_magnitude))
        yaw_secondary = max(
            abs(value) for index, value in enumerate(rotation_vector)
            if index != yaw_index)
        if (yaw_secondary > 0.0 and
                yaw_magnitude / yaw_secondary < AXIS_DOMINANCE_RATIO):
            raise ValidationError(
                "TeleDex axis validation CCW observation is ambiguous; "
                "dominant/secondary axis ratio %.3f is below %.3f" % (
                    yaw_magnitude / yaw_secondary, AXIS_DOMINANCE_RATIO))
        if forward_index == yaw_index:
            raise ValidationError(
                "TeleDex mount validation found the same phone axis for forward and yaw")
        base_x_phone = [0.0, 0.0, 0.0]
        base_z_phone = [0.0, 0.0, 0.0]
        base_x_phone[forward_index] = 1.0 if translation[forward_index] > 0.0 else -1.0
        base_z_phone[yaw_index] = 1.0 if rotation_vector[yaw_index] > 0.0 else -1.0
        base_y_phone = (
            base_z_phone[1] * base_x_phone[2] -
            base_z_phone[2] * base_x_phone[1],
            base_z_phone[2] * base_x_phone[0] -
            base_z_phone[0] * base_x_phone[2],
            base_z_phone[0] * base_x_phone[1] -
            base_z_phone[1] * base_x_phone[0])
        mount_rotation = tuple(
            value
            for row in range(3)
            for value in (base_x_phone[row], base_y_phone[row], base_z_phone[row]))
        detected_rotation = self._logger_module.validate_rotation_matrix(
            mount_rotation)
        if self.phone_to_base_rotation is None:
            raise ValidationError("TeleDex phone-to-base mount is not configured")
        forward_axis = (
            ("+" if translation[forward_index] > 0.0 else "-") +
            "xyz"[forward_index])
        yaw_axis = (
            ("+" if rotation_vector[yaw_index] > 0.0 else "-") +
            "xyz"[yaw_index])
        self.axis_validation = {
            "initial_receive_monotonic_s": initial.receive_monotonic_s,
            "forward_receive_monotonic_s": forward_pose.receive_monotonic_s,
            "ccw_receive_monotonic_s": ccw_pose.receive_monotonic_s,
            "relative_translation_m": list(translation),
            "relative_rotation_vector_rad": list(rotation_vector),
            "detected_phone_forward_axis": forward_axis,
            "detected_phone_yaw_axis_for_positive_ccw": yaw_axis,
            "selected_phone_to_base_rotation": list(
                self.phone_to_base_rotation),
            "detected_phone_to_base_rotation": list(detected_rotation),
            "phone_to_base_translation_m": list(self.phone_to_base_translation_m),
            "minimum_displacement_m": minimum_displacement_m,
            "minimum_yaw_rad": minimum_yaw_rad,
            "minimum_axis_dominance_ratio": AXIS_DOMINANCE_RATIO,
            "source_timestamp_available": self._source_timestamp_available,
            "limitation": (
                "TeleDex exposes no tracking-state or source timestamp in the "
                "installed API; this records local receive-time observations only."),
        }
        if any(
                abs(detected - configured) > 1e-6
                for detected, configured in zip(
                    detected_rotation, self.phone_to_base_rotation)):
            self.axis_validation["matches_selected_mount"] = False
            raise TeleDexMountMappingMismatch(
                "TeleDex axis validation does not match the selected "
                "phone-to-base mount; the detected axes and matrix were "
                "preserved without changing configuration",
                self.axis_validation)
        self.axis_validation["matches_selected_mount"] = True
        return dict(self.axis_validation)

    def poll_required(self, purpose: str) -> TeleDexPoseSample:
        """Poll one sample and fail closed for missing, malformed, or stale data."""
        sample = self.poll()
        self._ensure_trial_connection_continuity(purpose)
        if not sample.valid_sample:
            raise ValidationError("TeleDex " + purpose + " is invalid: " + sample.status_reason)
        age_s = self._sample_freshness_age(sample)
        if age_s is None:
            raise ValidationError("TeleDex " + purpose + " has no valid source-pose timestamp")
        if age_s >= self.stale_timeout_s:
            raise ValidationError(
                "TeleDex " + purpose + " source update is stale by %.3fs" % age_s)
        return sample

    def _ensure_trial_connection_continuity(self, purpose: str) -> None:
        diagnostics = self.stream_diagnostics()
        if diagnostics.connection_lost:
            raise ValidationError(
                "TeleDex " + purpose + " is invalid after transport disconnect")
        if (self._trial_connection_generation is not None and
                diagnostics.connection_generation != self._trial_connection_generation):
            raise ValidationError(
                "TeleDex " + purpose + " is invalid after transport reconnect")

    def poll(self) -> TeleDexPoseSample:
        """Preserve one accepted pose or one invalid attempt without silent loss."""
        receive_mono = self.monotonic()
        common = dict(
            receive_monotonic_s=receive_mono,
            receive_wall_time_iso=_iso_utc_now(),
            arkit_timestamp_s=None,
            x_m=None, y_m=None, z_m=None,
            r00=None, r01=None, r02=None, r10=None, r11=None,
            r12=None, r20=None, r21=None, r22=None)
        diagnostics = self.stream_diagnostics()
        common.update(
            websocket_message_count=diagnostics.websocket_message_count,
            websocket_message_age_s=diagnostics.websocket_message_age_s,
            latest_websocket_interarrival_s=(
                diagnostics.latest_websocket_interarrival_s),
            longest_websocket_interarrival_s=(
                diagnostics.longest_websocket_interarrival_s),
            connection_generation=diagnostics.connection_generation,
            connection_lost=diagnostics.connection_lost)
        if self._session is None:
            sample = TeleDexPoseSample(
                **common, valid_sample=False, status_reason="session is not started")
            self._trajectory.append(sample)
            return sample
        try:
            data = self._session.get_latest_data()
            position, rotation, source_timestamp = self._logger_module.validate_pose(data)
        except BaseException as error:
            sample = TeleDexPoseSample(
                **common, valid_sample=False,
                status_reason=type(error).__name__ + ": " + str(error))
            self._trajectory.append(sample)
            return sample
        # An upstream ARKit timestamp is stronger evidence of a new pose packet
        # than floating-point pose equality while the phone is stationary.
        fingerprint = tuple(position) + tuple(rotation)
        if source_timestamp is not None:
            fingerprint += (float(source_timestamp),)
        with self._stream_lock:
            raw_pose_changed = fingerprint != self._raw_pose_fingerprint
            if raw_pose_changed:
                if self._last_raw_pose_change_monotonic_s is not None:
                    interval_s = max(
                        0.0, receive_mono - self._last_raw_pose_change_monotonic_s)
                    self._longest_pose_change_interval_s = max(
                        interval_s, self._longest_pose_change_interval_s or 0.0)
                    if (not self._update_callback_available and
                            self._trial_receiving and
                            interval_s >= self.stale_timeout_s):
                        self._receive_discontinuities.append({
                            "basis": "raw_pose_change_fallback",
                            "gap_duration_s": interval_s,
                            "previous_receive_monotonic_s": (
                                self._last_raw_pose_change_monotonic_s),
                            "current_receive_monotonic_s": receive_mono,
                            "previous_source_timestamp_s": (
                                self._last_raw_pose_source_timestamp_s),
                            "current_source_timestamp_s": source_timestamp,
                            "previous_websocket_message_index": (
                                self._websocket_message_count),
                            "current_websocket_message_index": (
                                self._websocket_message_count),
                            "previous_pose_sample_index": self._raw_pose_generation,
                            "current_pose_sample_index": (
                                self._raw_pose_generation + 1),
                            "connection_generation": self._connection_generation,
                            "session_id": self._session_id,
                            "reconnect_observed": (
                                self._trial_connection_generation is not None and
                                self._connection_generation !=
                                self._trial_connection_generation),
                            "connection_lost": self._connection_lost,
                        })
                self._raw_pose_fingerprint = fingerprint
                self._raw_pose_generation += 1
                self._last_raw_pose_change_monotonic_s = receive_mono
                self._last_raw_pose_source_timestamp_s = source_timestamp
            raw_pose_generation = self._raw_pose_generation
            raw_pose_age_s = (
                None if self._last_raw_pose_change_monotonic_s is None else
                max(0.0, receive_mono - self._last_raw_pose_change_monotonic_s))
            self._source_timestamp_available |= source_timestamp is not None
        diagnostics = self.stream_diagnostics()
        common.update(
            websocket_message_count=diagnostics.websocket_message_count,
            websocket_message_age_s=diagnostics.websocket_message_age_s,
            latest_websocket_interarrival_s=(
                diagnostics.latest_websocket_interarrival_s),
            longest_websocket_interarrival_s=(
                diagnostics.longest_websocket_interarrival_s),
            connection_generation=diagnostics.connection_generation,
            connection_lost=diagnostics.connection_lost)
        values = dict(zip(
            ("r00", "r01", "r02", "r10", "r11", "r12", "r20", "r21", "r22"),
            rotation))
        common.update(values)
        common.update(
            arkit_timestamp_s=source_timestamp, x_m=position[0],
            y_m=position[1], z_m=position[2], valid_sample=True,
            raw_pose_generation=raw_pose_generation,
            raw_pose_age_s=raw_pose_age_s, raw_pose_changed=raw_pose_changed,
            status_reason="ok" if raw_pose_changed else "cached source pose")
        sample = TeleDexPoseSample(**common)
        if self._initial_pose is not None:
            sample = self._relative_sample(sample, position, rotation)
            previous = next(
                (value for value in reversed(self._trajectory)
                 if value.valid_sample and value.forward_displacement_m is not None),
                None)
            if previous is not None:
                self._path_length_m += math.hypot(
                    sample.forward_displacement_m - previous.forward_displacement_m,
                    sample.lateral_displacement_m - previous.lateral_displacement_m)
            else:
                self._path_length_m += math.hypot(
                    sample.forward_displacement_m, sample.lateral_displacement_m)
        self._trajectory.append(sample)
        return sample

    def _relative_sample(
            self, sample: TeleDexPoseSample, position, rotation) -> TeleDexPoseSample:
        initial = self._initial_pose
        reference_position = (initial.x_m, initial.y_m, initial.z_m)
        reference_rotation = (
            initial.r00, initial.r01, initial.r02,
            initial.r10, initial.r11, initial.r12,
            initial.r20, initial.r21, initial.r22)
        if self.phone_to_base_rotation is None:
            raise ValidationError("TeleDex phone-to-base mount is not configured")
        t_rel, r_rel = self._logger_module.relative_base_pose(
            reference_position, reference_rotation, position, rotation,
            self.phone_to_base_rotation, self.phone_to_base_translation_m)
        roll, pitch, yaw = self._logger_module.rpy_from_matrix(r_rel)
        self._last_yaw_unwrapped = self._logger_module.unwrap_angle(
            self._last_yaw_unwrapped, yaw)
        forward, lateral, planar, total = (
            self._logger_module.base_displacement_metrics(t_rel))
        values = asdict(sample)
        values.update(
            relative_x_m=t_rel[0], relative_y_m=t_rel[1],
            relative_z_m=t_rel[2], forward_displacement_m=forward,
            lateral_displacement_m=lateral, planar_displacement_m=planar,
            total_3d_displacement_m=total, yaw_rad=yaw,
            yaw_unwrapped_rad=self._last_yaw_unwrapped)
        return TeleDexPoseSample(**values)

    def finalize_after_stationarity(
            self, ros_timestamp_s: Optional[float]) -> TeleDexTrialResult:
        """Capture the final fresh physical reference after ROS stationarity."""
        if self._initial_pose is None:
            raise ValidationError("TeleDex reference was not captured before trial")
        final = self._wait_for_next_valid_pose(
            "final TeleDex reference after stationarity")
        self._ensure_trial_connection_continuity(
            "final TeleDex reference after stationarity")
        if final.forward_displacement_m is None or final.yaw_rad is None:
            raise ValidationError("final TeleDex reference was not relative to trial start")
        diagnostics = self.stream_diagnostics()
        if diagnostics.receive_discontinuities:
            event = diagnostics.receive_discontinuities[-1]
            raise ValidationError(
                "TeleDex trajectory has a receive discontinuity: "
                "gap_duration_s={gap_duration_s:.6f}; "
                "previous_receive_monotonic_s={previous_receive_monotonic_s:.6f}; "
                "current_receive_monotonic_s={current_receive_monotonic_s:.6f}; "
                "previous_websocket_message_index={previous_websocket_message_index}; "
                "current_websocket_message_index={current_websocket_message_index}; "
                "connection_generation={connection_generation}; "
                "reconnect_observed={reconnect_observed}; "
                "connection_lost={connection_lost}; "
                "session_id={session_id}; "
                "previous_source_timestamp_s={previous_source_timestamp_s}; "
                "current_source_timestamp_s={current_source_timestamp_s}".format(**event))
        return TeleDexTrialResult(
            initial_pose=self._initial_pose,
            final_pose=final,
            total_planar_path_length_m=self._path_length_m,
            net_forward_displacement_m=final.forward_displacement_m,
            lateral_displacement_m=final.lateral_displacement_m,
            final_yaw_rad=final.yaw_rad,
            final_yaw_unwrapped_rad=final.yaw_unwrapped_rad,
            source_timestamp_available=self._source_timestamp_available,
            phone_to_base_rotation=self.phone_to_base_rotation,
            phone_to_base_translation_m=self.phone_to_base_translation_m,
            ros_initial_timestamp_s=self._ros_initial_timestamp_s,
            ros_final_timestamp_s=ros_timestamp_s,
            discontinuities_observed=len(diagnostics.receive_discontinuities),
            stream_diagnostics=asdict(diagnostics))
