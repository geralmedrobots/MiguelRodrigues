#!/usr/bin/env python3
"""Standalone, read-only live diagnostics for a TeleDex/ARKit session.

This script intentionally imports neither ROS nor ``odometry_validation``.  It
opens the normal inbound TeleDex listener, observes received callbacks, and
never publishes a command or constructs a robot trial.
"""

import argparse
from dataclasses import asdict, dataclass
import importlib
import importlib.util
import json
import math
from pathlib import Path
import time
from threading import Lock
from typing import Callable, Dict, List, Optional, Sequence, Tuple


DEFAULT_RPY_DEG = (-90.0, 0.0, 0.0)
DEFAULT_TRANSLATION_M = (0.1425, -0.1000, -0.0075)
DEFAULT_STALE_TIMEOUT_S = 0.5


def _percentile(values: Sequence[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


def _optional_finite(data: object, keys: Sequence[str]) -> Optional[float]:
    if not isinstance(data, dict):
        return None
    for key in keys:
        value = data.get(key)
        try:
            value = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            return value
    return None


def _source_sequence(data: object) -> Optional[str]:
    if not isinstance(data, dict):
        return None
    for key in ("sequence", "seq", "frame_id"):
        if key in data:
            return str(data[key])
    return None


def load_logger(reference_root: Path):
    """Load only the standalone transform helper from an explicit directory."""
    logger_path = reference_root.expanduser() / "teledex_logger.py"
    if not logger_path.is_file():
        raise RuntimeError("TeleDex logger is unavailable: " + str(logger_path))
    spec = importlib.util.spec_from_file_location("teledex_diagnostics_logger", logger_path)
    if spec is None or spec.loader is None:
        raise RuntimeError("unable to load TeleDex logger: " + str(logger_path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    required = ("validate_pose", "compose_pose", "rotation_from_rpy",
                "validate_rotation_matrix", "rpy_from_matrix", "unwrap_angle")
    missing = [name for name in required if not callable(getattr(module, name, None))]
    if missing:
        raise RuntimeError("TeleDex logger API is incomplete: " + ", ".join(missing))
    return module


def load_session_factory() -> Callable[[], object]:
    try:
        return importlib.import_module("teledex").Session
    except (ImportError, AttributeError) as error:
        raise RuntimeError("installed TeleDex Session API is unavailable: " + str(error)) from error


@dataclass(frozen=True)
class DiagnosticEvent:
    event: str
    monotonic_s: float
    details: Dict[str, object]


class TeleDexDiagnostics:
    """Thread-safe callback observer with no ROS or actuator integration."""

    def __init__(self, logger, session_factory: Callable[[], object],
                 mount_rpy_deg: Tuple[float, float, float] = DEFAULT_RPY_DEG,
                 mount_translation_m: Tuple[float, float, float] = DEFAULT_TRANSLATION_M,
                 stale_timeout_s: float = DEFAULT_STALE_TIMEOUT_S,
                 monotonic: Callable[[], float] = time.monotonic,
                 event_sink: Optional[Callable[[DiagnosticEvent], None]] = None):
        if not math.isfinite(stale_timeout_s) or stale_timeout_s <= 0.0:
            raise ValueError("stale timeout must be finite and positive")
        if len(mount_rpy_deg) != 3 or len(mount_translation_m) != 3:
            raise ValueError("mount RPY and translation must each have three values")
        if not all(math.isfinite(value) for value in (*mount_rpy_deg, *mount_translation_m)):
            raise ValueError("mount values must be finite")
        self.logger = logger
        self.session_factory = session_factory
        self.stale_timeout_s = stale_timeout_s
        self.monotonic = monotonic
        self._event_sink = event_sink
        self.mount_rotation = logger.validate_rotation_matrix(logger.rotation_from_rpy(
            *(math.radians(value) for value in mount_rpy_deg)))
        self.mount_translation_m = tuple(mount_translation_m)
        self._lock = Lock()
        self._session = None
        self._session_id: Optional[str] = None
        self._connected = False
        self._connection_generation = 0
        self._disconnect_count = 0
        self._reconnect_count = 0
        self._callback_count = 0
        self._callback_times: List[float] = []
        self._last_callback_s: Optional[float] = None
        self._last_fingerprint = None
        self._repeated_pose_count = 0
        self._current_repeated_run = 0
        self._longest_repeated_run = 0
        self._last_source_timestamp: Optional[float] = None
        self._last_source_sequence: Optional[str] = None
        self._source_timestamp_stale = False
        self._tracking_provenance_available = False
        self._raw_position = None
        self._base_position = None
        self._wrapped_yaw_rad: Optional[float] = None
        self._unwrapped_yaw_rad: Optional[float] = None
        self._initial_base_position = None
        self._initial_unwrapped_yaw_rad: Optional[float] = None
        self._max_position_drift_m = 0.0
        self._max_yaw_drift_rad = 0.0
        self._invalid_reason: Optional[str] = "no valid TeleDex pose received"
        self._stale_active = False
        self._stale_event_count = 0
        self._events: List[DiagnosticEvent] = []

    def _event(self, event: str, **details: object) -> None:
        entry = DiagnosticEvent(event, self.monotonic(), details)
        self._events.append(entry)
        if self._event_sink is not None:
            self._event_sink(entry)

    def start(self) -> None:
        self._session = self.session_factory()
        configured = getattr(self._session, "session_id", None)
        self._session_id = (str(configured) if configured is not None else
                            type(self._session).__name__ + ":" + format(id(self._session), "x"))
        for name, callback in (("on_connect", self._on_connect),
                               ("on_disconnect", self._on_disconnect),
                               ("on_update", self._on_update)):
            registrar = getattr(self._session, name, None)
            if callable(registrar):
                registrar(callback)
        try:
            self._session.start()
        except BaseException:
            self.stop()
            raise

    def stop(self) -> None:
        session, self._session = self._session, None
        if session is not None:
            session.stop()

    def _on_connect(self, _session: object) -> None:
        with self._lock:
            was_connected = self._connected
            self._connected = True
            self._connection_generation += 1
            if was_connected or self._connection_generation > 1:
                self._reconnect_count += 1
                self._event("reconnect", connection_generation=self._connection_generation,
                            session_id=self._session_id)
            else:
                self._event("initial_connection", connection_generation=1,
                            session_id=self._session_id)

    def _on_disconnect(self, _session: object) -> None:
        with self._lock:
            self._connected = False
            self._disconnect_count += 1
            self._invalid_reason = "TeleDex transport is disconnected"
            self._event("disconnect", connection_generation=self._connection_generation,
                        session_id=self._session_id)

    def _on_update(self, _session: object, data: object) -> None:
        now = self.monotonic()
        with self._lock:
            prior_callback = self._last_callback_s
            if prior_callback is not None and now - prior_callback >= self.stale_timeout_s:
                self._event("receive_gap", gap_duration_s=now - prior_callback,
                            previous_receive_monotonic_s=prior_callback,
                            current_receive_monotonic_s=now,
                            connection_generation=self._connection_generation,
                            session_id=self._session_id)
            self._callback_count += 1
            self._callback_times.append(now)
            self._last_callback_s = now
            source_timestamp = _optional_finite(data, ("timestamp", "arkit_timestamp", "time"))
            source_sequence = _source_sequence(data)
            self._tracking_provenance_available |= (
                source_timestamp is not None or source_sequence is not None or
                isinstance(data, dict) and "tracking_state" in data)
            stale_source = (source_timestamp is not None and
                            self._last_source_timestamp is not None and
                            source_timestamp <= self._last_source_timestamp)
            if stale_source:
                self._source_timestamp_stale = True
                self._event("stale_source_timestamp", source_timestamp_s=source_timestamp,
                            previous_source_timestamp_s=self._last_source_timestamp)
            if source_timestamp is not None:
                self._last_source_timestamp = source_timestamp
            if source_sequence is not None:
                self._last_source_sequence = source_sequence
            try:
                position, rotation, _ignored = self.logger.validate_pose(data)
                base_position, base_rotation = self.logger.compose_pose(
                    position, rotation, self.mount_translation_m, self.mount_rotation)
                _roll, _pitch, yaw = self.logger.rpy_from_matrix(base_rotation)
            except BaseException as error:
                self._invalid_reason = type(error).__name__ + ": " + str(error)
                self._event("invalid_pose", reason=self._invalid_reason)
                return
            fingerprint = tuple(position) + tuple(rotation) + (
                (source_timestamp,) if source_timestamp is not None else ())
            if fingerprint == self._last_fingerprint:
                self._repeated_pose_count += 1
                self._current_repeated_run += 1
                self._longest_repeated_run = max(self._longest_repeated_run, self._current_repeated_run)
                self._event("cached_pose", repeated_run=self._current_repeated_run)
            else:
                self._current_repeated_run = 0
                self._last_fingerprint = fingerprint
            self._raw_position = tuple(position)
            self._base_position = tuple(base_position)
            self._wrapped_yaw_rad = yaw
            self._unwrapped_yaw_rad = self.logger.unwrap_angle(self._unwrapped_yaw_rad, yaw)
            if self._initial_base_position is None:
                self._initial_base_position = self._base_position
                self._initial_unwrapped_yaw_rad = self._unwrapped_yaw_rad
            self._max_position_drift_m = max(self._max_position_drift_m, math.sqrt(sum(
                (self._base_position[index] - self._initial_base_position[index]) ** 2
                for index in range(3))))
            self._max_yaw_drift_rad = max(self._max_yaw_drift_rad, abs(
                self._unwrapped_yaw_rad - self._initial_unwrapped_yaw_rad))
            if self._stale_active:
                self._event("valid_pose_after_stale", source_timestamp_s=source_timestamp)
            self._invalid_reason = None
            self._stale_active = False

    def snapshot(self) -> Dict[str, object]:
        with self._lock:
            now = self.monotonic()
            age = None if self._last_callback_s is None else max(0.0, now - self._last_callback_s)
            if age is not None and age >= self.stale_timeout_s and not self._stale_active:
                self._stale_active = True
                self._stale_event_count += 1
                self._invalid_reason = "TeleDex WebSocket update is stale by %.3fs" % age
                self._event("stale_update", age_s=age, threshold_s=self.stale_timeout_s)
            gaps = [second - first for first, second in zip(self._callback_times, self._callback_times[1:])]
            duration = (None if len(self._callback_times) < 2 else
                        self._callback_times[-1] - self._callback_times[0])
            return {
                "raw_phone_position_m": self._raw_position,
                "base_link_position_m": self._base_position,
                "wrapped_yaw_rad": self._wrapped_yaw_rad,
                "unwrapped_yaw_rad": self._unwrapped_yaw_rad,
                "callback_count": self._callback_count,
                "callback_rate_hz": None if not duration or duration <= 0.0 else
                (self._callback_count - 1) / duration,
                "current_interarrival_s": gaps[-1] if gaps else None,
                "median_interarrival_s": _percentile(gaps, 0.5),
                "p95_interarrival_s": _percentile(gaps, 0.95),
                "max_interarrival_s": max(gaps) if gaps else None,
                "time_since_last_update_s": age,
                "repeated_pose_count": self._repeated_pose_count,
                "longest_repeated_pose_run": self._longest_repeated_run,
                "connection_generation": self._connection_generation,
                "session_id": self._session_id,
                "connected": self._connected,
                "disconnect_count": self._disconnect_count,
                "reconnect_count": self._reconnect_count,
                "source_timestamp_s": self._last_source_timestamp,
                "source_sequence": self._last_source_sequence,
                "source_timestamp_stale": self._source_timestamp_stale,
                "arkit_tracking_provenance_available": self._tracking_provenance_available,
                "position_drift_m": self._max_position_drift_m,
                "yaw_drift_rad": self._max_yaw_drift_rad,
                "stale_event_count": self._stale_event_count,
                "valid": self._connected and self._invalid_reason is None and not self._stale_active,
                "fail_closed_reason": self._invalid_reason,
                "events": [asdict(event) for event in self._events],
            }


def stationary_result(snapshot: Dict[str, object], stale_timeout_s: float) -> Dict[str, object]:
    """Apply only transport/provenance thresholds; drift is reported, not invented."""
    reasons = []
    if snapshot["callback_count"] == 0:
        reasons.append("no callbacks received")
    if snapshot["max_interarrival_s"] is not None and snapshot["max_interarrival_s"] >= stale_timeout_s:
        reasons.append("receive gap exceeded stale timeout")
    if snapshot["disconnect_count"]:
        reasons.append("transport disconnected")
    if snapshot["source_timestamp_stale"]:
        reasons.append("source timestamp did not advance")
    if snapshot["stale_event_count"]:
        reasons.append("stale update observed")
    if not snapshot["valid"]:
        reasons.append(snapshot["fail_closed_reason"] or "stream is invalid")
    return {"mode": "stationary", "pass": not reasons, "failure_reasons": reasons,
            "stale_timeout_s": stale_timeout_s, "summary": snapshot}


def _print_live(snapshot: Dict[str, object]) -> None:
    fields = ("raw_phone_position_m", "base_link_position_m", "wrapped_yaw_rad",
              "unwrapped_yaw_rad", "callback_rate_hz", "current_interarrival_s",
              "median_interarrival_s", "p95_interarrival_s", "max_interarrival_s",
              "time_since_last_update_s", "repeated_pose_count", "longest_repeated_pose_run",
              "connected", "valid", "connection_generation", "session_id",
              "disconnect_count", "reconnect_count", "source_timestamp_s",
              "source_sequence", "arkit_tracking_provenance_available", "fail_closed_reason")
    print(" ".join(name + "=" + str(snapshot[name]) for name in fields), flush=True)


def _print_event(event: DiagnosticEvent) -> None:
    print("event=" + event.event + " monotonic_s=" + format(event.monotonic_s, ".6f") +
          " details=" + json.dumps(event.details, sort_keys=True), flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Standalone read-only TeleDex diagnostics")
    parser.add_argument("--duration", type=float, default=None,
                        help="seconds for stationary summary; omit for live display")
    parser.add_argument("--display-rate-hz", type=float, default=4.0)
    parser.add_argument("--reference-root", type=Path, default=Path("/opt/teledex_reference"))
    parser.add_argument("--phone-to-base-rpy-deg", type=float, nargs=3, default=DEFAULT_RPY_DEG)
    parser.add_argument("--phone-to-base-translation-m", type=float, nargs=3,
                        default=DEFAULT_TRANSLATION_M)
    parser.add_argument("--stale-timeout-s", type=float, default=DEFAULT_STALE_TIMEOUT_S)
    parser.add_argument("--json-output", type=Path,
                        help="write a new JSON summary after --duration mode")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0.0):
        raise SystemExit("--duration must be finite and positive")
    if not math.isfinite(args.display_rate_hz) or args.display_rate_hz <= 0.0:
        raise SystemExit("--display-rate-hz must be finite and positive")
    logger = load_logger(args.reference_root)
    diagnostics = TeleDexDiagnostics(
        logger, load_session_factory(), tuple(args.phone_to_base_rpy_deg),
        tuple(args.phone_to_base_translation_m), args.stale_timeout_s,
        event_sink=_print_event)
    diagnostics.start()
    try:
        start = time.monotonic()
        while args.duration is None or time.monotonic() - start < args.duration:
            snapshot = diagnostics.snapshot()
            if args.duration is None:
                _print_live(snapshot)
            time.sleep(1.0 / args.display_rate_hz)
        snapshot = diagnostics.snapshot()
        result = stationary_result(snapshot, args.stale_timeout_s) if args.duration is not None else snapshot
        if args.duration is not None:
            print(json.dumps(result, indent=2, sort_keys=True))
            if args.json_output is not None:
                with args.json_output.open("x", encoding="utf-8") as stream:
                    json.dump(result, stream, indent=2, sort_keys=True)
                    stream.write("\n")
        return 0 if (result["pass"] if args.duration is not None else result["valid"]) else 2
    finally:
        diagnostics.stop()


if __name__ == "__main__":
    raise SystemExit(main())
