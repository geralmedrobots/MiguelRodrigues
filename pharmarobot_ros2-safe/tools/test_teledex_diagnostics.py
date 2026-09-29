"""Offline tests for the standalone TeleDex diagnostics script."""

import importlib.util
import math
from pathlib import Path
import sys

import pytest


SPEC = importlib.util.spec_from_file_location(
    "teledex_diagnostics", Path(__file__).with_name("teledex_diagnostics.py"))
diagnostics_module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = diagnostics_module
SPEC.loader.exec_module(diagnostics_module)
TeleDexDiagnostics = diagnostics_module.TeleDexDiagnostics
stationary_result = diagnostics_module.stationary_result

IDENTITY = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)


class Clock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value


class Logger:
    def validate_pose(self, data):
        if not isinstance(data, dict):
            raise ValueError("sample is not a mapping")
        return tuple(data["position"]), tuple(data["rotation"]), data.get("timestamp")

    def rotation_from_rpy(self, _roll, _pitch, _yaw):
        return IDENTITY

    def validate_rotation_matrix(self, rotation):
        return tuple(rotation)

    def compose_pose(self, position, rotation, translation, mount_rotation):
        return (tuple(position[index] + translation[index] for index in range(3)),
                tuple(mount_rotation))

    def rpy_from_matrix(self, rotation):
        return 0.0, 0.0, math.atan2(rotation[3], rotation[0])

    def unwrap_angle(self, previous, current):
        if previous is None:
            return current
        return previous + (current - previous + math.pi) % (2 * math.pi) - math.pi


class Session:
    def __init__(self):
        self.connect_callbacks = []
        self.disconnect_callbacks = []
        self.update_callbacks = []
        self.stopped = False

    def on_connect(self, callback):
        self.connect_callbacks.append(callback)

    def on_disconnect(self, callback):
        self.disconnect_callbacks.append(callback)

    def on_update(self, callback):
        self.update_callbacks.append(callback)

    def start(self):
        for callback in self.connect_callbacks:
            callback(self)

    def stop(self):
        self.stopped = True

    def update(self, data):
        for callback in self.update_callbacks:
            callback(self, data)

    def disconnect(self):
        for callback in self.disconnect_callbacks:
            callback(self)

    def reconnect(self):
        for callback in self.connect_callbacks:
            callback(self)


def build(clock):
    session = Session()
    diagnostics = TeleDexDiagnostics(
        Logger(), lambda: session, stale_timeout_s=0.5, monotonic=clock)
    diagnostics.start()
    return diagnostics, session


def pose(x=0.0, timestamp=None):
    data = {"position": (x, 0.0, 0.0), "rotation": IDENTITY}
    if timestamp is not None:
        data["timestamp"] = timestamp
    return data


def test_healthy_60hz_stream_reports_rate_and_corrected_base_position():
    clock = Clock()
    diagnostics, session = build(clock)
    for index in range(61):
        clock.value = index / 60.0
        session.update(pose(index / 60.0, index))

    snapshot = diagnostics.snapshot()

    assert snapshot["callback_count"] == 61
    assert snapshot["callback_rate_hz"] == pytest.approx(60.0)
    assert snapshot["median_interarrival_s"] == pytest.approx(1 / 60.0)
    assert snapshot["base_link_position_m"] == pytest.approx((1.1425, -0.1, -0.0075))
    assert snapshot["valid"]


def test_stationary_identical_callbacks_are_fresh_but_counted_as_repeated():
    clock = Clock()
    diagnostics, session = build(clock)
    session.update(pose())
    clock.value = 0.1
    session.update(pose())

    snapshot = diagnostics.snapshot()

    assert snapshot["valid"]
    assert snapshot["repeated_pose_count"] == 1
    assert snapshot["longest_repeated_pose_run"] == 1


def test_cached_reads_without_callback_fail_closed_as_stale():
    clock = Clock()
    diagnostics, session = build(clock)
    session.update(pose())
    clock.value = 0.5

    snapshot = diagnostics.snapshot()

    assert not snapshot["valid"]
    assert "stale" in snapshot["fail_closed_reason"]
    assert snapshot["stale_event_count"] == 1


def test_short_jitter_is_accepted_but_receive_gap_is_logged():
    clock = Clock()
    diagnostics, session = build(clock)
    session.update(pose())
    clock.value = 0.059
    session.update(pose())
    assert diagnostics.snapshot()["valid"]
    clock.value = 0.6
    session.update(pose(0.1))

    events = diagnostics.snapshot()["events"]

    assert any(event["event"] == "receive_gap" for event in events)


def test_disconnect_reconnect_and_session_generation_are_reported():
    clock = Clock()
    diagnostics, session = build(clock)
    session.disconnect()
    disconnected = diagnostics.snapshot()
    session.reconnect()
    reconnected = diagnostics.snapshot()

    assert not disconnected["valid"]
    assert reconnected["connection_generation"] == 2
    assert reconnected["reconnect_count"] == 1
    assert reconnected["disconnect_count"] == 1


def test_stale_source_timestamp_fails_stationary_result():
    clock = Clock()
    diagnostics, session = build(clock)
    session.update(pose(timestamp=1.0))
    clock.value = 0.01
    session.update(pose(0.1, timestamp=1.0))

    result = stationary_result(diagnostics.snapshot(), 0.5)

    assert "source timestamp did not advance" in result["failure_reasons"]


def test_invalid_pose_fails_stationary_result_even_with_healthy_callbacks():
    clock = Clock()
    diagnostics, session = build(clock)
    session.update({"position": (0.0, 0.0, 0.0)})

    result = stationary_result(diagnostics.snapshot(), 0.5)

    assert not result["pass"]
    assert any("KeyError" in reason for reason in result["failure_reasons"])


def test_valid_pose_after_stale_recovery_event_is_emitted():
    clock = Clock()
    diagnostics, session = build(clock)
    session.update(pose())
    clock.value = 0.5
    diagnostics.snapshot()
    clock.value = 0.51
    session.update(pose(0.1))

    events = diagnostics.snapshot()["events"]

    assert any(event["event"] == "valid_pose_after_stale" for event in events)
