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

"""Hardware-independent TeleDex adapter tests."""

import math
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from odometry_validation.core import ValidationError
import odometry_validation.teledex_adapter as adapter_module
from odometry_validation.teledex_adapter import load_standalone_logger
from odometry_validation.teledex_adapter import load_teledex_session_factory
from odometry_validation.teledex_adapter import TeleDexReferenceAdapter
from odometry_validation.teledex_adapter import TeleDexStreamDiagnostics
from odometry_validation.teledex_adapter import TeleDexMountMappingMismatch
from odometry_validation.teledex_stream_diagnostic import collect_stream_diagnostics
from odometry_validation.teledex_adapter import (
    phone_to_base_translation_from_base_phone_position)


IDENTITY = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
OBSERVED_PHONE_TO_BASE = (
    1.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    0.0, -1.0, 0.0)
INITIAL_WORLD_PHONE = (
    1.0, 0.0, 0.0,
    0.0, 0.0, -1.0,
    0.0, 1.0, 0.0)
REFERENCE_ROOT = Path("/home/medrobots/teledex_reference")


@pytest.fixture(autouse=True)
def standalone_logger_double(monkeypatch):
    """Mock the separately deployed standalone logger for hermetic tests."""
    def matrix_multiply(left, right):
        return tuple(sum(left[row * 3 + k] * right[k * 3 + column]
                         for k in range(3))
                     for row in range(3) for column in range(3))

    def transpose(rotation):
        return tuple(rotation[column * 3 + row]
                     for row in range(3) for column in range(3))

    def matrix_vector(rotation, vector):
        return tuple(sum(rotation[row * 3 + column] * vector[column]
                         for column in range(3)) for row in range(3))

    def validate_pose(data):
        if not isinstance(data, dict):
            raise ValueError("sample is not a mapping")
        position = tuple(float(value) for value in data["position"])
        rotation = tuple(float(value) for value in data["rotation"])
        if len(position) != 3 or len(rotation) != 9:
            raise ValueError("malformed pose")
        if not all(math.isfinite(value) for value in position + rotation):
            raise ValueError("nonfinite pose")
        timestamp = data.get("timestamp")
        return position, rotation, timestamp

    def relative_pose(reference_position, reference_rotation, position, rotation):
        inverse = transpose(reference_rotation)
        return (
            matrix_vector(inverse, tuple(
                position[index] - reference_position[index] for index in range(3))),
            matrix_multiply(inverse, rotation))

    def rotation_from_rpy(roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return (
            cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
            -sp, cp * sr, cp * cr)

    def relative_base_pose(reference_position, reference_rotation,
                           position, rotation, mount_rotation,
                           mount_translation):
        reference_base_position = tuple(
            reference_position[index] +
            matrix_vector(reference_rotation, mount_translation)[index]
            for index in range(3))
        base_position = tuple(
            position[index] + matrix_vector(rotation, mount_translation)[index]
            for index in range(3))
        return relative_pose(
            reference_base_position,
            matrix_multiply(reference_rotation, mount_rotation),
            base_position, matrix_multiply(rotation, mount_rotation))

    def rpy_from_matrix(rotation):
        return 0.0, 0.0, math.atan2(rotation[3], rotation[0])

    def unwrap_angle(previous, current):
        if previous is None:
            return current
        delta = (current - previous + math.pi) % (2.0 * math.pi) - math.pi
        return previous + delta

    def base_displacement_metrics(translation):
        return (
            translation[0], translation[1],
            math.hypot(translation[0], translation[1]),
            math.sqrt(sum(value * value for value in translation)))

    def rotation_vector_from_matrix(rotation):
        angle = math.acos(max(-1.0, min(
            1.0, (rotation[0] + rotation[4] + rotation[8] - 1.0) / 2.0)))
        if angle < 1e-9:
            return (0.0, 0.0, 0.0)
        scale = angle / (2.0 * math.sin(angle))
        return (scale * (rotation[7] - rotation[5]),
                scale * (rotation[2] - rotation[6]),
                scale * (rotation[3] - rotation[1]))

    module = SimpleNamespace(
        base_displacement_metrics=base_displacement_metrics,
        validate_pose=validate_pose, relative_pose=relative_pose,
        relative_base_pose=relative_base_pose,
        rotation_from_rpy=rotation_from_rpy,
        rotation_vector_from_matrix=rotation_vector_from_matrix,
        validate_rotation_matrix=lambda rotation: tuple(rotation),
        rpy_from_matrix=rpy_from_matrix, unwrap_angle=unwrap_angle,
    )
    monkeypatch.setattr(
        "odometry_validation.teledex_adapter.load_standalone_logger",
        lambda _root: module)


class FakeSession:
    def __init__(self, samples, auto_updates=True):
        self.samples = iter(samples)
        self.auto_updates = auto_updates
        self.started = False
        self.stopped = False
        self._connect_callbacks = []
        self._disconnect_callbacks = []
        self._update_callbacks = []

    def on_connect(self, callback):
        self._connect_callbacks.append(callback)

    def on_disconnect(self, callback):
        self._disconnect_callbacks.append(callback)

    def on_update(self, callback):
        self._update_callbacks.append(callback)

    def start(self):
        self.started = True
        for callback in self._connect_callbacks:
            callback(self)

    def get_latest_data(self):
        value = next(self.samples)
        if isinstance(value, BaseException):
            raise value
        if self.auto_updates:
            self.update(value)
        return value

    def stop(self):
        self.stopped = True

    def disconnect(self):
        for callback in self._disconnect_callbacks:
            callback(self)

    def reconnect(self):
        for callback in self._connect_callbacks:
            callback(self)

    def update(self, data):
        for callback in self._update_callbacks:
            callback(self, data)


class Clock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value


def build_adapter(samples, clock=None, auto_updates=True):
    session = FakeSession(samples, auto_updates=auto_updates)
    clock = clock or Clock()
    return (
        TeleDexReferenceAdapter(
            REFERENCE_ROOT, (-90.0, 0.0, 0.0), (0.0, 0.0, 0.0),
            0.5, lambda: session,
            monotonic=clock, readiness_timeout_s=0.05,
            sleep=lambda duration: setattr(clock, "value", clock.value + duration)),
        session)


def test_measured_base_phone_position_is_inverted_into_cli_translation():
    """The CLI vector is base origin in phone axes, not phone origin in base axes."""
    translation = phone_to_base_translation_from_base_phone_position(
        (-0.1425, -0.0075, 0.40), (-90.0, 0.0, 0.0))
    assert translation == pytest.approx((0.1425, -0.40, -0.0075))


def test_rx_minus_90_maps_robot_forward_and_left_without_axis_swap():
    # R_phone_base = Rx(-90): base +X/+Y/+Z are phone +X/-Z/+Y.
    mount = (1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0)
    assert tuple(sum(mount[row * 3 + col] * (1.0, 0.0, 0.0)[col]
                     for col in range(3)) for row in range(3)) == (1.0, 0.0, 0.0)
    assert tuple(sum(mount[row * 3 + col] * (0.0, 1.0, 0.0)[col]
                     for col in range(3)) for row in range(3)) == (0.0, 0.0, -1.0)


def test_inverse_translation_round_trip_with_nonzero_lever_arm():
    mount = (1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0)
    base_phone = (-0.1425, -0.0075, 0.40)
    phone_base = phone_to_base_translation_from_base_phone_position(
        base_phone, (-90.0, 0.0, 0.0))
    # T_base_phone = inverse(T_phone_base): p_base_phone = -R_base_phone*t_phone_base.
    rotated = tuple(sum(mount[column * 3 + row] * phone_base[column]
                        for column in range(3)) for row in range(3))
    assert tuple(-value for value in rotated) == pytest.approx(base_phone)


def test_adapter_collects_relative_trajectory_and_path_length():
    clock = Clock()
    adapter, session = build_adapter((
        {"position": [0, 0, 0], "rotation": INITIAL_WORLD_PHONE},
        {"position": [0.2, 0, 0], "rotation": INITIAL_WORLD_PHONE},
        {"position": [0.5, 0, 0], "rotation": INITIAL_WORLD_PHONE,
         "timestamp": 8.0},
    ), clock)
    adapter.start()
    initial = adapter.begin_trial(100.0)
    assert session.started and initial.valid_sample
    clock.value = 0.1
    adapter.poll_required("trajectory")
    clock.value = 0.2
    result = adapter.finalize_after_stationarity(101.0)

    assert result.total_planar_path_length_m == pytest.approx(0.5)
    assert result.net_forward_displacement_m == pytest.approx(0.5)
    assert result.lateral_displacement_m == pytest.approx(0.0)
    assert result.source_timestamp_available
    assert result.ros_initial_timestamp_s == 100.0
    assert result.ros_final_timestamp_s == 101.0
    assert len(adapter.trajectory) == 3
    adapter.stop()
    assert session.stopped


def test_adapter_fails_closed_for_missing_or_malformed_reference():
    adapter, _session = build_adapter((None,))
    adapter.start()
    with pytest.raises(ValidationError, match="initial TeleDex reference unavailable"):
        adapter.begin_trial(1.0)


def test_adapter_waits_for_delayed_valid_first_sample():
    adapter, _session = build_adapter((None, {"position": [0, 0, 0], "rotation": IDENTITY}))
    adapter.start()
    assert adapter.begin_trial(1.0).valid_sample


def test_adapter_recovers_after_startup_disconnect():
    adapter, _session = build_adapter((ConnectionError("phone disconnected"),
                                       {"position": [0, 0, 0], "rotation": IDENTITY}))
    adapter.start()
    assert adapter.begin_trial(1.0).valid_sample


def test_adapter_rejects_stale_first_sample_before_timeout():
    clock = Clock()
    pose = {"position": [0, 0, 0], "rotation": IDENTITY}
    adapter, session = build_adapter(
        ({"position": [0, 0, 0], "rotation": IDENTITY},
         {"position": [0, 0, 0], "rotation": IDENTITY}), clock,
        auto_updates=False)
    adapter.start()
    session.update(pose)
    adapter.poll()  # A session cache already held this raw pose before the trial.
    clock.value = 1.0
    with pytest.raises(ValidationError, match="unavailable after"):
        adapter.begin_trial(1.0)


def test_adapter_rejects_cached_pose_during_motion_after_source_age_limit():
    """Repeated get_latest_data cache reads must not refresh ARKit freshness."""
    clock = Clock()
    pose = {"position": [0, 0, 0], "rotation": IDENTITY}
    adapter, session = build_adapter((pose, pose), clock)
    adapter.start()
    adapter.begin_trial(1.0)
    session.auto_updates = False
    clock.value = 0.51

    with pytest.raises(ValidationError, match="source update is stale"):
        adapter.poll_required("trajectory")


def test_adapter_records_websocket_gaps_separately_from_polling_and_pose_changes():
    clock = Clock()
    first = {"position": [0, 0, 0], "rotation": IDENTITY}
    second = {"position": [0.1, 0, 0], "rotation": IDENTITY}
    adapter, session = build_adapter((first, second), clock)
    adapter.start()
    adapter.begin_trial(1.0)
    session.auto_updates = False
    clock.value = 0.2
    session.update(second)
    adapter.poll_required("trajectory")

    diagnostics = adapter.stream_diagnostics()

    assert diagnostics.update_callback_available
    assert diagnostics.websocket_message_count == 2
    assert diagnostics.latest_websocket_interarrival_s == pytest.approx(0.2)
    assert diagnostics.longest_websocket_interarrival_s == pytest.approx(0.2)
    assert diagnostics.pose_change_count == 2


def test_adapter_accepts_normal_receive_jitter_with_unchanged_stationary_pose():
    """Callback receipt, rather than coordinate jitter, defines stream freshness."""
    clock = Clock()
    pose = {"position": [0, 0, 0], "rotation": IDENTITY}
    adapter, session = build_adapter((pose, pose), clock)
    adapter.start()
    adapter.begin_trial(1.0)
    session.auto_updates = False
    clock.value = 0.059
    session.update(pose)

    adapter.poll_required("normal receive jitter")

    diagnostics = adapter.stream_diagnostics()
    assert diagnostics.latest_websocket_interarrival_s == pytest.approx(0.059)
    assert diagnostics.receive_discontinuities == ()


def test_pretrial_stationary_coordinate_gap_does_not_invalidate_new_trial():
    """The first post-boundary packet is a reference, not a trial interruption."""
    clock = Clock()
    before_trial = {"position": [0, 0, 0], "rotation": IDENTITY}
    reference = {"position": [0.1, 0, 0], "rotation": IDENTITY}
    moved = {"position": [0.2, 0, 0], "rotation": IDENTITY}
    final = {"position": [0.3, 0, 0], "rotation": IDENTITY}
    adapter, _session = build_adapter((before_trial, reference, moved, final), clock)
    adapter.start()
    adapter.poll()
    clock.value = 46.986
    adapter.begin_trial(1.0)
    clock.value = 47.0
    adapter.poll_required("trajectory")
    result = adapter.finalize_after_stationarity(2.0)

    assert result.discontinuities_observed == 0
    assert result.stream_diagnostics["longest_pose_change_interval_s"] == pytest.approx(
        46.986)


def test_adapter_allows_short_stall_then_records_a_long_gap_after_recovery():
    clock = Clock()
    first = {"position": [0, 0, 0], "rotation": IDENTITY, "timestamp": 1.0}
    changed = {"position": [0.1, 0, 0], "rotation": IDENTITY, "timestamp": 2.0}
    adapter, session = build_adapter((first, first, changed), clock)
    adapter.start()
    adapter.begin_trial(1.0)
    session.auto_updates = False
    clock.value = 0.49
    adapter.poll_required("temporary stall")
    clock.value = 0.51
    session.update(changed)
    adapter.poll_required("recovered stream")

    assert adapter.stream_diagnostics().longest_pose_change_interval_s == pytest.approx(0.51)


def test_adapter_rejects_prolonged_stall_even_if_the_stream_later_recovers():
    """A trial that crosses the stale limit remains invalid, rather than resuming."""
    clock = Clock()
    first = {"position": [0, 0, 0], "rotation": IDENTITY, "timestamp": 1.0}
    changed = {"position": [0.1, 0, 0], "rotation": IDENTITY, "timestamp": 2.0}
    final = {"position": [0.2, 0, 0], "rotation": IDENTITY, "timestamp": 3.0}
    adapter, session = build_adapter((first, first, changed, final), clock)
    adapter.start()
    adapter.begin_trial(1.0)
    session.auto_updates = False
    clock.value = 0.51

    with pytest.raises(ValidationError, match="source update is stale"):
        adapter.poll_required("prolonged stall")

    session.update(changed)
    adapter.poll_required("post-failure diagnostic sample")
    assert adapter.stream_diagnostics().longest_pose_change_interval_s == pytest.approx(0.51)
    event = adapter.stream_diagnostics().receive_discontinuities[-1]
    assert event["gap_duration_s"] == pytest.approx(0.51)
    assert event["previous_websocket_message_index"] == 1
    assert event["current_websocket_message_index"] == 2
    assert event["previous_source_timestamp_s"] == pytest.approx(1.0)
    assert event["current_source_timestamp_s"] == pytest.approx(2.0)
    assert event["session_id"].startswith("FakeSession:")
    assert event["reconnect_observed"] is False
    clock.value = 0.52
    session.auto_updates = True
    with pytest.raises(ValidationError, match="receive discontinuity"):
        adapter.finalize_after_stationarity(2.0)


def test_adapter_fallback_without_update_callback_preserves_recovery_gap():
    """The no-callback compatibility path cannot silently recover a long gap."""
    clock = Clock()
    first = {"position": [0, 0, 0], "rotation": IDENTITY, "timestamp": 1.0}
    changed = {"position": [0.1, 0, 0], "rotation": IDENTITY, "timestamp": 2.0}
    final = {"position": [0.2, 0, 0], "rotation": IDENTITY, "timestamp": 3.0}
    adapter, session = build_adapter((first, changed, final), clock)
    session.on_update = None
    adapter.start()
    adapter.begin_trial(1.0)
    clock.value = 0.51
    adapter.poll_required("fallback recovery")
    clock.value = 0.52

    with pytest.raises(ValidationError, match="receive discontinuity"):
        adapter.finalize_after_stationarity(2.0)

    event = adapter.stream_diagnostics().receive_discontinuities[-1]
    assert event["basis"] == "raw_pose_change_fallback"
    assert event["previous_pose_sample_index"] == 1
    assert event["current_pose_sample_index"] == 2


def test_stationary_stream_diagnostic_never_creates_a_trial_or_ros_command():
    clock = Clock()

    class FakeDiagnosticAdapter:
        def __init__(self):
            self.started = False
            self.stopped = False
            self.polls = 0

        def start(self):
            self.started = True

        def stop(self):
            self.stopped = True

        def poll(self):
            self.polls += 1
            return SimpleNamespace(valid_sample=True, status_reason="ok")

        def stream_diagnostics(self):
            return TeleDexStreamDiagnostics(
                update_callback_available=True,
                websocket_message_count=self.polls,
                first_websocket_message_monotonic_s=0.0,
                last_websocket_message_monotonic_s=clock.value,
                websocket_message_age_s=0.0,
                latest_websocket_interarrival_s=0.1,
                longest_websocket_interarrival_s=0.1,
                pose_change_count=self.polls,
                last_pose_change_monotonic_s=clock.value,
                pose_change_age_s=0.0,
                longest_pose_change_interval_s=0.1,
                connection_generation=1,
                connection_lost=False,
                source_timestamp_available=False,
                source_sequence_available=False)

    adapter = FakeDiagnosticAdapter()
    result = collect_stream_diagnostics(
        adapter, 0.2, 10.0, monotonic=clock,
        sleep=lambda duration: setattr(clock, "value", clock.value + duration))

    assert adapter.started and adapter.stopped
    assert adapter.polls == 2
    assert result["ros_command_publication"] == "none"
    assert result["valid_poll_count"] == 2


def test_adapter_accepts_a_new_websocket_update_when_a_trial_restarts_stationary():
    """An identical stationary pose is fresh only when a packet arrived."""
    clock = Clock()
    initial = {"position": [0, 0, 0], "rotation": IDENTITY}
    fresh = {"position": [0.01, 0, 0], "rotation": IDENTITY}
    adapter, _session = build_adapter((initial, initial, fresh), clock)
    adapter.start()
    first = adapter.begin_trial(1.0)
    second = adapter.begin_trial(2.0)

    assert first.raw_pose_generation == 1
    assert second.raw_pose_generation == 1
    assert not second.raw_pose_changed
    assert second.websocket_message_count > first.websocket_message_count


def test_adapter_rejects_trial_restart_without_a_new_websocket_update():
    """A local cache read cannot establish the next trial's reference."""
    clock = Clock()
    pose = {"position": [0, 0, 0], "rotation": IDENTITY}
    adapter, session = build_adapter((pose, pose), clock)
    adapter.start()
    adapter.begin_trial(1.0)
    session.auto_updates = False

    with pytest.raises(ValidationError, match="unavailable after"):
        adapter.begin_trial(2.0)


def test_adapter_rejects_active_trial_after_transport_disconnect():
    adapter, session = build_adapter((
        {"position": [0, 0, 0], "rotation": IDENTITY},
        {"position": [0.1, 0, 0], "rotation": IDENTITY}))
    adapter.start()
    adapter.begin_trial(1.0)
    session.disconnect()

    with pytest.raises(ValidationError, match="transport disconnect"):
        adapter.poll_required("trajectory")


def test_adapter_rejects_active_trial_after_transport_reconnect():
    """A reconnect may establish a new ARKit world frame during a trial."""
    adapter, session = build_adapter((
        {"position": [0, 0, 0], "rotation": IDENTITY},
        {"position": [0.1, 0, 0], "rotation": IDENTITY}))
    adapter.start()
    adapter.begin_trial(1.0)
    session.reconnect()

    with pytest.raises(ValidationError, match="transport reconnect"):
        adapter.poll_required("trajectory")


def test_adapter_requires_reconnect_before_starting_a_new_trial():
    adapter, session = build_adapter((
        {"position": [0, 0, 0], "rotation": IDENTITY},))
    adapter.start()
    session.disconnect()

    with pytest.raises(ValidationError, match="transport is disconnected"):
        adapter.begin_trial(1.0)

    session.reconnect()
    assert adapter.begin_trial(2.0).valid_sample


def test_adapter_accepts_advanced_source_timestamp_for_stationary_new_pose():
    """A source timestamp proves a fresh packet when a stationary pose is equal."""
    stationary = {"position": [0, 0, 0], "rotation": IDENTITY, "timestamp": 1.0}
    updated = {"position": [0, 0, 0], "rotation": IDENTITY, "timestamp": 2.0}
    adapter, _session = build_adapter((stationary, updated))
    adapter.start()
    adapter.begin_trial(1.0)
    result = adapter.finalize_after_stationarity(2.0)

    assert result.final_pose.raw_pose_changed
    assert result.final_pose.raw_pose_generation == 2


def test_adapter_rejects_partial_first_sample():
    adapter, _session = build_adapter(({"position": [0, 0], "rotation": IDENTITY},))
    adapter.start()
    with pytest.raises(ValidationError, match="initial TeleDex reference unavailable"):
        adapter.begin_trial(1.0)


def test_adapter_fails_closed_when_connection_poll_raises():
    adapter, _session = build_adapter(({"position": [0, 0, 0], "rotation": IDENTITY},
                                       ConnectionError("phone disconnected")))
    adapter.start()
    adapter.begin_trial(1.0)
    with pytest.raises(ValidationError, match="trajectory is invalid"):
        adapter.poll_required("trajectory")


def test_adapter_yaw_unwraps_across_pi():
    def base_rotation(angle):
        return (
            math.cos(angle), -math.sin(angle), 0.0,
            math.sin(angle), math.cos(angle), 0.0,
            0.0, 0.0, 1.0)

    def phone_rotation(angle):
        return tuple(sum(base_rotation(angle)[row * 3 + k] *
                         INITIAL_WORLD_PHONE[k * 3 + column]
                         for k in range(3))
                     for row in range(3) for column in range(3))

    clock = Clock()
    adapter, _session = build_adapter((
        {"position": [0, 0, 0], "rotation": INITIAL_WORLD_PHONE},
        {"position": [0, 0, 0], "rotation": phone_rotation(math.radians(179))},
        {"position": [0, 0, 0], "rotation": phone_rotation(math.radians(-179))},
    ), clock)
    adapter.start()
    adapter.begin_trial(1.0)
    clock.value = 0.1
    adapter.poll_required("trajectory")
    clock.value = 0.2
    result = adapter.finalize_after_stationarity(2.0)
    assert math.degrees(result.final_yaw_unwrapped_rad) == pytest.approx(181.0)


def test_adapter_accumulates_rotation_larger_than_half_turn():
    def base_rotation(angle):
        return (
            math.cos(angle), -math.sin(angle), 0.0,
            math.sin(angle), math.cos(angle), 0.0,
            0.0, 0.0, 1.0)

    def phone_rotation(angle):
        return tuple(sum(base_rotation(angle)[row * 3 + k] *
                         INITIAL_WORLD_PHONE[k * 3 + column]
                         for k in range(3))
                     for row in range(3) for column in range(3))

    adapter, _session = build_adapter((
        {"position": [0, 0, 0], "rotation": INITIAL_WORLD_PHONE},
        {"position": [0, 0, 0], "rotation": phone_rotation(math.radians(120))},
        {"position": [0, 0, 0], "rotation": phone_rotation(math.radians(-120))},
    ))
    adapter.start()
    adapter.begin_trial(1.0)
    adapter.poll_required("trajectory")
    result = adapter.finalize_after_stationarity(2.0)

    assert math.degrees(result.final_yaw_unwrapped_rad) == pytest.approx(240.0)


@pytest.mark.parametrize(("start", "end", "old_yaw_deg", "base_yaw_deg"), (
    (
        ((0.106525376, 0.019154519, -0.019145668),
         (0.994428933, -0.000825150, -0.105406485,
          -0.105366826, -0.036301363, -0.993770659,
          -0.003006390, 0.999340594, -0.036186073)),
        ((0.112533368, -0.083804742, -0.016263507),
         (0.893538892, 0.017304856, 0.448652267,
          0.448901981, -0.053747553, -0.891963243,
          0.008678666, 0.998404622, -0.055793718)),
        -0.569381, 32.726080),
    (
        ((2.841095448, 0.498952538, -0.092008837),
         (0.624583483, -0.073523983, -0.777489424,
          -0.777677417, 0.032603588, -0.627817690,
          0.071508601, 0.996760428, -0.036814276)),
        ((3.090766191, 0.532689929, -0.090865985),
         (-0.670588970, 0.028494153, -0.741281688,
          -0.737785995, 0.078568414, 0.670446813,
          0.077345140, 0.996501505, -0.031664532)),
        32.531882, -80.717725),
))
def test_campaign_regression_old_phone_yaw_is_not_amr_yaw(
        start, end, old_yaw_deg, base_yaw_deg):
    adapter, _session = build_adapter(())
    adapter.start()
    logger = adapter._logger_module

    _translation, old_rotation = logger.relative_pose(*start, *end)
    _translation, base_rotation = logger.relative_base_pose(
        *start, *end, adapter.phone_to_base_rotation,
        adapter.phone_to_base_translation_m)

    assert math.degrees(logger.rpy_from_matrix(old_rotation)[2]) == pytest.approx(
        old_yaw_deg, abs=1e-5)
    assert math.degrees(logger.rpy_from_matrix(base_rotation)[2]) == pytest.approx(
        base_yaw_deg, abs=1e-5)


def test_mount_validation_records_forward_and_ccw_axes():
    ccw_phone = (
        math.cos(math.radians(20)), 0.0, math.sin(math.radians(20)),
        math.sin(math.radians(20)), 0.0, -math.cos(math.radians(20)),
        0.0, 1.0, 0.0)
    adapter, _session = build_adapter((
        {"position": [0, 0, 0], "rotation": INITIAL_WORLD_PHONE},
        {"position": [0.2, 0, 0], "rotation": INITIAL_WORLD_PHONE},
        {"position": [0, 0, 0], "rotation": ccw_phone},
    ))
    adapter.start()

    result = adapter.validate_mount_mapping(
        lambda _prompt: "", 0.05, math.radians(10))

    assert result["detected_phone_forward_axis"] == "+x"
    assert result["detected_phone_yaw_axis_for_positive_ccw"] == "+y"
    assert result["detected_phone_to_base_rotation"] == pytest.approx(
        OBSERVED_PHONE_TO_BASE)
    assert result["selected_phone_to_base_rotation"] == pytest.approx(
        OBSERVED_PHONE_TO_BASE)
    assert adapter.phone_to_base_rotation == pytest.approx(OBSERVED_PHONE_TO_BASE)


@pytest.mark.parametrize("ambiguous_sample", (
    ({"position": [0.2, 0.15, 0.0], "rotation": INITIAL_WORLD_PHONE},),
    ({"position": [0.2, 0.0, 0.0], "rotation": INITIAL_WORLD_PHONE},
     {"position": [0.0, 0.0, 0.0], "rotation": (
         0.925416578, -0.173648178, 0.336824089,
         0.336824089, 0.0, -0.925416578,
         0.173648178, 0.984807753, 0.0)}),
))
def test_mount_validation_rejects_ambiguous_observations(ambiguous_sample):
    if len(ambiguous_sample) == 1:
        forward = ambiguous_sample[0]
        ccw = {"position": [0, 0, 0], "rotation": (
            math.cos(math.radians(20)), 0.0, math.sin(math.radians(20)),
            math.sin(math.radians(20)), 0.0, -math.cos(math.radians(20)),
            0.0, 1.0, 0.0)}
    else:
        forward, ccw = ambiguous_sample
    adapter, _session = build_adapter((
        {"position": [0, 0, 0], "rotation": INITIAL_WORLD_PHONE},
        forward, ccw))
    adapter.start()

    with pytest.raises(ValidationError, match="ambiguous"):
        adapter.validate_mount_mapping(
            lambda _prompt: "", 0.05, math.radians(10))


def test_mount_validation_never_replaces_mismatched_selected_mount():
    adapter, _session = build_adapter((
        {"position": [0, 0, 0], "rotation": INITIAL_WORLD_PHONE},
        {"position": [0, 0.2, 0], "rotation": INITIAL_WORLD_PHONE},
        {"position": [0, 0, 0], "rotation": (
            math.cos(math.radians(20)), 0.0, math.sin(math.radians(20)),
            math.sin(math.radians(20)), 0.0, -math.cos(math.radians(20)),
            0.0, 1.0, 0.0)}))
    adapter.start()
    selected = adapter.phone_to_base_rotation

    with pytest.raises(
            TeleDexMountMappingMismatch,
            match="does not match the selected") as captured:
        adapter.validate_mount_mapping(
            lambda _prompt: "", 0.05, math.radians(10))

    assert adapter.phone_to_base_rotation == selected
    assert captured.value.observation["matches_selected_mount"] is False
    assert captured.value.observation["detected_phone_forward_axis"] == "-z"
    assert captured.value.observation["detected_phone_to_base_rotation"] != list(
        selected)


def test_adapter_path_length_accumulates_curved_backtracking_trajectory():
    adapter, _session = build_adapter((
        {"position": [0.0, 0.0, 0.0], "rotation": INITIAL_WORLD_PHONE},
        {"position": [0.4, 0.3, 0.0], "rotation": INITIAL_WORLD_PHONE},
        {"position": [0.1, 0.3, 0.0], "rotation": INITIAL_WORLD_PHONE},
        {"position": [0.1, 0.0, 0.0], "rotation": INITIAL_WORLD_PHONE},
    ))
    adapter.start()
    adapter.begin_trial(1.0)
    adapter.poll_required("curved trajectory")
    adapter.poll_required("backtracking trajectory")
    result = adapter.finalize_after_stationarity(2.0)

    assert result.total_planar_path_length_m == pytest.approx(1.1)
    assert result.net_forward_displacement_m == pytest.approx(0.1)


def _write_logger_api(path, marker="logger", omit=None):
    names = (
        "base_displacement_metrics", "relative_base_pose", "relative_pose",
        "rotation_from_rpy", "rotation_vector_from_matrix", "rpy_from_matrix",
        "unwrap_angle", "validate_pose", "validate_rotation_matrix")
    lines = ["MARKER = %r" % marker]
    lines.extend(
        "def %s(*args, **kwargs):\n    return None" % name
        for name in names if name != omit)
    path.write_text("\n\n".join(lines) + "\n", encoding="utf-8")


def test_standalone_preflight_rejects_missing_root_and_logger(tmp_path):
    with pytest.raises(ValidationError, match="directory does not exist"):
        load_standalone_logger(tmp_path / "missing")
    with pytest.raises(ValidationError, match="logger is unavailable"):
        load_standalone_logger(tmp_path)


def test_standalone_preflight_rejects_unreadable_logger(tmp_path, monkeypatch):
    logger = tmp_path / "teledex_logger.py"
    _write_logger_api(logger)
    real_access = os.access

    def fake_access(path, mode):
        if Path(path) == logger and mode == os.R_OK:
            return False
        return real_access(path, mode)

    monkeypatch.setattr(adapter_module.os, "access", fake_access)
    with pytest.raises(ValidationError, match="logger is not readable"):
        load_standalone_logger(tmp_path)


def test_standalone_preflight_rejects_incomplete_api(tmp_path):
    _write_logger_api(tmp_path / "teledex_logger.py", omit="relative_pose")
    with pytest.raises(ValidationError, match="missing: relative_pose"):
        load_standalone_logger(tmp_path)


def test_standalone_loader_resolves_each_explicit_root(tmp_path):
    first = tmp_path / "first"
    second = tmp_path / "second"
    first.mkdir()
    second.mkdir()
    _write_logger_api(first / "teledex_logger.py", marker="first")
    _write_logger_api(second / "teledex_logger.py", marker="second")

    assert load_standalone_logger(first).MARKER == "first"
    assert load_standalone_logger(second).MARKER == "second"


def test_teledex_package_preflight_rejects_import_and_api_failures():
    def missing(_name):
        raise ModuleNotFoundError("no teledex")

    with pytest.raises(ValidationError, match="package is unavailable"):
        load_teledex_session_factory(missing)
    with pytest.raises(ValidationError, match="no callable Session"):
        load_teledex_session_factory(lambda _name: SimpleNamespace())

    class IncompleteSession:
        def start(self):
            pass

    with pytest.raises(ValidationError, match="get_latest_data, stop"):
        load_teledex_session_factory(
            lambda _name: SimpleNamespace(Session=IncompleteSession))


def test_teledex_package_preflight_accepts_expected_session_api():
    class Session:
        def start(self):
            pass

        def get_latest_data(self):
            return None

        def stop(self):
            pass

    assert load_teledex_session_factory(
        lambda _name: SimpleNamespace(Session=Session)) is Session
