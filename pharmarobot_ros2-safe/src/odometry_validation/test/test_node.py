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

from types import SimpleNamespace
from threading import Event
import uuid

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import ReliabilityPolicy
from roboteq_ros2_driver.msg import WheelTicks

from odometry_validation.core import EvidenceWriter
from odometry_validation.core import CommandSample
from odometry_validation.core import EmergencyStopController
from odometry_validation.core import EmergencyStopError
from odometry_validation.core import GeometryConfig
from odometry_validation.core import ImuSample
from odometry_validation.core import OdomSample
from odometry_validation.core import ResponsiveOperatorInput
from odometry_validation.core import TrialSamples
from odometry_validation.core import TrialSpec
from odometry_validation.core import ValidationError
from odometry_validation.core import WheelTickSample
from odometry_validation.node import execute_trial
from odometry_validation.node import CMD_VEL_SAFE_TOPIC
from odometry_validation.node import DIAGNOSTICS_TOPIC
from odometry_validation.node import D455_IMU_TOPIC
from odometry_validation.node import ODOM_TOPIC
from odometry_validation.node import OdometryValidationNode
from odometry_validation.node import PRIMARY_IMU_TOPIC
from odometry_validation.node import WHEEL_TICKS_TOPIC
from odometry_validation.node import build_arg_parser
from odometry_validation.node import main
from odometry_validation.square import SquareConfig
from odometry_validation.square_trial import SquareValidationNode
from odometry_validation.square_trial import _validate_execution_limits
from odometry_validation.square_trial import build_square_parser


def construct_node(monkeypatch, args):
    monkeypatch.setattr(Node, "__init__", lambda self, *_args, **_kwargs: None)
    monkeypatch.setattr(
        Node, "create_publisher", lambda self, *_args, **_kwargs: object())
    monkeypatch.setattr(
        Node, "create_subscription", lambda self, *_args, **_kwargs: object())
    return OdometryValidationNode(args)


def construct_square_node(monkeypatch, args):
    monkeypatch.setattr(Node, "__init__", lambda self, *_args, **_kwargs: None)
    monkeypatch.setattr(
        Node, "create_publisher", lambda self, *_args, **_kwargs: object())
    monkeypatch.setattr(
        Node, "create_subscription", lambda self, *_args, **_kwargs: object())
    return SquareValidationNode(args, GeometryConfig(0.1, 0.5, 4096))


def test_square_control_mode_defaults_to_open_loop():
    assert build_square_parser().get_default("square_control_mode") == "open-loop"


def square_parser_args(*extra):
    return build_square_parser().parse_args([
        "--wheel-radius-m", "0.1",
        "--track-width-m", "0.5",
        "--encoder-ticks-per-revolution", "4096",
        *extra])


def test_open_loop_ignores_closed_loop_only_validation():
    args = square_parser_args(
        "--max-rotation-duration-s", "6.0",
        "--closed-loop-turn-timeout-s", "nan",
        "--closed-loop-max-yaw-disagreement-rad", "100.0")
    _validate_execution_limits(args, SquareConfig())


def test_closed_loop_rejects_weakened_sensor_disagreement_guard():
    args = square_parser_args(
        "--square-control-mode", "closed-loop",
        "--closed-loop-max-yaw-disagreement-rad", "100.0")
    with pytest.raises(ValueError, match="exceeds safety maximum"):
        _validate_execution_limits(args, SquareConfig())


def test_square_open_loop_turn_uses_unchanged_base_command_phase(monkeypatch):
    args = square_parser_args()
    node = construct_square_node(monkeypatch, args)
    node.motion_confirmed = True
    observed = []
    monkeypatch.setattr(
        OdometryValidationNode, "run_command_phase",
        lambda _self, spec, observation_hook=None: observed.append(spec))
    turn = SquareConfig().segments()[1]
    node.run_command_phase(turn)
    assert observed == [turn]


def test_closed_loop_mode_changes_turn_dispatch_but_not_translation(monkeypatch):
    args = square_parser_args(
        "--square-control-mode", "closed-loop")
    node = construct_square_node(monkeypatch, args)
    node.motion_confirmed = True
    controlled = []
    open_loop = []
    monkeypatch.setattr(
        node, "_run_closed_loop_turn",
        lambda spec, observation_hook=None: controlled.append(spec))
    monkeypatch.setattr(
        OdometryValidationNode, "run_command_phase",
        lambda _self, spec, observation_hook=None: open_loop.append(spec))
    leg, turn = SquareConfig().segments()[:2]
    node.run_command_phase(leg)
    node.run_command_phase(turn)
    assert open_loop == [leg]
    assert controlled == [turn]


def closed_loop_runtime_node(monkeypatch):
    args = square_parser_args(
        "--square-control-mode", "closed-loop",
        "--closed-loop-settle-hold-s", "0.10")
    node = construct_square_node(monkeypatch, args)
    node.motion_confirmed = True
    node.wheel_ticks = [WheelTickSample(0.0, 0, 0)]
    node.imu = [ImuSample(0.0, 0.0)]
    node.verify_ready_after_operator_input = lambda: None
    node.check_runtime_guards = lambda _spec: None
    return node


def test_closed_loop_runtime_slows_holds_and_publishes_final_zero(monkeypatch):
    node = closed_loop_runtime_node(monkeypatch)
    clock = [0.0]
    encoder_values = iter((0.0, 1.30, 1.56, 1.56, 1.56, 1.56))
    imu_values = iter((0.0, 1.30, 1.56, 1.56, 1.56, 1.56))
    published = []
    forwarded = []
    node._now_seconds = lambda: clock[0]
    node.spin_for = lambda duration: clock.__setitem__(0, clock[0] + duration)
    node.publish_twist = lambda linear, angular: published.append((linear, angular))
    node.publish_zero = lambda: published.append((0.0, 0.0))
    node.wait_for_expected_safe_command = (
        lambda spec, _deadline: forwarded.append(spec.angular_z))
    monkeypatch.setattr(
        "odometry_validation.square_trial.encoder_relative_yaw",
        lambda *_args, **_kwargs: next(encoder_values))
    monkeypatch.setattr(
        "odometry_validation.square_trial.imu_relative_yaw",
        lambda *_args, **_kwargs: next(imu_values))
    monkeypatch.setattr(
        "odometry_validation.square_trial.time.monotonic", lambda: clock[0])
    monkeypatch.setattr(
        "odometry_validation.square_trial.time.sleep",
        lambda duration: clock.__setitem__(0, clock[0] + duration))

    node._run_closed_loop_turn(SquareConfig().segments()[1])

    commands = [angular for _linear, angular in published]
    assert commands[0] == pytest.approx(0.3)
    assert 0.08 < commands[1] < commands[0]
    assert commands[-1] == 0.0
    assert forwarded[-1] == 0.0
    assert node.closed_loop_feedback[-1]["complete"]


def test_closed_loop_runtime_sensor_failure_commands_zero_in_finally(monkeypatch):
    node = closed_loop_runtime_node(monkeypatch)
    published = []
    node._now_seconds = lambda: 0.0
    node.spin_for = lambda _duration: None
    node.publish_zero = lambda: published.append((0.0, 0.0))
    monkeypatch.setattr(
        "odometry_validation.square_trial.encoder_relative_yaw",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            ValidationError("stale encoder sample gap")))

    with pytest.raises(ValidationError, match="stale encoder"):
        node._run_closed_loop_turn(SquareConfig().segments()[1])
    assert published == [(0.0, 0.0)]


def test_closed_loop_low_publish_rate_still_zeros_at_turn_deadline(monkeypatch):
    node = closed_loop_runtime_node(monkeypatch)
    node.args.publish_rate_hz = 0.01
    node.args.closed_loop_turn_timeout_s = 0.10
    clock = [0.0]
    published = []
    node._now_seconds = lambda: clock[0]
    node.spin_for = lambda duration: clock.__setitem__(0, clock[0] + duration)
    node.check_runtime_guards = lambda _spec: None
    node.publish_twist = lambda linear, angular: published.append((linear, angular))
    node.publish_zero = lambda: published.append((0.0, 0.0))
    node.last_safe = None
    node.command_mismatch_details = lambda _spec: "forwarding absent"
    monkeypatch.setattr(
        "odometry_validation.square_trial.encoder_relative_yaw",
        lambda *_args, **_kwargs: 0.0)
    monkeypatch.setattr(
        "odometry_validation.square_trial.imu_relative_yaw",
        lambda *_args, **_kwargs: 0.0)
    monkeypatch.setattr(
        "odometry_validation.square_trial.time.monotonic", lambda: clock[0])

    with pytest.raises(ValidationError, match="forwarding absent"):
        node._run_closed_loop_turn(SquareConfig().segments()[1])
    assert published[0] == (0.0, 0.3)
    assert published[-1] == (0.0, 0.0)
    assert clock[0] == pytest.approx(0.10)


def diagnostic_message(name, level, message="test diagnostic"):
    status = DiagnosticStatus()
    status.name = name
    status.level = bytes((level,))
    status.message = message
    return DiagnosticArray(status=[status])


def preflight_node(monkeypatch, ignored_names=(), max_level=1):
    args = SimpleNamespace(
        qos_depth=17,
        ignore_diagnostic=list(ignored_names),
        preflight_spin_s=0.0,
        required_node=[],
        required_topic=[],
        stale_timeout_s=1.0,
        zero_tolerance=1e-4,
        max_diagnostic_level=max_level)
    node = construct_node(monkeypatch, args)
    node.spin_for = lambda _duration: None
    node.snapshot_graph = lambda: (set(), set())
    node._now_seconds = lambda: 10.0
    node.last_wheel_time = 10.0
    node.last_imu_time = 10.0
    node.last_primary_imu_time = 10.0
    node.last_odom_time = 10.0
    node.last_safe = Twist()
    node.wheel_ticks = [WheelTickSample(10.0, 0, 0)]
    node.imu = [ImuSample(10.0, 0.0)]
    node.odom = [odom_sample(10.0)]
    node.safe_commands = [
        CommandSample(10.0, "/cmd_vel/safe", 0.0, 0.0)]
    return node


def stationarity_node(monkeypatch, sample_count=3):
    args = SimpleNamespace(
        qos_depth=17,
        ignore_diagnostic=[],
        stationary_samples=sample_count,
        wheel_tick_semantics="cumulative",
        stationary_tick_tolerance=0,
        stationarity_window_s=0.2,
        wheel_radius_m=0.0881,
        encoder_ticks_per_revolution=4096,
        stationary_linear_velocity_tolerance=0.01,
        stationary_angular_velocity_tolerance=0.02,
        stale_timeout_s=1.0,
        max_diagnostic_level=1,
        zero_tolerance=1e-4)
    node = construct_node(monkeypatch, args)
    node._now_seconds = lambda: 1.5
    node.last_wheel_time = 1.5
    node.last_odom_time = 1.5
    node.last_safe = Twist()
    node.last_safe_time = 1.5
    node.first_zero_timestamp_s = 1.0
    node.post_zero_wheel_start = 0
    node.post_zero_odom_start = 0
    node.post_zero_safe_start = 0
    node.safe_commands = (
        [CommandSample(1.0, "/cmd_vel/safe", 0.0, 0.0, "after_motion")])
    return node


def odom_sample(timestamp, linear=0.0, angular=0.0):
    return OdomSample(
        timestamp_s=timestamp,
        x_m=0.0,
        y_m=0.0,
        yaw_rad=0.0,
        linear_x_m_s=linear,
        angular_z_rad_s=angular,
        phase="after_motion")


def test_imu_subscriptions_are_best_effort_and_control_subscriptions_unchanged(
        monkeypatch):
    subscriptions = []

    monkeypatch.setattr(Node, "__init__", lambda self, *_args, **_kwargs: None)
    monkeypatch.setattr(
        Node, "create_publisher", lambda self, *_args, **_kwargs: object())

    def capture_subscription(_self, message_type, topic, callback, qos):
        subscriptions.append((message_type, topic, callback, qos))
        return object()

    monkeypatch.setattr(Node, "create_subscription", capture_subscription)

    depth = 17
    OdometryValidationNode(SimpleNamespace(qos_depth=depth))

    qos_by_topic = {topic: qos for _type, topic, _callback, qos in subscriptions}

    for topic in (PRIMARY_IMU_TOPIC, D455_IMU_TOPIC):
        qos = qos_by_topic[topic]
        assert qos.depth == depth
        assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
        assert qos.durability == DurabilityPolicy.VOLATILE

    for topic in (
            CMD_VEL_SAFE_TOPIC, WHEEL_TICKS_TOPIC, ODOM_TOPIC,
            DIAGNOSTICS_TOPIC):
        assert qos_by_topic[topic] == depth


def test_live_encoder_and_odom_callbacks_receive_published_messages(
        monkeypatch):
    unique = f"test_{uuid.uuid4().hex}"
    wheel_topic = f"/odometry_validation_test/{unique}/wheel_ticks"
    odom_topic = f"/odometry_validation_test/{unique}/odom"
    monkeypatch.setattr(
        "odometry_validation.node.WHEEL_TICKS_TOPIC", wheel_topic)
    monkeypatch.setattr("odometry_validation.node.ODOM_TOPIC", odom_topic)
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + uuid.uuid4().int % 100))
    context = Context()
    rclpy.init(context=context)
    args = SimpleNamespace(qos_depth=10, ignore_diagnostic=[])
    validator = OdometryValidationNode(args, context=context)
    publisher_node = Node("odometry_validation_test_source", context=context)
    wheel_publisher = publisher_node.create_publisher(
        WheelTicks, wheel_topic, 10)
    odom_publisher = publisher_node.create_publisher(Odometry, odom_topic, 10)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(validator)
    executor.add_node(publisher_node)
    try:
        wheel_message = WheelTicks()
        wheel_message.left_ticks = 123
        wheel_message.right_ticks = -456
        odom_message = Odometry()
        odom_message.pose.pose.position.x = 1.25
        odom_message.twist.twist.linear.x = 0.1
        for _attempt in range(100):
            wheel_publisher.publish(wheel_message)
            odom_publisher.publish(odom_message)
            executor.spin_once(timeout_sec=0.02)
            if validator.wheel_ticks and validator.odom:
                break

        assert validator.wheel_ticks[-1].left_ticks == 123
        assert validator.wheel_ticks[-1].right_ticks == -456
        assert validator.odom[-1].x_m == pytest.approx(1.25)
        assert validator.odom[-1].linear_x_m_s == pytest.approx(0.1)
    finally:
        executor.remove_node(publisher_node)
        executor.remove_node(validator)
        publisher_node.destroy_node()
        validator.destroy_node()
        executor.shutdown()
        rclpy.shutdown(context=context)


def test_repeatable_ignore_diagnostic_option_preserves_exact_names():
    parser = build_arg_parser()

    args = parser.parse_args([
        "--wheel-radius-m", "0.1",
        "--track-width-m", "0.5",
        "--encoder-ticks-per-revolution", "4096",
        "--ignore-diagnostic", "roboteq/channel_1_telemetry",
        "--ignore-diagnostic", "roboteq/channel_2_telemetry",
    ])

    assert args.ignore_diagnostic == [
        "roboteq/channel_1_telemetry",
        "roboteq/channel_2_telemetry",
    ]
    assert args.wheel_tick_semantics == "delta"


def test_container_teledex_reference_root_is_default_and_environment_configurable(
        monkeypatch):
    monkeypatch.delenv("TELEDEX_REFERENCE_ROOT", raising=False)
    assert build_arg_parser().get_default(
        "teledex_reference_root") == "/opt/teledex_reference"
    monkeypatch.setenv("TELEDEX_REFERENCE_ROOT", "/read-only/reference")
    assert build_arg_parser().get_default(
        "teledex_reference_root") == "/read-only/reference"


def test_max_diagnostic_level_only_accepts_warning_policy_values():
    parser = build_arg_parser()
    required = [
        "--wheel-radius-m", "0.1",
        "--track-width-m", "0.5",
        "--encoder-ticks-per-revolution", "4096",
    ]

    assert parser.parse_args(required + [
        "--max-diagnostic-level", "0"]).max_diagnostic_level == 0
    assert parser.parse_args(required + [
        "--max-diagnostic-level", "1"]).max_diagnostic_level == 1
    with pytest.raises(SystemExit):
        parser.parse_args(required + ["--max-diagnostic-level", "2"])


@pytest.mark.parametrize(
    ("option", "value"),
    (
        ("--publish-rate-hz", "nan"),
        ("--zero-publish-timeout-s", "inf"),
        ("--zero-publish-rate-hz", "nan"),
        ("--post-stop-settle-s", "inf"),
        ("--stale-timeout-s", "nan"),
        ("--between-trial-stop-s", "-inf"),
        ("--preflight-spin-s", "inf"),
        ("--zero-tolerance", "inf"),
        ("--command-tolerance", "nan"),
        ("--stationary-linear-velocity-tolerance", "inf"),
        ("--stationary-angular-velocity-tolerance", "nan"),
    ))
def test_main_rejects_nonfinite_timing_options(option, value):
    argv = [
        "--wheel-radius-m", "0.1",
        "--track-width-m", "0.5",
        "--encoder-ticks-per-revolution", "4096",
        option, value,
    ]

    with pytest.raises(SystemExit) as captured:
        main(argv)

    assert captured.value.code == 2


def test_allowlisted_level_two_diagnostic_does_not_block_preflight(monkeypatch):
    name = "roboteq/channel_1_telemetry"
    node = preflight_node(monkeypatch, ignored_names=(name,))

    node._diagnostics_callback(
        diagnostic_message(name, 2, "telemetry is stale"))

    node.verify_preflight()
    assert node.last_diagnostic_level == 0
    assert [sample.name for sample in node.ignored_diagnostics] == [name]


def test_cli_ignored_diagnostics_reach_node_runtime_guards(monkeypatch):
    parser = build_arg_parser()
    args = parser.parse_args([
        "--wheel-radius-m", "0.1",
        "--track-width-m", "0.5",
        "--encoder-ticks-per-revolution", "4096",
        "--max-diagnostic-level", "1",
        "--ignore-diagnostic", "roboteq/channel_1_telemetry",
        "--ignore-diagnostic", "roboteq/channel_2_telemetry",
    ])
    args.required_node = []
    args.required_topic = []
    node = construct_node(monkeypatch, args)
    node.spin_for = lambda _duration: None
    node.snapshot_graph = lambda: (set(), set())
    node.count_publishers = lambda _topic: 1
    node._now_seconds = lambda: 10.0
    node.last_wheel_time = 10.0
    node.last_primary_imu_time = 10.0
    node.last_odom_time = 10.0
    node.last_safe_time = 10.0
    node.last_safe = Twist()
    node.wheel_ticks = [WheelTickSample(10.0, 0, 0)]
    node.imu = [ImuSample(10.0, 0.0)]
    node.odom = [odom_sample(10.0)]
    node.safe_commands = [
        CommandSample(10.0, "/cmd_vel/safe", 0.0, 0.0)]

    node._diagnostics_callback(
        diagnostic_message("roboteq/channel_1_telemetry", 2, "stale"))
    node._diagnostics_callback(
        diagnostic_message("roboteq/channel_2_telemetry", 2, "stale"))

    node.verify_preflight()
    assert node.last_diagnostic_level == 0
    assert [sample.name for sample in node.ignored_diagnostics] == [
        "roboteq/channel_1_telemetry",
        "roboteq/channel_2_telemetry",
    ]

    spec = TrialSpec("rot-001-ccw", "rotation", 0.2, 2.0, "ccw")
    node.last_safe.angular.z = spec.angular_z
    node.last_test_command_timestamp_s = 9.9
    node.last_test_command_safe_index = 0
    node.safe_commands.append(
        CommandSample(10.0, "/cmd_vel/safe", 0.0, spec.angular_z))
    node.check_runtime_guards(spec)


def test_preflight_services_callbacks_before_validating(monkeypatch):
    node = preflight_node(monkeypatch)
    spins = []
    node.args.preflight_spin_s = 0.75
    node.spin_for = spins.append

    node.verify_preflight()

    assert spins == [0.75]


def test_preflight_rejects_retained_state_without_current_attempt_callbacks(
        monkeypatch):
    node = preflight_node(monkeypatch)
    node.wheel_ticks.clear()
    node.odom.clear()

    with pytest.raises(
            ValidationError, match="encoder messages are not available"):
        node.verify_preflight()


def test_non_allowlisted_level_two_diagnostic_still_blocks_preflight(monkeypatch):
    node = preflight_node(
        monkeypatch,
        ignored_names=("roboteq/channel_1_telemetry",))

    node._diagnostics_callback(
        diagnostic_message("roboteq/serial_connection", 2, "not ready"))

    with pytest.raises(ValidationError, match="diagnostics are not healthy"):
        node.verify_preflight()


@pytest.mark.parametrize(
    ("max_level", "blocks"),
    ((0, True), (1, False)))
def test_level_one_handling_remains_configurable(monkeypatch, max_level, blocks):
    node = preflight_node(monkeypatch, max_level=max_level)
    node._diagnostics_callback(
        diagnostic_message("roboteq/controller_faults", 1, "unsupported"))

    if blocks:
        with pytest.raises(ValidationError, match="diagnostics are not healthy"):
            node.verify_preflight()
    else:
        node.verify_preflight()


def test_allowlist_matching_is_exact(monkeypatch):
    node = preflight_node(
        monkeypatch,
        ignored_names=("roboteq/channel_1_telemetry",))

    node._diagnostics_callback(
        diagnostic_message("roboteq/channel_1_telemetry_extra", 2))

    with pytest.raises(ValidationError, match="diagnostics are not healthy"):
        node.verify_preflight()


def test_allowlisted_malformed_level_still_blocks_preflight(monkeypatch):
    name = "roboteq/channel_1_telemetry"
    node = preflight_node(monkeypatch, ignored_names=(name,))
    node.get_logger = lambda: SimpleNamespace(error=lambda _message: None)
    malformed = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=0, nanosec=0)),
        status=[SimpleNamespace(
            name=name,
            level=b"\x02\x00",
            message="malformed level")])

    node._diagnostics_callback(malformed)

    assert node.ignored_diagnostics == []
    with pytest.raises(ValidationError, match="diagnostics are not healthy"):
        node.verify_preflight()


def test_ignored_callback_samples_are_persisted_on_failed_preflight(
        monkeypatch, tmp_path):
    ignored_name = "roboteq/channel_1_telemetry"
    node = preflight_node(monkeypatch, ignored_names=(ignored_name,))
    node._diagnostics_callback(
        diagnostic_message(ignored_name, 2, "telemetry is stale"))
    node._diagnostics_callback(
        diagnostic_message("roboteq/serial_connection", 2, "not ready"))
    writer = EvidenceWriter(tmp_path)
    evidence_dir = writer.create({
        "ignored_diagnostic_names": [ignored_name],
    })

    with pytest.raises(ValidationError) as failure:
        node.verify_preflight()
    writer.write_failure([], failure.value, node.samples())

    failure_payload = (evidence_dir / "failure.json").read_text(
        encoding="utf-8")
    report = (evidence_dir / "report.md").read_text(encoding="utf-8")
    assert ignored_name in failure_payload
    assert "telemetry is stale" in failure_payload
    assert "Outcome: failed closed" in report
    assert ignored_name in report
    traceback_text = (evidence_dir / "traceback.txt").read_text(
        encoding="utf-8")
    assert "ValidationError: diagnostics are not healthy" in traceback_text


def test_operator_wait_polling_keeps_observations_fresh_before_command(
        monkeypatch):
    node = preflight_node(monkeypatch)
    release_prompt = Event()
    now = [10.0]
    polls = []
    node.last_wheel_time = 0.0
    node.last_imu_time = 0.0
    node.last_primary_imu_time = 0.0
    node._now_seconds = lambda: now[0]

    def prompt(_prompt):
        assert release_prompt.wait(timeout=1.0)
        return "0"

    def poll():
        polls.append("poll")
        now[0] += 0.1
        node.last_wheel_time = now[0]
        node.last_imu_time = now[0]
        node.last_primary_imu_time = now[0]
        if len(polls) == 3:
            release_prompt.set()

    operator_input = ResponsiveOperatorInput(
        prompt=prompt,
        poll=poll,
        notify=lambda _message: None,
        poll_interval_s=0.001)

    assert operator_input.read_float("heading: ") == pytest.approx(0.0)
    node.verify_ready_after_operator_input()
    assert len(polls) >= 3


def test_primary_imu_staleness_is_not_masked_by_d455_raw_traffic(monkeypatch):
    node = preflight_node(monkeypatch)
    node.last_imu_time = 10.0
    node.last_primary_imu_time = 8.0
    node.imu.append(ImuSample(
        10.0, 0.0, source_topic="/imu/d455/data_raw"))

    with pytest.raises(ValidationError, match="primary IMU messages are stale"):
        node.verify_preflight()

    with pytest.raises(ValidationError, match="stale primary IMU before command"):
        node.verify_ready_after_operator_input()


def test_runtime_guard_runs_before_each_additional_nonzero_publication(
        monkeypatch):
    events = []
    clock = [0.0]
    fake_node = SimpleNamespace(
        args=SimpleNamespace(publish_rate_hz=20.0, dry_run=False),
        motion_armed=False,
        nonzero_command_published=False,
        _now_seconds=lambda: clock[0],
        spin_for=lambda _duration: events.append("spin"),
        verify_ready_after_operator_input=lambda: events.append("ready"),
        publish_twist=lambda _linear, _angular: events.append("publish"))

    def fail_runtime_guard(_spec):
        events.append("guard")
        raise ValidationError("stale encoder during trial")

    fake_node.check_runtime_guards = fail_runtime_guard
    monkeypatch.setattr(
        "odometry_validation.node.time.monotonic", lambda: clock[0])
    monkeypatch.setattr(
        "odometry_validation.node.time.sleep",
        lambda duration: clock.__setitem__(0, clock[0] + duration))
    spec = TrialSpec("rot-001-ccw", "rotation", 0.3, 0.1, "ccw")

    with pytest.raises(ValidationError, match="stale encoder during trial"):
        OdometryValidationNode.run_command_phase(fake_node, spec)

    assert events == ["spin", "ready", "publish", "spin", "guard"]
    assert fake_node.motion_armed
    assert OdometryValidationNode.stationarity_required(fake_node)


def test_command_mismatch_diagnostics_include_expected_observed_timing_and_phase(
        monkeypatch):
    node = stationarity_node(monkeypatch)
    node._now_seconds = lambda: 12.0
    node.last_safe = Twist()
    node.last_safe.linear.x = 0.1
    node.last_safe.angular.z = -0.2
    node.last_safe_time = 11.5
    node.last_test_command_timestamp_s = 11.75
    node.last_wheel_time = 12.0
    node.last_primary_imu_time = 12.0
    node.args.command_tolerance = 1e-4
    node.sample_phase = "during_motion"
    node.count_publishers = lambda _topic: 2
    spec = TrialSpec("trans-001-forward", "translation", 0.3, 1.0, "forward")

    with pytest.raises(ValidationError) as error:
        node.check_runtime_guards(spec)
    message = str(error.value)
    assert "expected linear=0.300000, angular=0.000000" in message
    assert "observed linear=0.1, angular=-0.2" in message
    assert "sample_age_s=0.5" in message
    assert "publisher_count=2" in message
    assert "phase=during_motion" in message
    assert "elapsed_since_publication_s=0.25" in message


def test_wait_for_expected_safe_command_allows_bounded_propagation_race(monkeypatch):
    spec = TrialSpec("trans-001-forward", "translation", 0.3, 1.0, "forward")
    fake = SimpleNamespace(
        args=SimpleNamespace(stale_timeout_s=0.5, command_tolerance=1e-4),
        last_safe=None, last_safe_time=None,
        last_test_command_timestamp_s=10.0,
        last_test_command_safe_index=0, safe_commands=[],
        _now_seconds=lambda: 10.01)
    safe = Twist()
    safe.linear.x = spec.linear_x

    def spin_for(_duration):
        fake.last_safe = safe
        fake.last_safe_time = 10.01
        fake.safe_commands.append(
            CommandSample(10.01, "/cmd_vel/safe", spec.linear_x, 0.0))

    fake.spin_for = spin_for
    ticks = iter((0.0, 0.01, 0.02))
    monkeypatch.setattr("odometry_validation.node.time.monotonic", lambda: next(ticks))
    OdometryValidationNode.wait_for_expected_safe_command(fake, spec)


def test_wait_for_expected_safe_command_rejects_preexisting_matching_sample(
        monkeypatch):
    spec = TrialSpec("trans-001-forward", "translation", 0.3, 1.0, "forward")
    safe = Twist()
    safe.linear.x = spec.linear_x
    fake = SimpleNamespace(
        args=SimpleNamespace(stale_timeout_s=0.05, command_tolerance=1e-4),
        last_safe=safe, last_safe_time=9.0,
        last_test_command_timestamp_s=10.0, spin_for=lambda _duration: None,
        last_test_command_safe_index=1,
        safe_commands=[CommandSample(
            9.0, "/cmd_vel/safe", spec.linear_x, spec.angular_z)],
        _now_seconds=lambda: 10.0,
        command_mismatch_details=lambda _spec: "command mismatch")
    ticks = iter((0.0, 0.1))
    monkeypatch.setattr("odometry_validation.node.time.monotonic", lambda: next(ticks))
    with pytest.raises(ValidationError, match="command mismatch"):
        OdometryValidationNode.wait_for_expected_safe_command(fake, spec)


def test_first_readiness_guard_failure_is_armed_without_publishing(
        monkeypatch):
    publications = []
    fake_node = SimpleNamespace(
        args=SimpleNamespace(publish_rate_hz=20.0, dry_run=False),
        motion_armed=False,
        nonzero_command_published=False,
        spin_for=lambda _duration: None,
        verify_ready_after_operator_input=lambda: (_ for _ in ()).throw(
            ValidationError("stale encoder before command")),
        check_runtime_guards=lambda _spec: None,
        publish_twist=lambda linear, angular: publications.append(
            (linear, angular)))
    monkeypatch.setattr(
        "odometry_validation.node.time.monotonic", lambda: 0.0)
    spec = TrialSpec("rot-001-ccw", "rotation", 0.3, 0.1, "ccw")

    with pytest.raises(ValidationError, match="stale encoder before command"):
        OdometryValidationNode.run_command_phase(fake_node, spec)

    assert publications == []
    assert fake_node.motion_armed
    assert not fake_node.nonzero_command_published
    assert OdometryValidationNode.stationarity_required(fake_node)


def test_command_phase_records_ros_motion_window_around_nonzero_commands(
        monkeypatch):
    clock = [0.0]
    publications = []

    def advance(duration):
        clock[0] += duration

    fake_node = SimpleNamespace(
        args=SimpleNamespace(publish_rate_hz=20.0, dry_run=False),
        motion_armed=False,
        nonzero_command_published=False,
        command_start_timestamp_s=None,
        command_end_timestamp_s=None,
        sample_phase="before_motion",
        _now_seconds=lambda: 100.0 + clock[0],
        spin_for=lambda duration: advance(duration),
        verify_ready_after_operator_input=lambda: None,
        check_runtime_guards=lambda _spec: None,
        publish_twist=lambda linear, angular: publications.append(
            (100.0 + clock[0], linear, angular)))
    monkeypatch.setattr(
        "odometry_validation.node.time.monotonic", lambda: clock[0])
    monkeypatch.setattr("odometry_validation.node.time.sleep", advance)
    spec = TrialSpec("rot-001-ccw", "rotation", 0.3, 0.1, "ccw")

    OdometryValidationNode.run_command_phase(fake_node, spec)

    assert publications
    assert fake_node.command_start_timestamp_s == pytest.approx(publications[0][0])
    assert fake_node.command_end_timestamp_s >= publications[-1][0]
    assert fake_node.command_end_timestamp_s - fake_node.command_start_timestamp_s <= 0.11


def test_stationarity_allows_residual_motion_that_later_settles(monkeypatch):
    node = stationarity_node(monkeypatch)
    node.wheel_ticks = [
        WheelTickSample(1.0, 100, -100),
        WheelTickSample(1.1, 103, -102),
        WheelTickSample(1.2, 103, -102),
        WheelTickSample(1.3, 103, -102),
        WheelTickSample(1.4, 103, -102),
    ]
    node.odom = [odom_sample(1.2), odom_sample(1.3), odom_sample(1.4)]

    assessment = node.verify_stationary()

    assert assessment.stationary
    assert [sample.left_delta_ticks for sample in assessment.stationarity_samples] == [
        0, 0, 0]


def test_constant_nonzero_cumulative_ticks_are_stationary(monkeypatch):
    node = stationarity_node(monkeypatch)
    node.wheel_ticks = [
        WheelTickSample(timestamp, 54321, -12345)
        for timestamp in (1.0, 1.1, 1.2, 1.3)
    ]
    node.odom = [odom_sample(1.1), odom_sample(1.2), odom_sample(1.3)]

    assessment = node.verify_stationary()

    assert assessment.stationary
    assert assessment.tick_deltas_stationary


def test_changing_cumulative_ticks_are_moving(monkeypatch):
    node = stationarity_node(monkeypatch)
    node.wheel_ticks = [
        WheelTickSample(1.0, 100, 200),
        WheelTickSample(1.1, 101, 200),
        WheelTickSample(1.2, 102, 200),
        WheelTickSample(1.3, 103, 200),
    ]
    node.odom = [odom_sample(1.1), odom_sample(1.2), odom_sample(1.3)]

    assessment = node.verify_stationary()

    assert not assessment.stationary
    assert not assessment.tick_deltas_stationary
    assert "directional accumulation exceeded chatter bound" in assessment.reason


def test_repeated_nonzero_production_deltas_are_moving(monkeypatch):
    node = stationarity_node(monkeypatch)
    node.args.wheel_tick_semantics = "delta"
    node.wheel_ticks = [
        WheelTickSample(timestamp, 2, -2)
        for timestamp in (1.0, 1.1, 1.2)
    ]
    node.odom = [odom_sample(1.0), odom_sample(1.1), odom_sample(1.2)]

    assessment = node.verify_stationary()

    assert not assessment.stationary
    assert assessment.wheel_tick_semantics == "delta"
    assert [sample.left_delta_ticks for sample in assessment.stationarity_samples] == [
        2, 2, 2]


def test_transient_encoder_noise_is_discarded_after_latest_valid_window(
        monkeypatch):
    node = stationarity_node(monkeypatch)
    node.args.wheel_tick_semantics = "delta"
    node.wheel_ticks = [
        WheelTickSample(timestamp, left, right)
        for timestamp, left, right in (
            (1.0, 1, -1),
            (1.1, -1, 1),
            (1.2, 0, 0),
            (1.3, 0, 0),
            (1.4, 0, 0),
        )
    ]
    node.odom = [odom_sample(1.2), odom_sample(1.3), odom_sample(1.4)]

    assessment = node.verify_stationary()

    assert assessment.stationary
    assert [
        (sample.left_delta_ticks, sample.right_delta_ticks)
        for sample in assessment.stationarity_samples
    ] == [(0, 0), (0, 0), (0, 0)]


def chatter_stationarity_node(monkeypatch, left_ticks, right_ticks,
                              odom_linear=0.0, odom_angular=0.0):
    node = stationarity_node(monkeypatch)
    node.args.wheel_tick_semantics = "delta"
    node.args.stationary_samples = 5
    node.args.stationarity_window_s = 0.35
    node._now_seconds = lambda: 1.5
    node.last_wheel_time = 1.5
    node.last_odom_time = 1.5
    node.last_safe_time = 1.5
    timestamps = tuple(1.1 + 0.1 * index
                       for index in range(len(left_ticks)))
    node.wheel_ticks = [
        WheelTickSample(timestamp, left, right)
        for timestamp, left, right in zip(
            timestamps, left_ticks, right_ticks)]
    node.odom = [
        odom_sample(timestamp, odom_linear, odom_angular)
        for timestamp in timestamps]
    return node


def test_encoder_window_accepts_exact_zero(monkeypatch):
    node = chatter_stationarity_node(monkeypatch, [0] * 5, [0] * 5)

    assessment = node.verify_stationary()

    assert assessment.stationary
    assert assessment.encoder_activity_class == "zero_activity"
    assert assessment.encoder_window_duration_s == pytest.approx(0.4)
    assert assessment.left_net_ticks == 0
    assert assessment.right_absolute_ticks == 0
    assert assessment.left_absolute_displacement_mm == pytest.approx(0.0)


def test_encoder_window_covers_duration_at_high_callback_rate(monkeypatch):
    node = chatter_stationarity_node(
        monkeypatch, [0] * 9, [0] * 9)
    node.wheel_ticks = [
        WheelTickSample(1.1 + 0.05 * index, 0, 0)
        for index in range(9)]
    node.odom = [
        odom_sample(1.1 + 0.05 * index)
        for index in range(9)]
    node._now_seconds = lambda: 1.5
    node.last_wheel_time = 1.5
    node.last_odom_time = 1.5
    node.last_safe_time = 1.5

    assessment = node.verify_stationary()

    assert assessment.stationary
    assert assessment.observed_delta_samples == 8
    assert assessment.encoder_window_duration_s == pytest.approx(0.35)


def test_encoder_window_accepts_alternating_one_tick_chatter(monkeypatch):
    node = chatter_stationarity_node(
        monkeypatch, [0, 1, -1, 0, 1, -1], [0, 0, 0, 0, 0, 0])

    assessment = node.verify_stationary()

    assert assessment.stationary
    assert assessment.encoder_activity_class == "alternating_chatter"
    assert assessment.left_net_ticks == 0
    assert assessment.left_absolute_ticks == 4
    assert assessment.maximum_individual_delta_ticks == 1
    assert assessment.left_absolute_displacement_mm == pytest.approx(
        4 * 2.0 * 3.141592653589793 * 0.0881 * 1000.0 / 4096)


def test_encoder_window_accepts_one_sided_left_chatter(monkeypatch):
    node = chatter_stationarity_node(
        monkeypatch, [0, 1, -1, 0, 1, -1], [0, 0, 0, 0, 0, 0])

    assessment = node.verify_stationary()

    assert assessment.stationary
    assert assessment.right_absolute_ticks == 0
    assert assessment.left_directional_accumulation_ticks == 1


def test_encoder_window_accepts_small_bounded_zero_net_excursion(monkeypatch):
    node = chatter_stationarity_node(
        monkeypatch, [1, -1, 1, -1, 0], [0, 0, 0, 0, 0])

    assessment = node.verify_stationary()

    assert assessment.stationary
    assert assessment.left_net_ticks == 0
    assert assessment.left_absolute_ticks == 4


@pytest.mark.parametrize(
    "left_ticks,right_ticks,reason", (
        ([1, 1, 1, 1, 1], [0, 0, 0, 0, 0], "directional accumulation"),
        ([5, 5, 5, 5, 5], [5, 5, 5, 5, 5], "maximum encoder sample delta"),
        ([1, -1, 1, -1, 1], [0, 0, 0, 0, 0], "cumulative excursion"),
    ))
def test_encoder_window_rejects_sustained_or_excessive_motion(
        monkeypatch, left_ticks, right_ticks, reason):
    node = chatter_stationarity_node(monkeypatch, left_ticks, right_ticks)

    assessment = node.verify_stationary()

    assert not assessment.stationary
    assert not assessment.tick_deltas_stationary
    assert reason in assessment.reason


def test_encoder_window_rejects_symmetric_real_coast(monkeypatch):
    node = chatter_stationarity_node(
        monkeypatch, [8, 7, 6, 5, 4], [8, 7, 6, 5, 4])

    assessment = node.verify_stationary()

    assert not assessment.stationary
    assert assessment.encoder_activity_class == "directional_motion"
    assert assessment.left_net_displacement_mm > 0.0


def test_encoder_window_rejects_stale_encoder(monkeypatch):
    node = chatter_stationarity_node(monkeypatch, [0] * 5, [0] * 5)
    node.last_wheel_time = 0.0

    assessment = node.verify_stationary()

    assert not assessment.stationary
    assert not assessment.encoder_data_fresh
    assert "encoder data became stale" in assessment.reason


def test_encoder_chatter_does_not_mask_moving_odometry(monkeypatch):
    node = chatter_stationarity_node(
        monkeypatch, [0, 1, -1, 0, 1, -1], [0] * 6,
        odom_linear=0.02)

    assessment = node.verify_stationary()

    assert assessment.tick_deltas_stationary
    assert not assessment.odom_twist_stationary
    assert not assessment.stationary


def test_encoder_chatter_does_not_mask_nonzero_safe_command(monkeypatch):
    node = chatter_stationarity_node(
        monkeypatch, [0, 1, -1, 0, 1, -1], [0] * 6)
    nonzero = Twist()
    nonzero.linear.x = 0.1
    node.last_safe = nonzero
    node.safe_commands.append(CommandSample(
        1.5, "/cmd_vel/safe", 0.1, 0.0, "after_motion"))

    assessment = node.verify_stationary()

    assert assessment.tick_deltas_stationary
    assert not assessment.safe_zero
    assert not assessment.stationary


def test_encoder_transition_from_coast_to_chatter_is_accepted(monkeypatch):
    node = chatter_stationarity_node(
        monkeypatch, [5, 5, 4, 3, 2], [5, 5, 4, 3, 2])
    first = node.verify_stationary()
    assert not first.stationary

    node._now_seconds = lambda: 2.0
    node.last_wheel_time = 2.0
    node.last_odom_time = 2.0
    node.last_safe_time = 2.0
    node.wheel_ticks.extend([
        WheelTickSample(timestamp, left, right)
        for timestamp, left, right in (
            (1.6, 0, 0), (1.7, 1, -1), (1.8, -1, 1),
            (1.9, 0, 0), (2.0, 1, -1))])
    node.odom.extend([odom_sample(timestamp) for timestamp in
                      (1.6, 1.7, 1.8, 1.9, 2.0)])

    second = node.verify_stationary()

    assert second.stationary
    assert second.encoder_activity_class == "alternating_chatter"


def test_safe_command_zero_does_not_mask_continuing_physical_motion(
        monkeypatch):
    node = stationarity_node(monkeypatch)
    node.args.wheel_tick_semantics = "delta"
    node.wheel_ticks = [
        WheelTickSample(timestamp, 1, -1)
        for timestamp in (1.0, 1.1, 1.2)
    ]
    node.odom = [
        odom_sample(timestamp, angular=0.1)
        for timestamp in (1.0, 1.1, 1.2)
    ]

    assessment = node.verify_stationary()

    assert assessment.safe_zero
    assert not assessment.tick_deltas_stationary
    assert not assessment.odom_twist_stationary
    assert not assessment.stationary
    assert assessment.time_from_first_zero_to_safe_zero_s == pytest.approx(0.5)
    assert assessment.max_abs_left_delta_ticks == 1
    assert assessment.max_abs_right_delta_ticks == 1
    assert assessment.max_abs_odom_linear_x_m_s == pytest.approx(0.0)
    assert assessment.max_abs_odom_angular_z_rad_s == pytest.approx(0.1)


def test_odom_stationary_can_precede_encoder_stationarity(monkeypatch):
    node = stationarity_node(monkeypatch)
    node.args.wheel_tick_semantics = "delta"
    node._now_seconds = lambda: 2.0
    node.wheel_ticks = [
        WheelTickSample(timestamp, left, right)
        for timestamp, left, right in (
            (1.0, 1, 0), (1.1, 1, 0), (1.2, 1, 0))]
    node.odom = [odom_sample(timestamp) for timestamp in (1.0, 1.1, 1.2)]

    first = node.verify_stationary()

    assert first.odom_twist_stationary
    assert not first.tick_deltas_stationary
    assert first.first_odom_stationary_timestamp_s == 2.0
    assert first.first_encoder_stationary_timestamp_s is None
    assert first.last_encoder_motion_timestamp_s == 1.2

    node._now_seconds = lambda: 2.1
    node.wheel_ticks.extend([
        WheelTickSample(1.3, 0, 0),
        WheelTickSample(1.4, 0, 0)])
    second = node.verify_stationary()

    assert second.stationary
    assert second.first_odom_stationary_timestamp_s == 2.0
    assert second.first_encoder_stationary_timestamp_s == 2.1


def test_controlled_stop_guard_rejects_stale_data_immediately(monkeypatch):
    node = stationarity_node(monkeypatch)
    node.verify_graph_unchanged = lambda: None
    node._now_seconds = lambda: 10.0
    node.last_wheel_time = 8.0
    node.last_imu_time = 10.0
    node.last_primary_imu_time = 10.0
    node.last_odom_time = 10.0
    node.last_safe_time = 10.0

    with pytest.raises(
            ValidationError, match="stale encoder during controlled stop"):
        node.verify_controlled_stop_guards()


def test_controlled_stop_guard_rejects_fresh_safe_command_mismatch(monkeypatch):
    node = stationarity_node(monkeypatch)
    node.verify_graph_unchanged = lambda: None
    node._now_seconds = lambda: 10.0
    node.last_wheel_time = 10.0
    node.last_imu_time = 10.0
    node.last_primary_imu_time = 10.0
    node.last_odom_time = 10.0
    node.last_safe_time = 10.0
    node.first_zero_timestamp_s = 9.9
    node.post_zero_wheel_start = 0
    node.post_zero_imu_start = 0
    node.post_zero_odom_start = 0
    node.post_zero_safe_start = 0
    node.wheel_ticks = [WheelTickSample(10.0, 0, 0)]
    node.imu = [ImuSample(10.0, 0.0)]
    node.odom = [odom_sample(10.0)]
    nonzero = Twist()
    nonzero.angular.z = 0.2
    node.last_safe = nonzero
    node.safe_commands = [
        CommandSample(10.0, "/cmd_vel/safe", 0.0, 0.2, "after_motion")]

    with pytest.raises(ValidationError, match="command mismatch"):
        node.verify_controlled_stop_guards()


def test_controlled_stop_guard_rejects_graph_loss(monkeypatch):
    node = stationarity_node(monkeypatch)
    node.verify_graph_unchanged = lambda: (_ for _ in ()).throw(
        ValidationError("ROS graph changed unexpectedly"))

    with pytest.raises(ValidationError, match="ROS graph changed"):
        node.verify_controlled_stop_guards()


def test_controlled_stop_guard_rejects_diagnostic_fault(monkeypatch):
    node = stationarity_node(monkeypatch)
    node.verify_graph_unchanged = lambda: None
    node.first_zero_timestamp_s = 2.0
    node._now_seconds = lambda: 2.1
    node.post_zero_wheel_start = 0
    node.post_zero_imu_start = 0
    node.post_zero_odom_start = 0
    node.post_zero_safe_start = 0
    node.wheel_ticks = [WheelTickSample(2.1, 0, 0)]
    node.imu = [ImuSample(2.1, 0.0)]
    node.odom = [odom_sample(2.1)]
    node.safe_commands = [
        CommandSample(2.1, "/cmd_vel/safe", 0.0, 0.0, "after_motion")]
    node.last_diagnostic_level = 2

    with pytest.raises(
            ValidationError, match="diagnostics failure during controlled stop"):
        node.verify_controlled_stop_guards()


def test_stationarity_rejects_only_pre_zero_observations(monkeypatch):
    node = stationarity_node(monkeypatch)
    node.wheel_ticks = [
        WheelTickSample(timestamp, 500, -500, "during_motion")
        for timestamp in (1.0, 1.1, 1.2, 1.3)
    ]
    node.odom = [odom_sample(1.1), odom_sample(1.2), odom_sample(1.3)]
    node.safe_commands = [
        CommandSample(1.3, "/cmd_vel/safe", 0.0, 0.0, "during_motion")]
    node.post_zero_wheel_start = len(node.wheel_ticks)
    node.post_zero_odom_start = len(node.odom)
    node.post_zero_safe_start = len(node.safe_commands)

    assessment = node.verify_stationary()

    assert not assessment.stationary
    assert assessment.observed_delta_samples == 0
    assert "insufficient encoder deltas" in assessment.reason
    assert "insufficient odometry twist samples" in assessment.reason
    assert "no fresh /cmd_vel/safe sample" in assessment.reason


def test_delayed_pre_zero_callbacks_cannot_form_stationarity_window(monkeypatch):
    node = stationarity_node(monkeypatch)
    node.args.wheel_tick_semantics = "delta"
    node.first_zero_timestamp_s = 2.0
    node._now_seconds = lambda: 2.2
    node.post_zero_wheel_start = 0
    node.post_zero_odom_start = 0
    node.wheel_ticks = [
        WheelTickSample(timestamp, 0, 0)
        for timestamp in (1.6, 1.7, 1.8)
    ]
    node.odom = [
        odom_sample(timestamp) for timestamp in (1.6, 1.7, 1.8)]

    assessment = node.verify_stationary()

    assert not assessment.stationary
    assert assessment.observed_delta_samples == 0
    assert assessment.stationarity_samples == ()
    assert assessment.odom_samples == ()


def test_delayed_pre_zero_callback_does_not_refresh_stop_guard(monkeypatch):
    node = stationarity_node(monkeypatch)
    node.verify_graph_unchanged = lambda: None
    node.first_zero_timestamp_s = 2.0
    node._now_seconds = lambda: 3.1
    node.post_zero_wheel_start = 0
    node.post_zero_imu_start = 0
    node.post_zero_odom_start = 0
    node.post_zero_safe_start = 0
    node.wheel_ticks = [WheelTickSample(1.9, 0, 0)]
    node.imu = [ImuSample(3.0, 0.0)]
    node.odom = [odom_sample(3.0)]
    node.safe_commands = [
        CommandSample(3.0, "/cmd_vel/safe", 0.0, 0.0, "after_motion")]

    with pytest.raises(
            ValidationError, match="stale encoder during controlled stop"):
        node.verify_controlled_stop_guards()


def test_safe_zero_requires_a_fresh_post_zero_callback(monkeypatch):
    node = stationarity_node(monkeypatch)
    node.spin_for = lambda _duration: None
    node.post_zero_safe_start = len(node.safe_commands)

    assert not node.verify_safe_zero()

    node.safe_commands.append(
        CommandSample(1.1, "/cmd_vel/safe", 0.0, 0.0, "after_motion"))
    assert node.verify_safe_zero()


def test_safe_zero_freshness_is_reset_for_every_cleanup(monkeypatch):
    node = stationarity_node(monkeypatch)
    node.spin_for = lambda _duration: None

    node.begin_emergency_stop()
    node.prepare_emergency_stop_verification()
    node.safe_commands.append(
        CommandSample(1.1, "/cmd_vel/safe", 0.0, 0.0, "after_motion"))
    assert node.verify_safe_zero()

    node.begin_emergency_stop()
    node.prepare_emergency_stop_verification()
    assert not node.verify_safe_zero()


def test_emergency_stationarity_rejects_latched_safe_zero_after_stream_stales(
        monkeypatch):
    node = stationarity_node(monkeypatch)
    node.args.wheel_tick_semantics = "delta"
    node.args.stale_timeout_s = 0.5
    node.last_safe_time = 1.1
    node.safe_commands = [
        CommandSample(1.1, "/cmd_vel/safe", 0.0, 0.0, "after_motion")]
    node.wheel_ticks = [
        WheelTickSample(timestamp, 0, 0)
        for timestamp in (1.7, 1.8, 1.9)]
    node.odom = [odom_sample(timestamp) for timestamp in (1.7, 1.8, 1.9)]
    node._now_seconds = lambda: 1.9

    # The one post-zero acknowledgement remains useful for diagnosing that zero
    # propagated, but it cannot authorize cleanup success after the stream dies.
    assert node.verify_safe_zero()
    assessment = node.verify_stationary()

    assert not assessment.stationary
    assert not assessment.safe_zero
    assert not assessment.safe_command_fresh
    assert assessment.safe_command_age_s == pytest.approx(0.8)
    assert "/cmd_vel/safe sample became stale" in assessment.reason


def test_emergency_cleanup_cannot_combine_safe_latch_with_stale_safe_stream(
        monkeypatch):
    node = stationarity_node(monkeypatch)
    node.args.wheel_tick_semantics = "delta"
    node.args.stale_timeout_s = 0.5
    node.safe_commands = []
    node.last_safe = None
    node.last_safe_time = None
    node.wheel_ticks = []
    node.odom = []
    clock = [1.0]
    node._now_seconds = lambda: clock[0]

    def service_callbacks(duration_s):
        clock[0] += duration_s
        if not node.safe_commands:
            node.last_safe = Twist()
            node.last_safe_time = clock[0]
            node.safe_commands.append(CommandSample(
                clock[0], "/cmd_vel/safe", 0.0, 0.0, "after_motion"))
        moving = clock[0] < 1.7
        node.wheel_ticks.append(WheelTickSample(
            clock[0], 1 if moving else 0, -1 if moving else 0))
        node.odom.append(odom_sample(clock[0]))

    controller = EmergencyStopController(
        publish_zero=lambda: None,
        verify_safe_zero=node.verify_safe_zero,
        verify_stationary=node.verify_stationary,
        stationarity_required=lambda: True,
        sleep=service_callbacks,
        monotonic=lambda: clock[0])

    with pytest.raises(EmergencyStopError) as captured:
        controller.stop(
            timeout_s=1.0, rate_hz=10.0, mode="emergency_cleanup")
    record = captured.value.record

    assert record["safe_zero"]  # propagation was observed once and retained
    assert not record["stationary"]
    assert "sample became stale" in record["timeout_reason"]
    assert not record["stationarity_assessments"][-1]["safe_command_fresh"]


def test_execute_trial_eof_at_initial_heading_invokes_zero_cleanup():
    publishes = []
    fake_node = SimpleNamespace(
        reset_trial_buffers=lambda: None,
        verify_preflight=lambda: None,
        publish_zero=lambda: publishes.append("zero"),
        verify_safe_zero=lambda: True,
        verify_stationary=lambda: True)
    args = SimpleNamespace(
        dry_run=False,
        rotation_physical_mode="compass",
        rotation_angle_unit="deg",
        zero_publish_timeout_s=0.5,
        zero_publish_rate_hz=4.0,
        post_stop_settle_s=0.0,
        imu_bias_rad_s=0.0)
    operator_input = ResponsiveOperatorInput(
        prompt=lambda _prompt: (_ for _ in ()).throw(EOFError()),
        poll=lambda: None,
        notify=lambda _message: None,
        poll_interval_s=0.001)
    geometry = GeometryConfig(0.0881, 0.453, 4096)
    spec = TrialSpec("rot-001-ccw", "rotation", 0.3, 1.0, "ccw")

    with pytest.raises(EOFError):
        execute_trial(fake_node, args, geometry, spec, operator_input)

    assert publishes == ["zero"]


def test_execute_trial_dry_run_never_publishes_nonzero_command():
    publications = []
    fake_node = SimpleNamespace(
        reset_trial_buffers=lambda: None,
        verify_preflight=lambda: None,
        publish_zero=lambda: publications.append((0.0, 0.0)),
        verify_safe_zero=lambda: True,
        verify_stationary=lambda: True,
        get_logger=lambda: SimpleNamespace(warn=lambda _message: None),
        samples=lambda: TrialSamples())
    args = SimpleNamespace(
        dry_run=True,
        rotation_physical_mode="compass",
        rotation_angle_unit="deg",
        zero_publish_timeout_s=0.5,
        zero_publish_rate_hz=4.0,
        post_stop_settle_s=0.25,
        imu_bias_rad_s=0.0,
        wheel_tick_semantics="delta")
    operator_input = ResponsiveOperatorInput(
        prompt=lambda _prompt: pytest.fail("dry-run must not prompt for motion data"),
        poll=lambda: None,
        notify=lambda _message: None,
        poll_interval_s=0.001)
    geometry = GeometryConfig(0.0881, 0.453, 4096)
    spec = TrialSpec("rot-001-ccw", "rotation", 0.3, 1.0, "ccw")

    result, _samples = execute_trial(
        fake_node, args, geometry, spec, operator_input)

    assert result.measurements.commanded_angle_rad == pytest.approx(0.3)
    assert publications
    assert all(linear == 0.0 and angular == 0.0 for linear, angular in publications)


def test_execute_trial_dry_run_never_starts_or_polls_mocked_teledex():
    calls = []

    class MockTeleDex:
        def begin_trial(self, _timestamp):
            calls.append("begin")

        def poll_required(self, _purpose):
            calls.append("poll")

        def finalize_after_stationarity(self, _timestamp):
            calls.append("finalize")

    publications = []
    fake_node = SimpleNamespace(
        reset_trial_buffers=lambda: None,
        verify_preflight=lambda: None,
        publish_zero=lambda: publications.append((0.0, 0.0)),
        verify_safe_zero=lambda: True,
        verify_stationary=lambda: True,
        get_logger=lambda: SimpleNamespace(warn=lambda _message: None),
        samples=lambda: TrialSamples())
    args = SimpleNamespace(
        dry_run=True,
        rotation_physical_mode="compass",
        rotation_angle_unit="deg",
        zero_publish_timeout_s=0.5,
        zero_publish_rate_hz=4.0,
        post_stop_settle_s=0.25,
        imu_bias_rad_s=0.0,
        wheel_tick_semantics="delta")
    operator_input = ResponsiveOperatorInput(
        prompt=lambda _prompt: pytest.fail("dry-run must not prompt"),
        poll=lambda: None,
        notify=lambda _message: None,
        poll_interval_s=0.001)

    execute_trial(
        fake_node, args, GeometryConfig(0.0881, 0.453, 4096),
        TrialSpec("trans-001-forward", "translation", 0.1, 2.0, "forward"),
        operator_input, teledex=MockTeleDex())

    assert calls == []
    assert publications
    assert all(linear == 0.0 and angular == 0.0 for linear, angular in publications)


def test_preflight_failure_before_motion_skips_stationarity_cleanup():
    publishes = []
    spins = []
    stationarity_calls = []
    records = []
    fake_node = SimpleNamespace(
        reset_trial_buffers=lambda: None,
        verify_preflight=lambda: (_ for _ in ()).throw(
            ValidationError("encoder messages are not available")),
        publish_zero=lambda: publishes.append("zero"),
        verify_safe_zero=lambda: True,
        verify_stationary=lambda: stationarity_calls.append("called"),
        stationarity_required=lambda: False,
        record_emergency_stop=records.append,
        spin_for=spins.append,
        samples=lambda: TrialSamples())
    args = SimpleNamespace(
        dry_run=False,
        rotation_physical_mode="compass",
        rotation_angle_unit="deg",
        zero_publish_timeout_s=0.5,
        zero_publish_rate_hz=4.0,
        post_stop_settle_s=0.0,
        imu_bias_rad_s=0.0)
    operator_input = ResponsiveOperatorInput(
        prompt=lambda _prompt: "0",
        poll=lambda: None,
        notify=lambda _message: None,
        poll_interval_s=0.001)
    geometry = GeometryConfig(0.0881, 0.453, 4096)
    spec = TrialSpec("rot-001-ccw", "rotation", 0.3, 1.0, "ccw")

    with pytest.raises(
            ValidationError, match="encoder messages are not available"):
        execute_trial(fake_node, args, geometry, spec, operator_input)

    assert publishes == ["zero"]
    assert spins == [0.25]
    assert stationarity_calls == []
    assert records[-1]["safe_zero"]
    assert not records[-1]["stationarity_required"]


@pytest.mark.parametrize(("failure_phase",), (("before_motion",), ("during_motion",)))
def test_teledex_failure_invokes_existing_zero_cleanup_exactly_once(failure_phase):
    publishes = []

    class FakeTeleDex:
        def begin_trial(self, _timestamp):
            if failure_phase == "before_motion":
                raise ValidationError("TeleDex initial reference is invalid")

        def poll_required(self, _purpose):
            raise ValidationError("TeleDex trajectory is invalid")

    def run_command_phase(_spec, observation_hook):
        observation_hook()

    fake_node = SimpleNamespace(
        reset_trial_buffers=lambda: None,
        verify_preflight=lambda: None,
        _now_seconds=lambda: 1.0,
        run_command_phase=run_command_phase,
        publish_zero=lambda: publishes.append("zero"),
        verify_safe_zero=lambda: True,
        verify_stationary=lambda: True,
        stationarity_required=lambda: False,
        spin_for=lambda _duration: None,
        samples=lambda: TrialSamples())
    args = SimpleNamespace(
        dry_run=False,
        rotation_physical_mode="compass",
        rotation_angle_unit="deg",
        zero_publish_timeout_s=0.5,
        zero_publish_rate_hz=4.0,
        post_stop_settle_s=0.25,
        imu_bias_rad_s=0.0,
        wheel_tick_semantics="delta")
    operator_input = ResponsiveOperatorInput(
        prompt=lambda _prompt: pytest.fail("TeleDex mode must not prompt for compass input"),
        poll=lambda: None, notify=lambda _message: None, poll_interval_s=0.001)

    with pytest.raises(ValidationError, match="TeleDex"):
        execute_trial(
            fake_node, args, GeometryConfig(0.0881, 0.453, 4096),
            TrialSpec("rot-001-ccw", "rotation", 0.3, 1.0, "ccw"),
            operator_input, teledex=FakeTeleDex())

    assert publishes == ["zero"]
