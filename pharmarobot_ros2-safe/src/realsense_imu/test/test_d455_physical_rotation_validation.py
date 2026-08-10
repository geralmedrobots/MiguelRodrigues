import builtins
import importlib.util
import inspect
import json
import math
from pathlib import Path
import sys

import pytest

_PATH = (
    Path(__file__).parents[1]
    / "tools"
    / "d455_physical_rotation_validation.py"
)
_SPEC = importlib.util.spec_from_file_location("d455_rotation", _PATH)
rotation = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = rotation
_SPEC.loader.exec_module(rotation)


def endpoint(node):
    return rotation.GraphEndpoint(node=node, qos="best_effort/volatile")


def valid_preflight(**overrides):
    values = {
        "production_containers": ("pharmarobot_d455_sensor",),
        "validation_containers": (),
        "foreign_d455_owners": (),
        "apparmor_profile": "pharmarobot-d455-imu",
        "apparmor_enforcing": True,
        "sensor_container_id": "c" * 64,
        "sensor_image_id": "sha256:" + "d" * 64,
        "immutable_config_sha256": "e" * 64,
        "raw_publishers": (endpoint("realsense_imu_relay"),),
        "processed_publishers": (endpoint("d455_imu_processor"),),
        "diagnostics_publishers": (
            rotation.GraphEndpoint(
                "d455_imu_processor",
                rotation.RELIABLE_QOS,
            ),
        ),
        "safe_publishers": (
            rotation.GraphEndpoint(
                "command_arbiter",
                rotation.RELIABLE_QOS,
            ),
        ),
        "input_publishers": (),
        "input_subscribers": (
            rotation.GraphEndpoint(
                "command_arbiter",
                rotation.RELIABLE_QOS,
            ),
        ),
        "safe_subscribers": (
            endpoint("d455_imu_processor"),
            rotation.GraphEndpoint(
                "roboteq_ros2_driver",
                rotation.RELIABLE_QOS,
            ),
            rotation.GraphEndpoint(
                "d455_rotation_validator",
                rotation.RELIABLE_QOS,
            ),
        ),
        "relay_nodes": ("realsense_imu_relay",),
        "processor_nodes": ("d455_imu_processor",),
        "main_container_id": "f" * 64,
        "main_immutable_config_sha256": "a" * 64,
    }
    values.update(overrides)
    return rotation.PreflightSnapshot(**values)


def valid_host_snapshot(**overrides):
    source = valid_preflight()
    values = {
        field: getattr(source, field)
        for field in rotation.HostPreflightSnapshot.__dataclass_fields__
    }
    values.update(overrides)
    return rotation.HostPreflightSnapshot(**values)


def valid_ros_snapshot(**overrides):
    source = valid_preflight()
    values = {
        field: getattr(source, field)
        for field in rotation.RosPreflightSnapshot.__dataclass_fields__
    }
    values.update(overrides)
    return rotation.RosPreflightSnapshot(**values)


class FakeRuntime:
    def __init__(
        self,
        *,
        snapshot=None,
        residual=False,
        stationary=True,
        safe_mismatch=False,
        publish_delay=0.0,
        create_failure=False,
        wrong_imu_sign=False,
        stationary_wheel=False,
        wrong_wheel_sign=False,
        asymmetric_wheel=False,
        missing_odometry=False,
        authorization_failure_after=None,
    ):
        self.now = 0.0
        self.snapshot = snapshot or valid_preflight()
        self.residual = residual
        self.is_stationary = stationary
        self.safe_mismatch = safe_mismatch
        self.publish_delay = publish_delay
        self.create_failure = create_failure
        self.wrong_imu_sign = wrong_imu_sign
        self.stationary_wheel = stationary_wheel
        self.wrong_wheel_sign = wrong_wheel_sign
        self.asymmetric_wheel = asymmetric_wheel
        self.missing_odometry = missing_odometry
        self.authorization_failure_after = authorization_failure_after
        self.authorization_checks = 0
        self.created = 0
        self.closed = 0
        self.published = []
        self.current = rotation.twist(0.0)
        self.identity_checks = 0

    def monotonic(self):
        return self.now

    def wall_time_ns(self):
        return int(self.now * 1e9)

    def sleep(self, duration_s):
        self.now += duration_s

    def preflight_snapshot(self):
        return self.snapshot

    def create_publisher(self):
        self.created += 1
        if self.create_failure:
            raise RuntimeError("partial publisher creation")

    def assert_motion_authorized(self):
        self.authorization_checks += 1
        if (
            self.authorization_failure_after is not None
            and self.authorization_checks
            > self.authorization_failure_after
        ):
            raise rotation.ValidationError("host heartbeat is stale")

    def publish(self, payload):
        self.published.append((self.now, payload))
        self.current = payload
        self.now += self.publish_delay

    def verify_runtime_identity(self, expected):
        assert expected == self.snapshot
        self.identity_checks += 1

    def observe_safe(self, newer_than_s, timeout_s):
        del timeout_s
        payload = self.current
        if self.safe_mismatch and (
            payload["angular"]["z"] != 0.0
            or payload["linear"]["x"] != 0.0
        ):
            payload = rotation.twist(0.0)
        if self.residual and payload["angular"]["z"] == 0.0:
            payload = rotation.twist(0.01)
        return rotation.TimedTwist(
            max(self.now, newer_than_s),
            self.wall_time_ns(),
            payload,
            "safe",
            None,
        )

    def stationary(self, newer_than_s, timeout_s):
        del newer_than_s, timeout_s
        return self.is_stationary

    def capture_trial(self, trial, start_s, end_s):
        middle = (start_s + end_s) / 2.0
        measured_z = (
            -trial.angular_z
            if self.wrong_imu_sign
            else trial.angular_z
        )
        if trial.command_type == "straight_line":
            measured_z = 0.0
        covariance = (0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01)
        unavailable = (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        raw = [
            rotation.ImuSample(
                stamp,
                int(stamp * 1e9),
                rotation.EXPECTED_RAW_FRAME,
                (0.0, measured_z, 0.0),
                (0.0, 0.0, 9.81),
                unavailable,
                covariance,
                covariance,
            )
            for stamp in (start_s, middle, end_s)
        ]
        processed = [
            rotation.ImuSample(
                stamp,
                int(stamp * 1e9),
                rotation.EXPECTED_PROCESSED_FRAME,
                (0.0, 0.0, measured_z),
                (0.0, 0.0, 9.81),
                unavailable,
                covariance,
                covariance,
            )
            for stamp in (start_s, middle, end_s)
        ]
        diagnostics = [
            {"name": "D455 IMU/Raw Input", "level": 0},
            {"name": "D455 IMU/Processed Output", "level": 0},
            {"name": "D455 IMU/Bias", "level": 0},
            {"name": "D455 IMU/Transform", "level": 1},
            {"name": "D455 IMU/Covariance", "level": 1},
        ]
        if trial.command_type == "straight_line":
            sign = 1 if trial.linear_x >= 0.0 else -1
            if self.wrong_wheel_sign:
                sign *= -1
            left_end = 0 if self.stationary_wheel else sign * 100
            right_end = sign * (250 if self.asymmetric_wheel else 102)
        else:
            turn_sign = 1 if trial.angular_z >= 0.0 else -1
            left_end = 0 if self.stationary_wheel else -turn_sign * 100
            right_end = turn_sign * 102
        odometry = []
        if trial.command_type == "straight_line" and not self.missing_odometry:
            odometry = [
                {
                    "monotonic_s": start_s,
                    "x_m": 0.0,
                    "y_m": 0.0,
                    "yaw_rad": 0.0,
                },
                {
                    "monotonic_s": end_s,
                    "x_m": trial.linear_x * trial.duration_s,
                    "y_m": 0.0,
                    "yaw_rad": 0.0,
                },
            ]
        return {
            "safe": [],
            "raw_imu": raw,
            "processed_imu": processed,
            "diagnostics": diagnostics,
            "encoder": [
                {
                    "monotonic_s": start_s,
                    "message": "left_ticks=0\nright_ticks=0",
                },
                {
                    "monotonic_s": end_s,
                    "message": (
                        f"left_ticks={left_end}\nright_ticks={right_end}"
                    ),
                },
            ],
            "odometry": odometry,
        }

    def close(self):
        self.closed += 1


def make_campaign(tmp_path, runtime, rate_hz=20.0):
    plan = rotation.plan_payload(
        rotation.build_matrix([0.1], [0.1], 1),
        rate_hz,
    )
    writer = rotation.EvidenceWriter(tmp_path / "evidence", plan, False)
    return rotation.Campaign(
        runtime,
        writer,
        rate_hz=rate_hz,
        settle_timeout_s=1.0,
        zero_duration_s=0.15,
    )


def test_default_matrix_and_sign_mapping_are_exact():
    trials = rotation.build_matrix()
    assert len(trials) == 24
    assert {abs(item.angular_z) for item in trials} == {0.1, 0.2, 0.3}
    assert {item.duration_s for item in trials} == {2.0, 4.0}
    assert rotation.build_matrix([0.1], [2.0], 1)[0].angular_z == -0.1
    assert rotation.build_matrix([0.1], [2.0], 1)[1].angular_z == 0.1


@pytest.mark.parametrize(
    "speed", [0.0, 0.91, 1.00, float("nan"), float("inf")]
)
def test_unsafe_speed_rejected(speed):
    with pytest.raises(ValueError):
        rotation.build_matrix([speed], [2.0], 1)


def test_duplicate_and_unbounded_matrix_values_rejected():
    with pytest.raises(ValueError, match="duplicate"):
        rotation.build_matrix([0.1, 0.1], [2.0], 1)
    with pytest.raises(ValueError, match="repetitions"):
        rotation.build_matrix([0.1], [2.0], 3)


@pytest.mark.parametrize("speed", [0.50, 0.75])
def test_configurable_angular_velocity_cli_builds_matched_pair(
    tmp_path, speed
):
    result = rotation.main(
        [
            "--angular-velocity",
            str(speed),
            "--durations",
            "2.0",
            "--repetitions",
            "1",
            "--evidence-dir",
            str(tmp_path / "evidence"),
        ]
    )
    assert result == 0
    plan = json.loads(
        (tmp_path / "evidence" / "rotation-plan.json").read_text()
    )
    assert plan["requested_angular_velocities_rad_s"] == [speed]
    assert [row["angular_z"] for row in plan["matrix"]] == [-speed, speed]


def test_configurable_angular_velocity_rejects_speeds_conflict(tmp_path):
    with pytest.raises(rotation.ValidationError, match="cannot be combined"):
        rotation.main(
            [
                "--angular-velocity", "0.5", "--speeds", "0.1",
                "--evidence-dir", str(tmp_path / "evidence"),
            ]
        )


def test_linear_velocity_cli_builds_forward_backward_pair(tmp_path):
    result = rotation.main(
        [
            "--linear-velocity",
            "0.05",
            "--durations",
            "3.0",
            "--repetitions",
            "1",
            "--evidence-dir",
            str(tmp_path / "evidence"),
        ]
    )
    assert result == 0
    plan = json.loads(
        (tmp_path / "evidence" / "rotation-plan.json").read_text()
    )
    assert plan["command_types"] == ["straight_line"]
    assert plan["requested_linear_velocities_m_s"] == [0.05]
    assert [row["linear_x"] for row in plan["matrix"]] == [0.05, -0.05]
    assert [row["angular_z"] for row in plan["matrix"]] == [0.0, 0.0]


@pytest.mark.parametrize(
    "speed",
    [0.0, -0.1, 1.01, float("nan"), float("inf")],
)
def test_linear_velocity_rejects_unsafe_values(speed):
    with pytest.raises(ValueError):
        rotation.build_linear_matrix([speed], [3.0], 1)


def test_linear_velocity_rejects_rotation_speed_options(tmp_path):
    with pytest.raises(rotation.ValidationError, match="linear-velocity"):
        rotation.main(
            [
                "--linear-velocity",
                "0.05",
                "--speeds",
                "0.1",
                "--evidence-dir",
                str(tmp_path / "evidence"),
            ]
        )


def test_wheel_symmetry_reports_numeric_feedback():
    result = rotation.wheel_symmetry(
        [
            {"message": "left_ticks=10\nright_ticks=-10"},
            {"message": "left_ticks=30\nright_ticks=-31"},
        ]
    )
    assert result["available"] is True
    assert result["equal_magnitude"] is True
    assert result["opposite_direction"] is True


def test_wheel_symmetry_detects_same_direction_feedback():
    result = rotation.wheel_symmetry(
        [
            {"message": "left_ticks: 10\nright_ticks: 10"},
            {"message": "left_ticks: 30\nright_ticks: 31"},
        ]
    )
    assert result["available"] is True
    assert result["opposite_direction"] is False


def test_straight_line_trial_records_zero_angular_and_wheel_analysis(tmp_path):
    runtime = FakeRuntime()
    trials = rotation.build_linear_matrix([0.05], [0.1], 1)
    plan = rotation.plan_payload(trials, 20.0)
    writer = rotation.EvidenceWriter(tmp_path / "linear", plan, False)
    campaign = rotation.Campaign(
        runtime,
        writer,
        rate_hz=20.0,
        settle_timeout_s=1.0,
        zero_duration_s=0.15,
    )
    evidence = campaign.run(trials)[0]
    assert evidence.result == "passed"
    assert (
        evidence.analysis["command_interval"][
            "commanded_linear_velocity_m_s"
        ]
        == 0.05
    )
    assert (
        evidence.analysis["command_interval"][
            "commanded_angular_velocity_rad_s"
        ]
        == 0.0
    )
    assert evidence.analysis["wheel_symmetry"]["same_direction"] is True
    assert (
        evidence.analysis["wheel_symmetry"]["left_right_movement_ratio"]
        > 0.9
    )
    assert all(
        payload["angular"]["z"] == 0.0
        for _stamp, payload in runtime.published
    )
    assert any(
        payload["linear"]["x"] == 0.05
        for _stamp, payload in runtime.published
    )


def test_straight_line_stationary_wheel_aborts_pair(tmp_path):
    runtime = FakeRuntime(stationary_wheel=True)
    trials = rotation.build_linear_matrix([0.05], [0.1], 1)
    plan = rotation.plan_payload(trials, 20.0)
    writer = rotation.EvidenceWriter(tmp_path / "linear-fail", plan, False)
    campaign = rotation.Campaign(
        runtime,
        writer,
        rate_hz=20.0,
        settle_timeout_s=1.0,
        zero_duration_s=0.15,
    )
    with pytest.raises(rotation.ValidationError, match="stationary"):
        campaign.run(trials)
    motion_values = [
        payload["linear"]["x"]
        for _stamp, payload in runtime.published
        if payload["linear"]["x"] != 0.0
    ]
    assert set(motion_values) == {0.05}
    assert runtime.published[-1][1] == rotation.twist(0.0)


@pytest.mark.parametrize(
    "runtime_kwargs, message",
    [
        ({"wrong_wheel_sign": True}, "wheel encoder sign"),
        ({"asymmetric_wheel": True}, "asymmetric"),
        ({"missing_odometry": True}, "odometry evidence"),
    ],
)
def test_straight_line_feedback_failures_abort_pair(
    tmp_path,
    runtime_kwargs,
    message,
):
    runtime = FakeRuntime(**runtime_kwargs)
    trials = rotation.build_linear_matrix([0.05], [0.1], 1)
    plan = rotation.plan_payload(trials, 20.0)
    writer = rotation.EvidenceWriter(tmp_path / message, plan, False)
    campaign = rotation.Campaign(
        runtime,
        writer,
        rate_hz=20.0,
        settle_timeout_s=1.0,
        zero_duration_s=0.15,
    )
    with pytest.raises(rotation.ValidationError, match=message):
        campaign.run(trials)
    motion_values = [
        payload["linear"]["x"]
        for _stamp, payload in runtime.published
        if payload["linear"]["x"] != 0.0
    ]
    assert set(motion_values) == {0.05}
    assert runtime.published[-1][1] == rotation.twist(0.0)


@pytest.mark.parametrize(
    "override, message",
    [
        (
            {"raw_publishers": (endpoint("one"), endpoint("two"))},
            "raw publisher",
        ),
        ({"validation_containers": ("validator",)}, "validation container"),
        ({"foreign_d455_owners": ("foreign",)}, "foreign D455"),
        ({"diagnostics_publishers": ()}, "diagnostics publisher"),
        ({"apparmor_enforcing": False}, "AppArmor"),
        (
            {"input_publishers": (endpoint("competing_source"),)},
            "pre-existing command-input publisher",
        ),
        ({"input_subscribers": ()}, "command-arbiter input"),
    ],
)
def test_ownership_and_graph_fail_closed(override, message):
    with pytest.raises(rotation.ValidationError, match=message):
        rotation.validate_preflight(valid_preflight(**override))


def test_duplicate_validator_safe_subscriber_is_rejected():
    snapshot = valid_preflight()
    duplicate = snapshot.safe_subscribers + (
        rotation.GraphEndpoint(
            "d455_rotation_validator",
            rotation.RELIABLE_QOS,
            "duplicate",
        ),
    )
    with pytest.raises(rotation.ValidationError, match="safe-command"):
        rotation.validate_preflight(
            valid_preflight(safe_subscribers=duplicate)
        )


def test_continuous_publication_count_frequency_and_zero_boundaries(tmp_path):
    runtime = FakeRuntime()
    campaign = make_campaign(tmp_path, runtime)
    trial = rotation.build_matrix([0.1], [0.1], 1)[0]
    completed = campaign.run([trial])
    motion = [
        (stamp, payload)
        for stamp, payload in runtime.published
        if payload["angular"]["z"] != 0.0
    ]
    assert len(motion) == 2
    assert math.isclose(motion[1][0] - motion[0][0], 0.05)
    first_motion = runtime.published.index(motion[0])
    assert runtime.published[first_motion - 1][1] == rotation.twist(0.0)
    assert runtime.published[-1][1] == rotation.twist(0.0)
    assert completed[0].zero_verified
    assert runtime.closed == 1
    assert runtime.identity_checks >= 3


def test_publish_count_scales_with_duration(tmp_path):
    runtime = FakeRuntime()
    campaign = make_campaign(tmp_path, runtime, rate_hz=20.0)
    records = []
    campaign._publish_for(rotation.twist(0.1), 0.25, "motion", 0, records)
    assert len(records) == 5
    assert math.isclose(records[-1].monotonic_s - records[0].monotonic_s, 0.2)


def test_missed_publication_deadline_fails_without_catchup(tmp_path):
    runtime = FakeRuntime(publish_delay=0.08)
    campaign = make_campaign(tmp_path, runtime, rate_hz=20.0)
    records = []
    with pytest.raises(rotation.ValidationError, match="deadline"):
        campaign._publish_for(
            rotation.twist(0.1),
            0.25,
            "motion",
            0,
            records,
        )
    assert len(records) == 1


def test_heartbeat_guard_accepts_fresh_and_rejects_stale():
    now = [1_000_000_000]
    guard = rotation.HeartbeatGuard(
        "a" * 64,
        0.5,
        wall_time_ns=lambda: now[0],
    )
    guard.accept(
        json.dumps(
            {
                "token": "a" * 64,
                "wall_time_ns": now[0],
            }
        )
    )
    guard.check()
    now[0] += 500_000_001
    with pytest.raises(rotation.ValidationError, match="stale"):
        guard.check()


def test_subprocess_supervisor_streams_bound_heartbeat():
    token = "b" * 64
    code = (
        "import json,sys; "
        "row=json.loads(sys.stdin.readline()); "
        "assert row['token']==sys.argv[1]; "
        "print('heartbeat-ok')"
    )
    result = rotation.SubprocessRunner().run_supervised(
        [sys.executable, "-c", code, token],
        token,
    )
    assert result.returncode == 0
    assert result.stdout.strip() == "heartbeat-ok"


def test_subprocess_supervisor_collects_normal_broken_pipe(monkeypatch):
    class BrokenStdin:
        def write(self, _value):
            raise BrokenPipeError()

        def flush(self):
            raise AssertionError("flush must not follow broken write")

        def close(self):
            return None

    class CompletedProcess:
        stdin = BrokenStdin()
        returncode = 0

        def __init__(self):
            self.polls = 0

        def poll(self):
            self.polls += 1
            return None if self.polls == 1 else 0

        def communicate(self):
            return "completed\n", ""

    monkeypatch.setattr(
        rotation.subprocess,
        "Popen",
        lambda *_args, **_kwargs: CompletedProcess(),
    )
    result = rotation.SubprocessRunner().run_supervised(
        ["synthetic-worker"],
        "c" * 64,
    )
    assert result.returncode == 0
    assert result.stdout == "completed\n"


def test_lost_heartbeat_aborts_motion_and_preserves_zero_cleanup(tmp_path):
    runtime = FakeRuntime(authorization_failure_after=1)
    campaign = make_campaign(tmp_path, runtime)
    with pytest.raises(rotation.ValidationError, match="heartbeat"):
        campaign.run(rotation.build_matrix([0.1], [0.1], 1))
    assert runtime.published[-1][1] == rotation.twist(0.0)
    assert all(
        payload == rotation.twist(0.0)
        for _, payload in runtime.published[-3:]
    )


def test_safe_mismatch_during_motion_aborts_trial(tmp_path):
    runtime = FakeRuntime(safe_mismatch=True)
    campaign = make_campaign(tmp_path, runtime)
    with pytest.raises(rotation.ValidationError, match="safe"):
        campaign.run(rotation.build_matrix([0.1], [0.1], 1))
    assert runtime.closed == 1
    assert runtime.published[-1][1] == rotation.twist(0.0)


def test_stop_timeout_residual_aborts_and_closes(tmp_path):
    runtime = FakeRuntime(residual=True)
    campaign = make_campaign(tmp_path, runtime)
    with pytest.raises(rotation.ValidationError, match="timed out"):
        campaign.run(rotation.build_matrix([0.1], [0.1], 1))
    assert runtime.closed == 1
    assert all(
        payload["angular"]["z"] == 0.0
        for _, payload in runtime.published
    )


def test_stationary_timeout_aborts_remaining_trials(tmp_path):
    runtime = FakeRuntime(stationary=False)
    campaign = make_campaign(tmp_path, runtime)
    with pytest.raises(rotation.ValidationError, match="stationary"):
        campaign.run(rotation.build_matrix([0.1], [0.1], 1))
    assert not any(
        payload["angular"]["z"] != 0.0
        for _, payload in runtime.published
    )


def test_wrong_imu_sign_fails_and_stops_remaining_matrix(tmp_path):
    runtime = FakeRuntime(wrong_imu_sign=True)
    campaign = make_campaign(tmp_path, runtime)
    trials = rotation.build_matrix([0.1], [0.1], 1)
    with pytest.raises(rotation.ValidationError, match="sign"):
        campaign.run(trials)
    motion_values = [
        payload["angular"]["z"]
        for _, payload in runtime.published
        if payload["angular"]["z"] != 0.0
    ]
    assert set(motion_values) == {-0.1}
    assert campaign.completed[0].result == "failed"


def test_preflight_and_partial_publisher_failure_close_runtime(tmp_path):
    invalid = valid_preflight(apparmor_enforcing=False)
    runtime = FakeRuntime(snapshot=invalid)
    campaign = make_campaign(tmp_path, runtime)
    with pytest.raises(rotation.ValidationError, match="AppArmor"):
        campaign.run(rotation.build_matrix([0.1], [0.1], 1))
    assert runtime.created == 0
    assert runtime.closed == 1

    partial = FakeRuntime(create_failure=True)
    other = make_campaign(tmp_path / "partial", partial)
    with pytest.raises(RuntimeError, match="partial publisher"):
        other.run(rotation.build_matrix([0.1], [0.1], 1))
    assert partial.closed == 1
    assert partial.published[-1][1] == rotation.twist(0.0)


def test_evidence_json_csv_diagnostics_encoder_and_correlation(tmp_path):
    runtime = FakeRuntime()
    campaign = make_campaign(tmp_path, runtime)
    trial = rotation.build_matrix([0.1], [0.1], 1)[0]
    evidence = campaign.run([trial])[0]
    campaign.writer.finish("passed", runtime.snapshot, [evidence])
    directory = tmp_path / "evidence"
    payload = json.loads((directory / "trial-00.json").read_text())
    summary = json.loads((directory / "summary.json").read_text())
    assert payload["diagnostics"][0]["name"] == "D455 IMU/Raw Input"
    assert "left_ticks=0" in payload["encoder"][0]["message"]
    assert payload["command_samples"][0]["monotonic_s"] is not None
    assert payload["analysis"]["processed_imu"]["integrated_yaw_rad"] < 0.0
    assert (directory / "trial-00-commands.csv").read_text().startswith(
        "monotonic_s,wall_time_ns"
    )
    assert (directory / "trial-00-safe.csv").exists()
    assert (directory / "trial-00-raw-imu.csv").exists()
    assert (directory / "trial-00-processed-imu.csv").exists()
    assert summary["preflight"]["sensor_container_id"] == "c" * 64
    assert summary["plan_sha256"]
    assert (directory / "events.jsonl").read_text()


def test_integration_and_timestamp_validation():
    result = rotation.integrate_gyro([(0.0, 0.2), (0.5, 0.2), (1.0, 0.2)])
    assert math.isclose(result["integrated_yaw_rad"], 0.2)
    with pytest.raises(ValueError, match="monotonic"):
        rotation.integrate_gyro([(0.0, 0.1), (0.0, 0.1)])
    with pytest.raises(ValueError, match="non-finite"):
        rotation.integrate_gyro([(0.0, float("nan")), (1.0, 0.1)])


def test_dry_run_creates_no_runtime_or_publisher(tmp_path, capsys):
    called = []
    assert rotation.main(
        ["--evidence-dir", str(tmp_path / "dry")],
        runtime_factory=lambda: called.append(True),
    ) == 0
    assert called == []
    assert json.loads(capsys.readouterr().out)["dry_run"] is True


def test_missing_flag_and_confirmation_abort_before_runtime(tmp_path):
    called = []
    with pytest.raises(rotation.ValidationError, match="confirmation"):
        rotation.main(
            [
                "--enable-motion",
                "--evidence-dir",
                str(tmp_path / "motion"),
                "--speeds",
                "0.1",
                "--durations",
                "0.1",
                "--repetitions",
                "1",
            ],
            input_fn=lambda _prompt: "NO",
            runtime_factory=lambda: called.append(True),
        )
    assert called == []
    with pytest.raises(rotation.ValidationError, match="enable-motion"):
        rotation.require_operator_approval(False, lambda _prompt: "ROTATE")


def test_exact_confirmation_creates_runtime_after_prompt(tmp_path):
    runtime = FakeRuntime()
    prompts = []
    rotation.require_operator_approval(
        True,
        lambda prompt: prompts.append(prompt) or "ROTATE",
    )
    assert prompts and "ROTATE" in prompts[0]
    assert runtime.created == 0
    assert rotation.RosWorkerRuntime


@pytest.mark.parametrize("response", ["ROTATE\n", "ROTATE\r", " ROTATE\t"])
def test_confirmation_accepts_terminal_line_endings(response):
    rotation.require_operator_approval(True, lambda _prompt: response)


@pytest.mark.parametrize("response", ["ROTATE NOW", "RO TATE", "ROTATE\x00"])
def test_confirmation_rejects_non_exact_normalized_token(response):
    with pytest.raises(rotation.ValidationError, match="confirmation"):
        rotation.require_operator_approval(True, lambda _prompt: response)


def test_humble_byte_diagnostic_level_is_supported():
    runtime = object.__new__(rotation.RosWorkerRuntime)
    runtime.diagnostics = []
    runtime.monotonic = lambda: 1.0

    class Status:
        name = "D455 IMU/Bias"
        level = b"\x00"
        message = "calibrated"
        values = ()

    class Message:
        status = (Status(),)

    runtime._diagnostic_callback(Message())
    assert runtime.diagnostics[0]["level"] == 0


def test_self_observer_is_part_of_expected_safe_graph():
    rotation.validate_preflight(valid_preflight())


def test_successful_confirmation_runs_synthetic_campaign(tmp_path):
    runtime = FakeRuntime()
    result = rotation.main(
        [
            "--enable-motion",
            "--evidence-dir",
            str(tmp_path / "approved"),
            "--speeds",
            "0.1",
            "--durations",
            "0.1",
            "--repetitions",
            "1",
        ],
        input_fn=lambda _prompt: "ROTATE",
        runtime_factory=lambda: runtime,
    )
    assert result == 0
    assert runtime.created == 1
    summary = json.loads(
        (tmp_path / "approved" / "summary.json").read_text()
    )
    assert summary["result"] == "passed"
    assert summary["cleanup"]["final_zero_verified"] is True
    assert summary["cleanup"]["command_samples"]


def staged_worker(tmp_path):
    worker_hash = rotation.file_sha256(_PATH)
    path = (
        tmp_path
        / f"{rotation.WORKER_SOURCE_PREFIX}{worker_hash}.py"
    )
    path.write_bytes(_PATH.read_bytes())
    return path


def handoff_payload(*, approved, worker_path):
    plan = rotation.plan_payload(
        rotation.build_matrix([0.1], [0.1], 1),
        20.0,
    )
    host = rotation.host_snapshot_payload(valid_host_snapshot())
    ros = rotation.ros_snapshot_payload(valid_ros_snapshot())
    payload = {
        "schema": rotation.HANDOFF_SCHEMA,
        "worker_container": rotation.WORKER_CONTAINER,
        "worker_source_path": str(worker_path),
        "worker_sha256": rotation.file_sha256(worker_path),
        "plan": plan,
        "plan_sha256": plan["plan_sha256"],
        "host_snapshot": host,
        "host_snapshot_sha256": rotation.payload_sha256(host),
        "evidence_relative": (
            "src/realsense_imu/validation_evidence/synthetic-split"
        ),
        "worker_evidence_path": str(
            worker_path.parent
            / (
                rotation.WORKER_EVIDENCE_PREFIX
                + rotation.payload_sha256(plan)
            )
        ),
        "heartbeat_token": "a" * 64,
        "heartbeat_max_age_s": rotation.HEARTBEAT_MAX_AGE_S,
    }
    if approved:
        payload["ros_snapshot"] = ros
        payload["ros_snapshot_sha256"] = rotation.payload_sha256(ros)
        payload["approval"] = {
            "text": rotation.APPROVAL_TEXT,
            "binding_sha256": rotation.payload_sha256(
                rotation.approval_binding_payload(payload)
            ),
        }
    return payload


class FakeWorkerRuntime(FakeRuntime):
    def __init__(
        self,
        host_snapshot,
        approved_graph=None,
        heartbeat_guard=None,
    ):
        self.ros_snapshot = valid_ros_snapshot()
        assert approved_graph in (
            None,
            rotation.approval_graph_identity(self.ros_snapshot),
        )
        self.heartbeat_guard = heartbeat_guard
        super().__init__(
            snapshot=rotation.compose_preflight(
                host_snapshot,
                self.ros_snapshot,
            )
        )

    def ros_preflight_snapshot(self):
        return self.ros_snapshot


class FakeContainerRunner:
    def __init__(self, evidence_root):
        self.evidence_root = evidence_root
        self.calls = []

    def _completion(self, result="passed"):
        handoff = json.loads(
            (
                self.evidence_root
                / "host-worker-handoff.json"
            ).read_text()
        )
        return {
            "result": result,
            "error": "",
            "plan_sha256": handoff["plan_sha256"],
            "approval_binding_sha256": handoff[
                "approval"
            ]["binding_sha256"],
        }

    def run(self, args):
        self.calls.append(tuple(args))
        command = " ".join(args)
        if "--worker-preflight" in command:
            payload = {
                "result": "preflight_passed",
                "worker_sha256": rotation.file_sha256(_PATH),
                "ros_snapshot": rotation.ros_snapshot_payload(
                    valid_ros_snapshot()
                ),
            }
            return rotation.CommandResult(
                0,
                rotation.WORKER_RESULT_PREFIX
                + rotation.canonical_json(payload)
                + "\n",
                "",
            )
        if (
            len(args) >= 4
            and args[:2] == ["docker", "cp"]
            and args[2].startswith(
                f"{rotation.WORKER_CONTAINER}:/tmp/"
                f"{rotation.WORKER_EVIDENCE_PREFIX}"
            )
        ):
            destination = Path(args[3])
            destination.mkdir(parents=True)
            completion = self._completion()
            (destination / rotation.WORKER_COMPLETION_FILE).write_text(
                json.dumps(completion)
            )
            (destination / "summary.json").write_text(
                json.dumps(
                    {
                        "result": "passed",
                        "plan_sha256": completion["plan_sha256"],
                    }
                )
            )
            return rotation.CommandResult(0, "", "")
        if (
            "cat" in args
            and any(
                str(value).endswith(rotation.WORKER_COMPLETION_FILE)
                for value in args
            )
        ):
            return rotation.CommandResult(
                0,
                json.dumps(self._completion()) + "\n",
                "",
            )
        if "sha256sum" in args:
            return rotation.CommandResult(
                0,
                rotation.file_sha256(_PATH) + "  staged-worker.py\n",
                "",
            )
        return rotation.CommandResult(0, "", "")

    def run_supervised(self, args, heartbeat_token):
        self.calls.append(tuple(args))
        self.heartbeat_token = heartbeat_token
        handoff = json.loads(
            (
                self.evidence_root
                / "host-worker-handoff.json"
            ).read_text()
        )
        payload = {
            "result": "campaign_passed",
            "plan_sha256": handoff["plan_sha256"],
            "approval_binding_sha256": handoff[
                "approval"
            ]["binding_sha256"],
        }
        return rotation.CommandResult(
            0,
            rotation.WORKER_RESULT_PREFIX
            + rotation.canonical_json(payload)
            + "\n",
            "",
        )


def test_default_host_path_does_not_import_rclpy(tmp_path, monkeypatch):
    calls = []

    class FakeOrchestrator:
        def run(self, **kwargs):
            calls.append(kwargs)
            return 0

    original_import = builtins.__import__

    def reject_rclpy(name, *args, **kwargs):
        if name == "rclpy" or name.startswith("rclpy."):
            raise AssertionError("host attempted to import rclpy")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", reject_rclpy)
    result = rotation.main(
        [
            "--enable-motion",
            "--evidence-dir",
            str(tmp_path / "host"),
            "--speeds",
            "0.1",
            "--durations",
            "0.1",
            "--repetitions",
            "1",
        ],
        input_fn=lambda _prompt: "ROTATE",
        orchestrator_factory=FakeOrchestrator,
    )
    assert result == 0
    assert len(calls) == 1


def test_host_gate_failure_prevents_worker_start(tmp_path):
    runner = FakeContainerRunner(tmp_path / "unused")

    def fail_gate():
        raise rotation.ValidationError("ownership gate failed")

    orchestrator = rotation.HostOrchestrator(
        runner=runner,
        host_gate=fail_gate,
        workspace_root=tmp_path,
        script_path=_PATH,
    )
    evidence = (
        tmp_path
        / "src/realsense_imu/validation_evidence/gate-failure"
    )
    with pytest.raises(rotation.ValidationError, match="ownership"):
        orchestrator.run(
            plan=rotation.plan_payload(
                rotation.build_matrix([0.1], [0.1], 1),
                20.0,
            ),
            evidence_dir=evidence,
            input_fn=lambda _prompt: "ROTATE",
        )
    assert runner.calls == []
    assert not evidence.exists()


def test_host_orchestrates_fake_container_and_binds_handoff(tmp_path):
    evidence = (
        tmp_path
        / "src/realsense_imu/validation_evidence/fake-container"
    )
    runner = FakeContainerRunner(evidence)
    gate_calls = []

    def gate():
        gate_calls.append(True)
        return valid_host_snapshot()

    orchestrator = rotation.HostOrchestrator(
        runner=runner,
        host_gate=gate,
        workspace_root=tmp_path,
        script_path=_PATH,
    )
    plan = rotation.plan_payload(
        rotation.build_matrix([0.1], [0.1], 1),
        20.0,
    )

    def approve_after_preflight(prompt):
        assert any(
            "--worker-preflight" in " ".join(call)
            for call in runner.calls
        )
        assert "Binding SHA256" in prompt
        return "ROTATE"

    assert orchestrator.run(
        plan=plan,
        evidence_dir=evidence,
        input_fn=approve_after_preflight,
    ) == 0
    assert len(gate_calls) == 3
    worker_calls = [
        call
        for call in runner.calls
        if "--worker-" in " ".join(call)
    ]
    assert len(worker_calls) == 2
    for call in runner.calls:
        rendered = " ".join(call)
        assert "--privileged" not in rendered
        assert "/var/run/docker.sock" not in rendered
        assert "--mount" not in rendered
    for call in worker_calls:
        rendered = " ".join(call)
        assert "/ros_ws/src/" not in rendered
        assert rotation.WORKER_SOURCE_PREFIX in rendered
        assert "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4" in rendered
        assert "export SKIP_DEFAULT_XML=1" in rendered
        assert "unset FASTDDS_DEFAULT_PROFILES_FILE" in rendered
        assert "unset FASTRTPS_DEFAULT_PROFILES_FILE" in rendered
        assert "unset RMW_FASTRTPS_USE_QOS_FROM_XML" in rendered
    handoff = json.loads(
        (evidence / "host-worker-handoff.json").read_text()
    )
    assert handoff["plan_sha256"] == plan["plan_sha256"]
    assert handoff["worker_sha256"] == rotation.file_sha256(_PATH)
    assert handoff["evidence_relative"].endswith("fake-container")
    assert handoff["approval"]["text"] == "ROTATE"
    assert handoff["approval"]["binding_sha256"] == rotation.payload_sha256(
        rotation.approval_binding_payload(handoff)
    )
    assert runner.heartbeat_token == handoff["heartbeat_token"]
    assert handoff["worker_source_path"].startswith("/tmp/")
    assert handoff["worker_evidence_path"].startswith("/tmp/")
    assert (evidence / "host-final.json").exists()


def test_post_approval_host_drift_prevents_worker_execution(tmp_path):
    evidence = (
        tmp_path
        / "src/realsense_imu/validation_evidence/post-approval-drift"
    )
    runner = FakeContainerRunner(evidence)
    snapshots = [
        valid_host_snapshot(),
        valid_host_snapshot(main_container_id="b" * 64),
    ]

    def gate():
        return snapshots.pop(0)

    orchestrator = rotation.HostOrchestrator(
        runner=runner,
        host_gate=gate,
        workspace_root=tmp_path,
        script_path=_PATH,
    )
    with pytest.raises(rotation.ValidationError, match="after approval"):
        orchestrator.run(
            plan=rotation.plan_payload(
                rotation.build_matrix([0.1], [0.1], 1),
                20.0,
            ),
            evidence_dir=evidence,
            input_fn=lambda _prompt: "ROTATE",
        )
    assert not any(
        "--worker-execute" in " ".join(call)
        for call in runner.calls
    )
    assert (evidence / "host-pre-execution.json").exists()


def test_worker_failure_is_propagated_after_host_postflight(tmp_path):
    evidence = (
        tmp_path
        / "src/realsense_imu/validation_evidence/worker-failure"
    )

    class FailingRunner(FakeContainerRunner):
        def run_supervised(self, args, heartbeat_token):
            self.calls.append(tuple(args))
            self.heartbeat_token = heartbeat_token
            return rotation.CommandResult(
                23,
                "",
                "synthetic worker failure",
            )

    runner = FailingRunner(evidence)
    gate_calls = []

    def gate():
        gate_calls.append(True)
        return valid_host_snapshot()

    orchestrator = rotation.HostOrchestrator(
        runner=runner,
        host_gate=gate,
        workspace_root=tmp_path,
        script_path=_PATH,
    )
    with pytest.raises(rotation.ValidationError, match="worker failed"):
        orchestrator.run(
            plan=rotation.plan_payload(
                rotation.build_matrix([0.1], [0.1], 1),
                20.0,
            ),
            evidence_dir=evidence,
            input_fn=lambda _prompt: "ROTATE",
        )
    assert len(gate_calls) == 3
    command = json.loads(
        (evidence / "worker-execution-command.json").read_text()
    )
    assert command["returncode"] == 23
    assert (evidence / "host-postflight.json").exists()


def test_worker_copy_failure_retains_container_evidence(tmp_path):
    evidence = (
        tmp_path
        / "src/realsense_imu/validation_evidence/copy-failure"
    )

    class CopyFailRunner(FakeContainerRunner):
        def run(self, args):
            if (
                len(args) >= 3
                and args[:2] == ["docker", "cp"]
                and args[2].startswith(
                    f"{rotation.WORKER_CONTAINER}:/tmp/"
                    f"{rotation.WORKER_EVIDENCE_PREFIX}"
                )
            ):
                self.calls.append(tuple(args))
                return rotation.CommandResult(
                    9,
                    "",
                    "synthetic copy failure",
                )
            return super().run(args)

    runner = CopyFailRunner(evidence)
    orchestrator = rotation.HostOrchestrator(
        runner=runner,
        host_gate=valid_host_snapshot,
        workspace_root=tmp_path,
        script_path=_PATH,
    )
    with pytest.raises(rotation.ValidationError, match="retained"):
        orchestrator.run(
            plan=rotation.plan_payload(
                rotation.build_matrix([0.1], [0.1], 1),
                20.0,
            ),
            evidence_dir=evidence,
            input_fn=lambda _prompt: "ROTATE",
        )
    handoff = json.loads(
        (evidence / "host-worker-handoff.json").read_text()
    )
    worker_evidence = handoff["worker_evidence_path"]
    assert not any(
        "rm -rf" in " ".join(call)
        and worker_evidence in " ".join(call)
        for call in runner.calls
    )
    copy_status = json.loads(
        (evidence / "worker-evidence-copy.json").read_text()
    )
    assert copy_status["container_evidence_retained"] is True
    assert copy_status["recovery_path"] == worker_evidence


def test_interrupted_supervisor_without_completion_retains_evidence(
    tmp_path,
):
    evidence = (
        tmp_path
        / "src/realsense_imu/validation_evidence/interrupted-worker"
    )

    class InterruptedRunner(FakeContainerRunner):
        def run_supervised(self, args, heartbeat_token):
            self.calls.append(tuple(args))
            self.heartbeat_token = heartbeat_token
            raise KeyboardInterrupt()

        def run(self, args):
            if (
                "cat" in args
                and any(
                    str(value).endswith(
                        rotation.WORKER_COMPLETION_FILE
                    )
                    for value in args
                )
            ):
                self.calls.append(tuple(args))
                return rotation.CommandResult(1, "", "not complete")
            return super().run(args)

    runner = InterruptedRunner(evidence)
    orchestrator = rotation.HostOrchestrator(
        runner=runner,
        host_gate=valid_host_snapshot,
        workspace_root=tmp_path,
        script_path=_PATH,
        completion_timeout_s=0.0,
    )
    with pytest.raises(KeyboardInterrupt):
        orchestrator.run(
            plan=rotation.plan_payload(
                rotation.build_matrix([0.1], [0.1], 1),
                20.0,
            ),
            evidence_dir=evidence,
            input_fn=lambda _prompt: "ROTATE",
        )
    handoff = json.loads(
        (evidence / "host-worker-handoff.json").read_text()
    )
    worker_evidence = handoff["worker_evidence_path"]
    assert any(
        rotation.WORKER_COMPLETION_FILE in " ".join(call)
        for call in runner.calls
    )
    assert not any(
        "rm -rf" in " ".join(call)
        and worker_evidence in " ".join(call)
        for call in runner.calls
    )
    execution = json.loads(
        (evidence / "worker-execution-command.json").read_text()
    )
    assert execution["exception"].startswith("KeyboardInterrupt")


def test_worker_preflight_uses_fake_ros_container_without_publisher(
    tmp_path,
    capsys,
):
    handoff = tmp_path / "handoff.json"
    worker_path = staged_worker(tmp_path)
    handoff.write_text(
        json.dumps(
            handoff_payload(
                approved=False,
                worker_path=worker_path,
            )
        )
    )
    created = []

    def factory(host, approved, heartbeat):
        runtime = FakeWorkerRuntime(host, approved, heartbeat)
        created.append(runtime)
        return runtime

    assert rotation.worker_preflight(
        handoff,
        runtime_factory=factory,
        script_path=worker_path,
    ) == 0
    result = rotation.parse_worker_result(capsys.readouterr().out)
    assert result["result"] == "preflight_passed"
    assert created[0].created == 0
    assert created[0].closed == 1


def test_worker_graph_failure_prevents_publisher_creation(
    tmp_path,
):
    handoff = tmp_path / "handoff.json"
    worker_path = staged_worker(tmp_path)
    handoff.write_text(
        json.dumps(
            handoff_payload(
                approved=False,
                worker_path=worker_path,
            )
        )
    )
    created = []

    class InvalidWorkerRuntime(FakeWorkerRuntime):
        def __init__(self, host, approved, heartbeat):
            super().__init__(host, approved, heartbeat)
            self.ros_snapshot = valid_ros_snapshot(
                processed_publishers=(),
            )
            self.snapshot = rotation.compose_preflight(
                host,
                self.ros_snapshot,
            )
            created.append(self)

    with pytest.raises(rotation.ValidationError, match="processed"):
        rotation.worker_preflight(
            handoff,
            runtime_factory=InvalidWorkerRuntime,
            script_path=worker_path,
        )
    assert created[0].created == 0
    assert created[0].closed == 1


def test_worker_rejects_publisher_gid_change_after_approval():
    approved = valid_ros_snapshot(
        raw_publishers=(
            rotation.GraphEndpoint(
                "realsense_imu_relay",
                rotation.BEST_EFFORT_QOS,
                "raw-approved",
            ),
        ),
    )
    changed = valid_ros_snapshot(
        raw_publishers=(
            rotation.GraphEndpoint(
                "realsense_imu_relay",
                rotation.BEST_EFFORT_QOS,
                "raw-changed",
            ),
        ),
    )
    runtime = object.__new__(rotation.RosWorkerRuntime)
    runtime.host_snapshot = valid_host_snapshot()
    runtime.approved_graph = rotation.approval_graph_identity(approved)
    runtime.ros_preflight_snapshot = lambda: changed
    with pytest.raises(rotation.ValidationError, match="approved"):
        runtime.preflight_snapshot()


@pytest.mark.parametrize(
    "mutator, message",
    [
        (
            lambda payload: payload.update(worker_sha256="0" * 64),
            "worker code hash",
        ),
        (
            lambda payload: payload["plan"].update(plan_sha256="0" * 64),
            "plan hash",
        ),
        (
            lambda payload: payload.update(
                evidence_relative="../../unsafe"
            ),
            "evidence path",
        ),
        (
            lambda payload: payload["approval"].update(
                binding_sha256="0" * 64
            ),
            "approval binding",
        ),
    ],
)
def test_worker_rejects_tampered_handoff_before_runtime(
    mutator,
    message,
    tmp_path,
):
    worker_path = staged_worker(tmp_path)
    payload = handoff_payload(
        approved=True,
        worker_path=worker_path,
    )
    mutator(payload)
    called = []
    with pytest.raises(rotation.ValidationError, match=message):
        rotation.validate_handoff(
            payload,
            script_path=worker_path,
            require_approval=True,
        )
    assert called == []


def test_worker_requires_approval_before_runtime_or_publisher(tmp_path):
    handoff = tmp_path / "handoff.json"
    worker_path = staged_worker(tmp_path)
    handoff.write_text(
        json.dumps(
            handoff_payload(
                approved=False,
                worker_path=worker_path,
            )
        )
    )
    called = []
    with pytest.raises(rotation.ValidationError, match="ROS snapshot"):
        rotation.worker_execute(
            handoff,
            enable_motion=True,
            runtime_factory=lambda *_args: called.append(True),
            script_path=worker_path,
        )
    assert called == []


def test_worker_cli_rejects_host_matrix_arguments_before_ros(
    tmp_path,
    monkeypatch,
):
    handoff = tmp_path / "handoff.json"
    handoff.write_text(
        json.dumps(
            handoff_payload(
                approved=False,
                worker_path=staged_worker(tmp_path),
            )
        )
    )
    monkeypatch.setenv("D455_ROTATION_WORKER", "1")
    with pytest.raises(rotation.ValidationError, match="only from"):
        rotation.main(
            [
                "--worker-preflight",
                "--handoff",
                str(handoff),
                "--speeds",
                "0.1",
            ]
        )


def test_worker_execute_preserves_synthetic_cleanup_and_evidence(
    tmp_path,
    monkeypatch,
):
    handoff = tmp_path / "handoff.json"
    worker_path = staged_worker(tmp_path)
    payload = handoff_payload(
        approved=True,
        worker_path=worker_path,
    )
    handoff.write_text(json.dumps(payload))
    runtimes = []

    def factory(host, approved, heartbeat):
        runtime = FakeWorkerRuntime(host, approved, heartbeat)
        runtimes.append(runtime)
        return runtime

    monkeypatch.delenv(
        "D455_ROTATION_APPROVAL_BINDING",
        raising=False,
    )
    assert rotation.worker_execute(
        handoff,
        enable_motion=True,
        runtime_factory=factory,
        script_path=worker_path,
    ) == 0
    evidence = Path(payload["worker_evidence_path"])
    summary = json.loads((evidence / "summary.json").read_text())
    assert summary["result"] == "passed"
    assert summary["cleanup"]["final_zero_verified"] is True
    assert runtimes[0].created == 1
    assert runtimes[0].published[-1][1] == rotation.twist(0.0)


def test_ros_worker_adapter_has_no_host_gate_access():
    source = inspect.getsource(rotation.RosWorkerRuntime)
    assert "d455_production_container" not in source
    assert "docker" not in source.lower()
    assert "apparmor" not in source.lower()
    assert "subprocess" not in source


def test_graph_convergence_retries_and_records_each_attempt():
    runtime = object.__new__(rotation.RosWorkerRuntime)
    invalid = valid_ros_snapshot(processed_publishers=())
    valid = valid_ros_snapshot()
    snapshots = iter((invalid, valid))
    clock = [0.0]
    runtime.monotonic = lambda: clock[0]
    runtime.wall_time_ns = lambda: int(clock[0] * 1_000_000_000)
    runtime.sleep = lambda duration: clock.__setitem__(0, clock[0] + duration)
    runtime.ros_preflight_snapshot = lambda: next(snapshots)
    result = runtime.converge_graph(
        valid_host_snapshot(),
        timeout_s=2.0,
        poll_s=0.1,
    )
    assert result == valid
    assert runtime.graph_convergence.result == "converged"
    assert len(runtime.graph_convergence.attempts) == 2
    assert runtime.graph_convergence.attempts[0]["missing_or_conflicting"]
    assert runtime.graph_convergence.attempts[1]["result"] == "passed"


def test_graph_convergence_timeout_fails_closed_without_publisher():
    runtime = object.__new__(rotation.RosWorkerRuntime)
    clock = [0.0]
    runtime.monotonic = lambda: clock[0]
    runtime.wall_time_ns = lambda: int(clock[0] * 1_000_000_000)
    runtime.sleep = lambda duration: clock.__setitem__(0, clock[0] + duration)
    runtime.ros_preflight_snapshot = lambda: valid_ros_snapshot(
        processed_publishers=(),
    )
    with pytest.raises(rotation.ValidationError, match="timed out"):
        runtime.converge_graph(
            valid_host_snapshot(),
            timeout_s=0.01,
            poll_s=0.01,
        )
    assert runtime.graph_convergence.result == "timeout"
    assert runtime.graph_convergence.attempts
    assert not hasattr(runtime, "publisher")


def test_graph_convergence_rejects_duplicate_endpoint_then_recovers():
    runtime = object.__new__(rotation.RosWorkerRuntime)
    base = valid_ros_snapshot()
    duplicate = valid_ros_snapshot(
        raw_publishers=base.raw_publishers + base.raw_publishers,
    )
    snapshots = iter((duplicate, base))
    clock = [0.0]
    runtime.monotonic = lambda: clock[0]
    runtime.wall_time_ns = lambda: int(clock[0] * 1e9)
    runtime.sleep = lambda duration: clock.__setitem__(0, clock[0] + duration)
    runtime.ros_preflight_snapshot = lambda: next(snapshots)
    assert runtime.converge_graph(valid_host_snapshot(), timeout_s=1.0) == base
    first = runtime.graph_convergence.attempts[0]
    assert first["result"] == "partial"
    assert "raw publisher" in " ".join(first["missing_or_conflicting"])


def test_graph_convergence_records_qos_and_gid_details():
    runtime = object.__new__(rotation.RosWorkerRuntime)
    base = valid_ros_snapshot()
    mismatched = valid_ros_snapshot(
        processed_publishers=(
            rotation.GraphEndpoint(
                "d455_imu_processor", rotation.RELIABLE_QOS, "bad-gid"
            ),
        ),
    )
    snapshots = iter((mismatched, base))
    clock = [0.0]
    runtime.monotonic = lambda: clock[0]
    runtime.wall_time_ns = lambda: int(clock[0] * 1e9)
    runtime.sleep = lambda duration: clock.__setitem__(0, clock[0] + duration)
    runtime.ros_preflight_snapshot = lambda: next(snapshots)
    runtime.converge_graph(valid_host_snapshot(), timeout_s=1.0)
    observed = runtime.graph_convergence.attempts[0]["observed"]
    endpoint = observed["processed_publishers"][0]
    assert endpoint["qos"] == rotation.RELIABLE_QOS
    assert endpoint["gid"] == "bad-gid"
    assert runtime.graph_convergence.duration_s >= 0.0


def test_graph_convergence_rejects_unbounded_configuration():
    runtime = object.__new__(rotation.RosWorkerRuntime)
    with pytest.raises(ValueError, match="bounded range"):
        runtime.converge_graph(valid_host_snapshot(), timeout_s=16.0)
    with pytest.raises(ValueError, match="bounded range"):
        runtime.converge_graph(valid_host_snapshot(), poll_s=2.1)
