#!/usr/bin/env python3
"""Run the isolated D455 no-motion runtime checks with bounded ROS commands.

This tool is deliberately not installed as a robot-runtime executable.  It is
an operator-gated validation wrapper for an already-created isolated D455
container.  It never publishes a nonzero Twist and will not execute unless the
caller explicitly acknowledges the exact-zero-only gate.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import shlex
import subprocess
import sys
from textwrap import dedent


RAW_TOPIC = "/imu/d455/data_raw"
PROCESSED_TOPIC = "/imu/data"
SAFE_TOPIC = "/cmd_vel/safe"
RAW_FRAME = "d455_imu_optical_frame"
PROCESSED_FRAME = "d455_imu_link"
RATE_TIMEOUT_SECONDS = 8


def echo_command(topic: str) -> list[str]:
    """Return a bounded Humble-compatible sensor-data echo command."""
    return [
        "timeout",
        "5s",
        "ros2",
        "topic",
        "echo",
        topic,
        "sensor_msgs/msg/Imu",
        "--once",
        "--qos-reliability",
        "best_effort",
        "--qos-durability",
        "volatile",
    ]


def rate_command(topic: str) -> list[str]:
    """Return a bounded `ros2 topic hz` command accepted by ROS 2 Humble."""
    return [
        "timeout",
        "--foreground",
        "--signal=INT",
        "--kill-after=2s",
        f"{RATE_TIMEOUT_SECONDS}s",
        "ros2",
        "topic",
        "hz",
        topic,
    ]


def validated_workspace(workspace: str) -> str:
    """Return a safe absolute workspace path for the generated shell script."""
    if not workspace or "\x00" in workspace:
        raise ValueError("workspace must be a nonempty path without NUL")
    if not Path(workspace).is_absolute():
        raise ValueError("workspace must be an absolute path")
    return workspace


def runtime_script(workspace: str) -> str:
    """Build the in-container fail-closed validation script."""
    workspace = validated_workspace(workspace)
    shell_workspace = shlex.quote(workspace)
    raw_echo = " ".join(echo_command(RAW_TOPIC))
    processed_echo = " ".join(echo_command(PROCESSED_TOPIC))
    raw_rate = " ".join(rate_command(RAW_TOPIC))
    processed_rate = " ".join(rate_command(PROCESSED_TOPIC))
    return dedent(
        f"""\
        set -eo pipefail
        source /opt/ros/humble/setup.bash
        workspace_path={shell_workspace}
        source "$workspace_path/install/setup.bash"
        set -u
        run="$workspace_path/runtime-evidence"
        mkdir -p "$run"
        launch_pid=
        zero_pid=
        interrupted=0
        cleanup() {{
          status=$?
          trap - EXIT INT TERM
          set +e
          if [ -n "${{zero_pid}}" ]; then
            kill -INT -- "-${{zero_pid}}" 2>/dev/null || true
            wait "${{zero_pid}}" 2>/dev/null || true
          fi
          if [ -n "${{launch_pid}}" ]; then
            kill -INT -- "-${{launch_pid}}" 2>/dev/null || true
            wait "${{launch_pid}}" 2>/dev/null || true
          fi
          ros2 daemon stop >/dev/null 2>&1 || true
          sleep 1
          ps -eo pid,ppid,pgid,stat,args > "$run/post-cleanup-census.txt"
          if ps -eo stat= | grep -q Z; then
            echo ZOMBIE_PRESENT=1 >> "$run/post-cleanup-census.txt"
            exit 1
          fi
          owned_process_remains=0
          for owned_pgid in "${{launch_pid}}" "${{zero_pid}}"; do
            if [ -n "${{owned_pgid}}" ]; then
              if pgrep -a -g "${{owned_pgid}}" >> "$run/post-cleanup-census.txt"; then
                owned_process_remains=1
              fi
            fi
          done
          if [ "$owned_process_remains" -ne 0 ]; then
            echo OWNED_PROCESS_REMAINS=1 >> "$run/post-cleanup-census.txt"
            exit 1
          fi
          echo ZOMBIE_PRESENT=0 >> "$run/post-cleanup-census.txt"
          echo OWNED_PROCESS_REMAINS=0 >> "$run/post-cleanup-census.txt"
          exit "$status"
        }}
        abort_signal() {{
          interrupted=1
          exit 1
        }}
        trap cleanup EXIT
        trap abort_signal INT TERM

        test ! -e /dev/roboteq
        test ! -e /dev/ttyUSB0
        setsid ros2 launch realsense_imu robot_sensors.launch.py \
          serial_number:=146222250608 > "$run/robot-sensors.launch.log" 2>&1 &
        launch_pid=$!
        for unused in $(seq 1 20); do
          if {raw_echo} > "$run/raw-pre-zero.yaml" 2> "$run/raw-pre-zero.stderr"; then
            break
          fi
          sleep 1
        done
        test -s "$run/raw-pre-zero.yaml"
        grep -q "frame_id: {RAW_FRAME}" "$run/raw-pre-zero.yaml"
        grep -A 1 "orientation_covariance:" "$run/raw-pre-zero.yaml" | grep -q -- "-1.0"

        set +e
        {processed_echo} > "$run/processed-pre-zero.yaml" 2> "$run/processed-pre-zero.stderr"
        processed_pre_status=$?
        set -e
        printf "processed_pre_zero_status=%s\\n" "$processed_pre_status" > "$run/processed-pre-zero.result"
        test "$processed_pre_status" -eq 124
        test ! -s "$run/processed-pre-zero.yaml"

        timeout 5s ros2 topic info {SAFE_TOPIC} --verbose > "$run/safe-topic-info.txt"
        grep -q "Publisher count: 0" "$run/safe-topic-info.txt"
        grep -q "Subscription count: 1" "$run/safe-topic-info.txt"
        grep -q "Node name: d455_imu_processor" "$run/safe-topic-info.txt"

        setsid ros2 topic pub --rate 20 {SAFE_TOPIC} geometry_msgs/msg/Twist \
          "{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}" \
          > "$run/exact-zero-publisher.log" 2>&1 &
        zero_pid=$!
        sleep 8

        {processed_echo} > "$run/processed-post-zero.yaml" 2> "$run/processed-post-zero.stderr"
        test -s "$run/processed-post-zero.yaml"
        grep -q "frame_id: {PROCESSED_FRAME}" "$run/processed-post-zero.yaml"
        grep -A 1 "orientation_covariance:" "$run/processed-post-zero.yaml" | grep -q -- "-1.0"

        collect_rate() {{
          topic=$1
          output=$2
          set +e
          if [ "$topic" = "{RAW_TOPIC}" ]; then
            {raw_rate} > "$output" 2>&1
          else
            {processed_rate} > "$output" 2>&1
          fi
          rate_status=$?
          set -e
          printf "rate_status=%s\\n" "$rate_status" >> "$output"
          test "$rate_status" -eq 0 -o "$rate_status" -eq 124
          grep -q "average rate:" "$output"
        }}
        collect_rate {RAW_TOPIC} "$run/raw-rate.txt"
        collect_rate {PROCESSED_TOPIC} "$run/processed-rate.txt"

        timeout 5s ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray --once \
          --qos-reliability best_effort --qos-durability volatile \
          > "$run/diagnostics-post-zero.yaml" 2> "$run/diagnostics-post-zero.stderr"
        grep -q "D455 IMU/Bias" "$run/diagnostics-post-zero.yaml"
        grep -q "gyro bias calibrated" "$run/diagnostics-post-zero.yaml"
        cp "$run/robot-sensors.launch.log" "$run/robot-sensors.pre-cleanup.log"
        if grep -Eiq "Parameter .*(serial_number|raw_topic|expected_frame_id|processor_config|topic_name).*not supported|Traceback|AssertionError|process has died|upstream IMU frame mismatch" "$run/robot-sensors.pre-cleanup.log"; then
          exit 1
        fi
        printf "NO_NONZERO_TWIST=1\\n" > "$run/result.txt"
        printf "RUNTIME_VALIDATION=passed\\n" >> "$run/result.txt"
        """
    )


def run_checked(command: list[str]) -> None:
    """Run one host command and surface its bounded failure."""
    completed = subprocess.run(command, check=False)
    if completed.returncode != 0:
        raise RuntimeError(
            f"command failed ({completed.returncode}): {' '.join(command)}"
        )


def execute(container: str, workspace: str, evidence_dir: Path) -> None:
    """Run validation and copy only runtime artifacts into fresh evidence."""
    if evidence_dir.exists():
        raise ValueError(f"evidence directory already exists: {evidence_dir}")
    evidence_dir.mkdir(parents=True)
    script = runtime_script(workspace)
    (evidence_dir / "runtime-command.sh").write_text(script, encoding="utf-8")
    validation_error = None
    try:
        run_checked(["docker", "exec", container, "bash", "-lc", script])
    except Exception as error:
        validation_error = error

    copy_result = subprocess.run(
        [
            "docker",
            "cp",
            f"{container}:{workspace}/runtime-evidence/.",
            str(evidence_dir / "container-artifacts"),
        ],
        check=False,
    )
    if copy_result.returncode != 0:
        raise RuntimeError("failed to copy required runtime evidence") from validation_error
    if validation_error is not None:
        raise validation_error


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--container", required=True)
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--print-runtime-script", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--acknowledge-exact-zero-only",
        action="store_true",
        help="Required together with --execute; no nonzero Twist is supported.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Parse the explicit execution gate and run or print the wrapper."""
    args = parse_args(argv or sys.argv[1:])
    if args.print_runtime_script:
        print(runtime_script(args.workspace), end="")
        return 0
    if not args.execute:
        raise SystemExit("refusing to execute without --execute")
    if not args.acknowledge_exact_zero_only:
        raise SystemExit(
            "refusing to execute without --acknowledge-exact-zero-only"
        )
    execute(args.container, args.workspace, args.evidence_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
