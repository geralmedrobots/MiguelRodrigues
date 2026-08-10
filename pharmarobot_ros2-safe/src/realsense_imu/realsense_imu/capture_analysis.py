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

"""Prove a deliberate yaw interval from a ten-second IMU YAML capture."""

import argparse
from dataclasses import dataclass
from math import sqrt
from pathlib import Path
import re
from typing import Iterable, Optional, Sequence, Tuple


_STAMP = re.compile(r"^    sec: (\d+)$\n^    nanosec: (\d+)$", re.MULTILINE)
_ANGULAR_VELOCITY = re.compile(
    r"^angular_velocity:$\n"
    r"^  x: ([^\n]+)$\n"
    r"^  y: ([^\n]+)$\n"
    r"^  z: ([^\n]+)$",
    re.MULTILINE,
)
_WINDOWS = (
    ("stationary_pre", 0.0, 3.0),
    ("yaw", 3.0, 7.0),
    ("stationary_post", 7.0, 10.0),
)
_YAW_MULTIPLIER = 3.0
_MIN_YAW_ANGULAR_NORM = 0.05


class CaptureAnalysisError(ValueError):
    """Report malformed or insufficient capture evidence."""


@dataclass(frozen=True)
class ImuSample:
    """Timestamped angular-velocity measurement from one IMU message."""

    timestamp: float
    angular_velocity: Tuple[float, float, float]

    @property
    def angular_norm(self) -> float:
        """Return the Euclidean angular-velocity magnitude in rad/s."""
        return sqrt(sum(axis * axis for axis in self.angular_velocity))


@dataclass(frozen=True)
class WindowSummary:
    """Motion metrics for one required capture interval."""

    name: str
    count: int
    mean_angular_norm: float
    integrated_angular_motion: float


@dataclass(frozen=True)
class CaptureAnalysis:
    """Three-window result and the conservative yaw-motion decision."""

    windows: Tuple[WindowSummary, ...]
    yaw_proven: bool


def parse_imu_yaml(content: str) -> Tuple[ImuSample, ...]:
    """Parse the stable fields emitted by ``ros2 topic echo`` for IMU data."""
    samples = []
    for document in content.split("---"):
        stamp = _STAMP.search(document)
        angular_velocity = _ANGULAR_VELOCITY.search(document)
        if stamp is None and angular_velocity is None and not document.strip():
            continue
        if stamp is None or angular_velocity is None:
            raise CaptureAnalysisError("IMU YAML document lacks stamp or gyro")
        seconds = int(stamp.group(1)) + int(stamp.group(2)) / 1_000_000_000
        samples.append(
            ImuSample(
                seconds,
                tuple(
                    float(angular_velocity.group(index))
                    for index in (1, 2, 3)
                ),
            )
        )
    if not samples:
        raise CaptureAnalysisError("IMU YAML capture contains no messages")
    if any(
        current.timestamp <= previous.timestamp
        for previous, current in zip(samples, samples[1:])
    ):
        raise CaptureAnalysisError(
            "IMU YAML timestamps are not strictly increasing"
        )
    return tuple(samples)


def _integrate_norm(samples: Iterable[ImuSample]) -> float:
    values = tuple(samples)
    return sum(
        (previous.angular_norm + current.angular_norm)
        * (current.timestamp - previous.timestamp)
        / 2.0
        for previous, current in zip(values, values[1:])
    )


def analyze_capture(samples: Sequence[ImuSample]) -> CaptureAnalysis:
    """Require a yaw interval clearly above both stationary baselines."""
    if not samples:
        raise CaptureAnalysisError("IMU capture contains no messages")
    start = samples[0].timestamp
    summaries = []
    window_samples = {}
    for name, lower, upper in _WINDOWS:
        window = tuple(
            sample
            for sample in samples
            if lower <= sample.timestamp - start < upper
        )
        if len(window) < 2:
            raise CaptureAnalysisError(
                f"{name} interval has fewer than two samples"
            )
        window_samples[name] = window
        summaries.append(
            WindowSummary(
                name=name,
                count=len(window),
                mean_angular_norm=sum(sample.angular_norm for sample in window)
                / len(window),
                integrated_angular_motion=_integrate_norm(window),
            )
        )
    stationary_pre, yaw, stationary_post = summaries
    baseline_mean = max(
        stationary_pre.mean_angular_norm, stationary_post.mean_angular_norm
    )
    baseline_integral = max(
        stationary_pre.integrated_angular_motion,
        stationary_post.integrated_angular_motion,
    )
    yaw_window = window_samples["yaw"]
    yaw_duration = yaw_window[-1].timestamp - yaw_window[0].timestamp
    yaw_proven = (
        yaw.mean_angular_norm
        >= max(_MIN_YAW_ANGULAR_NORM, _YAW_MULTIPLIER * baseline_mean)
        and yaw.integrated_angular_motion
        >= max(
            _MIN_YAW_ANGULAR_NORM * yaw_duration,
            _YAW_MULTIPLIER * baseline_integral,
        )
    )
    return CaptureAnalysis(tuple(summaries), yaw_proven)


def main(args: Optional[Sequence[str]] = None) -> None:
    """Print a deterministic summary and fail if deliberate yaw is absent."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path, help="ten-second ros2 echo YAML")
    parsed_args = parser.parse_args(args)
    try:
        analysis = analyze_capture(
            parse_imu_yaml(parsed_args.capture.read_text(encoding="utf-8"))
        )
    except (CaptureAnalysisError, OSError) as exc:
        parser.error(str(exc))
    for window in analysis.windows:
        print(
            f"{window.name}: count={window.count} "
            f"mean_angular_norm={window.mean_angular_norm:.6f} "
            f"integrated_angular_motion={window.integrated_angular_motion:.6f}"
        )
    if not analysis.yaw_proven:
        parser.error(
            "deliberate yaw was not proven above stationary baselines"
        )


if __name__ == "__main__":
    main()
