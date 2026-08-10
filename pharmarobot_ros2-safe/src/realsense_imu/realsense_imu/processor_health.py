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

"""ROS-independent health and diagnostics state for the IMU processor."""

from collections import deque
from dataclasses import dataclass
import math
from typing import Dict, Tuple
from typing import Optional

from realsense_imu.bias_estimator import BiasSnapshot


OK = 0
WARN = 1
ERROR = 2


@dataclass(frozen=True)
class HealthConfig:
    """Thresholds for stale input and output-rate diagnostics."""

    stale_timeout_s: float
    minimum_output_rate_hz: float
    rate_window_samples: int = 200

    def __post_init__(self):
        """Validate health thresholds before tracking samples."""
        if not math.isfinite(self.stale_timeout_s) or (
            self.stale_timeout_s <= 0.0
        ):
            raise ValueError("stale_timeout_s must be positive and finite")
        if not math.isfinite(self.minimum_output_rate_hz) or (
            self.minimum_output_rate_hz < 0.0
        ):
            raise ValueError(
                "minimum_output_rate_hz must be nonnegative and finite"
            )
        if self.rate_window_samples < 2:
            raise ValueError("rate_window_samples must be at least 2")


@dataclass(frozen=True)
class HealthStatus:
    """One diagnostic status independent of diagnostic_msgs."""

    name: str
    level: int
    message: str
    values: Dict[str, str]


class ProcessorHealthTracker:
    """Track sample health, output rate, and current rejection state."""

    def __init__(self, config: HealthConfig):
        """Create an empty tracker with bounded output-rate history."""
        self.config = config
        self._last_raw_receive = None
        self._last_output = None
        self._last_rejection = None
        self._rejection_counts = {}
        self._dropout_count = 0
        self._raw_count = 0
        self._output_times = deque(maxlen=config.rate_window_samples)
        self._covariance_valid = False

    def record_raw_received(self, monotonic_s: float):
        """Record receipt of one raw DDS sample."""
        self._last_raw_receive = monotonic_s
        self._raw_count += 1

    def record_raw_accepted(self):
        """Clear transient validation errors after a valid raw sample."""
        self._last_rejection = None

    def record_rejection(self, reason: str):
        """Record one rejected sample and reason."""
        self._last_rejection = reason
        self._rejection_counts[reason] = (
            self._rejection_counts.get(reason, 0) + 1
        )

    def record_dropout(self):
        """Record one timestamp gap."""
        self._dropout_count += 1

    def record_output(
        self,
        monotonic_s: float,
        *,
        covariance_valid: bool,
    ):
        """Record one processed publication."""
        self._last_output = monotonic_s
        self._output_times.append(monotonic_s)
        self._covariance_valid = covariance_valid

    def _output_rate(self) -> float:
        if len(self._output_times) < 2:
            return 0.0
        span = self._output_times[-1] - self._output_times[0]
        if span <= 0.0:
            return 0.0
        return (len(self._output_times) - 1) / span

    def snapshot(
        self,
        now_monotonic_s: float,
        *,
        bias: BiasSnapshot,
        transform_calibrated: bool,
        covariance_calibrated: bool,
        transform_values: Optional[Dict[str, str]] = None,
        covariance_values: Optional[Dict[str, str]] = None,
    ) -> Tuple[HealthStatus, ...]:
        """Return current raw/output/transform/bias/covariance statuses."""
        raw_values = {
            "raw_count": str(self._raw_count),
            "dropout_count": str(self._dropout_count),
            "last_rejection": self._last_rejection or "",
        }
        raw_values.update(
            {
                f"rejected_{reason}": str(count)
                for reason, count in sorted(self._rejection_counts.items())
            }
        )
        if self._last_raw_receive is None:
            raw = HealthStatus(
                "D455 IMU/Raw Input",
                ERROR,
                "no raw input received",
                raw_values,
            )
        elif now_monotonic_s - self._last_raw_receive > (
            self.config.stale_timeout_s
        ):
            raw = HealthStatus(
                "D455 IMU/Raw Input", ERROR, "raw input is stale", raw_values
            )
        elif self._last_rejection:
            raw = HealthStatus(
                "D455 IMU/Raw Input",
                WARN,
                f"latest sample rejected: {self._last_rejection}",
                raw_values,
            )
        else:
            raw = HealthStatus(
                "D455 IMU/Raw Input", OK, "raw input healthy", raw_values
            )

        output_rate = self._output_rate()
        output_values = {"rate_hz": f"{output_rate:.6f}"}
        if self._last_output is None:
            output = HealthStatus(
                "D455 IMU/Processed Output",
                WARN,
                "processed output not yet available",
                output_values,
            )
        elif now_monotonic_s - self._last_output > (
            self.config.stale_timeout_s
        ):
            output = HealthStatus(
                "D455 IMU/Processed Output",
                ERROR,
                "processed output is stale",
                output_values,
            )
        elif output_rate < self.config.minimum_output_rate_hz:
            output = HealthStatus(
                "D455 IMU/Processed Output",
                WARN,
                "processed output rate below threshold",
                output_values,
            )
        else:
            output = HealthStatus(
                "D455 IMU/Processed Output",
                OK,
                "processed output healthy",
                output_values,
            )

        transform = HealthStatus(
            "D455 IMU/Transform",
            OK if transform_calibrated else WARN,
            (
                "configured transform calibrated"
                if transform_calibrated
                else "configured transform is provisional"
            ),
            dict(transform_values or {}),
        )
        bias_values = {
            "state": bias.state,
            "calibrated": str(bias.calibrated).lower(),
            "bias_x": f"{bias.bias[0]:.12g}",
            "bias_y": f"{bias.bias[1]:.12g}",
            "bias_z": f"{bias.bias[2]:.12g}",
            "residual_x": f"{bias.residual_stddev[0]:.12g}",
            "residual_y": f"{bias.residual_stddev[1]:.12g}",
            "residual_z": f"{bias.residual_stddev[2]:.12g}",
            "warmup_samples": str(bias.warmup_sample_count),
            "candidate_samples": str(bias.candidate_sample_count),
            "last_update_samples": str(bias.last_update_sample_count),
            "last_update_timestamp_s": (
                ""
                if bias.last_update_timestamp_s is None
                else f"{bias.last_update_timestamp_s:.9f}"
            ),
        }
        bias_status = HealthStatus(
            "D455 IMU/Bias",
            OK if bias.calibrated else WARN,
            "gyro bias calibrated" if bias.calibrated else bias.state,
            bias_values,
        )

        if not self._covariance_valid:
            covariance = HealthStatus(
                "D455 IMU/Covariance",
                ERROR,
                "no valid transformed covariance",
                {},
            )
        else:
            covariance = HealthStatus(
                "D455 IMU/Covariance",
                OK if covariance_calibrated else WARN,
                (
                    "covariance calibrated"
                    if covariance_calibrated
                    else "upstream covariance transformed but uncalibrated"
                ),
                dict(covariance_values or {}),
            )
        return (raw, output, transform, bias_status, covariance)
