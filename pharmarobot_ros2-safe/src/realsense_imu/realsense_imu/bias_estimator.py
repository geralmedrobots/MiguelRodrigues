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

"""Stationary raw-frame gyro-bias estimation."""

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Tuple


Vector3 = Tuple[float, float, float]
ZERO_VECTOR = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class BiasConfig:
    """Validated thresholds for stationary gyro-bias estimation."""

    warmup_duration_s: float
    warmup_min_samples: int
    stationary_window_duration_s: float
    stationary_min_samples: int
    gyro_stationary_threshold_rad_s: float
    gravity_m_s2: float
    acceleration_tolerance_m_s2: float
    max_sample_gap_s: float
    max_residual_stddev_rad_s: float
    online_update_enabled: bool
    online_update_alpha: float
    require_command_zero: bool = False

    def __post_init__(self):
        """Reject unsafe or nonsensical estimator thresholds."""
        finite_values = {
            "warmup_duration_s": self.warmup_duration_s,
            "stationary_window_duration_s": (
                self.stationary_window_duration_s
            ),
            "gyro_stationary_threshold_rad_s": (
                self.gyro_stationary_threshold_rad_s
            ),
            "gravity_m_s2": self.gravity_m_s2,
            "acceleration_tolerance_m_s2": (
                self.acceleration_tolerance_m_s2
            ),
            "max_sample_gap_s": self.max_sample_gap_s,
            "max_residual_stddev_rad_s": (
                self.max_residual_stddev_rad_s
            ),
            "online_update_alpha": self.online_update_alpha,
        }
        for name, value in finite_values.items():
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if self.warmup_duration_s < 0.0:
            raise ValueError("warmup_duration_s must be nonnegative")
        if self.warmup_min_samples < 0:
            raise ValueError("warmup_min_samples must be nonnegative")
        if self.stationary_window_duration_s < 0.0:
            raise ValueError(
                "stationary_window_duration_s must be nonnegative"
            )
        if self.stationary_min_samples < 1:
            raise ValueError("stationary_min_samples must be positive")
        if self.gyro_stationary_threshold_rad_s <= 0.0:
            raise ValueError(
                "gyro_stationary_threshold_rad_s must be positive"
            )
        if self.gravity_m_s2 <= 0.0:
            raise ValueError("gravity_m_s2 must be positive")
        if self.acceleration_tolerance_m_s2 < 0.0:
            raise ValueError(
                "acceleration_tolerance_m_s2 must be nonnegative"
            )
        if self.max_sample_gap_s <= 0.0:
            raise ValueError("max_sample_gap_s must be positive")
        if self.max_residual_stddev_rad_s < 0.0:
            raise ValueError(
                "max_residual_stddev_rad_s must be nonnegative"
            )
        if not 0.0 < self.online_update_alpha <= 1.0:
            raise ValueError("online_update_alpha must be in (0, 1]")


@dataclass(frozen=True)
class BiasSnapshot:
    """Externally visible state of the estimator."""

    state: str
    calibrated: bool
    bias: Vector3
    residual_stddev: Vector3
    warmup_sample_count: int
    candidate_sample_count: int
    last_update_sample_count: int
    last_update_timestamp_s: Optional[float]


@dataclass(frozen=True)
class BiasObservation:
    """Result of processing one IMU sample for bias learning."""

    sample_valid: bool
    stationary: bool
    reason: str
    bias_updated: bool
    snapshot: BiasSnapshot


def _vector(values: Iterable[float]) -> Vector3:
    result = tuple(float(value) for value in values)
    if len(result) != 3:
        raise ValueError("vector must contain exactly 3 values")
    return result


def _finite_vector(values: Iterable[float]) -> Optional[Vector3]:
    try:
        result = _vector(values)
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(value) for value in result) else None


def _norm(vector: Vector3) -> float:
    return math.sqrt(sum(value * value for value in vector))


def _mean(samples) -> Vector3:
    count = len(samples)
    return tuple(
        sum(sample[axis] for sample in samples) / count
        for axis in range(3)
    )


def _stddev(samples, mean: Vector3) -> Vector3:
    count = len(samples)
    return tuple(
        math.sqrt(
            sum(
                (sample[axis] - mean[axis]) ** 2
                for sample in samples
            ) / count
        )
        for axis in range(3)
    )


class GyroBiasEstimator:
    """Learn raw-frame gyro bias only from continuous stationary windows."""

    def __init__(self, config: BiasConfig):
        """Create an uncalibrated estimator with validated configuration."""
        self.config = config
        self._state = "warming_up"
        self._calibrated = False
        self._bias = ZERO_VECTOR
        self._residual = ZERO_VECTOR
        self._last_timestamp = None
        self._warmup_start = None
        self._warmup_samples = 0
        self._candidate_start = None
        self._candidate_samples = []
        self._last_update_samples = 0
        self._last_update_timestamp = None

    @property
    def bias(self) -> Vector3:
        """Return the current raw-frame bias estimate."""
        return self._bias

    @property
    def calibrated(self) -> bool:
        """Return whether an initial stationary window was accepted."""
        return self._calibrated

    def snapshot(self) -> BiasSnapshot:
        """Return immutable diagnostic state."""
        return BiasSnapshot(
            state=self._state,
            calibrated=self._calibrated,
            bias=self._bias,
            residual_stddev=self._residual,
            warmup_sample_count=self._warmup_samples,
            candidate_sample_count=len(self._candidate_samples),
            last_update_sample_count=self._last_update_samples,
            last_update_timestamp_s=self._last_update_timestamp,
        )

    def reject_sample(self, reason: str) -> BiasObservation:
        """Reject invalid input and discard any partial stationary window."""
        self._clear_candidate()
        self._state = reason
        return self._observation(False, False, reason, False)

    def _observation(
        self,
        sample_valid: bool,
        stationary: bool,
        reason: str,
        bias_updated: bool,
    ) -> BiasObservation:
        return BiasObservation(
            sample_valid=sample_valid,
            stationary=stationary,
            reason=reason,
            bias_updated=bias_updated,
            snapshot=self.snapshot(),
        )

    def _clear_candidate(self):
        self._candidate_start = None
        self._candidate_samples = []

    def _restart_warmup(self, timestamp_s: float):
        if not self._calibrated:
            self._warmup_start = None
            self._warmup_samples = 0
        self._clear_candidate()

    def observe(
        self,
        timestamp_s: float,
        angular_velocity: Iterable[float],
        linear_acceleration: Iterable[float],
        command_zero: Optional[bool] = None,
    ) -> BiasObservation:
        """Validate one sample and update bias only if it is stationary."""
        gyro = _finite_vector(angular_velocity)
        acceleration = _finite_vector(linear_acceleration)
        if (
            not math.isfinite(timestamp_s)
            or gyro is None
            or acceleration is None
        ):
            return self.reject_sample("nonfinite_sample")

        if self._last_timestamp is not None:
            if timestamp_s <= self._last_timestamp:
                self._last_timestamp = timestamp_s
                self._restart_warmup(timestamp_s)
                self._state = "timestamp_reset"
                return self._observation(
                    False, False, "timestamp_reset", False
                )
            if timestamp_s - self._last_timestamp > (
                self.config.max_sample_gap_s
            ):
                self._last_timestamp = timestamp_s
                self._restart_warmup(timestamp_s)
                self._state = "dropout"
                return self._observation(False, False, "dropout", False)
        self._last_timestamp = timestamp_s

        if not self._calibrated:
            if self._warmup_start is None:
                self._warmup_start = timestamp_s
            self._warmup_samples += 1
            warmup_elapsed = timestamp_s - self._warmup_start
            if (
                self._warmup_samples < self.config.warmup_min_samples
                or warmup_elapsed < self.config.warmup_duration_s
            ):
                self._state = "warming_up"
                return self._observation(
                    True, False, "warming_up", False
                )

        corrected_gyro = (
            tuple(gyro[axis] - self._bias[axis] for axis in range(3))
            if self._calibrated
            else gyro
        )
        command_allows_stationary = command_zero is not False
        if self.config.require_command_zero:
            command_allows_stationary = command_zero is True
        stationary = (
            _norm(corrected_gyro)
            <= self.config.gyro_stationary_threshold_rad_s
            and abs(_norm(acceleration) - self.config.gravity_m_s2)
            <= self.config.acceleration_tolerance_m_s2
            and command_allows_stationary
        )
        if not stationary:
            self._clear_candidate()
            self._state = (
                "holding_motion"
                if self._calibrated
                else "waiting_stationary"
            )
            return self._observation(True, False, "motion", False)

        if self._calibrated and not self.config.online_update_enabled:
            self._state = "calibrated"
            return self._observation(True, True, "calibrated", False)

        if self._candidate_start is None:
            self._candidate_start = timestamp_s
        self._candidate_samples.append(gyro)
        window_elapsed = timestamp_s - self._candidate_start
        if (
            len(self._candidate_samples)
            < self.config.stationary_min_samples
            or window_elapsed < self.config.stationary_window_duration_s
        ):
            self._state = (
                "collecting_update"
                if self._calibrated
                else "collecting_initial"
            )
            return self._observation(True, True, "collecting", False)

        candidate_mean = _mean(self._candidate_samples)
        candidate_residual = _stddev(
            self._candidate_samples, candidate_mean
        )
        sample_count = len(self._candidate_samples)
        if max(candidate_residual) > (
            self.config.max_residual_stddev_rad_s
        ):
            self._clear_candidate()
            self._state = "unstable_stationary_window"
            return self._observation(
                True, True, "unstable_stationary_window", False
            )

        if self._calibrated:
            alpha = self.config.online_update_alpha
            self._bias = tuple(
                (1.0 - alpha) * self._bias[axis]
                + alpha * candidate_mean[axis]
                for axis in range(3)
            )
        else:
            self._bias = candidate_mean
            self._calibrated = True
        self._residual = candidate_residual
        self._last_update_samples = sample_count
        self._last_update_timestamp = timestamp_s
        self._clear_candidate()
        self._state = "calibrated"
        return self._observation(True, True, "bias_updated", True)
