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

import math

import pytest

from realsense_imu.imu_transform import quaternion_to_rotation_matrix
from realsense_imu.imu_transform import rotate_covariance
from realsense_imu.imu_transform import rotate_vector
from realsense_imu.imu_transform import RotationValidationError
from realsense_imu.imu_transform import validate_rotation_matrix


IDENTITY = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


def test_known_identity_and_quarter_turn_vector_rotations():
    assert rotate_vector(IDENTITY, (1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0)

    half_angle = math.pi / 4.0
    rotation = quaternion_to_rotation_matrix(
        (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))
    )
    assert rotate_vector(rotation, (1.0, 0.0, 0.0)) == pytest.approx(
        (0.0, 1.0, 0.0)
    )


def test_observed_optical_positive_y_can_map_to_processed_positive_z():
    rotation = quaternion_to_rotation_matrix((0.5, 0.5, 0.5, 0.5))

    assert rotate_vector(rotation, (0.0, 1.0, 0.0)) == pytest.approx(
        (0.0, 0.0, 1.0)
    )


def test_covariance_rotation_preserves_off_diagonal_terms():
    rotation = quaternion_to_rotation_matrix((0.5, 0.5, 0.5, 0.5))
    covariance = (
        1.0, 0.1, 0.2,
        0.1, 2.0, 0.3,
        0.2, 0.3, 3.0,
    )

    result = rotate_covariance(rotation, covariance)

    assert result == pytest.approx(
        (
            3.0, 0.2, 0.3,
            0.2, 1.0, 0.1,
            0.3, 0.1, 2.0,
        )
    )


@pytest.mark.parametrize(
    "quaternion",
    [
        (0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 2.0),
        (0.0, 0.0, float("nan"), 1.0),
        (0.0, 0.0, 1.0),
    ],
)
def test_invalid_quaternion_is_rejected(quaternion):
    with pytest.raises(RotationValidationError):
        quaternion_to_rotation_matrix(quaternion)


def test_non_orthonormal_matrix_is_rejected():
    invalid = (
        (1.0, 0.0, 0.0),
        (0.0, 2.0, 0.0),
        (0.0, 0.0, 1.0),
    )

    with pytest.raises(RotationValidationError, match="orthonormal"):
        validate_rotation_matrix(invalid)


def test_nonfinite_matrix_is_rejected():
    invalid = (
        (1.0, 0.0, 0.0),
        (0.0, float("nan"), 0.0),
        (0.0, 0.0, 1.0),
    )

    with pytest.raises(RotationValidationError, match="finite"):
        validate_rotation_matrix(invalid)


def test_determinant_negative_one_reflection_is_rejected():
    reflection = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, -1.0),
    )

    with pytest.raises(RotationValidationError, match="reflection"):
        validate_rotation_matrix(reflection)
