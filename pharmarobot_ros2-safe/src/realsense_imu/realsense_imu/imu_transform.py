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

"""Validated 3D rotation helpers for D455 IMU measurements."""

import math
from typing import Iterable, Tuple


Vector3 = Tuple[float, float, float]
Matrix3 = Tuple[Vector3, Vector3, Vector3]
Covariance3 = Tuple[
    float, float, float,
    float, float, float,
    float, float, float,
]


class RotationValidationError(ValueError):
    """Report an invalid or reflective configured rotation."""


def _finite_values(values: Iterable[float], expected: int, name: str):
    result = tuple(float(value) for value in values)
    if len(result) != expected:
        raise RotationValidationError(
            f"{name} must contain exactly {expected} values"
        )
    if not all(math.isfinite(value) for value in result):
        raise RotationValidationError(
            f"{name} must contain only finite values"
        )
    return result


def determinant(matrix: Matrix3) -> float:
    """Return the determinant of one 3x3 row-major matrix."""
    return (
        matrix[0][0] * (
            matrix[1][1] * matrix[2][2]
            - matrix[1][2] * matrix[2][1]
        )
        - matrix[0][1] * (
            matrix[1][0] * matrix[2][2]
            - matrix[1][2] * matrix[2][0]
        )
        + matrix[0][2] * (
            matrix[1][0] * matrix[2][1]
            - matrix[1][1] * matrix[2][0]
        )
    )


def validate_rotation_matrix(
    values: Iterable[Iterable[float]],
    *,
    tolerance: float = 1.0e-6,
) -> Matrix3:
    """Require a finite, orthonormal, right-handed 3D rotation matrix."""
    rows = tuple(
        _finite_values(row, 3, f"rotation row {index}")
        for index, row in enumerate(values)
    )
    if len(rows) != 3:
        raise RotationValidationError(
            "rotation matrix must contain exactly 3 rows"
        )
    matrix = (rows[0], rows[1], rows[2])

    for row_index in range(3):
        for other_index in range(3):
            dot = sum(
                matrix[row_index][axis] * matrix[other_index][axis]
                for axis in range(3)
            )
            expected = 1.0 if row_index == other_index else 0.0
            if abs(dot - expected) > tolerance:
                raise RotationValidationError(
                    "rotation matrix must be orthonormal"
                )

    matrix_determinant = determinant(matrix)
    if abs(matrix_determinant - 1.0) > tolerance:
        if abs(matrix_determinant + 1.0) <= tolerance:
            raise RotationValidationError(
                "rotation matrix is a reflection with determinant -1"
            )
        raise RotationValidationError(
            "rotation matrix determinant must be +1"
        )
    return matrix


def quaternion_to_rotation_matrix(
    quaternion_xyzw: Iterable[float],
    *,
    tolerance: float = 1.0e-6,
) -> Matrix3:
    """Convert one normalized finite xyzw quaternion to a checked matrix."""
    x, y, z, w = _finite_values(
        quaternion_xyzw, 4, "rotation quaternion"
    )
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if abs(norm - 1.0) > tolerance:
        raise RotationValidationError(
            "rotation quaternion must be normalized"
        )

    matrix = (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )
    return validate_rotation_matrix(matrix, tolerance=tolerance)


def rotate_vector(matrix: Matrix3, vector: Iterable[float]) -> Vector3:
    """Rotate one finite 3D vector."""
    checked = _finite_values(vector, 3, "vector")
    return tuple(
        sum(matrix[row][column] * checked[column] for column in range(3))
        for row in range(3)
    )


def rotate_covariance(
    matrix: Matrix3,
    covariance: Iterable[float],
) -> Covariance3:
    """Rotate a row-major covariance as R * covariance * R-transpose."""
    flat = _finite_values(covariance, 9, "covariance")
    source = tuple(
        tuple(flat[row * 3 + column] for column in range(3))
        for row in range(3)
    )
    left = tuple(
        tuple(
            sum(
                matrix[row][axis] * source[axis][column]
                for axis in range(3)
            )
            for column in range(3)
        )
        for row in range(3)
    )
    rotated = tuple(
        tuple(
            sum(
                left[row][axis] * matrix[column][axis]
                for axis in range(3)
            )
            for column in range(3)
        )
        for row in range(3)
    )
    return tuple(
        rotated[row][column]
        for row in range(3)
        for column in range(3)
    )
