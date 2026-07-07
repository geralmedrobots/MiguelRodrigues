// Copyright 2026 Medrobots
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>

#include "roboteq_ros2_driver/odom_covariance.hpp"

namespace covariance = roboteq_ros2_driver::odom_covariance;

void expect_off_diagonals_zero(const covariance::CovarianceMatrix & matrix)
{
  for (std::size_t i = 0; i < matrix.size(); ++i) {
    if (i == covariance::kXIndex || i == covariance::kYIndex || i == covariance::kZIndex ||
      i == covariance::kRollIndex || i == covariance::kPitchIndex || i == covariance::kYawIndex)
    {
      continue;
    }
    EXPECT_DOUBLE_EQ(matrix[i], 0.0) << "index " << i;
  }
}

TEST(OdomCovariance, AssignsDefaultPoseDiagonal)
{
  const auto config = covariance::default_config();
  const auto matrix = covariance::build_pose_covariance(config);

  EXPECT_DOUBLE_EQ(matrix[covariance::kXIndex], config.pose_x);
  EXPECT_DOUBLE_EQ(matrix[covariance::kYIndex], config.pose_y);
  EXPECT_DOUBLE_EQ(matrix[covariance::kZIndex], config.pose_z);
  EXPECT_DOUBLE_EQ(matrix[covariance::kRollIndex], config.pose_roll);
  EXPECT_DOUBLE_EQ(matrix[covariance::kPitchIndex], config.pose_pitch);
  EXPECT_DOUBLE_EQ(matrix[covariance::kYawIndex], config.pose_yaw);
  expect_off_diagonals_zero(matrix);
}

TEST(OdomCovariance, AssignsDefaultTwistDiagonal)
{
  const auto config = covariance::default_config();
  const auto matrix = covariance::build_twist_covariance(config);

  EXPECT_DOUBLE_EQ(matrix[covariance::kXIndex], config.twist_linear_x);
  EXPECT_DOUBLE_EQ(matrix[covariance::kYIndex], config.twist_linear_y);
  EXPECT_DOUBLE_EQ(matrix[covariance::kZIndex], config.twist_linear_z);
  EXPECT_DOUBLE_EQ(matrix[covariance::kRollIndex], config.twist_angular_x);
  EXPECT_DOUBLE_EQ(matrix[covariance::kPitchIndex], config.twist_angular_y);
  EXPECT_DOUBLE_EQ(matrix[covariance::kYawIndex], config.twist_angular_z);
  expect_off_diagonals_zero(matrix);
}

TEST(OdomCovariance, UsesHighCovarianceForUnobservedDofs)
{
  const auto config = covariance::default_config();
  const auto pose = covariance::build_pose_covariance(config);
  const auto twist = covariance::build_twist_covariance(config);

  EXPECT_GE(pose[covariance::kZIndex], 1000000.0);
  EXPECT_GE(pose[covariance::kRollIndex], 1000000.0);
  EXPECT_GE(pose[covariance::kPitchIndex], 1000000.0);
  EXPECT_GE(twist[covariance::kYIndex], 1000000.0);
  EXPECT_GE(twist[covariance::kZIndex], 1000000.0);
  EXPECT_GE(twist[covariance::kRollIndex], 1000000.0);
  EXPECT_GE(twist[covariance::kPitchIndex], 1000000.0);
}

TEST(OdomCovariance, AssignsCustomPoseAndTwistValues)
{
  const covariance::OdometryCovarianceConfig config{
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
    7.0, 8.0, 9.0, 10.0, 11.0, 12.0};

  const auto pose = covariance::build_pose_covariance(config);
  const auto twist = covariance::build_twist_covariance(config);

  EXPECT_DOUBLE_EQ(pose[covariance::kXIndex], 1.0);
  EXPECT_DOUBLE_EQ(pose[covariance::kYIndex], 2.0);
  EXPECT_DOUBLE_EQ(pose[covariance::kZIndex], 3.0);
  EXPECT_DOUBLE_EQ(pose[covariance::kRollIndex], 4.0);
  EXPECT_DOUBLE_EQ(pose[covariance::kPitchIndex], 5.0);
  EXPECT_DOUBLE_EQ(pose[covariance::kYawIndex], 6.0);
  EXPECT_DOUBLE_EQ(twist[covariance::kXIndex], 7.0);
  EXPECT_DOUBLE_EQ(twist[covariance::kYIndex], 8.0);
  EXPECT_DOUBLE_EQ(twist[covariance::kZIndex], 9.0);
  EXPECT_DOUBLE_EQ(twist[covariance::kRollIndex], 10.0);
  EXPECT_DOUBLE_EQ(twist[covariance::kPitchIndex], 11.0);
  EXPECT_DOUBLE_EQ(twist[covariance::kYawIndex], 12.0);
}

TEST(OdomCovariance, SanitizesNegativeAndNonFiniteValues)
{
  const auto defaults = covariance::default_config();
  const covariance::OdometryCovarianceConfig config{
    -1.0,
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(),
    4.0,
    5.0,
    6.0,
    -7.0,
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(),
    10.0,
    11.0,
    12.0};

  const auto sanitized = covariance::sanitize_config(config);
  const auto pose = covariance::build_pose_covariance(config);
  const auto twist = covariance::build_twist_covariance(config);

  EXPECT_DOUBLE_EQ(sanitized.pose_x, defaults.pose_x);
  EXPECT_DOUBLE_EQ(sanitized.pose_y, defaults.pose_y);
  EXPECT_DOUBLE_EQ(sanitized.pose_z, defaults.pose_z);
  EXPECT_DOUBLE_EQ(sanitized.pose_roll, 4.0);
  EXPECT_DOUBLE_EQ(sanitized.pose_pitch, 5.0);
  EXPECT_DOUBLE_EQ(sanitized.pose_yaw, 6.0);
  EXPECT_DOUBLE_EQ(sanitized.twist_linear_x, defaults.twist_linear_x);
  EXPECT_DOUBLE_EQ(sanitized.twist_linear_y, defaults.twist_linear_y);
  EXPECT_DOUBLE_EQ(sanitized.twist_linear_z, defaults.twist_linear_z);
  EXPECT_DOUBLE_EQ(sanitized.twist_angular_x, 10.0);
  EXPECT_DOUBLE_EQ(sanitized.twist_angular_y, 11.0);
  EXPECT_DOUBLE_EQ(sanitized.twist_angular_z, 12.0);

  EXPECT_DOUBLE_EQ(pose[covariance::kXIndex], defaults.pose_x);
  EXPECT_DOUBLE_EQ(pose[covariance::kYIndex], defaults.pose_y);
  EXPECT_DOUBLE_EQ(pose[covariance::kZIndex], defaults.pose_z);
  EXPECT_DOUBLE_EQ(twist[covariance::kXIndex], defaults.twist_linear_x);
  EXPECT_DOUBLE_EQ(twist[covariance::kYIndex], defaults.twist_linear_y);
  EXPECT_DOUBLE_EQ(twist[covariance::kZIndex], defaults.twist_linear_z);
}
