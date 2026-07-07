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

#include <cmath>
#include <limits>

#include "roboteq_ros2_driver/odom_twist.hpp"

namespace odom_twist = roboteq_ros2_driver::odom_twist;
constexpr double kPi = 3.14159265358979323846;

TEST(OdomTwist, ReturnsZeroForFirstSample)
{
  const auto twist = odom_twist::calculate_measured_twist(1.0, 0.0, 0.5, 0.1, false);

  EXPECT_DOUBLE_EQ(twist.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular_z, 0.0);
}

TEST(OdomTwist, CalculatesForwardVelocity)
{
  const auto twist = odom_twist::calculate_measured_twist(0.5, 0.0, 0.0, 2.0, true);

  EXPECT_DOUBLE_EQ(twist.linear_x, 0.25);
  EXPECT_DOUBLE_EQ(twist.angular_z, 0.0);
}

TEST(OdomTwist, CalculatesReverseVelocity)
{
  const auto twist = odom_twist::calculate_measured_twist(-0.4, 1.0, 1.0, 2.0, true);

  EXPECT_DOUBLE_EQ(twist.linear_x, -0.2);
  EXPECT_DOUBLE_EQ(twist.angular_z, 0.0);
}

TEST(OdomTwist, CalculatesPureRotationVelocity)
{
  const auto twist = odom_twist::calculate_measured_twist(0.0, 0.25, 0.75, 0.5, true);

  EXPECT_DOUBLE_EQ(twist.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular_z, 1.0);
}

TEST(OdomTwist, ReturnsZeroForStoppedRobot)
{
  const auto twist = odom_twist::calculate_measured_twist(0.0, -0.4, -0.4, 1.0, true);

  EXPECT_DOUBLE_EQ(twist.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular_z, 0.0);
}

TEST(OdomTwist, ReturnsZeroForInvalidDt)
{
  const auto zero_dt = odom_twist::calculate_measured_twist(0.5, 0.0, 0.1, 0.0, true);
  const auto negative_dt = odom_twist::calculate_measured_twist(0.5, 0.0, 0.1, -1.0, true);
  const auto non_finite_dt = odom_twist::calculate_measured_twist(
    0.5, 0.0, 0.1, std::numeric_limits<double>::infinity(), true);

  EXPECT_DOUBLE_EQ(zero_dt.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(zero_dt.angular_z, 0.0);
  EXPECT_DOUBLE_EQ(negative_dt.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(negative_dt.angular_z, 0.0);
  EXPECT_DOUBLE_EQ(non_finite_dt.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(non_finite_dt.angular_z, 0.0);
}

TEST(OdomTwist, ReturnsZeroForNonFiniteInputs)
{
  const auto non_finite_linear = odom_twist::calculate_measured_twist(
    std::numeric_limits<double>::infinity(), 0.0, 0.1, 1.0, true);
  const auto non_finite_previous_yaw = odom_twist::calculate_measured_twist(
    0.1, std::numeric_limits<double>::quiet_NaN(), 0.1, 1.0, true);
  const auto non_finite_current_yaw = odom_twist::calculate_measured_twist(
    0.1, 0.0, std::numeric_limits<double>::infinity(), 1.0, true);

  EXPECT_DOUBLE_EQ(non_finite_linear.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(non_finite_linear.angular_z, 0.0);
  EXPECT_DOUBLE_EQ(non_finite_previous_yaw.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(non_finite_previous_yaw.angular_z, 0.0);
  EXPECT_DOUBLE_EQ(non_finite_current_yaw.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(non_finite_current_yaw.angular_z, 0.0);
}

TEST(OdomTwist, NormalizesYawDeltaAcrossWraparound)
{
  const double previous_yaw = kPi - 0.01;
  const double current_yaw = -kPi + 0.01;

  const auto twist = odom_twist::calculate_measured_twist(
    0.0, previous_yaw, current_yaw, 0.5, true);

  EXPECT_DOUBLE_EQ(twist.linear_x, 0.0);
  EXPECT_NEAR(twist.angular_z, 0.04, 1e-12);
}
