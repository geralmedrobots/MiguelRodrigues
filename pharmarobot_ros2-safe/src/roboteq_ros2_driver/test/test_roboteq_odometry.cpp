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

#include <chrono>
#include <climits>
#include <cmath>
#include <limits>

#include "roboteq_ros2_driver/roboteq_odometry.hpp"

namespace odometry = roboteq_ros2_driver::odometry;
constexpr double kPi = 3.14159265358979323846;

TEST(RoboteqOdometry, MapsDefaultChannelsWithExplicitPositiveEncoderSigns)
{
  const auto ticks = odometry::map_channel_encoder_sample_to_wheels(
    10, 20, "right", "left");

  ASSERT_TRUE(ticks.has_value());
  EXPECT_EQ(ticks->left_ticks, 20);
  EXPECT_EQ(ticks->right_ticks, 10);
}

TEST(RoboteqOdometry, MapsSwappedChannelsWithExplicitPositiveEncoderSigns)
{
  const auto ticks = odometry::map_channel_encoder_sample_to_wheels(
    10, 20, "left", "right");

  ASSERT_TRUE(ticks.has_value());
  EXPECT_EQ(ticks->left_ticks, 10);
  EXPECT_EQ(ticks->right_ticks, 20);
}

TEST(RoboteqOdometry, AppliesEachEncoderSignExactlyOnce)
{
  const auto positive = odometry::map_channel_encoder_sample_to_wheels(
    10, 20, "right", "left", 1, 1);
  const auto mixed = odometry::map_channel_encoder_sample_to_wheels(
    10, 20, "right", "left", -1, 1);
  const auto negative = odometry::map_channel_encoder_sample_to_wheels(
    10, 20, "right", "left", -1, -1);

  ASSERT_TRUE(positive.has_value());
  ASSERT_TRUE(mixed.has_value());
  ASSERT_TRUE(negative.has_value());
  EXPECT_EQ(positive->left_ticks, 20);
  EXPECT_EQ(positive->right_ticks, 10);
  EXPECT_EQ(mixed->left_ticks, 20);
  EXPECT_EQ(mixed->right_ticks, -10);
  EXPECT_EQ(negative->left_ticks, -20);
  EXPECT_EQ(negative->right_ticks, -10);
}

TEST(RoboteqOdometry, RejectsInvalidEncoderSignsAndChannelMappings)
{
  EXPECT_FALSE(
    odometry::map_channel_encoder_sample_to_wheels(
      INT_MAX, 20, "right", "left").has_value());
  EXPECT_FALSE(
    odometry::map_channel_encoder_sample_to_wheels(
      10, 20, "right", "right").has_value());
  EXPECT_FALSE(
    odometry::map_channel_encoder_sample_to_wheels(
      10, 20, "right", "left", 0, 1).has_value());
  EXPECT_FALSE(
    odometry::map_channel_encoder_sample_to_wheels(
      INT_MIN, 20, "right", "left", -1, -1).has_value());
}

TEST(RoboteqOdometry, HandlesFirstSampleWithZeroTwistAndIntegratesPose)
{
  odometry::OdometryIntegrator integrator;
  integrator.init(0.1, 0.5, 100);

  const auto result = integrator.integrate_channel_sample(
    10, 10, 0.0, "right", "left");

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->ticks.left_ticks, 10);
  EXPECT_EQ(result->ticks.right_ticks, 10);
  EXPECT_NEAR(result->pose.x, 2.0 * kPi * 0.1 * 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(result->pose.y, 0.0);
  EXPECT_DOUBLE_EQ(result->pose.theta, 0.0);
  EXPECT_DOUBLE_EQ(result->twist.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(result->twist.angular_z, 0.0);
}

TEST(RoboteqOdometry, FirstSampleIgnoresInvalidDtAndStillProducesZeroTwist)
{
  odometry::OdometryIntegrator integrator;
  integrator.init(0.1, 0.5, 100);

  const auto result = integrator.integrate_channel_sample(
    10, 10, std::numeric_limits<double>::quiet_NaN(), "right", "left");

  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->twist.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(result->twist.angular_z, 0.0);
}

TEST(RoboteqOdometry, RejectsInvalidDtAfterFirstSample)
{
  odometry::OdometryIntegrator integrator;
  integrator.init(0.1, 0.5, 100);

  ASSERT_TRUE(integrator.integrate_channel_sample(10, 10, 0.0, "right", "left").has_value());

  EXPECT_FALSE(integrator.integrate_channel_sample(20, 20, 0.0, "right", "left").has_value());
  EXPECT_FALSE(integrator.integrate_channel_sample(20, 20, -1.0, "right", "left").has_value());
  EXPECT_FALSE(
    integrator.integrate_channel_sample(
      20, 20, std::numeric_limits<double>::quiet_NaN(), "right", "left").has_value());
  EXPECT_FALSE(
    integrator.integrate_channel_sample(
      20, 20, std::numeric_limits<double>::infinity(), "right", "left").has_value());

  const auto result = integrator.integrate_channel_sample(30, 30, 2.0, "right", "left");

  ASSERT_TRUE(result.has_value());
  const double first_sample_distance = 2.0 * kPi * 0.1 * 0.1;
  const double second_sample_distance = 2.0 * kPi * 0.1 * 0.3;
  EXPECT_NEAR(result->pose.x, first_sample_distance + second_sample_distance, 1e-12);
  EXPECT_DOUBLE_EQ(result->pose.y, 0.0);
  EXPECT_DOUBLE_EQ(result->pose.theta, 0.0);
}

TEST(RoboteqOdometry, IntegratesReverseMotionWithNegativeEncoderSign)
{
  odometry::OdometryIntegrator integrator;
  integrator.init(0.1, 0.5, 100);

  const auto result = integrator.integrate_channel_sample(
    10, 10, 1.0, "right", "left", -1, -1);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->ticks.left_ticks, -10);
  EXPECT_EQ(result->ticks.right_ticks, -10);
  EXPECT_NEAR(result->pose.x, -2.0 * 3.14159265358979323846 * 0.1 * 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(result->pose.y, 0.0);
  EXPECT_DOUBLE_EQ(result->pose.theta, 0.0);
}

TEST(RoboteqOdometry, IntegratesTurningMotionAndRotationDirection)
{
  odometry::OdometryIntegrator integrator;
  integrator.init(0.1, 0.5, 100);

  ASSERT_TRUE(integrator.integrate_channel_sample(10, 10, 1.0, "right", "left").has_value());
  const auto result = integrator.integrate_channel_sample(
    10, 20, 1.0, "right", "left");

  ASSERT_TRUE(result.has_value());
  EXPECT_LT(result->pose.theta, 0.0);
  EXPECT_NE(result->twist.angular_z, 0.0);
}

TEST(RoboteqOdometry, CalculatesExpectedLinearAndAngularTwistForKnownTickDeltaAndDuration)
{
  odometry::OdometryIntegrator integrator;
  integrator.init(0.1, 0.5, 100);

  ASSERT_TRUE(integrator.integrate_channel_sample(0, 0, 0.0, "right", "left").has_value());
  const auto result = integrator.integrate_channel_sample(
    20, 10, 0.25, "right", "left");

  ASSERT_TRUE(result.has_value());
  const double meters_per_tick = (2.0 * kPi * 0.1) / 100.0;
  const double left_distance = meters_per_tick * 10.0;
  const double right_distance = meters_per_tick * 20.0;
  const double expected_linear_delta = (left_distance + right_distance) / 2.0;
  const double expected_angular_delta = (right_distance - left_distance) / 0.5;
  EXPECT_NEAR(result->twist.linear_x, expected_linear_delta / 0.25, 1e-12);
  EXPECT_NEAR(result->twist.angular_z, expected_angular_delta / 0.25, 1e-12);
}

TEST(RoboteqOdometry, ValidatesElapsedIntervals)
{
  using namespace std::chrono_literals;

  EXPECT_TRUE(odometry::is_valid_elapsed_interval(1e-12));
  EXPECT_FALSE(odometry::is_valid_elapsed_interval(0.0));
  EXPECT_FALSE(odometry::is_valid_elapsed_interval(-1e-12));
  EXPECT_FALSE(
    odometry::is_valid_elapsed_interval(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(
    odometry::is_valid_elapsed_interval(std::numeric_limits<double>::infinity()));

  const auto start = std::chrono::steady_clock::time_point{};
  const auto normal = odometry::monotonic_elapsed_interval(start, start + 250ms);
  ASSERT_TRUE(normal.has_value());
  EXPECT_DOUBLE_EQ(*normal, 0.25);

  const auto tiny = odometry::monotonic_elapsed_interval(start, start + 1ns);
  ASSERT_TRUE(tiny.has_value());
  EXPECT_DOUBLE_EQ(*tiny, 1e-9);

  EXPECT_FALSE(odometry::monotonic_elapsed_interval(start, start).has_value());
  EXPECT_FALSE(odometry::monotonic_elapsed_interval(start + 10ns, start).has_value());
}

TEST(RoboteqOdometry, MonotonicElapsedIntervalDependsOnlyOnDuration)
{
  using namespace std::chrono_literals;

  const auto epoch_a = std::chrono::steady_clock::time_point{} + 2s;
  const auto epoch_b = std::chrono::steady_clock::time_point{} + 2000s;

  const auto dt_a = odometry::monotonic_elapsed_interval(epoch_a, epoch_a + 125ms);
  const auto dt_b = odometry::monotonic_elapsed_interval(epoch_b, epoch_b + 125ms);

  ASSERT_TRUE(dt_a.has_value());
  ASSERT_TRUE(dt_b.has_value());
  EXPECT_DOUBLE_EQ(*dt_a, 0.125);
  EXPECT_DOUBLE_EQ(*dt_b, 0.125);
}
