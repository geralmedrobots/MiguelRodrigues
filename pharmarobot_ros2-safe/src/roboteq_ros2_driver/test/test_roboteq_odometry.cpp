#include "roboteq_ros2_driver/roboteq_odometry.hpp"

#include <climits>

#include <gtest/gtest.h>

namespace odometry = roboteq_ros2_driver::odometry;

TEST(RoboteqOdometry, MapsDefaultChannelsAndPreservesEncoderSignCorrection)
{
  const auto ticks = odometry::map_channel_encoder_sample_to_wheels(
    10, 20, "right", "left");

  ASSERT_TRUE(ticks.has_value());
  EXPECT_EQ(ticks->left_ticks, -20);
  EXPECT_EQ(ticks->right_ticks, -10);
}

TEST(RoboteqOdometry, MapsSwappedChannelsAndPreservesEncoderSignCorrection)
{
  const auto ticks = odometry::map_channel_encoder_sample_to_wheels(
    10, 20, "left", "right");

  ASSERT_TRUE(ticks.has_value());
  EXPECT_EQ(ticks->left_ticks, -10);
  EXPECT_EQ(ticks->right_ticks, -20);
}

TEST(RoboteqOdometry, RejectsInvalidEncoderOrChannelMapping)
{
  EXPECT_FALSE(odometry::map_channel_encoder_sample_to_wheels(
    INT_MAX, 20, "right", "left").has_value());
  EXPECT_FALSE(odometry::map_channel_encoder_sample_to_wheels(
    10, 20, "right", "right").has_value());
}

TEST(RoboteqOdometry, IntegratesFirstSampleWithZeroTwist)
{
  odometry::OdometryIntegrator integrator;
  integrator.init(0.1, 0.5, -100);

  const auto result = integrator.integrate_channel_sample(
    10, 10, 1.0, "right", "left");

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->ticks.left_ticks, -10);
  EXPECT_EQ(result->ticks.right_ticks, -10);
  EXPECT_NEAR(result->pose.x, 2.0 * 3.14159265358979323846 * 0.1 * 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(result->pose.y, 0.0);
  EXPECT_DOUBLE_EQ(result->pose.theta, 0.0);
  EXPECT_DOUBLE_EQ(result->twist.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(result->twist.angular_z, 0.0);
}

TEST(RoboteqOdometry, CalculatesTwistAfterFirstSample)
{
  odometry::OdometryIntegrator integrator;
  integrator.init(0.1, 0.5, -100);

  ASSERT_TRUE(integrator.integrate_channel_sample(10, 10, 1.0, "right", "left").has_value());
  const auto result = integrator.integrate_channel_sample(
    10, 10, 2.0, "right", "left");

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->twist.linear_x, (2.0 * 3.14159265358979323846 * 0.1 * 0.1) / 2.0, 1e-12);
  EXPECT_DOUBLE_EQ(result->twist.angular_z, 0.0);
}
