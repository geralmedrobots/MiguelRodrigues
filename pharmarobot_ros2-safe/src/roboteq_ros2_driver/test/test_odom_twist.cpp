#include "roboteq_ros2_driver/odom_twist.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

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
