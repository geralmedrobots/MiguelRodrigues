#include "roboteq_ros2_driver/roboteq_command_conversion.hpp"

#include <gtest/gtest.h>

namespace conversion = roboteq_ros2_driver::command_conversion;

TEST(CommandConversion, ConvertsForwardCommandToEqualWheelSpeeds)
{
  const auto wheels = conversion::twist_to_wheel_speeds(0.4, 0.0, 0.453);

  EXPECT_DOUBLE_EQ(wheels.left_mps, 0.4);
  EXPECT_DOUBLE_EQ(wheels.right_mps, 0.4);
}

TEST(CommandConversion, AppliesExplicitAngularConvention)
{
  const auto wheels = conversion::twist_to_wheel_speeds(0.0, 1.0, 0.4, -1);

  EXPECT_DOUBLE_EQ(wheels.left_mps, 0.2);
  EXPECT_DOUBLE_EQ(wheels.right_mps, -0.2);

  const auto neutral = conversion::twist_to_wheel_speeds(0.0, 1.0, 0.4, 1);
  EXPECT_DOUBLE_EQ(neutral.left_mps, -0.2);
  EXPECT_DOUBLE_EQ(neutral.right_mps, 0.2);
}

TEST(CommandConversion, MapsDefaultChannelsRightThenLeft)
{
  const conversion::WheelSpeeds wheels{1.0, 2.0};
  const auto channels = conversion::wheels_to_channels(wheels, "right", "left");

  ASSERT_TRUE(channels.has_value());
  EXPECT_DOUBLE_EQ(channels->channel_1_mps, 2.0);
  EXPECT_DOUBLE_EQ(channels->channel_2_mps, 1.0);
}

TEST(CommandConversion, MapsSwappedChannelsLeftThenRight)
{
  const conversion::WheelSpeeds wheels{1.0, 2.0};
  const auto channels = conversion::wheels_to_channels(wheels, "left", "right");

  ASSERT_TRUE(channels.has_value());
  EXPECT_DOUBLE_EQ(channels->channel_1_mps, 1.0);
  EXPECT_DOUBLE_EQ(channels->channel_2_mps, 2.0);
}

TEST(CommandConversion, RejectsInvalidChannelMapping)
{
  const conversion::WheelSpeeds wheels{1.0, 2.0};

  EXPECT_FALSE(conversion::wheels_to_channels(wheels, "right", "right").has_value());
}

TEST(CommandConversion, AppliesExplicitMotorSigns)
{
  const conversion::ChannelSpeeds channels{2.0, -3.0};
  const auto signed_channels = conversion::apply_motor_signs(channels, -1, 1);

  ASSERT_TRUE(signed_channels.has_value());
  EXPECT_DOUBLE_EQ(signed_channels->channel_1_mps, -2.0);
  EXPECT_DOUBLE_EQ(signed_channels->channel_2_mps, -3.0);
  EXPECT_FALSE(conversion::apply_motor_signs(channels, 0, 1).has_value());
  EXPECT_FALSE(conversion::apply_motor_signs(channels, 1, 2).has_value());
}

TEST(CommandConversion, ConvertsTwistToChannelsWithExplicitAngularSign)
{
  const auto channels = conversion::twist_to_channel_speeds(
    0.0, 1.0, 0.4, "right", "left", -1);

  ASSERT_TRUE(channels.has_value());
  EXPECT_DOUBLE_EQ(channels->channel_1_mps, -0.2);
  EXPECT_DOUBLE_EQ(channels->channel_2_mps, 0.2);
}
