#include "roboteq_ros2_driver/roboteq_command_conversion.hpp"

namespace roboteq_ros2_driver
{
namespace command_conversion
{
WheelSpeeds twist_to_wheel_speeds(
  double linear_x,
  double angular_z,
  double wheelbase,
  int command_angular_sign)
{
  const double corrected_angular_z = static_cast<double>(command_angular_sign) * angular_z;
  return {
    linear_x - (wheelbase * corrected_angular_z / 2.0),
    linear_x + (wheelbase * corrected_angular_z / 2.0)};
}

std::optional<ChannelSpeeds> wheels_to_channels(
  const WheelSpeeds & wheels,
  const std::string & channel_1,
  const std::string & channel_2)
{
  if (channel_1 == "right" && channel_2 == "left") {
    return ChannelSpeeds{wheels.right_mps, wheels.left_mps};
  }
  if (channel_1 == "left" && channel_2 == "right") {
    return ChannelSpeeds{wheels.left_mps, wheels.right_mps};
  }
  return std::nullopt;
}

std::optional<ChannelSpeeds> twist_to_channel_speeds(
  double linear_x,
  double angular_z,
  double wheelbase,
  const std::string & channel_1,
  const std::string & channel_2,
  int command_angular_sign)
{
  return wheels_to_channels(
    twist_to_wheel_speeds(linear_x, angular_z, wheelbase, command_angular_sign),
    channel_1,
    channel_2);
}

std::optional<ChannelSpeeds> apply_motor_signs(
  const ChannelSpeeds & channels,
  int motor_sign_1,
  int motor_sign_2)
{
  if ((motor_sign_1 != -1 && motor_sign_1 != 1) ||
    (motor_sign_2 != -1 && motor_sign_2 != 1))
  {
    return std::nullopt;
  }
  return ChannelSpeeds{
    motor_sign_1 * channels.channel_1_mps,
      motor_sign_2 * channels.channel_2_mps};
}

}  // namespace command_conversion
}  // namespace roboteq_ros2_driver
