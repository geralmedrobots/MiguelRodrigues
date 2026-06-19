#ifndef ROBOTEQ_ROS2_DRIVER__ROBOTEQ_COMMAND_CONVERSION_HPP_
#define ROBOTEQ_ROS2_DRIVER__ROBOTEQ_COMMAND_CONVERSION_HPP_

#include <optional>
#include <string>

namespace roboteq_ros2_driver
{
namespace command_conversion
{

struct WheelSpeeds
{
  double left_mps;
  double right_mps;
};

struct ChannelSpeeds
{
  double channel_1_mps;
  double channel_2_mps;
};

WheelSpeeds twist_to_wheel_speeds(
  double linear_x,
  double angular_z,
  double wheelbase);

std::optional<ChannelSpeeds> wheels_to_channels(
  const WheelSpeeds & wheels,
  const std::string & channel_1,
  const std::string & channel_2);

std::optional<ChannelSpeeds> twist_to_channel_speeds(
  double linear_x,
  double angular_z,
  double wheelbase,
  const std::string & channel_1,
  const std::string & channel_2);

}  // namespace command_conversion
}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__ROBOTEQ_COMMAND_CONVERSION_HPP_
