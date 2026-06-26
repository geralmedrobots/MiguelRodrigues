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
  double wheelbase,
  int command_angular_sign = 1);

std::optional<ChannelSpeeds> wheels_to_channels(
  const WheelSpeeds & wheels,
  const std::string & channel_1,
  const std::string & channel_2);

std::optional<ChannelSpeeds> apply_motor_signs(
  const ChannelSpeeds & channels,
  int motor_sign_1,
  int motor_sign_2);

std::optional<ChannelSpeeds> twist_to_channel_speeds(
  double linear_x,
  double angular_z,
  double wheelbase,
  const std::string & channel_1,
  const std::string & channel_2,
  int command_angular_sign = 1);

}  // namespace command_conversion
}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__ROBOTEQ_COMMAND_CONVERSION_HPP_
