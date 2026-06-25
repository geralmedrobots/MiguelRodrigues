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

#ifndef ROBOTEQ_ROS2_DRIVER__ROBOTEQ_ODOMETRY_HPP_
#define ROBOTEQ_ROS2_DRIVER__ROBOTEQ_ODOMETRY_HPP_

#include <optional>
#include <string>

#include "differential_drive_kinematics.hpp"
#include "roboteq_ros2_driver/odom_twist.hpp"

namespace roboteq_ros2_driver
{
namespace odometry
{

struct WheelTickDelta
{
  int left_ticks;
  int right_ticks;
};

struct IntegrationResult
{
  WheelTickDelta ticks;
  RobotPose pose;
  odom_twist::MeasuredTwist twist;
};

std::optional<WheelTickDelta> map_channel_encoder_sample_to_wheels(
  int channel_1_ticks,
  int channel_2_ticks,
  const std::string & channel_1,
  const std::string & channel_2,
  int encoder_sign_1 = 1,
  int encoder_sign_2 = 1);

class OdometryIntegrator
{
public:
  void init(
    double wheel_radius,
    double wheelbase,
    int encoder_cpr);

  std::optional<IntegrationResult> integrate_channel_sample(
    int channel_1_ticks,
    int channel_2_ticks,
    double dt,
    const std::string & channel_1,
    const std::string & channel_2,
    int encoder_sign_1 = 1,
    int encoder_sign_2 = 1);

private:
  DifferentialDriveKinematics kinematics_;
  RobotPose current_pose_{0.0, 0.0, 0.0};
  bool twist_initialized_{false};
};

}  // namespace odometry
}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__ROBOTEQ_ODOMETRY_HPP_
