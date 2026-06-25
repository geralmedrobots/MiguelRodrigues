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

#ifndef ROBOTEQ_ROS2_DRIVER__ODOM_COVARIANCE_HPP_
#define ROBOTEQ_ROS2_DRIVER__ODOM_COVARIANCE_HPP_

#include <array>

namespace roboteq_ros2_driver
{
namespace odom_covariance
{

using CovarianceMatrix = std::array<double, 36>;

constexpr int kXIndex = 0;
constexpr int kYIndex = 7;
constexpr int kZIndex = 14;
constexpr int kRollIndex = 21;
constexpr int kPitchIndex = 28;
constexpr int kYawIndex = 35;

struct OdometryCovarianceConfig
{
  double pose_x;
  double pose_y;
  double pose_z;
  double pose_roll;
  double pose_pitch;
  double pose_yaw;
  double twist_linear_x;
  double twist_linear_y;
  double twist_linear_z;
  double twist_angular_x;
  double twist_angular_y;
  double twist_angular_z;
};

OdometryCovarianceConfig default_config();
double sanitize_variance(double value, double fallback);
OdometryCovarianceConfig sanitize_config(const OdometryCovarianceConfig & config);
CovarianceMatrix build_pose_covariance(const OdometryCovarianceConfig & config);
CovarianceMatrix build_twist_covariance(const OdometryCovarianceConfig & config);

}  // namespace odom_covariance
}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__ODOM_COVARIANCE_HPP_
