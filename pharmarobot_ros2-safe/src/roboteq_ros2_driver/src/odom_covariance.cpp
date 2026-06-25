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

#include "roboteq_ros2_driver/odom_covariance.hpp"

#include <cmath>

namespace roboteq_ros2_driver
{
namespace odom_covariance
{

OdometryCovarianceConfig default_config()
{
  return {
    0.05,
    0.10,
    1000000.0,
    1000000.0,
    1000000.0,
    0.25,
    0.10,
    1000000.0,
    1000000.0,
    1000000.0,
    1000000.0,
    0.50,
  };
}

double sanitize_variance(double value, double fallback)
{
  if (!std::isfinite(value) || value < 0.0) {
    return fallback;
  }
  return value;
}

OdometryCovarianceConfig sanitize_config(const OdometryCovarianceConfig & config)
{
  const auto defaults = default_config();
  return {
    sanitize_variance(config.pose_x, defaults.pose_x),
    sanitize_variance(config.pose_y, defaults.pose_y),
    sanitize_variance(config.pose_z, defaults.pose_z),
    sanitize_variance(config.pose_roll, defaults.pose_roll),
    sanitize_variance(config.pose_pitch, defaults.pose_pitch),
    sanitize_variance(config.pose_yaw, defaults.pose_yaw),
    sanitize_variance(config.twist_linear_x, defaults.twist_linear_x),
    sanitize_variance(config.twist_linear_y, defaults.twist_linear_y),
    sanitize_variance(config.twist_linear_z, defaults.twist_linear_z),
    sanitize_variance(config.twist_angular_x, defaults.twist_angular_x),
    sanitize_variance(config.twist_angular_y, defaults.twist_angular_y),
    sanitize_variance(config.twist_angular_z, defaults.twist_angular_z),
  };
}

CovarianceMatrix build_pose_covariance(const OdometryCovarianceConfig & config)
{
  const auto sanitized = sanitize_config(config);
  CovarianceMatrix covariance{};
  covariance[kXIndex] = sanitized.pose_x;
  covariance[kYIndex] = sanitized.pose_y;
  covariance[kZIndex] = sanitized.pose_z;
  covariance[kRollIndex] = sanitized.pose_roll;
  covariance[kPitchIndex] = sanitized.pose_pitch;
  covariance[kYawIndex] = sanitized.pose_yaw;
  return covariance;
}

CovarianceMatrix build_twist_covariance(const OdometryCovarianceConfig & config)
{
  const auto sanitized = sanitize_config(config);
  CovarianceMatrix covariance{};
  covariance[kXIndex] = sanitized.twist_linear_x;
  covariance[kYIndex] = sanitized.twist_linear_y;
  covariance[kZIndex] = sanitized.twist_linear_z;
  covariance[kRollIndex] = sanitized.twist_angular_x;
  covariance[kPitchIndex] = sanitized.twist_angular_y;
  covariance[kYawIndex] = sanitized.twist_angular_z;
  return covariance;
}

}  // namespace odom_covariance
}  // namespace roboteq_ros2_driver
