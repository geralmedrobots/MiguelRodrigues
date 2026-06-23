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
