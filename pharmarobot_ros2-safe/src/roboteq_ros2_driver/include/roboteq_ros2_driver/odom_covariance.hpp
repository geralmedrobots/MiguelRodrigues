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
